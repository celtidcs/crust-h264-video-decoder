//! Contexto de procesamiento de píxeles y tareas desacopladas (`PixelCtx` y worker EDC).
//!
//! Implementa la arquitectura de desacoplamiento de entropía (*Entropy Decoupling Context*),
//! ejecutando la reconstrucción de macrobloques inter y el filtrado de desbloqueo en
//! un contexto dedicado u orientado a subprocesos paralelos sin interferir con el analizador sintáctico.

#![allow(
    clippy::too_many_arguments,
    clippy::let_and_return,
    clippy::doc_lazy_continuation
)]

use alloc::boxed::Box;

use crate::error::MbError;

/// The committed-value class of a deferred B_Skip span. Continuation
/// requires kind EQUALITY: the flush range-fills grids and recons the band
/// from these values alone.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum BzKind {
    /// ref 0 both lists, (0,0)/(0,0) — band = avg of both padded refs.
    ZeroBi,
    /// One active list at (0,0): (list, clamped ref index) — band = memcpy.
    ZeroUni(u8, u8),
    /// Uniform full-pel direct: clamped refs (-1 = inactive) + adjusted MVs,
    /// windows PREVALIDATED per MB at push (contiguous tiles ⇒ the union
    /// window is valid) — band = offset copy/avg. Chroma requires mv%8==0
    /// (also prevalidated).
    Fp {
        r0: i8,
        r1: i8,
        m0: (i16, i16),
        m1: (i16, i16),
    },
}

/// D9b: MC-call coalescing for a P inter MB from committed per-block (mv, ref).
/// Shared by residual and no-residual recon — filters are per-output-pixel so
/// wider rects are bit-identical to sixteen 4×4 calls.
pub(crate) fn coalesce_p_inter_mc(
    refs: &[crate::Ref],
    cw: usize,
    ccw: usize,
    mb_h: usize,
    mbx: usize,
    mby: usize,
    gmv: &[(i32, i32); 16],
    gref: &[usize; 16],
    pred_y: &mut [u8; 256],
    c_pred: &mut [[u8; 64]; 2],
) -> Result<(), MbError> {
    let (rh16, cch) = (mb_h * 16, mb_h * 8);
    let rect_eq = |x4: usize, y4: usize, w4: usize, h4: usize| -> bool {
        let t = y4 * 4 + x4;
        (0..h4).all(|dy| {
            (0..w4).all(|dx| {
                // 4x4 block grid: the index is always < 16, but LLVM cannot
                // infer it from the nested ranges. `& 15` states it.
                let b = ((y4 + dy) * 4 + (x4 + dx)) & 15;
                gmv[b] == gmv[t] && gref[b] == gref[t]
            })
        })
    };
    let mut mc_rect = |x4: usize, y4: usize, w4: usize, h4: usize| -> Result<(), MbError> {
        let b = y4 * 4 + x4;
        let (mv, gr) = (gmv[b & 15], gref[b & 15]);
        // Una posición que la lista no tiene es un estado incoherente (COR-3).
        let reference = refs.get(gr).ok_or(MbError::Inconsistent("refs"))?;
        let (w, h) = (w4 * 4, h4 * 4);
        if w == 16 {
            crusty_h264_common::inter::with_mc_scratch(|scr| {
                crusty_h264_common::inter::mc_luma_padded_pre(
                    scr,
                    reference.plano_luma(),
                    reference.lstride(),
                    crate::LPAD,
                    cw,
                    rh16,
                    mbx * 16,
                    mby * 16 + y4 * 4,
                    w,
                    h,
                    mv.0,
                    mv.1,
                    &mut pred_y[y4 * 64..y4 * 64 + w * h],
                )
            })?;
        } else {
            let mut t = [0u8; 256];
            crusty_h264_common::inter::with_mc_scratch(|scr| {
                crusty_h264_common::inter::mc_luma_padded_pre(
                    scr,
                    reference.plano_luma(),
                    reference.lstride(),
                    crate::LPAD,
                    cw,
                    rh16,
                    mbx * 16 + x4 * 4,
                    mby * 16 + y4 * 4,
                    w,
                    h,
                    mv.0,
                    mv.1,
                    &mut t[..w * h],
                )
            })?;
            for dy in 0..h {
                pred_y[(y4 * 4 + dy) * 16 + x4 * 4..][..w].copy_from_slice(&t[dy * w..dy * w + w]);
            }
        }
        let (cw4, ch4) = (w4 * 2, h4 * 2);
        let nc = cw4 * ch4;
        let (gu, gv) = (reference.plano_croma(0), reference.plano_croma(1));
        // U+V paired: one setup serves both planes (see mc_chroma_padded_pair).
        if cw4 == 8 {
            let [cu, cv] = &mut *c_pred;
            crusty_h264_common::inter::mc_chroma_padded_pair(
                gu,
                gv,
                reference.cstride(),
                crate::CPAD,
                ccw,
                cch,
                mbx * 8,
                mby * 8 + y4 * 2,
                cw4,
                ch4,
                mv.0,
                mv.1,
                &mut cu[y4 * 16..y4 * 16 + nc],
                &mut cv[y4 * 16..y4 * 16 + nc],
            )?;
        } else {
            let (mut tu, mut tv) = ([0u8; 64], [0u8; 64]);
            crusty_h264_common::inter::mc_chroma_padded_pair(
                gu,
                gv,
                reference.cstride(),
                crate::CPAD,
                ccw,
                cch,
                mbx * 8 + x4 * 2,
                mby * 8 + y4 * 2,
                cw4,
                ch4,
                mv.0,
                mv.1,
                &mut tu[..nc],
                &mut tv[..nc],
            )?;
            for (cc, tc) in [(0usize, &tu), (1, &tv)] {
                for dy in 0..ch4 {
                    c_pred[cc][(y4 * 2 + dy) * 8 + x4 * 2..][..cw4]
                        .copy_from_slice(&tc[dy * cw4..dy * cw4 + cw4]);
                }
            }
        }
        Ok(())
    };
    if rect_eq(0, 0, 4, 4) {
        mc_rect(0, 0, 4, 4)?;
    } else if rect_eq(0, 0, 4, 2) && rect_eq(0, 2, 4, 2) {
        mc_rect(0, 0, 4, 2)?;
        mc_rect(0, 2, 4, 2)?;
    } else if rect_eq(0, 0, 2, 4) && rect_eq(2, 0, 2, 4) {
        mc_rect(0, 0, 2, 4)?;
        mc_rect(2, 0, 2, 4)?;
    } else {
        for q in 0..4usize {
            let (qx, qy) = ((q % 2) * 2, (q / 2) * 2);
            if rect_eq(qx, qy, 2, 2) {
                mc_rect(qx, qy, 2, 2)?;
            } else if rect_eq(qx, qy, 2, 1) && rect_eq(qx, qy + 1, 2, 1) {
                mc_rect(qx, qy, 2, 1)?;
                mc_rect(qx, qy + 1, 2, 1)?;
            } else if rect_eq(qx, qy, 1, 2) && rect_eq(qx + 1, qy, 1, 2) {
                mc_rect(qx, qy, 1, 2)?;
                mc_rect(qx + 1, qy, 1, 2)?;
            } else {
                for j in 0..4usize {
                    mc_rect(qx + (j % 2), qy + (j / 2), 1, 1)?;
                }
            }
        }
    }
    Ok(())
}

/// One deferred pixel-reconstruction job (entropy-decouple E1 seam).
pub(crate) enum EdcJob {
    Skip {
        mbx: usize,
        mby: usize,
        mv: (i32, i32),
    },
    Inter(Box<PInterJob>),
    /// P inter with `cbp == 0` — the P-side twin of `BSkip` (D9).
    InterNoRes(Box<PInterNoResJob>),
}

/// A P inter macroblock with NO residual (`cbp == 0`) — motion only.
///
/// D9. `PInterJob` is 2,784 bytes and **93% of that is coefficient arrays**
/// (`luma_scan` 1024 + `luma8` 1024 + `cac` 512 + `cdc` 32 = 2,592). When
/// `cbp == 0` every one of them is ZERO, and the seam was heap-allocating,
/// filling, channel-passing and freeing all 2,592 bytes of nothing —
/// 12.8-37.5% of inter jobs on the x264 corpus, 15.8-44.1 MB per 60-frame pass.
///
/// This is the same pathology `EdcJob::BSkip` was introduced to fix on the B
/// side ("2.6 KB of ZEROED coefficient arrays for ~60% of B macroblocks — the
/// wall-time regression's main CPU tax"). It was never ported to P.
///
/// Consumer uses `recon_p_inter_nores` (MC + plane copy) — bit-identical to
/// `recon_p_inter` on zero residuals, without memset of 2.5 KB coeff arrays;
/// it carries only what nores recon reads.
pub(crate) struct PInterNoResJob {
    pub(crate) mbx: usize,
    pub(crate) mby: usize,
    pub(crate) t8: bool,
    pub(crate) gmv: [(i32, i32); 16],
    pub(crate) gref: [u8; 16],
}

/// The compact inputs of one CABAC P inter macroblock's reconstruction.
/// Shared all-zero residual planes. An Option-shaped residual field is `None`
/// exactly when the parse wrote nothing into it, and every consumer read is
/// already guarded by a zero-count test — so a `None` reader would have seen
/// zeros anyway. Pointing at one static beats zero-initialising 1 KB (luma) or
/// 512 B (chroma AC) of stack per macroblock.
pub(crate) static ZERO_LUMA_SCAN: [[i32; 16]; 16] = [[0; 16]; 16];
/// `(16384 + |td|/2) / td` for every `td` the spec can present. `td` is
/// `.clamp(-128, 127)` at both call sites (spec 8.4.1.2.3), so the divide has
/// exactly 256 possible inputs and a table replaces it EXACTLY -- this is not an
/// approximation, it is the same arithmetic evaluated ahead of time. `td == 0`
/// is guarded by the caller and maps to 0 here so the table has no hole.
static TX_FOR_TD: [i32; 256] = {
    let mut t = [0i32; 256];
    let mut i = 0usize;
    while i < 256 {
        let td = i as i32 - 128; // index 0 => -128 .. index 255 => 127
        t[i] = if td == 0 {
            0
        } else {
            (16384 + td.abs() / 2) / td
        };
        i += 1;
    }
    t
};

#[inline(always)]
pub(crate) fn tx_for_td(td: i32) -> i32 {
    debug_assert!((-128..=127).contains(&td), "td must be clamped before this");
    TX_FOR_TD[(td.clamp(-128, 127) + 128) as usize]
}

pub(crate) static ZERO_CAC: [[[i32; 16]; 4]; 2] = [[[0; 16]; 4]; 2];

pub(crate) struct PInterJob {
    pub(crate) mbx: usize,
    pub(crate) mby: usize,
    pub(crate) qp: u8,
    pub(crate) cbp_chroma: u32,
    /// transform_size_8x8_flag: the residual is `luma8`, not `luma_scan`.
    pub(crate) t8: bool,
    /// The committed per-block motion, copied at parse time so the worker
    /// never reads the parse thread's grids (E2). Ref indices clamped by the
    /// consumer, kept u8 (spec max 15).
    pub(crate) gmv: [(i32, i32); 16],
    pub(crate) gref: [u8; 16],
    /// `None` when no 4x4 luma block was coded — t8 macroblocks included,
    /// since those carry `luma8` instead.
    /// POOLED and reused across macroblocks (see `take_pinter_job`): only the
    /// blocks whose `nnzs` entry is nonzero hold the data of this macroblock; every
    /// other block is STALE and the consumer never reads it (it copies the
    /// prediction when nnz == 0). The parse zeroes exactly the blocks it is
    /// about to write.
    pub(crate) luma_scan: [[i32; 16]; 16],
    pub(crate) luma8: [[i32; 64]; 4],
    pub(crate) cdc: [[i32; 4]; 2],
    pub(crate) cac: [[[i32; 16]; 4]; 2],
    pub(crate) nnzs: [u8; 24],
}

impl PInterJob {
    pub(crate) fn blank() -> Self {
        PInterJob {
            mbx: 0,
            mby: 0,
            qp: 0,
            cbp_chroma: 0,
            t8: false,
            gmv: [(0, 0); 16],
            gref: [0; 16],
            luma_scan: [[0; 16]; 16],
            luma8: [[0; 64]; 4],
            cdc: [[0; 4]; 2],
            cac: [[[0; 16]; 4]; 2],
            nnzs: [0; 24],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inicializacion_trabajo_inter_en_blanco() {
        let job = PInterJob::blank();
        assert_eq!(job.mbx, 0);
        assert_eq!(job.mby, 0);
        assert_eq!(job.qp, 0);
        assert_eq!(job.cbp_chroma, 0);
        assert!(!job.t8);
        assert_eq!(job.nnzs, [0u8; 24]);
        assert_eq!(job.cdc, [[0; 4]; 2]);
    }

    #[test]
    fn tabla_tx_for_td_limites_y_clamp() {
        assert_eq!(tx_for_td(0), 0);
        assert_eq!(tx_for_td(-128), (16384 + 64) / -128);
        assert_eq!(tx_for_td(127), (16384 + 63) / 127);
    }
}
