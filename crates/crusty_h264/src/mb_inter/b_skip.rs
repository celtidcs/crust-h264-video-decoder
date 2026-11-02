//! `B_Skip` y sus tramos diferidos (§7.4.4, §8.4.1.2).

use super::b_mc::PESOS_PROMEDIO;
use crate::edc::BzKind;
use crate::error::MbError;
use crate::mb16::FrameDecoder;
use crate::mb_inter::{MovimientoBi, RegionB};
use crusty_h264_common::inter::MvNeighbor;

/// Tramo de una fila de `B_Skip` que se reconstruye de una vez: fila de macrobloques `row`,
/// primera columna `x0` y número de macrobloques `n`.
#[derive(Clone, Copy)]
pub(super) struct TramoSkip {
    pub(super) row: usize,
    pub(super) x0: usize,
    pub(super) n: usize,
}

impl FrameDecoder {
    /// Reconstructs a `B_Skip` macroblock: spatial-direct prediction, no residual.
    /// Defer one fast B_Skip's grid commit + recon into the pending row span.
    /// Continuation requires position adjacency AND kind equality.
    #[inline]
    pub(crate) fn bz_push(
        &mut self,
        mb_x: usize,
        mb_y: usize,
        kind: BzKind,
    ) -> Result<(), MbError> {
        match self.bzspan {
            Some((row, x0, ref mut n, k)) if row == mb_y && x0 + *n == mb_x && k == kind => *n += 1,
            _ => {
                self.bz_flush()?;
                self.bzspan = Some((mb_y, mb_x, 1, kind));
            }
        }
        Ok(())
    }

    /// Range-fill the pending span's grids. MUST run before anything reads
    /// the motion/coded/mode/nnz grids of a deferred MB: decode_b_mb entry,
    /// decode_b_skip's b_direct_nbrs, bS derivation (derive_bs_row callers
    /// via row_hook + deblock), and slice end (edc_flush covers it).
    #[inline(always)]
    pub(crate) fn bz_flush(&mut self) -> Result<(), MbError> {
        if self.bzspan.is_some() {
            self.bz_flush_slow()?;
        }
        Ok(())
    }

    pub(crate) fn bz_flush_slow(&mut self) -> Result<(), MbError> {
        let Some((row, x0, n, kind)) = self.bzspan.take() else {
            return Ok(());
        };
        // Grid values per kind (mirrors b_set_motion's inactive-list zeroing).
        let (g_r0, g_r1, g_m0, g_m1): (i32, i32, (i32, i32), (i32, i32)) = match kind {
            BzKind::ZeroBi => (0, 0, (0, 0), (0, 0)),
            BzKind::ZeroUni(0, ri) => (ri as i32, -1, (0, 0), (0, 0)),
            BzKind::ZeroUni(_, ri) => (-1, ri as i32, (0, 0), (0, 0)),
            BzKind::Fp { r0, r1, m0, m1 } => {
                let mm0 = if r0 >= 0 {
                    (m0.0 as i32, m0.1 as i32)
                } else {
                    (0, 0)
                };
                let mm1 = if r1 >= 0 {
                    (m1.0 as i32, m1.1 as i32)
                } else {
                    (0, 0)
                };
                (r0 as i32, r1 as i32, mm0, mm1)
            }
        };
        let w4 = self.mb_w * 4;
        let (b0, len) = (x0 * 4, n * 4);
        for dy in 0..4 {
            let a = (row * 4 + dy) * w4 + b0;
            self.ref_idx_y[a..a + len].fill(g_r0);
            self.mv_y[a..a + len].fill(g_m0);
            self.ref_idx1[a..a + len].fill(g_r1);
            self.mv1[a..a + len].fill(g_m1);
            self.inter_y[a..a + len].fill(true);
            self.coded_y[a..a + len].fill(true);
            self.modes_y[a..a + len].fill(2);
            self.nnz_y[a..a + len].fill(0);
        }
        match kind {
            BzKind::ZeroBi => self.bz_recon_band_bi(row, x0, n)?,
            BzKind::ZeroUni(list, ri) => self.bz_recon_band_copy(
                TramoSkip { row, x0, n },
                list as usize,
                ri as usize,
                (0, 0),
            )?,
            BzKind::Fp { r0, r1, m0, m1 } => {
                let mv0 = (m0.0 as i32, m0.1 as i32);
                let mv1 = (m1.0 as i32, m1.1 as i32);
                match (r0 >= 0, r1 >= 0) {
                    (true, true) => self.bz_recon_band_fp_bi(
                        TramoSkip { row, x0, n },
                        r0 as usize,
                        r1 as usize,
                        mv0,
                        mv1,
                    )?,
                    (true, false) => {
                        self.bz_recon_band_copy(TramoSkip { row, x0, n }, 0, r0 as usize, mv0)?
                    }
                    _ => self.bz_recon_band_copy(TramoSkip { row, x0, n }, 1, r1 as usize, mv1)?,
                }
            }
        }
        Ok(())
    }

    /// ZeroBi band: rows of 16n averaged from both padded refs (index 0).
    pub(crate) fn bz_recon_band_bi(
        &mut self,
        row: usize,
        x0: usize,
        n: usize,
    ) -> Result<(), MbError> {
        // Byte-identical to n recon_b_skip_zero_bi(_, _, 0, 0) calls: each
        // output byte is the same (a + b + 1) >> 1 of the same source bytes.
        let w = n * 16;
        let Some(rf0) = self.refs.first() else {
            return Ok(());
        };
        let Some(rf1) = self.refs1.first() else {
            return Err(MbError::Inconsistent("refs"));
        };
        let (l0st, l1st) = (rf0.lstride(), rf1.lstride());
        {
            let (ly0, ly1) = (rf0.plano_luma(), rf1.plano_luma());
            for dy in 0..16 {
                let y = row * 16 + dy;
                let s0 = (y + crate::LPAD) * l0st + crate::LPAD + x0 * 16;
                let s1 = (y + crate::LPAD) * l1st + crate::LPAD + x0 * 16;
                let d = y * self.cw + x0 * 16;
                // WIN: this bi-average was a scalar byte loop; `pixel_avg` computes
                // (a + b + 1) >> 1 exactly with pavgb, and the row helper walks the
                // span in kernel-legal widths.
                crusty_h264_common::inter::avg_row_into(
                    &ly0[s0..s0 + w],
                    &ly1[s1..s1 + w],
                    w,
                    &mut self.rec_y[d..d + w],
                );
            }
        }
        let (c0st, c1st) = (rf0.cstride(), rf1.cstride());
        let wc = n * 8;
        for c in 0..2 {
            let rc0 = rf0.plano_croma(c);
            let rc1 = rf1.plano_croma(c);
            let plane = if c == 0 {
                &mut self.rec_u
            } else {
                &mut self.rec_v
            };
            for dy in 0..8 {
                let y = row * 8 + dy;
                let s0 = (y + crate::CPAD) * c0st + crate::CPAD + x0 * 8;
                let s1 = (y + crate::CPAD) * c1st + crate::CPAD + x0 * 8;
                let d = y * self.ccw + x0 * 8;
                // WIN: this bi-average was a scalar byte loop; `pixel_avg` computes
                // (a + b + 1) >> 1 exactly with pavgb, and the row helper walks the
                // span in kernel-legal widths.
                crusty_h264_common::inter::avg_row_into(
                    &rc0[s0..s0 + wc],
                    &rc1[s1..s1 + wc],
                    wc,
                    &mut plane[d..d + wc],
                );
            }
        }
        Ok(())
    }

    /// Full-pel offset copy band from ONE list's padded planes. The caller
    /// prevalidated (per MB, at push): mv%8 == 0 both components and luma +
    /// chroma windows inside the padded planes — contiguous MB windows tile,
    /// so the span's union window is valid by induction.
    pub(super) fn bz_recon_band_copy(
        &mut self,
        tramo: TramoSkip,
        list: usize,
        ri: usize,
        mv: (i32, i32),
    ) -> Result<(), MbError> {
        let TramoSkip { row, x0, n } = tramo;
        let Some(rf) = (if list == 0 {
            self.refs.get(ri)
        } else {
            self.refs1.get(ri)
        }) else {
            return Err(MbError::Inconsistent("refs"));
        };
        let lst = rf.lstride();
        let (dx, dy) = ((mv.0 / 4) as isize, (mv.1 / 4) as isize);
        let w = n * 16;
        {
            let ly = rf.plano_luma();
            for r in 0..16 {
                let y = (row * 16 + r) as isize + dy;
                let src = ((y + crate::LPAD as isize) * lst as isize
                    + crate::LPAD as isize
                    + x0 as isize * 16
                    + dx) as usize;
                let d = (row * 16 + r) * self.cw + x0 * 16;
                self.rec_y[d..d + w].copy_from_slice(&ly[src..src + w]);
            }
        }
        let cst = rf.cstride();
        let (cdx, cdy) = ((mv.0 / 8) as isize, (mv.1 / 8) as isize);
        let wc = n * 8;
        for c in 0..2 {
            let rc = rf.plano_croma(c);
            let plane = if c == 0 {
                &mut self.rec_u
            } else {
                &mut self.rec_v
            };
            for r in 0..8 {
                let y = (row * 8 + r) as isize + cdy;
                let src = ((y + crate::CPAD as isize) * cst as isize
                    + crate::CPAD as isize
                    + x0 as isize * 8
                    + cdx) as usize;
                let d = (row * 8 + r) * self.ccw + x0 * 8;
                plane[d..d + wc].copy_from_slice(&rc[src..src + wc]);
            }
        }
        Ok(())
    }

    /// Full-pel bi band: rows of 16n averaged from two offset windows
    /// (prevalidated as above; implicit weights None/(32,32) guaranteed by
    /// the pushing arm).
    pub(super) fn bz_recon_band_fp_bi(
        &mut self,
        tramo: TramoSkip,
        r0: usize,
        r1: usize,
        mv0: (i32, i32),
        mv1: (i32, i32),
    ) -> Result<(), MbError> {
        let TramoSkip { row, x0, n } = tramo;
        let (Some(rf0), Some(rf1)) = (self.refs.get(r0), self.refs1.get(r1)) else {
            return Err(MbError::Inconsistent("refs"));
        };
        let (l0, l1) = (rf0.lstride(), rf1.lstride());
        let off = |st: usize, pad: isize, yy: isize, xx: isize| {
            (((yy + pad) * st as isize) + pad + xx) as usize
        };
        let w = n * 16;
        {
            let (ly0, ly1) = (rf0.plano_luma(), rf1.plano_luma());
            for r in 0..16 {
                let y = (row * 16 + r) as isize;
                let s0 = off(
                    l0,
                    crate::LPAD as isize,
                    y + (mv0.1 / 4) as isize,
                    x0 as isize * 16 + (mv0.0 / 4) as isize,
                );
                let s1 = off(
                    l1,
                    crate::LPAD as isize,
                    y + (mv1.1 / 4) as isize,
                    x0 as isize * 16 + (mv1.0 / 4) as isize,
                );
                let d = (row * 16 + r) * self.cw + x0 * 16;
                // WIN: this bi-average was a scalar byte loop; `pixel_avg` computes
                // (a + b + 1) >> 1 exactly with pavgb, and the row helper walks the
                // span in kernel-legal widths.
                crusty_h264_common::inter::avg_row_into(
                    &ly0[s0..s0 + w],
                    &ly1[s1..s1 + w],
                    w,
                    &mut self.rec_y[d..d + w],
                );
            }
        }
        let (c0, c1) = (rf0.cstride(), rf1.cstride());
        let wc = n * 8;
        for c in 0..2 {
            let rc0 = rf0.plano_croma(c);
            let rc1 = rf1.plano_croma(c);
            let plane = if c == 0 {
                &mut self.rec_u
            } else {
                &mut self.rec_v
            };
            for r in 0..8 {
                let y = (row * 8 + r) as isize;
                let s0 = off(
                    c0,
                    crate::CPAD as isize,
                    y + (mv0.1 / 8) as isize,
                    x0 as isize * 8 + (mv0.0 / 8) as isize,
                );
                let s1 = off(
                    c1,
                    crate::CPAD as isize,
                    y + (mv1.1 / 8) as isize,
                    x0 as isize * 8 + (mv1.0 / 8) as isize,
                );
                let d = (row * 8 + r) * self.ccw + x0 * 8;
                // WIN: this bi-average was a scalar byte loop; `pixel_avg` computes
                // (a + b + 1) >> 1 exactly with pavgb, and the row helper walks the
                // span in kernel-legal widths.
                crusty_h264_common::inter::avg_row_into(
                    &rc0[s0..s0 + wc],
                    &rc1[s1..s1 + wc],
                    wc,
                    &mut plane[d..d + wc],
                );
            }
        }
        Ok(())
    }

    /// Per-MB prevalidation for an Fp span push: mv%8 both components (chroma
    /// copies at mv/8) and luma + chroma windows inside the padded planes for
    /// every active list. False ⇒ the arm recons immediately instead.
    pub(crate) fn bz_fp_valid(
        &self,
        mbx: usize,
        mby: usize,
        r0: Option<usize>,
        r1: Option<usize>,
        mv0: (i32, i32),
        mv1: (i32, i32),
    ) -> bool {
        let chk = |rf: &crate::Ref, mv: (i32, i32)| -> bool {
            if mv.0 % 8 != 0 || mv.1 % 8 != 0 {
                return false;
            }
            let lst = rf.lstride();
            let lpad = crate::LPAD as isize;
            let cx = mbx as isize * 16 + (mv.0 / 4) as isize + lpad;
            let cy = mby as isize * 16 + (mv.1 / 4) as isize + lpad;
            let lrows = rf.lrows() as isize;
            if cx < 0 || cx + 16 > lst as isize || cy < 0 || cy + 16 > lrows {
                return false;
            }
            let cst = rf.cstride();
            let cpad = crate::CPAD as isize;
            let ccx = mbx as isize * 8 + (mv.0 / 8) as isize + cpad;
            let ccy = mby as isize * 8 + (mv.1 / 8) as isize + cpad;
            let crows = rf.crows() as isize;
            ccx >= 0 && ccx + 8 <= cst as isize && ccy >= 0 && ccy + 8 <= crows
        };
        r0.map_or(true, |i| self.refs.get(i).is_some_and(|r| chk(r, mv0)))
            && r1.map_or(true, |i| self.refs1.get(i).is_some_and(|r| chk(r, mv1)))
    }

    /// Hot prefix of decode_b_skip: the FORCED zero-bi run continuation,
    /// small enough to inline into the slice loops. Returns true when the MB
    /// was fully handled (deferred into the span); false falls to the cold
    /// body. Exactly the forced arm's conditions — no behavior change.
    ///
    /// # Errors
    /// `MbError::Inconsistent` si `bzero` no cubre al macrobloque o a sus vecinos.
    #[inline(always)]
    pub(crate) fn b_skip_hot(&mut self, mb_x: usize, mb_y: usize) -> Result<bool, MbError> {
        if !self.direct_spatial {
            return Ok(false);
        }
        let mbw = self.mb_w;
        let addr = mb_y * mbw + mb_x;
        // ONE slice that ENDS at `addr` proves all three neighbour reads. Each
        // was a separate check against the whole `bzero` grid even though every
        // index is strictly below `addr` (the guards above establish that), and
        // the trailing `bzero[addr] = true` a fourth. `get(..=addr)` states the
        // bound once, in the form the neighbour indexes are already inside.
        // The `get(..=addr)` slice did NOT fold these: it bounds the reads from
        // above, but LLVM still has to chain `mb_x > 0 => addr >= 1` and
        // `mb_y > 0 => addr >= mbw` through the `&&` sequence, and it will not.
        // Cada lectura se comprueba donde se hace; un índice fuera de la rejilla es un
        // estado incoherente (COR-3), no un vecino «distinto de cero».
        let up = addr.wrapping_sub(mbw);
        if !(mb_x > 0
            && mb_y > 0
            && mb_x + 1 < mbw
            && up >= self.slice_first_mb
            && self
                .bzero
                .get(addr - 1)
                .copied()
                .ok_or(MbError::Inconsistent("bzero"))?
            && self
                .bzero
                .get(up)
                .copied()
                .ok_or(MbError::Inconsistent("bzero"))?
            && self
                .bzero
                .get(up + 1)
                .copied()
                .ok_or(MbError::Inconsistent("bzero"))?)
        {
            return Ok(false);
        }
        let wgt = self.iw00();
        if !(wgt.is_none() || wgt == Some(PESOS_PROMEDIO)) {
            return Ok(false);
        }
        self.bz_push(mb_x, mb_y, BzKind::ZeroBi)?;
        *self
            .bzero
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("bzero"))? = true;
        Ok(true)
    }

    /// Zero this macroblock's 16 luma nnz entries, walking rows by adding `w4`
    /// instead of recomputing `(mb_y * 4 + dy) * w4 + mb_x * 4` four times.
    /// Shared by the three sites in decode_b_skip that had it open-coded.
    #[inline]
    pub(crate) fn clear_mb_nnz(&mut self, mb_x: usize, mb_y: usize, w4: usize) {
        let mut a = (mb_y * 4) * w4 + mb_x * 4;
        for _ in 0..4 {
            self.nnz_y[a..a + 4].fill(0);
            a += w4;
        }
    }

    /// Vecinos de la predicción directa espacial de un `B_Skip`. Si el tramo de `B_Skip` pendiente
    /// acaba justo a la izquierda, sus valores se deducen de su clase en vez de vaciarlo; si no, se
    /// vacía antes de leer las rejillas.
    ///
    /// # Errors
    /// Los de vaciar el tramo y leer las rejillas.
    fn vecinos_skip(
        &mut self,
        mb_x: usize,
        mb_y: usize,
    ) -> Result<([MvNeighbor; 3], [MvNeighbor; 3]), MbError> {
        // A pending span can only occupy the LEFT gather position, and the
        // left member's committed grid values are fully determined by the
        // span KIND — synthesize them instead of flushing, for EVERY kind
        // (the same mapping bz_flush writes).
        let patch = match self.bzspan {
            Some((r, x0, n, k)) if r == mb_y && x0 + n == mb_x => Some(k),
            _ => None,
        };
        if patch.is_none() {
            self.bz_flush()?;
        }
        let (mut n0, mut n1) = self.b_direct_nbrs(mb_x, mb_y)?;
        if let Some(k) = patch {
            let (r0, r1, m0, m1): (i32, i32, (i32, i32), (i32, i32)) = match k {
                BzKind::ZeroBi => (0, 0, (0, 0), (0, 0)),
                BzKind::ZeroUni(0, ri) => (ri as i32, -1, (0, 0), (0, 0)),
                BzKind::ZeroUni(_, ri) => (-1, ri as i32, (0, 0), (0, 0)),
                BzKind::Fp { r0, r1, m0, m1 } => {
                    let mm0 = if r0 >= 0 {
                        (m0.0 as i32, m0.1 as i32)
                    } else {
                        (0, 0)
                    };
                    let mm1 = if r1 >= 0 {
                        (m1.0 as i32, m1.1 as i32)
                    } else {
                        (0, 0)
                    };
                    (r0 as i32, r1 as i32, mm0, mm1)
                }
            };
            n0[0] = MvNeighbor {
                available: true,
                mv: m0,
                ref_idx: r0,
            };
            n1[0] = MvNeighbor {
                available: true,
                mv: m1,
                ref_idx: r1,
            };
        }
        Ok((n0, n1))
    }

    /// `B_Skip` con vectores de muestra entera (§8.4.1.2.2): si `colZeroFlag` es uniforme en el
    /// macrobloque, lo reconstruye como copia o promedio desplazado (en un tramo diferido o en el
    /// acto) y devuelve `true`; si no, `false` y sigue el camino general.
    ///
    /// # Errors
    /// Los de la reconstrucción y del tramo diferido.
    fn skip_b_pel_entero(
        &mut self,
        (mb_x, mb_y): (usize, usize),
        mov: MovimientoBi,
        (lref0, lref1): (usize, usize),
    ) -> Result<bool, MbError> {
        let MovimientoBi {
            refi0,
            mv0,
            refi1,
            mv1,
        } = mov;
        let (a0, a1) = (refi0 >= 0, refi1 >= 0);
        let w4 = self.mb_w * 4;
        // FULL-PEL arm (the pan case — shields): nonzero MVs make
        // colZeroFlag matter, so probe the four 8x8 corners; a UNIFORM
        // czg keeps the MB one region and the MC is an offset read.
        // Non-uniform czg or an out-of-pad window falls through.
        let mut cz_ok = true;
        let mut czv = false;
        // Same cz-relevance gate as decode_b_direct_n: only a ref-0
        // list with a nonzero MV can be changed by colZeroFlag (fp-arm
        // MVs are nonzero by construction, so refs alone decide).
        let cz_need = refi0 == 0 || refi1 == 0;
        // LOOP-INVARIANT: this gated a `break` INSIDE the probe loop, so
        // a fact known before the loop was re-tested on every probe.
        if cz_need {
            // Absolute block base, hoisted out of the four probes.
            let (bx4, by4) = (mb_x * 4, mb_y * 4);
            for (k, &(ox, oy)) in [(0usize, 0usize), (2, 0), (0, 2), (2, 2)]
                .iter()
                .enumerate()
            {
                // col_block takes MB-LOCAL block coords (the whole MB is
                // the region here); col_zero takes absolute — same
                // convention as decode_b_direct_n's probe loop.
                let (colx, coly) = self.col_block(ox, oy);
                let cz = self.col_zero(bx4 + colx, by4 + coly);
                if k == 0 {
                    czv = cz;
                } else if cz != czv {
                    cz_ok = false;
                    break;
                }
            }
        }
        if cz_ok {
            let m0 = if refi0 == 0 && czv { (0, 0) } else { mv0 };
            let m1 = if refi1 == 0 && czv { (0, 0) } else { mv1 };
            let r0c = (refi0 as usize).min(lref0);
            let r1c = (refi1 as usize).min(lref1);
            let (or0, or1) = (a0.then_some(r0c), a1.then_some(r1c));
            let wgt_ok = !(a0 && a1) || {
                let wgt = self.implicit_weights(r0c as i32, r1c as i32);
                wgt.is_none() || wgt == Some(PESOS_PROMEDIO)
            };
            // SPAN path: prevalidated windows + %8 chroma ⇒ defer as
            // an offset copy/avg band; otherwise the immediate per-MB
            // recon below (which handles interpolating chroma).
            if wgt_ok && self.bz_fp_valid(mb_x, mb_y, or0, or1, m0, m1) {
                let kind = BzKind::Fp {
                    r0: if a0 { r0c as i8 } else { -1 },
                    r1: if a1 { r1c as i8 } else { -1 },
                    m0: (m0.0 as i16, m0.1 as i16),
                    m1: (m1.0 as i16, m1.1 as i16),
                };
                self.bz_push(mb_x, mb_y, kind)?;
                return Ok(true);
            }
            let ok = wgt_ok && self.recon_b_skip_fp(mb_x, mb_y, or0, or1, m0, m1)?;
            if ok {
                self.b_set_motion(
                    RegionB {
                        mb_x,
                        mb_y,
                        px: 0,
                        py: 0,
                        rw: 16,
                        rh: 16,
                    },
                    MovimientoBi {
                        refi0,
                        mv0: m0,
                        refi1,
                        mv1: m1,
                    },
                );
                self.clear_mb_nnz(mb_x, mb_y, w4);
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// The B_Skip SLOW half: full spatial-direct recon plus the zero-residual
    /// plane copy. Split out so `pred_y`/`c_pred` (384 B) are built ONLY when a
    /// macroblock actually reaches it — the zero-bi / zero-uni / full-pel fast
    /// paths in decode_b_skip all return before this, and on LIGHT streams they
    /// take 90%+ of the calls.
    pub(crate) fn b_skip_slow(
        &mut self,
        mb_x: usize,
        mb_y: usize,
        nbrs: Option<([MvNeighbor; 3], [MvNeighbor; 3])>,
        w4: usize,
    ) -> Result<(), MbError> {
        let mut pred_y = [0u8; 256];
        let mut c_pred = [[0u8; 64]; 2];
        match nbrs {
            // Reuses the already-probed neighbours — the grid walk is the
            // expensive half of the derivation and paying it twice leaned on
            // fall-through-heavy streams (stockholm).
            Some((n0, n1)) => self.decode_b_direct_n(
                RegionB {
                    mb_x,
                    mb_y,
                    px: 0,
                    py: 0,
                    rw: 16,
                    rh: 16,
                },
                &mut pred_y,
                &mut c_pred,
                (n0, n1),
            )?,
            None => self.decode_b_direct(
                RegionB {
                    mb_x,
                    mb_y,
                    px: 0,
                    py: 0,
                    rw: 16,
                    rh: 16,
                },
                &mut pred_y,
                &mut c_pred,
            )?,
        }
        // Zero residual: the prediction IS the reconstruction — copy it row-wise,
        // stepping the destination by the stride rather than multiplying per row.
        let mut d = (mb_y * 16) * self.cw + mb_x * 16;
        for dy in 0..16 {
            self.rec_y[d..d + 16].copy_from_slice(&pred_y[dy * 16..dy * 16 + 16]);
            d += self.cw;
        }
        self.escribir_prediccion_croma((mb_x, mb_y), &c_pred);
        // nnz stays 0 (no residual) — clear the grids for neighbour context.
        self.clear_mb_nnz(mb_x, mb_y, w4);
        Ok(())
    }

    pub(crate) fn decode_b_skip(&mut self, mb_x: usize, mb_y: usize) -> Result<(), MbError> {
        // Each list length read ONCE: the emptiness test and every later
        // `len() - 1` clamp (six reads in all) come from these two.
        let (nref0, nref1) = (self.refs.len(), self.refs1.len());
        if nref0 == 0 || nref1 == 0 {
            return Err(MbError::Unsupported("B without references"));
        }
        let (lref0, lref1) = (nref0 - 1, nref1 - 1);
        let mbw = self.mb_w;
        let addr = mb_y * mbw + mb_x;
        let w4 = mbw * 4;
        // ZERO-BI FAST PATH: when spatial direct derives (0,0)/(0,0) bi motion,
        // colZeroFlag is IRRELEVANT (it only zeroes MVs that are already zero),
        // so the colocated probing, region split, b_mc staging and the pred
        // round-trip all collapse into one fused row-average from the two
        // padded refs straight into the recon planes. b_mc's bi blend applies
        // only IMPLICIT weights, so `None` or (32,32) (both exactly
        // (a+b+1)>>1) is the whole identity condition. FourPeople-class LIGHT
        // streams put 90%+ of their B_Skips here (BSK_ZBI counter).
        // pred_y / c_pred now live in `b_skip_slow` — see there.
        if self.direct_spatial {
            // (The FORCED zero-bi run continuation lives in b_skip_hot, inlined
            // at the loop call sites — this cold body only sees non-forced MBs.)
            let (n0, n1) = self.vecinos_skip(mb_x, mb_y)?;
            let (refi0, refi1, mv0, mv1, _dz) = Self::b_direct_refs_mvs(&n0, &n1);
            let (a0, a1) = (refi0 >= 0, refi1 >= 0);
            let fast = if a0 && a1 && mv0 == (0, 0) && mv1 == (0, 0) {
                // Same malformed-stream ref clamp as b_mc.
                let r0 = (refi0 as usize).min(lref0);
                let r1 = (refi1 as usize).min(lref1);
                // (0,0) dominates here and is already cached slice-wide by iw00.
                let wgt = if r0 == 0 && r1 == 0 {
                    self.iw00()
                } else {
                    self.implicit_weights(r0 as i32, r1 as i32)
                };
                if wgt.is_none() || wgt == Some(PESOS_PROMEDIO) {
                    if refi0 == 0 && refi1 == 0 {
                        *self
                            .bzero
                            .get_mut(addr)
                            .ok_or(MbError::Inconsistent("bzero"))? = true;
                        // Same constants as the forced arm: recon AND commit
                        // both defer into the span.
                        self.bz_push(mb_x, mb_y, BzKind::ZeroBi)?;
                        return Ok(());
                    }
                    self.recon_b_skip_zero_bi(mb_x, mb_y, r0, r1);
                    true
                } else {
                    false
                }
            } else if a0 != a1 && (if a0 { mv0 } else { mv1 }) == (0, 0) {
                // ZERO-UNI: one active list at (0,0) — b_mc's uni arms are plain
                // unweighted copies. Defer as a memcpy-band span.
                let (list, ri) = if a0 {
                    (0u8, (refi0 as usize).min(lref0) as u8)
                } else {
                    (1u8, (refi1 as usize).min(lref1) as u8)
                };
                self.bz_push(mb_x, mb_y, BzKind::ZeroUni(list, ri))?;
                return Ok(());
            } else if (!a0 || (mv0.0 % 4 == 0 && mv0.1 % 4 == 0))
                && (!a1 || (mv1.0 % 4 == 0 && mv1.1 % 4 == 0))
                && self.direct_8x8_inference
            {
                let mov = MovimientoBi {
                    refi0,
                    mv0,
                    refi1,
                    mv1,
                };
                if self.skip_b_pel_entero((mb_x, mb_y), mov, (lref0, lref1))? {
                    return Ok(());
                }
                false
            } else {
                false
            };
            if fast {
                self.b_set_motion(
                    RegionB {
                        mb_x,
                        mb_y,
                        px: 0,
                        py: 0,
                        rw: 16,
                        rh: 16,
                    },
                    MovimientoBi {
                        refi0,
                        mv0: (0, 0),
                        refi1,
                        mv1: (0, 0),
                    },
                );
                self.clear_mb_nnz(mb_x, mb_y, w4);
                return Ok(());
            }
            // Fall-through hands the ALREADY-probed neighbours to the slow
            // half so the grid walk is not paid twice.
            self.b_skip_slow(mb_x, mb_y, Some((n0, n1)), w4)
        } else {
            self.b_skip_slow(mb_x, mb_y, None, w4)
        }
    }

    /// B_Skip zero-bi recon: rec = avg(ref0, ref1') row-wise, straight from the
    /// padded planes. Byte-identical to b_mc at mv (0,0)/(0,0): full-pel MC is
    /// an identity read and the unweighted bi blend is exactly (a+b+1)>>1 —
    /// which the compiler emits as pavgb over the sliced zips, same as b_mc's
    /// blend site.
    pub(crate) fn recon_b_skip_zero_bi(&mut self, mbx: usize, mby: usize, r0i: usize, r1i: usize) {
        let (Some(rf0), Some(rf1)) = (self.refs.get(r0i), self.refs1.get(r1i)) else {
            return;
        };
        let (l0st, l1st) = (rf0.lstride(), rf1.lstride());
        {
            let (ly0, ly1) = (rf0.plano_luma(), rf1.plano_luma());
            for dy in 0..16 {
                let y = mby * 16 + dy;
                let s0 = (y + crate::LPAD) * l0st + crate::LPAD + mbx * 16;
                let s1 = (y + crate::LPAD) * l1st + crate::LPAD + mbx * 16;
                let d = y * self.cw + mbx * 16;
                // WIN: STRIDED destination, so rustc cannot do what it does for the
                // contiguous pred_y averages (which are already ideal vpavgb and are
                // left alone). Route the row through the pixel_avg kernel instead.
                crusty_h264_common::inter::avg_row_into(
                    &ly0[s0..s0 + 16],
                    &ly1[s1..s1 + 16],
                    16,
                    &mut self.rec_y[d..d + 16],
                );
            }
        }
        let (c0st, c1st) = (rf0.cstride(), rf1.cstride());
        for c in 0..2 {
            let rc0 = rf0.plano_croma(c);
            let rc1 = rf1.plano_croma(c);
            let plane = if c == 0 {
                &mut self.rec_u
            } else {
                &mut self.rec_v
            };
            for dy in 0..8 {
                let y = mby * 8 + dy;
                let s0 = (y + crate::CPAD) * c0st + crate::CPAD + mbx * 8;
                let s1 = (y + crate::CPAD) * c1st + crate::CPAD + mbx * 8;
                let d = y * self.ccw + mbx * 8;
                // WIN: STRIDED destination, so rustc cannot do what it does for the
                // contiguous pred_y averages (which are already ideal vpavgb and are
                // left alone). Route the row through the pixel_avg kernel instead.
                crusty_h264_common::inter::avg_row_into(
                    &rc0[s0..s0 + 8],
                    &rc1[s1..s1 + 8],
                    8,
                    &mut plane[d..d + 8],
                );
            }
        }
    }

    /// B_Skip full-pel recon with per-list integer offsets: uni = offset row
    /// copy, bi = offset row average ((a+b+1)>>1). Luma always copies here
    /// (caller guarantees mv%4==0); chroma copies at mv%8==0 and otherwise
    /// interpolates via b_mc_chroma — identical to the b_mc path's chroma
    /// half with no implicit weights. Returns false untouched when a luma
    /// window leaves the padded plane.
    pub(crate) fn recon_b_skip_fp(
        &mut self,
        mbx: usize,
        mby: usize,
        r0: Option<usize>,
        r1: Option<usize>,
        mv0: (i32, i32),
        mv1: (i32, i32),
    ) -> Result<bool, MbError> {
        let lpad = crate::LPAD as isize;
        // (start-row, start-col, stride, rows) of each active list's luma window.
        let win = |rf: &crate::RefFrame, mv: (i32, i32)| -> Option<(usize, usize)> {
            let lst = rf.lstride();
            let cx = mbx as isize * 16 + (mv.0 / 4) as isize + lpad;
            let cy = mby as isize * 16 + (mv.1 / 4) as isize + lpad;
            let lrows = rf.lrows() as isize;
            if cx < 0 || cx + 16 > lst as isize || cy < 0 || cy + 16 > lrows {
                None
            } else {
                Some((cy as usize * lst + cx as usize, lst))
            }
        };
        let w0 = match r0 {
            Some(i) => match self.refs.get(i).and_then(|rr| win(rr, mv0)) {
                Some(w) => Some(w),
                None => return Ok(false),
            },
            None => None,
        };
        let w1 = match r1 {
            Some(i) => match self.refs1.get(i).and_then(|rr| win(rr, mv1)) {
                Some(w) => Some(w),
                None => return Ok(false),
            },
            None => None,
        };
        {
            match (w0, w1) {
                (Some((s0, l0)), Some((s1, l1))) => {
                    let Some(rf0) = r0.and_then(|i| self.refs.get(i)) else {
                        return Ok(false);
                    };
                    // Same shape as the `rf0` line above: `w1` is `Some` only when
                    // `r1` is `Some(i)` AND `refs1.get(i)` returned a frame, so the
                    // `else` is unreachable -- but it is a REFUSAL, not a panic, and
                    // `recon_b_skip_fp` already answers `false` for every case it
                    // cannot fast-path. Indexing a `Vec` with an unwrapped `Option`
                    // spent a discriminant test, an `unwrap_failed` call and a bounds
                    // check to restate an invariant the caller had already proven.
                    let Some(rf1) = r1.and_then(|i| self.refs1.get(i)) else {
                        return Ok(false);
                    };
                    let (ly0, ly1) = (rf0.plano_luma(), rf1.plano_luma());
                    // WIN: this was 256 scalar (p + q + 1) >> 1 per macroblock, three
                    // modules from the  kernel that computes exactly that
                    // with  and already takes both source strides. One kernel
                    // call over the 16x16, then sixteen row copies out to the strided
                    // recon plane.
                    for r in 0..16 {
                        let d = (mby * 16 + r) * self.cw + mbx * 16;
                        crusty_h264_common::inter::avg_row_into(
                            &ly0[s0 + r * l0..],
                            &ly1[s1 + r * l1..],
                            16,
                            &mut self.rec_y[d..d + 16],
                        );
                    }
                }
                (Some((s0, l0)), None) => {
                    let Some(rf0) = r0.and_then(|i| self.refs.get(i)) else {
                        return Ok(false);
                    };
                    let ly = rf0.plano_luma();
                    for r in 0..16 {
                        let d = (mby * 16 + r) * self.cw + mbx * 16;
                        self.rec_y[d..d + 16].copy_from_slice(&ly[s0 + r * l0..s0 + r * l0 + 16]);
                    }
                }
                (None, Some((s1, l1))) => {
                    // Same shape as the `rf0` line above: `w1` is `Some` only when
                    // `r1` is `Some(i)` AND `refs1.get(i)` returned a frame, so the
                    // `else` is unreachable -- but it is a REFUSAL, not a panic, and
                    // `recon_b_skip_fp` already answers `false` for every case it
                    // cannot fast-path. Indexing a `Vec` with an unwrapped `Option`
                    // spent a discriminant test, an `unwrap_failed` call and a bounds
                    // check to restate an invariant the caller had already proven.
                    let Some(rf1) = r1.and_then(|i| self.refs1.get(i)) else {
                        return Ok(false);
                    };
                    let ly = rf1.plano_luma();
                    for r in 0..16 {
                        let d = (mby * 16 + r) * self.cw + mbx * 16;
                        self.rec_y[d..d + 16].copy_from_slice(&ly[s1 + r * l1..s1 + r * l1 + 16]);
                    }
                }
                (None, None) => return Ok(false),
            }
        }
        // Chroma: interpolating half — route through b_mc_chroma into a stage
        // (weights None: the caller only enters at implicit None/(32,32), and
        // (32,32) IS the plain average b_mc_chroma computes for None).
        let cch = self.mb_h * 8;
        let mut c_pred = [[0u8; 64]; 2];
        let (ri0, ri1) = (r0.map_or(-1, |i| i as i32), r1.map_or(-1, |i| i as i32));
        self.b_mc_chroma(
            RegionB {
                mb_x: mbx,
                mb_y: mby,
                px: 0,
                py: 0,
                rw: 16,
                rh: 16,
            },
            MovimientoBi {
                refi0: ri0,
                mv0,
                refi1: ri1,
                mv1,
            },
            &mut c_pred,
            None,
            cch,
        )?;
        self.escribir_prediccion_croma((mbx, mby), &c_pred);
        Ok(true)
    }
}
