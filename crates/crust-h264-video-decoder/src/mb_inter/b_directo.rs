//! Predicción directa espacial y temporal de los B (§8.4.1.2).

use super::b_mc::factor_escala_distancia;
use crate::error::MbError;
use crate::mb16::FrameDecoder;
use crate::mb_inter::{MovimientoBi, RegionB};
use crust_h264_common::inter::{predict_mv, MvNeighbor};

impl FrameDecoder {
    /// `colZeroFlag` for the 4×4 block at absolute block coords `(bx, by)`: true
    /// when `RefPicList1[0]` is a short-term picture whose co-located block uses
    /// reference 0 with a near-zero motion vector (spec §8.4.1.2.2).
    /// Co-located 4x4 block coords for the current block's `(bx4, by4)` within the
    /// macroblock, per spec 8.4.1.2.1. Under `direct_8x8_inference_flag` every 4x4
    /// in an 8x8 takes that 8x8's OUTER CORNER (`luma4x4BlkIdx = 5 * mbPartIdx`,
    /// i.e. (0,0) (3,0) (0,3) (3,3)); otherwise motion is genuinely per-4x4.
    ///
    /// 8.4.1.2.1 is SHARED by both direct modes, so spatial and temporal must map
    /// identically. They did not: temporal mapped the corner and spatial read the
    /// block's own coords, which is invisible while every 4x4 in the co-located 8x8
    /// carries the same motion -- true of every stream until sub-8x8 P partitions
    /// (x264 `--partitions p4x4`) make them differ. Hence one function.
    #[inline]
    pub(crate) fn col_block(&self, bx4: usize, by4: usize) -> (usize, usize) {
        if self.direct_8x8_inference {
            ((bx4 / 2) * 3, (by4 / 2) * 3)
        } else {
            (bx4, by4)
        }
    }

    pub(crate) fn col_zero(&self, bx: usize, by: usize) -> bool {
        // Fast path (1T frozen colocated, the default): the invariant checks
        // were hoisted to set_b_context; two grid loads + the threshold test.
        if self.col_ok {
            let Some(col) = self.refs1.first() else {
                return false;
            };
            let idx = by * self.col_w4 + bx;
            // `.get` on EVERY parallel array, not a length test on one of them.
            // The `idx >= ref_idx.len()` guard proved nothing about `mv`,
            // `ref_idx1` or `mv1` — separate Vecs with independent lengths — so
            // each of those reads carried its own panic path. They are built
            // together and always agree; the fallible form states that instead of
            // asserting it, and keeps the "no panic on malformed input" contract.
            let (cref, cmv) = match col.ref_idx.get(idx) {
                Some(&r0) if r0 >= 0 => match col.mv.get(idx) {
                    Some(&m) => (r0, m),
                    None => return false,
                },
                Some(_) => match (col.ref_idx1.get(idx), col.mv1.get(idx)) {
                    (Some(&r1), Some(&m)) if r1 >= 0 => (r1, m),
                    _ => return false,
                },
                None => return false,
            };
            return cref == 0 && cmv.0.abs() <= 1 && cmv.1.abs() <= 1;
        }
        let Some(col) = self.refs1.first() else {
            return false;
        };
        if col.long_term || col.w4 == 0 {
            return false;
        }
        let idx = by * col.w4 + bx;
        if idx >= col.ref_idx.len() {
            return false;
        }
        let (cref, cmv) = match col.ref_idx.get(idx) {
            Some(&r0) if r0 >= 0 => match col.mv.get(idx) {
                Some(&m) => (r0, m),
                None => return false,
            },
            Some(_) => match (col.ref_idx1.get(idx), col.mv1.get(idx)) {
                (Some(&r1), Some(&m)) if r1 >= 0 => (r1, m),
                _ => return false,
            },
            None => return false,
        };
        cref == 0 && cmv.0.abs() <= 1 && cmv.1.abs() <= 1
    }

    /// Spatial direct prediction for a region (whole MB or an 8×8): derives the
    /// per-list reference indices and base MVs, then motion-compensates each 4×4
    /// sub-block (applying `colZeroFlag`) and commits the motion (spec §8.4.1.2.2).
    pub(crate) fn decode_b_direct(
        &mut self,
        region: RegionB,
        pred_y: &mut [u8; 256],
        c_pred: &mut [[u8; 64]; 2],
    ) -> Result<(), MbError> {
        let RegionB {
            mb_x,
            mb_y,
            px,
            py,
            rw,
            rh,
        } = region;
        if !self.direct_spatial {
            return self.decode_b_direct_temporal(
                RegionB {
                    mb_x,
                    mb_y,
                    px,
                    py,
                    rw,
                    rh,
                },
                pred_y,
                c_pred,
            );
        }
        let (n0, n1) = self.b_direct_nbrs(mb_x, mb_y)?;
        self.decode_b_direct_n(
            RegionB {
                mb_x,
                mb_y,
                px,
                py,
                rw,
                rh,
            },
            pred_y,
            c_pred,
            (n0, n1),
        )?;
        Ok(())
    }

    /// Spec §8.4.1.2.2: the spatial-direct reference indices (min positive over
    /// the three neighbors, per list) and the predicted MVs. `direct_zero` is
    /// the both-lists-unavailable case, which forces ref 0 / mv (0,0). Shared
    /// by `decode_b_direct_n` and the B_Skip zero-bi fast path — ONE derivation,
    /// no drift.
    #[inline]
    pub(crate) fn b_direct_refs_mvs(
        n0: &[MvNeighbor; 3],
        n1: &[MvNeighbor; 3],
    ) -> (i32, i32, (i32, i32), (i32, i32), bool) {
        let min_pos = |a: i32, b: i32| {
            if a < 0 {
                b
            } else if b < 0 {
                a
            } else {
                a.min(b)
            }
        };
        let rid = |n: &[MvNeighbor; 3]| min_pos(min_pos(n[0].ref_idx, n[1].ref_idx), n[2].ref_idx);
        let (mut refi0, mut refi1) = (rid(n0), rid(n1));
        let direct_zero = refi0 < 0 && refi1 < 0;
        if direct_zero {
            refi0 = 0;
            refi1 = 0;
        }
        let mv0 = if refi0 >= 0 && !direct_zero {
            predict_mv(n0[0], n0[1], n0[2], refi0)
        } else {
            (0, 0)
        };
        let mv1 = if refi1 >= 0 && !direct_zero {
            predict_mv(n1[0], n1[1], n1[2], refi1)
        } else {
            (0, 0)
        };
        (refi0, refi1, mv0, mv1, direct_zero)
    }

    /// Predicción directa espacial de `region` con los vecinos A/B/C de las dos listas ya
    /// recogidos: deriva referencias y vectores (§8.4.1.2.2) y compensa con `b_direct_region`.
    ///
    /// # Errors
    /// Los de la compensación de la región.
    pub(crate) fn decode_b_direct_n(
        &mut self,
        region: RegionB,
        pred_y: &mut [u8; 256],
        c_pred: &mut [[u8; 64]; 2],
        (n0, n1): ([MvNeighbor; 3], [MvNeighbor; 3]),
    ) -> Result<(), MbError> {
        let RegionB {
            mb_x,
            mb_y,
            px,
            py,
            rw,
            rh,
        } = region;
        // H-48: DERIVATION-ONLY scope, dropped before the MC loop below. DecBDirect
        // wraps this function whole and therefore INCLUDES the `b_mc` calls it makes,
        // so its 1460 ns/call was never "MV derivation is slow" — that read was wrong.
        // This guard is what separates the two.
        let derived = Self::b_direct_refs_mvs(&n0, &n1);
        self.b_direct_region(
            RegionB {
                mb_x,
                mb_y,
                px,
                py,
                rw,
                rh,
            },
            pred_y,
            c_pred,
            derived,
        )?;
        Ok(())
    }

    /// The post-derivation half of spatial direct: czg probing (gated), region
    /// coalescing, MC + motion commit. Split out so B_8x8 direct subs derive
    /// ONCE per MB (the A/B/C neighbours and the rid/median result are
    /// MB-level; only czg varies per 8x8).
    pub(crate) fn b_direct_region(
        &mut self,
        region: RegionB,
        pred_y: &mut [u8; 256],
        c_pred: &mut [[u8; 64]; 2],
        derived: (i32, i32, (i32, i32), (i32, i32), bool),
    ) -> Result<(), MbError> {
        let RegionB {
            mb_x,
            mb_y,
            px,
            py,
            rw,
            rh,
        } = region;
        let (refi0, refi1, mv0, mv1, direct_zero) = derived;
        // Per 4×4 sub-block: colZeroFlag zeroes the ref-0 motion vector. cz is the
        // ONLY per-block variable (two possible (m0,m1) values for the region), and
        // the MC filters + bi-blend are per-output-pixel — so sub-blocks with equal
        // cz coalesce into one wider `b_mc`, BIT-IDENTICAL. A 16×16 direct MB paid
        // 16 bi-pred b_mc calls (~96 MC kernel entries) before this; typically 1 now.
        let (bx0, by0, bw, bh) = (px / 4, py / 4, rw / 4, rh / 4);
        // MASKED at every use below. The loop bounds are the region's `bw`/`bh` in
        // 4x4 units, which are always <= 4 for a macroblock but are RUNTIME values,
        // so none of the eight indexes into this fixed 4x4 grid was provable.
        let mut czg = [[false; 4]; 4]; // region-local, [dy][dx]
                                       // colZeroFlag can only change a list whose ref is 0 AND whose predicted
                                       // MV is nonzero (it zeroes MVs; zeroing (0,0) is a no-op). When neither
                                       // list qualifies, skip the probing entirely: czg stays false, the
                                       // region is uniform, and the m values are identical — the only change
                                       // is FEWER b_mc calls where the old czg would have split rects with
                                       // equal values (tiles of the same math, byte-identical).
        let cz_matters = (refi0 == 0 && mv0 != (0, 0)) || (refi1 == 0 && mv1 != (0, 0));
        // Uniformity is KNOWN after the probe fill — when every probe agreed,
        // the 16-bool coalesce scan and its recursion are skipped outright.
        let mut cz_mixed = false;
        if cz_matters && !direct_zero && self.direct_8x8_inference {
            // Under direct_8x8_inference every 4×4 in an 8×8 shares one colZeroFlag
            // (col_block collapses to the MB-corner). Probe once per 8×8 — same
            // czg, fewer wait_motion_ready / meta locks on the live-ref path.
            let mut oy = 0usize;
            while oy < bh {
                let h = (bh - oy).min(2);
                let mut ox = 0usize;
                while ox < bw {
                    let w = (bw - ox).min(2);
                    let (colx, coly) = self.col_block(bx0 + ox, by0 + oy);
                    let cz = self.col_zero(mb_x * 4 + colx, mb_y * 4 + coly);
                    cz_mixed |= cz != czg[0][0] && !(ox == 0 && oy == 0);
                    for dy in oy..oy + h {
                        for dx in ox..ox + w {
                            czg[dy & 3][dx & 3] = cz;
                        }
                    }
                    ox += w;
                }
                oy += h;
            }
        } else if cz_matters && !direct_zero {
            for dy in 0..bh {
                for dx in 0..bw {
                    let (colx, coly) = self.col_block(bx0 + dx, by0 + dy);
                    let cz = self.col_zero(mb_x * 4 + colx, mb_y * 4 + coly);
                    cz_mixed |= cz != czg[0][0] && !(dx == 0 && dy == 0);
                    czg[dy & 3][dx & 3] = cz;
                }
            }
        }
        let mut rects: [(usize, usize, usize, usize); 16] = [(0, 0, 0, 0); 16];
        let mut n = 0usize;
        if !cz_mixed {
            rects[0] = (0, 0, bw, bh);
            n = 1;
        } else {
            let uniform = |x: usize, y: usize, w: usize, h: usize| -> bool {
                let t = czg[y & 3][x & 3];
                (y..y + h).all(|dy| (x..x + w).all(|dx| czg[dy & 3][dx & 3] == t))
            };
            Self::coalesce_region(0, 0, bw, bh, &uniform, &mut |x, y, w, h| {
                rects[n & 15] = (x, y, w, h);
                n += 1;
            });
        }
        for &(x, y, w, h) in &rects[..n] {
            let cz = czg[y & 3][x & 3];
            let m0 = if refi0 == 0 && cz { (0, 0) } else { mv0 };
            let m1 = if refi1 == 0 && cz { (0, 0) } else { mv1 };
            let (lx, ly, lw, lh) = ((bx0 + x) * 4, (by0 + y) * 4, w * 4, h * 4);
            self.b_mc(
                RegionB {
                    mb_x,
                    mb_y,
                    px: lx,
                    py: ly,
                    rw: lw,
                    rh: lh,
                },
                MovimientoBi {
                    refi0,
                    mv0: m0,
                    refi1,
                    mv1: m1,
                },
                pred_y,
                c_pred,
            )?;

            self.b_set_motion(
                RegionB {
                    mb_x,
                    mb_y,
                    px: lx,
                    py: ly,
                    rw: lw,
                    rh: lh,
                },
                MovimientoBi {
                    refi0,
                    mv0: m0,
                    refi1,
                    mv1: m1,
                },
            );
        }
        Ok(())
    }

    /// Temporal direct prediction for a region (spec §8.4.1.2.3): for each 4×4
    /// (or per-8×8 corner under `direct_8x8_inference`), take the co-located
    /// List-0 motion from `RefPicList1[0]`, map its reference into the current
    /// List-0 by POC, and scale the motion vector by the POC distances.
    pub(crate) fn decode_b_direct_temporal(
        &mut self,
        region: RegionB,
        pred_y: &mut [u8; 256],
        c_pred: &mut [[u8; 64]; 2],
    ) -> Result<(), MbError> {
        let RegionB {
            mb_x,
            mb_y,
            px,
            py,
            rw,
            rh,
        } = region;
        let poc1 = self
            .refs1
            .first()
            .ok_or(MbError::Inconsistent("refs1"))?
            .pic_poc();
        let infer = self.direct_8x8_inference;
        // Under direct_8x8_inference every 4×4 in an 8×8 takes the same MB-corner
        // co-located motion, so motion-compensate the whole 8×8 in one call — this
        // hits the width-8 MC asm and pays the per-call tile/blend setup 4× less.
        // Without inference, motion is genuinely per-4×4. Bit-identical either way
        // (MC of an 8×8 with one MV == four 4×4 MCs with that same MV).
        let step = if infer { 8 } else { 4 };
        let mut sy = py;
        while sy < py + rh {
            let mut sx = px;
            while sx < px + rw {
                // Co-located 4×4 (the 8×8's MB-corner under inference) — shared with
                // the spatial path's colZeroFlag, which must map identically.
                let (colx, coly) = self.col_block(sx / 4, sy / 4);
                let (mvcol, refpoc) = {
                    let col = self.refs1.first().ok_or(MbError::Inconsistent("refs1"))?;
                    let idx = (mb_y * 4 + coly) * col.w4 + (mb_x * 4 + colx);
                    // intra co-located → zero motion, refIdxL0 = 0
                    if col.w4 == 0 {
                        ((0, 0), i32::MIN)
                    } else {
                        match (col.mv.get(idx), col.ref_poc.get(idx)) {
                            (Some(&m), Some(&pc)) if pc != i32::MIN => (m, pc),
                            (Some(_), Some(_)) => ((0, 0), i32::MIN),
                            _ => return Err(MbError::Inconsistent("col")),
                        }
                    }
                };
                // MapColToList0 (§8.4.1.2.3): el menor índice de la lista actual de la
                // referencia coubicada; «RefPicList0 shall contain refPicCol».
                let (refi0, mvc) = if refpoc == i32::MIN {
                    (0, (0, 0))
                } else {
                    let r = self
                        .refs
                        .iter()
                        .position(|f| f.pic_poc() == refpoc)
                        .ok_or(MbError::Inconsistent("refPicCol"))?;
                    (r, mvcol)
                };
                let r0 = self.refs.get(refi0).ok_or(MbError::Inconsistent("refs"))?;
                let refi0 = i32::try_from(refi0).map_err(|_| MbError::Inconsistent("refs"))?;
                let poc0 = r0.pic_poc();
                let escala = factor_escala_distancia(self.cur_poc, poc0, poc1);
                let (mv0, mv1) = match escala {
                    Some(dsf) if !r0.long_term => {
                        let m0 = ((dsf * mvc.0 + 128) >> 8, (dsf * mvc.1 + 128) >> 8);
                        (m0, (m0.0 - mvc.0, m0.1 - mvc.1))
                    }
                    _ => (mvc, (0, 0)),
                };
                self.b_mc_or_record(
                    RegionB {
                        mb_x,
                        mb_y,
                        px: sx,
                        py: sy,
                        rw: step,
                        rh: step,
                    },
                    MovimientoBi {
                        refi0,
                        mv0,
                        refi1: 0,
                        mv1,
                    },
                    pred_y,
                    c_pred,
                )?;
                self.b_set_motion(
                    RegionB {
                        mb_x,
                        mb_y,
                        px: sx,
                        py: sy,
                        rw: step,
                        rh: step,
                    },
                    MovimientoBi {
                        refi0,
                        mv0,
                        refi1: 0,
                        mv1,
                    },
                );
                sx += step;
            }
            sy += step;
        }
        Ok(())
    }
}
