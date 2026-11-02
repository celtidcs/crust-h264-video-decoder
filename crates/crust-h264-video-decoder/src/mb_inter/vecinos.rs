//! Vecinos y predicción de vectores de movimiento (UIT-T H.264 §8.4.1.3).

use crate::error::MbError;
use crate::mb16::FrameDecoder;
use crust_h264_common::inter::{predict_mv, MvNeighbor};

impl FrameDecoder {
    pub(crate) fn mv_neighbors(
        &self,
        mb_x: usize,
        mb_y: usize,
    ) -> Result<[MvNeighbor; 3], MbError> {
        let w4 = self.mb_w * 4;
        let get = |avail: bool, bx: isize, by: isize| -> Result<MvNeighbor, MbError> {
            if avail {
                let idx = by as usize * w4 + bx as usize;
                match (self.mv_y.get(idx), self.ref_idx_y.get(idx)) {
                    (Some(&m), Some(&r)) => Ok(MvNeighbor {
                        available: true,
                        mv: m,
                        ref_idx: r,
                    }),
                    _ => Err(MbError::Inconsistent("mv")),
                }
            } else {
                Ok(MvNeighbor::NONE)
            }
        };
        let (bx, by) = (mb_x as isize * 4, mb_y as isize * 4);
        let a = get(mb_x > 0 && self.nbr_in_slice(mb_x - 1, mb_y), bx - 1, by)?;
        let b = get(mb_y > 0 && self.nbr_in_slice(mb_x, mb_y - 1), bx, by - 1)?;
        let c = if mb_y > 0 && mb_x + 1 < self.mb_w && self.nbr_in_slice(mb_x + 1, mb_y - 1) {
            get(true, bx + 4, by - 1)?
        } else {
            get(
                mb_x > 0 && mb_y > 0 && self.nbr_in_slice(mb_x - 1, mb_y - 1),
                bx - 1,
                by - 1,
            )?
        };
        Ok([a, b, c])
    }

    pub(crate) fn mv_neighbors_block(
        &self,
        pbx: isize,
        pby: isize,
        pwb: isize,
    ) -> Result<[MvNeighbor; 3], MbError> {
        self.mv_neighbors_block_grid(pbx, pby, pwb, 0)
    }

    pub(crate) fn skip_mv(&self, mb_x: usize, mb_y: usize) -> Result<(i32, i32), MbError> {
        let [a, b, c] = self.mv_neighbors(mb_x, mb_y)?;
        Ok(
            if !a.available
                || !b.available
                || (a.ref_idx == 0 && a.mv == (0, 0))
                || (b.ref_idx == 0 && b.mv == (0, 0))
            {
                (0, 0)
            } else {
                predict_mv(a, b, c, 0)
            },
        )
    }

    pub(crate) fn set_mb_mv(
        &mut self,
        mb_x: usize,
        mb_y: usize,
        mv: (i32, i32),
        inter: bool,
        refi: i32,
    ) {
        // ROW FILLS. This wrote all sixteen cells of three grids one indexed
        // store at a time - FORTY-EIGHT separate bounds checks per call - and
        // re-evaluated `if inter` inside the inner loop. Each macroblock row is
        // four CONTIGUOUS cells, so twelve `fill`s cover it.
        let w4 = self.mb_w * 4;
        let r = if inter { refi } else { -1 };
        for dy in 0..4 {
            let a = (mb_y * 4 + dy) * w4 + mb_x * 4;
            self.mv_y[a..a + 4].fill(mv);
            self.inter_y[a..a + 4].fill(inter);
            self.ref_idx_y[a..a + 4].fill(r);
        }
    }

    /// Commit one inter partition's motion into the 4×4 grid (ref 0, 1-ref P).
    /// `(rx,ry,rw,rh)` are MB-relative luma pixels; committing before the next
    /// partition's prediction is what lets a later partition predict from it.
    pub(crate) fn commit_inter_grid(
        &mut self,
        (mb_x, mb_y): (usize, usize),
        rect: (usize, usize, usize, usize),
        mv: (i32, i32),
        refi: i8,
    ) {
        let (rx, ry, rw, rh) = rect;
        // Row-range fills instead of the per-4x4 scatter: partitions are
        // rectangles, so each grid row is one contiguous run (same values).
        let w4 = self.mb_w * 4;
        let (bx0, bw) = (mb_x * 4 + rx / 4, rw / 4);
        for by in ry / 4..ry / 4 + rh / 4 {
            let a = (mb_y * 4 + by) * w4 + bx0;
            self.mv_y[a..a + bw].fill(mv);
            self.inter_y[a..a + bw].fill(true);
            self.ref_idx_y[a..a + bw].fill(refi as i32);
            self.coded_y[a..a + bw].fill(true);
        }
    }

    // ---------------------------------------------------------------------
    // B-slice macroblock decoding
    // ---------------------------------------------------------------------

    /// Per-list (`list` 0 or 1) MV-prediction neighbors for the block region at
    /// `(pbx, pby)` of width `pwb` blocks — the L0/L1 analogue of
    /// `mv_neighbors_block`.
    pub(crate) fn mv_neighbors_list(
        &self,
        pbx: isize,
        pby: isize,
        pwb: isize,
        list: usize,
    ) -> Result<[MvNeighbor; 3], MbError> {
        self.mv_neighbors_block_grid(pbx, pby, pwb, list)
    }

    /// Spatial-direct A/B/C at the MB origin — same for every 8×8 in the MB
    /// (spec §8.4.1.2.2). B_8x8 callers hoist this once; 16×16 skip/direct walk
    /// once inside `decode_b_direct`.
    #[inline]
    pub(crate) fn b_direct_nbrs(
        &self,
        mb_x: usize,
        mb_y: usize,
    ) -> Result<([MvNeighbor; 3], [MvNeighbor; 3]), MbError> {
        self.mv_neighbors_both((mb_x * 4) as isize, (mb_y * 4) as isize, 4)
    }

    /// FUSED dual-list gather at arbitrary partition geometry: A/B/C positions
    /// and their availability (bounds + coded + slice) are list-independent —
    /// compute once, load both lists' grids from the same resolved index.
    pub(crate) fn mv_neighbors_both(
        &self,
        pbx: isize,
        pby: isize,
        pwb: isize,
    ) -> Result<([MvNeighbor; 3], [MvNeighbor; 3]), MbError> {
        let (w4, h4) = ((self.mb_w * 4) as isize, (self.mb_h * 4) as isize);
        let get2 = |bx: isize, by: isize| -> Result<(MvNeighbor, MvNeighbor), MbError> {
            if bx < 0
                || by < 0
                || bx >= w4
                || by >= h4
                || !self
                    .coded_y
                    .get((by * w4 + bx) as usize)
                    .copied()
                    .ok_or(MbError::Inconsistent("coded_y"))?
                || !self.nbr_in_slice(bx as usize / 4, by as usize / 4)
            {
                Ok((MvNeighbor::NONE, MvNeighbor::NONE))
            } else {
                // FOUR PARALLEL GRIDS at one index. The `coded_y` guard above
                // no acota ninguna: son `Vec` distintos. Una celda ausente
                // en cualquiera de ellas es un estado incoherente (COR-3), NO un
                // vecino no disponible (que la norma lee como mv 0, ref -1).
                let idx = (by * w4 + bx) as usize;
                match (
                    self.mv_y.get(idx),
                    self.ref_idx_y.get(idx),
                    self.mv1.get(idx),
                    self.ref_idx1.get(idx),
                ) {
                    (Some(&m0), Some(&r0), Some(&m1), Some(&r1)) => Ok((
                        MvNeighbor {
                            available: true,
                            mv: m0,
                            ref_idx: r0,
                        },
                        MvNeighbor {
                            available: true,
                            mv: m1,
                            ref_idx: r1,
                        },
                    )),
                    _ => Err(MbError::Inconsistent("mv")),
                }
            }
        };
        let (a0, a1) = get2(pbx - 1, pby)?;
        let (b0, b1) = get2(pbx, pby - 1)?;
        let (mut c0, mut c1) = get2(pbx + pwb, pby - 1)?;
        if !c0.available {
            // The C fallback is position-driven (topright unavailable ⇒
            // topleft), so both lists fall back together — same decision the
            // two per-list gathers made independently.
            let t = get2(pbx - 1, pby - 1)?;
            c0 = t.0;
            c1 = t.1;
        }
        Ok(([a0, b0, c0], [a1, b1, c1]))
    }

    pub(crate) fn mv_neighbors_block_grid(
        &self,
        pbx: isize,
        pby: isize,
        pwb: isize,
        list: usize,
    ) -> Result<[MvNeighbor; 3], MbError> {
        let (w4, h4) = ((self.mb_w * 4) as isize, (self.mb_h * 4) as isize);
        let (mvg, refg) = if list == 0 {
            (&self.mv_y, &self.ref_idx_y)
        } else {
            (&self.mv1, &self.ref_idx1)
        };
        let get = |bx: isize, by: isize| -> Result<MvNeighbor, MbError> {
            if bx < 0
                || by < 0
                || bx >= w4
                || by >= h4
                || !self
                    .coded_y
                    .get((by * w4 + bx) as usize)
                    .copied()
                    .ok_or(MbError::Inconsistent("coded_y"))?
                || !self.nbr_in_slice(bx as usize / 4, by as usize / 4)
            {
                Ok(MvNeighbor::NONE)
            } else {
                let idx = (by * w4 + bx) as usize;
                match (mvg.get(idx), refg.get(idx)) {
                    (Some(&m), Some(&r)) => Ok(MvNeighbor {
                        available: true,
                        mv: m,
                        ref_idx: r,
                    }),
                    _ => Err(MbError::Inconsistent("mv")),
                }
            }
        };
        let a = get(pbx - 1, pby)?;
        let b = get(pbx, pby - 1)?;
        let mut c = get(pbx + pwb, pby - 1)?;
        if !c.available {
            c = get(pbx - 1, pby - 1)?;
        }
        Ok([a, b, c])
    }

    /// Vectores de la lista 0 y referencias (recortadas a 0..=15) de los 16 bloques 4x4 del
    /// macrobloque `(mb_x, mb_y)` en orden de barrido, tal como los lleva un trabajo EDC de
    /// reconstrucción P (`PInterJob` / `PInterNoResJob`).
    pub(crate) fn movimiento_para_trabajo(
        &self,
        mb_x: usize,
        mb_y: usize,
    ) -> ([(i32, i32); 16], [u8; 16]) {
        let (mut gmv, mut gref) = ([(0i32, 0i32); 16], [0u8; 16]);
        let w4r = self.mb_w * 4;
        // ROW SLICES over BOTH grids. This was the single densest panic site
        // left in the decoder crate: sixteen iterations reading two PARALLEL
        // frame grids at the same index, and `mv_y`'s bound proved nothing
        // about `ref_idx_y`, so all thirty-two loads carried their own check.
        // Each macroblock row is four contiguous cells in both.
        for by in 0..4usize {
            let base = (mb_y * 4 + by) * w4r + mb_x * 4;
            let mvr = &self.mv_y[base..][..4];
            let rfr = &self.ref_idx_y[base..][..4];
            gmv[by * 4..][..4].copy_from_slice(mvr);
            for bx in 0..4usize {
                gref[by * 4 + bx] = rfr[bx].clamp(0, super::MAX_REF_TRABAJO) as u8;
            }
        }
        (gmv, gref)
    }
}
