//! Reconstrucción de los P inter y tramos de `P_Skip` diferidos (§8.4.2).

use crate::edc::{coalesce_p_inter_mc, PInterJob, PInterNoResJob};
use crate::error::MbError;
use crate::mb16::FrameDecoder;
use crate::mb_inter::ResiduoInter;
use crusty_h264_common::inter::{mc_chroma_padded, mc_luma_padded};

impl FrameDecoder {
    /// Reconstructs one CABAC P inter macroblock from its parse job — the
    /// pixel half of the entropy-decouple seam (docs/entropy-decouple-plan.md
    /// E1). Reads NOTHING from parse state except the frame grids this MB's
    /// parse already committed (its own block MVs/refs, re-gathered below —
    /// stable after commit) and the immutable DPB; called either inline
    /// (seam off / flush disabled) or in-order at a flush point. Byte-
    /// identical to the former inline block by construction: replay order
    /// equals inline order at every pixel-observable point (intra reads, row
    /// filtering) because flushes precede both.
    /// The P-inter reconstruction body, taking its inputs directly rather than
    /// through a `PInterJob`. Split out so the residual planes can be passed as
    /// `Option`s (absent == nothing was parsed) instead of forcing every caller
    /// to own a zeroed copy. NOTE: it re-gathers `gmv`/`gref` from the frame
    /// grids rather than reading the job's copies — those are for the worker,
    /// which must not touch the parse thread's grids.
    pub(crate) fn recon_p_inter_parts(
        &mut self,
        mbx: usize,
        mby: usize,
        qp: u8,
        res: ResiduoInter<'_>,
    ) -> Result<(), MbError> {
        let ResiduoInter {
            cbp_chroma,
            luma_scan,
            luma8,
            cdc,
            cac,
            nnzs,
        } = res;
        // `add_inter_residual` (and anything under it) reads `self.cur_qp`,
        // which at FLUSH time belongs to a later macroblock — replay must
        // restore this MB's qp. The x264 corpus (near-constant QP) could not
        // see this; the encoder's delta-QP roundtrip stream caught it.
        let saved_qp = self.cur_qp;
        self.cur_qp = qp;
        // ---- Recon: motion-comp (per 4×4 luma / co-located 2×2 chroma using the
        // committed grid MV — the 6-tap/bilinear filter is per-output-pixel, so
        // per-block MC is bit-identical to per-partition MC) + residual add via the
        // SAME reconstruct_4x4 as intra, with the MC output as the prediction.
        let mut pred_y = [0u8; 256];
        let mut c_pred = [[0u8; 64]; 2];
        self.prediccion_p((mbx, mby), &mut pred_y, &mut c_pred)?;
        // Residual add — the SAME helper the B path uses (this inline
        // copy was a duplicate; deduped when the zero-block fast path
        // landed so both paths share it).
        self.add_inter_residual(
            mbx,
            mby,
            &pred_y,
            &c_pred,
            ResiduoInter {
                luma_scan,
                luma8,
                cdc,
                cac,
                cbp_chroma,
                nnzs,
            },
        );
        self.cur_qp = saved_qp;
        Ok(())
    }

    /// Predicción de un P inter con su movimiento ya en las rejillas: compensa agrupando los
    /// bloques 4x4 de igual vector y referencia (§8.4.2.2) y aplica la predicción ponderada
    /// explícita si el corte la lleva (§8.4.2.3).
    ///
    /// # Errors
    /// `MbError::Inconsistent` si una referencia de la rejilla no está en la lista, y los del
    /// recorte de los planos.
    fn prediccion_p(
        &self,
        (mbx, mby): (usize, usize),
        pred_y: &mut [u8; 256],
        c_pred: &mut [[u8; 64]; 2],
    ) -> Result<(), MbError> {
        let w4r = self.mb_w * 4;
        // MC-CALL COALESCING (side-by-side descent, dec target #2): the old
        // loop paid 16 mc_luma(4×4) + 32 mc_chroma(2×2) per MB regardless of
        // partitioning — 48 calls even for a single-MV 16×16 MB, and the
        // per-call glue around 2.4M calls was ~40% of decoding real-world
        // (x264) streams. The 6-tap/bilinear filters are per-output-pixel,
        // so merging blocks with equal (mv, ref) into one wider MC call is
        // BIT-IDENTICAL; the rect ladder mirrors the partition shapes.
        let (rh16, cch) = (self.mb_h * 16, self.mb_h * 8);
        let mut gmv = [(0i32, 0i32); 16];
        let mut gref = [0usize; 16];
        let nrefs = self.refs.len() - 1;
        for by in 0..4usize {
            // Row-contiguous: one slice copy for the MVs.
            let row = (mby * 4 + by) * w4r + mbx * 4;
            gmv[by * 4..by * 4 + 4].copy_from_slice(&self.mv_y[row..row + 4]);
            let ridx = &self.ref_idx_y[row..row + 4];
            for bx in 0..4usize {
                // Per-block reference (multi-ref P): ref_idx_l0 committed to the
                // grid. Clamp — a corrupt stream can over-range it (never panic).
                gref[by * 4 + bx] = (ridx[bx].max(0) as usize).min(nrefs);
            }
        }
        // All blocks of the rect (in 4×4-block units) match its top-left?
        let rect_eq = |x4: usize, y4: usize, w4: usize, h4: usize| -> bool {
            let t = y4 * 4 + x4;
            (0..h4).all(|dy| {
                (0..w4).all(|dx| {
                    let b = ((y4 + dy) * 4 + (x4 + dx)) & 15;
                    gmv[b] == gmv[t] && gref[b] == gref[t]
                })
            })
        };
        let refs = &self.refs;
        let (cw, ccw) = (self.cw, self.ccw);
        let mc_rect = |x4: usize,
                       y4: usize,
                       w4: usize,
                       h4: usize,
                       pred_y: &mut [u8; 256],
                       c_pred: &mut [[u8; 64]; 2]|
         -> Result<(), MbError> {
            let b = y4 * 4 + x4;
            let (mv, gr) = (gmv[b & 15], gref[b & 15]);
            // `gref` guarda una posición de la lista de referencias; una posición que la lista
            // no tiene es un estado incoherente (COR-3), no «nada que predecir».
            let reference = refs.get(gr).ok_or(MbError::Inconsistent("refs"))?;
            let (w, h) = (w4 * 4, h4 * 4);
            // A FULL-WIDTH rect (w == 16, so x4 == 0) occupies contiguous
            // whole rows of `pred_y` — the MC output layout and the
            // destination layout coincide, so MC writes the prediction
            // buffer DIRECTLY. The staging copy exists only for narrow
            // rects, whose rows really are strided in `pred_y`. This is
            // the diagnosis's "stage-boundary materialization" tax paid
            // by the dominant 16×16/16×8 shapes: 256 B of `t` zeroing
            // plus a 256 B copy per rect, for nothing.
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
                    pred_y[(y4 * 4 + dy) * 16 + x4 * 4..][..w]
                        .copy_from_slice(&t[dy * w..dy * w + w]);
                }
            }
            let (cw4, ch4) = (w4 * 2, h4 * 2);
            let nc = cw4 * ch4;
            let (gu, gv) = (reference.plano_croma(0), reference.plano_croma(1));
            // U+V paired: one setup serves both planes (see
            // mc_chroma_padded_pair). Full-width coincidence:
            // cw4 == 8 rows are contiguous in the 8-wide plane.
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
            mc_rect(0, 0, 4, 4, pred_y, c_pred)?;
        } else if rect_eq(0, 0, 4, 2) && rect_eq(0, 2, 4, 2) {
            mc_rect(0, 0, 4, 2, pred_y, c_pred)?;
            mc_rect(0, 2, 4, 2, pred_y, c_pred)?;
        } else if rect_eq(0, 0, 2, 4) && rect_eq(2, 0, 2, 4) {
            mc_rect(0, 0, 2, 4, pred_y, c_pred)?;
            mc_rect(2, 0, 2, 4, pred_y, c_pred)?;
        } else {
            for q in 0..4usize {
                let (qx, qy) = ((q % 2) * 2, (q / 2) * 2);
                if rect_eq(qx, qy, 2, 2) {
                    mc_rect(qx, qy, 2, 2, pred_y, c_pred)?;
                } else if rect_eq(qx, qy, 2, 1) && rect_eq(qx, qy + 1, 2, 1) {
                    mc_rect(qx, qy, 2, 1, pred_y, c_pred)?;
                    mc_rect(qx, qy + 1, 2, 1, pred_y, c_pred)?;
                } else if rect_eq(qx, qy, 1, 2) && rect_eq(qx + 1, qy, 1, 2) {
                    mc_rect(qx, qy, 1, 2, pred_y, c_pred)?;
                    mc_rect(qx + 1, qy, 1, 2, pred_y, c_pred)?;
                } else {
                    for j in 0..4usize {
                        mc_rect(qx + (j % 2), qy + (j / 2), 1, 1, pred_y, c_pred)?;
                    }
                }
            }
        }
        // EXPLICIT WEIGHTED PREDICTION (spec 8.4.2.3). The CAVLC inter
        // path weights each partition after MC; the MC-call-coalescing
        // rewrite of this CABAC path lost it, and nothing caught that
        // because the effect is invisible unless a stream actually
        // carries non-default weights. x264's `weightp` DUPLICATES a
        // reference and distinguishes the copy ONLY by its weights, so
        // every macroblock picking the weighted index decoded unweighted
        // -- a silent, accumulating luma drift.
        //
        // Applied per 4x4 block rather than per partition: the weight
        // depends solely on the block's reference index, so the two are
        // equivalent, and `gref` already holds it for every block
        // regardless of which rect ladder rung ran.
        if self.weights.is_some() {
            for by in 0..4usize {
                for bx in 0..4usize {
                    let refi = gref[by * 4 + bx];
                    self.weight_partition(pred_y, c_pred, 0, refi, (bx * 4, by * 4, 4, 4))?;
                }
            }
        }
        Ok(())
    }

    /// Job-shaped entry point (EDC replay).
    pub(crate) fn recon_p_inter(&mut self, j: &PInterJob) -> Result<(), MbError> {
        self.recon_p_inter_parts(
            j.mbx,
            j.mby,
            j.qp,
            ResiduoInter {
                cbp_chroma: j.cbp_chroma,
                luma_scan: Some(&j.luma_scan),
                luma8: j.t8.then_some(&j.luma8),
                cdc: &j.cdc,
                cac: Some(&j.cac),
                nnzs: &j.nnzs,
            },
        )?;
        Ok(())
    }

    /// D9b: P inter with `cbp == 0` — MC + plane copy, no coeff memset / residual walk.
    /// Byte-identical to `recon_p_inter` on a zero-residual job: prediction is the recon.
    pub(crate) fn recon_p_inter_nores(&mut self, j: &PInterNoResJob) -> Result<(), MbError> {
        let w4r = self.mb_w * 4;
        // Parse-side nnz for single-thread EDC flush (MT commits earlier via edc_commit_nnz).
        // ROW FILLS. Both arms touch the same sixteen cells - four contiguous
        // per macroblock row - and wrote them one indexed store at a time (the
        // t8 arm through a 2x2-of-2x2 nest, the other through the z-order scan).
        for by in 0..4usize {
            let r = (j.mby * 4 + by) * w4r + j.mbx * 4;
            self.nnz_y[r..r + 4].fill(0);
            if j.t8 {
                self.coded_y[r..r + 4].fill(true);
            }
        }
        let mut pred_y = [0u8; 256];
        let mut c_pred = [[0u8; 64]; 2];
        // `self.refs.len() - 1` was re-loaded on every one of the sixteen
        // iterations; it is a per-macroblock invariant. Written once, not
        // zeroed-then-filled.
        let nrefs = self.refs.len() - 1;
        let gref: [usize; 16] = core::array::from_fn(|k| (j.gref[k] as usize).min(nrefs));
        coalesce_p_inter_mc(
            &self.refs,
            self.cw,
            self.ccw,
            self.mb_h,
            j.mbx,
            j.mby,
            &j.gmv,
            &gref,
            &mut pred_y,
            &mut c_pred,
        )?;
        // The identity early-out lives INSIDE `weight_partition`, so an x264
        // stream - which carries a pred_weight_table in EVERY P slice, identity
        // outside fades - still paid sixteen calls per macroblock to be told
        // there was nothing to do. Hoisted to one test.
        if self.weights.is_some() && !self.weights_l0id {
            for by in 0..4usize {
                for bx in 0..4usize {
                    let refi = gref[by * 4 + bx];
                    self.weight_partition(
                        &mut pred_y,
                        &mut c_pred,
                        0,
                        refi,
                        (bx * 4, by * 4, 4, 4),
                    )?;
                }
            }
        }
        // ONE span per plane: the destination is a stack of contiguous runs a
        // fixed stride apart, so the rows share a single bounds check instead of
        // one each (16 + 8 + 8 of them).
        let cw = self.cw;
        let ybase = (j.mby * 16) * cw + j.mbx * 16;
        let ywin = &mut self.rec_y[ybase..ybase + 15 * cw + 16];
        for dy in 0..16 {
            ywin[dy * cw..dy * cw + 16].copy_from_slice(&pred_y[dy * 16..dy * 16 + 16]);
        }
        self.escribir_prediccion_croma((j.mbx, j.mby), &c_pred);
        Ok(())
    }

    /// Defer one (0,0) P_Skip's grid commit into the pending P span.
    #[inline]
    pub(crate) fn pz_push(&mut self, mb_x: usize, mb_y: usize, recon: bool) -> Result<(), MbError> {
        match self.pzspan {
            Some((row, x0, ref mut n, r)) if row == mb_y && x0 + *n == mb_x && r == recon => {
                *n += 1
            }
            _ => {
                self.pz_flush()?;
                self.pzspan = Some((mb_y, mb_x, 1, recon));
            }
        }
        Ok(())
    }

    /// Range-fill the pending P span's grids (the deferral constants of
    /// decode_p_skip's commit: list-0 ref 0, mv (0,0), inter, coded, DC mode,
    /// deblock kind SKIP). Same flush points as the B span (span_flush).
    #[inline(always)]
    pub(crate) fn pz_flush(&mut self) -> Result<(), MbError> {
        if self.pzspan.is_some() {
            self.pz_flush_slow()?;
        }
        Ok(())
    }

    pub(crate) fn pz_flush_slow(&mut self) -> Result<(), MbError> {
        let Some((row, x0, n, recon)) = self.pzspan.take() else {
            return Ok(());
        };
        let w4 = self.mb_w * 4;
        let (b0, len) = (x0 * 4, n * 4);
        for dy in 0..4 {
            let a = (row * 4 + dy) * w4 + b0;
            self.mv_y[a..a + len].fill((0, 0));
            self.inter_y[a..a + len].fill(true);
            self.ref_idx_y[a..a + len].fill(0);
            self.coded_y[a..a + len].fill(true);
            self.modes_y[a..a + len].fill(2);
        }
        self.mb_kind[row * self.mb_w + x0..row * self.mb_w + x0 + n]
            .fill(crusty_h264_common::deblock::MB_KIND_SKIP);
        if recon {
            // The recon deferred with the commit: one band copy from ref[0]
            // replaces n queued EdcJob::Skip pushes AND the flush-time
            // run-coalescing scan that used to rediscover this very run.
            self.recon_p_skip_band(x0, row, n)?;
        }
        Ok(())
    }

    /// Pixel half of P_Skip (see the E1 seam note on `recon_p_inter`).
    /// Run-coalesced P_Skip reconstruction: `n` consecutive skip MBs on one
    /// row, all with `mv == (0,0)` and NO weighted prediction. At full-pel
    /// zero motion, MC is the identity read of `refs[0]` — so the recon is a
    /// straight band copy from the padded reference plane, no staging, no MC
    /// calls. Byte-identical to `n` calls of `recon_p_skip` by construction
    /// (mc_luma_padded/mc_chroma_padded at mv 0 return exactly these bytes).
    ///
    /// The (0,0) run theorem that makes ONE derivation cover the run: once a
    /// skip MB commits (ref 0, mv (0,0)), every later MB of the run derives
    /// (0,0) too — its left neighbour (or a row-start's missing neighbour)
    /// triggers §8.4.1.1's zero-MV rule in `skip_mv`.
    pub(crate) fn recon_p_skip_band(
        &mut self,
        mbx0: usize,
        mby: usize,
        n: usize,
    ) -> Result<(), MbError> {
        let w = n * 16;
        let rf0 = self.refs.first().ok_or(MbError::Inconsistent("refs"))?;
        let lst = rf0.lstride();
        {
            let ly = rf0.plano_luma();
            for dy in 0..16 {
                let y = mby * 16 + dy;
                let src = (y + crate::LPAD) * lst + crate::LPAD + mbx0 * 16;
                let dst = y * self.cw + mbx0 * 16;
                // ARMOR (fuzz find, inline-execution.md 11.11): on a MALFORMED
                // stream a reference can carry geometry that does not match the
                // open picture (mutated parameter sets / truncated ref planes),
                // so the in-frame band's ref window can leave the guard slice.
                // The band REFUSES rather than panics; a conformant stream
                // cannot take this arm (repro: scratchpad repro.264, plane 6400
                // vs index 6416). Output on malformed input is unspecified —
                // absence of panic is the contract the fuzz gate enforces.
                let (Some(d), Some(sr)) = (self.rec_y.get_mut(dst..dst + w), ly.get(src..src + w))
                else {
                    return Ok(());
                };
                d.copy_from_slice(sr);
            }
        }
        let cst = rf0.cstride();
        let wc = n * 8;
        for c in 0..2 {
            let rc = rf0.plano_croma(c);
            let plane = if c == 0 {
                &mut self.rec_u
            } else {
                &mut self.rec_v
            };
            for dy in 0..8 {
                let y = mby * 8 + dy;
                let src = (y + crate::CPAD) * cst + crate::CPAD + mbx0 * 8;
                let dst = y * self.ccw + mbx0 * 8;
                // Same armor as the luma band above.
                let (Some(d), Some(sr)) = (plane.get_mut(dst..dst + wc), rc.get(src..src + wc))
                else {
                    return Ok(());
                };
                d.copy_from_slice(sr);
            }
        }
        Ok(())
    }

    /// Full-pel P_Skip single: rec window = ref\[0\] window at an integer offset.
    /// Returns false (touching nothing) when any window leaves the padded
    /// plane — mc's edge clamping takes over there. Luma needs mv%4==0;
    /// chroma needs mv%8==0 (its frac is mv&7), so odd full-pel MVs copy luma
    /// and interpolate chroma.
    pub(crate) fn recon_p_skip_fullpel(
        &mut self,
        mbx: usize,
        mby: usize,
        mv: (i32, i32),
    ) -> Result<bool, MbError> {
        let (dx, dy) = ((mv.0 / 4) as isize, (mv.1 / 4) as isize);
        let Some(rf0) = self.refs.first() else {
            return Ok(false);
        };
        let lst = rf0.lstride();
        let lpad = crate::LPAD as isize;
        let cx = mbx as isize * 16 + dx + lpad;
        let cy = mby as isize * 16 + dy + lpad;
        {
            let ly = rf0.plano_luma();
            let lrows = rf0.lrows() as isize;
            if cx < 0 || cx + 16 > lst as isize || cy < 0 || cy + 16 > lrows {
                return Ok(false);
            }
            // ONE span each side: sixteen rows a fixed stride apart, so the
            // source and destination checks are paid once instead of per row.
            let (cw, sb) = (self.cw, cy as usize * lst + cx as usize);
            let db = (mby * 16) * cw + mbx * 16;
            let sw = &ly[sb..sb + 15 * lst + 16];
            let dw = &mut self.rec_y[db..db + 15 * cw + 16];
            for r in 0..16 {
                dw[r * cw..r * cw + 16].copy_from_slice(&sw[r * lst..r * lst + 16]);
            }
        }
        let cst = rf0.cstride();
        let cpad = crate::CPAD as isize;
        if mv.0 % 8 == 0 && mv.1 % 8 == 0 {
            let ccx = mbx as isize * 8 + (mv.0 / 8) as isize + cpad;
            let ccy = mby as isize * 8 + (mv.1 / 8) as isize + cpad;
            let crows = rf0.crows() as isize;
            if ccx >= 0 && ccx + 8 <= cst as isize && ccy >= 0 && ccy + 8 <= crows {
                for c in 0..2 {
                    let rc = rf0.plano_croma(c);
                    let plane = if c == 0 {
                        &mut self.rec_u
                    } else {
                        &mut self.rec_v
                    };
                    let (ccw, sb) = (self.ccw, ccy as usize * cst + ccx as usize);
                    let db = (mby * 8) * ccw + mbx * 8;
                    let sw = &rc[sb..sb + 7 * cst + 8];
                    let dw = &mut plane[db..db + 7 * ccw + 8];
                    for r in 0..8 {
                        dw[r * ccw..r * ccw + 8].copy_from_slice(&sw[r * cst..r * cst + 8]);
                    }
                }
                return Ok(true);
            }
        }
        // Odd full-pel (or chroma window out of pad): luma is already copied,
        // chroma still interpolates — identical to the mc path's chroma half.
        let cch = self.mb_h * 8;
        for c in 0..2 {
            let mut pc = [0u8; 64];
            let rc = if c == 0 {
                rf0.plano_croma(0)
            } else {
                rf0.plano_croma(1)
            };
            mc_chroma_padded(
                rc,
                cst,
                crate::CPAD,
                self.ccw,
                cch,
                mbx * 8,
                mby * 8,
                8,
                8,
                mv.0,
                mv.1,
                &mut pc,
            )?;
            let plane = if c == 0 {
                &mut self.rec_u
            } else {
                &mut self.rec_v
            };
            for r in 0..8 {
                let d = (mby * 8 + r) * self.ccw + mbx * 8;
                plane[d..d + 8].copy_from_slice(&pc[r * 8..r * 8 + 8]);
            }
        }
        Ok(true)
    }

    pub(crate) fn recon_p_skip(
        &mut self,
        mb_x: usize,
        mb_y: usize,
        mv: (i32, i32),
    ) -> Result<(), MbError> {
        // Full-pel + identity/no weights: MC is a pure offset read, so copy the
        // ref window straight into rec — no pred staging, no weight pass. The
        // window must sit inside the PADDED plane (outside it, mc's clamping
        // differs from a raw offset read, so fall through).
        if (self.weights.is_none() || self.weights_id0)
            && mv.0 % 4 == 0
            && mv.1 % 4 == 0
            && self.recon_p_skip_fullpel(mb_x, mb_y, mv)?
        {
            return Ok(());
        }
        let (ch, cch) = (self.mb_h * 16, self.mb_h * 8);

        let mut pred = [0u8; 256];
        let rf0 = self.refs.first().ok_or(MbError::Inconsistent("refs"))?;
        mc_luma_padded(
            rf0.plano_luma(),
            rf0.lstride(),
            crate::LPAD,
            self.cw,
            ch,
            mb_x * 16,
            mb_y * 16,
            16,
            16,
            mv.0,
            mv.1,
            &mut pred,
        )?;
        if let Some(wt) = &self.weights {
            if !self.weights_id0 {
                let (w, o) = wt.ponderacion_luma(0, 0)?;
                for p in pred.iter_mut() {
                    *p = crate::ponderacion::ponderar_muestra(*p, w, o, wt.luma_log2_denom);
                }
            }
        }
        {
            // ONE span for the sixteen rows (see `recon_p_inter_nores`).
            let (cw, base) = (self.cw, (mb_y * 16) * self.cw + mb_x * 16);
            let win = &mut self.rec_y[base..base + 15 * cw + 16];
            for dy in 0..16 {
                win[dy * cw..dy * cw + 16].copy_from_slice(&pred[dy * 16..dy * 16 + 16]);
            }
        }
        let (mut pu, mut pv) = ([0u8; 64], [0u8; 64]);
        {
            let rf0 = self.refs.first().ok_or(MbError::Inconsistent("refs"))?;
            let (gu, gv) = (rf0.plano_croma(0), rf0.plano_croma(1));
            crusty_h264_common::inter::mc_chroma_padded_pair(
                gu,
                gv,
                rf0.cstride(),
                crate::CPAD,
                self.ccw,
                cch,
                mb_x * 8,
                mb_y * 8,
                8,
                8,
                mv.0,
                mv.1,
                &mut pu,
                &mut pv,
            )?;
        }
        if let Some(wt) = &self.weights {
            if !self.weights_id0 {
                let denom = wt.chroma_log2_denom;
                for (plano, cc) in [(&mut pu, 0), (&mut pv, 1)] {
                    let (w, o) = wt.ponderacion_croma(0, 0, cc)?;
                    for p in plano.iter_mut() {
                        *p = crate::ponderacion::ponderar_muestra(*p, w, o, denom);
                    }
                }
            }
        }
        for (pc, plane) in [(&pu, &mut self.rec_u), (&pv, &mut self.rec_v)] {
            for dy in 0..8 {
                let d = (mb_y * 8 + dy) * self.ccw + mb_x * 8;
                plane[d..d + 8].copy_from_slice(&pc[dy * 8..dy * 8 + 8]);
            }
        }
        Ok(())
    }
}
