//! Macrobloques P en CAVLC: inter, P_8x8 y P_Skip (§7.3.5, §8.4.1.1).

use super::{read_mvd, MAX_SUB_MB_TYPE_P};
use crate::edc::{EdcJob, PInterNoResJob};
use crate::error::MbError;
use crate::mb16::FrameDecoder;
use crate::mb_cabac::read_ref_idx;
use crate::mb_inter::ResiduoInter;
use crate::particion::{restride, sub_mb_partitions};
use alloc::boxed::Box;
use crust_h264_common::cavlc::{decode_residual_block_into, read_cbp_inter};
use crust_h264_common::inter::{
    inter_partitions, mc_chroma_padded, mc_luma_padded, predict_mv, predict_partition_mv,
};
use crust_h264_common::predict::{CHROMA_4X4_SCAN_XY, LUMA_4X4_SCAN_XY};
use crust_h264_common::BitReader;

/// Los tres planos de coeficientes en que se analiza el residuo de un P inter: los del trabajo EDC
/// diferido o los de reserva del decodificador. Viajan juntos entre el análisis y la
/// reconstrucción.
struct PlanosResiduo<'a> {
    /// Luma 4x4 en orden de barrido (sin `transform_size_8x8_flag`).
    luma_scan: &'a mut [[i32; 16]; 16],
    /// Luma 8x8 en orden de barrido (con `transform_size_8x8_flag`).
    luma8: &'a mut [[i32; 64]; 4],
    /// AC de croma de las dos componentes.
    cac: &'a mut [[[i32; 16]; 4]; 2],
}

/// Subpartición de un `P_8x8` ya leída: origen `(px, py)` y tamaño `(w, h)` en el macrobloque,
/// referencia y vector.
type RegionP8x8 = (usize, usize, usize, usize, i32, (i32, i32));

impl FrameDecoder {
    /// Reconstructs an inter macroblock (`mode` 0 = P_L0_16x16, 1 = P_16x8,
    /// 2 = P_8x16): parse the per-partition motion vectors and residual,
    /// motion-compensate each partition, and add the residual.
    pub(crate) fn decode_inter(
        &mut self,
        r: &mut BitReader,
        mb_x: usize,
        mb_y: usize,
        mode: u8,
    ) -> Result<(), MbError> {
        if self.refs.is_empty() {
            return Err(MbError::Unsupported("inter without reference"));
        }
        // DEBLOCK CLASS: mode 0 is P_L0_16x16 — ONE partition, so all 16 blocks
        // share a reference and motion vector and no internal edge can reach
        // strength 1. Internal strengths then follow from coefficients alone, i.e.
        // 16 nnz bytes instead of a 24-block gather across 5-7 grids. Modes 1/2
        // (P_16x8 / P_8x16) have two partitions with independent motion and stay
        // UNSET (blind path).
        if mode == 0 {
            *self
                .mb_kind
                .get_mut(mb_y * self.mb_w + mb_x)
                .ok_or(MbError::Inconsistent("mb_kind"))? =
                crust_h264_common::deblock::MB_KIND_INTER_UNIFORM;
        }
        // QP (qp/qpc) is bound after mb_qp_delta is read below.
        let w4 = self.mb_w * 4;
        let layout = inter_partitions(mode);

        // mb_pred order (spec 7.3.5.1): all ref_idx_l0 first (only when more than
        // one reference is active), then all mvd_l0.
        let nparts = layout.len();
        let ref_idxs = self.leer_refs_l0(r, nparts)?;

        // Phase 1: per partition, ref-aware MV prediction + mvd, committing the
        // motion grid so a later partition predicts from an earlier one.
        let mut regiones: [RegionP8x8; 4] = [(0, 0, 0, 0, 0, (0, 0)); 4];
        {
            for (part, &(rx, ry, rw, rh)) in layout.iter().enumerate() {
                let refi = ref_idxs[part];
                let (pbx, pby) = ((mb_x * 4 + rx / 4) as isize, (mb_y * 4 + ry / 4) as isize);
                let [a, b, c] = self.mv_neighbors_block(pbx, pby, (rw / 4) as isize)?;
                let pmv = predict_partition_mv(mode, part, a, b, c, refi);
                let mvd_x = read_mvd(r)?;
                let mvd_y = read_mvd(r)?;
                let mv = (pmv.0 + mvd_x, pmv.1 + mvd_y);
                regiones[part] = (rx, ry, rw, rh, refi, mv);
                for by in ry / 4..ry / 4 + rh / 4 {
                    for bx in rx / 4..rx / 4 + rw / 4 {
                        let idx = (mb_y * 4 + by) * w4 + (mb_x * 4 + bx);
                        if let (Some(m), Some(it), Some(rf), Some(cd)) = (
                            self.mv_y.get_mut(idx),
                            self.inter_y.get_mut(idx),
                            self.ref_idx_y.get_mut(idx),
                            self.coded_y.get_mut(idx),
                        ) {
                            (*m, *it, *rf, *cd) = (mv, true, refi, true);
                        }
                    }
                }
            }
        }

        // Phase 2: motion-compensate each partition from its reference.
        let mut pred_y = [0u8; 256];
        let mut c_pred = [[0u8; 64]; 2];
        // D14 (CAVLC E-seam): when the seam is live the WORKER motion-compensates
        // from the committed MV grids, so skip MC here rather than computing a
        // prediction that would be discarded.
        let defer = self.edc_active;
        if !defer {
            self.prediccion_particiones_p(
                (mb_x, mb_y),
                &regiones[..nparts],
                &mut pred_y,
                &mut c_pred,
            )?;
        }

        // 16×16/16×8/8×16 partitions are all ≥ 8×8, so the 8×8 transform is allowed.
        self.inter_finish(r, (mb_x, mb_y), &pred_y, &c_pred, true, defer)
    }

    /// Shared inter tail: parse `coded_block_pattern` + `mb_qp_delta`, decode the
    /// luma/chroma residual, and add it to the already-built motion-compensated
    /// prediction. Used by both the 16×16/16×8/8×16 path and `P_8x8`.
    pub(crate) fn inter_finish(
        &mut self,
        r: &mut BitReader,
        (mb_x, mb_y): (usize, usize),
        pred_y: &[u8; 256],
        c_pred: &[[u8; 64]; 2],
        allow_8x8: bool,
        // D14: emit a worker job instead of reconstructing. Only the CAVLC
        // 16x16/16x8/8x16 path sets this; B and P_8x8 stay inline.
        defer: bool,
    ) -> Result<(), MbError> {
        let w4 = self.mb_w * 4;
        let cbp = { read_cbp_inter(r)? };
        let cbp_luma = cbp & 15;
        let cbp_chroma = cbp >> 4;
        // transform_size_8x8_flag follows cbp (before mb_qp_delta) when luma has
        // coefficients, the 8×8 transform is enabled, and every partition ≥ 8×8.
        let t8x8 = cbp_luma > 0 && self.transform_8x8_mode && allow_8x8 && r.read_bit()?;
        if t8x8 {
            *self
                .mb_t8x8
                .get_mut(mb_y * self.mb_w + mb_x)
                .ok_or(MbError::Inconsistent("mb_t8x8"))? = true;
            self.any_t8 = true;
        }
        if cbp != 0 {
            self.step_qp(r.read_se()?)?;
        }
        let (qp, _qpc) = (self.cur_qp, self.chroma_qp_for(self.cur_qp));

        // ---- luma residual ----
        self.nnz_cache_load(mb_x, mb_y)?;
        let mut nnzs = [0u8; 24];
        // PARSE STRAIGHT INTO THE DESTINATION. The deferred arm used to parse into
        // 1.5 KB of zeroed locals and then copy them into the pooled job (another
        // 1.5 KB, +1 KB each for t8x8); the inline arm zeroed the same locals.
        // Both now decode into a pre-owned plane set -- the pooled `PInterJob`
        // when deferring, the decoder's scratch boxes otherwise -- and zero only
        // the blocks the cbp codes. Uncoded slots stay dirty on purpose: the
        // consumer (`add_inter_residual` / the worker) skips every block whose
        // `nnzs` count is 0 and every chroma AC plane unless cbp_chroma == 2,
        // the contract the CABAC P arm has shipped on since round 4.
        let nores = cbp == 0;
        let mut job = (defer && !nores).then(|| self.take_pinter_job());
        let mut s_luma = if job.is_none() {
            self.scratch_luma
                .take()
                .or_else(|| Some(Box::new([[0i32; 16]; 16])))
        } else {
            None
        };
        let mut s_luma8 = if job.is_none() {
            self.scratch_luma8
                .take()
                .or_else(|| Some(Box::new([[0i32; 64]; 4])))
        } else {
            None
        };
        let mut s_cac = if job.is_none() {
            self.scratch_cac
                .take()
                .or_else(|| Some(Box::new([[[0i32; 16]; 4]; 2])))
        } else {
            None
        };
        let (luma_scan, luma8, cac) = if let Some(j) = job.as_deref_mut() {
            (&mut j.luma_scan, &mut j.luma8, &mut j.cac)
        } else {
            let sl = s_luma.as_deref_mut();
            let sl8 = s_luma8.as_deref_mut();
            let sc = s_cac.as_deref_mut();
            if let (Some(l), Some(l8), Some(c)) = (sl, sl8, sc) {
                (l, l8, c)
            } else {
                return Err(MbError::Truncated);
            }
        };
        let mut planos = PlanosResiduo {
            luma_scan,
            luma8,
            cac,
        };
        self.residuo_luma_cavlc(r, (mb_x, mb_y), cbp_luma, t8x8, &mut planos, &mut nnzs)?;
        let PlanosResiduo {
            luma_scan,
            luma8,
            cac,
        } = planos;

        // ---- chroma residual ----
        let c_recon_dc = self.residuo_croma_cavlc(r, (mb_x, mb_y), cbp_chroma, cac, &mut nnzs)?;

        // ---- reconstruction ----
        //
        // D13: this used to be a 109-line hand-rolled copy of the residual add.
        // It now calls the SAME `add_inter_residual` the CABAC path uses, which
        // is what makes the CAVLC E-seam possible at all: the two paths had
        // different residual representations (CAVLC pre-applied un_scan and
        // inv_quant at PARSE time; CABAC carries raw scan-order coefficients and
        // dequantises inside the helper), so no job could be shared. Carrying the
        // raw forms — which CAVLC already had in hand — converges them, deletes
        // a duplicate implementation, and lets a deferred job reuse the existing
        // worker recon instead of needing a second copy that could drift.
        if defer {
            // The residual representation now matches CABAC exactly (the
            // convergence commit), so the SAME `PInterJob` and the SAME worker
            // `recon_p_inter` serve both entropy coders — no second recon
            // implementation exists that could drift.
            let (gmv, gref) = self.movimiento_para_trabajo(mb_x, mb_y);
            // D9 applies here too: `cbp == 0` means all 2,592 coefficient bytes
            // of the 2,784-byte job are ZERO, so ship the 176-byte motion-only
            // form. Discovered on the CABAC path; it transfers for free because
            // the CAVLC arm now emits the SAME job type.
            let ej = if nores {
                let b = self.take_nores_job(PInterNoResJob {
                    mbx: mb_x,
                    mby: mb_y,
                    t8: t8x8,
                    gmv,
                    gref,
                });
                EdcJob::InterNoRes(b)
            } else {
                // Pooled box (see `take_pinter_job`); the CAVLC arm still parses into
                // locals, so this is a copy, not an allocation.
                let mut job = job.take().unwrap_or_else(|| self.take_pinter_job());
                job.mbx = mb_x;
                job.mby = mb_y;
                job.qp = qp;
                job.cbp_chroma = cbp_chroma;
                job.t8 = t8x8;
                job.gmv = gmv;
                job.gref = gref;
                // luma_scan / luma8 / cac were parsed in place.
                job.cdc = c_recon_dc;
                job.nnzs = nnzs;
                EdcJob::Inter(job)
            };
            self.edc_jobs.push(ej);
        } else {
            self.add_inter_residual(
                mb_x,
                mb_y,
                pred_y,
                c_pred,
                ResiduoInter {
                    luma_scan: Some(&*luma_scan),
                    luma8: t8x8.then_some(&*luma8),
                    cdc: &c_recon_dc,
                    cac: Some(&*cac),
                    cbp_chroma,
                    nnzs: &nnzs,
                },
            );
            // Give the scratch planes back (taken only on this arm).
            self.scratch_luma = s_luma;
            self.scratch_luma8 = s_luma8;
            self.scratch_cac = s_cac;
        }

        // MV grid + coded flags were set per partition; mark modes as DC.
        // Four contiguous cells per row - `fill`, not sixteen indexed stores.
        for lby in 0..4usize {
            let a = (mb_y * 4 + lby) * w4 + mb_x * 4;
            self.modes_y[a..a + 4].fill(2);
        }
        Ok(())
    }

    /// Residuo de luma de un P inter en CAVLC (§7.3.5.3), escrito directamente en `planos` (orden de
    /// barrido sin deshacer; `add_inter_residual` o el trabajo diferido lo descuantifican) y con el
    /// número de coeficientes de cada bloque en `nnzs` y en la rejilla `nnz_y`.
    /// # Errors
    /// `MbError` si el flujo se acaba o un bloque está mal codificado.
    fn residuo_luma_cavlc(
        &mut self,
        r: &mut BitReader,
        (mb_x, mb_y): (usize, usize),
        cbp_luma: u32,
        t8x8: bool,
        planos: &mut PlanosResiduo<'_>,
        nnzs: &mut [u8; 24],
    ) -> Result<(), MbError> {
        let w4 = self.mb_w * 4;
        // BUILD THE MACROBLOCK'S nnz RASTER ON THE STACK, COPY IT ROW-WISE ONCE.
        // Both arms below scattered sixteen individually bounds-checked stores
        // into the frame grid while the entropy parse ran. Nothing reads
        // `nnz_y` for THIS macroblock before the function returns - `nc_pred`
        // predicts from the separate `nnz_cache` - so the writes can be
        // deferred. The zero arm then costs NOTHING: the raster already holds
        // zeros.
        let mut nnz_raster = [0u8; 16];
        if t8x8 {
            for b8 in 0..4 {
                let (b8x, b8y) = (b8 % 2, b8 / 2);
                let (bx, by) = (mb_x * 4 + b8x * 2, mb_y * 4 + b8y * 2);
                let _ = (bx, by);
                if cbp_luma & (1 << b8) != 0 {
                    // Zero this 8x8 slot only (256 B) -- was a 1 KB `[[0;64];4]` insert + a 256 B copy.
                    planos.luma8[b8 & 3] = [0i32; 64];
                    for sub in 0..4 {
                        let (sx, sy) = (sub % 2, sub / 2);
                        let (cx, cy) = (b8x * 2 + sx, b8y * 2 + sy);
                        let nc = self.nc_pred(cx, cy);
                        let mut blk = [0i32; 16];
                        let total = decode_residual_block_into::<16, 16>(r, nc, &mut blk)?;
                        self.nnz_cache_set(cx, cy, total);
                        nnz_raster[cy * 4 + cx] = total;
                        // The PER-SUB-BLOCK count the next macroblock's nC prediction
                        // depends on -- summing these into one slot and letting the
                        // recon helper broadcast it back is what broke CAVLC 8x8.
                        // WIN: b8 < 4 and sub < 4, so the index is 0..=15 and the
                        // mask is a no-op that PROVES it -- the same idiom the
                        // neighbouring luma8[b8 & 3] / luma_scan[blk & 15] already use.
                        // Folds a panic_bounds_check out of inter_finish.
                        nnzs[(b8 * 4 + sub) & 15] = total;
                        // RAW 8x8 scan: coeff k of sub-block s at 4k + s (spec 7.3.5.3.2);
                        // `add_inter_residual` un-scans + dequantises, as for CABAC.
                        for (k, &v) in blk.iter().enumerate() {
                            planos.luma8[b8 & 3][(4 * k + sub) & 63] = v;
                        }
                    }
                } else {
                    for sub in 0..4 {
                        let (sx, sy) = (sub % 2, sub / 2);
                        self.nnz_cache_set(b8x * 2 + sx, b8y * 2 + sy, 0);
                        // `nnz_raster` is already zero here - no store needed.
                    }
                }
            }
        } else {
            for (blk, &(lbx, lby)) in LUMA_4X4_SCAN_XY.iter().enumerate() {
                let (bx, by) = (mb_x * 4 + lbx, mb_y * 4 + lby);
                let total = if cbp_luma & (1 << (blk / 4)) != 0 {
                    let nc = self.nc_pred(lbx, lby);
                    // Decoded STRAIGHT into the destination plane (RAW scan order, like
                    // CABAC); zero only this coded block first (the slot is pooled/dirty).
                    planos.luma_scan[blk & 15] = [0i32; 16];
                    decode_residual_block_into::<16, 16>(r, nc, &mut planos.luma_scan[blk & 15])?
                } else {
                    0
                };
                self.nnz_cache_set(lbx, lby, total);
                let _ = (bx, by);
                nnz_raster[(lby & 3) * 4 + (lbx & 3)] = total;
                // WIN: luma 4x4 index, 0..=15 (luma_scan[blk & 15] above relies on the
                // same range); the mask folds the second bounds check.
                nnzs[blk & 15] = total;
            }
        }
        // ONE contiguous copy per macroblock row.
        for by in 0..4usize {
            let a = (mb_y * 4 + by) * w4 + mb_x * 4;
            self.nnz_y[a..a + 4].copy_from_slice(&nnz_raster[by * 4..by * 4 + 4]);
        }
        Ok(())
    }

    /// Residuo de croma de un P inter en CAVLC: DC de las dos componentes y, con `cbp_chroma` 2, AC en
    /// `cac` (§7.3.5.3). Devuelve los DC sin descuantificar.
    /// # Errors
    /// `MbError` si el flujo se acaba o un bloque está mal codificado.
    fn residuo_croma_cavlc(
        &mut self,
        r: &mut BitReader,
        (mb_x, mb_y): (usize, usize),
        cbp_chroma: u32,
        cac: &mut [[[i32; 16]; 4]; 2],
        nnzs: &mut [u8; 24],
    ) -> Result<[[i32; 4]; 2], MbError> {
        let mut c_recon_dc = [[0i32; 4]; 2];
        if cbp_chroma != 0 {
            for slot in c_recon_dc.iter_mut() {
                // RAW, straight into the 4-word slot (dequantised in the helper).
                decode_residual_block_into::<4, 4>(r, -1, slot)?;
            }
        }
        if cbp_chroma == 2 {
            self.chroma_cache_load(mb_x, mb_y)?;
            let w2 = self.mb_w * 2;
            let mut cnnz = [[0u8; 4]; 2];
            for c in 0..2 {
                for &(bx, by) in &CHROMA_4X4_SCAN_XY {
                    let nc = self.chroma_nc_pred(c, bx, by);
                    let slot = &mut cac[c & 1][(by * 2 + bx) & 3];
                    *slot = [0i32; 16];
                    let total = decode_residual_block_into::<15, 16>(r, nc, slot)?; // RAW scan order
                    self.chroma_nnz_cache_set(c, bx, by, total);
                    // WIN: by < 2 and bx < 2, so 0..=3 -- the mask proves it against
                    // the [[u8; 4]; 2], exactly as the cac[c & 1][(by * 2 + bx) & 3]
                    // slot two lines above already does. Folds the last
                    // panic_bounds_check out of inter_finish.
                    cnnz[c & 1][(by * 2 + bx) & 3] = total;
                    // Masks are the proof: max index 16 + 4 + 2 + 1 = 23 < 24 (was a `.min(23)`).
                    nnzs[16 + (c & 1) * 4 + (by & 1) * 2 + (bx & 1)] = total;
                }
                for by in 0..2usize {
                    let a = (mb_y * 2 + by) * w2 + mb_x * 2;
                    self.nnz_c[c][a..a + 2].copy_from_slice(&cnnz[c][by * 2..by * 2 + 2]);
                }
            }
        }
        Ok(c_recon_dc)
    }

    /// Reconstructs a `P_8x8` macroblock: four 8×8 sub-macroblock partitions,
    /// each independently split (8×8 / 8×4 / 4×8 / 4×4) with its own motion
    /// vector(s). `ref0` is `P_8x8ref0` (every `ref_idx` forced to 0, not coded).
    pub(crate) fn decode_p8x8(
        &mut self,
        r: &mut BitReader,
        mb_x: usize,
        mb_y: usize,
        ref0: bool,
    ) -> Result<(), MbError> {
        if self.refs.is_empty() {
            return Err(MbError::Unsupported("inter without reference"));
        }
        let w4 = self.mb_w * 4;

        // mb_pred order (spec §7.3.5.2): all sub_mb_type, then all ref_idx_l0,
        // then all mvd_l0 (partition-major, sub-partition order within each).
        let (sub_types, ref_idxs) = self.leer_tipos_p8x8(r, ref0)?;

        // Per sub-partition (in decoding order): median MV prediction from the
        // committed neighbor grid, mvd, commit, then motion-compensate. Committing
        // before the next prediction is what lets sub-partitions chain correctly.
        // D14b: P_8x8 defers too. Syncing before it instead cost 9,772 pipeline
        // drains per stream (45 per 1000 MBs, vs CABAC's 3.3) because P_8x8 is
        // common in CAVLC P slices — and the seam measured 1.65-1.97x SLOWER
        // for it. Deferring is byte-identical for sub-partitions: the worker
        // motion-compensates per 4x4 from the committed grids, which is exactly
        // how the CABAC path already handles P_8x8, and a 6-tap filter applied
        // per-4x4 with the same MV gives the same pixels as one 8x8 call.
        let defer = self.edc_active;

        // ── PHASE 1: PARSE + COMMIT. Must always run to completion. ──────────
        //
        // These two are interleaved BY NECESSITY: each sub-partition's MV
        // prediction reads the grids the previous one committed, so they cannot
        // be separated from each other. But they consume BITSTREAM, so nothing
        // here may ever be skipped conditionally.
        //
        // This phase split is a STRUCTURAL guard, not a tidy-up. When MC lived
        // inside this loop, deferring it was written as a `break` — which also
        // skipped the `read_se` mvd reads, desynced the bitstream, and
        // mis-parsed as a B-slice sub_mb_type on a Baseline stream. It happened
        // to crash; a desync that stayed IN RANGE would have produced plausible
        // garbage instead. With MC in its own pass below, skipping pixel work
        // cannot reach the bitstream at all — the mistake is unavailable.
        let mut regions: [RegionP8x8; 16] = [(0, 0, 0, 0, 0, (0, 0)); 16];
        let mut nreg = 0usize;
        for part in 0..4usize {
            let refi = ref_idxs[part];
            let (b8x, b8y) = ((part % 2) * 8, (part / 2) * 8);
            for &(srx, sry, srw, srh) in sub_mb_partitions(sub_types[part]) {
                let (px, py) = (b8x + srx, b8y + sry);
                let (pbx, pby) = ((mb_x * 4 + px / 4) as isize, (mb_y * 4 + py / 4) as isize);
                let [a, b, c] = self.mv_neighbors_block(pbx, pby, (srw / 4) as isize)?;
                let pmv = predict_mv(a, b, c, refi);
                let mvd_x = read_mvd(r)?;
                let mvd_y = read_mvd(r)?;
                let mv = (pmv.0 + mvd_x, pmv.1 + mvd_y);
                for by in py / 4..py / 4 + srh / 4 {
                    for bx in px / 4..px / 4 + srw / 4 {
                        let idx = (mb_y * 4 + by) * w4 + (mb_x * 4 + bx);
                        if let (Some(m), Some(it), Some(rf), Some(cd)) = (
                            self.mv_y.get_mut(idx),
                            self.inter_y.get_mut(idx),
                            self.ref_idx_y.get_mut(idx),
                            self.coded_y.get_mut(idx),
                        ) {
                            (*m, *it, *rf, *cd) = (mv, true, refi, true);
                        }
                    }
                }
                regions[nreg & 15] = (px, py, srw, srh, refi, mv);
                nreg += 1;
            }
        }

        // ── PHASE 2: PIXEL WORK ONLY. Reads no bitstream; safe to skip. ──────
        let mut pred_y = [0u8; 256];
        let mut c_pred = [[0u8; 64]; 2];
        if !defer {
            self.prediccion_particiones_p(
                (mb_x, mb_y),
                &regions[..nreg],
                &mut pred_y,
                &mut c_pred,
            )?;
        }

        // P_8x8 allows the 8×8 transform only when every sub-partition is 8×8.
        let allow_8x8 = sub_types.iter().all(|&t| t == 0);
        self.inter_finish(r, (mb_x, mb_y), &pred_y, &c_pred, allow_8x8, defer)
    }

    /// `ref_idx_l0` de las `n` primeras particiones de un P inter (§7.3.5.1, §7.3.5.2), solo si
    /// hay más de una referencia activa; si no, todas valen 0.
    ///
    /// # Errors
    /// `MbError::Truncated` con una referencia que no existe, y los de lectura del flujo.
    fn leer_refs_l0(&self, r: &mut BitReader, n: usize) -> Result<[i32; 4], MbError> {
        let num_refs = self.refs.len();
        let mut ref_idxs = [0i32; 4];
        if self.num_ref_active > 1 {
            for ri in ref_idxs[..n].iter_mut() {
                *ri = read_ref_idx(r, self.num_ref_active)?;
                if *ri as usize >= num_refs {
                    return Err(MbError::Truncated); // references a non-existent picture
                }
            }
        }
        Ok(ref_idxs)
    }

    /// `sub_mb_type` de las cuatro particiones de un `P_8x8` y, si hay más de una referencia
    /// activa y no es `P_8x8ref0`, su `ref_idx_l0` (§7.3.5.2).
    ///
    /// # Errors
    /// `MbError::Unsupported` con un `sub_mb_type` mayor que 3, `MbError::Truncated` con una
    /// referencia que no existe, y los de lectura del flujo.
    fn leer_tipos_p8x8(
        &self,
        r: &mut BitReader,
        ref0: bool,
    ) -> Result<([u32; 4], [i32; 4]), MbError> {
        let mut sub_types = [0u32; 4];
        for st in sub_types.iter_mut() {
            let v = r.read_ue()?;
            if v > MAX_SUB_MB_TYPE_P {
                return Err(MbError::Unsupported("B-slice / invalid sub_mb_type"));
            }
            *st = v;
        }
        let ref_idxs = self.leer_refs_l0(r, if ref0 { 0 } else { 4 })?;
        Ok((sub_types, ref_idxs))
    }

    /// Fase de píxeles de un P inter en CAVLC (no lee el flujo), común a 16x16/16x8/8x16 y a
    /// `P_8x8`: compensa cada partición con su vector y referencia, la recoloca en `pred_y` y
    /// `c_pred` y aplica la predicción ponderada explícita.
    ///
    /// # Errors
    /// Los del recorte de los planos y de la ponderación.
    fn prediccion_particiones_p(
        &self,
        (mb_x, mb_y): (usize, usize),
        regiones: &[RegionP8x8],
        pred_y: &mut [u8; 256],
        c_pred: &mut [[u8; 64]; 2],
    ) -> Result<(), MbError> {
        let (ch, cch) = (self.mb_h * 16, self.mb_h * 8);
        for &(px, py, srw, srh, refi, mv) in regiones {
            let Some(reference) = self.refs.get(refi as usize) else {
                continue;
            };
            let mut tmp = [0u8; 256];
            mc_luma_padded(
                reference.plano_luma(),
                reference.lstride(),
                crate::LPAD,
                self.cw,
                ch,
                mb_x * 16 + px,
                mb_y * 16 + py,
                srw,
                srh,
                mv.0,
                mv.1,
                &mut tmp,
            )?;
            restride(pred_y, 16, px, py, &tmp, srw, srh);
            let (crx, cry, crw, crh) = (px / 2, py / 2, srw / 2, srh / 2);
            for (cc, cpl) in c_pred.iter_mut().enumerate() {
                let rc = if cc == 0 {
                    reference.plano_croma(0)
                } else {
                    reference.plano_croma(1)
                };
                let mut tc = [0u8; 64];
                mc_chroma_padded(
                    rc,
                    reference.cstride(),
                    crate::CPAD,
                    self.ccw,
                    cch,
                    mb_x * 8 + crx,
                    mb_y * 8 + cry,
                    crw,
                    crh,
                    mv.0,
                    mv.1,
                    &mut tc,
                )?;
                restride(cpl, 8, crx, cry, &tc, crw, crh);
            }
            self.weight_partition(pred_y, c_pred, 0, refi as usize, (px, py, srw, srh))?;
        }
        Ok(())
    }

    pub(crate) fn decode_p_skip(&mut self, mb_x: usize, mb_y: usize) -> Result<(), MbError> {
        // DEBLOCK CLASS: a P_Skip macroblock carries no coefficients and one
        // (ref, mv) for all 16 blocks, so every internal boundary strength is 0 by
        // §8.7.2.1 and the loop filter needs 9 block loads instead of 24. This is
        // the single highest-value classification: the MB-kind census measures Skip
        // at 36.4% (CAVLC) / 65.0% (main) / 57.8% (high) of real x264 corpora.
        // Written HERE because both the CAVLC and the CABAC slice loops funnel
        // through this one function.
        //
        // Deliberately NOT done for `B_Skip` — its motion is direct-derived and can
        // differ per 4×4 sub-block, so its internal edges can legally reach
        // strength 1. B_Skip stays UNSET and takes the blind path.
        // (kind commit moved into the span/immediate arms below.)
        // P_Skip always references index 0 (the most recent picture). Borrow it —
        // a full-frame `.cloned()` here was ~86% of total decode time (one ~3 MB
        // plane copy per skip MB, thousands per frame).
        if self.refs.is_empty() {
            return Err(MbError::Unsupported("P_Skip without reference"));
        }
        let addr = mb_y * self.mb_w + mb_x;
        // mb_x == 0: the left neighbor is off-frame, so §8.4.1.1's
        // unavailability rule forces (0,0) with no gather at all.
        let mv = if mb_x == 0 || self.skip_zero_next == addr {
            (0, 0)
        } else {
            // The gather reads the grids; the pending spans cannot contain
            // this MB's left (a non-forced skip means the previous MB was not
            // a (0,0) committer) — flush both before deriving.
            self.span_flush()?;
            self.skip_mv(mb_x, mb_y)?
        };
        self.skip_zero_next = if mv == (0, 0) { addr + 1 } else { usize::MAX };
        // Grid commits are PARSE state — (0,0) skips defer them into the P
        // span (the deferral constants); nonzero-MV skips commit immediately.
        if mv == (0, 0) {
            // Recon defers too when the band identity holds (1T, no/identity
            // weights) — the span flush band-copies instead of queueing jobs.
            let recon = self.weights.is_none() || self.weights_id0;
            self.pz_push(mb_x, mb_y, recon)?;
            if recon {
                return Ok(());
            }
        } else {
            *self
                .mb_kind
                .get_mut(mb_y * self.mb_w + mb_x)
                .ok_or(MbError::Inconsistent("mb_kind"))? =
                crust_h264_common::deblock::MB_KIND_SKIP;
            self.set_mb_mv(mb_x, mb_y, mv, true, 0);
            let w4 = self.mb_w * 4;
            for dy in 0..4 {
                let a = (mb_y * 4 + dy) * w4 + mb_x * 4;
                self.coded_y[a..a + 4].fill(true);
                self.modes_y[a..a + 4].fill(2);
            }
        }
        if self.edc_active {
            self.edc_jobs.push(EdcJob::Skip {
                mbx: mb_x,
                mby: mb_y,
                mv,
            });
            return Ok(());
        }
        self.recon_p_skip(mb_x, mb_y, mv)?;
        Ok(())
    }
}
