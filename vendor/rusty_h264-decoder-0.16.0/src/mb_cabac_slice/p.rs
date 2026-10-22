//! Macrobloques de los cortes P en CABAC: salto (`P_Skip`) e inter (UIT-T H.264 §7.3.5, §9.3).

use alloc::boxed::Box;

use rusty_h264_common::cabac_tables::CACHE30;
use rusty_h264_common::inter::{predict_mv, predict_partition_mv};
use rusty_h264_common::predict::nnz_raster_from_z;

use super::estado::{CorteCabac, PosicionMb, RejillasCorte, ResultadoMb};
use super::{
    cache_nzc_vecinos, cond_term_flag, valor_vecino, CTX_MB_SKIP_P, CTX_TRANSFORM_8X8,
    MB_TYPE_P_PRIMER_INTRA,
};
use crate::cabac_residuo::{parse_mb_residual_cabac, ResidualOut};
use crate::edc::{EdcJob, PInterNoResJob};
use crate::error::MbError;
use crate::mb16::FrameDecoder;
use crate::mb_cabac::{
    mb_type_p_eng, parse_cbp_cabac, parse_mb_qp_delta_cabac, parse_mvd_partition,
    parse_ref_idx_cabac, parse_sub_mb_type_p_cabac,
};

impl FrameDecoder {
    /// Un macrobloque de un corte P: `mb_skip_flag` y, si no se salta, `mb_type`; los inter se
    /// decodifican aquí y los intra se devuelven como [`ResultadoMb::Intra`] (UIT-T H.264 §7.3.4,
    /// §7.3.5, §9.3).
    ///
    /// # Errors
    /// Los de la decodificación del macrobloque.
    pub(super) fn mb_cabac_p(
        &mut self,
        corte: &mut CorteCabac<'_>,
        pos: PosicionMb,
    ) -> Result<ResultadoMb, MbError> {
        let CorteCabac {
            cab,
            last_delta_qp,
            rejillas,
            ..
        } = corte;
        let RejillasCorte { mb_skip, .. } = rejillas;
        let PosicionMb {
            addr,
            mbx,
            mby,
            left,
            top,
        } = pos;

        // Brick 3.1/3.2: P-slice mb_skip_flag, then mb_type (P mb_type is neighbour-
        // independent; intra sub-types map to the I dispatch).
        // Direct bool arithmetic — no Option chain on the hot path.
        let sctx = CTX_MB_SKIP_P
            + cond_term_flag(mb_skip, left, |b| !b, "mb_skip")?
            + cond_term_flag(mb_skip, top, |b| !b, "mb_skip")?;
        // ONE engine view carries mb_skip_flag, the terminate bin of a skipped
        // macroblock, and mb_type of a coded one (was 2-3 view/commit round
        // trips of 8 memory ops each per macroblock).
        let (data, mut e, ctx) = cab.view();
        if e.decode_decision(data, ctx, sctx) != 0 {
            *mb_skip
                .get_mut(addr)
                .ok_or(MbError::Inconsistent("mb_skip"))? = true;
            *last_delta_qp = 0; // skip codes no mb_qp_delta → delta ctxInc resets
                                // P_Skip recon reuses the entropy-free CAVLC primitive verbatim: it
                                // takes no bit-reader (skip has no coded syntax past the flag), just
                                // predicts the skip MV, motion-compensates, and commits the grid.
            self.decode_p_skip(mbx, mby)?;
            *self
                .mb_qp
                .get_mut(addr)
                .ok_or(MbError::Inconsistent("mb_qp"))? = self.cur_qp; // skip inherits QPy
            let fin_de_corte = e.decode_terminate(data);
            cab.commit(e);
            return Ok(ResultadoMb::Terminado(fin_de_corte));
        }
        let mbt = mb_type_p_eng(&mut e, data, ctx);
        cab.commit(e);
        // NON-SKIP P MB: the inter arms below predict MVs from the
        // grids (mv_neighbors_block) and the intra arm gathers — flush
        // the deferred P span (B span cannot be pending in a P slice,
        // but span_flush is one Option check each).
        self.span_flush()?;
        if mbt <= 3 {
            let allow8 = self.p_inter_movimiento(corte, pos, mbt)?;
            return self
                .p_inter_residuo(corte, pos, allow8)
                .map(ResultadoMb::Terminado);
        }
        // 5→0 (I_4x4), 6..29→1..24 (I_16x16), 30→25 (I_PCM)
        Ok(ResultadoMb::Intra(mbt - MB_TYPE_P_PRIMER_INTRA))
    }

    /// Movimiento de un P inter (`mb_type` 0..=3: 16x16, 16x8, 8x16 y 8x8): `ref_idx_l0` de cada
    /// partición y luego sus `mvd_l0` (§7.3.5.1, §7.3.5.2), vector predicho y guardado en las
    /// rejillas. Devuelve si el macrobloque admite `transform_size_8x8_flag`
    /// (`noSubMbPartSizeLessThan8x8Flag`).
    ///
    /// # Errors
    /// Los de la decodificación del macrobloque.
    pub(super) fn p_inter_movimiento(
        &mut self,
        corte: &mut CorteCabac<'_>,
        pos: PosicionMb,
        mbt: u32,
    ) -> Result<bool, MbError> {
        let CorteCabac { cab, rejillas, .. } = corte;
        let RejillasCorte { mb_ref, mb_mvd, .. } = rejillas;
        let PosicionMb {
            addr,
            mbx,
            mby,
            left,
            top,
        } = pos;
        let mbw = self.mb_w;

        // noSubMbPartSizeLessThan8x8Flag (spec 7.3.5): P_8x8 permits the
        // 8x8 transform only when every sub-partition is itself 8x8.
        let mut allow8 = true;
        // Inter MB (Bricks 3.3/3.4/3.5). 1-ref stream → ref_idx not coded (ref=0).
        // Build the 30-entry mvd/ref neighbour cache (openh264 WelsFillCacheInterCabac).
        let mut mvdc = [[0i16; 2]; 30];
        let mut refc = [-1i8; 30];
        // Index the neighbour's RECORD once, then read within it.
        // `mb_ref[l][bi]` is two bounds checks - one on the Vec, one
        // on the array - repeated for each of the four entries and
        // again for `mb_mvd`, i.e. sixteen checks per macroblock for
        // four neighbours. Binding the record hoists the Vec check
        // out; the inner index is a literal and folds.
        if let Some(l) = left {
            let (Some(lr), Some(lm)) = (mb_ref.get(l), mb_mvd.get(l)) else {
                return Err(MbError::Truncated);
            };
            for (ci, bi) in [(6usize, 3usize), (12, 7), (18, 11), (24, 15)] {
                refc[ci] = lr[bi];
                mvdc[ci] = lm[bi];
            }
        }
        if let Some(t) = top {
            let (Some(tr), Some(tm)) = (mb_ref.get(t), mb_mvd.get(t)) else {
                return Err(MbError::Truncated);
            };
            for (ci, bi) in [(1usize, 12usize), (2, 13), (3, 14), (4, 15)] {
                refc[ci] = tr[bi];
                mvdc[ci] = tm[bi];
            }
        }
        if mbx > 0 && mby > 0 {
            let a = addr - mbw - 1;
            if let (Some(r), Some(m)) = (mb_ref.get(a), mb_mvd.get(a)) {
                (refc[0], mvdc[0]) = (r[15], m[15]);
            }
        }
        if mby > 0 && mbx + 1 < mbw {
            let a = addr - mbw + 1;
            if let (Some(r), Some(m)) = (mb_ref.get(a), mb_mvd.get(a)) {
                (refc[5], mvdc[5]) = (r[12], m[12]);
            }
        }
        let mut mmvd = [[0i16; 2]; 16];
        let mut mref = [0i8; 16];
        // mb_pred (spec 7.3.5.1): all ref_idx_l0 FIRST (only when >1 active
        // ref), then all mvd + ref-aware predict + commit. `refidx!` parses one
        // partition's ref_idx (ctxIdxOffset 54, ctx from neighbour refc) and
        // seeds refc so a later partition's ref/mvd context sees it — mirror
        // of the encoder's two-phase emit_mb_cabac_p_inter.
        macro_rules! refidx {
            ($pi:expr, $zb:expr) => {{
                if self.num_ref_active > 1 {
                    let s = CACHE30[$pi & 15].clamp(6, 29);
                    let c0 = (refc[s - 1] > 0) as usize + 2 * (refc[s - 6] > 0) as usize;
                    let r = parse_ref_idx_cabac(cab, c0);
                    for &zb in $zb.iter() {
                        refc[CACHE30[zb & 15]] = r;
                    }
                    r
                } else {
                    0i8
                }
            }};
        }
        macro_rules! part {
            ($pi:expr, $zb:expr, $pred:expr, $rx:expr, $ry:expr, $rw:expr, $rh:expr, $refi:expr) => {{
                let (mvx, mvy) = parse_mvd_partition(
                    cab, $pi, $zb, &mut mvdc, &mut refc, &mut mmvd, &mut mref, $refi,
                );
                let [na, nb, nc] = self.mv_neighbors_block(
                    (mbx * 4 + $rx / 4) as isize,
                    (mby * 4 + $ry / 4) as isize,
                    ($rw / 4) as isize,
                )?;
                let pmv = $pred(na, nb, nc);
                self.commit_inter_grid(
                    (mbx, mby),
                    ($rx, $ry, $rw, $rh),
                    (pmv.0 + mvx, pmv.1 + mvy),
                    $refi,
                );
            }};
        }
        match mbt {
            0 => {
                // Sibling of CAVLC P_16x16: one (ref, mv) for all 16
                // blocks → every internal edge is strength 0 (§8.7.2.1).
                // Without this, CABAC Main/High P_16x16 stayed UNSET and
                // paid the blind 24-block bS gather.
                *self
                    .mb_kind
                    .get_mut(mby * self.mb_w + mbx)
                    .ok_or(MbError::Inconsistent("mb_kind"))? =
                    rusty_h264_common::deblock::MB_KIND_INTER_UNIFORM;
                let r0 = refidx!(0, &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
                part!(
                    0,
                    &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
                    |a, b, c| predict_partition_mv(0, 0, a, b, c, r0 as i32),
                    0,
                    0,
                    16,
                    16,
                    r0
                );
            }
            1 => {
                let r0 = refidx!(0, &[0, 1, 2, 3, 4, 5, 6, 7]);
                let r1 = refidx!(8, &[8, 9, 10, 11, 12, 13, 14, 15]);
                part!(
                    0,
                    &[0, 1, 2, 3, 4, 5, 6, 7],
                    |a, b, c| predict_partition_mv(1, 0, a, b, c, r0 as i32),
                    0,
                    0,
                    16,
                    8,
                    r0
                );
                part!(
                    8,
                    &[8, 9, 10, 11, 12, 13, 14, 15],
                    |a, b, c| predict_partition_mv(1, 1, a, b, c, r1 as i32),
                    0,
                    8,
                    16,
                    8,
                    r1
                );
            }
            2 => {
                let r0 = refidx!(0, &[0, 1, 2, 3, 8, 9, 10, 11]);
                let r1 = refidx!(4, &[4, 5, 6, 7, 12, 13, 14, 15]);
                part!(
                    0,
                    &[0, 1, 2, 3, 8, 9, 10, 11],
                    |a, b, c| predict_partition_mv(2, 0, a, b, c, r0 as i32),
                    0,
                    0,
                    8,
                    16,
                    r0
                );
                part!(
                    4,
                    &[4, 5, 6, 7, 12, 13, 14, 15],
                    |a, b, c| predict_partition_mv(2, 1, a, b, c, r1 as i32),
                    8,
                    0,
                    8,
                    16,
                    r1
                );
            }
            _ => {
                // P_8x8: 4 sub_mb_types, then 4 ref_idx (one per 8×8), then mvd.
                let mut subt = [0u32; 4];
                for st in &mut subt {
                    *st = parse_sub_mb_type_p_cabac(cab);
                }
                allow8 = subt.iter().all(|&t| t == 0);
                let mut pr = [0i8; 4];
                for (i, r) in pr.iter_mut().enumerate() {
                    let b = i * 4;
                    *r = refidx!(b, &[b, b + 1, b + 2, b + 3]);
                }
                for i in 0..4usize {
                    let b = i * 4;
                    let (ox, oy) = ((i % 2) * 8, (i / 2) * 8); // 8×8 pixel origin in MB
                    let ri = pr[i];
                    match subt[i] {
                        0 => part!(
                            b,
                            &[b, b + 1, b + 2, b + 3],
                            |a, b, c| predict_mv(a, b, c, ri as i32),
                            ox,
                            oy,
                            8,
                            8,
                            ri
                        ),
                        1 => {
                            part!(
                                b,
                                &[b, b + 1],
                                |a, b, c| predict_mv(a, b, c, ri as i32),
                                ox,
                                oy,
                                8,
                                4,
                                ri
                            );
                            part!(
                                b + 2,
                                &[b + 2, b + 3],
                                |a, b, c| predict_mv(a, b, c, ri as i32),
                                ox,
                                oy + 4,
                                8,
                                4,
                                ri
                            );
                        }
                        2 => {
                            part!(
                                b,
                                &[b, b + 2],
                                |a, b, c| predict_mv(a, b, c, ri as i32),
                                ox,
                                oy,
                                4,
                                8,
                                ri
                            );
                            part!(
                                b + 1,
                                &[b + 1, b + 3],
                                |a, b, c| predict_mv(a, b, c, ri as i32),
                                ox + 4,
                                oy,
                                4,
                                8,
                                ri
                            );
                        }
                        _ => {
                            for j in 0..4usize {
                                let (sx, sy) = ((j % 2) * 4, (j / 2) * 4);
                                part!(
                                    b + j,
                                    &[b + j],
                                    |a, b, c| predict_mv(a, b, c, ri as i32),
                                    ox + sx,
                                    oy + sy,
                                    4,
                                    4,
                                    ri
                                );
                            }
                        }
                    }
                }
            }
        }
        *mb_ref
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_ref"))? = mref;
        *mb_mvd
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_mvd"))? = mmvd;
        Ok(allow8)
    }

    /// `coded_block_pattern`, `transform_size_8x8_flag`, `mb_qp_delta` y residuo de un P inter, y su
    /// reconstrucción (diferida en un trabajo EDC). `allow8` dice si el movimiento admite la
    /// transformada 8x8. Devuelve `end_of_slice_flag`.
    ///
    /// # Errors
    /// Los de la decodificación del macrobloque.
    pub(super) fn p_inter_residuo(
        &mut self,
        corte: &mut CorteCabac<'_>,
        pos: PosicionMb,
        allow8: bool,
    ) -> Result<bool, MbError> {
        let CorteCabac {
            cab,
            last_delta_qp,
            rejillas,
            ..
        } = corte;
        let RejillasCorte {
            mb_cbp,
            mb_nzc,
            cbf_dc,
            ..
        } = rejillas;
        let PosicionMb {
            addr,
            mbx,
            mby,
            left,
            top,
        } = pos;

        // Inter cbp + residual (is_intra = false → cbf default nA=nB=0).
        let cbp = parse_cbp_cabac(
            cab,
            valor_vecino(mb_cbp, top, "mb_cbp")?,
            valor_vecino(mb_cbp, left, "mb_cbp")?,
        );
        *mb_cbp
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_cbp"))? = cbp as u8;
        // H-49: an INTER macroblock carries transform_size_8x8_flag AFTER cbp
        // (spec 7.3.5), present only when CodedBlockPatternLuma > 0 and
        // noSubMbPartSizeLessThan8x8Flag. Same context as the intra read.
        let t8 = self.transform_8x8_mode && (cbp & 15) != 0 && allow8 && {
            let a = cond_term_flag(&self.mb_t8x8, left, |f| f, "mb_t8x8")?;
            let b = cond_term_flag(&self.mb_t8x8, top, |f| f, "mb_t8x8")?;
            cab.decode_decision(CTX_TRANSFORM_8X8 + a + b) != 0
        };
        *self
            .mb_t8x8
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_t8x8"))? = t8;
        self.any_t8 |= t8;
        // D9c: cbp==0 never parses residuals — skip the 2.5 KB coeff
        // zero-init + PInterJob entirely when NORES is on (default).
        // Current-MB nzc slots stay unset under cbp==0 and export as 0
        // (same as the 0xff→0 scrub below), so mb_nzc = [0;24] is exact.
        if cbp == 0 {
            *last_delta_qp = 0;
            *self
                .mb_qp
                .get_mut(addr)
                .ok_or(MbError::Inconsistent("mb_qp"))? = self.cur_qp;
            *cbf_dc
                .get_mut(addr)
                .ok_or(MbError::Inconsistent("cbf_dc"))? = 0;
            *mb_nzc
                .get_mut(addr)
                .ok_or(MbError::Inconsistent("mb_nzc"))? = [0u8; 24];
            if self.refs.is_empty() {
                return Err(MbError::Unsupported("inter without reference"));
            }
            let (jgmv, jgref) = self.movimiento_para_trabajo(mbx, mby);
            let pj = PInterNoResJob {
                mbx,
                mby,
                t8,
                gmv: jgmv,
                gref: jgref,
            };
            if self.edc_active {
                self.edc_jobs.push(EdcJob::InterNoRes(Box::new(pj)));
            } else {
                self.recon_p_inter_nores(&pj)?;
            }
            let fin_de_corte = cab.decode_terminate();
            return Ok(fin_de_corte);
        }
        // POOLED JOB, BUILT IN PLACE: the residual parse writes straight into
        // the boxed job (no 2.8 KB stack-then-heap copy, no malloc/free per
        // macroblock), and only the blocks the cbp says will be parsed are
        // zeroed -- the consumer reads a block only when its nnz is nonzero.
        let mut job = self.take_pinter_job();
        job.t8 = t8;
        let (cbp_luma, cbp_chroma) = (cbp & 15, cbp >> 4);
        let mut nzc = cache_nzc_vecinos(mb_nzc, top, left)?;
        let mut cbfdc = 0u16;
        job.nnzs = [0u8; 24]; // parsed totalCoeff per block (see add_inter_residual)
                              // A cbp==0 MB codes no mb_qp_delta → the next MB's delta ctxInc sees 0.
        if cbp == 0 {
            *last_delta_qp = 0;
        }
        if cbp != 0 {
            let ndc = (
                valor_vecino(cbf_dc, top, "cbf_dc")?,
                valor_vecino(cbf_dc, left, "cbf_dc")?,
            );
            let qpd = parse_mb_qp_delta_cabac(cab, last_delta_qp);
            self.step_qp(qpd)?;
            parse_mb_residual_cabac::<false>(
                cab,
                &mut nzc,
                &mut cbfdc,
                ndc,
                cbp_luma,
                cbp_chroma,
                t8,
                ResidualOut {
                    luma: &mut job.luma_scan,
                    luma8: &mut job.luma8,
                    cdc: &mut job.cdc,
                    cac: &mut job.cac,
                    nnzs: &mut job.nnzs,
                },
            );
        }
        *self
            .mb_qp
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_qp"))? = self.cur_qp;
        *cbf_dc
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("cbf_dc"))? = cbfdc;
        let mn = nnz_raster_from_z(&job.nnzs);
        *mb_nzc
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_nzc"))? = mn;

        if self.refs.is_empty() {
            return Err(MbError::Unsupported("inter without reference"));
        }
        // NOTE (measured, do not "optimise" this away): EDC is
        // ALWAYS ON, so
        // even single-threaded this job is built and deferred —
        // that batching is the E1 loop-fission win, not overhead.
        // A "skip the job in 1T" bypass added here could never fire
        // and was removed.
        let (jgmv, jgref) = self.movimiento_para_trabajo(mbx, mby);
        // D9c: when `cbp == 0`, never materialise the 2.5 KB coeff
        // arrays into a `PInterJob` — ship `InterNoRes` (or call
        // `recon_p_inter_nores` inline).
        let nores = cbp == 0;
        if nores {
            let pj = PInterNoResJob {
                mbx,
                mby,
                t8,
                gmv: jgmv,
                gref: jgref,
            };
            if self.edc_active {
                let b = self.take_nores_job(pj);
                self.edc_jobs.push(EdcJob::InterNoRes(b));
            } else {
                self.recon_p_inter_nores(&pj)?;
            }
            let fin_de_corte = cab.decode_terminate();
            return Ok(fin_de_corte);
        }
        job.mbx = mbx;
        job.mby = mby;
        job.qp = self.cur_qp;
        job.cbp_chroma = cbp_chroma;
        job.gmv = jgmv;
        job.gref = jgref;
        if self.edc_active {
            self.edc_jobs.push(EdcJob::Inter(job));
        } else {
            self.recon_p_inter(&job)?;
            self.edc_job_pool.push(job);
        }

        Ok(cab.decode_terminate())
    }
}
