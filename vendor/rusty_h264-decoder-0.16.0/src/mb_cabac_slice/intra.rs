//! Macrobloques intra en CABAC, comunes a los tres tipos de corte: `mb_type` del corte I, `I_PCM`,
//! `I_16x16`, `I_4x4` e `I_8x8` (UIT-T H.264 §7.3.5, §8.3, §9.3).

use alloc::boxed::Box;

use rusty_h264_common::cabac_tables::NZC_CACHE;
use rusty_h264_common::predict::{nnz_raster_from_z, I16Mode, LUMA_4X4_SCAN_XY};
use rusty_h264_common::transform::inverse_quant_luma_dc_scan;
use rusty_h264_common::BitReader;

use super::estado::{CorteCabac, PosicionMb, RejillasCorte};
use super::{
    cache_nzc_vecinos, cond_term_flag, valor_vecino, BYTES_I_PCM, CATEGORIA_I_16X16,
    CATEGORIA_I_NXN, CATEGORIA_I_PCM, CBP_I_PCM, CTX_TRANSFORM_8X8, MB_TYPE_I_PCM, NNZ_I_PCM,
    NZC_NO_DISPONIBLE,
};
use crate::cabac::Cabac;
use crate::cabac_residuo::{
    parse_mb_residual_cabac, residual_block_eng, resolve_ndc, ResidualOut, RP_CHROMA_AC,
    RP_CHROMA_DC, RP_I16_AC, RP_I16_DC,
};
use crate::error::MbError;
use crate::mb16::FrameDecoder;
use crate::mb_cabac::{
    intra4x4_pred_mode_eng, parse_cbp_cabac, parse_intra_chroma_pred_mode_cabac,
    parse_mb_qp_delta_cabac, parse_mb_type_i_cabac,
};
use crate::mb_intra::modo_intra_desde_rem;

impl FrameDecoder {
    /// `mb_type` de un macrobloque de un corte I (§9.3.3.1.1.3: el contexto cuenta los vecinos
    /// que no son `I_NxN`).
    ///
    /// # Errors
    /// `MbError::Inconsistent` si la tabla de categorías no cubre un vecino disponible.
    pub(super) fn mb_type_cabac_i(
        corte: &mut CorteCabac<'_>,
        pos: PosicionMb,
    ) -> Result<u32, MbError> {
        let CorteCabac { cab, rejillas, .. } = corte;
        let RejillasCorte { cat, .. } = rejillas;
        let PosicionMb { left, top, .. } = pos;

        let li = cond_term_flag(cat, left, |c| c >= CATEGORIA_I_16X16, "cat")?;
        let ti = cond_term_flag(cat, top, |c| c >= CATEGORIA_I_16X16, "cat")?;
        Ok(parse_mb_type_i_cabac(cab, li + ti))
    }

    /// Un macrobloque intra de cualquier tipo de corte (`I_PCM`, `I_16x16`, `I_4x4` o `I_8x8`) con su
    /// `mb_type` ya leído y normalizado a la numeración del corte I (§7.4.5, tabla 7-11).
    /// Devuelve `end_of_slice_flag`.
    ///
    /// # Errors
    /// Los de la decodificación del macrobloque.
    pub(super) fn mb_cabac_intra(
        &mut self,
        corte: &mut CorteCabac<'_>,
        pos: PosicionMb,
        mb_type: u32,
    ) -> Result<bool, MbError> {
        let PosicionMb {
            mbx,
            mby,
            left,
            top,
            ..
        } = pos;

        // H-48: the CABAC intra path is INLINED in this loop, not routed through
        // `decode_intra_mb` (which only the CAVLC readers call) — wiring the scope
        // there reported ZERO calls against 480,510 intra-pred calls. All three
        // intra entries (I-slice, P-slice mb_type>3, B-slice bmt>=23) converge
        // here, so this is the one point that sees every intra MB.
        self.edc_flush()?; // intra reconstruction reads neighbour PIXELS
                           // Intra prediction ALSO reads the coded_y/modes_y/inter_y grids a
                           // deferred span still owes (this arm is inline — it never passes
                           // decode_b_mb's flush). Caught by the tempete ARM-DIFF.
        self.span_flush()?;
        // Intra bS is a constant pattern (4 on MB edges, 3 internal) — written
        // here because CABAC inlines I recon and never calls decode_intra_mb.
        *self
            .mb_kind
            .get_mut(mby * self.mb_w + mbx)
            .ok_or(MbError::Inconsistent("mb_kind"))? = rusty_h264_common::deblock::MB_KIND_INTRA;
        if mb_type == MB_TYPE_I_PCM {
            return self.intra_pcm(corte, pos);
        }
        // chroma-pred-mode ctxInc from neighbour chroma modes (1..=3).
        let cci = cond_term_flag(
            &corte.rejillas.cmode,
            left,
            |c| (1..=3).contains(&c),
            "cmode",
        )? + cond_term_flag(
            &corte.rejillas.cmode,
            top,
            |c| (1..=3).contains(&c),
            "cmode",
        )?;

        if mb_type != 0 {
            return self.intra_16x16(corte, pos, mb_type, cci);
        }
        self.intra_nxn(corte, pos, cci)
    }

    /// `I_PCM` (§7.3.5): las 384 muestras van en crudo dentro del flujo CABAC, alineadas a byte; el
    /// motor se reinicia tras ellas conservando los contextos (§9.3.1). Devuelve `end_of_slice_flag`.
    ///
    /// # Errors
    /// Los de la decodificación del macrobloque.
    pub(super) fn intra_pcm(
        &mut self,
        corte: &mut CorteCabac<'_>,
        pos: PosicionMb,
    ) -> Result<bool, MbError> {
        let CorteCabac {
            cab,
            last_delta_qp,
            rbsp,
            rejillas,
            ..
        } = corte;
        let RejillasCorte {
            cat,
            mb_cbp,
            cmode,
            mb_nzc,
            cbf_dc,
            ..
        } = rejillas;
        let PosicionMb { addr, mbx, mby, .. } = pos;

        // ---- I_PCM (spec §7.3.5): 384 raw byte-aligned sample bytes inside
        // the CABAC stream. The PCM marker was a terminate bin, so the engine
        // has stopped; DecodeFlush + pcm_alignment_zero_bit put the samples
        // at `pcm_start_byte()`, and the engine re-initialises after them
        // with its CONTEXTS KEPT (§9.3.1). All three slice types (I, P via
        // mbt 30, B via bmt 48) reach here as mb_type 25.
        let pcm = cab.pcm_start_byte();
        let end = pcm + BYTES_I_PCM;
        if end > rbsp.len() {
            return Err(MbError::Truncated);
        }
        let mut pr = BitReader::new(&rbsp[pcm..end]);
        self.decode_ipcm(&mut pr, mbx, mby)?;
        cab.reinit_at(end);
        // Neighbour context (§7.4.5 + §9.3.3.1.1.x inferences): intra, QPy
        // unchanged (no mb_qp_delta — its ctxInc resets), CodedBlockPattern
        // luma/chroma inferred 15/2, every coded_block_flag (incl. DC) 1,
        // nnz 16, chroma pred mode 0.
        // PARCHE-WODW (ARR-1): QPY de un I_PCM para el desbloqueo = 0 (§8.7.2.2);
        // `cur_qp` (predicción del siguiente) no cambia.
        *self
            .mb_qp
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_qp"))? = 0;
        *cat.get_mut(addr).ok_or(MbError::Inconsistent("cat"))? = CATEGORIA_I_PCM;
        *cmode.get_mut(addr).ok_or(MbError::Inconsistent("cmode"))? = 0;
        *mb_cbp
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_cbp"))? = CBP_I_PCM;
        *cbf_dc
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("cbf_dc"))? =
            (1 << RP_I16_DC) | (1 << RP_CHROMA_DC) | (1 << (RP_CHROMA_DC + 1));
        *mb_nzc
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_nzc"))? = [NNZ_I_PCM; 24];
        *last_delta_qp = 0;
        Ok(cab.decode_terminate())
    }

    /// `I_16x16` (`mb_type` 1..=24): modo de predicción y `coded_block_pattern` salen de `mb_type`;
    /// `cci` es el `ctxIdxInc` de `intra_chroma_pred_mode`. Devuelve `end_of_slice_flag`.
    ///
    /// # Errors
    /// Los de la decodificación del macrobloque.
    #[allow(clippy::unnecessary_lazy_evaluations)]
    // Invariante de rendimiento (filtrado.rs:683): evaluación perezosa para evitar
    // construir en la pila arrays de 1024 y 512 bytes en cada macrobloque.
    pub(super) fn intra_16x16(
        &mut self,
        corte: &mut CorteCabac<'_>,
        pos: PosicionMb,
        mb_type: u32,
        cci: usize,
    ) -> Result<bool, MbError> {
        let CorteCabac {
            cab,
            last_delta_qp,
            rejillas,
            ..
        } = corte;
        let RejillasCorte {
            cat,
            mb_cbp,
            cmode,
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

        // ---- I_16x16 (mb_type 1..=24): pred mode & cbp DERIVED from mb_type;
        // luma DC always coded. Syntax order: intra_chroma_pred_mode, mb_qp_delta,
        // luma DC (Hadamard), luma AC (if cbp_luma), chroma DC/AC. Mirrors the CAVLC
        // decode_i16, driven by the CABAC residual. ----
        let mt = mb_type - 1;
        let pred_mode = I16Mode::from_id(mt % 4);
        let cbp_chroma = (mt % 12) / 4;
        let cbp_luma_15 = mt / 12 == 1;
        let chroma_mode = parse_intra_chroma_pred_mode_cabac(cab, cci) as u8;
        *cmode.get_mut(addr).ok_or(MbError::Inconsistent("cmode"))? = chroma_mode as i32;
        *cat.get_mut(addr).ok_or(MbError::Inconsistent("cat"))? = CATEGORIA_I_16X16;
        *mb_cbp
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_cbp"))? =
            ((cbp_chroma as u8) << 4) | if cbp_luma_15 { 15 } else { 0 };
        let w4 = self.mb_w * 4;

        let mut nzc = cache_nzc_vecinos(mb_nzc, top, left)?;

        let ndc = (
            valor_vecino(cbf_dc, top, "cbf_dc")?,
            valor_vecino(cbf_dc, left, "cbf_dc")?,
        );
        let qpd = parse_mb_qp_delta_cabac(cab, last_delta_qp);
        self.step_qp(qpd)?;
        let qp = self.cur_qp;
        let mut cbfdc = 0u16;
        let nd = resolve_ndc(ndc, true);
        let (data, mut e, ctx) = cab.view();

        // Luma DC (iz=0, category I16_LUMA_DC, 16 coeffs) → Hadamard dequant.
        let mut dc_scan = [0i32; 16];
        residual_block_eng::<RP_I16_DC, 16>(
            &mut e,
            data,
            ctx,
            &mut nzc,
            &mut cbfdc,
            0,
            0,
            true,
            nd,
            &mut dc_scan,
        );
        // FUSED scan-order DC: un-scan + Hadamard + scale in one kernel (was 32 moves + 253 instrs).
        let recon_dc =
            inverse_quant_luma_dc_scan(&dc_scan, qp, self.scaling.as_ref().map(|s| s[0][0]));

        // Luma AC (iz 0..15, category I16_LUMA_AC, 15 coeffs) when cbp_luma set.
        // Materialised only when AC is actually coded — a DC-only
        // I_16x16 zeroed 1 KB of stack for nothing.
        let mut q_blocks: Option<[[i32; 16]; 16]> = None;
        let mut coded16 = 0u16;
        for (iz, &(lbx, lby)) in LUMA_4X4_SCAN_XY.iter().enumerate() {
            let total = if cbp_luma_15 {
                let mut ac = [0i32; 16];
                let t = residual_block_eng::<RP_I16_AC, 16>(
                    &mut e, data, ctx, &mut nzc, &mut cbfdc, iz, 0, true, nd, &mut ac,
                );
                q_blocks.get_or_insert_with(|| [[0i32; 16]; 16])[(lby & 3) * 4 + (lbx & 3)] = ac; // SCAN order (fused kernel)
                t as u8
            } else {
                nzc[NZC_CACHE[iz.min(23)].min(47)] = 0;
                0
            };
            coded16 |= ((total != 0) as u16) << ((lby & 3) * 4 + (lbx & 3));
            *self
                .nnz_y
                .get_mut((mby * 4 + lby) * w4 + (mbx * 4 + lbx))
                .ok_or(MbError::Inconsistent("nnz_y"))? = total;
        }

        let mut cdc = [[0i32; 4]; 2];
        let mut cac: Option<[[[i32; 16]; 4]; 2]> = None;
        let mut cnnz = [0u8; 8];
        if cbp_chroma >= 1 {
            for (i, cdc_i) in cdc.iter_mut().enumerate() {
                residual_block_eng::<RP_CHROMA_DC, 4>(
                    &mut e,
                    data,
                    ctx,
                    &mut nzc,
                    &mut cbfdc,
                    16 + i * 4,
                    i,
                    true,
                    nd,
                    cdc_i,
                );
            }
        }
        if cbp_chroma == 2 {
            let cacm = cac.get_or_insert_with(|| [[[0i32; 16]; 4]; 2]);
            for i in 0..2usize {
                for id4 in 0..4usize {
                    cnnz[(i * 4 + id4) & 7] = residual_block_eng::<RP_CHROMA_AC, 16>(
                        &mut e,
                        data,
                        ctx,
                        &mut nzc,
                        &mut cbfdc,
                        16 + i * 4 + id4,
                        i,
                        true,
                        nd,
                        &mut cacm[i][id4],
                    ) as u8;
                }
            }
        }

        cab.commit(e);
        // Luma recon: 16×16 intra prediction, then per-4×4 (dequant AC + injected DC).
        let top_ok = mby > 0
            && self.nbr_in_slice(mbx, mby - 1)
            && self.intra_nbr_ok(mbx * 4, mby * 4 - 1)?;
        let left_ok = mbx > 0
            && self.nbr_in_slice(mbx - 1, mby)
            && self.intra_nbr_ok(mbx * 4 - 1, mby * 4)?;
        self.recon_i16_luma(
            mbx,
            mby,
            pred_mode,
            top_ok,
            left_ok,
            q_blocks.as_ref(),
            coded16,
            &recon_dc,
            qp,
        )?;
        self.recon_chroma_cabac(
            mbx,
            mby,
            chroma_mode,
            &cdc,
            cac.as_ref(),
            &cnnz,
            cbp_chroma,
            top_ok,
            left_ok,
        )?;

        *self
            .mb_qp
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_qp"))? = self.cur_qp;
        *cbf_dc
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("cbf_dc"))? = cbfdc;
        let mut mn = [0u8; 24];
        mn[..4].copy_from_slice(&nzc[9..13]);
        mn[4..8].copy_from_slice(&nzc[17..21]);
        mn[8..12].copy_from_slice(&nzc[25..29]);
        mn[12..16].copy_from_slice(&nzc[33..37]);
        (mn[16], mn[17], mn[20], mn[21]) = (nzc[14], nzc[15], nzc[22], nzc[23]);
        (mn[18], mn[19], mn[22], mn[23]) = (nzc[38], nzc[39], nzc[46], nzc[47]);
        for v in mn.iter_mut() {
            if *v == NZC_NO_DISPONIBLE {
                *v = 0;
            }
        }
        *mb_nzc
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_nzc"))? = mn;

        Ok(cab.decode_terminate())
    }

    /// `I_4x4` o `I_8x8` (`mb_type` 0): `transform_size_8x8_flag`, modos de predicción por bloque,
    /// croma, `coded_block_pattern` y residuo; `cci` es el `ctxIdxInc` de `intra_chroma_pred_mode`.
    /// Devuelve `end_of_slice_flag`.
    ///
    /// # Errors
    /// Los de la decodificación del macrobloque.
    pub(super) fn intra_nxn(
        &mut self,
        corte: &mut CorteCabac<'_>,
        pos: PosicionMb,
        cci: usize,
    ) -> Result<bool, MbError> {
        let CorteCabac {
            cab,
            last_delta_qp,
            rejillas,
            ..
        } = corte;
        let RejillasCorte {
            cat,
            mb_cbp,
            cmode,
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

        *cat.get_mut(addr).ok_or(MbError::Inconsistent("cat"))? = CATEGORIA_I_NXN;
        let w4 = self.mb_w * 4;
        // H-49: transform_size_8x8_flag. For I_NxN it precedes the intra pred
        // modes (spec §7.3.5); ctxIdx = 399 + condTermFlagA + condTermFlagB,
        // each 1 when that neighbour MB carries the flag. Omitting this read is
        // what desynced every High-profile stream.
        let t8 = self.transform_8x8_mode && {
            let a = cond_term_flag(&self.mb_t8x8, left, |f| f, "mb_t8x8")?;
            let b = cond_term_flag(&self.mb_t8x8, top, |f| f, "mb_t8x8")?;
            cab.decode_decision(CTX_TRANSFORM_8X8 + a + b) != 0
        };
        *self
            .mb_t8x8
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_t8x8"))? = t8;
        self.any_t8 |= t8;
        // Hoisted ABOVE the mode loop (it was computed after it) so the
        // per-block mode prediction can use it - see `predict_i4_mode_fast`.
        let top_ok = mby > 0
            && self.nbr_in_slice(mbx, mby - 1)
            && self.intra_nbr_ok(mbx * 4, mby * 4 - 1)?;
        let left_ok = mbx > 0
            && self.nbr_in_slice(mbx - 1, mby)
            && self.intra_nbr_ok(mbx * 4 - 1, mby * 4)?;
        // Brick 2.4 + recon: derive & store each intra mode (prev-flag → the
        // neighbour-predicted mode, else rem), exactly as the CAVLC path.
        let (modes, modes8) = self.leer_modos_nxn(cab, (mbx, mby), t8, (top_ok, left_ok))?;
        let chroma_mode = parse_intra_chroma_pred_mode_cabac(cab, cci) as u8;
        *cmode.get_mut(addr).ok_or(MbError::Inconsistent("cmode"))? = chroma_mode as i32;
        let cbp = parse_cbp_cabac(
            cab,
            valor_vecino(mb_cbp, top, "mb_cbp")?,
            valor_vecino(mb_cbp, left, "mb_cbp")?,
        );
        *mb_cbp
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_cbp"))? = cbp as u8;
        let (cbp_luma, cbp_chroma) = (cbp & 15, cbp >> 4);

        // Build the padded nzc cache from neighbours (openh264 WelsFillCacheNonZeroCount).
        let mut nzc = cache_nzc_vecinos(mb_nzc, top, left)?;

        // Bricks 2.6 + 2.7: mb_qp_delta + residual (I_4x4 luma 4×4 + chroma DC/AC),
        // storing scan-order coefficients for recon.
        let mut cbfdc = 0u16;
        // Both materialised only when the parse writes them: an I_8x8
        // macroblock never touches luma_scan (it carries luma8), and
        // cbp_chroma < 2 never touches cac.
        // Scratch planes (see `scratch_luma`): zeroed per coded block by the parse.
        let mut luma_scan = self
            .scratch_luma
            .take()
            .unwrap_or_else(|| Box::new([[0i32; 16]; 16]));
        let mut luma8 = self
            .scratch_luma8
            .take()
            .unwrap_or_else(|| Box::new([[0i32; 64]; 4]));
        let mut cdc = [[0i32; 4]; 2];
        let mut cac = self
            .scratch_cac
            .take()
            .unwrap_or_else(|| Box::new([[[0i32; 16]; 4]; 2]));
        let mut nnzs = [0u8; 24]; // parse-side per-block coeff counts
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
            parse_mb_residual_cabac::<true>(
                cab,
                &mut nzc,
                &mut cbfdc,
                ndc,
                cbp_luma,
                cbp_chroma,
                t8,
                ResidualOut {
                    luma: &mut luma_scan,
                    luma8: &mut luma8,
                    cdc: &mut cdc,
                    cac: &mut cac,
                    nnzs: &mut nnzs,
                },
            );
            if t8 {
                // Per-cell nnz for the deblock/neighbour readers: the 8x8 total
                // (0 for an uncoded 8x8) in each of its four 4x4 cells.
                for id8 in 0..4usize {
                    let n = nnzs[id8 * 4];
                    let (b8x, b8y) = (id8 % 2, id8 / 2);
                    for sy in 0..2 {
                        for sx in 0..2 {
                            *self
                                .nnz_y
                                .get_mut((mby * 4 + b8y * 2 + sy) * w4 + (mbx * 4 + b8x * 2 + sx))
                                .ok_or(MbError::Inconsistent("nnz_y"))? = n;
                        }
                    }
                }
            }
        }
        *self
            .mb_qp
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_qp"))? = self.cur_qp;
        *cbf_dc
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("cbf_dc"))? = cbfdc;
        // Extract the MB's nzc (raster luma + chroma) for future neighbours.
        let mn = nnz_raster_from_z(&nnzs);
        *mb_nzc
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_nzc"))? = mn;

        // ---- Brick 4.3a: recon (I_4x4 luma + chroma) via the CAVLC-proven primitives.
        if t8 {
            self.reconstruir_luma_i8((mbx, mby), modes8, cbp_luma, &luma8, (top_ok, left_ok))?;
        } else {
            // The `t8` guard was INSIDE the loop (breaking on the first
            // iteration) and the `luma_scan` Option was re-resolved on EVERY one
            // of the sixteen blocks. Both are macroblock-invariant.
            self.reconstruir_luma_i4(
                (mbx, mby),
                &modes,
                &luma_scan,
                &nnzs,
                &mn,
                (top_ok, left_ok),
            )?;
        }
        let chroma_nnz: [u8; 8] = core::array::from_fn(|i| nnzs[16 + i]);
        self.recon_chroma_cabac(
            mbx,
            mby,
            chroma_mode,
            &cdc,
            (cbp_chroma == 2).then_some(&*cac),
            &chroma_nnz,
            cbp_chroma,
            top_ok,
            left_ok,
        )?;
        self.scratch_luma = Some(luma_scan);
        self.scratch_luma8 = Some(luma8);
        self.scratch_cac = Some(cac);

        // Brick 2.1: end_of_slice_flag.
        Ok(cab.decode_terminate())
    }

    /// Modos de predicción de luma de un `I_4x4` (16) o un `I_8x8` (4, repetidos en sus cuatro
    /// celdas 4x4): cada uno es el predicho por los vecinos o el que dé `rem_intra_pred_mode`
    /// (§7.3.5.1, §8.3.1.1, §8.3.2.1). Los guarda en `modes_y` y devuelve los 16 en orden de
    /// trama y, con `t8`, los 4 de 8x8. `disponibles` dice si hay vecino de arriba y de la
    /// izquierda para la predicción.
    ///
    /// # Errors
    /// `MbError::Inconsistent` si una celda cae fuera de `modes_y`.
    fn leer_modos_nxn(
        &mut self,
        cab: &mut Cabac<'_>,
        (mbx, mby): (usize, usize),
        t8: bool,
        (top_ok, left_ok): (bool, bool),
    ) -> Result<([u8; 16], [u8; 4]), MbError> {
        let w4 = self.mb_w * 4;
        let mut modes = [2u8; 16]; // raster [lby*4+lbx]
        let mut modes8 = [2u8; 4]; // one per 8×8 when t8
                                   // ONE engine view for all 16 (or 4) pred-mode reads of the macroblock.
        let (data, mut e, ctx) = cab.view();
        if t8 {
            // One mode per 8×8, broadcast to its four 4×4 cells so neighbour
            // mode prediction keeps working unchanged.
            for (b8, modo8) in modes8.iter_mut().enumerate() {
                let (b8x, b8y) = (b8 % 2, b8 / 2);
                let (bx, by) = (mbx * 4 + b8x * 2, mby * 4 + b8y * 2);
                let predicted =
                    self.predict_i4_mode_fast(bx, by, b8x * 2, b8y * 2, top_ok, left_ok)?;
                let rr = intra4x4_pred_mode_eng(&mut e, data, ctx);
                let actual = if rr < 0 {
                    predicted
                } else {
                    modo_intra_desde_rem(predicted, rr as u8)
                };
                *modo8 = actual;
                for dy in 0..2 {
                    for dx in 0..2 {
                        *self
                            .modes_y
                            .get_mut((by + dy) * w4 + (bx + dx))
                            .ok_or(MbError::Inconsistent("modes_y"))? = actual;
                        modes[(b8y * 2 + dy) * 4 + (b8x * 2 + dx)] = actual;
                    }
                }
            }
        } else {
            for &(lbx, lby) in &LUMA_4X4_SCAN_XY {
                let (bx, by) = (mbx * 4 + lbx, mby * 4 + lby);
                let predicted = self.predict_i4_mode_fast(bx, by, lbx, lby, top_ok, left_ok)?;
                let rr = intra4x4_pred_mode_eng(&mut e, data, ctx);
                let actual = if rr < 0 {
                    predicted
                } else {
                    modo_intra_desde_rem(predicted, rr as u8)
                };
                *self
                    .modes_y
                    .get_mut(by * w4 + bx)
                    .ok_or(MbError::Inconsistent("modes_y"))? = actual;
                modes[(lby & 3) * 4 + (lbx & 3)] = actual;
            }
        }
        cab.commit(e);
        Ok((modes, modes8))
    }

    /// Reconstrucción de la luma de un `I_8x8`: predicción de cada bloque 8x8 con su modo y, si
    /// `cbp_luma` lo marca como codificado, su residuo de `luma8` (§8.3.2, §8.5.13); marca las
    /// celdas como codificadas en `coded_y`. Usa el QP del macrobloque (`cur_qp`).
    ///
    /// # Errors
    /// Los de la reconstrucción de cada bloque.
    fn reconstruir_luma_i8(
        &mut self,
        (mbx, mby): (usize, usize),
        modes8: [u8; 4],
        cbp_luma: u32,
        luma8: &[[i32; 64]; 4],
        (top_ok, left_ok): (bool, bool),
    ) -> Result<(), MbError> {
        let w4 = self.mb_w * 4;
        let qp = self.cur_qp;
        // I_8x8 recon, reusing the CAVLC-proven primitives verbatim
        // (un_scan_8x8 / inv_quant8 / gather_i8 / intra8x8_pred /
        // add_residual_8x8). Only the ENTROPY half differed.
        for b8 in 0..4usize {
            let (b8x, b8y) = (b8 % 2, b8 / 2);
            let (bx, by) = (mbx * 4 + b8x * 2, mby * 4 + b8y * 2);
            let coded = cbp_luma & (1 << b8) != 0;
            let avail_top = b8y > 0 || top_ok;
            let avail_left = b8x > 0 || left_ok;
            self.recon_i8_block(
                bx,
                by,
                modes8[b8],
                avail_top,
                avail_left,
                coded.then_some(&luma8[b8]),
                qp,
            )?;
            for sy in 0..2 {
                // Row fill: the 2x2 cell block is two contiguous PAIRS.
                self.coded_y[(by + sy) * w4 + bx..][..2].fill(true);
            }
        }
        Ok(())
    }

    /// Reconstrucción de la luma de un `I_4x4`: los 16 bloques en orden de decodificación,
    /// predicción con su modo más residuo de `scans` (§8.3.1, §8.5.12); vuelca `mn` (los
    /// coeficientes por celda en orden de trama) a `nnz_y` y marca las celdas en `coded_y`. Usa
    /// el QP del macrobloque (`cur_qp`).
    ///
    /// # Errors
    /// Los de la reconstrucción de cada bloque.
    fn reconstruir_luma_i4(
        &mut self,
        (mbx, mby): (usize, usize),
        modes: &[u8; 16],
        scans: &[[i32; 16]; 16],
        nnzs: &[u8; 24],
        mn: &[u8; 24],
        (top_ok, left_ok): (bool, bool),
    ) -> Result<(), MbError> {
        let w4 = self.mb_w * 4;
        let qp = self.cur_qp;
        // Residual ladders decided up front (`i4_prepare`), then the serial
        // predict+add walk with the SIMD IDCT+add kernel per coded block.
        let lnnz: [u8; 16] = core::array::from_fn(|i| nnzs[i]);
        let kinds = self.i4_prepare(scans, &lnnz, qp);
        let dq0 = self.dq_const(qp, 0);
        for (blk, &(lbx, lby)) in LUMA_4X4_SCAN_XY.iter().enumerate() {
            let (bx, by) = (mbx * 4 + lbx, mby * 4 + lby);
            let at = lby > 0 || top_ok;
            let al = lbx > 0 || left_ok;
            self.recon_i4_block_res(
                bx,
                by,
                modes[(lby & 3) * 4 + (lbx & 3)],
                at,
                al,
                kinds[blk & 15],
                scans,
                &dq0,
            )?;
        }
        // Per-MB ROW COPIES for nnz_y (from the raster `mn` already built) and
        // coded_y (routing round): sixteen + sixteen checked scattered stores
        // per macroblock -> four + four row operations. Nothing inside this
        // macroblock reads either grid for its own blocks (interior top-right
        // availability is the z-order constant now).
        for ry in 0..4usize {
            let a = (mby * 4 + ry) * w4 + mbx * 4;
            self.nnz_y[a..a + 4].copy_from_slice(&mn[ry * 4..ry * 4 + 4]);
            self.coded_y[a..a + 4].fill(true);
        }
        Ok(())
    }
}
