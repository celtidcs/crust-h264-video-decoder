//! Decodificación de sintaxis intra en CAVLC (§7.3.5).

use rusty_h264_common::cavlc::{decode_residual_block_into, read_cbp_intra};
use rusty_h264_common::predict::{I16Mode, CHROMA_4X4_SCAN_XY, LUMA_4X4_SCAN_XY};
use rusty_h264_common::transform::inverse_quant_luma_dc_scan;
use rusty_h264_common::BitReader;

use super::modo_intra_desde_rem;
use crate::error::MbError;
use crate::mb16::FrameDecoder;

/// Cabecera sintáctica compartida por los macrobloques intra en CAVLC.
pub(crate) struct CabeceraIntraCavlc {
    /// Modo de predicción intra para crominancia (`intra_chroma_pred_mode`).
    pub(crate) chroma_mode: u8,
    /// Patrón de bloques codificados para luminancia (máscara de 4 bits).
    pub(crate) cbp_luma: u32,
    /// Patrón de bloques codificados para crominancia (0 = ninguno, 1 = solo DC, 2 = DC y AC).
    pub(crate) cbp_chroma: u32,
    /// Parámetro de cuantización resultante tras aplicar el delta del macrobloque (`QPy`).
    pub(crate) qp: u8,
}

impl FrameDecoder {
    /// Reconstruye un macrobloque `I_PCM` (muestras directas de 8 bits sin predicción ni cuantización).
    pub(crate) fn decode_ipcm(
        &mut self,
        r: &mut BitReader,
        mb_x: usize,
        mb_y: usize,
    ) -> Result<(), MbError> {
        r.align_to_byte()?;
        let (lx, ly) = (mb_x * 16, mb_y * 16);
        for dy in 0..16 {
            let row = &mut self.rec_y[(ly + dy) * self.cw + lx..][..16];
            for px in row.iter_mut() {
                #[allow(clippy::cast_possible_truncation)]
                {
                    *px = r.read_bits(8)? as u8;
                }
            }
        }
        let (cx, cy) = (mb_x * 8, mb_y * 8);
        let ccw = self.ccw;
        for plane in [&mut self.rec_u, &mut self.rec_v] {
            for dy in 0..8 {
                let row = &mut plane[(cy + dy) * ccw + cx..][..8];
                for px in row.iter_mut() {
                    #[allow(clippy::cast_possible_truncation)]
                    {
                        *px = r.read_bits(8)? as u8;
                    }
                }
            }
        }
        let (w4, w2) = (self.mb_w * 4, self.mb_w * 2);
        for ry in 0..4 {
            let base = (mb_y * 4 + ry) * w4 + mb_x * 4;
            self.nnz_y[base..][..4].fill(16);
            self.modes_y[base..][..4].fill(2);
            self.coded_y[base..][..4].fill(true);
            self.inter_y[base..][..4].fill(false);
            self.ref_idx_y[base..][..4].fill(-1);
            self.mv_y[base..][..4].fill((0, 0));
        }
        for c in 0..2 {
            for by in 0..2 {
                self.nnz_c[c][(mb_y * 2 + by) * w2 + mb_x * 2..][..2].fill(16);
            }
        }
        Ok(())
    }

    /// Lee la cabecera intra compartida en CAVLC (`chroma_mode`, `cbp` y actualización de QP).
    pub(crate) fn leer_cabecera_intra_cavlc(
        &mut self,
        r: &mut BitReader,
    ) -> Result<CabeceraIntraCavlc, MbError> {
        #[allow(clippy::cast_possible_truncation)]
        let chroma_mode = r.read_ue()? as u8;
        let cbp = read_cbp_intra(r)?;
        let cbp_luma = cbp & 15;
        let cbp_chroma = cbp >> 4;
        if cbp != 0 {
            self.step_qp(r.read_se()?)?;
        }
        let qp = self.cur_qp;
        Ok(CabeceraIntraCavlc {
            chroma_mode,
            cbp_luma,
            cbp_chroma,
            qp,
        })
    }

    /// Decodifica un macrobloque CAVLC `I_4x4`.
    #[allow(clippy::too_many_lines)]
    pub(crate) fn decode_i4x4(
        &mut self,
        r: &mut BitReader,
        mb_x: usize,
        mb_y: usize,
    ) -> Result<(), MbError> {
        let w4 = self.mb_w * 4;
        let mut modes = [2u8; 16];
        for &(lbx, lby) in &LUMA_4X4_SCAN_XY {
            let (bx, by) = (mb_x * 4 + lbx, mb_y * 4 + lby);
            let predicted = self.predict_i4_mode(bx, by)?;
            let actual = if r.read_bit()? {
                predicted
            } else {
                #[allow(clippy::cast_possible_truncation)]
                let rem = r.read_bits(3)? as u8;
                modo_intra_desde_rem(predicted, rem)
            };
            *self
                .modes_y
                .get_mut(by * w4 + bx)
                .ok_or(MbError::Inconsistent("modes_y"))? = actual;
            modes[(lby & 3) * 4 + (lbx & 3)] = actual;
        }

        let cab = self.leer_cabecera_intra_cavlc(r)?;
        let (chroma_mode, cbp_luma, cbp_chroma, qp) =
            (cab.chroma_mode, cab.cbp_luma, cab.cbp_chroma, cab.qp);

        let top_mb_avail = mb_y > 0
            && self.nbr_in_slice(mb_x, mb_y - 1)
            && self.intra_nbr_ok(mb_x * 4, mb_y * 4 - 1)?;
        let left_mb_avail = mb_x > 0
            && self.nbr_in_slice(mb_x - 1, mb_y)
            && self.intra_nbr_ok(mb_x * 4 - 1, mb_y * 4)?;
        self.nnz_cache_load(mb_x, mb_y)?;
        let mut nnz_raster = [0u8; 16];
        let mut scans = [[0i32; 16]; 16];
        let mut totals = [0u8; 16];
        for (blk, &(lbx, lby)) in LUMA_4X4_SCAN_XY.iter().enumerate() {
            let total = if cbp_luma & (1 << (blk / 4)) != 0 {
                let nc = self.nc_pred(lbx, lby);
                decode_residual_block_into::<16, 16>(r, nc, &mut scans[blk & 15])?
            } else {
                0
            };
            self.nnz_cache_set(lbx, lby, total);
            nnz_raster[(lby & 3) * 4 + (lbx & 3)] = total;
            totals[blk & 15] = total;
        }
        let kinds = self.i4_prepare(&scans, &totals, qp);
        let dq0 = self.dq_const(qp, 0);
        for (blk, &(lbx, lby)) in LUMA_4X4_SCAN_XY.iter().enumerate() {
            let (bx, by) = (mb_x * 4 + lbx, mb_y * 4 + lby);
            let avail_top = lby > 0 || top_mb_avail;
            let avail_left = lbx > 0 || left_mb_avail;
            self.recon_i4_block_res(
                bx,
                by,
                modes[(lby & 3) * 4 + (lbx & 3)],
                avail_top,
                avail_left,
                kinds[blk & 15],
                &scans,
                &dq0,
            )?;
        }
        for ry in 0..4usize {
            self.coded_y[(mb_y * 4 + ry) * w4 + mb_x * 4..][..4].fill(true);
        }
        for by in 0..4usize {
            let a = (mb_y * 4 + by) * w4 + mb_x * 4;
            self.nnz_y[a..a + 4].copy_from_slice(&nnz_raster[by * 4..by * 4 + 4]);
        }

        self.decode_chroma(r, mb_x, mb_y, cbp_chroma, chroma_mode)
    }

    /// Decodifica un macrobloque CAVLC `I_8x8` (High profile).
    #[allow(clippy::too_many_lines)]
    pub(crate) fn decode_i8x8(
        &mut self,
        r: &mut BitReader,
        mb_x: usize,
        mb_y: usize,
    ) -> Result<(), MbError> {
        let w4 = self.mb_w * 4;
        *self
            .mb_t8x8
            .get_mut(mb_y * self.mb_w + mb_x)
            .ok_or(MbError::Inconsistent("mb_t8x8"))? = true;
        self.any_t8 = true;

        let mut modes8 = [2u8; 4];
        for (b8, mode) in modes8.iter_mut().enumerate() {
            let (b8x, b8y) = (b8 % 2, b8 / 2);
            let (bx, by) = (mb_x * 4 + b8x * 2, mb_y * 4 + b8y * 2);
            let predicted = self.predict_i4_mode(bx, by)?;
            let actual = if r.read_bit()? {
                predicted
            } else {
                #[allow(clippy::cast_possible_truncation)]
                let rem = r.read_bits(3)? as u8;
                modo_intra_desde_rem(predicted, rem)
            };
            *mode = actual;
            for sy in 0..2 {
                self.modes_y[(by + sy) * w4 + bx..][..2].fill(actual);
            }
        }

        let cab = self.leer_cabecera_intra_cavlc(r)?;
        let (chroma_mode, cbp_luma, cbp_chroma, qp) =
            (cab.chroma_mode, cab.cbp_luma, cab.cbp_chroma, cab.qp);

        let top_mb_avail = mb_y > 0
            && self.nbr_in_slice(mb_x, mb_y - 1)
            && self.intra_nbr_ok(mb_x * 4, mb_y * 4 - 1)?;
        let left_mb_avail = mb_x > 0
            && self.nbr_in_slice(mb_x - 1, mb_y)
            && self.intra_nbr_ok(mb_x * 4 - 1, mb_y * 4)?;
        self.nnz_cache_load(mb_x, mb_y)?;

        let mut nnz_raster = [0u8; 16];
        for (b8, &mode8) in modes8.iter().enumerate() {
            let (b8x, b8y) = (b8 % 2, b8 / 2);
            let (bx, by) = (mb_x * 4 + b8x * 2, mb_y * 4 + b8y * 2);

            let mut scan8 = [0i32; 64];
            let coded8 = cbp_luma & (1 << b8) != 0;
            if coded8 {
                for sub in 0..4 {
                    let (sx, sy) = (sub % 2, sub / 2);
                    let (cx, cy) = (b8x * 2 + sx, b8y * 2 + sy);
                    let nc = self.nc_pred(cx, cy);
                    let mut blk = [0i32; 16];
                    let total = decode_residual_block_into::<16, 16>(r, nc, &mut blk)?;
                    self.nnz_cache_set(cx, cy, total);
                    nnz_raster[(cy & 3) * 4 + (cx & 3)] = total;
                    for k in 0..16 {
                        scan8[4 * k + sub] = blk[k];
                    }
                }
            } else {
                for sub in 0..4 {
                    let (sx, sy) = (sub % 2, sub / 2);
                    self.nnz_cache_set(b8x * 2 + sx, b8y * 2 + sy, 0);
                }
            }

            let avail_top = b8y > 0 || top_mb_avail;
            let avail_left = b8x > 0 || left_mb_avail;
            self.recon_i8_block(
                bx,
                by,
                mode8,
                avail_top,
                avail_left,
                coded8.then_some(&scan8),
                qp,
            )?;
            for sy in 0..2 {
                // Relleno de filas: el bloque de celdas 2x2 son dos pares contiguos.
                self.coded_y[(by + sy) * w4 + bx..][..2].fill(true);
            }
        }
        for ry in 0..4usize {
            let a = (mb_y * 4 + ry) * w4 + mb_x * 4;
            self.nnz_y[a..a + 4].copy_from_slice(&nnz_raster[ry * 4..ry * 4 + 4]);
        }

        self.decode_chroma(r, mb_x, mb_y, cbp_chroma, chroma_mode)
    }

    /// Decodifica un macrobloque CAVLC `I_16x16`.
    pub(crate) fn decode_i16(
        &mut self,
        r: &mut BitReader,
        mb_x: usize,
        mb_y: usize,
        mt: u32,
    ) -> Result<(), MbError> {
        let pred_mode = I16Mode::from_id(mt % 4);
        let cbp_chroma = (mt % 12) / 4;
        let cbp_luma_15 = mt / 12 == 1;
        #[allow(clippy::cast_possible_truncation)]
        let chroma_mode = r.read_ue()? as u8;
        self.step_qp(r.read_se()?)?;
        let qp = self.cur_qp;
        let w4 = self.mb_w * 4;

        self.nnz_cache_load(mb_x, mb_y)?;
        let nc_dc = self.nc_pred(0, 0);
        let mut dc_scan = [0i32; 16];
        decode_residual_block_into::<16, 16>(r, nc_dc, &mut dc_scan)?;
        let recon_dc =
            inverse_quant_luma_dc_scan(&dc_scan, qp, self.scaling.as_ref().map(|s| s[0][0]));

        let mut nnz_raster = [0u8; 16];
        let mut q_blocks = [[0i32; 16]; 16];
        for &(bx, by) in &LUMA_4X4_SCAN_XY {
            let total = if cbp_luma_15 {
                let nc = self.nc_pred(bx, by);
                let mut ac = [0i32; 16];
                let t = decode_residual_block_into::<15, 16>(r, nc, &mut ac)?;
                if t != 0 {
                    q_blocks[(by & 3) * 4 + (bx & 3)] = ac;
                }
                t
            } else {
                0
            };
            self.nnz_cache_set(bx, by, total);
            nnz_raster[(by & 3) * 4 + (bx & 3)] = total;
        }
        let coded16 = nnz_raster
            .iter()
            .enumerate()
            .fold(0u16, |m, (i, &n)| m | ((u16::from(n != 0)) << i));
        for by in 0..4usize {
            let a = (mb_y * 4 + by) * w4 + mb_x * 4;
            self.nnz_y[a..a + 4].copy_from_slice(&nnz_raster[by * 4..by * 4 + 4]);
        }

        let avail_top = mb_y > 0
            && self.nbr_in_slice(mb_x, mb_y - 1)
            && self.intra_nbr_ok(mb_x * 4, mb_y * 4 - 1)?;
        let avail_left = mb_x > 0
            && self.nbr_in_slice(mb_x - 1, mb_y)
            && self.intra_nbr_ok(mb_x * 4 - 1, mb_y * 4)?;
        self.recon_i16_luma(
            mb_x,
            mb_y,
            pred_mode,
            avail_top,
            avail_left,
            Some(&q_blocks),
            coded16,
            &recon_dc,
            qp,
        )?;
        for lby in 0..4usize {
            let a = (mb_y * 4 + lby) * w4 + mb_x * 4;
            self.modes_y[a..a + 4].fill(2);
        }

        self.decode_chroma(r, mb_x, mb_y, cbp_chroma, chroma_mode)
    }

    /// Decodifica y reconstruye el residuo de croma CAVLC (compartido por `I_4x4` e `I_16x16`).
    pub(crate) fn decode_chroma(
        &mut self,
        r: &mut BitReader,
        mb_x: usize,
        mb_y: usize,
        cbp_chroma: u32,
        chroma_mode: u8,
    ) -> Result<(), MbError> {
        let qpc = self.chroma_qp_for(self.cur_qp);
        let avail_top = mb_y > 0
            && self.nbr_in_slice(mb_x, mb_y - 1)
            && self.intra_nbr_ok(mb_x * 4, mb_y * 4 - 1)?;
        let avail_left = mb_x > 0
            && self.nbr_in_slice(mb_x - 1, mb_y)
            && self.intra_nbr_ok(mb_x * 4 - 1, mb_y * 4)?;

        let mut c_recon_dc = [[0i32; 4]; 2];
        if cbp_chroma != 0 {
            for (c, slot) in c_recon_dc.iter_mut().enumerate() {
                let mut dc = [0i32; 4];
                decode_residual_block_into::<4, 4>(r, -1, &mut dc)?;
                #[allow(clippy::cast_possible_truncation)]
                let qpc_trunc = qpc;
                *slot = self.dequant_chroma_dc(&dc, qpc_trunc, 1 + c);
            }
        }
        let mut c_q_blocks = [[[0i32; 16]; 4]; 2];
        let mut ccoded = [0u8; 2];
        if cbp_chroma == 2 {
            self.chroma_cache_load(mb_x, mb_y)?;
            let w2 = self.mb_w * 2;
            for c in 0..2 {
                for &(bx, by) in &CHROMA_4X4_SCAN_XY {
                    let nc = self.chroma_nc_pred(c, bx, by);
                    let mut ac = [0i32; 16];
                    let total = decode_residual_block_into::<15, 16>(r, nc, &mut ac)?;
                    self.chroma_nnz_cache_set(c, bx, by, total);
                    if let Some(n) =
                        self.nnz_c[c & 1].get_mut((mb_y * 2 + by) * w2 + (mb_x * 2 + bx))
                    {
                        *n = total;
                    }
                    if total != 0 {
                        ccoded[c & 1] |= 1u8 << ((by * 2 + bx) & 3);
                        c_q_blocks[c & 1][(by * 2 + bx) & 3] = ac;
                    }
                }
            }
        }
        self.recon_chroma_blocks(
            mb_x,
            mb_y,
            chroma_mode,
            avail_top,
            avail_left,
            &c_q_blocks,
            ccoded,
            &c_recon_dc,
            qpc,
        )?;
        Ok(())
    }

    /// Decodifica un macrobloque intra según su `mb_type` CAVLC (`I_4x4`, `I_16x16` o `I_PCM`).
    pub(crate) fn decode_intra_mb(
        &mut self,
        r: &mut BitReader,
        mb_x: usize,
        mb_y: usize,
        mb_type: u32,
    ) -> Result<(), MbError> {
        *self
            .mb_kind
            .get_mut(mb_y * self.mb_w + mb_x)
            .ok_or(MbError::Inconsistent("mb_kind"))? = rusty_h264_common::deblock::MB_KIND_INTRA;
        if mb_type == 0 {
            if self.transform_8x8_mode && r.read_bit()? {
                self.decode_i8x8(r, mb_x, mb_y)?;
            } else {
                self.decode_i4x4(r, mb_x, mb_y)?;
            }
        } else if (1..=24).contains(&mb_type) {
            self.decode_i16(r, mb_x, mb_y, mb_type - 1)?;
        } else if mb_type == 25 {
            self.edc_flush()?;
            self.decode_ipcm(r, mb_x, mb_y)?;
            self.pcm_qp_pending = true; // PARCHE-CRUSTY (ARR-1)
        } else {
            return Err(MbError::Unsupported(
                "only I_4x4 / I_16x16 / I_PCM macroblocks",
            ));
        }
        let w4 = self.mb_w * 4;
        for lby in 0..4usize {
            let a = (mb_y * 4 + lby) * w4 + mb_x * 4;
            self.coded_y[a..a + 4].fill(true);
        }
        Ok(())
    }
}
