//! Reconstrucción de macrobloques intra luma y croma (§8.3 y §8.5).

use crust_h264_common::predict::{
    add_residual_8x8, chroma8x8_pred, intra4x4_pred, intra8x8_pred, luma16x16_pred,
    reconstruct_4x4_dc_into, reconstruct_4x4_scan_into, I16Mode, CHROMA_4X4_SCAN_XY,
};
use crust_h264_common::transform::DequantQp;

use super::I4Res;
use crate::constantes::desescanear_8x8 as un_scan_8x8;
use crate::edc::{ZERO_CAC, ZERO_LUMA_SCAN};
use crate::error::MbError;
use crate::mb16::FrameDecoder;

impl FrameDecoder {
    /// Reconstruye un subbloque luma intra 4×4 aplicando la predicción y el residuo pre-procesado.
    ///
    /// # Errors
    /// `MbError::Inconsistent` si las tablas de vecinos no cubren el bloque (ver `gather_i4`).
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn recon_i4_block_res(
        &mut self,
        bx: usize,
        by: usize,
        mode: u8,
        at: bool,
        al: bool,
        kind: I4Res,
        scans: &[[i32; 16]; 16],
        dq: &DequantQp,
    ) -> Result<(), MbError> {
        let (px, py) = (bx * 4, by * 4);
        let (t, l, corner) = self.gather_i4(px, py, at, al, bx, by)?;
        let pred = intra4x4_pred(mode, at, al, &t, &l, corner);
        let r_off = py * self.cw + px;
        let cw = self.cw;
        match kind {
            I4Res::Zero => {
                let win = &mut self.rec_y[r_off..r_off + 3 * cw + 4];
                for r in 0..4 {
                    win[r * cw..r * cw + 4].copy_from_slice(&pred[r * 4..r * 4 + 4]);
                }
            }
            I4Res::Flat(v) => reconstruct_4x4_dc_into(v, &pred, 0, 4, &mut self.rec_y, r_off, cw),
            I4Res::Idx(i) => reconstruct_4x4_scan_into::<false>(
                &scans[i as usize & 15],
                dq,
                0,
                &pred,
                0,
                4,
                &mut self.rec_y,
                r_off,
                cw,
            ),
        }
        Ok(())
    }

    /// Reconstruye un bloque luma intra 8×8 (High profile).
    ///
    /// # Errors
    /// `MbError::Inconsistent` si las tablas de vecinos no cubren el bloque (ver `gather_i8`).
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn recon_i8_block(
        &mut self,
        bx: usize,
        by: usize,
        mode: u8,
        avail_top: bool,
        avail_left: bool,
        coeffs: Option<&[i32; 64]>,
        qp: u8,
    ) -> Result<(), MbError> {
        let (px, py) = (bx * 4, by * 4);
        let (t, l, corner, avail_corner) = self.gather_i8(px, py, avail_top, avail_left, bx, by)?;
        let pred = intra8x8_pred(mode, avail_top, avail_left, avail_corner, &t, &l, corner);
        match coeffs {
            None => {
                let (cw, base) = (self.cw, py * self.cw + px);
                let win = &mut self.rec_y[base..base + 7 * cw + 8];
                for dy in 0..8 {
                    win[dy * cw..dy * cw + 8].copy_from_slice(&pred[dy * 8..dy * 8 + 8]);
                }
            }
            Some(scan8) => {
                let raster = un_scan_8x8(scan8);
                let res8 = self.inv_quant8(&raster, qp, 0);
                let predb: [i32; 64] = core::array::from_fn(|i| i32::from(pred[i]));
                let recon = add_residual_8x8(&res8, &predb);
                let (cw, base) = (self.cw, py * self.cw + px);
                let win = &mut self.rec_y[base..base + 7 * cw + 8];
                for dy in 0..8 {
                    win[dy * cw..dy * cw + 8].copy_from_slice(&recon[dy * 8..dy * 8 + 8]);
                }
            }
        }
        Ok(())
    }

    /// Reconstruye un macrobloque luma `I_16x16` completo (predicción + 16 bloques AC + DC Hadamard).
    #[allow(clippy::too_many_arguments, clippy::too_many_lines)]
    pub(crate) fn recon_i16_luma(
        &mut self,
        mbx: usize,
        mby: usize,
        pred_mode: I16Mode,
        top_ok: bool,
        left_ok: bool,
        q_blocks: Option<&[[i32; 16]; 16]>,
        coded: u16,
        recon_dc: &[i32; 16],
        qp: u8,
    ) -> Result<(), MbError> {
        let q_blocks = q_blocks.unwrap_or(&ZERO_LUMA_SCAN);
        let w4 = self.mb_w * 4;
        let (lx, ly) = (mbx * 16, mby * 16);
        let mut t16 = [0u8; 16];
        let mut l16 = [0u8; 16];
        if top_ok {
            t16.copy_from_slice(self.top_y_row(ly, lx, 16));
        }
        if left_ok {
            let (cw, base) = (self.cw, ly * self.cw + lx - 1);
            for (s, &v) in l16.iter_mut().zip(self.rec_y[base..].iter().step_by(cw)) {
                *s = v;
            }
        }
        let corner = if top_ok && left_ok {
            self.top_y_px(ly, lx - 1)?
        } else {
            0
        };
        let pred_l = luma16x16_pred(pred_mode, top_ok, left_ok, &t16, &l16, corner);
        let dq0 = self.dq_const(qp, 0);
        for by in 0..4 {
            for bx in 0..4 {
                let p_off = (by * 4) * 16 + bx * 4;
                let r_off = (ly + by * 4) * self.cw + lx + bx * 4;
                if coded & (1u16 << ((by & 3) * 4 + (bx & 3))) == 0 {
                    reconstruct_4x4_dc_into(
                        (recon_dc[by * 4 + bx] + 32) >> 6,
                        &pred_l,
                        p_off,
                        16,
                        &mut self.rec_y,
                        r_off,
                        self.cw,
                    );
                } else {
                    reconstruct_4x4_scan_into::<true>(
                        &q_blocks[(by & 3) * 4 + (bx & 3)],
                        &dq0,
                        recon_dc[by * 4 + bx],
                        &pred_l,
                        p_off,
                        16,
                        &mut self.rec_y,
                        r_off,
                        self.cw,
                    );
                }
            }
        }
        for by in 0..4 {
            let r = (mby * 4 + by) * w4 + mbx * 4;
            self.modes_y[r..r + 4].fill(2);
            self.coded_y[r..r + 4].fill(true);
        }
        Ok(())
    }

    /// Reconstruye los bloques de croma de 8×8 para ambos planos Cb y Cr.
    #[allow(clippy::too_many_arguments, clippy::too_many_lines)]
    pub(crate) fn recon_chroma_blocks(
        &mut self,
        mb_x: usize,
        mb_y: usize,
        chroma_mode: u8,
        avail_top: bool,
        avail_left: bool,
        qac: &[[[i32; 16]; 4]; 2],
        coded: [u8; 2],
        dc: &[[i32; 4]; 2],
        qpc: u8,
    ) -> Result<(), MbError> {
        let (cx, cy) = (mb_x * 8, mb_y * 8);
        let dqc = [self.dq_const(qpc, 1), self.dq_const(qpc, 2)];
        for c in 0..2 {
            let mut ctop = [0u8; 8];
            let mut cleft = [0u8; 8];
            let mut ccorner = 0u8;
            {
                let rec_c = if c == 0 { &self.rec_u } else { &self.rec_v };
                if avail_top {
                    ctop.copy_from_slice(self.top_c_row(c, cy, cx, 8));
                }
                if avail_left {
                    let cbase = cy * self.ccw + cx - 1;
                    for (o, &v) in cleft
                        .iter_mut()
                        .zip(rec_c[cbase..].iter().step_by(self.ccw))
                    {
                        *o = v;
                    }
                }
                if avail_top && avail_left {
                    ccorner = self.top_c_px(c, cy, cx - 1)?;
                }
            }
            let pred8 = chroma8x8_pred(chroma_mode, avail_top, avail_left, &ctop, &cleft, ccorner);
            for &(bx, by) in &CHROMA_4X4_SCAN_XY {
                let p_off = (by * 4) * 8 + bx * 4;
                let ccw = self.ccw;
                let r_off = (cy + by * 4) * ccw + cx + bx * 4;
                if coded[c & 1] & (1u8 << ((by * 2 + bx) & 3)) == 0 {
                    let plane = if c == 0 {
                        &mut self.rec_u
                    } else {
                        &mut self.rec_v
                    };
                    reconstruct_4x4_dc_into(
                        (dc[c & 1][(by * 2 + bx) & 3] + 32) >> 6,
                        &pred8,
                        p_off,
                        8,
                        plane,
                        r_off,
                        ccw,
                    );
                } else {
                    let plane = if c == 0 {
                        &mut self.rec_u
                    } else {
                        &mut self.rec_v
                    };
                    reconstruct_4x4_scan_into::<true>(
                        &qac[c & 1][(by * 2 + bx) & 3],
                        &dqc[c & 1],
                        dc[c & 1][(by * 2 + bx) & 3],
                        &pred8,
                        p_off,
                        8,
                        plane,
                        r_off,
                        ccw,
                    );
                }
            }
        }
        Ok(())
    }

    /// Adaptador CABAC para la reconstrucción de los bloques de croma.
    #[allow(clippy::too_many_arguments, clippy::trivially_copy_pass_by_ref)]
    pub(crate) fn recon_chroma_cabac(
        &mut self,
        mb_x: usize,
        mb_y: usize,
        chroma_mode: u8,
        cdc: &[[i32; 4]; 2],
        cac: Option<&[[[i32; 16]; 4]; 2]>,
        cnnz: &[u8; 8],
        cbp_chroma: u32,
        avail_top: bool,
        avail_left: bool,
    ) -> Result<(), MbError> {
        let cac = cac.unwrap_or(&ZERO_CAC);
        let qpc = self.chroma_qp_for(self.cur_qp);
        let mut c_recon_dc = [[0i32; 4]; 2];
        if cbp_chroma != 0 {
            for c in 0..2 {
                #[allow(clippy::cast_possible_truncation)]
                let qpc_trunc = qpc;
                c_recon_dc[c] = self.dequant_chroma_dc(&cdc[c], qpc_trunc, 1 + c);
            }
        }
        let w2 = self.mb_w * 2;
        let mut ccoded = [0u8; 2];
        if cbp_chroma == 2 {
            for c in 0..2 {
                for &(bx, by) in &CHROMA_4X4_SCAN_XY {
                    let cnt = cnnz[(c * 4 + by * 2 + bx) & 7];
                    ccoded[c & 1] |= (u8::from(cnt != 0)) << ((by * 2 + bx) & 3);
                    if let Some(p) =
                        self.nnz_c[c & 1].get_mut((mb_y * 2 + by) * w2 + (mb_x * 2 + bx))
                    {
                        *p = cnt;
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
            cac,
            ccoded,
            &c_recon_dc,
            qpc,
        )?;
        Ok(())
    }
}
