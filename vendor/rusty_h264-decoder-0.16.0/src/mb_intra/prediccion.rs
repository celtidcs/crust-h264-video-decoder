//! Predicción intra 4×4 y 8×8: recolección de vecinos y modos más probables (§8.3.1, §8.3.2).

use super::{I4Res, I4_TR_IN_MB, I4_Z_OF_XY};
use crate::error::MbError;
use crate::mb16::FrameDecoder;

impl FrameDecoder {
    /// Clasifica y pre-descuantiza los bloques 4×4 para la reconstrucción intra.
    #[must_use]
    pub(crate) fn i4_prepare(
        &self,
        scans: &[[i32; 16]; 16],
        nnzs: &[u8; 16],
        qp: u8,
    ) -> [I4Res; 16] {
        let mut kinds = [I4Res::Zero; 16];
        for blk in 0..16 {
            let (nnz, scan) = (nnzs[blk], &scans[blk]);
            kinds[blk] = if nnz == 0 {
                I4Res::Zero
            } else if nnz == 1 && scan[0] != 0 {
                let f = self.dequant_dc4(scan[0], qp, 0);
                I4Res::Flat((f + 32) >> 6)
            } else {
                #[allow(clippy::cast_possible_truncation)]
                I4Res::Idx(blk as u8)
            };
        }
        kinds
    }

    /// Predice rápidamente el modo intra 4×4 comprobando disponibilidad local.
    ///
    /// # Errors
    /// `MbError::Inconsistent` si la tabla de modos no cubre un vecino disponible.
    #[inline]
    pub(crate) fn predict_i4_mode_fast(
        &self,
        bx: usize,
        by: usize,
        lbx: usize,
        lby: usize,
        top_ok: bool,
        left_ok: bool,
    ) -> Result<u8, MbError> {
        if self.constrained_intra {
            return self.predict_i4_mode(bx, by);
        }
        // Vecino A o B no disponible: dcPredModePredictedFlag = 1, modo 2 (Intra_4x4_DC),
        // UIT-T H.264 (08/2024) §8.3.1.1, pasos 2 y 3.
        if !((lbx > 0 || left_ok) && (lby > 0 || top_ok)) {
            return Ok(2);
        }
        let w4 = self.mb_w * 4;
        let l = self.modes_y.get(by * w4 + (bx - 1)).copied();
        let t = self.modes_y.get((by - 1) * w4 + bx).copied();
        match (l, t) {
            (Some(l), Some(t)) => Ok(l.min(t)),
            _ => Err(MbError::Inconsistent("modes_y")),
        }
    }

    /// Predice el modo intra 4×4 según vecinos izquierdo y superior (UIT-T H.264 §8.3.1.1).
    ///
    /// # Errors
    /// `MbError::Inconsistent` si la tabla de modos no cubre un vecino disponible.
    pub(crate) fn predict_i4_mode(&self, bx: usize, by: usize) -> Result<u8, MbError> {
        // Vecino no disponible (o inter con constrained_intra_pred_flag): modo 2 (Intra_4x4_DC),
        // §8.3.1.1, pasos 2 y 3.
        if bx == 0 || by == 0 {
            return Ok(2);
        }
        if !self.nbr_in_slice((bx - 1) / 4, by / 4)
            || !self.nbr_in_slice(bx / 4, (by - 1) / 4)
            || !self.intra_nbr_ok(bx - 1, by)?
            || !self.intra_nbr_ok(bx, by - 1)?
        {
            return Ok(2);
        }
        let w4 = self.mb_w * 4;
        let l = self.modes_y.get(by * w4 + (bx - 1)).copied();
        let t = self.modes_y.get((by - 1) * w4 + bx).copied();
        match (l, t) {
            (Some(l), Some(t)) => Ok(l.min(t)),
            _ => Err(MbError::Inconsistent("modes_y")),
        }
    }

    /// Recopila las muestras de referencia vecinas para un bloque luma intra 4×4.
    ///
    /// # Errors
    /// `MbError::Inconsistent` si `coded_y` no cubre el vecino superior derecho.
    pub(crate) fn gather_i4(
        &self,
        px: usize,
        py: usize,
        avail_top: bool,
        avail_left: bool,
        bx: usize,
        by: usize,
    ) -> Result<([u8; 8], [u8; 4], u8), MbError> {
        let (cw, w4) = (self.cw, self.mb_w * 4);
        let mut top = [0u8; 8];
        let mut left = [0u8; 4];
        let mut corner = 0;
        let (lbx, lby) = (bx & 3, by & 3);
        if avail_top {
            top[..4].copy_from_slice(self.top_y_row(py, px, 4));
            let tr_avail = if lby > 0 {
                lbx < 3 && (I4_TR_IN_MB >> I4_Z_OF_XY[lby][lbx]) & 1 == 1
            } else {
                bx + 1 < w4
                    && self
                        .coded_y
                        .get((by - 1) * w4 + (bx + 1))
                        .copied()
                        .ok_or(MbError::Inconsistent("coded_y"))?
                    && self.nbr_in_slice((bx + 1) / 4, (by - 1) / 4)
                    && self.intra_nbr_ok(bx + 1, by - 1)?
            };
            if tr_avail {
                top[4..8].copy_from_slice(self.top_y_row(py, px + 4, 4));
            } else {
                let t3 = top[3];
                top[4..8].fill(t3);
            }
        }
        if avail_left {
            let base = py * cw + px - 1;
            for (s, &v) in left.iter_mut().zip(self.rec_y[base..].iter().step_by(cw)) {
                *s = v;
            }
        }
        if avail_top && avail_left && ((lbx > 0 && lby > 0) || self.intra_nbr_ok(bx - 1, by - 1)?) {
            corner = self.top_y_px(py, px - 1)?;
        }
        Ok((top, left, corner))
    }

    /// Recopila las muestras de referencia vecinas para un bloque luma intra 8×8.
    ///
    /// Preserva estrictamente el PARCHE-WODW (ARR-2, hallado por Gemini): `p[-1,-1]` depende
    /// solo del bloque de la esquina, no de que exista bloque superior (§6.4.11.4, §8.3.2.2.1).
    ///
    /// # Errors
    /// `MbError::Inconsistent` si `coded_y` no cubre el vecino superior derecho.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn gather_i8(
        &self,
        px: usize,
        py: usize,
        avail_top: bool,
        avail_left: bool,
        bx: usize,
        by: usize,
    ) -> Result<([u8; 16], [u8; 8], u8, bool), MbError> {
        let (cw, w4) = (self.cw, self.mb_w * 4);
        let mut top = [0u8; 16];
        let mut left = [0u8; 8];
        let mut corner = 0;
        if avail_top {
            top[..8].copy_from_slice(self.top_y_row(py, px, 8));
            let tr_avail = bx + 2 < w4
                && self
                    .coded_y
                    .get((by - 1) * w4 + (bx + 2))
                    .copied()
                    .ok_or(MbError::Inconsistent("coded_y"))?
                && self.nbr_in_slice((bx + 2) / 4, (by - 1) / 4)
                && self.intra_nbr_ok(bx + 2, by - 1)?;
            if tr_avail {
                top[8..16].copy_from_slice(self.top_y_row(py, px + 8, 8));
            } else {
                let t7 = top[7];
                top[8..16].fill(t7);
            }
        }
        if avail_left {
            let base = py * cw + px - 1;
            for (s, &v) in left.iter_mut().zip(self.rec_y[base..].iter().step_by(cw)) {
                *s = v;
            }
        }
        let avail_corner = bx > 0
            && by > 0
            && self.nbr_in_slice((bx - 1) / 4, (by - 1) / 4)
            && self.intra_nbr_ok(bx - 1, by - 1)?;
        if avail_corner {
            corner = self.top_y_px(py, px - 1)?;
        }
        Ok((top, left, corner, avail_corner))
    }
}
