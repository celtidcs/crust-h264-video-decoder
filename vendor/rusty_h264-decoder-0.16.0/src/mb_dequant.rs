//! Cuantización inversa y escalado de coeficientes (UIT-T H.264 §8.5).
//!
//! Este módulo implementa el cálculo de parámetros de cuantización de luma y croma,
//! el ajuste de QP por macrobloque (`step_qp`), la asignación de tablas de cuantización
//! inversa planas y ponderadas por matrices de escalado (High Profile), y la
//! transformación inversa y descuantización de bloques 8×8 y componentes DC.

use rusty_h264_common::predict::chroma_qp;
use rusty_h264_common::transform::{
    inverse_quant_8x8_dq, inverse_quant_chroma_dc, inverse_quant_chroma_dc_weighted, Dequant8Qp,
    DequantQp, DQ8_FLAT, DQ_FLAT,
};

use crate::constantes::{BLOQUE_8X8_COEFS, QP_MAX, QP_MIN, QP_RANGO};
use crate::error::MbError;
use crate::mb16::FrameDecoder;

impl FrameDecoder {
    /// Per-(qp, list) dequant constants for the fused scan-order kernel: the flat
    /// table at zero cost, or the scaling-list build once per macroblock.
    #[inline]
    pub(crate) fn dq_const(&self, qp: u8, list: usize) -> DequantQp {
        match &self.scaling {
            Some(s) => DequantQp::weighted(qp, &s[list]),
            None => DQ_FLAT[(qp as usize).min(QP_MAX as usize)],
        }
    }

    /// Single-coefficient twin of `dequant` for position 0 (DC-only fast path).
    pub(crate) fn dequant_dc4(&self, level: i32, qp: u8, list: usize) -> i32 {
        rusty_h264_common::transform::dequantize_dc4(
            level,
            qp,
            self.scaling.as_ref().map(|s| s[list][0]),
        )
    }

    /// Inverse-quantizes a chroma DC block with scaling list `list`'s DC weight.
    pub(crate) fn dequant_chroma_dc(&self, levels: &[i32; 4], qp: u8, list: usize) -> [i32; 4] {
        match &self.scaling {
            Some(s) => inverse_quant_chroma_dc_weighted(levels, qp, s[list][0]),
            None => inverse_quant_chroma_dc(levels, qp),
        }
    }

    /// Steps QPy by `mb_qp_delta` (spec §7.4.5 modulo). Called exactly for
    /// residual-coded macroblocks (mb_qp_delta presence ⇔ cbp != 0 or I_16x16),
    /// which makes it the one chokepoint for the transform-bypass refusal:
    /// with `qpprime_y_zero_transform_bypass_flag` set and QP'Y == 0 the
    /// residual is LOSSLESS-bypassed (no transform, no quant — and DPCM intra
    /// forms), which this decoder does not implement. Refusing here is loud
    /// and exact: all-PCM lossless streams (no mb_qp_delta) still decode.
    pub(crate) fn step_qp(&mut self, delta: i32) -> Result<(), MbError> {
        self.cur_qp = (self.cur_qp as i32 + delta + QP_RANGO).rem_euclid(QP_RANGO) as u8;
        if self.transform_bypass && self.cur_qp == 0 {
            return Err(MbError::Unsupported(
                "transform-bypass (lossless) macroblock",
            ));
        }
        Ok(())
    }

    /// Maps a luma QP to its chroma QP, applying `chroma_qp_index_offset`
    /// (spec §8.5.8): `QPc = qpc_table(Clip3(0, 51, QPy + offset))`.
    pub(crate) fn chroma_qp_for(&self, qp_y: u8) -> u8 {
        let qpi = (qp_y as i32 + self.chroma_qp_offset).clamp(QP_MIN, QP_MAX as i32) as u8;
        chroma_qp(qpi)
    }

    /// Dequantizes + inverse-transforms an 8×8 luma block, applying the scaling
    /// matrix `list` (0 = intra, 1 = inter) or flat weights.
    pub(crate) fn inv_quant8(
        &self,
        raster: &[i32; BLOQUE_8X8_COEFS],
        qp: u8,
        list: usize,
    ) -> [i32; BLOQUE_8X8_COEFS] {
        // Prebuilt per-(qp, list) constants: the flat table at zero cost, or one
        // 64-entry build per macroblock for scaling lists (was 64 x two lookups +
        // a multiply per coefficient on every block).
        match &self.scaling8 {
            Some(s) => inverse_quant_8x8_dq(raster, &Dequant8Qp::build(qp, &s[list])),
            None => inverse_quant_8x8_dq(raster, &DQ8_FLAT[(qp as usize).min(QP_MAX as usize)]),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chroma_qp_index_offset_aplica_y_limita() {
        let mut fd = FrameDecoder::new(1, 1, 26, 0, alloc::vec::Vec::new(), 0, false, false, false);
        assert_eq!(fd.chroma_qp_for(30), 29);

        fd.chroma_qp_offset = 2;
        assert_eq!(fd.chroma_qp_for(30), 31);

        fd.chroma_qp_offset = -12;
        assert_eq!(fd.chroma_qp_for(5), chroma_qp(0));

        fd.chroma_qp_offset = 99;
        assert_eq!(fd.chroma_qp_for(40), chroma_qp(51));
    }

    #[test]
    fn step_qp_acumula_modulo_52() {
        let mut fd = FrameDecoder::new(1, 1, 26, 0, alloc::vec::Vec::new(), 0, false, false, false);
        fd.step_qp(4).unwrap();
        assert_eq!(fd.cur_qp, 30);
        fd.step_qp(-10).unwrap();
        assert_eq!(fd.cur_qp, 20);
        // Desbordamiento circular sobre 52
        fd.step_qp(40).unwrap();
        assert_eq!(fd.cur_qp, 8);
        // Delta negativo envolviendo hacia atras
        fd.step_qp(-20).unwrap();
        assert_eq!(fd.cur_qp, 40);
    }

    #[test]
    fn step_qp_rechaza_transform_bypass() {
        let mut fd = FrameDecoder::new(1, 1, 0, 0, alloc::vec::Vec::new(), 0, false, false, false);
        fd.transform_bypass = true;
        assert!(matches!(fd.step_qp(0), Err(MbError::Unsupported(_))));
    }

    #[test]
    fn dq_const_y_inv_quant8_planos() {
        let fd = FrameDecoder::new(1, 1, 26, 0, alloc::vec::Vec::new(), 0, false, false, false);
        let _dq = fd.dq_const(26, 0);
        let raster = [0i32; 64];
        let res = fd.inv_quant8(&raster, 26, 0);
        assert_eq!(res, [0i32; 64]);
    }
}
