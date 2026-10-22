//! Compensación de movimiento y reconstrucción inter (P y B) de macrobloques.
//!
//! Este módulo contiene la lógica de estimación y recopilación de vectores
//! de movimiento, modos Directo B, compensación de movimiento B y P,
//! tramos de bandas de salto (*skip spans*) y decodificación de macrobloques
//! inter (§8.4 de la norma UIT-T H.264), desacoplada de mb16.rs.

use rusty_h264_common::BitReader;

use crate::error::MbError;
use crate::mb16::FrameDecoder;

/// Cota de |`mvd`| en cuartos de muestra: ningún perfil ni nivel permite vectores cerca de ±2^17,
/// así que un valor mayor es un flujo corrupto (y la suma pmv + mvd podría desbordar `i32`).
const MAX_ABS_MVD: u32 = 1 << 17;
/// Primer `mb_type` intra de un corte B (tabla 7-14: 23 = `I_NxN` en la numeración del corte I).
pub(crate) const MB_TYPE_B_PRIMER_INTRA: u32 = 23;
/// `mb_type` de `B_8x8` (tabla 7-14).
pub(crate) const MB_TYPE_B_8X8: u32 = 22;
/// Mayor `sub_mb_type` de un `B_8x8` (tabla 7-18).
pub(crate) const MAX_SUB_MB_TYPE_B: u32 = 12;
/// Mayor `sub_mb_type` de un `P_8x8` (tabla 7-17).
pub(crate) const MAX_SUB_MB_TYPE_P: u32 = 3;
/// Mayor índice de referencia que cabe en un trabajo EDC (`u8` por bloque, lista de 16).
pub(crate) const MAX_REF_TRABAJO: i32 = 15;

/// CAVLC mvd_lX with a sanity bound. se(v) can legally code ±2^31-1, but
/// every profile/level caps |MV| far below ±2^17 quarter-pel units; beyond
/// that is a corrupt stream, and the unchecked pmv + mvd addition would
/// overflow i32 (debug panic / release wrap into an absurd vector).
pub(crate) fn read_mvd(r: &mut BitReader) -> Result<i32, MbError> {
    let v = r.read_se()?;
    if v.unsigned_abs() > MAX_ABS_MVD {
        return Err(MbError::Truncated);
    }
    Ok(v)
}

mod b_cavlc;
mod b_directo;
mod b_mc;
mod b_skip;
mod p_cavlc;
mod p_recon;
mod residuo;
mod vecinos;

pub(crate) use b_mc::{MovimientoBi, RegionB};
pub(crate) use residuo::ResiduoInter;

impl FrameDecoder {
    /// Copia la predicción de croma de un macrobloque (`c_pred`, 8x8 por componente) a los planos
    /// reconstruidos: la reconstrucción de un macrobloque sin residuo de croma.
    pub(crate) fn escribir_prediccion_croma(
        &mut self,
        (mb_x, mb_y): (usize, usize),
        c_pred: &[[u8; 64]; 2],
    ) {
        let ccw = self.ccw;
        let cbase = (mb_y * 8) * ccw + mb_x * 8;
        for (c, cp) in c_pred.iter().enumerate() {
            let plane = if c == 0 {
                &mut self.rec_u
            } else {
                &mut self.rec_v
            };
            // One window per plane: its bound proves every row copy below.
            let cwin = &mut plane[cbase..cbase + 7 * ccw + 8];
            for dy in 0..8 {
                cwin[dy * ccw..dy * ccw + 8].copy_from_slice(&cp[dy * 8..dy * 8 + 8]);
            }
        }
    }

    /// Flush BOTH pending grid spans — the one call every grid reader makes.
    #[inline]
    pub(crate) fn span_flush(&mut self) -> Result<(), MbError> {
        self.bz_flush()?;
        self.pz_flush()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_mvd_rango_valido() {
        let data = [0x80u8];
        let mut r = BitReader::new(&data);
        assert_eq!(read_mvd(&mut r).unwrap(), 0);
    }

    #[test]
    fn read_mvd_cota_superada_da_error() {
        let data = [0x00, 0x00, 0x00, 0x01];
        let mut r = BitReader::new(&data);
        assert!(read_mvd(&mut r).is_err());
    }
}
