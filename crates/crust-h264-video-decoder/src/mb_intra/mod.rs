//! Reconstrucción y predicción intra y croma de macrobloques.
//!
//! Este módulo contiene la lógica de decodificación y reconstrucción de macrobloques
//! intra (`I_4x4`, `I_8x8`, `I_16x16` e `I_PCM`) y de los planos de croma según la norma
//! UIT-T H.264 (§8.3 y §8.5), desacoplada del decodificador principal en submódulos
//! cohesivos: `prediccion`, `recon` y `cavlc`.

mod cavlc;
mod prediccion;
mod recon;

use crust_h264_common::predict::LUMA_4X4_SCAN_XY;

/// Clase de residuo de un subbloque luma intra 4×4 tras clasificación preliminar.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum I4Res {
    /// Sin residuo; la reconstrucción coincide exactamente con la predicción.
    Zero,
    /// Residuo constante (solo componente DC).
    Flat(i32),
    /// Índice del bloque residual completo dentro del arreglo procesado por el kernel fusionado.
    Idx(u8),
}

/// Índice en orden z del bloque 4×4 en posición raster `(x, y)` (inverso de [`LUMA_4X4_SCAN_XY`]).
pub const I4_Z_OF_XY: [[usize; 4]; 4] = {
    let mut t = [[0usize; 4]; 4];
    let mut i = 0;
    while i < 16 {
        let (x, y) = LUMA_4X4_SCAN_XY[i];
        t[y][x] = i;
        i += 1;
    }
    t
};

/// Máscara de bits donde el bit `i` indica si el vecino superior derecho del bloque `i` (en orden z)
/// reside dentro del mismo macrobloque y ya ha sido reconstruido (UIT-T H.264 §6.4.11.4).
pub const I4_TR_IN_MB: u16 = {
    let mut m = 0u16;
    let mut i = 0;
    while i < 16 {
        let (x, y) = LUMA_4X4_SCAN_XY[i];
        if y > 0 && x < 3 && I4_Z_OF_XY[y - 1][x + 1] < i {
            m |= 1 << i;
        }
        i += 1;
    }
    m
};

/// Modo de predicción intra 4x4 u 8x8 cuando el flujo no usa el predicho
/// (`prev_intra_pred_mode_flag` = 0): `rem_intra_pred_mode` se salta el modo predicho, así que
/// los restos iguales o mayores que él valen uno más (§8.3.1.1, ecuación 8-42; §8.3.2.1 en 8x8).
pub(crate) fn modo_intra_desde_rem(predicho: u8, rem: u8) -> u8 {
    if rem < predicho {
        rem
    } else {
        rem + 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn z_order_inverso_coincide_con_luma_scan_xy() {
        for (i, &(x, y)) in LUMA_4X4_SCAN_XY.iter().enumerate() {
            assert_eq!(I4_Z_OF_XY[y][x], i);
        }
    }

    #[test]
    fn tr_in_mb_mascara_cumple_condiciones_de_borde() {
        // Para bloques en la fila superior (y == 0), el vecino superior derecho está fuera del MB
        for (i, &(x, y)) in LUMA_4X4_SCAN_XY.iter().enumerate() {
            let bit = (I4_TR_IN_MB >> i) & 1;
            if y == 0 || x >= 3 {
                assert_eq!(bit, 0, "bloque en borde no debe tener TR interno");
            }
        }
    }
}
