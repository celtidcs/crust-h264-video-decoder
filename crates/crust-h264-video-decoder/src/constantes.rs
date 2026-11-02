//! Constantes y tablas canónicas de macrobloques según la norma UIT-T H.264.
//!
//! Centraliza los parámetros dimensionales de macrobloques (16×16), bloques (8×8),
//! sub-bloques (4×4), el rango de cuantización QP y la tabla de escaneo en zig-zag 8×8
//! (proceso de escaneo inverso según UIT-T H.264 §8.5.6), eliminando números mágicos según REF-4 y §4.2.

/// Dimensión en píxeles de un macrobloque estándar (UIT-T H.264 §6.4.1).
pub const MB_DIM: usize = 16;

/// Muestras de luma en un macrobloque 16×16, derivadas de [`MB_DIM`].
pub const MB_LUMA_MUESTRAS: usize = MB_DIM * MB_DIM;

/// Dimensión de un bloque de transformada 8×8 (UIT-T H.264 §8.5.6).
pub const BLOQUE_8X8_DIM: usize = 8;

/// Total de coeficientes en un bloque 8×8, derivado de [`BLOQUE_8X8_DIM`].
pub const BLOQUE_8X8_COEFS: usize = BLOQUE_8X8_DIM * BLOQUE_8X8_DIM;

/// Muestras de croma por componente en un macrobloque 4:2:0 (8×8), derivadas de [`BLOQUE_8X8_COEFS`].
pub const MB_CROMA_MUESTRAS: usize = BLOQUE_8X8_COEFS;

/// Dimensión de un sub-bloque de transformada 4×4.
pub const SUBBLOQUE_4X4_DIM: usize = 4;

/// Total de coeficientes en un bloque 4×4, derivado de [`SUBBLOQUE_4X4_DIM`].
pub const SUBBLOQUE_4X4_COEFS: usize = SUBBLOQUE_4X4_DIM * SUBBLOQUE_4X4_DIM;

/// Límite inferior del parámetro de cuantización QP para vídeo de 8 bits (UIT-T H.264 §7.4.3).
pub const QP_MIN: i32 = 0;

/// Límite superior del parámetro de cuantización QP para vídeo de 8 bits (UIT-T H.264 §7.4.3).
pub const QP_MAX: u8 = 51;

/// Rango módulo para la acumulación de delta QP (UIT-T H.264 §7.4.5), derivado de [`QP_MAX`].
pub const QP_RANGO: i32 = (QP_MAX as i32) + 1;

/// Desplazamiento base sumado a `pic_init_qp_minus26` en el PPS (UIT-T H.264 §7.4.2.2).
pub const QP_BASE_PPS: i32 = 26;

/// Valor de muestra neutra en vídeo de 8 bits por componente (`2^(BitDepth - 1)`, UIT-T H.264 §8.3).
pub const MUESTRA_NEUTRA_8BIT: u8 = 128;

/// Valor máximo de muestra en vídeo de 8 bits por componente (`(1 << BitDepth) - 1`, UIT-T H.264 §5.7 `Clip1Y`).
pub const MUESTRA_MAX_8BIT: i32 = 255;

/// Centinela en las cachés `nnz_l_cache` y `nnz_c_cache` que marca un bloque vecino como no disponible.
///
/// Se elige `0x80` (128) porque cualquier conteo válido de coeficientes en un sub-bloque 4×4 está en `0..=16`,
/// permitiendo comprobar disponibilidad de ambos vecinos sin ramificaciones (`left + top < 0x80`).
pub const NNZ_NO_DISPONIBLE: u8 = 0x80;

/// Máscara para extraer el conteo válido de coeficientes del único vecino disponible cuando el otro vale [`NNZ_NO_DISPONIBLE`].
pub const NNZ_MASCARA_VECINO_UNICO: i32 = (NNZ_NO_DISPONIBLE as i32) - 1;

/// Centinela de categoría de macrobloque vecino no disponible en la rejilla CABAC `sc_cat`.
pub const CATEGORIA_MB_NO_DISPONIBLE: u8 = 255;

/// Valor inicial del registro de rango aritmético `codIRange` al iniciar o reiniciar el motor CABAC (UIT-T H.264 §9.3.1.2).
pub const CABAC_RANGO_INICIAL: u32 = 510;

/// Rango mínimo de `codIRange` tras la renormalización aritmética `RenormD` (UIT-T H.264 §9.3.3.2.2).
pub const CABAC_RANGO_MIN_RENORM: u32 = 256;

/// Bits iniciales leídos del flujo para poblar `codIOffset` al arrancar el motor CABAC (UIT-T H.264 §9.3.1.2).
pub const CABAC_BITS_OFFSET_INICIAL: u8 = 9;

/// Total de modelos de contexto adaptativos definidos por la norma CABAC (UIT-T H.264 Tabla 9-11).
pub const CABAC_MODELOS_NORMA: usize = 460;

/// Capacidad del vector de contextos CABAC redondeada a potencia de dos para indexado con máscara `& 511`.
pub const CABAC_NUM_CONTEXTOS: usize = 512;

/// Máscara de índice de contexto CABAC (`CABAC_NUM_CONTEXTOS - 1`).
pub const CABAC_MASCARA_CONTEXTO: usize = CABAC_NUM_CONTEXTOS - 1;

/// Número de estados empaquetados (`state * 2 + mps`, `0..128`) en la tabla de transiciones CABAC.
pub const CABAC_ESTADOS_EMPAQUETADOS: usize = 128;

/// Número máximo de operaciones `memory_management_control_operation` admitidas en una cabecera de corte.
pub const MAX_OPERACIONES_MMCO: usize = 128;

/// Capacidad máxima de planos Y/U/V retenidos en `plane_pool` para reutilización de memoria (6 fotogramas × 3 planos).
pub const MAX_PLANOS_POOL: usize = 18;

/// Tabla de orden inverso de escaneo en zig-zag 8×8 para tramas progresivas (UIT-T H.264 §8.5.6).
///
/// Nota documental de fuentes (§2.5): el orden de escaneo 8×8 está especificado en el proceso §8.5.6
/// de la norma UIT-T H.264 (en distintas ediciones de la norma se referencia como Tabla 8-13 o Tabla 8-14;
/// la numeración unívoca de la tabla entre ediciones queda registrada como: **No comprobado**).
pub const ZIGZAG_8X8: [usize; BLOQUE_8X8_COEFS] = [
    0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5, 12, 19, 26, 33, 40, 48, 41, 34, 27, 20,
    13, 6, 7, 14, 21, 28, 35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51, 58, 59,
    52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
];

/// Reordena los coeficientes de un bloque 8×8 desde orden zig-zag a raster (UIT-T H.264 §8.5.6).
#[must_use]
#[inline]
pub fn desescanear_8x8(scan: &[i32; BLOQUE_8X8_COEFS]) -> [i32; BLOQUE_8X8_COEFS] {
    let mut out = [0i32; BLOQUE_8X8_COEFS];
    for k in 0..BLOQUE_8X8_COEFS {
        out[ZIGZAG_8X8[k]] = scan[k];
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn desescanear_8x8_posicion_cero_permanece_en_cero() {
        let mut scan = [0i32; 64];
        scan[0] = 42;
        let raster = desescanear_8x8(&scan);
        assert_eq!(raster[0], 42);
    }

    #[test]
    fn desescanear_8x8_posicion_final_permanece_al_final() {
        let mut scan = [0i32; 64];
        scan[63] = 99;
        let raster = desescanear_8x8(&scan);
        assert_eq!(raster[63], 99);
    }

    #[test]
    fn desescanear_8x8_recorre_todas_las_posiciones_exactas() {
        let mut scan = [0i32; 64];
        for (i, v) in scan.iter_mut().enumerate() {
            *v = i32::try_from(i).unwrap_or(0);
        }
        let raster = desescanear_8x8(&scan);
        for k in 0..64 {
            let esperado = i32::try_from(k).unwrap_or(0);
            assert_eq!(raster[ZIGZAG_8X8[k]], esperado);
        }
    }
}
