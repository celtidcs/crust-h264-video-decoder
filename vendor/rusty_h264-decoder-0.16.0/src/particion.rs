//! Particiones y formas de macrobloques inter B y P (H.264 §7.4.5, Tablas 7-14 y 7-18).
//!
//! Modela las direcciones de predicción (`BPred`), disposiciones geométricas
//! de particiones de macrobloque (16×16, 16×8, 8×16, 8×8) y subparticiones (8×4, 4×8, 4×4),
//! junto con rutinas especializadas de copia de planos por stride (`restride`).

/// Dirección de predicción para particiones de macrobloque B.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BPred {
    /// Predicción unidireccional utilizando la lista de referencia 0 (L0).
    L0,
    /// Predicción unidireccional utilizando la lista de referencia 1 (L1).
    L1,
    /// Predicción bidireccional utilizando ambas listas de referencia (L0 y L1).
    Bi,
}

impl BPred {
    /// Indica si esta dirección de predicción emplea la lista de referencia indicada (0 o 1).
    #[inline]
    #[must_use]
    pub const fn uses(self, list: usize) -> bool {
        matches!(
            (self, list),
            (BPred::L0 | BPred::Bi, 0) | (BPred::L1 | BPred::Bi, 1)
        )
    }
}

/// Región de partición rectangular `(x, y, w, h)` expresada en muestras luma.
pub type Region = (usize, usize, usize, usize);

pub(crate) const B16X16: &[(usize, usize, usize, usize)] = &[(0, 0, 16, 16)];
pub(crate) const B16X8: &[(usize, usize, usize, usize)] = &[(0, 0, 16, 8), (0, 8, 16, 8)];
pub(crate) const B8X16: &[(usize, usize, usize, usize)] = &[(0, 0, 8, 16), (8, 0, 8, 16)];

/// Decodifica la forma y modos de predicción de vectores de movimiento para `mb_type` en slices B (1..=21).
///
/// Devuelve `(mvmode, pred0, pred1)` donde `pred` se codifica como 1 = L0, 2 = L1, 3 = Bi.
/// Se utiliza también para verificar la coherencia exacta con las tablas del codificador.
#[must_use]
pub fn b_inter_shape(mb_type: u32) -> (u8, u8, u8) {
    let (_, mvmode, preds) = b_inter_layout(mb_type);
    let code = |p: BPred| match (p.uses(0), p.uses(1)) {
        (true, true) => 3,
        (true, false) => 1,
        _ => 2,
    };
    (mvmode, code(preds[0]), code(preds[1]))
}

/// Disposición geométrica y direcciones de predicción de un macrobloque B (H.264 Tabla 7-14).
#[inline]
#[must_use]
pub(crate) fn b_inter_layout(mb_type: u32) -> (&'static [Region], u8, [BPred; 2]) {
    use BPred::{Bi, L0, L1};
    match mb_type {
        1 => (B16X16, 0, [L0, L0]),
        2 => (B16X16, 0, [L1, L1]),
        3 => (B16X16, 0, [Bi, Bi]),
        4 => (B16X8, 1, [L0, L0]),
        5 => (B8X16, 2, [L0, L0]),
        6 => (B16X8, 1, [L1, L1]),
        7 => (B8X16, 2, [L1, L1]),
        8 => (B16X8, 1, [L0, L1]),
        9 => (B8X16, 2, [L0, L1]),
        10 => (B16X8, 1, [L1, L0]),
        11 => (B8X16, 2, [L1, L0]),
        12 => (B16X8, 1, [L0, Bi]),
        13 => (B8X16, 2, [L0, Bi]),
        14 => (B16X8, 1, [L1, Bi]),
        15 => (B8X16, 2, [L1, Bi]),
        16 => (B16X8, 1, [Bi, L0]),
        17 => (B8X16, 2, [Bi, L0]),
        18 => (B16X8, 1, [Bi, L1]),
        19 => (B8X16, 2, [Bi, L1]),
        20 => (B16X8, 1, [Bi, Bi]),
        _ => (B8X16, 2, [Bi, Bi]), // 21
    }
}

/// Indica si un `sub_mb_type` en slices B (1..=12) emplea la lista de referencia indicada.
#[inline]
#[must_use]
pub(crate) fn b_sub_uses(st: u32, list: usize) -> bool {
    let pred = match st {
        1 | 4 | 5 | 10 => 0, // L0
        2 | 6 | 7 | 11 => 1, // L1
        _ => 2,              // Bi (3, 8, 9, 12)
    };
    (list == 0 && pred != 1) || (list == 1 && pred != 0)
}

/// Formas de subpartición dentro de un bloque 8×8 para un `sub_mb_type` B (1..=12).
#[inline]
#[must_use]
pub(crate) fn b_sub_parts(st: u32) -> &'static [(usize, usize, usize, usize)] {
    match st {
        1..=3 => &[(0, 0, 8, 8)],
        4 | 6 | 8 => &[(0, 0, 8, 4), (0, 4, 8, 4)],
        5 | 7 | 9 => &[(0, 0, 4, 8), (4, 0, 4, 8)],
        _ => &[(0, 0, 4, 4), (4, 0, 4, 4), (0, 4, 4, 4), (4, 4, 4, 4)], // 10/11/12
    }
}

/// Disposición de particiones sub-macrobloque `(x, y, w, h)` dentro de 8×8 para slices P.
///
/// Codificación: 0 = 8×8, 1 = 8×4, 2 = 4×8, 3 = 4×4.
#[inline]
#[must_use]
pub(crate) fn sub_mb_partitions(sub_type: u32) -> &'static [(usize, usize, usize, usize)] {
    match sub_type {
        0 => &[(0, 0, 8, 8)],
        1 => &[(0, 0, 8, 4), (0, 4, 8, 4)],
        2 => &[(0, 0, 4, 8), (4, 0, 4, 8)],
        _ => &[(0, 0, 4, 4), (4, 0, 4, 4), (0, 4, 4, 4), (4, 4, 4, 4)],
    }
}

/// Copia un bloque rectangular contiguo de dimensiones `w × h` a un destino con salto de línea `dst_stride`.
///
/// Especializa las anchuras comunes (16, 8, 4, 2) para maximizar la vectorización y evitar saltos condicionales.
#[inline]
pub(crate) fn restride(
    dst: &mut [u8],
    dst_stride: usize,
    x0: usize,
    y0: usize,
    src: &[u8],
    w: usize,
    h: usize,
) {
    macro_rules! rows {
        ($n:expr) => {{
            for dy in 0..h {
                dst[(y0 + dy) * dst_stride + x0..][..$n].copy_from_slice(&src[dy * $n..][..$n]);
            }
        }};
    }
    match w {
        16 => rows!(16),
        8 => rows!(8),
        4 => rows!(4),
        2 => rows!(2),
        _ => {
            for dy in 0..h {
                dst[(y0 + dy) * dst_stride + x0..][..w].copy_from_slice(&src[dy * w..][..w]);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bpred_uses_listas() {
        assert!(BPred::L0.uses(0));
        assert!(!BPred::L0.uses(1));
        assert!(!BPred::L1.uses(0));
        assert!(BPred::L1.uses(1));
        assert!(BPred::Bi.uses(0));
        assert!(BPred::Bi.uses(1));
        assert!(!BPred::Bi.uses(2));
    }

    #[test]
    fn formas_b_inter_layout_consistentes() {
        let (r16, mode, preds) = b_inter_layout(1);
        assert_eq!(r16.len(), 1);
        assert_eq!(mode, 0);
        assert_eq!(preds, [BPred::L0, BPred::L0]);

        let (r16x8, mode1, _) = b_inter_layout(4);
        assert_eq!(r16x8.len(), 2);
        assert_eq!(mode1, 1);

        let (r8x16, mode2, _) = b_inter_layout(5);
        assert_eq!(r8x16.len(), 2);
        assert_eq!(mode2, 2);
    }

    #[test]
    fn subparticiones_p_cubren_64_muestras() {
        for st in 0..=3 {
            let parts = sub_mb_partitions(st);
            let mut total_samples = 0;
            for &(_, _, w, h) in parts {
                total_samples += w * h;
            }
            assert_eq!(total_samples, 64);
        }
    }

    #[test]
    fn subparticiones_b_cubren_64_muestras() {
        for st in 1..=12 {
            let parts = b_sub_parts(st);
            let mut total_samples = 0;
            for &(_, _, w, h) in parts {
                total_samples += w * h;
            }
            assert_eq!(total_samples, 64);
        }
    }

    #[test]
    fn restride_copia_correctamente() {
        let src = [1u8, 2, 3, 4, 5, 6, 7, 8];
        let mut dst = [0u8; 16];
        restride(&mut dst, 4, 0, 0, &src, 4, 2);
        assert_eq!(&dst[..8], &[1, 2, 3, 4, 5, 6, 7, 8]);
    }
}
