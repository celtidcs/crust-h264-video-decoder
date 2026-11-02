//! Recorte de ventanas de salida según la norma UIT-T H.264 §7.4.2.1.1.
//!
//! Este módulo contiene la definición de la ventana de recorte (`Recorte`) especificada
//! en el conjunto de parámetros de secuencia (SPS) y la lógica para aplicarla a los planos
//! de reconstrucción YUV 4:2:0 sin alterar las muestras originales decodificadas.

use alloc::vec;
use alloc::vec::Vec;
use crust_h264_common::YuvFrame;

/// Dimensiones en muestras de luma de un fotograma a recortar.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DimensionesFotograma {
    /// Ancho en muestras de luma.
    pub ancho: usize,
    /// Alto en muestras de luma.
    pub alto: usize,
}

/// Ventana de recorte del SPS en unidades de recorte (`frame_crop_*_offset`).
///
/// Según la norma UIT-T H.264 §7.4.2.1.1, en formato progresivo 4:2:0 cada unidad de recorte
/// vertical y horizontal equivale a 2 muestras de luma (1 muestra de croma).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Recorte {
    /// Desplazamiento desde el borde izquierdo en unidades de recorte.
    pub izquierda: usize,
    /// Desplazamiento desde el borde derecho en unidades de recorte.
    pub derecha: usize,
    /// Desplazamiento desde el borde superior en unidades de recorte.
    pub arriba: usize,
    /// Desplazamiento desde el borde inferior en unidades de recorte.
    pub abajo: usize,
}

impl Recorte {
    /// Aplica la ventana de recorte a los planos decodificados de luma y croma.
    ///
    /// Si no hay recorte activo (`Recorte::default()`), transfiere los vectores directamente
    /// para evitar asignaciones redundantes en memoria.
    #[must_use]
    pub fn aplicar(
        self,
        dims: DimensionesFotograma,
        rec_y: Vec<u8>,
        rec_u: Vec<u8>,
        rec_v: Vec<u8>,
    ) -> YuvFrame {
        let (cw, ch) = (dims.ancho, dims.alto);

        // Sin recorte (el caso más habitual): los planos de reconstrucción son la salida
        // final y se transfieren directamente sin clonar memoria.
        if self == Self::default() {
            return YuvFrame {
                width: cw,
                height: ch,
                y: rec_y,
                u: rec_u,
                v: rec_v,
            };
        }

        // El SPS ya rechazó un recorte que no cabe (params.rs), así que las restas no
        // pueden desbordar; los `saturating_sub` son una segunda barrera de seguridad.
        let (x0, y0) = (2 * self.izquierda, 2 * self.arriba);
        let dw = cw.saturating_sub(2 * (self.izquierda + self.derecha));
        let dh = ch.saturating_sub(2 * (self.arriba + self.abajo));
        let mut y = vec![0u8; dw * dh];
        for row in 0..dh {
            let origen = (y0 + row) * cw + x0;
            y[row * dw..row * dw + dw].copy_from_slice(&rec_y[origen..origen + dw]);
        }

        let ccw = cw / 2;
        let (cdw, cdh) = (dw / 2, dh / 2);
        let (cx0, cy0) = (self.izquierda, self.arriba);
        let mut u = vec![0u8; cdw * cdh];
        let mut v = vec![0u8; cdw * cdh];
        for row in 0..cdh {
            let origen = (cy0 + row) * ccw + cx0;
            u[row * cdw..row * cdw + cdw].copy_from_slice(&rec_u[origen..origen + cdw]);
            v[row * cdw..row * cdw + cdw].copy_from_slice(&rec_v[origen..origen + cdw]);
        }

        YuvFrame {
            width: dw,
            height: dh,
            y,
            u,
            v,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recorte_por_defecto_no_modifica_dimensiones() {
        let r = Recorte::default();
        let dims = DimensionesFotograma {
            ancho: 16,
            alto: 16,
        };
        let y = vec![10u8; 16 * 16];
        let u = vec![20u8; 8 * 8];
        let v = vec![30u8; 8 * 8];
        let f = r.aplicar(dims, y.clone(), u.clone(), v.clone());
        assert_eq!(f.width, 16);
        assert_eq!(f.height, 16);
        assert_eq!(f.y, y);
        assert_eq!(f.u, u);
        assert_eq!(f.v, v);
    }

    #[test]
    fn recorte_activo_extrae_subventana_correcta() {
        let r = Recorte {
            izquierda: 1,
            derecha: 1,
            arriba: 1,
            abajo: 1,
        };
        let dims = DimensionesFotograma {
            ancho: 16,
            alto: 16,
        };
        // 1 unidad = 2 muestras de luma. De 16x16 -> (16 - 4) x (16 - 4) = 12x12.
        let mut y = vec![0u8; 16 * 16];
        for row in 0..16 {
            for col in 0..16 {
                y[row * 16 + col] = (row * 16 + col) as u8;
            }
        }
        let u = vec![50u8; 8 * 8];
        let v = vec![60u8; 8 * 8];
        let f = r.aplicar(dims, y, u, v);
        assert_eq!(f.width, 12);
        assert_eq!(f.height, 12);
        // Primer píxel de luma debe ser (fila 2, col 2) = 2*16 + 2 = 34
        assert_eq!(f.y[0], 34);
    }
}
