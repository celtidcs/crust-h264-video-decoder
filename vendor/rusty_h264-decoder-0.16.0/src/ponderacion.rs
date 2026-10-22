//! Predicción ponderada explícita de muestras según la norma UIT-T H.264 §8.4.2.3.2.
//!
//! Este módulo contiene la tabla de ponderaciones explícitas ([`TablaPonderacion`]) leída desde
//! `pred_weight_table` en las cabeceras de slice P y B, así como las funciones de aplicación
//! de pesos y desplazamientos a las particiones de compensación de movimiento en planos
//! de luma y croma.

use alloc::vec::Vec;

use crate::error::MbError;

/// Región rectangular de una partición de macrobloque en coordenadas de luma o croma.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RegionParticion {
    /// Posición X inicial en el macrobloque.
    pub x: usize,
    /// Posición Y inicial en el macrobloque.
    pub y: usize,
    /// Ancho de la partición (16, 8, 4 o 2).
    pub ancho: usize,
    /// Alto de la partición.
    pub alto: usize,
}

impl RegionParticion {
    /// Crea una nueva región de partición rectangular.
    #[must_use]
    pub const fn nueva(x: usize, y: usize, ancho: usize, alto: usize) -> Self {
        Self { x, y, ancho, alto }
    }
}

/// Parámetros de ponderación lineal según H.264 §8.4.2.3.2: factor de escala, desplazamiento y log2 denom.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ParametrosPonderacion {
    /// Factor multiplicativo de ponderación.
    pub peso: i32,
    /// Desplazamiento aditivo.
    pub offset: i32,
    /// Exponente base 2 del denominador.
    pub log2_denom: i32,
}

impl ParametrosPonderacion {
    /// Crea un conjunto de parámetros de ponderación.
    #[must_use]
    pub const fn nuevo(peso: i32, offset: i32, log2_denom: i32) -> Self {
        Self {
            peso,
            offset,
            log2_denom,
        }
    }
}

/// Tabla de pesos y desplazamientos para predicción ponderada explícita (UIT-T H.264 §8.4.2.3.2).
///
/// Contiene los factores de ponderación lineal `w` y desplazamientos aditivos `o` para cada
/// índice de referencia en las listas 0 y 1, tanto para luma como para croma (Cb y Cr).
#[derive(Clone, Default)]
pub struct TablaPonderacion {
    /// Exponente base 2 del denominador para muestras de luma (`luma_log2_weight_denom`).
    pub luma_log2_denom: i32,
    /// Exponente base 2 del denominador para muestras de croma (`chroma_log2_weight_denom`).
    pub chroma_log2_denom: i32,
    /// `[lista][ref_idx] = (peso, desplazamiento)` para luma.
    pub luma: [Vec<(i32, i32)>; 2],
    /// `[lista][ref_idx][componente] = (peso, desplazamiento)` para croma (0: Cb, 1: Cr).
    pub chroma: [Vec<[(i32, i32); 2]>; 2],
}

/// Alias histórico de retrocompatibilidad para [`TablaPonderacion`].
pub type WeightTable = TablaPonderacion;

impl TablaPonderacion {
    /// Indica si el índice 0 de la lista 0 contiene ponderaciones identidad.
    ///
    /// Identidad: `w == 1 << denom` y `o == 0` para luma y ambas componentes de croma.
    #[must_use]
    pub fn es_identidad_ref0(&self) -> bool {
        self.luma[0]
            .first()
            .is_some_and(|&(w, o)| w == 1 << self.luma_log2_denom && o == 0)
            && self.chroma[0].first().is_some_and(|c| {
                c.iter()
                    .all(|&(w, o)| w == 1 << self.chroma_log2_denom && o == 0)
            })
    }

    /// Indica si todas las referencias de la lista especificada son ponderaciones identidad.
    #[must_use]
    pub fn es_identidad_lista(&self, list: usize) -> bool {
        let idx = list & 1;
        self.luma[idx]
            .iter()
            .all(|&(w, o)| w == 1 << self.luma_log2_denom && o == 0)
            && self.chroma[idx].iter().all(|c| {
                c.iter()
                    .all(|&(w, o)| w == 1 << self.chroma_log2_denom && o == 0)
            })
    }

    /// Obtiene el par (peso, desplazamiento) de luma para la lista y referencia indicadas.
    ///
    /// La tabla trae una entrada por `ref_idx` activo, con los valores por defecto de la norma ya
    /// puestos al analizar `pred_weight_table` (UIT-T H.264 08/2024, §7.4.3.2).
    ///
    /// # Errors
    /// `MbError::Inconsistent` si `refi` no está en la tabla.
    #[inline]
    pub fn ponderacion_luma(&self, list: usize, refi: usize) -> Result<(i32, i32), MbError> {
        self.luma[list & 1]
            .get(refi)
            .copied()
            .ok_or(MbError::Inconsistent("pesos de luma"))
    }

    /// Obtiene el par (peso, desplazamiento) de croma para la lista, referencia y componente (0: Cb, 1: Cr).
    ///
    /// # Errors
    /// `MbError::Inconsistent` si `refi` no está en la tabla (ver [`Self::ponderacion_luma`]).
    #[inline]
    pub fn ponderacion_croma(
        &self,
        list: usize,
        refi: usize,
        cc: usize,
    ) -> Result<(i32, i32), MbError> {
        self.chroma[list & 1]
            .get(refi)
            .map(|c| c[cc & 1])
            .ok_or(MbError::Inconsistent("pesos de croma"))
    }

    /// Aplica ponderación explícita a una muestra de luma individual.
    ///
    /// # Errors
    /// `MbError::Inconsistent` si `refi` no está en la tabla.
    pub fn aplicar_muestra_luma(
        &self,
        sample: u8,
        list: usize,
        refi: usize,
    ) -> Result<u8, MbError> {
        let (w, o) = self.ponderacion_luma(list, refi)?;
        Ok(ponderar_muestra(sample, w, o, self.luma_log2_denom))
    }

    /// Aplica ponderación explícita a una muestra de croma individual.
    ///
    /// # Errors
    /// `MbError::Inconsistent` si `refi` no está en la tabla.
    pub fn aplicar_muestra_croma(
        &self,
        sample: u8,
        list: usize,
        refi: usize,
        cc: usize,
    ) -> Result<u8, MbError> {
        let (w, o) = self.ponderacion_croma(list, refi, cc)?;
        Ok(ponderar_muestra(sample, w, o, self.chroma_log2_denom))
    }

    /// Aplica ponderación explícita a una partición rectangular de predicción de luma y croma.
    ///
    /// # Errors
    /// `MbError::Inconsistent` si `refi` no está en la tabla o la región se sale del bloque.
    pub fn aplicar_particion(
        &self,
        pred_y: &mut [u8; 256],
        c_pred: &mut [[u8; 64]; 2],
        list: usize,
        refi: usize,
        reg: RegionParticion,
    ) -> Result<(), MbError> {
        let (lw, lo) = self.ponderacion_luma(list, refi)?;
        let params_luma = ParametrosPonderacion::nuevo(lw, lo, self.luma_log2_denom);
        ponderar_bloque(pred_y, 16, reg, params_luma)?;

        let reg_chroma = RegionParticion::nueva(reg.x / 2, reg.y / 2, reg.ancho / 2, reg.alto / 2);
        for cc in 0..2 {
            let (cw, co) = self.ponderacion_croma(list, refi, cc)?;
            let params_chroma = ParametrosPonderacion::nuevo(cw, co, self.chroma_log2_denom);
            ponderar_bloque(&mut c_pred[cc & 1], 8, reg_chroma, params_chroma)?;
        }
        Ok(())
    }
}

/// Calcula la muestra ponderada según H.264 §8.4.2.3.2: `Clip1(((sample * w + 2^(denom-1)) >> denom) + o)`.
#[must_use]
#[inline]
pub fn ponderar_muestra(sample: u8, w: i32, o: i32, denom: i32) -> u8 {
    let s = i32::from(sample);
    let v = if denom >= 1 {
        ((s * w + (1 << (denom - 1))) >> denom) + o
    } else {
        s * w + o
    };
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    {
        v.clamp(0, crate::constantes::MUESTRA_MAX_8BIT) as u8
    }
}

/// Aplica la ponderación fila por fila con ancho constante conocido en tiempo de compilación.
#[inline]
pub fn ponderar_filas<const W: usize>(
    plane: &mut [u8],
    stride: usize,
    reg: RegionParticion,
    params: ParametrosPonderacion,
) {
    for dy in 0..reg.alto {
        let row = &mut plane[(reg.y + dy) * stride + reg.x..][..W];
        for s in row.iter_mut() {
            *s = ponderar_muestra(*s, params.peso, params.offset, params.log2_denom);
        }
    }
}

/// Aplica ponderación sobre un bloque rectangular dentro de un plano continuo.
///
/// # Errors
/// `MbError::Inconsistent` si, con un ancho no previsto, la región se sale del plano.
pub fn ponderar_bloque(
    plane: &mut [u8],
    stride: usize,
    reg: RegionParticion,
    params: ParametrosPonderacion,
) -> Result<(), MbError> {
    match reg.ancho {
        16 => ponderar_filas::<16>(plane, stride, reg, params),
        8 => ponderar_filas::<8>(plane, stride, reg, params),
        4 => ponderar_filas::<4>(plane, stride, reg, params),
        2 => ponderar_filas::<2>(plane, stride, reg, params),
        _ => {
            for dy in 0..reg.alto {
                for dx in 0..reg.ancho {
                    let i = (reg.y + dy) * stride + reg.x + dx;
                    let s = plane.get_mut(i).ok_or(MbError::Inconsistent("plane"))?;
                    *s = ponderar_muestra(*s, params.peso, params.offset, params.log2_denom);
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ponderacion_identidad_no_muta_muestra() {
        let denom = 5;
        let w = 1 << denom; // 32
        let o = 0;
        for s in [0u8, 128u8, 255u8] {
            assert_eq!(ponderar_muestra(s, w, o, denom), s);
        }
    }

    #[test]
    fn ponderacion_con_offset_y_escalado() {
        let denom = 5;
        let w = 16; // mitad
        let o = 10;
        // (100 * 16 + 16) >> 5 = (1616 >> 5) = 50. 50 + 10 = 60.
        assert_eq!(ponderar_muestra(100, w, o, denom), 60);
    }
}
