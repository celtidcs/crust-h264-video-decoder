//! Datos de cada corte (*slice*) de la imagen que se decodifica, en un único registro.
//!
//! El desbloqueo de una fila de macrobloques puede ejecutarse cuando la decodificación ya ha
//! entrado en un corte posterior, así que todo lo que el filtro necesita de un corte (su primer
//! macrobloque, sus mapas de referencias y sus parámetros de filtro) se guarda junto y se
//! consulta por el corte dueño de cada macrobloque, nunca por el corte en curso (COR-3, COR-4).

use alloc::vec::Vec;

use crust_h264_common::deblock::DesplazamientosFiltro;

use crate::error::MbError;

/// `disable_deblocking_filter_idc` de un corte (UIT-T H.264, §7.4.3 y §8.7).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IdcFiltro {
    /// 0: se filtran todos los bordes de los macrobloques del corte.
    Activo,
    /// 1: no se filtra ningún borde de los macrobloques del corte.
    Desactivado,
    /// 2: se filtran, salvo los bordes con un macrobloque de otro corte.
    SinBordesEntreCortes,
}

impl IdcFiltro {
    /// Interpreta el valor leído de la cabecera del corte; `None` si no es 0, 1 ni 2.
    #[must_use]
    pub fn desde_sintaxis(valor: u32) -> Option<Self> {
        match valor {
            0 => Some(Self::Activo),
            1 => Some(Self::Desactivado),
            2 => Some(Self::SinBordesEntreCortes),
            _ => None,
        }
    }
}

/// Parámetros del filtro de desbloqueo de un corte. Según §8.7 y §8.7.2 rigen todos los bordes
/// de los macrobloques del corte (los bordes izquierdo, superior e internos, donde está `q0`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FiltroCorte {
    /// Qué bordes se filtran.
    pub idc: IdcFiltro,
    /// `FilterOffsetA`/`FilterOffsetB`. Con [`IdcFiltro::Desactivado`] no hay bordes que
    /// filtrar y valen 0, como infiere la norma cuando no se transmiten.
    pub desplazamientos: DesplazamientosFiltro,
}

impl FiltroCorte {
    /// El filtro que se infiere cuando el PPS no transmite su control (§7.4.3): activo y sin
    /// desplazamientos.
    pub const POR_DEFECTO: Self = Self {
        idc: IdcFiltro::Activo,
        desplazamientos: DesplazamientosFiltro::CERO,
    };
}

/// Lo que el desbloqueo necesita de un corte ya empezado.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DatosCorte {
    /// Dirección del primer macrobloque del corte (`first_mb_in_slice`).
    pub primer_mb: usize,
    /// Parámetros del filtro del corte.
    pub filtro: FiltroCorte,
    /// Mapa índice de referencia de la lista 0 → identidad de imagen, con las listas del corte.
    pub poc0: Vec<i32>,
    /// Lo mismo para la lista 1 (vacío fuera de los cortes B).
    pub poc1: Vec<i32>,
}

/// Índice del corte dueño de `addr` en `cortes` (contiguos en orden de barrido, con su primer
/// macrobloque en orden ascendente). Función libre para poder usarla mientras otros campos del
/// decodificador están prestados.
///
/// # Errors
/// `MbError::Inconsistent` si `addr` es anterior al primer corte (o no hay cortes): una imagen
/// aceptada empieza siempre en el macrobloque 0.
pub(crate) fn corte_de(cortes: &[DatosCorte], addr: usize) -> Result<usize, MbError> {
    match cortes.binary_search_by(|c| c.primer_mb.cmp(&addr)) {
        Ok(i) => Ok(i),
        Err(i) => i.checked_sub(1).ok_or(MbError::Inconsistent("cortes")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn corte(primer_mb: usize) -> DatosCorte {
        DatosCorte {
            primer_mb,
            filtro: FiltroCorte::POR_DEFECTO,
            poc0: Vec::new(),
            poc1: Vec::new(),
        }
    }

    #[test]
    fn idc_solo_admite_los_tres_valores_de_la_norma() {
        assert_eq!(IdcFiltro::desde_sintaxis(0), Some(IdcFiltro::Activo));
        assert_eq!(IdcFiltro::desde_sintaxis(1), Some(IdcFiltro::Desactivado));
        assert_eq!(
            IdcFiltro::desde_sintaxis(2),
            Some(IdcFiltro::SinBordesEntreCortes)
        );
        assert_eq!(IdcFiltro::desde_sintaxis(3), None);
    }

    #[test]
    fn cada_macrobloque_pertenece_al_ultimo_corte_que_empieza_antes() {
        let cortes = [corte(0), corte(20), corte(40)];
        assert_eq!(corte_de(&cortes, 0), Ok(0));
        assert_eq!(corte_de(&cortes, 19), Ok(0));
        assert_eq!(corte_de(&cortes, 20), Ok(1));
        assert_eq!(corte_de(&cortes, 63), Ok(2));
        assert_eq!(
            corte_de(&[corte(5)], 4),
            Err(MbError::Inconsistent("cortes"))
        );
        assert_eq!(corte_de(&[], 0), Err(MbError::Inconsistent("cortes")));
    }
}
