//! Tipos y jerarquías de errores tipados del decodificador H.264.
//!
//! Centraliza los errores de decodificación a nivel de macrobloque (`MbError`)
//! y a nivel de secuencia/flujo (`DecodeError`), proporcionando conversiones
//! tipadas y mensajes descriptivos según las directrices de seguridad (§4.4).

use core::fmt;
use crusty_h264_common::bit_reader::OutOfData;

/// Errores durante la decodificación de una unidad de acceso o fotograma completo.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DecodeError {
    /// Fin prematuro o inesperado del flujo de bits.
    Truncated,
    /// Falta un conjunto de parámetros requerido (SPS o PPS) antes del slice.
    MissingParameterSet,
    /// Se encontró una herramienta o sintaxis no soportada en el perfil actual.
    Unsupported(&'static str),
    /// Un índice que la construcción del decodificador mantiene dentro de rango se salió de él:
    /// el estado interno ya no corresponde al flujo y se rechaza en vez de inventar un dato
    /// (§4.4). El texto nombra la tabla afectada.
    Inconsistent(&'static str),
}

impl From<OutOfData> for DecodeError {
    fn from(_: OutOfData) -> Self {
        DecodeError::Truncated
    }
}

impl From<crusty_h264_common::FueraDeRango> for MbError {
    fn from(err: crusty_h264_common::FueraDeRango) -> Self {
        MbError::Inconsistent(err.0)
    }
}

impl From<MbError> for DecodeError {
    fn from(err: MbError) -> Self {
        match err {
            MbError::Truncated => DecodeError::Truncated,
            MbError::Unsupported(s) => DecodeError::Unsupported(s),
            MbError::Inconsistent(s) => DecodeError::Inconsistent(s),
        }
    }
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DecodeError::Truncated => f.write_str("flujo de bits truncado o incompleto"),
            DecodeError::MissingParameterSet => {
                f.write_str("slice recibido antes del conjunto SPS/PPS necesario")
            }
            DecodeError::Unsupported(s) => {
                write!(f, "herramienta de codificación no soportada: {s}")
            }
            DecodeError::Inconsistent(s) => {
                write!(f, "estado interno incoherente con el flujo: {s}")
            }
        }
    }
}

impl core::error::Error for DecodeError {}

/// Causa por la cual un macrobloque individual no pudo ser decodificado.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MbError {
    /// Fin inesperado de datos durante el análisis del macrobloque.
    Truncated,
    /// El macrobloque emplea una característica fuera del subconjunto implementado.
    Unsupported(&'static str),
    /// Índice fuera de rango en una tabla que la construcción mantiene dentro de él
    /// (ver [`DecodeError::Inconsistent`]).
    Inconsistent(&'static str),
}

impl From<OutOfData> for MbError {
    fn from(_: OutOfData) -> Self {
        MbError::Truncated
    }
}

impl fmt::Display for MbError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MbError::Truncated => f.write_str("datos de macrobloque truncados"),
            MbError::Unsupported(s) => {
                write!(f, "característica de macrobloque no soportada: {s}")
            }
            MbError::Inconsistent(s) => {
                write!(f, "estado del macrobloque incoherente con el flujo: {s}")
            }
        }
    }
}

impl core::error::Error for MbError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conversion_de_outofdata_a_decode_error() {
        let err: DecodeError = OutOfData.into();
        assert_eq!(err, DecodeError::Truncated);
    }

    #[test]
    fn conversion_de_mberror_a_decode_error() {
        let err_mb = MbError::Truncated;
        let err_dec: DecodeError = err_mb.into();
        assert_eq!(err_dec, DecodeError::Truncated);

        let err_unsupported = MbError::Unsupported("test_feature");
        let err_dec_unsupp: DecodeError = err_unsupported.into();
        assert_eq!(err_dec_unsupp, DecodeError::Unsupported("test_feature"));
    }

    #[test]
    fn mensajes_de_error_formateados_correctamente() {
        let err = DecodeError::MissingParameterSet;
        let msg = alloc::format!("{err}");
        assert!(msg.contains("SPS/PPS"));
    }
}
