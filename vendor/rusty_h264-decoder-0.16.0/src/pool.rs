//! Fondo común de búferes y cuadrículas reciclables entre fotogramas (`GridPool`).
//!
//! Evita la sobrecarga recurrente de asignación dinámica de memoria (~1.65 MB
//! por fotograma en resolución estándar) y fallos de página suaves (*soft page faults*)
//! reutilizando las capacidades preasignadas a lo largo de la decodificación.

use crate::edc::{PInterJob, PInterNoResJob};
use alloc::boxed::Box;
use alloc::vec::Vec;

/// Búferes y cuadrículas temporales recicladas por imagen decodificada.
///
/// Los planos de reconstrucción no se incluyen aquí deliberadamente, ya que
/// se transfieren al consumidor final como salida del fotograma.
#[derive(Default)]
pub struct GridPool {
    /// Descriptores de trabajos diferidos retenidos entre fotogramas.
    /// Se almacenan en Box debido al gran tamaño de cada descriptor de macrobloque.
    #[allow(clippy::vec_box)]
    pub(crate) job_pool: Vec<Box<PInterJob>>,
    #[allow(clippy::vec_box)]
    pub(crate) nores_pool: Vec<Box<PInterNoResJob>>,
    /// Densidad estimada de bits por macrobloque para heurística de despacho E2.
    pub(crate) bits_per_mb: f64,
    pub(crate) mb_qp: Vec<u8>,
    pub(crate) bs_frame: Vec<rusty_h264_common::deblock::MbBs>,
    pub(crate) pk_prev: Vec<rusty_h264_common::deblock::MbPack>,
    pub(crate) pk_cur: Vec<rusty_h264_common::deblock::MbPack>,
    pub(crate) nnz_dbr: Vec<u8>,
    pub(crate) bak_y: Vec<u8>,
    pub(crate) bak_u: Vec<u8>,
    pub(crate) bak_v: Vec<u8>,
    pub(crate) nnz_y: Vec<u8>,
    pub(crate) nnz_c0: Vec<u8>,
    pub(crate) nnz_c1: Vec<u8>,
    pub(crate) modes_y: Vec<u8>,
    pub(crate) coded_y: Vec<bool>,
    pub(crate) mv_y: Vec<(i32, i32)>,
    pub(crate) inter_y: Vec<bool>,
    pub(crate) ref_idx_y: Vec<i32>,
    pub(crate) mv1: Vec<(i32, i32)>,
    pub(crate) ref_idx1: Vec<i32>,
    pub(crate) mb_t8x8: Vec<bool>,
    pub(crate) mb_kind: Vec<u8>,
    pub(crate) desplazamientos_mb: Vec<rusty_h264_common::deblock::DesplazamientosFiltro>,
    pub(crate) bzero: Vec<bool>,
    pub(crate) sc_cat: Vec<u8>,
    pub(crate) sc_cbp: Vec<u8>,
    pub(crate) sc_cmode: Vec<i32>,
    pub(crate) sc_nzc: Vec<[u8; 24]>,
    pub(crate) sc_cbfdc: Vec<u16>,
    pub(crate) sc_skip: Vec<bool>,
    pub(crate) sc_ref: Vec<[i8; 16]>,
    pub(crate) sc_mvd: Vec<[[i16; 2]; 16]>,
    pub(crate) sc_ref1: Vec<[i8; 16]>,
    pub(crate) sc_mvd1: Vec<[[i16; 2]; 16]>,
    pub(crate) sc_direct: Vec<bool>,
    pub(crate) ref_poc0: Vec<i32>,
    pub(crate) ref_poc1: Vec<i32>,
}

impl GridPool {
    /// Crea una nueva instancia con búferes vacíos.
    #[must_use]
    pub fn nuevo() -> Self {
        Self::default()
    }
}

/// Reutiliza la capacidad existente del vector `v` llenándolo con `n` copias de `val`.
///
/// Produce el mismo resultado observable que `vec![val; n]`, pero preserva la
/// memoria ya reservada en montículo si su capacidad es suficiente.
#[inline]
pub(crate) fn refill<T: Clone>(mut v: Vec<T>, n: usize, val: T) -> Vec<T> {
    v.clear();
    v.resize(n, val);
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn refill_preserva_capacidad_y_actualiza_elementos() {
        let mut v = Vec::with_capacity(16);
        v.push(99);
        let v2 = refill(v, 4, 7);
        assert_eq!(v2, alloc::vec![7, 7, 7, 7]);
        assert!(v2.capacity() >= 16);
    }

    #[test]
    fn grid_pool_nuevo_esta_inicializado_vacio() {
        let pool = GridPool::nuevo();
        assert_eq!(pool.bits_per_mb, 0.0);
        assert!(pool.mb_qp.is_empty());
        assert!(pool.nnz_y.is_empty());
    }
}
