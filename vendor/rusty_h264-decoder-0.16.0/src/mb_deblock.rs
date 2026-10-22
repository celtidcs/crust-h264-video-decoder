//! Filtro de desbloqueo en bucle (*in-loop deblocking filter*) para macrobloques H.264.
//!
//! Este módulo contiene la lógica de derivación de fuerza de borde (`bS`),
//! filtrado entrelazado por filas (*row-interleaved deblocking*) y filtrado
//! completo al cierre de imagen (§8.7 de la norma UIT-T H.264), desacoplada
//! de `mb16.rs`.

#[allow(unused_imports)]
use alloc::format;
#[allow(unused_imports)]
use alloc::string::String;
#[allow(unused_imports)]
use alloc::vec::Vec;

use rusty_h264_common::deblock::{
    derive_mb_records, filter_frame_rows_pre, intra_mb_bs, pack_mb, BlockInfo, MbBs, MbKind,
};

use crate::corte::{corte_de, IdcFiltro};
use crate::error::MbError;
use crate::mb16::FrameDecoder;

impl FrameDecoder {
    /// Deriva `bS` para la fila de macrobloques `r` a partir de las cuadrículas recién
    /// decodificadas hacia `bs_frame`, manteniendo la ventana móvil de dos filas.
    #[allow(
        clippy::too_many_lines,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::needless_range_loop
    )]
    pub(crate) fn derive_bs_row(&mut self, r: usize) -> Result<(), MbError> {
        // La derivación de bS lee las cuadrículas de movimiento/nnz: vaciar tramos diferidos.
        self.span_flush()?;
        let (mb_w, w4) = (self.mb_w, self.mb_w * 4);

        if self.any_t8 {
            for br in r * 4..r * 4 + 4 {
                let a = br * w4;
                self.nnz_dbr[a..a + w4].copy_from_slice(&self.nnz_y[a..a + w4]);
            }
            let t8_row_pre = &self.mb_t8x8[r * mb_w..][..mb_w];
            for mb_x in 0..mb_w {
                if !t8_row_pre[mb_x] {
                    continue;
                }
                let base = r * 4 * w4 + mb_x * 4;
                let mut src = [[0u8; 4]; 4];
                for (k, row) in src.iter_mut().enumerate() {
                    row.copy_from_slice(&self.nnz_y[base + k * w4..][..4]);
                }
                let mut out = [[0u8; 4]; 4];
                for b8 in 0..4usize {
                    let (cx, cy) = ((b8 % 2) * 2, (b8 / 2) * 2);
                    let any =
                        (src[cy][cx] | src[cy][cx + 1] | src[cy + 1][cx] | src[cy + 1][cx + 1]) > 0;
                    for sy in 0..2 {
                        for sx in 0..2 {
                            out[cy + sy][cx + sx] = u8::from(any);
                        }
                    }
                }
                for (k, row) in out.iter().enumerate() {
                    self.nnz_dbr[base + k * w4..][..4].copy_from_slice(row);
                }
            }
        }
        let info = BlockInfo {
            inter: &self.inter_y,
            nnz: if self.any_t8 {
                &self.nnz_dbr
            } else {
                &self.nnz_y
            },
            mv: &self.mv_y,
            ref_id: &self.ref_idx_y,
            mv1: &self.mv1,
            ref_id1: if self.ref_poc1.is_empty() {
                &[]
            } else {
                &self.ref_idx1
            },
            w4,
            t8x8: &self.mb_t8x8,
            bs: &[],
            poc0: &self.ref_poc0,
            poc1: &self.ref_poc1,
            kind: &self.mb_kind,
            desplazamientos: &[],
        };
        let has1 = !info.ref_id1.is_empty();
        core::mem::swap(&mut self.pk_prev, &mut self.pk_cur);
        self.pk_cur.clear();
        let row0 = r * mb_w;
        let bs_row = &mut self.bs_frame[row0..][..mb_w];
        let t8_row = &self.mb_t8x8[row0..][..mb_w];
        let kind_row: &[u8] = if self.mb_kind.is_empty() {
            &[]
        } else {
            &self.mb_kind[row0..][..mb_w]
        };
        let cortes = &self.cortes;
        for mb_x in 0..mb_w {
            let addr = row0 + mb_x;
            // Cada macrobloque se traduce con los mapas de SU corte (COR-3) y se filtra con
            // los parámetros de SU corte (COR-4): la fila puede derivarse ya en otro corte.
            let corte = corte_de(cortes, addr)?;
            let datos = cortes.get(corte).ok_or(MbError::Inconsistent("cortes"))?;
            let rec = pack_mb(
                &BlockInfo {
                    poc0: &datos.poc0,
                    poc1: &datos.poc1,
                    ..info
                },
                has1,
                mb_x,
                r,
            )?;
            self.pk_cur.push(rec);
            match kind_row.get(mb_x).copied().and_then(MbKind::from_u8) {
                Some(MbKind::Intra) => {
                    bs_row[mb_x] = intra_mb_bs(mb_x, r);
                }
                _ => {
                    let cur = self
                        .pk_cur
                        .get(mb_x)
                        .ok_or(MbError::Inconsistent("pk_cur"))?;
                    let left = if mb_x > 0 {
                        self.pk_cur.get(mb_x - 1)
                    } else {
                        None
                    };
                    let top = if r > 0 { self.pk_prev.get(mb_x) } else { None };
                    let mb_t8 = t8_row[mb_x];
                    let (mut bv, mut bh) = ([[0i32; 4]; 4], [[0i32; 4]; 4]);
                    let flat = derive_mb_records(cur, left, top, mb_t8, &mut bv, &mut bh);
                    let m = if flat {
                        debug_assert!(bv[1..] == [[0i32; 4]; 3] && bh[1..] == [[0i32; 4]; 3]);
                        let mut m = MbBs::default();
                        m.v[0] = core::array::from_fn(|sg| bv[0][sg] as u8);
                        m.h[0] = core::array::from_fn(|sg| bh[0][sg] as u8);
                        m
                    } else {
                        MbBs {
                            v: core::array::from_fn(|e| core::array::from_fn(|sg| bv[e][sg] as u8)),
                            h: core::array::from_fn(|e| core::array::from_fn(|sg| bh[e][sg] as u8)),
                        }
                    };
                    bs_row[mb_x] = m;
                }
            }
            // §8.7: el idc del corte del macrobloque decide qué bordes suyos se filtran. Un
            // macrobloque sin filtro queda con bS 0 en todos sus bordes; la fila se filtra igual.
            match datos.filtro.idc {
                IdcFiltro::Activo => {}
                IdcFiltro::Desactivado => bs_row[mb_x] = MbBs::default(),
                IdcFiltro::SinBordesEntreCortes => {
                    if mb_x > 0 && corte_de(cortes, addr - 1)? != corte {
                        bs_row[mb_x].v[0] = [0; 4];
                    }
                    if r > 0 && corte_de(cortes, addr - mb_w)? != corte {
                        bs_row[mb_x].h[0] = [0; 4];
                    }
                }
            }
            *self
                .desplazamientos_mb
                .get_mut(addr)
                .ok_or(MbError::Inconsistent("desplazamientos_mb"))? = datos.filtro.desplazamientos;
        }
        Ok(())
    }

    /// Punto de enganche de desbloqueo en bucle con fila portada por el invocador.
    #[inline(always)]
    #[allow(clippy::inline_always)]
    pub(crate) fn row_hook_at(&mut self, addr: usize, mby: usize) -> Result<(), MbError> {
        if mby <= self.bs_rows {
            return Ok(());
        }
        self.row_hook(addr, mby)?;
        Ok(())
    }

    /// Punto de enganche invocado en cada cabecera de macrobloque.
    #[allow(clippy::too_many_lines)]
    pub(crate) fn row_hook(&mut self, addr: usize, mby: usize) -> Result<(), MbError> {
        debug_assert_eq!(mby, addr / self.mb_w, "row_hook: mby must be addr's row");
        let done = mby;
        if done <= self.bs_rows {
            return Ok(());
        }
        self.edc_flush()?;
        while self.bs_rows < done {
            let r = self.bs_rows;
            self.derive_bs_row(r)?;
            self.bs_rows += 1;
            self.save_bak(r);
            self.filter_row(r)?;
            self.flt_rows = r + 1;
        }
        Ok(())
    }

    /// Guarda las muestras inferiores no filtradas de la fila `r` antes de que el filtrado las modifique.
    pub(crate) fn save_bak(&mut self, r: usize) {
        let y0 = (r * 16 + 15) * self.cw;
        self.bak_y.copy_from_slice(&self.rec_y[y0..y0 + self.cw]);
        let c0 = (r * 8 + 7) * self.ccw;
        self.bak_u.copy_from_slice(&self.rec_u[c0..c0 + self.ccw]);
        self.bak_v.copy_from_slice(&self.rec_v[c0..c0 + self.ccw]);
    }

    /// Filtra una fila de macrobloques con los desplazamientos alfa/beta de cada macrobloque.
    pub(crate) fn filter_row(&mut self, r: usize) -> Result<(), MbError> {
        let info = BlockInfo {
            inter: &self.inter_y,
            nnz: if self.any_t8 {
                &self.nnz_dbr
            } else {
                &self.nnz_y
            },
            mv: &self.mv_y,
            ref_id: &self.ref_idx_y,
            mv1: &self.mv1,
            ref_id1: &self.ref_idx1,
            w4: self.mb_w * 4,
            t8x8: &self.mb_t8x8,
            bs: &self.bs_frame,
            poc0: &[],
            poc1: &[],
            kind: &self.mb_kind,
            desplazamientos: &self.desplazamientos_mb,
        };
        filter_frame_rows_pre(
            &mut self.rec_y,
            &mut self.rec_u,
            &mut self.rec_v,
            self.mb_w,
            self.mb_h,
            r..r + 1,
            &self.mb_qp,
            self.chroma_qp_offset,
            &info,
        )?;
        Ok(())
    }

    /// PARCHE-WODW (ARR-1): vacía las reconstrucciones diferidas y los trabajos de píxel
    /// pendientes al cerrar la imagen. Antes solo lo hacía `deblock`, así que con el filtro de
    /// desbloqueo desactivado (`disable_deblocking_filter_idc = 1`) los últimos macrobloques
    /// saltados de la imagen se quedaban sin reconstruir.
    pub fn flush_deferred(&mut self) -> Result<(), MbError> {
        self.span_flush()?;
        self.edc_flush()?;
        Ok(())
    }

    /// Aplica el filtro de desbloqueo en bucle a las filas de la imagen que aún no se han
    /// filtrado, cada macrobloque con los parámetros de su corte.
    ///
    /// # Errors
    /// Los de la derivación de `bS` y del filtro común.
    #[allow(clippy::too_many_lines)]
    pub fn deblock(&mut self) -> Result<(), MbError> {
        self.span_flush()?;
        self.edc_flush()?;
        while self.bs_rows < self.mb_h {
            let r = self.bs_rows;
            self.derive_bs_row(r)?;
            self.bs_rows += 1;
        }

        let mut info = BlockInfo {
            inter: &[],
            nnz: &[],
            mv: &[],
            ref_id: &[],
            mv1: &[],
            ref_id1: &[],
            w4: self.mb_w * 4,
            t8x8: &self.mb_t8x8,
            bs: &[],
            poc0: &[],
            poc1: &[],
            kind: &self.mb_kind,
            desplazamientos: &self.desplazamientos_mb,
        };
        let bs_store = core::mem::take(&mut self.bs_frame);
        info.bs = &bs_store;
        let first_row = self.flt_rows;
        filter_frame_rows_pre(
            &mut self.rec_y,
            &mut self.rec_u,
            &mut self.rec_v,
            self.mb_w,
            self.mb_h,
            first_row..self.mb_h,
            &self.mb_qp,
            self.chroma_qp_offset,
            &info,
        )?;
        self.bs_frame = bs_store;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mb_bs_estructura_inicializada_en_cero() {
        let bs = MbBs::default();
        assert_eq!(bs.v, [[0u8; 4]; 4]);
        assert_eq!(bs.h, [[0u8; 4]; 4]);
    }
}
