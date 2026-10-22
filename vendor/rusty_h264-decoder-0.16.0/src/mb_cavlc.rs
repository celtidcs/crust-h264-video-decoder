//! AnÃ¡lisis sintÃ¡ctico CAVLC y control de cortes (UIT-T H.264 Â§9.2).
//!
//! Este mÃ³dulo implementa la decodificaciÃ³n de macrobloques y cortes codificados
//! con longitud variable adaptativa al contexto (CAVLC), incluyendo la gestiÃ³n
//! y predicciÃ³n de la cachÃ© de coeficientes no nulos (`nnz` y `nC`), el bucle
//! principal de corte CAVLC y el despacho multihilo con el trabajador EDC.
#![allow(clippy::needless_range_loop)]

use rusty_h264_common::BitReader;

use crate::constantes::{NNZ_MASCARA_VECINO_UNICO, NNZ_NO_DISPONIBLE};
use crate::corte::FiltroCorte;
use crate::edc::EdcJob;
use crate::error::MbError;
use crate::mb16::FrameDecoder;

impl FrameDecoder {
    pub(crate) fn nnz_cache_load(&mut self, mb_x: usize, mb_y: usize) -> Result<(), MbError> {
        let w4 = self.mb_w * 4;
        let top_unavail = mb_y == 0 || !self.nbr_in_slice(mb_x, mb_y - 1);
        let left_unavail = mb_x == 0 || !self.nbr_in_slice(mb_x - 1, mb_y);
        // The four TOP neighbours are one contiguous run of the nnz grid; only
        // the left column is strided. Hoisting the uniform `top_unavail` test out
        // of the loop turns four checked grid loads into one slice copy.
        if top_unavail {
            self.nnz_l_cache[1..5].fill(NNZ_NO_DISPONIBLE);
        } else {
            let src = &self.nnz_y[(mb_y * 4 - 1) * w4 + mb_x * 4..][..4];
            self.nnz_l_cache[1..5].copy_from_slice(src);
        }
        for lby in 0..4 {
            self.nnz_l_cache[(lby + 1) * 5] = if left_unavail {
                NNZ_NO_DISPONIBLE
            } else {
                self.nnz_y
                    .get((mb_y * 4 + lby) * w4 + (mb_x * 4 - 1))
                    .copied()
                    .ok_or(MbError::Inconsistent("nnz_y"))?
            };
        }
        Ok(())
    }

    #[inline]
    pub(crate) fn nc_pred(&self, lbx: usize, lby: usize) -> i32 {
        // MASKED, and it pays twice over: this helper and `nnz_cache_set` are
        // inlined into the intra path, the inter path AND both slice loops, so
        // the two unprovable indexes were replicated at every call site. The
        // cache is a 5x5 grid in a `[u8; 25]` and every caller passes a 4x4 block
        // coordinate (`LUMA_4X4_SCAN_XY`, `b8*2 + s`, or a literal), so `& 3` is
        // a no-op that puts the worst case at 4 * 5 + 4 = 24.
        let (lbx, lby) = (lbx & 3, lby & 3);
        let left = self.nnz_l_cache[(lby + 1) * 5 + lbx] as i32;
        let top = self.nnz_l_cache[lby * 5 + (lbx + 1)] as i32;
        let r = left + top;
        if r < i32::from(NNZ_NO_DISPONIBLE) {
            (r + 1) >> 1
        } else {
            r & NNZ_MASCARA_VECINO_UNICO
        }
    }

    #[inline]
    pub(crate) fn nnz_cache_set(&mut self, lbx: usize, lby: usize, total: u8) {
        self.nnz_l_cache[((lby & 3) + 1) * 5 + ((lbx & 3) + 1)] = total;
    }

    pub(crate) fn chroma_cache_load(&mut self, mb_x: usize, mb_y: usize) -> Result<(), MbError> {
        let w2 = self.mb_w * 2;
        let top_unavail = mb_y == 0 || !self.nbr_in_slice(mb_x, mb_y - 1);
        let left_unavail = mb_x == 0 || !self.nbr_in_slice(mb_x - 1, mb_y);
        for c in 0..2 {
            if top_unavail {
                self.nnz_c_cache[c][1..3].fill(NNZ_NO_DISPONIBLE);
            } else {
                let src = &self.nnz_c[c][(mb_y * 2 - 1) * w2 + mb_x * 2..][..2];
                self.nnz_c_cache[c][1..3].copy_from_slice(src);
            }
            for by in 0..2 {
                self.nnz_c_cache[c][(by + 1) * 3] = if left_unavail {
                    NNZ_NO_DISPONIBLE
                } else {
                    self.nnz_c[c & 1]
                        .get((mb_y * 2 + by) * w2 + (mb_x * 2 - 1))
                        .copied()
                        .ok_or(MbError::Inconsistent("nnz_c"))?
                };
            }
        }
        Ok(())
    }

    #[inline]
    pub(crate) fn chroma_nc_pred(&self, c: usize, bx: usize, by: usize) -> i32 {
        // Same shape as `nc_pred`, one size down: a 3x3 grid in `[u8; 9]`, two
        // planes, and every caller passes 0..1 for all three coordinates.
        let (c, bx, by) = (c & 1, bx & 1, by & 1);
        let left = self.nnz_c_cache[c][(by + 1) * 3 + bx] as i32;
        let top = self.nnz_c_cache[c][by * 3 + (bx + 1)] as i32;
        let r = left + top;
        if r < i32::from(NNZ_NO_DISPONIBLE) {
            (r + 1) >> 1
        } else {
            r & NNZ_MASCARA_VECINO_UNICO
        }
    }

    #[inline]
    pub(crate) fn chroma_nnz_cache_set(&mut self, c: usize, bx: usize, by: usize, total: u8) {
        self.nnz_c_cache[c & 1][((by & 1) + 1) * 3 + ((bx & 1) + 1)] = total;
    }

    /// Decodes one slice's macroblocks (raster order) starting at `first_mb`,
    /// until `more_rbsp_data()` is exhausted or the picture is full. Returns the
    /// next macroblock address (= total when the picture is complete). In a
    /// P-slice each macroblock is preceded by `mb_skip_run`. `filtro` son los parÃ¡metros del
    /// filtro de desbloqueo de este corte, que quedan registrados con Ã©l.
    ///
    /// # Errors
    /// Los de la decodificaciÃ³n de los macrobloques del corte.
    pub fn decode_slice_data(
        &mut self,
        r: &mut BitReader,
        is_p: bool,
        first_mb: usize,
        filtro: FiltroCorte,
    ) -> Result<usize, MbError> {
        // Same cross-slice guard as the CABAC loop head.
        self.span_flush()?;
        self.registrar_corte(first_mb, filtro);
        self.decode_slice_cavlc_inner(r, is_p, first_mb)
    }

    pub(crate) fn decode_slice_cavlc_inner(
        &mut self,
        r: &mut BitReader,
        is_p: bool,
        first_mb: usize,
    ) -> Result<usize, MbError> {
        let total = self.mb_w * self.mb_h;
        self.slice_first_mb = first_mb;
        self.edc_active = true;
        let mut addr = first_mb;
        // Same malformed-header divide-by-zero as the sibling slice loop above.
        let mbw_nz = self.mb_w.max(1);
        let (mut mbx, mut mby) = (addr % mbw_nz, addr / mbw_nz);
        let mbw_c = self.mb_w;
        while addr < total {
            if mbx == mbw_c {
                mbx = 0;
                mby += 1;
            }
            self.row_hook_at(addr, mby)?;
            if is_p || self.is_b {
                let skip_run = { r.read_ue()? } as usize;
                // A run past the picture end is a corrupt stream (ffmpeg errors
                // here too); a run TO the end is legal. Silently clamping used
                // to fill the remainder with skip MBs.
                if skip_run > total - addr {
                    return Err(MbError::Truncated);
                }
                // The run length is KNOWN here (CAVLC codes it as one syntax
                // element). P runs: after the first skip commits (0,0), every
                // later run MB is FORCED (0,0) by the zero-MV rule â€” process
                // the remainder SEGMENT-WISE: one span extension + one mb_qp
                // fill per row segment instead of a per-MB call + span match +
                // store. Non-(0,0) runs and B runs keep the per-MB loop. The
                // per-MB `addr >= total` check is gone: the run was validated
                // against `total - addr` above.
                let mut remaining = skip_run;
                while remaining > 0 {
                    debug_assert!(addr < total);
                    if mbx == mbw_c {
                        mbx = 0;
                        mby += 1;
                    }
                    if self.is_b {
                        if !self.b_skip_hot(mbx, mby)? {
                            self.decode_b_skip(mbx, mby)?;
                        }
                        *self
                            .mb_qp
                            .get_mut(addr)
                            .ok_or(MbError::Inconsistent("mb_qp"))? = self.cur_qp; // skip inherits QPy
                        addr += 1;
                        mbx += 1;
                        remaining -= 1;
                        continue;
                    }
                    self.decode_p_skip(mbx, mby)?;
                    *self
                        .mb_qp
                        .get_mut(addr)
                        .ok_or(MbError::Inconsistent("mb_qp"))? = self.cur_qp;
                    addr += 1;
                    mbx += 1;
                    remaining -= 1;
                    // Segment-wise remainder: forced (0,0) continuation holds
                    // exactly while skip_zero_next tracks addr (decode_p_skip
                    // set it iff this skip committed (0,0)).
                    // Worker mode (edc_tx) must route per-MB jobs through the
                    // channel â€” the parse thread no longer owns the planes.
                    while remaining > 0 && self.skip_zero_next == addr {
                        if mbx == mbw_c {
                            mbx = 0;
                            mby += 1;
                        }
                        // Row segment: run to row end or run end.
                        let seg = remaining.min(mbw_c - mbx);
                        let recon = self.weights.is_none() || self.weights_id0;
                        for k in 0..seg {
                            self.pz_push(mbx + k, mby, recon)?;
                        }
                        self.mb_qp[addr..addr + seg].fill(self.cur_qp);
                        if !recon {
                            for k in 0..seg {
                                self.edc_jobs.push(EdcJob::Skip {
                                    mbx: mbx + k,
                                    mby,
                                    mv: (0, 0),
                                });
                            }
                        }
                        addr += seg;
                        mbx += seg;
                        remaining -= seg;
                        self.skip_zero_next = addr;
                    }
                }
                if addr >= total {
                    break;
                }
                // A trailing skip run with no following macroblock ends the slice.
                if skip_run > 0 && !r.more_rbsp_data() {
                    break;
                }
            }
            // The skip run above may have crossed a row boundary within this
            // iteration â€” wrap before the non-skip MB decodes.
            if mbx == mbw_c {
                mbx = 0;
                mby += 1;
            }
            if self.is_b {
                // ORDER: B reconstructs inline (not seam-ready).
                self.edc_flush()?;
                self.decode_b_mb(r, mbx, mby)?;
            } else {
                // Non-skip CAVLC MB: inter MV prediction + intra gathers
                // read the grids â€” flush deferred spans.
                self.span_flush()?;
                self.decode_mb(r, mbx, mby, is_p)?;
            }
            // PARCHE-CRUSTY (ARR-1): un I_PCM se filtra con QPY = 0 (Â§8.7.2.2).
            let qp_desbloqueo = if core::mem::take(&mut self.pcm_qp_pending) {
                0
            } else {
                self.cur_qp
            };
            *self
                .mb_qp
                .get_mut(addr)
                .ok_or(MbError::Inconsistent("mb_qp"))? = qp_desbloqueo;
            addr += 1;
            mbx += 1;
            // CAVLC slice end: no more data after this macroblock.
            if !r.more_rbsp_data() {
                break;
            }
        }
        self.edc_flush()?; // slice end: no job crosses a slice boundary
        Ok(addr)
    }

    pub(crate) fn decode_mb(
        &mut self,
        r: &mut BitReader,
        mb_x: usize,
        mb_y: usize,
        is_p: bool,
    ) -> Result<(), MbError> {
        let mut mb_type = { r.read_ue()? };
        if is_p {
            // In P-slices, mb_type 0/1/2 are inter (16Ã—16, 16Ã—8, 8Ã—16),
            // 3 = P_8x8, 4 = P_8x8ref0 (ref_idx forced 0), 5+ intra.
            if mb_type <= 2 {
                return self.decode_inter(r, mb_x, mb_y, mb_type as u8);
            }
            if mb_type == 3 || mb_type == 4 {
                return self.decode_p8x8(r, mb_x, mb_y, mb_type == 4);
            }
            mb_type -= 5;
        }
        // ORDER: intra reconstruction reads neighbour PIXELS, so the worker
        // must have applied every deferred job before this point.
        self.edc_flush()?;
        self.decode_intra_mb(r, mb_x, mb_y, mb_type)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nc_pred_ambos_vecinos_disponibles() {
        let mut fd = FrameDecoder::new(1, 1, 26, 0, alloc::vec::Vec::new(), 0, false, false, false);
        // Para lbx=0, lby=0:
        // left estÃ¡ en (lby + 1)*5 + lbx = 1*5 + 0 = 5
        // top estÃ¡ en lby*5 + (lbx + 1) = 0*5 + 1 = 1
        fd.nnz_l_cache[5] = 2;
        fd.nnz_l_cache[1] = 4;
        // PredicciÃ³n: (2 + 4 + 1) >> 1 = 7 >> 1 = 3
        assert_eq!(fd.nc_pred(0, 0), 3);
    }

    #[test]
    fn nc_pred_solo_un_vecino_disponible() {
        let mut fd = FrameDecoder::new(1, 1, 26, 0, alloc::vec::Vec::new(), 0, false, false, false);
        // Superior no disponible (0x80), izquierdo disponible con valor 5
        fd.nnz_l_cache[5] = 5;
        fd.nnz_l_cache[1] = 0x80;
        assert_eq!(fd.nc_pred(0, 0), 5);

        // Izquierdo no disponible (0x80), superior disponible con valor 7
        fd.nnz_l_cache[5] = 0x80;
        fd.nnz_l_cache[1] = 7;
        assert_eq!(fd.nc_pred(0, 0), 7);
    }

    #[test]
    fn nc_pred_ningun_vecino_disponible() {
        let mut fd = FrameDecoder::new(1, 1, 26, 0, alloc::vec::Vec::new(), 0, false, false, false);
        // Ambos vecinos no disponibles (0x80)
        fd.nnz_l_cache[5] = 0x80;
        fd.nnz_l_cache[1] = 0x80;
        assert_eq!(fd.nc_pred(0, 0), 0);
    }

    #[test]
    fn nnz_cache_set_asigna_posicion_correcta() {
        let mut fd = FrameDecoder::new(1, 1, 26, 0, alloc::vec::Vec::new(), 0, false, false, false);
        fd.nnz_cache_set(2, 3, 11);
        // Ãndice esperado: ((3 & 3) + 1) * 5 + ((2 & 3) + 1) = 4 * 5 + 3 = 23
        assert_eq!(fd.nnz_l_cache[23], 11);
    }

    #[test]
    fn chroma_nc_pred_y_set() {
        let mut fd = FrameDecoder::new(1, 1, 26, 0, alloc::vec::Vec::new(), 0, false, false, false);
        // c=0, bx=0, by=0:
        // left: (by+1)*3 + bx = 1*3 + 0 = 3
        // top: by*3 + (bx+1) = 0*3 + 1 = 1
        fd.nnz_c_cache[0][3] = 3;
        fd.nnz_c_cache[0][1] = 5;
        // (3 + 5 + 1) >> 1 = 4
        assert_eq!(fd.chroma_nc_pred(0, 0, 0), 4);

        fd.chroma_nnz_cache_set(0, 1, 1, 9);
        // ((1 & 1) + 1) * 3 + ((1 & 1) + 1) = 2 * 3 + 2 = 8
        assert_eq!(fd.nnz_c_cache[0][8], 9);
    }
}
