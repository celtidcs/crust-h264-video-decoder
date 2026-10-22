//! Compensación de movimiento de los B, con predicción ponderada (§8.4.2.2, §8.4.2.3).

use rusty_h264_common::inter::{mc_chroma_padded_pair, McScratch};

use crate::edc::tx_for_td;
use crate::error::MbError;
use crate::mb16::FrameDecoder;

/// Región de un macrobloque B que se compensa: el macrobloque `(mb_x, mb_y)` y, dentro de él, el
/// rectángulo de origen `(px, py)` y tamaño `rw`×`rh` muestras de luma.
#[derive(Clone, Copy)]
pub(crate) struct RegionB {
    pub(crate) mb_x: usize,
    pub(crate) mb_y: usize,
    pub(crate) px: usize,
    pub(crate) py: usize,
    pub(crate) rw: usize,
    pub(crate) rh: usize,
}

/// Movimiento de las dos listas de una región B: índice de referencia (−1 = la lista no se usa) y
/// vector en cuartos de muestra de cada una.
#[derive(Clone, Copy)]
pub(crate) struct MovimientoBi {
    pub(crate) refi0: i32,
    pub(crate) mv0: (i32, i32),
    pub(crate) refi1: i32,
    pub(crate) mv1: (i32, i32),
}

/// Bipredicción sobre `dst`, que ya tiene la predicción de la lista 0: le suma la de la lista 1
/// (`q`) con el promedio `(p+q+1)>>1` o con los pesos implícitos `(w0, w1)` (§8.4.2.3).
///
/// SLICE-then-zip: proving the bounds ONCE lets rustc emit the whole
/// 256-byte average as 8 straight-line vpavgb ops (verified in
/// isolation, x86-64-v3); the indexed form kept a per-iteration
/// bounds check and a loop. A hand AVX2 kernel is refuted — the
/// compiler already emits the ideal instruction.
#[inline(always)]
fn mezclar_en(dst: &mut [u8], q: &[u8], weights: Option<(i32, i32)>) {
    match weights {
        None => {
            for (d, s) in dst.iter_mut().zip(q) {
                *d = ((*d as u16 + *s as u16 + 1) >> 1) as u8;
            }
        }
        Some((w0, w1)) => {
            for (d, s) in dst.iter_mut().zip(q) {
                *d = ((*d as i32 * w0 + *s as i32 * w1 + 32) >> 6).clamp(0, 255) as u8;
            }
        }
    }
}

/// Bipredicción de una fila: `dst` recibe la mezcla de `p` (lista 0) y `q` (lista 1), como
/// [`mezclar_en`].
#[inline(always)]
fn mezclar(dst: &mut [u8], p: &[u8], q: &[u8], weights: Option<(i32, i32)>) {
    match weights {
        None => {
            for ((d, p), q) in dst.iter_mut().zip(p).zip(q) {
                *d = ((*p as u16 + *q as u16 + 1) >> 1) as u8;
            }
        }
        Some((w0, w1)) => {
            for ((d, p), q) in dst.iter_mut().zip(p).zip(q) {
                *d = ((*p as i32 * w0 + *q as i32 * w1 + 32) >> 6).clamp(0, 255) as u8;
            }
        }
    }
}

/// Límites de `tb` y `td` en `DistScaleFactor` (§8.4.1.2.3, ecuaciones 8-197 y 8-198).
const TD_TB_MIN: i32 = -128;
/// Véase [`TD_TB_MIN`].
const TD_TB_MAX: i32 = 127;
/// Límites de `DistScaleFactor` (§8.4.1.2.3, ecuación 8-199).
const DSF_MIN: i32 = -1024;
/// Véase [`DSF_MIN`].
const DSF_MAX: i32 = 1023;
/// Suma de los dos pesos implícitos (§8.4.2.3.1: `w0 = 64 − w1`).
const PESO_IMPLICITO_TOTAL: i32 = 64;
/// Rango de `w1` para el que se usan los pesos implícitos; fuera de él, promedio (§8.4.2.3.1).
const PESO_IMPLICITO_MIN: i32 = -64;
/// Véase [`PESO_IMPLICITO_MIN`].
const PESO_IMPLICITO_MAX: i32 = 128;
/// Pesos implícitos que equivalen exactamente al promedio `(a+b+1)>>1`.
pub(crate) const PESOS_PROMEDIO: (i32, i32) = (32, 32);

/// `DistScaleFactor` (§8.4.1.2.3) de la imagen actual (`poc_actual`) entre dos referencias de POC
/// `poc0` y `poc1`; `None` si las dos referencias están a la misma distancia (`td` = 0). Lo usan
/// los pesos implícitos y el directo temporal; cada uno comprueba aparte sus referencias a largo
/// plazo.
pub(super) fn factor_escala_distancia(poc_actual: i32, poc0: i32, poc1: i32) -> Option<i32> {
    let td = (poc1 - poc0).clamp(TD_TB_MIN, TD_TB_MAX);
    if td == 0 {
        return None;
    }
    let tb = (poc_actual - poc0).clamp(TD_TB_MIN, TD_TB_MAX);
    let tx = tx_for_td(td);
    Some(((tb * tx + 32) >> 6).clamp(DSF_MIN, DSF_MAX))
}

impl FrameDecoder {
    /// Implicit bi-prediction weights `(w0, w1)` from POC distances (spec
    /// §8.4.2.3.2), or `None` for the plain average (idc≠2, uni-pred, or the
    /// equidistant / out-of-range fall-back to 32:32 which equals the average).
    pub(crate) fn implicit_weights(&self, refi0: i32, refi1: i32) -> Option<(i32, i32)> {
        if self.weighted_bipred_idc != 2 || refi0 < 0 || refi1 < 0 {
            return None;
        }
        let (Some(r0), Some(r1)) = (
            self.refs.get(refi0 as usize),
            self.refs1.get(refi1 as usize),
        ) else {
            return None;
        };
        if r0.long_term || r1.long_term {
            return None; // 32:32 → identical to the average
        }
        // td == 0 → None too: 32:32 → identical to the average.
        let dsf = factor_escala_distancia(self.cur_poc, r0.pic_poc(), r1.pic_poc())?;
        let w1 = dsf >> 2;
        if !(PESO_IMPLICITO_MIN..=PESO_IMPLICITO_MAX).contains(&w1) {
            return None; // out of range → 32:32 average
        }
        Some((PESO_IMPLICITO_TOTAL - w1, w1))
    }

    /// Motion-compensates a region with the given per-list refs/MVs. Bi-prediction
    /// is the simple `(a+b+1)>>1` average, or POC-weighted when implicit weighting
    /// (idc 2) is active. Writes into `pred_y`/`c_pred`.
    pub(crate) fn b_mc(
        &self,
        region: RegionB,
        mov: MovimientoBi,
        pred_y: &mut [u8; 256],
        c_pred: &mut [[u8; 64]; 2],
    ) -> Result<(), MbError> {
        let RegionB { px, py, rw, rh, .. } = region;
        let MovimientoBi {
            refi0,
            mv0,
            refi1,
            mv1,
        } = mov;
        // Malformed-stream armor, mirroring the P path: now that B slices actually
        // PARSE ref_idx (they used to be hardcoded to 0), a mutated stream can hand
        // us an index past the end of either list. Clamp rather than panic — the
        // crate is `forbid(unsafe_code)` and fuzz-gated to never panic, and a
        // wrong picture on garbage input carries no conformance duty.
        let refi0 = if refi0 >= 0 {
            (refi0 as usize).min(self.refs.len().saturating_sub(1)) as i32
        } else {
            -1
        };
        let refi1 = if refi1 >= 0 {
            (refi1 as usize).min(self.refs1.len().saturating_sub(1)) as i32
        } else {
            -1
        };
        if (refi0 >= 0 && self.refs.is_empty()) || (refi1 >= 0 && self.refs1.is_empty()) {
            return Ok(());
        }
        let cch = self.mb_h * 8;
        let weights = { self.implicit_weights(refi0, refi1) };
        // Bi-prediction blend: the weights decision is LOOP-INVARIANT, so every
        // blend site below matches on `weights` ONCE and runs a branch-free
        // pixel loop — the unweighted `(p+q+1)>>1` average then autovectorizes
        // (the per-pixel closure this replaces hid the invariant behind a
        // capture, and its chroma form was a &dyn call PER PIXEL).
        // FULL-WIDTH regions (px == 0, rw == 16 — every 16×16/16×8 partition and
        // most direct regions) occupy contiguous rows of `pred_y`, so MC writes
        // the destination DIRECTLY: uni-pred needs no staging at all, bi-pred
        // stages only the second list and blends in place. The staging arrays
        // (512 B zeroed per call before this) now exist only on the branches
        // that read them. Same fusion as the P path's mc_rect (WHYS Part 8).
        let full = px == 0 && rw == 16;

        // One scratch borrow for the whole region — both bi-pred passes included.
        // The closure yields whether the arm already ran the chroma half (the
        // bi-pred full-width arm does, to keep its staging alive) — a plain
        // `return` inside would exit the CLOSURE only and chroma would run twice.
        let mov_ajustado = MovimientoBi {
            refi0,
            mv0,
            refi1,
            mv1,
        };
        let chroma_done =
            rusty_h264_common::inter::with_mc_scratch(|scr| -> Result<bool, MbError> {
                // Full-width destination rows are contiguous in `pred_y`.
                let destino = py * 16..py * 16 + rw * rh;
                match (refi0 >= 0, refi1 >= 0, full) {
                    (true, false, true) => {
                        self.mc_luma_lista(scr, region, 0, refi0, mv0, &mut pred_y[destino])?;
                        Ok(false)
                    }
                    (false, true, true) => {
                        self.mc_luma_lista(scr, region, 1, refi1, mv1, &mut pred_y[destino])?;
                        Ok(false)
                    }
                    (true, true, true) => {
                        self.mc_luma_lista(
                            scr,
                            region,
                            0,
                            refi0,
                            mv0,
                            &mut pred_y[destino.clone()],
                        )?;
                        let mut b = [0u8; 256];
                        self.mc_luma_lista(scr, region, 1, refi1, mv1, &mut b[..rw * rh])?;
                        mezclar_en(&mut pred_y[destino], &b[..rw * rh], weights);
                        self.b_mc_chroma(region, mov_ajustado, c_pred, weights, cch)?;
                        Ok(true)
                    }
                    _ => {
                        // Narrow region — rows are strided in `pred_y`; stage and copy.
                        let (mut a, mut b) = ([0u8; 256], [0u8; 256]);
                        if refi0 >= 0 {
                            self.mc_luma_lista(scr, region, 0, refi0, mv0, &mut a[..rw * rh])?;
                        }
                        if refi1 >= 0 {
                            self.mc_luma_lista(scr, region, 1, refi1, mv1, &mut b[..rw * rh])?;
                        }
                        match (refi0 >= 0, refi1 >= 0) {
                            (true, true) => {
                                for dy in 0..rh {
                                    let (ar, br) =
                                        (&a[dy * rw..dy * rw + rw], &b[dy * rw..dy * rw + rw]);
                                    let base = (py + dy) * 16 + px;
                                    mezclar(&mut pred_y[base..base + rw], ar, br, weights);
                                }
                            }
                            (true, false) => {
                                for dy in 0..rh {
                                    let d = (py + dy) * 16 + px;
                                    pred_y[d..d + rw].copy_from_slice(&a[dy * rw..dy * rw + rw]);
                                }
                            }
                            _ => {
                                for dy in 0..rh {
                                    let d = (py + dy) * 16 + px;
                                    pred_y[d..d + rw].copy_from_slice(&b[dy * rw..dy * rw + rw]);
                                }
                            }
                        }
                        Ok(false)
                    }
                }
            })?;
        if chroma_done {
            return Ok(());
        }
        self.b_mc_chroma(region, mov_ajustado, c_pred, weights, cch)?;
        Ok(())
    }

    /// Compensación de luma de la lista `lista` (0 o 1) para `region`: la referencia `refi` de esa
    /// lista, desplazada por `mv`, escrita en `dst` (`rw`×`rh` muestras seguidas). La columna es
    /// `mb_x·16 + px` también en las regiones de ancho completo, donde `px` vale 0.
    ///
    /// # Errors
    /// `MbError::Inconsistent` si `refi` no está en la lista, y los del recorte del plano.
    #[inline]
    fn mc_luma_lista(
        &self,
        scr: &mut McScratch,
        region: RegionB,
        lista: usize,
        refi: i32,
        mv: (i32, i32),
        dst: &mut [u8],
    ) -> Result<(), MbError> {
        let refs = if lista == 0 { &self.refs } else { &self.refs1 };
        let Some(rf) = refs.get(refi as usize) else {
            return Err(MbError::Inconsistent("refs"));
        };
        rusty_h264_common::inter::mc_luma_padded_pre(
            scr,
            rf.plano_luma(),
            rf.lstride(),
            crate::LPAD,
            self.cw,
            self.mb_h * 16,
            region.mb_x * 16 + region.px,
            region.mb_y * 16 + region.py,
            region.rw,
            region.rh,
            mv.0,
            mv.1,
            dst,
        )?;
        Ok(())
    }

    /// Compensación de las dos componentes de croma desde la referencia `rf`: la región de
    /// `crw`×`crh` muestras que empieza en `(x, y)` del plano (de alto `cch`), desplazada por `mv`,
    /// escrita en `dst_u`/`dst_v`. U y V comparten toda la geometría: una sola llamada a
    /// `mc_chroma_padded_pair`.
    ///
    /// # Errors
    /// Los del recorte de los planos.
    #[inline]
    fn mc_croma_par(
        &self,
        rf: &crate::RefFrame,
        (x, y): (usize, usize),
        (crw, crh, cch): (usize, usize, usize),
        mv: (i32, i32),
        (dst_u, dst_v): (&mut [u8], &mut [u8]),
    ) -> Result<(), MbError> {
        let (gu, gv) = (rf.plano_croma(0), rf.plano_croma(1));
        mc_chroma_padded_pair(
            gu,
            gv,
            rf.cstride(),
            crate::CPAD,
            self.ccw,
            cch,
            x,
            y,
            crw,
            crh,
            mv.0,
            mv.1,
            dst_u,
            dst_v,
        )?;
        Ok(())
    }

    /// Chroma half of `b_mc`, with the same full-width direct-write fusion
    /// (crw == 8 rows are contiguous in the 8-wide `c_pred` planes).
    ///
    /// Chroma half of `b_mc`. U and V share every piece of MC geometry, so
    /// each list is ONE `mc_chroma_padded_pair` call (setup + range check paid
    /// once, kernels unchanged) instead of two per-plane calls — the per-plane
    /// pixel math is byte-identical to the old per-plane flow.
    pub(crate) fn b_mc_chroma(
        &self,
        region: RegionB,
        mov: MovimientoBi,
        c_pred: &mut [[u8; 64]; 2],
        weights: Option<(i32, i32)>,
        cch: usize,
    ) -> Result<(), MbError> {
        let RegionB {
            mb_x,
            mb_y,
            px,
            py,
            rw,
            rh,
        } = region;
        let MovimientoBi {
            refi0,
            mv0,
            refi1,
            mv1,
        } = mov;
        let (crx, cry, crw, crh) = (px / 2, py / 2, rw / 2, rh / 2);
        let full = crx == 0 && crw == 8;
        let n = crw * crh;
        // RESOLVE BOTH SLOTS ONCE, then let the match patterns bind them. Every
        // arm below re-indexed `refs`/`refs1` by a raw `refi` — six checked Vec
        // indexes across the four arms — although `a0`/`a1` already meant
        // exactly "this slot is present". Carrying `Option<&RefPic>` instead of
        // a bool makes "present" and "in range" the same fact.
        // Un índice no negativo que no está en la lista es un estado incoherente
        // (COR-3), no «lista sin usar».
        let rf0 = match usize::try_from(refi0) {
            Ok(i) => Some(self.refs.get(i).ok_or(MbError::Inconsistent("refs"))?),
            Err(_) => None,
        };
        let rf1 = match usize::try_from(refi1) {
            Ok(i) => Some(self.refs1.get(i).ok_or(MbError::Inconsistent("refs"))?),
            Err(_) => None,
        };
        let (a0, a1) = (rf0.is_some(), rf1.is_some());
        let [cu, cv] = c_pred;
        let origen = (mb_x * 8 + crx, mb_y * 8 + cry);
        match (a0, a1, full) {
            (true, false, true) | (false, true, true) => {
                let (Some(rf), mv) = (if a0 { rf0 } else { rf1 }, if a0 { mv0 } else { mv1 })
                else {
                    return Err(MbError::Inconsistent("refs"));
                };
                self.mc_croma_par(
                    rf,
                    origen,
                    (crw, crh, cch),
                    mv,
                    (&mut cu[cry * 8..cry * 8 + n], &mut cv[cry * 8..cry * 8 + n]),
                )?;
            }
            (true, true, true) => {
                let (Some(rfa), Some(rfb)) = (rf0, rf1) else {
                    return Err(MbError::Inconsistent("refs"));
                };
                self.mc_croma_par(
                    rfa,
                    origen,
                    (crw, crh, cch),
                    mv0,
                    (&mut cu[cry * 8..cry * 8 + n], &mut cv[cry * 8..cry * 8 + n]),
                )?;
                let (mut bu, mut bv) = ([0u8; 64], [0u8; 64]);
                self.mc_croma_par(
                    rfb,
                    origen,
                    (crw, crh, cch),
                    mv1,
                    (&mut bu[..n], &mut bv[..n]),
                )?;
                for (dst, stage) in [(&mut *cu, &bu), (&mut *cv, &bv)] {
                    mezclar_en(&mut dst[cry * 8..cry * 8 + n], &stage[..n], weights);
                }
            }
            _ => {
                // Narrow region — rows are strided in the 8-wide pred planes;
                // stage per list (paired), then copy/blend strided per plane.
                let (mut au, mut av, mut bu, mut bv) = ([0u8; 64], [0u8; 64], [0u8; 64], [0u8; 64]);
                if let Some(rf) = rf0 {
                    self.mc_croma_par(
                        rf,
                        origen,
                        (crw, crh, cch),
                        mv0,
                        (&mut au[..n], &mut av[..n]),
                    )?;
                }
                if let Some(rf) = rf1 {
                    self.mc_croma_par(
                        rf,
                        origen,
                        (crw, crh, cch),
                        mv1,
                        (&mut bu[..n], &mut bv[..n]),
                    )?;
                }
                for (dst, sa, sb) in [(&mut *cu, &au, &bu), (&mut *cv, &av, &bv)] {
                    for dy in 0..crh {
                        let base = (cry + dy) * 8 + crx;
                        let d = &mut dst[base..base + crw];
                        match (a0, a1) {
                            (true, true) => mezclar(
                                d,
                                &sa[dy * crw..dy * crw + crw],
                                &sb[dy * crw..dy * crw + crw],
                                weights,
                            ),
                            (true, false) => d.copy_from_slice(&sa[dy * crw..dy * crw + crw]),
                            _ => d.copy_from_slice(&sb[dy * crw..dy * crw + crw]),
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Commits a region's per-list motion to the 4×4 grids (and marks coded).
    pub(crate) fn b_set_motion(&mut self, region: RegionB, mov: MovimientoBi) {
        let RegionB {
            mb_x,
            mb_y,
            px,
            py,
            rw,
            rh,
        } = region;
        let MovimientoBi {
            refi0,
            mv0,
            refi1,
            mv1,
        } = mov;
        let w4 = self.mb_w * 4;
        let mv0w = if refi0 >= 0 { mv0 } else { (0, 0) };
        let mv1w = if refi1 >= 0 { mv1 } else { (0, 0) };
        let by0 = mb_y * 4 + py / 4;
        let bx0 = mb_x * 4 + px / 4;
        let (bw, bh) = (rw / 4, rh / 4);
        // Contiguous per-row slices: fill instead of per-4x4 stores (same values).
        // SEVEN parallel grids, seven separate range checks — they are disjoint
        // fields of `self`, so one `get_mut` apiece resolves them all together
        // and a short row simply writes nothing rather than panicking.
        for by in by0..by0 + bh {
            let row = by * w4 + bx0;
            let end = row + bw;
            let (Some(r0), Some(m0), Some(r1), Some(m1)) = (
                self.ref_idx_y.get_mut(row..end),
                self.mv_y.get_mut(row..end),
                self.ref_idx1.get_mut(row..end),
                self.mv1.get_mut(row..end),
            ) else {
                continue;
            };
            r0.fill(refi0);
            m0.fill(mv0w);
            r1.fill(refi1);
            m1.fill(mv1w);
            let (Some(it), Some(cd), Some(md)) = (
                self.inter_y.get_mut(row..end),
                self.coded_y.get_mut(row..end),
                self.modes_y.get_mut(row..end),
            ) else {
                continue;
            };
            it.fill(true);
            cd.fill(true);
            md.fill(2);
        }
    }

    /// Splits a `w`×`h` block region (4×4-block units) into the fewest rectangles
    /// whose contents are `uniform`, preferring partition-shaped cuts (whole →
    /// horizontal halves → vertical halves → quadrants). Emits at most w·h rects
    /// (the all-different worst case degenerates to per-block, i.e. the old loop).
    pub(crate) fn coalesce_region<
        U: Fn(usize, usize, usize, usize) -> bool,
        E: FnMut(usize, usize, usize, usize),
    >(
        x: usize,
        y: usize,
        w: usize,
        h: usize,
        uniform: &U,
        emit: &mut E,
    ) {
        if uniform(x, y, w, h) {
            emit(x, y, w, h);
            return;
        }
        if h > 1 && uniform(x, y, w, h / 2) && uniform(x, y + h / 2, w, h / 2) {
            emit(x, y, w, h / 2);
            emit(x, y + h / 2, w, h / 2);
            return;
        }
        if w > 1 && uniform(x, y, w / 2, h) && uniform(x + w / 2, y, w / 2, h) {
            emit(x, y, w / 2, h);
            emit(x + w / 2, y, w / 2, h);
            return;
        }
        match (w > 1, h > 1) {
            (true, true) => {
                for q in 0..4usize {
                    Self::coalesce_region(
                        x + (q % 2) * (w / 2),
                        y + (q / 2) * (h / 2),
                        w / 2,
                        h / 2,
                        uniform,
                        emit,
                    );
                }
            }
            (true, false) => {
                Self::coalesce_region(x, y, w / 2, h, uniform, emit);
                Self::coalesce_region(x + w / 2, y, w / 2, h, uniform, emit);
            }
            (false, true) => {
                Self::coalesce_region(x, y, w, h / 2, uniform, emit);
                Self::coalesce_region(x, y + h / 2, w, h / 2, uniform, emit);
            }
            (false, false) => emit(x, y, 1, 1),
        }
    }

    /// Reconstructs a `P_Skip` macroblock: motion-compensate from the reference
    /// at the skip MV, with no residual.
    /// Records a B MC region (threaded mode) or executes it inline — the SAME
    /// arguments as `b_mc`; the recorded arm resolves the implicit weights at
    /// parse time (identical function, parse-side data).
    pub(crate) fn b_mc_or_record(
        &mut self,
        region: RegionB,
        mov: MovimientoBi,
        pred_y: &mut [u8; 256],
        c_pred: &mut [[u8; 64]; 2],
    ) -> Result<(), MbError> {
        let RegionB {
            mb_x,
            mb_y,
            px,
            py,
            rw,
            rh,
        } = region;
        let MovimientoBi {
            refi0,
            mv0,
            refi1,
            mv1,
        } = mov;
        self.b_mc(
            RegionB {
                mb_x,
                mb_y,
                px,
                py,
                rw,
                rh,
            },
            MovimientoBi {
                refi0,
                mv0,
                refi1,
                mv1,
            },
            pred_y,
            c_pred,
        )?;
        Ok(())
    }
}
