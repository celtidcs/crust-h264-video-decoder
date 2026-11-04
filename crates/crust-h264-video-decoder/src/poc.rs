//! Derivación del recuento de orden de imagen (`PicOrderCnt`, UIT-T H.264 §8.2.1).
//!
//! Mantiene el estado entre imágenes en orden de decodificación para calcular `PicOrderCnt` en los
//! modos de tipo 0 (§8.2.1.1), tipo 1 (§8.2.1.2) y tipo 2 (§8.2.1.3), así como el
//! reajuste tras una operación `MMCO 5` (§8.2.5.4.5).

use crate::params::Sps;

/// Running picture-order-count derivation state.
/// Estado acumulado para la derivación de `PicOrderCnt` entre fotogramas (UIT-T H.264 §8.2.1).
#[derive(Clone, Default)]
pub(crate) struct PocState {
    prev_msb: i32,
    prev_lsb: i32,
    prev_frame_num: u32,
    prev_frame_num_offset: i64,
    prev_has_mmco5: bool,
}

impl PocState {
    /// Aplica el reinicio de estado tras una operación `MMCO 5` (UIT-T H.264 §8.2.5.4.5).
    pub(crate) fn apply_mmco5(&mut self, poc_type: u32, current_poc: i32) -> i32 {
        self.prev_has_mmco5 = true;
        match poc_type {
            0 => {
                // UIT-T H.264 (06/26) §8.2.1, pág. 140: tras una MMCO 5, TopFieldOrderCnt de la imagen pasa a
                // ser TopFieldOrderCnt − PicOrderCnt(imagen), y §8.2.1.1 toma ese valor como
                // prevPicOrderCntLsb de la siguiente (con prevPicOrderCntMsb = 0). `compute_poc` acaba de
                // guardar el Msb y el lsb de esta imagen de referencia, así que TopFieldOrderCnt = Msb + lsb.
                let top = self.prev_msb + self.prev_lsb;
                self.prev_msb = 0;
                self.prev_lsb = top - current_poc;
                // PicOrderCnt de la imagen tras el reajuste: Min(Top', Bottom') = 0.
                0
            }
            1 => {
                #[allow(clippy::cast_possible_truncation)]
                let new_poc = current_poc - self.prev_frame_num_offset as i32;
                self.prev_frame_num_offset = 0;
                self.prev_frame_num = 0;
                new_poc
            }
            2 => {
                self.prev_frame_num_offset = 0;
                self.prev_frame_num = 0;
                0
            }
            _ => current_poc,
        }
    }

    /// PARCHE-CRUSTY (ARR-1): `PicOrderCnt` de tipo 1 para un fotograma (H.264 §8.2.1.2).
    /// Aritmética en i64 con saturación: un flujo hostil no puede desbordar.
    fn compute_poc_type1(
        &mut self,
        sps: &Sps,
        is_idr: bool,
        nal_ref_idc: u8,
        frame_num: u32,
        delta_poc_0: i32,
        delta_poc_1: i32,
    ) -> i32 {
        let max_fn = 1i64 << sps.log2_max_frame_num;
        let (prev_fn_offset, prev_fn) = if is_idr || self.prev_has_mmco5 {
            (0, 0)
        } else {
            (self.prev_frame_num_offset, self.prev_frame_num)
        };
        let offset = if is_idr {
            0
        } else if prev_fn > frame_num {
            prev_fn_offset + max_fn
        } else {
            prev_fn_offset
        };
        let ciclo = &sps.offset_for_ref_frame;
        let mut abs_frame_num = if ciclo.is_empty() {
            0
        } else {
            offset + i64::from(frame_num)
        };
        if nal_ref_idc == 0 && abs_frame_num > 0 {
            abs_frame_num -= 1;
        }
        let mut esperado = 0i64;
        if abs_frame_num > 0 {
            #[allow(clippy::cast_possible_wrap)]
            let n = ciclo.len() as i64;
            let delta_ciclo: i64 = ciclo.iter().map(|&o| i64::from(o)).sum();
            let vueltas = (abs_frame_num - 1) / n;
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let dentro = ((abs_frame_num - 1) % n) as usize;
            esperado = vueltas.saturating_mul(delta_ciclo);
            for &o in ciclo.iter().take(dentro + 1) {
                esperado = esperado.saturating_add(i64::from(o));
            }
        }
        if nal_ref_idc == 0 {
            esperado = esperado.saturating_add(i64::from(sps.offset_for_non_ref_pic));
        }
        let (d0, d1) = if sps.delta_pic_order_always_zero {
            (0, 0)
        } else {
            (i64::from(delta_poc_0), i64::from(delta_poc_1))
        };
        let top = esperado.saturating_add(d0);
        let bottom = top
            .saturating_add(i64::from(sps.offset_for_top_to_bottom_field))
            .saturating_add(d1);
        self.prev_frame_num_offset = offset;
        self.prev_frame_num = frame_num;
        self.prev_has_mmco5 = false;
        #[allow(clippy::cast_possible_truncation)]
        {
            top.min(bottom)
                .clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
        }
    }

    /// Derives `PicOrderCnt` for the current picture (spec §8.2.1) and advances
    /// this state. `deltas` = (`delta_pic_order_cnt[0]`, `delta_pic_order_cnt_bottom`
    /// or `delta_pic_order_cnt[1]`). PARCHE-CRUSTY (ARR-1): type 1 now follows §8.2.1.2
    /// instead of being approximated by frame_num.
    pub(crate) fn compute_poc(
        &mut self,
        sps: &Sps,
        is_idr: bool,
        nal_ref_idc: u8,
        frame_num: u32,
        poc_lsb: u32,
        deltas: (i32, i32),
    ) -> i32 {
        let (delta_poc_0, delta_bottom) = deltas;
        match sps.pic_order_cnt_type {
            0 => {
                let max_lsb = 1i32 << sps.log2_max_pic_order_cnt_lsb;
                let (prev_msb, prev_lsb) = if is_idr {
                    (0, 0)
                } else if self.prev_has_mmco5 {
                    (0, self.prev_lsb)
                } else {
                    (self.prev_msb, self.prev_lsb)
                };
                #[allow(clippy::cast_possible_wrap)]
                let lsb = poc_lsb as i32;
                let msb = if lsb < prev_lsb && prev_lsb - lsb >= max_lsb / 2 {
                    prev_msb + max_lsb
                } else if lsb > prev_lsb && lsb - prev_lsb > max_lsb / 2 {
                    prev_msb - max_lsb
                } else {
                    prev_msb
                };
                let top = msb + lsb;
                let poc = top.min(top + delta_bottom);
                if nal_ref_idc != 0 {
                    self.prev_msb = msb;
                    self.prev_lsb = lsb;
                    self.prev_has_mmco5 = false;
                }
                poc
            }
            2 => {
                let max_fn = 1i64 << sps.log2_max_frame_num;
                let (prev_fn_offset, prev_fn) = if is_idr || self.prev_has_mmco5 {
                    (0, 0)
                } else {
                    (self.prev_frame_num_offset, self.prev_frame_num)
                };
                let offset = if is_idr {
                    0
                } else if prev_fn > frame_num {
                    prev_fn_offset + max_fn
                } else {
                    prev_fn_offset
                };
                let poc = if is_idr {
                    0
                } else {
                    2 * (offset + i64::from(frame_num)) - i64::from(nal_ref_idc == 0)
                };
                self.prev_frame_num_offset = offset;
                self.prev_frame_num = frame_num;
                self.prev_has_mmco5 = false;
                #[allow(clippy::cast_possible_truncation)]
                {
                    poc as i32
                }
            }
            _ => self.compute_poc_type1(
                sps,
                is_idr,
                nal_ref_idc,
                frame_num,
                delta_poc_0,
                delta_bottom,
            ),
        }
    }
}

#[cfg(test)]
mod tests;
