//! Macrobloques B en CAVLC: `ref_idx`, inter y B_8x8 (§7.3.5).

use super::{read_mvd, MAX_SUB_MB_TYPE_B, MB_TYPE_B_8X8, MB_TYPE_B_PRIMER_INTRA};
use crate::error::MbError;
use crate::mb16::FrameDecoder;
use crate::mb_cabac::read_ref_idx;
use crate::mb_inter::{MovimientoBi, RegionB};
use crate::particion::{b_inter_layout, b_sub_parts, b_sub_uses};
use rusty_h264_common::inter::{predict_mv, predict_partition_mv};
use rusty_h264_common::BitReader;

/// Máximo de subparticiones con `mvd` en una lista de un `B_8x8`: 4 particiones de 8x8 × 4
/// subparticiones de 4x4.
const MAX_MVD_B_8X8: usize = 16;

/// Sintaxis leída de un `B_8x8` en CAVLC.
struct SintaxisB8x8 {
    /// `sub_mb_type` de cada partición 8x8 (0 = directa).
    sub: [u32; 4],
    /// `ref_idx` de cada partición 8x8 y lista (−1 = la lista no se usa).
    refi: [[i32; 2]; 4],
    /// `mvd` de la lista 0 de las subparticiones que la usan, en el orden del flujo.
    mvd0: [(i32, i32); MAX_MVD_B_8X8],
    /// `mvd` de la lista 1, igual.
    mvd1: [(i32, i32); MAX_MVD_B_8X8],
}

impl FrameDecoder {
    /// Reads `ref_idx_lX` for a B partition (te(v)/ue(v) by the list's active
    /// count), bounds-checked against the available reference count.
    pub(crate) fn read_b_ref(&self, r: &mut BitReader, list: usize) -> Result<i32, MbError> {
        let (active, avail) = if list == 0 {
            (self.num_ref_active, self.refs.len())
        } else {
            (self.num_ref_active1, self.refs1.len())
        };
        let v = if active > 1 {
            read_ref_idx(r, active)?
        } else {
            0
        };
        if v as usize >= avail {
            return Err(MbError::Truncated);
        }
        Ok(v)
    }

    /// Reconstructs a B macroblock (spec Table 7-14): direct, L0/L1/Bi partitions,
    /// `B_8x8`, or intra.
    pub(crate) fn decode_b_mb(
        &mut self,
        r: &mut BitReader,
        mb_x: usize,
        mb_y: usize,
    ) -> Result<(), MbError> {
        // Non-skip B MBs gather neighbor motion/modes — flush any deferred
        // spans before those grids are read.
        self.span_flush()?;
        let mb_type = r.read_ue()?;
        if mb_type >= MB_TYPE_B_PRIMER_INTRA {
            return self.decode_intra_mb(r, mb_x, mb_y, mb_type - MB_TYPE_B_PRIMER_INTRA);
        }
        if self.refs.is_empty() || self.refs1.is_empty() {
            return Err(MbError::Unsupported("B without references"));
        }
        if mb_type == 0 {
            return self.b_cavlc_directo_16x16(r, mb_x, mb_y);
        }
        if mb_type == MB_TYPE_B_8X8 {
            return self.decode_b_8x8(r, mb_x, mb_y);
        }

        self.b_cavlc_particiones(r, mb_x, mb_y, mb_type)
    }

    /// `B_Direct_16x16` en CAVLC: vectores directos (forzados a cero bipredictivo cuando los tres
    /// vecinos lo son, como en CABAC) y residuo; la transformada 8x8 solo se admite con
    /// `direct_8x8_inference_flag`.
    ///
    /// # Errors
    /// Los de la predicción directa y del residuo.
    fn b_cavlc_directo_16x16(
        &mut self,
        r: &mut BitReader,
        mb_x: usize,
        mb_y: usize,
    ) -> Result<(), MbError> {
        let mut pred_y = [0u8; 256];
        let mut c_pred = [[0u8; 64]; 2];

        // B_Direct_16x16 — 8×8 transform allowed only with direct_8x8_inference.
        // FORCED derivation via the zero-bi bitmap (CAVLC parity with the
        // CABAC direct arm): gather + rid/median skipped, chains extended.
        let addr_f = mb_y * self.mb_w + mb_x;
        let forced = self.direct_spatial
            && mb_x > 0
            && mb_y > 0
            && mb_x + 1 < self.mb_w
            && addr_f - self.mb_w >= self.slice_first_mb
            && self
                .bzero
                .get(addr_f - 1)
                .copied()
                .ok_or(MbError::Inconsistent("bzero"))?
            && self
                .bzero
                .get(addr_f.wrapping_sub(self.mb_w))
                .copied()
                .ok_or(MbError::Inconsistent("bzero"))?
            && self
                .bzero
                .get(addr_f.wrapping_sub(self.mb_w) + 1)
                .copied()
                .ok_or(MbError::Inconsistent("bzero"))?;
        if forced {
            self.b_direct_region(
                RegionB {
                    mb_x,
                    mb_y,
                    px: 0,
                    py: 0,
                    rw: 16,
                    rh: 16,
                },
                &mut pred_y,
                &mut c_pred,
                (0, 0, (0, 0), (0, 0), false),
            )?;
            *self
                .bzero
                .get_mut(addr_f)
                .ok_or(MbError::Inconsistent("bzero"))? = true;
        } else {
            self.decode_b_direct(
                RegionB {
                    mb_x,
                    mb_y,
                    px: 0,
                    py: 0,
                    rw: 16,
                    rh: 16,
                },
                &mut pred_y,
                &mut c_pred,
            )?;
        }
        self.inter_finish(
            r,
            (mb_x, mb_y),
            &pred_y,
            &c_pred,
            self.direct_8x8_inference,
            false,
        )
    }

    /// B 16x16, 16x8 u 8x16 en CAVLC (`mb_type` 1..=21): `ref_idx` y `mvd` de cada partición y
    /// lista (§7.3.5.1), vector predicho, guardado y compensación, y residuo.
    ///
    /// # Errors
    /// Los del flujo, de la predicción y del residuo.
    fn b_cavlc_particiones(
        &mut self,
        r: &mut BitReader,
        mb_x: usize,
        mb_y: usize,
        mb_type: u32,
    ) -> Result<(), MbError> {
        let mut pred_y = [0u8; 256];
        let mut c_pred = [[0u8; 64]; 2];

        // 16x16 / 16x8 / 8x16 partitions with per-partition L0/L1/Bi.
        let (layout, mvmode, preds) = b_inter_layout(mb_type);
        // mb_pred order: ref_idx_l0 (all L0 parts), ref_idx_l1, mvd_l0, mvd_l1.
        let mut refi = [[-1i32; 2]; 2]; // [part][list]
        for (p, &(_, _, _, _)) in layout.iter().enumerate() {
            if preds[p & 1].uses(0) {
                refi[p][0] = self.read_b_ref(r, 0)?;
            }
        }
        for (p, _) in layout.iter().enumerate() {
            if preds[p & 1].uses(1) {
                refi[p][1] = self.read_b_ref(r, 1)?;
            }
        }
        let mut mvd = [[(0i32, 0i32); 2]; 2];
        for (p, _) in layout.iter().enumerate() {
            if preds[p & 1].uses(0) {
                mvd[p][0] = (read_mvd(r)?, read_mvd(r)?);
            }
        }
        for (p, _) in layout.iter().enumerate() {
            if preds[p & 1].uses(1) {
                mvd[p][1] = (read_mvd(r)?, read_mvd(r)?);
            }
        }
        // Per partition: predict + commit each list's MV, then motion-compensate.
        for (p, &(rx, ry, rw, rh)) in layout.iter().enumerate() {
            let (pbx, pby) = ((mb_x * 4 + rx / 4) as isize, (mb_y * 4 + ry / 4) as isize);
            let pwb = (rw / 4) as isize;
            let mut mv = [(0i32, 0i32); 2];
            for list in 0..2 {
                if refi[p][list] >= 0 {
                    let n = self.mv_neighbors_list(pbx, pby, pwb, list)?;
                    let pmv = predict_partition_mv(mvmode, p, n[0], n[1], n[2], refi[p][list]);
                    mv[list] = (pmv.0 + mvd[p][list].0, pmv.1 + mvd[p][list].1);
                }
            }
            let (region, mov) = (
                RegionB {
                    mb_x,
                    mb_y,
                    px: rx,
                    py: ry,
                    rw,
                    rh,
                },
                MovimientoBi {
                    refi0: refi[p & 3][0],
                    mv0: mv[0],
                    refi1: refi[p & 3][1],
                    mv1: mv[1],
                },
            );
            self.b_set_motion(region, mov);
            // Spec-correct bi-prediction (average of L0 and L1), matching the CABAC
            // path. This used to replicate an openh264 bug for a Bi 16x8/8x16
            // partition -- openh264 mis-handles the destination buffer there, so
            // partition 0 came out List-1-only and partition 1 List-0-only. That was
            // deliberate when openh264's h264dec WAS the conformance oracle, but the
            // gate is ffmpeg now and the CABAC path already went spec-correct; the
            // CAVLC path was simply left behind. Measured: mb_type 12..21 (every B
            // 16x8/8x16 with at least one Bi partition) were 100% wrong vs ffmpeg,
            // while 1..11 (no Bi partition) were only collaterally damaged.
            self.b_mc_or_record(region, mov, &mut pred_y, &mut c_pred)?;
        }
        self.inter_finish(r, (mb_x, mb_y), &pred_y, &c_pred, true, false)
    }

    /// Reconstructs a `B_8x8` macroblock: four 8×8 sub-macroblock partitions, each
    /// direct or L0/L1/Bi with its own sub-partitioning (spec Table 7-18).
    pub(crate) fn decode_b_8x8(
        &mut self,
        r: &mut BitReader,
        mb_x: usize,
        mb_y: usize,
    ) -> Result<(), MbError> {
        let SintaxisB8x8 {
            sub,
            refi,
            mvd0,
            mvd1,
        } = self.leer_sintaxis_b_8x8(r)?;
        let mut pred_y = [0u8; 256];
        let mut c_pred = [[0u8; 64]; 2];
        // Decode each 8×8 partition.
        // Spatial-direct A/B/C are MB-level — walk once if any sub is direct.
        // `dmemo=0` rewalks every direct 8×8 (A/B oracle).
        let hoisted = if self.direct_spatial && sub.contains(&0) {
            Some(self.b_direct_nbrs(mb_x, mb_y)?)
        } else {
            None
        };
        let (mut i0, mut i1) = (0usize, 0usize);
        for (p, &st) in sub.iter().enumerate() {
            let (b8x, b8y) = ((p % 2) * 8, (p / 2) * 8);
            if st == 0 {
                match hoisted {
                    Some((n0, n1)) => self.decode_b_direct_n(
                        RegionB {
                            mb_x,
                            mb_y,
                            px: b8x,
                            py: b8y,
                            rw: 8,
                            rh: 8,
                        },
                        &mut pred_y,
                        &mut c_pred,
                        (n0, n1),
                    )?,
                    None => self.decode_b_direct(
                        RegionB {
                            mb_x,
                            mb_y,
                            px: b8x,
                            py: b8y,
                            rw: 8,
                            rh: 8,
                        },
                        &mut pred_y,
                        &mut c_pred,
                    )?,
                }
                continue;
            }
            for &(sx, sy, sw, sh) in b_sub_parts(st) {
                let (px, py) = (b8x + sx, b8y + sy);
                let (pbx, pby) = ((mb_x * 4 + px / 4) as isize, (mb_y * 4 + py / 4) as isize);
                let pwb = (sw / 4) as isize;
                let mut mv = [(0i32, 0i32); 2];
                if b_sub_uses(st, 0) {
                    let n = self.mv_neighbors_list(pbx, pby, pwb, 0)?;
                    let pmv = predict_mv(n[0], n[1], n[2], refi[p & 3][0]);
                    let d = mvd0[i0 & 15];
                    i0 += 1;
                    mv[0] = (pmv.0 + d.0, pmv.1 + d.1);
                }
                if b_sub_uses(st, 1) {
                    let n = self.mv_neighbors_list(pbx, pby, pwb, 1)?;
                    let pmv = predict_mv(n[0], n[1], n[2], refi[p & 3][1]);
                    let d = mvd1[i1 & 15];
                    i1 += 1;
                    mv[1] = (pmv.0 + d.0, pmv.1 + d.1);
                }
                let (region, mov) = (
                    RegionB {
                        mb_x,
                        mb_y,
                        px,
                        py,
                        rw: sw,
                        rh: sh,
                    },
                    MovimientoBi {
                        refi0: refi[p & 3][0],
                        mv0: mv[0],
                        refi1: refi[p & 3][1],
                        mv1: mv[1],
                    },
                );
                self.b_set_motion(region, mov);
                self.b_mc_or_record(region, mov, &mut pred_y, &mut c_pred)?;
            }
        }
        // noSubMbPartSizeLessThan8x8: each sub-partition must be ≥ 8×8 (direct
        // counts only with the 8×8 inference flag).
        let allow_8x8 = sub.iter().all(|&st| {
            if st == 0 {
                self.direct_8x8_inference
            } else {
                st <= 3
            }
        });
        self.inter_finish(r, (mb_x, mb_y), &pred_y, &c_pred, allow_8x8, false)
    }

    /// Sintaxis de un `B_8x8` en CAVLC (§7.3.5.2): los cuatro `sub_mb_type`, los `ref_idx` de las
    /// subparticiones no directas (primero la lista 0, luego la 1) y sus `mvd` en el orden del
    /// flujo.
    ///
    /// # Errors
    /// `MbError::Unsupported` si un `sub_mb_type` pasa de 12, y los de lectura del flujo.
    fn leer_sintaxis_b_8x8(&self, r: &mut BitReader) -> Result<SintaxisB8x8, MbError> {
        let mut sub = [0u32; 4];
        for s in sub.iter_mut() {
            let v = r.read_ue()?;
            if v > MAX_SUB_MB_TYPE_B {
                return Err(MbError::Unsupported("invalid B sub_mb_type"));
            }
            *s = v;
        }
        // ref_idx for all 8×8 partitions (L0 batch, then L1 batch), for the
        // non-direct sub-partitions.
        let mut refi = [[-1i32; 2]; 4];
        for (p, &st) in sub.iter().enumerate() {
            if st != 0 && b_sub_uses(st, 0) {
                refi[p][0] = self.read_b_ref(r, 0)?;
            }
        }
        for (p, &st) in sub.iter().enumerate() {
            if st != 0 && b_sub_uses(st, 1) {
                refi[p][1] = self.read_b_ref(r, 1)?;
            }
        }
        // mvd: all mvd_l0 (partition-major, sub-partition order), then all mvd_l1.
        //
        // FIXED ARRAYS, NOT `Vec::new()` + push. These are per-MACROBLOCK on every
        // B_8x8, and a growing Vec allocated (and reallocated) twice per MB — on a
        // B-heavy stream that is thousands of allocations per frame for data whose
        // maximum size is a compile-time constant: 4 partitions x at most 4
        // sub-partitions = 16 entries. Indexing a fixed array cannot exceed that, and
        // an out-of-range index would panic rather than misbehave, so the bound is
        // enforced either way and this crate stays forbid(unsafe).
        let mut mvd0 = [(0i32, 0i32); MAX_MVD_B_8X8];
        let mut mvd1 = [(0i32, 0i32); MAX_MVD_B_8X8];
        let (mut n0, mut n1) = (0usize, 0usize);
        for &st in &sub {
            if st != 0 && b_sub_uses(st, 0) {
                for _ in b_sub_parts(st) {
                    mvd0[n0] = (read_mvd(r)?, read_mvd(r)?);
                    n0 += 1;
                }
            }
        }
        for &st in &sub {
            if st != 0 && b_sub_uses(st, 1) {
                for _ in b_sub_parts(st) {
                    mvd1[n1] = (read_mvd(r)?, read_mvd(r)?);
                    n1 += 1;
                }
            }
        }
        Ok(SintaxisB8x8 {
            sub,
            refi,
            mvd0,
            mvd1,
        })
    }
}
