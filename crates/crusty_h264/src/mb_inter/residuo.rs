//! Residuo de un macrobloque inter sumado a su predicción (§8.5).

use crate::constantes::desescanear_8x8 as un_scan_8x8;
use crate::edc::{ZERO_CAC, ZERO_LUMA_SCAN};
use crate::mb16::FrameDecoder;
use crusty_h264_common::predict::{
    add_residual_8x8, reconstruct_4x4_dc_into, reconstruct_4x4_scan_into, CHROMA_4X4_SCAN_XY,
    LUMA_4X4_SCAN_XY,
};

/// Residuo analizado de un macrobloque inter, listo para sumarse a su predicción. Un campo `None`
/// significa que el análisis no escribió nada ahí: se lee como el plano cero compartido.
#[derive(Clone, Copy)]
pub(crate) struct ResiduoInter<'a> {
    /// Luma 4x4 en orden de barrido.
    pub(crate) luma_scan: Option<&'a [[i32; 16]; 16]>,
    /// `Some` when the macroblock carries transform_size_8x8_flag: four 8x8
    /// blocks in 8x8 scan order, replacing the sixteen 4x4 luma blocks.
    pub(crate) luma8: Option<&'a [[i32; 64]; 4]>,
    /// DC de croma de las dos componentes.
    pub(crate) cdc: &'a [[i32; 4]; 2],
    /// AC de croma de las dos componentes.
    pub(crate) cac: Option<&'a [[[i32; 16]; 4]; 2]>,
    /// `CodedBlockPatternChroma` (0, 1 o 2).
    pub(crate) cbp_chroma: u32,
    /// Parsed totalCoeff per block, indexed exactly as the parse's `iz`:
    /// [0..16] luma 4x4 z-order (for t8, the 8x8 count sits at `id8*4`),
    /// [16..24] chroma AC as `16 + c*4 + id4`. The parser already counted
    /// every significant coefficient; re-deriving the counts here scanned
    /// 16-64 array elements per block (~400 loads/MB) for information the
    /// caller was holding — the diagnosis's stage-boundary re-derivation tax.
    pub(crate) nnzs: &'a [u8; 24],
}

impl FrameDecoder {
    /// Add a CABAC-parsed inter residual to an already-built motion-comp prediction
    /// (`pred_y`/`c_pred`), writing the reconstruction. Shared by the P and B inter
    /// paths — same `reconstruct_4x4` as intra, MC output as the prediction, inter
    /// scaling lists (luma 3 / chroma 4+c). `luma_scan[z]`/`cdc`/`cac` are the
    /// scan-order coefficients; uncoded blocks are zero so recon == prediction.
    pub(crate) fn add_inter_residual(
        &mut self,
        mb_x: usize,
        mb_y: usize,
        pred_y: &[u8; 256],
        c_pred: &[[u8; 64]; 2],
        res: ResiduoInter<'_>,
    ) {
        let ResiduoInter {
            luma_scan,
            luma8,
            cdc,
            cac,
            cbp_chroma,
            nnzs,
        } = res;
        // A `None` field means the parse wrote nothing there; every read below
        // is guarded by a zero-count test, so the shared zero plane is
        // read-equivalent to the per-macroblock zeroed stack array it replaces.
        let luma_scan = luma_scan.unwrap_or(&ZERO_LUMA_SCAN);
        let cac = cac.unwrap_or(&ZERO_CAC);
        let qp = self.cur_qp;
        if let Some(l8) = luma8 {
            self.residuo_luma_8x8((mb_x, mb_y), pred_y, l8, nnzs, qp);
        } else {
            self.residuo_luma_4x4((mb_x, mb_y), pred_y, luma_scan, nnzs, qp);
        }
        self.residuo_croma((mb_x, mb_y), c_pred, (cdc, cac), cbp_chroma, nnzs);
    }

    /// Luma con `transform_size_8x8_flag`: cuatro bloques 8x8 (desescaneo, cuantificación inversa
    /// con la lista inter 8x8, IDCT y suma a `pred_y`), con sus recuentos en `nnz_y` y `coded_y`.
    fn residuo_luma_8x8(
        &mut self,
        (mb_x, mb_y): (usize, usize),
        pred_y: &[u8; 256],
        l8: &[[i32; 64]; 4],
        nnzs: &[u8; 24],
        qp: u8,
    ) {
        let w4r = self.mb_w * 4;
        // INTER 8x8 luma: same primitives the I_8x8 and CAVLC paths use.
        for b8 in 0..4usize {
            let (b8x, b8y) = (b8 % 2, b8 / 2);
            // PER-CELL, not one aggregate broadcast over all four cells. CAVLC
            // codes an 8x8 block as four 4x4 sub-blocks and its nC predictor
            // reads these per-4x4 counts from `nnz_y`, so the broadcast
            // corrupted the NEXT macroblock's nC and desynced the parse -- which
            // is why CAVLC 8x8 streams ffmpeg accepts would not decode here. The
            // worker copy of this function never wrote `nnz_y` at all, so the
            // threaded path was unaffected and hid the defect. CABAC has no
            // per-4x4 counts, so its callers put the 8x8 total in all four slots.
            let nnz: u32 = (0..4).map(|k| nnzs[b8 * 4 + k] as u32).sum();
            // Two row slices, not four whole-grid indexed stores. The pair
            // written per row is contiguous and `nnzs` is a fixed 24-entry
            // array, so both sides become provable.
            for sy in 0..2 {
                let base = (mb_y * 4 + b8y * 2 + sy) * w4r + mb_x * 4 + b8x * 2;
                self.nnz_y[base..][..2].copy_from_slice(&nnzs[b8 * 4 + sy * 2..][..2]);
            }
            // The 4x4 inter path marks coded_y per block; the 8x8 branch must too,
            // or a later intra macroblock's neighbour availability is wrong.
            for sy in 0..2 {
                let base = (mb_y * 4 + b8y * 2 + sy) * w4r + mb_x * 4 + b8x * 2;
                self.coded_y[base..][..2].fill(true);
            }
            let (px, py) = (mb_x * 16 + b8x * 8, mb_y * 16 + b8y * 8);
            if nnz == 0 {
                // Zero residual: recon == pred — the 4x4 arm zero shortcut,
                // ported to the 8x8 branch.
                for dy in 0..8 {
                    let d = (py + dy) * self.cw + px;
                    let po = (b8y * 8 + dy) * 16 + b8x * 8;
                    self.rec_y[d..d + 8].copy_from_slice(&pred_y[po..po + 8]);
                }
            } else {
                let raster = un_scan_8x8(&l8[b8]);
                // list 1 = INTER 8x8 luma scaling list (0 is the intra one).
                let res8 = self.inv_quant8(&raster, qp, 1);
                let predb: [i32; 64] = core::array::from_fn(|i| {
                    pred_y[(b8y * 8 + i / 8) * 16 + (b8x * 8 + i % 8)] as i32
                });
                let recon = add_residual_8x8(&res8, &predb);
                // Eight row copies. This wrote SIXTY-FOUR individually
                // bounds-checked samples into the luma plane per coded 8x8
                // block — the same shape whose fix carried `recon_i8_block`.
                // Present in BOTH the main path and the EDC worker twin.
                for dy in 0..8 {
                    let d = (py + dy) * self.cw + px;
                    self.rec_y[d..][..8].copy_from_slice(&recon[dy * 8..][..8]);
                }
            }
        }
    }

    /// Luma 4x4: cada bloque se copia de la predicción (sin coeficientes), suma un DC plano o pasa
    /// por la IDCT; recuentos en `nnz_y`.
    fn residuo_luma_4x4(
        &mut self,
        (mb_x, mb_y): (usize, usize),
        pred_y: &[u8; 256],
        luma_scan: &[[i32; 16]; 16],
        nnzs: &[u8; 24],
        qp: u8,
    ) {
        let w4r = self.mb_w * 4;
        // RESIDUAL LADDER, then the SIMD IDCT+add kernel per parked block (SIMD census
        // 2026-09-05). The zero and DC-only ladders write the plane in the loop;
        // every other coded block parks its dequantised coefficients here and is
        // reconstructed by `reconstruct_4x4_into` = accel `idct4x4_add` (in-register
        // transposes, exact on i32). A lane-per-block BATCHED form was tried first
        // and lost on all-intra content: its scalar gathers cost more than the
        // butterflies they vectorised.
        let dq3 = self.dq_const(qp, 3);
        for (blk, &(lbx, lby)) in LUMA_4X4_SCAN_XY.iter().enumerate() {
            let nnz = nnzs[blk];
            if let Some(p) = self
                .nnz_y
                .get_mut((mb_y * 4 + lby) * w4r + (mb_x * 4 + lbx))
            {
                *p = nnz;
            }
            let cw = self.cw;
            let p_off = (lby * 4) * 16 + lbx * 4;
            let r_off = (mb_y * 4 + lby) * 4 * cw + (mb_x * 4 + lbx) * 4;
            if nnz == 0 {
                // Zero residual → recon == prediction EXACTLY (the integer IDCT is
                // linear so zeros map to zeros, and pred is already 0..=255) — copy
                // the pred rows straight into the plane. On real (sparse-cbp)
                // streams this is MOST of the 4×4 blocks.
                for r in 0..4 {
                    self.rec_y[r_off + r * cw..r_off + r * cw + 4]
                        .copy_from_slice(&pred_y[p_off + r * 16..p_off + r * 16 + 4]);
                }
                continue;
            }
            // DC-ONLY: the sole significant coefficient is scan position 0 (the
            // zig-zag starts at DC, and un_scan keeps it at raster 0), so the
            // whole dequant + IDCT collapses to one multiply and a flat add.
            if nnz == 1 && luma_scan[blk][0] != 0 {
                let f = self.dequant_dc4(luma_scan[blk][0], qp, 3);
                reconstruct_4x4_dc_into(
                    (f + 32) >> 6,
                    pred_y,
                    p_off,
                    16,
                    &mut self.rec_y,
                    r_off,
                    cw,
                );
            } else {
                // Fused un-scan + dequant over ONLY the significant coefficients,
                // then IDCT + add + clip straight into the plane — no `qb`, no
                // `deq`-from-dense, no `predb` gather, no `s`, no `store` call.
                //
                // HYBRID: the scatter walks scan positions with a data-dependent
                // branch per slot, which beats the branchless dense 16-multiply
                // loop only while the block is SPARSE. The DC/zero fast paths
                // already removed the sparsest blocks, so the population here
                // skews denser — above ~6 coefficients the dense loop wins.
                // FUSED: un-scan + dequant + IDCT + add in one kernel call (dense-over-scatter round).
                reconstruct_4x4_scan_into::<false>(
                    &luma_scan[blk],
                    &dq3,
                    0,
                    pred_y,
                    p_off,
                    16,
                    &mut self.rec_y,
                    r_off,
                    cw,
                );
            }
        }
    }

    /// Croma de las dos componentes: DC descuantificado y, con `cbp_chroma` 2, AC; recuentos en
    /// `nnz_c`.
    fn residuo_croma(
        &mut self,
        (mb_x, mb_y): (usize, usize),
        c_pred: &[[u8; 64]; 2],
        (cdc, cac): (&[[i32; 4]; 2], &[[[i32; 16]; 4]; 2]),
        cbp_chroma: u32,
        nnzs: &[u8; 24],
    ) {
        let qpc = self.chroma_qp_for(self.cur_qp);
        let w2r = self.mb_w * 2;
        let mut c_dc = [[0i32; 4]; 2];
        if cbp_chroma != 0 {
            for c in 0..2 {
                c_dc[c] = self.dequant_chroma_dc(&cdc[c], qpc, 4 + c);
            }
        }
        let dqc = [self.dq_const(qpc, 4), self.dq_const(qpc, 5)];
        let ccw = self.ccw;
        for c in 0..2 {
            for &(bx, by) in &CHROMA_4X4_SCAN_XY {
                let mut ac_nz = false;
                if cbp_chroma == 2 {
                    let n = nnzs[(16 + c * 4 + by * 2 + bx).min(23)];
                    if let Some(p) =
                        self.nnz_c[c & 1].get_mut((mb_y * 2 + by) * w2r + (mb_x * 2 + bx))
                    {
                        *p = n;
                    }
                    ac_nz = n != 0;
                }
                let dc = c_dc[c & 1][(by * 2 + bx) & 3];
                let p_off = (by * 4) * 8 + bx * 4;
                let r_off = (mb_y * 2 + by) * 4 * ccw + (mb_x * 2 + bx) * 4;
                let plane = if c == 0 {
                    &mut self.rec_u
                } else {
                    &mut self.rec_v
                };
                if dc == 0 && !ac_nz {
                    // Zero residual (no AC, zero DC) → recon == prediction exactly.
                    for r in 0..4 {
                        plane[r_off + r * ccw..r_off + r * ccw + 4]
                            .copy_from_slice(&c_pred[c][p_off + r * 8..p_off + r * 8 + 4]);
                    }
                    continue;
                }
                // DC-ONLY (no coded AC — covers every cbp_chroma==1 block and the
                // AC-empty blocks of cbp_chroma==2): the chroma DC arrives ALREADY
                // dequantized, so the residual is `(dc + 32) >> 6` flat.
                if !ac_nz {
                    reconstruct_4x4_dc_into(
                        (dc + 32) >> 6,
                        &c_pred[c],
                        p_off,
                        8,
                        plane,
                        r_off,
                        ccw,
                    );
                    continue;
                }
                // AC-only scan: index i is overall scan position i+1 (ac_shift=1).
                // Same sparse/dense hybrid as luma.
                reconstruct_4x4_scan_into::<true>(
                    &cac[c & 1][(by * 2 + bx) & 3],
                    &dqc[c & 1],
                    dc,
                    &c_pred[c],
                    p_off,
                    8,
                    plane,
                    r_off,
                    ccw,
                );
            }
        }
    }
}
