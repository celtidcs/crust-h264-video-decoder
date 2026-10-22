//! Macrobloques de los cortes B en CABAC: salto (`B_Skip`), directo e inter (UIT-T H.264 §7.3.5,
//! §8.4.1.2, §9.3).

use alloc::boxed::Box;

use rusty_h264_common::cabac_tables::{CACHE30, G_SCAN4};
use rusty_h264_common::inter::{predict_mv, predict_partition_mv};
use rusty_h264_common::predict::nnz_raster_from_z;

use super::estado::{CorteCabac, PosicionMb, RejillasCorte, ResultadoMb};
use super::{
    cache_nzc_vecinos, cond_term_flag, valor_vecino, CTX_MB_SKIP_B, CTX_MB_TYPE_B,
    CTX_TRANSFORM_8X8,
};
use crate::cabac_residuo::{parse_mb_residual_cabac, ResidualOut};
use crate::error::MbError;
use crate::mb16::FrameDecoder;
use crate::mb_cabac::{
    mb_type_b_eng, parse_cbp_cabac, parse_mb_qp_delta_cabac, parse_mvd_partition,
    parse_ref_idx_cabac, parse_sub_mb_type_b_cabac,
};
use crate::mb_inter::ResiduoInter;
use crate::mb_inter::{MovimientoBi, RegionB, MB_TYPE_B_PRIMER_INTRA};
use crate::particion::{b_inter_layout, b_sub_parts, b_sub_uses};

/// Cachés de contexto y movimiento de las dos listas de un macrobloque B inter: las de 30
/// entradas con los vecinos (`ctxIdxInc` de `ref_idx` y `mvd`, §9.3.3.1.1.6–7) y las de 16 con
/// lo que deja este macrobloque para las rejillas.
pub(super) struct MovimientoB {
    /// `mvd` de la lista 0 en la caché de vecinos.
    pub(super) mvdc0: [[i16; 2]; 30],
    /// `ref_idx` de la lista 0 en la caché de vecinos.
    pub(super) refc0: [i8; 30],
    /// `mvd` de la lista 1 en la caché de vecinos.
    pub(super) mvdc1: [[i16; 2]; 30],
    /// `ref_idx` de la lista 1 en la caché de vecinos.
    pub(super) refc1: [i8; 30],
    /// `mvd` de la lista 0 de cada bloque 4x4 del macrobloque.
    pub(super) mmvd0: [[i16; 2]; 16],
    /// `ref_idx` de la lista 0 de cada bloque 4x4 (−1 = sin lista 0).
    pub(super) mref0: [i8; 16],
    /// `mvd` de la lista 1 de cada bloque 4x4.
    pub(super) mmvd1: [[i16; 2]; 16],
    /// `ref_idx` de la lista 1 de cada bloque 4x4 (−1 = sin lista 1).
    pub(super) mref1: [i8; 16],
}

impl FrameDecoder {
    /// Un macrobloque de un corte B: `mb_skip_flag` y, si no se salta, `mb_type`; los inter se
    /// decodifican aquí y los intra se devuelven como [`ResultadoMb::Intra`] (UIT-T H.264 §7.3.4,
    /// §7.3.5, §9.3).
    ///
    /// # Errors
    /// Los de la decodificación del macrobloque.
    pub(super) fn mb_cabac_b(
        &mut self,
        corte: &mut CorteCabac<'_>,
        pos: PosicionMb,
    ) -> Result<ResultadoMb, MbError> {
        let CorteCabac {
            cab,
            last_delta_qp,
            rejillas,
            ..
        } = corte;
        let RejillasCorte {
            mb_skip, mb_direct, ..
        } = rejillas;
        let PosicionMb {
            addr,
            mbx,
            mby,
            left,
            top,
        } = pos;

        // NOTE: the per-macroblock edc_flush() that used to sit here is
        // hoisted to slice entry (see below) — nothing inside a B slice
        // pushes to edc_jobs.
        // B-slice: mb_skip_flag (ctx 24 + neighbour-not-skip), then B mb_type.
        let sctx = CTX_MB_SKIP_B
            + cond_term_flag(mb_skip, left, |b| !b, "mb_skip")?
            + cond_term_flag(mb_skip, top, |b| !b, "mb_skip")?;
        // ONE engine view carries mb_skip_flag, the terminate bin of a skipped
        // macroblock, and mb_type of a coded one (was 2-3 view/commit round
        // trips of 8 memory ops each per macroblock).
        let (data, mut e, ctx) = cab.view();
        if e.decode_decision(data, ctx, sctx) != 0 {
            if let (Some(sk), Some(di)) = (mb_skip.get_mut(addr), mb_direct.get_mut(addr)) {
                (*sk, *di) = (true, true);
            }
            *last_delta_qp = 0; // skip codes no mb_qp_delta → delta ctxInc resets
                                // B_Skip recon reuses the entropy-free CAVLC primitive (spatial/temporal
                                // direct with no residual), which also commits the motion grid.
                                // Hot prefix inline: forced run continuations skip the call.
            if !self.b_skip_hot(mbx, mby)? {
                self.decode_b_skip(mbx, mby)?;
            }
            *self
                .mb_qp
                .get_mut(addr)
                .ok_or(MbError::Inconsistent("mb_qp"))? = self.cur_qp;
            // Skip/direct blocks contribute mvd 0 to a later MB's mvd ctxInc; the
            // ref stays in-list so |mvd|=0 is summed (same result either way).
            // mb_ref/mb_ref1 stay at their -1 init: every reader is
            // either a `> 0` context test (-1 and 0 both false) or the
            // mvd-sum's `>= 0` gate — and a skip's mvd is (0,0), so
            // exclusion (-1) and inclusion-of-zero (0) give the same
            // sum. The 64-byte per-skip zero-fill was pure waste.
            let fin_de_corte = e.decode_terminate(data);
            cab.commit(e);
            return Ok(ResultadoMb::Terminado(fin_de_corte));
        }
        // NON-SKIP B MB: every arm below reads the grids INLINE
        // (H-48: CABAC never routes through decode_b_mb) — flush
        // the deferred spans. Caught by the tempete ARM-DIFF.
        self.span_flush()?;
        let bci = cond_term_flag(mb_direct, left, |b| !b, "mb_direct")?
            + cond_term_flag(mb_direct, top, |b| !b, "mb_direct")?;
        let bmt = { mb_type_b_eng(&mut e, data, ctx, CTX_MB_TYPE_B, bci) };
        cab.commit(e);
        if bmt < MB_TYPE_B_PRIMER_INTRA {
            let allow8 = self.b_inter_movimiento(corte, pos, bmt)?;
            return self
                .b_inter_residuo(corte, pos, allow8)
                .map(ResultadoMb::Terminado);
        }
        // 23→0 (I_4x4), 24..=47→1..24 (I_16x16), 48→25 (I_PCM)
        Ok(ResultadoMb::Intra(bmt - MB_TYPE_B_PRIMER_INTRA))
    }
    ///
    /// Movimiento de un B inter (`mb_type` 0..=22: directo 16x16, 16x16/16x8/8x16 con sus listas y
    /// `B_8x8`): `ref_idx` y `mvd` de las dos listas (§7.3.5.1, §7.3.5.2), vectores predichos o
    /// directos (§8.4.1.2), compensación de movimiento y guardado en las rejillas. Devuelve si el
    /// macrobloque admite `transform_size_8x8_flag` (`noSubMbPartSizeLessThan8x8Flag`).
    ///
    /// # Errors
    /// Los de la decodificación del macrobloque.
    pub(super) fn b_inter_movimiento(
        &mut self,
        corte: &mut CorteCabac<'_>,
        pos: PosicionMb,
        bmt: u32,
    ) -> Result<bool, MbError> {
        let CorteCabac {
            b_refs_ok,
            rejillas,
            ..
        } = corte;
        let RejillasCorte {
            mb_ref,
            mb_mvd,
            mb_ref1,
            mb_mvd1,
            ..
        } = rejillas;
        let b_refs_ok = *b_refs_ok;
        let PosicionMb {
            addr,
            mbx,
            mby,
            left,
            top,
        } = pos;
        let mbw = self.mb_w;

        // noSubMbPartSizeLessThan8x8Flag for B: direct MBs qualify only under
        // direct_8x8_inference_flag; B_8x8 needs every sub-partition 8x8.
        let mut allow8 = true;
        // ---- B inter: parse motion (mvd L0/L1; ref not coded on this 1-ref
        // stream) + residual. Recon (b_mc/direct) deferred to B.3. ----
        let mut mvdc0 = [[0i16; 2]; 30];
        let mut refc0 = [-1i8; 30];
        let mut mvdc1 = [[0i16; 2]; 30];
        let mut refc1 = [-1i8; 30];
        // WelsFillCacheInterCabac, per list (L0 = mb_ref/mb_mvd, L1 = mb_ref1/mb_mvd1).
        // The corner addresses and their guards are macroblock
        // properties, resolved ONCE here rather than rebuilt inside
        // each list expansion; each neighbour grid row is borrowed
        // once instead of re-indexed per entry.
        let tl = (mbx > 0 && mby > 0).then(|| addr - mbw - 1);
        let tr = (mby > 0 && mbx + 1 < mbw).then(|| addr - mbw + 1);
        macro_rules! fill {
            ($mrf:expr, $mmv:expr, $rc:expr, $mc:expr) => {{
                // `.get` PER PARALLEL GRID. The ref and mvd grids are
                // separate Vecs indexed by the same macroblock address,
                // so `$mrf[l]` proved nothing about `$mmv[l]` and each
                // of the four neighbour slots carried two panic paths —
                // doubled again because this macro expands once per
                // list. A `None` here degrades to exactly what the
                // cache already means by "neighbour unavailable" (the
                // `-1` ref it is initialised to), so the fallible form
                // is the honest one as well as the cheap one.
                if let Some(l) = left {
                    if let (Some(rr), Some(mm)) = ($mrf.get(l), $mmv.get(l)) {
                        for (ci, bi) in [(6usize, 3usize), (12, 7), (18, 11), (24, 15)] {
                            $rc[ci] = rr[bi];
                            $mc[ci] = mm[bi];
                        }
                    }
                }
                if let Some(t) = top {
                    if let (Some(rr), Some(mm)) = ($mrf.get(t), $mmv.get(t)) {
                        for (ci, bi) in [(1usize, 12usize), (2, 13), (3, 14), (4, 15)] {
                            $rc[ci] = rr[bi];
                            $mc[ci] = mm[bi];
                        }
                    }
                }
                if let Some(a) = tl {
                    if let (Some(rr), Some(mm)) = ($mrf.get(a), $mmv.get(a)) {
                        ($rc[0], $mc[0]) = (rr[15], mm[15]);
                    }
                }
                if let Some(a) = tr {
                    if let (Some(rr), Some(mm)) = ($mrf.get(a), $mmv.get(a)) {
                        ($rc[5], $mc[5]) = (rr[12], mm[12]);
                    }
                }
            }};
        }
        {
            fill!(mb_ref, mb_mvd, refc0, mvdc0);
            fill!(mb_ref1, mb_mvd1, refc1, mvdc1);
        }
        if !b_refs_ok {
            return Err(MbError::Unsupported("B without references"));
        }
        // Recon (mirrors CAVLC decode_b_mb / decode_b_8x8): predict each list's
        // MV off the committed grid + the CABAC-parsed mvd, commit, MC (bi-pred
        // blend), then add the residual. Prediction reads mmvd0/mmvd1 (the mvd
        // per raster block, splatted during the parse above).
        // pred_y / c_pred are slice-scope scratch (see the loop head).
        // Recording is a per-macroblock property — ask once, not per
        // partition, so the 1T path calls b_mc straight through.

        let mut m = MovimientoB {
            mvdc0,
            refc0,
            mvdc1,
            refc1,
            mmvd0: [[0i16; 2]; 16],
            mref0: [-1i8; 16],
            mmvd1: [[0i16; 2]; 16],
            mref1: [-1i8; 16],
        };
        if bmt == 0 {
            allow8 = self.b_directo_16x16(corte, pos, &mut m)?;
        } else if bmt == 22 {
            allow8 = self.b_8x8(corte, pos, &mut m)?;
        } else {
            self.b_particiones(corte, pos, &mut m, bmt)?;
        }
        // Paired stores: one index expression per grid pair.
        // Four per-macroblock grids at one address; each is its own
        // Vec, so each assignment carried its own check.
        let r = &mut corte.rejillas;
        if let (Some(r0), Some(d0)) = (r.mb_ref.get_mut(addr), r.mb_mvd.get_mut(addr)) {
            (*r0, *d0) = (m.mref0, m.mmvd0);
        }
        if let (Some(r1), Some(d1)) = (r.mb_ref1.get_mut(addr), r.mb_mvd1.get_mut(addr)) {
            (*r1, *d1) = (m.mref1, m.mmvd1);
        }
        Ok(allow8)
    }

    /// `B_Direct_16x16`: sin movimiento codificado; vectores directos (§8.4.1.2) y
    /// compensación. Devuelve `noSubMbPartSizeLessThan8x8Flag` (`direct_8x8_inference_flag`).
    ///
    /// # Errors
    /// Los de la decodificación del macrobloque.
    pub(super) fn b_directo_16x16(
        &mut self,
        corte: &mut CorteCabac<'_>,
        pos: PosicionMb,
        m: &mut MovimientoB,
    ) -> Result<bool, MbError> {
        let CorteCabac {
            pred_y,
            c_pred,
            rejillas,
            ..
        } = corte;
        let RejillasCorte { mb_direct, .. } = rejillas;
        let PosicionMb { addr, mbx, mby, .. } = pos;
        let mbw = self.mb_w;

        let MovimientoB { mref0, mref1, .. } = m;
        // B_Direct_16x16: no coded motion. A direct block contributes mvd 0
        // to a later MB's mvd ctxInc with its ref in-list (|0| summed).
        *mb_direct
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_direct"))? = true;
        let allow8 = self.direct_8x8_inference;
        (*mref0, *mref1) = ([0i8; 16], [0i8; 16]);
        // FORCED derivation via the zero-bi bitmap (same triple
        // as b_skip_hot): a direct-16 whose left/top/topright are
        // recorded ref0/(0,0) committers derives (0,0)/(0,0) bi —
        // skip the gather + derivation, run the region half
        // directly. Its own commit is zero-bi too, so it EXTENDS
        // forcing chains through coded direct MBs.
        // `mbw` is already carried by the loop, and the row-above
        // address is one subtraction, not three.
        let up = addr.wrapping_sub(mbw);
        let forced = mbx > 0 && mby > 0 && mbx + 1 < mbw && up >= self.slice_first_mb && {
            // Un corte que TERMINA en `addr` cubre las tres
            // lecturas de vecinos: las guardas de arriba las dejan
            // todas por debajo de `addr`; un fallo es un estado
            // incoherente (COR-3). Misma forma que `b_skip_hot`.
            let bz = self
                .bzero
                .get(..=addr)
                .ok_or(MbError::Inconsistent("bzero"))?;
            bz.get(addr - 1)
                .copied()
                .ok_or(MbError::Inconsistent("bzero"))?
                && bz.get(up).copied().ok_or(MbError::Inconsistent("bzero"))?
                && bz
                    .get(up + 1)
                    .copied()
                    .ok_or(MbError::Inconsistent("bzero"))?
        };
        if forced {
            self.b_direct_region(
                RegionB {
                    mb_x: mbx,
                    mb_y: mby,
                    px: 0,
                    py: 0,
                    rw: 16,
                    rh: 16,
                },
                pred_y,
                c_pred,
                (0, 0, (0, 0), (0, 0), false),
            )?;
            *self
                .bzero
                .get_mut(addr)
                .ok_or(MbError::Inconsistent("bzero"))? = true;
        } else {
            self.decode_b_direct(
                RegionB {
                    mb_x: mbx,
                    mb_y: mby,
                    px: 0,
                    py: 0,
                    rw: 16,
                    rh: 16,
                },
                pred_y,
                c_pred,
            )?;
        }
        Ok(allow8)
    }

    /// `B_8x8`: cuatro `sub_mb_type`, `ref_idx` por 8x8 de cada lista, `mvd` por subpartición
    /// (§7.3.5.2) y reconstrucción por subpartición (directa o predicha). Devuelve
    /// `noSubMbPartSizeLessThan8x8Flag`.
    ///
    /// # Errors
    /// Los de la decodificación del macrobloque.
    pub(super) fn b_8x8(
        &mut self,
        corte: &mut CorteCabac<'_>,
        pos: PosicionMb,
        m: &mut MovimientoB,
    ) -> Result<bool, MbError> {
        let CorteCabac {
            cab,
            pred_y,
            c_pred,
            ..
        } = corte;
        let PosicionMb { mbx, mby, .. } = pos;

        let MovimientoB {
            mvdc0,
            refc0,
            mvdc1,
            refc1,
            mmvd0,
            mref0,
            mmvd1,
            mref1,
            ..
        } = m;
        // B_8x8: 4 sub_mb_types, (ref not coded on 1-ref), then mvd
        // list-major → sub-MB → sub-partition (openh264 order).
        let mut subt = [0u32; 4];
        for s in &mut subt {
            *s = parse_sub_mb_type_b_cabac(cab);
        }
        let d8i = self.direct_8x8_inference;
        let allow8 = subt
            .iter()
            .all(|&t| if t == 0 { d8i } else { (1..=3).contains(&t) });
        // uses(list) depends only on the sub_mb_type — resolve the
        // whole 4x2 table once here; the ref_idx, mvd and recon
        // loops below all read it instead of re-asking.
        let su = [
            [b_sub_uses(subt[0], 0), b_sub_uses(subt[0], 1)],
            [b_sub_uses(subt[1], 0), b_sub_uses(subt[1], 1)],
            [b_sub_uses(subt[2], 0), b_sub_uses(subt[2], 1)],
            [b_sub_uses(subt[3], 0), b_sub_uses(subt[3], 1)],
        ];
        // A direct sub-partition contributes mvd 0 / ref in-list to the
        // ctxInc — both the per-MB export and the within-MB 30-cache that a
        // later (non-direct) sub in this MB reads.
        for (i, &st) in subt.iter().enumerate() {
            if st == 0 {
                let b = i * 4;
                for zb in b..b + 4 {
                    // each table read once, not twice
                    let (g, c) = (G_SCAN4[zb & 15], CACHE30[zb & 15]);
                    (mref0[g], mref1[g]) = (0, 0);
                    (refc0[c], refc1[c]) = (0, 0);
                }
            }
        }
        // ref_idx_l0 for all four 8x8s, then ref_idx_l1, then the mvds
        // (spec 7.3.5.2 sub_mb_pred). ONE ref per 8x8 -- never per
        // sub-partition -- and B_Direct_8x8 codes none.
        let mut sref = [[0i8; 2]; 4]; // [sub-MB][list]
        for list in 0..2usize {
            // Single-reference lists code no ref_idx at all.
            if (if list == 0 {
                self.num_ref_active
            } else {
                self.num_ref_active1
            }) <= 1
            {
                continue;
            }
            let rc = if list == 0 { &mut *refc0 } else { &mut *refc1 };
            for i in 0..4usize {
                let st = subt[i];
                if st == 0 || !su[i][list] {
                    continue;
                }
                let b = i * 4;
                let s = CACHE30[b & 15].clamp(6, 29);
                let c0 = (rc[s - 1] > 0) as usize + 2 * (rc[s - 6] > 0) as usize;
                let r = parse_ref_idx_cabac(cab, c0);
                for &zb in &[b, b + 1, b + 2, b + 3] {
                    rc[CACHE30[zb]] = r;
                }
                sref[i][list] = r;
            }
        }
        for list in 0..2usize {
            let (mmv, mrf, mc, rc) = if list == 0 {
                (&mut *mmvd0, &mut *mref0, &mut *mvdc0, &mut *refc0)
            } else {
                (&mut *mmvd1, &mut *mref1, &mut *mvdc1, &mut *refc1)
            };
            for i in 0..4usize {
                let st = subt[i];
                if st == 0 || !su[i][list] {
                    continue;
                }
                let b = i * 4;
                for &(sx, sy, sw, sh) in b_sub_parts(st) {
                    // Shapes are 8x8 / 8x4 / 4x8 / 4x4, so the block
                    // list is closed-form: `step` is 1 when the part
                    // spans both columns, 2 when it spans both rows.
                    let (w4b, h4b) = (sw / 4, sh / 4);
                    let base = b + (sy / 4) * 2 + sx / 4;
                    let step = if w4b == 2 { 1 } else { 2 };
                    let zb = [base, base + step, base + 2, base + 3];
                    let n = w4b * h4b;
                    parse_mvd_partition(cab, zb[0], &zb[..n], mc, rc, mmv, mrf, sref[i][list]);
                }
            }
        }
        // Recon each 8×8: direct sub → decode_b_direct; else per sub-part
        // predict (median) + commit + MC.
        // Spatial-direct A/B/C are MB-level — walk once if any sub is
        // direct. `dmemo=0` rewalks every direct 8×8 (A/B oracle).
        // Derivation hoist: A/B/C AND the rid/median result are
        // MB-level — derive once, run only the per-8x8 region half
        // (czg differs per sub) for every direct sub.
        let hoisted = if self.direct_spatial && subt.contains(&0) {
            let (n0, n1) = self.b_direct_nbrs(mbx, mby)?;
            Some(Self::b_direct_refs_mvs(&n0, &n1))
        } else {
            None
        };
        for (p, &st) in subt.iter().enumerate() {
            let (b8x, b8y) = ((p % 2) * 8, (p / 2) * 8);
            if st == 0 {
                match hoisted {
                    Some(derived) => self.b_direct_region(
                        RegionB {
                            mb_x: mbx,
                            mb_y: mby,
                            px: b8x,
                            py: b8y,
                            rw: 8,
                            rh: 8,
                        },
                        pred_y,
                        c_pred,
                        derived,
                    )?,
                    None => {
                        self.decode_b_direct(
                            RegionB {
                                mb_x: mbx,
                                mb_y: mby,
                                px: b8x,
                                py: b8y,
                                rw: 8,
                                rh: 8,
                            },
                            pred_y,
                            c_pred,
                        )?;
                    }
                }
                continue;
            }
            // From the table built at the top of this arm.
            let (u0, u1) = (su[p & 3][0], su[p & 3][1]);
            for &(sx, sy, sw, sh) in b_sub_parts(st) {
                let (px, py) = (b8x + sx, b8y + sy);
                let mut mv = [(0i32, 0i32); 2];
                let (bx4, by4) = ((mbx * 4 + px / 4) as isize, (mby * 4 + py / 4) as isize);
                let didx = (py / 4) * 4 + px / 4;
                // A BI sub-part gathers both lists at ONE position, so the
                // availability work (bounds + coded + slice) is shared —
                // the fusion the 16x16 layout path already had.
                if u0 && u1 {
                    let (n0, n1) = self.mv_neighbors_both(bx4, by4, (sw / 4) as isize)?;
                    for (list, nb) in [(0usize, &n0), (1, &n1)] {
                        let d = (if list == 0 { &*mmvd0 } else { &*mmvd1 })[didx & 15];
                        let pmv = predict_mv(nb[0], nb[1], nb[2], sref[p & 3][list & 1] as i32);
                        mv[list & 1] = (pmv.0 + d[0] as i32, pmv.1 + d[1] as i32);
                    }
                } else {
                    for list in 0..2usize {
                        if if list == 0 { u0 } else { u1 } {
                            let d = (if list == 0 { &*mmvd0 } else { &*mmvd1 })[didx & 15];
                            let nb = self.mv_neighbors_list(bx4, by4, (sw / 4) as isize, list)?;
                            let pmv = predict_mv(nb[0], nb[1], nb[2], sref[p & 3][list & 1] as i32);
                            mv[list & 1] = (pmv.0 + d[0] as i32, pmv.1 + d[1] as i32);
                        }
                    }
                }
                let refi0 = if u0 { sref[p & 3][0] as i32 } else { -1 };
                let refi1 = if u1 { sref[p & 3][1] as i32 } else { -1 };
                let (region, mov) = (
                    RegionB {
                        mb_x: mbx,
                        mb_y: mby,
                        px,
                        py,
                        rw: sw,
                        rh: sh,
                    },
                    MovimientoBi {
                        refi0,
                        mv0: mv[0],
                        refi1,
                        mv1: mv[1],
                    },
                );
                self.b_set_motion(region, mov);
                self.b_mc(region, mov, pred_y, c_pred)?;
            }
        }
        Ok(allow8)
    }

    /// B 16x16, 16x8 u 8x16 (`mb_type` 1..=21): `ref_idx` de cada partición y lista,
    /// luego sus `mvd` (§7.3.5.1), vector predicho, guardado y compensación.
    ///
    /// # Errors
    /// Los de la decodificación del macrobloque.
    pub(super) fn b_particiones(
        &mut self,
        corte: &mut CorteCabac<'_>,
        pos: PosicionMb,
        m: &mut MovimientoB,
        bmt: u32,
    ) -> Result<(), MbError> {
        let CorteCabac {
            cab,
            pred_y,
            c_pred,
            ..
        } = corte;
        let PosicionMb { mbx, mby, .. } = pos;

        let MovimientoB {
            mvdc0,
            refc0,
            mvdc1,
            refc1,
            mmvd0,
            mref0,
            mmvd1,
            mref1,
            ..
        } = m;
        let (layout, mvmode, preds) = b_inter_layout(bmt);
        let parts: &[(usize, &[usize])] = match mvmode {
            0 => &[(0, &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])],
            1 => &[
                (0, &[0, 1, 2, 3, 4, 5, 6, 7]),
                (8, &[8, 9, 10, 11, 12, 13, 14, 15]),
            ],
            _ => &[
                (0, &[0, 1, 2, 3, 8, 9, 10, 11]),
                (4, &[4, 5, 6, 7, 12, 13, 14, 15]),
            ],
        };
        // ref_idx_l0 for EVERY partition, then ref_idx_l1, then the mvds
        // (spec 7.3.5.1 macroblock_prediction). This was missing entirely
        // -- the B path assumed a single reference -- so any B slice with
        // more than one active reference in either list desynced the
        // arithmetic decoder at the first partition that codes a ref_idx,
        // and the slice ended early at a phantom end_of_slice_flag.
        let mut pref = [[0i8; 2]; 2]; // [partition][list]
        for list in 0..2usize {
            let active = if list == 0 {
                self.num_ref_active
            } else {
                self.num_ref_active1
            };
            if active <= 1 {
                continue;
            }
            let rc = if list == 0 { &mut *refc0 } else { &mut *refc1 };
            for (p, &(pidx, zb)) in parts.iter().enumerate() {
                if !preds[p & 1].uses(list) {
                    continue;
                }
                let s = CACHE30[pidx & 15].clamp(6, 29);
                let c0 = (rc[s - 1] > 0) as usize + 2 * (rc[s - 6] > 0) as usize;
                let r = parse_ref_idx_cabac(cab, c0);
                // Seed the cache so a later partition's ref/mvd ctxInc sees it.
                for &zbi in zb.iter() {
                    rc[CACHE30[zbi & 15]] = r;
                }
                pref[p & 1][list & 1] = r;
            }
        }
        // mvd parse order: list-major, partition-minor (openh264
        // ParseInterBMotionInfoCabac); the ctxInc reads the same-list cache.
        for list in 0..2usize {
            let (mmv, mrf, mc, rc) = if list == 0 {
                (&mut *mmvd0, &mut *mref0, &mut *mvdc0, &mut *refc0)
            } else {
                (&mut *mmvd1, &mut *mref1, &mut *mvdc1, &mut *refc1)
            };
            for (p, &(pidx, zb)) in parts.iter().enumerate() {
                if preds[p & 1].uses(list) {
                    parse_mvd_partition(cab, pidx, zb, mc, rc, mmv, mrf, pref[p & 1][list & 1]);
                }
            }
        }
        // Per-partition recon: predict each list's MV, commit, MC.
        let (mbb_x, mbb_y) = (mbx * 4, mby * 4);
        for (p, &(rx, ry, rw, rh)) in layout.iter().enumerate() {
            let mut mv = [(0i32, 0i32); 2];
            // uses(list) is a property of the layout — resolve both
            // once instead of five times per partition, and build
            // the neighbour coordinates once instead of per list.
            let (u0, u1) = (preds[p & 1].uses(0), preds[p & 1].uses(1));
            let (nx, ny) = ((mbb_x + rx / 4) as isize, (mbb_y + ry / 4) as isize);
            let (nw, didx) = ((rw / 4) as isize, (ry / 4) * 4 + rx / 4);
            // Bi partitions gather both lists at ONE position —
            // fuse the availability work (same trick as
            // b_direct_nbrs); uni partitions keep the single
            // gather.
            if u0 && u1 {
                let (n0, n1) = self.mv_neighbors_both(nx, ny, nw)?;
                for (list, n) in [(0usize, &n0), (1, &n1)] {
                    let d = (if list == 0 { &*mmvd0 } else { &*mmvd1 })[didx & 15];
                    let pmv = predict_partition_mv(
                        mvmode,
                        p,
                        n[0],
                        n[1],
                        n[2],
                        pref[p & 1][list & 1] as i32,
                    );
                    mv[list & 1] = (pmv.0 + d[0] as i32, pmv.1 + d[1] as i32);
                }
            } else {
                for list in 0..2usize {
                    if if list == 0 { u0 } else { u1 } {
                        let d = (if list == 0 { &*mmvd0 } else { &*mmvd1 })[didx & 15];
                        let n = self.mv_neighbors_list(nx, ny, nw, list)?;
                        let pmv = predict_partition_mv(
                            mvmode,
                            p,
                            n[0],
                            n[1],
                            n[2],
                            pref[p & 1][list & 1] as i32,
                        );
                        mv[list & 1] = (pmv.0 + d[0] as i32, pmv.1 + d[1] as i32);
                    }
                }
            }
            let refi0 = if u0 { pref[p & 1][0] as i32 } else { -1 };
            let refi1 = if u1 { pref[p & 1][1] as i32 } else { -1 };
            let (region, mov) = (
                RegionB {
                    mb_x: mbx,
                    mb_y: mby,
                    px: rx,
                    py: ry,
                    rw,
                    rh,
                },
                MovimientoBi {
                    refi0,
                    mv0: mv[0],
                    refi1,
                    mv1: mv[1],
                },
            );
            self.b_set_motion(region, mov);
            // Proper spec bi-prediction (average of L0+L1). NOTE: the CAVLC
            // decode_b_mb replicates an openh264 bug here for a Bi 16×8/8×16
            // partition; our pixel gate is ffmpeg (spec-correct), so we do NOT.
            self.b_mc(region, mov, pred_y, c_pred)?;
        }
        Ok(())
    }

    /// `coded_block_pattern`, `transform_size_8x8_flag`, `mb_qp_delta` y residuo de un B inter, y su
    /// suma a la predicción ya hecha. `allow8` dice si el movimiento admite la transformada 8x8.
    /// Devuelve `end_of_slice_flag`.
    ///
    /// # Errors
    /// Los de la decodificación del macrobloque.
    pub(super) fn b_inter_residuo(
        &mut self,
        corte: &mut CorteCabac<'_>,
        pos: PosicionMb,
        allow8: bool,
    ) -> Result<bool, MbError> {
        let CorteCabac {
            cab,
            last_delta_qp,
            pred_y,
            c_pred,
            rejillas,
            ..
        } = corte;
        let RejillasCorte {
            mb_cbp,
            mb_nzc,
            cbf_dc,
            ..
        } = rejillas;
        let PosicionMb {
            addr,
            mbx,
            mby,
            left,
            top,
        } = pos;

        // Inter cbp + residual (identical to the P path).
        let cbp = parse_cbp_cabac(
            cab,
            valor_vecino(mb_cbp, top, "mb_cbp")?,
            valor_vecino(mb_cbp, left, "mb_cbp")?,
        );
        *mb_cbp
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_cbp"))? = cbp as u8;
        // H-49: an INTER macroblock carries transform_size_8x8_flag AFTER cbp
        // (spec 7.3.5), present only when CodedBlockPatternLuma > 0 and
        // noSubMbPartSizeLessThan8x8Flag. Same context as the intra read.
        let t8 = self.transform_8x8_mode && (cbp & 15) != 0 && allow8 && {
            let a = cond_term_flag(&self.mb_t8x8, left, |f| f, "mb_t8x8")?;
            let b = cond_term_flag(&self.mb_t8x8, top, |f| f, "mb_t8x8")?;
            cab.decode_decision(CTX_TRANSFORM_8X8 + a + b) != 0
        };
        *self
            .mb_t8x8
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_t8x8"))? = t8;
        self.any_t8 |= t8;
        // D9c-B: coded B with cbp==0 never parses residuals — skip the
        // ~2.5 KB coeff zero-init + fat BJob when NORES is on (default).
        // MC already filled pred_y / edc_regions; recon == pred (same as
        // B_Skip / P InterNoRes). t8 is always false here ((cbp&15)==0).
        if cbp == 0 {
            *last_delta_qp = 0;
            *self
                .mb_qp
                .get_mut(addr)
                .ok_or(MbError::Inconsistent("mb_qp"))? = self.cur_qp;
            *cbf_dc
                .get_mut(addr)
                .ok_or(MbError::Inconsistent("cbf_dc"))? = 0;
            *mb_nzc
                .get_mut(addr)
                .ok_or(MbError::Inconsistent("mb_nzc"))? = [0u8; 24];
            // Inline twin of decode_b_skip's plane copy (MC already done).
            for dy in 0..16 {
                let d = (mby * 16 + dy) * self.cw + mbx * 16;
                self.rec_y[d..d + 16].copy_from_slice(&pred_y[dy * 16..dy * 16 + 16]);
            }
            self.escribir_prediccion_croma((mbx, mby), c_pred);
            let w4 = self.mb_w * 4;
            for dy in 0..4 {
                self.nnz_y[(mby * 4 + dy) * w4 + mbx * 4..][..4].fill(0);
            }

            let fin_de_corte = cab.decode_terminate();
            return Ok(fin_de_corte);
        }
        // Scratch planes (see `scratch_luma`): zeroed per coded block by the parse.
        let mut luma8 = self
            .scratch_luma8
            .take()
            .unwrap_or_else(|| Box::new([[0i32; 64]; 4]));
        let (cbp_luma, cbp_chroma) = (cbp & 15, cbp >> 4);
        let mut nzc = cache_nzc_vecinos(mb_nzc, top, left)?;
        let mut cbfdc = 0u16;
        let mut nnzs = [0u8; 24]; // parsed totalCoeff per block
        let mut luma_scan = self
            .scratch_luma
            .take()
            .unwrap_or_else(|| Box::new([[0i32; 16]; 16]));
        let mut cdc = [[0i32; 4]; 2];
        let mut cac = self
            .scratch_cac
            .take()
            .unwrap_or_else(|| Box::new([[[0i32; 16]; 4]; 2]));
        if cbp == 0 {
            *last_delta_qp = 0;
        }
        if cbp != 0 {
            let ndc = (
                valor_vecino(cbf_dc, top, "cbf_dc")?,
                valor_vecino(cbf_dc, left, "cbf_dc")?,
            );
            let qpd = parse_mb_qp_delta_cabac(cab, last_delta_qp);
            self.step_qp(qpd)?;
            parse_mb_residual_cabac::<false>(
                cab,
                &mut nzc,
                &mut cbfdc,
                ndc,
                cbp_luma,
                cbp_chroma,
                t8,
                ResidualOut {
                    luma: &mut luma_scan,
                    luma8: &mut luma8,
                    cdc: &mut cdc,
                    cac: &mut cac,
                    nnzs: &mut nnzs,
                },
            );
        }
        *self
            .mb_qp
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_qp"))? = self.cur_qp;
        *cbf_dc
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("cbf_dc"))? = cbfdc;
        let mn = nnz_raster_from_z(&nnzs);
        *mb_nzc
            .get_mut(addr)
            .ok_or(MbError::Inconsistent("mb_nzc"))? = mn;
        self.add_inter_residual(
            mbx,
            mby,
            pred_y,
            c_pred,
            ResiduoInter {
                luma_scan: Some(&*luma_scan),
                luma8: t8.then_some(&*luma8),
                cdc: &cdc,
                cac: (cbp_chroma == 2).then_some(&*cac),
                cbp_chroma,
                nnzs: &nnzs,
            },
        );

        self.scratch_luma = Some(luma_scan);
        self.scratch_luma8 = Some(luma8);
        self.scratch_cac = Some(cac);

        Ok(cab.decode_terminate())
    }
}
