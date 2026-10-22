//! Bucle de filtrado por franjas de filas de macrobloques (UIT-T H.264 §8.7).
//!
//! Contiene [`filter_frame_rows_impl`] y sus auxiliares de filtrado vertical,
//! horizontal y de croma con `#[inline(always)]`, preservando intacta la generación
//! de código y la elisión de comprobaciones de límites sobre las rebanadas por fila.

use super::*;

/// Filtra los cuatro grupos de bordes verticales de luma (`be` 0..4) de un macrobloque.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn filtrar_luma_vertical_mb(
    y: &mut [u8],
    mb_x: usize,
    mb_y: usize,
    cw: usize,
    qp_cur: i32,
    qp_row: &[u8],
    offset_a: i32,
    offset_b: i32,
    flat_inter: bool,
    mb_t8: bool,
    pre_bs: Option<&MbBs>,
    vnz: &[bool; 4],
    bs_v: &[[i32; 4]; 4],
    qp_left: &mut Option<i32>,
    int_y: &mut Option<(i32, i32, [i32; 3])>,
) {
    // ---- luma vertical edges (block columns 0..4) ----
    for be in 0..4usize {
        if be == 0 && mb_x == 0 {
            continue;
        }
        if flat_inter && be != 0 {
            continue; // internal bs all 0 (flat inter MB)
        }
        // 8×8-transform MBs: internal 4×4 edges (be 1, 3) aren't filtered.
        if mb_t8 && (be == 1 || be == 3) {
            continue;
        }
        let mb_edge = be == 0;
        // MASK FIRST. On the precomputed path the all-zero decision was
        // already made up front (one u32 compare per edge group), so a
        // zero group costs one bool here and never widens.
        let bs4: [i32; 4] = if let Some(m) = pre_bs {
            if !vnz[be] {
                continue;
            }
            m.v[be].map(|b| b as i32)
        } else {
            let b = bs_v[be];
            if b == [0i32; 4] {
                continue;
            }
            b
        };
        // Thresholds AFTER the all-zero early-out: on real content most
        // edges filter nothing, and computing α/β/tc0 (two clamps plus
        // three table loads, and a neighbour QP read on MB edges) for an
        // edge we are about to skip is pure waste.
        let (alpha_y, beta_y, tc0a) = if mb_edge {
            let ql = *qp_left.get_or_insert_with(|| {
                // `mb_x > 0` here, not just at the `continue` guard
                // above: the closure hides that fact from LLVM and it
                // emitted a bounds check for `mb_x - 1`.
                if mb_x > 0 {
                    qp_row[mb_x - 1] as i32
                } else {
                    qp_cur
                }
            });
            let qpav = (ql + qp_cur + 1) >> 1;
            thresholds(qpav, offset_a, offset_b)
        } else {
            *int_y.get_or_insert_with(|| thresholds(qp_cur, offset_a, offset_b))
        };
        let tc0_luma = |bs: i32| {
            if (1..4).contains(&bs) {
                tc0a[bs as usize - 1]
            } else {
                0
            }
        };
        let x = mb_x * 16 + be * 4;
        // Vertical edge via openh264's transpose → V-filter → transpose-back
        // (the `DeblockLumaLt4H` wrapper). tc per 4-row segment (−1 = skip).
        #[cfg(accel)]
        {
            let base = mb_y * 16 * cw + (x - 4); // p3 column, top row
            if bs4 == [4i32; 4] {
                rusty_h264_accel::deblock_luma_eq4_h(&mut y[base..], cw, alpha_y, beta_y);
            } else {
                let tc: [i8; 4] = core::array::from_fn(|i| {
                    if (1..4).contains(&bs4[i]) {
                        tc0_luma(bs4[i]) as i8
                    } else {
                        -1
                    }
                });
                rusty_h264_accel::deblock_luma_lt4_h(&mut y[base..], cw, alpha_y, beta_y, &tc);
            }
        }
        #[cfg(not(accel))]
        for (seg, &bs) in bs4.iter().enumerate() {
            if bs == 0 {
                continue;
            }
            let tc0 = tc0_luma(bs);
            for row in 0..4 {
                let yy = mb_y * 16 + seg * 4 + row;
                // Vertical edge → the line is CONTIGUOUS: the window
                // variant folds the per-sample bounds checks. `x >= 4`
                // here (the `be == 0 && mb_x == 0` edge is skipped
                // above), so `base - 4` cannot underflow.
                filter_luma_line_contig(y, yy * cw + x, bs, alpha_y, beta_y, tc0);
            }
        }
    }
}

/// Filtra los cuatro grupos de bordes horizontales de luma (`be` 0..4) de un macrobloque.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn filtrar_luma_horizontal_mb(
    y: &mut [u8],
    mb_x: usize,
    mb_y: usize,
    cw: usize,
    qp_cur: i32,
    qp_up_row: Option<&[u8]>,
    offset_a: i32,
    offset_b: i32,
    flat_inter: bool,
    mb_t8: bool,
    pre_bs: Option<&MbBs>,
    hnz: &[bool; 4],
    bs_h: &[[i32; 4]; 4],
    qp_up: &mut Option<i32>,
    int_y: &mut Option<(i32, i32, [i32; 3])>,
) {
    // ---- luma horizontal edges (block rows 0..4) ----
    for be in 0..4usize {
        if be == 0 && mb_y == 0 {
            continue;
        }
        if flat_inter && be != 0 {
            continue; // internal bs all 0 (flat inter MB)
        }
        if mb_t8 && (be == 1 || be == 3) {
            continue;
        }
        let mb_edge = be == 0;
        // MASK FIRST. On the precomputed path the all-zero decision was
        // already made up front (one u32 compare per edge group), so a
        // zero group costs one bool here and never widens.
        let bs4: [i32; 4] = if let Some(m) = pre_bs {
            if !hnz[be] {
                continue;
            }
            m.h[be].map(|b| b as i32)
        } else {
            let b = bs_h[be];
            if b == [0i32; 4] {
                continue;
            }
            b
        };
        // Thresholds after the early-out — see the vertical-edge note.
        let (alpha_y, beta_y, tc0a) = if mb_edge {
            let qu = *qp_up.get_or_insert_with(|| qp_up_row.map_or(qp_cur, |r| r[mb_x] as i32));
            let qpav = (qu + qp_cur + 1) >> 1;
            thresholds(qpav, offset_a, offset_b)
        } else {
            *int_y.get_or_insert_with(|| thresholds(qp_cur, offset_a, offset_b))
        };
        let tc0_luma = |bs: i32| {
            if (1..4).contains(&bs) {
                tc0a[bs as usize - 1]
            } else {
                0
            }
        };
        let yy = mb_y * 16 + be * 4;
        // openh264's DeblockLumaLt4V/Eq4V filter the whole 16-column horizontal
        // edge at once (p/q vertical; plane 16-aligned via AlignedBytes).
        // bit-identical spec filter; tc per 4-column segment (−1 = skip).
        #[cfg(accel)]
        {
            let base = (yy - 4) * cw + mb_x * 16; // p3 row (4 rows above q0)
            if bs4 == [4i32; 4] {
                rusty_h264_accel::deblock_luma_eq4_v(&mut y[base..], cw, alpha_y, beta_y);
            } else {
                let tc: [i8; 4] = core::array::from_fn(|i| {
                    if (1..4).contains(&bs4[i]) {
                        tc0_luma(bs4[i]) as i8
                    } else {
                        -1
                    }
                });
                rusty_h264_accel::deblock_luma_lt4_v(&mut y[base..], cw, alpha_y, beta_y, &tc);
            }
        }
        #[cfg(not(accel))]
        for (seg, &bs) in bs4.iter().enumerate() {
            if bs == 0 {
                continue;
            }
            let tc0 = tc0_luma(bs);
            for col in 0..4 {
                let x = mb_x * 16 + seg * 4 + col;
                let line = Line {
                    base: yy * cw + x,
                    step: cw as isize,
                };
                filter_luma_line(y, &line, bs, alpha_y, beta_y, tc0);
            }
        }
    }
}

/// Filtra los bordes verticales y horizontales de croma (8×8) reutilizando los `bS` de luma co-ubicados.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn filtrar_croma_mb(
    u: &mut [u8],
    v: &mut [u8],
    mb_x: usize,
    mb_y: usize,
    ccw: usize,
    qp_cur: i32,
    chroma_qp_offset: i32,
    qp_row: &[u8],
    qp_up_row: Option<&[u8]>,
    offset_a: i32,
    offset_b: i32,
    flat_inter: bool,
    pre_bs: Option<&MbBs>,
    vnz: &[bool; 4],
    hnz: &[bool; 4],
    bs_v: &[[i32; 4]; 4],
    bs_h: &[[i32; 4]; 4],
    qp_left: &mut Option<i32>,
    qp_up: &mut Option<i32>,
) {
    // Per-edge QP: deblock strength uses the average of the two adjacent
    // macroblocks' QPy (spec §8.7.2). For an internal edge both sides share the
    // current MB's QP. Chroma averages the two MBs' QPc.
    let qpc = |qpy_val: i32| {
        crate::predict::chroma_qp((qpy_val + chroma_qp_offset).clamp(0, 51) as u8) as i32
    };
    // ---- chroma edges (8×8): bS taken from the co-located luma edge ----
    // The chroma `tc` is the spec `tc0+1` (no ap/aq adjustment); bS varies per
    // 2-chroma-sample segment (= one co-located luma 4×4 block).
    #[cfg(accel)]
    {
        let tc0_of = |arr: [i32; 3], bs: i32| {
            if (1..4).contains(&bs) {
                arr[bs as usize - 1]
            } else {
                0
            }
        };
        // Chroma thresholds are derived per edge, AFTER that edge is known
        // to filter. Deriving all three sets up front cost three
        // `chroma_qp` lookups and three table lookups on every macroblock,
        // including the majority whose chroma edges are all bS 0.
        // `cur` is the macroblock's own chroma QP - a clamp plus a
        // `chroma_qp` table lookup that was redone on EVERY chroma edge.
        let qpc_cur = qpc(qp_cur);
        let mut int_c: Option<(i32, i32, [i32; 3])> = None;
        // vertical chroma edges → DeblockChromaLt4H/Eq4H (Cb+Cr together).
        // CARRY THE EDGE INDEX, don't recompute `cxe / 2`. The divisor
        // is a loop constant and the result is always 0 or 2, yet LLVM
        // still emitted a bounds check for `vnz[ce & 3]`. A literal
        // index folds it away.
        // `ce` is 0 or 2 by construction, but LLVM would not const-fold
        // it out of the two-element loop and kept a bounds check on every
        // `[ce]`. `& 3` states the range it could not infer; the values
        // are unchanged.
        for (cxe, ce) in [(0usize, 0usize), (4usize, 2usize)] {
            if cxe == 0 && mb_x == 0 {
                continue;
            }
            if flat_inter && cxe != 0 {
                continue; // internal bs all 0 (flat inter MB)
            }
            let mb_edge = cxe == 0;
            let x = mb_x * 8 + cxe;
            // MASK FIRST - and note the chroma edge groups are the SAME
            // stored entries the luma loops just tested (co-located luma
            // edges 0 and 2), so this reuses that decision instead of
            // re-scanning them. Census FILTROW: 4 chroma tests per MB.
            let bs4: [i32; 4] = if let Some(m) = pre_bs {
                if !vnz[ce & 3] {
                    continue;
                }
                m.v[ce & 3].map(|b| b as i32)
            } else {
                let b = bs_v[ce & 3]; // co-located luma edge, already derived
                if b == [0i32; 4] {
                    continue;
                }
                b
            };
            // MB edge: average with the LEFT neighbour's chroma QP,
            // taken from the same cached luma QP the luma loop resolved.
            let (alpha_c, beta_c, tc0c) = if mb_edge {
                let ql = *qp_left.get_or_insert_with(|| {
                    // `mb_x > 0` here, not just at the `continue` guard
                    // above: the closure hides that fact from LLVM and it
                    // emitted a bounds check for `mb_x - 1`.
                    if mb_x > 0 {
                        qp_row[mb_x - 1] as i32
                    } else {
                        qp_cur
                    }
                });
                thresholds((qpc(ql) + qpc_cur + 1) >> 1, offset_a, offset_b)
            } else {
                *int_c.get_or_insert_with(|| thresholds(qpc_cur, offset_a, offset_b))
            };
            let base = (mb_y * 8) * ccw + (x - 2); // p1 (2 cols left of q0)
            if bs4 == [4i32; 4] {
                rusty_h264_accel::deblock_chroma_eq4_h(
                    &mut u[base..],
                    &mut v[base..],
                    ccw,
                    alpha_c,
                    beta_c,
                );
            } else {
                let tc: [i8; 4] = core::array::from_fn(|i| {
                    if (1..4).contains(&bs4[i]) {
                        tc0_of(tc0c, bs4[i]) as i8 + 1
                    } else {
                        0
                    }
                });
                rusty_h264_accel::deblock_chroma_lt4_h(
                    &mut u[base..],
                    &mut v[base..],
                    ccw,
                    alpha_c,
                    beta_c,
                    &tc,
                );
            }
        }
        // horizontal chroma edges → DeblockChromaLt4V/Eq4V.
        for (cye, ce) in [(0usize, 0usize), (4usize, 2usize)] {
            if cye == 0 && mb_y == 0 {
                continue;
            }
            if flat_inter && cye != 0 {
                continue; // internal bs all 0 (flat inter MB)
            }
            let mb_edge = cye == 0;
            let yy = mb_y * 8 + cye;
            // MASK FIRST - and note the chroma edge groups are the SAME
            // stored entries the luma loops just tested (co-located luma
            // edges 0 and 2), so this reuses that decision instead of
            // re-scanning them. Census FILTROW: 4 chroma tests per MB.
            let bs4: [i32; 4] = if let Some(m) = pre_bs {
                if !hnz[ce & 3] {
                    continue;
                }
                m.h[ce & 3].map(|b| b as i32)
            } else {
                let b = bs_h[ce & 3]; // co-located luma edge, already derived
                if b == [0i32; 4] {
                    continue;
                }
                b
            };
            let (alpha_c, beta_c, tc0c) = if mb_edge {
                let qu = *qp_up.get_or_insert_with(|| qp_up_row.map_or(qp_cur, |r| r[mb_x] as i32));
                thresholds((qpc(qu) + qpc_cur + 1) >> 1, offset_a, offset_b)
            } else {
                *int_c.get_or_insert_with(|| thresholds(qpc_cur, offset_a, offset_b))
            };
            let base = (yy - 2) * ccw + mb_x * 8; // p1 (2 rows above q0)
            if bs4 == [4i32; 4] {
                rusty_h264_accel::deblock_chroma_eq4_v(
                    &mut u[base..],
                    &mut v[base..],
                    ccw,
                    alpha_c,
                    beta_c,
                );
            } else {
                let tc: [i8; 4] = core::array::from_fn(|i| {
                    if (1..4).contains(&bs4[i]) {
                        tc0_of(tc0c, bs4[i]) as i8 + 1
                    } else {
                        0
                    }
                });
                rusty_h264_accel::deblock_chroma_lt4_v(
                    &mut u[base..],
                    &mut v[base..],
                    ccw,
                    alpha_c,
                    beta_c,
                    &tc,
                );
            }
        }
    }
    #[cfg(not(accel))]
    {
        let _ = (vnz, hnz, qp_left, qp_up);
        // Chroma edge thresholds use the average of the two MBs' QPc.
        let cur_qpc = qpc(qp_cur);
        let (alpha_cv, beta_cv, tc0cv) = if mb_x > 0 {
            let ql = qp_row[mb_x - 1] as i32;
            thresholds((qpc(ql) + cur_qpc + 1) >> 1, offset_a, offset_b)
        } else {
            (0, 0, [0; 3]) // unused (cxe==0 skipped at frame edge)
        };
        let (alpha_ch, beta_ch, tc0ch) = if mb_y > 0 {
            let qu = qp_up_row.map_or(qp_cur, |r| r[mb_x] as i32);
            thresholds((qpc(qu) + cur_qpc + 1) >> 1, offset_a, offset_b)
        } else {
            (0, 0, [0; 3])
        };
        let (alpha_ci, beta_ci, tc0ci) = thresholds(cur_qpc, offset_a, offset_b);
        let tc0_of = |arr: [i32; 3], bs: i32| {
            if (1..4).contains(&bs) {
                arr[bs as usize - 1]
            } else {
                0
            }
        };
        // bS from the co-located luma edge — the STORED strengths when
        // available, exactly like the accel arm and the luma loops above.
        // This is not just the shared-derivation saving: on the precomputed
        // path the caller's view may carry NO syntax grids at all (the E2
        // worker's `PixelCtx::filter_row` passes `inter: &[]`), so live
        // derivation here is an out-of-bounds panic, not a slow path.
        let chroma_bs = |stored: &[[i32; 4]; 4], edge: usize, vertical: bool| -> [i32; 4] {
            // PRECOMPUTED strengths first — exactly like the luma loops and
            // the accel arm above. On this path `bs_v`/`bs_h` are never
            // populated (derivation is skipped by construction), so the
            // stored-strength fallback below would read their zero-init and skip
            // every chroma edge: scalar builds decoded packed-bS streams
            // with chroma deblock silently OFF (the shipped-0.11.0 scalar
            // chroma divergence, and the previously filed "Main-profile
            // chroma-deblock divergence" — the decoder-exoneration arm of
            // that hunt was an accel build and never executed this closure).
            if let Some(m) = pre_bs {
                return (if vertical {
                    m.v[edge / 2]
                } else {
                    m.h[edge / 2]
                })
                .map(|b| b as i32);
            }
            stored[edge / 2] // co-located luma edge, already derived
        };
        for plane in [&mut *u, &mut *v] {
            for cxe in [0usize, 4] {
                if cxe == 0 && mb_x == 0 {
                    continue;
                }
                if flat_inter && cxe != 0 {
                    continue;
                }
                let mb_edge = cxe == 0;
                // MB-left edge uses the cross-MB chroma avg; internal uses the MB's own.
                let (alpha_c, beta_c, tc0c) = if mb_edge {
                    (alpha_cv, beta_cv, tc0cv)
                } else {
                    (alpha_ci, beta_ci, tc0ci)
                };
                let bs4 = chroma_bs(bs_v, cxe, true);
                let x = mb_x * 8 + cxe;
                for row in 0..8 {
                    // Segment = the co-located luma block row (2 chroma rows each).
                    let bs = bs4[(row * 2) / 4];
                    if bs == 0 {
                        continue;
                    }
                    let yy = mb_y * 8 + row;
                    // Vertical edge → contiguous window (x >= 2: the
                    // left-border MB edge is skipped above).
                    filter_chroma_line_contig(
                        plane,
                        yy * ccw + x,
                        bs,
                        alpha_c,
                        beta_c,
                        tc0_of(tc0c, bs),
                    );
                }
            }
            for cye in [0usize, 4] {
                if cye == 0 && mb_y == 0 {
                    continue;
                }
                if flat_inter && cye != 0 {
                    continue;
                }
                let mb_edge = cye == 0;
                let (alpha_c, beta_c, tc0c) = if mb_edge {
                    (alpha_ch, beta_ch, tc0ch)
                } else {
                    (alpha_ci, beta_ci, tc0ci)
                };
                let bs4 = chroma_bs(bs_h, cye, false);
                let yy = mb_y * 8 + cye;
                for col in 0..8 {
                    // Segment = the co-located luma block column.
                    let bs = bs4[(col * 2) / 4];
                    if bs == 0 {
                        continue;
                    }
                    let line = Line {
                        base: yy * ccw + (mb_x * 8 + col),
                        step: ccw as isize,
                    };
                    filter_chroma_line(plane, &line, bs, alpha_c, beta_c, tc0_of(tc0c, bs));
                }
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn filter_frame_rows_impl<const PRE: bool>(
    y: &mut [u8],
    u: &mut [u8],
    v: &mut [u8],
    mb_w: usize,
    mb_h: usize,
    rows: core::ops::Range<usize>,
    mb_qp: &[u8],
    chroma_qp_offset: i32,
    info: &BlockInfo,
) -> Result<(), FueraDeRango> {
    if info.desplazamientos.len() != mb_w * mb_h {
        return Err(FueraDeRango("desplazamientos del filtro"));
    }
    let cw = mb_w * 16;
    let ccw = mb_w * 8;
    // ONE streaming pass over the frame arrays, not 3600 scattered gathers. `None`
    // when the frame carries List-1 data (B slices), which MbPack does not model —
    // those macroblocks keep the blind path.
    let scratch = PACK_SCRATCH.take();
    // WIN: PRE already means the strengths are precomputed, i.e. info.bs is NOT
    // empty -- but the compiler cannot see that through the slice. Stating it as
    //  folds the whole pack path away in the decoder instantiation: the
    // PACK_SCRATCH take/set, the MbPack buffer and pack_frame_into with it.
    let packs: Option<Vec<MbPack>> = if !PRE && info.bs.is_empty() {
        let mut buf = scratch;
        pack_frame_into(info, mb_w, mb_h, &mut buf)?;
        Some(buf)
    } else {
        PACK_SCRATCH.set(scratch);
        None
    };

    for mb_y in rows {
        // ROW SLICES. Every one of these grids is indexed `mb_y * mb_w + mb_x`
        // inside the macroblock loop, and the compiler cannot prove that in
        // range against the whole-frame length - it emitted a
        // `panic_bounds_check` for each. Sliced to THIS row, `[mb_x]` is
        // provably `< mb_w` and the checks fold away. Same shape that took
        // `pack_mb` from six panic edges to one.
        let row0 = mb_y * mb_w;
        // `&x[a..][..n]` and NOT `&x[a..a+n]`: the second form leaves the length
        // as an expression LLVM has to reassociate, and it then failed to prove
        // `mb_x < len` for the macroblock loop - the checks merely MOVED. The
        // re-slice makes the length literally `mb_w`.
        let qp_row = &mb_qp[row0..][..mb_w];
        let qp_up_row = if mb_y > 0 {
            Some(&mb_qp[row0 - mb_w..][..mb_w])
        } else {
            None
        };
        // WIN: PRE means the strengths ARE present, so the None arm is dead in the
        // decoder instantiation and every later `bs_row` test folds with it.
        let bs_row = if !PRE && info.bs.is_empty() {
            None
        } else {
            Some(&info.bs[row0..][..mb_w])
        };
        let t8_row = if info.t8x8.is_empty() {
            None
        } else {
            Some(&info.t8x8[row0..][..mb_w])
        };
        let desplazamientos_fila = info
            .desplazamientos
            .get(row0..row0 + mb_w)
            .ok_or(FueraDeRango("desplazamientos del filtro"))?;
        for mb_x in 0..mb_w {
            // `t8x8` may be empty (no MB uses the 8×8 transform — Baseline); treat
            // an empty grid as all-false so the caller can skip allocating it.
            // A "flat inter MB" — every 4x4 inter, zero nnz, one (ref, mv) pair (e.g.
            // any skip MB) — has bs = 0 on ALL its internal edges by §8.7.2.1 (no
            // coefficients, same reference, identical motion), so the six internal
            // edge groups can be skipped wholesale. Byte-identical control flow.
            // Gather the MB's 4×4 block state once (see `Blk`). Every boundary
            // strength below then reads the tile instead of the strided frame
            // arrays. Only valid for a single reference list; B slices keep the
            // original per-edge path untouched.
            // Precomputed strengths short-circuit the gather AND the derivation;
            // flat_inter and the t8x8 skips are already baked into the stored
            // zeros, which the all-zero early-out below handles identically.
            // PRE makes this a CONSTANT, which is what folds the blind subtree away.
            let precomputed = PRE || !info.bs.is_empty();
            // ALL-ZERO MACROBLOCK, DECIDED FIRST. Every edge group below opens
            // with an all-zero early-out, so a macroblock whose stored strengths
            // are all 0 filters nothing. On the decoder's PRECOMPUTED path that
            // is the dominant class - census DBSDERIVE allzero: 88.9%
            // screen_text, 78.1% akiyo, 63.5% FourPeople - and the check used to
            // sit AFTER a 128-byte bs_v/bs_h stack zero-init and a
            // bounds-checked t8x8 load, both of which it then discarded.
            // Byte-identical: same predicate, same `continue`, simply reached
            // before the work it throws away.
            // PRECOMPUTED EDGE MASK. The stored strengths are u8, so "is this
            // edge group all zero" is ONE u32 compare on the bytes - against a
            // 4-element i32 `.all()` scan over a widened copy. Computing all
            // eight here, once, serves BOTH the luma loops and the chroma loops
            // (which read the very same `bs_v[cxe/2]` / `bs_h[cye/2]` entries
            // and were re-testing them). Census FILTROW: 8 luma + 4 chroma
            // tests per macroblock, of which only 32.6-66.5% ever filter.
            let pre_bs = bs_row.map(|r| &r[mb_x]);
            // The all-zero macroblock is the DOMINANT class (63.5-88.9% on LIGHT
            // content), and two 16-byte compares settle it. Deciding it from the
            // eight per-group compares below instead would make the common case
            // pay for the mask it is about to throw away.
            if let Some(m) = pre_bs {
                if m.v == [[0u8; 4]; 4] && m.h == [[0u8; 4]; 4] {
                    continue;
                }
            }
            let (vnz, hnz) = match pre_bs {
                Some(m) => (
                    core::array::from_fn(|e| u32::from_ne_bytes(m.v[e]) != 0),
                    core::array::from_fn(|e| u32::from_ne_bytes(m.h[e]) != 0),
                ),
                None => ([true; 4], [true; 4]),
            };

            // KIND-AWARE FAST PATH. When the producer has classified this
            // macroblock, its strengths follow from syntax and the 24-block
            // neighbourhood gather is not needed at all: Intra reads NOTHING, Skip
            // reads 9 blocks, InterUniform reads 16 nnz bytes. `Inter` and UNSET
            // fall through to the blind path below, so an unclassified macroblock
            // costs speed and never correctness.
            //
            // The gather is skipped by CONSTRUCTION here rather than by an early-out
            // inside it — building the 5x5 `Tile` at all is an 800-byte
            // default-initialisation before 24 of its 25 entries are overwritten.
            // Read HERE, not at the top of the body: the all-zero early-out
            // above discards it, and it is a bounds-checked grid load per MB.
            let mb_t8 = t8_row.is_some_and(|r| r[mb_x]);
            let mut bs_v = [[0i32; 4]; 4];
            let mut bs_h = [[0i32; 4]; 4];
            // OUTLINED: the decoder never takes this path (see `derive_mb_general`).
            let flat_inter = if precomputed {
                false // the stored zeros already encode it
            } else {
                derive_mb_general(
                    info,
                    mb_x,
                    mb_y,
                    mb_w,
                    mb_t8,
                    packs.as_ref(),
                    &mut bs_v,
                    &mut bs_h,
                )?
            };
            // PER-MACROBLOCK QP AND INTERNAL THRESHOLDS. Every INTERNAL edge
            // group averages to this macroblock's own QP, so alpha/beta/tc0 are
            // identical across all six of them (three vertical + three
            // horizontal) - yet `thresholds()` (two clamps + three table loads)
            // was called per FILTERING edge: census FILTROW thresh == exactly
            // luma_filtered + chroma_filtered. Derived at most once per
            // macroblock here, and only if an internal edge actually filters.
            // `get_or_insert_with`, NOT `get_or_insert` - the latter evaluates
            // its argument eagerly (law banked in batch 10).
            let qp_cur = qp_row[mb_x] as i32;
            // Todos los bordes de este macrobloque tienen su `q0` en él: sus desplazamientos
            // rigen los umbrales de todos ellos (§8.7.2).
            let (offset_a, offset_b) = {
                let d = desplazamientos_fila[mb_x];
                (d.a(), d.b())
            };
            // Neighbour QPs are only read on a MACROBLOCK edge, and both the
            // luma and the chroma loops want the same two values - previously
            // each re-entered the `qpy` closure (five inlined bounds checks).
            let mut qp_left: Option<i32> = None;
            let mut qp_up: Option<i32> = None;
            let mut int_y: Option<(i32, i32, [i32; 3])> = None;
            filtrar_luma_vertical_mb(
                y,
                mb_x,
                mb_y,
                cw,
                qp_cur,
                qp_row,
                offset_a,
                offset_b,
                flat_inter,
                mb_t8,
                pre_bs,
                &vnz,
                &bs_v,
                &mut qp_left,
                &mut int_y,
            );
            filtrar_luma_horizontal_mb(
                y, mb_x, mb_y, cw, qp_cur, qp_up_row, offset_a, offset_b, flat_inter, mb_t8,
                pre_bs, &hnz, &bs_h, &mut qp_up, &mut int_y,
            );
            filtrar_croma_mb(
                u,
                v,
                mb_x,
                mb_y,
                ccw,
                qp_cur,
                chroma_qp_offset,
                qp_row,
                qp_up_row,
                offset_a,
                offset_b,
                flat_inter,
                pre_bs,
                &vnz,
                &hnz,
                &bs_v,
                &bs_h,
                &mut qp_left,
                &mut qp_up,
            );
        }
    }
    if let Some(buf) = packs {
        PACK_SCRATCH.set(buf);
    }
    Ok(())
}
