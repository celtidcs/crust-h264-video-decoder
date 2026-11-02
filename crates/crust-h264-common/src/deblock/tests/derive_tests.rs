use super::*;

/// `derive_mb_bs` must reproduce the per-edge derivation exactly. It replaces
/// per-edge intra tests with per-macroblock constant fills, so the test data
/// must respect the invariant that licences it: intra/inter is uniform across
/// a macroblock's 16 blocks.
/// The PACKED derivation must reproduce the tile derivation exactly — same
/// strengths AND the same `flat_inter`. This is the oracle for the whole
/// packed-layout brick: the packed path is a different traversal of the same
/// rule, so a byte-identical corpus run can tell you THAT it broke but not where,
/// and (as Brick 2 proved) a pixel-identical mistake can hide in `flat_inter`
/// while silently costing the optimisation it was built for.
/// The SIMD twin must equal the scalar oracle on the FULL u16, not merely on the
/// bits the derivation reads — the don't-care lanes are masked to zero in both so
/// a lane-boundary mistake cannot hide behind "that bit is unused anyway".
///
/// Exercises the cases the kernel's shifts are sensitive to: NO_REF blocks, refs
/// that differ, motion exactly at the |d| == 4 threshold (the `>= 4` boundary),
/// and large opposite-signed vectors where a saturating subtract could go wrong.
/// The uniform-motion SIMD twin must equal the scalar oracle. Cases are built to
/// straddle the decision: genuinely uniform macroblocks, ones differing in exactly
/// ONE lane of ONE plane (the case a broadcast-compare gets wrong if a plane is
/// dropped), and ones differing only on List-1.
#[test]
fn mb_uniform_simd_matches_scalar() {
    let mut st = 0x51ed270bu32;
    let mut rnd = || {
        st ^= st << 13;
        st ^= st >> 17;
        st ^= st << 5;
        st
    };
    for case in 0..6000 {
        let mut p = MbPack {
            inter: case % 7 != 0, // exercise the intra short-circuit too
            ..Default::default()
        };
        for k in 0..16 {
            p.ref_id[k] = 3;
            p.mvx[k] = 7;
            p.mvy[k] = -9;
            p.ref1[k] = if case % 3 == 0 { NO_REF } else { 5 };
            p.mvx1[k] = 2;
            p.mvy1[k] = -4;
        }
        // Perturb exactly one lane of one plane, often — that is the boundary.
        if case % 2 == 0 {
            let k = 1 + (rnd() % 15) as usize;
            match rnd() % 6 {
                0 => p.ref_id[k] += 1,
                1 => p.mvx[k] += 1,
                2 => p.mvy[k] += 1,
                3 => p.ref1[k] = p.ref1[k].wrapping_add(1),
                4 => p.mvx1[k] += 1,
                _ => p.mvy1[k] += 1,
            }
        }
        assert_eq!(
            mb_uniform(&p),
            mb_uniform_scalar(&p),
            "case {case}: uniform SIMD != scalar"
        );
    }
}

#[test]
fn bs_motion_masks_simd_matches_scalar() {
    let mut st = 0x9e3779b9u32;
    let mut rnd = || {
        st ^= st << 13;
        st ^= st >> 17;
        st ^= st << 5;
        st
    };
    for case in 0..4000 {
        let mut p = MbPack {
            inter: true,
            ..Default::default()
        };
        for k in 0..16 {
            let r = rnd();
            p.ref_id[k] = match case % 4 {
                0 => (r & 1) as i32,       // two references, frequent changes
                1 => 7,                    // all identical
                2 if r & 7 == 0 => NO_REF, // scattered intra/unused
                _ => (r & 3) as i32,
            };
            // Straddle the >= 4 threshold deliberately, and include extremes.
            p.mvx[k] = match case % 3 {
                0 => ((r >> 3) & 7) as i16 - 4,
                1 => ((r >> 3) & 0x7fff) as i16 - 16384,
                _ => ((r >> 3) & 1) as i16 * 4,
            };
            p.mvy[k] = match case % 3 {
                0 => ((r >> 9) & 7) as i16 - 4,
                1 => ((r >> 9) & 0x7fff) as i16 - 16384,
                _ => ((r >> 9) & 1) as i16 * 4,
            };
        }
        let want = bs_motion_masks_scalar(&p);
        let got = bs_motion_masks(&p);
        assert_eq!(
            got, want,
            "case {case}: SIMD (left={:#06x} up={:#06x}) != scalar (left={:#06x} up={:#06x})",
            got.0, got.1, want.0, want.1
        );
    }
}

#[test]
fn packed_matches_tile() {
    let (mb_w, mb_h) = (6usize, 5usize);
    let w4 = mb_w * 4;
    let n = w4 * mb_h * 4;
    let mut st = 0x1234abcdu32;
    let mut rnd = || {
        st ^= st << 13;
        st ^= st >> 17;
        st ^= st << 5;
        st
    };
    let (mut inter, mut nnz) = (vec![false; n], vec![0u8; n]);
    let (mut mv, mut ref_id) = (vec![(0i32, 0i32); n], vec![0i32; n]);
    for my in 0..mb_h {
        for mx in 0..mb_w {
            // intra/inter is a per-MACROBLOCK property; the packed record stores
            // one flag per macroblock and depends on that invariant.
            let mb_inter = rnd() & 3 != 0;
            // Deliberately include macroblocks that are uniform (exercising the
            // `uniform`/`flat_inter` fast paths) as well as fully varied ones.
            let uniform_mb = rnd() & 1 == 0;
            let (ur, umv) = (
                (rnd() & 1) as i32,
                ((rnd() & 7) as i32 - 4, (rnd() & 7) as i32 - 4),
            );
            let zero_coeffs = rnd() & 1 == 0;
            for by in 0..4 {
                for bx in 0..4 {
                    let i = (my * 4 + by) * w4 + mx * 4 + bx;
                    let r = rnd();
                    inter[i] = mb_inter;
                    nnz[i] = if uniform_mb && zero_coeffs {
                        0
                    } else if r & 0x30 != 0 {
                        (r >> 8 & 15) as u8
                    } else {
                        0
                    };
                    if mb_inter {
                        ref_id[i] = if uniform_mb { ur } else { (r >> 12 & 1) as i32 };
                        mv[i] = if uniform_mb {
                            umv
                        } else {
                            ((r >> 16 & 15) as i32 - 8, (r >> 20 & 15) as i32 - 8)
                        };
                    } else {
                        ref_id[i] = NO_REF;
                        mv[i] = (0, 0);
                    }
                }
            }
        }
    }
    // A List-1 plane too, so the two-slot set-matching rule is exercised — that
    // rule is order-independent (a pair matching after a SWAP is NOT different
    // motion), which a single-list grid cannot test at all.
    let (mut mv1, mut ref_id1) = (vec![(0i32, 0i32); n], vec![NO_REF; n]);
    for my in 0..mb_h {
        for mx in 0..mb_w {
            let mb_bi = rnd() & 1 == 0; // some macroblocks bi-predicted
            for by in 0..4 {
                for bx in 0..4 {
                    let i = (my * 4 + by) * w4 + mx * 4 + bx;
                    let r = rnd();
                    if inter[i] && mb_bi && r & 3 != 0 {
                        ref_id1[i] = (r >> 2 & 1) as i32;
                        mv1[i] = ((r >> 4 & 15) as i32 - 8, (r >> 8 & 15) as i32 - 8);
                    }
                }
            }
        }
    }
    for (tag, m1, r1) in [
        ("single-list", &[][..], &[][..]),
        ("two-list", &mv1[..], &ref_id1[..]),
    ] {
        let info = BlockInfo {
            inter: &inter,
            nnz: &nnz,
            mv: &mv,
            ref_id: &ref_id,
            poc0: &[],
            poc1: &[],
            mv1: m1,
            ref_id1: r1,
            w4,
            t8x8: &[],
            bs: &[],
            kind: &[],
            desplazamientos: &[],
        };
        let packs = pack_frame(&info, mb_w, mb_h).expect("frame packs");
        let _ = tag;

        let mut checked = 0usize;
        for mb_y in 0..mb_h {
            for mb_x in 0..mb_w {
                let tile = gather_tile(&info, mb_x, mb_y).expect("rejilla completa");
                let (uniform, flat) = scan_uniform_flat(&tile);
                for &mb_t8 in &[false, true] {
                    let (mut tv, mut th) = ([[0i32; 4]; 4], [[0i32; 4]; 4]);
                    derive_mb_bs(&tile, mb_x, mb_y, flat, uniform, mb_t8, (&mut tv, &mut th));

                    let (mut pv, mut ph) = ([[0i32; 4]; 4], [[0i32; 4]; 4]);
                    let pflat = derive_mb_packed(&packs, mb_w, mb_x, mb_y, mb_t8, &mut pv, &mut ph);

                    assert_eq!(
                        pflat, flat,
                        "MB ({mb_x},{mb_y}) t8={mb_t8}: flat_inter packed={pflat} tile={flat}"
                    );
                    // Compare only the edges the consuming loops actually read — the
                    // same masking the kind-vs-blind oracle uses, for the same reason.
                    for be in 0..4usize {
                        if be == 0 {
                            if mb_x > 0 {
                                assert_eq!(pv[0], tv[0], "MB ({mb_x},{mb_y}) left MB edge");
                            }
                            if mb_y > 0 {
                                assert_eq!(ph[0], th[0], "MB ({mb_x},{mb_y}) top MB edge");
                            }
                            continue;
                        }
                        if flat || (mb_t8 && (be == 1 || be == 3)) {
                            continue;
                        }
                        assert_eq!(pv[be], tv[be], "MB ({mb_x},{mb_y}) t8={mb_t8} v edge {be}");
                        assert_eq!(ph[be], th[be], "MB ({mb_x},{mb_y}) t8={mb_t8} h edge {be}");
                    }
                    checked += 1;
                }
            }
        }
        assert_eq!(
            checked,
            mb_w * mb_h * 2,
            "every macroblock checked both t8 ways ({tag})"
        );
    }
}

#[test]
fn derive_matches_per_edge() {
    let (mb_w, mb_h) = (6usize, 5usize);
    let w4 = mb_w * 4;
    let n = w4 * mb_h * 4;
    let mut st = 0xfeedfaceu32;
    let mut rnd = || {
        st ^= st << 13;
        st ^= st >> 17;
        st ^= st << 5;
        st
    };
    let (mut inter, mut nnz) = (vec![false; n], vec![0u8; n]);
    let (mut mv, mut ref_id) = (vec![(0i32, 0i32); n], vec![0i32; n]);
    for my in 0..mb_h {
        for mx in 0..mb_w {
            // one intra/inter decision per MACROBLOCK, as the bitstream has
            let mb_inter = rnd() & 3 != 0;
            for by in 0..4 {
                for bx in 0..4 {
                    let i = (my * 4 + by) * w4 + mx * 4 + bx;
                    let r = rnd();
                    inter[i] = mb_inter;
                    nnz[i] = if r & 0x30 != 0 {
                        (r >> 8 & 15) as u8
                    } else {
                        0
                    };
                    mv[i] = (((r >> 12) & 15) as i32 - 8, ((r >> 16) & 15) as i32 - 8);
                    ref_id[i] = if mb_inter {
                        ((r >> 20) & 3) as i32
                    } else {
                        NO_REF
                    };
                }
            }
        }
    }
    let info = BlockInfo {
        inter: &inter,
        nnz: &nnz,
        mv: &mv,
        ref_id: &ref_id,
        poc0: &[],
        poc1: &[],
        mv1: &[],
        ref_id1: &[],
        w4,
        t8x8: &[],
        bs: &[],
        kind: &[],
        desplazamientos: &[],
    };

    // The fused walk must agree with the two independent scans it replaced, on
    // every macroblock of this pseudo-random grid — the reference test the
    // fusion is gated on. Checked here rather than in its own test so it rides
    // the same varied intra/inter/nnz/motion mix.
    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            let tile = gather_tile(&info, mb_x, mb_y).expect("rejilla completa");
            assert_eq!(
                scan_uniform_flat(&tile),
                scan_two_pass(&tile),
                "fused predicate walk disagrees with the two-scan oracle at ({mb_x},{mb_y})"
            );
        }
    }

    let mut checked = 0;
    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            let tile = gather_tile(&info, mb_x, mb_y).expect("rejilla completa");
            // the flat-inter predicate exactly as `filter_frame` computes it
            let b0 = &tile[1][1];
            let flat = b0.inter
                && (1..5).all(|r| {
                    (1..5).all(|c| {
                        let b = &tile[r][c];
                        b.inter && !b.nz && b.same_motion(b0)
                    })
                });
            // …and `uniform_motion` likewise, derived here INDEPENDENTLY of
            // `scan_uniform_flat` so this stays an oracle for the fused walk
            // rather than a consumer of it.
            let uniform = b0.inter
                && (1..5).all(|r| (1..5).all(|c| tile[r][c].inter && tile[r][c].same_motion(b0)));
            for &mb_t8 in &[false, true] {
                let (mut bv, mut bh) = ([[0i32; 4]; 4], [[0i32; 4]; 4]);
                derive_mb_bs(&tile, mb_x, mb_y, flat, uniform, mb_t8, (&mut bv, &mut bh));
                for be in 0..4usize {
                    let mb_edge = be == 0;
                    let skip_internal = !mb_edge && (flat || (mb_t8 && (be == 1 || be == 3)));
                    for seg in 0..4 {
                        let want_v = if skip_internal || (mb_edge && mb_x == 0) {
                            0
                        } else {
                            bs_tile(&tile[seg + 1][be], &tile[seg + 1][be + 1], mb_edge)
                        };
                        let want_h = if skip_internal || (mb_edge && mb_y == 0) {
                            0
                        } else {
                            bs_tile(&tile[be][seg + 1], &tile[be + 1][seg + 1], mb_edge)
                        };
                        assert_eq!(
                            bv[be][seg], want_v,
                            "V mb=({mb_x},{mb_y}) be={be} seg={seg} t8={mb_t8}"
                        );
                        assert_eq!(
                            bh[be][seg], want_h,
                            "H mb=({mb_x},{mb_y}) be={be} seg={seg} t8={mb_t8}"
                        );
                        checked += 2;
                    }
                }
            }
        }
    }
    assert!(checked > 1500, "coverage too low: {checked}");
}
