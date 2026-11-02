use super::*;

/// Per-pixel quarter-pel luma sample (spec §8.4.2.2.1) — the readable reference
/// kept as the bit-exactness oracle for the block-kernel MC below.
fn luma_sample(
    reference: &[u8],
    cw: usize,
    ch: usize,
    ix: isize,
    iy: isize,
    fx: i32,
    fy: i32,
) -> i32 {
    let g = |dx: isize, dy: isize| {
        at(reference, cw, ch, ix + dx, iy + dy).expect("plano de prueba completo")
    };
    if fx == 0 && fy == 0 {
        return g(0, 0);
    }
    let hor6 = |dy: isize| {
        g(-2, dy) - 5 * g(-1, dy) + 20 * g(0, dy) + 20 * g(1, dy) - 5 * g(2, dy) + g(3, dy)
    };
    let ver6 = |dx: isize| {
        g(dx, -2) - 5 * g(dx, -1) + 20 * g(dx, 0) + 20 * g(dx, 1) - 5 * g(dx, 2) + g(dx, 3)
    };
    let b = || clip_u8((hor6(0) + 16) >> 5);
    let h = || clip_u8((ver6(0) + 16) >> 5);
    let m = || clip_u8((ver6(1) + 16) >> 5);
    let s = || clip_u8((hor6(1) + 16) >> 5);
    let j = || {
        let j1 = hor6(-2) - 5 * hor6(-1) + 20 * hor6(0) + 20 * hor6(1) - 5 * hor6(2) + hor6(3);
        clip_u8((j1 + 512) >> 10)
    };
    match (fx, fy) {
        (1, 0) => (g(0, 0) + b() + 1) >> 1,
        (2, 0) => b(),
        (3, 0) => (g(1, 0) + b() + 1) >> 1,
        (0, 1) => (g(0, 0) + h() + 1) >> 1,
        (1, 1) => (b() + h() + 1) >> 1,
        (2, 1) => (b() + j() + 1) >> 1,
        (3, 1) => (b() + m() + 1) >> 1,
        (0, 2) => h(),
        (1, 2) => (h() + j() + 1) >> 1,
        (2, 2) => j(),
        (3, 2) => (j() + m() + 1) >> 1,
        (0, 3) => (g(0, 1) + h() + 1) >> 1,
        (1, 3) => (h() + s() + 1) >> 1,
        (2, 3) => (j() + s() + 1) >> 1,
        _ => (m() + s() + 1) >> 1,
    }
}

fn rand_plane(pw: usize, ph: usize, seed: u32) -> Vec<u8> {
    let mut s = seed;
    (0..pw * ph)
        .map(|_| {
            s = s.wrapping_mul(1103515245).wrapping_add(12345);
            (s >> 16) as u8
        })
        .collect()
}

fn make_padded(exact: &[u8], pw: usize, ph: usize, pad: usize) -> (Vec<u8>, usize) {
    let stride = pw + 2 * pad;
    let mut padded = vec![0u8; stride * (ph + 2 * pad)];
    for y in 0..ph {
        let d = (y + pad) * stride + pad;
        padded[d..d + pw].copy_from_slice(&exact[y * pw..y * pw + pw]);
    }
    expand_plane(&mut padded, stride, pad, pw, ph);
    (padded, stride)
}

#[test]
fn mc_luma_padded_matches_exact() {
    let (pw, ph) = (48usize, 32usize);
    let exact = rand_plane(pw, ph, 0x77);
    let (padded, stride) = make_padded(&exact, pw, ph, PAD_L);
    for &(bw, bh) in &[(16usize, 16usize), (8, 8), (16, 8), (8, 16), (4, 4)] {
        for x0 in [0usize, 8, pw - bw] {
            for y0 in [0usize, 8, ph - bh] {
                for mvx in [-40i32, -20, -3, 0, 1, 2, 3, 7, 20, 40] {
                    for mvy in [-40i32, -20, -3, 0, 1, 2, 3, 7, 20, 40] {
                        let mut a = vec![0u8; bw * bh];
                        let mut b = vec![0u8; bw * bh];
                        mc_luma(&exact, pw, ph, x0, y0, bw, bh, mvx, mvy, &mut a)
                            .expect("tile completo");
                        mc_luma_padded(
                            &padded, stride, PAD_L, pw, ph, x0, y0, bw, bh, mvx, mvy, &mut b,
                        )
                        .expect("tile completo");
                        assert_eq!(a, b, "luma bw={bw} x0={x0} y0={y0} mv=({mvx},{mvy})");
                    }
                }
            }
        }
    }
}

#[test]
fn mc_chroma_padded_matches_exact() {
    let (pw, ph) = (24usize, 16usize);
    let exact = rand_plane(pw, ph, 0x99);
    let (padded, stride) = make_padded(&exact, pw, ph, PAD_C);
    for &(bw, bh) in &[(8usize, 8usize), (4, 4), (8, 4), (4, 8), (2, 2)] {
        for x0 in [0usize, 4, pw - bw] {
            for y0 in [0usize, 4, ph - bh] {
                for mvx in [-40i32, -16, -3, 0, 1, 5, 8, 16, 40] {
                    for mvy in [-40i32, -16, -3, 0, 1, 5, 8, 16, 40] {
                        let mut a = vec![0u8; bw * bh];
                        let mut b = vec![0u8; bw * bh];
                        mc_chroma(&exact, pw, ph, x0, y0, bw, bh, mvx, mvy, &mut a)
                            .expect("plano de prueba completo");
                        mc_chroma_padded(
                            &padded, stride, PAD_C, pw, ph, x0, y0, bw, bh, mvx, mvy, &mut b,
                        )
                        .expect("tile completo");
                        assert_eq!(a, b, "chroma bw={bw} x0={x0} y0={y0} mv=({mvx},{mvy})");
                    }
                }
            }
        }
    }
}

/// Campaign-3 oracle: the fused single-pass builder must fill H/V/C
/// byte-for-byte like the tile-walk builder, across geometries whose padded
/// dimensions are and are not multiples of 16 (partial edge tiles) and with
/// enough randomness to exercise the clip.
#[test]
fn fused_hpel_builder_matches_tiles() {
    let mut st = 0x5eed_f00d_dead_beefu64;
    let mut lcg = move || {
        st = st
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (st >> 33) as u8
    };
    for &(cw, ch, pad) in &[(64usize, 48usize, 16usize), (80, 64, 12), (176, 144, 32)] {
        let reference: Vec<u8> = (0..cw * ch).map(|_| lcg()).collect();
        let (pw, ph) = (cw + 2 * pad, ch + 2 * pad);
        // The same edge-replicated padded source `build_hpel_planes` constructs.
        let mut f = vec![0u8; pw * ph];
        for y in 0..ph {
            let sy = (y as isize - pad as isize).clamp(0, ch as isize - 1) as usize;
            let row = &reference[sy * cw..sy * cw + cw];
            let d = &mut f[y * pw..y * pw + pw];
            d[..pad].fill(row[0]);
            d[pad..pad + cw].copy_from_slice(row);
            d[pad + cw..].fill(row[cw - 1]);
        }
        let (mut h1, mut v1, mut c1) = (vec![0u8; pw * ph], vec![0u8; pw * ph], vec![0u8; pw * ph]);
        let (mut h2, mut v2, mut c2) = (vec![0u8; pw * ph], vec![0u8; pw * ph], vec![0u8; pw * ph]);
        build_hpel_tiles(&f, pw, ph, &mut h1, &mut v1, &mut c1).expect("tile completo");
        build_hpel_fused(&f, pw, ph, &mut h2, &mut v2, &mut c2);
        assert_eq!(h1, h2, "H plane differs at {cw}x{ch} pad {pad}");
        assert_eq!(v1, v2, "V plane differs at {cw}x{ch} pad {pad}");
        assert_eq!(c1, c2, "C plane differs at {cw}x{ch} pad {pad}");
        // AVX2 fused kernel: byte-identical to the scalar fused builder (and
        // transitively to the tiles) on every geometry, including widths whose
        // vector interior leaves scalar tails.
        #[cfg(accel)]
        {
            let (mut h3, mut v3, mut c3) =
                (vec![0u8; pw * ph], vec![0u8; pw * ph], vec![0u8; pw * ph]);
            if rusty_h264_accel::hpel_fused(&f, pw, ph, &mut h3, &mut v3, &mut c3) {
                assert_eq!(h1, h3, "AVX2 H plane differs at {cw}x{ch} pad {pad}");
                assert_eq!(v1, v3, "AVX2 V plane differs at {cw}x{ch} pad {pad}");
                assert_eq!(c1, c3, "AVX2 C plane differs at {cw}x{ch} pad {pad}");
            }
        }
    }
}

/// The plane cache must be BIT-IDENTICAL to `mc_luma` wherever it applies —
/// that identity is what keeps the motion search's decisions, and therefore the
/// bitstream, unchanged. Sweeps every sub-pel phase, every block size, and a
/// range of positions including ones near the frame edge (where it must decline).
#[test]
fn hpel_block_matches_mc_luma_exactly() {
    let (cw, ch) = (96usize, 64usize);
    let mut reference = vec![0u8; cw * ch];
    let mut s: u32 = 0x9E37_79B9;
    for p in reference.iter_mut() {
        s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *p = (s >> 24) as u8;
    }
    let planes = build_hpel_planes(&reference, cw, ch).expect("tile completo");

    let mut checked = 0;
    for (bw, bh) in [(16, 16), (16, 8), (8, 16), (8, 8), (8, 4), (4, 8), (4, 4)] {
        for fy in 0..4i32 {
            for fx in 0..4i32 {
                if fx == 0 && fy == 0 {
                    continue; // full-pel is the copy path, not the plane path
                }
                for y0 in (0..ch - bh).step_by(7) {
                    for x0 in (0..cw - bw).step_by(5) {
                        for &(dx, dy) in &[
                            (0i32, 0i32),
                            (4, 0),
                            (0, 4),
                            (-4, -4),
                            (8, 8),
                            (-64, -64),
                            (96, 40),
                            (-96, 60),
                            (40, -96),
                        ] {
                            let (mvx, mvy) = (dx + fx, dy + fy);
                            let mut want = [0u8; 256];
                            mc_luma(&reference, cw, ch, x0, y0, bw, bh, mvx, mvy, &mut want)
                                .expect("tile completo");
                            let mut got = [0u8; 256];
                            if !hpel_block(&planes, x0, y0, bw, bh, mvx, mvy, &mut got) {
                                continue; // declined (edge) -> caller falls back
                            }
                            assert_eq!(
                                &got[..bw * bh],
                                &want[..bw * bh],
                                "bw={bw} bh={bh} fx={fx} fy={fy} x0={x0} y0={y0} mv=({mvx},{mvy})"
                            );
                            checked += 1;
                        }
                    }
                }
            }
        }
    }
    assert!(checked > 5_000, "too few positions exercised: {checked}");
}

#[test]
fn mc_luma_block_kernels_match_per_pixel() {
    // The block-kernel MC must be bit-identical to the per-pixel `luma_sample`
    // reference for every quarter-pel position, across interior AND edge blocks
    // (negative / off-frame MVs that exercise the clamped tile vs `at()`).
    let (cw, ch) = (40usize, 32usize);
    let mut state = 0x1357_9bdfu32;
    let mut next = || {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (state >> 24) as u8
    };
    let reference: Vec<u8> = (0..cw * ch).map(|_| next()).collect();
    for &(bw, bh) in &[(16, 16), (8, 8), (4, 4), (8, 16), (16, 8)] {
        for &(x0, y0) in &[(8usize, 8usize), (0, 0), (cw - bw, ch - bh)] {
            for mvx in -9..=9 {
                for mvy in -9..=9 {
                    let mut got = vec![0u8; bw * bh];
                    mc_luma(&reference, cw, ch, x0, y0, bw, bh, mvx, mvy, &mut got)
                        .expect("tile completo");
                    let ix0 = x0 as isize + (mvx >> 2) as isize;
                    let iy0 = y0 as isize + (mvy >> 2) as isize;
                    let (fx, fy) = (mvx & 3, mvy & 3);
                    for dy in 0..bh {
                        for dx in 0..bw {
                            let want = luma_sample(
                                &reference,
                                cw,
                                ch,
                                ix0 + dx as isize,
                                iy0 + dy as isize,
                                fx,
                                fy,
                            ) as u8;
                            assert_eq!(
                                got[dy * bw + dx],
                                want,
                                "bw{bw}x{bh} at ({x0},{y0}) mv({mvx},{mvy}) px({dx},{dy})"
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn mc_chroma_block_matches_per_pixel() {
    let (cw, ch) = (24usize, 20usize);
    let mut state = 0xabcd_1234u32;
    let mut next = || {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (state >> 24) as u8
    };
    let reference: Vec<u8> = (0..cw * ch).map(|_| next()).collect();
    let pp = |ix: isize, iy: isize, fx: i32, fy: i32| -> u8 {
        let a = at(&reference, cw, ch, ix, iy).expect("plano de prueba completo");
        let b = at(&reference, cw, ch, ix + 1, iy).expect("plano de prueba completo");
        let c = at(&reference, cw, ch, ix, iy + 1).expect("plano de prueba completo");
        let d = at(&reference, cw, ch, ix + 1, iy + 1).expect("plano de prueba completo");
        (((8 - fx) * (8 - fy) * a + fx * (8 - fy) * b + (8 - fx) * fy * c + fx * fy * d + 32) >> 6)
            as u8
    };
    for &(bw, bh) in &[(8, 8), (4, 4), (8, 4), (4, 8)] {
        for &(x0, y0) in &[(4usize, 4usize), (0, 0), (cw - bw, ch - bh)] {
            for mvx in -12..=12 {
                for mvy in -12..=12 {
                    let mut got = vec![0u8; bw * bh];
                    mc_chroma(&reference, cw, ch, x0, y0, bw, bh, mvx, mvy, &mut got)
                        .expect("plano de prueba completo");
                    let ix0 = x0 as isize + (mvx >> 3) as isize;
                    let iy0 = y0 as isize + (mvy >> 3) as isize;
                    let (fx, fy) = (mvx & 7, mvy & 7);
                    for dy in 0..bh {
                        for dx in 0..bw {
                            assert_eq!(
                                got[dy * bw + dx],
                                pp(ix0 + dx as isize, iy0 + dy as isize, fx, fy),
                                "bw{bw}x{bh} at ({x0},{y0}) mv({mvx},{mvy}) px({dx},{dy})"
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn median_of_three() {
    assert_eq!(median3(1, 2, 3), 2);
    assert_eq!(median3(3, 1, 2), 2);
    assert_eq!(median3(-5, 0, 5), 0);
    assert_eq!(median3(7, 7, 2), 7);
}

#[test]
fn mv_predict_single_neighbor_uses_it() {
    let a = MvNeighbor {
        available: true,
        mv: (8, -4),
        ref_idx: 0,
    };
    // B and C unavailable, A available → predictor is A.
    assert_eq!(
        predict_mv(a, MvNeighbor::NONE, MvNeighbor::NONE, 0),
        (8, -4)
    );
}

#[test]
fn mv_predict_median_when_all_inter() {
    let a = MvNeighbor {
        available: true,
        mv: (4, 0),
        ref_idx: 0,
    };
    let b = MvNeighbor {
        available: true,
        mv: (8, 0),
        ref_idx: 0,
    };
    let c = MvNeighbor {
        available: true,
        mv: (12, 0),
        ref_idx: 0,
    };
    assert_eq!(predict_mv(a, b, c, 0), (8, 0));
}

#[test]
fn mv_predict_one_matching_ref_wins() {
    // Only B references ref 0; A and C are intra → predictor is B.
    let a = MvNeighbor {
        available: true,
        mv: (0, 0),
        ref_idx: -1,
    };
    let b = MvNeighbor {
        available: true,
        mv: (5, 7),
        ref_idx: 0,
    };
    let c = MvNeighbor {
        available: true,
        mv: (0, 0),
        ref_idx: -1,
    };
    assert_eq!(predict_mv(a, b, c, 0), (5, 7));
}

#[test]
fn mv_predict_distinguishes_refs() {
    // A references ref 1, B references ref 0, C intra. For cur_ref 0 only B
    // matches → B; for cur_ref 1 only A matches → A.
    let a = MvNeighbor {
        available: true,
        mv: (4, 4),
        ref_idx: 1,
    };
    let b = MvNeighbor {
        available: true,
        mv: (5, 7),
        ref_idx: 0,
    };
    let c = MvNeighbor {
        available: true,
        mv: (0, 0),
        ref_idx: -1,
    };
    assert_eq!(predict_mv(a, b, c, 0), (5, 7));
    assert_eq!(predict_mv(a, b, c, 1), (4, 4));
}

#[test]
fn mc_luma_zero_mv_copies() {
    let reference = vec![0, 1, 2, 3, 10, 11, 12, 13, 20, 21, 22, 23, 30, 31, 32, 33];
    let mut out = [0u8; 4];
    mc_luma(&reference, 4, 4, 1, 1, 2, 2, 0, 0, &mut out).expect("tile completo");
    assert_eq!(out, [11, 12, 21, 22]);
}

#[test]
fn mc_luma_clamps_at_edges() {
    let reference = vec![5, 6, 7, 8];
    let mut out = [0u8; 4];
    mc_luma(&reference, 2, 2, 0, 0, 2, 2, -40, -40, &mut out).expect("tile completo");
    assert_eq!(out, [5, 5, 5, 5]);
}

/// Plano de 1 muestra declarado 2×2: no cubre `cw × ch`.
const PLANO_CORTO: [u8; 1] = [5];

#[test]
fn mc_luma_plano_corto_entero_es_error() {
    let mut out = [0u8; 4];
    let r = mc_luma(&PLANO_CORTO, 2, 2, 0, 0, 2, 2, -4, -4, &mut out);
    assert!(matches!(r, Err(FueraDeRango(_))), "{r:?}");
}

#[test]
fn mc_luma_plano_corto_subpixel_es_error() {
    let mut out = [0u8; 4];
    let r = mc_luma(&PLANO_CORTO, 2, 2, 0, 0, 2, 2, -3, -3, &mut out);
    assert!(matches!(r, Err(FueraDeRango(_))), "{r:?}");
}

#[test]
fn at_muestra_ausente_es_error() {
    // Aunque la muestra (0,0) exista, el plano no cubre 2×2.
    assert!(matches!(at(&PLANO_CORTO, 2, 2, 0, 0), Err(FueraDeRango(_))));
    assert!(matches!(at(&PLANO_CORTO, 2, 2, 1, 1), Err(FueraDeRango(_))));
    assert_eq!(at(&[5, 6, 7, 8], 2, 2, 1, 1), Ok(8));
    assert!(matches!(at(&[], 0, 0, 0, 0), Err(FueraDeRango(_))));
}

#[test]
fn mc_luma_plano_vacio_es_error() {
    let mut out = [0u8; 4];
    let r = mc_luma(&[], 0, 0, 0, 0, 2, 2, 0, 0, &mut out);
    assert!(matches!(r, Err(FueraDeRango(_))), "{r:?}");
}

#[test]
fn mc_chroma_plano_corto_es_error() {
    let mut out = [0u8; 4];
    let entero = mc_chroma(&PLANO_CORTO, 2, 2, 0, 0, 2, 2, -8, -8, &mut out);
    assert!(matches!(entero, Err(FueraDeRango(_))), "{entero:?}");
    let subpixel = mc_chroma(&PLANO_CORTO, 2, 2, 0, 0, 2, 2, -3, -3, &mut out);
    assert!(matches!(subpixel, Err(FueraDeRango(_))), "{subpixel:?}");
}

#[test]
fn mc_chroma_plano_vacio_es_error() {
    let mut out = [0u8; 4];
    let r = mc_chroma(&[], 0, 0, 0, 0, 2, 2, 0, 0, &mut out);
    assert!(matches!(r, Err(FueraDeRango(_))), "{r:?}");
}

#[test]
fn mc_luma_halfpel_on_flat_is_flat() {
    // A flat reference must interpolate to the same flat value at any frac.
    let reference = vec![100u8; 8 * 8];
    let mut out = [0u8; 16];
    for &(fx, fy) in &[(2, 0), (0, 2), (2, 2), (1, 1), (3, 3)] {
        mc_luma(&reference, 8, 8, 2, 2, 4, 4, fx, fy, &mut out).expect("tile completo");
        assert!(out.iter().all(|&p| p == 100), "frac ({fx},{fy})");
    }
}

#[test]
fn mc_chroma_zero_mv_copies() {
    let reference = vec![0, 1, 2, 3, 10, 11, 12, 13, 20, 21, 22, 23, 30, 31, 32, 33];
    let mut out = [0u8; 4];
    mc_chroma(&reference, 4, 4, 1, 1, 2, 2, 0, 0, &mut out).expect("plano completo");
    assert_eq!(out, [11, 12, 21, 22]);
}

#[test]
fn mc_chroma_bilinear_midpoint() {
    // Horizontal ramp 0,8; chroma frac 4 (half) → midpoint 4.
    let reference = vec![0u8, 8, 0, 8];
    let mut out = [0u8; 1];
    mc_chroma(&reference, 2, 2, 0, 0, 1, 1, 4, 0, &mut out).expect("plano completo");
    assert_eq!(out[0], 4);
}
