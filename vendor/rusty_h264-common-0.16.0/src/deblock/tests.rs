use super::*;

/// The contiguous-window vertical-edge filters must be BIT-IDENTICAL to
/// the strided originals (which remain the horizontal-path production
/// code, so the oracle is live code, not a frozen copy). Randomized planes
/// and parameters, both `bs < 4` and `bs == 4` arms, thresholds spanning
/// pass/fail so the early-return path is covered too.
#[cfg(not(accel))]
#[test]
fn contig_line_filters_match_strided() {
    let mut st = 0xfeed_f00du32;
    let mut rnd = || {
        st ^= st << 13;
        st ^= st >> 17;
        st ^= st << 5;
        st
    };
    for case in 0..4000 {
        let n = 64usize;
        let mut a: Vec<u8> = (0..n).map(|_| (rnd() & 0xff) as u8).collect();
        let mut b = a.clone();
        let base = 8 + (rnd() as usize) % 48; // window stays inside [4, 60)
        let bs = 1 + (rnd() % 4) as i32; // 1..=4
        let alpha = (rnd() % 60) as i32;
        let beta = (rnd() % 20) as i32;
        let tc0 = (rnd() % 12) as i32;
        filter_luma_line(&mut a, &Line { base, step: 1 }, bs, alpha, beta, tc0);
        filter_luma_line_contig(&mut b, base, bs, alpha, beta, tc0);
        assert_eq!(a, b, "luma case {case}");
        filter_chroma_line(&mut a, &Line { base, step: 1 }, bs, alpha, beta, tc0);
        filter_chroma_line_contig(&mut b, base, bs, alpha, beta, tc0);
        assert_eq!(a, b, "chroma case {case}");
    }
}

/// The two-list AVX2 masks kernel vs the scalar set-matching twin, over
/// randomized two-list inputs. Unused slots deliberately carry GARBAGE
/// motion: the scalar twin ignores it via slot compaction, and the kernel
/// must neutralize it — equality here proves the neutralization.
#[cfg(all(target_arch = "x86_64", feature = "asm"))]
#[test]
fn two_list_masks_match_scalar() {
    let mut seed = 0x2545_F491u32;
    let mut rnd = move || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed
    };
    for _ in 0..50_000 {
        let mut p = MbPack {
            inter: true,
            ..Default::default()
        };
        for k in 0..16 {
            let r = rnd();
            p.ref_id[k] = match r & 3 {
                0 => NO_REF,
                1 => 10,
                2 => 20,
                _ => 30,
            };
            p.ref1[k] = match (r >> 2) & 3 {
                0 => NO_REF,
                1 => 10,
                2 => 40,
                _ => 20,
            };
            // Motion near the ±4 threshold; unused slots get garbage on purpose.
            p.mvx[k] = ((r >> 4) % 11) as i16 - 5;
            p.mvy[k] = ((r >> 8) % 11) as i16 - 5;
            p.mvx1[k] = ((r >> 12) % 11) as i16 - 5;
            p.mvy1[k] = ((r >> 16) % 11) as i16 - 5;
            if p.ref1[k] != NO_REF {
                p.l1_used |= 1 << k;
            }
        }
        let scalar = bs_motion_masks_scalar(&p);
        let simd = rusty_h264_accel::bs_motion_masks_two_list(
            &p.mvx, &p.mvy, &p.ref_id, &p.mvx1, &p.mvy1, &p.ref1, NO_REF,
        )
        .expect("AVX2 present on the test box");
        assert_eq!(
            simd, scalar,
            "l1_used={:04x} ref0={:?} ref1={:?}",
            p.l1_used, p.ref_id, p.ref1
        );
    }
}

/// The two boundary-strength arms must be the same function. `bs_branchless`
/// is the shipped path and `bs_branchy` the fallback/A-B arm, so any
/// divergence would silently change the filtered reconstruction — and, since
/// the reconstruction is the inter prediction reference, the bitstream.
#[test]
fn bs_arms_agree() {
    let (w4, h4) = (16usize, 16usize);
    let n = w4 * h4;
    // Deterministic pseudo-random block state covering every branch: intra vs
    // inter, coded vs uncoded, matching vs differing refs, near vs far motion.
    let mut st = 0x9e3779b9u32;
    let mut rnd = || {
        st ^= st << 13;
        st ^= st >> 17;
        st ^= st << 5;
        st
    };
    let mut inter = vec![false; n];
    let mut nnz = vec![0u8; n];
    let mut mv = vec![(0i32, 0i32); n];
    let mut ref_id = vec![0i32; n];
    // `mb_type` is a per-macroblock syntax element: every 4x4 block of a
    // macroblock is intra or inter together, and the tile derivation
    // asserts that (debug builds). Draw the flag once per MB.
    let (mbw, mbh) = (w4 / 4, n / (w4 * 4));
    let mb_inter: Vec<bool> = (0..mbw * mbh).map(|_| rnd() & 3 != 0).collect();
    let mb_of = |i: usize| (i / (w4 * 4)) * mbw + (i % w4) / 4;
    for i in 0..n {
        let r = rnd();
        inter[i] = mb_inter[mb_of(i)];
        nnz[i] = if r & 0x30 != 0 {
            (r >> 8 & 15) as u8
        } else {
            0
        };
        // Span the |Δ| >= 4 boundary in both components.
        mv[i] = (((r >> 12) & 15) as i32 - 8, ((r >> 16) & 15) as i32 - 8);
        ref_id[i] = if inter[i] {
            ((r >> 20) & 3) as i32
        } else {
            NO_REF
        };
    }
    let info = BlockInfo {
        inter: &inter,
        nnz: &nnz,
        mv: &mv,
        ref_id: &ref_id,
        mv1: &[],
        ref_id1: &[],
        w4,
        t8x8: &[],
        poc0: &[],
        poc1: &[],
        bs: &[],
        kind: &[],
        desplazamientos: &[],
    };
    let mut checked = 0;
    for q in 0..n {
        for &p in &[q.saturating_sub(1), q.saturating_sub(w4)] {
            for mb_edge in [false, true] {
                assert_eq!(
                    info.bs_branchy(p, q, mb_edge),
                    info.bs_branchless(p, q, mb_edge),
                    "bS mismatch at p={p} q={q} mb_edge={mb_edge}"
                );
                checked += 1;
            }
        }
    }
    assert!(checked > 1000, "expected broad coverage, checked {checked}");
}

/// Índice ausente del mapa de POC: error, no «sin referencia» (COR-3, `map_ref`).
#[test]
fn map_ref_indice_ausente_es_error() {
    let mapa = [40, 38];
    assert_eq!(map_ref(true, &mapa, 1), Ok(38));
    assert!(matches!(map_ref(true, &mapa, 2), Err(FueraDeRango(_))));
    assert_eq!(map_ref(true, &mapa, -1), Ok(NO_REF));
    assert_eq!(map_ref(false, &[], 7), Ok(7));
}

/// `t8x8` no vacío pero más corto que la imagen: error, no «sin 8×8» (COR-3).
#[test]
fn precompute_t8x8_corto_es_error() {
    let (w4, n) = (8usize, 32usize);
    let (inter, nnz) = (vec![true; n], vec![1u8; n]);
    let (mv, ref_id) = (vec![(0i32, 0i32); n], vec![0i32; n]);
    let con = |t8x8: &[bool]| {
        let info = BlockInfo {
            inter: &inter,
            nnz: &nnz,
            mv: &mv,
            ref_id: &ref_id,
            mv1: &[],
            ref_id1: &[],
            w4,
            t8x8,
            poc0: &[],
            poc1: &[],
            bs: &[],
            kind: &[],
            desplazamientos: &[],
        };
        let mut bs = Vec::new();
        precompute_bs_frame(&info, 2, 1, &mut bs).map(|()| bs)
    };
    let sin_8x8 = con(&[false, false]).expect("t8x8 completo");
    assert_eq!(con(&[]).expect("t8x8 vacío"), sin_8x8);
    assert_ne!(con(&[true, true]).expect("t8x8 completo"), sin_8x8);
    assert!(matches!(con(&[false]), Err(FueraDeRango(_))));
}
