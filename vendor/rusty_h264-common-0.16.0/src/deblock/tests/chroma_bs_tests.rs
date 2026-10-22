use super::*;

/// A chroma edge group is CO-LOCATED with a luma edge group — chroma edge 0
/// with luma edge 0, chroma edge 4 with luma edge 2 — and derives the
/// identical boundary strengths, because bS is a property of the 4×4 block
/// pair, not of the plane. This test is the licence for the derivation to run
/// once per luma edge and be reused by chroma; without it, 16 of the 48
/// per-macroblock derivations are recomputes.
#[test]
fn chroma_bs_matches_luma() {
    let (mb_w, mb_h) = (5usize, 4usize);
    let w4 = mb_w * 4;
    let n = w4 * mb_h * 4;
    let mut st = 0x1badb002u32;
    let mut rnd = || {
        st ^= st << 13;
        st ^= st >> 17;
        st ^= st << 5;
        st
    };
    let (mut inter, mut nnz) = (vec![false; n], vec![0u8; n]);
    let (mut mv, mut ref_id) = (vec![(0i32, 0i32); n], vec![0i32; n]);
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
    let mut checked = 0;
    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            let tile = gather_tile(&info, mb_x, mb_y).expect("rejilla completa");
            for (cxe, be) in [(0usize, 0usize), (4, 2)] {
                let mb_edge = cxe == 0;
                for seg in 0..4 {
                    // vertical: chroma column cxe/2 == luma column `be`
                    assert_eq!(
                        bs_tile(
                            &tile[seg + 1][cxe / 2],
                            &tile[seg + 1][cxe / 2 + 1],
                            mb_edge
                        ),
                        bs_tile(&tile[seg + 1][be], &tile[seg + 1][be + 1], mb_edge),
                        "V mb=({mb_x},{mb_y}) cxe={cxe} seg={seg}"
                    );
                    // horizontal: chroma row cye/2 == luma row `be`
                    assert_eq!(
                        bs_tile(
                            &tile[cxe / 2][seg + 1],
                            &tile[cxe / 2 + 1][seg + 1],
                            mb_edge
                        ),
                        bs_tile(&tile[be][seg + 1], &tile[be + 1][seg + 1], mb_edge),
                        "H mb=({mb_x},{mb_y}) cye={cxe} seg={seg}"
                    );
                    checked += 2;
                }
            }
        }
    }
    assert!(checked > 300, "coverage too low: {checked}");
}
