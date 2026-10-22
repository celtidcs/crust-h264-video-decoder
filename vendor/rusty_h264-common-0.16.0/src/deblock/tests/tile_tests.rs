use super::*;

/// The per-MB tile must reproduce the frame-array indexing exactly. This is
/// where a transcription slip would hide: the tile's (row, col) origin is the
/// top-left NEIGHBOUR, so every edge lookup is offset by one, and chroma edge
/// groups index it at half the luma rate.
#[test]
fn tile_matches_frame_indexing() {
    let (mb_w, mb_h) = (5usize, 4usize);
    let w4 = mb_w * 4;
    let n = w4 * mb_h * 4;
    let mut st = 0xdeadbeefu32;
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
            for be in 0..4 {
                let mb_edge = be == 0;
                if mb_edge && mb_x == 0 {
                    continue;
                }
                // luma vertical
                let abx = mb_x * 4 + be;
                for seg in 0..4 {
                    let aby = mb_y * 4 + seg;
                    assert_eq!(
                        info.bs(info.at(abx - 1, aby), info.at(abx, aby), mb_edge),
                        bs_tile(&tile[seg + 1][be], &tile[seg + 1][be + 1], mb_edge),
                        "luma V mb=({mb_x},{mb_y}) be={be} seg={seg}"
                    );
                    checked += 1;
                }
            }
            for be in 0..4 {
                let mb_edge = be == 0;
                if mb_edge && mb_y == 0 {
                    continue;
                }
                let aby = mb_y * 4 + be;
                for seg in 0..4 {
                    let abx = mb_x * 4 + seg;
                    assert_eq!(
                        info.bs(info.at(abx, aby - 1), info.at(abx, aby), mb_edge),
                        bs_tile(&tile[be][seg + 1], &tile[be + 1][seg + 1], mb_edge),
                        "luma H mb=({mb_x},{mb_y}) be={be} seg={seg}"
                    );
                    checked += 1;
                }
            }
            // chroma groups index the tile at half the luma rate
            for cxe in [0usize, 4] {
                let mb_edge = cxe == 0;
                if mb_edge && mb_x == 0 {
                    continue;
                }
                let abx = mb_x * 4 + cxe / 2;
                for seg in 0..4 {
                    let aby = mb_y * 4 + seg;
                    assert_eq!(
                        info.bs(info.at(abx - 1, aby), info.at(abx, aby), mb_edge),
                        bs_tile(
                            &tile[seg + 1][cxe / 2],
                            &tile[seg + 1][cxe / 2 + 1],
                            mb_edge
                        ),
                        "chroma V mb=({mb_x},{mb_y}) cxe={cxe} seg={seg}"
                    );
                    checked += 1;
                }
            }
            for cye in [0usize, 4] {
                let mb_edge = cye == 0;
                if mb_edge && mb_y == 0 {
                    continue;
                }
                let aby = mb_y * 4 + cye / 2;
                for seg in 0..4 {
                    let abx = mb_x * 4 + seg;
                    assert_eq!(
                        info.bs(info.at(abx, aby - 1), info.at(abx, aby), mb_edge),
                        bs_tile(
                            &tile[cye / 2][seg + 1],
                            &tile[cye / 2 + 1][seg + 1],
                            mb_edge
                        ),
                        "chroma H mb=({mb_x},{mb_y}) cye={cye} seg={seg}"
                    );
                    checked += 1;
                }
            }
        }
    }
    assert!(checked > 800, "coverage too low: {checked}");
}
