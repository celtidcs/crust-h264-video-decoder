//! In-loop deblocking filter (spec §8.7), all-intra case.
//!
//! Smooths block-edge discontinuities on the reconstructed frame. Because intra
//! prediction uses *pre*-deblocking samples, this runs as a post-pass over the
//! fully-reconstructed frame: macroblocks in raster order, vertical edges then
//! horizontal, filtered in place. For an all-intra picture the boundary
//! strength is positional — 4 on macroblock edges, 3 on internal 4×4 edges.

#[allow(unused_imports)]
use alloc::vec;
use alloc::vec::Vec;

use crate::rango::FueraDeRango;

/// `α` threshold indexed by `indexA` (= clipped QP), spec Table 8-16.
#[rustfmt::skip]
const ALPHA: [i32; 52] = [
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    4,4,5,6,7,8,9,10,12,13,15,17,20,22,25,28,
    32,36,40,45,50,56,63,71,80,90,101,113,127,144,162,182,203,226,255,255,
];

/// `β` threshold indexed by `indexB`.
#[rustfmt::skip]
const BETA: [i32; 52] = [
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    2,2,2,3,3,3,3,4,4,4,6,6,7,7,8,8,
    9,9,10,10,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,
];

/// `tc0` indexed by `[indexA][bS-1]` for bS ∈ {1,2,3}.
#[rustfmt::skip]
const TC0: [[i32; 3]; 52] = [
    [0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],
    [0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],
    [0,0,0],[0,0,1],[0,0,1],[0,0,1],[0,0,1],[0,1,1],[0,1,1],[1,1,1],
    [1,1,1],[1,1,1],[1,1,1],[1,1,2],[1,1,2],[1,1,2],[1,1,2],[1,2,3],
    [1,2,3],[2,2,3],[2,2,4],[2,3,4],[2,3,4],[3,3,5],[3,4,6],[3,4,6],
    [4,5,7],[4,5,8],[4,6,9],[5,7,10],[6,8,11],[6,8,13],[7,10,14],[8,11,16],
    [9,12,18],[10,13,20],[11,15,23],[13,17,25],
];

// The scalar per-line filters below are the `#[cfg(not(accel))]` production
// path in `filter_frame_rows` (default-features build — no SIMD crate). Under
// the accel cfg they compile out entirely, so don't let an --all-features
// dead-code pass fool you into deleting them (it did, once).
#[cfg(not(accel))]
#[inline]
fn clip1(v: i32) -> u8 {
    v.clamp(0, 255) as u8
}

#[cfg(not(accel))]
#[inline]
fn clip3(lo: i32, hi: i32, v: i32) -> i32 {
    v.clamp(lo, hi)
}

/// One sample line crossing an edge: `p3..p0 | q0..q3` (indices 0..3 from the
/// edge outward). Reads/writes a plane along `stride`-spaced positions.
#[cfg(not(accel))]
struct Line {
    /// Byte offset of q0 (the first sample on the "right"/"below" side).
    base: usize,
    /// Step between adjacent samples across the edge (1 horizontally, `stride`
    /// vertically).
    step: isize,
}

/// Filters luma samples across one edge line. `bs` is 3 (internal) or 4 (MB edge).
#[cfg(not(accel))]
#[allow(clippy::too_many_arguments)]
fn filter_luma_line(plane: &mut [u8], line: &Line, bs: i32, alpha: i32, beta: i32, tc0: i32) {
    let at = |i: isize| -> i32 { plane[(line.base as isize + i * line.step) as usize] as i32 };
    let (p0, p1, p2, p3) = (at(-1), at(-2), at(-3), at(-4));
    let (q0, q1, q2, q3) = (at(0), at(1), at(2), at(3));

    if (p0 - q0).abs() >= alpha || (p1 - p0).abs() >= beta || (q1 - q0).abs() >= beta {
        return;
    }
    let set = |plane: &mut [u8], i: isize, v: u8| {
        plane[(line.base as isize + i * line.step) as usize] = v;
    };
    let ap = (p2 - p0).abs();
    let aq = (q2 - q0).abs();

    if bs < 4 {
        let tc = tc0 + (ap < beta) as i32 + (aq < beta) as i32;
        let delta = clip3(-tc, tc, (((q0 - p0) << 2) + (p1 - q1) + 4) >> 3);
        set(plane, -1, clip1(p0 + delta));
        set(plane, 0, clip1(q0 - delta));
        if ap < beta {
            let d = clip3(-tc0, tc0, (p2 + ((p0 + q0 + 1) >> 1) - (p1 << 1)) >> 1);
            set(plane, -2, clip1(p1 + d));
        }
        if aq < beta {
            let d = clip3(-tc0, tc0, (q2 + ((p0 + q0 + 1) >> 1) - (q1 << 1)) >> 1);
            set(plane, 1, clip1(q1 + d));
        }
    } else {
        let strong = (p0 - q0).abs() < (alpha >> 2) + 2;
        if strong && ap < beta {
            set(
                plane,
                -1,
                clip1((p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) >> 3),
            );
            set(plane, -2, clip1((p2 + p1 + p0 + q0 + 2) >> 2));
            set(plane, -3, clip1((2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) >> 3));
        } else {
            set(plane, -1, clip1((2 * p1 + p0 + q1 + 2) >> 2));
        }
        if strong && aq < beta {
            set(
                plane,
                0,
                clip1((q2 + 2 * q1 + 2 * q0 + 2 * p0 + p1 + 4) >> 3),
            );
            set(plane, 1, clip1((q2 + q1 + q0 + p0 + 2) >> 2));
            set(plane, 2, clip1((2 * q3 + 3 * q2 + q1 + q0 + p0 + 4) >> 3));
        } else {
            set(plane, 0, clip1((2 * q1 + q0 + p1 + 2) >> 2));
        }
    }
}

/// [`filter_luma_line`] specialized for VERTICAL edges (`step == 1`): the
/// eight samples `p3..p0|q0..q3` are the CONSECUTIVE bytes around `base`, so
/// one fixed-extent window (`&mut x[a..][..8]` — the shape whose length LLVM
/// keeps literal) replaces eight checked strided indexings. Bit-identical to
/// the strided form by `contig_line_filters_match_strided`, which keeps the
/// strided original (still the horizontal-path production code) as the oracle.
#[cfg(not(accel))]
fn filter_luma_line_contig(
    plane: &mut [u8],
    base: usize,
    bs: i32,
    alpha: i32,
    beta: i32,
    tc0: i32,
) {
    let w = &mut plane[base - 4..][..8];
    let (p0, p1, p2, p3) = (w[3] as i32, w[2] as i32, w[1] as i32, w[0] as i32);
    let (q0, q1, q2, q3) = (w[4] as i32, w[5] as i32, w[6] as i32, w[7] as i32);
    if (p0 - q0).abs() >= alpha || (p1 - p0).abs() >= beta || (q1 - q0).abs() >= beta {
        return;
    }
    let ap = (p2 - p0).abs();
    let aq = (q2 - q0).abs();
    if bs < 4 {
        let tc = tc0 + (ap < beta) as i32 + (aq < beta) as i32;
        let delta = clip3(-tc, tc, (((q0 - p0) << 2) + (p1 - q1) + 4) >> 3);
        w[3] = clip1(p0 + delta);
        w[4] = clip1(q0 - delta);
        if ap < beta {
            let d = clip3(-tc0, tc0, (p2 + ((p0 + q0 + 1) >> 1) - (p1 << 1)) >> 1);
            w[2] = clip1(p1 + d);
        }
        if aq < beta {
            let d = clip3(-tc0, tc0, (q2 + ((p0 + q0 + 1) >> 1) - (q1 << 1)) >> 1);
            w[5] = clip1(q1 + d);
        }
    } else {
        let strong = (p0 - q0).abs() < (alpha >> 2) + 2;
        if strong && ap < beta {
            w[3] = clip1((p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) >> 3);
            w[2] = clip1((p2 + p1 + p0 + q0 + 2) >> 2);
            w[1] = clip1((2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) >> 3);
        } else {
            w[3] = clip1((2 * p1 + p0 + q1 + 2) >> 2);
        }
        if strong && aq < beta {
            w[4] = clip1((q2 + 2 * q1 + 2 * q0 + 2 * p0 + p1 + 4) >> 3);
            w[5] = clip1((q2 + q1 + q0 + p0 + 2) >> 2);
            w[6] = clip1((2 * q3 + 3 * q2 + q1 + q0 + p0 + 4) >> 3);
        } else {
            w[4] = clip1((2 * q1 + q0 + p1 + 2) >> 2);
        }
    }
}

/// [`filter_chroma_line`] specialized for VERTICAL edges — the 4-byte twin of
/// [`filter_luma_line_contig`], same oracle test.
#[cfg(not(accel))]
fn filter_chroma_line_contig(
    plane: &mut [u8],
    base: usize,
    bs: i32,
    alpha: i32,
    beta: i32,
    tc0: i32,
) {
    let w = &mut plane[base - 2..][..4];
    let (p0, p1) = (w[1] as i32, w[0] as i32);
    let (q0, q1) = (w[2] as i32, w[3] as i32);
    if (p0 - q0).abs() >= alpha || (p1 - p0).abs() >= beta || (q1 - q0).abs() >= beta {
        return;
    }
    if bs < 4 {
        let tc = tc0 + 1;
        let delta = clip3(-tc, tc, (((q0 - p0) << 2) + (p1 - q1) + 4) >> 3);
        w[1] = clip1(p0 + delta);
        w[2] = clip1(q0 - delta);
    } else {
        w[1] = clip1((2 * p1 + p0 + q1 + 2) >> 2);
        w[2] = clip1((2 * q1 + q0 + p1 + 2) >> 2);
    }
}

/// Filters chroma samples across one edge line (only p0/q0 are modified).
#[cfg(not(accel))]
fn filter_chroma_line(plane: &mut [u8], line: &Line, bs: i32, alpha: i32, beta: i32, tc0: i32) {
    let at = |i: isize| -> i32 { plane[(line.base as isize + i * line.step) as usize] as i32 };
    let (p0, p1) = (at(-1), at(-2));
    let (q0, q1) = (at(0), at(1));
    if (p0 - q0).abs() >= alpha || (p1 - p0).abs() >= beta || (q1 - q0).abs() >= beta {
        return;
    }
    let set = |plane: &mut [u8], i: isize, v: u8| {
        plane[(line.base as isize + i * line.step) as usize] = v;
    };
    if bs < 4 {
        let tc = tc0 + 1;
        let delta = clip3(-tc, tc, (((q0 - p0) << 2) + (p1 - q1) + 4) >> 3);
        set(plane, -1, clip1(p0 + delta));
        set(plane, 0, clip1(q0 - delta));
    } else {
        set(plane, -1, clip1((2 * p1 + p0 + q1 + 2) >> 2));
        set(plane, 0, clip1((2 * q1 + q0 + p1 + 2) >> 2));
    }
}

/// `FilterOffsetA` y `FilterOffsetB` de un macrobloque (UIT-T H.264, §7.4.3): el doble de
/// `slice_alpha_c0_offset_div2` y `slice_beta_offset_div2` del corte al que pertenece.
///
/// Invariante: ambos valores están en `[-LIMITE, LIMITE]`; solo [`Self::nuevo`] los construye.
/// Caben en `i8`, así que la tabla de una imagen ocupa dos bytes por macrobloque.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DesplazamientosFiltro {
    a: i8,
    b: i8,
}

impl DesplazamientosFiltro {
    /// Valor absoluto máximo de `FilterOffsetA`/`FilterOffsetB`: los `*_offset_div2` van de −6 a
    /// 6 (§7.4.3) y se multiplican por dos.
    pub const LIMITE: i32 = 12;

    /// Sin desplazamiento: lo que infiere §7.4.3 cuando el corte no los transmite.
    pub const CERO: Self = Self { a: 0, b: 0 };

    /// Construye el par si ambos valores están en `[-LIMITE, LIMITE]`.
    ///
    /// Devuelve `None` si alguno se sale: el llamador decide qué error de su capa le corresponde.
    #[must_use]
    pub fn nuevo(a: i32, b: i32) -> Option<Self> {
        let en_rango = |v: i32| {
            i8::try_from(v)
                .ok()
                .filter(|v| i32::from(*v).abs() <= Self::LIMITE)
        };
        Some(Self {
            a: en_rango(a)?,
            b: en_rango(b)?,
        })
    }

    /// `FilterOffsetA`: desplaza `indexA` (umbral α y `tC0`).
    #[must_use]
    pub fn a(self) -> i32 {
        i32::from(self.a)
    }

    /// `FilterOffsetB`: desplaza `indexB` (umbral β).
    #[must_use]
    pub fn b(self) -> i32 {
        i32::from(self.b)
    }
}

/// Per-4×4-block macroblock info driving boundary-strength derivation.
pub struct BlockInfo<'a> {
    /// `true` if the block is **inter**-coded (an intra block is `!inter`). Taking
    /// the caller's existing inter mask avoids allocating an inverted intra mask
    /// per frame in both the decoder and encoder.
    pub inter: &'a [bool],
    /// Non-zero coefficient count of the block.
    pub nnz: &'a [u8],
    /// List-0 block motion vector (quarter-pel); ignored for intra.
    pub mv: &'a [(i32, i32)],
    /// List-0 reference *picture identity* (a stable per-picture id — PicOrderCnt
    /// for the decoder, ref index for the encoder; `i32::MIN` = unused/intra).
    /// Boundary strength compares the *set* of reference pictures, so the same
    /// picture used via different lists matches (spec §8.7.2.1).
    pub ref_id: &'a [i32],
    /// List-1 motion + reference identity for B blocks (`ref_id1 = i32::MIN`
    /// everywhere for P/I, so the extra slot is a no-op there).
    pub mv1: &'a [(i32, i32)],
    pub ref_id1: &'a [i32],
    /// Block-grid width (`mb_w * 4`).
    pub w4: usize,
    /// Per-macroblock `transform_size_8x8_flag` (length `mb_w * mb_h`). When set,
    /// the macroblock's internal 4×4 luma edges (sample columns/rows 4 and 12)
    /// are *not* transform boundaries and must not be filtered (spec §8.7).
    pub t8x8: &'a [bool],
    /// Boundary strengths already derived by the caller, one entry per macroblock
    /// in raster order. Empty = derive them here (the decoder path).
    pub bs: &'a [MbBs],
    /// OPTIONAL reference→identity maps (Brick: WHYS Part 15 item 2). When
    /// non-empty, `ref_id`/`ref_id1` hold RAW per-block reference INDICES
    /// (negative = none) and these ≤16-entry tables map index → picture
    /// identity (POC). This lets the decoder pass its `ref_idx` grids directly
    /// instead of materializing two frame-wide pre-mapped `Vec<i32>` shims
    /// (230-460 KB + 57,600 mapped elements per frame). Empty = `ref_id`
    /// already carries identities (the encoder path and all older callers).
    pub poc0: &'a [i32],
    pub poc1: &'a [i32],
    /// Per-macroblock derivation CLASS (`MB_KIND_*`), one entry per macroblock in
    /// raster order. Empty, or `MB_KIND_UNSET`, means "not classified" and the
    /// blind gather+derive runs for that macroblock.
    ///
    /// SAFETY-BY-DESIGN: an unclassified macroblock costs SPEED, never
    /// correctness. That inversion is deliberate — the alternative design, where
    /// the producer must hit every macroblock exit point or strengths silently go
    /// wrong, is exactly the failure `MbBs::UNSET` exists to guard against.
    ///
    /// The producer must be CONSERVATIVE. In particular `B_Skip` and
    /// `B_Direct_16x16` are NOT `MB_KIND_SKIP`: their motion comes from direct
    /// prediction and may differ per 4x4 sub-block, so internal edges can legally
    /// reach strength 1. Only classify what is uniform BY SYNTAX.
    pub kind: &'a [u8],
    /// Desplazamientos del filtro de cada macrobloque en orden de barrido (longitud
    /// `mb_w * mb_h`), tomados de SU corte: cada borde se filtra con los del macrobloque que
    /// contiene `q0` (§8.7.2). Solo los leen las funciones que filtran; la derivación de `bS`
    /// los ignora y admite una tabla vacía.
    pub desplazamientos: &'a [DesplazamientosFiltro],
}

/// Not classified — take the blind path. Any value outside `0..=3` behaves this way.
pub const MB_KIND_UNSET: u8 = 255;
pub const MB_KIND_INTRA: u8 = 0;
pub const MB_KIND_SKIP: u8 = 1;
pub const MB_KIND_INTER_UNIFORM: u8 = 2;
pub const MB_KIND_INTER: u8 = 3;

impl MbKind {
    /// Decode a producer-supplied class byte. `None` = derive blindly.
    #[inline]
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            MB_KIND_INTRA => Some(MbKind::Intra),
            MB_KIND_SKIP => Some(MbKind::Skip),
            MB_KIND_INTER_UNIFORM => Some(MbKind::InterUniform),
            MB_KIND_INTER => Some(MbKind::Inter),
            _ => None,
        }
    }
}

/// One macroblock's boundary strengths: `[edge group][segment]` per direction.
///
/// 32 bytes, mirroring x264's `uint8_t bs[2][8][4]`. Deriving these during ENCODE
/// lets the deblocking pass skip the neighbourhood gather and the derivation.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct MbBs {
    /// Vertical edges (block columns 0..4); index 0 is the macroblock edge.
    pub v: [[u8; 4]; 4],
    /// Horizontal edges (block rows 0..4); index 0 is the macroblock edge.
    pub h: [[u8; 4]; 4],
}

impl MbBs {
    /// "Not yet derived". The macroblock loop has several exits (free skip,
    /// greedy skip, coded) and every one must store its strengths; missing one
    /// leaves zeros, silently disabling deblocking for that macroblock — which is
    /// exactly the bug the byte-identical gate caught during bring-up.
    pub const UNSET: MbBs = MbBs {
        v: [[0xFF; 4]; 4],
        h: [[0xFF; 4]; 4],
    };
}

/// Sentinel for an unused reference slot.
const NO_REF: i32 = i32::MIN;

/// One 4×4 block's deblocking state, gathered into a per-macroblock tile.
///
/// The frame-wide `inter`/`nnz`/`mv`/`ref_id` arrays are indexed `by * w4 + bx`,
/// so deriving boundary strengths straight from them costs ~290 scattered loads
/// per macroblock (each block is re-read by up to four edges, and vertical edge
/// groups stride by `w4`). Gathering the 24 blocks an MB can touch into this
/// contiguous tile once turns all 48 edge decisions into stack/L1 reads. x264
/// gets the same effect from its `scan8` cache, which is what lets its
/// `deblock_strength` kernel be SIMD at ~15 ns/MB.
#[derive(Clone, Copy, Default)]
struct Blk {
    inter: bool,
    /// `nnz != 0` — the count itself never matters, only whether it is non-zero.
    nz: bool,
    ref_id: i32,
    mvx: i32,
    mvy: i32,
    /// List-1 slot (B slices). `NO_REF` on P/I tiles and on uni-L0 blocks, which
    /// keeps every comparison below on its single-list fast path there.
    ref1: i32,
    mv1x: i32,
    mv1y: i32,
}

/// `core::array::from_fn` para cuatro elementos cuyo productor puede fallar: la
/// tabla, o el primer error (ningún elemento se inventa).
///
/// # Errors
/// El primer `Err` que devuelva `f`.
#[inline]
fn intentar_4<T: Copy + Default, E>(mut f: impl FnMut(usize) -> Result<T, E>) -> Result<[T; 4], E> {
    let mut a = [T::default(); 4];
    for (i, v) in a.iter_mut().enumerate() {
        *v = f(i)?;
    }
    Ok(a)
}

impl Blk {
    /// REFUTED THREE WAYS — do NOT try to fold these checks again. The six
    /// grids below are separate slices at one index, 33 panic paths, the densest
    /// cluster in either crate. Every technique that removes them costs MORE than
    /// they do, because `gather_tile` calls this 24 times per macroblock and any
    /// per-field fallback is paid 144 times:
    ///
    /// | technique                              | panics | instrs (gather_tile) |
    /// | -------------------------------------- | ------ | -------------------- |
    /// | baseline (this code)                   |   33   |   983                |
    /// | caller-side window slice of all 6 grids |   26   |  1431  (+57.6%)      |
    /// | `.get` + `let ... else { return }`      |    0   |  2410  (+145%)       |
    /// | `.get` + branchless `unwrap_or`         |    0   |  1521  (+54.7%)      |
    ///
    /// Three independent shapes converging on the same answer is the finding:
    /// these checks are cheap, perfectly predicted, and at their floor. Only an
    /// AoS layout (one array of block records instead of six parallel grids)
    /// could change it, and that is a data-structure change, not a fold.
    ///
    /// COR-3 (WoDW, acordado con el verificador): los `unwrap_or` de arriba hacían que
    /// un bloque ausente se leyera como «intra, sin coeficientes, sin movimiento»: un
    /// bloque inventado. Ahora es `FueraDeRango`; su coste se mide en tiempo de
    /// decodificación de flujos completos (velocidad.py), no contando instrucciones.
    ///
    /// # Errors
    /// `FueraDeRango` si alguna rejilla no cubre el índice `i`.
    #[inline]
    fn load(info: &BlockInfo, i: usize) -> Result<Self, FueraDeRango> {
        let (mvx, mvy) = *info.mv.get(i).ok_or(FueraDeRango("mv"))?;
        let (ref1, (mv1x, mv1y)) = if info.ref_id1.is_empty() {
            (NO_REF, (0, 0))
        } else {
            (
                info.rid1_at(i)?,
                *info.mv1.get(i).ok_or(FueraDeRango("mv1"))?,
            )
        };
        Ok(Blk {
            inter: *info.inter.get(i).ok_or(FueraDeRango("inter"))?,
            nz: *info.nnz.get(i).ok_or(FueraDeRango("nnz"))? != 0,
            ref_id: info.rid_at(i)?,
            mvx,
            mvy,
            ref1,
            mv1x,
            mv1y,
        })
    }

    /// Whether two blocks are identical for flat-inter purposes (both lists —
    /// on P tiles the list-1 fields are uniformly `NO_REF`/0, so the extra
    /// compares are always-true and the P behaviour is unchanged).
    #[inline]
    fn same_motion(&self, o: &Blk) -> bool {
        self.ref_id == o.ref_id
            && self.mvx == o.mvx
            && self.mvy == o.mvy
            && self.ref1 == o.ref1
            && self.mv1x == o.mv1x
            && self.mv1y == o.mv1y
    }
}

/// The bS==1 motion test on tile entries — mirrors [`BlockInfo::inter_bs1`]
/// exactly (its oracle), including the two-slot B rule, but reads registers
/// instead of strided frame arrays. The single-list fast path fires whenever
/// neither side carries a List-1 slot: all of P, and B edges between uni-L0
/// blocks (equivalent to the general rule at n<=1 — a differing ref covers the
/// differing-slot-count case, two unused slots give false).
#[inline]
fn bs1_tile(p: &Blk, q: &Blk) -> bool {
    if (p.ref1 == NO_REF) & (q.ref1 == NO_REF) {
        // WIN: same |d| >= 4 identity as `pk_differs`, of which this is the per-block
        // twin. `Blk` holds i32 motion, hence the wider cast here.
        let far4 = |d: i32| (d.wrapping_add(3) as u32) >= 7;
        let far = far4(p.mvx - q.mvx) | far4(p.mvy - q.mvy);
        return (p.ref_id != q.ref_id) | ((p.ref_id != NO_REF) & far);
    }
    let used = |b: &Blk| {
        let mut v = [(0i32, (0i32, 0i32)); 2];
        let mut n = 0usize;
        if b.ref_id != NO_REF {
            v[n] = (b.ref_id, (b.mvx, b.mvy));
            n += 1;
        }
        if b.ref1 != NO_REF {
            v[n] = (b.ref1, (b.mv1x, b.mv1y));
            n += 1;
        }
        (v, n)
    };
    let (pv, pn) = used(p);
    let (qv, qn) = used(q);
    if pn != qn {
        return true;
    }
    // WIN: same |d| >= 4 identity, and `|` rather than `||` so the second component
    // carries no branch of its own.
    let far = |a: (i32, i32), b: (i32, i32)| {
        let f = |d: i32| (d.wrapping_add(3) as u32) >= 7;
        f(a.0 - b.0) | f(a.1 - b.1)
    };
    match pn {
        0 => false,
        1 => pv[0].0 != qv[0].0 || far(pv[0].1, qv[0].1),
        _ => {
            let direct = !far(pv[0].1, qv[0].1) && !far(pv[1].1, qv[1].1);
            let swap = !far(pv[0].1, qv[1].1) && !far(pv[1].1, qv[0].1);
            if pv[0].0 == pv[1].0 {
                qv[0].0 != pv[0].0 || qv[1].0 != pv[0].0 || !(direct || swap)
            } else if pv[0].0 == qv[0].0 && pv[1].0 == qv[1].0 {
                !direct
            } else if pv[0].0 == qv[1].0 && pv[1].0 == qv[0].0 {
                !swap
            } else {
                true
            }
        }
    }
}

/// Boundary strength from tile entries (spec §8.7.2.1).
/// Branchless for the same reason as [`BlockInfo::bs_branchless`], but now with
/// every operand already in a register rather than behind a strided load.
/// Production derives intra strengths separately and calls [`bs_inter`]; this
/// full form is kept as the oracle the fast path is pinned against in tests.
#[cfg(test)]
#[inline]
fn bs_tile(p: &Blk, q: &Blk, mb_edge: bool) -> i32 {
    let intra = !(p.inter & q.inter);
    let nz = p.nz | q.nz;
    let moved = bs1_tile(p, q);
    let intra_bs = if mb_edge { 4 } else { 3 };
    let non_intra = if nz { 2 } else { moved as i32 };
    if intra {
        intra_bs
    } else {
        non_intra
    }
}

/// Boundary strength for an edge where BOTH sides are inter — the only case left
/// once the per-macroblock intra fills in [`derive_mb_bs`] have run. It can never
/// return 3 or 4, which is precisely why x264's `deblock_strength` kernel is
/// branch-light enough to vectorise: the intra strengths are not its job.
#[inline]
fn bs_inter(p: &Blk, q: &Blk) -> i32 {
    let nz = p.nz | q.nz;
    let moved = bs1_tile(p, q);
    if nz {
        2
    } else {
        moved as i32
    }
}

/// Derive one macroblock's 32 boundary strengths (2 directions × 4 edge groups ×
/// 4 segments), x264-style.
///
/// The structural point: `mb_type` is a per-MACROBLOCK syntax element, so a
/// macroblock is wholly intra or wholly inter and intra-ness is not a per-edge
/// property. That turns the two expensive cases into constant fills —
///   * intra macroblock → internal edges are the constant 3, its own macroblock
///     edges the constant 4;
///   * flat inter (skip) macroblock → every internal strength is 0;
///
/// — and leaves [`bs_inter`] for the rest. Pinned by `derive_matches_per_edge`.
#[inline]
fn derive_mb_bs(
    tile: &Tile,
    mb_x: usize,
    mb_y: usize,
    flat_inter: bool,
    uniform_motion: bool,
    mb_t8: bool,
    (bs_v, bs_h): (&mut [[i32; 4]; 4], &mut [[i32; 4]; 4]),
) {
    let cur_intra = !tile[1][1].inter;

    // ---- macroblock edges: 4 if EITHER side is intra, else an inter compare ----
    if mb_x > 0 {
        bs_v[0] = if cur_intra || !tile[1][0].inter {
            [4; 4]
        } else {
            core::array::from_fn(|seg| bs_inter(&tile[seg + 1][0], &tile[seg + 1][1]))
        };
    }
    if mb_y > 0 {
        bs_h[0] = if cur_intra || !tile[0][1].inter {
            [4; 4]
        } else {
            core::array::from_fn(|seg| bs_inter(&tile[0][seg + 1], &tile[1][seg + 1]))
        };
    }

    // ---- internal edges ----
    if flat_inter {
        return; // all zero by construction (no coefficients, one shared ref+mv)
    }
    for be in 1..4usize {
        // An 8×8-transform macroblock has no transform boundary at 4×4 edges 1/3.
        if mb_t8 && (be == 1 || be == 3) {
            continue;
        }
        if cur_intra {
            bs_v[be] = [3; 4];
            bs_h[be] = [3; 4];
        } else if uniform_motion {
            // Coefficients alone; 0 or 2, no motion compare.
            bs_v[be] = core::array::from_fn(|seg| {
                2 * (tile[seg + 1][be].nz | tile[seg + 1][be + 1].nz) as i32
            });
            bs_h[be] = core::array::from_fn(|seg| {
                2 * (tile[be][seg + 1].nz | tile[be + 1][seg + 1].nz) as i32
            });
        } else {
            // Both sides are inside this macroblock, so no neighbour read at all.
            bs_v[be] =
                core::array::from_fn(|seg| bs_inter(&tile[seg + 1][be], &tile[seg + 1][be + 1]));
            bs_h[be] =
                core::array::from_fn(|seg| bs_inter(&tile[be][seg + 1], &tile[be + 1][seg + 1]));
        }
    }
}

/// What the encoder already knows about a macroblock the moment it finishes
/// coding it. Passing it in turns most macroblocks into constant fills and
/// removes the neighbourhood gather that made deriving in the encode loop cost
/// MORE than deriving in the deblocking pass (the loop's working set is far more
/// contended, so a 24-block gather there evicts live encoder data).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MbKind {
    /// Intra: internal strengths are the constant 3, its own macroblock edges 4.
    /// Needs NO block reads at all.
    Intra,
    /// Skip — no coefficients, one shared reference and motion vector — so every
    /// internal strength is 0 and only the two macroblock edges are derived.
    Skip,
    /// Coded inter with a SINGLE partition (P_L0_16x16 — the fast preset's only
    /// inter mode). All 16 blocks share one reference and motion vector, so no
    /// internal edge can reach strength 1: internal strengths depend on
    /// coefficients alone, and the derivation reads 16 nnz bytes instead of
    /// gathering 24 blocks across four grids.
    InterUniform,
    /// Coded inter, multiple partitions: the full derivation.
    Inter,
}

/// Boundary strengths of an INTRA macroblock, which follow from availability
/// alone -- no grids are read at all.
///
/// Split out of `derive_mb_kind` because it is the ONLY kind the decoder's row
/// derivation asks for: Skip and InterUniform route to the packed
/// `derive_mb_records` arm, and the kind-loads arm that would ask for them is a
/// build-time `false`. With this call site taking the constant directly, nothing
/// in the decoder references `derive_mb_kind` and LTO drops the whole gathering
/// function out of the decode binary.
#[inline]
pub fn intra_mb_bs(mb_x: usize, mb_y: usize) -> MbBs {
    // Constant per (left available, top available): one 32-byte copy instead of a
    // zeroed default + two conditional fills + a loop (routing round).
    const fn intra_bs(left: bool, top: bool) -> MbBs {
        MbBs {
            v: [if left { [4; 4] } else { [0; 4] }, [3; 4], [3; 4], [3; 4]],
            h: [if top { [4; 4] } else { [0; 4] }, [3; 4], [3; 4], [3; 4]],
        }
    }
    const INTRA_BS: [[MbBs; 2]; 2] = [
        [intra_bs(false, false), intra_bs(false, true)],
        [intra_bs(true, false), intra_bs(true, true)],
    ];
    INTRA_BS[(mb_x > 0) as usize][(mb_y > 0) as usize]
}

/// Boundary strengths for a macroblock whose kind the caller already knows.
///
/// `Intra` reads nothing; `Skip` reads one of its own blocks plus the neighbour
/// column/row (9 instead of 24); only `Inter` pays the full gather.
/// `derive_mb_kind_into` writes the deblock loop's `i32` arrays directly.
///
/// # Errors
/// `FueraDeRango` si las rejillas de `info` o sus mapas de POC no cubren el macrobloque.
pub fn derive_mb_kind_into(
    info: &BlockInfo,
    mb_x: usize,
    mb_y: usize,
    kind: MbKind,
    bs_v: &mut [[i32; 4]; 4],
    bs_h: &mut [[i32; 4]; 4],
) -> Result<(), FueraDeRango> {
    // Direct i32 writes — no MbBs staging + 32 u8→i32 casts.
    *bs_v = [[0; 4]; 4];
    *bs_h = [[0; 4]; 4];
    let (bx0, by0) = (mb_x * 4, mb_y * 4);
    let w4 = info.w4;
    match kind {
        MbKind::Intra => {
            if mb_x > 0 {
                bs_v[0] = [4; 4];
            }
            if mb_y > 0 {
                bs_h[0] = [4; 4];
            }
            for e in 1..4 {
                bs_v[e] = [3; 4];
                bs_h[e] = [3; 4];
            }
        }
        MbKind::Skip => {
            let me = Blk::load(info, by0 * w4 + bx0)?;
            if mb_x > 0 {
                bs_v[0] = intentar_4(|seg| {
                    let p = Blk::load(info, (by0 + seg) * w4 + bx0 - 1)?;
                    Ok(if p.inter { bs_inter(&p, &me) } else { 4 })
                })?;
            }
            if mb_y > 0 {
                bs_h[0] = intentar_4(|seg| {
                    let p = Blk::load(info, (by0 - 1) * w4 + bx0 + seg)?;
                    Ok(if p.inter { bs_inter(&p, &me) } else { 4 })
                })?;
            }
        }
        MbKind::InterUniform => {
            // FOUR ROW SLICES. The internal edges read only this macroblock's
            // own sixteen nnz cells, but did it as FORTY-EIGHT separately
            // bounds-checked whole-grid loads (two per edge per segment, for
            // three edges in each orientation). The cells are four contiguous
            // runs of four, so four slices cover every read and `[e]`/`[e-1]`
            // are then provably in range.
            let nz: [&[u8]; 4] = core::array::from_fn(|r| &info.nnz[(by0 + r) * w4 + bx0..][..4]);
            for e in 1..4usize {
                bs_v[e] = core::array::from_fn(|seg| {
                    2 * ((nz[seg][e] != 0) | (nz[seg][e - 1] != 0)) as i32
                });
                bs_h[e] = core::array::from_fn(|seg| {
                    2 * ((nz[e][seg] != 0) | (nz[e - 1][seg] != 0)) as i32
                });
            }
            if mb_x > 0 {
                bs_v[0] = intentar_4(|seg| {
                    let qi = (by0 + seg) * w4 + bx0;
                    let p = Blk::load(info, qi - 1)?;
                    Ok(if p.inter {
                        bs_inter(&p, &Blk::load(info, qi)?)
                    } else {
                        4
                    })
                })?;
            }
            if mb_y > 0 {
                bs_h[0] = intentar_4(|seg| {
                    let qi = by0 * w4 + bx0 + seg;
                    let p = Blk::load(info, qi - w4)?;
                    Ok(if p.inter {
                        bs_inter(&p, &Blk::load(info, qi)?)
                    } else {
                        4
                    })
                })?;
            }
        }
        MbKind::Inter => {
            let m = derive_mb(info, mb_x, mb_y, false)?;
            for e in 0..4 {
                for sg in 0..4 {
                    bs_v[e][sg] = m.v[e][sg] as i32;
                    bs_h[e][sg] = m.h[e][sg] as i32;
                }
            }
        }
    }
    Ok(())
}

/// Intensidades de borde de un macrobloque de clase conocida (versión que devuelve el registro).
///
/// # Errors
/// `FueraDeRango` si las rejillas de `info` o sus mapas de POC no cubren el macrobloque.
pub fn derive_mb_kind(
    info: &BlockInfo,
    mb_x: usize,
    mb_y: usize,
    kind: MbKind,
) -> Result<MbBs, FueraDeRango> {
    let (bx0, by0) = (mb_x * 4, mb_y * 4);
    let w4 = info.w4;
    Ok(match kind {
        MbKind::Intra => intra_mb_bs(mb_x, mb_y),
        MbKind::Skip => {
            // Internal strengths stay 0: no coefficients and one shared (ref, mv)
            // means no internal edge can reach strength 1 or 2.
            let mut m = MbBs::default();
            let me = Blk::load(info, by0 * w4 + bx0)?; // all 16 blocks are identical
            if mb_x > 0 {
                m.v[0] = intentar_4(|seg| {
                    let p = Blk::load(info, (by0 + seg) * w4 + bx0 - 1)?;
                    Ok(if p.inter { bs_inter(&p, &me) as u8 } else { 4 })
                })?;
            }
            if mb_y > 0 {
                m.h[0] = intentar_4(|seg| {
                    let p = Blk::load(info, (by0 - 1) * w4 + bx0 + seg)?;
                    Ok(if p.inter { bs_inter(&p, &me) as u8 } else { 4 })
                })?;
            }
            m
        }
        MbKind::InterUniform => {
            let mut m = MbBs::default();
            // Internal edges: uniform motion means only coefficients can raise a
            // strength, and then only to 2.
            // FOUR ROW SLICES — the exact twin of `derive_mb_kind_into`'s
            // InterUniform arm (16 panics to 0, -22% instructions there). The
            // internal edges read only this macroblock's own sixteen nnz cells,
            // as four contiguous runs of four, but did it as forty-eight
            // separately checked whole-grid loads.
            let nz: [&[u8]; 4] = core::array::from_fn(|r| &info.nnz[(by0 + r) * w4 + bx0..][..4]);
            for e in 1..4usize {
                m.v[e] = core::array::from_fn(|seg| {
                    2 * ((nz[seg][e] != 0) | (nz[seg][e - 1] != 0)) as u8
                });
                m.h[e] = core::array::from_fn(|seg| {
                    2 * ((nz[e][seg] != 0) | (nz[e - 1][seg] != 0)) as u8
                });
            }
            // Macroblock edges still cross into the neighbour, and our own
            // coefficients vary per block, so both sides are read per segment.
            if mb_x > 0 {
                m.v[0] = intentar_4(|seg| {
                    let qi = (by0 + seg) * w4 + bx0;
                    let p = Blk::load(info, qi - 1)?;
                    Ok(if p.inter {
                        bs_inter(&p, &Blk::load(info, qi)?) as u8
                    } else {
                        4
                    })
                })?;
            }
            if mb_y > 0 {
                m.h[0] = intentar_4(|seg| {
                    let qi = by0 * w4 + bx0 + seg;
                    let p = Blk::load(info, qi - w4)?;
                    Ok(if p.inter {
                        bs_inter(&p, &Blk::load(info, qi)?) as u8
                    } else {
                        4
                    })
                })?;
            }
            m
        }
        MbKind::Inter => derive_mb(info, mb_x, mb_y, false)?,
    })
}

/// Derive one macroblock's boundary strengths from the block grids — the entry
/// point for computing them during ENCODE.
///
/// `info.ref_id` may carry the encoder's raw reference indices (negative for
/// intra) rather than the `NO_REF` sentinel: reference identity is only ever
/// compared between two INTER blocks, which always hold a valid index.
/// PACKED per-macroblock boundary-strength inputs — the x264-shaped layout.
///
/// Everything one macroblock's derivation needs for its own 16 blocks, contiguous,
/// in raster order (`k = row * 4 + col`). This is the layout `deblock_strength_avx2`
/// works from, and the reason x264 can vectorise a job we currently do 32 times
/// scalar and branchy.
///
/// Sized by the CEILING PROBE (`examples/bs_layout_ceiling.rs`), which measured, at
/// 720p on synthetic-but-representative content:
///
/// * cache lines touched per macroblock: **16-20 strided vs 2 packed**
/// * gather cost (cache-warm): **31.1 ns/MB strided vs 12.5 ns/MB packed** (2.49x)
///
/// Read that second row carefully, because it REFUTED the original justification for
/// this work. The whole derivation costs ~400 ns/MB in context, so eliminating the
/// gather buys only ~19 ns/MB — about 5% of the derivation, ~1% of decode. **The
/// derivation is COMPUTE-bound, not gather-bound**: ~370 ns/MB is the 32 per-edge
/// `bs_inter` evaluations themselves (32 edges x ~20-30 cycles ~= 210-320 ns @ 3 GHz,
/// which matches). So this layout does NOT pay for itself as a memory optimisation.
/// It exists to make the ARITHMETIC vectorisable — `nnz` collapses to shift-and-or on
/// a 16-bit mask, and the motion test becomes i16 subtract/abs/compare across 16
/// lanes. That distinction decides whether the SIMD stage is worth building, and it
/// is the good case: this codebase's flat SIMD results were all memory-bound kernels,
/// its wins (DCT/SATD) were compute-bound.
///
/// SINGLE-LIST ONLY. B macroblocks carrying a List-1 slot fall back to the blind
/// path; the two-list rule in `bs1_tile` is not reproduced here.
#[derive(Clone, Copy)]
#[repr(C, align(32))]
pub struct MbPack {
    /// Bit `k` set = block `k` has coefficients. The derivation only ever asks
    /// "is this block coded", never for the count.
    pub nnz_mask: u16,
    /// `mb_type` is a per-MACROBLOCK syntax element, so intra-ness is not per-block.
    pub inter: bool,
    _pad: u8,
    /// Quarter-pel motion, per block, SPLIT into x and y planes.
    ///
    /// Structure-of-arrays on purpose: interleaved `(x,y)` pairs would force the SIMD
    /// twin to OR adjacent lanes together (a pairwise reduction), whereas split planes
    /// let the x and y compares live in separate registers and OR whole-register.
    /// 16 x i16 is exactly one 256-bit load each.
    pub mvx: [i16; 16],
    pub mvy: [i16; 16],
    /// Bit k set = block k carries a List-1 slot. When this is 0 for a macroblock,
    /// every internal edge takes the single-list rule and the AVX2 kernel applies;
    /// otherwise the two-list set-matching rule runs scalar for that macroblock.
    pub l1_used: u16,
    pub mvx1: [i16; 16],
    pub mvy1: [i16; 16],
    pub ref1: [i32; 16],
    /// Reference PICTURE IDENTITY per block (`NO_REF` = intra/unused). Kept i32
    /// because the decoder supplies a POC, which must stay comparable across slices
    /// whose reference lists differ — a `ref_idx` would compare equal across slices
    /// that mean different pictures.
    pub ref_id: [i32; 16],
}

impl BlockInfo<'_> {
    /// Block `i`'s List-0 reference identity, applying the optional index map.
    /// Fallible grid accessors for the BLIND (non-tile) derivation arm. The
    /// decoder never enters it — the packed tile path owns every frame — so the
    /// cost of a conditional move here is irrelevant, while the panic paths were
    /// not. A missing block reads as intra with no motion, which is the maximum
    /// boundary strength and therefore the conservative answer.
    #[cfg(test)]
    #[inline]
    fn inter_at(&self, i: usize) -> bool {
        self.inter.get(i).copied().unwrap_or(false)
    }
    #[cfg(test)]
    #[inline]
    fn nnz_at(&self, i: usize) -> u8 {
        self.nnz.get(i).copied().unwrap_or(0)
    }
    #[cfg(test)]
    #[inline]
    fn ref_at(&self, i: usize) -> i32 {
        self.ref_id.get(i).copied().unwrap_or(NO_REF)
    }
    #[cfg(test)]
    #[inline]
    fn ref1_at(&self, i: usize) -> i32 {
        self.ref_id1.get(i).copied().unwrap_or(NO_REF)
    }
    #[cfg(test)]
    #[inline]
    fn mv_at(&self, i: usize) -> (i32, i32) {
        self.mv.get(i).copied().unwrap_or((0, 0))
    }
    #[cfg(test)]
    #[inline]
    fn mv1_at(&self, i: usize) -> (i32, i32) {
        self.mv1.get(i).copied().unwrap_or((0, 0))
    }

    /// [`Self::rid`] with a fallible grid read (see `Blk::load`).
    ///
    /// # Errors
    /// `FueraDeRango` si `ref_id` no cubre `i` o el mapa de POC no cubre
    /// el índice de referencia guardado ahí.
    #[inline]
    fn rid_at(&self, i: usize) -> Result<i32, FueraDeRango> {
        let r = *self.ref_id.get(i).ok_or(FueraDeRango("ref_id"))?;
        map_ref(!self.poc0.is_empty(), self.poc0, r)
    }

    /// [`Self::rid1`] with a fallible grid read.
    ///
    /// # Errors
    /// `FueraDeRango` si `ref_id1` no cubre `i` o el mapa de POC no cubre
    /// el índice de referencia guardado ahí.
    #[inline]
    fn rid1_at(&self, i: usize) -> Result<i32, FueraDeRango> {
        let r = *self.ref_id1.get(i).ok_or(FueraDeRango("ref_id1"))?;
        map_ref(!self.poc1.is_empty(), self.poc1, r)
    }
}

impl Default for MbPack {
    fn default() -> Self {
        Self {
            nnz_mask: 0,
            inter: false,
            _pad: 0,
            mvx: [0; 16],
            mvy: [0; 16],
            l1_used: 0,
            mvx1: [0; 16],
            mvy1: [0; 16],
            ref1: [NO_REF; 16],
            ref_id: [NO_REF; 16],
        }
    }
}

/// Build the packed records for a whole frame in ONE streaming pass.
///
/// Each frame-wide array is read once, sequentially and prefetch-friendly, instead of
/// being hit 3600 times at scattered per-macroblock offsets. Returns `None` when the
/// frame carries List-1 data (B slices), which this layout does not model.
///
/// # Errors
/// `FueraDeRango` si las rejillas de `info` o sus mapas de POC no cubren la imagen.
pub fn pack_frame(info: &BlockInfo, mb_w: usize, mb_h: usize) -> Result<Vec<MbPack>, FueraDeRango> {
    let mut out = Vec::new();
    pack_frame_into(info, mb_w, mb_h, &mut out)?;
    Ok(out)
}

/// [`pack_frame`] into a RECYCLED buffer (Brick: WHYS Part 15 item 1). A fresh
/// `Vec<MbPack>` was ~1.1 MB allocated + first-touch-faulted EVERY frame — the
/// same per-frame-materialization mechanism the banked GridPool brick removed.
/// Each record is built in a local and pushed, so a reused allocation is never
/// pre-filled: one write per byte, no defaults pass, no page faults.
///
/// # Errors
/// `FueraDeRango` si las rejillas de `info` o sus mapas de POC no cubren la imagen.
pub fn pack_frame_into(
    info: &BlockInfo,
    mb_w: usize,
    mb_h: usize,
    out: &mut Vec<MbPack>,
) -> Result<(), FueraDeRango> {
    let has1 = !info.ref_id1.is_empty();
    out.clear();
    out.reserve(mb_w * mb_h);
    for mb_y in 0..mb_h {
        for mb_x in 0..mb_w {
            out.push(pack_mb(info, has1, mb_x, mb_y)?);
        }
    }
    Ok(())
}

/// One macroblock's packed record — the unit of both `pack_frame_into` and the
/// rolling-window precompute pass.
/// Reference index -> picture identity, with the map presence hoisted OUT.
/// `BlockInfo::rid`/`rid1` re-ask `poc.is_empty()` on every call, i.e. 16 times
/// per macroblock for a slice-invariant answer; `mapped` is that answer,
/// resolved once by the caller. Byte-identical to `rid`/`rid1`.
///
/// # Errors
/// `FueraDeRango` si un índice no negativo no está en el mapa de POC. El llamador pasa el
/// mapa del corte dueño del bloque, así que en un flujo válido siempre está; un índice
/// negativo es «sin referencia» (`NO_REF`), no un error.
#[inline]
fn map_ref(mapped: bool, poc: &[i32], r: i32) -> Result<i32, FueraDeRango> {
    match usize::try_from(r) {
        Ok(i) if mapped => poc.get(i).copied().ok_or(FueraDeRango("mapa de POC")),
        Ok(_) => Ok(r),
        Err(_) if mapped => Ok(NO_REF),
        Err(_) => Ok(r),
    }
}

/// Empaqueta los 16 bloques del macrobloque `(mb_x, mb_y)` en un registro [`MbPack`].
///
/// # Errors
/// `FueraDeRango` si las rejillas o los mapas de POC no cubren el macrobloque.
#[inline]
pub fn pack_mb(
    info: &BlockInfo,
    has1: bool,
    mb_x: usize,
    mb_y: usize,
) -> Result<MbPack, FueraDeRango> {
    let mut rec = MbPack::default();
    let (bx0, by0) = (mb_x * 4, mb_y * 4);
    let w4 = info.w4;
    let base = by0 * w4 + bx0;
    rec.inter = *info.inter.get(base).ok_or(FueraDeRango("inter"))?;
    // Map presence is a property of the SLICE, not the block (see `map_ref`).
    let map0 = !info.poc0.is_empty();
    let map1 = !info.poc1.is_empty();

    for r in 0..4 {
        let row = base + r * w4;
        // ROW SLICES, NOT STRIDED INDEXES. One bounds check per grid per ROW
        // instead of one per BLOCK - the same shape the batch-10 motion-grid
        // gathers used. It took `pack_mb` from six `panic_bounds_check` edges
        // to one, which is also what lets LLVM dead-store-eliminate the parts
        // of `MbPack::default()` that the loop fully overwrites (`mvx`, `mvy`,
        // `ref_id`). The List-1 arrays are NOT dead when `has1` is false -
        // `mb_uniform` hands `mvx1`/`mvy1`/`ref1` to the AVX2 kernel
        // unconditionally - so their defaults must stand.
        let nnz = &info.nnz[row..row + 4];
        let mvr = &info.mv[row..row + 4];
        let rid = &info.ref_id[row..row + 4];
        // `map0` is a property of the SLICE, so it is invariant across all 16
        // blocks -- hand-unswitched here so the identity case carries no test at
        // all and the mapped case tests nothing per block either.
        for c in 0..4 {
            let k = r * 4 + c;
            // Branchless: the original tested `nnz != 0` and branched to set the
            // bit, a data-dependent branch on coefficient presence.
            rec.nnz_mask |= ((nnz[c] != 0) as u16) << k;
            rec.mvx[k] = mvr[c].0 as i16;
            rec.mvy[k] = mvr[c].1 as i16;
        }
        if map0 {
            for (dst, &r0) in rec.ref_id[r * 4..r * 4 + 4].iter_mut().zip(rid) {
                *dst = map_ref(true, info.poc0, r0)?;
            }
        } else {
            rec.ref_id[r * 4..r * 4 + 4].copy_from_slice(rid);
        }
        if has1 {
            let mv1r = &info.mv1[row..row + 4];
            let rid1 = &info.ref_id1[row..row + 4];
            for c in 0..4 {
                let k = r * 4 + c;
                let r1 = map_ref(map1, info.poc1, rid1[c])?;
                rec.ref1[k] = r1;
                rec.mvx1[k] = mv1r[c].0 as i16;
                rec.mvy1[k] = mv1r[c].1 as i16;
                rec.l1_used |= ((r1 != NO_REF) as u16) << k;
            }
        }
    }
    Ok(rec)
}

/// FUSED pack+derive over the whole frame, with a TWO-ROW ROLLING WINDOW of
/// packed records (WHYS Part 16 root-cause lever, bounded form).
///
/// The frame-buffer pipeline was: pack ALL 3600 records into a ~1.1 MB Vec,
/// then a second interleaved loop re-reads them (plus a per-MB gate ladder) to
/// derive strengths. Here each record is DERIVED the moment it is built —
/// while it and its left/top neighbours are L1-hot — and only `2 × mb_w`
/// records (~50 KB at 720p) ever exist. Output is one `MbBs` (32 B) per
/// macroblock, which `filter_frame` consumes over its existing PRECOMPUTED
/// path, collapsing the per-MB derivation machinery in the hot loop entirely.
///
/// Byte-identical by the existing oracle chain: `derive_mb_records` is
/// `derive_mb_packed`'s own core (pinned to the tile twin by
/// `packed_matches_tile`), and the precomputed consumer path is pinned by the
/// encoder's use of it. `flat_inter` loop-skips are subsumed by the stored
/// zeros + the consuming loops' all-zero early-outs (documented there).
///
/// # Errors
/// `FueraDeRango` si las rejillas de `info` o sus mapas de POC no cubren la imagen.
/// Un `t8x8` vacío significa «ningún macrobloque usa la transformada 8×8»; uno no vacío
/// y más corto que `mb_w × mb_h` es error.
pub fn precompute_bs_frame(
    info: &BlockInfo,
    mb_w: usize,
    mb_h: usize,
    out: &mut Vec<MbBs>,
) -> Result<(), FueraDeRango> {
    let has1 = !info.ref_id1.is_empty();
    out.clear();
    out.reserve(mb_w * mb_h);
    let mut prev_row: Vec<MbPack> = Vec::with_capacity(mb_w);
    let mut cur_row: Vec<MbPack> = Vec::with_capacity(mb_w);
    for mb_y in 0..mb_h {
        cur_row.clear();
        for mb_x in 0..mb_w {
            cur_row.push(pack_mb(info, has1, mb_x, mb_y)?);
            // `split_last` names the just-pushed record AND everything before it,
            // so `cur` and `left` both come from the same proven split.
            let Some((cur, before)) = cur_row.split_last() else {
                continue;
            };
            let left = if mb_x > 0 { before.last() } else { None };
            let top = if mb_y > 0 { prev_row.get(mb_x) } else { None };
            let mb_t8 = !info.t8x8.is_empty()
                && *info
                    .t8x8
                    .get(mb_y * mb_w + mb_x)
                    .ok_or(FueraDeRango("t8x8"))?;
            let (mut bv, mut bh) = ([[0i32; 4]; 4], [[0i32; 4]; 4]);
            derive_mb_records(cur, left, top, mb_t8, &mut bv, &mut bh);
            let mut m = MbBs {
                v: [[0; 4]; 4],
                h: [[0; 4]; 4],
            };
            for e in 0..4 {
                for sg in 0..4 {
                    m.v[e][sg] = bv[e][sg] as u8;
                    m.h[e][sg] = bh[e][sg] as u8;
                }
            }
            out.push(m);
        }
        core::mem::swap(&mut prev_row, &mut cur_row);
    }
    Ok(())
}

#[cfg(feature = "std")]
thread_local! {
    /// Recycled `pack_frame` buffer (~1.1 MB at 720p), `Cell` so `filter_frame`
    /// can `take`/`set` without holding a borrow across its whole MB loop.
    static PACK_SCRATCH: core::cell::Cell<Vec<MbPack>> = const { core::cell::Cell::new(Vec::new()) };
}

/// Without `std` there are no threads and no thread-locals: the pack buffer
/// is allocated per frame instead of recycled. Same `take`/`set` shape so
/// the call sites do not change.
#[cfg(not(feature = "std"))]
struct PackScratch;
#[cfg(not(feature = "std"))]
impl PackScratch {
    fn take(&self) -> Vec<MbPack> {
        Vec::new()
    }
    fn set(&self, _buf: Vec<MbPack>) {}
}
#[cfg(not(feature = "std"))]
static PACK_SCRATCH: PackScratch = PackScratch;

/// THE KERNEL INTERFACE — the whole motion half of the derivation as two bitmasks.
///
/// `bit k` of the returned pair answers "does block `k` differ in motion from its
/// LEFT neighbour block `k-1`" and "...from its ABOVE neighbour block `k-4`", using
/// the §8.7.2.1 rule that `bs_inter` applies per edge:
///
/// ```text
///   differs(a,b) = ref[a] != ref[b]  ||  (ref[a] != NO_REF && (|dmvx| >= 4 || |dmvy| >= 4))
/// ```
///
/// Bits at `k % 4 == 0` (left) and `k < 4` (up) are DON'T-CARE: those edges are
/// macroblock edges, derived separately against the neighbouring record. That is not
/// an accident of convenience — it is what makes the SIMD twin cheap, because the
/// lane positions a within-128-bit-lane shift corrupts are exactly those positions.
///
/// Collapsing the derivation to masks is the point of the packed layout: the 24
/// internal edges stop being 24 branchy `bs_inter` calls and become bit tests against
/// these masks OR'd with `nnz_mask`, which is the form `deblock_strength_avx2` works in.
/// Scalar twin of the uniform-motion test — the oracle, and the path on any CPU
/// without AVX2. Short-circuits, which is why the SIMD twin's win concentrates on
/// UNIFORM macroblocks (where this walks all 15 comparisons).
#[inline]
pub fn mb_uniform_scalar(p: &MbPack) -> bool {
    p.inter
        && (1..16).all(|k| {
            p.ref_id[k] == p.ref_id[0]
                && p.mvx[k] == p.mvx[0]
                && p.mvy[k] == p.mvy[0]
                && p.ref1[k] == p.ref1[0]
                && p.mvx1[k] == p.mvx1[0]
                && p.mvy1[k] == p.mvy1[0]
        })
}

/// Dispatcher: AVX2 when present, scalar otherwise. `p.inter` is a per-macroblock
/// flag and is checked here, so the kernel only ever answers the 16-lane question.
#[inline]
pub fn mb_uniform(p: &MbPack) -> bool {
    #[cfg(all(target_arch = "x86_64", feature = "asm"))]
    if p.inter {
        if let Some(u) =
            rusty_h264_accel::mb_uniform(&p.mvx, &p.mvy, &p.ref_id, &p.mvx1, &p.mvy1, &p.ref1)
        {
            return u;
        }
    }
    mb_uniform_scalar(p)
}

#[inline]
pub fn bs_motion_masks_scalar(p: &MbPack) -> (u16, u16) {
    let differs = |a: usize, b: usize| -> bool { pk_differs(p, a, p, b) };
    let (mut left, mut up) = (0u16, 0u16);
    for k in 0..16usize {
        if k % 4 != 0 && differs(k, k - 1) {
            left |= 1 << k;
        }
        if k >= 4 && differs(k, k - 4) {
            up |= 1 << k;
        }
    }
    // Don't-care bits are already zero here by construction; the masks are spelled out
    // so the SIMD twin (which must clear them explicitly) is comparable on the FULL u16.
    (left & 0xEEEE, up & 0xFFF0)
}

/// Dispatcher: the AVX2 twin when the CPU has it, the scalar oracle otherwise.
///
/// The scalar version stays the default on any non-AVX2 CPU and remains the
/// reference the SIMD twin is gated against — it is never deleted.
#[inline]
pub fn bs_motion_masks(p: &MbPack) -> (u16, u16) {
    #[cfg(all(target_arch = "x86_64", feature = "asm"))]
    if p.l1_used == 0 {
        // Single-list fast kernel: all of P and uni-L0 B macroblocks.
        if let Some(m) = rusty_h264_accel::bs_motion_masks(&p.mvx, &p.mvy, &p.ref_id, NO_REF) {
            return m;
        }
    } else {
        // TWO-LIST kernel (WHYS Part 16's named lever): the §8.7.2.1
        // set-matching rule vectorized, so B macroblocks with List-1 slots no
        // longer fall to the scalar per-edge walk. Gated bit-exact against the
        // scalar twin by `bs_motion_masks_two_list_matches_scalar` (accel).
        if let Some(m) = rusty_h264_accel::bs_motion_masks_two_list(
            &p.mvx, &p.mvy, &p.ref_id, &p.mvx1, &p.mvy1, &p.ref1, NO_REF,
        ) {
            return m;
        }
    }
    bs_motion_masks_scalar(p)
}

/// The §8.7.2.1 motion test over packed operands — the EXACT twin of `bs1_tile`,
/// including the two-list set-matching rule (which is order-independent: a block pair
/// whose two reference pictures match after a SWAP is not "different motion").
///
/// The single-list fast path fires whenever neither side carries a List-1 slot, which
/// is all of P and most uni-predicted B blocks; only the remainder pays the set match.
/// HOT-PREFIX SPLIT. `derive_mb_records` carries THIRTY-TWO call sites to this
/// predicate, and every one of them was an out-of-line `call`: the function is
/// marked `#[inline]`, but its two-list set-matching tail is far too large for
/// LLVM to inline at that many sites, so the SINGLE-LIST fast path -- a handful
/// of compares, and 100% of P content -- paid a call, a spill and a return.
///
/// The prefix is now `#[inline(always)]` and the two-list tail is outlined
/// behind `#[inline(never)]`, so the common case folds into the caller and the
/// rare case costs exactly the call it always cost. Same shape as the
/// `b_skip_hot` hot-prefix split. Pure refactor: identical predicate, identical
/// operand order, no arithmetic touched.
#[inline(always)]
fn pk_differs(p: &MbPack, pk: usize, q: &MbPack, qk: usize) -> bool {
    let p_has1 = p.ref1[pk] != NO_REF;
    let q_has1 = q.ref1[qk] != NO_REF;
    if !p_has1 && !q_has1 {
        // WIN: |d| >= 4 without the abs. For i16 d, `(d + 3) as u16 >= 7` is the same
        // predicate -- d in -3..=3 maps to 0..=6, d >= 4 maps to >= 7, and any d <= -4
        // wraps to a large unsigned. Two `abs` (sign-mask, xor, subtract each) become
        // an add and a compare, on the hottest per-edge predicate in the derivation.
        let far4 = |d: i16| (d.wrapping_add(3) as u16) >= 7;
        let far = far4(p.mvx[pk] - q.mvx[qk]) | far4(p.mvy[pk] - q.mvy[qk]);
        return (p.ref_id[pk] != q.ref_id[qk]) | ((p.ref_id[pk] != NO_REF) & far);
    }
    pk_differs_two_list(p, pk, q, qk)
}

/// The two-list tail of [`pk_differs`], outlined so it cannot bloat the 32
/// inlined fast paths. Reached only when a block on either side carries a
/// List-1 slot.
#[inline(never)]
fn pk_differs_two_list(p: &MbPack, pk: usize, q: &MbPack, qk: usize) -> bool {
    // Collect the USED reference slots, exactly as `bs1_tile::used` does.
    let used = |b: &MbPack, k: usize| {
        // Every field of `MbPack` is a fixed `[_; 16]` and `k` is a 4x4 block
        // index, so `& 15` is a no-op that folds six checks per side.
        let k = k & 15;
        let mut v = [(0i32, (0i32, 0i32)); 2];
        let mut n = 0usize;
        if b.ref_id[k] != NO_REF {
            v[n] = (b.ref_id[k], (b.mvx[k] as i32, b.mvy[k] as i32));
            n += 1;
        }
        if b.ref1[k] != NO_REF {
            v[n] = (b.ref1[k], (b.mvx1[k] as i32, b.mvy1[k] as i32));
            n += 1;
        }
        (v, n)
    };
    let (pv, pn) = used(p, pk);
    let (qv, qn) = used(q, qk);
    if pn != qn {
        return true;
    }
    // WIN: same |d| >= 4 identity, and `|` rather than `||` so the second component
    // carries no branch of its own.
    let far = |a: (i32, i32), b: (i32, i32)| {
        let f = |d: i32| (d.wrapping_add(3) as u32) >= 7;
        f(a.0 - b.0) | f(a.1 - b.1)
    };
    match pn {
        0 => false,
        1 => pv[0].0 != qv[0].0 || far(pv[0].1, qv[0].1),
        _ => {
            let direct = !far(pv[0].1, qv[0].1) && !far(pv[1].1, qv[1].1);
            let swap = !far(pv[0].1, qv[1].1) && !far(pv[1].1, qv[0].1);
            if pv[0].0 == pv[1].0 {
                qv[0].0 != pv[0].0 || qv[1].0 != pv[0].0 || !(direct || swap)
            } else if pv[0].0 == qv[0].0 && pv[1].0 == qv[1].0 {
                !direct
            } else if pv[0].0 == qv[1].0 && pv[1].0 == qv[0].0 {
                !swap
            } else {
                true
            }
        }
    }
}

#[inline]
fn pk_nz(p: &MbPack, k: usize) -> bool {
    (p.nnz_mask >> k) & 1 != 0
}

/// `bs_inter` over packed operands — both sides inter, so 3 and 4 are unreachable.
/// Mirrors `bs1_tile`'s single-list fast path exactly.
#[inline]
fn pk_bs_inter(p: &MbPack, pk: usize, q: &MbPack, qk: usize) -> i32 {
    if pk_nz(p, pk) | pk_nz(q, qk) {
        return 2;
    }
    // (removed: two `ref_id` loads that were bound and then discarded through
    // `let _ = (pr, qr)`. `pk_differs` reads the same fields itself, so these were
    // pure dead loads on the hottest per-edge predicate in the deblock stage.)
    pk_differs(p, pk, q, qk) as i32
}

/// Derive one macroblock's strengths from PACKED records — the byte-identical twin of
/// `derive_mb_bs`, pinned against it by `packed_matches_tile`.
pub fn derive_mb_packed(
    packs: &[MbPack],
    mb_w: usize,
    mb_x: usize,
    mb_y: usize,
    mb_t8: bool,
    bs_v: &mut [[i32; 4]; 4],
    bs_h: &mut [[i32; 4]; 4],
) -> bool {
    // Row slices: `[mb_x]` and `[mb_x - 1]` are then provably inside a slice of
    // length `mb_w`, instead of three unprovable whole-frame indexings.
    let row = &packs[mb_y * mb_w..][..mb_w];
    let Some(cur) = row.get(mb_x) else {
        return false;
    };
    let left = if mb_x > 0 { row.get(mb_x - 1) } else { None };
    let top = if mb_y > 0 {
        packs[(mb_y - 1) * mb_w..][..mb_w].get(mb_x)
    } else {
        None
    };
    derive_mb_records(cur, left, top, mb_t8, bs_v, bs_h)
}

/// The record-based core of [`derive_mb_packed`]: derive one macroblock's
/// strengths from ITS OWN record plus the left/top neighbours' — the shape the
/// rolling-window precompute pass ([`precompute_bs_frame`]) needs, where only
/// two rows of records exist at a time.
pub fn derive_mb_records(
    cur: &MbPack,
    left: Option<&MbPack>,
    top: Option<&MbPack>,
    mb_t8: bool,
    bs_v: &mut [[i32; 4]; 4],
    bs_h: &mut [[i32; 4]; 4],
) -> bool {
    let cur_intra = !cur.inter;

    // Both predicates from the packed record: uniform motion needs no neighbour, and
    // "no coefficients" is `nnz_mask == 0` — a single register test replacing a
    // 16-block walk.
    //
    // REFUTED (WHYS Part 16, z=-2.11 REGRESSION, reverted): deriving uniformity
    // from `bs_motion_masks` to save "the second kernel call". The two calls are
    // ASYMMETRIC: this one is a cheap AVX2 compare-to-block-0 that handles
    // two-list B data in vector code, while the masks kernel falls to the SCALAR
    // set-matching walk whenever `l1_used != 0` — most B inter MBs. The
    // population reaching this function on B frames is uniform-heavy, so the
    // "fusion" replaced their one cheap vector call with an expensive scalar
    // walk. Do not retry without making the masks kernel two-list-capable first.
    let uniform = mb_uniform(cur);
    let flat_inter = uniform && cur.nnz_mask == 0;

    if let Some(l) = left {
        bs_v[0] = if cur_intra || !l.inter {
            [4; 4]
        } else {
            core::array::from_fn(|seg| pk_bs_inter(l, seg * 4 + 3, cur, seg * 4))
        };
    }
    if let Some(t) = top {
        bs_h[0] = if cur_intra || !t.inter {
            [4; 4]
        } else {
            core::array::from_fn(|seg| pk_bs_inter(t, 12 + seg, cur, seg))
        };
    }

    if flat_inter {
        return true; // internal strengths are 0 by construction
    }
    // Only the general path consumes these; intra fills constants and uniform motion
    // reads coefficients alone.
    let masks = if cur_intra || uniform {
        (0, 0)
    } else {
        bs_motion_masks(cur)
    };
    for be in 1..4usize {
        if mb_t8 && (be == 1 || be == 3) {
            continue;
        }
        if cur_intra {
            bs_v[be] = [3; 4];
            bs_h[be] = [3; 4];
        } else if uniform {
            // Coefficients alone; the whole edge group is a shift-and-or on the mask.
            bs_v[be] = core::array::from_fn(|seg| {
                2 * (pk_nz(cur, seg * 4 + be) | pk_nz(cur, seg * 4 + be - 1)) as i32
            });
            bs_h[be] = core::array::from_fn(|seg| {
                2 * (pk_nz(cur, be * 4 + seg) | pk_nz(cur, (be - 1) * 4 + seg)) as i32
            });
        } else {
            // The general path, now pure bit tests: coefficients from `nnz_mask`,
            // motion from the precomputed masks. No per-edge branching at all.
            let (left, up) = masks;
            bs_v[be] = core::array::from_fn(|seg| {
                let k = seg * 4 + be;
                if pk_nz(cur, k) | pk_nz(cur, k - 1) {
                    2
                } else {
                    ((left >> k) & 1) as i32
                }
            });
            bs_h[be] = core::array::from_fn(|seg| {
                let k = be * 4 + seg;
                if pk_nz(cur, k) | pk_nz(cur, k - 4) {
                    2
                } else {
                    ((up >> k) & 1) as i32
                }
            });
        }
    }
    false
}

/// ONE walk of the macroblock's own 16 blocks yielding BOTH predicates the
/// derivation needs: `(uniform_motion, flat_inter)`.
///
/// They are the same walk, by construction:
/// ```text
///   uniform_motion = b0.inter AND every block { inter AND same_motion(b0) }
///   flat_inter     = uniform_motion AND no block carries coefficients
/// ```
/// They used to be two independent 16-block scans — and the `uniform_motion` one
/// ran BEFORE `derive_mb_bs`'s `if flat_inter { return }` early-out, so every Skip
/// macroblock paid a full 16-block × 6-compare motion scan whose result was then
/// discarded. The MB-kind census measures Skip at 36.4% (CAVLC) / 65.0% (main) /
/// 57.8% (high) of macroblocks, so that dead scan ran on most of the corpus.
///
/// Byte-identical to the two-scan form: when the walk breaks early both
/// predicates are false, exactly as both original `.all()` chains would be.
#[inline]
fn scan_uniform_flat(tile: &Tile) -> (bool, bool) {
    let b0 = &tile[1][1];
    if !b0.inter {
        return (false, false); // intra MB — neither predicate applies
    }
    let mut any_nz = false;
    for row in &tile[1..5] {
        for b in &row[1..5] {
            if !b.inter || !b.same_motion(b0) {
                return (false, false);
            }
            any_nz |= b.nz;
        }
    }
    (true, !any_nz)
}

/// La rama anterior a la fusión, conservada tal cual como oráculo de `scan_uniform_flat`.
#[cfg(test)]
fn scan_two_pass(tile: &Tile) -> (bool, bool) {
    let b0 = &tile[1][1];
    let flat = b0.inter
        && (1..5).all(|r| {
            (1..5).all(|c| {
                let b = &tile[r][c];
                b.inter && !b.nz && b.same_motion(b0)
            })
        });
    let uniform =
        b0.inter && (1..5).all(|r| (1..5).all(|c| tile[r][c].inter && tile[r][c].same_motion(b0)));
    (uniform, flat)
}

/// Intensidades de borde de un macrobloque sin clasificar (recogida completa).
///
/// # Errors
/// `FueraDeRango` si las rejillas de `info` o sus mapas de POC no cubren el macrobloque.
pub fn derive_mb(
    info: &BlockInfo,
    mb_x: usize,
    mb_y: usize,
    mb_t8: bool,
) -> Result<MbBs, FueraDeRango> {
    let tile = gather_tile(info, mb_x, mb_y)?;
    let (uniform_motion, flat_inter) = scan_uniform_flat(&tile);
    let (mut bs_v, mut bs_h) = ([[0i32; 4]; 4], [[0i32; 4]; 4]);
    derive_mb_bs(
        &tile,
        mb_x,
        mb_y,
        flat_inter,
        uniform_motion,
        mb_t8,
        (&mut bs_v, &mut bs_h),
    );
    let pack = |a: [[i32; 4]; 4]| a.map(|e| e.map(|x| x as u8));
    Ok(MbBs {
        v: pack(bs_v),
        h: pack(bs_h),
    })
}

/// The 5×5 neighbourhood an MB's edges can reach: row/col 0 are the top and left
/// neighbour blocks, rows/cols 1..=4 the MB's own 4×4 grid. Entries outside the
/// picture stay `Default` and are never read (the frame-edge groups are skipped).
type Tile = [[Blk; 5]; 5];

/// Gather the tile for macroblock (`mb_x`, `mb_y`).
fn gather_tile(info: &BlockInfo, mb_x: usize, mb_y: usize) -> Result<Tile, FueraDeRango> {
    let mut t: Tile = Default::default();
    let (bx0, by0) = (mb_x * 4, mb_y * 4);
    // REFUTED AND REVERTED: window-slicing all six grids so the twenty-four
    // `Blk::load`s share one bound per grid removed 7 of 33 panics and cost
    // +57.6% instructions (908 -> 1431) - the closure, the conditional List-1
    // slices and the inlined ref mapping are far more code than the checks they
    // fold. The per-load checks here are cheap and well predicted.
    // The MB's own blocks: four contiguous runs of four.
    for r in 0..4 {
        let row = (by0 + r) * info.w4 + bx0;
        for c in 0..4 {
            t[r + 1][c + 1] = Blk::load(info, row + c)?;
        }
    }
    if mb_x > 0 {
        for r in 0..4 {
            t[r + 1][0] = Blk::load(info, (by0 + r) * info.w4 + bx0 - 1)?;
        }
    }
    if mb_y > 0 {
        let row = (by0 - 1) * info.w4 + bx0;
        for c in 0..4 {
            t[0][c + 1] = Blk::load(info, row + c)?;
        }
    }
    // `mb_type` is a per-macroblock syntax element, so every 4×4 block of a
    // macroblock shares its intra/inter status. `derive_mb_bs` depends on this to
    // replace per-edge intra tests with per-macroblock constant fills.
    debug_assert!(
        (1..5).all(|r| (1..5).all(|c| t[r][c].inter == t[1][1].inter)),
        "macroblock ({mb_x},{mb_y}) mixes intra and inter 4x4 blocks"
    );
    Ok(t)
}

/// DEFAULT OFF. Deriving boundary strengths in the encode loop makes the
/// deblocking stage 1.4-1.7x faster but does NOT reduce total encode time: the
/// block grids were never cold (~90 KB at CIF, L2-resident), so the derivation
/// costs the same in a streaming pass, and the encode loop's contended working
/// set makes it cost MORE there — measured, the loop grew about twice the
/// derivation's own cost. Kept behind this switch with its tests, because the
/// machinery is what a future commit-time derivation (values still in registers,
/// no grid re-read) would build on.
static BS_PRECOMP: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);

/// Whether callers may supply precomputed boundary strengths (the encoder path).
pub fn precomputed_bs_enabled() -> bool {
    BS_PRECOMP.load(core::sync::atomic::Ordering::Relaxed) != 0
}

/// Toggle the precomputed-strength path so a benchmark can ALTERNATE the two
/// designs inside ONE process.
#[doc(hidden)]
pub fn set_precomputed_bs(on: bool) {
    BS_PRECOMP.store(on as u8, core::sync::atomic::Ordering::Relaxed);
}

/// Derivación por borde directamente sobre las tablas de la imagen: el ORÁCULO contra el
/// que se prueba la tesela (`tile_matches_frame_indexing`, `bs_arms_agree`). En producción
/// solo se deriva por la tesela o por las intensidades precalculadas.
#[cfg(test)]
impl BlockInfo<'_> {
    #[inline]
    fn at(&self, bx: usize, by: usize) -> usize {
        by * self.w4 + bx
    }

    /// Boundary strength between left/above block `p` and current block `q`
    /// (spec §8.7.2.1). `mb_edge` is true on macroblock boundaries.
    ///
    /// Written branchlessly on purpose. Real content mixes intra/inter, coded and
    /// uncoded blocks, and per-block motion, so the natural short-circuit form
    /// (`if intra … else if nnz … else if motion …`) mispredicts on nearly every
    /// 4×4 edge: the anatomy bench measures the identical code at ~240 ns/MB on
    /// uniform data and ~515 ns/MB once the block state varies, which is the
    /// whole of our gap to x264 here (x264 derives bS with a branchless SIMD
    /// kernel). Evaluating all three candidates costs a few extra loads and beats
    /// paying a ~15-cycle mispredict per edge.
    #[inline]
    fn bs(&self, p: usize, q: usize, mb_edge: bool) -> i32 {
        self.bs_branchless(p, q, mb_edge)
    }

    /// The original short-circuit form, kept as the A/B arm and the fallback.
    /// Misma salida que [`Self::bs_branchless`] por construcción; hoy solo es el
    /// oráculo de las pruebas.
    fn bs_branchy(&self, p: usize, q: usize, mb_edge: bool) -> i32 {
        if !self.inter_at(p) || !self.inter_at(q) {
            if mb_edge {
                4
            } else {
                3
            }
        } else if self.nnz_at(p) > 0 || self.nnz_at(q) > 0 {
            2
        } else if self.inter_bs1(p, q) {
            1
        } else {
            0
        }
    }

    #[inline]
    fn bs_branchless(&self, p: usize, q: usize, mb_edge: bool) -> i32 {
        let intra = !(self.inter_at(p) & self.inter_at(q));
        let nz = (self.nnz_at(p) | self.nnz_at(q)) != 0;
        let moved = self.inter_bs1(p, q);
        let intra_bs = if mb_edge { 4 } else { 3 };
        // Priority intra > coefficients > motion, as selects rather than branches.
        let motion_bs = moved as i32; // 1 or 0
        let non_intra = if nz { 2 } else { motion_bs };
        if intra {
            intra_bs
        } else {
            non_intra
        }
    }

    /// Whether two residual-free inter blocks get boundary strength 1: they use
    /// different reference pictures, a different number of motion vectors, or a
    /// motion vector differs by ≥ 1 full sample (matched by reference picture, so
    /// the same picture in different lists is recognised). Spec §8.7.2.1.
    fn inter_bs1(&self, p: usize, q: usize) -> bool {
        // Single-list fast path (P and I slices — `ref_id1` empty). This is the
        // overwhelmingly common case, and it collapses to two comparisons: the
        // general path below builds a two-slot [(ref, mv); 2] array per side and
        // matches on the slot count, which the anatomy bench showed dominates
        // deblocking. Exactly equivalent here: with one list, `pn`/`qn` are just
        // "is the slot used", so a differing ref_id covers both the
        // different-count and different-picture cases, and two unused slots give
        // pn == qn == 0 => false.
        if self.ref_id1.is_empty() {
            // Branchless (see `bs`): `|`/`&` rather than `||`/`&&` so there is no
            // data-dependent branch here either. The single `is_empty` test above
            // is uniform across a whole frame and predicts perfectly.
            let (rp, rq) = (self.ref_at(p), self.ref_at(q));
            let (a, b) = (self.mv_at(p), self.mv_at(q));
            let f4 = |d: i32| (d.wrapping_add(3) as u32) >= 7;
            let far = f4(a.0 - b.0) | f4(a.1 - b.1);
            // Differing refs ⇒ bS 1 (this also covers "one slot used, one not",
            // the general path's differing-count case). Both unused ⇒ 0.
            return (rp != rq) | ((rp != NO_REF) & far);
        }
        // (reference id, motion vector) for each used prediction slot.
        let used = |i: usize| {
            let mut v = [(0i32, (0i32, 0i32)); 2];
            let mut n = 0;
            if self.ref_at(i) != NO_REF {
                v[n] = (self.ref_at(i), self.mv_at(i));
                n += 1;
            }
            // `ref_id1` may be empty (P frames have no List-1 — the caller skips
            // building it, since every entry would be NO_REF anyway).
            if !self.ref_id1.is_empty() && self.ref1_at(i) != NO_REF {
                v[n] = (self.ref1_at(i), self.mv1_at(i));
                n += 1;
            }
            (v, n)
        };
        let (pv, pn) = used(p);
        let (qv, qn) = used(q);
        if pn != qn {
            return true; // different number of motion vectors
        }
        // WIN: same |d| >= 4 identity, and `|` rather than `||` so the second component
        // carries no branch of its own.
        let far = |a: (i32, i32), b: (i32, i32)| {
            let f = |d: i32| (d.wrapping_add(3) as u32) >= 7;
            f(a.0 - b.0) | f(a.1 - b.1)
        };
        match pn {
            0 => false,
            1 => pv[0].0 != qv[0].0 || far(pv[0].1, qv[0].1),
            _ => {
                // Two references each: the picture *sets* must match, and the
                // motion vectors for corresponding pictures must be close. If both
                // slots are the same picture, either pairing is acceptable.
                let direct = !far(pv[0].1, qv[0].1) && !far(pv[1].1, qv[1].1);
                let swap = !far(pv[0].1, qv[1].1) && !far(pv[1].1, qv[0].1);
                if pv[0].0 == pv[1].0 {
                    qv[0].0 != pv[0].0 || qv[1].0 != pv[0].0 || !(direct || swap)
                } else if pv[0].0 == qv[0].0 && pv[1].0 == qv[1].0 {
                    !direct
                } else if pv[0].0 == qv[1].0 && pv[1].0 == qv[0].0 {
                    !swap
                } else {
                    true // different picture sets
                }
            }
        }
    }
}

/// Applies the deblocking filter in place to a fully-reconstructed frame. `qp`
/// is the (constant) luma QP, `qpc` the chroma QP, and `info` supplies the
/// per-block state used to derive boundary strengths (for an all-intra frame
/// this reduces to the fixed 4/3 strengths).
/// Edge thresholds `(α, β, tc0[bS-1])` for a given averaged QP and the slice's
/// filter offsets (spec §8.7.2.2): α/tc0 indexed by `indexA`, β by `indexB`.
#[inline]
fn thresholds(qpav: i32, offset_a: i32, offset_b: i32) -> (i32, i32, [i32; 3]) {
    // MEASUREMENT ONLY (env-gated): counts REAL invocations, so the per-MB
    // caching of the internal-edge set is directly visible.
    let ia = (qpav + offset_a).clamp(0, 51) as usize;
    let ib = (qpav + offset_b).clamp(0, 51) as usize;
    (ALPHA[ia], BETA[ib], TC0[ia])
}

/// Aplica el filtro de desbloqueo a toda la imagen (§8.7).
///
/// # Errors
/// `FueraDeRango` si las rejillas de `info` o sus mapas de POC no cubren la imagen.
#[allow(clippy::too_many_arguments)]
pub fn filter_frame(
    y: &mut [u8],
    u: &mut [u8],
    v: &mut [u8],
    mb_w: usize,
    mb_h: usize,
    mb_qp: &[u8],
    chroma_qp_offset: i32,
    offset_a: i32,
    offset_b: i32,
    info: &BlockInfo,
) -> Result<(), FueraDeRango> {
    // El codificador usa un único par por imagen: se reparte aquí a todos sus macrobloques.
    let par = DesplazamientosFiltro::nuevo(offset_a, offset_b)
        .ok_or(FueraDeRango("desplazamientos del filtro"))?;
    let desplazamientos = vec![par; mb_w * mb_h];
    filter_frame_rows(
        y,
        u,
        v,
        mb_w,
        mb_h,
        0..mb_h,
        mb_qp,
        chroma_qp_offset,
        &BlockInfo {
            desplazamientos: &desplazamientos,
            ..*info
        },
    )
}

/// The NON-PRECOMPUTED derivation, outlined.
///
/// The decoder always supplies precomputed strengths (`info.bs` non-empty), so
/// every line of this - the kind fast path, the packed records, the 5x5 `Tile`
/// gather and the blind 16-block `flat_inter` scan - is dead on the shipped
/// decode path. It was nonetheless inlined into `filter_frame_rows`, where the
/// line-attributed assembly put THIRTEEN of the function's twenty-six
/// `panic_bounds_check` edges inside it, along with the 800-byte `Tile` that
/// drives the stack frame. Outlining leaves the hot loop with the precomputed
/// arm only; the encoder and the tile/packed A/B arms reach it through one
/// call, exactly as before.
///
/// Returns `flat_inter`; writes the strengths through `bs_v` / `bs_h`.
#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn derive_mb_general(
    info: &BlockInfo,
    mb_x: usize,
    mb_y: usize,
    mb_w: usize,
    mb_t8: bool,
    packs: Option<&Vec<MbPack>>,
    bs_v: &mut [[i32; 4]; 4],
    bs_h: &mut [[i32; 4]; 4],
) -> Result<bool, FueraDeRango> {
    let fast_kind = match info
        .kind
        .get(mb_y * mb_w + mb_x)
        .copied()
        .and_then(MbKind::from_u8)
    {
        Some(k) if k != MbKind::Inter => Some(k),
        _ => None,
    };

    let blind_tile = fast_kind.is_none() && packs.is_none();
    // Declared before the predicate chain: the packed derivation produces
    // `flat_inter` and the strengths in ONE traversal, so it cannot be split
    // across the two chains the way the tile path is.
    let packed_mb = packs.filter(|_| fast_kind.is_none());
    // MATERIALISE THE TILE ONLY ON THE BLIND PATH. Writing
    // `else { Default::default() }` here cost an 800-byte zero-init of the
    // 5x5 `Tile` on every CLASSIFIED macroblock — i.e. this brick removed a
    // gather and added ~4 GB of memset, and measured 1.2-17.0% SLOWER in
    // 4/4 pairs. `Option` keeps the fast path free of it entirely.
    let tile = if blind_tile {
        Some(gather_tile(info, mb_x, mb_y)?)
    } else {
        None
    };

    // ONE walk yields both predicates — see `scan_uniform_flat`. The
    // non-tile arm has no `uniform_motion` consumer (it derives per-edge
    // off the frame arrays), so it keeps returning `flat_inter` alone.
    let mut uniform_motion = false;
    let flat_inter = if let Some(k) = fast_kind {
        match k {
            // No coefficients and one shared (ref, mv): every internal
            // strength is 0 by construction.
            MbKind::Skip => true,
            // CAUGHT BY THE ORACLE: a single-partition inter macroblock
            // with NO coefficients is flat too — uniform motion means the
            // blind predicate reduces to exactly "no block has nnz".
            // Hardcoding `false` here was pixel-identical (the strengths
            // are all 0 either way) but made the consuming loops walk the
            // internal edge groups instead of skipping them, throwing away
            // part of the win. 16 contiguous nnz bytes — the same data
            // `derive_mb_kind` already reads for this class.
            MbKind::InterUniform => {
                // Row slices: one range check per ROW instead of one per block.
                let (bx0, by0) = (mb_x * 4, mb_y * 4);
                (0..4).all(|r| {
                    info.nnz[(by0 + r) * info.w4 + bx0..][..4]
                        .iter()
                        .all(|&n| n == 0)
                })
            }
            _ => false,
        }
    } else if let Some(pk) = packed_mb {
        derive_mb_packed(pk, mb_w, mb_x, mb_y, mb_t8, bs_v, bs_h)
    } else {
        let (u, f) = match tile.as_ref() {
            Some(t) => scan_uniform_flat(t),
            None => (false, false),
        };
        uniform_motion = u;
        f
    };
    // ---- boundary strengths for the whole macroblock, derived ONCE ----
    // The chroma edge groups are CO-LOCATED with luma edges 0 and 2 and
    // derive identical strengths (pinned by `chroma_bs_matches_luma`), so
    // deriving them in the chroma loops recomputed 16 of the 48 per-MB
    // strengths. Guards mirror the consuming loops exactly, so nothing is
    // derived that was not derived before; edges left at zero are exactly
    // the edges those loops skip.
    if packed_mb.is_some() {
        // Already written by `derive_mb_packed` above: unlike the tile path,
        // the packed derivation produces `flat_inter` AND the strengths in one
        // traversal, so it cannot be split across the two chains.
    } else if let Some(k) = fast_kind {
        // Write i32 strengths straight into the consuming arrays. Going via
        // `derive_mb_kind`'s packed `MbBs` cost 32 u8->i32 stores per
        // classified macroblock that the blind path never pays — pure
        // addition on the path that is supposed to be cheaper.
        derive_mb_kind_into(info, mb_x, mb_y, k, bs_v, bs_h)?;
    } else if blind_tile {
        if let Some(t) = tile.as_ref() {
            derive_mb_bs(
                t,
                mb_x,
                mb_y,
                flat_inter,
                uniform_motion,
                mb_t8,
                (bs_v, bs_h),
            );
        }
    }
    Ok(flat_inter)
}

/// [`filter_frame`] restricted to macroblock rows `rows` — the row-interleave
/// campaign's unit (docs/row-interleave-plan.md R3): the decoder filters row
/// `r` the moment it finishes decoding, spec raster order preserved exactly.
/// Filter rows with the boundary strengths ALREADY PRECOMPUTED in `info.bs`.
///
/// Identical to [`filter_frame_rows`] except that "precomputed" is a CONSTANT
/// rather than `!info.bs.is_empty()`. That single fact makes the entire blind
/// derivation subtree -- `derive_mb_general`, `gather_tile`, `derive_mb_bs`,
/// `derive_mb_kind_into` -- statically unreachable, so LTO drops ~4.3k
/// instructions of it out of a decoder that never executed any of it. The
/// decoder always precomputes (the row-deblock path fills `bs_frame`), it just
/// had no way to say so.
///
/// # Errors
/// `FueraDeRango` si las rejillas de `info` o sus mapas de POC no cubren la imagen.
#[allow(clippy::too_many_arguments)]
pub fn filter_frame_rows_pre(
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
    filter_frame_rows_impl::<true>(y, u, v, mb_w, mb_h, rows, mb_qp, chroma_qp_offset, info)
}

/// Aplica el filtro de desbloqueo a una franja de filas de macrobloques.
///
/// # Errors
/// `FueraDeRango` si las rejillas de `info` o sus mapas de POC no cubren la imagen.
#[allow(clippy::too_many_arguments)]
pub fn filter_frame_rows(
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
    filter_frame_rows_impl::<false>(y, u, v, mb_w, mb_h, rows, mb_qp, chroma_qp_offset, info)
}

mod filtrado;
use filtrado::filter_frame_rows_impl;

#[cfg(test)]
#[path = "deblock/tests/chroma_bs_tests.rs"]
mod chroma_bs_tests;
#[cfg(test)]
#[path = "deblock/tests/derive_tests.rs"]
mod derive_tests;
/// Desplazamientos del filtro por macrobloque (COR-4, D-2026-10-12-2).
#[cfg(test)]
#[path = "deblock/tests/desplazamientos_por_macrobloque.rs"]
mod desplazamientos_por_macrobloque;
#[cfg(test)]
mod tests;
#[cfg(test)]
#[path = "deblock/tests/tile_tests.rs"]
mod tile_tests;
