//! Inter prediction primitives: motion compensation and motion-vector
//! prediction. Shared by the encoder (reconstruction) and decoder so the two
//! agree bit-for-bit. Motion vectors are in quarter-pel units; phase 4b uses
//! the integer part only (full-pel), with the 6-tap/bilinear sub-pel filters
//! arriving in 4c.

#[allow(unused_imports)]
use alloc::vec;
use alloc::vec::Vec;

use crate::rango::FueraDeRango;

/// Median of three values (`a + b + c − min − max`).
#[inline]
pub fn median3(a: i32, b: i32, c: i32) -> i32 {
    a + b + c - a.min(b).min(c) - a.max(b).max(c)
}

/// A motion-vector predictor neighbor: whether the neighbor block is available
/// (inside the picture/decoded), its motion vector, and its reference index
/// (`-1` for intra/unavailable neighbors, which contribute a zero MV with a
/// non-matching reference).
#[derive(Clone, Copy)]
pub struct MvNeighbor {
    pub available: bool,
    pub mv: (i32, i32),
    pub ref_idx: i32,
}

impl MvNeighbor {
    pub const NONE: MvNeighbor = MvNeighbor {
        available: false,
        mv: (0, 0),
        ref_idx: -1,
    };
}

/// Median motion-vector prediction for a partition (spec §8.4.1.3.1), against
/// the current partition's reference `cur_ref`. `a`/`b`/`c` are the left, above,
/// and above-right neighbors. If exactly one neighbor shares `cur_ref`, its MV is
/// the predictor; otherwise the component-wise median is used.
pub fn predict_mv(a: MvNeighbor, b: MvNeighbor, c: MvNeighbor, cur_ref: i32) -> (i32, i32) {
    // Per-neighbor (mv, refIdx): a usable inter neighbor keeps its mv+ref, else
    // a zero MV with ref −1 (never matches a valid `cur_ref` ≥ 0).
    let resolve = |n: MvNeighbor| -> ((i32, i32), i32) {
        if n.available && n.ref_idx >= 0 {
            (n.mv, n.ref_idx)
        } else {
            ((0, 0), -1)
        }
    };
    let (mva, ra) = resolve(a);
    let (mut mvb, mut rb) = resolve(b);
    let (mut mvc, mut rc) = resolve(c);

    // If both B and C are unavailable but A is available, B and C take A.
    if !b.available && !c.available && a.available {
        mvb = mva;
        rb = ra;
        mvc = mva;
        rc = ra;
    }

    let matches = (ra == cur_ref) as i32 + (rb == cur_ref) as i32 + (rc == cur_ref) as i32;
    if matches == 1 {
        if ra == cur_ref {
            mva
        } else if rb == cur_ref {
            mvb
        } else {
            mvc
        }
    } else {
        (median3(mva.0, mvb.0, mvc.0), median3(mva.1, mvb.1, mvc.1))
    }
}

/// Directional MV prediction for a sub-partition (spec §8.4.1.3.2) against the
/// partition's reference `cur_ref`. `mode` is the inter `mb_type` (0 = 16×16,
/// 1 = 16×8, 2 = 8×16). 16×8/8×16 use a specific neighbor directly when it shares
/// `cur_ref`; otherwise (and always for 16×16) the median.
pub fn predict_partition_mv(
    mode: u8,
    part: usize,
    a: MvNeighbor,
    b: MvNeighbor,
    c: MvNeighbor,
    cur_ref: i32,
) -> (i32, i32) {
    // 16x16 (mode 0) has no directional shortcut and is the dominant shape —
    // leave before building the closure or entering the table.
    if mode == 0 {
        return predict_mv(a, b, c, cur_ref);
    }
    let m = |n: MvNeighbor| n.available && n.ref_idx == cur_ref;
    match (mode, part) {
        (1, 0) if m(b) => b.mv, // 16×8 top → above
        (1, 1) if m(a) => a.mv, // 16×8 bottom → left
        (2, 0) if m(a) => a.mv, // 8×16 left → left
        (2, 1) if m(c) => c.mv, // 8×16 right → above-right
        _ => predict_mv(a, b, c, cur_ref),
    }
}

/// Inter `mb_type` → luma partition regions `(x, y, w, h)` in samples.
pub fn inter_partitions(mode: u8) -> &'static [(usize, usize, usize, usize)] {
    match mode {
        1 => &[(0, 0, 16, 8), (0, 8, 16, 8)], // P_16x8
        2 => &[(0, 0, 8, 16), (8, 0, 8, 16)], // P_8x16
        // P_8x8: four 8×8 sub-macroblocks in raster (decode_p8x8) order, each its
        // own MV. (Sub-8×8 shapes 8×4/4×8/4×4 within an 8×8 are a further split.)
        3 => &[(0, 0, 8, 8), (8, 0, 8, 8), (0, 8, 8, 8), (8, 8, 8, 8)],
        _ => &[(0, 0, 16, 16)], // P_L0_16x16
    }
}

/// Última coordenada válida de un eje de `n` muestras, para recortar contra el borde.
///
/// # Errors
/// `FueraDeRango` si el eje está vacío (`n == 0`): `clamp(0, -1)` entraría en pánico.
#[inline]
fn ultima(n: usize) -> Result<isize, FueraDeRango> {
    n.checked_sub(1)
        .and_then(|u| isize::try_from(u).ok())
        .ok_or(FueraDeRango("plano de referencia vacío"))
}

/// Comprueba que `reference` es un plano no vacío que cubre `cw × ch` muestras.
///
/// # Errors
/// `FueraDeRango` si `cw` o `ch` es 0, si `cw × ch` desborda o si el plano es más corto.
#[inline]
fn plano_cubre(reference: &[u8], cw: usize, ch: usize) -> Result<(), FueraDeRango> {
    match cw.checked_mul(ch) {
        Some(n) if n > 0 && reference.len() >= n => Ok(()),
        _ => Err(FueraDeRango("plano de referencia")),
    }
}

/// Muestra de referencia con recorte al borde de la imagen.
///
/// Un flujo malformado puede llegar aquí con `(cw, ch)` que no corresponden al plano
/// recibido. Antes se leía entonces una muestra negra (`unwrap_or(0)`); ahora es un error
/// (control de límites en compensación de movimiento).
///
/// # Errors
/// `FueraDeRango` si el plano está vacío o no cubre `cw × ch`.
#[inline]
fn at(reference: &[u8], cw: usize, ch: usize, x: isize, y: isize) -> Result<i32, FueraDeRango> {
    plano_cubre(reference, cw, ch)?;
    let xx = x.clamp(0, ultima(cw)?) as usize;
    let yy = y.clamp(0, ultima(ch)?) as usize;
    reference
        .get(yy * cw + xx)
        .map(|&m| i32::from(m))
        .ok_or(FueraDeRango("plano de referencia"))
}

#[inline]
fn clip_u8(v: i32) -> i32 {
    v.clamp(0, 255)
}

// ---- Block-level luma MC, mirroring openh264 `mc.cpp` (`McHorVerNN_c`) ----
//
// openh264 computes each half-pel plane once per block (not per pixel) and
// averages clipped planes for the quarter positions. Our `luma_sample` recomputed
// the 6-tap for every pixel; these kernels compute each plane once. Bit-identical:
// the 6-tap is separable with exact (un-rounded) integer intermediates, so
// horizontal-then-vertical equals vertical-then-horizontal, and the clamped tile
// reproduces `at()` exactly.

/// Max luma interpolation tile: a 16×16 block plus the 6-tap halo (2 left/up,
/// 3 right/down) → 21×21.
const LUMA_TILE: usize = 21;

/// Per-thread scratch for the sub-pel MC path.
///
/// These three buffers used to be created — and therefore **zero-initialised** —
/// inside every `mc_luma` call, and the tile was additionally returned *by value*.
/// That is ~1.4 KB of `memset` plus a 441-byte move per call, paid identically
/// whether the block is 16×16 (256 useful samples) or 4×4 (16). Profiling put
/// `inter-mc` at 166 s of a 266 s quality-preset encode over 991 M calls, and a
/// block-size sweep showed a 4×4 sub-pel call costing *more* than a 16×16 one
/// (248 vs 172 ns) — i.e. the cost was the per-call buffers, not the filter.
///
/// Hoisting them to thread-local scratch makes that setup once per thread instead
/// of once per candidate. Byte-identical: both `luma_tile_into` paths write the
/// whole `(bw+5)×(bh+5)` region before it is read, and `a`/`b` are written over
/// `bw*bh` samples by `luma_h`/`luma_v`/`luma_centre` before `pixel_avg`/`avg_full`
/// read the same range — so the zero-fill was dead in every case.
pub struct McScratch {
    tile: [u8; LUMA_TILE * LUMA_TILE],
    a: [u8; 256],
    b: [u8; 256],
}

/// Runs `f` with the thread's MC scratch borrowed ONCE. Callers that issue many
/// MC calls in a row (the bi-pred pair, the P-path rect ladder) hoist the
/// TLS lookup + RefCell borrow out of the per-call path by taking this at
/// region/MB scope and calling `mc_luma_padded_pre` inside — that fixed
/// per-call cost is paid twice per bi-pred region, ~1M sub-pel calls per clip.
#[inline]
pub fn with_mc_scratch<R>(f: impl FnOnce(&mut McScratch) -> R) -> R {
    MC_SCRATCH.with(|s| f(&mut s.borrow_mut()))
}

#[cfg(feature = "std")]
thread_local! {
    static MC_SCRATCH: core::cell::RefCell<McScratch> = const {
        core::cell::RefCell::new(McScratch {
            tile: [0; LUMA_TILE * LUMA_TILE],
            a: [0; 256],
            b: [0; 256],
        })
    };
}

/// Without `std` the MC scratch (under 1 KB) lives on the stack of each
/// call instead of in a thread-local; the `with` shape is kept so the call
/// sites do not change.
#[cfg(not(feature = "std"))]
struct McScratchCell;
#[cfg(not(feature = "std"))]
impl McScratchCell {
    fn with<R>(&self, f: impl FnOnce(&core::cell::RefCell<McScratch>) -> R) -> R {
        let cell = core::cell::RefCell::new(McScratch {
            tile: [0; LUMA_TILE * LUMA_TILE],
            a: [0; 256],
            b: [0; 256],
        });
        f(&cell)
    }
}
#[cfg(not(feature = "std"))]
static MC_SCRATCH: McScratchCell = McScratchCell;

/// Extracts the `(bw+5)×(bh+5)` reference neighbourhood around the full-pel origin
/// `(ix0,iy0)` into `t`, clamping at the frame border — the edge-extended input
/// openh264's kernels assume. The block's top-left sample lands at tile `(2,2)`.
/// Returns the tile stride. Every sample in `[0, (bh+5)*ts)` is written.
///
/// # Errors
/// `FueraDeRango` si el plano está vacío o no cubre `cw × ch` (antes, muestra negra).
fn luma_tile_into(
    t: &mut [u8],
    reference: &[u8],
    (cw, ch): (usize, usize),
    (ix0, iy0): (isize, isize),
    (bw, bh): (usize, usize),
) -> Result<usize, FueraDeRango> {
    let ts = bw + 5;
    // Interior fast path: the whole `(bw+5)×(bh+5)` halo is inside the frame, so no
    // edge clamp is needed. Extract by contiguous row copies (a vectorized memcpy)
    // — the unconditional per-pixel `clamp` on the slow path defeats
    // autovectorization even when (as here) it would always be a no-op.
    if ix0 - 2 >= 0
        && iy0 - 2 >= 0
        && ix0 - 2 + ts as isize <= cw as isize
        && iy0 - 2 + (bh + 5) as isize <= ch as isize
        && reference.len() >= cw * ch
    {
        let (rx0, ry0) = ((ix0 - 2) as usize, (iy0 - 2) as usize);
        for ty in 0..bh + 5 {
            let src = (ry0 + ty) * cw + rx0;
            t[ty * ts..ty * ts + ts].copy_from_slice(&reference[src..src + ts]);
        }
        return Ok(ts);
    }
    let (ux, uy) = (ultima(cw)?, ultima(ch)?);
    for ty in 0..bh + 5 {
        let ry = (iy0 - 2 + ty as isize).clamp(0, uy) as usize * cw;
        let trow = &mut t[ty * ts..][..ts];
        for (tx, o) in trow.iter_mut().enumerate() {
            let rx = (ix0 - 2 + tx as isize).clamp(0, ux) as usize;
            *o = *reference
                .get(ry + rx)
                .ok_or(FueraDeRango("plano de referencia"))?;
        }
    }
    Ok(ts)
}

/// Horizontal half-pel plane (`McHorVer20`): `clip((6tapₕ + 16) >> 5)`, block
/// shifted by `(dr, dc)` tile rows/cols.
fn luma_h(t: &[u8], ts: usize, bw: usize, bh: usize, dr: usize, dc: usize, dst: &mut [u8]) {
    #[cfg(accel)]
    if bw == 16 || bw == 8 {
        rusty_h264_accel::mc_hor20(t, (2 + dr) * ts + 2 + dc, ts, dst, bw, bh);
        return;
    }
    #[cfg(accel)]
    if bw == 4 && bh <= 16 {
        let off = (2 + dr) * ts + 2 + dc;
        if off >= 2 && t.len() >= off + (bh - 1) * ts + 8 + 3 {
            w4_via_w8(dst, bh, |s| {
                rusty_h264_accel::mc_hor20(t, off, ts, s, 8, bh)
            });
            return;
        }
    }
    // TWO SLICES PER ROW, and the geometry is why this works here and NOT in
    // `luma_centre`. These six taps are HORIZONTAL — `t[p-2]` through `t[p+3]`
    // lie in one contiguous run — so a single `bw + 5` slice covers every read
    // the whole row makes. `luma_centre`'s taps stride by `ts`, six separate
    // runs, which is what made the identical-looking edit cost +9.3% there.
    for r in 0..bh {
        let base = (2 + r + dr) * ts + 2 + dc;
        let src = &t[base - 2..][..bw + 5];
        let drow = &mut dst[r * bw..][..bw];
        // `windows(6)` yields a slice of EXACTLY six, so all six taps fold —
        // `src[c] ..= src[c + 5]` still needed `c < bw` related to `bw + 5`.
        for (o, w) in drow.iter_mut().zip(src.windows(6)) {
            let f = w[0] as i32 - 5 * w[1] as i32 + 20 * w[2] as i32 + 20 * w[3] as i32
                - 5 * w[4] as i32
                + w[5] as i32;
            *o = clip_u8((f + 16) >> 5) as u8;
        }
    }
}

/// 4-WIDE LUMA BLOCKS THROUGH THE 8-WIDE KERNELS (SIMD census 2026-09-05,
/// findings #3-5). The accel luma kernels serve `w` in {8, 16}; sub-8x8
/// partitions (4x4 / 4x8) fell to the scalar loops below -- `luma_v` paying
/// SIX `.get().unwrap_or(0)` checked loads per sample. Compose instead: run the
/// 8-wide kernel into a scratch (the four discarded columns are real samples of
/// the padded plane, kept in bounds by the caller's length check, which
/// mirrors the kernel's own assert) and copy the left four of each row.
/// Byte-identical: every kept sample is the same taps at the same position.
/// The quarter-pel dispatchers' compose fallbacks (`luma_h` + `avg_full`,
/// `luma_h` + `luma_v` + `pixel_avg`, ...) then run on kernels at w == 4 too.
#[cfg(accel)]
#[inline]
fn w4_via_w8(dst: &mut [u8], bh: usize, run: impl FnOnce(&mut [u8])) {
    let mut s = [0u8; 8 * 16];
    run(&mut s[..8 * bh]);
    for (d, srow) in dst.chunks_exact_mut(4).zip(s.chunks_exact(8)).take(bh) {
        d.copy_from_slice(&srow[..4]);
    }
}

/// Las seis filas de `bw` muestras de la ventana vertical de 6 tomas cuya primera fila
/// empieza en `inicio` (filas `inicio + k * ts`, k = 0..6).
///
/// # Errors
/// `FueraDeRango` si la tesela no contiene las seis filas.
#[inline]
fn seis_filas(t: &[u8], inicio: usize, ts: usize, bw: usize) -> Result<[&[u8]; 6], FueraDeRango> {
    let fila = |k: usize| {
        t.get(inicio + k * ts..)
            .and_then(|s| s.get(..bw))
            .ok_or(FueraDeRango("tile"))
    };
    Ok([fila(0)?, fila(1)?, fila(2)?, fila(3)?, fila(4)?, fila(5)?])
}

/// Suma vertical de 6 tomas `a − 5b + 20c + 20d − 5e + f` de seis filas, columna a columna.
#[inline]
fn tomas_verticales(f: [&[u8]; 6]) -> impl Iterator<Item = i32> + '_ {
    f[0].iter()
        .zip(f[1])
        .zip(f[2])
        .zip(f[3])
        .zip(f[4])
        .zip(f[5])
        .map(|(((((&a, &b), &c), &d), &e), &g)| {
            i32::from(a) - 5 * i32::from(b) + 20 * i32::from(c) + 20 * i32::from(d)
                - 5 * i32::from(e)
                + i32::from(g)
        })
}

/// Vertical half-pel plane (`McHorVer02`): `clip((6tapᵥ + 16) >> 5)`.
///
/// # Errors
/// `FueraDeRango` si la tesela no contiene las tomas del bloque.
fn luma_v(
    t: &[u8],
    ts: usize,
    bw: usize,
    bh: usize,
    dr: usize,
    dc: usize,
    dst: &mut [u8],
) -> Result<(), FueraDeRango> {
    #[cfg(accel)]
    if bw == 16 || bw == 8 {
        rusty_h264_accel::mc_ver02(t, (2 + dr) * ts + 2 + dc, ts, dst, bw, bh);
        return Ok(());
    }
    #[cfg(accel)]
    if bw == 4 && bh <= 16 {
        let off = (2 + dr) * ts + 2 + dc;
        if off >= 2 * ts && t.len() >= off + (bh + 2) * ts + 8 {
            w4_via_w8(dst, bh, |s| {
                rusty_h264_accel::mc_ver02(t, off, ts, s, 8, bh)
            });
            return Ok(());
        }
    }
    // COR-3: las seis filas de tomas se toman una vez por fila de salida y se recorren con
    // `zip`, sin índice por muestra ni muestra inventada (el antiguo
    // `t.get(q).unwrap_or(0)` leía una toma ausente como negra).
    for r in 0..bh {
        let inicio = (r + dr) * ts + 2 + dc;
        let filas = seis_filas(t, inicio, ts, bw)?;
        let drow = &mut dst[r * bw..][..bw];
        for (o, f) in drow.iter_mut().zip(tomas_verticales(filas)) {
            *o = clip_u8((f + 16) >> 5) as u8;
        }
    }
    Ok(())
}

/// Centre half-pel plane (`McHorVer22`): vertical 6-tap to 16-bit intermediates,
/// then horizontal 6-tap — `clip((·+ 512) >> 10)`.
///
/// # Errors
/// `FueraDeRango` si la tesela no contiene las tomas del bloque.
fn luma_centre(
    t: &[u8],
    ts: usize,
    bw: usize,
    bh: usize,
    dst: &mut [u8],
) -> Result<(), FueraDeRango> {
    #[cfg(accel)]
    if bw == 16 || bw == 8 {
        rusty_h264_accel::mc_centre(t, ts, dst, bw, bh);
        return Ok(());
    }
    #[cfg(accel)]
    if bw == 4 && bh <= 16 && t.len() >= (bh + 4) * ts + 8 + 5 {
        w4_via_w8(dst, bh, |s| rusty_h264_accel::mc_centre(t, ts, s, 8, bh));
        return Ok(());
    }
    // REFUTED, do not retry: row-slicing the six vertical taps (and `dst`)
    // removed ONE panic path of eight and cost +9.3% instructions (323 -> 353).
    // `bw`/`bh` are runtime values, so each slice length is a runtime bound the
    // compiler cannot fold — six of them per row outweigh the per-sample checks
    // they replace. Same shape as `weight_partition`'s inner rows (+28%).
    // COR-3 (Crusty_H264): las tomas `t.get(q).unwrap_or(0)` por muestra leían una muestra
    // ausente como negra. Ahora las seis filas se toman una vez por fila; el coste se
    // mide en tiempo de decodificación de flujos completos (velocidad.py), no contando
    // instrucciones.
    let mut itmp = [0i32; LUMA_TILE];
    for r in 0..bh {
        let filas = seis_filas(t, r * ts, ts, bw + 5)?;
        for (slot, f) in itmp[..bw + 5].iter_mut().zip(tomas_verticales(filas)) {
            *slot = f;
        }
        // The earlier refutation bundled SEVEN changes; this is the one of them
        // that stands alone. The vertical-tap slices were the cost — the
        // destination row is contiguous and independent of them.
        let drow = &mut dst[r * bw..][..bw];
        for (o, w) in drow.iter_mut().zip(itmp[..bw + 5].windows(6)) {
            let f = w[0] - 5 * w[1] + 20 * w[2] + 20 * w[3] - 5 * w[4] + w[5];
            *o = clip_u8((f + 512) >> 10) as u8;
        }
    }
    Ok(())
}

/// Bi-average two STRIDED luma windows into a contiguous `w * h` block.
///
/// The B full-pel skip path had this as a hand-rolled scalar byte loop -- 256
/// `(p + q + 1) >> 1` per macroblock -- three modules from the `pixel_avg`
/// kernel that computes exactly that with `pavgb`. The kernel already takes
/// both source strides, which is what the two reference windows need; only a
/// public door was missing.
pub fn avg_row_into(a: &[u8], b: &[u8], w: usize, out: &mut [u8]) {
    #[cfg(accel)]
    if out.len() >= w && a.len() >= w && b.len() >= w {
        // The kernel is defined at widths 16/8/4, so walk the row in those
        // pieces. Every caller here is a whole number of macroblocks wide, so
        // the 16-wide loop does all the work and the 8/4 tails never run on
        // luma; chroma lands on the 8-wide step.
        let mut c = 0;
        while c + 16 <= w {
            rusty_h264_accel::pixel_avg(&mut out[c..c + 16], &a[c..], 16, &b[c..], 16, 16, 1);
            c += 16;
        }
        while c + 8 <= w {
            rusty_h264_accel::pixel_avg(&mut out[c..c + 8], &a[c..], 8, &b[c..], 8, 8, 1);
            c += 8;
        }
        while c + 4 <= w {
            rusty_h264_accel::pixel_avg(&mut out[c..c + 4], &a[c..], 4, &b[c..], 4, 4, 1);
            c += 4;
        }
        if c == w {
            return;
        }
    }
    // Scalar oracle, and the path on a non-accel build or a width the kernel
    // cannot cover exactly.
    for (d, (&x, &y)) in out[..w].iter_mut().zip(a[..w].iter().zip(&b[..w])) {
        *d = ((x as u16 + y as u16 + 1) >> 1) as u8;
    }
}

/// `PixelAvg_c`: `(a + b + 1) >> 1` of two clipped planes.
fn pixel_avg(a: &[u8], b: &[u8], bw: usize, bh: usize, dst: &mut [u8]) {
    let n = bw * bh;
    // Same leak `avg_full` fixed, for the TWO-FILTER quarter positions
    // ((1,1)/(3,1)/(1,3)/(3,3) and the centre-adjacent four): the average on
    // top of the asm half-pel kernels was a scalar runtime-width loop. The MC
    // census prices quarter-pel at ~68% of decoder MC cycles, and the eight
    // positions served here all end in this loop. `pavgb` computes
    // `(a + b + 1) >> 1` exactly — byte-identical. The scalar loop stays as
    // the non-accel path and the oracle.
    //
    // The TRUE block geometry is passed, never reconstructed from `n`: the
    // openh264 row loops are unrolled by more than one row, so a synthetic
    // (16, n/16) shape with h < 4 over-runs the block — that was a real
    // SEGFAULT on every sub-8x8 stream (first cut of this dispatch, caught by
    // the corpus gate). The kernels are exercised at every real (bw, bh ≥ 4)
    // shape by `avg_full` already.
    #[cfg(accel)]
    if (bw == 16 || bw == 8 || bw == 4) && a.len() >= n && b.len() >= n && dst.len() >= n {
        rusty_h264_accel::pixel_avg(&mut dst[..n], &a[..n], bw, &b[..n], bw, bw, bh);
        return;
    }
    // ZIP, not three indexed reads per pixel. `dst[i]`, `a[i]` and `b[i]` are
    // three INDEPENDENT slices, so every pixel carried three bounds checks; the
    // zip proves all three once for the whole block. (Contrast `luma_centre`,
    // where the same idea lost: there it would be six slices PER ROW against a
    // runtime width — here it is three for the entire block.)
    for (d, (&x, &y)) in dst[..n].iter_mut().zip(a[..n].iter().zip(&b[..n])) {
        *d = ((x as i32 + y as i32 + 1) >> 1) as u8;
    }
}

/// `PixelAvg_c` of a half-pel plane with a full-pel block shifted by `(dr, dc)`.
fn avg_full(
    t: &[u8],
    ts: usize,
    (bw, bh): (usize, usize),
    (dr, dc): (usize, usize),
    half: &[u8],
    dst: &mut [u8],
) {
    // The QUARTER-PEL step. `luma_h`/`luma_v`/`luma_centre` have been on asm for a
    // long time; this average layered on top of them stayed a scalar per-pixel loop
    // with a runtime width. On real x264 streams ~85% of decoder MC cycles are
    // quarter-pel, so this loop was handing the kernel's win back on the large
    // majority of MC. `pavgb` computes `(a + b + 1) >> 1` exactly — byte-identical.
    #[cfg(accel)]
    if bw == 16 || bw == 8 || bw == 4 {
        let base = (2 + dr) * ts + 2 + dc;
        rusty_h264_accel::pixel_avg(dst, &t[base..], ts, half, bw, bw, bh);
        return;
    }
    // Row-at-a-time zip: the destination walks in `bw` chunks, and the two
    // sources become one slice each per row instead of one check per sample.
    for (r, drow) in dst[..bw * bh].chunks_mut(bw).enumerate() {
        let base = (2 + r + dr) * ts + 2 + dc;
        let trow = &t[base..][..bw];
        let hrow = &half[r * bw..][..bw];
        for ((d, &x), &y) in drow.iter_mut().zip(trow).zip(hrow) {
            *d = ((x as i32 + y as i32 + 1) >> 1) as u8;
        }
    }
}

/// One-filter qpel (`(1,0)/(3,0)/(0,1)/(0,3)`): fused half+avg when accel + w∈{8,16}.
/// `horiz`: horizontal 6-tap; else vertical. `fshift`: full-pel offset (0 or 1) in the
/// filter axis (`fdc` for horiz, `fdr` for vert) — matches `avg_full`'s (0,1)/(1,0).
fn qpel_one_filter(
    t: &[u8],
    ts: usize,
    bw: usize,
    bh: usize,
    horiz: bool,
    fshift: usize,
    out: &mut [u8],
) -> Result<(), FueraDeRango> {
    #[cfg(accel)]
    if bw == 16 || bw == 8 {
        let off = 2 * ts + 2;
        if horiz {
            rusty_h264_accel::mc_hor_qpel(t, off, ts, out, bw, bh, fshift);
        } else {
            rusty_h264_accel::mc_ver_qpel(t, off, ts, out, bw, bh, fshift);
        }
        return Ok(());
    }
    // Compose oracle / narrow widths / non-accel: half into a stack scratch, then avg.
    let mut half = [0u8; 256];
    let n = bw * bh;
    debug_assert!(n <= 256);
    if horiz {
        luma_h(t, ts, bw, bh, 0, 0, &mut half[..n]);
        avg_full(t, ts, (bw, bh), (0, fshift), &half[..n], out);
    } else {
        luma_v(t, ts, bw, bh, 0, 0, &mut half[..n])?;
        avg_full(t, ts, (bw, bh), (fshift, 0), &half[..n], out);
    }
    Ok(())
}

/// Two-filter HV qpel: horizontal half into `a`, then vertical half+avg into `out`
/// (kills the second scratch store).
fn qpel_hv(
    t: &[u8],
    ts: usize,
    (bw, bh): (usize, usize),
    (hdr, hdc, vdr, vdc): (usize, usize, usize, usize),
    a: &mut [u8],
    out: &mut [u8],
) -> Result<(), FueraDeRango> {
    #[cfg(accel)]
    if bw == 16 || bw == 8 {
        // FULL fusion: both halves + avg in one kernel loop — no `a` staging,
        // no second kernel call (the previous two-step form, a ver02+avg kernel
        // reading back a staged half plane, was ripped once this landed).
        rusty_h264_accel::mc_hv_qpel(t, ts, out, bw, bh, hdr, hdc, vdr, vdc);
        return Ok(());
    }
    luma_h(t, ts, bw, bh, hdr, hdc, a);
    let n = bw * bh;
    let mut b = [0u8; 256];
    debug_assert!(n <= 256);
    luma_v(t, ts, bw, bh, vdr, vdc, &mut b[..n])?;
    pixel_avg(a, &b[..n], bw, bh, out);
    Ok(())
}

/// Centre-adjacent qpel ((2,1)/(2,3)/(1,2)/(3,2)): fused centre + half + avg.
/// The half-pel operand is DERIVED from the centre's own full-precision pass-1
/// buffer (its rounded form), so the 3-kernel 2-staging compose collapses to
/// one pass-1 + one fused pass-2/avg.
#[allow(clippy::too_many_arguments)]
fn qpel_centre_adj(
    t: &[u8],
    ts: usize,
    bw: usize,
    bh: usize,
    horiz: bool,
    fshift: usize,
    a: &mut [u8],
    b: &mut [u8],
    out: &mut [u8],
) -> Result<(), FueraDeRango> {
    #[cfg(accel)]
    if bw == 16 || bw == 8 {
        if horiz {
            rusty_h264_accel::mc_centre_hq(t, ts, out, bw, bh, fshift);
        } else {
            rusty_h264_accel::mc_centre_vq(t, ts, out, bw, bh, fshift);
        }
        return Ok(());
    }
    // Compose oracle / narrow widths / non-accel.
    if horiz {
        luma_h(t, ts, bw, bh, fshift, 0, a);
    } else {
        luma_v(t, ts, bw, bh, 0, fshift, a)?;
    }
    luma_centre(t, ts, bw, bh, b)?;
    pixel_avg(a, b, bw, bh, out);
    Ok(())
}

/// The `McLuma_c` `[mvx&3][mvy&3]` dispatch over the clamped tile (sub-pel only;
/// `(0,0)` is handled by the full-pel copy path in [`mc_luma`]).
#[allow(clippy::too_many_arguments)]
fn mc_luma_subpel(
    t: &[u8],
    ts: usize,
    bw: usize,
    bh: usize,
    fx: i32,
    fy: i32,
    a: &mut [u8],
    b: &mut [u8],
    out: &mut [u8],
) -> Result<(), FueraDeRango> {
    // `a`/`b` are caller-owned scratch (see `McScratch`): every arm below fully
    // writes the `n` samples it later reads, so their prior contents are dead.
    // One-filter qpel no longer needs them when fused (see `qpel_one_filter`).
    match (fx, fy) {
        (2, 0) => luma_h(t, ts, bw, bh, 0, 0, out),
        (0, 2) => luma_v(t, ts, bw, bh, 0, 0, out)?,
        (2, 2) => luma_centre(t, ts, bw, bh, out)?,
        // One-filter qpel: fuse half-pel + pavgb (openh264 McHorVer10/30/01/03).
        (1, 0) => qpel_one_filter(t, ts, bw, bh, true, 0, out)?,
        (3, 0) => qpel_one_filter(t, ts, bw, bh, true, 1, out)?,
        (0, 1) => qpel_one_filter(t, ts, bw, bh, false, 0, out)?,
        (0, 3) => qpel_one_filter(t, ts, bw, bh, false, 1, out)?,
        (1, 1) => qpel_hv(t, ts, (bw, bh), (0, 0, 0, 0), a, out)?,
        (3, 1) => qpel_hv(t, ts, (bw, bh), (0, 0, 0, 1), a, out)?,
        (1, 3) => qpel_hv(t, ts, (bw, bh), (1, 0, 0, 0), a, out)?,
        (3, 3) => qpel_hv(t, ts, (bw, bh), (1, 0, 0, 1), a, out)?,
        (2, 1) => qpel_centre_adj(t, ts, bw, bh, true, 0, a, b, out)?,
        (2, 3) => qpel_centre_adj(t, ts, bw, bh, true, 1, a, b, out)?,
        (1, 2) => qpel_centre_adj(t, ts, bw, bh, false, 0, a, b, out)?,
        (3, 2) => qpel_centre_adj(t, ts, bw, bh, false, 1, a, b, out)?,
        _ => {}
    }
    Ok(())
}

/// The three half-pel luma planes of one reference picture, at full frame size.
///
/// x264 filters these ONCE per reference frame and then every sub-pel motion-search
/// candidate is a strided copy from one plane (half positions) or a 2-tap average of
/// two (quarter positions). We previously re-ran the 6-tap per candidate: measured at
/// 166 s of a 266 s quality-preset encode over 991 M calls, against x264's 188 ms of
/// `hpel-filter` for the entire corpus.
///
/// Naming follows the spec's sample grid at full-pel `G`:
///   `h` = `b` (half-pel horizontal) · `v` = `h` (half-pel vertical) · `c` = `j` (centre)
#[derive(Clone, Debug)]
pub struct HpelPlanes {
    /// Edge-replicated FULL-pel plane. Needed as an operand for the quarter
    /// positions that average against `G`, and padded so out-of-frame candidates
    /// resolve without a bounds decline.
    pub f: Vec<u8>,
    pub h: Vec<u8>,
    pub v: Vec<u8>,
    pub c: Vec<u8>,
    /// Row stride of every plane (`cw + 2*HPEL_PAD`).
    pub stride: usize,
    pub pad: usize,
    pub pw: usize,
    pub ph: usize,
    pub cw: usize,
    pub ch: usize,
}

/// Border added to every cached plane, in samples.
///
/// Sized so a ±24 motion search around any macroblock still lands inside: a 16-wide
/// block at the right edge reaches `cw + 24`, and reading its `+1` neighbour needs
/// `cw + 41` ≤ `cw + PAD`. Without a border the plane cache DECLINED every candidate
/// whose vector left the picture — 62.5% of the remaining `mc_luma` fallbacks, at
/// ~195 ns against 33 ns for a plane read.
///
/// SWEPT (foreman / bus / park_joy), because a bigger border is a
/// TRADE — more candidates covered, but a larger working set (4 planes over
/// `(cw+2P)(ch+2P)`) and a costlier per-frame build:
///
/// | pad | foreman fallbacks | bus | park_joy | build (foreman) |
/// |---|---|---|---|---|
/// | 0  | 207,537 | 838,581 | 6,354,258 | 5.7 ms |
/// | 8  | 127,407 | 774,592 | 6,172,777 | 4.8 ms |
/// | **16** | **127,274** | **765,625** | **6,122,122** | **6.1 ms** |
/// | 32 | 127,262 | 764,999 | 6,093,358 | 9.0 ms |
///
/// Padding replicated around each cached half-pel plane.
///
/// 32, raised from 16 after the edge full-pel path landed. The earlier sweep found the
/// pad made NO difference and was correctly recorded as refuted -- but that measurement
/// ran against a population dominated by FULL-PEL declines, which `hpel_ref` now serves.
/// The refutation expired when its baseline moved. Re-swept, pad 32 removes essentially
/// every remaining search fallback (football 17,662 -> 175, foreman 3,238 -> 0).
///
/// Priced at the PIPELINE level, not by component arithmetic -- the component estimate
/// was wrong in both directions (it mixed an rdtsc cycle census with profiler
/// milliseconds via an assumed clock, and read a 23-sample build cost off single runs).
/// Paired ABBA, median of paired ratios: bus 1.113x (14/14), blue_sky 1.054x (6/6),
/// park_joy 1.050x (11/12), football 1.026x (11/14), foreman 1.015x (ns),
/// crowd_run 1.005x (ns). No clip regresses; the gain tracks edge-overhang density, so
/// fast pans benefit most and large frames least.
///
/// Byte-identical at 16/32/64 -- a wider pad grows the planes but never changes a value
/// read.
pub const HPEL_PAD: usize = 32;

/// Builds the three half-pel planes for `reference` (`cw`×`ch`, MB-aligned).
///
/// Bit-exact with [`mc_luma`] by CONSTRUCTION: it walks the frame in the same 16×16
/// blocks, extracts the same clamped halo via [`luma_tile_into`], and runs the same
/// `luma_h`/`luma_v`/`luma_centre` kernels. Each output sample depends only on the
/// six clamped input samples around it, so the value is independent of which block
/// computed it.
/// Edge-replicates a `w`×`h` plane into a `(w+2·pad)`×`(h+2·pad)` padded copy —
/// openh264's `ExpandPicture`. `mc_luma` clamps each tap independently via `at()`,
/// and clamping IS edge replication, so filtering the padded plane is bit-identical
/// to the clamped-read original (the argument `build_hpel_planes` rests on; the
/// decoder's per-reference padding reuses it so per-MC-call tile extraction dies).
pub fn pad_plane(src: &[u8], w: usize, h: usize, pad: usize) -> Vec<u8> {
    pad_plane_into(Vec::new(), src, w, h, pad)
}

/// `pad_plane` reusing a recycled buffer. Every byte of the padded plane is
/// written by the row loop below (left fill + interior copy + right fill, all
/// `ph` rows), so a RIGHT-SIZED recycled buffer needs no clearing at all — and,
/// unlike a fresh `vec![0; n]` (whose zero pages are free but whose first
/// touches page-fault), its pages are already mapped and warm. This is what the
/// earlier pad_plane memset-elimination attempt (refuted, WHYS ledger) was
/// missing: on a FRESH alloc the memset is free; the win only exists when the
/// allocation itself is recycled.
pub fn pad_plane_into(mut f: Vec<u8>, src: &[u8], w: usize, h: usize, pad: usize) -> Vec<u8> {
    let (pw, ph) = (w + 2 * pad, h + 2 * pad);
    if f.len() != pw * ph {
        f.clear();
        f.resize(pw * ph, 0);
    }
    for y in 0..ph {
        let sy = (y as isize - pad as isize).clamp(0, h as isize - 1) as usize;
        let row = &src[sy * w..sy * w + w];
        let d = &mut f[y * pw..y * pw + pw];
        let (Some(&l), Some(&r)) = (row.first(), row.last()) else {
            continue;
        };
        d[..pad].fill(l);
        d[pad..pad + w].copy_from_slice(row);
        d[pad + w..].fill(r);
    }
    f
}

/// Construye la fuente acolchada y los tres planos de medio píxel de una referencia.
///
/// # Errors
/// `FueraDeRango` si una tesela no contiene las tomas que necesita.
pub fn build_hpel_planes(
    reference: &[u8],
    cw: usize,
    ch: usize,
) -> Result<HpelPlanes, FueraDeRango> {
    let pad = HPEL_PAD;
    let (pw, ph) = (cw + 2 * pad, ch + 2 * pad);
    // 1. Edge-replicated source (see `pad_plane` for why this is bit-identical).
    let f = pad_plane(reference, cw, ch, pad);
    // 2. Filter the three half-pel planes over the padded area.
    let mut h = vec![0u8; pw * ph];
    let mut v = vec![0u8; pw * ph];
    let mut c = vec![0u8; pw * ph];
    // Dispatch ladder: the AVX2 FUSED single pass (side-by-side descent: the tile
    // walk measured 8× x264's hpel-filter, whose fused shape this kernel mirrors)
    // → tile walk. Both are byte-identical (`fused_hpel_builder_matches_tiles`,
    // `avx2_fused_matches_scalar_fused`); the choice is speed only.
    #[cfg(accel)]
    let done = rusty_h264_accel::hpel_fused(&f, pw, ph, &mut h, &mut v, &mut c);
    #[cfg(not(accel))]
    let done = false;
    if !done {
        build_hpel_tiles(&f, pw, ph, &mut h, &mut v, &mut c)?;
    }
    Ok(HpelPlanes {
        f,
        h,
        v,
        c,
        stride: pw,
        pad,
        pw,
        ph,
        cw,
        ch,
    })
}

/// The ORIGINAL builder — walks 16×16 tiles through `luma_tile_into` + the MC
/// kernels. Es el camino sin el constructor AVX2 y el oráculo del constructor fusionado.
fn build_hpel_tiles(
    f: &[u8],
    pw: usize,
    ph: usize,
    h: &mut [u8],
    v: &mut [u8],
    c: &mut [u8],
) -> Result<(), FueraDeRango> {
    let mut tile = [0u8; LUMA_TILE * LUMA_TILE];
    let mut blk = [0u8; 256];
    let mut by = 0;
    while by < ph {
        let mut bx = 0;
        while bx < pw {
            let bh = 16.min(ph - by);
            let bwid = 16.min(pw - bx);
            let ts = luma_tile_into(
                &mut tile,
                f,
                (pw, ph),
                (bx as isize, by as isize),
                (bwid, bh),
            )?;
            for kind in 0u8..3 {
                let plane: &mut [u8] = match kind {
                    0 => &mut *h,
                    1 => &mut *v,
                    _ => &mut *c,
                };
                match kind {
                    0 => luma_h(&tile, ts, bwid, bh, 0, 0, &mut blk),
                    1 => luma_v(&tile, ts, bwid, bh, 0, 0, &mut blk)?,
                    _ => luma_centre(&tile, ts, bwid, bh, &mut blk)?,
                }
                for r in 0..bh {
                    plane[(by + r) * pw + bx..][..bwid].copy_from_slice(&blk[r * bwid..][..bwid]);
                }
            }
            bx += 16;
        }
        by += 16;
    }
    Ok(())
}

/// Campaign 3 (lets-win-optimize.md): the FUSED single-pass builder — x264's
/// `hpel_filter` shape. The tile walk re-extracts a `(16+5)²` halo per 16×16 block
/// (1.7× redundant reads), re-runs per-tile dispatch, and copies each block out
/// three times; this pass reads each source row band once and writes H, V, C
/// directly. BYTE-IDENTICAL to the tile builder: the same 6-tap integer formulas
/// over the same clamped `f` samples (interior rows/cols never clamp; the ±2/3
/// borders of the padded plane clamp exactly like `luma_tile_into`'s halo does),
/// and the centre's i32 vertical intermediates reproduce `luma_centre`'s order.
/// Fijado por `fused_hpel_builder_matches_tiles`, la única prueba que aún lo usa.
#[cfg(test)]
fn build_hpel_fused(f: &[u8], pw: usize, ph: usize, h: &mut [u8], v: &mut [u8], c: &mut [u8]) {
    let cl = |i: isize, hi: usize| i.clamp(0, hi as isize - 1) as usize;
    // Extended per-row buffers: index j covers column j-2 (clamped into the plane).
    let mut vt_buf = vec![0i32; pw + 5]; // vertical 6-tap intermediates (unrounded)
    let mut hb_buf = vec![0u8; pw + 5]; // this row's source with clamped column halo
                                        // STATE THE LENGTH ONCE. Read back as `vt[x] ..= vt[x + 5]` for `x < pw`,
                                        // which is in range by construction — but against a `Vec`'s runtime length
                                        // LLVM cannot see it, so all six taps checked on every column of every row.
                                        // Reborrowing at the literal `pw + 5` makes `x + 5 < pw + 5` provable.
    let vt = &mut vt_buf[..pw + 5];
    let hb = &mut hb_buf[..pw + 5];
    for y in 0..ph {
        let (ym2, ym1, y0, yp1, yp2, yp3) = (
            cl(y as isize - 2, ph) * pw,
            cl(y as isize - 1, ph) * pw,
            y * pw,
            cl(y as isize + 1, ph) * pw,
            cl(y as isize + 2, ph) * pw,
            cl(y as isize + 3, ph) * pw,
        );
        // Column-clamped borders (≤5 samples each side); the interior runs with
        // DIRECT indices — a per-element clamp in this loop defeats
        // autovectorization and measured the whole builder 2.2× slower.
        // SIX SOURCE ROWS, bound once. The taps stride by `pw`, which is the
        // geometry whose row-slicing was REFUTED in `luma_centre` (+9.3%) — but
        // that runs at bw = 4, where six slices replace twenty-four checks. Here
        // `pw` is the frame width, so six slices replace ~6 * pw of them. Width
        // is the variable that refutation did not vary.
        let rows: [&[u8]; 6] = [
            &f[ym2..][..pw],
            &f[ym1..][..pw],
            &f[y0..][..pw],
            &f[yp1..][..pw],
            &f[yp2..][..pw],
            &f[yp3..][..pw],
        ];
        // Branchless taps: a `.get` per row is a conditional move, and it beats
        // both the indexed form and a per-row re-slice (the shape refuted on
        // `luma_centre`). Same finding as `luma_v`.
        let tap = |x: usize| {
            let g = |k: usize| rows[k & 7].get(x).copied().unwrap_or(0) as i32;
            g(0) - 5 * g(1) + 20 * g(2) + 20 * g(3) - 5 * g(4) + g(5)
        };
        // Walk `vt`/`hb` in three ZIPPED segments rather than indexing them: the
        // buffers are `pw + 5` long and every `j` is inside, but relating the two
        // is what LLVM would not do. Three slice pairs cover all `pw + 5` writes.
        let mid = rows[2];
        let (lo, edge) = (
            mid.first().copied().unwrap_or(0),
            mid.last().copied().unwrap_or(0),
        );
        let tap_edge = tap(pw.saturating_sub(1));
        for (v, h) in vt[..2].iter_mut().zip(hb[..2].iter_mut()) {
            (*v, *h) = (tap(0), lo);
        }
        for (x, (v, h)) in vt[2..pw + 2]
            .iter_mut()
            .zip(hb[2..pw + 2].iter_mut())
            .enumerate()
        {
            (*v, *h) = (tap(x), mid.get(x).copied().unwrap_or(0));
        }
        for (v, h) in vt[pw + 2..].iter_mut().zip(hb[pw + 2..].iter_mut()) {
            (*v, *h) = (tap_edge, edge);
        }
        let hrow = &mut h[y0..y0 + pw];
        let vrow = &mut v[y0..y0 + pw];
        let crow = &mut c[y0..y0 + pw];
        // `windows(6)` hands back a slice of EXACTLY six, so `w[0]..w[5]` are
        // provable outright — the `vt[x] ..= vt[x + 5]` form needed LLVM to
        // relate `x < pw` to the buffer's length and it would not. `zip` bounds
        // the output row the same way.
        for (o, &t) in vrow.iter_mut().zip(&vt[2..]) {
            *o = clip_u8((t + 16) >> 5) as u8;
        }
        for (o, w) in crow.iter_mut().zip(vt.windows(6)) {
            let s = w[0] - 5 * w[1] + 20 * w[2] + 20 * w[3] - 5 * w[4] + w[5];
            *o = clip_u8((s + 512) >> 10) as u8;
        }
        for (o, w) in hrow.iter_mut().zip(hb.windows(6)) {
            let s = w[0] as i32 - 5 * w[1] as i32 + 20 * w[2] as i32 + 20 * w[3] as i32
                - 5 * w[4] as i32
                + w[5] as i32;
            *o = clip_u8((s + 16) >> 5) as u8;
        }
    }
}

/// Descent C: the SINGLE-PLANE (half-pel) phases — (2,0)->h, (0,2)->v, (2,2)->c —
/// are already contiguous at plane stride, exactly like the interior full-pel case.
/// Hand the consumer `(plane, base, stride)` so it can read them IN PLACE rather than
/// copy 256 bytes into a temp first. Byte-identical to `hpel_block` by construction:
/// same plane, same base, same samples — only the copy is elided.
/// Returns `None` for quarter-pel (a two-plane average, which must be materialized),
/// for out-of-range coordinates, and whenever `hpel_block` would decline.
#[inline]
#[must_use]
pub fn hpel_ref(
    p: &HpelPlanes,
    x0: usize,
    y0: usize,
    bw: usize,
    bh: usize,
    mvx: i32,
    mvy: i32,
) -> Option<(&[u8], usize, usize)> {
    let (fx, fy) = (mvx & 3, mvy & 3);
    let plane = match (fx, fy) {
        (2, 0) => &p.h,
        (0, 2) => &p.v,
        (2, 2) => &p.c,
        // Descent E: FULL-PEL off the frame edge. `hpel_block` declines these before its
        // bounds check ("the full-pel copy path is already optimal"), so the caller fell
        // back to a per-pixel clamped `mc_luma` -- measured at 76-79% of all mc_luma
        // TIME, the single largest consumer of that stage. But `f` IS the padded,
        // edge-replicated reference, so it reproduces mc_luma's clamp exactly and can be
        // read in place like any other phase. (Widening the pad does NOT fix this: the
        // decline count is identical at pad 16 and pad 64 -- it was never a bounds issue.)
        (0, 0) => &p.f,
        _ => return None,
    };
    let (ix0, iy0) = (
        x0 as isize + (mvx >> 2) as isize,
        y0 as isize + (mvy >> 2) as isize,
    );
    let (px, py) = (ix0 + p.pad as isize, iy0 + p.pad as isize);
    // Identical guard to `hpel_block`, including its `+1` slack, so the two paths
    // accept exactly the same candidate set (no bitstream drift from a wider path).
    if px < 0
        || py < 0
        || px + bw as isize + 1 > p.pw as isize
        || py + bh as isize + 1 > p.ph as isize
    {
        return None;
    }
    let base = py as usize * p.stride + px as usize;
    if base + (bh - 1) * p.stride + bw > plane.len() {
        return None;
    }
    Some((plane, base, p.stride))
}

/// Par de operandos de plano y desplazamientos base para una posición de cuarto de píxel:
/// `(plano_a, base_a, plano_b, base_b, paso_fila)`.
pub type ParReferenciasQpel<'a> = (&'a [u8], usize, &'a [u8], usize, usize);

/// Challenge-1 A3: the QUARTER-pel companion of [`hpel_ref`] — resolves the two
/// plane operands whose `(a+b+1)>>1` average IS the prediction, so the ME cost can
/// be one fused avg+SATD kernel pass instead of materialize-256-bytes-then-SATD.
/// Same bounds guard as [`hpel_block`] (identical accepted-candidate set); returns
/// `(plane_a, base_a, plane_b, base_b, stride)`. `None` for non-quarter phases
/// (those are [`hpel_ref`]'s) and whenever `hpel_block` would decline.
#[inline]
#[must_use]
pub fn hpel_qpel_refs(
    p: &HpelPlanes,
    x0: usize,
    y0: usize,
    bw: usize,
    bh: usize,
    mvx: i32,
    mvy: i32,
) -> Option<ParReferenciasQpel<'_>> {
    let (fx, fy) = (mvx & 3, mvy & 3);
    // Quarter phases only: at least one fractional part is odd.
    if fx & 1 == 0 && fy & 1 == 0 {
        return None;
    }
    let (ix0, iy0) = (
        x0 as isize + (mvx >> 2) as isize,
        y0 as isize + (mvy >> 2) as isize,
    );
    let (px, py) = (ix0 + p.pad as isize, iy0 + p.pad as isize);
    if px < 0
        || py < 0
        || px + bw as isize + 1 > p.pw as isize
        || py + bh as isize + 1 > p.ph as isize
    {
        return None;
    }
    let stride = p.stride;
    let base = py as usize * stride + px as usize;
    let g: &[u8] = &p.f;
    // The SAME operand table as `hpel_block`'s quarter arm (the spec's `m`/`s`
    // neighbour shifts baked into the base offsets).
    let (pa, oa, pb, ob): (&[u8], usize, &[u8], usize) = match (fx, fy) {
        (1, 0) => (g, 0, &p.h, 0),
        (3, 0) => (g, 1, &p.h, 0),
        (0, 1) => (g, 0, &p.v, 0),
        (0, 3) => (g, stride, &p.v, 0),
        (1, 1) => (&p.h, 0, &p.v, 0),
        (3, 1) => (&p.h, 0, &p.v, 1),
        (1, 3) => (&p.h, stride, &p.v, 0),
        (3, 3) => (&p.h, stride, &p.v, 1),
        (2, 1) => (&p.h, 0, &p.c, 0),
        (2, 3) => (&p.h, stride, &p.c, 0),
        (1, 2) => (&p.v, 0, &p.c, 0),
        _ => (&p.v, 1, &p.c, 0), // (3, 2)
    };
    // Slice-length check so a short plane can never OOB (the `+1` slack in the
    // bounds test above covers the shifted operand's reach geometrically; this
    // makes it a hard guarantee at the slice level too).
    if base + oa + (bh - 1) * stride + bw > pa.len()
        || base + ob + (bh - 1) * stride + bw > pb.len()
    {
        return None;
    }
    Some((pa, base + oa, pb, base + ob, stride))
}

#[allow(clippy::too_many_arguments)]
pub fn hpel_block(
    p: &HpelPlanes,
    x0: usize,
    y0: usize,
    bw: usize,
    bh: usize,
    mvx: i32,
    mvy: i32,
    out: &mut [u8],
) -> bool {
    let (ix0, iy0) = (
        x0 as isize + (mvx >> 2) as isize,
        y0 as isize + (mvy >> 2) as isize,
    );
    let (fx, fy) = (mvx & 3, mvy & 3);
    if fx == 0 && fy == 0 {
        return false; // the full-pel copy path is already optimal
    }
    // Padded coordinates. The `+1` slack covers the `m`/`s` neighbours.
    let (px, py) = (ix0 + p.pad as isize, iy0 + p.pad as isize);
    if px < 0
        || py < 0
        || px + bw as isize + 1 > p.pw as isize
        || py + bh as isize + 1 > p.ph as isize
    {
        return false;
    }
    let stride = p.stride;
    let base = py as usize * stride + px as usize;

    let single = match (fx, fy) {
        (2, 0) => Some(&p.h),
        (0, 2) => Some(&p.v),
        (2, 2) => Some(&p.c),
        _ => None,
    };
    // Descent C: single-plane (half-pel) reads are CONTIGUOUS AT PLANE STRIDE — they
    // could be SATD'd in place like the interior full-pel path instead of copied into
    // a temp. Count the split before building that path.
    if let Some(src) = single {
        for r in 0..bh {
            out[r * bw..r * bw + bw].copy_from_slice(&src[base + r * stride..][..bw]);
        }
        return true;
    }

    // Quarter-pel: (a + b + 1) >> 1 of two planes. The operand table (the spec's
    // `m`/`s` neighbour shifts) lives in ONE place — `hpel_qpel_refs` — so the
    // materializing path here and the fused avg+SATD path can never drift apart.
    let Some((pa, ba, pb, bb, qstride)) = hpel_qpel_refs(p, x0, y0, bw, bh, mvx, mvy) else {
        return false;
    };
    // The asm read (inline-execution.md §11.6c) refuted "LLVM already gets this":
    // all three `avg_rows` widths compile fully SCALAR — 0 `pavgb`, 0 xmm — while
    // `pixel_avg` is the SAME `(a+b+1)>>1` with SSE2/NEON arms and a differential
    // test. Compose, don't write: the kernel takes the accel build; `avg_rows`
    // stays below as the reachable scalar twin (non-accel builds and the oracle).
    #[cfg(accel)]
    if bw == 16 || bw == 8 || bw == 4 {
        rusty_h264_accel::pixel_avg(out, &pa[ba..], qstride, &pb[bb..], qstride, bw, bh);
        return true;
    }
    match bw {
        16 => avg_rows::<16>(pa, ba, pb, bb, qstride, bh, out),
        8 => avg_rows::<8>(pa, ba, pb, bb, qstride, bh, out),
        4 => avg_rows::<4>(pa, ba, pb, bb, qstride, bh, out),
        _ => return false,
    }
    true
}

#[inline]
fn avg_rows<const BW: usize>(
    pa: &[u8],
    oa: usize,
    pb: &[u8],
    ob: usize,
    cw: usize,
    bh: usize,
    out: &mut [u8],
) {
    for r in 0..bh {
        let sa = &pa[oa + r * cw..][..BW];
        let sb = &pb[ob + r * cw..][..BW];
        let o = &mut out[r * BW..r * BW + BW];
        for i in 0..BW {
            o[i] = ((sa[i] as u16 + sb[i] as u16 + 1) >> 1) as u8;
        }
    }
}

/// Quarter-pel motion compensation of a `bw`×`bh` luma block (`McLuma_c`).
///
/// # Errors
/// `FueraDeRango` si el plano está vacío o no cubre `cw × ch`.
#[allow(clippy::too_many_arguments)]
pub fn mc_luma(
    reference: &[u8],
    cw: usize,
    ch: usize,
    x0: usize,
    y0: usize,
    bw: usize,
    bh: usize,
    mvx: i32,
    mvy: i32,
    out: &mut [u8],
) -> Result<(), FueraDeRango> {
    plano_cubre(reference, cw, ch)?;
    let (ix0, iy0) = (
        x0 as isize + (mvx >> 2) as isize,
        y0 as isize + (mvy >> 2) as isize,
    );
    let (fx, fy) = (mvx & 3, mvy & 3);
    // Descent E: time this call into its (size, phase) bucket. Guard drops at fn end.
    if fx == 0 && fy == 0 {
        // Full-pel: a verbatim copy of the reference (`McCopy_c`). Interior → a
        // row-wise slice copy (auto-vectorized); edge → per-pixel clamped.
        if ix0 >= 0
            && iy0 >= 0
            && ix0 + bw as isize <= cw as isize
            && iy0 + bh as isize <= ch as isize
            && reference.len() >= cw * ch
        {
            let (rx, ry) = (ix0 as usize, iy0 as usize);
            // Fixed-size fast path for the full 16×16 MB (the overwhelming common
            // case). With `bw` a runtime parameter each `copy_from_slice(..bw)` is a
            // variable-length `memcpy` CALL per row (~38 ns / MB); const-16 slices
            // let LLVM emit inline 16-byte moves (~2 ns / MB). Byte-identical.
            if bw == 16 && bh == 16 {
                for dy in 0..16 {
                    let s = &reference[(ry + dy) * cw + rx..];
                    out[dy * 16..dy * 16 + 16].copy_from_slice(&s[..16]);
                }
            } else {
                for dy in 0..bh {
                    out[dy * bw..dy * bw + bw]
                        .copy_from_slice(&reference[(ry + dy) * cw + rx..][..bw]);
                }
            }
        } else {
            for dy in 0..bh {
                let orow = &mut out[dy * bw..][..bw];
                for (dx, o) in orow.iter_mut().enumerate() {
                    *o = at(reference, cw, ch, ix0 + dx as isize, iy0 + dy as isize)? as u8;
                }
            }
        }
        return Ok(());
    }
    // Sub-pel: extract the clamped tile once, then run the openh264 block kernels.
    // Tile + half-pel staging live in per-thread scratch (see `McScratch`) so the
    // ~1.4 KB of zero-fill and the tile's return-by-value copy are not repaid on
    // every motion-search candidate. Destructuring gives `tile` and `a`/`b` as
    // disjoint borrows of the same scratch.
    MC_SCRATCH.with(|s| {
        let McScratch { tile, a, b } = &mut *s.borrow_mut();
        let ts = luma_tile_into(tile, reference, (cw, ch), (ix0, iy0), (bw, bh))?;
        mc_luma_subpel(tile, ts, bw, bh, fx, fy, a, b, out)
    })
}

/// Eighth-pel bilinear motion compensation of a `bw`×`bh` chroma block (spec
/// §8.4.2.2.2). The chroma motion vector equals the luma MV; for 4:2:0 it is
/// interpreted at eighth-chroma-sample resolution.
///
/// # Errors
/// `FueraDeRango` si el plano está vacío o no cubre `cw × ch` (antes, muestra negra).
#[allow(clippy::too_many_arguments)]
pub fn mc_chroma(
    reference: &[u8],
    cw: usize,
    ch: usize,
    x0: usize,
    y0: usize,
    bw: usize,
    bh: usize,
    mvx: i32,
    mvy: i32,
    out: &mut [u8],
) -> Result<(), FueraDeRango> {
    plano_cubre(reference, cw, ch)?;
    let (ix0, iy0) = (
        x0 as isize + (mvx >> 3) as isize,
        y0 as isize + (mvy >> 3) as isize,
    );
    let (fx, fy) = (mvx & 7, mvy & 7);
    // Full-pel and fully inside the frame: `(64·a + 32) >> 6 == a`, a verbatim copy.
    // Skip the per-pixel bilinear + 4× clamped `at()`; copy row-wise. Bit-identical.
    if fx == 0
        && fy == 0
        && ix0 >= 0
        && iy0 >= 0
        && ix0 + bw as isize <= cw as isize
        && iy0 + bh as isize <= ch as isize
        && reference.len() >= cw * ch
    {
        let (rx, ry) = (ix0 as usize, iy0 as usize);
        // Const-8 fast path for the full 8×8 chroma block (skip / P_16x16 common
        // case): fixed-size slices → inline 8-byte moves, not a runtime-length
        // `memcpy` call per row. Byte-identical. See the luma twin in `mc_luma`.
        if bw == 8 && bh == 8 {
            for dy in 0..8 {
                let src = &reference[(ry + dy) * cw + rx..];
                out[dy * 8..dy * 8 + 8].copy_from_slice(&src[..8]);
            }
        } else {
            for dy in 0..bh {
                let src = &reference[(ry + dy) * cw + rx..][..bw];
                out[dy * bw..dy * bw + bw].copy_from_slice(src);
            }
        }
        return Ok(());
    }
    // Sub-pel (and full-pel edge): extract the clamped (bw+1)×(bh+1) tile once —
    // the edge-extended input `McChromaWithFragMv_c` reads — then the bilinear
    // `A·p + B·p₊₁ + C·p₊ₛ + D·p₊ₛ₊₁` (`g_kuiABCD` weights), `(·+32)>>6`. A chroma
    // block is at most 8×8 (half a 16×16 MB), so a 9×9 tile suffices. Bit-identical:
    // the clamped tile reproduces `at()`, and full-pel weights give `(64·a+32)>>6==a`.
    let ts = bw + 1;
    let mut t = [0u8; 9 * 9];
    // Interior fast path (see `luma_tile`): clamp-free contiguous row copies.
    if ix0 >= 0
        && iy0 >= 0
        && ix0 + ts as isize <= cw as isize
        && iy0 + (bh + 1) as isize <= ch as isize
        && reference.len() >= cw * ch
    {
        let (rx0, ry0) = (ix0 as usize, iy0 as usize);
        for ty in 0..bh + 1 {
            let src = (ry0 + ty) * cw + rx0;
            t[ty * ts..ty * ts + ts].copy_from_slice(&reference[src..src + ts]);
        }
    } else {
        let (ux, uy) = (ultima(cw)?, ultima(ch)?);
        for ty in 0..bh + 1 {
            let ry = (iy0 + ty as isize).clamp(0, uy) as usize * cw;
            let trow = &mut t[ty * ts..][..ts];
            for (tx, o) in trow.iter_mut().enumerate() {
                let rx = (ix0 + tx as isize).clamp(0, ux) as usize;
                *o = *reference
                    .get(ry + rx)
                    .ok_or(FueraDeRango("plano de referencia"))?;
            }
        }
    }
    let (wa, wb, wc, wd) = ((8 - fx) * (8 - fy), fx * (8 - fy), (8 - fx) * fy, fx * fy);
    // 8-wide chroma (full-MB and 16×16-partition chroma — the common case) → the
    // openh264 SSE2 bilinear over the same clamped tile. Width 2/4 stay scalar
    // (width-4 is only an MMX kernel; width 2 has none). Bit-identical.
    #[cfg(accel)]
    if bw == 8 {
        let abcd = [wa as u8, wb as u8, wc as u8, wd as u8];
        rusty_h264_accel::mc_chroma_w8(&t, ts, out, bw, &abcd, bh);
        return Ok(());
    }
    // Fourth site of the two-row `windows(2)` bilinear (see `mc_chroma_padded`).
    for r in 0..bh {
        let (r0, r1) = (&t[r * ts..][..bw + 1], &t[(r + 1) * ts..][..bw + 1]);
        let orow = &mut out[r * bw..][..bw];
        for ((o, a), b) in orow.iter_mut().zip(r0.windows(2)).zip(r1.windows(2)) {
            let v = wa * a[0] as i32 + wb * a[1] as i32 + wc * b[0] as i32 + wd * b[1] as i32;
            *o = ((v + 32) >> 6) as u8;
        }
    }
    Ok(())
}

// ---- Padded-reference MC (openh264 ExpandPicture style) ----
//
// A reference plane is stored with a replicated-edge border (`PAD_L` luma /
// `PAD_C` chroma pixels), the picture origin at `(pad, pad)` and `stride` = the
// padded width. MC then reads the frame DIRECTLY at the MV offset — no per-call
// clamped tile — because reads into the border hit valid replicated pixels. For
// the rare MV whose 6-tap halo would exceed the border, we fall back to the
// clamped tile (reading the padded interior), which is bit-identical to the
// exact-frame path.
//
// STATUS: implemented + bit-exact (the `mc_*_padded_matches_exact` tests + a full
// decoder wiring verified 35/35 corpus MATCH), but measured **~0** vs the tile
// path on x86-64 — once `luma_tile`'s interior fast path made extraction a
// vectorized copy, the remaining win (skipping the copy) is offset by the
// padded direct read's worse kernel cache locality (full-frame stride vs the
// L1-resident tile) plus the per-frame expand/copy cost. Kept UNWIRED as a ready
// option for a workload/target where tile extraction dominates (e.g. slower
// memory, or hand-asm kernels tuned for the big-stride read). To wire: store
// `RefFrame` planes padded (`expand_plane` in `as_reference`) and call these
// instead of `mc_luma`/`mc_chroma`.

/// Luma reference border width (covers full-pel MVs to ±30 px before fallback).
pub const PAD_L: usize = 32;
/// Chroma reference border width (= `PAD_L`/2, matching the half-rate chroma MV).
pub const PAD_C: usize = 16;

/// Fills the `pad`-wide replicated-edge border of a plane whose picture (`pw×ph`)
/// sits at offset `(pad, pad)` with `stride`. Mirrors openh264 `ExpandPictureLuma_c`:
/// left/right cols replicate the edge pixel; top/bottom rows replicate the (already
/// edge-filled) first/last picture row, so the corners come out right.
pub fn expand_plane(buf: &mut [u8], stride: usize, pad: usize, pw: usize, ph: usize) {
    // ONE slice per row, then two fills. The edge reads and the 2*pad writes were
    // all separate checks into the whole plane; the row segment covers them.
    for y in 0..ph {
        // Split rather than index: `seg[pad]` and `seg[pad + pw - 1]` still
        // checked, because nothing rules out `pw == 0`. `first`/`last` on the
        // middle run say the same thing without a panic path.
        let seg = &mut buf[(y + pad) * stride..][..2 * pad + pw];
        let (lpad, rest) = seg.split_at_mut(pad);
        let (mid, rpad) = rest.split_at_mut(pw);
        let (Some(&left), Some(&right)) = (mid.first(), mid.last()) else {
            continue;
        };
        lpad.fill(left);
        rpad.fill(right);
    }
    let first = pad * stride;
    let last = (pad + ph - 1) * stride;
    for y in 0..pad {
        buf.copy_within(first..first + stride, y * stride);
        buf.copy_within(last..last + stride, (pad + ph + y) * stride);
    }
}

/// Compensación de movimiento de luma sobre un plano acolchado.
///
/// # Errors
/// `FueraDeRango` si el plano acolchado no contiene las muestras que lee el bloque.
#[allow(clippy::too_many_arguments)]
pub fn mc_luma_padded(
    padded: &[u8],
    stride: usize,
    pad: usize,
    pw: usize,
    ph: usize,
    x0: usize,
    y0: usize,
    bw: usize,
    bh: usize,
    mvx: i32,
    mvy: i32,
    out: &mut [u8],
) -> Result<(), FueraDeRango> {
    with_mc_scratch(|scr| {
        mc_luma_padded_pre(
            scr, padded, stride, pad, pw, ph, x0, y0, bw, bh, mvx, mvy, out,
        )
    })
}

/// [`mc_luma_padded`] with the scratch pre-borrowed (see [`with_mc_scratch`]).
///
/// # Errors
/// `FueraDeRango` si el plano acolchado no contiene las muestras que lee el bloque.
#[allow(clippy::too_many_arguments)]
pub fn mc_luma_padded_pre(
    scr: &mut McScratch,
    padded: &[u8],
    stride: usize,
    pad: usize,
    pw: usize,
    ph: usize,
    x0: usize,
    y0: usize,
    bw: usize,
    bh: usize,
    mvx: i32,
    mvy: i32,
    out: &mut [u8],
) -> Result<(), FueraDeRango> {
    let (ix0, iy0) = (
        x0 as isize + (mvx >> 2) as isize,
        y0 as isize + (mvy >> 2) as isize,
    );
    let (fx, fy) = (mvx & 3, mvy & 3);
    // Size x phase census for the DECODER's MC. The facility existed but was wired
    // only into `mc_luma` (the encoder's), so the decoder's own distribution had
    // never been measured -- and it is the one that decides which const-width fast
    // paths are worth having.
    let p = pad as isize;
    let (lo_x, lo_y) = (ix0 - 2, iy0 - 2);
    // Malformed-stream armor: a mutated stream can hand a reference whose plane
    // no longer matches the caller's geometry (mid-stream SPS change). The fast
    // paths index arithmetically, so they require the buffer to be intact; the
    // fallback below reads checked. One compare on the hot path.
    let intact = stride >= pw + 2 * pad && padded.len() >= stride * (ph + 2 * pad);
    let in_range = intact
        && lo_x >= -p
        && lo_y >= -p
        && lo_x + (bw + 5) as isize <= pw as isize + p
        && lo_y + (bh + 5) as isize <= ph as isize + p;
    if in_range {
        if fx == 0 && fy == 0 {
            // CONST-WIDTH row copies. With a runtime `bw` each row lowers to a
            // variable-length `memcpy` CALL, and the census priced a 16x16 full-pel
            // copy at 377 cycles -- ~10x what 256 bytes should cost. H.264 emits
            // only these five widths.
            // ONE span each side, then const-width rows inside it: the source
            // and destination bounds checks are paid once for the block instead
            // of once per row, and the widths stay literal so each row is still
            // a fixed-size move rather than a variable-length memcpy CALL.
            let sb = ((iy0 + p) as usize) * stride + (ix0 + p) as usize;
            macro_rules! rows {
                ($n:expr) => {{
                    let sw = &padded[sb..sb + (bh - 1) * stride + $n];
                    let ow = &mut out[..(bh - 1) * $n + $n];
                    for dy in 0..bh {
                        ow[dy * $n..dy * $n + $n]
                            .copy_from_slice(&sw[dy * stride..dy * stride + $n]);
                    }
                }};
            }
            match bw {
                16 => rows!(16),
                8 => rows!(8),
                4 => rows!(4),
                _ => {
                    let sw = &padded[sb..sb + (bh - 1) * stride + bw];
                    let ow = &mut out[..(bh - 1) * bw + bw];
                    for dy in 0..bh {
                        ow[dy * bw..dy * bw + bw]
                            .copy_from_slice(&sw[dy * stride..dy * stride + bw]);
                    }
                }
            }
        } else {
            let halo = ((lo_y + p) as usize) * stride + (lo_x + p) as usize;
            mc_luma_subpel(
                &padded[halo..],
                stride,
                bw,
                bh,
                fx,
                fy,
                &mut scr.a,
                &mut scr.b,
                out,
            )?;
        }
        return Ok(());
    }
    // Extreme MV: clamp the halo to the real picture, read the padded interior.
    let ts = bw + 5;
    let mut t = [0u8; LUMA_TILE * LUMA_TILE];
    for ty in 0..bh + 5 {
        let py = (lo_y + ty as isize).clamp(0, ph as isize - 1) as usize;
        let ry = (py + pad) * stride;
        // COR-3: una muestra fuera del plano acolchado es un estado incoherente, no
        // gris (el antiguo `unwrap_or(128)`); la fila de origen se toma una vez.
        let fila = padded
            .get(ry + pad..)
            .and_then(|s| s.get(..pw))
            .ok_or(FueraDeRango("plano luma"))?;
        let trow = &mut t[ty * ts..][..ts];
        for (tx, o) in trow.iter_mut().enumerate() {
            let px = (lo_x + tx as isize).clamp(0, pw as isize - 1) as usize;
            *o = *fila.get(px).ok_or(FueraDeRango("plano luma"))?;
        }
    }
    if fx == 0 && fy == 0 {
        for dy in 0..bh {
            let s = (dy + 2) * ts + 2;
            out[dy * bw..dy * bw + bw].copy_from_slice(&t[s..s + bw]);
        }
    } else {
        mc_luma_subpel(&t, ts, bw, bh, fx, fy, &mut scr.a, &mut scr.b, out)?;
    }
    Ok(())
}

/// Eighth-pel chroma MC reading a padded reference directly. Bit-identical to
/// [`mc_chroma`] on the equivalent exact frame.
/// U+V pair form of [`mc_chroma_padded`]: both chroma planes share EVERY piece
/// of MC geometry (mv, frac weights, stride, coordinates, bounds) — only the
/// plane base differs — so one setup + one range check serves two kernel
/// invocations. Per-plane pixel math is byte-identical to two single calls.
/// The extreme-MV clamp path is rare and stays on the single-plane fn.
///
/// # Errors
/// `FueraDeRango` si el plano acolchado no contiene las muestras que lee el bloque.
#[allow(clippy::too_many_arguments)]
pub fn mc_chroma_padded_pair(
    pu: &[u8],
    pv: &[u8],
    stride: usize,
    pad: usize,
    pw: usize,
    ph: usize,
    x0: usize,
    y0: usize,
    bw: usize,
    bh: usize,
    mvx: i32,
    mvy: i32,
    outu: &mut [u8],
    outv: &mut [u8],
) -> Result<(), FueraDeRango> {
    let (ix0, iy0) = (
        x0 as isize + (mvx >> 3) as isize,
        y0 as isize + (mvy >> 3) as isize,
    );
    let (fx, fy) = (mvx & 7, mvy & 7);
    let p = pad as isize;
    let intact = stride >= pw + 2 * pad
        && pu.len() >= stride * (ph + 2 * pad)
        && pv.len() >= stride * (ph + 2 * pad);
    let in_range = intact
        && ix0 >= -p
        && iy0 >= -p
        && ix0 + (bw + 1) as isize <= pw as isize + p
        && iy0 + (bh + 1) as isize <= ph as isize + p;
    if !in_range {
        // Rare (extreme MV or short plane): fall back to the single-plane fn,
        // whose clamp path is the conformance reference.
        mc_chroma_padded(pu, stride, pad, pw, ph, x0, y0, bw, bh, mvx, mvy, outu)?;
        mc_chroma_padded(pv, stride, pad, pw, ph, x0, y0, bw, bh, mvx, mvy, outv)?;
        return Ok(());
    }
    if fx == 0 && fy == 0 {
        for dy in 0..bh {
            let src = ((iy0 + dy as isize + p) as usize) * stride + (ix0 + p) as usize;
            outu[dy * bw..dy * bw + bw].copy_from_slice(&pu[src..src + bw]);
            outv[dy * bw..dy * bw + bw].copy_from_slice(&pv[src..src + bw]);
        }
        return Ok(());
    }
    let (wa, wb, wc, wd) = ((8 - fx) * (8 - fy), fx * (8 - fy), (8 - fx) * fy, fx * fy);
    let halo = ((iy0 + p) as usize) * stride + (ix0 + p) as usize;
    #[cfg(accel)]
    {
        let abcd = [wa as u8, wb as u8, wc as u8, wd as u8];
        if bw == 8 {
            rusty_h264_accel::mc_chroma_w8(&pu[halo..], stride, outu, bw, &abcd, bh);
            rusty_h264_accel::mc_chroma_w8(&pv[halo..], stride, outv, bw, &abcd, bh);
            return Ok(());
        }
        if bw == 4 {
            rusty_h264_accel::mc_chroma_w4(&pu[halo..], stride, outu, bw, &abcd, bh);
            rusty_h264_accel::mc_chroma_w4(&pv[halo..], stride, outv, bw, &abcd, bh);
            return Ok(());
        }
        // 2-WIDE (the chroma of 4-wide luma partitions; finding #6): the w4
        // kernel into a 4-stride scratch, keep 2 columns per row. The two extra
        // source columns are in bounds by the same check the kernel asserts.
        if bw == 2
            && bh <= 8
            && pu.len() >= halo + bh * stride + 5
            && pv.len() >= halo + bh * stride + 5
        {
            for (pl, out) in [(pu, &mut *outu), (pv, &mut *outv)] {
                let mut s = [0u8; 4 * 8];
                rusty_h264_accel::mc_chroma_w4(&pl[halo..], stride, &mut s, 4, &abcd, bh);
                for (d, srow) in out.chunks_exact_mut(2).zip(s.chunks_exact(4)).take(bh) {
                    d.copy_from_slice(&srow[..2]);
                }
            }
            return Ok(());
        }
    }
    for (pl, out) in [(pu, &mut *outu), (pv, &mut *outv)] {
        // TWO ROWS, `windows(2)`. The bilinear tap reads (c, c+1) on this row
        // and the next — four separately checked whole-plane loads per
        // sample. Each window is a slice of EXACTLY two, so all four fold.
        for r in 0..bh {
            let base = halo + r * stride;
            let (r0, r1) = (&pl[base..][..bw + 1], &pl[base + stride..][..bw + 1]);
            let orow = &mut out[r * bw..][..bw];
            for ((o, a), b) in orow.iter_mut().zip(r0.windows(2)).zip(r1.windows(2)) {
                let v = wa * a[0] as i32 + wb * a[1] as i32 + wc * b[0] as i32 + wd * b[1] as i32;
                *o = ((v + 32) >> 6) as u8;
            }
        }
    }
    Ok(())
}

/// Compensación de movimiento de croma sobre un plano acolchado.
///
/// # Errors
/// `FueraDeRango` si el plano acolchado no contiene las muestras que lee el bloque.
#[allow(clippy::too_many_arguments)]
pub fn mc_chroma_padded(
    padded: &[u8],
    stride: usize,
    pad: usize,
    pw: usize,
    ph: usize,
    x0: usize,
    y0: usize,
    bw: usize,
    bh: usize,
    mvx: i32,
    mvy: i32,
    out: &mut [u8],
) -> Result<(), FueraDeRango> {
    let (ix0, iy0) = (
        x0 as isize + (mvx >> 3) as isize,
        y0 as isize + (mvy >> 3) as isize,
    );
    let (fx, fy) = (mvx & 7, mvy & 7);
    let p = pad as isize;
    let intact = stride >= pw + 2 * pad && padded.len() >= stride * (ph + 2 * pad);
    let in_range = intact
        && ix0 >= -p
        && iy0 >= -p
        && ix0 + (bw + 1) as isize <= pw as isize + p
        && iy0 + (bh + 1) as isize <= ph as isize + p;
    let (wa, wb, wc, wd) = ((8 - fx) * (8 - fy), fx * (8 - fy), (8 - fx) * fy, fx * fy);
    if in_range {
        if fx == 0 && fy == 0 {
            // ONE span each side: `bh` rows a fixed stride apart, so the
            // source and destination checks are paid once, not per row.
            let sb = ((iy0 + p) as usize) * stride + (ix0 + p) as usize;
            let sw = &padded[sb..sb + (bh - 1) * stride + bw];
            let ow = &mut out[..(bh - 1) * bw + bw];
            for dy in 0..bh {
                ow[dy * bw..dy * bw + bw].copy_from_slice(&sw[dy * stride..dy * stride + bw]);
            }
            return Ok(());
        }
        let halo = ((iy0 + p) as usize) * stride + (ix0 + p) as usize;
        #[cfg(accel)]
        {
            // H-38: BOTH widths now take asm. The 4-wide path was missing, so every
            // 8×8 B-direct sub-block and sub-8×8 partition fell to the scalar
            // bilinear below — chroma MC then cost as much as luma on half the
            // pixels. Both kernels are pinned bit-identical to that scalar arm.
            let abcd = [wa as u8, wb as u8, wc as u8, wd as u8];
            if bw == 8 {
                rusty_h264_accel::mc_chroma_w8(&padded[halo..], stride, out, bw, &abcd, bh);
                return Ok(());
            }
            if bw == 4 {
                rusty_h264_accel::mc_chroma_w4(&padded[halo..], stride, out, bw, &abcd, bh);
                return Ok(());
            }
            if bw == 2 && bh <= 8 && padded.len() >= halo + bh * stride + 5 {
                let mut s = [0u8; 4 * 8];
                rusty_h264_accel::mc_chroma_w4(&padded[halo..], stride, &mut s, 4, &abcd, bh);
                for (d, srow) in out.chunks_exact_mut(2).zip(s.chunks_exact(4)).take(bh) {
                    d.copy_from_slice(&srow[..2]);
                }
                return Ok(());
            }
        }
        // TWO ROWS, `windows(2)`. The bilinear tap reads (c, c+1) on this row
        // and the next — four separately checked whole-plane loads per
        // sample. Each window is a slice of EXACTLY two, so all four fold.
        for r in 0..bh {
            let base = halo + r * stride;
            let (r0, r1) = (
                &padded[base..][..bw + 1],
                &padded[base + stride..][..bw + 1],
            );
            let orow = &mut out[r * bw..][..bw];
            for ((o, a), b) in orow.iter_mut().zip(r0.windows(2)).zip(r1.windows(2)) {
                let v = wa * a[0] as i32 + wb * a[1] as i32 + wc * b[0] as i32 + wd * b[1] as i32;
                *o = ((v + 32) >> 6) as u8;
            }
        }
        return Ok(());
    }
    // Extreme MV: clamp the (bw+1)² halo to the real picture, read the padded interior.
    let ts = bw + 1;
    let mut t = [0u8; 9 * 9];
    for ty in 0..bh + 1 {
        let py = (iy0 + ty as isize).clamp(0, ph as isize - 1) as usize;
        let ry = (py + pad) * stride;
        // COR-3: fuera del plano acolchado es error, no gris (ver la gemela de luma).
        let fila = padded
            .get(ry + pad..)
            .and_then(|s| s.get(..pw))
            .ok_or(FueraDeRango("plano croma"))?;
        let trow = &mut t[ty * ts..][..ts];
        for (tx, o) in trow.iter_mut().enumerate() {
            let px = (ix0 + tx as isize).clamp(0, pw as isize - 1) as usize;
            *o = *fila.get(px).ok_or(FueraDeRango("plano croma"))?;
        }
    }
    // Same two-row `windows(2)` shape as the padded chroma paths.
    for r in 0..bh {
        let (r0, r1) = (&t[r * ts..][..bw + 1], &t[(r + 1) * ts..][..bw + 1]);
        let orow = &mut out[r * bw..][..bw];
        for ((o, a), b) in orow.iter_mut().zip(r0.windows(2)).zip(r1.windows(2)) {
            let v = wa * a[0] as i32 + wb * a[1] as i32 + wc * b[0] as i32 + wd * b[1] as i32;
            *o = ((v + 32) >> 6) as u8;
        }
    }
    Ok(())
}

#[cfg(test)]
#[cfg(test)]
mod tests;
