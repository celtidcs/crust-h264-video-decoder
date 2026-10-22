//! Búfer de imágenes decodificadas (DPB) y marcado de referencias (`dec_ref_pic_marking`, UIT-T H.264 §8.2.5).
//!
//! Define el fotograma de referencia con bordes rellenados ([`RefFrame`]), las operaciones de
//! gestión de memoria [`Mmco`] (§7.4.3.3) y las rutinas del [`Decoder`] para relleno de huecos
//! en `frame_num` (§8.2.5.2), ventana deslizante, marcado adaptativo y reciclaje de planos.

use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use rusty_h264_common::BitReader;

use crate::constantes::{MAX_OPERACIONES_MMCO, MAX_PLANOS_POOL, MUESTRA_NEUTRA_8BIT};
use crate::error::DecodeError;
use crate::Decoder;

/// A reference picture: deblocked reconstruction at coded resolution.
/// Stored now (4a); read by motion compensation in 4b.
/// Shared handle to a reference picture. The DPB and every per-slice reference
/// list hold `Arc`s: list construction used to DEEP-CLONE each entry's planes +
/// motion grids per slice (H-32 found ~600 KB+/slice of pure memcpy on B
/// streams); an `Arc` clone is a refcount bump reading the same bytes, so the
/// change is byte-identical by construction. `Arc::make_mut` covers the one
/// mutation site (MMCO long-term marking).
pub(crate) type Ref = Arc<RefFrame>;

/// Contador atómico monotónico para asignar identificadores únicos de imagen (`pic_id`).
static NEXT_PIC_ID: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(1);

/// Devuelve un nuevo identificador único de imagen para distinguir referencias en el filtro de bordes (§8.7.2.1).
pub(crate) fn next_pic_id() -> u32 {
    NEXT_PIC_ID.fetch_add(1, core::sync::atomic::Ordering::Relaxed)
}

/// Luma / chroma pad of every [`RefFrame`] plane. Luma 16 serves MVs overshooting
/// the picture by up to ~14 px in place (chroma: half that, matching); wilder MVs
/// take `mc_*_padded`'s clamped-halo fallback — correct, just slower.
pub(crate) const LPAD: usize = 16;

/// Margen de relleno de borde en croma (8 píxeles por lado para 4:2:0).
pub(crate) const CPAD: usize = 8;

/// Tope de seguridad para el tamaño del DPB tras marcado adaptativo.
const TOPE_SEGURIDAD_DPB: usize = 16;

/// Fotograma de referencia reconstruido y filtrado, con planos rellenados en los bordes.
#[derive(Debug, Default, Clone)]
pub(crate) struct RefFrame {
    /// EDGE-PADDED planes (openh264 `ExpandPicture`): built ONCE per reference
    /// frame so motion compensation reads them in place — the per-MC-call
    /// clamped-tile extraction (~400 B copied per call, ~100 MB/clip on real
    /// streams) dies with this. Luma pad [`LPAD`], chroma pad [`CPAD`]; strides
    /// via [`RefFrame::lstride`]/[`RefFrame::cstride`].
    pub py: Vec<u8>,
    pub pu: Vec<u8>,
    pub pv: Vec<u8>,
    pub cw: usize,
    pub ch: usize,
    /// `frame_num` of the picture, for PicNum-based reference-list reordering.
    pub frame_num: u32,
    /// `PicOrderCnt` of the picture, for B-slice reference-list ordering.
    pub poc: i32,
    /// Per-4×4-block List-0 motion field (motion vector + reference index, `-1`
    /// for intra), and the block-grid width. Read as the *co-located* picture's
    /// motion for B-slice direct prediction (`colZeroFlag`, temporal direct).
    pub mv: Vec<(i32, i32)>,
    pub ref_idx: Vec<i32>,
    /// Per-4×4-block **List-1** motion. Needed because the co-located motion
    /// derivation (spec §8.4.1.2.1) falls back to List-1 when the co-located block
    /// has no List-0 prediction (`predFlagL0Col == 0`). A co-located picture only
    /// contains L1-only blocks when it is itself a B picture — which is precisely
    /// what b-pyramid produces, so this stayed unexercised until B-references
    /// appeared.
    pub mv1: Vec<(i32, i32)>,
    pub ref_idx1: Vec<i32>,
    /// Per-4×4-block POC of the List-0 picture each block referenced (`i32::MIN`
    /// for intra). Used by temporal direct's `MapColToList0` (the co-located
    /// reference index alone is meaningless in the current list).
    pub ref_poc: Vec<i32>,
    pub w4: usize,
    /// Long-term reference state. Long-term refs sit after short-term ones in
    /// `RefPicList0` (ordered by `long_term_idx` ascending) and survive the
    /// sliding window until explicitly unmarked (spec §8.2.4).
    pub long_term: bool,
    pub long_term_idx: u32,
    /// Globally unique, monotonically increasing identity distinguishing
    /// distinct pictures in the DPB for in-loop deblocking filter strength
    /// derivation (ITU-T Rec. H.264 §8.7.2.1).
    pub pic_id: u32,
}

impl RefFrame {
    /// Plano de luma con bordes rellenados, listo para la compensación de movimiento.
    #[inline]
    pub(crate) fn plano_luma(&self) -> &[u8] {
        &self.py
    }

    /// Plano de croma con bordes rellenados: `plano == 0` es Cb y cualquier otro valor, Cr.
    #[inline]
    pub(crate) fn plano_croma(&self, plano: usize) -> &[u8] {
        if plano == 0 {
            &self.pu
        } else {
            &self.pv
        }
    }

    #[inline]
    pub(crate) fn fn_num(&self) -> u32 {
        self.frame_num
    }

    #[inline]
    pub(crate) fn pic_poc(&self) -> i32 {
        self.poc
    }

    #[inline]
    pub(crate) fn pic_id(&self) -> u32 {
        self.pic_id
    }

    #[inline]
    pub(crate) fn is_long_term(&self) -> bool {
        self.long_term
    }

    #[inline]
    pub(crate) fn lt_idx(&self) -> u32 {
        self.long_term_idx
    }

    /// Paso de fila (*stride*) del plano de luma con bordes rellenados.
    #[must_use]
    #[inline]
    pub fn lstride(&self) -> usize {
        self.cw + 2 * LPAD
    }

    /// Paso de fila (*stride*) de cada plano de croma con bordes rellenados.
    #[must_use]
    #[inline]
    pub fn cstride(&self) -> usize {
        self.cw / 2 + 2 * CPAD
    }

    /// Rows in the PADDED luma plane. Four call sites recovered this as
    /// `plane.len() / lstride()` -- an integer DIVIDE by a runtime value, per
    /// P_Skip macroblock and per B-skip validity test, to recompute a constant
    /// property of the frame. The plane is allocated `(cw + 2*LPAD) *
    /// (ch + 2*LPAD)` and the stride is the first factor, so the row count is
    /// the second one, exactly.
    #[must_use]
    #[inline]
    pub fn lrows(&self) -> usize {
        self.ch + 2 * LPAD
    }

    /// Rows in either PADDED chroma plane (`(cw/2 + 2*CPAD) * (ch/2 + 2*CPAD)`).
    #[must_use]
    #[inline]
    pub fn crows(&self) -> usize {
        self.ch / 2 + 2 * CPAD
    }
}

/// A memory-management control operation (`dec_ref_pic_marking`, spec §7.4.3.3).
#[derive(Clone, Copy, Debug)]
pub(crate) enum Mmco {
    /// 1: mark a short-term reference (by PicNum) as unused.
    Unref(u32),
    /// 2: mark a long-term reference (by LongTermPicNum) as unused.
    UnrefLong(u32),
    /// 3: assign a short-term reference (by PicNum) a LongTermFrameIdx.
    AssignLong(u32, u32),
    /// 4: drop long-term references with idx ≥ max_long_term_frame_idx_plus1.
    MaxLong(u32),
    /// 5: empty the DPB (and reset the current picture's frame_num to 0).
    Reset,
    /// 6: mark the current picture long-term with this LongTermFrameIdx.
    CurrentLong(u32),
}

/// Lee la sintaxis `dec_ref_pic_marking` (UIT-T H.264 §7.3.3.3) para imágenes de referencia (`nal_ref_idc != 0`).
pub(crate) fn leer_marcado_referencia(
    r: &mut BitReader,
    is_idr: bool,
    mmco_ops: &mut Vec<Mmco>,
) -> Result<bool, DecodeError> {
    if is_idr {
        let _no_output_of_prior_pics = r.read_bit()?;
        let idr_long_term = r.read_bit()?; // long_term_reference_flag
        return Ok(idr_long_term);
    }
    if r.read_bit()? {
        // adaptive_ref_pic_marking_mode_flag
        loop {
            let op = r.read_ue()?;
            match op {
                0 => break,
                1 => mmco_ops.push(Mmco::Unref(r.read_ue()?)),
                2 => mmco_ops.push(Mmco::UnrefLong(r.read_ue()?)),
                3 => {
                    let diff = r.read_ue()?;
                    let idx = r.read_ue()?;
                    mmco_ops.push(Mmco::AssignLong(diff, idx));
                }
                4 => mmco_ops.push(Mmco::MaxLong(r.read_ue()?)),
                5 => mmco_ops.push(Mmco::Reset),
                6 => mmco_ops.push(Mmco::CurrentLong(r.read_ue()?)),
                _ => return Err(DecodeError::Unsupported("invalid MMCO")),
            }
            if mmco_ops.len() > MAX_OPERACIONES_MMCO {
                return Err(DecodeError::Truncated);
            }
        }
    }
    Ok(false)
}

impl Decoder {
    /// Moves the padded planes of retired (DPB-evicted) reference frames into
    /// the recycle pool. Called after the current picture's `FrameDecoder` is
    /// consumed, at which point a retired frame's `Arc` is normally unique; a
    /// frame something still holds (it shouldn't) is simply dropped un-recycled.
    pub(crate) fn reclaim_retired(&mut self) {
        for arc in self.retired.drain(..) {
            if let Ok(rf) = Arc::try_unwrap(arc) {
                if !rf.py.is_empty() {
                    self.plane_pool.push(rf.py);
                    self.plane_pool.push(rf.pu);
                    self.plane_pool.push(rf.pv);
                }
            }
        }
        // Bound the pool: 6 pictures' worth of planes (3 each) covers any
        // realistic ref churn; beyond that we'd just be hoarding memory.
        self.plane_pool.truncate(MAX_PLANOS_POOL);
    }

    /// Inserts "non-existing" short-term reference frames for each `frame_num`
    /// skipped since the previous reference picture (spec §8.2.5.2). Their samples
    /// are unspecified (a conformant stream never references them); we use mid-grey
    /// so any accidental reference is benign. They occupy DPB slots and advance the
    /// sliding window, keeping PicNum/ref-list derivation correct.
    pub(crate) fn insert_frame_num_gaps(
        &mut self,
        frame_num: u32,
        max_fn: u32,
        max_refs: usize,
        w: usize,
        h: usize,
    ) {
        if max_fn == 0 {
            return;
        }
        let start = (self.prev_ref_frame_num + 1) % max_fn;
        let gap = (frame_num + max_fn - start) % max_fn;
        if gap == 0 {
            return;
        }
        // Each placeholder is inserted at the front then the DPB is truncated to
        // `max_refs`, so for a gap larger than that only the most recent `max_refs`
        // placeholders can survive. Materialise just those — a malformed stream can
        // declare a gap of MaxFrameNum-1 (up to 65535), and allocating that many
        // full frames would be a CPU/memory DoS.
        let cap = max_refs.max(1);
        let n = (gap as usize).min(cap);
        let (cw, ch) = (w, h);
        #[allow(clippy::cast_possible_truncation)]
        let mut expected = (frame_num + max_fn - n as u32) % max_fn;
        for _ in 0..n {
            self.refs.insert(
                0,
                Arc::new(RefFrame {
                    // Uniform grey: the padded plane of a uniform plane is itself.
                    py: vec![MUESTRA_NEUTRA_8BIT; (cw + 2 * LPAD) * (ch + 2 * LPAD)],
                    pu: vec![MUESTRA_NEUTRA_8BIT; (cw / 2 + 2 * CPAD) * (ch / 2 + 2 * CPAD)],
                    pv: vec![MUESTRA_NEUTRA_8BIT; (cw / 2 + 2 * CPAD) * (ch / 2 + 2 * CPAD)],
                    cw,
                    ch,
                    frame_num: expected,
                    poc: 0,
                    mv: Vec::new(),
                    ref_idx: Vec::new(),
                    mv1: Vec::new(),
                    ref_idx1: Vec::new(),
                    ref_poc: Vec::new(),
                    w4: 0,
                    long_term: false,
                    long_term_idx: 0,
                    pic_id: next_pic_id(),
                }),
            );
            while self.refs.len() > cap {
                if let Some(pos) = self.refs.iter().rposition(|r| !r.is_long_term()) {
                    self.refs.remove(pos);
                } else {
                    break;
                }
            }
            expected = (expected + 1) % max_fn;
        }
        self.prev_ref_frame_num = (frame_num + max_fn - 1) % max_fn;
    }

    /// Inserts the just-decoded picture into the DPB and marks references
    /// (spec §8.2.5). With no MMCO commands this is the sliding window (evict the
    /// oldest short-term reference past capacity); with MMCO it is adaptive
    /// marking, including long-term assignment.
    ///
    /// Takes `reference` BY VALUE and MOVES it into the DPB (the caller's local is
    /// dropped right after) — the old `&mut` + `insert(0, reference.clone())` cloned
    /// all three planes (~1.35 MB/frame) a second time, on top of `as_reference`'s
    /// necessary clone. Returns the picture's final `frame_num` (0 after MMCO 5) for
    /// the caller's gap-detection tracking, since `reference` is gone after the move.
    pub(crate) fn apply_ref_marking(
        &mut self,
        mut reference: RefFrame,
        ops: &[Mmco],
        frame_num: u32,
        log2_max_frame_num: u32,
        max_refs: usize,
    ) -> u32 {
        let max = 1i64 << log2_max_frame_num;
        let curr = i64::from(frame_num);
        let pic_num = |rf: &RefFrame| -> i64 {
            let f = i64::from(rf.frame_num);
            if f > curr {
                f - max
            } else {
                f
            }
        };

        if ops.is_empty() {
            // Sliding window: insert the current (short-term) picture, then evict
            // the oldest short-term reference while over capacity (long-term refs
            // are retained).
            let out_fn = reference.frame_num;
            self.refs.insert(0, Arc::new(reference));
            while self.refs.len() > max_refs {
                if let Some(pos) = self.refs.iter().rposition(|r| !r.long_term) {
                    // Park the evicted frame; its planes are reclaimed on the
                    // next picture boundary (see `reclaim_retired`).
                    let evicted = self.refs.remove(pos);
                    self.retired.push(evicted);
                } else {
                    break;
                }
            }
            return out_fn;
        }

        // Adaptive marking (MMCO), applied in order.
        for &op in ops {
            match op {
                Mmco::Unref(diff) => {
                    let target = curr - (i64::from(diff) + 1);
                    self.refs.retain(|r| r.long_term || pic_num(r) != target);
                }
                Mmco::UnrefLong(ltpn) => {
                    self.refs
                        .retain(|r| !(r.long_term && r.long_term_idx == ltpn));
                }
                Mmco::AssignLong(diff, idx) => {
                    let target = curr - (i64::from(diff) + 1);
                    self.refs
                        .retain(|r| !(r.long_term && r.long_term_idx == idx));
                    for r in &mut self.refs {
                        if !r.long_term && pic_num(r) == target {
                            // Rare op; make_mut only copies if a slice still holds it.
                            let r = Arc::make_mut(r);
                            r.long_term = true;
                            r.long_term_idx = idx;
                        }
                    }
                }
                Mmco::MaxLong(max_plus1) => {
                    self.refs
                        .retain(|r| !(r.long_term && r.long_term_idx + 1 > max_plus1));
                }
                Mmco::Reset => {
                    self.refs.clear();
                    reference.frame_num = 0;
                }
                Mmco::CurrentLong(idx) => {
                    self.refs
                        .retain(|r| !(r.long_term && r.long_term_idx == idx));
                    reference.long_term = true;
                    reference.long_term_idx = idx;
                }
            }
        }
        let out_fn = reference.frame_num;
        self.refs.insert(0, Arc::new(reference));
        // Safety net so a malformed marking stream can't grow the DPB unbounded.
        let cap = max_refs.max(TOPE_SEGURIDAD_DPB);
        if self.refs.len() > cap {
            self.refs.truncate(cap);
        }
        out_fn
    }
}
