//! Pure-Rust H.264 decoder — Constrained Baseline **+ B-slices + most of High
//! profile**, CAVLC and CABAC.
//!
//! Validated **bit-exact against Cisco's `h264dec`** on 35 of 35 clean streams
//! from openh264's conformance corpus; the CABAC paths were brought up
//! symbol-by-symbol against an instrumented openh264 oracle and are gated
//! **pixel-exact vs ffmpeg**. The reconstruction path is shared with the encoder
//! (via `rusty_h264-common`), so the two halves agree bit-for-bit by
//! construction.
//!
//! Decodes: full intra (`I_16x16`/`I_4x4`/`I_8x8`/`I_PCM`), inter
//! (`P_Skip`/16×16/16×8/8×16/`P_8x8`) with quarter-pel motion compensation,
//! B-slices (temporal + spatial direct, implicit + explicit weighted
//! prediction), the 8×8 transform and 8×8 intra prediction, scaling matrices,
//! in-loop deblocking, and a multi-reference DPB with POC reordering and MMCO.
//! CABAC covers I, P and B slices incl. High-profile 8×8 residual (not yet: `I_PCM`).
//!
//! This crate is `#![forbid(unsafe_code)]` and is **fuzzed to never panic or
//! hang** on malformed input — errors surface as [`DecodeError`].
//!
//! [`Decoder::decode_stream`] is the one-call entry point (frames in display
//! order); [`Decoder::decode`] is the streaming form (one picture per access
//! unit, in decode order — pair it with [`Decoder::last_poc`]).

extern crate alloc;

mod cabac;
pub mod cabac_residuo;
mod cabecera_corte;
pub mod constantes;
pub mod corte;
pub(crate) mod dpb;
pub mod edc;
pub mod error;
mod listas_ref;
mod mb16;
pub mod mb_cabac;
pub mod mb_cabac_slice;
pub mod mb_cavlc;
pub mod mb_deblock;
pub mod mb_dequant;
pub mod mb_inter;
pub mod mb_intra;
mod params;
pub mod particion;
mod poc;
pub mod ponderacion;
pub mod pool;
pub mod recorte;
pub mod reordenacion;

pub use cabecera_corte::{au_is_idr, split_access_units};
pub(crate) use dpb::{next_pic_id, Ref, RefFrame, CPAD, LPAD};
pub use error::{DecodeError, MbError};
pub use params::{Pps, Sps};
pub use particion::b_inter_shape;
pub use ponderacion::{ParametrosPonderacion, RegionParticion, WeightTable};
pub use pool::GridPool;
pub use recorte::{DimensionesFotograma, Recorte};
pub use reordenacion::{capacidad_dpb, OrdenPresentacionH264, ReordenacionH264, MAX_IMAGENES_DPB};

pub mod cabac_test {
    pub use crate::cabac::Cabac;
    pub use crate::mb_cabac::parse_cbp_cabac;
    pub use crate::mb_cabac::parse_mb_qp_delta_cabac;
    pub use crate::mb_cabac::parse_mb_type_b;
    pub use crate::mb_cabac::parse_ref_idx_cabac;
    pub use crate::particion::b_inter_shape;
}

use cabecera_corte::{flush_gop, leer_filtro_corte, parse_pred_weight_table, resolve_scaling};
use constantes::{MB_DIM, QP_MAX, QP_MIN};
use corte::FiltroCorte;
use dpb::{leer_marcado_referencia, Mmco};
use listas_ref::{
    build_ref_list_b, build_ref_list_p, parse_ref_pic_list_modification, ContextoListaB,
};
use mb16::{FrameDecoder, ParametrosCuadro};
use poc::PocState;
use rusty_h264_common::nal::{emulation_unprevent, split_annex_b};
use rusty_h264_common::{BitReader, NalUnitType, YuvFrame};

/// Desplazamiento en bits del campo `nal_ref_idc` en el primer octeto NAL (UIT-T H.264 §7.3.1).
const DESPLAZAMIENTO_NAL_REF_IDC: u8 = 5;

/// Máscara de 2 bits para extraer `nal_ref_idc` (UIT-T H.264 §7.3.1).
const MASCARA_NAL_REF_IDC: u8 = 3;

/// Valor máximo de `cabac_init_idc` en cortes P/B (UIT-T H.264 §7.4.3).
const MAX_CABAC_INIT_IDC: u32 = 2;

/// Identificador `profile_idc` del perfil Baseline / Constrained Baseline (UIT-T H.264 Anexo A.2.1).
const PERFIL_IDC_BASELINE: u8 = 66;

/// A picture being assembled from one or more slices (spec allows a picture to
/// be split into multiple slices). Finalized — deblocked, output, and entered
/// into the DPB — once all its macroblocks are decoded.
struct PendingPic {
    fd: mb16::FrameDecoder,
    frame_num: u32,
    poc: i32,
    poc_type: u32,
    next_mb: usize,
    total_mb: usize,
    slice_count: u16,
    /// PARCHE-CRUSTY (ARR-2): recorte completo (izquierda, derecha, arriba, abajo).
    crop: mb16::Recorte,
    max_refs: usize,
    log2_max_frame_num: u32,
    /// `false` for a non-reference picture (nal_ref_idc == 0): output it but do
    /// not enter it into the DPB.
    is_reference: bool,
    idr_long_term: bool,
    mmco_ops: Vec<Mmco>,
}

/// A Constrained Baseline H.264 decoder. Holds the most recent parameter sets
/// and the previous decoded picture (the inter reference) across calls.
#[derive(Default)]
pub struct Decoder {
    /// Active parameter sets, keyed by id — a stream may carry several and switch
    /// between them per slice (spec §7.3.2.1/.2).
    // 11.11: Arc'd so the per-slice "clone to end the map borrow" is a
    // refcount bump, not a struct copy (scaling lists included).
    pub(crate) sps: alloc::collections::BTreeMap<u32, alloc::sync::Arc<Sps>>,
    pub(crate) pps: alloc::collections::BTreeMap<u32, alloc::sync::Arc<Pps>>,
    /// Decoded-picture buffer (most-recent first); `ref_idx` indexes into this.
    pub(crate) refs: Vec<Ref>,
    /// The picture currently being assembled from its slices, if any.
    cur: Option<PendingPic>,
    /// Picture-order-count state (spec §8.2.1). Tracks the previous reference
    /// picture's MSB/LSB (type 0) and frame-num offset (types 1/2) so display
    /// order can be recovered — needed once B-pictures (out-of-order) land.
    pub(crate) poc: PocState,
    /// `PicOrderCnt` of the most recently returned picture (display-order key).
    pub(crate) last_poc: i32,
    pub(crate) last_has_mmco5: bool,
    pub(crate) last_frame_num: u32,
    /// `frame_num` of the previous short-term reference picture, for detecting
    /// gaps in `frame_num` (spec §8.2.5.2).
    pub(crate) prev_ref_frame_num: u32,
    /// Per-picture grid allocations, handed from the finished picture to the next
    /// one instead of being freed and re-allocated. See `mb16::GridPool`.
    grid_pool: GridPool,
    /// 11.11: slice-header scratch (reorder lists + MMCO ops), recycled.
    sc_reorder0: Vec<(u32, u32)>,
    sc_reorder1: Vec<(u32, u32)>,
    sc_mmco: Vec<Mmco>,
    /// Recycled padded-plane buffers from evicted reference frames, drawn by
    /// `as_reference_pooled`. Bounded (see `reclaim_retired`).
    pub(crate) plane_pool: Vec<Vec<u8>>,
    /// Reference frames evicted from the DPB whose planes have not been
    /// reclaimed yet. Reclamation must wait until the evicting picture's
    /// `FrameDecoder` is consumed — while it lives it still holds `Arc` clones
    /// of its ref lists, so `Arc::try_unwrap` would fail at eviction time.
    pub(crate) retired: Vec<Ref>,
}

impl Decoder {
    /// Creates a decoder with no parameter sets yet.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Decodes a complete Annex-B access unit, returning the reconstructed,
    /// cropped frame if the access unit contained a coded picture.
    ///
    /// # Errors
    /// Returns [`DecodeError`] if the bitstream is malformed, truncated, or uses an unsupported feature.
    pub fn decode(&mut self, annex_b: &[u8]) -> Result<Option<YuvFrame>, DecodeError> {
        let mut frame = None;
        // The Annex-B scan and the RBSP unescape are each a FULL byte-wise pass over
        // the stream, and neither was timed — they landed in the unnamed residue that
        // the anatomy measured at 23-30% of decode outside the MB bodies.
        let nals = split_annex_b(annex_b);
        for nal in nals {
            if nal.is_empty() {
                continue;
            }
            let nal_type = NalUnitType::from_id(nal[0]);
            let rbsp = emulation_unprevent(&nal[1..]);
            match nal_type {
                NalUnitType::Sps => {
                    let s = Sps::parse(&rbsp)?;
                    self.sps
                        .insert(s.seq_parameter_set_id, alloc::sync::Arc::new(s));
                }
                NalUnitType::Pps => {
                    let p = Pps::parse(&rbsp)?;
                    self.pps
                        .insert(p.pic_parameter_set_id, alloc::sync::Arc::new(p));
                }
                NalUnitType::IdrSlice | NalUnitType::NonIdrSlice => {
                    let nal_ref_idc = (nal[0] >> DESPLAZAMIENTO_NAL_REF_IDC) & MASCARA_NAL_REF_IDC;
                    let is_idr = nal_type == NalUnitType::IdrSlice;
                    if let Some(f) = self.decode_slice(&rbsp, is_idr, nal_ref_idc)? {
                        frame = Some(f);
                    }
                }
                _ => {} // SEI, AUD, etc. ignored
            }
        }
        Ok(frame)
    }

    /// Decodes a complete Annex-B byte stream and returns every picture in
    /// **display order** (`PicOrderCnt` within each GOP; an IDR ends a GOP).
    ///
    /// This is the convenient whole-stream entry point — it handles access-unit
    /// splitting, multi-slice picture assembly, and B-picture reordering — versus
    /// the lower-level per-access-unit [`Decoder::decode`], which returns pictures
    /// in decode order.
    ///
    /// # Errors
    /// Returns [`DecodeError`] if any access unit in the stream fails to decode.
    pub fn decode_stream(&mut self, annex_b: &[u8]) -> Result<Vec<YuvFrame>, DecodeError> {
        let mut out = Vec::new();
        let mut gop: Vec<(i32, YuvFrame)> = Vec::new();
        for au in split_access_units(annex_b) {
            if au_is_idr(au) {
                flush_gop(&mut gop, &mut out); // emit the prior GOP before the IDR
            }
            if let Some(frame) = self.decode(au)? {
                gop.push((self.last_poc, frame));
                if self.last_has_mmco5 {
                    flush_gop(&mut gop, &mut out);
                }
            }
        }
        flush_gop(&mut gop, &mut out);
        Ok(out)
    }

    /// The `PicOrderCnt` of the most recently returned picture. Pictures are
    /// returned in decode order; sorting them by this value yields display order
    /// (the only difference is reordered B-pictures).
    #[must_use]
    pub fn last_poc(&self) -> i32 {
        self.last_poc
    }

    #[must_use]
    pub fn last_has_mmco5(&self) -> bool {
        self.last_has_mmco5
    }

    #[must_use]
    pub fn last_frame_num(&self) -> u32 {
        self.last_frame_num
    }

    fn compute_poc(
        &mut self,
        sps: &Sps,
        is_idr: bool,
        nal_ref_idc: u8,
        frame_num: u32,
        poc_lsb: u32,
        deltas: (i32, i32),
    ) -> i32 {
        self.poc
            .compute_poc(sps, is_idr, nal_ref_idc, frame_num, poc_lsb, deltas)
    }

    fn decode_slice(
        &mut self,
        rbsp: &[u8],
        is_idr: bool,
        nal_ref_idc: u8,
    ) -> Result<Option<YuvFrame>, DecodeError> {
        let mut r = BitReader::new(rbsp);
        // --- slice_header ---
        let first_mb_in_slice = r.read_ue()? as usize;
        let slice_type = r.read_ue()?;
        let is_p = matches!(slice_type, 0 | 5);
        let is_b = matches!(slice_type, 1 | 6);
        let is_i = matches!(slice_type, 2 | 7);
        if !is_p && !is_b && !is_i {
            return Err(DecodeError::Unsupported("SP/SI slices"));
        }
        // Resolve the parameter sets this slice references (by id).
        let pic_parameter_set_id = r.read_ue()?;
        let pps = self
            .pps
            .get(&pic_parameter_set_id)
            .cloned()
            .ok_or(DecodeError::MissingParameterSet)?;
        let sps = self
            .sps
            .get(&pps.seq_parameter_set_id)
            .cloned()
            .ok_or(DecodeError::MissingParameterSet)?;
        let sps = &sps;
        let pps = &pps;
        // CABAC (entropy_coding_mode_flag=1) has an entirely different slice-data parse
        // (docs/cabac-decode-plan.md). I-slice CABAC is being brought up; the CABAC MB
        // loop gates P/B until Phase 3. `cabac_init_idc` (P/B only) is read below.
        let cabac = pps.entropy_coding_mode_flag;
        let frame_num = r.read_bits(sps.log2_max_frame_num)?;
        if is_idr {
            let _idr_pic_id = r.read_ue()?;
        }
        // pic_order_cnt fields (spec §7.3.3). `field_pic_flag` is always 0
        // (frame_mbs_only). Captured to derive PicOrderCnt for display ordering.
        let mut poc_lsb = 0u32;
        let mut delta_poc_bottom = 0i32;
        let mut delta_poc_0 = 0i32; // PARCHE-CRUSTY (ARR-1): delta_pic_order_cnt[0]
        if sps.pic_order_cnt_type == 0 {
            poc_lsb = r.read_bits(sps.log2_max_pic_order_cnt_lsb)?;
            if pps.bottom_field_pic_order_present {
                delta_poc_bottom = r.read_se()?;
            }
        } else if sps.pic_order_cnt_type == 1 && !sps.delta_pic_order_always_zero {
            delta_poc_0 = r.read_se()?;
            if pps.bottom_field_pic_order_present {
                delta_poc_bottom = r.read_se()?; // delta_pic_order_cnt[1]
            }
        }
        // PicOrderCnt is determined by the first slice of the picture; later
        // slices share it (and must not re-advance the POC state).
        let pic_poc = if first_mb_in_slice == 0 {
            self.compute_poc(
                sps,
                is_idr,
                nal_ref_idc,
                frame_num,
                poc_lsb,
                (delta_poc_0, delta_poc_bottom),
            )
        } else {
            // COR-3, pendiente de acuerdo: sin imagen en curso este 0 es provisional; el
            // corte se rechaza SIEMPRE más abajo («slice continues a missing picture») y
            // nunca da imagen. Adelantar el rechazo cambia el motivo en 6 flujos de la red.
            self.cur.as_ref().map_or(0, |p| p.poc)
        };
        // redundant_pic_cnt: a non-zero value marks a *redundant* coded picture
        // (an alternative representation of the primary picture). A primary
        // decoder discards it (spec §7.4.3, §8.2.5 note). Must be read here or the
        // rest of the slice header desyncs.
        if pps.redundant_pic_cnt_present_flag {
            let redundant_pic_cnt = r.read_ue()?;
            if redundant_pic_cnt != 0 {
                return Ok(None);
            }
        }
        // B slices choose direct-mode derivation here (spec §7.3.3).
        let direct_spatial = if is_b { r.read_bit()? } else { true };
        let mut num_ref_idx_l0 = pps.num_ref_idx_l0_default as usize;
        let mut num_ref_idx_l1 = pps.num_ref_idx_l1_default as usize;
        let mut reorder_l0: Vec<(u32, u32)> = core::mem::take(&mut self.sc_reorder0);
        reorder_l0.clear();
        let mut reorder_l1: Vec<(u32, u32)> = core::mem::take(&mut self.sc_reorder1);
        reorder_l1.clear();
        if is_p || is_b {
            // num_ref_idx_active_override_flag
            if r.read_bit()? {
                num_ref_idx_l0 = (r.read_ue()? + 1) as usize;
                if is_b {
                    num_ref_idx_l1 = (r.read_ue()? + 1) as usize;
                }
            }
            // ref_pic_list_modification_flag_l0
            if r.read_bit()? {
                parse_ref_pic_list_modification(&mut r, &mut reorder_l0)?;
            }
            if is_b && r.read_bit()? {
                // ref_pic_list_modification_flag_l1
                parse_ref_pic_list_modification(&mut r, &mut reorder_l1)?;
            }
        }
        // Explicit weighted prediction carries a pred_weight_table() here. P
        // (weighted_pred) uses single-list weights; B explicit bipred (idc 1) is
        // not yet wired into the bi-pred averaging, so refuse that. Implicit
        // bipred (idc 2) carries no table.
        let weights = if is_p && pps.weighted_pred {
            Some(parse_pred_weight_table(&mut r, num_ref_idx_l0, 0, false)?)
        } else if is_b && pps.weighted_bipred_idc == 1 {
            return Err(DecodeError::Unsupported("explicit B weighted prediction"));
        } else {
            None
        };
        // dec_ref_pic_marking (spec §7.3.3.3) — present only for reference
        // pictures (nal_ref_idc != 0). Reading it for a non-reference slice would
        // desync the rest of the header.
        let mut mmco_ops: Vec<Mmco> = core::mem::take(&mut self.sc_mmco);
        mmco_ops.clear();
        let idr_long_term = if nal_ref_idc == 0 {
            // non-reference picture: no marking syntax
            false
        } else {
            leer_marcado_referencia(&mut r, is_idr, &mut mmco_ops)?
        };
        // cabac_init_idc (spec §7.3.3) — CABAC context-model preset, P/B slices only.
        // Spec range [0,2]; a larger (corrupt) value would index the 4-model context-init
        // table out of bounds, so reject it here.
        let cabac_init_idc = if cabac && !is_i {
            let v = r.read_ue()?;
            if v > MAX_CABAC_INIT_IDC {
                return Err(DecodeError::Unsupported("invalid cabac_init_idc"));
            }
            v
        } else {
            0
        };
        let slice_qp_delta = r.read_se()?;
        // When deblocking_filter_control_present_flag is 0 the slice carries no
        // disable_deblocking_filter_idc and it is inferred 0 — i.e. the in-loop
        // filter is ON by default (spec §7.4.3). (Our own encoder always signals
        // the control explicitly, so this default was previously untested.)
        let filtro = if pps.deblocking_filter_control_present_flag {
            leer_filtro_corte(&mut r)?
        } else {
            FiltroCorte::POR_DEFECTO
        };
        let slice_qp =
            u8::try_from((pps.pic_init_qp + slice_qp_delta).clamp(QP_MIN, i32::from(QP_MAX)))
                .unwrap_or(0);

        // Synthesize placeholder short-term references for any gap in frame_num
        // (spec §8.2.5.2) so the DPB / PicNum mapping stays correct.
        if first_mb_in_slice == 0 && !is_idr && sps.gaps_in_frame_num_allowed {
            self.insert_frame_num_gaps(
                frame_num,
                1u32 << sps.log2_max_frame_num,
                sps.max_num_ref_frames.max(1) as usize,
                sps.pic_width_in_mbs * MB_DIM,
                sps.pic_height_in_mbs * MB_DIM,
            );
        }

        // Build the reference list(s) for this slice. P uses RefPicList0 only;
        // B uses RefPicList0 and RefPicList1 (POC-ordered).
        let max_fn = 1u32 << sps.log2_max_frame_num;
        let (ref_list0, ref_list1) = if is_b {
            build_ref_list_b(
                &self.refs,
                &ContextoListaB {
                    curr_poc: pic_poc,
                    curr_frame_num: frame_num,
                    max_frame_num: max_fn,
                    num0: num_ref_idx_l0,
                    num1: num_ref_idx_l1,
                    mods0: &reorder_l0,
                    mods1: &reorder_l1,
                },
            )?
        } else if is_p {
            (
                build_ref_list_p(&self.refs, frame_num, max_fn, num_ref_idx_l0, &reorder_l0)?,
                Vec::new(),
            )
        } else {
            (Vec::new(), Vec::new())
        };
        // Return the reorder scratch: capacity survives to the next slice.
        self.sc_reorder0 = reorder_l0;
        self.sc_reorder1 = reorder_l1;
        // --- picture assembly ---
        // first_mb_in_slice == 0 starts a new picture; otherwise this slice
        // continues the one in flight. An IDR clears the DPB at its first slice.
        if first_mb_in_slice == 0 {
            if is_idr {
                self.refs.clear();
            }
            // H-49: the CABAC macroblock loop never decodes `transform_size_8x8_flag`
            // — both reads of it sit on the CAVLC `BitReader`, and `decode_i8x8` only
            // accepts one. A PPS with `transform_8x8_mode_flag` set therefore desyncs
            // the arithmetic decoder within a few macroblocks, and the failure surfaces
            // as a bogus `CABAC I_PCM` far from its cause (the mb_type parse lands on
            // 25 out of garbage). Fail fast and accurately instead: a wrong error that
            // points at the wrong feature costs more than a missing feature does.
            let params = ParametrosCuadro {
                mb_w: sps.pic_width_in_mbs,
                mb_h: sps.pic_height_in_mbs,
                qp: slice_qp,
                chroma_qp_offset: pps.chroma_qp_index_offset,
                refs: ref_list0,
                num_ref_active: num_ref_idx_l0,
                constrained_intra: pps.constrained_intra_pred_flag,
                transform_8x8_mode: pps.transform_8x8_mode_flag,
                b_possible: sps.profile_idc != PERFIL_IDC_BASELINE, // b_possible: Baseline/Constrained Baseline (66) forbid B
            };
            let mut fd = FrameDecoder::with_pool(params, core::mem::take(&mut self.grid_pool));
            if is_b {
                fd.set_b_context(
                    ref_list1,
                    num_ref_idx_l1,
                    direct_spatial,
                    pic_poc,
                    pps.weighted_bipred_idc,
                    sps.direct_8x8_inference,
                );
            }
            if sps.has_scaling || pps.pic_scaling_matrix_present {
                let (s4, s8) = resolve_scaling(sps, pps);
                fd.set_scaling(s4, s8);
            }
            fd.set_transform_bypass(sps.transform_bypass);
            if let Some(w) = weights {
                fd.set_weights(w);
            }
            // A pending picture still here means the previous one never reached
            // total_mb and a new picture is now displacing it. That is a DECODER
            // desync, and dropping it silently is how the missing-B-slice-ref_idx
            // defect stayed hidden: the picture simply never entered the DPB, and
            // the failure surfaced hundreds of macroblocks later as "bitstream
            // truncated" from a reference-list modification asking for it. Refuse
            // to swallow it -- an incomplete picture must announce itself.
            if let Some(prev) = self.cur.take() {
                if prev.next_mb < prev.total_mb {
                    return Err(DecodeError::Truncated);
                }
            }
            self.cur = Some(PendingPic {
                fd,
                frame_num,
                poc: pic_poc,
                poc_type: sps.pic_order_cnt_type,
                next_mb: 0,
                total_mb: sps.pic_width_in_mbs * sps.pic_height_in_mbs,
                slice_count: 0,
                crop: mb16::Recorte {
                    izquierda: sps.frame_crop_left as usize,
                    derecha: sps.frame_crop_right as usize,
                    arriba: sps.frame_crop_top as usize,
                    abajo: sps.frame_crop_bottom as usize,
                },
                max_refs: sps.max_num_ref_frames.max(1) as usize,
                log2_max_frame_num: sps.log2_max_frame_num,
                is_reference: nal_ref_idc != 0,
                idr_long_term,
                mmco_ops,
            });
        } else {
            // Continuation slice: reset the per-slice QP + reference list.
            let Some(pic) = self.cur.as_mut() else {
                return Err(DecodeError::Unsupported(
                    "slice continues a missing picture",
                ));
            };
            // COR-3: el trabajo diferido del corte anterior se completa con SU estado.
            pic.fd.cerrar_corte()?;
            pic.fd.begin_slice(slice_qp, ref_list0, num_ref_idx_l0);
            if is_b {
                pic.fd.set_b_context(
                    ref_list1,
                    num_ref_idx_l1,
                    direct_spatial,
                    pic.poc,
                    pps.weighted_bipred_idc,
                    sps.direct_8x8_inference,
                );
            }
            if sps.has_scaling || pps.pic_scaling_matrix_present {
                let (s4, s8) = resolve_scaling(sps, pps);
                pic.fd.set_scaling(s4, s8);
            }
            pic.fd.set_transform_bypass(sps.transform_bypass);
            if let Some(w) = weights {
                pic.fd.set_weights(w);
            }
            // Latest slice's marking parameters win at finalization.
            pic.idr_long_term |= idr_long_term;
            // PARCHE-CRUSTY (ARR-1): todos los cortes de una imagen llevan el mismo
            // dec_ref_pic_marking (H.264 §7.4.3.3); acumularlos repetía cada MMCO una vez
            // por corte y un MMCO 3 repetido borraba la referencia a largo plazo recién creada.
            pic.mmco_ops.clear();
            pic.mmco_ops.extend(mmco_ops);
        }

        let pic = self.cur.as_mut().ok_or(DecodeError::Truncated)?;
        let first = first_mb_in_slice.min(pic.total_mb);
        let next = if cabac {
            // cabac_alignment_one_bit → the slice data is byte-aligned from here.
            r.align_to_byte().map_err(|_| DecodeError::Truncated)?;
            let (data, start) = (r.data(), r.bit_pos() / 8);
            pic.fd
                .decode_slice_data_cabac(crate::mb_cabac_slice::ArranqueCorte {
                    rbsp: data,
                    start_byte: start,
                    slice_qp,
                    cabac_init_idc,
                    is_i,
                    is_p,
                    first_mb: first,
                    filtro,
                })
        } else {
            pic.fd.decode_slice_data(&mut r, is_p, first, filtro)
        }?;
        pic.next_mb = next;
        pic.slice_count += 1;

        if pic.next_mb < pic.total_mb {
            return Ok(None); // picture not yet complete
        }

        // --- finalize the completed picture ---
        let pic = self.cur.take().ok_or(DecodeError::Truncated)?;
        self.finalizar_imagen(pic).map(Some)
    }

    /// Finaliza una imagen completada: vaciado diferido, filtro desbloqueante, marcado DPB y reciclaje.
    fn finalizar_imagen(&mut self, pic: PendingPic) -> Result<YuvFrame, DecodeError> {
        let PendingPic {
            mut fd,
            frame_num,
            poc,
            poc_type,
            crop,
            max_refs,
            log2_max_frame_num,
            is_reference,
            idr_long_term,
            mmco_ops,
            ..
        } = pic;
        let has_mmco5 = mmco_ops.iter().any(|&op| matches!(op, Mmco::Reset));
        let display_poc = poc;
        if has_mmco5 {
            self.poc.apply_mmco5(poc_type, poc);
            self.prev_ref_frame_num = 0;
        }
        self.last_poc = display_poc;
        self.last_has_mmco5 = has_mmco5;
        self.last_frame_num = frame_num;
        // PARCHE-CRUSTY (ARR-1): el vaciado no puede depender de que haya filtro.
        fd.flush_deferred()?;
        // Siempre: los macrobloques de un corte sin filtro tienen todos sus bS a 0 (COR-4).
        fd.deblock()?;
        // The necessary DPB plane clone (rec_y/u/v → RefFrame) — measured as its own
        // stage, OUTSIDE the Finalize scope so the two don't double-count.
        let reference = if is_reference {
            Some(fd.as_reference_pooled(&mut self.plane_pool))
        } else {
            None
        };
        if let Some(mut reference) = reference {
            reference.frame_num = frame_num;
            reference.poc = poc;
            if idr_long_term {
                reference.long_term = true;
                reference.long_term_idx = 0;
            }
            let reference = reference; // RefFrame from as_reference_pooled
            self.prev_ref_frame_num = self.apply_ref_marking(
                reference,
                &mmco_ops,
                frame_num,
                log2_max_frame_num,
                max_refs,
            );
        }
        self.sc_mmco = mmco_ops; // scratch back (capacity survives)
        let (frame, pool) = fd.into_frame_recycle(crop);
        self.grid_pool = pool;
        self.reclaim_retired();
        Ok(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ref_at(poc: i32, fnum: u32) -> Ref {
        alloc::sync::Arc::new(RefFrame {
            py: vec![],
            pu: vec![],
            pv: vec![],
            cw: 0,
            ch: 0,
            frame_num: fnum,
            poc,
            mv: Vec::new(),
            ref_idx: Vec::new(),
            mv1: Vec::new(),
            ref_idx1: Vec::new(),
            ref_poc: Vec::new(),
            w4: 0,
            long_term: false,
            long_term_idx: 0,
            pic_id: next_pic_id(),
        })
    }

    #[test]
    fn b_ref_lists_ordered_by_poc() {
        // Current POC 4; DPB has past (0,2) and future (6,8) references.
        let dpb = vec![ref_at(8, 4), ref_at(6, 3), ref_at(2, 1), ref_at(0, 0)];
        let (l0, l1) = build_ref_list_b(
            &dpb,
            &ContextoListaB {
                curr_poc: 4,
                curr_frame_num: 5,
                max_frame_num: 16,
                num0: 4,
                num1: 4,
                mods0: &[],
                mods1: &[],
            },
        )
        .unwrap();
        // List0: nearer past first (desc), then nearer future (asc).
        assert_eq!(
            l0.iter().map(|r| r.poc).collect::<Vec<_>>(),
            vec![2, 0, 6, 8]
        );
        // List1: nearer future first (asc), then nearer past (desc).
        assert_eq!(
            l1.iter().map(|r| r.poc).collect::<Vec<_>>(),
            vec![6, 8, 2, 0]
        );
    }

    #[test]
    fn b_ref_list1_swap_when_equal() {
        // Only past references -> List0 and List1 initialize identically, so
        // List1's first two entries are swapped (spec §8.2.4.2.3).
        let dpb = vec![ref_at(4, 2), ref_at(2, 1), ref_at(0, 0)];
        let (l0, l1) = build_ref_list_b(
            &dpb,
            &ContextoListaB {
                curr_poc: 6,
                curr_frame_num: 3,
                max_frame_num: 16,
                num0: 3,
                num1: 3,
                mods0: &[],
                mods1: &[],
            },
        )
        .unwrap();
        assert_eq!(l0.iter().map(|r| r.poc).collect::<Vec<_>>(), vec![4, 2, 0]);
        assert_eq!(l1.iter().map(|r| r.poc).collect::<Vec<_>>(), vec![2, 4, 0]);
    }

    #[test]
    fn frame_num_gaps_insert_placeholders() {
        let mut d = Decoder::new();
        d.prev_ref_frame_num = 2;
        // frame_num jumps 2 -> 5: placeholders for the skipped 3 and 4.
        d.insert_frame_num_gaps(5, 16, 8, 16, 16);
        let fns: Vec<u32> = d.refs.iter().map(|r| r.frame_num).collect();
        assert_eq!(fns, vec![4, 3], "most-recent placeholder at the front");
        assert_eq!(d.prev_ref_frame_num, 4);
        assert!(
            d.refs.iter().all(|r| r.py.iter().all(|&p| p == 128)),
            "grey fill"
        );
    }

    #[test]
    fn frame_num_gaps_wrap_and_noop() {
        // Wrap across MaxFrameNum: prev 14, frame_num 1 (max 16) -> fill 15, 0.
        let mut d = Decoder::new();
        d.prev_ref_frame_num = 14;
        d.insert_frame_num_gaps(1, 16, 8, 16, 16);
        assert_eq!(
            d.refs.iter().map(|r| r.frame_num).collect::<Vec<_>>(),
            vec![0, 15]
        );
        // No gap (consecutive) inserts nothing.
        let mut d = Decoder::new();
        d.prev_ref_frame_num = 3;
        d.insert_frame_num_gaps(4, 16, 8, 16, 16);
        assert!(d.refs.is_empty());
    }

    #[test]
    fn missing_param_sets_errors() {
        let mut d = Decoder::new();
        // A lone (fake) IDR slice header: first_mb_in_slice=0, slice_type=7 (I),
        // pic_parameter_set_id=0 — then the PPS lookup fails (none stored).
        let nal = rusty_h264_common::NalUnit::new(3, NalUnitType::IdrSlice, vec![0x88, 0x80]);
        let err = d.decode(&nal.to_annex_b()).unwrap_err();
        assert_eq!(err, DecodeError::MissingParameterSet);
    }
}
