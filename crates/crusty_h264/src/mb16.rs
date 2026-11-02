//! I_16x16 macroblock decoding — the mirror of the encoder's `mb16`.
//!
//! Parses each macroblock's residuals and reconstructs it with the exact same
//! prediction + inverse-transform helpers the encoder uses, so decoder output
//! matches encoder reconstruction bit-for-bit.
#![allow(clippy::needless_range_loop)]

#[allow(unused_imports)]
use alloc::borrow::ToOwned;
#[allow(unused_imports)]
use alloc::boxed::Box;
#[allow(unused_imports)]
use alloc::format;
#[allow(unused_imports)]
use alloc::string::{String, ToString};
#[allow(unused_imports)]
use alloc::vec;
#[allow(unused_imports)]
use crusty_h264_common::predict::{CHROMA_4X4_SCAN_XY, LUMA_4X4_SCAN_XY};
use crusty_h264_common::YuvFrame;

pub(crate) use crate::edc::*;

/// Reconstructed coded-size planes plus CAVLC `nnz` context grids.
pub struct FrameDecoder {
    pub(crate) mb_w: usize,
    pub(crate) mb_h: usize,
    /// Slice QP (`SliceQPy`) — the deblock filter's frame-level QP.
    pub(crate) qp: u8,
    /// Running luma QP (`QPy`), carried across macroblocks and stepped by each
    /// `mb_qp_delta` (spec §7.4.5). Equals `qp` on constant-QP streams.
    pub(crate) cur_qp: u8,
    /// PARCHE-CRUSTY (ARR-1): el último macrobloque CAVLC fue I_PCM. Su QPY para el filtro de
    /// desbloqueo es 0 (H.264 §8.7.2.2), pero no altera la predicción de QP del siguiente.
    pub(crate) pcm_qp_pending: bool,
    /// `chroma_qp_index_offset` from the active PPS (§8.5.8).
    pub(crate) chroma_qp_offset: i32,
    pub(crate) cw: usize,
    pub(crate) ch: usize,
    pub(crate) ccw: usize,
    pub(crate) rec_y: Vec<u8>,
    pub(crate) rec_u: Vec<u8>,
    pub(crate) rec_v: Vec<u8>,
    /// Per-macroblock luma QP (`QPy`), for per-edge deblock strength.
    pub(crate) mb_qp: Vec<u8>,
    /// First macroblock address of the slice currently being decoded. Neighbors
    /// with a lower address belong to an earlier slice and are "not available"
    /// for prediction (spec §8.3/§8.4). Slices are contiguous raster ranges (we
    /// reject FMO/slice-groups), so address ≥ this ⇔ same slice.
    pub(crate) slice_first_mb: usize,
    pub(crate) nnz_y: Vec<u8>,
    pub(crate) nnz_c: [Vec<u8>; 2],
    pub(crate) modes_y: Vec<u8>,
    pub(crate) coded_y: Vec<bool>,
    /// Per-4×4-block List-0 motion (mv + ref index, `-1` = no L0). For P slices
    /// this is the only motion; B slices add the List-1 grids below.
    pub(crate) mv_y: Vec<(i32, i32)>,
    pub(crate) inter_y: Vec<bool>,
    pub(crate) ref_idx_y: Vec<i32>,
    /// Per-4×4-block List-1 motion for B slices (`ref_idx1 = -1` = no L1).
    pub(crate) mv1: Vec<(i32, i32)>,
    pub(crate) ref_idx1: Vec<i32>,
    /// `RefPicList1` and B-slice flags (unused outside B slices).
    pub(crate) refs1: Vec<crate::Ref>,
    pub(crate) num_ref_active1: usize,
    pub(crate) is_b: bool,
    /// True if the stream's profile permits B-slices (`profile_idc != 66`). When
    /// false (Baseline / Constrained Baseline), `as_reference_pooled` skips the per-block
    /// motion (mv/ref_idx/ref_poc) that only B temporal/spatial direct ever reads.
    pub(crate) b_possible: bool,
    pub(crate) direct_spatial: bool,
    pub(crate) nnz_l_cache: [u8; 25],
    pub(crate) nnz_c_cache: [[u8; 9]; 2],
    /// Decoded-picture buffer (most-recent first); empty in I-slices. `ref_idx`
    /// indexes into this list.
    pub(crate) refs: Vec<crate::Ref>,
    /// `num_ref_idx_l0_active` for the current slice — drives whether `ref_idx`
    /// is coded (active > 1) and its te(v)/ue(v) form, independently of how many
    /// reference pictures actually exist (spec §7.4.5.1, §9.1).
    pub(crate) num_ref_active: usize,
    /// `constrained_intra_pred_flag`: when set, intra prediction may only use
    /// samples from intra-coded neighbors (inter neighbors are "not available").
    pub(crate) constrained_intra: bool,
    /// High-profile 4×4 scaling matrices in **raster** order, indexed by
    /// `[Y-intra, Cb-intra, Cr-intra, Y-inter, Cb-inter, Cr-inter]`. `None` = flat.
    pub(crate) scaling: Option<[[i32; 16]; 6]>,
    /// High-profile 8×8 luma scaling matrices in raster order `[Y-intra, Y-inter]`
    /// (4:2:0 has only these two). `None` = flat.
    pub(crate) scaling8: Option<[[i32; 64]; 2]>,
    /// `transform_8x8_mode_flag` from the PPS: enables `transform_size_8x8_flag`.
    pub(crate) transform_8x8_mode: bool,
    /// SPS `qpprime_y_zero_transform_bypass_flag` — refusal gate in `step_qp`.
    pub(crate) transform_bypass: bool,
    /// Los cortes de la imagen en orden de decodificación, uno por corte (ver [`DatosCorte`]).
    /// El desbloqueo consulta el del corte dueño de cada macrobloque.
    pub(crate) cortes: Vec<DatosCorte>,
    /// Desplazamientos del filtro de cada macrobloque (los de su corte), escritos al derivar su
    /// `bS` y leídos por el filtro común (§8.7.2).
    pub(crate) desplazamientos_mb: Vec<DesplazamientosFiltro>,
    /// Per-macroblock `transform_size_8x8_flag` (for deblocking: internal 4×4
    /// luma edges of 8×8-transform MBs are not filtered).
    pub(crate) mb_t8x8: Vec<bool>,
    // ---- Row-interleaved deblocking state (docs/row-interleave-plan.md) ----
    /// Per-MB boundary strengths, filled row-by-row as decode completes rows.
    pub(crate) bs_frame: Vec<crusty_h264_common::deblock::MbBs>,
    /// Rows whose bS is derived (watermark).
    pub(crate) bs_rows: usize,
    /// Rows already deblock-FILTERED (watermark; R3).
    pub(crate) flt_rows: usize,
    /// Two-row rolling window of packed records (prev = row r-1, cur = row r).
    pub(crate) pk_prev: Vec<crusty_h264_common::deblock::MbPack>,
    pub(crate) pk_cur: Vec<crusty_h264_common::deblock::MbPack>,
    /// Transform-block coded mask (nnz with the 8x8 OR applied), filled per row.
    pub(crate) nnz_dbr: Vec<u8>,
    /// Unfiltered bottom rows of the last-filtered MB row (intra reads these).
    pub(crate) bak_y: Vec<u8>,
    pub(crate) bak_u: Vec<u8>,
    pub(crate) bak_v: Vec<u8>,
    /// Entropy-decouple: deferred pixel jobs + the per-slice activation flag
    /// (CABAC slices only — the CAVLC loop has no flush hooks).
    pub(crate) edc_jobs: Vec<EdcJob>,
    /// Recycled job boxes (see `take_pinter_job`): a coded inter macroblock
    /// used to malloc, fill, memcpy and free a 2.8 KB box every time.
    #[allow(clippy::vec_box)]
    pub(crate) edc_job_pool: Vec<Box<PInterJob>>,
    /// Pool de reciclaje de cajas en heap para trabajos inter sin residuo (`PInterNoResJob`);
    /// evita la sobrecarga constante de asignación y liberación en el bucle caliente EDC.
    #[allow(clippy::vec_box)]
    pub(crate) edc_nores_pool: Vec<Box<PInterNoResJob>>,
    /// Coefficient planes for the B and intra arms (which reconstruct inline,
    /// so no job carries them): taken for a macroblock, zeroed per CODED block
    /// by the parse, put back after recon. `None` only while in use.
    pub(crate) scratch_luma: Option<Box<[[i32; 16]; 16]>>,
    pub(crate) scratch_luma8: Option<Box<[[i32; 64]; 4]>>,
    pub(crate) scratch_cac: Option<Box<[[[i32; 16]; 4]; 2]>>,
    pub(crate) edc_active: bool,
    /// D12: bits/MB carried in from previous slices (0 = not yet known).
    pub(crate) bits_per_mb: f64,
    /// Per-macroblock deblock derivation CLASS (`MB_KIND_*`), so the loop filter
    /// can skip the 24-block neighbourhood gather on macroblocks whose strengths
    /// are determined by syntax alone. Starts UNSET; anything left UNSET simply
    /// takes the blind path, so a missed producer site costs speed, not
    /// correctness. Only classes that are uniform BY SYNTAX are written — notably
    /// NOT `B_Skip`/`B_Direct`, whose direct-derived motion varies per 4×4.
    pub(crate) mb_kind: Vec<u8>,
    /// Explicit weighted-prediction tables, when active for this slice.
    pub(crate) weights: Option<WeightTable>,
    /// Colocated-picture fast-probe cache (set_b_context): true when refs1[0]
    /// is frozen (1T), short-term, with motion grids — col_zero then skips
    /// the Option + live + long_term + w4 branch chain per probe.
    pub(crate) col_ok: bool,
    pub(crate) col_w4: usize,
    /// True once ANY macroblock in this picture used the 8x8 transform. The
    /// `nnz_dbr` row copy + the per-row t8 scan in `derive_bs_row` exist ONLY
    /// to apply the 8x8 coded-status OR; with no t8 macroblock `nnz_dbr` is a
    /// byte-for-byte copy of `nnz_y`, so both are skipped and the derivation
    /// reads `nnz_y` directly. Census: t8mb=0 on every cavlc/main stream.
    pub(crate) any_t8: bool,
    /// Raster address whose P_Skip MV is FORCED (0,0): the previous
    /// decode_p_skip was at addr-1 and committed (0,0), so this MB's left
    /// neighbor either fires §8.4.1.1's zero-MV rule (in-slice: ref 0,
    /// (0,0)) or is unavailable (out-of-slice — same rule). Interior MBs of
    /// a skip run skip the 3-neighbor gather entirely.
    pub(crate) skip_zero_next: usize,
    /// Pending B_Skip grid+recon span: (mb row, x0, n, kind) of CONSECUTIVE
    /// fast B_Skips with IDENTICAL committed values, deferred and range-
    /// filled/band-reconned at flush. Flushed before ANY grid or pixel
    /// reader runs — see span_flush callers.
    pub(crate) bzspan: Option<(usize, usize, usize, BzKind)>,
    /// Pending P_Skip (0,0) grid-commit span — the P mirror of `bzspan`
    /// (values: ref 0 list-0, mv (0,0), inter, coded, DC mode, kind SKIP).
    /// The bool records whether the RECON is deferred too (1T, identity/no
    /// weights) — decided at push time, not re-derived at flush (begin_slice
    /// may have changed the fields in between).
    pub(crate) pzspan: Option<(usize, usize, usize, bool)>,
    /// Per-picture bitmap: MB decoded as a ref0/(0,0)-both-lists zero-bi fast
    /// B_Skip. Never cleared mid-picture — false only ever means "derive
    /// normally" — so no MB path carries a clearing duty.
    pub(crate) bzero: Vec<bool>,
    /// Pooled CABAC slice scratch — refilled (not reallocated) at slice entry;
    /// carried across pictures via `GridPool`. Taken out of `self` at
    /// `decode_slice_cabac_inner` entry and put back at its normal exit (an
    /// error exit simply forfeits the pooled allocation).
    pub(crate) sc_cat: Vec<u8>,
    pub(crate) sc_cbp: Vec<u8>,
    pub(crate) sc_cmode: Vec<i32>,
    pub(crate) sc_nzc: Vec<[u8; 24]>,
    pub(crate) sc_cbfdc: Vec<u16>,
    pub(crate) sc_skip: Vec<bool>,
    pub(crate) sc_ref: Vec<[i8; 16]>,
    pub(crate) sc_mvd: Vec<[[i16; 2]; 16]>,
    pub(crate) sc_ref1: Vec<[i8; 16]>,
    pub(crate) sc_mvd1: Vec<[[i16; 2]; 16]>,
    pub(crate) sc_direct: Vec<bool>,
    /// Lazily cached `implicit_weights(0, 0)` — slice-constant (POCs of
    /// refs[0]/refs1[0] and cur don't change within a slice). Reset in
    /// set_b_context.
    pub(crate) iw00: Option<Option<(i32, i32)>>,
    /// Cached `weights.es_identidad_lista(0)` — all list-0 refs identity; gates the
    /// whole weight_partition pass for P inter (one choke point, all sites).
    pub(crate) weights_l0id: bool,
    /// Cached `weights.es_identidad_ref0()` — identity tables (the x264 weightp=2
    /// common case) make every ref-0 weighting pass an exact no-op, and the
    /// skip fast paths check this per MB, so it is computed once per slice.
    pub(crate) weights_id0: bool,
    /// Current picture's `PicOrderCnt` (for temporal direct + implicit weighting).
    pub(crate) cur_poc: i32,
    /// `weighted_bipred_idc` (0 = none/average, 1 = explicit, 2 = implicit).
    pub(crate) weighted_bipred_idc: u8,
    /// `direct_8x8_inference_flag` (B direct co-located sub-block selection).
    pub(crate) direct_8x8_inference: bool,
    /// Slice-stable ref→POC tables for bS (≤16 entries). `derive_bs_row` used
    /// to `collect()` these every row — same values for the whole slice.
    pub(crate) ref_poc0: Vec<i32>,
    pub(crate) ref_poc1: Vec<i32>,
}

/// Explicit weighted-prediction tables (spec §7.4.3.2 / §8.4.2.3.2). Per
/// reference list, per ref index: a luma `(weight, offset)` and two chroma
pub use crate::ponderacion::WeightTable;

/// Recycled per-picture scratch grids.
///
/// `FrameDecoder::new` used to allocate ~1.65 MB of frame-wide grids for EVERY
/// coded picture and drop them when the picture finished. The sampled profiler
/// prices that (stage `dec-setup`) at 6.7% of decode — larger than dequant,
/// reconstruct and intra prediction combined, and none of it is codec work.
///
/// Two costs are being paid, and the allocation is the bigger one. A ~460 KB
/// `Vec` goes straight to the OS, so every page is a fresh zero page and the
/// decoder takes a soft page fault on FIRST TOUCH of each 4 KB — a cost charged
/// to whatever per-macroblock stage happens to touch it first, not to the
/// allocation. Handing the same buffers back keeps the pages mapped and warm.
///
/// The initialising fill is NOT skipped: these grids are read as neighbour
/// context (`modes_y` must read 2/DC, `ref_idx_y` must read -1) before every
/// block that writes them, so a stale value from the previous picture is a
/// correctness bug, not a performance trade. `clear()` + `resize()` keeps the
/// fill and drops only the allocation.
///
pub use crate::recorte::Recorte;

use crate::corte::{DatosCorte, FiltroCorte};
use crate::error::MbError;
use crate::pool::refill;
pub use crate::pool::GridPool;
use crusty_h264_common::deblock::DesplazamientosFiltro;

/// Parámetros de inicialización de un decodificador de cuadro ([`FrameDecoder`]).
pub struct ParametrosCuadro {
    /// Anchura del cuadro expresada en número de macrobloques (`pic_width_in_mbs`).
    pub mb_w: usize,
    /// Altura del cuadro expresada en número de macrobloques (`pic_height_in_mbs`).
    pub mb_h: usize,
    /// Parámetro de cuantización inicial de luminancia para el corte (`slice_qp`).
    pub qp: u8,
    /// Desplazamiento de cuantización de crominancia respecto al corte (`chroma_qp_index_offset`).
    pub chroma_qp_offset: i32,
    /// Lista de imágenes de referencia decodificadas disponibles para compensación de movimiento.
    pub refs: Vec<crate::Ref>,
    /// Cantidad de referencias activas en la Lista 0 (`num_ref_idx_l0_active`).
    pub num_ref_active: usize,
    /// Bandera `constrained_intra_pred_flag` (restringe predicción intra a muestras intra).
    pub constrained_intra: bool,
    /// Bandera `transform_8x8_mode_flag` del PPS (habilita transformada de 8×8).
    pub transform_8x8_mode: bool,
    /// Indica si el perfil del flujo admite cortes predictivos B (`profile_idc != 66`).
    pub b_possible: bool,
}

impl ParametrosCuadro {
    /// Parámetros auxiliares para pruebas unitarias con dimensiones fijadas a 1×1 macrobloque.
    #[cfg(test)]
    pub fn para_prueba(qp: u8, chroma_qp_offset: i32) -> Self {
        Self {
            mb_w: 1,
            mb_h: 1,
            qp,
            chroma_qp_offset,
            refs: Vec::new(),
            num_ref_active: 0,
            constrained_intra: false,
            transform_8x8_mode: false,
            b_possible: false,
        }
    }
}

impl FrameDecoder {
    /// Non-pooled ctor — tests only; production always enters via `with_pool`
    /// (`GridPool` recycling, see lib.rs).
    #[cfg(test)]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mb_w: usize,
        mb_h: usize,
        qp: u8,
        chroma_qp_offset: i32,
        refs: Vec<crate::Ref>,
        num_ref_active: usize,
        constrained_intra: bool,
        transform_8x8_mode: bool,
        b_possible: bool,
    ) -> Self {
        Self::with_pool(
            ParametrosCuadro {
                mb_w,
                mb_h,
                qp,
                chroma_qp_offset,
                refs,
                num_ref_active,
                constrained_intra,
                transform_8x8_mode,
                b_possible,
            },
            GridPool::default(),
        )
    }

    /// As `new`, but reusing a previous picture's grid allocations. See `GridPool`.
    pub fn with_pool(params: ParametrosCuadro, pool: GridPool) -> Self {
        let (mb_w, mb_h) = (params.mb_w, params.mb_h);
        let qp = params.qp;
        let chroma_qp_offset = params.chroma_qp_offset;
        let refs = params.refs;
        let num_ref_active = params.num_ref_active;
        let constrained_intra = params.constrained_intra;
        let transform_8x8_mode = params.transform_8x8_mode;
        let b_possible = params.b_possible;
        let (cw, ch) = (mb_w * 16, mb_h * 16);
        let (ccw, cch) = (cw / 2, ch / 2);
        let bits_per_mb = pool.bits_per_mb;
        let ref_poc0 = {
            let mut v = pool.ref_poc0;
            v.clear();
            v.extend(refs.iter().map(|f| f.pic_id() as i32));
            v
        };
        Self {
            mb_w,
            mb_h,
            qp,
            cur_qp: qp,
            pcm_qp_pending: false,
            chroma_qp_offset,
            cw,
            ch,
            ccw,
            rec_y: vec![0; cw * ch],
            rec_u: vec![0; ccw * cch],
            rec_v: vec![0; ccw * cch],
            mb_qp: refill(pool.mb_qp, mb_w * mb_h, qp),
            slice_first_mb: 0,
            nnz_y: refill(pool.nnz_y, (mb_w * 4) * (mb_h * 4), 0),
            nnz_c: [
                refill(pool.nnz_c0, (mb_w * 2) * (mb_h * 2), 0),
                refill(pool.nnz_c1, (mb_w * 2) * (mb_h * 2), 0),
            ],
            modes_y: refill(pool.modes_y, (mb_w * 4) * (mb_h * 4), 2),
            coded_y: refill(pool.coded_y, (mb_w * 4) * (mb_h * 4), false),
            mv_y: refill(pool.mv_y, (mb_w * 4) * (mb_h * 4), (0, 0)),
            inter_y: refill(pool.inter_y, (mb_w * 4) * (mb_h * 4), false),
            ref_idx_y: refill(pool.ref_idx_y, (mb_w * 4) * (mb_h * 4), -1),
            mv1: refill(pool.mv1, (mb_w * 4) * (mb_h * 4), (0, 0)),
            ref_idx1: refill(pool.ref_idx1, (mb_w * 4) * (mb_h * 4), -1),
            refs1: Vec::new(),
            num_ref_active1: 0,
            is_b: false,
            b_possible,
            direct_spatial: true,
            nnz_l_cache: [0x80; 25],
            nnz_c_cache: [[0x80; 9]; 2],
            refs,
            num_ref_active,
            constrained_intra,
            scaling: None,
            scaling8: None,
            transform_8x8_mode,
            transform_bypass: false,
            cortes: Vec::new(),
            desplazamientos_mb: refill(
                pool.desplazamientos_mb,
                mb_w * mb_h,
                DesplazamientosFiltro::CERO,
            ),
            mb_t8x8: refill(pool.mb_t8x8, mb_w * mb_h, false),
            bs_frame: refill(pool.bs_frame, mb_w * mb_h, Default::default()),
            bs_rows: 0,
            flt_rows: 0,
            pk_prev: {
                let mut v = pool.pk_prev;
                v.clear();
                v
            },
            pk_cur: {
                let mut v = pool.pk_cur;
                v.clear();
                v
            },
            nnz_dbr: refill(pool.nnz_dbr, (mb_w * 4) * (mb_h * 4), 0),
            bak_y: refill(pool.bak_y, cw, 0),
            bak_u: refill(pool.bak_u, ccw, 0),
            bak_v: refill(pool.bak_v, ccw, 0),
            edc_jobs: Vec::new(),
            edc_job_pool: pool.job_pool,
            edc_nores_pool: pool.nores_pool,
            scratch_luma: None,
            scratch_luma8: None,
            scratch_cac: None,
            edc_active: false,
            bits_per_mb,
            mb_kind: refill(
                pool.mb_kind,
                mb_w * mb_h,
                crusty_h264_common::deblock::MB_KIND_UNSET,
            ),
            weights: None,
            col_ok: false,
            col_w4: 0,
            any_t8: false,
            skip_zero_next: usize::MAX,
            bzero: refill(pool.bzero, mb_w * mb_h, false),
            sc_cat: pool.sc_cat,
            sc_cbp: pool.sc_cbp,
            sc_cmode: pool.sc_cmode,
            sc_nzc: pool.sc_nzc,
            sc_cbfdc: pool.sc_cbfdc,
            sc_skip: pool.sc_skip,
            sc_ref: pool.sc_ref,
            sc_mvd: pool.sc_mvd,
            sc_ref1: pool.sc_ref1,
            sc_mvd1: pool.sc_mvd1,
            sc_direct: pool.sc_direct,
            bzspan: None,
            pzspan: None,
            iw00: None,
            weights_id0: false,
            weights_l0id: false,
            cur_poc: 0,
            weighted_bipred_idc: 0,
            direct_8x8_inference: false,
            ref_poc0,
            ref_poc1: {
                let mut v = pool.ref_poc1;
                v.clear();
                v
            },
        }
    }

    fn refresh_ref_pocs(&mut self) {
        self.ref_poc0.clear();
        self.ref_poc0
            .extend(self.refs.iter().map(|f| f.pic_id() as i32));
        self.ref_poc1.clear();
        self.ref_poc1
            .extend(self.refs1.iter().map(|f| f.pic_id() as i32));
    }

    /// Sets the explicit weighted-prediction tables for this slice.
    pub fn set_weights(&mut self, weights: WeightTable) {
        self.weights_id0 = weights.es_identidad_ref0();
        self.weights_l0id = weights.es_identidad_lista(0);
        self.weights = Some(weights);
    }

    /// Applies explicit uni-prediction weighting to a motion-compensated partition
    /// (luma `pred_y` region + the two chroma planes), if weighting is active.
    /// `list` is the reference list and `refi` the partition's reference index.
    ///
    /// # Errors
    /// `MbError::Inconsistent` si `refi` no está en la tabla de pesos.
    pub(crate) fn weight_partition(
        &self,
        pred_y: &mut [u8; 256],
        c_pred: &mut [[u8; 64]; 2],
        list: usize,
        refi: usize,
        rect: (usize, usize, usize, usize),
    ) -> Result<(), MbError> {
        let (rx, ry, rw, rh) = rect;
        let Some(wt) = &self.weights else {
            return Ok(());
        };
        // Identity list-0 table (x264 weightp=2 outside fades): the whole pass
        // is an exact per-pixel no-op — skip it.
        if list == 0 && self.weights_l0id {
            return Ok(());
        }
        // REFUTED, reverted: row-slicing these loops measured +28% instructions
        // on `weight_partition` (the extents are runtime values, so the slice
        // bounds cost more than the per-sample checks they replaced). The real
        // win for this function was hoisting the identity test to its CALLERS,
        // which stands.
        // RESOLVE ONCE, APPLY MANY. `apply_luma` re-read `self.luma[list][refi]`
        // — an array index, a Vec deref and an element index, two of them bounds
        // checked — on EVERY sample, up to 256 luma plus 128 chroma per call.
        // The weight is a property of the partition, not of the pixel. (The loop
        // SHAPE is untouched: row-slicing it is refuted above.)
        // MASKED, not row-sliced. `pred_y` is a fixed `[u8; 256]` and `c_pred` a
        // `[[u8; 64]; 2]`, so `& 255` / `& 63` are no-ops that prove the index
        // outright — where SLICING these loops cost +28% (the extents are runtime
        // values, so the slice bounds outweigh the checks). Same lesson as
        // `luma_centre`: the refutation was of one SHAPE, not of the goal.
        let reg = crate::ponderacion::RegionParticion::nueva(rx, ry, rw, rh);
        wt.aplicar_particion(pred_y, c_pred, list, refi, reg)?;
        Ok(())
    }

    /// SPS `qpprime_y_zero_transform_bypass_flag` — see [`Self::step_qp`].
    pub fn set_transform_bypass(&mut self, on: bool) {
        self.transform_bypass = on;
    }

    /// Sets the High-profile scaling matrices (raster order: six 4×4 lists, two
    /// 8×8 luma lists). The caller un-zig-zags the SPS lists. Flat is the default.
    pub fn set_scaling(&mut self, scaling: [[i32; 16]; 6], scaling8: [[i32; 64]; 2]) {
        self.scaling = Some(scaling);
        self.scaling8 = Some(scaling8);
    }

    /// Sets the B-slice context for the slice about to be decoded: `RefPicList1`,
    /// its active count, and the direct-mode flag.
    #[allow(clippy::too_many_arguments)]
    pub fn set_b_context(
        &mut self,
        refs1: Vec<crate::Ref>,
        num_ref_active1: usize,
        direct_spatial: bool,
        cur_poc: i32,
        weighted_bipred_idc: u8,
        direct_8x8_inference: bool,
    ) {
        self.is_b = true;
        self.refs1 = refs1;
        self.num_ref_active1 = num_ref_active1;
        self.direct_spatial = direct_spatial;
        self.cur_poc = cur_poc;
        self.weighted_bipred_idc = weighted_bipred_idc;
        self.direct_8x8_inference = direct_8x8_inference;
        self.iw00 = None; // slice-scoped cache: lists/POC may have changed
                          // Colocated fast-probe cache (see col_zero / col_zero_fast).
        self.col_ok = self
            .refs1
            .first()
            .is_some_and(|c| !c.long_term && c.w4 != 0);
        self.col_w4 = self.refs1.first().map_or(0, |c| c.w4);
        self.refresh_ref_pocs();
    }

    /// Cached `implicit_weights(0, 0)` — constant within a slice.
    pub(crate) fn iw00(&mut self) -> Option<(i32, i32)> {
        // Read the discriminant ONCE. The `is_none()` + `unwrap()` pair tested it
        // twice and carried an `unwrap_failed` path for a value this very function
        // had just written. `get_or_insert_with` cannot be used here: the closure
        // would need `&mut self` while the `Option` is already borrowed.
        match self.iw00 {
            Some(v) => v,
            None => {
                let v = if self.refs.is_empty() || self.refs1.is_empty() {
                    None
                } else {
                    self.implicit_weights(0, 0)
                };
                self.iw00 = Some(v);
                v
            }
        }
    }

    /// Completa el trabajo diferido del corte que termina: los tramos de rejilla pendientes y
    /// las reconstrucciones EDC. Debe llamarse ANTES de sustituir el estado por corte (listas
    /// de referencias, pesos, escalado), porque ese trabajo lo lee al ejecutarse.
    ///
    /// # Errors
    /// Los de [`Self::span_flush`] y [`Self::edc_flush`].
    pub fn cerrar_corte(&mut self) -> Result<(), MbError> {
        self.span_flush()?;
        self.edc_flush()
    }

    /// Resets per-slice state before decoding a continuation slice of the same
    /// picture: the running QP (each slice carries its own `slice_qp`) and the
    /// reference list (each slice may reorder it).
    pub fn begin_slice(&mut self, slice_qp: u8, refs: Vec<crate::Ref>, num_ref_active: usize) {
        self.cur_qp = slice_qp;
        self.qp = slice_qp;
        self.refs = refs;
        self.num_ref_active = num_ref_active;
        self.weights = None; // re-set per slice if a pred_weight_table is present
        self.weights_id0 = false;
        self.weights_l0id = false;
        self.refresh_ref_pocs();
    }

    /// Whether the neighbor macroblock at `(nbx, nby)` is in the slice currently
    /// being decoded (address ≥ the slice's first MB). For single-slice pictures
    /// `slice_first_mb == 0`, so this is always true and prediction is unchanged.
    #[inline]
    pub(crate) fn nbr_in_slice(&self, nbx: usize, nby: usize) -> bool {
        nby * self.mb_w + nbx >= self.slice_first_mb
    }

    /// Whether the neighbor 4×4 block at `(nbx, nby)` may contribute to intra
    /// prediction. With `constrained_intra_pred`, an inter-coded neighbor is
    /// treated as unavailable (spec §8.3.1.2.{1,2}); otherwise always usable.
    #[inline]
    ///
    /// # Errors
    /// `MbError::Inconsistent` si `inter_y` no cubre el vecino.
    pub(crate) fn intra_nbr_ok(&self, nbx: usize, nby: usize) -> Result<bool, MbError> {
        // Fallible read: this is inlined at eleven sites across the intra paths,
        // and `constrained_intra` short-circuits it on nearly every stream, so
        // the check was pure tax on a value that is usually never loaded.
        // Un vecino fuera de rango no es «no disponible»: los llamadores ya
        // decidieron la disponibilidad, así que significa que la rejilla ya no corresponde al
        // flujo (COR-3).
        if !self.constrained_intra {
            return Ok(true);
        }
        self.inter_y
            .get(nby * (self.mb_w * 4) + nbx)
            .map(|&inter| !inter)
            .ok_or(MbError::Inconsistent("inter_y"))
    }

    /// Registra el corte que empieza en `primer_mb` con su `filtro` y los mapas de referencias
    /// vigentes (`ref_poc0`/`ref_poc1`, ya ajustados a las listas del corte). Lo llaman los dos
    /// bucles de corte (CAVLC y CABAC) antes de su primer macrobloque.
    pub(crate) fn registrar_corte(&mut self, primer_mb: usize, filtro: FiltroCorte) {
        self.cortes.push(DatosCorte {
            primer_mb,
            filtro,
            poc0: self.ref_poc0.clone(),
            poc1: self.ref_poc1.clone(),
        });
    }

    /// Top-neighbour LUMA pixel for intra prediction: reads the unfiltered
    /// backup row when the row above has already been deblock-filtered by the
    /// row-interleave (flt_rows gates it; 0 when the interleave is off, so
    /// this compiles to the plain read on the fallback path).
    #[inline]
    ///
    /// # Errors
    /// `MbError::Inconsistent` si la muestra cae fuera del plano: los llamadores ya
    /// comprobaron que el vecino está disponible, así que no es una muestra «no disponible».
    pub(crate) fn top_y_px(&self, py: usize, x: usize) -> Result<u8, MbError> {
        if py % 16 == 0 && self.flt_rows * 16 >= py {
            self.bak_y.get(x).copied()
        } else {
            self.rec_y.get((py - 1) * self.cw + x).copied()
        }
        .ok_or(MbError::Inconsistent("rec_y"))
    }

    /// Slice form of [`Self::top_y_px`] for the contiguous 16-wide I16 gather.
    #[inline]
    pub(crate) fn top_y_row(&self, py: usize, x: usize, n: usize) -> &[u8] {
        if py % 16 == 0 && self.flt_rows * 16 >= py {
            &self.bak_y[x..x + n]
        } else {
            &self.rec_y[(py - 1) * self.cw + x..][..n]
        }
    }

    /// Top-neighbour CHROMA pixel (plane `c`: 0 = U, 1 = V).
    #[inline]
    ///
    /// # Errors
    /// `MbError::Inconsistent` si la muestra cae fuera del plano (ver [`Self::top_y_px`]).
    pub(crate) fn top_c_px(&self, c: usize, cy: usize, x: usize) -> Result<u8, MbError> {
        if cy % 8 == 0 && self.flt_rows * 8 >= cy {
            let bak = if c == 0 { &self.bak_u } else { &self.bak_v };
            bak.get(x).copied()
        } else {
            let rec = if c == 0 { &self.rec_u } else { &self.rec_v };
            rec.get((cy - 1) * self.ccw + x).copied()
        }
        .ok_or(MbError::Inconsistent("rec_c"))
    }

    /// Slice form of [`Self::top_c_px`] for the 8-wide chroma gather.
    #[inline]
    pub(crate) fn top_c_row(&self, c: usize, cy: usize, x: usize, n: usize) -> &[u8] {
        if cy % 8 == 0 && self.flt_rows * 8 >= cy {
            if c == 0 {
                &self.bak_u[x..x + n]
            } else {
                &self.bak_v[x..x + n]
            }
        } else {
            let rec = if c == 0 { &self.rec_u } else { &self.rec_v };
            &rec[(cy - 1) * self.ccw + x..][..n]
        }
    }

    /// Snapshots the (deblocked) reconstruction as a reference picture, drawing
    /// its padded-plane allocations from `pool` (recycled
    /// planes of evicted DPB frames — see `Decoder::reclaim_retired`). ~1.9 MB of
    /// fresh allocation per reference picture otherwise (`dpb-clone` stage, 3-4%
    /// of decode, mostly first-touch page faults).
    pub fn as_reference_pooled(&self, pool: &mut Vec<Vec<u8>>) -> crate::RefFrame {
        // The per-block motion (mv/ref_idx/ref_poc) is read ONLY by B temporal/spatial
        // direct (`col.mv/ref_idx/ref_poc`, guarded on `w4 != 0` + `idx < len`). On
        // Baseline/Constrained-Baseline streams (no B) it's pure waste — skip the two
        // grid clones + the per-block ref_poc resolve/alloc. `w4 = 0` makes the B
        // readers no-op even on malformed input.
        let (mv, ref_idx, mv1, ref_idx1, ref_poc, w4) = if self.b_possible {
            (
                self.mv_y.clone(),
                self.ref_idx_y.clone(),
                self.mv1.clone(),
                self.ref_idx1.clone(),
                // Resolve each block's List-0 ref index to the referenced picture's
                // POC, so temporal direct can map it into the current list.
                // Via a tiny per-ref LUT: the per-block bounds + Option chain +
                // Ref pointer chase (57k blocks at 720p, once per reference
                // frame) becomes one table index. Identical output: LUT slots
                // past refs.len() hold MIN, exactly what .get() returned.
                {
                    let mut poc_lut = [i32::MIN; 32];
                    for (i, f) in self.refs.iter().take(32).enumerate() {
                        poc_lut[i & 31] = f.pic_poc();
                    }
                    self.ref_idx_y
                        .iter()
                        .map(|&r| {
                            if (0..32).contains(&r) {
                                poc_lut[r as usize]
                            } else {
                                i32::MIN
                            }
                        })
                        .collect()
                },
                self.mb_w * 4,
            )
        } else {
            (
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                0,
            )
        };
        // Pop an exact-size recycled buffer per plane; a miss falls back to a
        // fresh allocation inside `pad_plane_into`.
        let mut take = |len: usize| -> Vec<u8> {
            match pool.iter().position(|v| v.len() == len) {
                Some(i) => pool.swap_remove(i),
                None => Vec::new(),
            }
        };
        let (lpw, lph) = (self.cw + 2 * crate::LPAD, self.ch + 2 * crate::LPAD);
        let (cpw, cph) = (self.ccw + 2 * crate::CPAD, self.ch / 2 + 2 * crate::CPAD);
        crate::RefFrame {
            // Pad once here (ExpandPicture) instead of extracting a clamped tile
            // on every MC call — same copy class as the old plane clone.
            py: crusty_h264_common::inter::pad_plane_into(
                take(lpw * lph),
                &self.rec_y,
                self.cw,
                self.ch,
                crate::LPAD,
            ),
            pu: crusty_h264_common::inter::pad_plane_into(
                take(cpw * cph),
                &self.rec_u,
                self.ccw,
                self.ch / 2,
                crate::CPAD,
            ),
            pv: crusty_h264_common::inter::pad_plane_into(
                take(cpw * cph),
                &self.rec_v,
                self.ccw,
                self.ch / 2,
                crate::CPAD,
            ),
            cw: self.cw,
            ch: self.ch,
            frame_num: 0, // set by the caller (decode_slice knows frame_num)
            poc: 0,       // set by the caller
            mv,
            ref_idx,
            mv1,
            ref_idx1,
            ref_poc,
            w4,
            long_term: false,
            long_term_idx: 0,
            pic_id: crate::next_pic_id(),
        }
    }

    /// Flush the entropy-decouple job queue: replay every deferred pixel job
    /// in parse order. Called before any intra macroblock (its reconstruction
    /// reads neighbour PIXELS), before row filtering, at B-branch entry, at
    /// slice end, and at `deblock()` as a backstop.
    /// Per-macroblock guard: the B arm calls this once per B MB — the empty
    /// test inlines, the drain body stays outlined.
    /// A job box from the pool (or a fresh one when the pool is empty -- the
    /// first row of a picture, or MT mode where boxes cross to the worker).
    #[inline]
    pub(crate) fn take_pinter_job(&mut self) -> Box<PInterJob> {
        self.edc_job_pool
            .pop()
            .unwrap_or_else(|| Box::new(PInterJob::blank()))
    }

    #[inline]
    pub(crate) fn take_nores_job(&mut self, pj: PInterNoResJob) -> Box<PInterNoResJob> {
        match self.edc_nores_pool.pop() {
            Some(mut b) => {
                *b = pj;
                b
            }
            None => Box::new(pj),
        }
    }

    #[inline(always)]
    pub(crate) fn edc_flush(&mut self) -> Result<(), MbError> {
        if self.edc_jobs.is_empty() {
            return Ok(());
        }
        self.edc_flush_slow()?;
        Ok(())
    }

    fn edc_flush_slow(&mut self) -> Result<(), MbError> {
        let mut jobs = core::mem::take(&mut self.edc_jobs);
        // Band identity holds unweighted OR under identity weights (the x264
        // weightp=2 common case: a table in every P slice, identity outside fades).
        let band_ok = self.weights.is_none() || self.weights_id0;
        let mut i = 0usize;
        while i < jobs.len() {
            let j = &jobs[i];
            match j {
                EdcJob::Skip { mbx, mby, mv } => {
                    // mb_skip_run band coalescing: gather the maximal run of
                    // consecutive same-row (0,0)-skips and recon it as ONE
                    // band copy (weighted-P falls back — the copy identity
                    // only holds unweighted).
                    if *mv == (0, 0) && band_ok {
                        let (x0, y0) = (*mbx, *mby);
                        let mut n = 1usize;
                        while let Some(EdcJob::Skip {
                            mbx: nx,
                            mby: ny,
                            mv: nmv,
                        }) = jobs.get(i + n)
                        {
                            if *ny == y0 && *nx == x0 + n && *nmv == (0, 0) {
                                n += 1;
                            } else {
                                break;
                            }
                        }
                        self.recon_p_skip_band(x0, y0, n)?;
                        i += n;
                        continue;
                    }
                    self.recon_p_skip(*mbx, *mby, *mv)?;
                }
                EdcJob::Inter(job) => {
                    self.recon_p_inter(job)?;
                }
                EdcJob::InterNoRes(job) => {
                    // D9b: do NOT to_full() — that memset ~2.5 KB of zeros per MB
                    // then walked add_inter_residual's all-zero path. MC + plane copy.
                    self.recon_p_inter_nores(job)?;
                }
            }
            i += 1;
        }
        // RECYCLE the job boxes (was: drop = one `free` per coded inter MB).
        for j in jobs.drain(..) {
            match j {
                EdcJob::Inter(b) => self.edc_job_pool.push(b),
                EdcJob::InterNoRes(b) => self.edc_nores_pool.push(b),
                _ => {}
            }
        }
        // Hand the (now empty) Vec back so its allocation is reused.
        self.edc_jobs = jobs;
        Ok(())
    }

    /// Crops the reconstructed coded-size planes to the display window.
    /// `into_frame`, additionally handing the per-picture grids back for reuse by
    /// the next picture. See `GridPool` for why this is worth doing.
    pub fn into_frame_recycle(mut self, crop: Recorte) -> (YuvFrame, GridPool) {
        let [c0, c1] = core::mem::take(&mut self.nnz_c);
        let pool = GridPool {
            job_pool: core::mem::take(&mut self.edc_job_pool),
            nores_pool: core::mem::take(&mut self.edc_nores_pool),
            bits_per_mb: self.bits_per_mb,
            mb_qp: core::mem::take(&mut self.mb_qp),
            bs_frame: core::mem::take(&mut self.bs_frame),
            pk_prev: core::mem::take(&mut self.pk_prev),
            pk_cur: core::mem::take(&mut self.pk_cur),
            nnz_dbr: core::mem::take(&mut self.nnz_dbr),
            bak_y: core::mem::take(&mut self.bak_y),
            bak_u: core::mem::take(&mut self.bak_u),
            bak_v: core::mem::take(&mut self.bak_v),
            nnz_y: core::mem::take(&mut self.nnz_y),
            nnz_c0: c0,
            nnz_c1: c1,
            modes_y: core::mem::take(&mut self.modes_y),
            coded_y: core::mem::take(&mut self.coded_y),
            mv_y: core::mem::take(&mut self.mv_y),
            inter_y: core::mem::take(&mut self.inter_y),
            ref_idx_y: core::mem::take(&mut self.ref_idx_y),
            mv1: core::mem::take(&mut self.mv1),
            ref_idx1: core::mem::take(&mut self.ref_idx1),
            mb_t8x8: core::mem::take(&mut self.mb_t8x8),
            mb_kind: core::mem::take(&mut self.mb_kind),
            desplazamientos_mb: core::mem::take(&mut self.desplazamientos_mb),
            bzero: core::mem::take(&mut self.bzero),
            sc_cat: core::mem::take(&mut self.sc_cat),
            sc_cbp: core::mem::take(&mut self.sc_cbp),
            sc_cmode: core::mem::take(&mut self.sc_cmode),
            sc_nzc: core::mem::take(&mut self.sc_nzc),
            sc_cbfdc: core::mem::take(&mut self.sc_cbfdc),
            sc_skip: core::mem::take(&mut self.sc_skip),
            sc_ref: core::mem::take(&mut self.sc_ref),
            sc_mvd: core::mem::take(&mut self.sc_mvd),
            sc_ref1: core::mem::take(&mut self.sc_ref1),
            sc_mvd1: core::mem::take(&mut self.sc_mvd1),
            sc_direct: core::mem::take(&mut self.sc_direct),
            ref_poc0: core::mem::take(&mut self.ref_poc0),
            ref_poc1: core::mem::take(&mut self.ref_poc1),
        };
        (self.into_frame(crop), pool)
    }

    /// Aplica la ventana de recorte delegando en el módulo especializado `recorte.rs`.
    pub fn into_frame(self, crop: Recorte) -> YuvFrame {
        crop.aplicar(
            crate::recorte::DimensionesFotograma {
                ancho: self.cw,
                alto: self.ch,
            },
            self.rec_y,
            self.rec_u,
            self.rec_v,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crusty_h264_common::predict::chroma_qp;

    fn fd(qp: u8, offset: i32) -> FrameDecoder {
        let mut p = ParametrosCuadro::para_prueba(qp, offset);
        p.num_ref_active = 1;
        p.b_possible = true;
        FrameDecoder::with_pool(p, GridPool::default())
    }

    #[test]
    fn mb_qp_delta_accumulates_mod_52() {
        let mut d = fd(26, 0);
        assert_eq!(d.cur_qp, 26, "QPy starts at the slice QP");
        d.step_qp(4).unwrap();
        assert_eq!(d.cur_qp, 30); // 26 + 4
        d.step_qp(-10).unwrap();
        assert_eq!(d.cur_qp, 20); // carries from the previous MB, not the slice
                                  // Wrap-around: (20 + 40 + 52) % 52 = 112 % 52 = 8.
        d.step_qp(40).unwrap();
        assert_eq!(d.cur_qp, 8);
        // Negative wrap: (8 - 20 + 52) % 52 = 40.
        d.step_qp(-20).unwrap();
        assert_eq!(d.cur_qp, 40);
    }

    #[test]
    fn chroma_qp_index_offset_applied_and_clamped() {
        // Offset 0 reproduces the bare luma->chroma table (QP30 -> 29).
        assert_eq!(fd(0, 0).chroma_qp_for(30), 29);
        // Positive offset shifts the table lookup (QP30 + 2 -> table[2] = 31).
        assert_eq!(fd(0, 2).chroma_qp_for(30), 31);
        // The qPi index is clamped into 0..=51 before the lookup.
        assert_eq!(fd(0, -12).chroma_qp_for(5), chroma_qp(0));
        assert_eq!(fd(0, 99).chroma_qp_for(40), chroma_qp(51));
    }
}
