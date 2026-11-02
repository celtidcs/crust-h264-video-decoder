//! Análisis de subestructuras de cabecera de corte (`pred_weight_table`, matrices de escalado,
//! parámetros del filtro de bordes) y partición del flujo Annex B en unidades de acceso (AU).

use alloc::vec;
use alloc::vec::Vec;
use crust_h264_common::deblock::DesplazamientosFiltro;
use crust_h264_common::nal::split_annex_b;
use crust_h264_common::{BitReader, NalUnitType, YuvFrame};

use crate::constantes::{BLOQUE_8X8_COEFS, SUBBLOQUE_4X4_COEFS, ZIGZAG_8X8};
use crate::corte::{FiltroCorte, IdcFiltro};
use crate::error::DecodeError;
use crate::params::{
    Pps, Sps, DEFAULT_4X4_INTER, DEFAULT_4X4_INTRA, DEFAULT_8X8_INTER, DEFAULT_8X8_INTRA,
};
use crate::ponderacion::WeightTable;

/// Valor máximo permitido para `luma_log2_weight_denom` y `chroma_log2_weight_denom` (UIT-T H.264 §7.4.3.2).
const MAX_LOG2_PESO_DENOM: i32 = 7;

/// Peso plano por defecto en las matrices de cuantización (`16`, UIT-T H.264 §7.4.2.1.1.1).
const PESO_ESCALADO_PLANO: u8 = 16;

/// Número de listas de escalado 4×4 en croma 4:2:0 (3 intra + 3 inter).
const NUM_LISTAS_ESCALADO_4X4: usize = 6;

/// Número de listas de escalado 8×8 de luma en croma 4:2:0 (1 intra + 1 inter).
const NUM_LISTAS_ESCALADO_8X8: usize = 2;

/// Tabla de orden inverso de escaneo en zig-zag 4×4 (UIT-T H.264 §8.5.6, Tabla 8-13).
const ZIGZAG_4X4: [usize; SUBBLOQUE_4X4_COEFS] =
    [0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15];

/// Emits a GOP's buffered pictures in display order (sorted by `PicOrderCnt`).
pub(crate) fn flush_gop(gop: &mut Vec<(i32, YuvFrame)>, out: &mut Vec<YuvFrame>) {
    gop.sort_by_key(|(poc, _)| *poc);
    out.extend(gop.drain(..).map(|(_, f)| f));
}

/// Whether an access unit contains an IDR coded-slice NAL.
///
/// Public for harnesses that reimplement `decode_stream`'s display-order emit
/// with an early stop (e.g. correctness probes that only need the first N pictures).
#[must_use]
pub fn au_is_idr(au: &[u8]) -> bool {
    split_annex_b(au)
        .iter()
        .any(|n| !n.is_empty() && NalUnitType::from_id(n[0]) == NalUnitType::IdrSlice)
}

#[cfg(doc)]
use crate::Decoder;

/// Splits an Annex-B byte stream into access units, each ending after a VCL
/// (coded-slice) NAL with any preceding parameter-set/SEI NALs attached. Start
/// codes are preserved so each unit can be passed straight to [`Decoder::decode`].
///
/// Public because [`Decoder::decode`] takes ONE access unit: a caller that wants
/// decode-order pictures, or wants to drop each picture as it arrives instead of
/// accumulating the stream like [`Decoder::decode_stream`], needs this to feed it.
#[must_use]
pub fn split_access_units(stream: &[u8]) -> Vec<&[u8]> {
    // (offset of the start code, whether the NAL it begins is a VCL slice).
    let mut codes: Vec<(usize, bool)> = Vec::new();
    let mut i = 0;
    // `get(i..i + 3)` rather than `i + 3 <= len` plus three whole-buffer
    // indexes: the loop condition bounds `i` but LLVM re-checks each of the
    // three reads anyway, and this runs once per BYTE of the bitstream.
    while let Some(w) = stream.get(i..i + 3) {
        if w[0] == 0 && w[1] == 0 && w[2] == 1 {
            // Un código de inicio sin byte de cabecera detrás (fin del flujo) no abre unidad VCL.
            let is_vcl = stream.get(i + 3).is_some_and(|&b| {
                matches!(
                    NalUnitType::from_id(b),
                    NalUnitType::IdrSlice | NalUnitType::NonIdrSlice
                )
            });
            // Include a leading zero (4-byte start code) in the unit boundary.
            let sc = if i > 0 && stream.get(i - 1) == Some(&0) {
                i - 1
            } else {
                i
            };
            codes.push((sc, is_vcl));
            i += 3;
        } else {
            i += 1;
        }
    }
    if codes.is_empty() {
        return vec![stream];
    }
    let mut aus = Vec::new();
    let mut start = codes[0].0;
    for k in 0..codes.len() {
        if codes[k].1 {
            let end = codes.get(k + 1).map_or(stream.len(), |c| c.0);
            aus.push(&stream[start..end]);
            start = end;
        }
    }
    aus
}

/// Parses a `pred_weight_table()` (spec §7.3.3.2) for the active reference lists
/// (4:2:0 → chroma weights always present). List 1 is parsed only for B slices.
pub(crate) fn parse_pred_weight_table(
    r: &mut BitReader,
    num_l0: usize,
    num_l1: usize,
    is_b: bool,
) -> Result<WeightTable, DecodeError> {
    #[allow(clippy::cast_possible_wrap)]
    let luma_log2_denom = r.read_ue()? as i32;
    #[allow(clippy::cast_possible_wrap)]
    let chroma_log2_denom = r.read_ue()? as i32;
    // Spec §7.4.3.2 constrains both weight denoms to [0, 7]; a malformed stream can
    // carry any ue(v). Reject before `1 << denom` (which overflows for denom ≥ 31)
    // so a corrupt bitstream is rejected gracefully, never panics.
    if !(0..=MAX_LOG2_PESO_DENOM).contains(&luma_log2_denom)
        || !(0..=MAX_LOG2_PESO_DENOM).contains(&chroma_log2_denom)
    {
        return Err(DecodeError::Unsupported("invalid weight denom"));
    }
    let mut wt = WeightTable {
        luma_log2_denom,
        chroma_log2_denom,
        ..Default::default()
    };
    let lists: &[(usize, usize)] = if is_b {
        &[(0, num_l0), (1, num_l1)]
    } else {
        &[(0, num_l0)]
    };
    for &(list, n) in lists {
        let mut luma = Vec::with_capacity(n);
        let mut chroma = Vec::with_capacity(n);
        for _ in 0..n {
            let (mut lw, mut lo) = (1 << luma_log2_denom, 0);
            if r.read_bit()? {
                lw = r.read_se()?;
                lo = r.read_se()?;
            }
            luma.push((lw, lo));
            let mut ch = [(1 << chroma_log2_denom, 0); 2];
            if r.read_bit()? {
                for slot in &mut ch {
                    *slot = (r.read_se()?, r.read_se()?);
                }
            }
            chroma.push(ch);
        }
        wt.luma[list & 1] = luma;
        wt.chroma[list & 1] = chroma;
    }
    Ok(wt)
}

/// Resolves the effective scaling matrices for a slice from the SPS lists and
/// any PPS override (fall-back rule B), returning them un-zig-zagged to raster
/// order: six 4×4 lists [Y/Cb/Cr intra, Y/Cb/Cr inter] and two 8×8 luma lists
/// [Y-intra, Y-inter].
pub(crate) fn resolve_scaling(
    sps: &Sps,
    pps: &Pps,
) -> (
    [[i32; SUBBLOQUE_4X4_COEFS]; NUM_LISTAS_ESCALADO_4X4],
    [[i32; BLOQUE_8X8_COEFS]; NUM_LISTAS_ESCALADO_8X8],
) {
    // 8×8 frame zig-zag scan → raster index (spec Table 8-12).
    const ZZ8: [usize; BLOQUE_8X8_COEFS] = ZIGZAG_8X8;
    // Effective zig-zag lists: a PPS override (rule B) takes precedence; an absent
    // PPS list falls back to the SPS list (or the default / previous PPS list).
    let mut z4 = [[PESO_ESCALADO_PLANO; SUBBLOQUE_4X4_COEFS]; NUM_LISTAS_ESCALADO_4X4];
    for i in 0..NUM_LISTAS_ESCALADO_4X4 {
        z4[i] = if pps.pic_scaling_matrix_present {
            if pps.scaling_present_4x4[i] {
                pps.scaling_4x4[i]
            } else {
                match i {
                    0 if sps.has_scaling => sps.scaling_4x4[0],
                    0 => DEFAULT_4X4_INTRA,
                    3 if sps.has_scaling => sps.scaling_4x4[3],
                    3 => DEFAULT_4X4_INTER,
                    _ => z4[i - 1],
                }
            }
        } else {
            sps.scaling_4x4[i]
        };
    }
    let mut z8 = [[PESO_ESCALADO_PLANO; BLOQUE_8X8_COEFS]; NUM_LISTAS_ESCALADO_8X8];
    for (i, list) in z8.iter_mut().enumerate() {
        *list = if pps.pic_scaling_matrix_present {
            if pps.scaling_present_8x8[i] {
                pps.scaling_8x8[i]
            } else if sps.has_scaling {
                sps.scaling_8x8[i]
            } else if i == 0 {
                DEFAULT_8X8_INTRA
            } else {
                DEFAULT_8X8_INTER
            }
        } else {
            sps.scaling_8x8[i]
        };
    }
    let mut out4 = [[i32::from(PESO_ESCALADO_PLANO); SUBBLOQUE_4X4_COEFS]; NUM_LISTAS_ESCALADO_4X4];
    for (li, list) in out4.iter_mut().enumerate() {
        for k in 0..SUBBLOQUE_4X4_COEFS {
            list[ZIGZAG_4X4[k]] = i32::from(z4[li][k]);
        }
    }
    let mut out8 = [[i32::from(PESO_ESCALADO_PLANO); BLOQUE_8X8_COEFS]; NUM_LISTAS_ESCALADO_8X8];
    for (li, list) in out8.iter_mut().enumerate() {
        for k in 0..BLOQUE_8X8_COEFS {
            list[ZZ8[k]] = i32::from(z8[li][k]);
        }
    }
    (out4, out8)
}

/// Lee `disable_deblocking_filter_idc` y, si el filtro no queda desactivado,
/// `slice_alpha_c0_offset_div2` y `slice_beta_offset_div2` (§7.3.3), y los valida (§7.4.3).
/// Con idc 1 los desplazamientos no se transmiten y se infieren 0.
///
/// # Errors
/// `Truncated` si faltan bits; `Unsupported` si el idc no es 0, 1 ni 2 o un desplazamiento se
/// sale de `[-6, 6]` (los mismos casos que rechaza `FFmpeg`).
pub(crate) fn leer_filtro_corte(r: &mut BitReader) -> Result<FiltroCorte, DecodeError> {
    let idc = IdcFiltro::desde_sintaxis(r.read_ue()?).ok_or(DecodeError::Unsupported(
        "invalid disable_deblocking_filter_idc",
    ))?;
    if idc == IdcFiltro::Desactivado {
        return Ok(FiltroCorte {
            idc,
            desplazamientos: DesplazamientosFiltro::CERO,
        });
    }
    // FilterOffset = slice_*_offset_div2 × 2 (§7.4.3).
    let (alfa, beta) = (r.read_se()?, r.read_se()?);
    let desplazamientos = alfa
        .checked_mul(2)
        .zip(beta.checked_mul(2))
        .and_then(|(a, b)| DesplazamientosFiltro::nuevo(a, b))
        .ok_or(DecodeError::Unsupported(
            "deblocking filter offset out of range",
        ))?;
    Ok(FiltroCorte {
        idc,
        desplazamientos,
    })
}
