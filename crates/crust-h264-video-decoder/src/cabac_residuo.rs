//! Decodificación de coeficientes residuales mediante CABAC (UIT-T H.264 §9.3.2.4 y §9.3.3).
//!
//! Este módulo implementa la decodificación de entropía de bloques residuales luma y croma:
//! mapas de significancia ([`SIG8X8`], [`LAST8X8`]), banderas de bloque codificado ([`RES_CBF`]),
//! y niveles de coeficientes mediante sufijos UEG0 y bypass.

use crate::cabac::{Ctx, Engine as Eng};
use crate::mb_cabac::cabac_exp_bypass;
pub use crust_h264_common::cabac_tables::NZC_CACHE;
use crust_h264_common::cabac_tables::{
    LAST8X8, RES_CBF, RES_MAP, RES_MAXC2, RES_MAXPOS, RES_ONE, SIG8X8,
};

// Índices de propiedades residuales (post GetMbResProperty, CABAC): índice de tabla de contextos.
pub const RP_I16_DC: usize = 1;
pub const RP_I16_AC: usize = 2;
pub const RP_LUMA_4X4: usize = 3;
pub const RP_CHROMA_DC: usize = 7; // U (V=8, mismos desplazamientos)
pub const RP_CHROMA_AC: usize = 9; // U (V=10, mismos desplazamientos)
pub const RP_LUMA_8X8: usize = 6;

/// Sufijo de nivel de coeficiente UEG0 (`DecodeUEGLevelCabac`): prefijo TU en `c` (<=13) y sufijo bypass EG0.
#[inline]
#[must_use]
pub fn cabac_ueg_level(e: &mut Eng, data: &[u8], ctx: &mut Ctx, c: usize) -> u32 {
    if e.decode_decision(data, ctx, c) == 0 {
        return 0;
    }
    let mut code = 0u32;
    let mut tmp;
    loop {
        tmp = e.decode_decision(data, ctx, c);
        code += 1;
        if tmp == 0 || code == 12 {
            break;
        }
    }
    if tmp != 0 {
        let (v, e2) = cabac_exp_bypass(*e, data, 0);
        *e = e2;
        code += v + 1;
    }
    code
}

/// Decodifica un bloque residual individual mediante CABAC (UIT-T H.264 §9.3.3),
/// genérico sobre la categoría CABAC (`RP`) y longitud de bloque (`N` = 4 / 16 / 64).
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
#[inline]
pub fn residual_block_eng<const RP: usize, const N: usize>(
    e: &mut Eng,
    data: &[u8],
    ctx: &mut Ctx,
    nzc: &mut [u8; 48],
    cbf_dc: &mut u16,
    iz: usize,
    plane: usize,
    is_intra: bool,
    ndc: (u16, u16),
    out: &mut [i32; N],
) -> u32 {
    const { assert!(N == 4 || N == 16 || N == 64) };

    // ---- coded_block_flag ----
    let is8 = RP == RP_LUMA_8X8;
    let is_dc = RP == RP_I16_DC || RP == RP_CHROMA_DC;
    let bit = RP + plane;
    let (mut na, mut nb) = (u8::from(is_intra), u8::from(is_intra));
    let scan = NZC_CACHE[iz.min(23)].clamp(8, 47);
    if is_dc {
        nb = ((ndc.0 >> bit) & 1) as u8;
        na = ((ndc.1 >> bit) & 1) as u8;
    } else {
        let (nbc, nac) = (nzc[scan - 8], nzc[scan - 1]);
        if nbc != 0xff {
            nb = u8::from(nbc != 0);
        }
        if nac != 0xff {
            na = u8::from(nac != 0);
        }
    }
    if !is8 {
        let cbf = e.decode_decision(data, ctx, 85 + RES_CBF[RP] + (na + (nb << 1)) as usize);
        if cbf == 0 {
            if !is_dc {
                nzc[scan] = 0;
            }
            return 0;
        }
        if is_dc {
            *cbf_dc |= 1 << bit;
        }
    }

    // ---- significance map ----
    #[allow(clippy::cast_sign_loss)]
    let maxpos = RES_MAXPOS[RP] as usize;
    debug_assert!(maxpos < N);
    let (map, last) = if is8 {
        (402, 417)
    } else {
        let m = RES_MAP[RP];
        (105 + m, 166 + m)
    };

    let mut sig = 0u64;
    let mut last_hit = false;
    for i in 0..maxpos {
        let (mi, li) = if is8 {
            (
                (SIG8X8[i & 63] & 15) as usize,
                (LAST8X8[i & 63] & 15) as usize,
            )
        } else {
            (i, i)
        };
        if e.decode_decision(data, ctx, map + mi) != 0 {
            sig |= 1u64 << i;
            if e.decode_decision(data, ctx, last + li) != 0 {
                last_hit = true;
                break;
            }
        }
    }
    if !last_hit {
        sig |= 1u64 << maxpos;
    }
    let coeff_num = sig.count_ones();

    // ---- levels ----
    let one = 227 + RES_ONE[RP];
    let abs = one + 5;
    #[allow(clippy::cast_sign_loss)]
    let maxc2 = RES_MAXC2[RP] as usize;
    let (mut c1, mut c2) = (1usize, 0usize);
    let mut m = sig;
    while m != 0 {
        let i = 63 - m.leading_zeros() as usize;
        m &= !(1u64 << i);
        #[allow(clippy::cast_possible_wrap)]
        let mut level = 1 + e.decode_decision(data, ctx, one + c1) as i32;
        if level == 2 {
            #[allow(clippy::cast_possible_wrap)]
            {
                level += cabac_ueg_level(e, data, ctx, abs + c2) as i32;
            }
            c2 = (c2 + 1).min(maxc2);
            c1 = 0;
        } else if c1 != 0 {
            c1 = (c1 + 1).min(4);
        }
        if e.decode_bypass_sign() != 0 {
            level = -level;
        }
        out[i & (N - 1)] = level;
    }
    #[allow(clippy::cast_possible_truncation)]
    let cn = coeff_num as u8;
    if is8 {
        for k in 0..4 {
            nzc[NZC_CACHE[(iz + k).min(23)].min(47)] = cn;
        }
    } else if !is_dc {
        nzc[scan] = cn;
    }
    coeff_num
}

/// Destino de escritura para la decodificación de bloques residuales de macrobloque.
pub struct ResidualOut<'a> {
    pub luma: &'a mut [[i32; 16]; 16],
    pub luma8: &'a mut [[i32; 64]; 4],
    pub cdc: &'a mut [[i32; 4]; 2],
    pub cac: &'a mut [[[i32; 16]; 4]; 2],
    /// Total de coeficientes por bloque indexado por `iz`; debe ser cero al entrar.
    pub nnzs: &'a mut [u8; 24],
}

/// Resuelve los indicadores de `coded_block_flags` de vecinos para componentes DC.
#[inline]
#[must_use]
pub fn resolve_ndc(ndc: (Option<u16>, Option<u16>), is_intra: bool) -> (u16, u16) {
    // Vecino no disponible (`None`): condTermFlagN = 1 si el macrobloque actual es intra y 0 si
    // es inter, UIT-T H.264 08/2024, §9.3.3.1.1.9.
    let d = if is_intra { 0xFFFF } else { 0 };
    (ndc.0.unwrap_or(d), ndc.1.unwrap_or(d))
}

/// Decodifica todo el residuo de un macrobloque no-I16 en una única pasada del motor aritmético.
#[allow(clippy::too_many_arguments, clippy::needless_pass_by_value)]
pub fn parse_mb_residual_cabac<const INTRA: bool>(
    cab: &mut crate::cabac::Cabac,
    nzc: &mut [u8; 48],
    cbf_dc: &mut u16,
    ndc: (Option<u16>, Option<u16>),
    cbp_luma: u32,
    cbp_chroma: u32,
    t8: bool,
    o: ResidualOut,
) {
    let is_intra = INTRA;
    let nd = resolve_ndc(ndc, INTRA);
    let (data, mut e, ctx) = cab.view();
    for id8 in 0..4usize {
        if cbp_luma & (1 << id8) != 0 {
            if t8 {
                o.luma8[id8] = [0i32; 64];
                #[allow(clippy::cast_possible_truncation)]
                let n = residual_block_eng::<RP_LUMA_8X8, 64>(
                    &mut e,
                    data,
                    ctx,
                    nzc,
                    cbf_dc,
                    id8 * 4,
                    0,
                    is_intra,
                    nd,
                    &mut o.luma8[id8],
                ) as u8;
                o.nnzs[id8 * 4..id8 * 4 + 4].fill(n);
            } else {
                o.luma[id8 * 4..id8 * 4 + 4].fill([0i32; 16]);
                for id4 in 0..4usize {
                    let iz = id8 * 4 + id4;
                    #[allow(clippy::cast_possible_truncation)]
                    let count = residual_block_eng::<RP_LUMA_4X4, 16>(
                        &mut e,
                        data,
                        ctx,
                        nzc,
                        cbf_dc,
                        iz,
                        0,
                        is_intra,
                        nd,
                        &mut o.luma[iz],
                    ) as u8;
                    o.nnzs[iz] = count;
                }
            }
        } else {
            for k in 0..4 {
                nzc[NZC_CACHE[(id8 * 4 + k).min(23)].min(47)] = 0;
            }
        }
    }
    if cbp_chroma >= 1 {
        *o.cdc = [[0i32; 4]; 2];
        for i in 0..2usize {
            residual_block_eng::<RP_CHROMA_DC, 4>(
                &mut e,
                data,
                ctx,
                nzc,
                cbf_dc,
                16 + i * 4,
                i,
                is_intra,
                nd,
                &mut o.cdc[i],
            );
        }
    }
    if cbp_chroma == 2 {
        *o.cac = [[[0i32; 16]; 4]; 2];
        for i in 0..2usize {
            for id4 in 0..4usize {
                #[allow(clippy::cast_possible_truncation)]
                let count = residual_block_eng::<RP_CHROMA_AC, 16>(
                    &mut e,
                    data,
                    ctx,
                    nzc,
                    cbf_dc,
                    16 + i * 4 + id4,
                    i,
                    is_intra,
                    nd,
                    &mut o.cac[i][id4],
                ) as u8;
                o.nnzs[16 + i * 4 + id4] = count;
            }
        }
    }
    cab.commit(e);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolve_ndc_intra_usa_mascara_completa_por_defecto() {
        let (a, b) = resolve_ndc((None, None), true);
        assert_eq!(a, 0xFFFF);
        assert_eq!(b, 0xFFFF);

        let (a2, b2) = resolve_ndc((Some(0x12), None), true);
        assert_eq!(a2, 0x12);
        assert_eq!(b2, 0xFFFF);
    }

    #[test]
    fn resolve_ndc_inter_usa_cero_por_defecto() {
        let (a, b) = resolve_ndc((None, None), false);
        assert_eq!(a, 0);
        assert_eq!(b, 0);

        let (a2, b2) = resolve_ndc((None, Some(0x34)), false);
        assert_eq!(a2, 0);
        assert_eq!(b2, 0x34);
    }
}
