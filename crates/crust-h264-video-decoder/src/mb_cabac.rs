//! Decodificación de elementos de sintaxis de macrobloques mediante CABAC (UIT-T H.264 §9.3).
//!
//! Este módulo centraliza el análisis sintáctico de tipos de macrobloque (I, P y B),
//! modos de predicción intra (luma y croma), subparticiones, índices de referencia,
//! diferencias de vectores de movimiento (MVD), delta QP y patrones de bloques codificados
//! (CBP) conforme a las tablas y transiciones de estado de la norma UIT-T H.264.

use crate::cabac::{Ctx, Engine as Eng};
use crust_h264_common::bit_reader::OutOfData;
pub use crust_h264_common::cabac_tables::{CACHE30, G_SCAN4};
use crust_h264_common::BitReader;

/// Decodificación unaria de longitud limitada en el motor aritmético CABAC.
///
/// Utilizada principalmente para la lectura de la magnitud de `mb_qp_delta`.
#[must_use]
pub fn cabac_unary(e: &mut Eng, data: &[u8], ctx: &mut Ctx, c: usize, off: usize) -> u32 {
    if e.decode_decision(data, ctx, c) == 0 {
        return 0;
    }
    let mut sym = 0;
    loop {
        let bin = e.decode_decision(data, ctx, c + off);
        sym += 1;
        // Se limita la longitud unaria a 512 para evitar bucles infinitos en flujos truncados.
        if bin == 0 || sym >= 512 {
            break;
        }
    }
    sym
}

/// Decodificación Exp-Golomb de orden k en modo bypass (UIT-T H.264 §9.3.2.4 / `DecodeExpBypassCabac`).
///
/// Toma el motor por valor y lo devuelve para garantizar que los llamadores críticos mantengan
/// el estado en registros de CPU sin escapes de memoria en pila.
#[inline(never)]
#[must_use]
pub fn cabac_exp_bypass(mut e: Eng, data: &[u8], mut count: i32) -> (u32, Eng) {
    let mut sym = 0u32;
    loop {
        let c = e.decode_bypass(data);
        if c == 1 {
            sym += 1 << count;
            count += 1;
        }
        if c == 0 || count == 16 {
            break;
        }
    }
    let mut sym2 = 0u32;
    while count > 0 {
        count -= 1;
        if e.decode_bypass(data) != 0 {
            sym2 |= 1 << count;
        }
    }
    (sym + sym2, e)
}

/// Decodificación de `mb_qp_delta` mediante CABAC (UIT-T H.264 §9.3.2.7).
///
/// Utiliza el desplazamiento de contexto 60 (`ctxIdxOffset = 60`) y actualiza el último delta QP.
pub fn parse_mb_qp_delta_cabac(cab: &mut crate::cabac::Cabac, last_delta_qp: &mut i32) -> i32 {
    const O: usize = 60;
    let ctx_inc = usize::from(*last_delta_qp != 0);
    let mut qp_delta = 0;
    let (data, mut e, ctx) = cab.view();
    if e.decode_decision(data, ctx, O + ctx_inc) != 0 {
        let code = cabac_unary(&mut e, data, ctx, O + 2, 1) + 1;
        #[allow(clippy::cast_possible_wrap)]
        {
            qp_delta = ((code + 1) >> 1) as i32;
        }
        if code & 1 == 0 {
            qp_delta = -qp_delta;
        }
    }
    cab.commit(e);
    *last_delta_qp = qp_delta;
    qp_delta
}

/// Decodifica `sub_mb_type` para slices P mediante CABAC (UIT-T H.264 §9.3.2.5, base ctx 21).
///
/// Valores de retorno: 0 = 8×8, 1 = 8×4, 2 = 4×8, 3 = 4×4.
pub fn parse_sub_mb_type_p_cabac(cab: &mut crate::cabac::Cabac) -> u32 {
    const S: usize = 21;
    let (data, mut e, ctx) = cab.view();
    let v = if e.decode_decision(data, ctx, S) != 0 {
        0
    } else if e.decode_decision(data, ctx, S + 1) != 0 {
        3 - e.decode_decision(data, ctx, S + 2)
    } else {
        1
    };
    cab.commit(e);
    v
}

/// Sub-análisis de sintaxis de `mb_type` intra para slices inter P/B (UIT-T H.264 §9.3.2.5).
///
/// Retorna: 0 = `I_4x4`, 1..=24 = `I_16x16`, 25 = `I_PCM`.
#[inline]
#[must_use]
pub fn intra_mb_type_eng(e: &mut Eng, data: &[u8], ctx: &mut Ctx, base: usize) -> u32 {
    if e.decode_decision(data, ctx, base) == 0 {
        return 0; // I_4x4
    }
    if e.decode_terminate(data) {
        return 25; // I_PCM
    }
    let mut t = 1 + 12 * e.decode_decision(data, ctx, base + 1); // cbp_luma != 0
    if e.decode_decision(data, ctx, base + 2) != 0 {
        t += 4 + 4 * e.decode_decision(data, ctx, base + 2);
    }
    t += 2 * e.decode_decision(data, ctx, base + 3);
    t += e.decode_decision(data, ctx, base + 3);
    t
}

/// Decodifica `mb_type` en slices B mediante CABAC (UIT-T H.264 Tabla 7-14).
///
/// Expuesto de forma pública para permitir la validación directa frente al codificador.
#[doc(hidden)]
pub fn parse_mb_type_b(cab: &mut crate::cabac::Cabac, ctx_inc: usize) -> u32 {
    parse_mb_type_b_cabac(cab, ctx_inc)
}

/// Decodifica `mb_type` en slices B mediante CABAC con contexto base 27.
#[inline]
pub fn parse_mb_type_b_cabac(cab: &mut crate::cabac::Cabac, ctx_inc: usize) -> u32 {
    const B: usize = 27;
    let (data, mut e, ctx) = cab.view();
    let v = mb_type_b_eng(&mut e, data, ctx, B, ctx_inc);
    cab.commit(e);
    v
}

/// Motor de decisiones CABAC para `mb_type` en slices B.
#[inline]
#[must_use]
pub fn mb_type_b_eng(e: &mut Eng, data: &[u8], ctx: &mut Ctx, b: usize, ctx_inc: usize) -> u32 {
    if e.decode_decision(data, ctx, b + ctx_inc) == 0 {
        return 0; // B_Direct_16x16
    }
    if e.decode_decision(data, ctx, b + 3) == 0 {
        return 1 + e.decode_decision(data, ctx, b + 5); // 16x16 L0 / L1
    }
    let mut m = e.decode_decision(data, ctx, b + 4) << 3;
    m |= e.decode_decision(data, ctx, b + 5) << 2;
    m |= e.decode_decision(data, ctx, b + 5) << 1;
    m |= e.decode_decision(data, ctx, b + 5);
    if m < 8 {
        return m + 3;
    }
    if m == 13 {
        return intra_mb_type_eng(e, data, ctx, 32) + 23;
    }
    if m == 14 {
        return 11; // B_Bi_8x16
    }
    if m == 15 {
        return 22; // B_8x8
    }
    m = (m << 1) | e.decode_decision(data, ctx, b + 5);
    m - 4
}

/// Decodifica `sub_mb_type` en slices B mediante CABAC (UIT-T H.264 Tabla 7-18, base ctx 36).
///
/// Retorna 0..=12 (0 = `B_Direct_8x8`, 1 = `B_L0_8x8`, ..., 12 = `B_Bi_4x4`).
pub fn parse_sub_mb_type_b_cabac(cab: &mut crate::cabac::Cabac) -> u32 {
    const B: usize = 36;
    let (data, mut e, ctx) = cab.view();
    let v = 'v: {
        if e.decode_decision(data, ctx, B) == 0 {
            break 'v 0; // B_Direct_8x8
        }
        if e.decode_decision(data, ctx, B + 1) == 0 {
            break 'v 1 + e.decode_decision(data, ctx, B + 3); // B_L0_8x8 / B_L1_8x8
        }
        let mut st = 3u32;
        if e.decode_decision(data, ctx, B + 2) != 0 {
            if e.decode_decision(data, ctx, B + 3) != 0 {
                break 'v 11 + e.decode_decision(data, ctx, B + 3); // B_L1_4x4 / B_Bi_4x4
            }
            st += 4;
        }
        st += 2 * e.decode_decision(data, ctx, B + 3);
        st += e.decode_decision(data, ctx, B + 3);
        st
    };
    cab.commit(e);
    v
}

/// Decodifica la diferencia de vector de movimiento `mvd` (x, y) de una partición inter.
///
/// Actualiza la caché local de 30 elementos y el estado de vectores del macrobloque.
#[allow(clippy::too_many_arguments)]
pub fn parse_mvd_partition(
    cab: &mut crate::cabac::Cabac,
    part_idx: usize,
    zblocks: &[usize],
    mvdc: &mut [[i16; 2]; 30],
    refc: &mut [i8; 30],
    mmvd: &mut [[i16; 2]; 16],
    mref: &mut [i8; 16],
    ref_idx: i8,
) -> (i32, i32) {
    let s = CACHE30[part_idx & 15].clamp(6, 29);
    let (av_b, av_a) = (refc[s - 6] >= 0, refc[s - 1] >= 0);
    let (nb, na) = (mvdc[s - 6], mvdc[s - 1]);
    let ctx = |comp: usize| -> usize {
        let mut a = 0i32;
        if av_b {
            a += i32::from(nb[comp].unsigned_abs());
        }
        if av_a {
            a += i32::from(na[comp].unsigned_abs());
        }
        if a >= 3 {
            1 + usize::from(a > 32)
        } else {
            0
        }
    };
    let (cx, cy) = (ctx(0), ctx(1));
    let (data, mut e, ctx) = cab.view();
    let mvx = mvd_component(&mut e, data, ctx, 0, cx);
    let mvy = mvd_component(&mut e, data, ctx, 1, cy);
    cab.commit(e);
    let mvd_par = [mvx, mvy];
    if zblocks.len() == 16 {
        mmvd.fill(mvd_par);
        mref.fill(ref_idx);
        for r in [7usize, 13, 19, 25] {
            mvdc[r..r + 4].fill(mvd_par);
            refc[r..r + 4].fill(ref_idx);
        }
    } else {
        for &zb in zblocks {
            let (c, g) = (CACHE30[zb & 15], G_SCAN4[zb & 15]);
            mvdc[c] = mvd_par;
            refc[c] = ref_idx;
            mmvd[g] = mvd_par;
            mref[g] = ref_idx;
        }
    }
    (i32::from(mvx), i32::from(mvy))
}

/// Decodifica el índice de referencia `ref_idx` mediante CABAC (UIT-T H.264 §9.3.2.2).
pub fn parse_ref_idx_cabac(cab: &mut crate::cabac::Cabac, ctx0: usize) -> i8 {
    const B: usize = 54;
    let (data, mut e, ctx) = cab.view();
    let mut r = 0i8;
    let mut bin_idx = 0u32;
    while bin_idx < 32 {
        let c = match bin_idx {
            0 => ctx0,
            1 => 4,
            _ => 5,
        };
        if e.decode_decision(data, ctx, B + c) == 0 {
            break;
        }
        r += 1;
        bin_idx += 1;
    }
    cab.commit(e);
    r
}

/// Sufijo de magnitud UEG3 para diferencias de vector de movimiento MVD.
#[inline]
#[must_use]
pub fn decode_ueg_mv(e: &mut Eng, data: &[u8], ctx: &mut Ctx, base: usize) -> u32 {
    const P2C: [usize; 8] = [0, 1, 2, 3, 3, 3, 3, 3];
    if e.decode_decision(data, ctx, base) == 0 {
        return 0;
    }
    let mut code = 0u32;
    let mut count = 1usize;
    let mut tmp;
    loop {
        tmp = e.decode_decision(data, ctx, base + P2C[count & 7]);
        code += 1;
        count += 1;
        if tmp == 0 || count == 8 {
            break;
        }
    }
    if tmp != 0 {
        let (v, e2) = cabac_exp_bypass(*e, data, 3);
        *e = e2;
        code += v + 1;
    }
    code
}

/// Decodifica una componente de diferencia de vector de movimiento (MVD).
#[inline]
#[must_use]
pub fn mvd_component(e: &mut Eng, data: &[u8], ctx: &mut Ctx, comp: usize, ctx_inc: usize) -> i16 {
    let base = 40 + comp * 7;
    if e.decode_decision(data, ctx, base + ctx_inc) == 0 {
        return 0;
    }
    #[allow(clippy::cast_possible_truncation)]
    let mag = (decode_ueg_mv(e, data, ctx, base + 3) + 1) as i16;
    if e.decode_bypass(data) != 0 {
        -mag
    } else {
        mag
    }
}

/// Decodifica `mb_type` en slices P mediante CABAC (UIT-T H.264 §9.3.2.5).
#[inline]
#[must_use]
pub fn mb_type_p_eng(e: &mut Eng, data: &[u8], ctx: &mut Ctx) -> u32 {
    const S: usize = 11;
    'v: {
        if e.decode_decision(data, ctx, S + 3) == 0 {
            break 'v if e.decode_decision(data, ctx, S + 4) != 0 {
                if e.decode_decision(data, ctx, S + 6) != 0 {
                    1
                } else {
                    2
                }
            } else if e.decode_decision(data, ctx, S + 5) != 0 {
                3
            } else {
                0
            };
        }
        if e.decode_decision(data, ctx, S + 6) == 0 {
            break 'v 5; // I_4x4
        }
        if e.decode_terminate(data) {
            break 'v 30; // I_PCM
        }
        let mut t = 6 + e.decode_decision(data, ctx, S + 7) * 12;
        if e.decode_decision(data, ctx, S + 8) != 0 {
            t += 4;
            if e.decode_decision(data, ctx, S + 8) != 0 {
                t += 4;
            }
        }
        t += e.decode_decision(data, ctx, S + 9) << 1;
        t += e.decode_decision(data, ctx, S + 9);
        t
    }
}

/// Decodifica `mb_type` en slices I mediante CABAC (UIT-T H.264 §9.3.2.5).
pub fn parse_mb_type_i_cabac(cab: &mut crate::cabac::Cabac, ctx_inc: usize) -> u32 {
    const O: usize = 3;
    let (data, mut e, ctx) = cab.view();
    let v = 'v: {
        if e.decode_decision(data, ctx, O + ctx_inc) == 0 {
            break 'v 0; // I_NxN
        }
        if e.decode_terminate(data) {
            break 'v 25; // I_PCM
        }
        let mut t = 1 + e.decode_decision(data, ctx, O + 3) * 12;
        if e.decode_decision(data, ctx, O + 4) != 0 {
            t += 4;
            if e.decode_decision(data, ctx, O + 5) != 0 {
                t += 4;
            }
        }
        t += e.decode_decision(data, ctx, O + 6) << 1;
        t += e.decode_decision(data, ctx, O + 7);
        t
    };
    cab.commit(e);
    v
}

/// Decodifica el modo de predicción intra 4×4 u 8×8 sobre la vista actual del motor.
#[inline]
#[must_use]
pub fn intra4x4_pred_mode_eng(e: &mut Eng, data: &[u8], ctx: &mut Ctx) -> i32 {
    const IPR: usize = 68;
    if e.decode_decision(data, ctx, IPR) == 1 {
        return -1;
    }
    #[allow(clippy::cast_possible_wrap)]
    {
        let mut m = e.decode_decision(data, ctx, IPR + 1) as i32;
        m |= (e.decode_decision(data, ctx, IPR + 1) as i32) << 1;
        m |= (e.decode_decision(data, ctx, IPR + 1) as i32) << 2;
        m
    }
}

/// Decodifica el modo de predicción intra de croma mediante CABAC (UIT-T H.264 §9.3.2.2).
pub fn parse_intra_chroma_pred_mode_cabac(cab: &mut crate::cabac::Cabac, ctx_inc: usize) -> u32 {
    const CIPR: usize = 64;
    if cab.decode_decision(CIPR + ctx_inc) == 0 {
        return 0;
    }
    if cab.decode_decision(CIPR + 3) == 0 {
        return 1;
    }
    if cab.decode_decision(CIPR + 3) == 0 {
        return 2;
    }
    3
}

/// Decodifica el patrón de bloques codificados `coded_block_pattern` (CBP) mediante CABAC.
pub fn parse_cbp_cabac(cab: &mut crate::cabac::Cabac, top: Option<u8>, left: Option<u8>) -> u32 {
    const CBP: usize = 73;
    // `None` = vecino no disponible: condTermFlagN = 0 tanto para los bins de luma
    // (ctxIdxOffset 73) como para los de croma (77), UIT-T H.264 08/2024, §9.3.3.1.1.4.
    let tz = top.map_or(0u32, |c| !u32::from(c) & 0xF);
    let lz = left.map_or(0u32, |c| !u32::from(c) & 0xF);
    let tc = top.map_or(0u32, |c| u32::from(c >> 4));
    let lc = left.map_or(0u32, |c| u32::from(c >> 4));
    let (data, mut e, ctx) = cab.view();
    let b0 = e.decode_decision(
        data,
        ctx,
        CBP + (((lz >> 1) & 1) + (((tz >> 2) & 1) << 1)) as usize,
    );
    let b1 = e.decode_decision(
        data,
        ctx,
        CBP + ((b0 ^ 1) + (((tz >> 3) & 1) << 1)) as usize,
    );
    let b2 = e.decode_decision(
        data,
        ctx,
        CBP + (((lz >> 3) & 1) + ((b0 ^ 1) << 1)) as usize,
    );
    let b3 = e.decode_decision(data, ctx, CBP + ((b2 ^ 1) + ((b1 ^ 1) << 1)) as usize);
    let mut cbp = b0 | (b1 << 1) | (b2 << 2) | (b3 << 3);
    let (ct, cl) = (u32::from(tc != 0), u32::from(lc != 0));
    if e.decode_decision(data, ctx, CBP + 4 + (cl + (ct << 1)) as usize) != 0 {
        let (ct2, cl2) = (u32::from(tc == 2), u32::from(lc == 2));
        let c1 = e.decode_decision(data, ctx, CBP + 8 + (cl2 + (ct2 << 1)) as usize);
        cbp |= 1 << (4 + c1);
    }
    cab.commit(e);
    cbp
}

/// Lee el índice de referencia `ref_idx` desde el flujo CAVLC / bitstream.
///
/// # Errors
/// Devuelve [`OutOfData`] si se agotan los bits del flujo antes de completar el valor.
pub fn read_ref_idx(r: &mut BitReader, num_ref_active: usize) -> Result<i32, OutOfData> {
    if num_ref_active == 2 {
        Ok(i32::from(!r.read_bit()?))
    } else {
        #[allow(clippy::cast_possible_wrap)]
        Ok(r.read_ue()? as i32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_ref_idx_con_dos_referencias_activas_invierte_bit() {
        // En te(v) con 2 referencias: bit 1 codifica 0, bit 0 codifica 1.
        let bytes = [0b1000_0000u8];
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_ref_idx(&mut r, 2).unwrap(), 0);

        let bytes_cero = [0b0000_0000u8];
        let mut r2 = BitReader::new(&bytes_cero);
        assert_eq!(read_ref_idx(&mut r2, 2).unwrap(), 1);
    }

    #[test]
    fn read_ref_idx_con_mas_de_dos_referencias_lee_ue() {
        // En ue(v): bit 1 -> 0; bits 010 -> 1; bits 011 -> 2
        let bytes = [0b1010_0110u8];
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_ref_idx(&mut r, 3).unwrap(), 0);
        assert_eq!(read_ref_idx(&mut r, 3).unwrap(), 1);
        assert_eq!(read_ref_idx(&mut r, 3).unwrap(), 2);
    }
}
