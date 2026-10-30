//! Construcción y modificación de listas de imágenes de referencia (`RefPicList0` y `RefPicList1`, UIT-T H.264 §8.2.4).
//!
//! Implementa la inicialización por `FrameNumWrap` para cortes P (§8.2.4.2.1), la inicialización
//! por `PicOrderCnt` para cortes B (§8.2.4.2.3) y el proceso `ref_pic_list_modification` (§8.2.4.3).

use alloc::boxed::Box;
use alloc::vec::Vec;
use rusty_h264_common::BitReader;

use crate::dpb::{Ref, RefFrame};
use crate::error::DecodeError;

/// Código de fin de la lista de modificaciones de índices de referencia (`modification_of_pic_nums_idc == 3`).
const IDC_MODIFICACION_FIN: u32 = 3;

/// Código de modificación mediante índice de referencia a largo plazo (`modification_of_pic_nums_idc == 2`).
const IDC_MODIFICACION_LARGO_PLAZO: u32 = 2;

/// Número máximo de comandos `ref_pic_list_modification` aceptados en una cabecera de corte.
const MAX_MODIFICACIONES_LISTA_REF: usize = 64;

/// Parámetros de construcción y modificación de las dos listas de referencia de un corte B.
pub(crate) struct ContextoListaB<'a> {
    pub curr_poc: i32,
    pub curr_frame_num: u32,
    pub max_frame_num: u32,
    pub num0: usize,
    pub num1: usize,
    pub mods0: &'a [(u32, u32)],
    pub mods1: &'a [(u32, u32)],
}

/// Parses a `ref_pic_list_modification` command list (spec §7.3.3.1) into
/// `(modification_of_pic_nums_idc, value)` pairs, stopping at idc 3.
pub(crate) fn parse_ref_pic_list_modification(
    r: &mut BitReader,
    out: &mut Vec<(u32, u32)>,
) -> Result<(), DecodeError> {
    loop {
        let idc = r.read_ue()?;
        if idc == IDC_MODIFICACION_FIN {
            break;
        }
        if idc > IDC_MODIFICACION_FIN {
            return Err(DecodeError::Unsupported(
                "invalid ref_pic_list_modification",
            ));
        }
        let val = r.read_ue()?; // abs_diff_pic_num_minus1 / long_term_pic_num
        out.push((idc, val));
        if out.len() > MAX_MODIFICACIONES_LISTA_REF {
            return Err(DecodeError::Truncated); // runaway / corrupt
        }
    }
    Ok(())
}

/// Builds the P-slice `RefPicList0`: short-term references ordered by descending
/// `FrameNumWrap`, then long-term by ascending idx (spec §8.2.4.2.1), with any
/// `ref_pic_list_modification` applied.
pub(crate) fn build_ref_list_p(
    dpb: &[Ref],
    curr_frame_num: u32,
    max_frame_num: u32,
    num_active: usize,
    mods: &[(u32, u32)],
) -> Result<Vec<Ref>, DecodeError> {
    let curr = i64::from(curr_frame_num);
    let max = i64::from(max_frame_num);
    let pic_num = |fnum: u32| -> i64 {
        let f = i64::from(fnum);
        if f > curr {
            f - max
        } else {
            f
        }
    };
    let mut init: Vec<Ref> = dpb.iter().filter(|r| !r.is_long_term()).cloned().collect();
    init.sort_by_key(|rf| core::cmp::Reverse(pic_num(rf.fn_num())));
    let mut long: Vec<Ref> = dpb.iter().filter(|r| r.is_long_term()).cloned().collect();
    long.sort_by_key(|rf| rf.lt_idx());
    init.extend(long);
    apply_list_modification(init, curr_frame_num, max_frame_num, num_active, mods)
}

/// Builds the B-slice `RefPicList0` and `RefPicList1` (spec §8.2.4.2.3), ordered
/// by `PicOrderCnt` relative to the current picture: List0 leads with nearer
/// past pictures, List1 with nearer future pictures. Long-term references follow.
/// Per-list `ref_pic_list_modification` is then applied.
pub(crate) fn build_ref_list_b(
    dpb: &[Ref],
    ctx: &ContextoListaB<'_>,
) -> Result<(Vec<Ref>, Vec<Ref>), DecodeError> {
    let mut less: Vec<Ref> = dpb
        .iter()
        .filter(|r| !r.is_long_term() && r.pic_poc() < ctx.curr_poc)
        .cloned()
        .collect();
    let mut greater: Vec<Ref> = dpb
        .iter()
        .filter(|r| !r.is_long_term() && r.pic_poc() > ctx.curr_poc)
        .cloned()
        .collect();
    let mut long: Vec<Ref> = dpb.iter().filter(|r| r.is_long_term()).cloned().collect();
    less.sort_by_key(|r| core::cmp::Reverse(r.pic_poc())); // nearest past first
    greater.sort_by_key(|r| r.pic_poc()); // nearest future first
    long.sort_by_key(|r| r.lt_idx());

    let mut init0 = less.clone();
    init0.extend(greater.clone());
    init0.extend(long.clone());
    let mut init1 = greater;
    init1.extend(less);
    init1.extend(long);

    // When List1 equals List0 and has more than one entry, swap its first two
    // entries (spec §8.2.4.2.3). PARCHE-CRUSTY (ARR-2): la comparación es sobre las
    // listas INICIALES completas, antes de recortarlas a num_ref_idx_lX_active; con
    // una sola referencia activa en la lista 1 el intercambio también se aplica
    // (último fotograma B sin referencias futuras: salía con la referencia errónea).
    if init1.len() > 1
        && init0.len() == init1.len()
        && init0
            .iter()
            .zip(init1.iter())
            .all(|(a, b)| same_picture(a, b))
    {
        // Slice pattern: `swap(0, 1)` checks both indexes even under the
        // `len() > 1` guard above; destructuring proves them.
        if let [a, b, ..] = &mut init1[..] {
            core::mem::swap(a, b);
        }
    }

    let list0 = apply_list_modification(
        init0,
        ctx.curr_frame_num,
        ctx.max_frame_num,
        ctx.num0,
        ctx.mods0,
    )?;
    let list1 = apply_list_modification(
        init1,
        ctx.curr_frame_num,
        ctx.max_frame_num,
        ctx.num1,
        ctx.mods1,
    )?;
    Ok((list0, list1))
}

/// Two DPB entries refer to the same picture (used for the List1 swap rule).
fn same_picture(a: &RefFrame, b: &RefFrame) -> bool {
    a.is_long_term() == b.is_long_term()
        && if a.is_long_term() {
            a.lt_idx() == b.lt_idx()
        } else {
            a.pic_poc() == b.pic_poc()
        }
}

/// Applies `ref_pic_list_modification` to an initialized reference list and
/// truncates it to `num_active` (spec §8.2.4.3). `init` is the full ordered list;
/// the result is `num_active` entries, possibly reordered. idc 0/1 reference
/// short-term pictures by PicNum, idc 2 long-term ones by LongTermFrameIdx.
fn apply_list_modification(
    init: Vec<Ref>,
    curr_frame_num: u32,
    max_frame_num: u32,
    num_active: usize,
    mods: &[(u32, u32)],
) -> Result<Vec<Ref>, DecodeError> {
    if mods.is_empty() {
        let mut init = init;
        init.truncate(num_active.max(1));
        return Ok(init);
    }
    let curr = i64::from(curr_frame_num);
    let max = i64::from(max_frame_num);
    let mut list = init.clone();
    let mut pic_num_pred = curr;
    let mut refidx = 0usize;
    for &(idc, val) in mods {
        let matches: Box<dyn Fn(&RefFrame) -> bool> = if idc == IDC_MODIFICACION_LARGO_PLAZO {
            Box::new(move |r: &RefFrame| r.is_long_term() && r.lt_idx() == val)
        } else {
            let abs_diff = i64::from(val) + 1;
            let no_wrap = if idc == 0 {
                let x = pic_num_pred - abs_diff;
                if x < 0 {
                    x + max
                } else {
                    x
                }
            } else {
                let x = pic_num_pred + abs_diff;
                if x >= max {
                    x - max
                } else {
                    x
                }
            };
            pic_num_pred = no_wrap;
            let target = if no_wrap > curr {
                no_wrap - max
            } else {
                no_wrap
            };
            Box::new(move |r: &RefFrame| {
                let f = i64::from(r.fn_num());
                let pn = if f > curr { f - max } else { f };
                !r.is_long_term() && pn == target
            })
        };
        let found = init.iter().find(|r| matches(r)).cloned();
        let Some(found) = found else {
            return Err(DecodeError::Truncated); // references a picture not in the DPB
        };
        if refidx > list.len() {
            break;
        }
        list.insert(refidx, found);
        if let Some(dup) = list
            .iter()
            .enumerate()
            .skip(refidx + 1)
            .find(|(_, r)| matches(r))
            .map(|(i, _)| i)
        {
            list.remove(dup);
        }
        refidx += 1;
        if refidx >= num_active {
            break;
        }
    }
    list.truncate(num_active.max(1));
    Ok(list)
}
