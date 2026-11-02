//! Pruebas de la derivación de `PicOrderCnt` (UIT-T H.264 06/26, §8.2.1).
//!
//! Cada valor esperado se deriva a mano de las fórmulas de la norma (citadas en cada prueba), no
//! del código que se prueba. El SPS de prueba se escribe bit a bit con [`BitWriter`] y se lee con
//! el analizador real [`Sps::parse`], de modo que la prueba usa los mismos campos que un flujo.
//! En todas las secuencias `MaxFrameNum` y `MaxPicOrderCntLsb` valen 16 (`log2 = 4`).

use super::PocState;
use crate::params::Sps;
use crust_h264_common::bit_writer::BitWriter;

/// `profile_idc` de Baseline (UIT-T H.264 Anexo A): sin la sección de croma ni matrices de escala.
const PERFIL_BASELINE: u32 = 66;
/// `level_idc` 3.0; no interviene en el cálculo del POC.
const NIVEL_3_0: u32 = 30;
/// `log2_max_frame_num_minus4` y `log2_max_pic_order_cnt_lsb_minus4`: 0 → máximos de 16.
const LOG2_MENOS_4: u32 = 0;
/// `MaxFrameNum` = `MaxPicOrderCntLsb` = 2^(0 + 4).
const MAXIMO: i32 = 16;
/// `nal_ref_idc` de una imagen de referencia (cualquier valor distinto de 0 vale lo mismo aquí).
const REFERENCIA: u8 = 1;
/// `nal_ref_idc` de una imagen no de referencia.
const NO_REFERENCIA: u8 = 0;

/// Parámetros del ciclo de POC de tipo 1 (§7.4.2.1.1).
struct CicloTipo1 {
    offset_for_non_ref_pic: i32,
    offset_for_top_to_bottom_field: i32,
    offset_for_ref_frame: &'static [i32],
}

/// Escribe y analiza un SPS Baseline de 1x1 macrobloques con el tipo de POC pedido.
fn sps(pic_order_cnt_type: u32, ciclo: Option<&CicloTipo1>) -> Sps {
    let mut w = BitWriter::new();
    w.write_bits(PERFIL_BASELINE, 8);
    w.write_bits(0, 8); // constraint_set*_flag y reserved_zero_2bits
    w.write_bits(NIVEL_3_0, 8);
    w.write_ue(0); // seq_parameter_set_id
    w.write_ue(LOG2_MENOS_4); // log2_max_frame_num_minus4
    w.write_ue(pic_order_cnt_type);
    match pic_order_cnt_type {
        0 => w.write_ue(LOG2_MENOS_4), // log2_max_pic_order_cnt_lsb_minus4
        1 => {
            let c = ciclo.expect("el tipo 1 necesita su ciclo");
            w.write_bit(true); // delta_pic_order_always_zero_flag
            w.write_se(c.offset_for_non_ref_pic);
            w.write_se(c.offset_for_top_to_bottom_field);
            w.write_ue(c.offset_for_ref_frame.len() as u32);
            for &o in c.offset_for_ref_frame {
                w.write_se(o);
            }
        }
        _ => {}
    }
    w.write_ue(1); // max_num_ref_frames
    w.write_bit(false); // gaps_in_frame_num_value_allowed_flag
    w.write_ue(0); // pic_width_in_mbs_minus1
    w.write_ue(0); // pic_height_in_map_units_minus1
    w.write_bit(true); // frame_mbs_only_flag
    w.write_bit(true); // direct_8x8_inference_flag
    w.write_bit(false); // frame_cropping_flag
    w.write_bit(false); // vui_parameters_present_flag
    w.rbsp_trailing_bits();
    Sps::parse(&w.into_bytes()).expect("SPS de prueba válido")
}

/// Una imagen de la secuencia: (IDR, `nal_ref_idc`, `frame_num`, `pic_order_cnt_lsb`, deltas).
type Imagen = (bool, u8, u32, u32, (i32, i32));

/// Imagen de referencia no IDR sin deltas.
fn r(frame_num: u32, lsb: u32) -> Imagen {
    (false, REFERENCIA, frame_num, lsb, (0, 0))
}

/// Imagen no de referencia sin deltas.
fn n(frame_num: u32, lsb: u32) -> Imagen {
    (false, NO_REFERENCIA, frame_num, lsb, (0, 0))
}

/// IDR (siempre de referencia, `frame_num` 0).
const IDR: Imagen = (true, REFERENCIA, 0, 0, (0, 0));

/// Calcula el POC de cada imagen en orden de decodificación con un único estado.
fn pocs(sps: &Sps, imagenes: &[Imagen]) -> Vec<i32> {
    let mut estado = PocState::default();
    imagenes
        .iter()
        .map(|&(idr, ref_idc, fnum, lsb, deltas)| {
            estado.compute_poc(sps, idr, ref_idc, fnum, lsb, deltas)
        })
        .collect()
}

// ---------------------------------------------------------------- tipo 0 (§8.2.1.1)

/// (8-3)/(8-4): sin vuelta, PicOrderCntMsb = 0 y el POC es el propio `pic_order_cnt_lsb`.
#[test]
fn tipo0_lsb_directo() {
    assert_eq!(pocs(&sps(0, None), &[IDR, r(1, 4), r(2, 8)]), vec![0, 4, 8]);
}

/// (8-3): lsb 0 < prevLsb 12 y 12 − 0 ≥ 16/2 → PicOrderCntMsb = 0 + 16 → POC 16; luego 16 + 2.
#[test]
fn tipo0_vuelta_de_lsb_suma_el_maximo() {
    let esperado = vec![0, 4, 8, 12, MAXIMO, MAXIMO + 2];
    let secuencia = [IDR, r(1, 4), r(2, 8), r(3, 12), r(4, 0), r(5, 2)];
    assert_eq!(pocs(&sps(0, None), &secuencia), esperado);
}

/// §8.2.1.1: prevPicOrderCntMsb/Lsb salen de la imagen de **referencia** anterior. Ref lsb 6
/// (6 − 0 = 6 ≤ 8 → 6), no-ref lsb 8 (8 − 6 = 2 → 8) y ref lsb 0: frente a prevLsb 6,
/// 6 − 0 = 6 < 8 → Msb 0 → 0. Si la no-ref avanzara el estado, frente a 8, 8 − 0 = 8 ≥ 8 daría
/// Msb 16 → 16. (Los valores respetan la media ventana MaxPicOrderCntLsb / 2 = 8 de (8-3).)
#[test]
fn tipo0_no_referencia_no_avanza_el_estado() {
    let secuencia = [IDR, r(1, 6), n(2, 8), r(2, 0)];
    assert_eq!(pocs(&sps(0, None), &secuencia), vec![0, 6, 8, 0]);
}

/// (8-1): PicOrderCnt(fotograma) = Min(TopFieldOrderCnt, BottomFieldOrderCnt), con
/// BottomFieldOrderCnt = TopFieldOrderCnt + delta_pic_order_cnt_bottom = 8 − 1.
#[test]
fn tipo0_campo_inferior_anterior_da_el_minimo() {
    let secuencia = [IDR, (false, REFERENCIA, 1, 8, (0, -1))];
    assert_eq!(pocs(&sps(0, None), &secuencia), vec![0, 7]);
}

/// Pág. 140 + §8.2.1.1: tras MMCO 5 en la ref con lsb 6, su TopFieldOrderCnt pasa a ser
/// TopFieldOrderCnt − PicOrderCnt = 6 − 6 = 0, y la siguiente imagen usa prevPicOrderCntMsb = 0 y
/// prevPicOrderCntLsb = 0. Ref con lsb 14: 14 > 0 y 14 − 0 > 8 → Msb = −16 → POC −2.
///
/// Vigila la corrección de D-2026-11-02-1 (antes daba 14: `apply_mmco5` dejaba `prev_lsb` en el
/// lsb de la imagen, 6, en lugar de su TopFieldOrderCnt reajustado, 0).
#[test]
fn tipo0_tras_mmco5_el_lsb_previo_es_el_top_reajustado() {
    let sps = sps(0, None);
    let mut estado = PocState::default();
    assert_eq!(estado.compute_poc(&sps, true, REFERENCIA, 0, 0, (0, 0)), 0);
    let poc_mmco5 = estado.compute_poc(&sps, false, REFERENCIA, 1, 6, (0, 0));
    assert_eq!(poc_mmco5, 6);
    estado.apply_mmco5(0, poc_mmco5);
    assert_eq!(
        estado.compute_poc(&sps, false, REFERENCIA, 1, 14, (0, 0)),
        -2
    );
}

// ---------------------------------------------------------------- tipo 1 (§8.2.1.2)

/// Ciclo de un solo fotograma de referencia con desplazamiento 2 y no-referencias a −1.
const CICLO: CicloTipo1 = CicloTipo1 {
    offset_for_non_ref_pic: -1,
    offset_for_top_to_bottom_field: 0,
    offset_for_ref_frame: &[2],
};

/// (8-7)–(8-10): absFrameNum = FrameNumOffset + frame_num (−1 si no es de referencia);
/// expectedPicOrderCnt = ((abs − 1) / 1) · 2 + 2, más offset_for_non_ref_pic si no es de referencia.
/// ref 1 → abs 1 → 2; ref 2 → abs 2 → 4; no-ref 3 → abs 2 → 4 − 1 = 3; ref 3 → abs 3 → 6.
#[test]
fn tipo1_ciclo_y_no_referencia() {
    let secuencia = [IDR, r(1, 0), r(2, 0), n(3, 0), r(3, 0)];
    assert_eq!(pocs(&sps(1, Some(&CICLO)), &secuencia), vec![0, 2, 4, 3, 6]);
}

/// (8-6): prevFrameNum 15 > frame_num 0 → FrameNumOffset = 0 + 16 → abs 16 → 15 · 2 + 2 = 32.
#[test]
fn tipo1_vuelta_de_frame_num_suma_el_maximo() {
    let mut secuencia = vec![IDR];
    secuencia.extend((1..16).map(|f| r(f, 0)));
    secuencia.push(r(0, 0));
    let resultado = pocs(&sps(1, Some(&CICLO)), &secuencia);
    assert_eq!(resultado[15], 30);
    assert_eq!(resultado[16], 2 * MAXIMO);
}

// ---------------------------------------------------------------- tipo 2 (§8.2.1.3)

/// (8-12): IDR → 0; ref → 2 · (FrameNumOffset + frame_num).
#[test]
fn tipo2_referencias_de_dos_en_dos() {
    assert_eq!(pocs(&sps(2, None), &[IDR, r(1, 0), r(2, 0)]), vec![0, 2, 4]);
}

/// (8-12): no-ref → 2 · (FrameNumOffset + frame_num) − 1; frame_num 2 → 3, y la ref 2 siguiente → 4.
#[test]
fn tipo2_no_referencia_resta_uno() {
    let secuencia = [IDR, r(1, 0), n(2, 0), r(2, 0)];
    assert_eq!(pocs(&sps(2, None), &secuencia), vec![0, 2, 3, 4]);
}

/// (8-11): prevFrameNum 15 > frame_num 0 → FrameNumOffset = 16 → 2 · (16 + 0) = 32.
#[test]
fn tipo2_vuelta_de_frame_num_suma_el_maximo() {
    let mut secuencia = vec![IDR];
    secuencia.extend((1..16).map(|f| r(f, 0)));
    secuencia.push(r(0, 0));
    let resultado = pocs(&sps(2, None), &secuencia);
    assert_eq!(resultado[15], 30);
    assert_eq!(resultado[16], 2 * MAXIMO);
}

/// §8.2.1.3: si la imagen anterior incluía MMCO 5, prevFrameNumOffset = 0, y su frame_num se
/// considera 0 (§7.4.3); la ref con frame_num 1 da 2 · (0 + 1) = 2.
///
/// La protección es doble: `apply_mmco5` pone a 0 el desplazamiento y el `frame_num` previos *y*
/// marca `prev_has_mmco5`, que `compute_poc` también trata como (0, 0). Por eso una mutación de un
/// solo punto no pone roja esta prueba; quitar las dos a la vez, sí.
#[test]
fn tipo2_tras_mmco5_el_desplazamiento_vuelve_a_cero() {
    let sps = sps(2, None);
    let mut estado = PocState::default();
    assert_eq!(estado.compute_poc(&sps, true, REFERENCIA, 0, 0, (0, 0)), 0);
    for f in 1..16 {
        estado.compute_poc(&sps, false, REFERENCIA, f, 0, (0, 0));
    }
    let poc_mmco5 = estado.compute_poc(&sps, false, REFERENCIA, 0, 0, (0, 0));
    assert_eq!(poc_mmco5, 2 * MAXIMO);
    estado.apply_mmco5(2, poc_mmco5);
    assert_eq!(estado.compute_poc(&sps, false, REFERENCIA, 1, 0, (0, 0)), 2);
}
