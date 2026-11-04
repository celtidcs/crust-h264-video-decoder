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
    sps_con_huecos(pic_order_cnt_type, ciclo, false)
}

/// Escribe y analiza un SPS Baseline permitiendo configurar gaps_in_frame_num_value_allowed_flag.
fn sps_con_huecos(pic_order_cnt_type: u32, ciclo: Option<&CicloTipo1>, gaps_allowed: bool) -> Sps {
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
    w.write_bit(gaps_allowed); // gaps_in_frame_num_value_allowed_flag
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

// ---------------------------------------------------------------- Casos D-2026-10-21-1 y D-2026-11-04-1

/// Caso no conforme tipo 2: ref 5 -> no-ref 10 -> ref 7.
/// UIT-T H.264 (06/2026) §8.2.1.3 (pág. 119):
/// «Let prevFrameNum be equal to the frame_num of the previous picture in decoding order.»
/// - Pic 0: IDR -> POC 0.
/// - Pic 1: Ref fn 5 -> prev_fn 0 <= 5 -> offset 0 -> POC 2*(0+5) = 10.
/// - Pic 2: No-ref fn 10 -> prev_fn 5 <= 10 -> offset 0 -> POC 2*(0+10) - 1 = 19.
/// - Pic 3: Ref fn 7:
///   En la norma: prev_fn es 10 (de la no-ref anterior). Como 10 > 7, (8-11) deriva
///   FrameNumOffset = prevFrameNumOffset + MaxFrameNum = 0 + 16 = 16 -> POC = 2*(16+7) = 46.
///   En el código previo (b80d1fc), el estado solo se actualizaba con referencias:
///   prev_fn seguía en 5, 5 <= 7 -> offset 0 -> POC = 14.
///
/// Con el código sin parchear da 14; la aserción exige 46 (ROJO sin corrección).
#[test]
fn tipo2_no_conforme_no_ref_avanza_prev_fn() {
    let secuencia = [IDR, r(5, 0), n(10, 0), r(7, 0)];
    let resultado = pocs(&sps(2, None), &secuencia);
    assert_eq!(resultado, vec![0, 10, 19, 46]);
}

/// Caso no conforme tipo 1: ref 5 -> no-ref 10 -> ref 7.
/// UIT-T H.264 (06/2026) §8.2.1.2 (pág. 118):
/// «Let prevFrameNum be equal to the frame_num of the previous picture in decoding order.»
/// Ciclo [2], offset_non_ref = -1, delta_always_zero = true.
/// - Pic 0: IDR -> POC 0.
/// - Pic 1: Ref fn 5 -> offset 0, abs 5 -> POC (5-1)*2 + 2 = 10.
/// - Pic 2: No-ref fn 10 -> offset 0, abs 10 - 1 = 9 -> POC (9-1)*2 + 2 - 1 = 17.
/// - Pic 3: Ref fn 7:
///   En la norma: prev_fn es 10 (no-ref anterior). 10 > 7 -> (8-6) FrameNumOffset = 16.
///   absFrameNum = 16 + 7 = 23 -> expected = (23-1)*2 + 2 = 46.
///   En el código previo (b80d1fc): prev_fn seguía en 5 <= 7 -> offset 0, abs 7 -> POC 14.
///
/// Con el código sin parchear da 14; la aserción exige 46 (ROJO sin corrección).
#[test]
fn tipo1_no_conforme_no_ref_avanza_prev_fn() {
    let secuencia = [IDR, r(5, 0), n(10, 0), r(7, 0)];
    let resultado = pocs(&sps(1, Some(&CICLO)), &secuencia);
    assert_eq!(resultado, vec![0, 10, 17, 46]);
}

/// §8.2.1.3: tras MMCO 5, una imagen no de referencia posterior restablece `prev_has_mmco5` a false.
/// Secuencia: Ref fn 0 con MMCO 5 -> No-ref fn 2 -> Ref fn 1.
/// - Ref con MMCO 5: pone prev_has_mmco5 = true, prev_fn = 0, offset = 0.
/// - No-ref fn 2: se procesa con offset 0 -> POC 3. Tras procesarla, como la imagen anterior
///   inmediata para la siguiente es esta no-ref (que no incluyó MMCO 5), prev_has_mmco5 debe ser false
///   y prev_fn = 2.
/// - Ref fn 1: como prev_has_mmco5 es false, se evalúa prev_fn (2) > frame_num (1), lo que dispara
///   FrameNumOffset = 0 + 16 = 16 -> POC = 2*(16+1) = 34.
///
/// Sin la corrección, prev_has_mmco5 seguía en true, forzando offset 0 y dando POC 2.
#[test]
fn tipo2_mmco5_seguida_de_no_ref_y_ref() {
    let sps = sps(2, None);
    let mut estado = PocState::default();
    assert_eq!(estado.compute_poc(&sps, true, REFERENCIA, 0, 0, (0, 0)), 0);
    for f in 1..16 {
        estado.compute_poc(&sps, false, REFERENCIA, f, 0, (0, 0));
    }
    let poc_mmco5 = estado.compute_poc(&sps, false, REFERENCIA, 0, 0, (0, 0));
    assert_eq!(poc_mmco5, 2 * MAXIMO);
    estado.apply_mmco5(2, poc_mmco5);
    // No-ref fn 2
    let poc_noref = estado.compute_poc(&sps, false, NO_REFERENCIA, 2, 0, (0, 0));
    assert_eq!(poc_noref, 3); // 2*(0+2) - 1
                              // Ref fn 1: prev_fn era 2 > 1 -> offset 16 -> 2*(16+1) = 34
    let poc_ref1 = estado.compute_poc(&sps, false, REFERENCIA, 1, 0, (0, 0));
    assert_eq!(poc_ref1, 34);
}

/// §8.2.1.2: tras MMCO 5, una imagen no de referencia posterior restablece `prev_has_mmco5` a false en tipo 1.
/// Secuencia: Ref fn 0 con MMCO 5 -> No-ref fn 2 -> Ref fn 1.
/// - Ref con MMCO 5: pone prev_has_mmco5 = true, prev_fn = 0, offset = 0.
/// - No-ref fn 2: se procesa con offset 0. Al finalizar, prev_has_mmco5 debe restablecerse a false y prev_fn = 2.
/// - Ref fn 1: como prev_has_mmco5 es false y frame_num (1) < prev_frame_num (2), se detecta la vuelta de frame_num
///   y FrameNumOffset pasa de 0 a 16.
///
/// Si prev_has_mmco5 no se restablece a false tras la no-referencia, FrameNumOffset se forzaría indebidamente a 0.
#[test]
fn tipo1_mmco5_seguida_de_no_ref_y_ref() {
    let ciclo = CicloTipo1 {
        offset_for_non_ref_pic: 0,
        offset_for_top_to_bottom_field: 0,
        offset_for_ref_frame: &[10],
    };
    let sps = sps(1, Some(&ciclo));
    let mut estado = PocState::default();
    assert_eq!(estado.compute_poc(&sps, true, REFERENCIA, 0, 0, (0, 0)), 0);
    estado.apply_mmco5(1, 0);
    // No-ref fn 2 (offset 0 -> abs_fn = 0 + 2 - 1 = 1 -> 1 ciclo = 10)
    let poc_noref = estado.compute_poc(&sps, false, NO_REFERENCIA, 2, 0, (0, 0));
    assert_eq!(poc_noref, 10);
    // Ref fn 1: prev_fn era 2 > 1 -> vuelta detectada -> FrameNumOffset = 16 -> fn 1 + 16 = 17 -> 17*10 = 170
    // Si prev_has_mmco5 permaneciese en true, offset sería 0 -> fn 1 -> 1*10 = 10 != 170
    let poc_ref1 = estado.compute_poc(&sps, false, REFERENCIA, 1, 0, (0, 0));
    assert_eq!(poc_ref1, 170);
}

/// Derivación secuencial de inexistentes tras imagen no de referencia con vuelta de MaxFrameNum (§8.2.1.3 y §8.2.5.2).
/// Ref fn 1..15 -> No-ref fn 0 -> Ref fn 3 (con hueco fn 0, 1, 2).
/// Derivación aritmética paso a paso según §8.2.1.3:
/// - Ref fn 15: prev_fn = 15, offset = 0, prev_ref_fn = 15. POC = 30.
/// - No-ref fn 0: prev_fn (15) > fn (0) -> offset = 16. POC = 2*(16+0)-1 = 31.
///   Estado tras no-ref: prev_fn = 0, offset = 16, prev_ref_fn = 15.
/// - Inexistentes en hueco previo a Ref fn 3 (UnusedShortTermFrameNum = 0, 1, 2):
///   - Inexistente fn 0: prev_fn = 0 <= 0 -> offset = 16 -> POC = 2*(16+0) = 32.
///   - Inexistente fn 1: prev_fn = 0 <= 1 -> offset = 16 -> POC = 2*(16+1) = 34.
///   - Inexistente fn 2: prev_fn = 1 <= 2 -> offset = 16 -> POC = 2*(16+2) = 36.
/// - Ref fn 3 (actual): prev_fn = 2 <= 3 -> offset = 16 -> POC = 2*(16+3) = 38.
///
/// Si se evaluara con prev_ref_fn (15 >= 3), sumaría otra vuelta a 32 derivando POC 70 erróneo.
#[test]
fn tipo2_vuelta_contador_inexistentes_tras_no_ref() {
    let mut d = crate::Decoder::new();
    let s = sps(2, None);
    // IDR fn 0 -> POC 0
    d.poc.compute_poc(&s, true, 1, 0, 0, (0, 0));
    d.prev_ref_frame_num = 0;
    // Ref fn 1..=15
    for f in 1..=15 {
        d.poc.compute_poc(&s, false, 1, f, 0, (0, 0));
        d.prev_ref_frame_num = f;
    }
    // No-ref fn 0: avanza prev_frame_num a 0 y offset a 16. prev_ref_frame_num sigue en 15.
    let no_ref_poc = d.poc.compute_poc(&s, false, 0, 0, 0, (0, 0));
    assert_eq!(no_ref_poc, 31);
    assert_eq!(d.prev_ref_frame_num, 15);

    // Ref fn 3 llega con hueco (unused = 0, 1, 2)
    // insert_frame_num_gaps debe evaluar los inexistentes con prev_frame_num (0), NO con prev_ref_frame_num (15)
    d.insert_frame_num_gaps(3, 16, 8, 16, 16, &s);

    // Inexistentes en DPB: 2 (poc 36), 1 (poc 34), 0 (poc 32)
    let pocs: Vec<i32> = d.refs.iter().map(|r| r.poc).collect();
    assert_eq!(pocs, vec![36, 34, 32]);
    let fns: Vec<u32> = d.refs.iter().map(|r| r.frame_num).collect();
    assert_eq!(fns, vec![2, 1, 0]);

    // Ref 3 actual
    let ref3_poc = d.poc.compute_poc(&s, false, 1, 3, 0, (0, 0));
    assert_eq!(ref3_poc, 38);
}

// ---------------------------------------------------------------- Secuencias conformes B1–B4 (Criterio B)

/// B1 (Tipo 2, no-ref intercaladas sin hueco, MaxFrameNum = 16):
/// Derivación: [IDR fn 0 (POC 0), Ref 1 (POC 2), No-ref 2 (POC 3), Ref 2 (POC 4), No-ref 3 (POC 5), Ref 3 (POC 6)].
/// En la norma y en el código coinciden en todos los valores.
#[test]
fn tipo2_conforme_b1_no_ref_intercaladas_sin_hueco() {
    let secuencia = [IDR, r(1, 0), n(2, 0), r(2, 0), n(3, 0), r(3, 0)];
    assert_eq!(pocs(&sps(2, None), &secuencia), vec![0, 2, 3, 4, 5, 6]);
}

/// B2 (Tipo 2, vuelta de MaxFrameNum = 16 con no-ref intercalada y huecos permitidos):
/// SPS con gaps_in_frame_num_value_allowed_flag = 1.
/// - IDR fn 0 -> POC 0.
/// - Ref fn 14 -> POC 28 (2*(0+14)).
/// - No-ref fn 15 -> POC 29 (2*(0+15)-1).
/// - Ref fn 0 -> prev_fn 15 > 0 -> offset = 16 -> POC 32 (2*(16+0)).
#[test]
fn tipo2_conforme_b2_vuelta_max_frame_num_con_huecos() {
    let s = sps_con_huecos(2, None, true);
    let secuencia = [IDR, r(14, 0), n(15, 0), r(0, 0)];
    assert_eq!(pocs(&s, &secuencia), vec![0, 28, 29, 32]);
}

/// B3 (Tipo 1, ciclo [2], offset_non_ref = -1, vuelta con huecos permitidos):
/// SPS con gaps_in_frame_num_value_allowed_flag = 1.
/// - IDR fn 0 -> POC 0.
/// - Ref fn 14 -> abs 14 -> POC 28.
/// - No-ref fn 15 -> abs 15 - 1 = 14 -> POC 27.
/// - Ref fn 0 -> prev_fn 15 > 0 -> offset = 16 -> abs 16 -> POC 32.
#[test]
fn tipo1_conforme_b3_vuelta_max_frame_num_con_huecos() {
    let s = sps_con_huecos(1, Some(&CICLO), true);
    let secuencia = [IDR, r(14, 0), n(15, 0), r(0, 0)];
    assert_eq!(pocs(&s, &secuencia), vec![0, 28, 27, 32]);
}

/// B4 (Tipo 2, huecos permitidos, afirma explícitamente ambos inexistentes tras no-ref):
/// - IDR fn 0 -> POC 0. PrevRef = 0.
/// - Ref fn 2: hueco fn 1 (POC 2) -> actual fn 2: POC 4. PrevRef = 2.
/// - No-ref fn 3: POC 2*(0+3)-1 = 5. PrevRef sigue en 2.
/// - Ref fn 5: hueco desde PrevRef (2) abarca fn 3 y fn 4.
///   Inexistente fn 3: POC 6 (2*(0+3)).
///   Inexistente fn 4: POC 8 (2*(0+4)).
///   Actual fn 5: POC 10 (2*(0+5)).
#[test]
fn tipo2_conforme_b4_huecos_permitidos_afirma_ambos_inexistentes() {
    let mut d = crate::Decoder::new();
    let s = sps_con_huecos(2, None, true);
    // IDR fn 0 -> POC 0
    let poc0 = d.poc.compute_poc(&s, true, 1, 0, 0, (0, 0));
    assert_eq!(poc0, 0);
    d.prev_ref_frame_num = 0;

    // Ref fn 2: hueco fn 1
    d.insert_frame_num_gaps(2, 16, 8, 16, 16, &s);
    assert_eq!(d.refs[0].frame_num, 1);
    assert_eq!(d.refs[0].poc, 2);
    let poc2 = d.poc.compute_poc(&s, false, 1, 2, 0, (0, 0));
    assert_eq!(poc2, 4);
    d.prev_ref_frame_num = 2;

    // No-ref fn 3: POC 2*(0+3)-1 = 5. PrevRef sigue en 2.
    let poc3_nr = d.poc.compute_poc(&s, false, 0, 3, 0, (0, 0));
    assert_eq!(poc3_nr, 5);
    assert_eq!(d.prev_ref_frame_num, 2);

    // Ref fn 5: hueco desde PrevRef (2) abarca fn 3 y fn 4.
    d.insert_frame_num_gaps(5, 16, 8, 16, 16, &s);
    assert_eq!(d.refs[0].frame_num, 4);
    assert_eq!(d.refs[0].poc, 8);
    assert_eq!(d.refs[1].frame_num, 3);
    assert_eq!(d.refs[1].poc, 6);

    // Actual fn 5: POC 10 (2*(0+5))
    let poc5 = d.poc.compute_poc(&s, false, 1, 5, 0, (0, 0));
    assert_eq!(poc5, 10);
}

/// Una imagen no de referencia que causa una vuelta de MaxFrameNum
/// debe actualizar `prev_frame_num_offset` para las imágenes posteriores (§8.2.1.2 y §8.2.1.3).
/// Secuencia: Ref 14 -> No-ref 0 -> Ref 1.
/// - Ref 14: offset 0.
/// - No-ref 0: 14 > 0 -> offset 16. Tras procesarla, `prev_frame_num_offset` debe ser 16.
/// - Ref 1: 0 <= 1 -> offset hereda 16 -> Tipo 2 POC = 34.
///
/// Si `prev_frame_num_offset` no se actualizase con la imagen no de referencia,
/// permanecería en 0 derivando POC 2 en lugar de 34.
#[test]
fn tipo2_no_ref_avanza_prev_fn_offset() {
    let secuencia = [IDR, r(14, 0), n(0, 0), r(1, 0)];
    assert_eq!(pocs(&sps(2, None), &secuencia), vec![0, 28, 31, 34]);
}

#[test]
fn tipo1_no_ref_avanza_prev_fn_offset() {
    let secuencia = [IDR, r(14, 0), n(0, 0), r(1, 0)];
    assert_eq!(pocs(&sps(1, Some(&CICLO)), &secuencia), vec![0, 28, 29, 34]);
}

/// Medición directa del coste de `insert_frame_num_gaps` en el peor caso posible:
/// - Tipo 1 con ciclo de 255 offsets
/// - MaxFrameNum = 65536
/// - gap = 65534 (hueco máximo, pues frame_num == PrevRefFrameNum no genera hueco)
/// - max_refs = 16
///
/// Ejecuta n=100 iteraciones para medir media, mínimo y máximo en milisegundos.
#[test]
#[ignore = "benchmark de medición temporal; no determinista bajo carga del sistema"]
fn medicion_coste_insercion_hueco_peor_caso() {
    let offsets = vec![1i32; 255];
    let mut w = BitWriter::new();
    w.write_bits(PERFIL_BASELINE, 8);
    w.write_bits(0, 8);
    w.write_bits(NIVEL_3_0, 8);
    w.write_ue(0); // seq_parameter_set_id
    w.write_ue(12); // log2_max_frame_num_minus4 -> MaxFrameNum = 65536
    w.write_ue(1); // pic_order_cnt_type = 1
    w.write_bit(true); // delta_pic_order_always_zero_flag
    w.write_se(0); // offset_for_non_ref_pic
    w.write_se(0); // offset_for_top_to_bottom_field
    w.write_ue(255); // num_ref_frames_in_pic_order_cnt_cycle = 255
    for &o in &offsets {
        w.write_se(o);
    }
    w.write_ue(16); // max_num_ref_frames = 16
    w.write_bit(true); // gaps_in_frame_num_value_allowed_flag
    w.write_ue(0); // pic_width_in_mbs_minus1
    w.write_ue(0); // pic_height_in_map_units_minus1
    w.write_bit(true); // frame_mbs_only_flag
    w.write_bit(true); // direct_8x8_inference_flag
    w.write_bit(false); // frame_cropping_flag
    w.write_bit(false); // vui_parameters_present_flag
    w.rbsp_trailing_bits();

    let sps = Sps::parse(&w.into_bytes()).expect("SPS sintético peor caso debe parsear");

    let n = 100;
    let mut tiempos = alloc::vec::Vec::with_capacity(n);
    for _ in 0..n {
        let mut d = crate::Decoder::new();
        d.prev_ref_frame_num = 0;
        let t0 = std::time::Instant::now();
        d.insert_frame_num_gaps(65535, 65536, 16, 16, 16, &sps);
        let dt = t0.elapsed();
        tiempos.push(dt.as_secs_f64() * 1000.0);
        assert_eq!(d.refs.len(), 16);
    }

    let min = tiempos.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = tiempos.iter().cloned().fold(0.0, f64::max);
    let sum: f64 = tiempos.iter().sum();
    let media = sum / n as f64;
    println!(
        "\n[BENCH COSTE HUECO PEOR CASO (n=100)] media={:.4} ms, min={:.4} ms, max={:.4} ms",
        media, min, max
    );
}
