//! Ajustes del filtro de desbloqueo distintos por corte (COR-4, D-2026-10-12-2).
//!
//! Vector de partida: x264 (FFmpeg 8.1, CAVLC con `-coder 0`, `-x264-params slice-max-mbs=20`),
//! 10 imágenes de 128×128 (8×8 macrobloques). Los cortes empiezan en los macrobloques 0, 20, 40 y
//! 60, así que las filas 2 y 7 contienen dos cortes cada una. Sobre él:
//! - `filtro_desplaz.264`: `tests/cortes_data/desplazar.py … 6 6` pone `slice_alpha_c0_offset_div2`
//!   y `slice_beta_offset_div2` a 6 en los cortes impares de cada imagen;
//! - `filtro_idc01.264`: `tests/cortes_data/idc.py … 0,1` alterna `disable_deblocking_filter_idc`
//!   entre 0 y 1 por corte;
//! - `filtro_idc0212.264`: `idc.py … 0,2,1,2`.
//!
//! Según UIT-T H.264 08/2024, §8.7 y §8.7.2, los desplazamientos y el indicador de cada borde se
//! toman del corte del macrobloque que contiene la muestra `q0`, que es siempre el macrobloque
//! que se filtra. La referencia YUV es la decodificación de FFmpeg 8.1; OpenH264 da la misma.

use crusty_h264::Decoder;

#[test]
fn desplazamientos_del_filtro_por_corte_igual_que_ffmpeg() {
    comparar_con_ffmpeg(
        include_bytes!("../../../tests/cortes_data/filtro_desplaz.264"),
        include_bytes!("../../../tests/cortes_data/filtro_desplaz_ref.yuv"),
    );
}

#[test]
fn filtro_desactivado_en_cortes_alternos_igual_que_ffmpeg() {
    comparar_con_ffmpeg(
        include_bytes!("../../../tests/cortes_data/filtro_idc01.264"),
        include_bytes!("../../../tests/cortes_data/filtro_idc01_ref.yuv"),
    );
}

#[test]
fn filtro_activo_sin_bordes_y_desactivado_por_corte_igual_que_ffmpeg() {
    comparar_con_ffmpeg(
        include_bytes!("../../../tests/cortes_data/filtro_idc0212.264"),
        include_bytes!("../../../tests/cortes_data/filtro_idc0212_ref.yuv"),
    );
}

/// Decodifica `flujo` (10 imágenes de 128×128) y exige la misma salida que `referencia`.
fn comparar_con_ffmpeg(flujo: &[u8], referencia: &[u8]) {
    let imagenes = Decoder::new().decode_stream(flujo).expect("decodificar");
    assert_eq!(imagenes.len(), 10);

    let mut nuestra = Vec::with_capacity(referencia.len());
    for f in &imagenes {
        assert_eq!((f.width, f.height), (128, 128));
        nuestra.extend_from_slice(&f.y);
        nuestra.extend_from_slice(&f.u);
        nuestra.extend_from_slice(&f.v);
    }
    assert!(
        nuestra == referencia,
        "la reconstrucción con ajustes del filtro distintos por corte difiere de FFmpeg"
    );
}
