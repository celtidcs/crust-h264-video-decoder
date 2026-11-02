//! Varios cortes por imagen con listas de referencias DISTINTAS entre cortes (COR-3, `map_ref`).
//!
//! Vectores: x264 (FFmpeg 8.1, `-x264-params slices=4:bframes=0:weightp=0:ref=3:scenecut=0`,
//! 10 imágenes de 128×128; uno CAVLC con `-coder 0` y otro CABAC) a los que `tests/cortes_data/reordenar.py` añade, en los cortes P impares
//! de cada imagen con dos o más referencias activas, una `ref_pic_list_modification` válida que
//! pone primero la imagen `CurrPicNum − 2` (UIT-T H.264 08/2024, §7.3.3.1, §8.2.4.3.1). La
//! referencia YUV es la decodificación de FFmpeg 8.1. Sin la modificación, rusty, OpenH264 y
//! FFmpeg dan la misma salida; con ella, rusty difería en el desbloqueo de las últimas filas de
//! cada corte y en el último macrobloque de cada corte (trabajo diferido ejecutado con la lista
//! del corte siguiente).

use crust_h264_video_decoder::Decoder;

#[test]
fn cortes_con_listas_distintas_igual_que_ffmpeg() {
    comparar_con_ffmpeg(
        include_bytes!("../../../tests/cortes_data/mslice_reorden.264"),
        include_bytes!("../../../tests/cortes_data/mslice_reorden_ref.yuv"),
    );
}

#[test]
fn cortes_cabac_con_listas_distintas_igual_que_ffmpeg() {
    comparar_con_ffmpeg(
        include_bytes!("../../../tests/cortes_data/mslice_reorden_cabac.264"),
        include_bytes!("../../../tests/cortes_data/mslice_reorden_cabac_ref.yuv"),
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
        "la reconstrucción con listas distintas por corte difiere de FFmpeg"
    );
}
