//! Objetivo de fuzzing para el decodificador de vídeo completo sobre API pública.
//!
//! Evalúa la robustez de `Decoder::decode_stream` y `Decoder::decode` procesando
//! tanto flujos enteros como secuencias divididas por unidades de acceso (`split_access_units`),
//! garantizando que cualquier flujo hostil o malformado sea tolerado o rechazado limpiamente
//! sin abortos inesperados ni pánicos incontrolados.

#![no_main]

use crust_h264_video_decoder::Decoder;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // Probar decodificación de flujo completo
    let mut decoder = Decoder::new();
    let _ = decoder.decode_stream(data);

    // Probar decodificación alimentada por unidades de acceso
    let mut streaming_decoder = Decoder::new();
    for au in crust_h264_video_decoder::split_access_units(data) {
        let _ = streaming_decoder.decode(au);
    }
});
