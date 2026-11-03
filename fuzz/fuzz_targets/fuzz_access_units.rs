//! Objetivo de fuzzing para la separación de unidades de acceso (AU).
//!
//! Evalúa la robustez de `split_access_units` al analizar secuencias Annex B
//! y segmentarlas en unidades de acceso de acuerdo con los límites de corte
//! y tipos de NAL, ante datos arbitrarios o truncados.

#![no_main]

use crust_h264_video_decoder::split_access_units;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let _aus = split_access_units(data);
});
