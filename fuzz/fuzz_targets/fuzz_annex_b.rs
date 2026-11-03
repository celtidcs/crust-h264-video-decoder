//! Objetivo de fuzzing para la capa de encapsulación Annex B.
//!
//! Evalúa la robustez de `split_annex_b`, el análisis del tipo de unidad NAL
//! y la des-prevención de emulación de bytes (`emulation_unprevent`) ante secuencias
//! de bytes arbitrarias y no conformes.

#![no_main]

use crust_h264_common::nal::{emulation_unprevent, split_annex_b, NalUnitType};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let nals = split_annex_b(data);
    for nal_data in nals {
        if !nal_data.is_empty() {
            let header = nal_data[0];
            let _ref_idc = (header >> 5) & 0x03;
            let _nal_type = NalUnitType::from_id(header);
            let _rbsp = emulation_unprevent(&nal_data[1..]);
        }
    }
});
