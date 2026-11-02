//! Primitivas compartidas del decodificador H.264 `crusty_h264`, derivadas de
//! `rusty_h264-common` 0.16.0 (Mata Network, BSD-2).
//!
//! # Asignador global
//!
//! La feature `global-alloc` (**desactivada por defecto**) instala [`rusty_alloc`] como
//! asignador de todo el proceso. `#[global_allocator]` es único por programa: actívela solo si
//! su programa no declara ya otro asignador. La feature `asm` (también desactivada por defecto)
//! enruta los núcleos calientes a `rusty_h264-accel`, cuyos intrínsecos SIMD son `unsafe`.
//!
//! This crate is the foundation both the encoder and decoder sit on. It is
//! `#![forbid(unsafe_code)]`: the bit-twiddling core of an H.264 codec is
//! exactly where memory-safety bugs hide in the C implementations, so we keep
//! it provably safe.
//!
//! Modules mirror the concerns shared across `codec/common` in Cisco's
//! openh264:
//! - [`bit_writer`] / [`bit_reader`] — MSB-first bit packing + Exp-Golomb.
//! - [`nal`] — NAL units, Annex-B framing, RBSP emulation prevention.
//! - [`types`] — shared enums and the raw YUV frame container.
//!
//! El crate es `#![forbid(unsafe_code)]`, sin excepciones.
#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;
#[cfg(test)]
extern crate std;
#[cfg(all(not(feature = "std"), not(feature = "libm")))]
compile_error!(
    "crusty_h264-common without `std` needs the `libm` feature for its floating-point math"
);

// ---------------------------------------------------------------------------
// `no_std` shims. Every `std` use left in this crate is a diagnostic: an
// environment knob (`RS_H264_*` / `RFF_*`), a stderr census, the profiler.
// Without `std` there is no environment and no stderr, so a knob reads as
// unset and a print is a no-op: the shipped defaults, which is what a chip
// runs. Defined here, before the modules, so they are in textual scope.
// ---------------------------------------------------------------------------

#[cfg(not(feature = "std"))]
macro_rules! eprintln {
    ($($t:tt)*) => {{
        let _ = ::core::format_args!($($t)*);
    }};
}
#[cfg(not(feature = "std"))]
#[allow(unused_macros)]
macro_rules! println {
    ($($t:tt)*) => {{
        let _ = ::core::format_args!($($t)*);
    }};
}

/// Process-wide allocator (see the crate docs).
#[cfg(feature = "global-alloc")]
#[global_allocator]
static ALLOC: rusty_alloc_api::RustyAlloc = rusty_alloc_api::RustyAlloc;

pub mod aligned;
pub mod bit_reader;
pub mod bit_writer;
pub mod cabac_tables;
pub mod cavlc;
pub mod deblock;

/// Whether the vendored SIMD kernels are compiled in (`asm` feature on x86-64).
/// Exposed so benchmarks can state which path they measured — a harness that
/// silently falls back to the scalar twin reports numbers that look like a
/// regression in the fast path.
pub const ACCEL: bool = cfg!(accel);
pub mod inter;
pub mod nal;
pub mod predict;
pub mod rango;
/// 64-bit atomics where the target has them, the portable fallback where not.
#[doc(hidden)]
pub mod atomic {
    #[cfg(target_has_atomic = "64")]
    pub use core::sync::atomic::AtomicU64;
    #[cfg(not(target_has_atomic = "64"))]
    pub use portable_atomic::AtomicU64;
}

/// `OnceLock` on both sides of the ladder: `std::sync::OnceLock` with `std`,
/// a `once_cell::race::OnceBox` (atomics + one heap box per cell) without.
/// Same `new` / `get_or_init` shape, so a lazily built table or a cached knob
/// in a `static` reads identically on the host and on a chip.
#[doc(hidden)]
pub mod once {
    #[cfg(feature = "std")]
    pub use std::sync::OnceLock;

    #[cfg(not(feature = "std"))]
    pub struct OnceLock<T>(once_cell::race::OnceBox<T>);

    #[cfg(not(feature = "std"))]
    impl<T> OnceLock<T> {
        /// An empty cell.
        pub const fn new() -> Self {
            OnceLock(once_cell::race::OnceBox::new())
        }
        /// The value, initialising it with `f` on first use.
        pub fn get_or_init(&self, f: impl FnOnce() -> T) -> &T {
            self.0.get_or_init(|| alloc::boxed::Box::new(f()))
        }
        /// The value if initialised.
        pub fn get(&self) -> Option<&T> {
            self.0.get()
        }
        /// Set the value if the cell is empty; otherwise hand it back.
        pub fn set(&self, v: T) -> Result<(), T> {
            self.0.set(alloc::boxed::Box::new(v)).map_err(|b| *b)
        }
    }

    #[cfg(not(feature = "std"))]
    impl<T> Default for OnceLock<T> {
        fn default() -> Self {
            Self::new()
        }
    }
}

pub mod fmath;
pub mod transform;
pub mod types;

pub use bit_reader::BitReader;
pub use bit_writer::BitWriter;
pub use nal::{NalUnit, NalUnitType};
pub use rango::FueraDeRango;
pub use types::{ChromaFormat, Profile, YuvFrame, YuvPlanes};
