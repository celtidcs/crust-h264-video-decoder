# crust-h264-common

Primitivas compartidas del decodificador H.264 [`crust-h264-video-decoder`](https://crates.io/crates/crust-h264-video-decoder):
lectura y escritura de bits, Exp-Golomb, unidades NAL y Annex-B, transformadas, predicción intra,
compensación de movimiento y filtro de desbloqueo.

Deriva de [`rusty_h264-common`](https://crates.io/crates/rusty_h264-common) 0.16.0, de
Mata Network, publicado con licencia BSD-2-Clause.

## Seguridad de memoria

El código de este crate no contiene `unsafe` (`#![forbid(unsafe_code)]`, que el compilador
hace cumplir). Algunas dependencias sí lo usan internamente: `wide`, `safe_arch` y `bytemuck`.

## Features

Por defecto solo `std`.

| Feature | Efecto |
|---|---|
| `std` | Biblioteca estándar (sin ella, `no_std` + `alloc`, y hace falta `libm`). |
| `asm` | Núcleos SIMD de `rusty_h264-accel` (intrínsecos `unsafe`). |
| `global-alloc` | Instala `rusty_alloc-api` como asignador global de todo el proceso. |
| `libm` | Matemática de coma flotante sin `std`. |

## Licencia

BSD-2-Clause. Copyright (c) 2026, Mata Network. Véase `LICENSE`.
