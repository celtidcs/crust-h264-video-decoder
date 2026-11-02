# crusty_h264

Decodificador de vídeo H.264 / AVC (UIT-T H.264) escrito en Rust.

Deriva de [`rusty_h264-decoder`](https://crates.io/crates/rusty_h264-decoder) 0.16.0, de
Mata Network, publicado con licencia BSD-2-Clause, y le añade correcciones de conformidad con
la norma UIT-T y una reorganización modular. Las correcciones se describen en
`PARCHES-CONFORMIDAD.md`.

## Qué decodifica

- Perfiles Baseline y Main, y la mayor parte de High: CAVLC y CABAC, cortes B, predicción
  ponderada y transformada 8x8.
- Solo vídeo progresivo, crominancia 4:2:0 y 8 bits por muestra. Rechaza sin pánico el vídeo
  entrelazado, FMO/ASO y los perfiles de 10 bits, 4:2:2 o 4:4:4.

## Seguridad de memoria

El código de este crate y el de `crusty_h264-common` no contienen `unsafe`
(`#![forbid(unsafe_code)]`, que el compilador hace cumplir) ni llaman a C. Algunas
dependencias sí usan `unsafe` internamente: `wide`, `safe_arch` y `bytemuck`.

## Features

Ninguna está activada por defecto.

| Feature | Efecto |
|---|---|
| `asm` | Núcleos SIMD de `rusty_h264-accel` (intrínsecos `unsafe`). |
| `global-alloc` | Instala `rusty_alloc-api` como asignador global de todo el proceso. |
| `libm` | Matemática de coma flotante sin `std`. |

## Uso

```rust
use crusty_h264::Decoder;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let flujo = std::fs::read("video.264")?; // flujo Annex-B
    let mut decodificador = Decoder::new();
    let fotogramas = decodificador
        .decode_stream(&flujo)
        .map_err(|error| format!("no se pudo decodificar: {error:?}"))?;
    println!("{} fotogramas", fotogramas.len());
    Ok(())
}
```

Más ejemplos en el manual de uso del repositorio:
<https://github.com/celtidcs/crusty_h264>.

## Licencia

BSD-2-Clause. Copyright (c) 2026, Mata Network. Véase `LICENSE`.
