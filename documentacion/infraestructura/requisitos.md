# Requisitos del Sistema — Crust H264 Video Decoder

## 1. Entorno de Compilación y Ejecución

- **Sistema Operativo**:
  - Windows 10 / 11 (x86_64).
  - Linux (distribuciones modernas basadas en glibc 2.31+ o musl, arquitectura x86_64 / aarch64).
  - macOS (Apple Silicon / Intel x86_64).
- **Rust Toolchain**:
  - Rust estable 1.80.0 o posterior (soporte completo de edición 2021 y MSRV 1.80).
  - Gestor de paquetes `cargo`.
  - Componentes recomendados: `rustfmt`, `clippy`, `cargo-audit`.
- **Python**:
  - Python 3.9 o superior (exclusivamente para la orquestación de la red de equivalencia `red.py` y cálculo de métricas de velocidad).

## 2. Requisitos de Hardware

- **Memoria RAM**: 512 MB mínimo para compilación; consumo en ejecución inferior a 32 MB para secuencias 1080p gracias al búfer acotado de reordenación.
- **Espacio en Disco**: ~150 MB para el código fuente y las secuencias de calibración.
