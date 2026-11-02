# Guía de Clonado y Puesta en Marcha — Crust H264 Video Decoder

Esta guía permite clonar y validar el proyecto `crust-h264-video-decoder` desde cero en cualquier máquina con soporte de Rust.

---

## 1. Clonado del Repositorio

```bash
git clone https://github.com/celtidcs/crust-h264-video-decoder.git
cd crust-h264-video-decoder
```

---

## 2. Compilación y Puertas de Aceptación Inmediatas

Para confirmar que el entorno de trabajo y el compilador están perfectamente sincronizados:

```bash
# 1. Compilación del workspace
cargo check

# 2. Ejecución de la batería de pruebas
cargo test

# 3. Linter Clippy sin avisos
cargo clippy --all-targets -- -D warnings

# 4. Formato estricto
cargo fmt --check
```

---

## 3. Validación de la Red de Equivalencia (2.129 flujos)

Si dispone de Python instalado en el sistema, puede verificar la fidelidad frente a la línea base:

```bash
# En Windows (PowerShell)
python red.py comprobar

# En Linux / macOS
python3 red.py comprobar
```

El proceso procesará en paralelo los 2.129 flujos Annex B y confirmará la paridad absoluta con el mensaje:
`RED EN VERDE: 2129/2129 idénticas`.
