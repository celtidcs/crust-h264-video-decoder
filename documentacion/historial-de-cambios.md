# Historial de Cambios y Auditoría — Crusty_H264

Este documento recoge de forma cronológica la evolución del decodificador, documentando las auditorías mecánicas, las correcciones de conformidad UIT-T y la reestructuración arquitectónica de la base de código.

---

## 2026-11-02 — Versión v0.1.2: Pruebas Verificables desde un Clon Limpio y Crates Propios

- **Pruebas en un clon limpio**: las pruebas de integración de `rusty_h264-decoder` (`filtro_por_corte`, `mslice_reorden`, `mslice_idc2`, `ipcm_cabac`) incrustan siete salidas de referencia `tests/{cabac,cortes}_data/*_ref.yuv` que la regla `*.yuv` de `.gitignore` excluía; en un clon de v0.1.0 o v0.1.1 esas pruebas no compilaban. Ahora se versionan, y `*.yuv` y `*.264` se declaran binarios en `.gitattributes` para que git no altere sus bytes.
- **Documentación**: comentarios y documentos sin referencias al proceso interno de desarrollo; se precisa que la ausencia de `unsafe` cubre el código propio de las bibliotecas y se declaran las dependencias que usan `unsafe` (`wide`, `safe_arch`, `bytemuck` y, con la feature `asm`, `rusty_h264-accel`).
- **Crates propios para crates.io (sin publicar)**: `vendor/rusty_h264-decoder-0.16.0` pasa a `crates/crusty_h264` (biblioteca `crusty_h264`) y `vendor/rusty_h264-common-0.16.0` a `crates/crusty_h264-common`, ambos en la versión 0.1.2 y con crédito a Mata Network como autor original. Las features `asm` (núcleos SIMD de `rusty_h264-accel`) y `global-alloc` (asignador global `rusty_alloc-api`) dejan de estar activadas por defecto. **Cambio incompatible para quien dependía de las rutas o nombres anteriores**: `rusty_h264_decoder::` pasa a ser `crusty_h264::`.
- **Red de equivalencia REF-1**: la línea base `red/linea-base.tsv` no incluía tres flujos reales añadidos a `vel/` (`cavlc720`, `safari1080`, `safari1080p`), por lo que `scripts/red.py comprobar` no podía pasar desde un clon limpio. Se completan; la red da ahora 2.132/2.132 desde `git archive`, y la salida decodificada es idéntica antes y después del renombrado de los crates.
- **Verificación**: `cargo test --all-features`, `cargo clippy --all-targets --all-features -- -D warnings`, `cargo fmt --check` y `cargo audit` en verde en Linux y Windows desde un `git archive` limpio (common: 99 pruebas; decoder: 70).

---

## 2026-10-30 — Versión v0.1.1: Saneamiento Integral de Codificación UTF-8 / BOM y Exportación de Reordenación

- **Motivo y Detección**: Hallazgos comunicados mediante auditoría cruzada externa:
  1. Presencia de marca BOM (`\xef\xbb\xbf`) y caracteres con doble codificación mojibake (UTF-8 interpretado como CP1252/Latin-1 y re-codificado: `â€”`, `Ã—`, `Ã­`) en 14 archivos del proyecto (incluyendo descripción de `Cargo.toml`, comentarios y docstrings).
  2. Módulo huérfano `reordenacion.rs` en el árbol de `rusty_h264-decoder`, el cual no estaba expuesto en `src/lib.rs` y requería `#[path]` externo para ser consumido por el banco de pruebas.
- **Acciones Realizadas**:
  - Eliminación determinista de marcas BOM y reversión exacta del mojibake en los 14 archivos afectados mediante transcodificación byte a byte sin pérdida. Cero archivos con BOM y cero caracteres residuales en todo el repositorio.
  - Exportación formal de `pub mod reordenacion;` y tipos principales (`OrdenPresentacionH264`, `ReordenacionH264`, `capacidad_dpb`, `MAX_IMAGENES_DPB`) en `rusty_h264-decoder/src/lib.rs`.
  - Compatibilidad `#![no_std]` garantizada en `reordenacion.rs` incorporando importaciones explícitas de `alloc::vec::Vec`.
  - Refactorización de `src/main.rs` para consumir `rusty_h264_decoder::reordenacion` directamente como dependencia de biblioteca, eliminando la directiva `#[path]`.
  - Actualización del versionado SemVer a `0.1.1` (`v0.1.1`).
- **Verificación Mecánica**:
  - `cargo test --all-targets`: superado (incluyendo las 7 pruebas unitarias de reordenación integradas en el test runner de la biblioteca).
  - `cargo clippy --all-targets -- -D warnings`: 0 avisos.
  - `cargo fmt --check`: 0 diferencias.
  - `cargo audit`: 0 vulnerabilidades (35 dependencias escaneadas).
  - `contar_panicos.py`: 0 puntos de pánico en producción (`common: 0`, `decoder: 0`).

---

## 2026-10-22 — Ejecución de Pruebas Pendientes, Incorporación Full HD 1080p y Verificación Multirresolución

- **Alcance**: Incorporación de secuencias Full HD (1080p) al banco de velocidad (`vel/`) y ejecución de pruebas de rendimiento comparativas (720p, 1080p, 4K) frente a oráculos externos.
- **Acciones Realizadas**:
  - Creación de secuencias Full HD normalizadas (`vel/safari1080.264` a 1920x1036 y `vel/safari1080p.264` a 1920x1080) con perfil High, B-frames piramidales, CABAC y ponderación.
  - Actualización del arnés de evaluación de rendimiento `velocidad.py` incorporando las secuencias Full HD y 4K.
  - Medición comparativa frente al decodificador C/C++ y ensamblador de Cisco OpenH264:
    - **720p**: Crusty_H264 1.162 ms (~103 fps) vs OpenH264 1.217 ms (Crusty 4.5% más rápido).
    - **1080p**: Crusty_H264 3.285 ms (~36.5 fps) vs OpenH264 3.648 ms (Crusty 10.0% más rápido).
    - **4K UHD**: Crusty_H264 15.728 ms (~7.6 fps) vs OpenH264 18.124 ms (Crusty 13.2% más rápido).
  - Verificación estricta de las cuatro puertas mecánicas (§1.1):
    - `cargo test`: 99 pruebas en `common`, 55 en `decoder`, 7 de integración por corte y 7 en `banco_h264` (100% VERDE).
    - `cargo clippy --all-targets -- -D warnings`: 0 avisos.
    - `cargo fmt --check`: 0 diferencias.
    - `cargo audit`: 0 vulnerabilidades (35 dependencias escaneadas).
    - `contar_panicos.py`: 0 puntos de pánico en código de producción (`common: 0`, `decoder: 0`).
  - Documentación de la tabla de rendimiento multirresolución en `documentacion/funcionalidades.md`.

---

## 2026-10-22 — Creación e Independización del Proyecto Crusty_H264

- **Alcance**: Escisión del decodificador H.264 seguro para constituir un proyecto autónomo e independiente en `crusty_h264`.
- **Estructura Creada**:
  - Crates consolidados: `rusty_h264-common-0.16.0` y `rusty_h264-decoder-0.16.0`.
  - Banco de pruebas CLI (`banco_h264`), suite de mutaciones y scripts de verificación (`red.py`, `velocidad.py`, `contar_panicos.py`).
  - Integración del módulo `reordenacion.rs` dentro de la librería del decodificador.
  - Implementación del sistema modular de control y verificación formal de calidad.
  - Suite de documentación canónica en español con prosa humana y diagramas explicativos.
- **Evidencia Mecánica**:
  - `cargo check`: salida 0.
  - `cargo test`: 99 pruebas de common, 55 unitarias de decoder, 7 pruebas de reordenación y 7 pruebas de integración por corte, todas en VERDE.
  - `cargo clippy --all-targets -- -D warnings`: 0 avisos.
  - `cargo fmt --check`: 0 diferencias.
  - `cargo audit`: 0 vulnerabilidades (35 dependencias analizadas).

---

## 2026-10-13 a 2026-10-21 — Serie `NUC-H264` (Modularización Nuclear y SOLID)

Proceso de refactorización estructural profunda sobre los archivos monolíticos originales:

1. **`NUC-1-base` (`0c8bf82`)**:
   - Extracción de constantes compartidas a `constantes.rs`.
   - Eliminación de números mágicos en tablas de desescaneo de coeficientes y tamaños de bloque.

2. **`NUC-4a` (`224e81c`)**:
   - Descomposición de `lib.rs` (que contaba con 1.673 líneas) en submódulos atómicos: `poc.rs` (orden de imagen), `dpb.rs` (búfer de imágenes decodificadas), `listas_ref.rs` (ordenación de referencias) y `cabecera_corte.rs`.
   - Reducción del pedantic clippy de 598 a 550 avisos.

3. **`NUC-5` (`a755373`, `3d6037b`)**:
   - Modularización de `deblock.rs` (de 3.594 a 2.058 líneas) e `inter.rs` (de 2.311 a 1.816 líneas).
   - Creación de `deblock/filtrado.rs`, `deblock/tests*` e `inter/tests.rs` (99/99 tests idénticos).
   - Clippy normal de `common` llevado de 20 a 0 avisos.

4. **`NUC-2` (`ca0daa0`)**:
   - Descomposición de `mb_cabac_slice.rs` (2.116 líneas) en `mb_cabac_slice/{mod,estado,p,b,intra}.rs`.
   - Cero pérdida de comentarios de ingeniería y rendimiento dentro de ±1.5% en 5 secuencias de referencia.

5. **`NUC-3` (`3827972`)**:
   - Descomposición de `mb_inter.rs` (4.008 líneas) en 8 módulos especializados (`vecinos.rs`, `residuo.rs`, `p_cavlc.rs`, `p_recon.rs`, `b_mc.rs`, `b_directo.rs`, `b_skip.rs`, `b_cavlc.rs`).
   - Retirada de los 12 atributos `#[allow(too_many_arguments)]`. Unificación de algoritmos de movimiento EDC y compensación de movimiento.

6. **`NUC-4b` (`054e15e`, `fdb0265`, `20fcb06`)**:
   - Documentación rigurosa de estructuras públicas (`ParametrosCuadro`, `CabeceraIntraCavlc`).
   - Deduplicación del análisis de cabecera intra en CAVLC y corrección ortográfica de tildes técnicas.

7. **`NUC-2b` (`7835ae8`, `26c3a78`)**:
   - Unificación de `modo_intra_desde_rem` en `mb_intra/mod.rs` (4 copias de la ecuación 8-42 reducidas a una sola función pura).
   - Modularización de `intra_nxn` en funciones atómicas: `leer_modos_nxn`, `reconstruir_luma_i8` y `reconstruir_luma_i4`.

---

## 2026-10-08 a 2026-10-13 — Serie `COR-1` a `COR-4` (Correcciones de Conformidad UIT-T)

Corrección de defectos semánticos y desviaciones de la norma oficial UIT-T H.264:

- **`COR-1` (Vaciado de macrobloques diferidos)**:
  - Cuando `disable_deblocking_filter_idc = 1`, la reconstrucción diferida de macrobloques saltados no se completaba, dejando los últimos macrobloques de la imagen en negro o corruptos. Arreglado con vaciado forzado al cierre del cuadro.
- **`COR-2` (Filtrado de bloques I_PCM)**:
  - La norma (§8.7.2.2) exige que los bloques `I_PCM` se filtren con un parámetro de cuantización $QP_Y = 0$. El decodificador original heredaba erróneamente el QP del corte.
- **`COR-3` (Parámetros por corte y eliminación de valores inventados)**:
  - Retirada sistemática de respaldos silenciosos con `unwrap_or(0)` que enmascaraban flujos truncados. Sustitución por propagación de errores `DecodeError::FueraDeRango`.
- **`COR-4` (Filtrado de desbloqueo independiente en límites de corte)**:
  - Corrección de la asignación de offsets de desbloqueo (`offset_a`, `offset_b`) y parámetros de activación de filtro cuando una fila de macrobloques abarca múltiples cortes con configuraciones dispares. Validado píxel a píxel contra FFmpeg.
- **Arreglos en cálculo de POC y MMCO 5**:
  - Corrección de desbordamientos en cálculo de POC tipo 1 y preservación de referencias a largo plazo en operaciones MMCO 5.
