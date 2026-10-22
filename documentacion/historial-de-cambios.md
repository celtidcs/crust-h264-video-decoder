# Historial de Cambios y AuditorÃ­a â€” Crusty_H264

Este documento recoge de forma cronolÃ³gica la evoluciÃ³n del decodificador, documentando las auditorÃ­as mecÃ¡nicas, las correcciones de conformidad UIT-T y la reestructuraciÃ³n arquitectÃ³nica de la base de cÃ³digo.

---

## 2026-10-22 â€” EjecuciÃ³n de Pruebas Pendientes, IncorporaciÃ³n Full HD 1080p y VerificaciÃ³n MultirresoluciÃ³n

- **Mandato**: ResoluciÃ³n del Ã­tem pendiente del mandato del Director: incorporaciÃ³n de secuencias Full HD (1080p) al banco de velocidad (`vel/`) y ejecuciÃ³n de pruebas de rendimiento comparativas (720p, 1080p, 4K) frente a orÃ¡culos externos.
- **Acciones Realizadas**:
  - CreaciÃ³n de secuencias Full HD normalizadas (`vel/safari1080.264` a 1920x1036 y `vel/safari1080p.264` a 1920x1080) con perfil High, B-frames piramidales, CABAC y ponderaciÃ³n.
  - ActualizaciÃ³n del arnÃ©s de evaluaciÃ³n de rendimiento `velocidad.py` incorporando las secuencias Full HD y 4K.
  - MediciÃ³n comparativa frente al decodificador C/C++ y ensamblador de Cisco OpenH264:
    - **720p**: Crusty_H264 1.162 ms (~103 fps) vs OpenH264 1.217 ms (Crusty 4.5% mÃ¡s rÃ¡pido).
    - **1080p**: Crusty_H264 3.285 ms (~36.5 fps) vs OpenH264 3.648 ms (Crusty 10.0% mÃ¡s rÃ¡pido).
    - **4K UHD**: Crusty_H264 15.728 ms (~7.6 fps) vs OpenH264 18.124 ms (Crusty 13.2% mÃ¡s rÃ¡pido).
  - VerificaciÃ³n estricta de las cuatro puertas mecÃ¡nicas (Â§1.1):
    - `cargo test`: 99 pruebas en `common`, 55 en `decoder`, 7 de integraciÃ³n por corte y 7 en `banco_h264` (100% VERDE).
    - `cargo clippy --all-targets -- -D warnings`: 0 avisos.
    - `cargo fmt --check`: 0 diferencias.
    - `cargo audit`: 0 vulnerabilidades (35 dependencias escaneadas).
    - `contar_panicos.py`: 0 puntos de pÃ¡nico en cÃ³digo de producciÃ³n (`common: 0`, `decoder: 0`).
  - DocumentaciÃ³n de la tabla de rendimiento multirresoluciÃ³n en `documentacion/funcionalidades.md`.

---

## 2026-10-22 â€” CreaciÃ³n e IndependizaciÃ³n del Proyecto Crusty_H264

- **Mandato**: Orden directa del Director de escindir el decodificador H.264 seguro desarrollado en Crusty_H264 para constituir un proyecto autÃ³nomo e independiente en `proyecto/crusty_h264`.
- **Estructura Creada**:
  - Crates consolidados: `rusty_h264-common-0.16.0` y `rusty_h264-decoder-0.16.0`.
  - Banco de pruebas CLI (`banco_h264`), suite de mutaciones y scripts de verificaciÃ³n (`red.py`, `velocidad.py`, `contar_panicos.py`).
  - IntegraciÃ³n del mÃ³dulo `reordenacion.rs` dentro de la librerÃ­a del decodificador.
  - ImplementaciÃ³n del sistema de gobernanza y coordinaciÃ³n tripartita (`.gestor/canal/`, `AGENTS.md`, `CLAUDE.md`, `GEMINI.md`).
  - Suite de documentaciÃ³n canÃ³nica en espaÃ±ol con prosa humana y diagramas explicativos.
- **Evidencia MecÃ¡nica**:
  - `cargo check`: salida 0.
  - `cargo test`: 99 pruebas de common, 55 unitarias de decoder, 7 pruebas de reordenaciÃ³n y 7 pruebas de integraciÃ³n por corte, todas en VERDE.
  - `cargo clippy --all-targets -- -D warnings`: 0 avisos.
  - `cargo fmt --check`: 0 diferencias.
  - `cargo audit`: 0 vulnerabilidades (35 dependencias analizadas).

---

## 2026-10-13 a 2026-10-21 â€” Serie `NUC-H264` (ModularizaciÃ³n Nuclear y SOLID)

Proceso de refactorizaciÃ³n estructural profunda sobre los archivos monolÃ­ticos originales:

1. **`NUC-1-base` (`0c8bf82`)**:
   - ExtracciÃ³n de constantes compartidas a `constantes.rs`.
   - EliminaciÃ³n de nÃºmeros mÃ¡gicos en tablas de desescaneo de coeficientes y tamaÃ±os de bloque.

2. **`NUC-4a` (`224e81c`)**:
   - DescomposiciÃ³n de `lib.rs` (que contaba con 1.673 lÃ­neas) en submÃ³dulos atÃ³micos: `poc.rs` (orden de imagen), `dpb.rs` (bÃºfer de imÃ¡genes decodificadas), `listas_ref.rs` (ordenaciÃ³n de referencias) y `cabecera_corte.rs`.
   - ReducciÃ³n del pedantic clippy de 598 a 550 avisos.

3. **`NUC-5` (`a755373`, `3d6037b`)**:
   - ModularizaciÃ³n de `deblock.rs` (de 3.594 a 2.058 lÃ­neas) e `inter.rs` (de 2.311 a 1.816 lÃ­neas).
   - CreaciÃ³n de `deblock/filtrado.rs`, `deblock/tests*` e `inter/tests.rs` (99/99 tests idÃ©nticos).
   - Clippy normal de `common` llevado de 20 a 0 avisos.

4. **`NUC-2` (`ca0daa0`)**:
   - DescomposiciÃ³n de `mb_cabac_slice.rs` (2.116 lÃ­neas) en `mb_cabac_slice/{mod,estado,p,b,intra}.rs`.
   - Cero pÃ©rdida de comentarios de ingenierÃ­a y rendimiento dentro de Â±1.5% en 5 secuencias de referencia.

5. **`NUC-3` (`3827972`)**:
   - DescomposiciÃ³n de `mb_inter.rs` (4.008 lÃ­neas) en 8 mÃ³dulos especializados (`vecinos.rs`, `residuo.rs`, `p_cavlc.rs`, `p_recon.rs`, `b_mc.rs`, `b_directo.rs`, `b_skip.rs`, `b_cavlc.rs`).
   - Retirada de los 12 atributos `#[allow(too_many_arguments)]`. UnificaciÃ³n de algoritmos de movimiento EDC y compensaciÃ³n de movimiento.

6. **`NUC-4b` (`054e15e`, `fdb0265`, `20fcb06`)**:
   - DocumentaciÃ³n rigurosa de estructuras pÃºblicas (`ParametrosCuadro`, `CabeceraIntraCavlc`).
   - DeduplicaciÃ³n del anÃ¡lisis de cabecera intra en CAVLC y correcciÃ³n ortogrÃ¡fica de tildes tÃ©cnicas.

7. **`NUC-2b` (`7835ae8`, `26c3a78`)**:
   - UnificaciÃ³n de `modo_intra_desde_rem` en `mb_intra/mod.rs` (4 copias de la ecuaciÃ³n 8-42 reducidas a una sola funciÃ³n pura).
   - ModularizaciÃ³n de `intra_nxn` en funciones atÃ³micas: `leer_modos_nxn`, `reconstruir_luma_i8` y `reconstruir_luma_i4`.

---

## 2026-10-08 a 2026-10-13 â€” Serie `COR-1` a `COR-4` (Correcciones de Conformidad UIT-T)

CorrecciÃ³n de defectos semÃ¡nticos y desviaciones de la norma oficial UIT-T H.264:

- **`COR-1` (Vaciado de macrobloques diferidos)**:
  - Cuando `disable_deblocking_filter_idc = 1`, la reconstrucciÃ³n diferida de macrobloques saltados no se completaba, dejando los Ãºltimos macrobloques de la imagen en negro o corruptos. Arreglado con vaciado forzado al cierre del cuadro.
- **`COR-2` (Filtrado de bloques I_PCM)**:
  - La norma (Â§8.7.2.2) exige que los bloques `I_PCM` se filtren con un parÃ¡metro de cuantizaciÃ³n $QP_Y = 0$. El decodificador original heredaba errÃ³neamente el QP del corte.
- **`COR-3` (ParÃ¡metros por corte y eliminaciÃ³n de valores inventados)**:
  - Retirada sistemÃ¡tica de respaldos silenciosos con `unwrap_or(0)` que enmascaraban flujos truncados. SustituciÃ³n por propagaciÃ³n de errores `DecodeError::FueraDeRango`.
- **`COR-4` (Filtrado de desbloqueo independiente en lÃ­mites de corte)**:
  - CorrecciÃ³n de la asignaciÃ³n de offsets de desbloqueo (`offset_a`, `offset_b`) y parÃ¡metros de activaciÃ³n de filtro cuando una fila de macrobloques abarca mÃºltiples cortes con configuraciones dispares. Validado pÃ­xel a pÃ­xel contra FFmpeg.
- **Arreglos en cÃ¡lculo de POC y MMCO 5**:
  - CorrecciÃ³n de desbordamientos en cÃ¡lculo de POC tipo 1 y preservaciÃ³n de referencias a largo plazo en operaciones MMCO 5.
