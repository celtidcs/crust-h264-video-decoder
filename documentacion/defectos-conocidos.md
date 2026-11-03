# Defectos Conocidos y Decisiones de Diseño — Crust H264 Video Decoder

Este registro formal detalla los límites de diseño conscientes, las excepciones analizadas y el comportamiento ante casos límite, conforme a lo estipulado en el mandato de verificación independiente (§2.3).

---

## 1. Exclusiones de Diseño Conscientes (Reducción de Superficie de Ataque)

Por política deliberada de seguridad y simplicidad, el decodificador rechaza de forma determinista y sin pánico los siguientes elementos sintácticos obsoletos o de extrema complejidad:

1. **Vídeo Entrelazado (Campos Pares / Impares / MBAFF)**:
   - *Motivo*: Diseñado para la televisión de tubo catódico de principios de siglo. En la web moderna y en streaming seguro, prácticamente el 100% del vídeo se codifica en barrido progresivo. Soportar entrelazado duplica la complejidad del filtro de desbloqueo y la compensación de movimiento sin beneficio práctico.
   - *Comportamiento*: Detección en cabecera SPS (`field_pic_flag` o `mb_adaptive_frame_field_flag`) y rechazo inmediato con `DecodeError::UnsupportedProfile`.

2. **Ordenación Flexible de Macrobloques (FMO) y Cortes Arbitrarios (ASO)**:
   - *Motivo*: Características del *Baseline Profile* pensadas para redes móviles con alta pérdida de paquetes mediante mapas de grupos de macrobloques. Su uso real hoy en día es nulo y constituye un vector clásico de desbordamiento en parsers.
   - *Comportamiento*: Rechazo limpio si `num_slice_groups_minus1 > 0`.

3. **Cortes Redundantes (`redundant_pic_cnt`)**:
   - *Motivo*: Redundancia de paquetes en capa de transporte de vídeo, innecesaria sobre protocolos modernos TCP/TLS o QUIC.

---

## 2. Puntos Ciegos y Fichas de Seguimiento

### D-2026-10-21-1: POC de Tipos 1 y 2 tras Imágenes No de Referencia
- **Qué dice la norma** (UIT-T H.264 06/26, §8.2.1.2 y §8.2.1.3, págs. 142–143): `prevFrameNum` y `prevFrameNumOffset` se toman de la **imagen anterior en orden de decodificación**, sea o no de referencia; con huecos en `frame_num`, esa imagen puede ser un fotograma «inexistente» inferido (§8.2.5.2). Para el tipo 0 (§8.2.1.1) la norma sí dice «imagen de **referencia** anterior».
- **Qué hace el código**: `crates/crust-h264-video-decoder/src/poc.rs` solo actualiza ese estado con imágenes de referencia, y el POC se calcula antes de insertar los fotogramas inexistentes de un hueco.
- **Descartado**: sin huecos en `frame_num`, una imagen no de referencia lleva el mismo `frame_num` que la referencia siguiente y las dos fórmulas dan el mismo resultado; con huecos, los fotogramas inferidos son consecutivos y la detección de la vuelta del contador también coincide (comprobado a mano en casos con y sin vuelta).
- **Sospecha sin confirmar**: que algún flujo, conforme u hostil, produzca una salida distinta por esta diferencia de redacción.
- **Siguiente paso**: fabricar un flujo de prueba con huecos y POC de tipo 1 o 2 y compararlo con un decodificador de referencia externo.
- **Cobertura**: pruebas unitarias de la fórmula en `crates/crust-h264-video-decoder/src/poc/tests.rs` (tipos 0, 1 y 2, vuelta del contador e imágenes no de referencia).

### D-2026-11-02-1: POC de Tipo 0 después de una MMCO 5 — CORREGIDO
- **Qué fallaba**: según la norma (UIT-T H.264 06/26, §8.2.1, pág. 140, y §8.2.1.1), tras decodificar una imagen con `memory_management_control_operation` igual a 5, su `TopFieldOrderCnt` pasa a ser `TopFieldOrderCnt − PicOrderCnt` (0 en un fotograma progresivo), y la imagen siguiente toma ese valor como `prevPicOrderCntLsb`. `PocState::apply_mmco5` conservaba en su lugar el `pic_order_cnt_lsb` de la imagen.
- **Corrección**: `apply_mmco5` (tipo 0) guarda ahora como `prevPicOrderCntLsb` el `TopFieldOrderCnt` reajustado. La prueba `crates/crust-h264-video-decoder/src/poc/tests.rs::tipo0_tras_mmco5_el_lsb_previo_es_el_top_reajustado` lo vigila (con el comportamiento anterior falla: da 14 en lugar de −2).
- **Comprobado contra la salida oficial**: los cuatro flujos de conformidad de la UIT-T con MMCO 5 (MR4_TANDBERG_C con POC de tipo 0 y lsb distinto de 0 en esas imágenes; MR5_TANDBERG_C, tipo 1; MR2_TANDBERG_E y MR3_TANDBERG_B, tipo 2) coinciden fotograma a fotograma (300/300) con su YUV oficial antes y después del cambio; la red de equivalencia sigue en 2.132/2.132.
- **Pregunta abierta, sin cambio**: el decodificador presenta la propia imagen con MMCO 5 con su POC anterior al reajuste. Con ese comportamiento, MR4_TANDBERG_C coincide con la salida oficial, así que no se modifica sin un flujo que demuestre lo contrario.

### D-2026-11-02-2: Pánico por desbordamiento en la prueba de movimiento del desbloqueo — CORREGIDO
- **Qué fallaba**: `pk_differs` (`crates/crust-h264-common/src/deblock.rs`), la prueba de movimiento de §8.7.2.1 (`|Δmv| >= 4` en cuartos de píxel), restaba dos componentes `i16` con `-`. En un flujo no conforme la diferencia puede no caber en `i16`, y con las comprobaciones aritméticas activas (compilación de depuración, pruebas, fuzzing) el decodificador entraba en pánico («attempt to subtract with overflow»). Lo encontró el fuzzing de `fuzz_decode_stream` partiendo del corpus `fuzz/base`.
- **Corrección**: la resta es ahora `wrapping_sub`, con vuelta en `i16`. Es lo que ya hacía la compilación de release, así que la salida no cambia, y es lo mismo que hace el núcleo AVX2 de `rusty_h264-accel` (feature `asm`). Se descartó la resta en `i32` porque haría que las compilaciones con y sin `asm` decidieran distinto, y porque el empaquetado ya trunca los vectores a `i16`.
- **Barrera**: `crates/crust-h264-common/src/deblock/tests/desbordamiento_movimiento.rs`. Sin la corrección falla con el mismo pánico, en la misma línea, que el caso del fuzzing.
- **Comprobado**: red de equivalencia 2.132/2.132 idéntica; velocidad igual que antes (razón entre 0,992 y 1,002 en los siete flujos de `vel/`).

### D-2026-11-02-3: El núcleo AVX2 discrepa del escalar cuando la diferencia vuelve a `i16::MIN` — ABIERTO
- **Qué falla**: con `--features asm` en una CPU con AVX2, `rusty_h264-accel::bs_motion_masks` calcula `abs_epi16(sub_epi16(a, b))`, y `abs_epi16(i16::MIN)` vale `i16::MIN`, que no es `> 3`: el núcleo dice «movimiento parecido» donde la versión escalar dice «distinto». Solo ocurre cuando la diferencia con vuelta es exactamente −32768, es decir, con vectores fuera de los límites de la norma (flujos no conformes). No hay pánico; cambia como mucho la fuerza del filtro en esas aristas.
- **Cómo se reproduce**: `cargo test --all-features desbordamiento -- --ignored` en `crates/crust-h264-common` (prueba `despachador_igual_a_escalar_cuando_la_diferencia_vuelve_a_minimo`). Medido en Linux: el despachador da (0, 0) y el escalar (6, 32). Sin `asm`, el despachador es el escalar y la prueba pasa.
- **Descartado**: los demás pares extremos (siete, en la misma prueba sin `#[ignore]`) dan la misma respuesta en el núcleo y en el escalar.
- **Siguiente paso**: decidir si se corrige en el escalar (para que imite al núcleo) o se propone la corrección del núcleo al proyecto original. No se ha hecho porque el núcleo es una dependencia externa y la decisión necesita el acuerdo de los tres agentes.

---

## 3. Estado Frente a Oráculos Externos

- **Cisco OpenH264 (v0.9.x / v2.x)**:
  - En los 35 flujos oficiales de conformidad progresiva de la UIT-T, `Crust H264 Video Decoder` es **bit a bit idéntico** a la salida de OpenH264.
- **FFmpeg (libavcodec)**:
  - En las secuencias de prueba con cortes múltiples y transiciones de desbloqueo (`filtro_desplaz.264`, `cabac_mslice_idc2.264`), la reconstrucción es **píxel a píxel idéntica** al decodificador de referencia de FFmpeg.
