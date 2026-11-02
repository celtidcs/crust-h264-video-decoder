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

### D-2026-11-02-1: POC de Tipo 0 después de una MMCO 5
- **Qué falla y cómo se reproduce**: según la norma (UIT-T H.264 06/26, §8.2.1, pág. 140, y §8.2.1.1), tras decodificar una imagen con `memory_management_control_operation` igual a 5, su `TopFieldOrderCnt` pasa a ser `TopFieldOrderCnt − PicOrderCnt` (0 en un fotograma progresivo), y la siguiente imagen toma ese valor como `prevPicOrderCntLsb`. `PocState::apply_mmco5` conserva en su lugar el `pic_order_cnt_lsb` de la imagen. Reproducción: `cargo test -p crust-h264-video-decoder --lib poc::tests::tipo0_tras_mmco5_el_lsb_previo_es_el_top_reajustado -- --ignored` (MaxPicOrderCntLsb 16; imagen con MMCO 5 y lsb 6; la siguiente referencia, con lsb 14, debería dar POC −2 y da 14).
- **Hecho adicional**: el decodificador presenta la imagen con MMCO 5 con su POC anterior al reajuste (el valor que devuelve `apply_mmco5` no se usa).
- **Pendiente de comprobar**: este documento (§3) registra que los flujos de conformidad progresivos coinciden bit a bit con Cisco OpenH264, pero no se ha comprobado cuáles contienen una MMCO 5 con POC de tipo 0, así que eso no descarta nada por sí solo.
- **Sospecha sin confirmar**: que un flujo con MMCO 5, POC de tipo 0 y un `pic_order_cnt_lsb` distinto de 0 en esa imagen se presente en un orden distinto del de la norma.
- **Siguiente paso**: corregir `apply_mmco5` para el tipo 0 según la norma, con un flujo de prueba que lo demuestre frente a un oráculo externo y la red de equivalencia como control de que nada más cambia.

---

## 3. Estado Frente a Oráculos Externos

- **Cisco OpenH264 (v0.9.x / v2.x)**:
  - En los 35 flujos oficiales de conformidad progresiva de la UIT-T, `Crust H264 Video Decoder` es **bit a bit idéntico** a la salida de OpenH264.
- **FFmpeg (libavcodec)**:
  - En las secuencias de prueba con cortes múltiples y transiciones de desbloqueo (`filtro_desplaz.264`, `cabac_mslice_idc2.264`), la reconstrucción es **píxel a píxel idéntica** al decodificador de referencia de FFmpeg.
