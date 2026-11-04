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

### D-2026-10-21-1: POC de Tipos 1 y 2 tras Imágenes No de Referencia y Huecos de frame_num — CORREGIDO
- **Qué fallaba**: según la norma (UIT-T H.264 06/26, §8.2.1.2 y §8.2.1.3, págs. 118–119), `prevFrameNum` y `prevFrameNumOffset` se toman de la **imagen anterior en orden de decodificación**, sea o no de referencia; con huecos en `frame_num`, esa imagen puede ser un fotograma «inexistente» inferido (§8.2.5.2). Para el tipo 0 (§8.2.1.1, pág. 118) la norma sí dice «imagen de **referencia** anterior». `crates/crust-h264-video-decoder/src/poc.rs` solo actualizaba ese estado con imágenes de referencia, y el POC se calculaba en `lib.rs` antes de procesar los fotogramas inexistentes de un hueco.
- **Corrección**:
  1. `PocState::compute_poc` actualiza secuencialmente `prev_frame_num` y `prev_frame_num_offset` con imágenes no de referencia en tipos 1 y 2 (§8.2.1.2, §8.2.1.3).
  2. En `lib.rs`, `insert_frame_num_gaps` se ejecuta inmediatamente antes de `compute_poc` (y ambos tras el descarte de cortes redundantes con `redundant_pic_cnt != 0`, UIT-T H.264 cláusula 3 definición 3.132, pág. 13 de la norma, PDF pág. 37, y §7.4.3), garantizando que las imágenes inexistentes inferidas actualicen el estado secuencial antes del cálculo del POC de la imagen actual.
- **Barreras y verificación**:
  - Pruebas unitarias en `crates/crust-h264-video-decoder/src/poc/tests.rs`: `tipo1_no_conforme_no_ref_avanza_prev_fn`, `tipo2_no_conforme_no_ref_avanza_prev_fn`, `tipo1_no_ref_avanza_prev_fn_offset`, `tipo1_mmco5_seguida_de_no_ref_y_ref`, `tipo2_mmco5_seguida_de_no_ref_y_ref`, `tipo2_vuelta_contador_inexistentes_tras_no_ref`.
  - Pruebas de integración en `crates/crust-h264-video-decoder/src/lib.rs`: `huecos_procesados_antes_de_compute_poc_en_imagen_actual`, `corte_redundante_no_avanza_poc_ni_dpb`, `e_oraculo_end_to_end_tipo0`, `e_oraculo_end_to_end_tipo1`, `e_oraculo_end_to_end_tipo1_acordado`.
  - Verificado mecánicamente mediante las pruebas unitarias y de integración citadas, que vigilan la derivación secuencial de POC tras imágenes no de referencia, la propagación de prev_frame_num_offset y el descarte previo de cortes redundantes.

### D-2026-11-04-1: Fotogramas inexistentes con POC incorrecto y presencia indebida en listas B con pic_order_cnt_type 0 — CORREGIDO
- **Qué fallaba**:
  1. En `crates/crust-h264-video-decoder/src/dpb.rs`, `insert_frame_num_gaps` insertaba fotogramas inexistentes marcados con `poc: 0` fijo sin derivar su POC. Según UIT-T H.264 (06/2026) §8.2.5.2 (pág. 132), cuando `pic_order_cnt_type != 0`, se debe invocar el proceso §8.2.1 para cada fotograma inexistente con `frame_num = UnusedShortTermFrameNum`, `nal_ref_idc != 0`, IDR = 0 y deltas = 0.
  2. Según §8.2.4.2.3 (pág. 126), cuando `pic_order_cnt_type == 0`, los fotogramas inexistentes **no deben incluirse** en `RefPicList0` ni en `RefPicList1` en cortes B. En `crates/crust-h264-video-decoder/src/listas_ref.rs` (`build_ref_list_b`), no se discriminaban los fotogramas inexistentes, incorporándose indebidamente con POC 0 en `less`, `greater` o `long`.
  3. En flujos no conformes donde `frame_num == PrevRefFrameNum`, `insert_frame_num_gaps` calculaba indebidamente un hueco completo de `MaxFrameNum - 1` fotogramas, omitiendo la condición de invocación de §8.2.5.2 («This process is invoked when frame_num is not equal to PrevRefFrameNum and is not equal to ( PrevRefFrameNum + 1 ) % MaxFrameNum»).
- **Corrección**:
  1. `insert_frame_num_gaps` ahora recibe `&Sps` obligatorio y deriva el POC de cada fotograma inexistente invocando `compute_poc` secuencialmente con `nal_ref_idc = 1` cuando `pic_order_cnt_type != 0`. Para `pic_order_cnt_type == 0`, se asigna POC 0 sin invocar `compute_poc`, preservando invariante de `prev_poc_msb`/`prev_poc_lsb`.
  2. En `listas_ref.rs`, `build_ref_list_b` unifica el filtrado de candidatos (`candidatos = dpb.iter().filter(|r| !(filtrar_inexistentes && r.is_non_existing()))`), excluyendo de forma estricta los fotogramas inexistentes de `less`, `greater` y `long` cuando `pic_order_cnt_type == 0`.
  3. Guarda en `insert_frame_num_gaps`: `if max_fn == 0 || frame_num == self.prev_ref_frame_num { return; }` conforme a la condición de invocación de §8.2.5.2.
  4. Coste aritmético acotado en F: en el DPB solo se materializan los últimos `n = min(gap, cap) <= 16` fotogramas. Como `gap < MaxFrameNum`, la secuencia de inexistentes contiene a lo sumo una vuelta de `MaxFrameNum`; si ocurre en el tramo omitido, el primer inexistente materializado `u = (start + skip_count) % MaxFrameNum` cumple `u < prev_frame_num`, detectando la vuelta de forma idéntica. Por tanto, basta evaluar `compute_poc` únicamente en los `n` fotogramas materializados (`n <= 16`), reduciendo el coste aritmético de derivación de POC a `O(min(gap, cap) * cycle) <= 16 * 255 = 4080` sumas (la función completa incluye además la asignación de planos neutros para los fotogramas materializados en el DPB según la resolución de la secuencia).
- **Barreras y verificación**:
  - Pruebas unitarias: `repro_tipo0_inexistentes_fuera_de_lista_b`, `repro_tipo2_poc_de_inexistentes`, `tipo0_hueco_no_invoca_compute_poc_para_preservar_msb_lsb`, `tipo0_inexistentes_excluidos_de_greater_y_l1`, `tipo0_inexistentes_excluidos_de_long_si_estuviesen_marcados`, `g_flujo_no_conforme_guarda_fn_igual_prev_ref_fn`.
  - Verificado mecánicamente mediante la suite de pruebas de comportamiento que aíslan la presencia de inexistentes en listas L0/L1, la derivación secuencial de POC y la preservación del estado de tipo 0.

### D-2026-11-02-1: POC de Tipo 0 después de una MMCO 5 — CORREGIDO
- **Qué fallaba**: según la norma (UIT-T H.264 06/26, §8.2.1, pág. 140, y §8.2.1.1), tras decodificar una imagen con `memory_management_control_operation` igual a 5, su `TopFieldOrderCnt` pasa a ser `TopFieldOrderCnt − PicOrderCnt` (0 en un fotograma progresivo), y la imagen siguiente toma ese valor como `prevPicOrderCntLsb`. `PocState::apply_mmco5` conservaba en su lugar el `pic_order_cnt_lsb` de la imagen.
- **Corrección**: `apply_mmco5` (tipo 0) guarda ahora como `prevPicOrderCntLsb` el `TopFieldOrderCnt` reajustado. La prueba `crates/crust-h264-video-decoder/src/poc/tests.rs::tipo0_tras_mmco5_el_lsb_previo_es_el_top_reajustado` lo vigila (con el comportamiento anterior falla: da 14 en lugar de −2).
- **Comprobado contra la salida oficial**: los cuatro flujos de conformidad de la UIT-T con MMCO 5 (MR4_TANDBERG_C con POC de tipo 0 y lsb distinto de 0 en esas imágenes; MR5_TANDBERG_C, tipo 1; MR2_TANDBERG_E y MR3_TANDBERG_B, tipo 2) coinciden fotograma a fotograma (300/300) con su YUV oficial antes y después del cambio; la red de equivalencia sigue en 2.132/2.132.
- **Pregunta abierta, sin cambio**: el decodificador presenta la propia imagen con MMCO 5 con su POC anterior al reajuste. Con ese comportamiento, MR4_TANDBERG_C coincide con la salida oficial, así que no se modifica sin un flujo que demuestre lo contrario.

### D-2026-11-02-2: Pánico por desbordamiento en la prueba de movimiento del desbloqueo — CORREGIDO
- **Qué fallaba**: `pk_differs` (`crates/crust-h264-common/src/deblock.rs`), la prueba de movimiento de §8.7.2.1 (`|Δmv| >= 4` en cuartos de píxel), restaba dos componentes `i16` con `-`. En un flujo no conforme la diferencia puede no caber en `i16`, y con las comprobaciones aritméticas activas (compilación de depuración, pruebas, fuzzing) el decodificador entraba en pánico («attempt to subtract with overflow»). Lo encontró el fuzzing de `fuzz_decode_stream` partiendo del corpus `fuzz/base`.
- **Corrección**: la resta es ahora `wrapping_sub`, con vuelta en `i16`. Es lo que ya hacía la compilación de release, así que la salida no cambia. Desde la corrección de D-2026-11-02-3 los vectores empaquetados están saturados y su diferencia nunca llega a dar la vuelta; `wrapping_sub` se mantiene para que un registro empaquetado construido de otra forma tampoco pueda provocar el pánico.
- **Barreras**: `crates/crust-h264-common/src/deblock/tests/desbordamiento_movimiento.rs`. Sin la corrección falla con el mismo pánico, en la misma línea, que el caso del fuzzing.
- **Comprobado**: red de equivalencia 2.132/2.132 idéntica; velocidad igual que antes (razón entre 0,992 y 1,002 en los siete flujos de `vel/`).

### D-2026-11-02-3: El núcleo AVX2 discrepaba del escalar cuando la diferencia volvía a `i16::MIN` — CORREGIDO
- **Qué fallaba**: con `--features asm` en una CPU con AVX2, `rusty_h264-accel::bs_motion_masks` calcula `abs_epi16(sub_epi16(a, b))`, y `abs_epi16(i16::MIN)` vale `i16::MIN`, que no es `> 3`: el núcleo decía «movimiento parecido» donde la versión escalar decía «distinto». La causa de fondo estaba en `pack_mb`, que **truncaba** a `i16` (`as i16`) los vectores de movimiento, guardados en `i32`: un vector no conforme se convertía en otro arbitrario (40000 → −25536), y la diferencia de dos vectores truncados podía valer `i16::MIN`. Además, la misma prueba «|Δmv| ≥ 4» se hacía de tres maneras distintas (resta en `i16` con vuelta en el camino empaquetado de una lista, resta en `i32` en el de dos listas y en el camino de baldosas, y el núcleo AVX2), y la resta en `i32` del camino de baldosas podía desbordar con vectores cercanos a `i32::MIN`/`i32::MAX`. Solo con flujos no conformes; en ese caso la salida podía depender de la CPU.
- **Corrección**: los vectores que entran en la derivación de intensidades de borde se **saturan** a `[-16384, 16383]` cuartos de muestra (`COTA_MV_DESBLOQUEO` = 2^14, función `saturar_componente_mv`), tanto en `pack_mb` como en el camino de baldosas (`Blk::load`). UIT-T H.264, Anexo A (§A.3.1), limita un flujo conforme a `[-2048, 2047.75]` muestras en horizontal y a `[-512, 511.75]` en vertical (el `MaxVmvR` máximo de la tabla A-1), es decir `[-8192, 8191]` y `[-2048, 2047]` cuartos: la saturación nunca cambia un flujo conforme. La diferencia de dos componentes saturados cae en `[-32767, 32767]`: cabe en `i16` sin vuelta, nunca vale `i16::MIN` y ninguna resta desborda, así que los cuatro caminos deciden igual.
- **Barrera**: `crates/crust-h264-common/src/deblock/tests/saturacion_vectores.rs` (empaquetado, camino de baldosas frente a camino empaquetado y despachador frente a escalar con vectores `i32` extremos, en una y dos listas). La antigua prueba con `#[ignore]` se ha trasladado ahí y ya no se ignora. Las siete mutaciones de control (quitar la saturación en cada uno de los cuatro campos empaquetados y en los dos pares de `Blk::load`, o ampliar la cota a 2^15 − 1) la ponen en rojo; las dos de `Blk::load` reproducen además el desbordamiento en `i32`.
- **Comprobado**: red de equivalencia 2.132/2.132 idéntica frente a la versión anterior, y 2.132/2.132 idéntica entre la compilación sin `asm` y con `asm` (AVX2). Ninguna entrada de la red reproducía la discrepancia: solo aparece con vectores extremos, los que cubre la barrera. Coste medido con contador de instrucciones: entre un 0,24 % y un 0,40 % más de instrucciones en los siete flujos de `vel/`, por debajo del ruido del tiempo de ejecución medido en la máquina de pruebas.

---

## 3. Estado Frente a Oráculos Externos

- **Cisco OpenH264 (v0.9.x / v2.x)**:
  - En los 35 flujos oficiales de conformidad progresiva de la UIT-T, `Crust H264 Video Decoder` es **bit a bit idéntico** a la salida de OpenH264.
- **FFmpeg (libavcodec)**:
  - En las secuencias de prueba con cortes múltiples y transiciones de desbloqueo (`filtro_desplaz.264`, `cabac_mslice_idc2.264`), la reconstrucción es **píxel a píxel idéntica** al decodificador de referencia de FFmpeg.
