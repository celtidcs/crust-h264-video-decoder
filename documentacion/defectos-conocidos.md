# Defectos Conocidos y Decisiones de Diseño — Crusty_H264

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

### D-2026-10-21-1: POC de Tipo 2 en Imágenes No de Referencia
- **Qué ocurre**: En el cálculo de POC de tipo 2 (§8.2.1.3), las imágenes no de referencia (`nal_ref_idc == 0`) deben derivar su POC a partir de la última imagen de referencia previa menos un desplazamiento. La batería de 2.129 flujos no incluye secuencias sintéticas extremas que mezclen tipo 2 continuo con fotogramas desechables no ordenados.
- **Evidencia actual**: La red de 2.129 secuencias está al 100% en verde. Ningún flujo real conocido ha mostrado divergencia.
- **Mitigación**: Los flujos reales de producción en MP4 y WebM emplean casi universalmente POC de tipo 0 (con `pic_order_cnt_lsb`) o tipo 1.

---

## 3. Estado Frente a Oráculos Externos

- **Cisco OpenH264 (v0.9.x / v2.x)**:
  - En los 35 flujos oficiales de conformidad progresiva de la UIT-T, `Crusty_H264` es **bit a bit idéntico** a la salida de OpenH264.
- **FFmpeg (libavcodec)**:
  - En las secuencias de prueba con cortes múltiples y transiciones de desbloqueo (`filtro_desplaz.264`, `cabac_mslice_idc2.264`), la reconstrucción es **píxel a píxel idéntica** al decodificador de referencia de FFmpeg.
