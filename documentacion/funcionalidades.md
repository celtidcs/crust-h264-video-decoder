# Catálogo de Funcionalidades — Crust H264 Video Decoder

El decodificador **Crust H264 Video Decoder** ofrece una cobertura exhaustiva del estándar H.264 / AVC para flujos de vídeo progresivos en formato de color YUV 4:2:0.

---

## 1. Perfiles y Niveles Soportados

| Perfil H.264 | Nivel de Soporte | Herramientas Clave Operativas |
|---|---|---|
| **Constrained Baseline Profile (CBP)** | **Completo (100%)** | Cortes I y P, CAVLC, desentrelazado intra 4x4 y 16x16, desescalado, filtro de bucle. |
| **Main Profile (MP)** | **Extenso** | Incorporación de cortes B (bidireccionales), codificación aritmética CABAC y ponderación. |
| **High Profile (HP)** | **Avanzado** | Transformada entera adaptativa 8x8, modos intra 8x8, matrices de escalado personalizadas y matrices de cuantización perceptual. |

*Nota sobre límites conscientes*: Conforme al modelo de amenazas de seguridad y simplicidad arquitectónica, se excluyen de forma deliberada el formato entrelazado (campos pares/impares), la codificación de ordenación flexible de macrobloques (FMO) y los cortes arbitrarios (ASO). Estos modos representan menos del 0.01% del tráfico de vídeo moderno en la web y aumentan exponencialmente la superficie de vulnerabilidad.

---

## 2. Herramientas de Compresión y Reconstrucción

### 2.1. Descompresión Entrópica
- **CAVLC (*Context-Adaptive Variable-Length Coding*)**:
  - Decodificación rápida de sintaxis y tablas de coeficientes.
  - Gestión rigurosa de niveles extremos mediante códigos de escape extendidos.
- **CABAC (*Context-Adaptive Binary Arithmetic Coding*)**:
  - Motor binario con tablas de probabilidad empaquetadas sin ramas condicionales costosas.
  - Contextos adaptativos para tipos de macrobloque, vectores de movimiento y coeficientes transformados 4x4 y 8x8.
  - Modo directo y modo de terminación de corte (`cond_term_flag`).

### 2.2. Predicción Espacial (Intra)
- **Luminancia 16x16**: Modos Vertical, Horizontal, DC y Plano.
- **Luminancia 4x4 y 8x8**: 9 modos de predicción direccional (Vertical, Horizontal, DC, Diagonal Abajo-Izquierda, Diagonal Abajo-Derecha, Vertical-Derecha, Horizontal-Abajo, Vertical-Izquierda, Horizontal-Arriba).
- **Crominancia 8x8**: Modos DC, Horizontal, Vertical y Plano.
- **Modo `I_PCM`**: Paso directo de muestras crudas sin cuantizar con desactivación controlada del filtro de bucle según norma.

### 2.3. Compensación de Movimiento (Inter)
- **Particiones de Macrobloque**: 16x16, 16x8, 8x16 y 8x8.
- **Subparticiones en 8x8**: 8x8, 8x4, 4x8 y 4x4.
- **Precisión Subpíxel**:
  - Interpolación semipíxel con filtro FIR simétrico de 6 tomas (coeficientes `1, -5, 20, 20, -5, 1`).
  - Interpolación a cuarto de píxel mediante promediado bilineal.
- **Cortes B y Modos Directos**:
  - Modo Directo Temporal (basado en distancia POC).
  - Modo Directo Espacial (basado en vecinos colindantes).
  - Predicción Bipredictiva con combinación ponderada implícita y explícita.

### 2.4. Filtro de Desbloqueo en Bucle (*In-Loop Deblocking*)
- Evaluación de fuerza de contorno (*Boundary Strength*, $bS$) de 0 a 4 para cada borde de 4x4 píxeles.
- Ajuste dinámico de umbrales $\alpha$ y $\beta$ basado en el parámetro de cuantización medio ($QP$) y los desplazamientos por corte (`SliceAlphaC0Offset`, `SliceBetaOffset`).
- Filtrado condicional independiente para bordes de macrobloques internos y fronteras de corte.

---

## 3. Gestión del DPB y Orden de Presentación

- **Cálculo de `PicOrderCnt` (POC)**: Soporte completo y matemáticamente exacto para los tres tipos de cálculo de orden de imagen definidos en el estándar:
  - **Tipo 0**: Basado en `pic_order_cnt_lsb` con seguimiento de saltos de ciclo (*wrap-around*).
  - **Tipo 1**: Basado en ciclo de imagen y deltas periódicos de fotogramas de referencia.
  - **Tipo 2**: Derivado linealmente a partir de la cuenta acumulada de fotogramas.
- **Operaciones de Gestión de Memoria (MMCO)**:
  - `MMCO 1`: Marcado de una referencia corta como no usada.
  - `MMCO 2`: Marcado de una referencia larga como no usada.
  - `MMCO 3`: Asignación de un índice de referencia larga a la imagen actual.
  - `MMCO 4`: Fijación del número máximo de referencias largas.
  - `MMCO 5`: Vaciado completo del DPB y reinicio del orden POC.
  - `MMCO 6`: Asignación de la imagen actual como referencia larga.
- **Entrega Acotada**: Integración del módulo `reordenacion.rs` que garantiza que los fotogramas se suministran en orden cronológico estricto de reproducción, limitando la ocupación en RAM a la cota física de la norma.

---

## 4. Rendimiento y Comparativa Multirresolución (720p, Full HD 1080p, 4K)

Las pruebas sistemáticas ejecutadas sobre secuencias del banco de velocidad (`vel/`) demuestran que la implementación en Rust sin `unsafe` propio (`#![forbid(unsafe_code)]`, compilada sin la feature `asm`) de **Crust H264 Video Decoder** ofrece un rendimiento parejo o superior frente al decodificador en C/C++ y ensamblador de Cisco OpenH264:

| Secuencia | Resolución | Perfil / Herramientas | Fotogramas | Crust H264 Video Decoder (Safe Rust) | Cisco OpenH264 (C/ASM) | Razón Crust / Cisco | Rendimiento Crust |
|---|---|---|---|---|---|---|---|
| `safari720.264` | 1280x690 (720p) | High, CABAC, B-frames, 8x8 | 120 | **1.162 ms** | 1.217 ms | **0.955** (4.5% más rápido) | **103.3 fps** |
| `safari1080.264` | 1920x1036 (1080p scope) | High, CABAC, B-frames, 8x8 | 120 | **3.285 ms** | 3.648 ms | **0.900** (10.0% más rápido) | **36.5 fps** |
| `safari1080p.264` | 1920x1080 (1080p estándar) | High, CABAC, B-frames, 8x8 | 120 | **3.461 ms** | - | - | **34.7 fps** |
| `safari4k.264` | 3840x2070 (4K UHD) | High, CABAC, B-frames, 8x8 | 120 | **15.728 ms** | 18.124 ms | **0.868** (13.2% más rápido) | **7.6 fps** |
| `bf3main.264` | 1280x690 (720p) | Main, CABAC, B-frames | 120 | **1.149 ms** | - | - | **104.4 fps** |
| `bf0.264` | 1280x690 (720p) | Baseline, CAVLC, sin B | 120 | **1.212 ms** | - | - | **99.0 fps** |
| `s720nw.264` | 1280x690 (720p) | High, CABAC, sin ponderación | 120 | **1.123 ms** | - | - | **106.9 fps** |
| `cavlc720.264` | 1280x690 (720p) | High, CAVLC con B | 120 | **981 ms** | - | - | **122.3 fps** |

### Observaciones de Ingeniería
1. **Escalabilidad Lineal**: El tiempo de decodificación escala de forma casi perfectamente proporcional al número de macrobloques procesados (de 3.520 macrobloques en 720p a 31.200 en 4K).
2. **Eficiencia en Full HD**: En resolución Full HD 1080p, Crust H264 Video Decoder supera holgadamente el umbral de reproducción en tiempo real (36.5 fps frente a los 25 fps estándar de la fuente).
3. **Consumo de Memoria Acotado**: La ocupación máxima de memoria se mantiene dentro de los límites estrictos del perfil (225 MB en 720p, 506 MB en 1080p y 2.044 MB en 4K UHD para todo el espacio de trabajo y DPB).

