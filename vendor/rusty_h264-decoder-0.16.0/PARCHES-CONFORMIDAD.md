# Copia corregida de `rusty_h264-decoder` 0.16.0

Esta carpeta es `rusty_h264-decoder` 0.16.0 tal como se publica en crates.io
(https://crates.io/crates/rusty_h264-decoder, repositorio https://github.com/remade-with-rust/rusty_h264),
con su licencia BSD-2-Clause y su aviso de copyright intactos, más ocho correcciones. Cada línea
cambiada lleva la marca `PARCHE-CRUSTY`. Ninguna de ellas está en la rama principal del autor a fecha
de 2026-10-08 (comprobado en el commit `5eeec05` del 2026-10-03).

Todas se midieron con los flujos de conformidad oficiales de la UIT-T (H.264.1,
https://www.itu.int/wftp3/av-arch/jvt-site/draft_conformance/): el flujo falla antes del cambio,
sale idéntico a la reconstrucción oficial después, y el resto de la batería no cambia.

1. **Vaciado al cerrar la imagen** (`lib.rs`, `mb16.rs`: `flush_deferred`). Las reconstrucciones
   diferidas de los macrobloques saltados solo se completaban dentro del filtro de desbloqueo, así
   que con `disable_deblocking_filter_idc = 1` los últimos macrobloques de la imagen quedaban sin
   reconstruir. Arregla NL2_Sony_H, SVA_CL1_E, SVA_NL2_E, CANL2_Sony_E, CANL3_SVA_B, AUD_MW_E,
   CANL3_Sony_C, CANL4_SVA_B y NL3_SVA_E. Con los arreglos 1 a 4 juntos salen también exactos
   CAPM3_Sony_D, HCAFR2_HHI_A y HCAFR3_HHI_A (no se midió cuál de los cuatro los arregla).
2. **QP de los I_PCM en el desbloqueo** (`mb16.rs`). Un I_PCM se filtra con QPY = 0 (§8.7.2.2);
   se guardaba el QP del corte. Arregla CAPCM1_Sand_E.
3. **Marcado de referencias por imagen, no por corte** (`lib.rs`). El `dec_ref_pic_marking` se
   acumulaba una vez por corte; con varios cortes, un MMCO 3 repetido borraba la referencia a largo
   plazo recién creada. Arregla MR1_BT_A (junto con el 4).
4. **POC de tipo 1** (`params.rs`, `lib.rs`, `frame_mt.rs`). Se aproximaba con `frame_num * 2` y la
   salida se desordenaba cuando `frame_num` daba la vuelta; ahora sigue §8.2.1.2.
5. **Recorte izquierdo y superior** (`params.rs`, `lib.rs`, `mb16.rs`). Solo se aplicaban el derecho
   y el inferior. Además, la validación del recorte sumaba en `u32`, que en compilación release
   podía dar la vuelta con un flujo hostil; ahora suma en `u64`. Arregla CVFC1_Sony_C.
6. **Intercambio de la lista 1** (`lib.rs`). La regla «si la lista 1 es igual a la 0, intercambiar
   sus dos primeras entradas» (§8.2.4.2.3) se evaluaba sobre las listas recortadas y exigía más de
   una referencia activa; se evalúa sobre las listas iniciales completas. Arregla HCHP2_HHI_A y
   HCMP1_HHI_A (fallaba el último fotograma B, sin referencias futuras).

7. **Esquina de la predicción intra 8×8** (`mb16.rs`, `gather_i8`). La muestra p[−1,−1] solo se
   daba por disponible si también lo estaba el bloque superior; la norma solo exige que el bloque de
   la esquina esté en el mismo corte y sea intra cuando hay `constrained_intra_pred` (§6.4.11.4,
   §8.3.2.2.1). Arregla HCHP1_HHI_B. Causa hallada por Gemini.

8. **Referencias tras MMCO 5 y en el filtro de bordes** (`lib.rs`, `mb16.rs`). Tres causas:
   el tope de la lista de referencias recortaba por el final y podía tirar referencias a largo
   plazo (ahora expulsa la corta más antigua); tras un MMCO 5 no se reiniciaba el estado del POC
   (tipos 0, 1 y 2) ni se cerraba el grupo de imágenes; y el filtro de bordes identificaba cada
   referencia por su POC, que se repite entre secuencias, así que dos imágenes distintas con el
   mismo POC parecían la misma (§8.7.2.1). Ahora cada referencia lleva un identificador propio.
   Arregla MR3_TANDBERG_B, MR4_TANDBERG_C y MR5_TANDBERG_C. Causa y cambio hallados por Gemini;
   de su parche se excluyó una modificación ajena a estos flujos (`abl_deblock`).

9. **Cortes con listas de referencias distintas** (`lib.rs`, `mb16.rs`, `mb_deblock.rs`, en la
   refactorización; véase abajo). Hallado por Crusty_H264, no por un flujo de la UIT-T: ninguno de los
   que se aceptan tiene cortes de una misma imagen con listas distintas.

Con las ocho, los 95 flujos progresivos 4:2:0 de la batería salen idénticos (90) o se rechazan con
error (5: dos con FMO, dos con tipos de corte mezclados en una imagen y uno con predicción ponderada
explícita en B); ninguno da imagen errónea. (El arreglo 4 tocaba también `frame_mt.rs`, que ya no
existe: se retiró con el resto del código de hilos, véase abajo.)

## Refactorización y endurecimiento posteriores (2026-10-08 a 2026-10-12)

Después de los ocho arreglos, la copia (decodificador y su `rusty_h264-common`) se reorganizó para
cumplir las normas de escritura de Crusty_H264, sin cambiar ni un byte de lo que decodifica. La red de
equivalencia (2 129 entradas: los flujos de la UIT-T, salidas de codificadores reales y 2 000
mutaciones, con la huella SHA-256 de cada salida o el motivo exacto de cada rechazo) dio idéntico
en cada paso, en Windows y en Linux, con y sin la aceleración `asm`.

Se retiró el código que nunca se ejecutaba con `decode()`: el modo multihilo y el hilo de
reconstrucción diferida, que además tragaban errores y tenían bucles sin salida en el modo sin `std`
(que también se retiró), y todo el diagnóstico del autor (variables de entorno `RS_H264_*`/`RFF_*`,
contadores, perfilador, volcados y 28 interruptores fijos con sus ramas muertas).

Después se quitaron los datos inventados. El código original, para no entrar nunca en pánico, leía
como un valor por defecto cualquier posición que se saliera de una tabla: un bloque ausente pasaba a
ser «intra, sin coeficientes, sin movimiento», una muestra ausente era negra o gris, una referencia
que no estaba en la lista se convertía en la primera, y varias reconstrucciones se saltaban en
silencio. Ahora esos casos, que la construcción del decodificador hace imposibles, devuelven el
error `Inconsistent` (o `FueraDeRango` en la parte común) y el flujo se rechaza. Los valores que sí
fija la norma para un vecino no disponible se conservan, con la cláusula citada de la edición
08/2024 de la UIT-T H.264. La compensación de movimiento sin aceleración quedó además un 10 % más
rápida al recorrer filas enteras en lugar de comprobar cada muestra.

La compensación de movimiento sobre el plano exacto (`mc_luma`, `mc_chroma`) tampoco inventa ya
muestras: comprueba al entrar que el plano cubre sus dimensiones, y un plano de ancho o alto 0, que
antes entraba en pánico, es un error.

Al investigar por qué el filtro de bordes podía encontrar un índice de referencia fuera de su mapa
apareció un defecto del original que afecta a flujos válidos. Cuando los cortes de una imagen
reordenan sus listas de referencias de forma distinta (`ref_pic_list_modification`), el último
macrobloque de cada corte se reconstruía con la lista del corte siguiente, porque su trabajo
diferido se ejecutaba después del cambio de corte, y las últimas filas de cada corte se filtraban
traduciendo sus referencias con el mapa del corte siguiente. Ahora el trabajo diferido se completa
antes de cambiar de corte y cada macrobloque se filtra con el mapa de su corte. Con un vídeo de x264
al que se añade esa reordenación (válida) en cortes alternos, la salida pasa a ser idéntica a la de
FFmpeg, en CAVLC y en CABAC; la prueba `mslice_reorden` lo fija. Con el mapa correcto, un índice
ausente ya no puede significar nada legítimo y es un error: eso cambia el resultado de 72 entradas
mutadas de la red y de ninguna sin mutar.

Queda una decisión abierta, y hasta tomarla se conserva el comportamiento anterior: si un corte
CABAC leído más allá de su final debe rechazarse (hoy se completa con ceros). Cambia el resultado de
flujos corruptos de la red, no de ningún flujo válido.

## Filtro de bordes por corte (2026-10-21)

El original guardaba un solo juego de parámetros del filtro de bordes para toda la imagen. Si los
cortes de una imagen llevaban desplazamientos distintos (`slice_alpha_c0_offset_div2`,
`slice_beta_offset_div2`), las filas que se filtraban ya dentro del corte siguiente usaban los del
corte siguiente; y el indicador que desactiva el filtro (`disable_deblocking_filter_idc`) regía para
toda la imagen según el primer corte que lo desactivaba o según el último corte. La norma (§8.7 y
§8.7.2) dice que ambos se toman del corte del macrobloque que se filtra.

Ahora cada corte se guarda en un único registro (`corte.rs`: primer macrobloque, parámetros del
filtro y mapas de referencias), un macrobloque de un corte sin filtro tiene intensidad 0 en todos sus
bordes, y el filtro común (`rusty_h264-common`) recibe los desplazamientos de cada macrobloque en
lugar de un par por llamada. Con tres vídeos de x264 de cuatro cortes por imagen, empezados a mitad
de fila, con desplazamientos o indicadores distintos por corte, la salida pasa a ser idéntica a la
de FFmpeg y OpenH264; la prueba `filtro_por_corte` lo fija.

La cabecera del corte se valida además como hace FFmpeg: un indicador distinto de 0, 1 o 2, o un
desplazamiento fuera de −6…6, rechaza el flujo. Eso cambia el resultado de 30 entradas mutadas de la
red (26 ya se rechazaban por otro motivo; las otras 4 las rechaza también FFmpeg) y de ninguna sin
mutar.

