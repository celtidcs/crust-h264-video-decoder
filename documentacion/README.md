# Índice de Documentación — Crust H264 Video Decoder

Este directorio contiene la documentación formal, exhaustiva y en español del proyecto **Crust H264 Video Decoder**, redactada con rigor técnico y prosa humana conforme a los estándares exigidos en el manual de gobernanza (§5).

---

## Estructura General

1. [**Arquitectura de Software (`arquitectura.md`)**](arquitectura.md)  
   Describe en detalle la descomposición modular, la aplicación estricta de los principios SOLID, la gestión de memoria sin código inseguro (`forbid(unsafe_code)`), la separación de capas entre primitivas matemáticas y el motor de flujo, y el algoritmo del búfer de reordenación acotada de imágenes (DPB).

2. [**Catálogo de Funcionalidades (`funcionalidades.md`)**](funcionalidades.md)  
   Detalla el soporte exhaustivo de los perfiles H.264 (Constrained Baseline, Main y High Profile), la decodificación entrópica dual (CAVLC y CABAC), la compensación de movimiento a cuarto de píxel, la predicción ponderada explícita e implícita, y la gestión de listas de referencia con operaciones MMCO.

3. [**Manual de Uso e Integración (`manual-de-uso.md`)**](manual-de-uso.md)  
   Guía didáctica para programadores que deseen integrar `crust-h264-video-decoder` en sus aplicaciones Rust, o para investigadores y analistas de seguridad que operen la herramienta de línea de comandos `banco_h264` y ejecuten la red de equivalencia REF-1.

4. [**Historial de Cambios y Auditoría (`historial-de-cambios.md`)**](historial-de-cambios.md)  
   Crónica cronológica de la evolución del decodificador: desde los fallos normativos detectados en la versión original de crates.io hasta las correcciones de precisión (serie COR-1 a COR-4) y la nuclearización modular de componentes monolíticos (serie NUC-1 a NUC-5).

5. [**Defectos Conocidos y Límites de Diseño (`defectos-conocidos.md`)**](defectos-conocidos.md)  
   Registro honesto y comprobable de los límites funcionales asumidos (perfiles entrelazados, FMO/ASO deliberadamente excluidos para reducir la superficie de ataque), así como de las pruebas de oráculo contra FFmpeg y Cisco OpenH264.

6. [**Infraestructura y Reproducibilidad (`infraestructura/`)**](infraestructura/)  
   Especificaciones del entorno de compilación, requisitos mínimos, plataformas soportadas (Windows y Linux) y guías de reproducción determinista.
