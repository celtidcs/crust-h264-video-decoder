# Crusty_H264 — Decodificador H.264 Seguro, Conforme y Puro en Rust

Bienvenido al repositorio oficial de **Crusty_H264**, la implementación moderna, auditada y 100% en Rust seguro (`#![forbid(unsafe_code)]`) del estándar de compresión de vídeo H.264 / AVC (ISO/IEC 14496-10 / UIT-T H.264).

---

## 1. Visión General del Proyecto

En el ecosistema informático contemporáneo, los decodificadores multimedia programados en C y C++ representan históricamente uno de los vectores más críticos de vulnerabilidad y explotación remota (desbordamientos de búfer, desreferencias de punteros nulos, carreras de memoria y uso tras liberación).

**Crusty_H264** nace como una alternativa de ingeniería rigurosa, orientada a ofrecer:
1. **Seguridad Absoluta de Memoria**: Eliminación total de FFI, código C legado y bloques inseguros (`unsafe: 0`).
2. **Fidelidad y Conformidad Normativa**: Resolución milimétrica de las discrepancias del estándar UIT-T H.264, validada bit a bit contra oráculos orquestados con FFmpeg y la suite de pruebas oficial de la UIT-T.
3. **Arquitectura Limpia y Modular (SOLID)**: Reestructuración integral de componentes monolíticos en módulos atómicos de responsabilidad única.
4. **Cota Estricta de Recursos**: Búfer de reordenación de imágenes decodificadas (DPB) adaptativo según el nivel del estándar, imposibilitando el consumo desmedido de memoria RAM ante vídeos hostiles o con grupos de imágenes extensos.

---

## 2. Mapa Documental del Proyecto

La documentación de este proyecto se encuentra articulada en el directorio [`documentacion/`](documentacion/) de manera exhaustiva y estructurada:

| Documento | Contenido y Propósito |
|---|---|
| [`documentacion/README.md`](documentacion/README.md) | Guía de introducción, fundamentos y navegación del proyecto. |
| [`documentacion/arquitectura.md`](documentacion/arquitectura.md) | Diseño del software, capas, patrones SOLID, gestión de estado y desacoplamiento. |
| [`documentacion/funcionalidades.md`](documentacion/funcionalidades.md) | Catálogo técnico de perfiles soportados (Baseline, Main, High), herramientas y sintaxis. |
| [`documentacion/manual-de-uso.md`](documentacion/manual-de-uso.md) | Guía práctica de integración como librería en Rust y uso del banco de evaluación CLI. |
| [`documentacion/historial-de-cambios.md`](documentacion/historial-de-cambios.md) | Registro cronológico minucioso de auditorías, mutaciones, serie COR y serie NUC. |
| [`documentacion/defectos-conocidos.md`](documentacion/defectos-conocidos.md) | Bitácora técnica de oráculos, limitaciones de diseño conscientes e investigaciones. |
| [`documentacion/infraestructura/`](documentacion/infraestructura/) | Requisitos del sistema, guías de clonado, entorno de pruebas y scripts de compilación. |

---

## 3. Estructura de Crates y Código Fuente

El proyecto se distribuye en dos componentes centrales y un banco de evaluación:

```text
crusty_h264/
├── Cargo.toml                              # Manifiesto raíz y orquestación del workspace
├── vendor/
│   ├── rusty_h264-common-0.16.0/           # Primitivas matemáticas, predicción, transformadas y DCT
│   └── rusty_h264-decoder-0.16.0/          # Motor de decodificación H.264 y reordenación DPB
├── src/                                    # Ejecutable del banco de evaluación y CLI (`banco_h264`)
├── scripts/                                # Scripts ejecutables de verificación, auditoría y velocidad
├── tests/                                  # Pruebas de integración, verificación de cortes y fuzzing
└── documentacion/                          # Documentación canónica en español
```

---

## 4. Guía Rápida de Compilación y Verificación

### Prerrequisitos
- **Rust Toolchain**: Versión 1.80 o superior (compatible con Rust 2021).
- **Cargo Audit y Clippy** instalados en el entorno de desarrollo.

### Comandos de Validación Inmediata
```bash
# Comprobación de compilación del decodificador
cargo check

# Ejecución de la suite completa de pruebas unitarias
cargo test

# Verificación de linter estricto sin avisos
cargo clippy --all-targets -- -D warnings

# Verificación de formato estricto
cargo fmt --check

# Auditoría de dependencias frente a vulnerabilidades
cargo audit
```

---

## 5. Licencia y Atribución

Este proyecto preserva íntegramente la licencia **BSD-2-Clause** de la base original desarrollada por Mata Network, reconociendo el trabajo fundacional y protegiendo las adiciones, refactorizaciones, correcciones y módulos desarrollados por nuestro equipo de ingeniería.
