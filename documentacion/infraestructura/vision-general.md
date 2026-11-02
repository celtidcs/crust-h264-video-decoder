# Visión General de Infraestructura — Crusty_H264

## 1. Topología del Proyecto

Las dependencias de terceros se descargan de crates.io en la primera compilación; los datos de prueba están todos en el repositorio:

```text
crusty_h264/
├── Cargo.toml                    # Paquete del banco (`banco_h264`); no es un workspace
├── crates/                       # Bibliotecas del decodificador
│   ├── crusty_h264-common/       # Primitivas matemáticas
│   └── crusty_h264/              # Lógica de decodificación y reordenación
├── src/                          # CLI y herramientas de diagnóstico
├── tests/                        # Casos de prueba de integración
├── conf/, fuzz/, calib/, cod/    # Insumos de calibración y secuencias de prueba oficiales
└── red.py, velocidad.py          # Scripts de orquestación de red y rendimiento
```

## 2. Garantías de Reproducibilidad

1. **Determinismo**: Toda la batería de pruebas de mutantes (`fuzz`) utiliza semillas deterministas; dos ejecuciones sobre la misma plataforma generan la misma secuencia exacta de mutaciones.
2. **Aislamiento**: El decodificador no realiza operaciones de disco durante la descompresión ni abre sockets de red.
