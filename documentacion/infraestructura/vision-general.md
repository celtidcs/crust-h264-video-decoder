# Visión General de Infraestructura — Crusty_H264

## 1. Topología del Proyecto

El proyecto está diseñado para funcionar de manera completamente autónoma sin dependencias externas de red durante la compilación o las pruebas:

```text
crusty_h264/
├── Cargo.toml                    # Declaración del workspace y configuración de compilación release
├── vendor/                       # Código fuente de las librerías del decodificador
│   ├── rusty_h264-common-0.16.0/ # Primitivas matemáticas
│   └── rusty_h264-decoder-0.16.0/# Lógica de decodificación y reordenación
├── src/                          # CLI y herramientas de diagnóstico
├── tests/                        # Casos de prueba de integración
├── conf/, fuzz/, calib/, cod/    # Insumos de calibración y secuencias de prueba oficiales
└── red.py, velocidad.py          # Scripts de orquestación de red y rendimiento
```

## 2. Garantías de Reproducibilidad

1. **Determinismo**: Toda la batería de pruebas de mutantes (`fuzz`) utiliza semillas deterministas; dos ejecuciones sobre la misma plataforma generan la misma secuencia exacta de mutaciones.
2. **Aislamiento**: El decodificador no realiza operaciones de disco durante la descompresión ni abre sockets de red.
