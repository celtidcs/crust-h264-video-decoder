"""Cuenta puntos de pánico de producción en los crates de un directorio (por defecto `crates/`).

Uso: python contar_panicos.py [<directorio de crates>]

Se recorre `src/` entero (incluidas subcarpetas). En cada archivo cuenta lo que hay antes del primer
módulo de pruebas (`#[cfg(test)]` seguido de `mod`); un `#[cfg(test)]` suelto (p. ej. sobre un
`extern crate`) no corta el recuento. Los archivos `tests.rs` o dentro de una carpeta `tests/` se
consideran de pruebas y se informan aparte.
"""
import glob
import os
import re
import sys

PATRON = re.compile(r"\.unwrap\(\)|\.expect\(|unreachable!|panic!\(")
MODULO_PRUEBAS = re.compile(r"#\[cfg\(test\)\]\s*(pub(\(crate\))?\s+)?mod\s")

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
crates = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RAIZ, "crates")
for crate in sorted(glob.glob(os.path.join(crates, "*"))):
    produccion, pruebas = {}, {}
    for f in glob.glob(os.path.join(crate, "src", "**", "*.rs"), recursive=True):
        t = open(f, encoding="utf-8").read()
        relativo = os.path.relpath(f, os.path.join(crate, "src")).replace("\\", "/")
        if os.path.basename(f) == "tests.rs" or "/tests/" in "/" + relativo:
            pruebas[relativo] = len(PATRON.findall(t))
            continue
        m = MODULO_PRUEBAS.search(t)
        if m:
            t = t[:m.start()]
        produccion[relativo] = len(PATRON.findall(t))
    print(f"{os.path.basename(crate)}: producción {sum(produccion.values())} "
          f"{ {k: v for k, v in produccion.items() if v} } | archivos de pruebas {sum(pruebas.values())}")
