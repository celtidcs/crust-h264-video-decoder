"""Lista los `unwrap_or*`, `unwrap()` y `expect()` del código de producción de la copia H.264.

Uso: python scripts/valores_inventados.py
Salta las líneas de comentario y los bloques marcados con `#[cfg(test)]` (función, `impl` o
módulo, delimitados por llaves). Calibración: `inter.rs:at()` en `626e91c` debe aparecer.
"""
import glob
import re
from pathlib import Path

PROYECTO = Path(__file__).resolve().parent.parent
RAIZ = PROYECTO / 'crates'
PATRON = re.compile(r'unwrap_or(_default|_else)?\(|\.unwrap\(\)|\.expect\(')


def lineas_de_prueba(lineas):
    marca = [False] * len(lineas)
    i = 0
    while i < len(lineas):
        if lineas[i].strip() != '#[cfg(test)]':
            i += 1
            continue
        k, prof, abierto = i + 1, 0, False
        while k < len(lineas):
            prof += lineas[k].count('{') - lineas[k].count('}')
            abierto = abierto or '{' in lineas[k]
            if (abierto and prof <= 0) or (not abierto and lineas[k].rstrip().endswith(';')):
                break
            k += 1
        for t in range(i, min(k + 1, len(lineas))):
            marca[t] = True
        i = k + 1
    return marca


def main():
    for f in sorted(glob.glob(str(RAIZ / 'rusty_h264-*/src/**/*.rs'), recursive=True)):
        lineas = Path(f).read_text(encoding='utf-8').split('\n')
        prueba = lineas_de_prueba(lineas)
        for n, l in enumerate(lineas):
            s = l.strip()
            if not prueba[n] and not s.startswith('//') and PATRON.search(l):
                print(f'{Path(f).relative_to(RAIZ)}:{n + 1}: {s[:110]}')


if __name__ == '__main__':
    main()
