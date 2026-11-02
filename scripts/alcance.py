"""Alcance aproximado de llamadas dentro de un archivo Rust.

Uso: python alcance.py <archivo.rs> <raíz1,raíz2,...> [patrón]
Corta el archivo en funciones (`fn nombre(` hasta la llave que la cierra, saltando #[cfg(test)]
mod), toma como llamadas los identificadores seguidos de `(` y recorre desde las raíces. Imprime
las funciones alcanzables y las líneas con `patrón` (por defecto unwrap_or|map_or|is_some_and)
separadas en alcanzables / no alcanzables. Aproximado: no resuelve rasgos ni nombres repetidos.
"""
import re
import sys

sys.path.insert(0, '.')
from editar import llave_pareja

ruta, raices = sys.argv[1], sys.argv[2].split(',')
patron = re.compile(sys.argv[3] if len(sys.argv) > 3 else r'unwrap_or|\.map_or\(|is_some_and')
t = open(ruta, encoding='utf-8').read()
corte = t.find('#[cfg(test)]\nmod ')
cuerpo = t if corte < 0 else t[:corte]
funciones = {}
for m in re.finditer(r'\bfn (\w+)\s*(?:<[^>{]*>)?\s*\(', cuerpo):
    abre = cuerpo.find('{', m.end())
    cierra = llave_pareja(cuerpo, abre)
    ini = cuerpo.count('\n', 0, m.start()) + 1
    fin = cuerpo.count('\n', 0, cierra) + 1
    funciones.setdefault(m.group(1), []).append((ini, fin, cuerpo[abre:cierra]))
llamadas = {f: set(re.findall(r'\b(\w+)\s*(?:::<[^>]*>)?\(', ''.join(b for _, _, b in v))) & funciones.keys()
            for f, v in funciones.items()}
vistos, pila = set(), [r for r in raices if r in funciones]
while pila:
    f = pila.pop()
    if f not in vistos:
        vistos.add(f)
        pila.extend(llamadas[f] - vistos)
rangos = [(i, j, f) for f in vistos for i, j, _ in funciones[f]]
print('alcanzables:', ' '.join(sorted(vistos)))
for n, linea in enumerate(cuerpo.split('\n'), 1):
    if patron.search(linea) and not linea.strip().startswith('//'):
        dentro = [f for i, j, f in rangos if i <= n <= j]
        print(('ALCANZABLE ' + dentro[0] if dentro else 'no alcanzable').ljust(34), n, linea.strip()[:90])
