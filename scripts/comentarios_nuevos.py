"""Comentarios con texto NUEVO (no desplazado del autor) añadidos en un rango de commits.

Uso: python comentarios_nuevos.py [base] [punta]   (por defecto 0ec5a68 HEAD)
Un comentario añadido cuyo texto (sin sangría ni marcador) también aparece como línea retirada
en el mismo rango, o como fragmento del texto de comentarios del original publicado
(`h264/vendor-original`, con los espacios normalizados: cubre los párrafos del autor solo
re-partidos en líneas), se considera del autor y no se lista. Marca `EN` las líneas que parecen inglés
(palabras funcionales inglesas sin palabras funcionales españolas).
"""
import glob
import re
import subprocess
import sys

BASE, PUNTA = (sys.argv[1:] + ['0ec5a68', 'HEAD'])[:2] if len(sys.argv) < 3 else sys.argv[1:3]
COMENTARIO = re.compile(r'^\s*(//[/!]?)\s?(.*)$')
INGLES = re.compile(r"\b(the|is|are|and|of|to|with|from|this|that|it|on|by|not|which|when|be|an)\b", re.I)
ESPANOL = re.compile(r"\b(el|la|los|las|de|del|que|en|un|una|por|para|con|se|es|sin|al|y|o)\b")


def texto(linea):
    m = COMENTARIO.match(linea)
    return m.group(2).strip() if m else None


diff = subprocess.run(['git', '-C', 'h264', 'diff', '-U0', BASE, PUNTA, '--', '*.rs'],
                      capture_output=True, text=True, encoding='utf-8').stdout
def texto_original():
    trozos = []
    for f in glob.glob('h264/vendor-original/**/*.rs', recursive=True):
        for l in open(f, encoding='utf-8'):
            t = texto(l)
            if t:
                trozos.append(t)
    return ' '.join(' '.join(trozos).split())


ORIGINAL = texto_original()
retiradas = {texto(l[1:]) for l in diff.splitlines() if l.startswith('-') and not l.startswith('---')}
archivo, linea = None, 0
for l in diff.splitlines():
    if l.startswith('+++ '):
        archivo = l[6:]
    elif l.startswith('@@'):
        linea = int(re.search(r'\+(\d+)', l).group(1))
    elif l.startswith('+'):
        t = texto(l[1:])
        if t and t not in retiradas and ' '.join(t.split()) not in ORIGINAL:
            marca = 'EN' if INGLES.search(t) and not ESPANOL.search(t) else '  '
            print(f'{marca} {archivo}:{linea}: {t[:120]}')
        linea += 1
