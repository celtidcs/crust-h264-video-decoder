"""Medición de velocidad del decodificador rusty (Claude, COR-3; Codex pide medir antes de optimizar).

Uso: python velocidad.py <banco_A.exe> [<banco_B.exe>] [--veces N]
Decodifica cada flujo de vel/ N veces por banco, ALTERNANDO A y B (A B B A ...) para repartir el
ruido térmico y de caché; imprime la mediana de ms (solo decodificación, sin lectura de disco)
por flujo y la razón B/A. Comprueba además que ambos bancos dan la misma huella.
"""
import re
import statistics
import subprocess
import sys
from pathlib import Path

RAIZ = Path(__file__).resolve().parent.parent
VEL = RAIZ / "vel"
FLUJOS = [
    "safari720.264",
    "safari1080.264",
    "safari4k.264",
    "bf3main.264",
    "bf0.264",
    "s720nw.264",
    "cavlc720.264",
]  # cavlc720: x264 CAVLC High con B (NUC-3); safari: escala 720p / 1080p / 4K


def una(banco, flujo):
    s = subprocess.run([banco, "decodificar", "rusty", str(VEL / flujo)], capture_output=True, text=True).stdout
    m = re.search(r"ms=(\d+).*huella=([0-9a-f]+)", s)
    if not m:
        sys.exit(f"{banco} {flujo}: salida inesperada: {s[:200]}")
    return int(m.group(1)), m.group(2)


def main():
    args = sys.argv[1:]
    veces = 5
    if "--veces" in args:
        k = args.index("--veces")
        veces = int(args[k + 1])
        del args[k:k + 2]
    bancos = [str(Path(b).resolve()) for b in args]
    for flujo in FLUJOS:
        tiempos = {b: [] for b in bancos}
        huellas = set()
        for i in range(veces):
            orden = bancos if i % 2 == 0 else list(reversed(bancos))
            for b in orden:
                ms, h = una(b, flujo)
                tiempos[b].append(ms)
                huellas.add(h)
        medianas = [statistics.median(tiempos[b]) for b in bancos]
        linea = f"{flujo:16s} " + "  ".join(f"{m:7.0f} ms" for m in medianas)
        if len(bancos) == 2:
            linea += f"  B/A = {medianas[1] / medianas[0]:.3f}"
        print(linea + ("  HUELLAS DISTINTAS" if len(huellas) > 1 else ""))


if __name__ == "__main__":
    main()
