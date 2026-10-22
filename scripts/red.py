"""Red de equivalencia REF-1 de la refactorización del decodificador H.264.

Uso:
  python red.py preparar          extrae los flujos y genera los mutantes (una sola vez)
  python red.py base              guarda la línea base: huella o error de cada entrada
  python red.py comprobar         repite y compara con la línea base; termina con código 1 si cambia algo

Cada entrada se decodifica con `banco_h264 decodificar rusty`. Se registra la huella SHA-256 de
toda la salida y el número de fotogramas, o el motivo exacto del rechazo, o el código de salida si el
proceso cae. Dos ejecuciones son equivalentes solo si TODAS las líneas son idénticas.
"""
import concurrent.futures
import glob
import os
import subprocess
import sys
import zipfile

H = os.environ.get("RED_H", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# RED_BANCO permite verificar otra copia del banco (p. ej. un git worktree) sin tocar la de trabajo.
# RED_MODO elige el modo del banco (por defecto `rusty`; `rusty-au` = por unidades de acceso con el
# búfer de reordenación de Crusty_H264).
MODO = os.environ.get("RED_MODO", "rusty")
BANCO_CANDIDATOS = [
    os.path.join(H, "material_secundario", "target", "release", "banco_h264.exe" if os.name == "nt" else "banco_h264"),
    os.path.join(H, "target", "release", "banco_h264.exe" if os.name == "nt" else "banco_h264"),
    os.path.join(H, "h264", "target", "release", "banco_h264.exe" if os.name == "nt" else "banco_h264"),
]
BANCO_DEFECTO = next((b for b in BANCO_CANDIDATOS if os.path.exists(b)), BANCO_CANDIDATOS[0])
BANCO = os.environ.get("RED_BANCO", BANCO_DEFECTO)
RED = os.environ.get("RED_DIR", os.path.join(H, "red"))
FLUJOS = os.path.join(RED, "flujos")
MUTANTES = os.path.join(RED, "mutantes")
BASE = os.path.join(RED, "linea-base.tsv")
EXT_FLUJO = (".264", ".jsv", ".26l", ".avc", ".jvt", ".h264", ".bit")
TIPOS_MUTACION = ("bytes", "truncar", "duplicar", "cabecera")
SEMILLAS = 100
TOPE_SEGUNDOS = 300
HILOS = 8


def preparar():
    os.makedirs(FLUJOS, exist_ok=True)
    os.makedirs(MUTANTES, exist_ok=True)
    clases = {l.split("\t")[0]: l.rstrip("\n").split("\t") for l in open(H + "/conf/clases.tsv", encoding="utf-8")}
    for z in sorted(glob.glob(H + "/conf/zip/*.zip")):
        nombre = os.path.basename(z)[:-4]
        c = clases.get(nombre)
        if not c or c[4] != "progresivo" or c[2] != "yuv420p":
            continue
        zf = zipfile.ZipFile(z)
        miembros = [m for m in zf.namelist() if m.lower().endswith(EXT_FLUJO)]
        if miembros:
            open(f"{FLUJOS}/uit-{nombre}.264", "wb").write(zf.read(miembros[0]))
    for origen, prefijo in ((H + "/cod", "cod"), (H + "/calib", "calib"), (H + "/vel", "real")):
        for f in glob.glob(origen + "/*.264"):
            destino = f"{FLUJOS}/{prefijo}-{os.path.basename(f)}"
            open(destino, "wb").write(open(f, "rb").read())
    for base in sorted(glob.glob(H + "/fuzz/base/*.264")):
        for tipo in TIPOS_MUTACION:
            for s in range(1, SEMILLAS + 1):
                destino = f"{MUTANTES}/{os.path.basename(base)[:-4]}-{tipo}-{s}.264"
                subprocess.run([BANCO, "mutar", base, destino, str(s), tipo], check=True)
    print("flujos:", len(os.listdir(FLUJOS)), "mutantes:", len(os.listdir(MUTANTES)))


def medir(ruta):
    try:
        r = subprocess.run([BANCO, "decodificar", MODO, ruta], capture_output=True, text=True,
                           timeout=TOPE_SEGUNDOS)
    except subprocess.TimeoutExpired:
        return "TOPE"
    linea = (r.stdout.strip().splitlines() or [""])[-1]
    if r.returncode != 0:
        return f"CAIDA codigo={r.returncode}"
    campos = dict(p.split("=", 1) for p in linea.split() if "=" in p)
    if " ok " in linea:
        return f"ok fotogramas={campos['fotogramas']} {campos['ancho']}x{campos['alto']} huella={campos['huella']}"
    return f"rechazo motivo={campos.get('motivo', '?')}"


def ejecutar():
    entradas = sorted(glob.glob(FLUJOS + "/*.264")) + sorted(glob.glob(MUTANTES + "/*.264"))
    with concurrent.futures.ThreadPoolExecutor(HILOS) as pool:
        resultados = list(pool.map(medir, entradas))
    return [f"{os.path.basename(e)}\t{r}" for e, r in zip(entradas, resultados)]


def main():
    orden = sys.argv[1] if len(sys.argv) > 1 else ""
    if orden == "preparar":
        preparar()
    elif orden == "base":
        lineas = ejecutar()
        open(BASE, "w", encoding="utf-8", newline="\n").write("\n".join(lineas) + "\n")
        print("línea base:", len(lineas), "entradas")
    elif orden == "comprobar":
        base = open(BASE, encoding="utf-8").read().splitlines()
        actual = ejecutar()
        distintas = [(b, a) for b, a in zip(base, actual) if b != a]
        if len(base) != len(actual):
            print(f"DISTINTO número de entradas: base {len(base)}, ahora {len(actual)}")
            sys.exit(1)
        for b, a in distintas:
            print("ANTES ", b)
            print("AHORA ", a)
        print(f"{'RED EN VERDE' if not distintas else 'RED EN ROJO'}: {len(actual) - len(distintas)}/{len(actual)} idénticas")
        sys.exit(1 if distintas else 0)
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
