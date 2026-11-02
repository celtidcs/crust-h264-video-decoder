#!/bin/sh
# Verificación independiente (REF-15) de un commit de la refactorización, en un git worktree
# aparte para no tocar el árbol de trabajo de quien refactoriza. Todo queda dentro del proyecto
# (aislamiento estricto: prohibido escribir fuera de la carpeta del proyecto).
# Uso: sh verificar_commit.sh <commit> [rapido]
#   rapido: solo compila el banco y pasa la red REF-1.
# Completo: además pruebas del crate, clippy normal y pedantic, formato, unsafe y pánicos.
# Termina con 0 solo si la compilación, la red y todas las pruebas pasan; con 1 si algo falla y
# con 2 si no se puede preparar el worktree. Las cifras de clippy, fmt, unsafe y pánicos son
# mediciones que se comparan a mano con la anterior, no cambian el código de salida.
# Solo se verifica lo que contiene el commit: no se copia nada del árbol de trabajo (lo que
# ocultó durante dos entregas que los YUV de referencia no estaban en git).
set -u
H="$(cd "$(dirname "$0")/.." && pwd)"
C="$1"
MODO="${2:-completo}"
W="$H/v"
T="$H/vt"
FALLO=0
# Sin restos (limpieza estricta): el worktree se retira siempre, también si
# el script termina por un fallo o por una interrupción.
retirar_worktree() { git -C "$H" worktree remove --force "$W" 2>/dev/null; }
trap retirar_worktree EXIT
trap 'exit 1' INT TERM
echo "=== commit $C: $(git -C "$H" log -1 --format=%s "$C")"
retirar_worktree
git -C "$H" worktree add -q --detach "$W" "$C" || exit 2
mkdir -p "$T"
echo "huella línea base: $(sha256sum "$H/red/linea-base.tsv" | cut -c1-16)"
# Si la compilación falla se para: la red con un ejecutable viejo daría un verde falso.
if ! ( cd "$W" && CARGO_TARGET_DIR="$T" cargo build --release -q 2>"$T/ultima-compilacion.txt" ); then
  grep -E '^(warning|error)' -A6 "$T/ultima-compilacion.txt" | head -40
  echo "COMPILACIÓN FALLIDA: red no ejecutada"
  exit 1
fi
grep -E '^(warning|error)' "$T/ultima-compilacion.txt" | sort | uniq -c
# La salida va a un archivo para conservar el código de salida de red.py (una tubería lo perdería).
if ! ( cd "$H/red" && RED_BANCO="$T/release/banco_h264.exe" python "$H/scripts/red.py" comprobar >"$T/ultima-red.txt" 2>&1 ); then
  FALLO=1
fi
grep -E '^(ANTES|AHORA|RED|DISTINTO)' "$T/ultima-red.txt" | head -12
[ "$FALLO" -ne 0 ] && echo "RED FALLIDA"
[ "$MODO" = "rapido" ] && exit "$FALLO"
for crate in crusty_h264 crusty_h264-common; do
  echo "--- $crate"
  # Desde COR-1b (2026-10-12) el decodificador ya no tiene la feature `std` (siempre la usa).
  FEAT="--features std"
  grep -q '^std = ' "$W/crates/$crate/Cargo.toml" || FEAT=""
  SALIDA="$T/ultimas-pruebas-$crate.txt"
  if ! ( cd "$W/crates/$crate" && CARGO_TARGET_DIR="$T" cargo test --release -q --no-default-features $FEAT >"$SALIDA" 2>&1 ); then
    FALLO=1
    echo "PRUEBAS FALLIDAS en $crate (detalle en $SALIDA):"
    grep -E '^error' -A6 "$SALIDA" | head -30
  fi
  grep -E '^test result|FAILED|^error' "$SALIDA" | grep -v ' 0 passed; 0 failed; 0 ignored'
  echo "clippy normal (avisos): $(cd "$W/crates/$crate" && CARGO_TARGET_DIR="$T" cargo clippy --release --no-default-features $FEAT --lib 2>&1 | grep -c '^warning')"
  echo "clippy pedantic (avisos): $(cd "$W/crates/$crate" && CARGO_TARGET_DIR="$T" cargo clippy --release --no-default-features $FEAT --lib -- -W clippy::pedantic 2>&1 | grep -c '^warning')"
  echo "fmt (bloques con diferencias): $(cd "$W/crates/$crate" && cargo fmt --check -- --config newline_style=Native 2>/dev/null | grep -c '^Diff in')"
  # forbid puede estar en Cargo.toml ([lints]) o como atributo de crate en src/lib.rs.
  echo "forbid unsafe: Cargo.toml $(grep -c 'unsafe_code = "forbid"' "$W/crates/$crate/Cargo.toml"), lib.rs $(grep -c '^#!\[forbid(unsafe_code)\]' "$W/crates/$crate/src/lib.rs") | unsafe en src: $(grep -rnw 'unsafe' --include=*.rs "$W/crates/$crate/src" | grep -vE '^\S+:\s*//|forbid|//.*unsafe' | wc -l)"
done
python "$H/scripts/contar_panicos.py" "$W/crates"
[ "$FALLO" -ne 0 ] && echo "VERIFICACIÓN FALLIDA"
exit "$FALLO"
