#!/bin/sh
# H264-2: mutaciones deterministas; códigos: 0 normal, 101 pánico de Rust, 124 tope de tiempo, otro = caída del proceso.
S="C:/Users/celti/wodw-herramientas/h264"
B=$S/h264/target/release/banco_h264.exe
N=${1:-100}
R=$S/fuzz/resultados.tsv; : > "$R"
for base in "$S"/fuzz/base/*.264; do
  for tipo in bytes truncar duplicar cabecera; do
    s=1; while [ $s -le $N ]; do
      m=$S/fuzz/m.264
      "$B" mutar "$base" "$m" $s $tipo
      for d in rusty openh264; do
        out=$(timeout 20 "$B" decodificar $d "$m" 2>&1); c=$?
        pico=$(echo "$out" | grep -o 'pico_mb=[0-9]*' | tail -1)
        printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$(basename $base)" $tipo $s $d $c "$pico" >> "$R"
        if [ $c -ne 0 ]; then cp "$m" "$S/fuzz/caso-$(basename $base .264)-$tipo-$s-$d.264"; echo "$out" | tail -3 > "$S/fuzz/caso-$(basename $base .264)-$tipo-$s-$d.txt"; fi
      done
      s=$((s+1))
    done
  done
done
echo FIN
