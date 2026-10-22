#!/bin/sh
# Uso: comparar.sh antes.tsv despues.tsv  — solo flujos progresivos 4:2:0 (clases.tsv)
awk -F'\t' 'FILENAME==ARGV[1]{ok[$1]=($5=="progresivo" && $3=="yuv420p"); next}
FILENAME==ARGV[2]{if(ok[$1]) a[$1]=$5; next}
ok[$1] && FNR>1 {b[$1]=$5}
END{for(k in b){c[b[k]]++; if(a[k]!=b[k]) print "CAMBIO", k, a[k], "->", b[k]}; for(k in c) print "TOTAL", k, c[k]}' clases.tsv "$1" "$2" | sort
