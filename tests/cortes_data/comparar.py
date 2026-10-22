"""Compara dos .yuv 4:2:0 fotograma a fotograma: fotogramas distintos, filas de macrobloque
de luma afectadas y diferencia máxima. Uso: python comparar.py a.yuv b.yuv ancho alto"""
import sys

a, b = (open(f, 'rb').read() for f in sys.argv[1:3])
w, h = int(sys.argv[3]), int(sys.argv[4])
tam = w * h * 3 // 2
for k in range(len(a) // tam):
    fa, fb = a[k * tam:(k + 1) * tam], b[k * tam:(k + 1) * tam]
    if fa == fb:
        continue
    filas, maximo, n = set(), 0, 0
    for y in range(h):
        ra, rb = fa[y * w:(y + 1) * w], fb[y * w:(y + 1) * w]
        if ra != rb:
            d = [abs(x - z) for x, z in zip(ra, rb) if x != z]
            n += len(d)
            maximo = max(maximo, max(d))
            filas.add(y // 16)
    print(f'fotograma {k}: {n} muestras de luma distintas, máx {maximo}, filas MB {sorted(filas)}')
