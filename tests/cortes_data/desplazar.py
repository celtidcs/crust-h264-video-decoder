"""Cambia `slice_alpha_c0_offset_div2` y `slice_beta_offset_div2` de los cortes impares de
cada imagen (sospecha de COR-3: el filtro de las filas retrasadas usaría los desplazamientos del
corte siguiente). Uso: python desplazar.py <entrada.264> <salida.264> <alfa> <beta>
Solo CAVLC (sin alineación que recalcular). Los valores deben estar en [-6, 6] (§7.4.3)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import cabeceras as cab
from reordenar import a_bits, a_bytes, con_emulacion, ue


def se(v):
    return ue(2 * v - 1 if v > 0 else -2 * v)


def main():
    entrada, salida_ruta, alfa, beta = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
    spss, ppss, salida, indice, cambiados = {}, {}, bytearray(), 0, 0
    for nal in cab.nals(open(entrada, 'rb').read()):
        tipo = nal[0] & 0x1F
        cuerpo = cab.rbsp(nal[1:])
        b = cab.Bits(cuerpo)
        if tipo == 7:
            i, s = cab.leer_sps(b)
            spss[i] = s
        elif tipo == 8:
            i, p = cab.leer_pps(b)
            ppss[i] = p
        elif tipo in (1, 5):
            c = cab.leer_corte(b, tipo, spss, ppss)
            indice = 0 if c['primer_mb'] == 0 else indice + 1
            if indice % 2 == 1 and 'pos_alfa' in c:
                bits = a_bits(cuerpo)
                pos = c['pos_alfa']
                viejo = se(c['alfa']) + se(c['beta'])
                assert bits[pos:pos + len(viejo)] == viejo
                fin = bits.rindex('1')
                carga = bits[:pos] + se(alfa) + se(beta) + bits[pos + len(viejo):fin]
                carga += '1' + '0' * (-(len(carga) + 1) % 8)
                nal = nal[:1] + con_emulacion(a_bytes(carga))
                cambiados += 1
        salida += b'\x00\x00\x00\x01' + nal
    open(salida_ruta, 'wb').write(salida)
    print(f'cortes cambiados: {cambiados}')


if __name__ == '__main__':
    main()
