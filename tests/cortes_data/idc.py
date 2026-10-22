"""Reescribe `disable_deblocking_filter_idc` de cada corte según un patrón cíclico por índice de
corte dentro de la imagen (COR-4, sospecha de D-2026-10-12-2 sobre el idc). Uso:
python idc.py <entrada.264> <salida.264> <patrón>, p. ej. «0,1,2,1». Solo CAVLC. Con idc 1 se
quitan alfa y beta (§7.3.3); con idc 0 o 2 se conservan los del corte o se ponen a 0."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import cabeceras as cab
from reordenar import a_bits, a_bytes, con_emulacion, ue
from desplazar import se


def main():
    entrada, salida_ruta = sys.argv[1], sys.argv[2]
    patron = [int(x) for x in sys.argv[3].split(',')]
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
            nuevo = patron[indice % len(patron)]
            if nuevo != c['idc_filtro']:
                assert 'pos_alfa' in c, 'el corte de partida debe tener idc distinto de 1'
                viejo = ue(c['idc_filtro']) + se(c['alfa']) + se(c['beta'])
                pos = c['pos_alfa'] - len(ue(c['idc_filtro']))
                bits = a_bits(cuerpo)
                assert bits[pos:pos + len(viejo)] == viejo
                campo = ue(nuevo) + ('' if nuevo == 1 else se(c['alfa']) + se(c['beta']))
                fin = bits.rindex('1')
                carga = bits[:pos] + campo + bits[pos + len(viejo):fin]
                carga += '1' + '0' * (-(len(carga) + 1) % 8)
                nal = nal[:1] + con_emulacion(a_bytes(carga))
                cambiados += 1
        salida += b'\x00\x00\x00\x01' + nal
    open(salida_ruta, 'wb').write(salida)
    print(f'cortes cambiados: {cambiados}')


if __name__ == '__main__':
    main()
