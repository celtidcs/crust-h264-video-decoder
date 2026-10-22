"""Reordena la lista 0 de los cortes P impares de cada imagen (COR-3, `map_ref`).

Uso: python reordenar.py <entrada.264> <salida.264>
En cada corte P con índice impar dentro de su imagen y al menos 2 referencias activas, cambia
`ref_pic_list_modification_flag_l0 = 0` por la orden «poner primero la imagen CurrPicNum − 2»
(modification_of_pic_nums_idc 0, abs_diff_pic_num_minus1 1, fin 3): la lista pasa de
[n−1, n−2, …] a [n−2, n−1, …]. El flujo sigue siendo válido (§7.3.3.1, §8.2.4.3.1); solo cambia
qué imagen nombra cada `ref_idx` en esos cortes, de modo que cortes vecinos de la misma imagen
tienen listas distintas. En CABAC recalcula los `cabac_alignment_one_bit` tras la cabecera.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))  # cabeceras.py, en wodw-herramientas/h264
import cabeceras as cab


def ue(v):
    v += 1
    n = v.bit_length()
    return '0' * (n - 1) + format(v, 'b')


def a_bits(datos):
    return ''.join(format(b, '08b') for b in datos)


def a_bytes(bits):
    return bytes(int(bits[i:i + 8], 2) for i in range(0, len(bits), 8))


def con_emulacion(rbsp):
    salida, ceros = bytearray(), 0
    for b in rbsp:
        if ceros >= 2 and b <= 3:
            salida.append(3)
            ceros = 0
        salida.append(b)
        ceros = ceros + 1 if b == 0 else 0
    return bytes(salida)


def ppss_cabac(ppss, cuerpo, spss):
    """`entropy_coding_mode_flag` del PPS que nombra la cabecera de este corte."""
    b = cab.Bits(cuerpo)
    b.ue(), b.ue()
    return ppss[b.ue()]['cabac']


def main():
    datos = open(sys.argv[1], 'rb').read()
    spss, ppss, salida, indice, cambiados = {}, {}, bytearray(), 0, 0
    for nal in cab.nals(datos):
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
            if c['tipo'] == 0 and indice % 2 == 1 and c['ref0'] >= 2 and not c['mod0']:
                bits = a_bits(cuerpo)
                pos = c['pos_mod0']
                assert bits[pos] == '0'
                fin = bits.rindex('1')  # rbsp_stop_one_bit
                orden = '1' + ue(0) + ue(1) + ue(3)
                if ppss_cabac(ppss, cuerpo, spss):
                    fin_cab = c['fin_cabecera']
                    assert fin_cab is not None, 'cabecera CABAC no recorrible'
                    datos_corte = bits[(fin_cab + 7) // 8 * 8:]
                    cabecera = bits[:pos] + orden + bits[pos + 1:fin_cab]
                    carga = cabecera + '1' * (-len(cabecera) % 8) + datos_corte
                else:
                    carga = bits[:pos] + orden + bits[pos + 1:fin]
                    carga += '1' + '0' * (-(len(carga) + 1) % 8)
                nal = nal[:1] + con_emulacion(a_bytes(carga))
                cambiados += 1
        salida += b'\x00\x00\x00\x01' + nal
    open(sys.argv[2], 'wb').write(salida)
    print(f'cortes reordenados: {cambiados}')


if __name__ == '__main__':
    main()
