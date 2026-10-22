"""Lee las cabeceras de corte de un flujo H.264 Annex B hasta la modificación de listas.

Uso: python cabeceras.py <flujo.264> [...]          resumen por flujo
     python cabeceras.py --detalle <flujo.264>       una línea por corte

Para cada corte: primer macrobloque, tipo, `frame_num`, tamaño activo de las listas y si lleva
`ref_pic_list_modification`. El resumen cuenta las imágenes con varios cortes y, de ellas, las
que tienen cortes con listas DISTINTAS (otro tamaño activo u otra modificación): son las que
ejercitan el mapa de referencias por corte del filtro de desbloqueo (COR-3, `map_ref`).
Norma: UIT-T H.264 (08/2024) §7.3.2.1.1 (SPS), §7.3.2.2 (PPS), §7.3.3 (cabecera de corte),
§7.3.3.1 (modificación de listas). Solo un grupo de cortes (sin FMO).
"""
import sys


def rbsp(nal):
    """Quita los bytes de prevención de emulación (00 00 03)."""
    salida, ceros = bytearray(), 0
    for b in nal:
        if ceros >= 2 and b == 3:
            ceros = 0
            continue
        salida.append(b)
        ceros = ceros + 1 if b == 0 else 0
    return bytes(salida)


def nals(datos):
    i, n = 0, len(datos)
    inicios = []
    while i + 3 <= n:
        if datos[i] == 0 and datos[i + 1] == 0 and datos[i + 2] == 1:
            inicios.append(i + 3)
            i += 3
        else:
            i += 1
    for k, s in enumerate(inicios):
        fin = inicios[k + 1] - 3 if k + 1 < len(inicios) else n
        while fin > s and datos[fin - 1] == 0:
            fin -= 1
        yield datos[s:fin]


class Bits:
    def __init__(self, datos):
        self.d, self.p = datos, 0

    def u(self, n):
        v = 0
        for _ in range(n):
            byte = self.d[self.p >> 3] if (self.p >> 3) < len(self.d) else 0
            v = (v << 1) | ((byte >> (7 - (self.p & 7))) & 1)
            self.p += 1
        return v

    def ue(self):
        ceros = 0
        while self.u(1) == 0:
            ceros += 1
            if ceros > 31:
                raise ValueError('ue demasiado largo')
        return (1 << ceros) - 1 + self.u(ceros)

    def se(self):
        k = self.ue()
        return (k + 1) // 2 if k & 1 else -(k // 2)


def saltar_lista_escalado(b, tam):
    ultimo, siguiente = 8, 8
    for _ in range(tam):
        if siguiente:
            siguiente = (ultimo + b.se() + 256) % 256
        ultimo = siguiente or ultimo


def leer_sps(b):
    perfil = b.u(8)
    b.u(16)
    sps_id = b.ue()
    croma = 1
    if perfil in (100, 110, 122, 244, 44, 83, 86, 118, 128, 138, 139, 134, 135):
        croma = b.ue()
        if croma == 3:
            b.u(1)
        b.ue(), b.ue(), b.u(1)
        if b.u(1):
            for i in range(8 if croma != 3 else 12):
                if b.u(1):
                    saltar_lista_escalado(b, 16 if i < 6 else 64)
    s = {'log2_fn': b.ue() + 4, 'poc': b.ue()}
    if s['poc'] == 0:
        s['log2_lsb'] = b.ue() + 4
    elif s['poc'] == 1:
        s['delta_cero'] = b.u(1)
        b.se(), b.se()
        for _ in range(b.ue()):
            b.se()
    b.ue(), b.u(1), b.ue(), b.ue()
    s['solo_fotogramas'] = b.u(1)
    return sps_id, s


def leer_pps(b):
    pps_id, sps_id = b.ue(), b.ue()
    p = {'sps': sps_id, 'cabac': b.u(1), 'campo_inferior': b.u(1)}
    if b.ue():
        raise ValueError('FMO no soportado')
    p['ref0'], p['ref1'] = b.ue() + 1, b.ue() + 1
    p['ponderada'], p['bipred'] = b.u(1), b.u(2)
    b.se(), b.se(), b.se()
    p['control_filtro'] = b.u(1)
    b.u(1)
    p['redundante'] = b.u(1)
    return pps_id, p


def leer_modificacion(b):
    ordenes = []
    if b.u(1):
        while True:
            idc = b.ue()
            if idc == 3:
                break
            ordenes.append((idc, b.ue()))
    return tuple(ordenes)


def leer_corte(b, tipo_nal, spss, ppss):
    c = {'primer_mb': b.ue(), 'tipo': b.ue() % 5}
    p = ppss[b.ue()]
    s = spss[p['sps']]
    c['frame_num'] = b.u(s['log2_fn'])
    campo = 0
    if not s['solo_fotogramas']:
        campo = b.u(1)
        if campo:
            b.u(1)
    if tipo_nal == 5:
        b.ue()
    if s['poc'] == 0:
        c['poc_lsb'] = b.u(s['log2_lsb'])
        if p['campo_inferior'] and not campo:
            b.se()
    elif s['poc'] == 1 and not s['delta_cero']:
        b.se()
        if p['campo_inferior'] and not campo:
            b.se()
    if p['redundante']:
        b.ue()
    es_b = c['tipo'] == 1
    if es_b:
        b.u(1)
    c['ref0'], c['ref1'], c['mod0'], c['mod1'] = 0, 0, (), ()
    if c['tipo'] in (0, 1, 3):
        c['ref0'], c['ref1'] = p['ref0'], p['ref1'] if es_b else 0
        if b.u(1):
            c['ref0'] = b.ue() + 1
            if es_b:
                c['ref1'] = b.ue() + 1
        c['pos_mod0'] = b.p
        c['mod0'] = leer_modificacion(b)
        if es_b:
            c['mod1'] = leer_modificacion(b)
    c['fin_cabecera'] = fin_de_cabecera(b, c, p, tipo_nal)
    return c


def fin_de_cabecera(b, c, p, tipo_nal):
    """Posición del primer bit tras la cabecera (antes de `cabac_alignment_one_bit`), o `None`
    si la cabecera lleva elementos que este lector no recorre (tabla de pesos, marcado adaptativo,
    cortes SP/SI). Solo se usa para reescribir cortes CABAC."""
    if c['tipo'] in (3, 4) or (p['ponderada'] and c['tipo'] == 0) or (p['bipred'] == 1 and c['tipo'] == 1):
        return None
    if c.get('nal_ref_idc', 1):
        if tipo_nal == 5:
            b.u(2)
        elif b.u(1):
            return None
    if p['cabac'] and c['tipo'] != 2:
        b.ue()
    b.se()
    if p['control_filtro']:
        c['idc_filtro'] = b.ue()
        if c['idc_filtro'] != 1:
            c['pos_alfa'] = b.p
            c['alfa'], c['beta'] = b.se(), b.se()
    return b.p


def cortes(datos):
    spss, ppss = {}, {}
    for nal in nals(datos):
        if not nal:
            continue
        tipo = nal[0] & 0x1F
        b = Bits(rbsp(nal[1:]))
        if tipo == 7:
            i, s = leer_sps(b)
            spss[i] = s
        elif tipo == 8:
            i, p = leer_pps(b)
            ppss[i] = p
        elif tipo in (1, 5):
            yield leer_corte(b, tipo, spss, ppss)


def imagenes(lista):
    actual = []
    for c in lista:
        if actual and c['primer_mb'] == 0:
            yield actual
            actual = []
        actual.append(c)
    if actual:
        yield actual


def main():
    detalle = sys.argv[1] == '--detalle'
    for ruta in sys.argv[2 if detalle else 1:]:
        try:
            lista = list(cortes(open(ruta, 'rb').read()))
        except (ValueError, KeyError, IndexError) as e:
            print(f'{ruta}: no leído ({e})')
            continue
        if detalle:
            for c in lista:
                print(c)
            continue
        imgs = list(imagenes(lista))
        varios = [i for i in imgs if len(i) > 1]
        distintas = [i for i in varios
                     if len({(c['ref0'], c['ref1'], c['mod0'], c['mod1']) for c in i if c['tipo'] != 2}) > 1]
        print(f'{ruta}: {len(imgs)} imágenes, {len(varios)} con varios cortes, '
              f'{len(distintas)} con listas distintas entre cortes')


if __name__ == '__main__':
    main()
