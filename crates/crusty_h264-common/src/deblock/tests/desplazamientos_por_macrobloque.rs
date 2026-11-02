use super::*;

/// QP de las dos imágenes de prueba: `indexA` 20 da α = 7 sin desplazamiento y α = 32 con +12.
const QP: u8 = 20;
/// Salto de luma en el borde entre los dos macrobloques: no se filtra con α = 7 y sí con 32.
const SALTO: u8 = 20;

/// Filtra una imagen de 2×1 macrobloques intra con un salto de luma en `x = 16` (el borde
/// izquierdo del macrobloque 1) y los desplazamientos `tabla`.
fn filtrar(tabla: &[DesplazamientosFiltro]) -> Result<Vec<u8>, FueraDeRango> {
    let (mb_w, mb_h) = (2, 1);
    let mut y: Vec<u8> = (0..16 * 32)
        .map(|i| if i % 32 < 16 { 100 } else { 100 + SALTO })
        .collect();
    let (mut u, mut v) = (vec![128u8; 8 * 16], vec![128u8; 8 * 16]);
    let bs = [intra_mb_bs(0, 0), intra_mb_bs(1, 0)];
    let info = BlockInfo {
        inter: &[],
        nnz: &[],
        mv: &[],
        ref_id: &[],
        mv1: &[],
        ref_id1: &[],
        w4: mb_w * 4,
        t8x8: &[],
        bs: &bs,
        poc0: &[],
        poc1: &[],
        kind: &[],
        desplazamientos: tabla,
    };
    filter_frame_rows(
        &mut y,
        &mut u,
        &mut v,
        mb_w,
        mb_h,
        0..mb_h,
        &[QP; 2],
        0,
        &info,
    )?;
    Ok(y)
}

fn par(a: i32, b: i32) -> DesplazamientosFiltro {
    DesplazamientosFiltro::nuevo(a, b).expect("par en rango")
}

#[test]
fn solo_se_construyen_pares_en_rango() {
    assert_eq!(par(12, -12).a(), 12);
    assert_eq!(par(12, -12).b(), -12);
    assert!(DesplazamientosFiltro::nuevo(13, 0).is_none());
    assert!(DesplazamientosFiltro::nuevo(0, -13).is_none());
    assert!(DesplazamientosFiltro::nuevo(300, 0).is_none());
}

#[test]
fn una_tabla_que_no_cubre_la_imagen_es_error() {
    assert_eq!(
        filtrar(&[par(0, 0)]),
        Err(FueraDeRango("desplazamientos del filtro"))
    );
}

#[test]
fn el_borde_se_filtra_con_los_desplazamientos_del_macrobloque_de_q0() {
    let sin = filtrar(&[par(0, 0), par(0, 0)]).expect("filtrar");
    let en_q0 = filtrar(&[par(0, 0), par(12, 12)]).expect("filtrar");
    let en_p0 = filtrar(&[par(12, 12), par(0, 0)]).expect("filtrar");
    // Calibración: sin desplazamiento el salto supera α y el borde queda intacto.
    assert!(sin.chunks(32).all(|f| f[15] == 100 && f[16] == 100 + SALTO));
    // Con +12 en el macrobloque de `q0` el borde se filtra; en el de `p0` no cuenta.
    assert_ne!(en_q0, sin);
    assert_eq!(en_p0, sin);
}
