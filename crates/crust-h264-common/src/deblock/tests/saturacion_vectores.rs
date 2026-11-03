//! Barrera de V4 (D-2026-11-02-3): los vectores de movimiento que entran en la
//! derivación de intensidades de borde (UIT-T H.264 §8.7.2.1) se saturan a
//! `[-16384, 16383]` cuartos de muestra, tanto al empaquetar (`pack_mb`) como en
//! el camino de baldosas (`Blk::load`).
//!
//! Con eso, para cualquier vector de entrada (también los de flujos no conformes):
//! los dos caminos dan la misma intensidad, ninguna resta desborda y el
//! despachador (núcleo AVX2 con la feature `asm`) coincide con el escalar, porque
//! la diferencia de dos componentes saturados nunca vuelve a `i16::MIN`.

use super::*;

/// Cota esperada, escrita aquí a partir del criterio publicado (±2^14) y no
/// tomada de la constante de producción.
const COTA_ESPERADA: i32 = 16384;

/// Umbral de §8.7.2.1: cuatro cuartos de muestra, una muestra entera.
const UMBRAL_CUARTOS: i64 = 4;

/// Valores `i32` de prueba: los extremos del tipo, valores que la antigua
/// truncación `as i16` convertía en otros (40000 → −25536, `i32::MAX` → −1) y los
/// dos lados de la cota.
const VALORES: [i32; 12] = [
    i32::MIN,
    i32::MAX,
    -40000,
    40000,
    -32768,
    32767,
    -16385,
    16384,
    -16384,
    16383,
    0,
    3,
];

/// Componente del vector en que se pone el valor de prueba.
#[derive(Clone, Copy, Debug)]
enum Componente {
    X0,
    Y0,
    X1,
    Y1,
}

const COMPONENTES: [Componente; 4] = [
    Componente::X0,
    Componente::Y0,
    Componente::X1,
    Componente::Y1,
];

/// Saturación de referencia, escrita aparte de la de producción.
fn saturado_esperado(v: i32) -> i32 {
    if v >= COTA_ESPERADA {
        COTA_ESPERADA - 1
    } else if v < -COTA_ESPERADA {
        -COTA_ESPERADA
    } else {
        v
    }
}

/// Una imagen de 1×1 macrobloques inter, sin coeficientes y con la misma
/// referencia en todos los bloques; todo el movimiento vale `b` salvo el bloque 1
/// (fila 0, columna 1), que vale `a` en la componente `comp`. Las únicas aristas
/// con movimiento distinto son, pues, 0–1 y 1–2 (verticales) y 1–5 (horizontal).
struct Imagen {
    inter: Vec<bool>,
    nnz: Vec<u8>,
    mv: Vec<(i32, i32)>,
    ref_id: Vec<i32>,
    mv1: Vec<(i32, i32)>,
    ref_id1: Vec<i32>,
}

impl Imagen {
    fn nueva(comp: Componente, dos_listas: bool, a: i32, b: i32) -> Self {
        let pon = |v: &mut [(i32, i32)], x: bool| {
            for (k, mv) in v.iter_mut().enumerate() {
                let valor = if k == 1 { a } else { b };
                if x {
                    mv.0 = valor;
                } else {
                    mv.1 = valor;
                }
            }
        };
        let mut mv = vec![(0, 0); 16];
        let mut mv1 = vec![(0, 0); 16];
        match comp {
            Componente::X0 => pon(&mut mv, true),
            Componente::Y0 => pon(&mut mv, false),
            Componente::X1 => pon(&mut mv1, true),
            Componente::Y1 => pon(&mut mv1, false),
        }
        Imagen {
            inter: vec![true; 16],
            nnz: vec![0; 16],
            mv,
            ref_id: vec![0; 16],
            mv1,
            // Lista 1 con otra imagen de referencia (identidad 1), o sin usar.
            ref_id1: vec![if dos_listas { 1 } else { NO_REF }; 16],
        }
    }

    fn info(&self) -> BlockInfo<'_> {
        BlockInfo {
            inter: &self.inter,
            nnz: &self.nnz,
            mv: &self.mv,
            ref_id: &self.ref_id,
            mv1: &self.mv1,
            ref_id1: &self.ref_id1,
            w4: 4,
            t8x8: &[false],
            bs: &[],
            poc0: &[],
            poc1: &[],
            kind: &[],
            desplazamientos: &[],
        }
    }
}

/// Casos: cada componente con cada par de valores; las componentes de la lista 1
/// solo tienen sentido con dos listas, las de la lista 0 se prueban con una y dos.
fn casos() -> impl Iterator<Item = (Componente, bool, i32, i32)> {
    COMPONENTES.into_iter().flat_map(|comp| {
        let listas: &[bool] = match comp {
            Componente::X0 | Componente::Y0 => &[false, true],
            Componente::X1 | Componente::Y1 => &[true],
        };
        listas.iter().flat_map(move |&dos| {
            VALORES
                .into_iter()
                .flat_map(move |a| VALORES.into_iter().map(move |b| (comp, dos, a, b)))
        })
    })
}

/// Suma de todas las intensidades internas de un macrobloque.
fn suma_interna(v: &[[i32; 4]; 4], h: &[[i32; 4]; 4]) -> i32 {
    v[1..].iter().chain(&h[1..]).flatten().sum()
}

#[test]
fn el_empaquetado_satura_cada_componente() {
    let mut comprobados = 0;
    for (comp, dos, a, b) in casos() {
        let img = Imagen::nueva(comp, dos, a, b);
        let packs = pack_frame(&img.info(), 1, 1).expect("imagen de 1×1 completa");
        let p = &packs[0];
        let (campo, entrada): (&[i16; 16], &[(i32, i32)]) = match comp {
            Componente::X0 => (&p.mvx, &img.mv),
            Componente::Y0 => (&p.mvy, &img.mv),
            Componente::X1 => (&p.mvx1, &img.mv1),
            Componente::Y1 => (&p.mvy1, &img.mv1),
        };
        for (k, &empaquetado) in campo.iter().enumerate() {
            let original = match comp {
                Componente::X0 | Componente::X1 => entrada[k].0,
                Componente::Y0 | Componente::Y1 => entrada[k].1,
            };
            assert_eq!(
                i32::from(empaquetado),
                saturado_esperado(original),
                "{comp:?} dos_listas={dos} bloque {k}: {original}"
            );
        }
        comprobados += 1;
    }
    // Calibración: 2 componentes × 2 modos + 2 componentes × 1 modo, 12×12 pares.
    assert_eq!(comprobados, 6 * 144);
}

#[test]
fn baldosas_y_empaquetado_dan_la_misma_intensidad_sin_desbordar() {
    for (comp, dos, a, b) in casos() {
        let img = Imagen::nueva(comp, dos, a, b);
        let info = img.info();
        let lejos = (i64::from(saturado_esperado(a)) - i64::from(saturado_esperado(b))).abs()
            >= UMBRAL_CUARTOS;
        let esperada = if lejos { 3 } else { 0 };

        let baldosas = derive_mb(&info, 0, 0, false).expect("imagen de 1×1 completa");
        let (bv, bh) = (
            baldosas.v.map(|f| f.map(i32::from)),
            baldosas.h.map(|f| f.map(i32::from)),
        );
        assert_eq!(
            suma_interna(&bv, &bh),
            esperada,
            "baldosas: {comp:?} dos_listas={dos} a={a} b={b}"
        );

        let packs = pack_frame(&info, 1, 1).expect("imagen de 1×1 completa");
        let (mut pv, mut ph) = ([[0i32; 4]; 4], [[0i32; 4]; 4]);
        derive_mb_packed(&packs, 1, 0, 0, false, &mut pv, &mut ph);
        assert_eq!(
            (pv, ph),
            (bv, bh),
            "empaquetado != baldosas: {comp:?} dos_listas={dos} a={a} b={b}"
        );
    }
}

/// Antes `despachador_igual_a_escalar_cuando_la_diferencia_vuelve_a_minimo`, con
/// `#[ignore]` en `desbordamiento_movimiento.rs` (D-2026-11-02-3). Ahora parte de
/// los vectores `i32` reales y pasa por `pack_mb`: los pares que con la antigua
/// truncación daban una diferencia `i16::MIN` ya no llegan así al núcleo.
#[test]
fn empaquetado_evita_que_la_diferencia_vuelva_a_minimo() {
    // Pares que con `as i16` volvían a `i16::MIN`: 0 − (−32768) y 32767 − (−1),
    // y 40000 (→ −25536) frente a 7232.
    for (a, b) in [(0, -32768), (32767, -1), (40000, 7232)] {
        assert_eq!(
            (a as i16).wrapping_sub(b as i16),
            i16::MIN,
            "calibración a={a} b={b}"
        );
        for dos in [false, true] {
            let img = Imagen::nueva(Componente::X0, dos, a, b);
            let packs = pack_frame(&img.info(), 1, 1).expect("imagen de 1×1 completa");
            assert_eq!(
                bs_motion_masks(&packs[0]),
                bs_motion_masks_scalar(&packs[0]),
                "a={a} b={b} dos_listas={dos}: despachador (AVX2 con `asm`) != escalar"
            );
        }
    }
    // Y con los doce valores en las cuatro componentes.
    for (comp, dos, a, b) in casos() {
        let img = Imagen::nueva(comp, dos, a, b);
        let packs = pack_frame(&img.info(), 1, 1).expect("imagen de 1×1 completa");
        assert_eq!(
            bs_motion_masks(&packs[0]),
            bs_motion_masks_scalar(&packs[0]),
            "{comp:?} a={a} b={b} dos_listas={dos}: despachador != escalar"
        );
    }
}
