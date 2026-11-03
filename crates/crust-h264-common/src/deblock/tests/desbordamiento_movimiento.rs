//! Barrera de V3 (D-2026-11-02-2): la prueba de movimiento del desbloqueo
//! (UIT-T H.264 §8.7.2.1, `|Δmv| >= 4` en cuartos de píxel) no puede entrar en
//! pánico cuando la diferencia de dos vectores de un `MbPack` construido a mano no
//! cabe en `i16`.
//!
//! Lo que produce `pack_mb` ya no llega a esas diferencias (V4, D-2026-11-02-3:
//! vectores saturados a ±2^14; barrera en `saturacion_vectores.rs`). Aquí se
//! prueban paquetes hechos a mano con la semántica de vuelta en `i16`
//! (`wrapping_sub`) de `pk_differs`, para que siga sin haber pánico.

use super::*;

/// Pares `(a, b)` de componentes cuya diferencia `a - b` desborda `i16`, más los
/// dos bordes del umbral tras la vuelta (−4 lejos, −3 cerca) y el caso que vuelve
/// exactamente a `i16::MIN`.
const PARES_EXTREMOS: [(i16, i16); 9] = [
    (i16::MAX, i16::MIN), // 65535 → vuelve a −1
    (i16::MIN, i16::MAX), // −65535 → vuelve a 1
    (20000, -20000),      // 40000 → vuelve a −25536
    (-20000, 20000),      // −40000 → vuelve a 25536
    (i16::MIN, 1),        // −32769 → vuelve a 32767
    (0, i16::MIN),        // 32768 → vuelve a i16::MIN
    (32764, i16::MIN),    // 65532 → vuelve a −4 (lejos)
    (32765, i16::MIN),    // 65533 → vuelve a −3 (cerca)
    (i16::MAX, -1),       // 32768 → vuelve a i16::MIN
];

/// Referencia de la decisión, escrita aparte de la de producción: diferencia con
/// vuelta en `i16` y valor absoluto en `i32`, donde `i16::MIN` sí tiene opuesto.
fn lejos_esperado(a: i16, b: i16) -> bool {
    /// Umbral de §8.7.2.1: cuatro cuartos de píxel, un píxel entero.
    const UMBRAL_CUARTOS: i32 = 4;
    i32::from(a.wrapping_sub(b)).abs() >= UMBRAL_CUARTOS
}

/// Paquete de una sola lista con la misma referencia en los 16 bloques y todo el
/// movimiento a cero, para que solo cuenten los bloques que cada caso cambia.
fn paquete_una_lista() -> MbPack {
    let mut p = MbPack {
        inter: true,
        ..Default::default()
    };
    p.ref_id = [0; 16];
    p.ref1 = [NO_REF; 16];
    p.l1_used = 0;
    p
}

#[test]
fn pk_differs_sin_panico_con_diferencias_fuera_de_i16() {
    for (a, b) in PARES_EXTREMOS {
        for eje_y in [false, true] {
            let mut p = paquete_una_lista();
            let mut q = paquete_una_lista();
            if eje_y {
                p.mvy[0] = a;
                q.mvy[0] = b;
            } else {
                p.mvx[0] = a;
                q.mvx[0] = b;
            }
            assert_eq!(
                pk_differs(&p, 0, &q, 0),
                lejos_esperado(a, b),
                "a={a} b={b} eje_y={eje_y}"
            );
        }
    }
}

/// Paquete con todo el movimiento horizontal a `b` salvo el bloque 1, que vale `a`:
/// así las únicas aristas con movimiento distinto son 1–0 y 2–1 (izquierda) y 5–1
/// (arriba), y todas comparan `a` con `b`.
fn paquete_par(a: i16, b: i16) -> MbPack {
    let mut p = paquete_una_lista();
    p.mvx = [b; 16];
    p.mvx[1] = a;
    p
}

/// `true` si la diferencia de `a` y `b`, con vuelta en `i16`, es exactamente
/// `i16::MIN`: el único caso en que el núcleo AVX2 discrepa de `pk_differs` con un
/// paquete hecho a mano (D-2026-11-02-3). `pack_mb` no produce esos pares: lo vigila
/// `saturacion_vectores::empaquetado_evita_que_la_diferencia_vuelva_a_minimo`.
fn vuelve_a_minimo(a: i16, b: i16) -> bool {
    a.wrapping_sub(b) == i16::MIN || b.wrapping_sub(a) == i16::MIN
}

#[test]
fn despachador_igual_a_escalar_con_diferencias_fuera_de_i16() {
    let pares = PARES_EXTREMOS
        .iter()
        .filter(|&&(a, b)| !vuelve_a_minimo(a, b));
    let mut comprobados = 0;
    for &(a, b) in pares {
        let p = paquete_par(a, b);
        assert_eq!(
            bs_motion_masks(&p),
            bs_motion_masks_scalar(&p),
            "a={a} b={b}: despachador (AVX2 con `asm`) != escalar"
        );
        comprobados += 1;
    }
    // Calibración: el filtro no puede dejar la prueba vacía en silencio.
    assert_eq!(comprobados, 7);
}
