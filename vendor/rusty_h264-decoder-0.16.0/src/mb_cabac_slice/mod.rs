//! Decodificación de los cortes CABAC (UIT-T H.264 §7.3.4, §7.3.5 y §9.3).
//!
//! `FrameDecoder::decode_slice_data_cabac` registra el corte y recorre sus macrobloques en
//! `decode_slice_cabac_inner`, que prepara el estado del corte (`estado`), reparte cada
//! macrobloque y avanza. Cada tipo de macrobloque vive en su módulo: `p` (`P_Skip` y P inter),
//! `b` (`B_Skip`, directo, `B_8x8` y particiones) e `intra` (`I_PCM`, `I_16x16`, `I_4x4` e `I_8x8`,
//! comunes a los tres tipos de corte). Aquí quedan también lo que comparten (`ctxIdxInc` de los
//! vecinos y caché de coeficientes) y la media de bits por macrobloque que usa el reparto
//! entre caminos de decodificación (`note_slice_density`).

use crate::error::MbError;
use crate::mb16::FrameDecoder;

pub use estado::ArranqueCorte;

/// `ctxIdxOffset` de `mb_skip_flag` en los cortes P (UIT-T H.264, tabla 9-34).
const CTX_MB_SKIP_P: usize = 11;
/// `ctxIdxOffset` de `mb_skip_flag` en los cortes B (tabla 9-34).
const CTX_MB_SKIP_B: usize = 24;
/// `ctxIdxOffset` del prefijo de `mb_type` en los cortes B (tabla 9-34).
const CTX_MB_TYPE_B: usize = 27;
/// `ctxIdxOffset` de `transform_size_8x8_flag` (tabla 9-34).
const CTX_TRANSFORM_8X8: usize = 399;
/// Primer `mb_type` intra de un corte P (tabla 7-13: 5 = `I_NxN` en la numeración del corte I).
const MB_TYPE_P_PRIMER_INTRA: u32 = 5;
/// `mb_type` de `I_PCM` en la numeración del corte I (tabla 7-11).
const MB_TYPE_I_PCM: u32 = 25;
/// Bytes de muestras de un `I_PCM` en 4:2:0 de 8 bits: 256 de luma y 2 × 64 de croma (§7.3.5).
const BYTES_I_PCM: usize = 384;
/// `coded_block_pattern` que se infiere para un `I_PCM`: croma 2 y luma 15 (§7.4.5).
const CBP_I_PCM: u8 = (2 << 4) | 15;
/// Coeficientes no nulos que se infieren en cada bloque de un `I_PCM` (§9.3.3.1.1.9).
const NNZ_I_PCM: u8 = 16;
/// Categorías intra de la rejilla `cat` (vecinos del `ctxIdxInc` de `mb_type` I, §9.3.3.1.1.3).
const CATEGORIA_I_NXN: u8 = 0;
/// Categoría de un `I_16x16`; con `I_PCM` son las que cuentan como «no `I_NxN`».
const CATEGORIA_I_16X16: u8 = 2;
/// Categoría de un `I_PCM`.
const CATEGORIA_I_PCM: u8 = 25;
/// Posiciones de la caché de coeficientes no nulos de un macrobloque y sus vecinos
/// (disposición de `OpenH264`, `WelsFillCacheNonZeroCount`).
const TAM_CACHE_NZC: usize = 48;
/// Valor de la caché de coeficientes para un bloque no disponible (vecino fuera del corte).
const NZC_NO_DISPONIBLE: u8 = 0xff;
/// Peso de la densidad anterior en la media exponencial de bits por macrobloque: tres cuartos
/// del valor anterior y un cuarto del corte nuevo, para que un corte atípico no cambie el modo.
const PESO_DENSIDAD_ANTERIOR: f64 = 0.75;

mod b;
mod estado;
mod intra;
mod p;

use estado::{CorteCabac, PosicionMb, RejillasCorte, ResultadoMb};

/// `condTermFlagN` de un macrobloque vecino (UIT-T H.264 08/2024, §9.3.3.1.1): 0 si el vecino no
/// está disponible (`None`); si lo está, 1 cuando su valor en `tabla` cumple la condición de la
/// cláusula del elemento sintáctico (`cumple`) y 0 si no.
///
/// # Errors
/// `MbError::Inconsistent(nombre)` si `tabla` no cubre la dirección de un vecino disponible.
#[inline]
fn cond_term_flag<T: Copy>(
    tabla: &[T],
    vecino: Option<usize>,
    cumple: impl Fn(T) -> bool,
    nombre: &'static str,
) -> Result<usize, MbError> {
    Ok(valor_vecino(tabla, vecino, nombre)?.map_or(0, |v| usize::from(cumple(v))))
}

/// Valor de `tabla` para un macrobloque vecino: `Ok(None)` si el vecino no está disponible y
/// `Ok(Some(valor))` si lo está.
///
/// # Errors
/// `MbError::Inconsistent(nombre)` si `tabla` no cubre la dirección de un vecino disponible: eso
/// no es «no disponible» y no se trata como tal.
#[inline]
fn valor_vecino<T: Copy>(
    tabla: &[T],
    vecino: Option<usize>,
    nombre: &'static str,
) -> Result<Option<T>, MbError> {
    vecino
        .map(|n| tabla.get(n).copied().ok_or(MbError::Inconsistent(nombre)))
        .transpose()
}

/// Caché de coeficientes no nulos de un macrobloque y sus vecinos A (izquierda) y B (arriba), en
/// la disposición de `OpenH264` (`WelsFillCacheNonZeroCount`): las posiciones de los bloques de los
/// vecinos disponibles llevan sus valores, las de arriba a la izquierda y arriba a la derecha 0, y
/// el resto `NZC_NO_DISPONIBLE`. De ella salen los `ctxIdxInc` de `coded_block_flag` (§9.3.3.1.1.9).
///
/// # Errors
/// `MbError::Inconsistent` si `mb_nzc` no cubre un vecino disponible.
#[inline]
fn cache_nzc_vecinos(
    mb_nzc: &[[u8; 24]],
    top: Option<usize>,
    left: Option<usize>,
) -> Result<[u8; TAM_CACHE_NZC], MbError> {
    let mut nzc = [NZC_NO_DISPONIBLE; TAM_CACHE_NZC];
    if let Some(t) = top {
        let tnz = mb_nzc.get(t).ok_or(MbError::Inconsistent("mb_nzc"))?;
        nzc[1..5].copy_from_slice(&tnz[12..16]);
        (nzc[0], nzc[5], nzc[29]) = (0, 0, 0);
        (nzc[6], nzc[7], nzc[30], nzc[31]) = (tnz[20], tnz[21], tnz[22], tnz[23]);
    }
    if let Some(l) = left {
        let lnz = mb_nzc.get(l).ok_or(MbError::Inconsistent("mb_nzc"))?;
        (nzc[8], nzc[16], nzc[24], nzc[32]) = (lnz[3], lnz[7], lnz[11], lnz[15]);
        (nzc[13], nzc[21], nzc[37], nzc[45]) = (lnz[17], lnz[21], lnz[19], lnz[23]);
    }
    Ok(nzc)
}

impl FrameDecoder {
    /// CABAC slice-data decode (docs/cabac-decode-plan.md), brought up brick by brick
    /// against the instrumented openh264 oracle. Phase 1: verify engine init; the
    /// syntax layer (Phase 2+) is WIP. `filtro` son los parámetros del filtro de desbloqueo de
    /// este corte, que quedan registrados con él.
    ///
    /// # Errors
    /// Los de la decodificación de los macrobloques del corte.
    pub fn decode_slice_data_cabac(
        &mut self,
        arranque: ArranqueCorte<'_>,
    ) -> Result<usize, MbError> {
        let first_mb = arranque.first_mb;
        let filtro = arranque.filtro;
        let bytes = arranque.rbsp.len().saturating_sub(arranque.start_byte);
        self.registrar_corte(first_mb, filtro);
        let r = self.decode_slice_cabac_inner(arranque);
        self.note_slice_density(bytes, first_mb, &r);
        r
    }

    /// Feed the D12 dispatch its density signal from a slice just decoded.
    /// Exponentially smoothed so one atypical slice cannot flip the arm, and
    /// only ever read on the NEXT slice — the current one is already committed.
    fn note_slice_density(&mut self, bytes: usize, first_mb: usize, r: &Result<usize, MbError>) {
        let Ok(end) = r else { return };
        let mbs = end.saturating_sub(first_mb);
        if mbs == 0 {
            return;
        }
        let bpm = (bytes * 8) as f64 / mbs as f64;
        self.bits_per_mb = if self.bits_per_mb == 0.0 {
            bpm
        } else {
            PESO_DENSIDAD_ANTERIOR * self.bits_per_mb + (1.0 - PESO_DENSIDAD_ANTERIOR) * bpm
        };
    }

    /// Recorre los macrobloques de un corte CABAC desde `first_mb` hasta `end_of_slice_flag` o
    /// el final de la imagen, y devuelve la dirección siguiente. Cada tipo de macrobloque se
    /// decodifica en su módulo (`p`, `b`, `intra`); aquí solo se prepara el corte, se reparte
    /// cada macrobloque y se avanza.
    ///
    /// # Errors
    /// `MbError::Truncated` si el corte empieza fuera de la imagen, y los de cada macrobloque.
    fn decode_slice_cabac_inner(&mut self, arranque: ArranqueCorte<'_>) -> Result<usize, MbError> {
        let ArranqueCorte {
            rbsp,
            start_byte,
            slice_qp,
            cabac_init_idc,
            is_i,
            is_p,
            first_mb,
            filtro: _,
        } = arranque;
        self.edc_active = true;
        let cab = crate::cabac::Cabac::new(rbsp, start_byte, slice_qp as i32, cabac_init_idc, is_i);
        let (range, _offset) = cab.dbg_state();
        debug_assert_eq!(
            range,
            crate::constantes::CABAC_RANGO_INICIAL,
            "CABAC init range"
        );

        let mbw = self.mb_w;
        let total = self.mb_w * self.mb_h;
        let rejillas = RejillasCorte::tomar(self, total);
        // Multi-slice availability (spec §6.4.x): a macroblock before this
        // slice's first_mb is NOT available — for pixel-domain prediction
        // (`nbr_in_slice`, like the CAVLC twin sets) AND for every CABAC ctx
        // neighbour below. The per-slice ctx arrays' defaults are not the
        // "unavailable" value (cat 255 reads as available-I16, mb_cbp 0 as
        // all-zero-cbp, mb_t8x8 is a frame grid…), so the `left`/`top`
        // options themselves are gated on slice membership — one gate that
        // every downstream ctxIdxInc read inherits. Confirmed against ffmpeg
        // on an x264 `--slices 4` stream, which desynced without this.
        self.slice_first_mb = first_mb;
        let mut addr = first_mb;

        // A new slice's first MBs may read grids a previous slice's deferred
        // spans still owe (P-after-B in one picture included).
        self.span_flush()?;
        // `mbw` is `pic_width_in_mbs_minus1 + 1` from the SPS. It cannot be 0 in
        // a conformant stream, but it is a runtime value, so this div+rem pair
        // carried a divide-by-zero panic reachable from a MALFORMED header --
        // on the untrusted-input path, at slice entry. `.max(1)` retires both
        // checks and cannot change a conformant decode.
        let mbw_nz = mbw.max(1);
        let (mut mbx, mut mby) = (addr % mbw_nz, addr / mbw_nz);
        // Slice-invariant B properties, formerly re-derived per macroblock.
        if self.is_b {
            self.edc_flush()?; // drain anything a preceding P slice queued
        }
        // Loop-invariant: the lowest `addr` whose top neighbour is in this slice.
        let top_lim = first_mb + mbw;
        let mut corte = CorteCabac {
            cab,
            rejillas,
            last_delta_qp: 0,
            rbsp,
            b_refs_ok: !self.refs.is_empty() && !self.refs1.is_empty(),
            // Reused across macroblocks: MC overwrites every byte it reads back (all
            // B layouts cover the full macroblock), so the per-MB 384-byte zero-init
            // these used to pay was dead. Byte-identity is the guard.
            pred_y: [0u8; 256],
            c_pred: [[0u8; 64]; 2],
        };
        // BOUND the entropy-coded loop (fuzzer-surfaced): a mutated stream can
        // never produce decode_terminate and the engine zero-fills forever. The
        // bound needs checking ONCE at entry — every in-loop path that advances
        // `addr` already breaks on `addr >= total` before continuing.
        if addr >= total {
            return Err(MbError::Truncated);
        }
        loop {
            // A REAL check, not `debug_assert`: it states `addr < total` once per
            // macroblock for everything below. (Until NUC-2 the grids were also
            // reborrowed as slices of length `total` here so that `[addr]` needed no
            // bounds check; every grid access is now `get`/`get_mut`, and removing the
            // reborrows measured 0.99–1.01 on the CABAC speed set.)
            if addr >= total {
                break;
            }
            // Carried coordinates: one compare-and-wrap replaces the per-MB
            // div+mod pair (and row_hook's own division).
            if mbx == mbw {
                mbx = 0;
                mby += 1;
            }
            self.row_hook_at(addr, mby)?;
            let pos = PosicionMb {
                addr,
                mbx,
                mby,
                left: (mbx > 0 && addr > first_mb).then(|| addr - 1),
                top: (mby > 0 && addr >= top_lim).then(|| addr - mbw),
            };
            // P y B decodifican aquí sus macrobloques inter; un intra de cualquier corte
            // converge en `mb_cabac_intra` con su `mb_type` en la numeración del corte I.
            let resultado = if is_p {
                self.mb_cabac_p(&mut corte, pos)?
            } else if self.is_b {
                self.mb_cabac_b(&mut corte, pos)?
            } else {
                ResultadoMb::Intra(Self::mb_type_cabac_i(&mut corte, pos)?)
            };
            let fin_de_corte = match resultado {
                ResultadoMb::Terminado(fin) => fin,
                ResultadoMb::Intra(mb_type) => self.mb_cabac_intra(&mut corte, pos, mb_type)?,
            };
            addr += 1;
            mbx += 1;
            if fin_de_corte || addr >= total {
                break;
            }
        }
        self.edc_flush()?; // slice end: no job crosses a slice boundary
                           // Return the pooled slice scratch (an error exit forfeits it — the
                           // next slice then refills fresh allocations, correctness unchanged).
        corte.rejillas.devolver(self);
        Ok(addr)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// condTermFlagN: 0 sin vecino o con la bandera puesta, 1 si el vecino está y no la tiene
    /// (§9.3.3.1.1.1); una dirección fuera de la tabla es incoherencia, no un 0 inventado.
    #[test]
    fn cond_term_flag_sigue_la_norma_y_rechaza_indices_fuera_de_rango() {
        let banderas = [true, false];
        let no_puesta = |b: bool| !b;
        assert_eq!(cond_term_flag(&banderas, None, no_puesta, "t"), Ok(0));
        assert_eq!(cond_term_flag(&banderas, Some(0), no_puesta, "t"), Ok(0));
        assert_eq!(cond_term_flag(&banderas, Some(1), no_puesta, "t"), Ok(1));
        assert_eq!(
            cond_term_flag(&banderas, Some(2), no_puesta, "t"),
            Err(MbError::Inconsistent("t"))
        );
    }

    /// `valor_vecino` distingue «sin vecino» (`Ok(None)`) de «tabla que no cubre al vecino».
    #[test]
    fn valor_vecino_no_confunde_indice_ausente_con_vecino_no_disponible() {
        let tabla = [7u8, 9];
        assert_eq!(valor_vecino(&tabla, None, "t"), Ok(None));
        assert_eq!(valor_vecino(&tabla, Some(1), "t"), Ok(Some(9)));
        assert_eq!(
            valor_vecino(&tabla, Some(2), "t"),
            Err(MbError::Inconsistent("t"))
        );
    }

    #[test]
    fn note_slice_density_calcula_suavizado_exponencial() {
        let mut fd = FrameDecoder::new(1, 1, 26, 0, alloc::vec::Vec::new(), 0, false, false, false);
        assert_eq!(fd.bits_per_mb, 0.0);
        fd.note_slice_density(100, 0, &Ok(10));
        // 100 bytes = 800 bits / 10 mbs = 80.0
        assert_eq!(fd.bits_per_mb, 80.0);
        fd.note_slice_density(200, 0, &Ok(10));
        // 0.75 * 80.0 + 0.25 * (1600 / 10) = 60.0 + 40.0 = 100.0
        assert_eq!(fd.bits_per_mb, 100.0);
    }

    #[test]
    fn note_slice_density_ignora_error() {
        let mut fd = FrameDecoder::new(1, 1, 26, 0, alloc::vec::Vec::new(), 0, false, false, false);
        fd.note_slice_density(100, 0, &Err(MbError::Truncated));
        assert_eq!(fd.bits_per_mb, 0.0);
    }
}
