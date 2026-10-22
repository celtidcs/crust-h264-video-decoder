//! Estado de un corte CABAC mientras se decodifican sus macrobloques: el motor aritmético, las
//! rejillas de contexto por macrobloque y la posición del macrobloque en curso.

use alloc::vec::Vec;

use crate::cabac::Cabac;
use crate::corte::FiltroCorte;
use crate::mb16::FrameDecoder;
use crate::pool::refill;

/// Rejillas de contexto CABAC por macrobloque del corte, en orden de barrido (longitud
/// `mb_w * mb_h`). Valen para los `ctxIdxInc` de los vecinos (§9.3.3.1.1) y se toman del pool del
/// decodificador al empezar el corte y se le devuelven al terminar.
///
/// Las tres de B (`mb_ref1`, `mb_mvd1`, `mb_direct`) solo existen en los cortes B: en los demás
/// están vacías (unos 292 KB menos por corte a 720p).
///
/// Vienen del pool en vez de crearse por corte (antes eran `vec![..]` nuevos, la misma clase de
/// páginas nuevas por corte que `GridPool` evita en las rejillas de imagen); `refill` deja un
/// contenido idéntico al de un vector nuevo.
pub(super) struct RejillasCorte {
    /// Categoría intra: 0 = `I_4x4/I_8x8`, 2 = `I_16x16`, 25 = `I_PCM`, 255 = no intra.
    pub(super) cat: Vec<u8>,
    /// `coded_block_pattern` de cada macrobloque.
    pub(super) mb_cbp: Vec<u8>,
    /// `intra_chroma_pred_mode` (−1 = no intra).
    pub(super) cmode: Vec<i32>,
    /// Coeficientes no nulos: 16 de luma en orden de barrido y 8 de croma.
    pub(super) mb_nzc: Vec<[u8; 24]>,
    /// `coded_block_flag` de los bloques DC.
    pub(super) cbf_dc: Vec<u16>,
    /// `mb_skip_flag`.
    pub(super) mb_skip: Vec<bool>,
    /// Referencia de la lista 0 de cada bloque 4×4 (−1 = intra).
    pub(super) mb_ref: Vec<[i8; 16]>,
    /// Diferencia de vector de la lista 0 de cada bloque 4×4 (para el `ctxIdxInc` de `mvd`).
    pub(super) mb_mvd: Vec<[[i16; 2]; 16]>,
    /// Referencia de la lista 1 (solo cortes B).
    pub(super) mb_ref1: Vec<[i8; 16]>,
    /// Diferencia de vector de la lista 1 (solo cortes B).
    pub(super) mb_mvd1: Vec<[[i16; 2]; 16]>,
    /// Macrobloque directo o saltado (solo cortes B).
    pub(super) mb_direct: Vec<bool>,
    /// Si el corte es B y lleva las tres rejillas de B.
    con_b: bool,
}

impl RejillasCorte {
    /// Toma las rejillas del pool de `fd` y las rellena para `total` macrobloques con sus valores
    /// de «no disponible». El contenido es idéntico al de rejillas nuevas; solo se reutiliza la
    /// memoria.
    pub(super) fn tomar(fd: &mut FrameDecoder, total: usize) -> Self {
        let con_b = fd.is_b;
        Self {
            cat: refill(
                core::mem::take(&mut fd.sc_cat),
                total,
                crate::constantes::CATEGORIA_MB_NO_DISPONIBLE,
            ),
            mb_cbp: refill(core::mem::take(&mut fd.sc_cbp), total, 0u8),
            cmode: refill(core::mem::take(&mut fd.sc_cmode), total, -1i32),
            mb_nzc: refill(core::mem::take(&mut fd.sc_nzc), total, [0u8; 24]),
            cbf_dc: refill(core::mem::take(&mut fd.sc_cbfdc), total, 0u16),
            mb_skip: refill(core::mem::take(&mut fd.sc_skip), total, false),
            mb_ref: refill(core::mem::take(&mut fd.sc_ref), total, [-1i8; 16]),
            mb_mvd: refill(core::mem::take(&mut fd.sc_mvd), total, [[0i16; 2]; 16]),
            mb_ref1: if con_b {
                refill(core::mem::take(&mut fd.sc_ref1), total, [-1i8; 16])
            } else {
                Vec::new()
            },
            mb_mvd1: if con_b {
                refill(core::mem::take(&mut fd.sc_mvd1), total, [[0i16; 2]; 16])
            } else {
                Vec::new()
            },
            mb_direct: if con_b {
                refill(core::mem::take(&mut fd.sc_direct), total, false)
            } else {
                Vec::new()
            },
            con_b,
        }
    }

    /// Devuelve las rejillas al pool de `fd`. Las de B solo se devuelven si el corte las tomó:
    /// las vacías de un corte P o I no deben sustituir a las del pool.
    pub(super) fn devolver(self, fd: &mut FrameDecoder) {
        fd.sc_cat = self.cat;
        fd.sc_cbp = self.mb_cbp;
        fd.sc_cmode = self.cmode;
        fd.sc_nzc = self.mb_nzc;
        fd.sc_cbfdc = self.cbf_dc;
        fd.sc_skip = self.mb_skip;
        fd.sc_ref = self.mb_ref;
        fd.sc_mvd = self.mb_mvd;
        if self.con_b {
            fd.sc_ref1 = self.mb_ref1;
            fd.sc_mvd1 = self.mb_mvd1;
            fd.sc_direct = self.mb_direct;
        }
    }
}

/// Datos de la cabecera del corte con los que arranca su decodificación CABAC.
pub struct ArranqueCorte<'a> {
    /// RBSP del corte.
    pub rbsp: &'a [u8],
    /// Primer byte de `slice_data()` (tras `cabac_alignment_one_bit`).
    pub start_byte: usize,
    /// `SliceQPY`: inicializa los contextos (§9.3.1.1).
    pub slice_qp: u8,
    /// `cabac_init_idc` (tabla de inicialización de contextos de P y B).
    pub cabac_init_idc: u32,
    /// Corte I (o SI): usa su propia tabla de inicialización.
    pub is_i: bool,
    /// Corte P.
    pub is_p: bool,
    /// `first_mb_in_slice`.
    pub first_mb: usize,
    /// Parámetros del filtro de desbloqueo de este corte.
    pub filtro: FiltroCorte,
}

/// Lo que dura todo el corte mientras se decodifican sus macrobloques.
pub(super) struct CorteCabac<'a> {
    /// Motor aritmético CABAC con sus contextos.
    pub(super) cab: Cabac<'a>,
    /// Rejillas de contexto por macrobloque.
    pub(super) rejillas: RejillasCorte,
    /// `mb_qp_delta` del macrobloque anterior en orden de decodificación (su `ctxIdxInc`,
    /// §9.3.3.1.1.5); 0 si no lo llevaba.
    pub(super) last_delta_qp: i32,
    /// RBSP del corte (para leer las muestras de un `I_PCM`).
    pub rbsp: &'a [u8],
    /// Si las dos listas de referencias tienen alguna imagen (requisito de un B inter).
    pub(super) b_refs_ok: bool,
    /// Predicción de luma del macrobloque B en curso (memoria reutilizada entre macrobloques).
    pub(super) pred_y: [u8; 256],
    /// Predicción de croma del macrobloque B en curso.
    pub(super) c_pred: [[u8; 64]; 2],
}

/// Posición del macrobloque en curso y sus vecinos A (izquierda) y B (arriba) cuando están
/// disponibles: dentro de la imagen y del corte (§6.4.9).
#[derive(Clone, Copy)]
pub(super) struct PosicionMb {
    /// Dirección en orden de barrido.
    pub(super) addr: usize,
    /// Columna (en macrobloques).
    pub(super) mbx: usize,
    /// Fila (en macrobloques).
    pub(super) mby: usize,
    /// Dirección del vecino izquierdo, si está disponible.
    pub(super) left: Option<usize>,
    /// Dirección del vecino de arriba, si está disponible.
    pub(super) top: Option<usize>,
}

/// Cómo termina la decodificación de un macrobloque de un corte P o B.
pub(super) enum ResultadoMb {
    /// Decodificado; lleva `end_of_slice_flag`.
    Terminado(bool),
    /// Es intra: falta decodificarlo, con este `mb_type` en la numeración del corte I.
    Intra(u32),
}
