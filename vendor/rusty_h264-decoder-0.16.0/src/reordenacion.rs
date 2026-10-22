//! Reordenación de salida de H.264: el decodificador entrega las imágenes en orden de
//! decodificación y aquí se devuelven en orden de presentación (`PicOrderCnt`), guardando como
//! mucho las que caben en el búfer de imágenes decodificadas (DPB) del nivel del flujo.
//!
//! Es el «vaciado» del DPB de la norma (UIT-T H.264, anexo C.4.5.3): cuando el búfer se llena
//! sale la de menor POC, y una IDR o una `memory_management_control_operation` 5 vacían todo lo
//! pendiente. Sin la cota, un grupo de imágenes largo retendría el vídeo entero en memoria.
//!
//! El módulo no depende de nada del resto de WoDW: el banco de equivalencia del decodificador lo
//! incluye tal cual (`#[path]`) para comprobarlo sobre la red de flujos de referencia.

/// Máximo de imágenes del DPB en cualquier nivel (anexo A.3.1: `max_dec_frame_buffering` ≤ 16).
pub const MAX_IMAGENES_DPB: usize = 16;

/// Lado de un macrobloque en muestras de luma.
const LADO_MACROBLOQUE: usize = 16;

/// `MaxDpbMbs` de cada `level_idc` (UIT-T H.264, tabla A-1). El 9 es el nivel 1b; un 11 puede
/// serlo también (con `constraint_set3_flag`), y se toma el valor del 1.1, que es mayor.
const MAX_DPB_MBS: [(u8, usize); 20] = [
    (9, 396),
    (10, 396),
    (11, 900),
    (12, 2_376),
    (13, 2_376),
    (20, 2_376),
    (21, 4_752),
    (22, 8_100),
    (30, 8_100),
    (31, 18_000),
    (32, 20_480),
    (40, 32_768),
    (41, 32_768),
    (42, 34_816),
    (50, 110_400),
    (51, 184_320),
    (52, 184_320),
    (60, 696_320),
    (61, 696_320),
    (62, 696_320),
];

/// Imágenes que caben en el DPB de un flujo de nivel `level_idc` con imágenes de
/// `ancho`×`alto` muestras de luma (anexo A.3.1, ecuación A-3), entre 1 y [`MAX_IMAGENES_DPB`].
/// Un nivel desconocido da el máximo: ordena bien cualquier flujo válido a costa de más memoria.
#[must_use]
pub fn capacidad_dpb(level_idc: u8, ancho: usize, alto: usize) -> usize {
    let Some(&(_, max_mbs)) = MAX_DPB_MBS.iter().find(|(nivel, _)| *nivel == level_idc) else {
        return MAX_IMAGENES_DPB;
    };
    let macrobloques = ancho.div_ceil(LADO_MACROBLOQUE) * alto.div_ceil(LADO_MACROBLOQUE);
    (max_mbs / macrobloques.max(1)).clamp(1, MAX_IMAGENES_DPB)
}

/// Imágenes decodificadas pendientes de salir, con su `PicOrderCnt`.
///
/// Invariante: nunca guarda más de `capacidad` imágenes, y cada una que sale tiene el menor POC
/// de las pendientes en ese momento.
pub struct ReordenacionH264<T> {
    /// Máximo de imágenes retenidas (el DPB del nivel).
    capacidad: usize,
    /// Pendientes en orden de llegada.
    pendientes: Vec<(i32, T)>,
}

impl<T> ReordenacionH264<T> {
    /// Búfer vacío que retiene como mucho `capacidad` imágenes (acotada a 1..=16).
    #[must_use]
    pub fn new(capacidad: usize) -> Self {
        let capacidad = capacidad.clamp(1, MAX_IMAGENES_DPB);
        Self {
            capacidad,
            pendientes: Vec::with_capacity(capacidad + 1),
        }
    }

    /// Añade una imagen recién decodificada y devuelve las que ya pueden salir, en orden de
    /// presentación: todas si la imagen lleva una MMCO 5 (`mmco5`, que reinicia el POC), y si no,
    /// las de menor POC que desborden la capacidad.
    pub fn entrar(&mut self, poc: i32, imagen: T, mmco5: bool) -> Vec<T> {
        self.pendientes.push((poc, imagen));
        if mmco5 {
            return self.vaciar();
        }
        let mut salida = Vec::new();
        while self.pendientes.len() > self.capacidad {
            salida.extend(self.sacar_menor());
        }
        salida
    }

    /// Todas las pendientes en orden de presentación: al llegar una IDR y al terminar el flujo.
    pub fn vaciar(&mut self) -> Vec<T> {
        // Orden estable: a igual POC sale primero la que llegó antes.
        self.pendientes.sort_by_key(|(poc, _)| *poc);
        self.pendientes
            .drain(..)
            .map(|(_, imagen)| imagen)
            .collect()
    }

    /// Imágenes retenidas ahora mismo (para las pruebas de la cota).
    #[cfg(test)]
    #[must_use]
    pub fn retenidas(&self) -> usize {
        self.pendientes.len()
    }

    /// Saca la pendiente de menor POC (la primera que llegó si hay empate).
    fn sacar_menor(&mut self) -> Option<T> {
        let indice = self
            .pendientes
            .iter()
            .enumerate()
            .min_by_key(|(i, (poc, _))| (*poc, *i))
            .map(|(i, _)| i)?;
        Some(self.pendientes.remove(indice).1)
    }
}

/// Orden de presentación de un flujo H.264 entero: un [`ReordenacionH264`] por secuencia (de una
/// IDR a la siguiente), creado con la primera imagen, cuando ya se conocen sus dimensiones.
pub struct OrdenPresentacionH264<T> {
    /// `level_idc` del flujo: con las dimensiones, fija la capacidad de cada búfer.
    nivel: u8,
    /// Búfer de la secuencia en curso (ninguno antes de su primera imagen).
    bufer: Option<ReordenacionH264<T>>,
}

impl<T> OrdenPresentacionH264<T> {
    /// Orden de presentación para un flujo de nivel `level_idc`.
    #[must_use]
    pub fn new(level_idc: u8) -> Self {
        Self {
            nivel: level_idc,
            bufer: None,
        }
    }

    /// Empieza una unidad de acceso: si es una IDR (`es_idr`), devuelve todo lo pendiente de la
    /// secuencia anterior, en orden, y la siguiente imagen abre un búfer nuevo.
    pub fn empezar_unidad(&mut self, es_idr: bool) -> Vec<T> {
        if es_idr { self.vaciar() } else { Vec::new() }
    }

    /// Añade una imagen decodificada de `ancho`×`alto` con su POC y su indicador de MMCO 5, y
    /// devuelve las que ya pueden salir (véase [`ReordenacionH264::entrar`]).
    pub fn entrar(
        &mut self,
        imagen: T,
        (ancho, alto): (usize, usize),
        poc: i32,
        mmco5: bool,
    ) -> Vec<T> {
        let nivel = self.nivel;
        self.bufer
            .get_or_insert_with(|| ReordenacionH264::new(capacidad_dpb(nivel, ancho, alto)))
            .entrar(poc, imagen, mmco5)
    }

    /// Todo lo pendiente en orden de presentación (al terminar el flujo).
    pub fn vaciar(&mut self) -> Vec<T> {
        self.bufer.take().map_or_else(Vec::new, |mut b| b.vaciar())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mete `(poc, id)` en orden de decodificación y devuelve los `id` en el orden en que salen.
    fn orden_de_salida(capacidad: usize, entradas: &[(i32, u32, bool)]) -> Vec<u32> {
        let mut b = ReordenacionH264::new(capacidad);
        let mut salida = Vec::new();
        for &(poc, id, mmco5) in entradas {
            salida.extend(b.entrar(poc, id, mmco5));
            assert!(b.retenidas() <= capacidad, "retiene más que su capacidad");
        }
        salida.extend(b.vaciar());
        salida
    }

    #[test]
    fn ordena_las_b_por_poc() {
        // I0 P6 B2 B4 P12 B8 B10 (POC), decodificadas en ese orden.
        let entradas = [
            (0, 0, false),
            (6, 1, false),
            (2, 2, false),
            (4, 3, false),
            (12, 4, false),
            (8, 5, false),
            (10, 6, false),
        ];
        assert_eq!(orden_de_salida(2, &entradas), vec![0, 2, 3, 1, 5, 6, 4]);
    }

    #[test]
    fn no_retiene_mas_que_su_capacidad_y_saca_la_menor() {
        let mut b = ReordenacionH264::new(2);
        assert!(b.entrar(4, 'a', false).is_empty());
        assert!(b.entrar(2, 'b', false).is_empty());
        assert_eq!(b.entrar(6, 'c', false), vec!['b']);
        assert_eq!(b.retenidas(), 2);
    }

    #[test]
    fn mmco5_vacia_todo_en_orden() {
        let mut b = ReordenacionH264::new(4);
        assert!(b.entrar(8, 1, false).is_empty());
        assert!(b.entrar(4, 2, false).is_empty());
        assert_eq!(b.entrar(6, 3, true), vec![2, 3, 1]);
        assert_eq!(b.retenidas(), 0);
    }

    #[test]
    fn vaciar_es_estable_a_igual_poc() {
        let mut b = ReordenacionH264::new(4);
        assert!(b.entrar(2, 'x', false).is_empty());
        assert!(b.entrar(2, 'y', false).is_empty());
        assert_eq!(b.vaciar(), vec!['x', 'y']);
    }

    #[test]
    fn la_capacidad_se_acota_entre_uno_y_dieciseis() {
        assert_eq!(ReordenacionH264::<u8>::new(0).capacidad, 1);
        assert_eq!(ReordenacionH264::<u8>::new(99).capacidad, MAX_IMAGENES_DPB);
    }

    #[test]
    fn una_idr_saca_antes_lo_pendiente_de_la_secuencia_anterior() {
        // Nivel 4.0 en 1080p: capacidad 4, así que nada sale solo.
        let mut orden = OrdenPresentacionH264::new(40);
        let dims = (1920, 1080);
        assert!(orden.empezar_unidad(true).is_empty());
        assert!(orden.entrar('a', dims, 0, false).is_empty());
        assert!(orden.entrar('c', dims, 4, false).is_empty());
        assert!(orden.entrar('b', dims, 2, false).is_empty());
        // Nueva IDR: el POC vuelve a 0, y lo anterior sale entero y ordenado ANTES.
        assert_eq!(orden.empezar_unidad(true), vec!['a', 'b', 'c']);
        assert!(orden.empezar_unidad(false).is_empty());
        assert!(orden.entrar('d', dims, 0, false).is_empty());
        assert_eq!(orden.vaciar(), vec!['d']);
    }

    #[test]
    fn capacidad_por_nivel_y_tamano() {
        // Nivel 3.1 en 720p: 18 000 / (80 × 45) = 5.
        assert_eq!(capacidad_dpb(31, 1280, 720), 5);
        // Nivel 4.0 en 1080p (1088 en macrobloques): 32 768 / (120 × 68) = 4.
        assert_eq!(capacidad_dpb(40, 1920, 1080), 4);
        // Imagen diminuta: el tope es 16.
        assert_eq!(capacidad_dpb(51, 16, 16), MAX_IMAGENES_DPB);
        // Nivel desconocido: el tope.
        assert_eq!(capacidad_dpb(7, 1920, 1080), MAX_IMAGENES_DPB);
        // Imagen más grande que el nivel: al menos 1.
        assert_eq!(capacidad_dpb(10, 1920, 1080), 1);
    }
}
