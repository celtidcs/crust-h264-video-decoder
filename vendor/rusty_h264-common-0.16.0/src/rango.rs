//! Error de índice fuera de rango compartido por el desbloqueo y la compensación de movimiento.

/// Índice fuera de rango en una tabla, rejilla o plano que la construcción del decodificador
/// mantiene en rango: el estado ya no corresponde al flujo. Se devuelve en lugar de inventar un
/// dato (COR-3); el texto nombra la tabla. El decodificador lo convierte en su propio error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FueraDeRango(pub &'static str);
