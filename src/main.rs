//! Banco de evaluación H.264 (criterios H264-1…5) para Crusty_H264.
//!
//! Uso:
//!   banco_h264 decodificar <rusty|rusty-au|openh264> <flujo.264> [salida.yuv]
//!       `rusty-au` decodifica unidad de acceso a unidad de acceso y ordena con el búfer de
//!       reordenación acotado de Crusty_H264 (el mismo archivo, incluido con `#[path]`).
//!       Decodifica el flujo Annex B y escribe una línea de resultado:
//!       `RESULTADO <decodificador> <ok|error> fotogramas=N ancho=W alto=H ms=T pico_mb=M huellas=<sha256 de todo>`
//!       Con `salida.yuv` vuelca los fotogramas (Y,U,V compactos, sin paso) para comparar.
//!   banco_h264 mutar <entrada> <salida> <semilla> <tipo>
//!       Genera una mutación determinista del flujo (tipos: bytes, truncar, duplicar, cabecera).
//!   banco_h264 comparar <a.yuv> <b.yuv> <ancho> <alto>
//!       Compara dos volcados fotograma a fotograma:
//!       `COMPARACION fotogramas_a=N fotogramas_b=M iguales=K primera_diferencia=I|ninguna`

use sha2::{Digest, Sha256};

// El búfer de reordenación de Crusty_H264 (exportado formalmente por la biblioteca).
use crusty_h264::reordenacion as reordenacion_h264;
use std::alloc::{GlobalAlloc, Layout, System};
use std::io::Write;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

/// Asignador que cuenta el pico de memoria Rust del proceso (las reservas de C de OpenH264 no
/// pasan por aquí: el pico solo es comparable para el candidato en Rust).
struct Contador;
static ACTUAL: AtomicUsize = AtomicUsize::new(0);
static PICO: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for Contador {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(l) };
        if !p.is_null() {
            let n = ACTUAL.fetch_add(l.size(), Ordering::Relaxed) + l.size();
            PICO.fetch_max(n, Ordering::Relaxed);
        }
        p
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        unsafe { System.dealloc(p, l) };
        ACTUAL.fetch_sub(l.size(), Ordering::Relaxed);
    }
}
#[global_allocator]
static ASIGNADOR: Contador = Contador;

/// Fotograma compacto: ancho, alto y planos Y, U, V sin relleno.
struct Fotograma {
    ancho: usize,
    alto: usize,
    datos: Vec<u8>,
}

/// `level_idc` del primer SPS del flujo (cuarto byte de su NAL), o 0 si no hay ninguno.
fn nivel_del_primer_sps(flujo: &[u8]) -> u8 {
    /// `nal_unit_type` del SPS.
    const TIPO_SPS: u8 = 7;
    flujo
        .windows(7)
        .find(|w| w[..3] == [0, 0, 1] && w[3] & 0x1F == TIPO_SPS)
        .map_or(0, |w| w[6])
}

/// Decodificación unidad de acceso a unidad de acceso, con su mismo orden de presentación
/// (vaciado en cada IDR y búfer acotado al DPB del nivel).
fn decodificar_rusty_au(flujo: &[u8]) -> Result<Vec<Fotograma>, String> {
    let mut dec = crusty_h264::Decoder::new();
    let mut orden = reordenacion_h264::OrdenPresentacionH264::new(nivel_del_primer_sps(flujo));
    let mut fotos = Vec::new();
    for au in crusty_h264::split_access_units(flujo) {
        fotos.extend(orden.empezar_unidad(crusty_h264::au_is_idr(au)));
        if let Some(f) = dec.decode(au).map_err(|e| format!("{e:?}"))? {
            let dims = (f.width, f.height);
            fotos.extend(orden.entrar(f, dims, dec.last_poc(), dec.last_has_mmco5()));
        }
    }
    fotos.extend(orden.vaciar());
    Ok(fotos
        .into_iter()
        .map(|f| {
            let mut datos = f.y;
            datos.extend_from_slice(&f.u);
            datos.extend_from_slice(&f.v);
            Fotograma {
                ancho: f.width,
                alto: f.height,
                datos,
            }
        })
        .collect())
}

fn decodificar_rusty(flujo: &[u8]) -> Result<Vec<Fotograma>, String> {
    let mut dec = crusty_h264::Decoder::new();
    let fotos = dec.decode_stream(flujo).map_err(|e| format!("{e:?}"))?;
    Ok(fotos
        .into_iter()
        .map(|f| {
            let mut datos = f.y;
            datos.extend_from_slice(&f.u);
            datos.extend_from_slice(&f.v);
            Fotograma {
                ancho: f.width,
                alto: f.height,
                datos,
            }
        })
        .collect())
}

fn compactar(img: &openh264::decoder::DecodedYUV<'_>) -> Fotograma {
    use openh264::formats::YUVSource;
    let (ancho, alto) = img.dimensions();
    let (py, pu, pv) = img.strides();
    let mut datos = Vec::with_capacity(ancho * alto * 3 / 2);
    for f in 0..alto {
        datos.extend_from_slice(&img.y()[f * py..f * py + ancho]);
    }
    let (ac, hc) = (ancho.div_ceil(2), alto.div_ceil(2));
    for f in 0..hc {
        datos.extend_from_slice(&img.u()[f * pu..f * pu + ac]);
    }
    for f in 0..hc {
        datos.extend_from_slice(&img.v()[f * pv..f * pv + ac]);
    }
    Fotograma { ancho, alto, datos }
}

/// OpenH264 de referencia: API desde la fuente, sin vaciado tras cada paquete, recogida final.
fn decodificar_openh264(flujo: &[u8], por_au: bool) -> Result<Vec<Fotograma>, String> {
    let cfg = openh264::decoder::DecoderConfig::new()
        .flush_after_decode(openh264::decoder::Flush::NoFlush);
    let mut dec =
        openh264::decoder::Decoder::with_api_config(openh264::OpenH264API::from_source(), cfg)
            .map_err(|e| e.to_string())?;
    let mut salida = Vec::new();
    let paquetes: Vec<&[u8]> = if por_au {
        crusty_h264::split_access_units(flujo)
    } else {
        openh264::nal_units(flujo).collect()
    };
    for nal in paquetes {
        match dec.decode(nal) {
            Ok(Some(img)) => salida.push(compactar(&img)),
            Ok(None) => {}
            Err(e) => return Err(e.to_string()),
        }
    }
    for img in dec.flush_remaining().map_err(|e| e.to_string())? {
        salida.push(compactar(&img));
    }
    Ok(salida)
}

fn decodificar(args: &[String]) {
    let flujo = std::fs::read(&args[3]).expect("leer flujo");
    let inicio = Instant::now();
    let resultado = match args[2].as_str() {
        "rusty" => decodificar_rusty(&flujo),
        "rusty-au" => decodificar_rusty_au(&flujo),
        "openh264" => decodificar_openh264(&flujo, false),
        "openh264-au" => decodificar_openh264(&flujo, true),
        otro => panic!("decodificador desconocido {otro}"),
    };
    let ms = inicio.elapsed().as_millis();
    let pico_mb = PICO.load(Ordering::Relaxed) / (1024 * 1024);
    match resultado {
        Ok(fotos) => {
            let mut h = Sha256::new();
            let mut archivo = args
                .get(4)
                .map(|r| std::io::BufWriter::new(std::fs::File::create(r).expect("crear salida")));
            for f in &fotos {
                h.update(&f.datos);
                if let Some(a) = archivo.as_mut() {
                    a.write_all(&f.datos).expect("escribir");
                }
            }
            let (w, al) = fotos.first().map_or((0, 0), |f| (f.ancho, f.alto));
            let tamanos_distintos = fotos.iter().any(|f| f.ancho != w || f.alto != al);
            println!(
                "RESULTADO {} ok fotogramas={} ancho={w} alto={al} ms={ms} pico_mb={pico_mb} tamanos_variables={tamanos_distintos} huella={}",
                args[2],
                fotos.len(),
                h.finalize()
                    .iter()
                    .map(|b| format!("{b:02x}"))
                    .collect::<String>()
            );
        }
        Err(e) => println!(
            "RESULTADO {} error ms={ms} pico_mb={pico_mb} motivo={}",
            args[2],
            e.replace(char::is_whitespace, "_")
        ),
    }
}

/// Generador congruencial lineal (Knuth MMIX): determinista y sin dependencias.
fn siguiente(estado: &mut u64) -> u64 {
    *estado = estado
        .wrapping_mul(6_364_136_223_846_793_005)
        .wrapping_add(1_442_695_040_888_963_407);
    *estado >> 33
}

fn mutar(args: &[String]) {
    let mut d = std::fs::read(&args[2]).expect("leer");
    let mut s: u64 = args[4].parse().expect("semilla");
    let n = d.len().max(1) as u64;
    match args[5].as_str() {
        "bytes" => {
            let veces = 1 + siguiente(&mut s) % 16;
            for _ in 0..veces {
                let i = (siguiente(&mut s) % n) as usize;
                d[i] = siguiente(&mut s) as u8;
            }
        }
        "truncar" => {
            let i = (siguiente(&mut s) % n) as usize;
            d.truncate(i);
        }
        "duplicar" => {
            let a = (siguiente(&mut s) % n) as usize;
            let largo = (siguiente(&mut s) % 4096) as usize;
            let b = (a + largo).min(d.len());
            let trozo = d[a..b].to_vec();
            let i = (siguiente(&mut s) % n) as usize;
            d.splice(i..i, trozo);
        }
        // Daña los primeros bytes tras cada código de inicio de las 8 primeras NAL: ataca SPS/PPS
        // y cabeceras de corte, donde viven tamaños, recuentos y desplazamientos.
        "cabecera" => {
            let mut vistos = 0;
            let mut i = 0;
            while i + 4 < d.len() && vistos < 8 {
                if d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1 {
                    let j = i + 3 + 1 + (siguiente(&mut s) % 6) as usize;
                    if j < d.len() {
                        d[j] = siguiente(&mut s) as u8;
                    }
                    vistos += 1;
                    i += 3;
                } else {
                    i += 1;
                }
            }
        }
        otro => panic!("tipo desconocido {otro}"),
    }
    std::fs::write(&args[3], d).expect("escribir");
}

fn comparar(args: &[String]) {
    let a = std::fs::read(&args[2]).expect("a");
    let b = std::fs::read(&args[3]).expect("b");
    let (w, h): (usize, usize) = (args[4].parse().unwrap(), args[5].parse().unwrap());
    let t = w * h + 2 * (w.div_ceil(2) * h.div_ceil(2));
    let (na, nb) = (a.len() / t, b.len() / t);
    let mut iguales = 0;
    let mut primera = None;
    for i in 0..na.min(nb) {
        if a[i * t..(i + 1) * t] == b[i * t..(i + 1) * t] {
            iguales += 1;
        } else if primera.is_none() {
            primera = Some(i);
        }
    }
    let resto = !a.len().is_multiple_of(t) || !b.len().is_multiple_of(t);
    println!(
        "COMPARACION fotogramas_a={na} fotogramas_b={nb} iguales={iguales} primera_diferencia={} resto_no_multiplo={resto}",
        primera.map_or("ninguna".into(), |i| i.to_string())
    );
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("decodificar") => decodificar(&args),
        Some("mutar") => mutar(&args),
        Some("comparar") => comparar(&args),
        Some("diag") => {
            let flujo = std::fs::read(&args[2]).expect("leer");
            let mut dec = crusty_h264::Decoder::new();
            for (i, au) in crusty_h264::split_access_units(&flujo).iter().enumerate() {
                let tipos: Vec<u8> = openh264::nal_units(au)
                    .map(|n| {
                        let k = n.iter().position(|&b| b == 1).map_or(0, |p| p + 1);
                        n.get(k).map_or(255, |b| b & 31)
                    })
                    .collect();
                match dec.decode(au) {
                    Ok(f) => println!("AU {i} nal={tipos:?} ok foto={}", f.is_some()),
                    Err(e) => {
                        println!("AU {i} nal={tipos:?} ERROR {e:?} bytes={}", au.len());
                        break;
                    }
                }
            }
        }
        _ => eprintln!("uso: ver la cabecera del fuente"),
    }
}
