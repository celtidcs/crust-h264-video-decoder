# Manual de Uso de Crust H264 Video Decoder

Tienes toda la razon en tu planteamiento: Crust H264 Video Decoder es, fundamentalmente, una libreria de software (un conjunto de paquetes o crates de Rust), pensada para que otros programas la utilicen cuando necesitan decodificar video de forma rapida y completamente segura. 

No es una aplicacion con ventanas ni un reproductor con interfaz grafica como VLC. Sin embargo, para poder probarla, medir su velocidad y verificar que decodifica exactamente igual que otros reproductores, el proyecto incluye un programa de consola (CLI) llamado banco_h264.

En esta guia te explicamos con claridad las dos formas de utilizarlo:
1. Como herramienta de linea de comandos para probar y convertir videos.
2. Como libreria para integrarla dentro de tus propios programas en Rust.

---

## 1. Uso como herramienta de consola (banco_h264)

Si compilas el proyecto, obtienes un ejecutable llamado banco_h264 (en Windows sera banco_h264.exe). Este programa te permite abrir cualquier archivo de video H.264 en bruto (Annex B) y hacer varias cosas utiles.

### Paso 1: Compilar la herramienta
Abre una terminal en la carpeta del proyecto y ejecuta:
```bash
cargo build --release
```
El ejecutable optimizado quedara listo en la carpeta de compilacion (por ejemplo en target/release/banco_h264.exe).

### Paso 2: Probar un video y comprobar si es valido
Para decodificar un video y comprobar en pantalla cuantos fotogramas tiene, su resolucion exacta y el tiempo que tarda en procesarlo, escribe:
```bash
banco_h264 decodificar rusty mi_video.264
```
El programa te respondera en una linea con datos claros:
- Si el video es correcto (ok) o si contiene fallos (error).
- Cuantos fotogramas ha leido.
- La resolucion (por ejemplo 1920x1080).
- Los milisegundos exactos que ha tardado.
- La memoria maxima que ha consumido.
- Una huella digital (SHA-256) de toda la imagen decodificada.

### Paso 3: Guardar el video decodificado a un archivo
Si quieres extraer los fotogramas descomprimidos para verlos o compararlos, solo tienes que anadir el nombre del archivo de salida al final del comando:
```bash
banco_h264 decodificar rusty mi_video.264 fotogramas_salida.yuv
```
El archivo resultante contendra los fotogramas en formato crudo YUV 4:2:0. Puedes abrir este archivo con programas como YUV Viewer, Vooya o FFmpeg para ver el video fotograma a fotograma con tus propios ojos.

### Paso 4: Comparar dos resultados fotograma a fotograma
Si quieres comprobar si Crust H264 Video Decoder decodifica exactamente lo mismo que otro decodificador (por ejemplo OpenH264 o FFmpeg):
1. Decodificas con Crust:
   banco_h264 decodificar rusty video.264 salida_crust.yuv
2. Decodificas con OpenH264:
   banco_h264 decodificar openh264 video.264 salida_cisco.yuv
3. Comparas ambos archivos indicando su ancho y alto:
   banco_h264 comparar salida_crust.yuv salida_cisco.yuv 1920 1080

El programa comparara pixel a pixel ambos archivos y te dira si son totalmente identicos o en que fotograma aparece la primera discrepancia.

---

## 2. Uso como libreria dentro de tu propio programa en Rust

Si estas programando una aplicacion (por ejemplo, un navegador web, un reproductor multimedia, una herramienta de camaras de seguridad o un servidor de streaming), puedes anadir Crust H264 Video Decoder a tu archivo Cargo.toml para decodificar video directamente en tu codigo.

### Paso 1: Configurar dependencias
En el archivo Cargo.toml de tu aplicacion, anades la ruta donde tienes Crust H264 Video Decoder:

```toml
[dependencies]
crust-h264-video-decoder = { path = "ruta/a/crust-h264-video-decoder/crates/crust-h264-video-decoder" }
```

### Paso 2: Caso sencillo - Decodificar un archivo completo
Si tienes en memoria todos los bytes del video y solo quieres obtener la lista de imagenes para mostrarlas o procesarlas:

```rust
use crust_h264_video_decoder::Decoder;
use std::fs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Cargas los bytes del archivo de video
    let bytes_video = fs::read("video.264")?;

    // 2. Creas una instancia del decodificador
    let mut decodificador = Decoder::new();

    // 3. Decodificas todo el video de una vez
    // Devuelve los fotogramas en el orden en que deben reproducirse
    let fotogramas = decodificador.decode_stream(&bytes_video)
        .map_err(|error| format!("No se pudo decodificar el video: {:?}", error))?;

    println!("Se han decodificado {} fotogramas con exito.", fotogramas.len());

    // 4. Recorres las imagenes obtenidas
    for (numero, imagen) in fotogramas.iter().enumerate() {
        println!(
            "Fotograma {}: tamano {}x{} pixeles.",
            numero, imagen.width, imagen.height
        );
        // imagen.y contiene los datos de luminosidad (luma)
        // imagen.u e imagen.v contienen los datos de color (croma)
    }

    Ok(())
}
```

### Paso 3: Caso avanzado - Streaming y reproductores en tiempo real
En aplicaciones reales como reproductores de video o transmisiones en directo, no tienes el video entero de golpe: recibes paquetes de red o muestras desde un archivo MP4 poco a poco.

En este caso, se decodifica paquete a paquete y se utiliza el modulo de reordenacion acotada para que las imagenes salgan en el orden exacto de reproduccion sin gastar memoria de mas:

```rust
use crust_h264_video_decoder::{split_access_units, au_is_idr, Decoder};
use crust_h264_video_decoder::reordenacion::OrdenPresentacionH264;

fn procesar_flujo_en_directo(datos_video: &[u8], nivel_h264: u8) {
    let mut decodificador = Decoder::new();
    let mut organizador = OrdenPresentacionH264::new(nivel_h264);

    // Separamos el flujo en paquetes individuales (unidades de acceso)
    for paquete in split_access_units(datos_video) {
        // Si el paquete es un fotograma clave (IDR), liberamos lo que habia pendiente
        let fotogramas_listos = organizador.empezar_unidad(au_is_idr(paquete));
        mostrar_en_pantalla(fotogramas_listos);

        // Decodificamos el paquete actual
        if let Ok(Some(imagen)) = decodificador.decode(paquete) {
            let dimensiones = (imagen.width, imagen.height);
            let orden_temporal = decodificador.last_poc();
            let reinicio_memoria = decodificador.last_has_mmco5();

            // Guardamos en el bufer acotado y recogemos los que ya toca mostrar
            let listos = organizador.entrar(
                imagen, 
                dimensiones, 
                orden_temporal, 
                reinicio_memoria
            );
            mostrar_en_pantalla(listos);
        }
    }

    // Al terminar la transmision, vaciamos los ultimos fotogramas que queden
    mostrar_en_pantalla(organizador.vaciar());
}

fn mostrar_en_pantalla<T>(_imagenes: Vec<T>) {
    // Aqui tu aplicacion dibujaria la imagen en la ventana o la enviaria a la pantalla
}
```

---

## 3. Como comprobar que el decodificador no comete ni un solo fallo

El proyecto incluye un script en Python en la carpeta de utilidades (`scripts/red.py`) que somete al decodificador a una prueba masiva con 2.129 videos oficiales y mutaciones complejas creadas para intentar romperlo.

Para ejecutar esta comprobacion:
```bash
python scripts/red.py comprobar
```
El script procesara los miles de videos en paralelo y comparara cada resultado con la linea base oficial. Si todo el codigo funciona a la perfeccion, veras el mensaje:
`RED EN VERDE: 2129/2129 idénticas`.
Cualquier fallo, por minimo que sea, se detectara en el acto con indicacion exacta del archivo que ha discrepado.
