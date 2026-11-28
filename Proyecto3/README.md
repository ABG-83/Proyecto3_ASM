# Proyecto Grupal 2 – Control de frenado con PID digital

Banco de frenado: un motor DC hace girar la rueda, un servomotor MG90S la frena con una zapata y una ESP32-S3 mide la velocidad con un sensor KY-033 y ejecuta un PID discreto que sigue una referencia de velocidad decreciente.

Entrega y defensa: **2 de diciembre de 2026, 11:50 pm**.

## Estructura y responsables

```
Proyecto3/
├── firmware/BancoFrenado/      Firmware de la ESP32-S3 (Arduino-ESP32 3.x)
│   ├── BancoFrenado.ino        Lazo principal sin delay()             → D integra
│   ├── Config.h                Pines, Ts y límites                    → todos (avisar)
│   ├── Sensor.h                Medición de velocidad                  → A
│   ├── Actuadores.h            Motor y servo                          → A
│   ├── Antibloqueo.h           Extra: antibloqueo simplificado        → A
│   ├── Registro.h              Formato CSV de salida                  → B
│   ├── Referencia.h            Rampa de velocidad decreciente         → C
│   ├── Control.h               PID digital con anti-windup            → C
│   └── Ensayos.h               Máquina de estados y comandos serie    → D
├── scripts/
│   ├── python/
│   │   ├── capturar.py         Captura del puerto serie a CSV         → B
│   │   ├── leer_ensayo.py      Lector común de CSV                    → B
│   │   ├── identificar_modelo.py Ajuste del modelo de la planta        → B
│   │   └── validar_modelo.py  Validación con capturas independientes → B
│   └── matlab/
│       ├── disenar_controlador.m Diseño en Laplace y Z                → C
│       ├── metricas.m          IAE, RMSE, tiempo de frenado...        → D
│       └── comparar_ensayos.m  Lazo abierto contra PID                → D
├── datos/
│   ├── identificacion/         Ensayos a lazo abierto para el ajuste  → B
│   ├── validacion/             Ensayos NO usados en el ajuste         → B
│   ├── ajuste/                 Pruebas intermedias del PID            → D
│   ├── finales/                3 repeticiones × 2 velocidades × OA/PID → D
│   └── extras/                 Datos de los puntos extra              → quien lo haga
└── documentacion/
    ├── hardware/               Esquema eléctrico y materiales         → A
    └── paper/                  Paper IEEE (máximo 4 páginas)
        ├── main.tex            No editar sin avisar
        ├── referencias.bib     Cada quien agrega sus referencias
        └── secciones/          Un archivo por sección (ver main.tex)
```

## Reglas del grupo

- **Una rama por persona desde `develop`**, por ejemplo `feature/planta`, `feature/identificacion`, `feature/control` y `feature/ensayos`. Cuando algo funcione: pull request o merge a `develop`.
- **Commits pequeños y frecuentes, desde la cuenta de cada quien.** El profe revisa el avance incremental y el trabajo en equipo.
- **Cada quien edita sus archivos.** Si hay que tocar un archivo ajeno o `Config.h`, se avisa primero.
- **Nombre de los datos:** `modo_parametro_repeticion.csv`, por ejemplo `oa_u050_w120_r1.csv` o `pid_a4_w120_r2.csv`.
- **Al final:** merge de `develop` a `main` y crear el tag `proyecto2-v1.0`.

## Compilar el firmware

Arduino IDE con el paquete esp32 de Espressif 3.x, placa **ESP32S3 Dev Module**, y abrir `firmware/BancoFrenado/BancoFrenado.ino`. El código de la Tarea 2 (`../Tarea2/Banco.h`) sirve de referencia para portar las partes que ya funcionan.

## Formato del registro de ensayos

El puerto serie usa 115200 baudios. Cada CSV contiene un solo ensayo y
comienza con `t_ms,modo,ref,w,e,u,pwm,nueva`, precedido opcionalmente por
comentarios. Cada fila registra el tiempo real desde el inicio del ensayo en
milisegundos, el modo `OA` o `PID`, la referencia, la velocidad y el error
`w - ref` en rad/s, la acción de frenado entre 0 y 1 y el PWM del motor.
En `OA` se registra una referencia de 0. `nueva` vale 1 cuando el sensor
ha actualizado la velocidad y 0 cuando el valor sigue retenido.

Las líneas con `#` conservan el estado y los eventos en el mismo archivo.
Para un evento con tiempo de ensayo, usar `# EVENT t_ms=<ms> <nombre>`,
por ejemplo al cortar el motor o cambiar la acción de frenado. El registro
devuelve `false` si no pudo enviar una línea. El código del ensayo puede
reintentar el encabezado antes de empezar el muestreo, sin esperar dentro
del lazo de control.

Desde `Proyecto3`, con `pyserial` instalado, iniciar la captura con
`python scripts/python/capturar.py --port COM4 --salida datos/identificacion/oa_u050_w120_r1.csv`.
La captura conserva también un TXT con el intercambio serie. En Python,
`from leer_ensayo import leer_ensayo` permite cargar el registro desde los
scripts de `scripts/python`. La llamada `muestras, eventos = leer_ensayo(archivo)`
devuelve las filas con sus tipos numéricos, el campo `t` en segundos y los
comentarios del registro. El procedimiento de ajuste en Python está en
[`datos/README.md`](datos/README.md).
