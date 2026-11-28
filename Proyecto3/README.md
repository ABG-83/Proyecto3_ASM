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
│   ├── python/capturar.py      Captura del puerto serie a CSV         → B
│   └── matlab/
│       ├── leer_ensayo.m       Lector común de CSV                    → B
│       ├── identificar_modelo.m  Ajuste del modelo                    → B
│       ├── validar_modelo.m    Validación con datos nuevos            → B
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
