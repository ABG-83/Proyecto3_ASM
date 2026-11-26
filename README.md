# Proyecto3_ASM – Banco de frenado con PID digital

Proyecto del curso **CE-1110 Análisis de Señales Mixtas** (Instituto Tecnológico de Costa Rica, II semestre 2026).

El banco hace girar una rueda con un motor DC y la frena con una zapata movida por un servomotor. Una ESP32-S3 mide la velocidad con un sensor óptico y controla el freno con un PID digital, para que la rueda siga una referencia de velocidad.

**Integrantes:** Aldo Alessandro Bagliano Gómez, José Pablo Fernández Rojas, José Andrés Solano Mora y Joan Jesús Ugalde Zárate.

## Estructura del repositorio

```
Proyecto3/                   Proyecto Grupal 2 (ver Proyecto3/README.md)
Tarea2/                      Tarea 2: modelado y código base
├── Banco_Completo.ino       Firmware de pruebas del banco (ESP32-S3)
├── Banco.h                  Periféricos, máquina de estados, comandos serie
├── ControlCore.h            PID discreto y pruebas internas (SELFTEST)
├── pid_frenado/
│   └── pid_frenado.ino      Código base mostrado en el documento de la Tarea 2 (Pregunta 3)
├── capturar.py              Captura por puerto serie a CSV
├── analizar_escalon.m       Identificación del modelo a partir del escalón (MATLAB)
├── MATLAB_Ensayo_Real.m     Datos y gráficas del primer ensayo real
└── VERIFICACION.json        Estado de las pruebas del firmware
```

## Hardware y conexiones

| Componente | Conexión a la ESP32-S3 |
| --- | --- |
| Sensor óptico KY-033 | V+ → 3V3, G → GND, S → GPIO8 |
| Puente H L298N (motor de la rueda) | ENA → GPIO4, IN1 → GPIO5, IN2 → GPIO6 |
| Servomotor MG90S (freno) | Señal → GPIO7 (alimentación de 5 V aparte, GND común) |
| Alimentación del motor | Batería de 9 V al L298N |

## Compilar y cargar el firmware

1. Arduino IDE con el paquete **esp32 de Espressif, versión 3.x**.
2. Placa: **ESP32S3 Dev Module**.
3. En *Herramientas*: **USB Mode = Hardware CDC and JTAG** y **USB CDC On Boot = Enabled**. Si no, el código no compila a propósito.
4. Abrir `Tarea2/Banco_Completo.ino`. El IDE pide crear una carpeta `Banco_Completo` y mover el archivo: aceptar, copiar también `Banco.h` y `ControlCore.h` a esa carpeta, y cargar.

Al encender, el motor queda en PWM 0 y el servo sin pulsos: el banco nunca se mueve solo.

## Uso por el Monitor Serie (115200 baudios, cada orden con Enter)

| Orden | Acción |
| --- | --- |
| `H` / `?` | Ayuda / estado |
| `C`, `V1500`, `+`, `-`, `F+`, `F-` | Mover el servo a mano para calibrar |
| `L` / `P` | Guardar posición libre / de contacto de la zapata (**calibrar sin motor**) |
| `K` | Solo motor |
| `OA` | Escalón de frenado a lazo abierto (para caracterizar) |
| `PID` | Frenado con control PID |
| `COAST` | Frenado libre con el motor apagado |
| `U0.25` | Tamaño del escalón |
| `REF80` | Referencia de velocidad en rpm |
| `KP…`, `KI…`, `KD…` | Ganancias del PID |
| `PPR1` | Marcas por vuelta del disco (debe coincidir con las marcas reales) |
| `S` / `R` | Parar y liberar el freno |
| `X` / `!` | Parada de emergencia (PWM 0) |
| `SELFTEST` | Pruebas internas del PID sin mover actuadores |

## Captura y análisis del escalón

```bash
pip install pyserial
python Tarea2/capturar.py --port COM4 --salida ensayo_OA.csv
```

Dentro de la captura se escriben las mismas órdenes del Monitor Serie (por ejemplo `OA`), y `SALIR` termina. Luego, en MATLAB (requiere Control System Toolbox):

```matlab
resultado = analizar_escalon('ensayo_OA.csv', 1)   % 1 = marcas por vuelta
```

## Notas de diseño

- **Periodo de control:** Ts = 20 ms (50 Hz), igual a la frecuencia del PWM del servo. La tarea de control se planifica con `micros()`; el periodo **no** depende de ninguna espera.
- **`delay(1)` al final de `loop()`:** solo cede tiempo al sistema operativo de la ESP32 entre revisiones del reloj. No fija el periodo de muestreo, que lo marca el planificador con `micros()`.
- **Sensor:** se muestrea cada 1 ms con `esp_timer` y cada fase debe mantenerse 5 ms para aceptarse. Con una sola marca por vuelta la velocidad se actualiza una vez por vuelta; se recomiendan 8 marcas (`PPR8`).
- **Freno de acción inversa:** el error es `e = w − ref`, y la acción se limita a [0, 1] con anti-windup por integración condicional.

## Estado

Según `VERIFICACION.json`: el firmware compila, se cargó y pasa las pruebas internas. Aún **no** se han calibrado físicamente el freno ni las ganancias del PID, ni se ha medido el escalón real.

## Licencia

Ver [LICENSE](LICENSE).
