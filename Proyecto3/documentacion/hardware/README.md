# Hardware del banco de frenado

Responsable: Integrante A (enunciado, sección 2.2).

Entregables que van en esta carpeta:

- [ ] `esquema_electrico.pdf` (y su fuente, por ejemplo `.fzz` de Fritzing o el proyecto de KiCad)
- [ ] Lista de materiales (tabla abajo)
- [ ] Fotos del montaje en `../paper/figuras/`

## Conexiones

| Componente | Conexión a la ESP32-S3 |
| --- | --- |
| Sensor óptico KY-033 | V+ → 3V3, G → GND, S → GPIO8 |
| Puente H L298N | ENA → GPIO4, IN1 → GPIO5, IN2 → GPIO6 |
| Servomotor MG90S | Señal → GPIO7 (5 V aparte, GND común) |
| Motor de la rueda | Salida A del L298N, batería de 9 V |

## Lista de materiales

| Componente | Cantidad | Función |
| --- | --- | --- |
| TODO (A) | | |
