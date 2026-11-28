// =============================================================================
// Registro.h — Salida CSV de cada muestra
// Responsable: Integrante B
//
// Este formato lo consumen capturar.py y todos los scripts de MATLAB.
// Si se cambia una columna, avisar al grupo y actualizar los scripts.
//
// Columnas (una fila por periodo de control):
//   t_ms     tiempo desde el inicio del ensayo [ms]
//   modo     OA | PID
//   ref      referencia [rad/s] (en OA se registra 0)
//   w        velocidad medida [rad/s]
//   e        error w - ref [rad/s]
//   u        acción de frenado [0..1]
//   pwm      PWM del motor
//   nueva    1 si la medición de w es nueva en esta muestra
// Las líneas que empiezan con '#' son comentarios (eventos, estado, ganancias).
// =============================================================================
#pragma once
#include "Config.h"

// Imprime el encabezado al iniciar un ensayo.
inline void registroEncabezado() {
  // TODO (B)
}

// Imprime una fila. No debe bloquear el lazo de control.
inline void registroFila(uint32_t tMs, const char *modo, float ref, float w,
                         float u, uint8_t pwm, bool nueva) {
  (void)tMs; (void)modo; (void)ref; (void)w; (void)u; (void)pwm; (void)nueva;
  // TODO (B)
}

// Imprime un comentario (línea con '#').
inline void registroEvento(const char *texto) {
  Serial.print("# ");
  Serial.println(texto);
}
