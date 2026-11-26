// =============================================================================
// Control.h — PID digital
// Responsable: Integrante C
//
// Punto de partida: Tarea2/ControlCore.h y la Pregunta 3 de la Tarea 2.
// Requisitos (enunciado, sección 2.4):
//   - Acción inversa: e = w - w_ref (si va más rápido que la referencia, frenar más).
//   - Discretización documentada (por ejemplo, integral por Euler hacia atrás).
//   - Derivada sobre la medición y filtrada (ruido de medición).
//   - Saturación en [U_MIN, U_MAX] y anti-windup.
//   - Usar Ts() de Config.h: las ganancias dependen del periodo.
// =============================================================================
#pragma once
#include "Config.h"

struct GananciasPID {
  float kp;
  float ki;    // [1/s]
  float kd;    // [s]
  float tf;    // constante de tiempo del filtro derivativo [s]
};

// Valores iniciales: reemplazar por los del diseño con el modelo identificado.
inline GananciasPID pidGanancias{0.0f, 0.0f, 0.0f, 0.10f};

// Borra el estado interno (integral, derivada) al iniciar un ensayo.
inline void pidReiniciar(float w) {
  (void)w;
  // TODO (C)
}

// Calcula u[k] en [U_MIN, U_MAX]. Se llama exactamente una vez cada Ts.
inline float pidCalcular(float wRef, float w) {
  (void)wRef; (void)w;
  // TODO (C)
  return 0.0f;
}
