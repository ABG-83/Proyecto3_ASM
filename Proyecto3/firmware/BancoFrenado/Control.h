// =============================================================================
// Control.h - PID digital
//
// Ley de control (forma paralela, discretizada con Euler hacia atrás):
//
//   e[k] = w[k] - w_ref[k]                       (acción inversa)
//   P[k] = Kp * e[k]
//   I[k] = I[k-1] + Ki * Ts * e[k]               (solo si no hay saturación)
//   D[k] = Kd * df[k]
//   df[k] = alfa * df[k-1] + (1 - alfa) * (w[k] - w[k-1]) / Ts
//   alfa = Tf / (Tf + Ts)                        (filtro pasa-bajas de 1er orden)
//   u[k] = sat(P[k] + I[k-1] + D[k], 0, 1)
//
// Decisiones de diseño:
//   - e = w - w_ref: si la rueda va más rápido que la referencia, frenar más.
//   - Derivada sobre la medición (no sobre el error): evita picos cuando la
//     referencia cambia. Como w_ref es una rampa, d(e)/dt ≈ dw/dt - (-a); el
//     término constante no aporta nada a la dinámica.
//   - Anti-windup por integración condicional: no se integra mientras la
//     salida está saturada en la dirección del error.
//   - Las ganancias dependen de Ts: se usa Ts() de Config.h.
// =============================================================================
#pragma once
#include "Config.h"

struct GananciasPID {
  float kp;
  float ki;    // [1/s]
  float kd;    // [s]
  float tf;    // constante de tiempo del filtro derivativo [s]
};

// Valores iniciales: SIMC (PI) con el modelo de la Tarea 2
// (scripts/matlab/disenar_controlador.m). Se actualizan con el modelo de B.
inline GananciasPID pidGanancias{0.41f, 0.25f, 0.0f, 0.10f};

struct EstadoPID {
  float integral;    // término integral acumulado (ya multiplicado por Ki)
  float dFiltrada;   // derivada filtrada de la velocidad [rad/s^2]
  float wAnterior;   // medición anterior [rad/s]
  bool  saturado;    // true si la última salida quedó en un límite
};

inline EstadoPID pidEstado{0.0f, 0.0f, 0.0f, false};

// Borra el estado interno al iniciar un ensayo. w = velocidad actual,
// para que la primera derivada no dé un salto.
inline void pidReiniciar(float w) {
  pidEstado = {0.0f, 0.0f, w, false};
}

// Calcula u[k] en [U_MIN, U_MAX]. Se llama exactamente una vez cada Ts.
inline float pidCalcular(float wRef, float w) {
  const float ts = Ts();
  const GananciasPID &g = pidGanancias;
  EstadoPID &s = pidEstado;

  const float e = w - wRef;

  // Derivada de la medición, filtrada.
  const float alfa = g.tf / (g.tf + ts);
  const float dw = (w - s.wAnterior) / ts;
  s.dFiltrada = alfa * s.dFiltrada + (1.0f - alfa) * dw;
  s.wAnterior = w;

  const float P = g.kp * e;
  const float D = g.kd * s.dFiltrada;
  const float uSinSat = P + s.integral + D;
  const float u = constrain(uSinSat, U_MIN, U_MAX);

  // Anti-windup: integración condicional.
  const bool satArriba = (uSinSat > U_MAX) && (e > 0.0f);
  const bool satAbajo  = (uSinSat < U_MIN) && (e < 0.0f);
  s.saturado = satArriba || satAbajo;
  if (!s.saturado) {
    s.integral += g.ki * ts * e;
    s.integral = constrain(s.integral, U_MIN, U_MAX);
  }
  return u;
}