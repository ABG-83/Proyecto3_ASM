// =============================================================================
// Referencia.h — Trayectoria de velocidad decreciente
// Responsable: Integrante C
//
// El freno solo desacelera: la referencia debe ser físicamente alcanzable.
// Mínimo: rampa w_ref(t) = w0 - a*t, saturada en 0.
// La desaceleración a se elige con el modelo identificado (Integrante B):
// debe ser mayor que la del frenado libre y menor que la del freno a fondo.
// =============================================================================
#pragma once
#include "Config.h"

// Fija la velocidad inicial w0 [rad/s] y la desaceleración a [rad/s^2].
inline void referenciaIniciar(float w0, float a) {
  (void)w0; (void)a;
  // TODO (C)
}

// Referencia [rad/s] a los t segundos de iniciado el frenado.
inline float referencia(float t) {
  (void)t;
  // TODO (C)
  return 0.0f;
}
