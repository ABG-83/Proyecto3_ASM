// =============================================================================
// Referencia.h — Trayectoria de velocidad decreciente
// Responsable: Integrante C
//
// El freno solo desacelera: la referencia debe ser físicamente alcanzable.
// Rampa: w_ref(t) = w0 - a*t, saturada en 0.
// La desaceleración a se elige con el modelo identificado (Integrante B):
// debe ser mayor que la del frenado libre y menor que la del freno a fondo.
// Con el modelo de la Tarea 2: libre ~2,9 rad/s^2, a fondo ~8,7 rad/s^2 -> a = 4.
// =============================================================================
#pragma once
#include "Config.h"

inline float refW0 = 0.0f;   // velocidad al iniciar el frenado [rad/s]
inline float refA  = 4.0f;   // desaceleración de la rampa [rad/s^2]

// Fija la velocidad inicial w0 [rad/s] y la desaceleración a [rad/s^2].
// Se llama una vez, justo cuando se corta el motor y empieza el frenado.
inline void referenciaIniciar(float w0, float a) {
  refW0 = (w0 > 0.0f) ? w0 : 0.0f;
  refA  = (a  > 0.0f) ? a  : 0.0f;
}

// Referencia [rad/s] a los t segundos de iniciado el frenado.
inline float referencia(float t) {
  const float r = refW0 - refA * t;
  return (r > 0.0f) ? r : 0.0f;   // nunca negativa: la rueda no gira al revés
}