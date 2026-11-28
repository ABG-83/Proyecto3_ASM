// =============================================================================
// Antibloqueo.h — EXTRA: antibloqueo simplificado (hasta 5 puntos)
// Responsable: Integrante A
//
// Solo se activa cuando lo básico ya funciona.
// Idea: si la desaceleración medida supera un umbral justificado (caída
// anormal de velocidad), liberar o modular el freno por un tiempo corto.
// Debe llamarse "antibloqueo simplificado", NO medición de deslizamiento.
// =============================================================================
#pragma once
#include "Config.h"

inline bool antibloqueoActivo = false;   // se enciende/apaga por comando

// Recibe la acción del PID y la medición; devuelve la acción final.
inline float antibloqueoAplicar(float u, float w) {
  (void)w;
  // TODO (A)
  return u;
}
