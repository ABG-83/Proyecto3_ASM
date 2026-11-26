// =============================================================================
// Ensayos.h — Máquina de estados de los ensayos y comandos por serie
// Responsable: Integrante D
//
// Punto de partida: start(), controlTick() y readPort() de Tarea2/Banco.h.
// Requisitos (enunciado, secciones 2.2, 2.5 y 2.6):
//   - Arranque a velocidad repetible: motor hasta régimen, luego se corta.
//   - Modo OA: acción de frenado fija (lazo abierto).
//   - Modo PID: sigue la referencia decreciente (Referencia.h + Control.h).
//   - Condiciones iniciales equivalentes entre repeticiones.
//   - Parada de emergencia siempre disponible.
//
// Comandos mínimos sugeridos (cada uno con Enter):
//   OA, PID, S (parar), X/! (emergencia), U<valor>, W0<rpm>, A<rad/s2>,
//   KP<v>, KI<v>, KD<v>, TS<ms>, PPR<n>, ?
// =============================================================================
#pragma once
#include "Config.h"
#include "Sensor.h"
#include "Actuadores.h"
#include "Referencia.h"
#include "Control.h"
#include "Registro.h"

enum class Estado { REPOSO, ARRANQUE, FRENADO, FIN };

// Se llama una vez cada Ts desde loop(). No bloquea.
inline void ensayoTick() {
  const Medicion m = sensorLeer();
  (void)m;
  // TODO (D): REPOSO -> ARRANQUE -> FRENADO -> FIN
  //   FRENADO en OA:  freno(U fija)
  //   FRENADO en PID: freno(pidCalcular(referencia(t), m.w))
  //   Cada muestra:   registroFila(...)
}

// Lee comandos del puerto serie sin bloquear (máximo una línea por llamada).
inline void comandosAtender() {
  // TODO (D)
}
