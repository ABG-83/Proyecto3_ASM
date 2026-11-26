// =============================================================================
// Sensor.h — Medición de velocidad angular con el KY-033
// Responsable: Integrante A
//
// Punto de partida: sampleSensor() y la medición de Tarea2/Banco.h.
// Requisitos:
//   - Muestrear el sensor cada MUESTREO_SENSOR_US con esp_timer (sin delay).
//   - Aceptar una marca solo si cada fase dura >= ESTABLE_US.
//   - Velocidad = 2*pi / (MARCAS_POR_VUELTA * tiempo_entre_marcas).
//   - Si pasa demasiado tiempo sin marcas, la rueda se considera detenida.
//   - Proteger con portENTER_CRITICAL lo que se comparte con el timer.
// =============================================================================
#pragma once
#include "Config.h"

struct Medicion {
  float w;          // velocidad angular [rad/s]
  bool  valida;     // false si aún no hay dos marcas o la rueda está detenida
  bool  nueva;      // true si llegó una marca nueva desde la lectura anterior
};

// Configura el pin y arranca el timer de muestreo. Devuelve false si falla.
inline bool sensorIniciar() {
  pinMode(PIN_SENSOR, INPUT_PULLUP);
  // TODO (A): crear y arrancar el esp_timer de MUESTREO_SENSOR_US.
  return true;
}

// Reinicia el conteo (por ejemplo, al cambiar MARCAS_POR_VUELTA).
inline void sensorReiniciar() {
  // TODO (A)
}

// Última velocidad medida. No bloquea.
inline Medicion sensorLeer() {
  Medicion m{0.0f, false, false};
  // TODO (A)
  return m;
}
