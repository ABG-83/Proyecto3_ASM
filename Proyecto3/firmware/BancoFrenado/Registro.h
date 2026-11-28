// =============================================================================
// Registro.h — Salida CSV de cada muestra
// Responsable: Integrante B
//
// Este formato lo consumen capturar.py y leer_ensayo.py.
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
#include <cstdio>

// Entrega la línea al búfer serie solo si cabe completa. Llamar desde loop().
inline bool registroEnviar(const char *linea, int longitud) {
  return longitud > 0 && Serial.availableForWrite() >= longitud &&
         Serial.write(reinterpret_cast<const uint8_t *>(linea), longitud) ==
             static_cast<size_t>(longitud);
}

// El ensayo debe enviar el encabezado antes de producir la primera muestra.
inline bool registroEncabezado() {
#if ARDUINO_USB_CDC_ON_BOOT
  // El puerto USB no debe esperar al equipo que está capturando.
  Serial.setTxTimeoutMs(0);
#endif
  constexpr char encabezado[] = "t_ms,modo,ref,w,e,u,pwm,nueva\n";
  return registroEnviar(encabezado, sizeof(encabezado) - 1);
}

// Una sola escritura conserva juntas todas las señales de la muestra.
inline bool registroFila(uint32_t tMs, const char *modo, float ref, float w,
                         float u, uint8_t pwm, bool nueva) {
  char linea[96];
  const int longitud = std::snprintf(
      linea, sizeof(linea), "%lu,%s,%.6f,%.6f,%.6f,%.6f,%u,%u\n",
      static_cast<unsigned long>(tMs), modo, ref, w, w - ref, u,
      static_cast<unsigned>(pwm), static_cast<unsigned>(nueva));
  return longitud < static_cast<int>(sizeof(linea)) && registroEnviar(linea, longitud);
}

// Los eventos comparten el puerto sin confundirse con las muestras.
inline bool registroEvento(const char *texto) {
  char linea[96];
  const int longitud = std::snprintf(linea, sizeof(linea), "# %s\n", texto);
  return longitud < static_cast<int>(sizeof(linea)) && registroEnviar(linea, longitud);
}
