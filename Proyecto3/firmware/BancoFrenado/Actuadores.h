// =============================================================================
// Actuadores.h — Motor (L298N) y freno (MG90S)
// Responsable: Integrante A
//
// Punto de partida: configurePwm(), setMotor(), setServo() y la calibración
// de las posiciones libre/contacto de Tarea2/Banco.h.
// Requisitos:
//   - PWM por LEDC: motor a 5 kHz/8 bits, servo a 50 Hz.
//   - u = 0 -> zapata libre; u = 1 -> presión máxima (calibrada en el banco).
//   - Saturar siempre u en [U_MIN, U_MAX] y el PWM del motor en PWM_MOTOR_MAX.
//   - Al encender: motor en 0 y freno libre.
// =============================================================================
#pragma once
#include "Config.h"

// Configura LEDC y deja todo apagado. Devuelve false si falla.
inline bool actuadoresIniciar() {
  // TODO (A)
  return true;
}

// PWM del motor (0..PWM_MOTOR_MAX). 0 apaga la tracción.
inline void motor(uint8_t pwm) {
  (void)pwm;
  // TODO (A)
}

// Acción de frenado normalizada u en [0, 1].
inline void freno(float u) {
  (void)u;
  // TODO (A)
}

// Parada de emergencia: motor en 0 y freno libre.
inline void pararTodo() {
  motor(0);
  freno(0.0f);
}
