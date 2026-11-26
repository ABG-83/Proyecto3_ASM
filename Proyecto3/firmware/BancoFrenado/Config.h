// =============================================================================
// Config.h — Constantes compartidas del banco de frenado
// Responsable: TODOS (cambiar solo de acuerdo con el grupo)
//
// Aquí van los pines, el periodo de muestreo y los límites físicos.
// Ningún otro archivo debe tener números "mágicos": todo sale de aquí.
// =============================================================================
#pragma once
#include <Arduino.h>

// ---------------- Pines (mismas conexiones del banco, ver Tarea2) -------------
constexpr uint8_t PIN_SENSOR = 8;   // KY-033 (S)
constexpr uint8_t PIN_ENA    = 4;   // L298N, PWM del motor
constexpr uint8_t PIN_IN1    = 5;   // L298N, sentido
constexpr uint8_t PIN_IN2    = 6;   // L298N, sentido
constexpr uint8_t PIN_SERVO  = 7;   // MG90S del freno

// ---------------- Muestreo ----------------------------------------------------
// Periodo del lazo de control. Variable (no constexpr) para el extra de
// comparar dos periodos de muestreo (Integrante C).
extern uint32_t TS_US;                         // valor inicial en BancoFrenado.ino
inline float Ts() { return TS_US * 1.0e-6f; }  // en segundos

// ---------------- Sensor ------------------------------------------------------
constexpr uint32_t MUESTREO_SENSOR_US = 1000;  // lectura del KY-033 cada 1 ms
constexpr uint32_t ESTABLE_US         = 5000;  // filtro: fase estable >= 5 ms
extern uint32_t MARCAS_POR_VUELTA;             // PPR; debe coincidir con el disco

// ---------------- Límites de seguridad ---------------------------------------
constexpr uint8_t  PWM_MOTOR_MAX     = 160;    // fuente de 9 V
constexpr uint32_t T_MAX_ENSAYO_MS   = 15000;  // ningún ensayo dura más que esto

// ---------------- Acción de control ------------------------------------------
constexpr float U_MIN = 0.0f;   // zapata libre
constexpr float U_MAX = 1.0f;   // presión máxima calibrada
