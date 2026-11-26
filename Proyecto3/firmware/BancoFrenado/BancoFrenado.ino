// =============================================================================
// BancoFrenado.ino — Programa principal (Proyecto Grupal 2, CE-1110)
// Responsable: Integrante D integra; nadie más edita este archivo sin avisar.
//
// Placa: ESP32S3 Dev Module, núcleo Arduino-ESP32 3.x.
// Cada parte vive en su propio .h (ver README de Proyecto3).
// El lazo de control se planifica con micros(): NO usar delay() en el lazo.
// =============================================================================
#include "Config.h"
#include "Sensor.h"
#include "Actuadores.h"
#include "Referencia.h"
#include "Control.h"
#include "Registro.h"
#include "Antibloqueo.h"
#include "Ensayos.h"

uint32_t TS_US = 20000;             // Ts inicial: 20 ms (50 Hz), ver Tarea 2
uint32_t MARCAS_POR_VUELTA = 1;     // cambiar a 8 cuando el disco tenga 8 marcas

static uint32_t tProximo = 0;

void setup() {
  Serial.begin(115200);
  bool ok = actuadoresIniciar();    // primero: deja motor y freno apagados
  ok = sensorIniciar() && ok;
  registroEvento(ok ? "BANCO LISTO" : "ERROR AL INICIAR PERIFERICOS");
  tProximo = micros() + TS_US;
}

void loop() {
  comandosAtender();
  const uint32_t ahora = micros();
  if ((int32_t)(ahora - tProximo) >= 0) {   // robusto al desborde de micros()
    tProximo += TS_US;                      // sin deriva acumulada
    ensayoTick();
  }
}
