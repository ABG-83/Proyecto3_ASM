/*
  Banco de frenado (Proyecto Grupal 2) - Codigo base del PID digital SIN delay()
  CE-1110 Analisis de Senales Mixtas - Tarea 2, Pregunta 3

  Placa: ESP32-S3 Dev Module (nucleo Arduino-ESP32 version 3.x).
  Conexiones (las mismas de la prueba de motor + RPM del banco):
    - Sensor IR KY-033: V+ -> 3V3, G -> GND, S -> GPIO8 (marcas en el disco).
    - L298N, canal A (motor de la rueda): ENA -> GPIO4, IN1 -> GPIO5, IN2 -> GPIO6.
    - Servo MG90S del freno: senal -> GPIO7 (alimentacion de 5 V aparte, GND comun).

  Estructura:
    - esp_timer cada 1 ms: muestrea el sensor y filtra rebotes (fase estable >= 5 ms),
      igual que la prueba del banco. Guarda el instante de cada marca.
    - Planificador por micros(): la tarea de control corre exactamente cada TS_US.
    - Maquina de estados: REPOSO -> ARRANQUE (motor gira) -> FRENADO (PID) -> FIN.
    - PID discreto: integral por rectangulos, derivada filtrada sobre la medicion,
      saturacion [0,1] y anti-windup por integracion condicional.
    - Registro CSV por los dos puertos serie: t_ms,ref,w,e,u

  Comandos por Monitor Serie: A = iniciar ensayo, S o ! = parar,
  P<valor>, I<valor>, D<valor> = cambiar ganancias (ej. P0.02).

  El freno es "de accion inversa": si la rueda va MAS rapido que la referencia
  hay que frenar MAS. Por eso el error se define como e = w - ref.
*/

#include <Arduino.h>
#include <esp_timer.h>

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Selecciona ESP32S3 Dev Module."
#endif

// ------------------------------ Pines ------------------------------------
constexpr uint8_t SENSOR_PIN = 8;   // KY-033
constexpr uint8_t MOTOR_ENA  = 4;   // PWM del motor de la rueda (L298N)
constexpr uint8_t MOTOR_IN1  = 5;
constexpr uint8_t MOTOR_IN2  = 6;
constexpr uint8_t SERVO_PIN  = 7;   // MG90S del freno

// ------------------------- Sensor ----------------------------------------
// IMPORTANTE: con 1 sola marca la velocidad solo se actualiza una vez por vuelta.
// Se recomienda pegar 8 marcas iguales en el disco y cambiar este valor a 8.
constexpr uint32_t PULSOS_POR_VUELTA = 1;
constexpr uint32_t RPM_MAX = 600;                 // limite de la prueba del banco
constexpr uint32_t RPM_MIN = 30;                  // por debajo se considera detenido
constexpr uint32_t MUESTREO_SENSOR_US = 1000;     // muestreo del sensor: 1 kHz
constexpr uint32_t ESTABLE_US = 5000;             // cada fase debe durar >= 5 ms
constexpr uint32_t INTERVALO_MIN_US = 60000000UL / (RPM_MAX * PULSOS_POR_VUELTA);
constexpr uint32_t TIMEOUT_US       = 60000000UL / (RPM_MIN * PULSOS_POR_VUELTA);

// ------------------------- Control ----------------------------------------
constexpr uint32_t TS_US = 20000UL;               // Ts = 20 ms -> fs = 50 Hz
constexpr float    TS    = TS_US * 1.0e-6f;

constexpr bool     LAZO_CERRADO = true;   // false = escalon a lazo abierto (P6/P7)
constexpr float    U_ESCALON    = 0.5f;   // accion fija para el ensayo a lazo abierto
constexpr uint8_t  PWM_MAX_PRUEBA = 160;  // tope de la prueba del banco (fuente 9 V)
constexpr uint8_t  PWM_MOTOR    = 140;    // PWM de arranque (0-160)
constexpr uint32_t T_ARRANQUE_MS = 3000;  // giro libre antes de frenar
constexpr float    DECEL_REF    = 4.0f;   // pendiente de la referencia [rad/s^2]
                                          // (120 rpm = 12,6 rad/s -> 0 en ~3 s, alcanzable segun la P5)
constexpr uint32_t T_MAX_FRENADO_MS = 8000;

// ------------------------- Servo MG90S ------------------------------------
constexpr uint32_t SERVO_FREQ = 50;       // periodo de 20 ms
constexpr uint8_t  SERVO_RES  = 14;       // 16384 pasos en 20 ms (~1,2 us)
constexpr int US_LIBRE = 1100;   // zapata sin tocar el disco (CALIBRAR en el banco)
constexpr int US_MAX   = 1600;   // presion maxima permitida (CALIBRAR en el banco)

// ------------------------- PID --------------------------------------------
// Valores iniciales por SIMC con el modelo de la Pregunta 5:
//   K = 29,17 rad/s, tau1 = 5,06 s, theta = 0,21 s, tau_c = theta
//   Kp = tau1 / (K (tau_c + theta)) = 0,41 ;  Ti = min(tau1, 4 (tau_c + theta)) = 1,68 s
// Se ajustan experimentalmente cuando se tenga el modelo identificado (P7).
float Kp = 0.41f;
float Ki = 0.25f;    // Kp / Ti  [1/s]
float Kd = 0.0f;     // se empieza en 0: con pocas marcas la derivada es muy ruidosa
constexpr float TF_D   = 0.10f;                // const. de tiempo del filtro derivativo [s]
constexpr float ALFA_D = TF_D / (TF_D + TS);
constexpr float U_MIN = 0.0f, U_MAX = 1.0f;    // 0 = freno libre, 1 = freno a fondo

// ------------------------- Datos del sensor (compartidos) -----------------
portMUX_TYPE datosMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t ultimoPulsoUs = 0, periodoUs = 0;
volatile bool hayPulso = false, hayPeriodo = false;
int  nivelCrudo = HIGH, nivelEstable = HIGH;   // solo los usa la tarea del timer
uint32_t ultimoCambioUs = 0;
bool armado = false;
esp_timer_handle_t timerSensor = nullptr;

// ------------------------- Estado del ensayo ------------------------------
enum Estado { REPOSO, ARRANQUE, FRENADO, FIN };
Estado estado = REPOSO;
uint32_t tProximo = 0, tInicioEst = 0, sobrecargas = 0;
float w0 = 0.0f, integral = 0.0f, dwFiltrada = 0.0f, wAnterior = 0.0f;

// ======================= Salida por los dos puertos =======================
void escribir(const char *texto) {
  if (Serial) Serial.print(texto);   // USB nativo (CDC)
  Serial0.print(texto);              // puente USB-UART
}

// ======================= Sensor: tarea del esp_timer (cada 1 ms) ===========
// Corre en la tarea del temporizador, no imprime ni espera.
// Una marca valida = fase LOW estable seguida de fase HIGH estable (>= 5 ms c/u).
void muestrearSensor(void *) {
  const uint32_t ahora = micros();
  const int nivel = digitalRead(SENSOR_PIN);
  if (nivel != nivelCrudo) { nivelCrudo = nivel; ultimoCambioUs = ahora; }
  if (ahora - ultimoCambioUs < ESTABLE_US) return;

  const bool cambio = (nivelEstable != nivelCrudo);
  nivelEstable = nivelCrudo;
  if (nivelEstable == LOW) { armado = true; return; }
  if (!(cambio && armado)) return;
  armado = false;

  const uint32_t tPulso = ultimoCambioUs;        // inicio de la fase, no fin del filtro
  portENTER_CRITICAL(&datosMux);
  const uint32_t intervalo = tPulso - ultimoPulsoUs;
  if (!hayPulso || intervalo >= INTERVALO_MIN_US) {
    if (hayPulso) { periodoUs = intervalo; hayPeriodo = true; }
    ultimoPulsoUs = tPulso;
    hayPulso = true;
  }
  portEXIT_CRITICAL(&datosMux);
}

// Velocidad angular [rad/s] a partir del tiempo entre marcas.
float leerVelocidad() {
  portENTER_CRITICAL(&datosMux);
  uint32_t periodo = periodoUs;
  const uint32_t ultimo = ultimoPulsoUs;
  const bool valido = hayPeriodo;
  portEXIT_CRITICAL(&datosMux);

  const uint32_t desde = micros() - ultimo;
  if (!valido || desde > TIMEOUT_US) return 0.0f;    // rueda detenida
  // Si la marca siguiente ya tarda mas que el ultimo periodo, la rueda va mas lenta:
  // se usa ese tiempo como cota para no sobrestimar la velocidad.
  if (desde > periodo) periodo = desde;
  return (2.0f * PI) / (PULSOS_POR_VUELTA * (periodo * 1.0e-6f));
}

// ======================= Actuadores =======================================
void aplicarFreno(float u) {           // u en [0,1]
  u = constrain(u, U_MIN, U_MAX);
  const float anchoUs = US_LIBRE + u * (US_MAX - US_LIBRE);
  ledcWrite(SERVO_PIN, (uint32_t)(anchoUs / 20000.0f * ((1UL << SERVO_RES) - 1)));
}

void motor(uint8_t pwm) {              // 0..PWM_MAX_PRUEBA, un solo sentido
  if (pwm > PWM_MAX_PRUEBA) pwm = PWM_MAX_PRUEBA;
  digitalWrite(MOTOR_IN1, pwm ? HIGH : LOW);
  digitalWrite(MOTOR_IN2, LOW);
  ledcWrite(MOTOR_ENA, pwm);
}

// ======================= PID discreto =====================================
void reiniciarPID(float w) { integral = 0.0f; dwFiltrada = 0.0f; wAnterior = w; }

// Devuelve u[k] en [0,1]. Error de accion inversa: e = w - ref.
float calcularPID(float ref, float w) {
  const float e = w - ref;

  // Derivada sobre la medicion (evita "derivative kick") y filtrada.
  const float dw = (w - wAnterior) / TS;
  dwFiltrada = ALFA_D * dwFiltrada + (1.0f - ALFA_D) * dw;
  wAnterior = w;

  const float uSinSat = Kp * e + integral + Kd * dwFiltrada;
  const float u = constrain(uSinSat, U_MIN, U_MAX);

  // Anti-windup: integracion condicional.
  const bool satArriba = (uSinSat > U_MAX) && (e > 0.0f);
  const bool satAbajo  = (uSinSat < U_MIN) && (e < 0.0f);
  if (!satArriba && !satAbajo) {
    integral += Ki * TS * e;           // integral por rectangulos (Euler)
    integral = constrain(integral, U_MIN, U_MAX);
  }
  return u;
}

// Referencia decreciente fisicamente alcanzable: rampa desde w0 hasta 0.
float referencia(float tSeg) {
  const float r = w0 - DECEL_REF * tSeg;
  return (r > 0.0f) ? r : 0.0f;
}

// ======================= Tarea periodica (cada Ts) ========================
void tareaControl() {
  const float w = leerVelocidad();
  const uint32_t tEst = millis() - tInicioEst;
  char fila[80];

  switch (estado) {
    case REPOSO:
      motor(0); aplicarFreno(0.0f);
      break;

    case ARRANQUE:                     // el motor lleva la rueda a velocidad
      motor(PWM_MOTOR); aplicarFreno(0.0f);
      if (tEst >= T_ARRANQUE_MS) {
        motor(0);                      // se corta la traccion y empieza el frenado
        w0 = w;
        reiniciarPID(w);
        estado = FRENADO;
        tInicioEst = millis();
        escribir("t_ms,ref,w,e,u\n");
      }
      break;

    case FRENADO: {
      const float ref = referencia(tEst * 1.0e-3f);
      const float u = LAZO_CERRADO ? calcularPID(ref, w) : U_ESCALON;
      aplicarFreno(u);
      snprintf(fila, sizeof(fila), "%lu,%.2f,%.2f,%.2f,%.3f\n",
               (unsigned long)tEst, ref, w, w - ref, u);
      escribir(fila);
      if (w <= 0.0f || tEst >= T_MAX_FRENADO_MS) { estado = FIN; tInicioEst = millis(); }
      break;
    }

    case FIN:
      aplicarFreno(0.0f); motor(0);
      snprintf(fila, sizeof(fila), "# FIN sobrecargas=%lu\n", (unsigned long)sobrecargas);
      escribir(fila);
      estado = REPOSO;
      break;
  }
}

// ======================= Tareas de fondo (sin periodo fijo) ===============
void procesarComando(char c, Stream &canal) {
  switch (toupper((unsigned char)c)) {
    case 'A': if (estado == REPOSO) { estado = ARRANQUE; tInicioEst = millis(); escribir("# ARRANQUE\n"); } break;
    case 'S': case '!': estado = FIN; break;
    case 'P': Kp = canal.parseFloat(); break;
    case 'I': Ki = canal.parseFloat(); break;
    case 'D': Kd = canal.parseFloat(); break;
    default: break;
  }
}

void tareasFondo() {
  // Se atiende como maximo un caracter por canal en cada vuelta: nunca bloquea.
  if (Serial.available())  procesarComando((char)Serial.read(), Serial);
  if (Serial0.available()) procesarComando((char)Serial0.read(), Serial0);
}

// ======================= Arduino ==========================================
void setup() {
  // Motor apagado desde el primer instante.
  for (uint8_t pin : {MOTOR_ENA, MOTOR_IN1, MOTOR_IN2}) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  Serial0.begin(115200);
  Serial.begin(115200);
  Serial.setTimeout(5);  Serial0.setTimeout(5);   // parseFloat no se queda esperando

  ledcAttach(MOTOR_ENA, 5000, 8);                 // igual que la prueba del banco
  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);
  motor(0);
  aplicarFreno(0.0f);

  nivelCrudo = nivelEstable = digitalRead(SENSOR_PIN);
  ultimoCambioUs = micros();
  esp_timer_create_args_t args = {};
  args.callback = &muestrearSensor;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name = "sensor_rpm";
  args.skip_unhandled_events = true;
  if (esp_timer_create(&args, &timerSensor) != ESP_OK ||
      esp_timer_start_periodic(timerSensor, MUESTREO_SENSOR_US) != ESP_OK) {
    escribir("ERROR_TEMPORIZADOR\n");
  }

  escribir("# Banco de frenado listo. A = iniciar, S o ! = parar.\n");
  tProximo = micros() + TS_US;
}

void loop() {
  const uint32_t ahora = micros();
  // Comparacion con resta con signo: funciona aunque micros() se desborde.
  if ((int32_t)(ahora - tProximo) >= 0) {
    tProximo += TS_US;                // se suma Ts al instante PROGRAMADO: sin deriva
    tareaControl();
    if ((int32_t)(micros() - tProximo) >= 0) {   // la tarea tardo mas que Ts
      sobrecargas++;
      tProximo = micros() + TS_US;
    }
  }
  tareasFondo();                      // todo lo demas corre entre muestras
}
