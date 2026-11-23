#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <driver/ledc.h>
#include <esp_timer.h>
#include <cstdarg>
#include <ctype.h>
#include "ControlCore.h"
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Selecciona ESP32S3 Dev Module"
#endif
#if ARDUINO_USB_MODE!=1 || ARDUINO_USB_CDC_ON_BOOT!=1
#error "USB Mode Hardware CDC and JTAG; USB CDC On Boot Enabled"
#endif

constexpr int SENSOR=8,SERVO=7,ENA=4,IN1=5,IN2=6;
constexpr ledc_mode_t PWM_MODE=LEDC_LOW_SPEED_MODE;
constexpr ledc_channel_t MOTOR_CH=LEDC_CHANNEL_0,SERVO_CH=LEDC_CHANNEL_2;
constexpr uint32_t TS_US=20000;
constexpr float TWO_PI_F=6.28318530718f;
struct LogMessage {char text[256];};
QueueHandle_t logQueue=nullptr;
volatile uint32_t logLost=0;
bool ready=false,servoOn=false;
int servoUs=0,motorPwm=0,freeUs=0,contactUs=0;
uint32_t manualSince=0,servoSince=0;
uint32_t ppr=1;
float stepU=.25f,targetRpm=80,refW=0,u=0;
control::PID pid;
enum class Mode {NONE,K,OA,PID,COAST};
enum class State {IDLE,SETTLE,KICK,WARM,ACTIVE,RELEASE};
Mode mode=Mode::NONE;State state=State::IDLE;
uint32_t stateSince=0,runSince=0,nextTick=0,lastTick=0,missedTicks=0,maxLate=0;
float initialW=0;
uint32_t loggedPulse=0;
portMUX_TYPE sensorMux=portMUX_INITIALIZER_UNLOCKED;
struct SensorData {
  uint32_t count=0,last=0,period=0,changed=0,sampled=0,rawEdges=0,rejected=0,gaps=0;
  int raw=HIGH,stable=HIGH;
  bool hasPulse=false,hasPeriod=false,armed=false;
};
SensorData sensor;
uint32_t minInterval=100000;
esp_timer_handle_t sensorTimer=nullptr;
struct Measurement {SensorData s;uint32_t age=0,timeout=0;float w=0;bool valid=false,bound=false;};

void logf(const char *fmt,...) {
  if(!logQueue)return;
  LogMessage msg;va_list args;va_start(args,fmt);vsnprintf(msg.text,sizeof(msg.text),fmt,args);va_end(args);
  if(xQueueSend(logQueue,&msg,0)!=pdTRUE)++logLost;
}
void loggerTask(void*) {
  LogMessage msg;
  for(;;)if(xQueueReceive(logQueue,&msg,portMAX_DELAY)==pdTRUE) {
    size_t n=strlen(msg.text);Serial0.write((const uint8_t*)msg.text,n);
    // Native USB is optional; do not let a closed second monitor block the UART log.
    if(Serial && Serial.availableForWrite()>=int(n))Serial.write((const uint8_t*)msg.text,n);
  }
}
const char *modeName() {
  switch(mode){case Mode::K:return "K";case Mode::OA:return "OA";case Mode::PID:return "PID";case Mode::COAST:return "COAST";default:return "NONE";}
}
const char *stateName() {
  switch(state){case State::SETTLE:return "SETTLE";case State::KICK:return "KICK";case State::WARM:return "WARM";case State::ACTIVE:return "ACTIVE";case State::RELEASE:return "RELEASE";default:return "IDLE";}
}
bool setMotor(int pwm) {
  pwm=constrain(pwm,0,255);
  if(!ready)return false;
  // At zero: disable first, then both directions LOW (coast).
  if(pwm) {digitalWrite(IN2,LOW);digitalWrite(IN1,HIGH);}
  esp_err_t e=ledc_set_duty_and_update(PWM_MODE,MOTOR_CH,pwm,0);
  if(!pwm){digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);}
  if(e!=ESP_OK) {ledc_stop(PWM_MODE,MOTOR_CH,0);digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);ready=false;motorPwm=0;logf("# ERROR_PWM_MOTOR:%d\n",e);return false;}
  motorPwm=pwm;return true;
}
bool setServo(int us) {
  if(!ready||us<1200||us>1800)return false;
  uint32_t duty=(uint32_t(us)*16384UL+10000)/20000;
  esp_err_t e=ledc_set_duty_and_update(PWM_MODE,SERVO_CH,duty,0);
  if(e!=ESP_OK) {ledc_stop(PWM_MODE,SERVO_CH,0);ready=false;servoOn=false;logf("# ERROR_PWM_SERVO:%d\n",e);return false;}
  if(!servoOn)servoSince=millis();servoOn=true;servoUs=us;return true;
}
void servoOff() {
  if(ledc_set_duty_and_update(PWM_MODE,SERVO_CH,0,0)!=ESP_OK)ledc_stop(PWM_MODE,SERVO_CH,0);
  servoOn=false;servoUs=0;
}
void emergency(const char *why) {
  if(ready)setMotor(0);
  else{ledc_stop(PWM_MODE,MOTOR_CH,0);digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);motorPwm=0;}
  servoOff();state=State::IDLE;mode=Mode::NONE;u=0;
  logf("# PARADA:%s; PWM0, no corta alimentacion fisica\n",why);
}
bool calibrated() {return freeUs>=1200&&freeUs<=1800&&contactUs>=1200&&contactUs<=1800&&abs(contactUs-freeUs)>=20&&abs(contactUs-freeUs)<=400;}
void brake(float command) {u=control::clamp(command,0,1);if(!setServo(control::pulse(u,freeUs,contactUs)))emergency("SERVO");}
void changeState(State s) {state=s;stateSince=millis();logf("# EVENT t_ms=%lu estado=%s modo=%s pwm=%d u=%.4f\n",(unsigned long)(millis()-runSince),stateName(),modeName(),motorPwm,u);}
void normalStop(const char *why) {
  if(!setMotor(0)){emergency(why);return;}
  u=0;
  if(freeUs>=1200&&freeUs<=1800) {setServo(freeUs);changeState(State::RELEASE);}
  else {servoOff();state=State::IDLE;mode=Mode::NONE;}
  logf("# DETENIDO:%s\n",why);
}
void sampleSensor(void*) {
  uint32_t now=micros();int level=digitalRead(SENSOR);
  portENTER_CRITICAL(&sensorMux);
  if(now-sensor.sampled>3000) {
    ++sensor.gaps;sensor.raw=level;sensor.stable=level;sensor.changed=now;
    sensor.armed=false;sensor.hasPulse=false;sensor.hasPeriod=false;
  } else if(level!=sensor.raw){sensor.raw=level;sensor.changed=now;if(level==HIGH)++sensor.rawEdges;}
  sensor.sampled=now;
  if(now-sensor.changed>=5000) {
    bool changed=sensor.stable!=sensor.raw;sensor.stable=sensor.raw;
    if(sensor.stable==LOW)sensor.armed=true;
    else if(changed&&sensor.armed) {
      sensor.armed=false;uint32_t dt=sensor.changed-sensor.last;
      if(!sensor.hasPulse||dt>=minInterval) {
        if(sensor.hasPulse){sensor.period=dt;sensor.hasPeriod=true;}
        sensor.last=sensor.changed;sensor.hasPulse=true;++sensor.count;
      }else ++sensor.rejected;
    }
  }
  portEXIT_CRITICAL(&sensorMux);
}
Measurement measure() {
  Measurement m;portENTER_CRITICAL(&sensorMux);m.s=sensor;portEXIT_CRITICAL(&sensorMux);
  m.age=m.s.hasPulse?micros()-m.s.last:0;
  m.timeout=uint32_t(constrain(uint64_t(m.s.period)*3,1500000ULL,10000000ULL));
  m.valid=m.s.hasPeriod&&m.s.period>0&&m.age<=m.timeout;
  if(m.valid) {
    // Beyond the last period this is only an upper bound on average speed,
    // not a fresh observation. Both raw period and bound flag are in CSV.
    uint32_t effective=max(m.s.period,m.age);m.bound=m.age>m.s.period;
    m.w=TWO_PI_F*1000000.0f/(float(ppr)*effective);
  }
  return m;
}
void resetSensor() {
  portENTER_CRITICAL(&sensorMux);
  sensor=SensorData{};sensor.raw=digitalRead(SENSOR);sensor.stable=sensor.raw;
  sensor.changed=micros();sensor.sampled=sensor.changed;minInterval=100000/ppr;
  portEXIT_CRITICAL(&sensorMux);loggedPulse=0;
}
void status() {
  Measurement m=measure();
  logf("# ESTADO V1 listo=%d modo=%s estado=%s pwm=%d servo_us=%d libre=%d contacto=%d calibrado=%d ppr=%lu rpm=%.2f valido=%d\n",ready,modeName(),stateName(),motorPwm,servoUs,freeUs,contactUs,calibrated(),(unsigned long)ppr,m.w*60/TWO_PI_F,m.valid);
  logf("# AJUSTES kp=%.6f ki=%.6f kd=%.6f ref_rpm=%.2f U=%.3f atrasos=%lu max_us=%lu log_perdido=%lu huecos_sensor=%lu\n",pid.kp,pid.ki,pid.kd,targetRpm,stepU,(unsigned long)missedTicks,(unsigned long)maxLate,(unsigned long)logLost,(unsigned long)m.s.gaps);
}
bool saveCalibration() {
  Preferences prefs;if(!prefs.begin("banco_freno",false))return false;
  bool ok=prefs.putInt("libre",freeUs)==4&&prefs.putInt("contacto",contactUs)==4;
  prefs.end();resetSensor();return ok;
}
void start(Mode wanted) {
  if(!ready||state!=State::IDLE||!freeUs||(wanted!=Mode::K&&!calibrated())) {logf("# BLOQUEADO: necesita reposo y L; OA/PID/COAST tambien P. ?:estado\n");return;}
  if(!setServo(freeUs)){emergency("LIBERAR");return;}
  resetSensor();pid.reset(0);u=0;refW=0;mode=wanted;runSince=millis();missedTicks=0;maxLate=0;
  logf("# INICIO modo=%s PPR=%lu; motor255/500ms luego160; PID inicial SIN SINTONIZAR\n",modeName(),(unsigned long)ppr);
  logf("t_ms,ref,w,e,u,rpm,pulsos,periodo_us,nuevo,edad_ms,valido,cota,pwm,servo_us,estado,modo,huecos,descartes,atraso_us,log_perdido\n");
  changeState(State::SETTLE);
}
void controlTick(float dt,uint32_t late) {
  Measurement m=measure();uint32_t elapsed=millis()-stateSince;
  if(!ready){if(state!=State::IDLE)emergency("PERIFERICOS");return;}
  if(state==State::SETTLE&&elapsed>=500){if(setMotor(255))changeState(State::KICK);else emergency("MOTOR");}
  else if(state==State::KICK&&elapsed>=500){if(setMotor(160))changeState(State::WARM);else emergency("MOTOR");}
  else if(state==State::WARM) {
    if(elapsed>=3500&&!m.valid){normalStop("SIN_DOS_PULSOS_EN_ARRANQUE");}
    else if(elapsed>=5000) {
      if(mode==Mode::K)normalStop("FIN_K");
      else if(!m.valid||m.w<2)normalStop("VELOCIDAD_INSUFICIENTE");
      else {
        initialW=m.w;pid.reset(m.w);
        if(mode==Mode::COAST)setMotor(0);
        if(mode==Mode::OA)brake(stepU);
        if(state==State::WARM)changeState(State::ACTIVE);
      }
    }
  }else if(state==State::ACTIVE) {
    if(!m.valid){normalStop("SIN_PULSOS_RECIENTES_NO_PRUEBA_CERO");}
    else if(mode==Mode::OA) {refW=0;if(elapsed>=6000){brake(0);changeState(State::RELEASE);}}
    else if(mode==Mode::PID||mode==Mode::COAST) {
      refW=mode==Mode::PID?targetRpm*TWO_PI_F/60:max(0.0f,initialW-40.0f*elapsed*.001f);
      brake(pid.step(refW,m.w,dt));
      if(state==State::ACTIVE&&elapsed>=(mode==Mode::PID?10000u:6000u)) {brake(0);changeState(State::RELEASE);}
    }
  }else if(state==State::RELEASE&&elapsed>=2000) {
    setMotor(0);servoOff();logf("# FIN modo=%s perdidas=%lu\n",modeName(),(unsigned long)logLost);state=State::IDLE;mode=Mode::NONE;refW=0;u=0;
  }
  // No automatic signal cuts during an experiment. Manual calibration has a 60s watchdog.
  if(state==State::IDLE&&servoOn&&millis()-manualSince>=60000){servoOff();logf("# FIN_CALIBRACION_60S\n");}
  static uint32_t lastLog=0;
  if(state!=State::IDLE&&millis()-lastLog>=100) {
    lastLog=millis();bool fresh=m.s.count!=loggedPulse;loggedPulse=m.s.count;
    logf("%lu,%.4f,%.4f,%.4f,%.4f,%.2f,%lu,%lu,%d,%ld,%d,%d,%d,%d,%s,%s,%lu,%lu,%lu,%lu\n",(unsigned long)(millis()-runSince),refW,m.valid?m.w:NAN,m.valid?m.w-refW:NAN,u,m.valid?m.w*60/TWO_PI_F:NAN,(unsigned long)m.s.count,(unsigned long)m.s.period,fresh,m.s.hasPulse?long(m.age/1000):-1L,m.valid,m.bound,motorPwm,servoUs,stateName(),modeName(),(unsigned long)m.s.gaps,(unsigned long)m.s.rejected,(unsigned long)late,(unsigned long)logLost);
  }
}
bool number(const char *s,float &v) {if(!*s)return false;char *end;v=strtof(s,&end);return end!=s&&!*end&&std::isfinite(v);}
void help() {logf("# C centro; V1200..1800; +/-50us; F+/F-10us; L guarda libre; P guarda contacto y libera; R libera; K motor; OA escalon; PID regula; COAST frena motor apagado; S para/libera; X/! PWM0\n");logf("# U0.25 escalon; REF80 rpm; KP0.02 KI0.01 KD0.0005; PPR1; ?; SELFTEST. Cada orden con Enter. Calibrar L/P SIN motor.\n");}
void command(char *s) {
  while(isspace((unsigned char)*s))++s;
  char *end=s+strlen(s);while(end>s&&isspace((unsigned char)end[-1]))*--end=0;
  for(char *p=s;*p;++p)*p=toupper((unsigned char)*p);
  if(!*s)return;
  if(!strcmp(s,"X")||!strcmp(s,"!")){emergency("USUARIO");return;}
  if(!strcmp(s,"S")||!strcmp(s,"R")){normalStop("USUARIO");return;}
  if(!strcmp(s,"?")){status();return;}
  if(!strcmp(s,"H")){help();return;}
  if(state!=State::IDLE){logf("# OCUPADO: S para/libera; X emergencia\n");return;}
  if(!strcmp(s,"SELFTEST")){const char *err=control::selftest();logf("# SELFTEST %s %s; sin mover actuadores\n",err?"FAIL":"PASS",err?err:"inverse_error underspeed_releases upper_saturation conditional_antiwindup no_reference_derivative_kick reversed_servo_mapping servo_clamping");return;}
  if(!strcmp(s,"K")){start(Mode::K);return;}
  if(!strcmp(s,"OA")){start(Mode::OA);return;}
  if(!strcmp(s,"PID")){start(Mode::PID);return;}
  if(!strcmp(s,"COAST")){start(Mode::COAST);return;}
  int wanted=-1;float val;
  if(!strcmp(s,"C"))wanted=1500;
  else if(!strcmp(s,"+"))wanted=servoOn?servoUs+50:1500;
  else if(!strcmp(s,"-"))wanted=servoOn?servoUs-50:1500;
  else if(!strcmp(s,"F+"))wanted=servoOn?servoUs+10:1500;
  else if(!strcmp(s,"F-"))wanted=servoOn?servoUs-10:1500;
  else if(s[0]=='V'&&number(s+1,val))wanted=int(std::lround(val));
  if(wanted!=-1) {
    if(wanted<1200||wanted>1800){logf("# LIMITE_CALIBRACION 1200..1800us\n");return;}
    if(setServo(wanted)){manualSince=millis();logf("# SERVO_US=%d continuo hasta60s; X corta. No es medida de angulo\n",servoUs);}else emergency("CALIBRACION");return;
  }
  if(!strcmp(s,"L")||!strcmp(s,"P")) {
    if(!servoOn){logf("# CALIBRAR: primero C o V... con brazo libre\n");return;}
    if(s[0]=='L'){freeUs=servoUs;contactUs=0;}
    else {
      if(!freeUs||abs(servoUs-freeUs)<20||abs(servoUs-freeUs)>400){logf("# P_INVALIDA: primero L, diferencia20..400us; contacto SUAVE\n");return;}
      contactUs=servoUs;setServo(freeUs);
    }
    bool ok=saveCalibration();manualSince=millis();logf("# CALIBRACION libre=%d contacto=%d guardado=%d\n",freeUs,contactUs,ok);return;
  }
  if(!strncmp(s,"PPR",3)&&number(s+3,val)&&val>=1&&val<=20&&val==floorf(val)){ppr=(uint32_t)val;resetSensor();logf("# PPR=%lu debe coincidir con marcas reales\n",(unsigned long)ppr);return;}
  if(!strncmp(s,"REF",3)&&number(s+3,val)&&val>=10&&val<=500){targetRpm=val;status();return;}
  if(s[0]=='U'&&number(s+1,val)&&val>0&&val<=1){stepU=val;status();return;}
  if(!strncmp(s,"KP",2)&&number(s+2,val)&&val>=0&&val<=5){pid.kp=val;status();return;}
  if(!strncmp(s,"KI",2)&&number(s+2,val)&&val>=0&&val<=5){pid.ki=val;status();return;}
  if(!strncmp(s,"KD",2)&&number(s+2,val)&&val>=0&&val<=1){pid.kd=val;status();return;}
  help();
}
void readPort(Stream &port,char *buffer,uint8_t &length,bool &discard) {
  int budget=64;
  while(port.available()&&budget--) {
    char c=char(port.read());
    if(c=='!'){emergency("USUARIO");length=0;discard=false;}
    else if(c=='\n'||c=='\r'){if(!discard){buffer[length]=0;command(buffer);}length=0;discard=false;}
    else if(!discard){if(length<63)buffer[length++]=c;else{emergency("ORDEN_LARGA");length=0;discard=true;}}
  }
}
esp_err_t configurePwm() {
  ledc_timer_config_t t={};t.speed_mode=PWM_MODE;t.timer_num=LEDC_TIMER_0;t.duty_resolution=LEDC_TIMER_8_BIT;t.freq_hz=5000;t.clk_cfg=LEDC_AUTO_CLK;
  esp_err_t e=ledc_timer_config(&t);if(e!=ESP_OK)return e;
  t.timer_num=LEDC_TIMER_1;t.duty_resolution=LEDC_TIMER_14_BIT;t.freq_hz=50;
  if((e=ledc_timer_config(&t))!=ESP_OK)return e;
  ledc_channel_config_t c={};c.gpio_num=ENA;c.speed_mode=PWM_MODE;c.channel=MOTOR_CH;c.intr_type=LEDC_INTR_DISABLE;c.timer_sel=LEDC_TIMER_0;
  if((e=ledc_channel_config(&c))!=ESP_OK)return e;
  c.gpio_num=SERVO;c.channel=SERVO_CH;c.timer_sel=LEDC_TIMER_1;
  if((e=ledc_channel_config(&c))!=ESP_OK)return e;
  if((e=ledc_fade_func_install(0))!=ESP_OK)return e;
  if((e=ledc_set_duty_and_update(PWM_MODE,MOTOR_CH,0,0))!=ESP_OK)return e;
  return ledc_set_duty_and_update(PWM_MODE,SERVO_CH,0,0);
}
void setup() {
  for(int pin:{ENA,IN1,IN2,SERVO}){digitalWrite(pin,LOW);pinMode(pin,OUTPUT);}
  Serial0.setTxBufferSize(2048);Serial0.begin(115200);Serial.begin(115200);
  logQueue=xQueueCreate(48,sizeof(LogMessage));
  if(!logQueue||xTaskCreatePinnedToCore(loggerTask,"log_banco",4096,nullptr,1,nullptr,0)!=pdPASS)while(true)delay(1000);
  esp_err_t e=configurePwm();ready=e==ESP_OK;
  pinMode(SENSOR,INPUT_PULLUP);resetSensor();
  esp_timer_create_args_t args={};args.callback=sampleSensor;args.name="rpm_banco";args.dispatch_method=ESP_TIMER_TASK;args.skip_unhandled_events=true;
  if(e==ESP_OK)e=esp_timer_create(&args,&sensorTimer);
  if(e==ESP_OK)e=esp_timer_start_periodic(sensorTimer,1000);
  ready=e==ESP_OK;
  Preferences prefs;if(prefs.begin("banco_freno",true)){freeUs=prefs.getInt("libre",0);contactUs=prefs.getInt("contacto",0);prefs.end();}
  if(freeUs<1200||freeUs>1800){freeUs=0;contactUs=0;}
  if(contactUs&&!calibrated())contactUs=0;
  const char *test=control::selftest();if(test)ready=false;
  logf("# BANCO_COMPLETO_V1 listo=%d error=%d SELFTEST=%s. MOTOR PWM0 y SERVO SIN PULSOS al arrancar\n",ready,e,test?test:"PASS");
  help();status();lastTick=micros();nextTick=lastTick+TS_US;
}
void loop() {
  static char usb[64]={},uart[64]={};static uint8_t nu=0,nr=0;static bool du=false,dr=false;
  readPort(Serial,usb,nu,du);readPort(Serial0,uart,nr,dr);
  uint32_t now=micros();
  if(int32_t(now-nextTick)>=0) {
    uint32_t late=now-nextTick;maxLate=max(maxLate,late);missedTicks+=late/TS_US;
    float dt=(now-lastTick)*1e-6f;lastTick=now;nextTick+=(late/TS_US+1)*TS_US;
    if(state!=State::IDLE&&dt>.10f)normalStop("ATRASO_CONTROL_MAYOR100MS");
    else controlTick(dt,late);
  }
  delay(1);
}
