#pragma once
#include <cmath>
#include <cstdint>
namespace control {
inline float clamp(float x,float a,float b) {return x<a?a:(x>b?b:x);}
inline int pulse(float u,int freeUs,int contactUs) {
  return int(std::lround(freeUs+clamp(u,0,1)*(contactUs-freeUs)));
}
struct PID {
  float kp=.020f,ki=.010f,kd=.0005f,tf=.05f;
  float integral=0,derivative=0,previous=0;
  void reset(float w) {integral=0;derivative=0;previous=w;}
  float step(float ref,float w,float dt) {
    if(!std::isfinite(ref)||!std::isfinite(w)||dt<=0||dt>.25f)return 0;
    float e=w-ref,alpha=tf/(tf+dt);
    derivative=alpha*derivative+(1-alpha)*(w-previous)/dt;previous=w;
    float base=kp*e+kd*derivative;
    float next=clamp(integral+ki*e*dt,0,1);
    float raw=base+next;
    if((raw>=0&&raw<=1)||(raw>1&&e<0)||(raw<0&&e>0))integral=next;
    return clamp(base+integral,0,1);
  }
};
// Pure assertions: no pins, PWM, NVS or simulated claims about the physical plant.
inline const char* selftest() {
  PID p;p.kp=.1f;p.ki=0;p.kd=0;p.reset(10);
  if(std::fabs(p.step(8,10,.02f)-.2f)>.0001f)return "inverse_error";
  if(p.step(12,10,.02f)!=0)return "underspeed_releases";
  p.ki=1;p.reset(10);
  for(int k=0;k<100;++k)if(p.step(-10,10,.02f)!=1)return "upper_saturation";
  if(p.integral!=0)return "conditional_antiwindup";
  p.kp=0;p.ki=0;p.kd=1;p.reset(10);
  if(p.step(0,10,.02f)!=0||p.step(20,10,.02f)!=0)return "no_reference_derivative_kick";
  if(pulse(0,1600,1400)!=1600||pulse(1,1600,1400)!=1400||pulse(.25f,1600,1400)!=1550)return "reversed_servo_mapping";
  if(pulse(-1,1200,1800)!=1200||pulse(2,1200,1800)!=1800)return "servo_clamping";
  return nullptr;
}
}
