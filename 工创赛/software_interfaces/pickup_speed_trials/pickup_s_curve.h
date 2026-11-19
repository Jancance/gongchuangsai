#ifndef PICKUP_S_CURVE_H
#define PICKUP_S_CURVE_H
#include <stdint.h>
#include <math.h>
/* Symmetric7-phase reference. RPM/RPMs limits are exact model limits, not a
 * guarantee of plant tracking/torque/jerk on the vehicle. No fixed bottom dwell.
 * Uses revolutions andseconds internally; q' reaches zero at turnaround. */
typedef struct {float distance,total,duration[7],jerk[7],rpm,accel;} PickupSCurve;
static uint32_t PickupSCurve_Begin(PickupSCurve *p,float distance,float rpm,float accel,float ramp)
{
 float v,a,j,tj,ta,peak,dc,tv;uint32_t i;
 if(!p || !(distance>=0.0f && distance<=1000.0f) || !(rpm>0.0f && rpm<=3000.0f) ||
  !(accel>0.0f && accel<=65535.0f) || !(ramp>=.001f && ramp<=1.0f))return 0U;
 v=rpm/60.0f;a=accel/60.0f;j=a/ramp;tj=fminf(ramp,sqrtf(v/j));
 ta=fmaxf(0.0f,v/a-tj);peak=j*tj*(ta+tj);dc=peak*(ta+2.0f*tj);
 if(distance>=dc)tv=(distance-dc)/peak;
 else {
  tv=0.0f;
  if(distance>=2.0f*a*ramp*ramp){tj=ramp;ta=(-3.0f*tj+sqrtf(tj*tj+4.0f*distance/a))/2.0f;}
  else {tj=powf(distance/(2.0f*j),1.0f/3.0f);ta=0.0f;}
 }
 p->distance=distance;p->rpm=rpm;p->accel=accel;p->total=0.0f;
 p->duration[0]=p->duration[2]=p->duration[4]=p->duration[6]=tj;
 p->duration[1]=p->duration[5]=ta;p->duration[3]=tv;
 for(i=0U;i<7U;i++){p->jerk[i]=0.0f;p->total+=p->duration[i];}
 p->jerk[0]=p->jerk[6]=j;p->jerk[2]=p->jerk[4]=-j;return 1U;
}
static void PickupSCurve_Eval(const PickupSCurve *p,float elapsed,float *position,float *velocity)
{
 float x=0.0f,v=0.0f,a=0.0f,dt,j;uint32_t i;
 if(elapsed>=p->total){*position=p->distance;*velocity=0.0f;return;}
 if(elapsed<0.0f)elapsed=0.0f;
 for(i=0U;i<7U && elapsed>0.0f;i++){
  dt=fminf(elapsed,p->duration[i]);j=p->jerk[i];
  x+=v*dt+.5f*a*dt*dt+j*dt*dt*dt/6.0f;
  v+=a*dt+.5f*j*dt*dt;a+=j*dt;elapsed-=dt;
 }
 *position=fminf(p->distance,fmaxf(0.0f,x));*velocity=fmaxf(0.0f,v);
}
#endif
