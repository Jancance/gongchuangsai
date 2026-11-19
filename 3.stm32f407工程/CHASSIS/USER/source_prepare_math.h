#ifndef SOURCE_PREPARE_MATH_H
#define SOURCE_PREPARE_MATH_H
#include <math.h>
static __inline float SourcePrep_Smooth(float u)
{if(u<=0)return 0;if(u>=1)return 1;return u*u*u*(10.0f+u*(-15.0f+6.0f*u));}
static __inline float SourcePrep_Duration(float delta,float speed,float accel)
{
 if(!(fabsf(delta)<=180 && speed>0 && speed<=300 && accel>0 && accel<=360))return -1;
 return fmaxf(.5f,fmaxf(fabsf(delta)*1.875f/speed,sqrtf(fabsf(delta)*5.774f/accel)));
}
#endif
