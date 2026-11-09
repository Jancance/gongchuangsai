#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "dm_can2.h"
int main(void){
 CanTxMsg tx; int i; float r,d,finite,v; unsigned q;
 assert(DM_CAN2_SmoothVelocity(0)==0 && DM_CAN2_SmoothVelocity(1)==0);
 assert(fabsf(DM_CAN2_SmoothVelocity(.5f)-1.875f)<.00001f);
 for(i=1;i<999;i++){
  r=i/1000.f; d=DM_CAN2_SmoothVelocity(r);
  finite=(DM_CAN2_SmoothStep(r+.0005f)-DM_CAN2_SmoothStep(r-.0005f))/.001f;
  assert(fabsf(finite-d)<.003f);
  v=-2.5f*d/15.f;
  assert(DM_CAN2_BuildMitTrajectory(1.f,v,12.5f,30.f,10.f,&tx));
  q=((unsigned)tx.Data[2]<<4)|(tx.Data[3]>>4);
  assert(fabsf(((float)q*60.f/4095.f-30.f)-v)<60.f/4095.f+.00001f);
 }
 assert(!DM_CAN2_BuildMitTrajectory(0,NAN,12.5f,30.f,10.f,&tx));
 assert(!DM_CAN2_BuildMitTrajectory(0,.5001f,12.5f,30.f,10.f,&tx));
 assert(!DM_CAN2_BuildMitTrajectory(0,-.5001f,12.5f,30.f,10.f,&tx));
 assert(DM_CAN2_BuildMitHold(0,12.5f,30.f,10.f,&tx));
 assert((((unsigned)tx.Data[2]<<4)|(tx.Data[3]>>4))==2047);
 assert(DM_CAN2_BuildMitPreload(1,12.5f,30.f,10.f,&tx));
 assert((((unsigned)(tx.Data[3]&15)<<8)|tx.Data[4])==0);
 puts("998 derivative/negative velocity vectors, invalid velocity rejection, hold/preload compatibility passed");
 return 0;
}
