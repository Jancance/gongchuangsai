#include <stdio.h>
#include "dm_can2_startup_plan.h"
int main(void){float d=999,z=-296.544037f;unsigned n=0;
#define C(x) do{n++;if(!(x)){printf("fail %u\\n",n);return 1;}}while(0)
C(DM_CAN2_StartupCcwDelta(z+180,z,&d)&&fabsf(d-180)<0.01f);
C(DM_CAN2_StartupCcwDelta(z+177.833679f,z,&d)&&d>180 && d<183);
C(DM_CAN2_StartupCcwDelta(z+190,z,&d)&&fabsf(d-170)<0.01f);
C(DM_CAN2_StartupCcwDelta(z,z,&d)&&d==0);
C(DM_CAN2_StartupCcwDelta(z-1,z,&d)&&d==0);
C(!DM_CAN2_StartupCcwDelta(z+90,z,&d));
C(!DM_CAN2_StartupCcwDelta(z+270,z,&d));
C(!DM_CAN2_StartupCcwDelta(NAN,z,&d));
C(!DM_CAN2_StartupCcwDelta(z,INFINITY,&d));
C(!DM_CAN2_StartupCcwDelta(z,z,0));
printf("CCW plan checks=%u passed\\n",n);return 0;}