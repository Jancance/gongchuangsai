
#include "leftup_policy_candidate.h"
#include <stdio.h>
int main(void){float v;unsigned failures=0,checks=0;
#define CK(x) do{checks++;if(!(x))failures++;}while(0)
 CK(!DM_CAN2_StartupReturnDelta(39.047f,-296.544037f,0,&v));
 CK(DM_CAN2_StartupReturnDelta(39.047f,-296.544037f,1,&v) && fabsf(v-24.408963f)<.01f);
 CK(DM_CAN2_StartupReturnDelta(-116.544037f,-296.544037f,1,&v) && fabsf(v+180)<.01f);
 CK(!DM_CAN2_StartupReturnDelta(-96.544037f,-296.544037f,1,&v));
 CK(!DM_CAN2_StartupReturnDelta(NAN,-296.544037f,1,&v));
 CK(DM_CAN2_StartupReturnDelta(-296.544037f,-296.544037f,1,&v) && v==0);
 printf("LEFTUP checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
