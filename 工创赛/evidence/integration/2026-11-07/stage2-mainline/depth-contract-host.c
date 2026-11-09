#include "source_parallel_flow.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static unsigned checks,failures;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL %d\n",__LINE__);}}while(0)
int main(void){SPF_Config c;SPF_Flow f;float values[]={-1,0,108400,118800,118801,NAN,INFINITY};unsigned i;memset(&c,0,sizeof(c));c.run_id=c.pose_version=c.live_references_valid=c.deploy_sweep_valid=1;c.transaction_timeout_ms=12000;c.gripper_open_deg=210;
 for(i=0;i<7;i++){memset(&f,0,sizeof(f));c.deploy=(SPF_Pose){180,0,values[i]};CK(SPF_Begin(&f,&c,0)==(i==1||i==2||i==3));}
 CK(SPF_MAX_LIFT_PULSES==118800U);printf("depth contract checks=%u failures=%u\n",checks,failures);return failures?1:0;}
