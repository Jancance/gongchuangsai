#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "source_parallel_flow.h"
static unsigned checks;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %d %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t tick,started,polls,stops,read_misses,fatal;
static uint32_t prepare(void *c,uint32_t bit,const SPF_Command *cmd){(void)c;(void)bit;(void)cmd;if(read_misses){read_misses--;return 0;}return 1;}
static uint32_t start(void*c,uint32_t bit,const SPF_Command*cmd){(void)c;(void)cmd;started|=bit;tick+=160;return 1;}
static uint32_t poll(void*c,uint32_t bit){(void)c;(void)bit;polls++;return fatal?2:1;}
static uint32_t stop(void*c,uint32_t bit){(void)c;stops|=bit;return 1;}
static uint32_t clockms(void*c){(void)c;return tick;}
static SPF_Command command(void){SPF_Command c={0};c.sequence=1;c.kind=SPF_CMD_TRANSFER;c.axes_mask=7;c.goal.extension_mm=65;c.goal.lift_pulses=600;c.goal.dm_deg=180;c.dm_deg_s=SPF_DM_DEG_S;c.extension_rpm=SPF_EXT_RPM;c.lift_rpm=SPF_LIFT_RPM;c.stepper_accel_rpm_s=SPF_STEPPER_ACCEL;c.keep_vm_on=1;return c;}
int main(void){
 SPF_Group g={0};SPF_Command c=command();SPF_GroupIO io={0,prepare,start,poll,stop,clockms};unsigned i;
 read_misses=2;CHECK(SPF_GroupBegin(&g,&c,&io,1,90,12000));
 for(i=0;i<12 && g.state!=3;i++)SPF_GroupService(&g);
 CHECK(g.state==3 && !g.fault && started==7 && g.last_start_ms-g.first_start_ms>90);
 CHECK(stops==0); /* start scheduling delay is not a stop condition */
 memset(&g,0,sizeof(g));tick=0;started=0;read_misses=1000;
 CHECK(SPF_GroupBegin(&g,&c,&io,1,90,12000));SPF_GroupService(&g);CHECK(!started && !g.fault);
 tick=12000;SPF_GroupService(&g);CHECK(g.state==4 && g.fault==3);SPF_GroupService(&g);CHECK(g.stop_mask==0 && stops==7);
 memset(&g,0,sizeof(g));tick=0;started=stops=read_misses=0;fatal=1;
 CHECK(SPF_GroupBegin(&g,&c,&io,1,90,12000));
 for(i=0;i<12 && g.state!=4;i++)SPF_GroupService(&g);
 CHECK(g.state==4 && g.fault==7);SPF_GroupService(&g);CHECK(stops==7 && g.stop_mask==0);
 printf("PASS simplified actual parallel group: %u checks; skew logged only, transient read wait, persistent timeout and real fault STOP\n",checks);return 0;
}
