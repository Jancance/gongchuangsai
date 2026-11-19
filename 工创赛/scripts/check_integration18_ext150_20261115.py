"""Exercise actual driver-parameter writer and extension poll code on the host."""
from pathlib import Path
import json
import subprocess

ROOT=Path(__file__).resolve().parents[1]
P=ROOT/'tmp/source_top_extension150_20261115/project'
E=ROOT/'evidence/integration/2026-11-15/integration18/ext150-clog3000'
EXPECTED_CORRIDOR_DEG=107.0


def run_fixture(name,source,offline=0):
    file=E/(name+'.c');exe=E/(name+'.exe');file.write_text(source,encoding='utf-8')
    command=['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function',
        '-Wno-misleading-indentation',f'-DAPP_MOTOR_OFFLINE_ONLY={offline}',
        '-I'+str(P/'CHASSIS/USER'),'-I'+str(P/'USER'),str(file),'-lm','-o',str(exe)]
    build=subprocess.run(command,capture_output=True,text=True)
    text=build.stdout+build.stderr
    if build.returncode:
        (E/(name+'.log')).write_text(text,encoding='utf-8');raise RuntimeError(text)
    result=subprocess.run([str(exe)],capture_output=True,text=True,check=True)
    text+=result.stdout+result.stderr;(E/(name+'.log')).write_text(text,encoding='utf-8')
    print(text.strip(),flush=True);return result.stdout.strip()


def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    can=(P/'CHASSIS/SYSTEM/CAN/can.c').read_bytes().decode('gb18030')
    writer=can[can.index('uint32_t CAN1_ApplyExtensionClog3000('):can.index('uint32_t VehicleArm_Exchange(')]
    prefix=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "extension_clog_settings.h"
typedef uint32_t TickType_t;
typedef struct {uint32_t ExtId,IDE,RTR,DLC;uint8_t Data[8];}CanTxMsg;
typedef CanTxMsg CanRxMsg;
#define CAN1 1
#define CAN_FIFO0 0
#define CAN_Id_Extended 4
#define CAN_RTR_Data 0
#define CAN_TxStatus_Ok 1
#define CAN_TxStatus_Failed 2
#define pdMS_TO_TICKS(x) (x)
static unsigned checks;
#define CK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
static uint8_t driver[36],wire[36];
static unsigned ticks,frames,reads,ack_pending,ack_value=2,miss_ack,bad_readback,fail_read,fail_tx;
static const uint8_t original[36]={0x42,0x25,0x18,0,1,1,3,2,0,16,1,0,0,2,0x58,0x0b,0xb8,0x0b,0xb8,5,0x78,5,7,0,1,1,1,0,8,8,0x98,7,0xd0,0,8,0x6b};
static void reset(void){memcpy(driver,original,36);memset(wire,0,36);ticks=frames=reads=ack_pending=miss_ack=bad_readback=fail_read=fail_tx=0;ack_value=2;}
static uint32_t CAN1_ReadExtensionConfiguration(uint8_t out[36]){reads++;if(fail_read==reads)return 5;memcpy(out,driver,36);return 1;}
static unsigned CAN_Transmit(unsigned bus,CanTxMsg *t){unsigned i;CK(bus==1 && t->IDE==CAN_Id_Extended && t->RTR==0);CK(t->ExtId==((6U<<8)|frames) && t->Data[0]==0x48);CK(t->DLC>=2 && t->DLC<=8);if(fail_tx)return 3;wire[0]=0x48;for(i=1;i<t->DLC;i++)wire[frames*7+i]=t->Data[i];frames++;if(frames==5){CK(wire[1]==0xd1 && wire[2]==1 && wire[35]==0x6b);if(ack_value==2){memcpy(driver+3,wire+3,33);if(bad_readback)driver[8]^=1;}ack_pending=!miss_ack;}return 0;}
static unsigned CAN_TransmitStatus(unsigned bus,unsigned mailbox){(void)bus;(void)mailbox;return CAN_TxStatus_Ok;}
static void CAN_CancelTransmit(unsigned bus,unsigned mailbox){(void)bus;(void)mailbox;}
static unsigned CAN_MessagePending(unsigned bus,unsigned fifo){(void)bus;(void)fifo;return ack_pending;}
static void StartupHome_Receive(CanRxMsg *r){memset(r,0,sizeof(*r));r->ExtId=6U<<8;r->IDE=CAN_Id_Extended;r->DLC=3;r->Data[0]=0x48;r->Data[1]=ack_value;r->Data[2]=0x6b;ack_pending=0;}
static TickType_t xTaskGetTickCount(void){return ticks;}
static void vTaskDelay(unsigned ms){ticks+=ms;}
'''
    tests=r'''
int main(void){uint8_t before[36],after[36],frame[36];uint32_t ack=99,r;unsigned i;
 reset();CK(ExtensionClog3000_Frame(driver,frame));CK(frame[0]==0x48 && frame[1]==0xd1 && frame[2]==1 && frame[31]==0x0b && frame[32]==0xb8);
 for(i=3;i<36;i++)if(i!=31 && i!=32)CK(frame[i]==driver[i]);
#if APP_MOTOR_OFFLINE_ONLY
 r=CAN1_ApplyExtensionClog3000(before,after,&ack);CK(r==9 && frames==0 && reads==0);
#else
 r=CAN1_ApplyExtensionClog3000(before,after,&ack);CK(r==1 && ack==2 && frames==5 && reads==2);CK(!memcmp(before,original,36) && ExtensionClog3000_Readback(before,after));
 for(i=0;i<36;i++)if(i!=31 && i!=32)CK(after[i]==original[i]);
 reset();driver[31]=0x0b;driver[32]=0xb8;r=CAN1_ApplyExtensionClog3000(before,after,&ack);CK(r==1 && frames==0 && ack==0 && reads==1);
 reset();driver[26]=0;r=CAN1_ApplyExtensionClog3000(before,after,&ack);CK(r==7 && frames==0 && driver[26]==0);
 reset();driver[2]=0x12;r=CAN1_ApplyExtensionClog3000(before,after,&ack);CK(r==7 && frames==0);
 reset();ack_value=0xe2;r=CAN1_ApplyExtensionClog3000(before,after,&ack);CK(r==6 && ack==0xe2 && !memcmp(driver,original,36));
 reset();miss_ack=1;r=CAN1_ApplyExtensionClog3000(before,after,&ack);CK(r==5 && ack==0 && frames==5 && ExtensionClog3000_Readback(before,after));
 reset();bad_readback=1;r=CAN1_ApplyExtensionClog3000(before,after,&ack);CK(r==6 && ack==2);
 reset();fail_read=1;r=CAN1_ApplyExtensionClog3000(before,after,&ack);CK(r==5 && frames==0);
 reset();fail_tx=1;r=CAN1_ApplyExtensionClog3000(before,after,&ack);CK(r==4 && !memcmp(driver,original,36));
 reset();CK(CAN1_ApplyExtensionClog3000(0,after,&ack)==7 && frames==0);
#endif
 printf("PASS actual config writer: %u checks; offline=%u; only Clog_Ms changes, ACK/readback exact, no motion/clear\n",checks,APP_MOTOR_OFFLINE_ONLY);return 0;}
'''
    results={}
    for offline in [0,1]:results['driver_'+str(offline)]=run_fixture('driver-write-'+str(offline),prefix+writer+tests,offline)
    source=(P/'CHASSIS/USER/vehicle_source_parallel.inc').read_bytes().decode('utf-8')
    axis=source[source.index('typedef struct {uint32_t phase'):source.index('static VSP_Axis')]
    window=source[source.index('static uint32_t VSP_PositionWindow('):source.index('static uint32_t VSP_Start(')]
    poll=source[source.index('static uint32_t VSP_Poll('):source.index('static uint32_t VSP_Stop(')]
    fixture=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "source_parallel_flow.h"
#include "vehicle_parameters.h"
static unsigned checks;
#define CK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
'''+f'#define EXPECTED_CORRIDOR_DEG {EXPECTED_CORRIDOR_DEG:.8f}f\n'+axis+r'''
static VSP_Axis vsp_axis[2];static SPF_Group vsp_group;
static uint32_t tick,flags=5,g_dm_jog_first_fault,g_dm_jog_state,g_dm_jog_result,g_dm_posvel_holding,g_dm_can2_feedback_tick;
static float g_dm_can2_logical_position_deg;static struct{uint8_t Data[8];}s_rx;
static uint32_t xTaskGetTickCount(void){return tick;}
static uint32_t Low_Busy(void){return 1;}
static uint32_t Low_ObserveReturnSettling(void){return 0;}
static uint32_t VSP_Query(uint8_t a,uint8_t c,uint8_t n){(void)a;(void)n;memset(&s_rx,0,sizeof(s_rx));if(c==0x3a)s_rx.Data[1]=(uint8_t)flags;return 1;}
static uint32_t VSP_Position(uint8_t a,VSP_Axis *v){(void)a;v->position_tick=tick;v->position_cycle++;return 1;}
'''+window+poll+r'''
static uint32_t cycle(uint32_t bit){uint32_t a=VSP_Poll(0,bit);if(a)return a;a=VSP_Poll(0,bit);if(a)return a;return VSP_Poll(0,bit);}
int main(void){VSP_Axis *a=&vsp_axis[0];
 CK(SPF_EXT_RPM==150 && SOURCE_EXTENSION_STALL_CONFIRM_MS==3000 && SOURCE_EXTENSION_DONE_DEG==71 && fabsf(SOURCE_EXTENSION_CORRIDOR_DEG-EXPECTED_CORRIDOR_DEG)<.001f && SOURCE_EXTENSION_ZERO_DONE_DEG==17.75f);
 vsp_group.command.kind=SPF_CMD_PICK;vsp_group.command.goal.extension_mm=161.68f;
 a->before=-2307;a->goal=-5738.76f;a->current=-5214.8f;
 tick=1000;CK(cycle(SPF_EXT)==0 && a->stall_active);
 tick=3000;CK(cycle(SPF_EXT)==0);tick=3999;CK(cycle(SPF_EXT)==0);tick=4000;CK(VSP_Poll(0,SPF_EXT)==2);
 memset(a,0,sizeof(*a));flags=13;tick=1;CK(VSP_Poll(0,SPF_EXT)==2);
 memset(a,0,sizeof(*a));flags=3;a->goal=100;a->before=0;a->current=29;CK(cycle(SPF_EXT)==0);CK(cycle(SPF_EXT)==1);
 memset(a,0,sizeof(*a));flags=3;a->goal=100;a->before=0;a->current=28;CK(cycle(SPF_EXT)==0 && !a->samples);CK(cycle(SPF_EXT)==0 && !a->samples);
 a->current=100+EXPECTED_CORRIDOR_DEG-.01f;CK(VSP_PositionWindow(a,0));a->current=100+EXPECTED_CORRIDOR_DEG+.01f;CK(!VSP_PositionWindow(a,0));
 a->before=-2306.8999f;a->goal=-5695.146f;a->current=-5858.1001f;CK(VSP_PositionWindow(a,0)==(EXPECTED_CORRIDOR_DEG>163.0f));
 memset(vsp_axis,0,sizeof(vsp_axis));flags=5;CK(VSP_Poll(0,SPF_LIFT)==2);
 printf("PASS actual VSP poll: %u checks;3s stalls, hard protection immediate, arrival unchanged, configured path boundary checked\n",checks);return 0;}
'''
    results['poll']=run_fixture('actual-vsp-poll',fixture)
    diag=(P/'CHASSIS/USER/low_readonly_diag.inc').read_bytes().decode('gb18030').replace('#include "power.h"','')
    diag_prefix=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef struct{uint32_t active;}PowerCommandStatus;
typedef struct{uint8_t Data[8];}CanRxMsg;
static uint32_t g_it_state,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched,g_chassis_auto_active;
static uint32_t g_lift_ruler_stop_pending,g_vehicle_arm_stop_pending,g_source_parallel_stop_mask;
static uint32_t busy,owned,queries,writes,flags=3,speed,checks;
#define CK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
static void Power_ReadCommandStatus(PowerCommandStatus*p){p->active=1;}
static uint32_t Low_Busy(void){return busy;}
static uint32_t VehicleSourceParallel_Owns(void){return owned;}
static uint32_t VehicleArm_Exchange(uint8_t a,const uint8_t*q,uint8_t n,uint8_t rn,void*out){CanRxMsg*r=out;(void)rn;CK(a==6 && n==2 && q[1]==0x6b);CK(q[0]==0x1a || q[0]==0x3a || q[0]==0x35 || q[0]==0x36);queries++;memset(r,0,sizeof(*r));r->Data[0]=q[0];if(q[0]==0x3a)r->Data[1]=flags;if(q[0]==0x35)r->Data[3]=speed;return 1;}
static uint32_t CAN1_ReadExtensionConfiguration(uint8_t*p){memset(p,0,36);p[0]=0x42;return 1;}
static uint32_t CAN1_ApplyExtensionClog3000(uint8_t*b,uint8_t*a,uint32_t*ack){writes++;memset(b,0,36);memset(a,0,36);*ack=2;return 1;}
'''
    diag_tests=r'''
int main(void){
 Low_ReadonlyDiag();CK(!queries && !writes);
 g_recovery_diag_request=1;Low_ReadonlyDiag();CK(queries==4 && !writes && g_recovery_diag_done==1);
 g_recovery_diag_request=2;Low_ReadonlyDiag();CK(queries==8 && writes==1 && g_recovery_config_result==1 && g_recovery_config_ack==2);
 flags=0;g_recovery_diag_request=2;Low_ReadonlyDiag();CK(writes==2); /* stopped disabled drive needs no enable */
 flags=13;g_recovery_diag_request=2;Low_ReadonlyDiag();CK(writes==2 && g_recovery_config_result==9);
 flags=3;speed=1;g_recovery_diag_request=2;Low_ReadonlyDiag();CK(writes==2);speed=0;
 busy=1;g_recovery_diag_request=2;Low_ReadonlyDiag();CK(writes==2);busy=0;
 owned=1;g_recovery_diag_request=2;Low_ReadonlyDiag();CK(writes==2);owned=0;
 g_source_parallel_stop_mask=2;g_recovery_diag_request=2;Low_ReadonlyDiag();CK(writes==2);
 CK(!g_it_state && !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched);
 printf("PASS actual parameter request service: %u checks; no automatic write, request1 readonly, zero-speed request2 only, faults preserved\n",checks);return 0;}
'''
    results['explicit_request']=run_fixture('actual-config-request',diag_prefix+diag+diag_tests)
    (E/'parameter-and-poll-checks.json').write_text(json.dumps(dict(passed=True,results=results,hardware_access=False,driver_parameters_applied=False),indent=2),encoding='utf-8')


if __name__=='__main__':main()
