"""Compile the actual diagnostic service, admission gate and strict read whitelist against fake I/O."""
from pathlib import Path
import json,subprocess
R=Path(__file__).resolve().parents[1];P=R/'tmp/pickup_second_diag_20261110/project'
U=P/'CHASSIS/USER';E=R/'evidence/integration/2026-11-10/second-diag'
it=(U/'integration10_complete.inc').read_text(encoding='utf-8')
gate=it[it.index('static void it_extension_diag_service(void)'):it.index('void Integration10_Service(void)')]
prefix=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef APP_MOTOR_OFFLINE_ONLY
#define APP_MOTOR_OFFLINE_ONLY 0
#endif
typedef struct {uint8_t Data[8];} CanRxMsg;
static uint32_t tick=100,calls,last_cmd,io_result=1;
static uint32_t xTaskGetTickCount(void){return tick;}
static uint32_t VehicleArm_Exchange(uint8_t address,const uint8_t *q,uint8_t len,uint8_t rlen,void *out){
 CanRxMsg *r=(CanRxMsg*)out;
 if(address!=6 || len!=2 || q[1]!=0x6b ||
  !((q[0]==0x1a && rlen==4) || ((q[0]==0x33 || q[0]==0x34 || q[0]==0x36 || q[0]==0x37)&&rlen==7) ||
    (q[0]==0x35&&rlen==5) || (q[0]==0x3a&&rlen==3))){fprintf(stderr,"NOT READ ONLY\n");exit(1);}
 calls++;last_cmd=q[0];memset(r,0,sizeof(*r));r->Data[0]=q[0];r->Data[1]=1;r->Data[rlen-1]=0x6b;return io_result;
}
#include "extension_readonly_diag.inc"
#include "extension_diag_read.h"
static uint32_t checks;
#define CK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %u: %s\n",__LINE__,#x);exit(1);}}while(0)
typedef struct {uint32_t active;} PowerCommandStatus;
static uint32_t powered=1,platform=1,arm_busy,owned,source_active,fast_active,dm_busy;
static uint32_t g_it_state=9,g_it_auto_state=9,g_it_request,g_lift_ruler_request,g_lift_ruler_stop_pending;
static uint32_t g_vehicle_arm_stop_pending,g_source_parallel_stop_mask,g_startup_axes_state=4,g_dm_jog_request;
static void Power_ReadCommandStatus(PowerCommandStatus *p){p->active=powered;}
static uint32_t it_platform(void){return platform;}
static uint32_t VehicleArm_Busy(void){return arm_busy;}
static uint32_t VehicleSourceParallel_Owns(void){return owned;}
static uint32_t SourceStage2_IsActive(void){return source_active;}
static uint32_t Stage2Fast_IsActive(void){return fast_active;}
static uint32_t DM_CAN2_JogIsBusy(void){return dm_busy;}
'''
test=r'''
int main(void){
 uint8_t q[3]={0x34,0x6b,0};
 CK(ExtensionDiag_ReadAllowed(6,q,2,7));q[0]=0x37;CK(ExtensionDiag_ReadAllowed(6,q,2,7));
 CK(!ExtensionDiag_ReadAllowed(5,q,2,7));CK(!ExtensionDiag_ReadAllowed(6,q,3,7));
 CK(!ExtensionDiag_ReadAllowed(6,q,2,5));q[0]=0xfd;CK(!ExtensionDiag_ReadAllowed(6,q,2,7));
 q[0]=0x34;q[1]=0;CK(!ExtensionDiag_ReadAllowed(6,q,2,7));CK(!ExtensionDiag_ReadAllowed(6,0,2,7));
 g_ext_diag_request=1;it_extension_diag_service();
#if APP_MOTOR_OFFLINE_ONLY
 CK(g_ext_diag_state==3 && g_ext_diag_rejected==9 && calls==0 && g_ext_diag_cycle==1);
#else
 CK(g_ext_diag_state==1 && g_ext_diag_cycle==1 && g_ext_diag_index==1 && calls==1 && last_cmd==0x1a);
 CK(g_ext_diag_context==1 && g_it_state==9 && g_it_auto_state==9); /* fault state untouched */
 for(uint32_t i=1;i<7;i++){
  uint32_t before=calls;tick+=10;io_result=i==2?5:1;it_extension_diag_service();
  CK(calls==before+1 && g_ext_diag_index==i+1);
  CK(g_ext_diag_tick[i]==tick && g_ext_diag_raw[i][0]==g_ext_diag_cmd[i]);
 }
 CK(g_ext_diag_state==2 && g_ext_diag_result[2]==5); /* missing read is not motion fault */
 uint32_t done_calls=calls;it_extension_diag_service();CK(calls==done_calls);
 /* Every moving/unsafe gate is tested with a new diagnostic request. */
 uint32_t *deny[]={&g_it_request,&g_lift_ruler_request,&g_lift_ruler_stop_pending,
  &g_vehicle_arm_stop_pending,&g_source_parallel_stop_mask,&arm_busy,&owned,&source_active,&fast_active,&g_dm_jog_request,&dm_busy};
 for(uint32_t i=0;i<sizeof(deny)/sizeof(deny[0]);i++){
  *deny[i]=1;g_ext_diag_request=1;uint32_t before=calls;it_extension_diag_service();
  CK(g_ext_diag_state==3 && g_ext_diag_rejected==1 && calls==before);*deny[i]=0;
 }
 uint32_t before=calls;
 powered=0;g_ext_diag_request=1;it_extension_diag_service();CK(g_ext_diag_state==3 && calls==before);powered=1;
 platform=0;g_ext_diag_request=1;it_extension_diag_service();CK(g_ext_diag_state==3 && calls==before);platform=1;
 g_it_state=1;g_ext_diag_request=1;it_extension_diag_service();CK(g_ext_diag_state==3 && calls==before);g_it_state=9;
 g_it_auto_state=1;g_ext_diag_request=1;it_extension_diag_service();CK(g_ext_diag_state==3 && calls==before);g_it_auto_state=9;
 g_startup_axes_state=3;g_ext_diag_request=1;it_extension_diag_service();CK(g_ext_diag_state==3 && calls==before);g_startup_axes_state=4;
 g_ext_diag_request=2;it_extension_diag_service();CK(g_ext_diag_state==3 && g_ext_diag_rejected==2 && calls==before);
 g_it_state=2;g_it_auto_state=2;g_ext_diag_request=1;io_result=1;it_extension_diag_service();
 CK(g_ext_diag_state==1 && g_ext_diag_context==0 && calls==before+1);
 g_lift_ruler_stop_pending=1;it_extension_diag_service();CK(g_ext_diag_state==3 && calls==before+1);
#endif
 printf("PASS %u diagnostic gate/readonly/whitelist checks; offline=%u\n",checks,APP_MOTOR_OFFLINE_ONLY);return 0;
}
'''
for offline in (0,1):
 src=E/f'diag-readonly-{offline}.c';exe=E/f'diag-readonly-{offline}.exe';src.write_text(prefix+gate+test,encoding='utf-8')
 c=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-variable',
  f'-DAPP_MOTOR_OFFLINE_ONLY={offline}','-I',str(U),str(src),'-o',str(exe)],capture_output=True,text=True)
 log=c.stdout+c.stderr
 if not c.returncode:
  c=subprocess.run([str(exe)],capture_output=True,text=True,timeout=15);log+=c.stdout+c.stderr
 (E/f'diag-readonly-{offline}.log').write_text(log,encoding='utf-8');print(log,flush=True);assert c.returncode==0
# The early-return/FAULT path and ordinary tail both invoke the same gated owner service.
assert 'it_auto_service((uint32_t)xTaskGetTickCount());it_extension_diag_service();return;' in it
assert 'it_auto_service((uint32_t)xTaskGetTickCount());\n it_extension_diag_service();' in it
can=(P/'CHASSIS/SYSTEM/CAN/can.c').read_bytes().decode('gb18030',errors='replace')
assert 'if(ExtensionDiag_ReadAllowed(address,q,length,reply_length))' in can
(E/'diagnostics-checks.json').write_text(json.dumps({'passed':True,'actual_gate_and_service':True,
 'strict_readonly_whitelist':True,'one_read_per_owner_call':True,'fault_state_preserved':True,
 'offline_no_queries':True,'hardware_access':False},indent=2)+'\n')
