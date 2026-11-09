"""Full actual-C parser -> mission -> driver binding -> evidence ledger replay.

Only device completion, metric/route hooks and physical evidence are synthetic.
No serial, debugger, firmware write, calibration or motor access.
"""
from pathlib import Path
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/integration12_stage2_vision_20261107/project/CHASSIS/USER'
OUT = ROOT / 'evidence/integration/2026-11-07/stage2-mainline'

PREFIX = r'''
#include "source_stage2_backend.h"
#include "source_stage2_owner.h"
#include "source_stage2_vision_bridge.h"
#include "source_vision_runtime.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
static uint32_t now_ms;
static uint8_t uart_bytes[128];static unsigned uart_head,uart_tail;
volatile uint32_t g_uart6_rx_errors;
static uint32_t xTaskGetTickCount(void){return now_ms;}
static void USART6_VisionEnableReceive(void){}
static uint8_t USART6_VisionReadByte(uint8_t*b){if(uart_tail==uart_head)return 0;*b=uart_bytes[uart_tail++];return 1;}
static uint8_t USART6_VisionSend(const uint8_t*b,uint16_t n){(void)b;(void)n;return 1;}
static uint32_t SourceObserver_SharedByte(uint8_t b,uint32_t now,uint32_t active){(void)b;(void)now;(void)active;return 0;}
static uint32_t SourceObserver_TrySend(uint8_t(*send)(const uint8_t*,uint16_t)){(void)send;return 0;}
static uint32_t g_it_request,g_it_state,g_lift_compare_target_pulses=118800;
static uint32_t g_source_parallel_cause,g_vehicle_arm_cause,g_source_parallel_state;
static uint32_t g_source_parallel_stop_mask,g_vehicle_arm_stop_pending,g_lift_ruler_stop_pending;
static uint32_t owned,busy,arm_calls,tray_calls,grip_calls,route_calls,route_polls;
static uint32_t checks,failures,negative;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL %d t%u\n",__LINE__,now_ms);}}while(0)
static uint32_t it_platform(void){return 1;}
static uint32_t SingleTeach_ReferencesReady(void){return 1;}
static uint32_t VehicleArm_Busy(void){return busy;}
static uint32_t VehicleSourceParallel_Owns(void){return owned;}
static void VehicleSourceParallel_Cancel(void){owned=busy=0;g_source_parallel_stop_mask=0;}
static uint32_t VehicleArm_BeginSourceParallel(const SPF_Command*c,uint32_t verified,uint32_t driving){
 CK(verified==1);if(c->kind==SPF_CMD_ROUTE_AND_STOW)CK(driving==1);
 CK(c->axes_mask>0 && c->axes_mask<=7);arm_calls++;owned=busy=1;g_source_parallel_state=1;
 if(c->kind==SPF_CMD_PICK){
  CK(c->goal.lift_pulses==118800);
  CK(c->goal.dm_deg==(c->item==0?200:c->item==1?190:165));
  CK(c->goal.extension_mm==(c->item==0?80:c->item==1?48:60));
 }
 return 1;}
static uint32_t jiazhua(float deg){CK(deg==140 || deg==210);grip_calls++;return 1;}
static uint32_t CarrierPwm_Write(uint32_t pwm){
 CK(pwm==(tray_calls==0?500:tray_calls==1?1389:2278));tray_calls++;return 1;}
'''

SUFFIX = r'''
#include "source_stage2_backend.inc"
static uint32_t align_begin(void*p,const SPF_Command*c,uint32_t now){(void)p;(void)c;(void)now;return 1;}
static uint32_t align_poll(void*p,const SPF_Command*c,S2_Event*e,uint32_t now){
 (void)p;memset(e,0,sizeof(*e));e->run_id=c->run_id;e->sequence=c->sequence;e->evidence_flags=S2_ALIGNED;e->observed_ms=now;return 1;}
static uint32_t route_begin(void*p,const SPF_Command*c,uint32_t now){
 (void)p;(void)c;(void)now;CK(owned==1 && g_vis_loaded_mask==7 && g_vis_batch_complete);route_calls++;return 1;}
static uint32_t route_poll(void*p,const SPF_Command*c,uint32_t now){(void)p;(void)c;(void)now;return ++route_polls>=3?1:0;}
static uint32_t route_stop(void*p,uint32_t now){(void)p;(void)now;return 1;}
static uint32_t path(void*p,const SPF_Command*c){(void)p;(void)c;return 1;}
static void material(unsigned item){
 uint8_t frame[3][6]={{0xcc,2,4,6,1,0xdd},{0xcc,2,4,6,0,0xdd},{0xcc,2,0,6,0,0xdd}};
 memcpy(uart_bytes,frame[item],6);uart_head=6;uart_tail=0;Vision_Poll();
}
int main(int argc,char**argv){
 SPF_Config cfg;S2_BackendHooks hooks={0,align_begin,align_poll,route_begin,route_poll,route_stop,path};
 S2_ServoTiming timing={700,400};SPF_Command cmd;S2_Event evidence;S2_SourceObservation metadata;uint32_t colors[3];
 (void)argv;negative=argc>1;
 CK(Vision_SetTask("146+123+614+231",1,10));g_vis_taught_mask=7;
 g_vis_region_deg[0]=190;g_vis_region_deg[1]=165;g_vis_region_deg[2]=200;
 g_vis_region_ext_mm[0]=48;g_vis_region_ext_mm[1]=60;g_vis_region_ext_mm[2]=80;
 memset(&cfg,0,sizeof(cfg));cfg.run_id=10;cfg.pose_version=7;
 cfg.live_references_valid=cfg.deploy_sweep_valid=cfg.carrier_height_valid=1;
 cfg.transfer_sweep_valid=cfg.stow_while_driving_valid=1;cfg.transaction_timeout_ms=2000;
 cfg.requested_colors[0]=1;cfg.requested_colors[1]=4;cfg.requested_colors[2]=6;
 cfg.deploy=(SPF_Pose){180,48,0};cfg.transfer=(SPF_Pose){35.758f,0,7200};cfg.place=cfg.transfer;
 cfg.initial=(SPF_Pose){0,0,0};cfg.gripper_open_deg=210;cfg.gripper_close_deg=140;cfg.initial_gripper_deg=145;
 CK(SourceStage2Backend_Start(&cfg,&hooks,&timing,0));
 for(now_ms=10;now_ms<16000;now_ms+=10){
  if(owned){owned=busy=0;g_source_parallel_state=3;}
  if(g_source_stage2_state==SPF_WAIT_TARGET){
   material(g_vis_task_item);memset(&metadata,0,sizeof(metadata));
   metadata.run_id=10;metadata.pose_version=7;metadata.instance_id=100+g_vis_task_item;
   metadata.captured_ms=now_ms;metadata.usable_until_ms=now_ms+3000;
   metadata.capture_verified=metadata.stationary=metadata.sweep_valid=1;metadata.safe_exit_budget_ms=2400;
   CK(SourceStage2Backend_Select(&metadata,now_ms));
  }
  if(SourceStage2Backend_ReadActive(&cmd) && (cmd.kind==SPF_CMD_CLOSE || cmd.kind==SPF_CMD_RELEASE)){
   memset(&evidence,0,sizeof(evidence));evidence.run_id=10;evidence.sequence=cmd.sequence;
   evidence.observed_ms=now_ms;evidence.instance_id=100+g_vis_task_item;
   evidence.color_id=g_vis_task_colors[g_vis_task_item];evidence.slot=g_vis_task_item+1;
   evidence.evidence_flags=cmd.kind==SPF_CMD_CLOSE?(S2_SOURCE_CLEAR|S2_HELD):(S2_RELEASED|S2_PRESENT|S2_STABLE);
   if(negative && cmd.kind==SPF_CMD_RELEASE)evidence.evidence_flags&=~S2_STABLE;
   (void)SourceStage2Backend_Observe(&evidence,now_ms);
  }
  SourceStage2_Service(now_ms);
  if(g_source_stage2_completed || (g_source_stage2_fault && !SourceStage2_IsActive()))break;
 }
 if(!negative){
  CK(g_source_stage2_completed && g_source_stage2_state==SPF_DONE && g_vis_task_item==3);
  CK(Vision_ReadLoadedBatch(10,1,colors) && colors[0]==1 && colors[1]==4 && colors[2]==6);
  CK(g_source_stage2_loaded_mask==7 && route_calls==1 && route_polls==3);
  CK(tray_calls==3 && grip_calls==7 && arm_calls==13);
 }else{
  CK(!g_source_stage2_completed && g_source_stage2_fault && !SourceStage2_IsActive());
  CK(!g_vis_loaded_mask && !route_calls && g_vis_task_item==0);
 }
 printf("stage2 bound replay negative=%u checks=%u failures=%u arm_groups=%u tray=%u route_publications=%u\n",
  negative,checks,failures,arm_calls,tray_calls,route_calls);return failures?1:0;
}
'''


def main():
    src, exe = OUT / 'bound-replay.c', OUT / 'bound-replay.exe'
    text = (STAGE / 'integration10_complete.inc').read_bytes()
    actual = text[text.index(b'volatile uint32_t g_src_txn_revision'):text.index(b'static uint32_t it_platform')]
    src.write_bytes(PREFIX.encode('ascii') + actual + SUFFIX.encode('ascii'))
    command = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
               '-Wno-misleading-indentation', '-I' + str(STAGE), str(src),
               str(STAGE / 'source_stage2_owner.c'), str(STAGE / 'source_stage2_runtime.c'),
               str(STAGE / 'source_stage2_vision_bridge.c'), str(STAGE / 'source_parallel_flow.c'),
               '-lm', '-o', str(exe)]
    built = subprocess.run(command, capture_output=True, text=True)
    (OUT / 'bound-replay-build.log').write_text(built.stdout + built.stderr, encoding='utf-8')
    assert built.returncode == 0, built.stderr
    reports = []
    for args in [[], ['unstable-place']]:
        ran = subprocess.run([str(exe.resolve())] + args, capture_output=True, text=True)
        print(ran.stdout)
        reports.append(ran.stdout.strip())
        assert ran.returncode == 0, ran.stdout + ran.stderr
    (OUT / 'bound-replay-checks.json').write_text(json.dumps({
        'passed': True, 'results': reports, 'actual_C_chain': [
            'parser/task selection', 'autonomous owner', 'SPF coordinator',
            'real motor/servo adapter', 'physical evidence ledger', 'route/stow publication'],
        'mocked': ['devices', 'calibration/profile', 'physical evidence', 'metric/route task hooks'],
        'hardware_access': False, 'real_vision_or_loaded_test': False,
    }, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
