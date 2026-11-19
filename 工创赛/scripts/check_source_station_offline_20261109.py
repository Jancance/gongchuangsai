"""Actual C station/body/pzq vision admission with explicitly simulated I/O."""
from pathlib import Path
import ast,importlib.util,json,subprocess
R=Path(__file__).resolve().parents[1];U=R/'tmp/source_station_offline_20261109/project/CHASSIS/USER'
E=R/'evidence/integration/2026-11-09/source-station-offline'
def build(name,text,extra=()):
 src=E/(name+'.c');exe=E/(name+'.exe');src.write_text(text,encoding='utf-8')
 r=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',
  '-Wno-unused-function','-Wno-unused-variable','-I',str(U),str(src),*map(str,extra),'-lm','-o',str(exe)],capture_output=True,text=True)
 log=r.stdout+r.stderr
 if r.returncode==0:
  r=subprocess.run([str(exe)],capture_output=True,text=True,timeout=15);log+=r.stdout+r.stderr
 (E/(name+'.log')).write_text(log,encoding='utf-8');print(log,flush=True);assert r.returncode==0,name
 return log.strip()

MISSION=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "source_stage2_body_port.h"
#define SOURCE_STATION_SITE_H
#define SOURCE_STATION_ENABLE 1U
#define SOURCE_STATION_POSE_VERSION 77U
#define SOURCE_STATION_PZQ_POSE_VERIFIED 1U
#define VEHICLE_STAGE1_SCAN_FRESH_MS 1000U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define CK(x) do{if(!(x)){fprintf(stderr,"FAIL mission %d: %s\n",__LINE__,#x);exit(1);}}while(0)
typedef struct {unsigned active,cause,result,rotation_stop_mask,session;} ChassisAutoStatus;
static ChassisAutoStatus body={0,0,1,15,12};
static uint32_t g_vehicle_run_id=9,g_vehicle_stage=1,g_vehicle_state=2,g_vehicle_cause;
static uint32_t g_rtos_fault,g_safety_fault,g_can_ctrl_fault_latched,s_motion_session_open,g_vehicle_alignment_state=1,g_stage2_ready;
static uint32_t g_saoma_accept_count,g_saoma_frame_start_ms,g_saoma_frame_end_ms,g_saoma_last_result;
static char SAOMA_Data[16];static uint32_t scanner,profile_valid,align_result,begin_calls,loaded;
volatile uint32_t g_it_auto_state,g_it_auto_vision_mode;
static uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus*s){*s=body;return 1;}
static uint32_t StartupAxes_Holding(void){return 1;}
static uint32_t Stage2Site_ActivateOnce(void){return profile_valid;}
uint32_t SourceStation_MechanicsReady(void){return 1;}
uint32_t Stage2Body_BeginAlignment(uint32_t run,uint32_t seq,uint32_t now){(void)now;CK(run==9 && seq==1);begin_calls++;return 1;}
uint32_t Stage2Body_ReadAlignment(uint32_t run,uint32_t seq,S2_Event*e){(void)run;(void)seq;memset(e,0,sizeof *e);return align_result;}
void Stage2Body_Cancel(void){align_result=2;}
static void SAOMA_EnableReceive(void){scanner=1;}
static void SAOMA_DisableReceive(void){scanner=0;}
static uint32_t FieldMap_TaskCodeValid(const char*c){return strlen(c)==15 && c[3]=='+';}
uint32_t Vision_ReadLoadedBatch(uint32_t run,uint32_t batch,uint32_t colors[3]){(void)colors;return loaded && run==9 && batch==1;}
#include "source_station_mission.inc"
int main(void){char code[16];uint32_t run,batch;
 CK(!SourceStation_Service(1) && !scanner && !SourceStation_PickAllowed(9));
 g_vehicle_stage=2;g_vehicle_state=8;
 CK(SourceStation_Service(10) && g_source_station_state==SS_WAIT_PROFILE && !scanner && !begin_calls);
 profile_valid=1;SourceStation_Service(20);CK(g_source_station_state==SS_ALIGN && begin_calls==1 && !scanner);
 /* A QR captured before alignment must not start anything. */
 g_saoma_accept_count=5;strcpy(SAOMA_Data,"146+123+614+231");g_saoma_last_result=1;
 g_saoma_frame_start_ms=21;g_saoma_frame_end_ms=22;SourceStation_Service(25);CK(!ss_task_valid);
 align_result=1;SourceStation_Service(30);CK(g_source_station_state==SS_WAIT_QR && scanner && SourceStation_HoldsBody());
 SourceStation_Service(31);CK(!ss_task_valid);
 g_saoma_accept_count=6;g_saoma_frame_start_ms=25;SourceStation_Service(40);CK(!ss_task_valid);
 g_saoma_frame_start_ms=41;g_saoma_frame_end_ms=43;SourceStation_Service(45);
 CK(g_source_station_state==SS_READY && !scanner && SourceStation_ReadTask(code,&run,&batch) && run==9 && batch==1);
 CK(SourceStation_PickAllowed(9) && !SourceStation_PickAllowed(10));
 g_it_auto_vision_mode=1;g_it_auto_state=1;SourceStation_Service(50);CK(g_source_station_state==SS_PICKING);
 g_it_auto_state=2;SourceStation_Service(55);CK(g_source_station_state==SS_PICKING && !g_stage2_ready);
 loaded=1;SourceStation_Service(60);CK(g_source_station_state==SS_DONE && g_stage2_ready);
 body.session++;CK(!SourceStation_PickAllowed(9));SourceStation_Service(65);CK(g_source_station_state==SS_FAULT && !ss_task_valid);
 puts("PASS: actual TaskA station: arrival-only align, unconfigured wait, staleQR rejected, post-align manualQR, body session separation, real loaded evidence before completion. SIMULATED I/O.");return 0;
}
'''

def check_body():
 spec=importlib.util.spec_from_file_location('old_body',R/'scripts/check_stage2_body_port_20261107.py')
 mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);fixture=mod.FIXTURE
 fixture=fixture.replace('static uint32_t holding=1;',
  'uint32_t SourceStation_HoldsBody(void){return 0;}\nstatic uint32_t holding=1;')
 fixture='#define SOURCE_PICK_TEACH_TABLE_H\n'+fixture
 fixture=fixture.replace('l=limits();p.calibrated=0;',
  'l=limits();l.route3_profile_verified=0;p.calibrated=0;')
 fixture=fixture.replace('c=command(SPF_CMD_ROUTE_AND_STOW,2);CK(hooks.route_begin(0,&c,200)==1);',
  'c=command(SPF_CMD_ROUTE_AND_STOW,2);CK(hooks.route_begin(0,&c,199)==2);l.route3_profile_verified=1;s2bp.limits.route3_profile_verified=1;CK(hooks.route_begin(0,&c,200)==1);')
 return build('body-port',fixture)

def check_body_isolation():
 old=ast.parse((R/'scripts/check_stage2_body_port_20261107.py').read_text(encoding='utf-8'))
 prefix=next(ast.literal_eval(n.value) for n in ast.walk(old) if isinstance(n,ast.Assign) and
  any(isinstance(t,ast.Name) and t.id=='power_prefix' for t in n.targets))
 prefix+='\nstatic unsigned station_holds;\nstatic uint32_t SourceStation_HoldsBody(void){return station_holds;}\n'
 main=(U/'main.c').read_bytes().decode('latin1').replace('\r\n','\n')
 def extract(sig):
  start=main.index(sig+'\n{');pos=main.index('{',start)+1;depth=1
  while depth:depth+=(main[pos]=='{')-(main[pos]=='}');pos+=1
  return main[start:pos]
 tests=r'''
int main(void){
 power.active=1;status.rotation_stop_mask=15;keeps=1;
 if(App_SerialIdleBlocked(0)!=0)return 1;
 station_holds=1;if(!(App_SerialIdleBlocked(0)&512))return 2;
 status.result=1;status.rotation_stop_mask=15;
 if(!MotionFinish() || !g_vm_test_request)return 3;
 station_holds=0;keeps=0;s_motion_session_open=1;g_vm_test_request=1;
 if(!MotionFinish() || g_vm_test_request)return 4;
 puts("PASS: actual main admission blocks body start during station scan/pick, scoped power hold preserves ordinary finish behavior. SIMULATED STATUS.");return 0;}
'''
 return build('body-isolation',prefix+extract('uint32_t MotionFinish(void)')+'\n'+
              extract('static uint32_t App_SerialIdleBlocked(uint32_t now)')+tests)

VISION=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "source_station.h"
#define SOURCE_STATION_SITE_H
#define SOURCE_STATION_ENABLE 1U
#define SOURCE_STATION_POSE_VERSION 77U
#define SOURCE_STATION_FRAME_MAX_AGE_MS 150U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define VEHICLE_INTEGRATED_ENABLE 0
#define CK(x) do{if(!(x)){fprintf(stderr,"FAIL vision %d: %s\n",__LINE__,#x);exit(1);}}while(0)
#include "source_parallel_flow.h"
static uint32_t now_ms,allowed=1,g_uart6_rx_errors;
static uint32_t xTaskGetTickCount(void){return now_ms;}
static uint8_t USART6_VisionReadByte(uint8_t*b){(void)b;return 0;}
static void USART6_VisionEnableReceive(void){}
static uint8_t USART6_VisionSend(const uint8_t*b,uint16_t n){(void)b;(void)n;return 1;}
static uint32_t SourceObserver_SharedByte(uint8_t b,uint32_t now,uint32_t binary){(void)b;(void)now;(void)binary;return 0;}
static uint32_t SourceObserver_TrySend(uint8_t(*f)(const uint8_t*,uint16_t)){(void)f;return 0;}
uint32_t SourceStation_PickAllowed(uint32_t run){return allowed && run==9;}
static uint32_t g_it_state=2,g_it_sequence,g_it_request,g_it_action,g_it_axes,g_it_rejected,g_it_cause;
static uint32_t g_lift_ruler_request,g_lift_ruler_stop_pending,g_it_lead5_grip_done;
static float g_it_dm_deg,g_it_extension_mm,g_it_carrier_auto_deg,g_it_lift_pulses;
static float g_startup_extension_current_deg,g_extension_home_reference_deg;
static uint32_t writes;
static uint32_t jiazhua(float deg){CK(deg==140 || deg==210);writes++;return 1;}
static uint32_t it_platform(void){return 1;}
static uint32_t it_at_top(void){return 1;}
static uint32_t SingleTeach_ReferencesReady(void){return 1;}
static uint32_t VehicleArm_Busy(void){return 0;}
static uint32_t SourceStage2_IsActive(void){return 0;}
static uint32_t Stage2Fast_IsActive(void){return 0;}
static void it_fail(uint32_t why){g_it_state=9;g_it_cause=why;}
'''

VISION_TEST=r'''
static void material(uint32_t a,uint32_t b,uint32_t c){
 uint8_t frame[6]={0xcc,2,(uint8_t)a,(uint8_t)b,(uint8_t)c,0xdd};
 for(uint32_t i=0;i<6;i++)Vision_Byte(frame[i]);
}
static void completed(uint32_t request,uint32_t now){now_ms=now;g_it_request=0;g_it_action=request;g_it_state=2;g_it_sequence++;it_auto_service(now);}
static SourceStationTarget target(uint32_t instance){
 SourceStationTarget t={9,77,0,0,0,0,1,1,1,500};t.material_cycle=g_vis_mat_cycle;
 t.instance_id=instance;t.captured_ms=now_ms;t.usable_until_ms=now_ms+1000;return t;
}
int main(void){
 SourceStationTarget t;uint32_t colors[3];
 CK(Vision_SetTask("146+123+614+231",1,9));now_ms=100;SourceStation_VisionService(now_ms);
 CK(g_it_auto_vision_mode && g_it_auto_state==1 && g_it_auto_phase==7 && !g_it_request);
 material(4,6,1);t=target(111);t.capture_verified=0;CK(!SourceStation_OfferTarget(&t,now_ms));
 t=target(111);t.run=10;CK(!SourceStation_OfferTarget(&t,now_ms));
 t=target(111);CK(SourceStation_OfferTarget(&t,now_ms));it_auto_service(now_ms+151);CK(!g_it_request);
 /* Task1 is down(red1), not hard-wired left/slot index. */
 now_ms=300;material(4,6,1);t=target(111);CK(SourceStation_OfferTarget(&t,now_ms));
 it_auto_service(now_ms);CK(g_it_request==3 && g_it_dm_deg==180 && g_it_auto_region==2);
 completed(3,400);CK(g_it_request==14 && g_it_extension_mm==0);
 completed(14,500);CK(g_it_request==15);completed(15,600);CK(g_it_auto_phase==6);
 it_auto_service(900);CK(!g_it_request && g_it_auto_item==0); /* no fake placement */
 CK(Vision_ReportGrasp(9,g_vis_pending_seq,1,1,900,900));
 CK(Vision_ReportPlaced(9,g_vis_pending_seq,1,1,1,1,900,900));
 it_auto_service(900);CK(g_it_auto_item==1 && g_it_auto_phase==7);
 now_ms=1000;material(4,6,0);t=target(111);CK(SourceStation_OfferTarget(&t,now_ms));
 it_auto_service(now_ms);CK(!g_it_request); /* repeated instance */
 t=target(222);CK(SourceStation_OfferTarget(&t,now_ms));it_auto_service(now_ms);
 CK(g_it_request==3 && g_it_dm_deg==195 && g_it_auto_region==0);
 completed(3,1100);CK(g_it_extension_mm==156);completed(14,1200);completed(15,1300);
 CK(Vision_ReportGrasp(9,g_vis_pending_seq,1,1,1500,1500));
 CK(Vision_ReportPlaced(9,g_vis_pending_seq,2,1,1,1,1500,1500));it_auto_service(1500);
 CK(g_it_auto_item==2 && g_it_auto_phase==7);
 now_ms=1600;material(0,6,0);t=target(333);CK(SourceStation_OfferTarget(&t,now_ms));it_auto_service(now_ms);
 CK(g_it_dm_deg==165 && g_it_auto_region==1);completed(3,1700);CK(g_it_extension_mm==160);
 completed(14,1800);completed(15,1900);CK(!Vision_ReadLoadedBatch(9,1,colors));
 CK(Vision_ReportGrasp(9,g_vis_pending_seq,1,1,2100,2100));
 CK(Vision_ReportPlaced(9,g_vis_pending_seq,3,1,1,1,2100,2100));it_auto_service(2100);
 CK(g_it_request==3 && g_it_dm_deg==0 && Vision_ReadLoadedBatch(9,1,colors));
 completed(3,2200);CK(g_it_auto_state==2 && g_vis_loaded_mask==7);
 /* An aim may consume the exit budget; no descent on an expired target. */
 g_it_auto_state=0;g_it_request=0;g_it_state=2;g_vis_task_valid=0;
 CK(Vision_SetTask("146+123+614+231",1,9));now_ms=3000;SourceStation_VisionService(now_ms);
 /* Re-enter a fresh synthetic fixture (real service never restarts same run). */
 g_it_auto_vision_mode=1;CK(it_auto_start(now_ms));material(4,6,1);t=target(444);
 t.usable_until_ms=3500;CK(SourceStation_OfferTarget(&t,now_ms));it_auto_service(now_ms);
 CK(g_it_request==3 && g_vis_pending_seq);
 CK(!Vision_ReportGrasp(9,g_vis_pending_seq,1,1,now_ms,now_ms));
 CK(!Vision_ReportPlaced(9,g_vis_pending_seq,1,1,1,1,now_ms,now_ms));
 completed(3,3100);CK(g_it_auto_phase==7 && !g_it_request && !g_vis_pending_seq);
 /* Losing parked station permission stops an active vision chain. */
 g_it_auto_state=1;allowed=0;SourceStation_VisionService(2300);CK(g_it_auto_state==9 && g_it_cause==14);
 puts("PASS: actual UART parser+QR+vision selector+pzq chain: requested colors choose down/left/right, stale/no-provider/repeated-instance reject,200ms+real placement gates, body-motion cancel. SIMULATED VISION/EVIDENCE/MOTION.");return 0;
}
'''

def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed']
 mission_source=(U/'vehicle_mission.inc').read_bytes().decode('latin1')
 begin=mission_source.index('    if(Stage2Body_Service(now))')
 end=mission_source.index('    if(s_vehicle_queue_pending)',begin)
 dispatch=mission_source[begin:end]
 mission=MISSION.replace('int main(void){',
  'uint32_t Stage2Body_Service(uint32_t now){(void)now;align_result=2;return 1;}\n'
  'static void dispatch_actual(uint32_t now){\n'+dispatch+'}\nint main(void){')
 mission=mission.replace(' puts("PASS: actual TaskA station:',
  ' g_source_station_state=SS_ALIGN;g_source_station_reason=0;align_result=0;scanner=1;\n'
  ' dispatch_actual(90);CK(g_source_station_state==SS_FAULT && !scanner);\n'
  ' puts("PASS: actual TaskA station:')
 logs={'mission':build('station-mission',mission),'body':check_body(),'body_isolation':check_body_isolation()}
 source=(U/'integration10_complete.inc').read_bytes().decode('utf-8',errors='replace')
 actual=source[source.index('volatile uint32_t g_src_txn_revision'):source.index('static uint32_t it_platform')]
 logs['vision']=build('station-vision',VISION+actual+'\n#include "integration13_immediate.inc"\n#include "source_station_vision.inc"\n'+VISION_TEST)
 (E/'host-checks.json').write_text(json.dumps({'passed':True,'results':logs,'actual_candidate_C':True,
  'I_O':'simulated devices,frames,physical evidence androute','hardware_access':False},indent=2)+'\n')
if __name__=='__main__':main()
