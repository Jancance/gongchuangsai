"""Execute actual node6 parameter helper, active homing state machine and signed gates."""
from pathlib import Path
import json,subprocess,sys,hashlib,struct,datetime
from prepare_stage2_extension_mirror_20261117 import R,BASE,P,E

def function(text,signature):
 start=text.index(signature);pos=text.index('{',start);level=1;end=pos+1
 while level:
  if text[end]=='{':level+=1
  elif text[end]=='}':level-=1
  end+=1
 return text[start:end]

def ctest():
 helper=function((P/'CHASSIS/SYSTEM/CAN/can.c').read_text(encoding='gb18030'),'uint32_t StartupAxis_SetHomeDirection(')
 defs=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "vehicle_parameters.h"
#define APP_MOTOR_OFFLINE_ONLY 0
#define APP_FAULT_CUT_VM_ENABLE 0
static unsigned critical_depth;
#define taskENTER_CRITICAL() (++critical_depth)
#define taskEXIT_CRITICAL() (--critical_depth)
#define CAN_Id_Extended 4
#define CAN_RTR_Data 0
#define CAN_TxStatus_Ok 1
#define CAN_TxStatus_Failed 2
#define CAN_FIFO0 0
#define CAN1 0
#define CAN1_SINGLE_TX_WAIT_MS 20
#define CAN1_SINGLE_REPLY_WAIT_MS 80
#define pdMS_TO_TICKS(x) (x)
typedef uint32_t TickType_t;
typedef struct {uint32_t ExtId,IDE,RTR,DLC;uint8_t Data[8];} CanTxMsg;
typedef CanTxMsg CanRxMsg;
typedef struct {unsigned active;} PowerCommandStatus;
extern volatile uint32_t g_startup_axes_vm_request,g_startup_axes_state;
extern volatile float g_startup_extension_current_deg;
static unsigned checks,tick=1000,ack=2,pending,framecount,reads,corrupt,cancel_frame,home_cancel,home_calls,abort_calls,stop_calls,nonzero,read_fail;
static uint32_t g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
static uint8_t driver[17];static CanTxMsg frames[3];
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL line%u %s\n",__LINE__,#x);exit(1);}}while(0)
static TickType_t xTaskGetTickCount(void){return tick++;}
static void vTaskDelay(unsigned ms){CHECK(!critical_depth);tick+=ms;}
static void Power_ReadCommandStatus(PowerCommandStatus*p){p->active=1;}
static unsigned CAN_MessagePending(unsigned a,unsigned b){(void)a;(void)b;return pending;}
static void StartupHome_Receive(CanRxMsg*r){memset(r,0,sizeof(*r));r->ExtId=0x600;r->IDE=CAN_Id_Extended;r->DLC=3;r->Data[0]=0x4c;r->Data[1]=ack;r->Data[2]=0x6b;pending=0;}
static unsigned CAN_Transmit(unsigned a,CanTxMsg*t){(void)a;CHECK(critical_depth==1);CHECK(framecount<3);frames[framecount++]=*t;
 if(framecount==cancel_frame){g_startup_axes_state=6;g_startup_axes_vm_request=0;}
 if(framecount==3){pending=1;if(ack==2)driver[2]=VEHICLE_EXTENSION_HOME_DIRECTION;if(corrupt<17)driver[corrupt]^=1;}
 return 0;}
static unsigned CAN_TransmitStatus(unsigned a,unsigned b){(void)a;(void)b;return CAN_TxStatus_Ok;}
static void CAN_CancelTransmit(unsigned a,unsigned b){(void)a;(void)b;}
uint32_t StartupAxis_ReadHomeParameters(uint8_t out[17]){reads++;if(read_fail==reads)return 5;memcpy(out,driver,17);return 1;}
void StartupAxes_Cancel(void);
static uint32_t StartupAxis_Exchange(const uint8_t*q,uint8_t n,uint8_t expected,void*out){CanRxMsg*r=out;(void)n;(void)expected;
 memset(r,0,sizeof(*r));r->Data[0]=q[0];r->Data[1]=2;
 if(q[0]==0x3a)r->Data[1]=1;
 if(q[0]==0x3b)r->Data[1]=3;
 if(q[0]==0x35){r->Data[1]=0;r->Data[3]=nonzero;}
 if(q[0]==0x36)r->Data[1]=0;
 if(q[0]==0x9a){home_calls++;if(home_cancel)StartupAxes_Cancel();}
 if(q[0]==0x9c)abort_calls++;
 if(q[0]==0xfe)stop_calls++;
 return 1;}
'''
 actual=(P/'CHASSIS/USER/extension_home_only.inc').read_text(encoding='utf-8')
 suffix=r'''
static void reset(unsigned direction){static const uint8_t p[17]={0x22,0,0,0,30,0,0,0x27,0x10,1,0x2c,3,0x20,0,60,0,0x6b};
 memcpy(driver,p,17);driver[2]=direction;g_startup_axes_state=0;g_startup_axes_vm_request=0;
 g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=0;ack=2;pending=framecount=reads=cancel_frame=home_cancel=home_calls=abort_calls=stop_calls=nonzero=read_fail=0;corrupt=99;tick=1000;eh_started=eh_stop_pending=0;g_eh_first_reason=0;}
static unsigned set(void){uint8_t before[17],after[17];memcpy(before,driver,17);g_startup_axes_state=2;g_startup_axes_vm_request=1;return StartupAxis_SetHomeDirection(before,after);}
static void steps(unsigned n){while(n--)StartupAxes_Service(1,1);}
int main(void){unsigned i,j,r;uint8_t before[17],after[17];
 reset(0);CHECK(set()==1 && framecount==3 && reads==2 && driver[2]==1);
 {const uint8_t expected[3][8]={{0x4c,0xae,0,0,1,0,30,0},{0x4c,0,0x27,0x10,1,0x2c,3,0x20},{0x4c,0,60,0,0x6b}};
  for(i=0;i<3;i++){CHECK(frames[i].ExtId==(0x600|i));CHECK(frames[i].IDE==CAN_Id_Extended && frames[i].RTR==CAN_RTR_Data);CHECK(frames[i].DLC==(i==2?5:8));CHECK(!memcmp(frames[i].Data,expected[i],frames[i].DLC));}}
 reset(1);CHECK(set()==1 && !framecount && reads==2);
 for(i=0;i<17;i++){reset(0);corrupt=i;CHECK(set()==7 && framecount==3);}
 for(i=0;i<4;i++){reset(0);ack=(unsigned[]){0,0xe2,0xee,0x9f}[i];CHECK(set()!=1 && framecount==3 && reads==1);}
 for(i=1;i<=3;i++){reset(0);cancel_frame=i;CHECK(set()==9 && framecount==i && g_startup_axes_state==6);}
 for(i=0;i<7;i++)for(j=0;j<2;j++){reset(0);memcpy(before,driver,17);g_startup_axes_state=i;g_startup_axes_vm_request=j;
  r=StartupAxis_SetHomeDirection(before,after);CHECK((r==1)==(i==2 && j==1));if(i!=2 || !j)CHECK(!framecount);}
 for(i=0;i<3;i++){reset(0);g_startup_axes_state=2;g_startup_axes_vm_request=1;memcpy(before,driver,17);
  if(i==0)g_safety_fault=1;if(i==1)g_rtos_fault=1;if(i==2)g_can_ctrl_fault_latched=1;
  CHECK(StartupAxis_SetHomeDirection(before,after)==9 && !framecount);}
 for(i=1;i<=2;i++){reset(0);read_fail=i;CHECK(set()!=1);}
 /* Actual active startup include: parameter check -> zero-speed -> write -> readback -> 9a. */
 reset(0);CHECK(StartupAxes_Begin());steps(6);CHECK(home_calls==1 && framecount==3 && g_startup_axes_state==3);
 reset(1);CHECK(StartupAxes_Begin());steps(6);CHECK(home_calls==1 && !framecount);
 reset(0);ack=0xe2;CHECK(StartupAxes_Begin());steps(6);CHECK(!home_calls && g_startup_axes_state==5 && g_startup_axes_cause==4);
 reset(0);corrupt=2;CHECK(StartupAxes_Begin());steps(6);CHECK(!home_calls && g_startup_axes_state==5 && g_startup_axes_cause==7);
 reset(0);nonzero=1;CHECK(StartupAxes_Begin());steps(6);CHECK(!home_calls && !framecount && g_startup_axes_state==5 && g_startup_axes_cause==5);
 reset(0);driver[4]=31;CHECK(StartupAxes_Begin());steps(6);CHECK(!home_calls && !framecount && g_startup_axes_cause==7);
 reset(0);cancel_frame=2;CHECK(StartupAxes_Begin());steps(6);CHECK(!home_calls && g_startup_axes_state==6 && framecount==2);
 reset(0);home_cancel=1;CHECK(StartupAxes_Begin());steps(6);CHECK(home_calls==1 && g_startup_axes_state==6 && eh_stop_pending);steps(1);CHECK(abort_calls==1 && stop_calls==1 && !eh_stop_pending);
 reset(0);CHECK(StartupAxes_Begin());steps(6);StartupAxes_Safety(0);steps(1);CHECK(g_startup_axes_state==5 && abort_calls==1 && stop_calls==1);
 printf("PASS actual mirror helper + active homing: %u checks; CAN600/601/602 DLC8/8/5, save0, same-dir skip, full readback, ACK/cancel/fault/stop, no9a on mismatch\n",checks);return 0;}
'''
 path=E/'actual-mirror-home.c';path.write_text(defs+'\n'+helper+'\n'+actual+'\n'+suffix,encoding='utf-8')
 exe=E/'actual-mirror-home.exe'
 result=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Wno-misleading-indentation','-I'+str(P/'USER'),str(path),'-lm','-o',str(exe)],capture_output=True,text=True)
 (E/'actual-mirror-home-build.log').write_text(result.stdout+result.stderr);assert result.returncode==0,result.stderr
 result=subprocess.run([str(exe)],capture_output=True,text=True);(E/'actual-mirror-home.log').write_text(result.stdout+result.stderr);print(result.stdout+result.stderr,flush=True);assert result.returncode==0
 return result.stdout.strip()

def hosttest():
 sys.path.insert(0,str(R/'src'))
 from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard
 from smart_carrier.vision_manipulation.mirror_committed_pick_board import MirrorCommittedPickBoard
 from smart_carrier.vision_manipulation.mirror_committed_pick_board import flash_segments_digest
 keys=('g_low_fault g_it_cause g_it_rejected g_safety_fault g_rtos_fault g_can_ctrl_fault_latched g_chassis_auto_active g_low_state g_it_request g_source_body_step_state g_source_body_step_request g_startup_axes_cause g_lift_ruler_cause g_dm_jog_first_fault').split()
 value={k:0 for k in keys};value.update(g_safety_ready=1,g_it_state=2,g_lift_compare_top_valid=1,g_lift_compare_backoff_valid=1,g_extension_home_reference_valid=1,g_extension_hold_confirmed=1,g_startup_axes_state=4,g_dm_can2_logical_position_deg=180,lift_depth_pulses=600)
 checks=0;K=315.5*360/3200
 for origin in (-5000,0,.7,5000):
  for error in (-71,-70,0,70,71):
   for sign in (-1,1):
    value.update(g_extension_home_reference_deg=origin,g_startup_extension_current_deg=origin+sign*65*K+error)
    assert CommittedPickBoard.pose_ready(value)==(sign==-1);assert MirrorCommittedPickBoard.pose_ready(value)==(sign==1);checks+=2
 for error in (-72,72):
  value['g_startup_extension_current_deg']=value['g_extension_home_reference_deg']+65*K+error
  assert not MirrorCommittedPickBoard.pose_ready(value);checks+=1
 # A generator must still undergo complete original Flash verification.
 class Target:
  calls=0
  def get_state(self):self.calls+=1;return type('State',(),{'name':'RUNNING'})()
  def read_memory_block8(self,address,length):self.calls+=1;assert address==0x08000000;return list(b'abc')
  def read32(self,address):self.calls+=1;return 1
 segments=[(0x08000000,b'abc')];manifest=E/'host-test-manifest.json'
 candidate={'extend_sign':1,'home_direction':1,'images':{'Vehicle_LiftRuler':{'flash_segments_sha256':flash_segments_digest(segments)}}}
 manifest.write_text(json.dumps(candidate));target=Target()
 board=MirrorCommittedPickBoard(target,{'g_low_commit_version':123},(s for s in segments),candidate_manifest=manifest)
 assert board.verified_flash_bytes==3 and target.calls==3;checks+=1
 for change in ('sign','home','bytes'):
  bad=json.loads(json.dumps(candidate));data=segments
  if change=='sign':bad['extend_sign']=-1
  if change=='home':bad['home_direction']=0
  if change=='bytes':data=[(0x08000000,b'abd')]
  manifest.write_text(json.dumps(bad));target=Target()
  try:MirrorCommittedPickBoard(target,{'g_low_commit_version':123},data,candidate_manifest=manifest)
  except ValueError:pass
  else:raise AssertionError(change)
  assert target.calls==0;checks+=1
 manifest.unlink()
 (E/'host-checks.json').write_text(json.dumps({'passed':True,'checks':checks,'legacy_default_sign':-1,'mirror_sign':1,'hardware_access':False},indent=2)+'\n')
 print('PASS host legacy/mirror directional gates:',checks,flush=True)
 return checks

def gatetest():
 parts=[]
 for file,signature in [('pzq_commit_overlap.inc','static uint32_t Low_ExtensionAt('),('pzq_region_batch.inc','static uint32_t Region_AtObserveExtension('),('stage3_cycle.inc','static void S3_ReadActual(')]:
  b=(P/'CHASSIS/USER'/file).read_bytes()
  parts.append(function(b.decode('latin1'),signature))
 pre=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "vehicle_parameters.h"
#define APP_MOTOR_OFFLINE_ONLY 0
static unsigned checks,low_ext_tick;
static float g_startup_extension_current_deg,g_extension_home_reference_deg;
static float g_stage3_actual_yaw,g_stage3_actual_x,g_stage3_actual_depth,g_stage3_actual_z_mm,g_dm_can2_logical_position_deg,g_vehicle_lift_current_deg;
static struct {unsigned top_scale;int top_position_raw;} g_lift_ruler_controller;
typedef struct {float current;} VSP_Axis;
static unsigned VehicleSourceParallel_Owns(void){return 0;}
static unsigned VSP_Position(unsigned a,VSP_Axis*p){(void)a;p->current=g_startup_extension_current_deg;return 1;}
static unsigned xTaskGetTickCount(void){return 100;}
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL line%u %s\n",__LINE__,#x);exit(1);}}while(0)
'''
 post=r'''
int main(void){unsigned i,j;float k=VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f;
 const float origins[]={-5000,0,.7f,5000},mm[]={0,1,30,48,65,70,156,160,170};
 for(i=0;i<4;i++)for(j=0;j<9;j++){
  g_extension_home_reference_deg=origins[i];g_startup_extension_current_deg=origins[i]+mm[j]*k;
  CHECK(Low_ExtensionAt(mm[j]));S3_ReadActual();CHECK(fabsf(g_stage3_actual_x-mm[j])<.001f);
  if(mm[j]>5){g_startup_extension_current_deg=origins[i]-mm[j]*k;CHECK(!Low_ExtensionAt(mm[j]));}
 }
 for(i=0;i<4;i++){
  g_extension_home_reference_deg=origins[i];g_startup_extension_current_deg=origins[i]+65*k;CHECK(Region_AtObserveExtension());
  g_startup_extension_current_deg=origins[i]-65*k;CHECK(!Region_AtObserveExtension());
  g_startup_extension_current_deg=origins[i]+65*k+46;CHECK(!Region_AtObserveExtension());
 }
 printf("PASS actual signed Low/Region/S3 gates: %u checks; origin offsets/full travel/wrong direction\n",checks);return 0;}
'''
 path=E/'actual-mirror-gates.c';path.write_text(pre+'\n'.join(parts)+post,encoding='utf-8');exe=E/'actual-mirror-gates.exe'
 r=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-I'+str(P/'USER'),str(path),'-lm','-o',str(exe)],capture_output=True,text=True)
 (E/'actual-mirror-gates-build.log').write_text(r.stdout+r.stderr);assert not r.returncode,r.stderr
 r=subprocess.run([str(exe)],capture_output=True,text=True);(E/'actual-mirror-gates.log').write_text(r.stdout+r.stderr);print(r.stdout+r.stderr,flush=True);assert not r.returncode
 return r.stdout.strip()

def enqueue_test():
 actual=function((P/'CHASSIS/SYSTEM/CAN/can.c').read_text(encoding='gb18030'),'static uint32_t CAN1_ExchangeSingle(')
 pre=r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define VEHICLE_EXTENSION_HOME_ONLY 1
#define VEHICLE_STARTUP_GROUP_ENABLE 0
#define CAN_Id_Extended 4
#define CAN_RTR_Data 0
#define CAN_TxStatus_Ok 1
#define CAN_TxStatus_Failed 2
#define CAN_TxStatus_NoMailBox 4
#define CAN_FIFO0 0
#define CAN1 0
#define CAN_Mode_Normal 0
#define CAN1_SINGLE_TX_WAIT_MS 20
#define CAN1_SINGLE_REPLY_WAIT_MS 80
#define pdMS_TO_TICKS(x) (x)
typedef uint32_t TickType_t;
typedef struct {uint32_t ExtId,IDE,RTR,DLC;uint8_t Data[8];} CanTxMsg;
typedef CanTxMsg CanRxMsg;
volatile uint32_t g_startup_axes_vm_request,g_startup_axes_state,g_extension_home_capture,g_extension_home_done_notice;
static unsigned s_can1_normal_ready=1,s_can_ctrl_trace_active,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
static unsigned g_can_ctrl_trace_flush_count,g_can_ctrl_trace_phase,g_can_ctrl_trace_tx_ms,s_can_ctrl_trace_tick,g_can_ctrl_trace_rx_count;
static CanRxMsg g_can_ctrl_trace_last_rx;
static unsigned tick,depth,sent,cancel_at_flush,pending;
#define taskENTER_CRITICAL() (++depth)
#define taskEXIT_CRITICAL() (--depth)
static unsigned xTaskGetTickCount(void){return tick++;}
static unsigned CAN1_InitMode(unsigned a){(void)a;return 1;}
static void vTaskDelay(unsigned a){tick+=a;}
static unsigned CAN_MessagePending(unsigned a,unsigned b){(void)a;(void)b;if(cancel_at_flush){cancel_at_flush=0;g_startup_axes_state=6;g_startup_axes_vm_request=0;}return pending;}
static void StartupHome_Receive(CanRxMsg*r){memset(r,0,sizeof(*r));r->IDE=4;r->ExtId=0x600;r->DLC=3;r->Data[0]=0x9a;r->Data[1]=2;r->Data[2]=0x6b;pending=0;}
static unsigned CAN_Transmit(unsigned a,CanTxMsg*t){(void)a;(void)t;if(depth!=1)return 4;sent++;pending=1;return 0;}
static unsigned CAN_TransmitStatus(unsigned a,unsigned b){(void)a;(void)b;return 1;}
static void CAN_CancelTransmit(unsigned a,unsigned b){(void)a;(void)b;}
'''
 post=r'''
int main(void){uint8_t q[]={0x9a,2,0,0x6b};CanRxMsg r;unsigned result;
 g_startup_axes_state=2;g_startup_axes_vm_request=1;cancel_at_flush=1;
 result=CAN1_ExchangeSingle(6,q,4,3,&r);if(result!=9 || sent || depth || g_startup_axes_state!=6)return 1;
 g_startup_axes_state=2;g_startup_axes_vm_request=1;
 result=CAN1_ExchangeSingle(6,q,4,3,&r);if(result!=1 || sent!=1 || depth)return 2;
 puts("PASS actual 9A final enqueue: cancel during preTX flush rejects, active atomic enqueue succeeds; 2 checks");return 0;}
'''
 path=E/'actual-mirror-enqueue.c';path.write_text(pre+actual+post,encoding='utf-8');exe=E/'actual-mirror-enqueue.exe'
 r=subprocess.run(['gcc','-std=c99','-Wall','-Wextra',str(path),'-o',str(exe)],capture_output=True,text=True)
 (E/'actual-mirror-enqueue-build.log').write_text(r.stdout+r.stderr);assert not r.returncode,r.stderr
 r=subprocess.run([str(exe)],capture_output=True,text=True);(E/'actual-mirror-enqueue.log').write_text(r.stdout+r.stderr);print(r.stdout+r.stderr,flush=True);assert not r.returncode
 return r.stdout.strip()

def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed']
 c=ctest();g=gatetest();n=enqueue_test();h=hosttest();(E/'c-checks.json').write_text(json.dumps({'passed':True,'actual_c':c,'actual_gates':g,'actual_enqueue':n,'host_checks':h,'hardware_access':False},indent=2)+'\n')
if __name__=='__main__':main()
