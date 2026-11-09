"""Actual adapter/CAN/home-parameter packet assembly on fake CAN hardware."""
from pathlib import Path
import hashlib
import json
import subprocess
from check_lift_speed_compare_20261106 import FIXTURE
from check_lift_ruler_dispatch_20261105 import read, function

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/lift_speed_compare_20261106/project'
OUT = ROOT / 'evidence/integration/2026-11-06/lift-speed-compare'

STUBS = r'''
#include "lift_ruler_adapter.h"
typedef struct {uint8_t Data[8],DLC,IDE,RTR;uint32_t ExtId;} CanRxMsg;
typedef CanRxMsg CanTxMsg;
typedef uint32_t TickType_t;
typedef struct {uint32_t active;} PowerCommandStatus;
volatile uint32_t g_safety_ready,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
volatile uint32_t g_can_test_result,g_can_guard_ready,g_vm_test_request,g_chassis_auto_active;
volatile uint32_t g_vehicle_arm_state,g_startup_axes_state,g_dm_jog_request,g_dm_jog_state;
volatile uint32_t g_startup_axes_vm_request,g_vehicle_arm_stop_pending;
static uint32_t power_active=1,s_can1_normal_ready=1,other_axis,legacy_calls,g_vehicle_key;
static uint8_t pending_frame[15],pending_length;static CanRxMsg queue[4];static unsigned qi,qn;
#define CAN1 1
#define CAN_FIFO0 0
#define CAN_Mode_Normal 0
#define CAN_Id_Extended 1
#define CAN_RTR_Data 0
#define CAN_TxStatus_Ok 1
#define CAN_TxStatus_Failed 2
#define CAN_TxStatus_NoMailBox 4
#define CAN1_SINGLE_TX_WAIT_MS 20U
#define CAN1_SINGLE_REPLY_WAIT_MS 30U
#define portTICK_PERIOD_MS 1U
#define pdMS_TO_TICKS(v) (v)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define VEHICLE_LIFT_RULER_ONLY 1U
#define VEHICLE_EXTENSION_HOME_ONLY 0U
#define VEHICLE_STARTUP_GROUP_ENABLE 1U
#define VEHICLE_DM_STARTUP_ONLY 0U
#define VEHICLE_STARTUP_GROUP_ONLY 0U
#define VEHICLE_INTEGRATED_ENABLE 1U
static uint32_t xTaskGetTickCount(void){return now;}
static void vTaskDelay(uint32_t t){now+=t;}
static void Power_ReadCommandStatus(PowerCommandStatus*p){p->active=power_active;}
static uint32_t CAN1_InitMode(unsigned mode){(void)mode;return 1;}
static uint32_t CAN_MessagePending(unsigned can,unsigned fifo){(void)can;(void)fifo;return qn-qi;}
static void StartupHome_Receive(CanRxMsg*r){*r=queue[qi++];}
static uint32_t CAN_TransmitStatus(unsigned can,unsigned mb){(void)can;(void)mb;return CAN_TxStatus_Ok;}
static void CAN_CancelTransmit(unsigned can,unsigned mb){(void)can;(void)mb;}
static uint32_t CAN1_ExchangeSingle(uint8_t address,const uint8_t*q,uint8_t n,uint8_t rn,CanRxMsg*r){
 if(address!=5){other_axis++;return 0;}
#if APP_MOTOR_OFFLINE_ONLY
 (void)q;(void)n;(void)rn;(void)r;return 9U;
#else
 memset(r,0,sizeof(*r));return bus(&g_lift_ruler_controller,q,n,rn,r->Data);
#endif
}
static uint8_t CAN_Transmit(unsigned can,const CanTxMsg*t){
 uint8_t address=t->ExtId>>8;unsigned packet=t->ExtId&255;(void)can;
 if(address!=5){other_axis++;return 3;}
 if(t->Data[0]==0x22 && t->DLC==2 && t->Data[1]==0x6b){
  uint8_t payload[17];unsigned offset=1,j,n;home_params(payload);qi=0;qn=3;
  for(j=0;j<3;j++){
   memset(&queue[j],0,sizeof(queue[j]));n=17-offset;if(n>7)n=7;
   queue[j].Data[0]=0x22;memcpy(queue[j].Data+1,payload+offset,n);offset+=n;
   queue[j].DLC=n+1;queue[j].ExtId=0x500U+j;queue[j].IDE=1;queue[j].RTR=0;
  }
  if(!strcmp(test,"params_bad_sequence"))queue[1].ExtId=0x503;
  return 0;
 }
 if(!packet){pending_frame[0]=t->Data[0];pending_length=1;}
 memcpy(pending_frame+pending_length,t->Data+1,t->DLC-1);pending_length+=t->DLC-1;
 if(pending_length==15){qi=0;qn=0;memset(&queue[0],0,sizeof(queue[0]));
  qn=bus(&g_lift_ruler_controller,pending_frame,15,3,queue[0].Data);
  queue[0].DLC=3;queue[0].IDE=1;queue[0].RTR=0;queue[0].ExtId=(uint32_t)address<<8;
 }return 0;
}
uint32_t VehicleArm_Exchange(uint8_t,const uint8_t*,uint8_t,uint8_t,void*);
static void StartupAxes_Service(unsigned a,unsigned b){(void)a;(void)b;legacy_calls++;}
static unsigned StartupAxes_Holding(void){legacy_calls++;return 1;}
static void DM_CAN2_ModuleService(unsigned a,unsigned b){(void)a;(void)b;legacy_calls++;}
static unsigned VehicleArm_Busy(void){legacy_calls++;return 0;}
static void VehicleArm_Service(void){legacy_calls++;}
static void App_CombinedTestStep(void){legacy_calls++;}
'''

MAIN = r'''
static void reset_case(const char*name){
 test=name;now=0;moves=homes=stops=aborts=forbidden=enabled=engine=0;motor=0;scale=100;
 other_axis=legacy_calls=qi=qn=pending_length=0;g_lift_ruler_request=g_lift_ruler_rejected=g_vm_test_request=0;
 g_safety_ready=g_can_test_result=g_can_guard_ready=1;g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=0;
 g_chassis_auto_active=g_vehicle_arm_state=g_startup_axes_state=g_dm_jog_request=g_dm_jog_state=0;
 g_startup_axes_vm_request=g_vehicle_arm_stop_pending=g_vehicle_key=0;power_active=1;
 LiftRuler_Init(&g_lift_ruler_controller);
}
static void run_case(const char*name){
 unsigned k,i,cancelled,before_m,before_h;uint32_t base;reset_case(name);
 for(i=0;i<10;i++){dispatch();LiftRuler_DebugSafety();now+=20;}
 CK(!moves && !homes && !g_vm_test_request && !legacy_calls);
 for(k=0;k<3;k++){
  g_vehicle_key=1;mission_key();CK(!g_vehicle_key && g_lift_ruler_request==LRC_REQUEST_START);
  base=now;cancelled=0;
  for(i=0;i<1700;i++){
   now=base+i*20;
   if(!cancelled && ((!strcmp(name,"cancel_home") && homes) ||
      ((!strcmp(name,"cancel_pre") || !strcmp(name,"stop_ack_missing")) && moves))){
    g_vehicle_key=1;mission_key();cancelled=1;
   }
   LiftRuler_DebugSafety();dispatch();
#if APP_MOTOR_OFFLINE_ONLY
   if(i>4)break;
#else
   if(g_lift_ruler_state>=LRC_DONE && !g_lift_ruler_stop_pending)break;
#endif
  }
  CK(!other_axis && !legacy_calls && !forbidden);
#if APP_MOTOR_OFFLINE_ONLY
  CK(!moves && !homes && !stops && !g_vm_test_request && g_lift_ruler_rejected==2);break;
#else
  CK(g_vm_test_request==1);
  if(!strcmp(name,"normal")){
   CK(g_lift_ruler_state==LRC_DONE && g_lift_compare_operation==k+1 && g_lift_ruler_samples==k+1 && g_lift_compare_top_valid);
   CK(moves==k+1 && homes==(k==0?1:k==1?1:2));
   if(k!=1)CK(g_lift_compare_home_seen && g_lift_compare_home_reported);
   before_m=moves;before_h=homes;
   for(i=0;i<10;i++){now+=20;dispatch();LiftRuler_DebugSafety();}
   CK(moves==before_m && homes==before_h && g_vm_test_request==1);
   continue;
  }
  CK(g_lift_ruler_state!=LRC_DONE && !g_lift_compare_top_valid && !g_lift_ruler_samples);
  if(!strcmp(name,"stop_ack_missing"))CK(g_lift_ruler_stop_pending && g_lift_ruler_stop_unconfirmed);
  else CK(!g_lift_ruler_stop_pending && stops>0);
  if(homes)CK(aborts>0);
  break;
#endif
 }
#if !APP_MOTOR_OFFLINE_ONLY
 if(!strcmp(name,"normal")){
  LiftRuler_DebugKey();CK(g_lift_ruler_rejected==3 && !g_lift_ruler_request);
  dispatch();CK(g_vm_test_request==1 && g_lift_compare_operation==3);
 }
#endif
}
int main(void){
 const char*names[]={"normal","wrong_menu","params_bad_sequence","cancel_pre","cancel_home", ...
 "home_ack_lost","stop_ack_missing"};unsigned i;uint8_t en[]={0xf3,0xab,1,0,0x6b},home[]={0x9a,2,0,0x6b};CanRxMsg rx;
 for(i=0;i<sizeof(names)/sizeof(names[0]);i++)run_case(names[i]);
 reset_case("legacy_bypass");g_startup_axes_state=4;g_startup_axes_vm_request=1;
 CK(VehicleArm_Exchange(6,en,5,3,&rx)!=1);CK(VehicleArm_Exchange(5,en,5,3,&rx)!=1);
 CK(VehicleArm_Exchange(5,home,4,3,&rx)!=1);CK(!moves && !homes && !enabled);
 printf("COMPARE_DISPATCH offline=%u scenarios=8 checks=%u failures=%u\n",APP_MOTOR_OFFLINE_ONLY,checks,failures);
 return failures?1:0;
}
'''.replace(', ...', ',')


def main():
    assert json.loads((OUT / 'core-checks.json').read_text(encoding='utf-8'))['passed']
    adapter = '\n'.join(s for s in read(STAGE / 'CHASSIS/USER/lift_ruler_adapter.c').splitlines() if not s.startswith('#include'))
    can_text = read(STAGE / 'CHASSIS/SYSTEM/CAN/can.c')
    can = function(can_text, 'uint32_t VehicleArm_Exchange(')
    helper = function(can_text, 'uint32_t LiftRuler_ReadHomeParameters(')
    helper = '\n#if !APP_MOTOR_OFFLINE_ONLY\n' + helper + '\n#else\nuint32_t LiftRuler_ReadHomeParameters(uint8_t p[17]){(void)p;return 9U;}\n#endif\n'
    main_text = read(STAGE / 'CHASSIS/USER/main.c')
    begin = main_text.index('                if(VEHICLE_LIFT_RULER_ONLY)LiftRuler_DebugService();')
    dispatch = 'static void dispatch(void){\n' + main_text[begin:main_text.index('#else', begin)] + '\n}\n'
    mission = function(read(STAGE / 'CHASSIS/USER/vehicle_mission.inc'), 'static void VehicleMission_Service(void)')
    prefix = mission[:mission.index(' else {')]
    prefix = prefix[prefix.index('    taskENTER_CRITICAL'):]
    mission = 'static void mission_key(void){uint32_t key;\n' + prefix + '\n}\n'
    fixture = FIXTURE[:FIXTURE.index('static void reset_case(')]
    source = OUT / 'compare-dispatch-host.c'
    source.write_text(fixture + STUBS + adapter + '\n' + can + helper + dispatch + mission + MAIN, encoding='utf-8')
    results = []
    for offline in [0, 1]:
        exe = OUT / f'compare-dispatch-{offline}.exe'
        cmd = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Wno-misleading-indentation',
               f'-DAPP_MOTOR_OFFLINE_ONLY={offline}', '-I' + str(STAGE / 'CHASSIS/USER'),
               str(source), str(STAGE / 'CHASSIS/USER/lift_ruler_calibration.c'), '-lm', '-o', str(exe)]
        built = subprocess.run(cmd, capture_output=True, text=True)
        (OUT / f'dispatch-{offline}-build.log').write_text(built.stdout + built.stderr, encoding='utf-8')
        assert built.returncode == 0, built.stderr
        ran = subprocess.run([str(exe)], capture_output=True, text=True)
        (OUT / f'dispatch-{offline}.log').write_text(ran.stdout + ran.stderr, encoding='utf-8')
        print(ran.stdout)
        assert ran.returncode == 0, ran.stdout + ran.stderr
        results.append({'offline': offline, 'passed': True, 'output': ran.stdout.strip()})
    paths = ['CHASSIS/USER/main.c', 'CHASSIS/USER/vehicle_mission.inc', 'CHASSIS/SYSTEM/CAN/can.c',
             'CHASSIS/USER/lift_ruler_adapter.c', 'CHASSIS/USER/lift_ruler_adapter.h',
             'CHASSIS/USER/lift_ruler_calibration.c', 'CHASSIS/USER/lift_ruler_calibration.h']
    report = {'passed': True, 'results': results,
              'scope': 'actual adapter, actual CAN/home17-byte assembly, extracted task branch; fake bus/ticks/plant',
              'sha256': {p: hashlib.sha256((STAGE / p).read_bytes()).hexdigest() for p in paths},
              'firmware_downloaded': False, 'hardware_operated': False}
    (OUT / 'dispatch-checks.json').write_text(json.dumps(report, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
