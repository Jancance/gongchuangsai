"""Exercise staged adapter and actual CAN exchange with simulated transport."""
from pathlib import Path
import hashlib
import json
import subprocess
from check_lift_ruler_candidate_20261105 import UNIT
from finish_lift_ruler_candidate_20261105 import function_end

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/lift_ruler_20261105/project'
OUT = ROOT / 'evidence/integration/2026-11-05/lift-ruler-candidate'


def read(path):
    raw = path.read_bytes()
    try:
        return raw.decode('utf-8')
    except UnicodeDecodeError:
        return raw.decode('gbk')


def function(text, name):
    begin = text.index(name)
    return text[begin:function_end(text, name) + 1]


STUBS = r'''
#include <stdlib.h>
typedef struct {uint8_t Data[8],DLC,IDE,RTR;uint32_t ExtId;} CanRxMsg;
typedef CanRxMsg CanTxMsg;
typedef uint32_t TickType_t;
typedef struct {uint32_t active;} PowerCommandStatus;
volatile uint32_t g_safety_ready,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
volatile uint32_t g_can_test_result,g_can_guard_ready,g_vm_test_request,g_chassis_auto_active;
volatile uint32_t g_vehicle_arm_state,g_startup_axes_state,g_dm_jog_request,g_dm_jog_state;
volatile uint32_t g_startup_axes_vm_request,g_vehicle_arm_stop_pending;
static uint32_t power_active=1,s_can1_normal_ready=1,other_axis,legacy_calls,g_vehicle_key;
static uint8_t pending_frame[15],pending_length;static CanRxMsg queued;static unsigned queued_valid;
#define CAN1 1
#define CAN_FIFO0 0
#define CAN_Mode_Normal 0
#define CAN_Id_Extended 1
#define CAN_RTR_Data 0
#define CAN_TxStatus_Ok 1
#define CAN_TxStatus_Failed 2
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
static uint32_t CAN_MessagePending(unsigned can,unsigned fifo){(void)can;(void)fifo;return queued_valid;}
static void StartupHome_Receive(CanRxMsg*r){*r=queued;queued_valid=0;}
static uint32_t CAN_TransmitStatus(unsigned can,unsigned mb){(void)can;(void)mb;return CAN_TxStatus_Ok;}
static void CAN_CancelTransmit(unsigned can,unsigned mb){(void)can;(void)mb;}
static uint32_t CAN1_ExchangeSingle(uint8_t address,const uint8_t*q,uint8_t n,uint8_t rn,CanRxMsg*r){
 if(address!=5){other_axis++;return 0;}
 memset(r,0,sizeof(*r));return bus(&g_lift_ruler_controller,q,n,rn,r->Data);
}
static uint8_t CAN_Transmit(unsigned can,const CanTxMsg*t){
 uint8_t address=t->ExtId>>8;unsigned packet=t->ExtId&255;(void)can;
 if(address!=5){other_axis++;return 3;}
 if(!packet){pending_frame[0]=t->Data[0];pending_length=1;}
 memcpy(pending_frame+pending_length,t->Data+1,t->DLC-1);pending_length+=t->DLC-1;
 if(pending_length==15){memset(&queued,0,sizeof(queued));
  queued_valid=bus(&g_lift_ruler_controller,pending_frame,15,3,queued.Data);
  queued.DLC=3;queued.IDE=CAN_Id_Extended;queued.RTR=CAN_RTR_Data;queued.ExtId=(uint32_t)address<<8;
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
 test=name;now=0;moves=stops=enable=forbidden=other_axis=legacy_calls=0;delta=speed=0;scale=100;
 queued_valid=pending_length=0;g_lift_ruler_request=g_lift_ruler_rejected=g_vm_test_request=0;
 g_safety_ready=g_can_test_result=g_can_guard_ready=1;g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=0;
 g_chassis_auto_active=g_vehicle_arm_state=g_startup_axes_state=g_dm_jog_request=g_dm_jog_state=0;
 g_startup_axes_vm_request=g_vehicle_arm_stop_pending=g_vehicle_key=0;power_active=1;
 LiftRuler_Init(&g_lift_ruler_controller);
}
int main(void){
 const char*names[]={"normal_scaled","cancel","stop_ack_missing","stop_then_ack","guard_lost"};
 unsigned i,j,cancelled;uint8_t en[]={0xf3,0xab,1,0,0x6b};CanRxMsg rx;
 for(i=0;i<sizeof(names)/sizeof(names[0]);i++){
  reset_case(names[i]);
  for(j=0;j<4;j++){dispatch();LiftRuler_DebugSafety();now+=50;}
  CK(!moves && !enable && !g_vm_test_request && !legacy_calls);
  g_vehicle_key=1;mission_key();CK(!g_vehicle_key && g_lift_ruler_request==LRC_REQUEST_START);
  cancelled=0;
  for(j=0;j<180;j++){
   if(moves && !cancelled && i>0){
    if(i==4)g_safety_ready=0;
    else{g_vehicle_key=1;mission_key();}
    cancelled=1;
   }
   LiftRuler_DebugSafety();dispatch();now+=50;
   if(g_lift_ruler_state>=LRC_DONE && !g_lift_ruler_stop_pending)break;
  }
#if APP_MOTOR_OFFLINE_ONLY
  CK(!moves && !enable && !stops && !g_vm_test_request && g_lift_ruler_rejected==2);
#else
  CK(moves==1 && !legacy_calls && !other_axis && !forbidden && g_vm_test_request==1);
  if(!i)CK(g_lift_ruler_state==LRC_DONE && g_lift_ruler_samples==1);
  else{CK(g_lift_ruler_state>=LRC_CANCELLED && !g_lift_ruler_samples);
   if(i==2)CK(g_lift_ruler_stop_pending && g_lift_ruler_stop_unconfirmed && stops>3);
   else CK(!g_lift_ruler_stop_pending && stops>=1);
  }
  j=moves;g_vehicle_key=1;mission_key();dispatch();CK(moves==j);
#endif
 }
 reset_case("admission");g_startup_axes_state=4;g_startup_axes_vm_request=1;
 CK(VehicleArm_Exchange(6,en,5,3,&rx)!=1);CK(VehicleArm_Exchange(5,en,5,3,&rx)!=1);
 CK(!moves && !enable && !other_axis);
 printf("DISPATCH offline=%u scenarios=6 checks=%u failures=%u\n",APP_MOTOR_OFFLINE_ONLY,checks,failures);
 return failures?1:0;
}
'''


def main():
    proof = json.loads((OUT / 'dispatch-matlab-result.json').read_text(encoding='utf-8'))
    assert proof['passed']
    adapter = read(STAGE / 'CHASSIS/USER/lift_ruler_adapter.c')
    adapter = '\n'.join(s for s in adapter.splitlines() if not s.startswith('#include'))
    can = function(read(STAGE / 'CHASSIS/SYSTEM/CAN/can.c'), 'uint32_t VehicleArm_Exchange(')
    main_c = read(STAGE / 'CHASSIS/USER/main.c')
    begin = main_c.index('                if(VEHICLE_LIFT_RULER_ONLY)LiftRuler_DebugService();')
    end = main_c.index('#else', begin)
    dispatch = 'static void dispatch(void){\n' + main_c[begin:end] + '\n}\n'
    mission = function(read(STAGE / 'CHASSIS/USER/vehicle_mission.inc'), 'static void VehicleMission_Service(void)')
    # The entire ordinary branch is excluded here; Keil builds check it too.
    prefix = mission[:mission.index(' else {')]
    prefix = prefix[prefix.index('    taskENTER_CRITICAL'):]
    mission_key = 'static void mission_key(void){uint32_t key;\n' + prefix + '\n}\n'
    fixture = UNIT[:UNIT.index('static void run(')].replace('#include <math.h>', '#include <math.h>\n#include <stdlib.h>\n#include "lift_ruler_adapter.h"')
    source = OUT / 'dispatch-host.c'
    source.write_text(fixture + STUBS + adapter + '\n' + can + '\n' + dispatch + mission_key + MAIN, encoding='utf-8')
    results = []
    for offline in [0, 1]:
        exe = OUT / f'dispatch-host-{offline}.exe'
        cmd = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra',
               f'-DAPP_MOTOR_OFFLINE_ONLY={offline}', '-I' + str(STAGE / 'CHASSIS/USER'),
               str(source), str(STAGE / 'CHASSIS/USER/lift_ruler_calibration.c'), '-lm', '-o', str(exe)]
        compiled = subprocess.run(cmd, capture_output=True, text=True)
        (OUT / f'dispatch-host-{offline}-build.log').write_text(compiled.stdout + compiled.stderr, encoding='utf-8')
        assert compiled.returncode == 0, compiled.stderr
        ran = subprocess.run([str(exe)], capture_output=True, text=True)
        (OUT / f'dispatch-host-{offline}.log').write_text(ran.stdout + ran.stderr, encoding='utf-8')
        print(ran.stdout)
        assert ran.returncode == 0, ran.stdout + ran.stderr
        results.append({'offline': offline, 'passed': True, 'output': ran.stdout.strip()})
    paths = ['CHASSIS/USER/main.c', 'CHASSIS/USER/vehicle_mission.inc', 'CHASSIS/SYSTEM/CAN/can.c',
             'CHASSIS/USER/lift_ruler_adapter.c', 'CHASSIS/USER/lift_ruler_calibration.c']
    report = {'passed': True, 'results': results, 'matlab_checks': proof['check_count'],
              'scope': 'actual adapter and CAN exchange, extracted mode dispatch, simulated bus/ticks/power; not RTOS or physical timing',
              'sha256': {p: hashlib.sha256((STAGE / p).read_bytes()).hexdigest() for p in paths},
              'firmware_downloaded': False, 'hardware_operated': False}
    (OUT / 'dispatch-checks.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
