"""Mocked peripheral checks of actual concurrent startup controller and VM decision."""
from pathlib import Path
from vehicle_check_paths import firmware_root
import json,subprocess

R=Path(__file__).resolve().parents[1]
S=firmware_root()
O=R/'evidence/integration/2026-11-03/early-scan-startup'
STUB=O/'axis-stubs';STUB.mkdir(exist_ok=True)
for name,text in {
    'FreeRTOS.h':'#include <stdint.h>\ntypedef uint32_t TickType_t;\n',
    'task.h':'uint32_t xTaskGetTickCount(void);\n',
    'power.h':'typedef struct {uint32_t active;} PowerCommandStatus;\nvoid Power_ReadCommandStatus(PowerCommandStatus*);\n',
    'can.h':'typedef struct {uint8_t Data[8];} CanRxMsg;\n',
    'dm_can2.h':'''#define DM_CAN2_POSVEL_SPEED_DEG_S 300.0f
extern volatile uint32_t g_dm_startup_go,g_dm_jog_first_fault;
extern uint32_t g_dm_point_enable,g_dm_jog_request,g_dm_jog_state;
extern uint32_t g_dm_can2_motor_state,g_dm_jog_result,g_dm_posvel_holding;
extern float g_dm_posvel_speed_deg_s;
void DM_CAN2_ModuleService(uint32_t,uint32_t);
uint32_t DM_CAN2_JogIsBusy(void);
''',
}.items():(STUB/name).write_text(text,encoding='utf-8')

unit=r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#define APP_CONFIG_H
#define CHASSIS_AUTO_TEST_H
#define CHASSIS_AUTO_DONE 9U
#define CHASSIS_AUTO_FAULT 10U
typedef struct {uint32_t state,active,result,cause,rotation_stop_mask;} ChassisAutoStatus;
static ChassisAutoStatus backend;
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *s){*s=backend;return 1;}
volatile uint32_t g_vehicle_arm_dm_owned=0,g_vehicle_arm_axis_owned=0;
volatile float g_vehicle_lift_current_deg=0.0f;
volatile uint32_t g_vehicle_lift_feedback_valid=0U;
uint32_t VehicleArm_Busy(void){return 0;}
static uint32_t tick=100,power_on=1,dm_prepared,dm_done,dm_calls,ext_enable,ext_moves,bad_barrier,fail_query;
static float ext_current=-36.0f;
static uint32_t ext_flags=3,ext_speed;
static uint8_t move_data[15];
volatile uint32_t g_dm_startup_go,g_dm_jog_first_fault;
uint32_t g_dm_point_enable,g_dm_jog_request,g_dm_jog_state;
uint32_t g_dm_can2_motor_state,g_dm_jog_result,g_dm_posvel_holding;
float g_dm_posvel_speed_deg_s;
uint32_t xTaskGetTickCount(void){return tick;}
uint32_t DM_CAN2_JogIsBusy(void){return g_dm_jog_state>0 && g_dm_jog_state<7;}
void DM_CAN2_ModuleService(uint32_t ready,uint32_t busy){
 dm_calls++;
 if(g_dm_jog_request==3){g_dm_jog_request=0;g_dm_jog_state=8;g_dm_posvel_holding=0;return;}
 if(g_dm_jog_request==6){g_dm_jog_request=0;g_dm_jog_state=1;}
 if(dm_prepared && !g_dm_startup_go){g_dm_jog_state=3;g_dm_can2_motor_state=1;}
 if(g_dm_startup_go){g_dm_jog_state=dm_done?7:4;g_dm_jog_result=dm_done;g_dm_posvel_holding=dm_done;}
}
#include "startup_axes.c"
void Power_ReadCommandStatus(PowerCommandStatus *p){p->active=power_on;}
uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t length,uint8_t n,void *out){
 CanRxMsg *p=out;uint32_t raw=(uint32_t)(fabsf(ext_current)*10.0f+0.5f);memset(p,0,sizeof(*p));
 if(fail_query)return 5;
 if(q[0]==0x1a){p->Data[2]=0x80;return 1;}
 if(q[0]==0x36){p->Data[1]=ext_current<0;p->Data[2]=raw>>24;p->Data[3]=raw>>16;p->Data[4]=raw>>8;p->Data[5]=raw;return 1;}
 if(q[0]==0x3a){p->Data[1]=ext_flags;return 1;}
 if(q[0]==0x35){p->Data[2]=ext_speed>>8;p->Data[3]=ext_speed;return 1;}
 if(q[0]==0xf3){ext_enable++;p->Data[1]=2;return 1;}
 if(q[0]==0xfd){ext_moves++;memcpy(move_data,q,15);if(!g_dm_startup_go || !dm_prepared || !ext_enable)bad_barrier++;p->Data[1]=2;return 1;}
 if(q[0]==0xfe){p->Data[1]=2;return 1;}
 return 7;
}
uint32_t VehicleArm_Exchange(uint8_t address,const uint8_t *q,uint8_t n,uint8_t reply_n,void *out){return StartupAxis_Exchange(q,n,reply_n,out);}
static uint32_t checks,failed;
#define C(x) do{checks++;if(!(x)){printf("failed line %u check %u\n",__LINE__,checks);failed++;}}while(0)
static void reset(void){
 tick=100;power_on=1;dm_prepared=dm_done=dm_calls=ext_enable=ext_moves=bad_barrier=fail_query=0;
 ext_current=-36;ext_flags=3;ext_speed=0;g_dm_jog_state=g_dm_can2_motor_state=g_dm_posvel_holding=g_dm_jog_result=0;
 g_dm_jog_request=g_dm_startup_go=g_dm_jog_first_fault=0;g_startup_axes_state=g_startup_axes_vm_request=0;
 memset(&backend,0,sizeof(backend));g_startup_extension_reference_valid=1;
}
static void step(void){tick+=20;StartupAxes_Service(1,1);}
static uint32_t power_enabled,vm_request,g_safety_ready,g_motion_auto_stop_cut_vm,g_wheel_test_cut_vm,g_chassis_auto_cut_vm;
static void Power_Enable(void){power_enabled=1;}
static void Power_Disable(void){power_enabled=0;}
static void actual_power_decision(void)
{
__POWER_DECISION__
}
int main(void){
#if APP_MOTOR_OFFLINE_ONLY
 C(!StartupAxes_Begin());C(!g_startup_axes_vm_request);StartupAxes_Service(1,1);C(!ext_enable && !dm_calls);
#else
 reset();StartupAxes_Service(1,1);C(!dm_calls && !ext_enable && !ext_moves && !g_startup_axes_vm_request);
 g_startup_extension_reference_valid=0;C(StartupAxes_Begin());step();step();
 C(g_startup_axes_state==8 && !g_startup_axes_vm_request && !ext_moves && !dm_calls);
 C(fabsf(g_startup_extension_current_deg+36)<0.01f);
 reset();C(StartupAxes_Begin());C(!StartupAxes_Begin());step();step();
 C(g_startup_axes_state==2 && ext_enable==1 && !ext_moves && !g_dm_startup_go);
 C(g_dm_posvel_speed_deg_s==150.0f);
 for(uint32_t i=0;i<4;i++)step();C(!ext_moves && !g_dm_startup_go);
 dm_prepared=1;step();C(g_startup_axes_state==3 && ext_moves==1 && !bad_barrier && g_dm_startup_go);
 C(move_data[0]==0xfd && move_data[1]==0 && move_data[12]==2 && move_data[14]==0x6b);
 C(move_data[6]==4 && move_data[7]==0x24 && move_data[2]==0xaf && move_data[3]==0xff);
 C((((uint32_t)move_data[8]<<24)|((uint32_t)move_data[9]<<16)|((uint32_t)move_data[10]<<8)|move_data[11])==3600);
 for(uint32_t i=0;i<10;i++)step();C(!StartupAxes_Holding());
 ext_current=0;dm_done=1;for(uint32_t i=0;i<40;i++)step();C(StartupAxes_Holding());C(ext_moves==1 && g_dm_posvel_speed_deg_s==300.0f);
 backend=(ChassisAutoStatus){9,0,1,0,15};C(StartupAxes_NormalStopHold());
 g_safety_ready=1;vm_request=0;g_chassis_auto_cut_vm=1;actual_power_decision();C(power_enabled);
 backend.cause=1;actual_power_decision();C(!power_enabled);backend.cause=0;
 g_safety_ready=0;actual_power_decision();C(!power_enabled);g_safety_ready=1;
 g_motion_auto_stop_cut_vm=1;actual_power_decision();C(!power_enabled);g_motion_auto_stop_cut_vm=0;
 StartupAxes_Cancel();C(g_startup_axes_state==6 && !g_startup_axes_vm_request && g_dm_jog_request==3);
 actual_power_decision();C(!power_enabled);
 reset();C(StartupAxes_Begin());fail_query=1;step();C(g_startup_axes_state==5 && !g_startup_axes_vm_request && !ext_moves);
 reset();C(StartupAxes_Begin());tick+=160;StartupAxes_Safety(1);C(g_startup_axes_state==5 && !g_startup_axes_vm_request);
 reset();C(StartupAxes_Begin());StartupAxes_Safety(0);C(g_startup_axes_state==5);
 reset();ext_current=0;C(StartupAxes_Begin());step();step();dm_prepared=1;step();dm_done=1;
 for(uint32_t i=0;i<40;i++)step();C(StartupAxes_Holding() && !ext_moves);
 ext_current=3;for(uint32_t i=0;i<10;i++)step();C(g_startup_axes_state==5 && !g_startup_axes_vm_request);
 reset();C(StartupAxes_Begin());step();step();dm_prepared=1;step();ext_flags=9;for(uint32_t i=0;i<10;i++)step();C(g_startup_axes_state==5);
#endif
#if !APP_MOTOR_OFFLINE_ONLY
 reset();C(StartupAxes_Begin());g_dm_jog_first_fault=12;StartupAxes_Safety(1);C(g_startup_axes_state==5 && !g_startup_axes_vm_request && g_startup_axes_cause==7);
 reset();C(StartupAxes_Begin());backend.state=10;backend.cause=9;StartupAxes_Safety(1);C(g_startup_axes_state==5 && !g_startup_axes_vm_request);
#endif
 printf("startup axes checks=%u failures=%u\n",checks,failed);return failed?1:0;}
'''
safety=(S/'CHASSIS/SYSTEM/SAFETY/safety.c').read_bytes().decode('gbk')
lo=safety.index('    if ((g_safety_ready == 1U) && (vm_request == 1U ||')
hi=safety.index('    /*',lo)
unit=unit.replace('__POWER_DECISION__',safety[lo:hi])
(O/'axes-unit.c').write_text(unit,encoding='utf-8')
reports=[]
for offline in [0,1]:
    exe=O/f'axes-unit-{offline}.exe'
    cmd=['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-DSTARTUP_EXTENSION_HOME_ENABLE=0','-DSTARTUP_EXTENSION_HOME_TEST_ONLY=0',f'-DAPP_MOTOR_OFFLINE_ONLY={offline}',
        '-I'+str(STUB),'-I'+str(S/'CHASSIS/USER'),str(O/'axes-unit.c'),'-lm','-o',str(exe)]
    p=subprocess.run(cmd,capture_output=True,text=True,errors='replace');assert p.returncode==0,p.stderr[-5000:]
    p=subprocess.run([str(exe)],capture_output=True,text=True);print(offline,p.stdout,flush=True);assert p.returncode==0,p.stdout
    reports.append(dict(offline=offline,output=p.stdout,passed=True))
(O/'axes-check.json').write_text(json.dumps(dict(actual_c=True,peripherals_mocked=True,
    hardware_accessed=False,cases=reports),indent=2),encoding='utf-8')
