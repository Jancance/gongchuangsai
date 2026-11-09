"""Actual Task A body-port logic; route/device/metric input are models."""
from pathlib import Path
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/integration12_stage2_vision_20261107/project/CHASSIS/USER'
OUT = ROOT / 'evidence/integration/2026-11-07/stage2-mainline'
FIXTURE = r'''
#include "source_stage2_body_port.h"
#include "source_observer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define VEHICLE_LIFT_RULER_ONLY 0U
#define APP_MOTOR_OFFLINE_ONLY 0U
#define VEHICLE_STAGE_TIMEOUT_MS 120000U
#define VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS 2000U
#define VEHICLE_MIN_ROUTE_CM 1.0f
#define FIELD_BODY_HALF_MM 140.0f
#define MOTION_FORWARD_DISTANCE 100U
typedef struct{float x_mm,y_mm;}FieldPoint;
typedef struct{unsigned kind;float amount,speed;}RouteRow;
typedef struct{unsigned active,session,cause,rotation_stop_mask;}ChassisAutoStatus;
static ChassisAutoStatus backend;
static struct{unsigned state,index;}g_combined_test;
static unsigned g_vehicle_run_id=10,g_vehicle_stage=2,g_vehicle_state=8,g_vehicle_cause;
static unsigned g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched,g_vehicle_stage_mask;
static unsigned g_vehicle_alignment_state;
volatile uint32_t g_source_stage2_completed;
static unsigned s_vehicle_queue_pending,s_vehicle_queue_begin,s_serial_start_permit,s_motion_session_open;
static unsigned g_vehicle_queue_wait_ms,g_vehicle_queue_attempts,g_vehicle_queue_last_mask;
static unsigned s_vehicle_handoff_valid=1,stopped=1,loaded=1,capture_ok=1,queue_ok=1,heading_mask;
static unsigned source_invalidates,metric_ready,queue_calls,route_calls,stop_calls,metric_services;
static FieldPoint s_vehicle_handoff={2060,1200};static RouteRow last_row;
static MetricAlignmentObservation observation;
static uint32_t holding=1;
static uint32_t StartupAxes_Holding(void){return holding;}
uint32_t SourceStage2_IsActive(void){return 0;}
static uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus*s){*s=backend;return 1;}
static uint32_t VehicleMission_Stopped(void){return stopped;}
static uint32_t VehicleMission_CaptureHandoff(void){return capture_ok;}
static uint32_t FieldMap_AxisClear(FieldPoint a,FieldPoint b,float half){(void)half;return a.x_mm>=0 && b.x_mm>=0 && b.x_mm<2400 && b.y_mm>=0 && b.y_mm<2400;}
static uint32_t VehicleMission_Queue(uint32_t stage,const RouteRow*rows,uint32_t n){
 (void)stage;(void)n;last_row=*rows;queue_calls++;if(!queue_ok)return 0;
 stopped=0;backend.active=1;backend.session++;backend.rotation_stop_mask=0;g_combined_test.state=1;return 1;}
static uint32_t VehicleMission_Load(uint32_t stage){route_calls++;if(!queue_ok)return 0;g_vehicle_stage=stage;g_vehicle_state=2;
 stopped=0;backend.active=1;backend.session++;backend.rotation_stop_mask=0;g_combined_test.state=1;g_combined_test.index=0;return 1;}
static uint32_t VehicleMission_PublishQueued(uint32_t stage){(void)stage;s_vehicle_queue_pending=0;return 1;}
static uint32_t App_SerialIdleBlocked(uint32_t now){(void)now;return heading_mask;}
static void MotionStop(void){stop_calls++;}
uint32_t Vision_ReadLoadedBatch(uint32_t run,uint32_t batch,uint32_t colors[3]){(void)colors;return loaded && run==10 && batch==1;}
void SourceObserver_Init(const MetricAlignmentProfile*p){(void)p;metric_ready=0;}
void SourceObserver_Invalidate(void){metric_ready=0;source_invalidates++;}
void SourceObserver_Service(uint32_t now){(void)now;metric_services++;}
uint32_t SourceObserver_Read(uint32_t now,MetricAlignmentObservation*out){(void)now;if(!metric_ready)return MA_NO_REQUEST;*out=observation;return MA_OK;}
#include "source_stage2_body_port.inc"
static unsigned checks,failures;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL %d state%u reason%u\n",__LINE__,s2bp.state,s2bp.reason);}}while(0)
static uint32_t path(void*p,const SPF_Command*c){(void)p;(void)c;return 1;}
static MetricAlignmentProfile profile(void){MetricAlignmentProfile p={1,1,2,3,4,5,200,900,20000,1000};return p;}
static S2_BodyLimits limits(void){S2_BodyLimits l={1,1,2000,500,3,30};return l;}
static void reset(void){
 memset(&s2bp,0,sizeof(s2bp));g_vehicle_run_id=10;g_vehicle_stage=2;g_vehicle_state=8;g_vehicle_cause=0;
 g_source_stage2_completed=0;backend=(ChassisAutoStatus){0,50,0,15};g_combined_test.state=3;g_combined_test.index=0;
 stopped=loaded=capture_ok=queue_ok=1;s_vehicle_queue_pending=0;metric_ready=0;queue_calls=route_calls=stop_calls=0;
 memset(&observation,0,sizeof(observation));}
static SPF_Command command(unsigned kind,unsigned seq){SPF_Command c;memset(&c,0,sizeof(c));c.run_id=10;c.sequence=seq;c.kind=kind;c.route_stage=3;return c;}
int main(void){MetricAlignmentProfile p;S2_BodyLimits l;S2_BackendHooks hooks;SPF_Command c;S2_Event e;unsigned old;
 reset();p=profile();l=limits();p.calibrated=0;CK(!Stage2Body_Configure(&p,&l));p.calibrated=1;
 CK(Stage2Body_Configure(&p,&l));CK(Stage2Body_GetHooks(&hooks,0,path));
 c=command(SPF_CMD_ALIGN,1);CK(hooks.alignment_begin(0,&c,100)==1);CK(Stage2Body_Owns() && Stage2Body_KeepsPower());
 holding=0;CK(!Stage2Body_KeepsPower());holding=1;
 g_rtos_fault=1;CK(!Stage2Body_KeepsPower());g_rtos_fault=0;
 g_safety_fault=1;CK(!Stage2Body_KeepsPower());g_safety_fault=0;
 g_can_ctrl_fault_latched=1;CK(!Stage2Body_KeepsPower());g_can_ctrl_fault_latched=0;
 CK(Stage2Body_Service(110)==1 && s2bp.state==S2BP_OBSERVE);metric_ready=1;CK(Stage2Body_Service(120)==1 && s2bp.state==S2BP_DONE);
 CK(hooks.alignment_poll(0,&c,&e,130)==1 && e.run_id==10 && e.sequence==1 && e.evidence_flags==S2_ALIGNED);
 CK(!Stage2Body_Owns() && !Stage2Body_KeepsPower());
 c=command(SPF_CMD_ROUTE_AND_STOW,2);CK(hooks.route_begin(0,&c,200)==1);CK(hooks.route_begin(0,&c,201)==1);
 CK(Stage2Body_Service(210)==1 && route_calls==1 && s2bp.state==S2BP_ROUTE && g_vehicle_stage_mask==2);
 Stage2Body_Service(220);CK(s2bp.session==backend.session && s2bp.session_segment==0);
 g_combined_test.index=1;backend.session++;Stage2Body_Service(230);CK(s2bp.session==backend.session && s2bp.session_segment==1);
 stopped=1;backend.active=0;backend.rotation_stop_mask=15;g_combined_test.state=3;Stage2Body_Service(240);
 CK(hooks.route_poll(0,&c,250)==1 && Stage2Body_Owns());
 g_source_stage2_completed=1;CK(!Stage2Body_Owns() && Stage2Body_Service(260)==0);
 CK(route_calls==1);

 reset();CK(Stage2Body_Configure(&p,&l));CK(Stage2Body_GetHooks(&hooks,0,path));c=command(SPF_CMD_ALIGN,3);
 hooks.alignment_begin(0,&c,100);Stage2Body_Service(110);observation.dx_milli_mm=15000;metric_ready=1;old=source_invalidates;Stage2Body_Service(120);
 CK(s2bp.state==S2BP_MOVE && queue_calls==1 && last_row.kind==MOTION_FORWARD_DISTANCE+2 && fabsf(last_row.amount-1.5f)<.001f);
 CK(fabsf(last_row.speed-3)<.001f && source_invalidates==old+1 && !metric_ready);
 Stage2Body_Service(130);stopped=1;backend.active=0;backend.rotation_stop_mask=15;g_combined_test.state=3;Stage2Body_Service(140);
 CK(s2bp.state==S2BP_OBSERVE && !metric_ready && hooks.alignment_poll(0,&c,&e,141)==0);
 observation.dx_milli_mm=1000;metric_ready=1;Stage2Body_Service(150);CK(hooks.alignment_poll(0,&c,&e,151)==1);

 reset();CK(Stage2Body_Configure(&p,&l));Stage2Body_GetHooks(&hooks,0,path);c=command(SPF_CMD_ALIGN,4);
 hooks.alignment_begin(0,&c,100);Stage2Body_Service(110);observation.dx_milli_mm=5000;metric_ready=1;Stage2Body_Service(120);
 CK(s2bp.state==S2BP_FAULT && s2bp.reason==S2BP_SMALL_STEP && queue_calls==0);

 reset();CK(Stage2Body_Configure(&p,&l));Stage2Body_GetHooks(&hooks,0,path);c=command(SPF_CMD_ROUTE_AND_STOW,5);
 hooks.route_begin(0,&c,100);Stage2Body_Service(110);Stage2Body_Service(120);backend.session++;Stage2Body_Service(130);
 CK(s2bp.state==S2BP_FAULT && s2bp.reason==S2BP_SESSION);
 CK(!hooks.route_stop(0,140));Stage2Body_Service(150);CK(s2bp.state==S2BP_STOPPING && stop_calls==1);
 backend.active=0;backend.rotation_stop_mask=7;Stage2Body_Service(160);CK(s2bp.state==S2BP_STOPPING);
 backend.rotation_stop_mask=15;Stage2Body_Service(170);CK(hooks.route_stop(0,180)==1 && s2bp.state==S2BP_STOPPED);

 reset();CK(Stage2Body_Configure(&p,&l));Stage2Body_GetHooks(&hooks,0,path);loaded=0;c=command(SPF_CMD_ROUTE_AND_STOW,6);
 hooks.route_begin(0,&c,100);Stage2Body_Service(110);CK(route_calls==0 && s2bp.state==S2BP_FAULT);
 printf("stage2 TaskA body-port checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''


def main():
    assert json.loads((OUT / 'body-port-matlab.json').read_text())['passed']
    src, exe = OUT / 'body-port-host.c', OUT / 'body-port-host.exe'
    src.write_text(FIXTURE, encoding='ascii')
    built = subprocess.run(['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
                            '-Wno-misleading-indentation', '-Wno-unused-function', '-I' + str(STAGE),
                            str(src), '-lm', '-o', str(exe)], capture_output=True, text=True)
    (OUT / 'body-port-host-build.log').write_text(built.stdout + built.stderr, encoding='utf-8')
    assert built.returncode == 0, built.stderr
    ran = subprocess.run([str(exe.resolve())], capture_output=True, text=True)
    (OUT / 'body-port-host-check.log').write_text(ran.stdout + ran.stderr, encoding='utf-8')
    print(ran.stdout)
    assert ran.returncode == 0, ran.stdout + ran.stderr
    # Compile the exact changed main.c functions: unrelated callers must retain
    # their old VM gate, and completion still needs the actual four-wheel stop.
    main_source = (STAGE / 'main.c').read_bytes().decode('latin1').replace('\r\n', '\n')

    def extract(signature):
        start = main_source.index(signature + '\n{')
        opening = main_source.index('{', start)
        depth = 1
        pos = opening + 1
        while depth:
            depth += (main_source[pos] == '{') - (main_source[pos] == '}')
            pos += 1
        return main_source[start:pos]

    power_prefix = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define APP_MOTOR_OFFLINE_ONLY 0
#define APP_TEST_MODE 3
#define HR_IDLE 0
#define MOTION_DONE 2
typedef struct{unsigned active,result,cut_vm,cause,rotation_stop_mask;}ChassisAutoStatus;
typedef struct{unsigned active;}PowerCommandStatus;
static ChassisAutoStatus status;
static PowerCommandStatus power;
static unsigned keeps,source_ok=1,exclusive=1,holding=1;
static unsigned s_motion_session_open=1,s_combined_keep_vm=1,s_combined_end_ms,s_motion_list_submit=1,g_vm_test_request=1;
static unsigned g_motion_plan_valid=1,g_translation_hold_selftest=1,s_serial_proto_ok=1,g_heading_exec_trip,g_can_ctrl_fault_latched;
static struct{unsigned state,request,completed,cause,index;}g_combined_test;
static struct{unsigned phase,request,trip;}g_forward_hold;
static struct{float rate_dps;}s_heading_source;
static uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus*s){*s=status;return 1;}
static void Power_ReadCommandStatus(PowerCommandStatus*p){*p=power;}
static uint32_t Stage2Body_KeepsPower(void){return keeps;}
static uint32_t MotionGetState(void){return MOTION_DONE;}
static uint32_t xTaskGetTickCount(void){return 100;}
static uint32_t StartupAxes_Holding(void){return holding;}
static uint32_t CAN1_EmmSettingsConfirmed(void){return 1;}
static uint32_t App_HeadingRuntimeSource(uint32_t now){(void)now;return source_ok;}
static uint32_t App_HeadingRuntimeExclusive(void){return exclusive;}
'''
    power_tests = r'''
int main(void){unsigned n=0;
#define CK(v) do{n++;if(!(v)){printf("power FAIL %d\n",__LINE__);return 1;}}while(0)
 power.active=1;status.rotation_stop_mask=15;g_vm_test_request=1;
 CK(App_SerialIdleBlocked(0)&4);keeps=1;CK(App_SerialIdleBlocked(0)==0);
 source_ok=0;CK(App_SerialIdleBlocked(0)&32);source_ok=1;
 exclusive=0;CK(App_SerialIdleBlocked(0)&64);exclusive=1;
 status.active=1;CK(App_SerialIdleBlocked(0)&8);status.active=0;
 status.result=1;status.rotation_stop_mask=7;CK(!MotionFinish());
 status.rotation_stop_mask=15;CK(MotionFinish() && g_vm_test_request==1 && !s_motion_session_open);
 s_motion_session_open=1;keeps=0;CK(MotionFinish() && !g_vm_test_request);
 s_motion_session_open=1;g_vm_test_request=1;status.cause=8;CK(!MotionFinish() && g_vm_test_request==1);
 printf("source-scoped power admission checks=%u failures=0\n",n);return 0;}
'''
    power_src, power_exe = OUT / 'body-power-host.c', OUT / 'body-power-host.exe'
    power_src.write_text(power_prefix + extract('uint32_t MotionFinish(void)') + '\n'
                         + extract('static uint32_t App_SerialIdleBlocked(uint32_t now)')
                         + power_tests, encoding='latin1')
    compiled_power = subprocess.run(['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
                                     '-Wno-misleading-indentation', '-Wno-unused-variable',
                                     str(power_src), '-o', str(power_exe)], capture_output=True, text=True)
    assert compiled_power.returncode == 0, compiled_power.stderr
    power_result = subprocess.run([str(power_exe.resolve())], capture_output=True, text=True)
    (OUT / 'body-power-check.log').write_text(power_result.stdout + power_result.stderr, encoding='utf-8')
    assert power_result.returncode == 0, power_result.stdout + power_result.stderr
    print(power_result.stdout)
    (OUT / 'body-port-checks.json').write_text(json.dumps({
        'passed': True, 'actual_TaskA_adapter': True, 'result': ran.stdout.strip(),
        'device_route_metric_methods': 'mocked', 'hardware_access': False,
        'source_isolated_profile_remains_disabled': True,
        'actual_main_power_function_checks': power_result.stdout.strip(),
    }, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
