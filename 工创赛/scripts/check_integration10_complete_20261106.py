"""验证实际新分派、真实并行CAN适配include和升降返回API；硬件/时钟由桩代替。"""
from pathlib import Path
import ast,json,re,subprocess
ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'tmp/integration10_complete_20261106/project'
SRC=STAGE/'CHASSIS/USER'
OUT=ROOT/'evidence/integration/2026-11-06/integration10-complete'

def run(name,unit,extra=()):
    p=OUT/(name+'.c');p.write_text(unit,encoding='utf-8')
    args=['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',
          '-I'+str(OUT/'stubs'),'-I'+str(SRC),str(p),*[str(x) for x in extra],'-lm','-o',str(OUT/(name+'.exe'))]
    r=subprocess.run(args,capture_output=True,encoding='utf-8',errors='replace')
    (OUT/(name+'-build.log')).write_text(r.stdout+r.stderr,encoding='utf-8');assert r.returncode==0,r.stderr
    r=subprocess.run([str(OUT/(name+'.exe'))],capture_output=True,encoding='utf-8')
    (OUT/(name+'.log')).write_text(r.stdout+r.stderr,encoding='utf-8');print(r.stdout,end='');assert r.returncode==0
    return r.stdout.strip()

def main():
    assert json.loads((OUT/'matlab-result.json').read_text(encoding='utf-8'))['passed']
    (OUT/'stubs').mkdir(exist_ok=True)
    (OUT/'stubs/chassis_auto_test.h').write_text('typedef struct {unsigned active,result,cause,rotation_stop_mask;} ChassisAutoStatus;\nint ChassisAuto_ReadStatus(ChassisAutoStatus *);\n')
    # 原样复制实际include，令其从桩目录解析外设头；被测代码字节保持一致。
    (OUT/'stubs/vehicle_source_parallel.inc').write_bytes((SRC/'vehicle_source_parallel.inc').read_bytes())
    result={}
    tree=ast.parse((ROOT/'scripts/check_source_parallel_port_20261106.py').read_text(encoding='utf-8'))
    unit=next(n.value.value for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='unit' for t in n.targets) and isinstance(n.value,ast.Constant))
    marker='#include "vehicle_source_parallel.inc"'
    unit=unit.replace(marker,'''volatile float g_startup_extension_current_deg;
volatile unsigned g_startup_extension_feedback_valid;
static unsigned SingleTeach_ReferencesReady(void){return holding;}
static float SingleTeach_TopDeg(void){return 12.34f;}
'''+marker)
    unit=unit.replace('goal-12195','goal-12207.34f')
    unit=unit.replace('motor[0].current=10;','motor[0].current=10;motor[1].current=12.34f;')
    marker=' printf("SOURCE_PARALLEL_REAL_PORT'
    tests=r'''
 reset();vsp_owned=1;vsp_group.state=1;vsp_group.command.axes_mask=SPF_LIFT;
 {unsigned char enable[5]={0xf3,0xab,1,0,0x6b};unsigned char wrong[15]={0};
  vsp_axis[1].phase=3;CK(VehicleArm_SourceParallelCanAllowed(5,enable,5));
  CK(!VehicleArm_SourceParallelCanAllowed(6,enable,5));
  vsp_axis[1].phase=2;CK(!VehicleArm_SourceParallelCanAllowed(5,enable,5));
  memset(vsp_axis[1].move,0x55,15);vsp_group.started=SPF_LIFT;
  CK(VehicleArm_SourceParallelCanAllowed(5,vsp_axis[1].move,15));
  CK(!VehicleArm_SourceParallelCanAllowed(5,wrong,15));
  vsp_owned=0;CK(!VehicleArm_SourceParallelCanAllowed(5,vsp_axis[1].move,15));}
'''
    assert marker in unit;unit=unit.replace(marker,tests+marker)
    result['port']=run('complete-port',unit,[SRC/'source_parallel_flow.c'])

    inc=(SRC/'integration10_complete.inc').read_text(encoding='utf-8')
    own=set(re.findall(r'g_it_\w+',inc))
    names=(set(re.findall(r'\bg_\w+',inc))-own-{'g_lift_ruler_controller'})|{'g_vm_test_request'}
    floats={'g_vehicle_lift_current_deg','g_startup_extension_current_deg','g_extension_home_reference_deg','g_dm_can2_logical_position_deg'}
    header='''#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "source_parallel_flow.h"
#include "lift_ruler_calibration.h"
LiftRulerController g_lift_ruler_controller;
'''
    header+='\n'.join(('float ' if n in floats else 'unsigned ')+n+';' for n in sorted(names))
    header+=r'''
unsigned tick,owned,starts,cancels,services,closes,opens,carrier,checks,failures;
SPF_Command sent;
#define CK(v) do{checks++;if(!(v)){failures++;printf("DISPATCH FAIL %d %s
",__LINE__,#v);}}while(0)
unsigned xTaskGetTickCount(void){return tick;}
unsigned SingleTeach_ReferencesReady(void){return g_startup_axes_state==4 && g_extension_home_reference_valid && g_extension_hold_confirmed && g_vm_test_request;}
unsigned VehicleSourceParallel_Owns(void){return owned;}
void VehicleSourceParallel_Cancel(void){cancels++;}
void VehicleArm_Service(void){services++;}
unsigned VehicleArm_Busy(void){return owned;}
unsigned DM_CAN2_JogIsBusy(void){return 0;}
unsigned StartupAxes_Begin(void){starts++;g_startup_axes_state=1;return 1;}
void StartupAxes_Cancel(void){cancels++;}
void StartupAxes_Service(unsigned a,unsigned b){(void)a;(void)b;}
unsigned jiazhua(float angle){if(angle==140)closes++;else if(angle==210)opens++;return 1;}
unsigned CarrierPwm_Write(unsigned pulse){carrier=pulse;return 1;}
unsigned VehicleArm_BeginSourceParallel(const SPF_Command*c,unsigned v,unsigned d){(void)d;CK(v==1);sent=*c;owned=1;g_source_parallel_state=1;return 1;}
#include "integration10_complete.inc"
static void req(unsigned v){g_it_request=v;Integration10_Service();}
static void advance(unsigned ms){tick+=ms;g_dm_can2_feedback_tick=tick;Integration10_Service();}
static void pose_start(void){g_dm_can2_read_cycle++;g_dm_can2_feedback_valid=1;g_dm_can2_feedback_tick=tick;Integration10_Service();CK(owned);}
static void pose_done(void){owned=0;g_source_parallel_state=3;advance(20);CK(g_it_state==2);}
int main(void){
 g_safety_ready=g_can_test_result=1;
 req(3);CK(g_it_rejected==4 && !owned);
 req(1);CK(g_it_state==1 && g_it_lift_permit && g_lift_ruler_request==1);
 req(1);CK(g_it_rejected==1);
 g_lift_ruler_request=0;g_lift_ruler_state=4;g_lift_ruler_controller.top_valid=1;
 g_lift_ruler_controller.top_scale=g_lift_ruler_controller.scale=100;
 g_lift_ruler_controller.top_position_raw=g_lift_ruler_controller.position_after_raw=1234;
 g_lift_ruler_samples=1;g_vm_test_request=1;advance(20);CK(g_it_state==2 && !g_it_lift_permit);
 CK(fabsf(g_vehicle_lift_current_deg-12.34f)<.001f);
 g_vehicle_lift_current_deg+=2;req(2);CK(g_it_rejected==5 && !starts);
 g_vehicle_lift_current_deg=12.34f;req(2);CK(starts==1 && g_it_state==1);
 g_startup_axes_state=4;g_extension_home_reference_valid=g_extension_hold_confirmed=1;
 g_extension_home_reference_deg=g_startup_extension_current_deg=30;advance(20);CK(g_it_state==2);
 g_it_axes=2;g_it_dm_deg=180;g_it_extension_mm=48;g_it_lift_pulses=0;
 req(3);CK(g_it_state==1);g_it_extension_mm=99;pose_start();CK(sent.goal.extension_mm==48);
 g_startup_extension_current_deg=30-48*3.6f;pose_done();
 g_it_axes=4;g_it_lift_pulses=108400;g_it_extension_mm=48;g_dm_can2_logical_position_deg=180;
 req(3);pose_start();CK(sent.goal.lift_pulses==108400);g_vehicle_lift_current_deg+=12195;pose_done();
 g_it_axes=2;req(3);CK(g_it_rejected==6);
 req(5);advance(600);CK(closes==1 && g_it_state==2);
 req(4);CK(g_it_state==1 && g_lift_ruler_request==3);
 g_lift_ruler_request=0;g_lift_ruler_samples++;advance(20);CK(g_it_state==2 && fabsf(g_vehicle_lift_current_deg-12.34f)<.001f);
 g_it_axes=1;g_it_dm_deg=35.10794f;req(3);CK(g_it_rejected==6);
 g_it_axes=2;g_it_extension_mm=0;req(3);pose_start();g_startup_extension_current_deg=30;pose_done();
 g_it_axes=1;req(3);pose_start();CK(fabsf(sent.goal.dm_deg-35.10794f)<.001f);g_dm_can2_logical_position_deg=35.10794f;pose_done();
 g_it_axes=4;g_it_lift_pulses=8000;req(3);CK(g_it_rejected==6);
 g_it_place_depth_limit_pulses=16000;req(3);pose_start();CK(sent.goal.lift_pulses==8000);pose_done();
 req(6);advance(600);CK(opens==1 && g_it_state==2);
 req(4);g_lift_ruler_request=0;g_lift_ruler_samples++;advance(20);CK(g_it_state==2);
 req(8);advance(1500);CK(carrier==500 && g_it_state==2);
 g_dm_can2_read_cycle=0;req(7);g_dm_can2_read_cycle=1;advance(20);CK(g_it_state==2);
 g_it_axes=2;g_it_extension_mm=48;g_it_lift_pulses=0;req(3);pose_start();req(9);CK(g_it_state==9 && g_it_cause==2 && cancels);
 req(1);CK(g_it_rejected==1 && g_it_cause==2 && g_vm_test_request==1);
 owned=0;g_it_state=g_it_cause=0;g_it_request=7;g_dm_can2_read_cycle=0;g_dm_can2_feedback_valid=0;
 Integration10_Service();tick+=15001;Integration10_Service();CK(g_it_state==9 && g_it_cause==4);
 printf("COMPLETE_DISPATCH checks=%u failures=%u
",checks,failures);return failures?1:0;
}
'''
    # C字符串里的换行必须作为反斜线n，不能被宿主多行文本展开。
    header=header.replace(' %s\n",',' %s\\n",').replace('failures=%u\n",','failures=%u\\n",')
    result['dispatch']=run('complete-dispatch',header)
    unit=(ROOT/'evidence/integration/2026-11-06/integration10-single-teach/top-host.c').read_text(encoding='utf-8')
    marker=' printf("TOP_START cases='
    tests=r'''
 {LiftRulerController c;reset_case("normal");LiftRuler_Init(&c);motor=-11565;
  c.state=LRC_DONE;c.vm_request=1;c.top_valid=1;c.top_scale=c.scale=100;c.top_position_raw=-1246500;c.operation=1;
  CK(!LiftRuler_StartReturn(&c,now,0) && c.operation==1);
  CK(LiftRuler_StartReturn(&c,now,1) && c.operation==2);
  for(i=0;i<2500;i++){now+=20;LiftRuler_Service(&c,now,1,1,1,bus,&c);if(c.state>=LRC_DONE && !c.stop_pending)break;}
  CK(c.state==LRC_DONE && c.operation==3 && c.top_valid && homes==1 && moves==1);
  CK(c.top_position_raw==-1246500);}
'''
    assert marker in unit;unit=unit.replace(marker,tests+marker)
    result['return_api']=run('complete-return',unit,[SRC/'lift_ruler_calibration.c'])
    adapter=(SRC/'lift_ruler_adapter.c').read_text(encoding='utf-8')
    body=re.search(r'static uint32_t ruler_ready\(void\)\s*\{.*?\n\}',adapter,re.S).group()
    names=sorted(set(re.findall(r'\bg_\w+',body)))
    unit='#include <stdint.h>\n#include <stdio.h>\n'+'\n'.join('uint32_t '+n+';' for n in names)
    unit+='\nuint32_t xTaskGetTickCount(void){return 1000U;}\n'+body+r'''
int main(void){unsigned checks=0,fail=0;
#define C(v) do{checks++;if(!(v))fail++;}while(0)
 g_safety_ready=g_can_test_result=g_it_lift_permit=1;
 C(ruler_ready());g_it_lift_permit=0;C(!ruler_ready());g_it_lift_permit=1;
 g_vehicle_arm_state=1;C(!ruler_ready());g_vehicle_arm_state=2;C(ruler_ready());
 g_startup_axes_state=4;C(!ruler_ready());g_extension_hold_confirmed=1;C(ruler_ready());
 g_dm_jog_state=7;C(!ruler_ready());g_dm_jog_result=g_dm_posvel_holding=1;
 g_dm_can2_feedback_tick=901;C(ruler_ready());g_dm_can2_feedback_tick=900;C(!ruler_ready());
 g_dm_can2_feedback_tick=1000;C(ruler_ready());g_startup_axes_cause=1;C(!ruler_ready());g_startup_axes_cause=0;
 g_safety_fault=1;C(!ruler_ready());g_safety_fault=0;g_can_ctrl_fault_latched=1;C(!ruler_ready());
 printf("ACTUAL_LIFT_READINESS checks=%u failures=%u\n",checks,fail);return fail?1:0;}
'''
    result['lift_readiness']=run('complete-readiness',unit)
    (OUT/'host-result.json').write_text(json.dumps({'passed':True,'results':result,'hardware_accessed':False,'scope':'actual dispatcher/port/return C; fake buses, clock and mechanical plant'},indent=2),encoding='utf-8')

if __name__=='__main__':main()
