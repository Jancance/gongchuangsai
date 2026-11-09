"""实际升降C边界/停止测试，及实际X角度编码；模拟CAN不发硬件。"""
from pathlib import Path
import hashlib,json,subprocess
R=Path(__file__).resolve().parents[1]
S=R/'tmp/arm_source_single_20261102/project'
E=R/'evidence/p2-arm/2026-11-02/source-pick-single'
H=R/'tmp/arm_source_single_20261102/lift-host'
B=R/'tmp/arm_x_20261101/state-host'
H.mkdir(parents=True,exist_ok=True)
s=(B/'lift.c').read_text();s=s[:s.index('int main(')]
s=s.replace('static int32_t pos;','static int32_t pos;static unsigned protection;')
s=s.replace('r->Data[1]=(enabled?1:0)|(moving?0:2);',
            'r->Data[1]=(enabled?1:0)|(moving?0:2)|(protection?12:0);')
s+=r'''
int main(int argc,char**argv){const char*c=argv[1];(void)argc;
 assert(LIFT_DOWN_MAX_PULSES==105000&&LIFT_UP_MAX_PULSES==233333);
 assert(g_lift_down_pulses==5000);
 if(!strcmp(c,"scaled"))options=132;
 if(!strcmp(c,"wrong_fw"))options=6;
 tick(120);assert(g_lift_state==4&&!moves);
 if(!strcmp(c,"boot")){tick(100);assert(!moves);puts(c);return 0;}
 unsigned pulses=105000;
 if(!strcmp(c,"below_cap"))pulses=104999;
 if(!strcmp(c,"over_cap"))pulses=105001;
 if(!strcmp(c,"zero"))pulses=0;
 if(!strcmp(c,"ordinary"))pulses=5000;
 if(!strcmp(c,"protection"))protection=1;
 if(!strcmp(c,"stop"))moving=1;
 if(!strcmp(c,"fractional_speed"))speed_raw=5;
 g_lift_down_pulses=pulses;g_lift_motion_request=1;tick(80);
 if(!strcmp(c,"over_cap")||!strcmp(c,"zero")){
  assert(g_lift_motion_state==9&&g_lift_motion_result==6&&!moves);
 }else if(!strcmp(c,"wrong_fw")){
  assert(g_lift_motion_state==9&&g_lift_motion_result==4&&!moves);
 }else if(!strcmp(c,"protection")){
  assert(g_lift_motion_state==9&&g_lift_motion_result==5&&!moves&&stops==1);
 }else if(!strcmp(c,"stop")||!strcmp(c,"fractional_speed")){
  assert(moves==1&&g_lift_motion_state==6);
  g_lift_motion_request=3;tick(1);assert(stops==1&&g_lift_motion_state==8);
 }else{
  uint32_t expected;assert(ArmX_AngleUnits(&enc,pulses,&expected));
  assert(moves==1&&g_lift_motion_state==7&&g_lift_motion_result==1&&last_angle==expected);
  if(pulses==105000)assert(last_angle==((options&128)?1181250U:118125U));
 }
 puts(c);return 0;}
'''
(H/'lift-test.c').write_text(s)
cmd=['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(B),'-I'+str(S/'SYSTEM/CAN'),
     '-I'+str(S/'HARDWARE/LIFT'),'-I'+str(S/'HARDWARE/EXTENSION'),
     str(H/'lift-test.c'),str(S/'HARDWARE/LIFT/lift.c'),str(S/'HARDWARE/LIFT/lift_protocol.c'),
     '-o',str(H/'lift-test.exe')]
r=subprocess.run(cmd,capture_output=True,text=True);assert r.returncode==0,r.stderr
cases='boot full scaled below_cap over_cap zero ordinary protection stop fractional_speed wrong_fw'.split()
logs=[]
for case in cases:
    r=subprocess.run([str(H/'lift-test.exe'),case],capture_output=True,text=True)
    logs.append(case+': '+str(r.returncode)+' '+r.stdout+r.stderr)
    if r.returncode:print('\n'.join(logs));raise SystemExit(1)

# 直接抽取当前实际CAN的构帧入口，确保节点5上限确实传入X编码层。
t=(S/'SYSTEM/CAN/can.c').read_bytes().decode('latin1')
a=t.index('uint32_t CAN1_ArmXBuildMove(');b=t.index('\nstatic uint32_t CAN1_ArmXExchange(',a)
function=t[a:b]
transport=r'''
#include <assert.h>
#include <stdint.h>
#include "arm_x.h"
#include "lift_config.h"
#include "extension_config.h"
static ArmXEncoding s_arm_x_encoding[2];
@@FUNCTION@@
int main(void){uint8_t q[15];
 for(unsigned scaled=0;scaled<2;scaled++){
  assert(ArmX_Configure((uint8_t)(4|(scaled?128:0)),16,&s_arm_x_encoding[0]));
  assert(CAN1_ArmXBuildMove(5,0,105000,18464,45055,45055,q));
  uint32_t angle=((uint32_t)q[8]<<24)|((uint32_t)q[9]<<16)|((uint32_t)q[10]<<8)|q[11];
  assert(angle==(scaled?1181250U:118125U)&&q[0]==0xFD&&q[12]==2&&q[14]==0x6B);
  assert(!CAN1_ArmXBuildMove(5,0,105001,18464,45055,45055,q));
  assert(!CAN1_ArmXBuildMove(5,0,0,18464,45055,45055,q));
 }
 return 0;}
'''.replace('@@FUNCTION@@',function)
(H/'x-build-test.c').write_text(transport)
cmd=['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(S/'SYSTEM/CAN'),
     '-I'+str(S/'HARDWARE/LIFT'),'-I'+str(S/'HARDWARE/EXTENSION'),str(H/'x-build-test.c'),
     '-o',str(H/'x-build-test.exe')]
r=subprocess.run(cmd,capture_output=True,text=True);assert r.returncode==0,r.stderr
r=subprocess.run([str(H/'x-build-test.exe')],capture_output=True,text=True);assert r.returncode==0,r.stderr
logs.append('actual CAN X builder both precision options/boundaries: 0')
(E/'lift-check.log').write_text('\n'.join(logs))
files=['USER/arm_sequence.c','USER/arm_sequence.h','HARDWARE/LIFT/lift_config.h','HARDWARE/LIFT/lift.h']
(E/'lift-review.json').write_text(json.dumps(dict(passed=True,hardware_stubs=True,cases=cases,
    actual_can_builder_precision_cases=2,actual_can_builder_boundary_vectors=6,
    source_sha256={n:hashlib.sha256((S/n).read_bytes()).hexdigest() for n in files},
    protected_owning_sources={n:hashlib.sha256((S/n).read_bytes()).hexdigest()
    for n in ['SYSTEM/CAN/can.c','SYSTEM/CAN/arm_x.h','HARDWARE/LIFT/lift.c','HARDWARE/LIFT/lift_protocol.c']}),indent=2))
print(len(cases),'actual lift C scenarios and6 actual CAN X builder vectors passed; no hardware')
