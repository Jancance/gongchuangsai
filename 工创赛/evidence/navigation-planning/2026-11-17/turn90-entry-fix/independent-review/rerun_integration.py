from pathlib import Path
import ctypes, hashlib, importlib.util, json, re, struct, subprocess, sys, xml.etree.ElementTree as ET
R=Path.cwd(); E=R/'工创赛/evidence/navigation-planning/2026-11-17/turn90-entry-fix'; U=R/'3.stm32f407工程/CHASSIS/USER'; B=Path('D:/Keil_v5/ARM/arm5_compiler/Bin'); variant='before' if '--before' in sys.argv else ('before-motion' if '--before-motion' in sys.argv else 'fixed');T=E/'independent-review'/('actual-c' if variant=='fixed' else 'actual-c-'+variant);T.mkdir(exist_ok=True)
def extract(source, signature):
    start=source.index(signature); opening=source.index('{',start);depth=1;end=opening+1
    while depth:
        if source[end]=='{':depth+=1
        elif source[end]=='}':depth-=1
        end+=1
    return source[start:end]
motion=((E/'app_motion_runtime.c.before') if ('--before' in sys.argv or '--before-motion' in sys.argv) else (U/'app_motion_runtime.c')).read_bytes().decode('gbk');serial=((E/'app_serial_runtime.c.before') if '--before' in sys.argv else (U/'app_serial_runtime.c')).read_bytes().decode('gbk');heading=(U/'app_heading_runtime.c').read_bytes().decode('gbk')
load=extract(motion,'static uint32_t App_CombinedLoadAction(')
turn=extract(motion,'uint32_t TurnStart(').replace('uint32_t TurnStart(','static uint32_t RealTurnStart(',1)
perimeter=extract(motion,'static uint32_t App_PerimeterTurnStart(').replace('static uint32_t App_PerimeterTurnStart(','static uint32_t RealPerimeterTurnStart(',1)
telemetry=serial[serial.index('    values[56] = (int32_t)(('):serial.index('    if (g_motion_plan_valid != 1U)')]
start_expr=heading[heading.index('            prepared_ok=(*inputs->route_turn_active) ?'):heading.index('            if(prepared_ok)prepared_ok=ChassisAuto_PrepareHeading')]
milli=extract(serial,'static int32_t App_SerialMilli(')
prefix=r"""
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"
#include "app_entry.h"
#include "app_motion_runtime.h"
#include "app_heading_runtime.h"
#include "heading_source.h"
#include "route_heading_reference.h"
#include "motion_control.h"
#undef taskENTER_CRITICAL
#undef taskEXIT_CRITICAL
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static MotionAction s_motion_actions[8];
static uint32_t s_motion_action_count;
#define MOTION_ACTION_COUNT s_motion_action_count
volatile uint32_t g_debug_plan_id;
volatile CombinedTestDebug g_combined_test;
volatile uint32_t g_heading_exec_trip,g_heading_exec_phase,g_heading_exec_state,g_heading_exec_request,g_motion_fault_diag_valid;
volatile float g_heading_exec_relative_deg;
volatile HeadingStopTrace g_heading_stop_trace;
static uint32_t s_motion_list_submit,s_combined_turn,s_motion_turn_max_rpm,s_combined_base_rpm,s_combined_run_ms,s_motion_has_action;
static float s_combined_distance_mm,s_combined_reference_mm_s;
static uint32_t mode,turn_calls,perimeter_calls,move_calls,reset_calls;
static RouteHeadingReference s_route_heading_reference;
static float s_route_heading_target,source_yaw;
static uint32_t s_route_heading_turn_active;
volatile float g_route_heading_targets[8],g_route_heading_start_yaw[8];
static uint32_t HWT101_TimingBegin(void){return 1;}
static void HWT101_TimingEnd(uint32_t kind,uint32_t tick){(void)kind;(void)tick;}
static void HWT101_DiagnosticArm(void){}
void AppHeadingRuntime_ResetSession(void){reset_calls++;}
uint32_t App_HeadingSourceRead(AppHeadingSource *heading){memset(heading,0,sizeof(*heading));heading->yaw_deg=source_yaw;return 1;}
static uint32_t App_MotionAccept(void){return 1;}
static uint32_t App_MoveSubmit(uint32_t d,uint32_t rpm,uint32_t ms,float mm,float ref){move_calls++;return 1;}
uint32_t MoveDistanceStart(uint32_t d,uint32_t rpm,float cm){move_calls++;return 1;}
uint32_t MoveStart(uint32_t d,uint32_t rpm,uint32_t ms){move_calls++;return 1;}
float MotionRpmToCmPerSec(float rpm){return rpm*(3.14159265358979323846f*77.0f/600.0f);}
static AppMotionObservation live;
const AppMotionObservation *AppMotion_Live(void){return &live;}
"""
wrappers=r"""
uint32_t TurnStart(uint32_t d,float a,uint32_t rpm){turn_calls++;return RealTurnStart(d,a,rpm);}
static uint32_t App_PerimeterTurnStart(uint32_t d,float a,uint32_t rpm){perimeter_calls++;return RealPerimeterTurnStart(d,a,rpm);}
"""
tele=r"""
static void make_telemetry(int32_t *values){
 ChassisAutoStatus status={0};
 memset(values,0,64*sizeof(*values));
 values[38]=(g_combined_test.state==0U)?(s_motion_actions[0].kind==MOTION_TURN?15U:s_motion_actions[0].base_rpm):(s_combined_turn?15U:s_combined_base_rpm);
 values[39]=130;
"""+telemetry+r"""
}
static void reset_case(uint32_t id,float yaw){
 memset(s_motion_actions,0,sizeof(s_motion_actions));memset((void*)&g_combined_test,0,sizeof(g_combined_test));
 s_motion_action_count=0;g_debug_plan_id=id;s_combined_turn=mode=s_motion_has_action=s_motion_list_submit=0;
 s_combined_distance_mm=s_combined_reference_mm_s=0;s_combined_base_rpm=s_combined_run_ms=0;
 g_heading_exec_trip=g_heading_exec_phase=g_heading_exec_state=g_heading_exec_request=0;
 s_motion_turn_max_rpm=5;turn_calls=perimeter_calls=move_calls=reset_calls=0;
 memset(&s_route_heading_reference,0,sizeof(s_route_heading_reference));RouteHeading_Init(&s_route_heading_reference,yaw);
 source_yaw=yaw;live.action_count=&s_motion_action_count;live.turn=&s_combined_turn;live.mode=&mode;
 live.has_action=&s_motion_has_action;live.distance_mm=&s_combined_distance_mm;live.reference_mm_s=&s_combined_reference_mm_s;live.actions=s_motion_actions;
}
"""
test=r"""
volatile uint32_t review_assertions,case_count;
volatile float debug_values[8];
volatile int32_t exported_telemetry[3][64];
volatile float exported_sessions[36][7];
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t id,z,dr,step,n,prepared_ok,done,index=0;float starts[3]={37,170,-170};uint32_t steps[3]={10,20,40};
 int32_t values[64];
 reset_case(8,37);CK(AppDebugPlan_Load(8,s_motion_actions,&s_motion_action_count));make_telemetry(values);memcpy((void*)exported_telemetry[0],values,sizeof(values));CK(values[56]==4 && values[57]==40000);
 for(id=20;id<=21;id++){
  reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));make_telemetry(values);
  CK(values[56]==3 && values[57]==(-2147483647L-1L) && values[38]==82 && values[39]==130);memcpy((void*)exported_telemetry[id-19],values,sizeof(values));
  s_motion_action_count=2;CK(!App_CombinedLoadAction(0));s_motion_action_count=1;CK(!App_CombinedLoadAction(1));
  s_motion_actions[0].relative_deg=-s_motion_actions[0].relative_deg;CK(!App_CombinedLoadAction(0));s_motion_actions[0].relative_deg=-s_motion_actions[0].relative_deg;
  s_motion_actions[0].relative_deg*=.99f;CK(!App_CombinedLoadAction(0));s_motion_actions[0].relative_deg=id==20?90:-90;
  s_motion_actions[0].base_rpm=81;CK(!App_CombinedLoadAction(0));s_motion_actions[0].base_rpm=82;
  s_motion_actions[0].run_ms=1;CK(!App_CombinedLoadAction(0));s_motion_actions[0].run_ms=0;
  s_motion_actions[0].distance_mm=1;CK(!App_CombinedLoadAction(0));s_motion_actions[0].distance_mm=0;
  s_motion_actions[0].reference_mm_s=1;CK(!App_CombinedLoadAction(0));s_motion_actions[0].reference_mm_s=0;
  s_motion_actions[0].kind=MOTION_FORWARD;CK(!App_CombinedLoadAction(0));s_motion_actions[0].kind=MOTION_TURN;
  CK(!turn_calls && !perimeter_calls && !move_calls);
  for(z=0;z<3;z++)for(dr=0;dr<2;dr++)for(step=0;step<3;step++){
   HeadingSession s_heading_prepared;HeadingSessionInput input={0};HeadingSessionOutput output={0};MotionProfileSample sample;
   AppHeadingRuntimeInputs inlinks={0};const AppHeadingRuntimeInputs *inputs=&inlinks;
   float base=starts[z]+(dr?.01f:0),relative=id==20?90:-90,s_heading_exec_relative;
   reset_case(id,starts[z]);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));CK(App_CombinedLoadAction(0));
   CK(turn_calls==1 && !perimeter_calls && !move_calls && reset_calls==1 && !s_route_heading_turn_active && s_combined_turn && s_motion_has_action && g_heading_exec_request==1);
   CK(g_heading_exec_relative_deg==relative && s_motion_turn_max_rpm==82);
   input.max_rpm=s_motion_turn_max_rpm;input.healthy=input.platform_ok=1;input.sequence=1;input.yaw_deg=base;
   inlinks.route_turn_active=&s_route_heading_turn_active;inlinks.route_target=&s_route_heading_target;
   s_heading_exec_relative=g_heading_exec_relative_deg;HeadingSession_Init(&s_heading_prepared);
"""+start_expr+r"""
   CK(prepared_ok && s_heading_prepared.brake.profile.valid && fabsf(s_heading_prepared.brake.profile.peak_speed-90)<.001f);
   CK(fabsf(HeadingController_ErrorDeg(output.target_deg,base)-relative)<.001f);
   HeadingSession_ReportBegin(&s_heading_prepared,1,&output);
   input.backend_active=1;for(n=40;n<=200;n+=40){input.now_ms=input.sample_ms=n;input.sequence++;HeadingSession_Step(&s_heading_prepared,&input,&output);CK(output.action!=HS_ACTION_ABORT);}
   debug_values[0]=s_heading_prepared.brake.tick;debug_values[1]=output.command_rpm;debug_values[2]=s_heading_prepared.turn_running_seen;debug_values[3]=s_heading_prepared.state;debug_values[4]=s_heading_prepared.cause;debug_values[5]=s_heading_prepared.turn_trial;
   CK(s_heading_prepared.brake.tick==200 && output.command_rpm==0 && !s_heading_prepared.turn_running_seen);
   input.backend_running=1;done=0;
   for(n=steps[step];n<8000;n+=steps[step]){
    input.now_ms=input.sample_ms=200+n;input.sequence++;
    MotionProfile_Sample(&s_heading_prepared.brake.profile,(n-steps[step])/1000.0f,&sample);
    input.yaw_deg=HeadingController_WrapDeg(base+(relative>0?1:-1)*sample.position_mm);input.rate_dps=(relative>0?1:-1)*sample.speed_mm_s;
    if(n>=400)input.backend_running=0;
    HeadingSession_Step(&s_heading_prepared,&input,&output);CK(output.action!=HS_ACTION_ABORT);
    CK(s_heading_prepared.brake.profile_tick==200+steps[step] && s_heading_prepared.turn_running_seen);
    if(output.action==HS_ACTION_STOP){
     HeadingSession_ReportStop(&s_heading_prepared,1,&output);input.backend_active=0;input.backend_result=1;input.backend_stop_mask=7;
     HeadingSession_Step(&s_heading_prepared,&input,&output);CK(output.action!=HS_ACTION_COMPLETE);
     input.backend_stop_mask=15;HeadingSession_Step(&s_heading_prepared,&input,&output);CK(output.action==HS_ACTION_COMPLETE);done=1;break;
    }
   }
   CK(done);exported_sessions[index][0]=id;exported_sessions[index][1]=base;exported_sessions[index][2]=output.target_deg;
   exported_sessions[index][3]=s_heading_prepared.brake.profile.peak_speed;exported_sessions[index][4]=steps[step];exported_sessions[index][5]=s_heading_prepared.state;exported_sessions[index][6]=n;index++;case_count++;
   g_combined_test.state=3;make_telemetry(values);CK(values[56]==3 && values[57]==(-2147483647L-1L));
  }
 }
 for(id=8;id<=19;id++){
  reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));CK(App_CombinedLoadAction(0));CK(move_calls==1 && !turn_calls && !perimeter_calls && !s_route_heading_turn_active);
  reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));s_motion_action_count=2;CK(!App_CombinedLoadAction(0));
 }
 reset_case(0,37);s_motion_action_count=1;s_motion_actions[0].kind=MOTION_TURN;s_motion_actions[0].base_rpm=82;s_motion_actions[0].relative_deg=90;
 CK(App_CombinedLoadAction(0));CK(perimeter_calls==1 && !turn_calls && s_route_heading_turn_active);
 reset_case(0,37);s_motion_action_count=1;s_motion_actions[0].kind=MOTION_FORWARD_DISTANCE;s_motion_actions[0].base_rpm=52;s_motion_actions[0].reference_mm_s=200;
 CK(App_CombinedLoadAction(0));CK(move_calls==1 && !turn_calls && !perimeter_calls);
 return 0;
}
"""
test=test.replace(' return 0;\n}\n','\n for(id=20;id<=21;id++){\n  uint32_t k,rpms[6]={0,5,13,27,81,83};\n  for(k=0;k<6;k++){\n   reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));s_motion_actions[0].base_rpm=rpms[k];CK(!App_CombinedLoadAction(0));CK(!turn_calls && !perimeter_calls && !move_calls);\n  }\n  reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));s_motion_action_count=0;CK(!App_CombinedLoadAction(0));s_motion_action_count=9;CK(!App_CombinedLoadAction(0));\n  reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));s_motion_actions[0].relative_deg+=(id==20?.001f:-.001f);CK(!App_CombinedLoadAction(0));\n  reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));g_heading_exec_phase=HR_RUN;CK(!App_CombinedLoadAction(0));CK(!s_motion_has_action);\n  reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));s_combined_turn=1;s_motion_has_action=1;g_combined_test.state=1;make_telemetry(values);CK(values[56]==3 && values[57]==(-2147483647L-1L));\n }\n reset_case(22,37);s_motion_action_count=1;s_motion_actions[0].kind=MOTION_TURN;s_motion_actions[0].base_rpm=82;s_motion_actions[0].relative_deg=90;CK(!App_CombinedLoadAction(0));\n'+' return 0;\n}\n');
unit=prefix+turn+'\n'+perimeter+'\n'+wrappers+load+'\n'+milli+tele+test
(T/'unit.c').write_bytes(unit.encode('gbk'))
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p.resolve()),b,len(b));return b.value
control=ET.parse(U.parents[1]/'gongxun-f407.uvprojx').getroot().find('./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls')
opts=['--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','--diag_suppress=177,550','-DAPP_VM_CALIBRATION_ONLY_ENABLE=0','-DAPP_VM_ONLY_ENABLE=0','-DAPP_WHEEL_ENABLE_HOLD_ENABLE=0','-DAPP_TURN90_90_ENABLE=1','-DAPP_TURN_BRAKE_TRIAL_DPS=90','-DAPP_BASELINE40_ENABLE=1','-DVEHICLE_INTEGRATED_ENABLE=0','-DVEHICLE_STAGE1_ENABLE=0']
opts+=['-D'+d for d in control.findtext('Define').split(',')]
for inc in control.findtext('IncludePath').split(';'):opts+=['-I',short(U.parents[1]/inc)]
def run(cmd,log):
 cp=subprocess.run(cmd,cwd=T,capture_output=True,timeout=90);(T/log).write_bytes(cp.stdout+cp.stderr);assert cp.returncode==0,(cp.stdout+cp.stderr).decode('gbk',errors='replace')
objects=[]
for p in [T/'unit.c',U/'app_entry.c',U/'chassis_kinematics.c',U/'motion_profile.c',U/'heading_session.c',U/'heading_controller.c']:
 obj=p.stem+'.o';objects.append(obj);run([str(B/'armcc.exe'),*opts,'-c',short(p),'-o',obj],obj+'.log')
(T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x50000 { *(+RW,+ZI) } }\n')
run([str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct',*objects,'--library_type=microlib','-o','unit.axf'],'link.log')
sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
from unicorn import arm_const as reg
blob=(T/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',blob);machine=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
for addr,size in [(0x08000000,0x100000),(0x20000000,0x50000),(0xE0000000,0x100000)]:machine.mem_map(addr,size)
for n in range(elf[12]):
 _,kind,flags,addr,offset,size,*_=struct.unpack_from('<IIIIIIIIII',blob,elf[6]+n*elf[11])
 if flags&2 and size:machine.mem_write(addr,b'\0'*size if kind==8 else blob[offset:offset+size])
stop=0x080FF000;machine.reg_write(reg.UC_ARM_REG_SP,0x2004FFF0);machine.reg_write(reg.UC_ARM_REG_LR,stop|1);machine.emu_start(elf[4]|1,stop,timeout=60000000,count=200000000)
assert machine.reg_read(reg.UC_ARM_REG_PC)==stop
result=machine.reg_read(reg.UC_ARM_REG_R0);symbols=subprocess.check_output([str(B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=T).decode(errors='replace')
def addr(name):return int(re.search(r'\b'+name+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
def words(name,count,fmt='i'):return struct.unpack('<'+fmt*count,machine.mem_read(addr(name),count*4))
assert result==0,(result,unit.splitlines()[result-1] if result else '',words('debug_values',8,'f'))
report={'status':'PASS','hardware_accessed':False,'c_assertions':words('review_assertions',1,'I')[0],'session_cases':words('case_count',1,'I')[0],'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [U/'app_motion_runtime.c',U/'app_serial_runtime.c',U/'app_heading_runtime.c',U/'heading_session.c']},'limitations':['HAL/RTOS/CAN acceptance and backend results are stubs; no on-board timing, stack or physical trajectory claim.','Heading input follows the reference for entry/lifecycle testing; existing MATLAB dynamics are separate.']}
rows=words('exported_telemetry',192);report['c_telemetry']=[list(rows[i*64:(i+1)*64]) for i in range(3)]
sessions=words('exported_sessions',36*7,'f');report['sessions']=[list(sessions[i*7:(i+1)*7]) for i in range(36)]
client_path=E.parent/'turn90-90-prep/p1_serial_motion_turn90.py';spec=importlib.util.spec_from_file_location('client',client_path);client=importlib.util.module_from_spec(spec);spec.loader.exec_module(client)
def record(plan):
 vals=report['c_telemetry'][0 if plan==8 else plan-19]
 r=dict.fromkeys(client.FIELDS,0);r.update(dict(zip(client.FIELDS[38:40],vals[38:40])))
 r.update(action_kind=vals[56],target_theoretical_milli_cm_s=vals[57],debug_plan_id=plan,motion_mode=3,source_valid=1,selftest=1,protocol_selftest=1,sequence_count=1,distance_mode=0)
 return r
report['client_runs']=[]
for plan in [20,21]:
 c=client.Controller('run-once',3,plan,expected_complete_powered=True);frames=[];tick=1000;seq=0
 def feed(r):
  global tick,seq
  tick+=100;seq+=1;r.update(tick_ms=tick,seq=seq)
  if r['debug_plan_id']==8:c.baseline_config=client.baseline40_config(8);c.baseline_tick=tick-10
  names=client.FIELDS+client.SEQUENCE_FIELDS+client.CAN_DIAG_FIELDS+client.ACTION_FIELDS+client.DISPLAY_FIELDS+client.DISTANCE_FIELDS+['distance_fault_reason','can_stack_words','debug_plan_id']
  prefix=('TMQ,'+','.join(str(r.get(k,0)) for k in names)+',').encode('ascii')
  wire=prefix+('%04X'%client.binascii.crc_hqx(prefix,0xffff)).encode('ascii')+b'\n'
  parsed=client.parse_telemetry(wire);assert parsed['action_kind']==r['action_kind']
  frames.extend(c.consume(parsed,tick/1000))
 for _ in range(3):feed(record(8))
 assert c.stage=='plan_wait'
 r=record(plan);r['remote_accepted']=1;feed(r);assert c.plan_selected
 for _ in range(3):r=record(plan);r['remote_accepted']=1;feed(r)
 assert c.stage=='arm_wait'
 r=record(plan);r.update(remote_state=1,remote_accepted=2,token_seq=c.token[0],token_tick=c.token[1]);feed(r);assert c.start_sent
 r=record(plan);r.update(phase=2,remote_state=2,remote_accepted=3,token_seq=c.token[0],token_tick=c.token[1]);feed(r);assert c.stage=='running'
 assert all(s[5]==3 for s in report['sessions'] if s[0]==plan),report['sessions']
 r=record(plan);r.update(phase=3,backend_state=9,backend_result=1,stop_mask=15,vm_request=1,pe2_active=1,sequence_state=3,sequence_completed=1);feed(r)
 assert c.status=='software_complete'
 verbs=[f.decode().split(',')[3] for f in frames];assert verbs==['PLAN'+str(plan),'ARM','START'],verbs
 report['client_runs'].append({'plan':plan,'commands':verbs,'status':c.status,'telemetry_action_kind_from_actual_c':record(plan)['action_kind']})
(E/'independent-review'/'integration_checks.json').write_text(json.dumps(report,indent=2),encoding='utf8');print(json.dumps({k:v for k,v in report.items() if k not in ['c_telemetry','sessions']}))
