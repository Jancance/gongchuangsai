"""Full actual chassis backend fixture plus actual telemetry/mission consumers."""
from pathlib import Path
import json,subprocess,shutil
from prepare_stage2_settle_ring_20261117 import R,P,E
def function(text,name):
 start=text.index(name);pos=text.index('{',start);level=1;end=pos+1
 while level:
  if text[end]=='{':level+=1
  elif text[end]=='}':level-=1
  end+=1
 return text[start:end]
def main():
 E.mkdir(parents=True,exist_ok=True)
 source=(R/'evidence/integration/2026-11-03/no-motion/handoff-backend-unit.c').read_text(encoding='utf-8')
 prefix=source[:source.index('#include "motion_control.h"')]
 prefix=prefix.replace('static void ck(uint32_t v){checks++;if(!v){failures++;printf("failed line check %u\\n",checks);}}',
  '#define ck(v) do{checks++;if(!(v)){printf("FAIL line%u check%u %s\\n",__LINE__,checks,#v);exit(1);}}while(0)')
 prefix=prefix.replace('#include <stdio.h>','#include <stdio.h>\n#include <stdlib.h>')
 (E/'chassis-source').mkdir(exist_ok=True)
 shutil.copyfile(P/'CHASSIS/USER/chassis_auto_test.c',E/'chassis-source/chassis_auto_test.c')
 prefix=prefix.replace('#include \"chassis_auto_test.c\"','#include \"chassis-source/chassis_auto_test.c\"')
 telemetry=function((P/'CHASSIS/USER/main.c').read_text(encoding='gb18030'),'static void App_PositionEndpointTelemetry(void)')
 support=r'''
uint32_t SourceBodyStep_SmallDistanceWindow(void){return 1;}
uint32_t SourceBodyStep_WheelWindow(void){return 0;}
uint32_t SourceBodyStep_FastAlignmentWindow(void){return 1;}
static unsigned emitted,fail_send,last_emitted;static char last_wire[512];
static unsigned App_SerialDiagnosticWindow(unsigned t,unsigned age){(void)t;(void)age;return 1;}
static uint16_t App_SerialCrc(const char*p,unsigned n){(void)p;(void)n;return 0;}
static unsigned App_DebugSendString(const char*p){if(fail_send)return 0;emitted++;snprintf(last_wire,sizeof(last_wire),"%s",p);return 1;}
'''+telemetry+r'''
static void reset(void){
 test_tick=1000;test_depth=test_primask=hardware_calls=0;
 g_rtos_fault=g_safety_fault=g_can_ctrl_fault_latched=g_motion_diag_fault_latched=0;
 g_motion_auto_stop_state=g_motion_auto_stop_cut_vm=g_wheel_test_active=g_wheel_test_cut_vm=0;
 g_safety_ready=g_can_guard_ready=1;ChassisAuto_Init();
}
static void begin_sequence(void){reset();ck(ChassisAuto_BeginSequence());ChassisAuto_SafetyUpdate(1);}
static void mark_samples(void){ChassisPositionEndpoints *e=&s_endpoints[s_endpoint_index];unsigned i;
 e->start_mask=e->end_mask=15;for(i=0;i<4;i++){e->start_raw[i]=100+i;e->end_raw[i]=200+i;e->start_ms[i]=test_tick-10;e->end_ms[i]=test_tick;}}
static void finish_true(unsigned heading){
 unsigned i;g_chassis_auto_state=CHASSIS_AUTO_SETTLE;g_chassis_auto_segment=2;g_chassis_auto_active=1;
 s_heading_session=heading;s_finish_requested=1;s_phase_tick=test_tick-CHASSIS_AUTO_GAP_MS;
 s_speed_valid=15;s_status_valid=15;g_chassis_auto_stop_mask[2]=15;
 for(i=0;i<4;i++){s_status[i]=1;s_speed_tick[i]=s_status_tick[i]=test_tick;g_chassis_auto_driver_rpm[i]=0;}
 if(heading)mark_samples();
 ChassisAuto_CanStep();if(g_chassis_auto_state!=CHASSIS_AUTO_DONE)printf("debug state%u result%u cause%u fault%u active%u\n",g_chassis_auto_state,g_chassis_auto_result,g_chassis_auto_cause,s_distance_fault_reason,g_chassis_auto_active);ck(g_chassis_auto_state==CHASSIS_AUTO_DONE && !g_chassis_auto_active && g_chassis_auto_result==1);
 ck(s_endpoints[s_endpoint_index].outcome==1);ck(ChassisAuto_ReleaseCompleted(g_chassis_auto_session));
}
static void prepared(unsigned mode){ChassisAutoPreparedTranslation p;MotionProfile profile;unsigned rpm,ms;
 ck(MotionRoute_Build(mode,20,45,&profile,&rpm,&ms));
 ck(ChassisAuto_PrepareRouteTranslation(mode,rpm,ms,20,45,&p));
 ck(ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session));}
static void check_order(unsigned latest){unsigned i;ChassisPositionEndpoints e;
 ck(s_endpoint_count==(latest<8?latest:8));
 for(i=0;i<s_endpoint_count;i++){ck(ChassisAuto_ReadPositionEndpoints(i,&e));ck(e.session==latest-s_endpoint_count+1+i);}
 ck(!ChassisAuto_ReadPositionEndpoints(s_endpoint_count,&e));
}
int main(void){unsigned i,before;ChassisPositionEndpoints save[8],e;ChassisAutoPreparedTranslation p;
 begin_sequence();
 for(i=1;i<=20;i++){
  prepared(i%4);ck(s_endpoint_index<8 && s_endpoint_count<=8);check_order(i);
  memcpy(save,s_endpoints,sizeof(save));before=g_chassis_auto_session;
  ck(!ChassisAuto_BeginPreparedTranslation(&(ChassisAutoPreparedTranslation){0},before));ck(!memcmp(save,s_endpoints,sizeof(save)));
  finish_true(1);ck(!g_chassis_auto_cause && !g_can_ctrl_fault_latched);
  if(i==1){fail_send=1;App_PositionEndpointTelemetry();ck(!emitted);fail_send=0;}
  App_PositionEndpointTelemetry();ck(emitted==i);
 }
 /* Diagnostic count cannot mutate a genuinely unfinished oldest record. */
 s_endpoints[s_endpoint_head].outcome=0;memcpy(save,s_endpoints,sizeof(save));before=g_chassis_auto_session;
 ck(!Auto_EndpointAvailable());
 ck(MotionRoute_Build(0,20,45,&p.profile,&p.base_rpm,&p.run_ms));
 ck(ChassisAuto_PrepareRouteTranslation(0,p.base_rpm,p.run_ms,20,45,&p));
 ck(!ChassisAuto_BeginPreparedTranslation(&p,before));ck(g_chassis_auto_session==before && !memcmp(save,s_endpoints,sizeof(save)));
 s_endpoints[s_endpoint_head].outcome=2;prepared(0);ck(g_chassis_auto_session==21);finish_true(1);
 /* Real fault/cancel preserves rejection and marks a terminal record. */
 prepared(0);Auto_CutLocked(7);ck(g_chassis_auto_state==CHASSIS_AUTO_FAULT && s_endpoints[s_endpoint_index].outcome==2);
 ck(!ChassisAuto_ReleaseCompleted(g_chassis_auto_session));
 begin_sequence();
 /* Basic modes share the real DONE path, never fake endpoint samples. */
 for(i=0;i<12;i++){ck(Auto_BeginPreparedBasicLocked(0,&(ChassisMotorCommand[12]){{0}}[0]));finish_true(0);ck(!s_endpoints[s_endpoint_index].start_mask && !s_endpoints[s_endpoint_index].end_mask);App_PositionEndpointTelemetry();}
 ck(s_endpoint_count==8);
 begin_sequence();g_chassis_auto_session=0xfffffffeU;
 for(i=0;i<3;i++){prepared(0);ck(g_chassis_auto_session==(i?i:0xffffffffU));finish_true(1);}
 ck(ChassisAuto_ReadPositionEndpoints(0,&e) && e.session==0xffffffffU);ck(s_endpoint_count==3);
 Auto_PositionReset(0);ck(!s_endpoint_count && !s_endpoint_head && s_endpoint_index==8);
 printf("PASS complete actual chassis ring: %u checks; real prepared/ReleaseCompleted20+, active refusal, terminal2 recycle, basic DONE masks, faults, getters/sessionwrap, telemetry>8 retry\n",checks);return failures?1:0;}
'''
 mission=function((P/'CHASSIS/USER/vehicle_mission.inc').read_text(encoding='utf-8'),'static uint32_t VehicleMission_CaptureHandoff(void)')
 msupport='''
#include "field_map.h"
static uint32_t g_stage1_motion_skipped,s_vehicle_zone=1U;
static int64_t s_vehicle_encoder_start[4];
static uint32_t s_vehicle_encoder_based,s_vehicle_handoff_session,s_vehicle_handoff_valid;
static FieldPoint s_vehicle_handoff;
'''+mission
 support=support.replace('static void reset(void){',msupport+'\nstatic void reset(void){',1)
 support=support.replace('finish_true(1);ck(!g_chassis_auto_cause && !g_can_ctrl_fault_latched);',
  'finish_true(1);ck(!g_chassis_auto_cause && !g_can_ctrl_fault_latched);\n  g_chassis_auto_result=1;ck(VehicleMission_CaptureHandoff() && s_vehicle_handoff_session==i);g_chassis_auto_result=0;')
 path=E/'actual-chassis-ring.c';path.write_text(prefix+support,encoding='utf-8');exe=E/'actual-chassis-ring.exe'
 cmd=['gcc','-std=c99','-Wno-unused-function','-Wno-misleading-indentation','-DAPP_MOTOR_OFFLINE_ONLY=0','-I'+str(R/'evidence/integration/2026-11-03/no-motion/backend-stubs'),'-I'+str(E),'-I'+str(P/'CHASSIS/USER'),str(path),str(P/'CHASSIS/USER/chassis_kinematics.c'),str(P/'CHASSIS/USER/motion_profile.c'),'-lm','-o',str(exe)]
 r=subprocess.run(cmd,capture_output=True,text=True);(E/'actual-chassis-ring-build.log').write_text(r.stdout+r.stderr);assert r.returncode==0,r.stderr
 r=subprocess.run([str(exe)],capture_output=True,text=True);(E/'actual-chassis-ring.log').write_text(r.stdout+r.stderr);print(r.stdout+r.stderr,flush=True);assert r.returncode==0
 # Compile the untouched legacy-distance #else path, not a hand-coded ring model.
 legacy=prefix.replace('#include "chassis-source/chassis_auto_test.c"', '#include "app_config.h"\n#include "motion_profile.h"\n#undef APP_DISTANCE_SCURVE_ACTIVE\n#define APP_DISTANCE_SCURVE_ACTIVE 0\n/* Fixture-only storage for unrelated blend code that assumes the active profile. */\nstatic MotionProfile s_distance_profile;\nstatic MotionProfileSample s_distance_reference;\nstatic uint32_t s_profile_done;\n#include "chassis-source/chassis_auto_test.c"')
 assert '#define APP_DISTANCE_SCURVE_ACTIVE 0' in legacy
 legacy_support=support[:support.index('int main(void){')]+r'''
int main(void){unsigned i;begin_sequence();
 for(i=1;i<=20;i++){ck(ChassisAuto_BeginTranslationDistance(0,5,100));check_order(i);finish_true(1);}
 s_endpoints[s_endpoint_head].outcome=0;ck(!ChassisAuto_BeginTranslationDistance(0,5,100));
 ck(!g_chassis_auto_active);s_endpoints[s_endpoint_head].outcome=2;ck(ChassisAuto_BeginTranslationDistance(0,5,100));
 finish_true(1);printf("PASS actual legacy-distance branch: %u checks; >20, unfinished refused, terminal recycled, original DONE/Release\n",checks);return failures?1:0;}
'''
 lp=E/'actual-legacy-distance-ring.c';lp.write_text(legacy+legacy_support,encoding='utf-8');le=E/'actual-legacy-distance-ring.exe'
 lcmd=[str(lp) if x==str(path) else str(le) if x==str(exe) else x for x in cmd]
 r=subprocess.run(lcmd,capture_output=True,text=True);(E/'actual-legacy-distance-build.log').write_text(r.stdout+r.stderr);assert r.returncode==0,r.stderr
 r=subprocess.run([str(le)],capture_output=True,text=True);(E/'actual-legacy-distance-ring.log').write_text(r.stdout+r.stderr);print(r.stdout+r.stderr,flush=True);assert r.returncode==0
 (E/'c-checks.json').write_text(json.dumps({'passed':True,'full_chassis_backend':True,'actual_mission_handoff':True,'both_distance_paths':True,'telemetry_real_function':True,'hardware_access':False},indent=2)+'\n')
if __name__=='__main__':main()
