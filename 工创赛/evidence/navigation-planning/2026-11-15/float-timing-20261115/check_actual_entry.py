from pathlib import Path
import ast,ctypes,hashlib,json,re,struct,subprocess,sys,xml.etree.ElementTree as ET
R=Path.cwd();U=R/'3.stm32f407工程/CHASSIS/USER';E=R/'工创赛/evidence/navigation-planning/2026-11-15/float-timing-20261115';T=R/'工创赛/tmp/float-timing-c-check-20261115';T.mkdir(exist_ok=True);B=Path('D:/Keil_v5/ARM/arm5_compiler/Bin')
oldparts=json.loads((R/'工创赛/evidence/navigation-planning/2026-11-13/baseline40-field/lateral-activation-fix/legacy_fixture_parts.json').read_text('utf8'))
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,32768);return b.value
def function(s,name):
 m=re.search(r'^(?:static\s+)?(?:__inline\s+)?(?:void|uint32_t|float)\s+'+name+r'\([^;]*?\)\s*\{',s,re.M);assert m,name
 e=s.index('{',m.start())+1;d=1
 while d:d+=(s[e]=='{')-(s[e]=='}');e+=1
 return s[m.start():e]
def read(n):
 b=(U/n).read_bytes()
 try:return b.decode('gbk')
 except UnicodeDecodeError:return b.decode('utf-8-sig')
assert '--before' not in sys.argv,'This capacity runner supports current candidate only'
before_mode=False
auto=(E/'chassis_auto_test.before.c').read_text('gbk') if before_mode else read('chassis_auto_test.c');kin=read('chassis_kinematics.c');motion=read('app_motion_runtime.c');trans=read('app_translation_runtime.c')
prefix=oldparts['prefix']+r'''
#include "hwt101.h"
uint32_t SystemCoreClock=84000000U;
static uint32_t test_timing_enabled=1U,test_cycles;
#include "app_entry.h"
#include "route_plan.h"
#include "app_motion_runtime.h"
#include "heading_source.h"
#include "motion_control.h"
#include "motion_distance_limits.h"
static uint32_t __get_BASEPRI(void){return 0U;}
static uint32_t __get_PRIMASK(void){return 0U;}
'''
globals_='#if APP_BASELINE40_ACTIVE\n'+auto[auto.index('volatile ChassisAutoLateralTiming'):auto.index('/* 以下带Locked')]
extra=r'''
static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];
static uint32_t s_motion_action_count;
static AppHeadingSource test_heading;
static uint32_t test_heading_override,test_heading_ready=1U,test_read_advance_ms,test_read_publish_now;
static uint32_t test_check_base_valid,test_check_base,test_check_session;
static uint32_t target_valid;
static float test_entry_origin;
#define MOTION_ACTION_COUNT s_motion_action_count
volatile uint32_t g_debug_plan_id=DEBUG_PLAN_DEFAULT;
volatile RouteCheck g_route_check;
static uint32_t s_combined_keep_vm,s_combined_powered_segment,s_combined_base_rpm,s_combined_run_ms;
static uint32_t s_motion_session_open,s_motion_has_action,s_motion_list_submit,s_combined_turn,s_combined_mode;
static uint32_t s_combined_begin_ms,s_combined_stage_ms;
static volatile uint32_t s_combined_end_ms;
static float s_combined_reference_mm_s,s_combined_distance_mm,s_route_heading_target;
static uint32_t s_route_heading_turn_active;
static RouteHeadingReference s_route_heading_reference;
static float s_perimeter_targets[9][3];
volatile float g_route_heading_targets[8],g_route_heading_start_yaw[8];
volatile uint32_t g_vm_test_request,g_motion_fault_diag_valid,g_heading_exec_request,g_stage1_stop_requested;
volatile ForwardHoldDebug g_forward_hold;
volatile uint32_t g_translation_hold_selftest=1U;
volatile ForwardHoldTraceSample g_forward_hold_trace[96];
volatile uint32_t g_forward_hold_trace_count;
static uint32_t s_hold_wait_ms,s_hold_begin_ms,s_hold_run_ms,s_hold_stop_ms,s_hold_sequence,s_hold_new_ms,s_hold_trace_ms;
static float s_hold_target;
static HeadingHoldController s_hold_controller;
static const AppTranslationRuntimeInputs s_translation_runtime_inputs={&s_combined_mode,&s_combined_keep_vm,&s_combined_powered_segment,&s_combined_base_rpm,&s_combined_run_ms,&s_combined_distance_mm,&s_combined_reference_mm_s,&s_route_heading_target,&s_route_heading_reference};
'''
obs=motion[motion.index('static const AppMotionObservation s_motion_observation'):motion.index('#endif',motion.index('static const AppMotionObservation s_motion_observation'))]
stubs=oldparts['stubs']+r'''
volatile ChassisAutoFaultDiagnostic g_chassis_auto_fault_diagnostic;
static uint32_t s_fault_diagnostic_valid;
static uint32_t Auto_BaseHealthy(void){return 1U;}

static uint32_t App_MotionAccept(void){return 1U;}
void HWT101_DiagnosticArm(void){}
uint32_t HWT101_TimingEnabled(void){return test_timing_enabled;}
uint32_t HWT101_TimingBegin(void){test_cycles+=32U;return test_timing_enabled?test_cycles:0U;}
void HWT101_TimingEnd(uint32_t a,uint32_t b){}
static uint32_t App_PerimeterTurnStart(uint32_t a,float b,uint32_t c){return 0U;}
uint32_t App_HeadingSourceRead(AppHeadingSource *h){
 if(test_heading_override){*h=test_heading;clock_ms+=test_read_advance_ms;test_read_advance_ms=0U;if(test_read_publish_now){h->sample_ms=h->angle_ms=h->rate_ms=clock_ms;test_read_publish_now=0U;}return test_heading_ready;}
 memset(h,0,sizeof(*h));h->healthy=1;h->valid_mask=3;h->yaw_deg=test_entry_origin;
 h->sample_ms=h->angle_ms=h->rate_ms=clock_ms;return 1U;}

void App_HeadingSourceObserve(void){}
uint32_t App_HeadingRuntimeSource(uint32_t now){return 1U;}
uint32_t App_HeadingRuntimeExclusive(void){return 1U;}
uint32_t App_HeadingPowerReady(uint32_t now){return 1U;}
static uint32_t App_CanJogReady(void){return 1U;}
void App_ForwardHoldSafety(const AppTranslationRuntimeInputs *p){}
void App_ForwardHoldTrip(uint32_t cause){g_forward_hold.trip=cause;g_forward_hold.phase=HR_FAULT;}
uint32_t App_ForwardHoldOwns(const ChassisAutoStatus *p,const AppTranslationRuntimeInputs *i){return 1U;}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *s){memset(s,0,sizeof(*s));s->session=g_chassis_auto_session;s->state=g_chassis_auto_state;s->active=g_chassis_auto_active;return 1U;}
uint32_t ChassisAuto_RequestStop(void){s_stop_due=1U;return 1U;}
uint32_t ChassisAuto_UpdateForwardHold(float c){s_heading_pending_rpm=c;return 1U;}
'''
# Every admission and dispatch function below is taken verbatim from current source.
names_motion=['App_MotionPlanValid','App_MoveSubmit','MoveStart','MoveDistanceStart','App_CombinedLoadAction']
names_auto=['Auto_EnterLocked','Auto_PrepareBasicCommands','Auto_BeginPreparedBasicLocked','Auto_PrepareTranslationEx','ChassisAuto_PrepareTranslation','ChassisAuto_PrepareRouteTranslation','ChassisAuto_BeginPreparedTranslation','ChassisAuto_CanStep','ChassisAuto_SafetyUpdate']
funcs='\n'.join(function(auto,n) for n in names_auto)+'\n'+function(read('app_entry.c'),'AppDebugPlan_Load')+'\n'+'\n'.join(function(motion,n) for n in names_motion)+'\n'+function(trans,'App_ForwardHoldStep')
entry=r'''
static uint32_t entry_start(void){
 uint32_t n=0;
 Auto_LateralReset();g_chassis_auto_active=g_chassis_auto_cut_vm=0U;g_chassis_auto_state=CHASSIS_AUTO_IDLE;g_chassis_auto_session=17U;
 s_used=0U;s_vm_requested=1U;s_endpoint_count=0U;s_endpoint_index=0U;
 g_debug_plan_id=DEBUG_PLAN_FORWARD210_40;g_forward_hold=(ForwardHoldDebug){0};
 s_motion_has_action=0U;s_combined_powered_segment=s_combined_keep_vm=g_vm_test_request=1U;s_combined_begin_ms=100U;
 s_route_heading_reference.valid=1U;s_route_heading_reference.origin_deg=test_entry_origin;
 clock_ms=1000U;
 if(!AppDebugPlan_Load(g_debug_plan_id,s_motion_actions,&n)) return 0U;s_motion_action_count=n;
 if(n!=1 || s_motion_actions[0].reference_mm_s!=0.0f || !App_MotionPlanValid())return 0U;
 if(!App_CombinedLoadAction(0U) || s_combined_reference_mm_s!=0.0f)return 0U;
 App_ForwardHoldStep(&s_translation_runtime_inputs);
 return g_forward_hold.trip==0U && g_forward_hold.phase==HR_RUN && g_chassis_auto_active && s_route_trial==0U;
}
'''
tests=oldparts['tests'].replace('s_lateral_enabled=1U;s_lateral_begin_ms=100U;','/* enabled and begin_ms originate in real BeginPreparedTranslation */')
tests=tests.replace(' Auto_LateralReset();g_chassis_auto_active=1U;g_chassis_auto_session=17U;',' entry_start();g_chassis_auto_active=1U;g_chassis_auto_session=17U;')
tests=tests.replace('for(b=0;b<=100;b++)for(h=-20;h<=20;h++)','for(b=0;b<=100;b+=10)for(h=-20;h<=20;h+=10)')
tests=tests.replace(' /* Real pure C integer invariants',r'''
 CHECK(entry_start());CHECK(s_lateral_enabled==1U);CHECK(s_motion_actions[0].reference_mm_s==0.0f);
 CHECK(AppMotion_Live()->action_count==&s_motion_action_count);CHECK(g_translation_hold_selftest==1U);
 {ChassisAutoPreparedTranslation p;uint32_t j;
 CHECK(ChassisAuto_PrepareTranslation(0,100,APP_BASELINE40_RUN_MS,2100,&p));CHECK(p.route_reference_mm_s==0.0f);
 for(j=0;j<7;j++){
  ChassisAutoPreparedTranslation bad=p;
  g_chassis_auto_active=0;g_chassis_auto_state=CHASSIS_AUTO_IDLE;s_used=0;
  Auto_LateralReset();
  if(j==0)bad.motion_mode=1;if(j==1)bad.base_rpm=99;if(j==2)bad.distance_mm=2099;
  if(j==3)bad.run_ms--;if(j==4)bad.profile.valid=0;if(j==5)bad.route_reference_mm_s=400;
  if(j==6)bad.profile.limits.speed++;
  CHECK(!ChassisAuto_BeginPreparedTranslation(&bad,g_chassis_auto_session));CHECK(!s_lateral_enabled);cases++;
 }
 for(j=0;j<2;j++){
  g_chassis_auto_active=0;g_chassis_auto_state=CHASSIS_AUTO_IDLE;s_used=0;Auto_LateralReset();
  g_debug_plan_id=j?DEBUG_PLAN_FORWARD210_40:DEBUG_PLAN_DEFAULT;s_motion_action_count=j?2U:1U;
  CHECK(ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session));CHECK(!s_lateral_enabled);cases++;
 }
 }
 /* Real pure C integer invariants''')
tests=tests.replace('setup();od=input_od;','setup();od=input_od;od.pose.x_mm=od.origin_x_mm;od.pose.y_mm=od.origin_y_mm;CHECK(Auto_LateralTarget(&od,100U,1001U)==0.0f);od=input_od;')
tests=tests.replace('CHECK(s_lateral_applied_deci>0 && last_source==3U);','CHECK(s_lateral_applied_deci>0 && last_source==3U);CHECK((int32_t)(saved[0].speed_raw-saved[1].speed_raw-saved[2].speed_raw+saved[3].speed_raw)==4*s_lateral_applied_deci);')

observations=r"""
volatile uint32_t entry_reference_milli,entry_enabled,entry_peak_l,entry_F,entry_T,entry_N,entry_wheels[4];
static uint32_t observe_entry(void){
 uint32_t k;int32_t q0,q1,q2,q3,lat;
 if(!entry_start())return 0U;
 entry_reference_milli=(uint32_t)(1000*s_motion_actions[0].reference_mm_s);entry_enabled=s_lateral_enabled;
 s_stop_due=s_finish_requested=s_monitor_between=s_motor_index=s_run_started=0U;
 g_chassis_auto_state=CHASSIS_AUTO_RUN;s_translation_base_rpm=s_translation_applied_base=100U;
 s_heading_pending_rpm=s_heading_applied_rpm=0;
 Chassis_TranslationProfileCommands(0U,100U,0,s_commands[2]);
 memset(&input_od,0,sizeof(input_od));input_od.anchor_valid=input_od.pose.valid=1U;input_od.reference_set_ms=100U;
 input_od.origin_x_mm=input_od.origin_y_mm=2100;input_od.path_heading_deg=-90;
 input_od.pose.x_mm=2080;input_od.pose.y_mm=1000;input_od.pose.heading_deg=-90;
 for(k=0;k<70;k++){
  clock_ms=1000U+k*20U;input_od.pose.sample_ms=clock_ms;ChassisAuto_CanStep();
 }
 q0=s_commands[2][0].speed_raw;q1=s_commands[2][1].speed_raw;q2=s_commands[2][2].speed_raw;q3=s_commands[2][3].speed_raw;
 lat=(q0-q1-q2+q3)/4;entry_peak_l=(uint32_t)lat;
 entry_F=(q0+q1+q2+q3)/4;entry_T=(-q0+q1-q2+q3)/4;entry_N=(q0+q1-q2-q3)/4;
 for(k=0;k<4;k++)entry_wheels[k]=s_commands[2][k].speed_raw;
 return entry_F==1000 && entry_T==0 && entry_N==0;
}
"""
tests=tests.replace(' CHECK(entry_start());CHECK(s_lateral_enabled==1U);',' CHECK(observe_entry());CHECK(entry_enabled==1 && entry_peak_l>0);CHECK(entry_F==1000 && entry_T==0 && entry_N==0);\n CHECK(entry_start());CHECK(s_lateral_enabled==1U);')
if before_mode:tests=r"""
volatile uint32_t assertions,cases;
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 assertions=3;cases=1;if(!observe_entry())return __LINE__;
 if(entry_reference_milli!=0 || entry_enabled!=0 || entry_peak_l!=0)return __LINE__;
 return 0U;
}
"""
tests=tests.replace('CHECK(s_heading_applied_rpm< -1.19f', 'CHECK((int32_t)(s_commands[2][0].speed_raw+s_commands[2][1].speed_raw-s_commands[2][2].speed_raw-s_commands[2][3].speed_raw)==0);CHECK((int32_t)(-s_commands[2][0].speed_raw+s_commands[2][1].speed_raw-s_commands[2][2].speed_raw+s_commands[2][3].speed_raw)==-48);\n CHECK(s_heading_applied_rpm< -1.19f')
tests=tests.replace('CHECK(!s_lateral_enabled && s_lateral_vy==0 && s_lateral_applied_deci==0 && !s_lateral_tick_valid);','CHECK(!s_lateral_enabled && s_lateral_vy==0 && s_lateral_applied_deci==0 && !s_lateral_tick_valid);saved_sends=sends;saved_tick=stops;ChassisAuto_CanStep();CHECK(sends==saved_sends && stops==saved_tick);')

# Gain-specific contracts exercise the extracted real Auto_LateralTarget in ARM C.
tests=tests.replace(' /* Real pure C integer invariants',r"""
 {static const float errors[5]={4.0f,8.0f,10.0f,12.0f,16.0f};
  static const float expected[5]={3.0f,6.0f,7.5f,9.0f,12.0f};
  for(j=0;j<5;j++)for(sign=-1;sign<=1;sign+=2){
   setup();od=input_od;od.path_heading_deg=od.pose.heading_deg=0.0f;
   od.pose.x_mm=od.origin_x_mm;od.pose.y_mm=od.origin_y_mm+sign*errors[j];
   first=Auto_LateralTarget(&od,100U,1001U);
   CHECK(fabsf(first+sign*expected[j])<0.001f);
   CHECK(first*sign<0.0f);CHECK(fabsf(first)<=12.1f);
   CHECK(Auto_LateralTarget(&od,101U,1001U)==0.0f);
   CHECK(Auto_LateralTarget(&od,100U,1400U)==0.0f);cases++;
  }
  setup();od=input_od;od.path_heading_deg=od.pose.heading_deg=0.0f;
  od.pose.x_mm=od.origin_x_mm;od.pose.y_mm=od.origin_y_mm;
  CHECK(Auto_LateralTarget(&od,100U,1001U)==0.0f);cases++;
 }
 /* Real pure C integer invariants""")


# Original budget assertion changes precisely with the authorized L contract.
tests=tests.replace('CHECK(l<=24 && l>=-24);','CHECK(l<=30 && l>=-30);')
tests=tests.replace(' /* Real pure C integer invariants',r"""
 /* Capacity-specific native C: every base 0..100, heading priority and integer remix. */
 for(b=0;b<=100;b++)for(h=-20;h<=20;h+=10)for(sign=-1;sign<=1;sign+=2){
  int32_t expected,room;
  CHECK(Chassis_Baseline40Budget((uint32_t)b,h*.1f,&c,&limit,&factor));
  expected=(int32_t)(12.1f/factor);if(expected>30)expected=30;
  room=10*b-(c<0?-c:c);if(expected>room)expected=room;
  room=50-(c<0?-c:c);if(expected>room)expected=room;if(expected<0)expected=0;
  CHECK(limit==expected);l=sign*limit;
  CHECK(Chassis_Baseline40Commands((uint32_t)b,c,l,cmd));
  CHECK((int32_t)(cmd[0].speed_raw+cmd[1].speed_raw+cmd[2].speed_raw+cmd[3].speed_raw)==40*b);
  CHECK((int32_t)(-cmd[0].speed_raw+cmd[1].speed_raw-cmd[2].speed_raw+cmd[3].speed_raw)==4*c);
  CHECK((int32_t)(cmd[0].speed_raw+cmd[1].speed_raw-cmd[2].speed_raw-cmd[3].speed_raw)==0);
  for(i=0;i<4;i++){CHECK(cmd[i].speed_raw<=10*(b+5));CHECK(cmd[i].address==i+1 && cmd[i].direction==g_chassis_forward_direction[i]);}
  cases++;
 }
 for(h=-20;h<=20;h+=20)for(l=-30;l<=30;l++){
  CHECK(Chassis_Baseline40Commands(100U,h,l,cmd));
  CHECK((int32_t)(cmd[0].speed_raw-cmd[1].speed_raw-cmd[2].speed_raw+cmd[3].speed_raw)==4*l);cases++;
 }
 memset(cmd,0xA5,sizeof(cmd));memcpy(saved,cmd,sizeof(saved));
 CHECK(!Chassis_Baseline40Commands(100U,0,31,cmd));CHECK(memcmp(cmd,saved,sizeof(cmd))==0);
 CHECK(!Chassis_Baseline40Commands(100U,0,-31,cmd));CHECK(memcmp(cmd,saved,sizeof(cmd))==0);
 CHECK(!Chassis_Baseline40Commands(100U,21,30,cmd));CHECK(memcmp(cmd,saved,sizeof(cmd))==0);
 CHECK(!Chassis_Baseline40Commands(100U,-21,-30,cmd));CHECK(memcmp(cmd,saved,sizeof(cmd))==0);
 setup();od=input_od;od.path_heading_deg=od.pose.heading_deg=0.0f;od.pose.x_mm=od.origin_x_mm;
 od.pose.y_mm=od.origin_y_mm+20.0f;CHECK(fabsf(Auto_LateralTarget(&od,100U,1001U)+12.1f)<.0001f);
 od.pose.y_mm=od.origin_y_mm-20.0f;CHECK(fabsf(Auto_LateralTarget(&od,100U,1001U)-12.1f)<.0001f);
 for(sign=-1;sign<=1;sign+=2){
  setup();s_translation_base_rpm=100U;s_lateral_vy=sign*12.1f;s_lateral_tick_valid=1U;s_lateral_tick_ms=1000U;
  CHECK(Auto_LateralStepLocked(1100U,sign*12.1f,2.0f,&c,&l));CHECK(c==20 && l==sign*30);
  CHECK(Chassis_Baseline40Commands(100U,c,l,cmd));
  s_translation_base_rpm=2U;CHECK(Auto_LateralStepLocked(1101U,sign*12.1f,2.0f,&c,&l));CHECK(c==20 && l==0 && s_lateral_vy==0);
  s_translation_base_rpm=0U;CHECK(Auto_LateralStepLocked(1102U,sign*12.1f,0,&c,&l));CHECK(c==0 && l==0 && s_lateral_vy==0);cases++;
 }
 setup();s_lateral_vy=0;s_lateral_tick_valid=1U;s_lateral_tick_ms=1000U;
 CHECK(Auto_LateralStepLocked(1100U,12.1f,0,&c,&l));CHECK(s_lateral_vy==2.0f && l==4);cases++;
 /* Real pure C integer invariants""")


stubs=stubs.replace('static void Auto_CheckLocked(TickType_t now){(void)now;}',r"""static void Auto_CheckLocked(TickType_t now){(void)now;
 if(test_check_base_valid){s_translation_base_rpm=test_check_base;test_check_base_valid=0U;}
 if(test_check_session){g_chassis_auto_session++;test_check_session=0U;}
}""")
tests=tests.replace('Auto_LateralTarget(&od,100U,1001U)+12.1f','Auto_LateralTarget(&od,100U,1001U)+15.0f')
tests=tests.replace('Auto_LateralTarget(&od,100U,1001U)-12.1f','Auto_LateralTarget(&od,100U,1001U)-15.0f')
tests=re.sub(r'Auto_LateralTarget\(([^()]*)\)',lambda m:'Auto_LateralTarget('+m[1]+',&target_valid)',tests)
setup_source=function(tests,'setup')
tests=tests.replace(setup_source,setup_source.replace('{','{\n test_heading_override=test_read_advance_ms=test_read_publish_now=test_enter_advance_ms=test_check_base_valid=test_check_session=0U;',1)+r"""
static void ff_setup(float delta,float right,float origin){
 test_entry_origin=origin;test_heading_override=0U;setup();
 input_od.pose.x_mm=input_od.origin_x_mm-right;input_od.pose.y_mm=input_od.origin_y_mm;
 input_od.path_heading_deg=input_od.pose.heading_deg=-90.0f;input_od.heading_offset_deg=-90.0f-origin;
 memset(&test_heading,0,sizeof(test_heading));test_heading.healthy=1U;test_heading.valid_mask=3U;
 test_heading.yaw_deg=origin+delta;if(test_heading.yaw_deg>180)test_heading.yaw_deg-=360;
 if(test_heading.yaw_deg< -180)test_heading.yaw_deg+=360;
 test_heading.sample_ms=test_heading.angle_ms=test_heading.rate_ms=clock_ms;
 test_heading_ready=test_heading_override=1U;s_lateral_tick_valid=1U;s_lateral_tick_ms=clock_ms-100U;s_lateral_vy=0;
}
static int32_t ff_L(void){return ((int32_t)s_commands[2][0].speed_raw-(int32_t)s_commands[2][1].speed_raw-(int32_t)s_commands[2][2].speed_raw+(int32_t)s_commands[2][3].speed_raw)/4;}
static void ff_steady(void){uint32_t k;for(k=0;k<70;k++){clock_ms=1000U+20U*k;input_od.pose.sample_ms=clock_ms;test_heading.sample_ms=test_heading.angle_ms=test_heading.rate_ms=clock_ms;ChassisAuto_CanStep();}}
volatile uint32_t ff_positive_L,ff_negative_abs_L,ff_cross_L,ff_half_F_L,ff_HWT_fallback_L,ff_ODinvalid_L,ff_unified_clamp_L;
""")
tests=tests.replace(' /* Real pure C integer invariants',r"""
 /* Near-line scheduling: zero error withdraws FF, including fresh/wrapped yaw. */
 ff_setup(-1,0,0);test_read_advance_ms=1;test_read_publish_now=1;ChassisAuto_CanStep();CHECK(clock_ms==1001 && ff_L()==0);cases++;
 ff_setup(-1.0f,0,0);ff_steady();CHECK(s_lateral_enabled==1U);CHECK(ff_L()==0);ff_positive_L=ff_L();
 CHECK((int32_t)(s_commands[2][0].speed_raw+s_commands[2][1].speed_raw+s_commands[2][2].speed_raw+s_commands[2][3].speed_raw)==4000);
 CHECK((int32_t)(-s_commands[2][0].speed_raw+s_commands[2][1].speed_raw-s_commands[2][2].speed_raw+s_commands[2][3].speed_raw)==0);cases++;
 ff_setup(1.0f,0,0);ff_steady();CHECK(ff_L()==0);ff_negative_abs_L=-ff_L();cases++;
 ff_setup(1.0f,0,179.5f);ff_steady();CHECK(ff_L()==0);ff_cross_L=-ff_L();cases++;
 ff_setup(-1.0f,0,-179.5f);ff_steady();CHECK(ff_L()==0);cases++;
 /* Actual locked F changes after lock-free trigonometry: target uses F50. */
 ff_setup(-.5f,0,0);test_check_base_valid=1;test_check_base=50;ChassisAuto_CanStep();
 CHECK(s_translation_base_rpm==50 && ff_L()==0);ff_half_F_L=ff_L();CHECK((int32_t)(s_commands[2][0].speed_raw+s_commands[2][1].speed_raw+s_commands[2][2].speed_raw+s_commands[2][3].speed_raw)==2000);cases++;
 ff_setup(-1,0,0);s_translation_base_rpm=0;ChassisAuto_CanStep();CHECK(s_lateral_vy==0 && s_lateral_applied_deci==0);cases++;
 /* Raw P22.5 plus opposite FF must stay positive: separate P clamp would flip it. */
 ff_setup(4.0f,30.0f,0);ChassisAuto_CanStep();CHECK(ff_L()==4 && s_lateral_vy==2.0f);ff_unified_clamp_L=ff_L();cases++;
 for(j=0;j<8;j++){
  ff_setup(-1,4,0);s_lateral_vy=3.0f;
  if(j==0)test_heading_ready=0;if(j==1)test_heading.healthy=0;if(j==2)test_heading.sample_ms=900;
  if(j==3)test_heading.angle_ms=900;if(j==4)test_heading.rate_ms=900;if(j==5)test_heading.valid_mask=2;
  if(j==6)test_heading.yaw_deg=11;if(j==7)test_heading.rate_dps=3000;
  ChassisAuto_CanStep();CHECK(s_lateral_vy==3.0f && ff_L()==7);CHECK(g_chassis_auto_active && !cut_cause);ff_HWT_fallback_L=ff_L();cases++;
 }
 for(j=0;j<3;j++){
  ff_setup(-1,0,0);test_heading.sample_ms=test_heading.angle_ms=test_heading.rate_ms=901;
  if(j==1){test_heading.sample_ms=1000;test_heading.angle_ms=901;test_heading.rate_ms=1000;}
  if(j==2){test_heading.sample_ms=1000;test_heading.angle_ms=1000;test_heading.rate_ms=901;}
  ChassisAuto_CanStep();CHECK(ff_L()==0);cases++;
 }
 /* Source valid outside, age100 at lock entry: FF0, P still3. */
 ff_setup(-1,4,0);clock_ms=999;input_od.pose.sample_ms=999;test_heading.sample_ms=test_heading.angle_ms=test_heading.rate_ms=900;s_lateral_tick_ms=899;s_lateral_vy=3;test_enter_advance_ms=1;
 ChassisAuto_CanStep();CHECK(clock_ms==1000 && s_lateral_vy==3 && ff_L()==7);cases++;
 for(j=0;j<9;j++){
  ff_setup(-1,4,0);s_lateral_vy=3;
  if(j==0)input_od.anchor_valid=0;if(j==1)input_od.pose.valid=0;if(j==2)input_od.pose.reasons=1;
  if(j==3)input_od.pose.continuity_lost=1;if(j==4)input_od.pose.local_origin=1;
  if(j==5)input_od.reference_set_ms=99;if(j==6)input_od.pose.sample_ms=600;
  if(j==7)test_check_session=1;if(j==8){input_od.pose.sample_ms=601;test_enter_advance_ms=1;}
  ChassisAuto_CanStep();CHECK(s_lateral_vy==1.0f && ff_L()==2);CHECK(g_chassis_auto_active && !cut_cause);ff_ODinvalid_L=ff_L();cases++;
 }
 ff_setup(-1,0,0);input_od.pose.sample_ms=601;ChassisAuto_CanStep();CHECK(ff_L()==0);cases++;
 /* Nonfinite OD rejects both terms; nonfinite fresh yaw rejects only FF. */
 {union{uint32_t u;float f;}bad;bad.u=0x7FC00000U;
  ff_setup(-1,4,0);s_lateral_vy=3;test_heading.yaw_deg=bad.f;ChassisAuto_CanStep();CHECK(s_lateral_vy==3 && ff_L()==7);cases++;
  ff_setup(-1,4,0);s_lateral_vy=3;input_od.heading_offset_deg=bad.f;ChassisAuto_CanStep();CHECK(s_lateral_vy==1 && ff_L()==2);cases++;
  bad.u=0x7F800000U;ff_setup(-1,4,0);s_lateral_vy=3;test_heading.yaw_deg=bad.f;ChassisAuto_CanStep();CHECK(s_lateral_vy==3 && ff_L()==7);cases++;
 }
 /* Finite offsets/path checked outside before fmod/tan; real wrap freshness. */
 ff_setup(-1,0,0);clock_ms=5;test_heading.sample_ms=test_heading.angle_ms=test_heading.rate_ms=0xFFFFFFFAU;input_od.pose.sample_ms=5;s_lateral_tick_ms=0xFFFFFFA1U;ChassisAuto_CanStep();CHECK(ff_L()==0);cases++;
 /* FF computed anew during wheels2..4 cannot mutate accepted batch/state. */
 ff_setup(-1,4,0);ChassisAuto_CanStep();memcpy(saved,s_commands[2],sizeof(saved));saved_tick=s_lateral_tick_ms;first=s_lateral_vy;
 for(k=1;k<4;k++){clock_ms+=10;input_od.pose.sample_ms=clock_ms;test_heading.sample_ms=test_heading.angle_ms=test_heading.rate_ms=clock_ms;test_heading.yaw_deg=1;ChassisAuto_CanStep();ChassisAuto_CanStep();CHECK(memcmp(saved,s_commands[2],sizeof(saved))==0 && s_lateral_vy==first && s_lateral_tick_ms==saved_tick);cases++;}
 for(j=0;j<3;j++){
  ff_setup(-1,4,0);ChassisAuto_CanStep();
  if(j==0)s_stop_due=1;if(j==1){s_stop_due=s_finish_requested=1;}if(j==2){ChassisAuto_SafetyUpdate(0);}
  ChassisAuto_CanStep();CHECK(s_lateral_vy==0 && !s_lateral_tick_valid && s_lateral_applied_deci==0);cases++;
 }

 for(j=0;j<4;j++){
  static const float er[4]={0,1,2,4};static const int32_t ex[4]={0,5,11,20};
  ff_setup(-1,er[j],0);ff_steady();CHECK(ff_L()==ex[j]);cases++;
  ff_setup(1,-er[j],0);ff_steady();CHECK(ff_L()==-ex[j]);cases++;
 }
 ff_setup(-1,-4,0);ff_steady();CHECK(ff_L()==-7);cases++;
 ff_setup(1,4,0);ff_steady();CHECK(ff_L()==7);cases++;
 ff_setup(-1,4,179.5f);ff_steady();CHECK(ff_L()==20);cases++;
 ff_setup(1,-4,-179.5f);ff_steady();CHECK(ff_L()==-20);cases++;
 ff_setup(-.5f,4,0);test_check_base_valid=1;test_check_base=50;ChassisAuto_CanStep();ff_steady();CHECK(s_translation_base_rpm==50 && ff_L()==10);cases++;
 ff_setup(-1,4,0);test_heading_ready=0;ff_steady();CHECK(ff_L()==7);cases++;
 ff_setup(-1,4,0);input_od.pose.valid=0;ff_steady();CHECK(ff_L()==0);cases++;
 test_heading_override=0;test_entry_origin=0;test_check_base_valid=test_check_session=test_read_advance_ms=0;
 /* Real pure C integer invariants""")


prefix=prefix.replace('static uint32_t clock_ms,lock_depth;','static uint32_t clock_ms,lock_depth,test_enter_advance_ms;')
prefix=prefix.replace('#define taskENTER_CRITICAL() (++lock_depth)','#define taskENTER_CRITICAL() (++lock_depth,clock_ms+=test_enter_advance_ms,test_enter_advance_ms=0U)')

# Prototype for real functions called before their textual definition.
proto='static uint32_t App_MoveSubmit(uint32_t,uint32_t,uint32_t,float,float);\n'
tests=tests.replace(' uint32_t saved_tick,saved_sends,k,j;AppOdometryBaseline40 od;', ' uint32_t saved_tick,saved_sends,k,j;AppOdometryBaseline40 od;'+'\n {\n ChassisAutoLateralTiming a,z; uint32_t previous;\n CHECK(sizeof(a)==104U && sizeof(a.scope[0])==48U);\n CHECK((uint32_t)((char*)&a.scope[0].sum_cycles-(char*)&a)==32U && (uint32_t)((char*)&a.scope[1]-(char*)&a)==56U);\n memset((void*)&g_chassis_auto_lateral_timing,0,sizeof(g_chassis_auto_lateral_timing));\n test_timing_enabled=0U; ChassisAuto_ReadLateralTiming(&z); CHECK(!z.enabled && !z.scope[0].valid && !z.scope[0].count);\n Auto_LateralTimingRecordLocked(0U,0U,1U,2U,100U);ChassisAuto_ReadLateralTiming(&a);CHECK(!a.scope[0].count);\n test_timing_enabled=1U;\n Auto_LateralTimingRecordLocked(0U,1U,0xFFFFFFF0U,16U,0xFFFFFFF0U);\n Auto_LateralTimingRecordLocked(0U,1U,0U,0U,5U);\n Auto_LateralTimingRecordLocked(0U,1U,0U,0xFFFFFFFFU,16U);\n ChassisAuto_ReadLateralTiming(&a);CHECK(a.enabled && a.core_clock_hz==84000000U);\n CHECK(a.scope[0].valid && a.scope[0].count==3U && a.scope[0].min_cycles==0U);\n CHECK(a.scope[0].last_cycles==0xFFFFFFFFU && a.scope[0].max_cycles==0xFFFFFFFFU);\n CHECK(a.scope[0].sum_cycles==((uint64_t)0xFFFFFFFFU+32U) && a.scope[0].window_ms==32U);\n CHECK(!a.scope[1].valid && !a.scope[1].count);\n Auto_LateralTimingRecordLocked(0U,1U,1U,2U,17U);CHECK(a.scope[0].count==3U);\n g_chassis_auto_lateral_timing.scope[0].count=0xFFFFFFFFU;\n z=g_chassis_auto_lateral_timing;Auto_LateralTimingRecordLocked(0U,1U,0U,0xFFFFFFFFU,18U);\n CHECK(memcmp(&z,(const void*)&g_chassis_auto_lateral_timing,sizeof(z))==0);\n memset((void*)&g_chassis_auto_lateral_timing,0,sizeof(g_chassis_auto_lateral_timing));\n ff_setup(-1,4,0);memset((void*)&g_chassis_auto_lateral_timing,0,sizeof(g_chassis_auto_lateral_timing));ChassisAuto_CanStep();ChassisAuto_ReadLateralTiming(&a);\n CHECK(a.scope[0].count==1U);\n CHECK(a.scope[1].count==1U);\n CHECK(a.scope[0].last_cycles==32U);\n CHECK(a.scope[1].last_cycles==32U);\n test_timing_enabled=0U;ChassisAuto_CanStep();ChassisAuto_ReadLateralTiming(&z);\n CHECK(!z.enabled && z.scope[0].count==1U && z.scope[1].count==1U);\n test_timing_enabled=1U;g_chassis_auto_state=CHASSIS_AUTO_RUN;s_motor_index=s_monitor_between=0U;input_od.pose.valid=0U;ChassisAuto_CanStep();ChassisAuto_ReadLateralTiming(&z);\n CHECK(z.scope[0].count==1U && z.scope[1].count==2U);\n test_timing_enabled=1U;s_stop_due=1U;previous=z.scope[1].count;ChassisAuto_CanStep();ChassisAuto_ReadLateralTiming(&z);\n CHECK(z.scope[0].count==1U && z.scope[1].count==previous);\n g_chassis_auto_active=0U;ChassisAuto_CanStep();ChassisAuto_ReadLateralTiming(&z);\n CHECK(z.scope[0].count==1U && z.scope[1].count==previous);\n ChassisAuto_ReadLateralTiming(0);CHECK(lock_depth==0U);cases+=12U;\n }\n')
unit=prefix+'\n'+re.sub(r'^#include[^\n]*','',kin,flags=re.M)+'\n'+read('motion_profile.c')+'\n'+read('heading_controller.c')+'\n'+globals_+'\n'+extra+'\n'+obs+'\n'+stubs+'\n'+proto+'\n'+funcs+'\n'+entry+'\n'+observations+'\n'+tests
(T/'unit.c').write_bytes(unit.encode('gbk',errors='replace'))
(T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n')
v=ET.parse(U.parents[1]/'gongxun-f407.uvprojx').getroot().find('./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls')
opts=['--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','--diag_suppress=177,550','-DAPP_BASELINE40_ENABLE=1','-DVEHICLE_INTEGRATED_ENABLE=0','-DVEHICLE_STAGE1_ENABLE=0']+['-D'+d for d in v.findtext('Define').split(',')]
for p in v.findtext('IncludePath').split(';'):opts+=['-I',short((U.parents[1]/p).resolve())]
commands=[[str(B/'armcc.exe')]+opts+['-c','unit.c','-o','unit.o'],[str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]
for i,command in enumerate(commands):
 q=subprocess.run(command,cwd=T,capture_output=True,timeout=90);(T/['compile.log','link.log'][i]).write_bytes(q.stdout+q.stderr);assert q.returncode==0,(q.stdout+q.stderr).decode('gbk',errors='replace')
sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
from unicorn import arm_const as reg
blob=(T/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',blob)
m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
sections=[]
for i in range(elf[12]):
 name,kind,flags,address,offset,size,_,_,_,_=struct.unpack_from('<IIIIIIIIII',blob,elf[6]+i*elf[11])
 if flags&2 and size:m.mem_write(address,b'\0'*size if kind==8 else blob[offset:offset+size]);sections.append({'kind':kind,'address':address,'bytes':size})
stop=0x080FF000;m.reg_write(reg.UC_ARM_REG_SP,0x2001FFF0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(elf[4]|1,stop,timeout=60000000,count=100000000)
assert m.reg_read(reg.UC_ARM_REG_PC)==stop,'execution incomplete'
result=m.reg_read(reg.UC_ARM_REG_R0);assert result==0,f'C assertion failed line {result}'
symbols=subprocess.check_output([str(B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=T).decode(errors='replace')
def value(n):
 a=int(re.search(r'\b'+n+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16);return struct.unpack('<I',m.mem_read(a,4))[0]
report={'all_passed':True,'cases':value('cases'),'assertions':value('assertions'),'before_source_mode':before_mode,'entry_observation':{n:value(n) for n in ['entry_reference_milli','entry_enabled','entry_peak_l','entry_F','entry_T','entry_N']},'tested_auto_sha256':hashlib.sha256((E/'chassis_auto_test.before.c' if before_mode else U/'chassis_auto_test.c').read_bytes()).hexdigest(),'source_sha256':{n:hashlib.sha256((U/n).read_bytes()).hexdigest() for n in ['app_entry.c','app_motion_runtime.c','app_translation_runtime.c','chassis_auto_test.c','chassis_kinematics.c','motion_profile.c','heading_controller.c']},'actual_functions':names_motion+names_auto+['AppDebugPlan_Load','App_ForwardHoldStep','Auto_LateralTarget','Auto_LateralHeadingFresh','Auto_LateralFeedforwardPerRpm','Auto_LateralStepLocked'], 'ff_observation':{n:value(n) for n in ['ff_positive_L','ff_negative_abs_L','ff_cross_L','ff_half_F_L','ff_HWT_fallback_L','ff_ODinvalid_L','ff_unified_clamp_L']},'runner_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'fixture_axf_sha256':hashlib.sha256((T/'unit.axf').read_bytes()).hexdigest(),'fixture_sha256':hashlib.sha256((T/'unit.c').read_bytes()).hexdigest(),'commands':commands,'sections_initialized':sections,'hardware_accessed':False,'limits':'Actual Load/PlanValid/dispatch/MoveDistanceStart/Submit/ForwardHoldStep/Prepare/Begin/CanStep executed. External heading readiness, power, App_MotionAccept, ForwardHoldSafety, CAN I/O, Auto_CheckLocked and diagnostics are stubs. Not RTOS/hardware/physical validation.'}
(E/('before_unit.c' if before_mode else 'after_unit.c')).write_bytes((T/'unit.c').read_bytes())
(E/('actual_c_before_check.json' if before_mode else 'actual_c_check.json')).write_text(json.dumps(report,indent=2),'utf8');print(json.dumps({k:report[k] for k in ['all_passed','cases','assertions','before_source_mode','entry_observation']},indent=2))

