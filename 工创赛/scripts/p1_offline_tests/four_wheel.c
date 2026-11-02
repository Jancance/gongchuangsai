/* Host-only test adapter. CAN/VM/RTOS are stubs; src is copied live firmware. */
#include "src/chassis_auto_test.c"
#include "src/app_entry.c"
#include <math.h>
uint32_t test_tick,test_depth,test_primask;
volatile uint32_t g_rtos_fault,g_safety_fault,g_safety_ready,g_can_guard_ready;
volatile uint32_t g_motion_diag_fault_latched,g_motion_auto_stop_state,g_motion_auto_stop_cut_vm;
volatile uint32_t g_wheel_test_active,g_wheel_test_cut_vm,g_can_ctrl_fault_latched;
volatile uint32_t g_can_poll_result[4],g_can_poll_flags[4],g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
static uint32_t checks,first_failure,scenario,step;
static float position[4],velocity[4],target[4],history[4][128];
static uint32_t peak_command[4],calls[4],stop_calls[4];
/* case, result, cause, detail, wheel, distance reason, reached, stop mask,
 * cut request, elapsed ms, four positions um, four command peaks, checks */
volatile uint32_t results[10][19];
/* One sample is streamed through a single emulator hook; no giant RAM trace.
 * These test-only symbols are never linked into the vehicle firmware. */
volatile float trace_values[53];
volatile uint32_t trace_count;
#if defined(__CC_ARM)
__attribute__((noinline))
#endif
void Trace_Emit(void){trace_count++;}
#define CHECK(x) do {checks++;if(!(x)&&!first_failure)first_failure=__LINE__;}while(0)
static void record_trace(void)
{
 uint32_t w;MotionProfileSample reference;float mean=0,calibrated=0;
 float elapsed=(float)(test_tick-s_run_tick)*0.001f;
 CHECK(MotionProfile_Sample(&s_distance_profile,elapsed,&reference)!=MOTION_PROFILE_INVALID);
 CHECK(reference.speed_mm_s>=-0.001f && reference.speed_mm_s<=200.01f);
 CHECK(fabsf(reference.acceleration_mm_s2)<=s_distance_profile.limits.acceleration+0.001f);
 CHECK(fabsf(reference.jerk_mm_s3)<=s_distance_profile.limits.jerk+0.001f);
 trace_values[0]=(float)scenario;trace_values[1]=elapsed;
 trace_values[2]=reference.position_mm;trace_values[3]=reference.speed_mm_s;
 trace_values[4]=reference.acceleration_mm_s2;trace_values[5]=reference.jerk_mm_s3;
 trace_values[6]=(float)s_translation_base_rpm;trace_values[7]=s_profile_follow_rpm;
 trace_values[8]=s_profile_follow_rate;trace_values[9]=s_distance_progress_mm;
 trace_values[10]=(float)g_chassis_auto_state;trace_values[11]=(float)g_chassis_auto_result;
 trace_values[12]=(float)g_chassis_auto_cause;trace_values[13]=(float)s_distance_fault_reason;
 trace_values[14]=(float)s_distance_reached;trace_values[15]=(float)g_chassis_auto_stop_mask[2];
 trace_values[16]=(float)g_chassis_auto_cut_vm;trace_values[17]=(float)s_profile_done;
 for(w=0;w<4;w++){
  trace_values[18+w]=target[w];trace_values[22+w]=velocity[w];
  trace_values[26+w]=(float)g_chassis_auto_driver_rpm[w];
  trace_values[30+w]=(float)(test_tick-s_speed_tick[w]);
  trace_values[34+w]=position[w];trace_values[38+w]=(float)calls[w];
  trace_values[42+w]=(float)stop_calls[w];mean+=position[w]*0.25f;
 }
 CHECK(Chassis_CalibrateDistanceMm(0,mean,&calibrated)==1);
 trace_values[46]=calibrated;trace_values[47]=s_distance_target_mm;
 trace_values[48]=s_distance_profile.limits.acceleration;trace_values[49]=s_distance_profile.limits.jerk;
 trace_values[50]=s_distance_profile.duration_s;
 trace_values[51]=s_distance_profile.boundary_s[4];
 trace_values[52]=(float)s_finish_requested;
 Trace_Emit();
}
void Power_ReadCommandStatus(PowerCommandStatus*p){p->active=1;}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg*r){r->Data[1]=1;return 1;}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg*r,int32_t*v)
{
 uint32_t w=a-1;int32_t raw=(int32_t)(fabsf(velocity[w])+0.5f);
 if(scenario==3 && a==3 && step>=300)return 0;
 if(scenario==6 && a==2 && step>=300)raw=81;
 *v=g_chassis_forward_direction[w] ? -raw : raw;
 if(scenario==7 && a==4 && step>=300)*v=raw;
 return 1;
}
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg*r,uint8_t*sign,uint32_t*raw)
{
 if(scenario==4 && a==1 && step>=300)return 0;
 *sign=g_chassis_forward_direction[a-1];
 *raw=(uint32_t)(position[a-1]*65536.0f/(3.14159265358979323846f*77.0f)+0.5f);
 return 1;
}
uint32_t CAN1_SetMotorEnable(uint8_t a,uint8_t b,CanRxMsg*r){return 1;}
uint32_t CAN1_SetEmmSpeedRaw(uint8_t a,uint8_t b,uint16_t v,uint8_t c,CanRxMsg*r)
{
 uint32_t w=a-1;CHECK(test_depth==0);CHECK(v<=54);CHECK(c==0);
 target[w]=(float)v;calls[w]++;if(v>peak_command[w])peak_command[w]=v;
 return 1;
}
uint32_t CAN1_StopMotor(uint8_t a,CanRxMsg*r)
{
 CHECK(test_depth==0);stop_calls[a-1]++;
 if(scenario==5 && a==2)return 0;
 target[a-1]=0;return 1;
}
static void begin(void)
{
 ChassisAutoPreparedTranslation p;uint32_t w,j;MotionProfileSample left,right;
 test_tick=1000;test_depth=test_primask=0;
 g_rtos_fault=g_safety_fault=g_can_ctrl_fault_latched=0;g_safety_ready=g_can_guard_ready=1;
 g_motion_diag_fault_latched=g_motion_auto_stop_state=g_motion_auto_stop_cut_vm=0;
 g_wheel_test_active=g_wheel_test_cut_vm=0;
 for(w=0;w<4;w++){
  position[w]=velocity[w]=target[w]=0;peak_command[w]=calls[w]=stop_calls[w]=0;
  for(j=0;j<128;j++)history[w][j]=0;
 }
 ChassisAuto_Init();CHECK(ChassisAuto_BeginSequence()==1);ChassisAuto_SafetyUpdate(1);
 CHECK(ChassisAuto_PrepareTranslation(0,52,25000,2100,&p)==1);
 CHECK(fabsf(p.profile.peak_speed-200.0f)<0.01f);
 /* Position, speed and acceleration must meet continuously at phase boundaries.
  * Jerk is allowed to step in a seven-phase S curve; only its magnitude is bounded. */
 for(j=1;j<MOTION_PROFILE_PHASES;j++){
  float t=p.profile.boundary_s[j];
  if(t<0.0001f)continue;
  CHECK(MotionProfile_Sample(&p.profile,t-0.0001f,&left)!=0);
  CHECK(MotionProfile_Sample(&p.profile,t+0.0001f,&right)!=0);
  CHECK(fabsf(right.position_mm-left.position_mm)<=0.051f);
  CHECK(fabsf(right.speed_mm_s-left.speed_mm_s)<=0.016f);
  CHECK(fabsf(right.acceleration_mm_s2-left.acceleration_mm_s2)<=0.017f);
 }
 CHECK(MotionProfile_Sample(&p.profile,p.profile.duration_s,&right)==MOTION_PROFILE_DONE);
 CHECK(right.position_mm==2100.0f && right.speed_mm_s==0 && right.acceleration_mm_s2==0);
 CHECK(ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session)==1);
 /* Isolate RUN/STOP behavior. VM startup/enable confirmation is outside this adapter. */
 g_chassis_auto_state=CHASSIS_AUTO_RUN;s_run_started=1;s_run_tick=s_profile_follow_tick=test_tick;
 s_status_valid=s_speed_valid=s_position_valid=15;s_endpoints[s_endpoint_index].start_mask=15;
 for(w=0;w<4;w++){
  s_status[w]=1;s_status_tick[w]=s_speed_tick[w]=s_position_tick[w]=test_tick;
  s_position.sign[w]=g_chassis_forward_direction[w];s_position.raw[w]=0;
  s_distance_seen_ms[w]=test_tick;s_distance_seen_raw[w]=0;s_distance_seen_sign[w]=g_chassis_forward_direction[w];
 }
}
uint32_t test_entry(void)
{
 uint32_t w,k;
 { MotionAction actions[8];uint32_t count=0;
   CHECK(AppDebugPlan_Load(7,actions,&count)==1 && count==1);
   CHECK(actions[0].kind==MOTION_FORWARD_DISTANCE && actions[0].base_rpm==52 && actions[0].distance_mm==2100);
   CHECK(AppDebugPlan_Load(5,actions,&count)==0 && AppDebugPlan_Load(8,actions,&count)==0);
   for(k=1;k<=3;k++){CHECK(AppDebugPlan_Load(k,actions,&count)==1 && count==1);CHECK(actions[0].base_rpm==15 && actions[0].distance_mm==300);}
   CHECK(AppDebugPlan_Load(4,actions,&count)==1 && count==3);
   CHECK(AppDebugPlan_Load(6,actions,&count)==1 && count==1 && actions[0].distance_mm==2100 && actions[0].base_rpm==15);
 }
 for(scenario=0;scenario<10;scenario++){
  begin();
  for(step=0;step<3000 && g_chassis_auto_active;step++){
   test_tick=1000+step*10;
   for(w=0;w<4;w++){
    uint32_t lag=(scenario==0 ? 0 : w*2),index=(step+127-lag)%128;
    float demand=step>lag ? history[w][index] : 0;
    float gain=(scenario==2 && w==2) ? 0.8f : 1.0f;
    float tau=scenario==0 ? 0.02f : 0.02f+0.01f*(float)w;
    velocity[w]+=(demand*gain-velocity[w])*(0.01f/tau);
    position[w]+=velocity[w]*(3.14159265358979323846f*77.0f/60.0f)*0.01f;
   }
   if(!(scenario==8 && step>=300))s_heading_command_tick=test_tick;
   if(scenario==5 && step==300)CHECK(ChassisAuto_RequestStop()==1);
   if(scenario==9 && step==300)CHECK(ChassisAuto_RequestStop()==1);
   /* Safety runs independently even when the CAN task is deliberately stalled. */
   Auto_CheckLocked(test_tick);
   if(!(scenario==9 && step>=300))ChassisAuto_CanStep();
   for(w=0;w<4;w++)history[w][step%128]=target[w];
   CHECK(test_depth==0);CHECK(s_translation_base_rpm<=52);
   record_trace();
  }
  results[scenario][0]=scenario;results[scenario][1]=g_chassis_auto_result;
  results[scenario][2]=g_chassis_auto_cause;results[scenario][3]=g_chassis_auto_fault_diagnostic.reason;
  results[scenario][4]=g_chassis_auto_fault_diagnostic.address;results[scenario][5]=s_distance_fault_reason;
  results[scenario][6]=s_distance_reached;results[scenario][7]=g_chassis_auto_stop_mask[2];
  results[scenario][8]=g_chassis_auto_cut_vm;results[scenario][9]=(step ? step-1 : 0)*10;
  for(w=0;w<4;w++){results[scenario][10+w]=(uint32_t)(position[w]*1000);results[scenario][14+w]=peak_command[w];}
  results[scenario][18]=checks;
  CHECK(step<3000);
  if(scenario<2){CHECK(g_chassis_auto_result==1);CHECK(s_distance_reached);CHECK(g_chassis_auto_stop_mask[2]==15);}
  else {
   CHECK(g_chassis_auto_result==2);CHECK(g_chassis_auto_cut_vm==1);CHECK(!s_distance_reached);
   if(scenario==2)CHECK(s_distance_fault_reason==5);
   if(scenario==3)CHECK(g_chassis_auto_cause==3 && g_chassis_auto_fault_diagnostic.address==3);
   if(scenario==4)CHECK(g_chassis_auto_cause==3 && g_chassis_auto_fault_diagnostic.address==1);
   if(scenario==5)CHECK(g_chassis_auto_cause==5);
   if(scenario==6 || scenario==7)CHECK(g_chassis_auto_cause==8);
   if(scenario==8)CHECK(g_chassis_auto_fault_diagnostic.reason==1);
   if(scenario==9)CHECK(g_chassis_auto_cause==6 && test_tick<=4510);
  }
 }
 return first_failure ? 0x80000000U|first_failure : checks;
}
