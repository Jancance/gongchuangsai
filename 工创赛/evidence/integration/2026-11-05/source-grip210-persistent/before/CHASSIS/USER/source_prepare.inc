/* 仅由vehicle_arm.c包含，所有CAN/PWM仍由原唯一CAN任务串行服务。 */
#include "chassis_auto_test.h"
#include "source_prepare_math.h"
volatile uint32_t g_source_prepare_state=0U,g_source_prepare_cause=0U,g_source_prepare_done_mask=0U;
volatile uint32_t g_source_prepare_release_ms=0U,g_source_prepare_elapsed_ms=0U,g_source_prepare_segment=0U;
volatile float g_source_prepare_extension_mm=0,g_source_prepare_grip_deg=STARTUP_GROUP_GRIPPER_ANGLE_DEG;
volatile SourcePrepFault g_source_prepare_fault;
volatile SourcePrepTrace g_source_prepare_trace[SOURCE_PREP_TRACE_CAPACITY];
volatile uint32_t g_source_prepare_trace_count,g_source_prepare_trace_dropped;
volatile uint32_t g_source_prepare_command_units,g_source_prepare_command_options;
volatile int32_t g_source_prepare_command_before_mdeg,g_source_prepare_command_goal_mdeg;
volatile int32_t g_source_prepare_driver_target_mdeg;

#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t sp_begin,sp_phase,sp_dm_cycle,sp_samples,sp_grip_tick;
static uint32_t sp_bad_pos,sp_dm_posted,sp_read_wait,sp_read_begin,sp_position_tick,sp_last_command,sp_last_result;
static SourcePrepFault sp_bad_snapshot;
static uint32_t sp_settle_begin,sp_settle_samples,sp_target_pending;
/* Keep the existing envelope; sample count is not a time measurement. */
#define SOURCE_PREP_POS_TOL_DEG 10.0f
static float sp_origin,sp_goal,sp_before;
static int32_t SourcePrep_Mdeg(float v)
{return v>=-2000000.0f && v<=2000000.0f?(int32_t)(v*1000.0f):INT32_MIN;}
static SourcePrepFault SourcePrep_Snapshot(uint32_t why)
{
 SourcePrepFault f;uint32_t i;memset(&f,0,sizeof(f));
 f.reason=why;f.tick=(uint32_t)xTaskGetTickCount();f.phase=sp_phase;f.segment=g_source_prepare_segment;
 f.command=sp_last_command;f.result=sp_last_result;f.rx_id=s_rx.ExtId;f.dlc=s_rx.DLC;
 for(i=0;i<4;i++){f.raw_hi=(f.raw_hi<<8)|s_rx.Data[i];f.raw_lo=(f.raw_lo<<8)|s_rx.Data[i+4];}
 f.bad_count=sp_bad_pos;f.position_age=f.tick-sp_position_tick;
 f.origin_mdeg=SourcePrep_Mdeg(sp_origin);f.current_mdeg=SourcePrep_Mdeg(s_current);f.goal_mdeg=SourcePrep_Mdeg(sp_goal);
 return f;
}
static void SourcePrep_RecordFault(uint32_t why)
{
 SourcePrepFault f;
 if(g_source_prepare_fault.reason)return;
 f=(why==23U && sp_bad_snapshot.reason)?sp_bad_snapshot:SourcePrep_Snapshot(why);
 f.reason=0U;g_source_prepare_fault=f;g_source_prepare_fault.reason=why;
}
static uint32_t SourcePrep_Exchange(const uint8_t *q,uint8_t n,uint8_t rn)
{
 if(n==15U && q[0]==0xfdU){
  uint32_t part,i,j;SourcePrepTrace t;
  for(part=0U;part<2U;part++){
   memset(&t,0,sizeof(t));t.tick=(uint32_t)xTaskGetTickCount();t.phase=sp_phase;t.segment=g_source_prepare_segment;
   t.command=0x100U+part*0x100U+q[0];
   for(j=0;j<8;j++){uint8_t v=(part*8U+j<n)?q[part*8U+j]:0U;if(j<4U)t.raw_hi=(t.raw_hi<<8)|v;else t.raw_lo=(t.raw_lo<<8)|v;}
   t.options=s_options;t.units=g_source_prepare_command_units;
   t.before_mdeg=g_source_prepare_command_before_mdeg;t.goal_mdeg=g_source_prepare_command_goal_mdeg;t.driver_mdeg=g_source_prepare_driver_target_mdeg;
   i=g_source_prepare_trace_count;if(i<SOURCE_PREP_TRACE_CAPACITY){g_source_prepare_trace[i]=t;g_source_prepare_trace_count=i+1U;}else g_source_prepare_trace_dropped++;
  }
 }
 memset(&s_rx,0,sizeof(s_rx));sp_last_command=q[0];
 sp_last_result=VehicleArm_Exchange(6U,q,n,rn,&s_rx);
 {SourcePrepTrace t;uint32_t i;t.tick=(uint32_t)xTaskGetTickCount();t.phase=sp_phase;t.segment=g_source_prepare_segment;
  t.command=q[0];t.result=sp_last_result;t.raw_hi=t.raw_lo=0U;
  for(i=0;i<4;i++){t.raw_hi=(t.raw_hi<<8)|s_rx.Data[i];t.raw_lo=(t.raw_lo<<8)|s_rx.Data[i+4];}
  t.options=s_options;t.units=g_source_prepare_command_units;
  t.before_mdeg=g_source_prepare_command_before_mdeg;t.goal_mdeg=g_source_prepare_command_goal_mdeg;t.driver_mdeg=g_source_prepare_driver_target_mdeg;
  i=g_source_prepare_trace_count;
  if(i<SOURCE_PREP_TRACE_CAPACITY){g_source_prepare_trace[i]=t;g_source_prepare_trace_count=i+1U;}
  else g_source_prepare_trace_dropped++;
 }
 return sp_last_result;
}
static void SourcePrep_Fail(uint32_t why);
static uint32_t SourcePrep_Query(uint8_t command,uint8_t n)
{
 uint8_t q[2]={command,0x6b};uint32_t now;
 if(SourcePrep_Exchange(q,2U,n)==1U){sp_read_wait=0U;return 1U;}
 sp_samples=0U;now=(uint32_t)xTaskGetTickCount();
 if(!sp_read_wait){sp_read_wait=1U;sp_read_begin=now;}
 if(now-sp_read_begin>=400U)SourcePrep_Fail(26U);
 return 0U;
}
static uint32_t SourcePrep_Position(void)
{
 uint32_t raw;
 if(!SourcePrep_Query(0x36,7U))return 0U;
 if(s_rx.Data[1]>1U){SourcePrep_Fail(27U);return 0U;}
 raw=((uint32_t)s_rx.Data[2]<<24)|((uint32_t)s_rx.Data[3]<<16)|((uint32_t)s_rx.Data[4]<<8)|s_rx.Data[5];
 s_current=(s_rx.Data[1]?-1.0f:1.0f)*(float)raw/((s_options&0x80U)?100.0f:10.0f);return 1U;
}
static uint32_t SourcePrep_StartPositionValid(void)
{
 float travel=(s_current-sp_origin)*STARTUP_EXTENSION_EXTEND_SIGN;
 float first=VEHICLE_SOURCE_PREP_EXTENSION_SEGMENT_MM*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f;
 /* Rebound inside the known first segment does not change the homing origin. */
 return travel>=-SOURCE_PREP_POS_TOL_DEG && travel<=first && fabsf(s_current-sp_goal)<=180.0f;
}
static uint32_t SourcePrep_Move(void)
{
 uint32_t units,speed;float delta=sp_goal-s_current;uint8_t move[15]={0};
 if(!(fabsf(delta)<=180.0f && fabsf(sp_goal-sp_origin)<=252.1f))return 0;
 /* Ruler + independent 0x31 on 2026-11-05 proves node6 Scale-enabled
  * command AND 0x33/0x36 feedback use 100 raw units per degree. */
 units=(uint32_t)(fabsf(delta)*((s_options&0x80U)?100.0f:10.0f)+.5f);
 speed=(uint32_t)(VEHICLE_SOURCE_PREP_EXTENSION_RPM*10.0f+.5f);
 move[0]=0xfd;move[1]=delta<0;move[2]=move[4]=VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S>>8;
 move[3]=move[5]=VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S&255;
 move[6]=speed>>8;move[7]=speed&255;move[8]=units>>24;move[9]=units>>16;
 move[10]=units>>8;move[11]=units;move[12]=2;move[14]=0x6b;
 sp_before=s_current;sp_samples=0;sp_target_pending=1U;
 g_source_prepare_command_units=units;g_source_prepare_command_options=s_options;
 g_source_prepare_command_before_mdeg=SourcePrep_Mdeg(s_current);g_source_prepare_command_goal_mdeg=SourcePrep_Mdeg(sp_goal);
 return SourcePrep_Exchange(move,15,3)==1 && (s_rx.Data[1]==2 || s_rx.Data[1]==0x9f);
}
static void SourcePrep_Fail(uint32_t why)
{SourcePrep_RecordFault(why);g_source_prepare_cause=why;g_source_prepare_state=4;g_dm_source_go=0;arm_fail(why);}
static void SourcePrep_Service(void)
{
 uint32_t now=(uint32_t)xTaskGetTickCount();float ratio,angle,delta;uint8_t enable[5]={0xf3,0xab,1,0,0x6b};
 g_source_prepare_elapsed_ms=now-sp_begin;
 if(!StartupAxes_Holding() || now-sp_begin>=VEHICLE_SOURCE_PREP_TIMEOUT_MS){SourcePrep_Fail(20);return;}
 if(g_dm_jog_first_fault || (g_dm_jog_state==8 && g_dm_jog_result!=2)){SourcePrep_Fail(21);return;}
 s_address=6;
 if(g_source_prepare_state==1){
  switch(sp_phase){
  case 0:
   if(!sp_dm_posted){
   g_vehicle_arm_dm_owned=g_dm_integrated_owner=1;g_vehicle_arm_axis_owned=6;
   g_dm_source_smooth=1;g_dm_source_go=0;g_dm_source_accel_deg_s2=VEHICLE_SOURCE_PREP_DM_ACCEL_DEG_S2;
   g_dm_point_enable=1;g_dm_posvel_speed_deg_s=VEHICLE_SOURCE_PREP_DM_SPEED_DEG_S;
   g_dm_pick_target_deg=g_dm_place_target_deg=VEHICLE_SOURCE_PREP_DM_DEG;
   g_dm_jog_request=VEHICLE_SOURCE_PREP_DM_DEG>=g_dm_can2_logical_position_deg?4:5;
   sp_dm_cycle=g_dm_can2_read_cycle;sp_dm_posted=1U;
   }
if(!SourcePrep_Query(0x1a,4))return; if((s_rx.Data[2]&3)){SourcePrep_Fail(22);return;}
   s_options=s_rx.Data[2];sp_phase=1;return;
  case 1:
if(!SourcePrep_Position())return; if(!SourcePrep_StartPositionValid()){SourcePrep_Fail(23);return;}
   sp_goal=sp_origin+STARTUP_EXTENSION_EXTEND_SIGN*VEHICLE_SOURCE_PREP_EXTENSION_SEGMENT_MM*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f;
   sp_phase=2;return;
  case 2:
   if(SourcePrep_Exchange(enable,5,3)!=1 || s_rx.Data[1]!=2){SourcePrep_Fail(22);return;}
   sp_settle_begin=(uint32_t)xTaskGetTickCount();sp_settle_samples=0U;
   sp_phase=3;return;
  case 3:
   if(now-sp_settle_begin>=2000U){SourcePrep_Fail(28U);return;}
if(!SourcePrep_Query(0x3a,3))return; if(!(s_rx.Data[1]&1) || (s_rx.Data[1]&0x0c)){SourcePrep_Fail(24);return;}
   sp_phase=4;return;
  case 4:
if(!SourcePrep_Query(0x35,5)){sp_settle_samples=0U;return;}
   if(now-sp_settle_begin>=2000U){SourcePrep_Fail(28U);return;}
   if(s_rx.Data[1]>1U){SourcePrep_Fail(27U);return;}
   if(s_rx.Data[2] || s_rx.Data[3])sp_settle_samples=0U;else sp_settle_samples++;
   sp_phase=sp_settle_samples>=3U?5U:3U;return;
  default:
   if(g_dm_jog_state!=3 || g_dm_can2_read_cycle==sp_dm_cycle || now-g_dm_can2_feedback_tick>=100)return;
if(!SourcePrep_Position())return; if(!SourcePrep_StartPositionValid()){SourcePrep_Fail(23);return;}
   /* 同一服务轮发布DM放行/PWM起点/伸缩命令，不宣称硬件同时起动。 */
   g_source_prepare_release_ms=sp_grip_tick=now;g_dm_source_go=1;
   if(!jiazhua(STARTUP_GROUP_GRIPPER_ANGLE_DEG) || !SourcePrep_Move()){SourcePrep_Fail(22);return;}
   g_source_prepare_segment=1;g_source_prepare_state=2;sp_phase=6;sp_position_tick=(uint32_t)xTaskGetTickCount();return;
  }
 }
 ratio=(float)(now-g_source_prepare_release_ms)/(float)VEHICLE_SOURCE_PREP_GRIP_RAMP_MS;
 angle=STARTUP_GROUP_GRIPPER_ANGLE_DEG+(VEHICLE_SOURCE_PREP_GRIP_DEG-STARTUP_GROUP_GRIPPER_ANGLE_DEG)*SourcePrep_Smooth(ratio);
 if(now-sp_grip_tick>=20){if(!jiazhua(angle)){SourcePrep_Fail(25);return;}g_source_prepare_grip_deg=angle;sp_grip_tick=now;}
 if(now-g_source_prepare_release_ms>=VEHICLE_SOURCE_PREP_GRIP_RAMP_MS+VEHICLE_SOURCE_PREP_GRIP_SETTLE_MS)
  g_source_prepare_done_mask|=4; /* 舵机只有命令及等待，没有真实到位反馈。 */
 if(g_dm_can2_read_cycle!=sp_dm_cycle && g_dm_jog_state==7 && g_dm_jog_result==1 &&
    g_dm_posvel_holding && now-g_dm_can2_feedback_tick<100 && fabsf(g_dm_can2_logical_position_deg-VEHICLE_SOURCE_PREP_DM_DEG)<=3)
  g_source_prepare_done_mask|=1;
 if(!(g_source_prepare_done_mask&2)){
  if(sp_target_pending){
   uint32_t raw;float target;
   if(!SourcePrep_Query(0x33,7))return;
   if(s_rx.Data[1]>1U){SourcePrep_Fail(27U);return;}
   raw=((uint32_t)s_rx.Data[2]<<24)|((uint32_t)s_rx.Data[3]<<16)|((uint32_t)s_rx.Data[4]<<8)|s_rx.Data[5];
   target=(s_rx.Data[1]?-1.0f:1.0f)*(float)raw/((s_options&0x80U)?100.0f:10.0f);
   g_source_prepare_driver_target_mdeg=SourcePrep_Mdeg(target);sp_target_pending=0U;
   if(!(fabsf(target-sp_goal)<=SOURCE_PREP_POS_TOL_DEG)){SourcePrep_Fail(29U);return;}
   return;
  }
  if(now-sp_position_tick>=400U){SourcePrep_Fail(sp_bad_pos?23U:26U);return;}
  if(sp_phase==6){
if(!SourcePrep_Query(0x3a,3))return; if(!(s_rx.Data[1]&1) || (s_rx.Data[1]&0x0c)){SourcePrep_Fail(24);return;}
   s_flags=s_rx.Data[1];sp_phase=7;return;
  }
  if(sp_phase==7){
   if(!SourcePrep_Position())return;
   if(s_current<fminf(sp_before,sp_goal)-SOURCE_PREP_POS_TOL_DEG || s_current>fmaxf(sp_before,sp_goal)+SOURCE_PREP_POS_TOL_DEG){
    sp_samples=0U;sp_bad_pos++;
    if(!sp_bad_snapshot.reason)sp_bad_snapshot=SourcePrep_Snapshot(23U);
    if(sp_bad_pos>=8U){SourcePrep_Fail(23);return;}
    sp_phase=6U;return; /* Recheck driver health before another position sample. */
   }
   sp_bad_pos=0U;memset(&sp_bad_snapshot,0,sizeof(sp_bad_snapshot));sp_position_tick=(uint32_t)xTaskGetTickCount();
   delta=(s_current-sp_origin)*STARTUP_EXTENSION_EXTEND_SIGN;
   g_source_prepare_extension_mm=delta*3200.0f/(VEHICLE_EXTENSION_PULSES_PER_MM*360.0f);sp_phase=8;return;
  }
if(!SourcePrep_Query(0x35,5))return; /* 读失败重试 */
  if(!s_rx.Data[2] && !s_rx.Data[3] && (s_flags&2) && fabsf(s_current-sp_goal)<=.3f)sp_samples++;else sp_samples=0;
  if(sp_samples>=3){
   if(g_source_prepare_segment==1){
    sp_goal=sp_origin+STARTUP_EXTENSION_EXTEND_SIGN*VEHICLE_SOURCE_PREP_EXTENSION_MM*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f;
    if(!SourcePrep_Move()){SourcePrep_Fail(22);return;}g_source_prepare_segment=2;
   }else {g_source_prepare_done_mask|=2;StartupAxes_SetExtensionHoldTarget(sp_goal);}
  }
  sp_phase=6;
 }
 if(g_source_prepare_done_mask==7){g_source_prepare_state=3;g_vehicle_arm_state=2;g_vehicle_arm_axis_owned=g_vehicle_arm_dm_owned=0;g_dm_source_smooth=0;}
}
#else
static void SourcePrep_RecordFault(uint32_t why){(void)why;}
#endif
uint32_t VehicleArm_BeginSourcePreparation(void)
{
#if APP_MOTOR_OFFLINE_ONLY
 return 0;
#else
 ChassisAutoStatus status;
 if(VehicleArm_Busy() || !StartupAxes_Holding() || !g_startup_extension_feedback_valid || !g_extension_hold_confirmed)return 0;
 (void)ChassisAuto_ReadStatus(&status);
 if(status.active || status.result!=1 || status.cause || status.rotation_stop_mask!=15)return 0;
 if(!(VEHICLE_SOURCE_PREP_EXTENSION_MM>0 && VEHICLE_SOURCE_PREP_EXTENSION_MM<=70 &&
      VEHICLE_SOURCE_PREP_EXTENSION_SEGMENT_MM>0 && VEHICLE_SOURCE_PREP_EXTENSION_SEGMENT_MM<=35 &&
      VEHICLE_SOURCE_PREP_EXTENSION_MM<=2*VEHICLE_SOURCE_PREP_EXTENSION_SEGMENT_MM &&
      VEHICLE_EXTENSION_PULSES_PER_MM==32.0f &&
      VEHICLE_SOURCE_PREP_DM_DEG==180 && VEHICLE_SOURCE_PREP_DM_SPEED_DEG_S>0 && VEHICLE_SOURCE_PREP_DM_SPEED_DEG_S<=150 &&
      VEHICLE_SOURCE_PREP_EXTENSION_RPM>0 && VEHICLE_SOURCE_PREP_EXTENSION_RPM<=30 &&
      VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S>=1 && VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S<=300 &&
      VEHICLE_SOURCE_PREP_GRIP_DEG>=0 && VEHICLE_SOURCE_PREP_GRIP_DEG<=180 &&
      VEHICLE_SOURCE_PREP_GRIP_RAMP_MS>=500 && VEHICLE_SOURCE_PREP_GRIP_RAMP_MS<=3000 &&
      VEHICLE_SOURCE_PREP_TIMEOUT_MS>=5000 && VEHICLE_SOURCE_PREP_TIMEOUT_MS<=15000))return 0;
 /* Publish only: all bus transactions remain in VehicleArm_Service / CAN task. */
 if(!g_extension_home_reference_valid)return 0;
 sp_origin=g_extension_home_reference_deg;
 if(!(sp_origin>=-100000 && sp_origin<=100000))return 0;
 sp_goal=sp_origin+STARTUP_EXTENSION_EXTEND_SIGN*VEHICLE_SOURCE_PREP_EXTENSION_SEGMENT_MM*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f;
 sp_dm_posted=sp_read_wait=sp_target_pending=0U;
 g_source_prepare_trace_count=g_source_prepare_trace_dropped=0U;
 g_source_prepare_command_units=g_source_prepare_command_options=0U;
 g_source_prepare_driver_target_mdeg=INT32_MIN;sp_position_tick=(uint32_t)xTaskGetTickCount();
 sp_last_command=sp_last_result=0U;memset(&sp_bad_snapshot,0,sizeof(sp_bad_snapshot));
 memset((void *)&g_source_prepare_fault,0,sizeof(g_source_prepare_fault));
 sp_begin=(uint32_t)xTaskGetTickCount();sp_phase=sp_samples=0;
 g_source_prepare_cause=g_source_prepare_done_mask=g_source_prepare_segment=g_source_prepare_release_ms=sp_bad_pos=0;
 g_source_prepare_extension_mm=0;g_source_prepare_grip_deg=STARTUP_GROUP_GRIPPER_ANGLE_DEG;
 g_source_prepare_state=1;g_vehicle_arm_cause=0;
 g_vehicle_arm_state=1;return 1; /* Publish busy last, after the request is initialized. */
#endif
}
