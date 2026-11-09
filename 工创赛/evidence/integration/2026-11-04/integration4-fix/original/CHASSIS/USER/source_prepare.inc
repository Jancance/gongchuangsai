/* 仅由vehicle_arm.c包含，所有CAN/PWM仍由原唯一CAN任务串行服务。 */
#include "chassis_auto_test.h"
#include "source_prepare_math.h"
volatile uint32_t g_source_prepare_state=0U,g_source_prepare_cause=0U,g_source_prepare_done_mask=0U;
volatile uint32_t g_source_prepare_release_ms=0U,g_source_prepare_elapsed_ms=0U,g_source_prepare_segment=0U;
volatile float g_source_prepare_extension_mm=0,g_source_prepare_grip_deg=STARTUP_GROUP_GRIPPER_ANGLE_DEG;
#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t sp_begin,sp_phase,sp_dm_cycle,sp_samples,sp_grip_tick;
static uint32_t sp_bad_pos;
/* 伸缩位置反馈有毛刺：单次越界先忽略，连续 8 次(约400ms)才判故障 */
#define SOURCE_PREP_POS_TOL_DEG 10.0f
static float sp_origin,sp_goal,sp_before;
static uint32_t SourcePrep_Move(void)
{
 uint32_t units,speed;float delta=sp_goal-s_current;uint8_t move[15]={0};
 if(!(fabsf(delta)<=180.0f && fabsf(sp_goal-sp_origin)<=252.1f))return 0;
 units=(uint32_t)(fabsf(delta)*((s_options&0x80U)?100.0f:10.0f)+.5f);
 speed=(uint32_t)(VEHICLE_SOURCE_PREP_EXTENSION_RPM*10.0f+.5f);
 move[0]=0xfd;move[1]=delta<0;move[2]=move[4]=VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S>>8;
 move[3]=move[5]=VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S&255;
 move[6]=speed>>8;move[7]=speed&255;move[8]=units>>24;move[9]=units>>16;
 move[10]=units>>8;move[11]=units;move[12]=2;move[14]=0x6b;
 sp_before=s_current;sp_samples=0;
 return VehicleArm_Exchange(6,move,15,3,&s_rx)==1 && (s_rx.Data[1]==2 || s_rx.Data[1]==0x9f);
}
static void SourcePrep_Fail(uint32_t why)
{g_source_prepare_cause=why;g_source_prepare_state=4;g_dm_source_go=0;arm_fail(why);}
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
   g_vehicle_arm_dm_owned=g_dm_integrated_owner=1;g_vehicle_arm_axis_owned=6;
   g_dm_source_smooth=1;g_dm_source_go=0;g_dm_source_accel_deg_s2=VEHICLE_SOURCE_PREP_DM_ACCEL_DEG_S2;
   g_dm_point_enable=1;g_dm_posvel_speed_deg_s=VEHICLE_SOURCE_PREP_DM_SPEED_DEG_S;
   g_dm_pick_target_deg=g_dm_place_target_deg=VEHICLE_SOURCE_PREP_DM_DEG;
   g_dm_jog_request=VEHICLE_SOURCE_PREP_DM_DEG>=g_dm_can2_logical_position_deg?4:5;
   sp_dm_cycle=g_dm_can2_read_cycle;
if(!query(0x1a,4))return; if((s_rx.Data[2]&3)){SourcePrep_Fail(22);return;}
   s_options=s_rx.Data[2];sp_phase=1;return;
  case 1:
if(!position())return; if(fabsf(s_current-sp_origin)>SOURCE_PREP_POS_TOL_DEG){SourcePrep_Fail(23);return;}
   sp_goal=sp_origin+STARTUP_EXTENSION_EXTEND_SIGN*VEHICLE_SOURCE_PREP_EXTENSION_SEGMENT_MM*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f;
   sp_phase=2;return;
  case 2:
   if(VehicleArm_Exchange(6,enable,5,3,&s_rx)!=1 || s_rx.Data[1]!=2){SourcePrep_Fail(22);return;}
   sp_phase=3;return;
  case 3:
if(!query(0x3a,3))return; if(!(s_rx.Data[1]&1) || (s_rx.Data[1]&0x0c)){SourcePrep_Fail(24);return;}
   sp_phase=4;return;
  case 4:
if(!query(0x35,5))return; if(s_rx.Data[2] || s_rx.Data[3]){SourcePrep_Fail(24);return;}
   sp_phase=5;return;
  default:
   if(g_dm_jog_state!=3 || g_dm_can2_read_cycle==sp_dm_cycle || now-g_dm_can2_feedback_tick>=100)return;
if(!position())return; if(fabsf(s_current-sp_origin)>SOURCE_PREP_POS_TOL_DEG){SourcePrep_Fail(23);return;}
   /* 同一服务轮发布DM放行/PWM起点/伸缩命令，不宣称硬件同时起动。 */
   g_source_prepare_release_ms=sp_grip_tick=now;g_dm_source_go=1;
   if(!jiazhua(STARTUP_GROUP_GRIPPER_ANGLE_DEG) || !SourcePrep_Move()){SourcePrep_Fail(22);return;}
   g_source_prepare_segment=1;g_source_prepare_state=2;sp_phase=6;return;
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
  if(sp_phase==6){
if(!query(0x3a,3))return; if(!(s_rx.Data[1]&1) || (s_rx.Data[1]&0x0c)){SourcePrep_Fail(24);return;}
   s_flags=s_rx.Data[1];sp_phase=7;return;
  }
  if(sp_phase==7){
   if(!position())return;
   if(s_current<fminf(sp_before,sp_goal)-SOURCE_PREP_POS_TOL_DEG || s_current>fmaxf(sp_before,sp_goal)+SOURCE_PREP_POS_TOL_DEG){
    /* 位置毛刺：忽略本样本继续采样，连续多次异常才判故障 */
    if(++sp_bad_pos>=8U){SourcePrep_Fail(23);return;}
    return;
   }
   sp_bad_pos=0U;
   delta=(s_current-sp_origin)*STARTUP_EXTENSION_EXTEND_SIGN;
   g_source_prepare_extension_mm=delta*3200.0f/(VEHICLE_EXTENSION_PULSES_PER_MM*360.0f);sp_phase=8;return;
  }
if(!query(0x35,5))return; /* 读失败重试 */
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
 /* 基准必须取"源区准备开始时的当前位置"，不能用回零瞬间的旧值：
  * 回零后的保持期间机械会缓慢漂移，旧基准会让 case1 误报 cause 23。 */
 s_address=6U;
 if(!position())return 0;
 sp_origin=s_current;
 if(!(sp_origin>=-100000 && sp_origin<=100000))return 0;
 sp_begin=(uint32_t)xTaskGetTickCount();sp_phase=sp_samples=0;
 g_source_prepare_cause=g_source_prepare_done_mask=g_source_prepare_segment=g_source_prepare_release_ms=sp_bad_pos=0;
 g_source_prepare_extension_mm=0;g_source_prepare_grip_deg=STARTUP_GROUP_GRIPPER_ANGLE_DEG;
 g_source_prepare_state=g_vehicle_arm_state=1;g_vehicle_arm_cause=0;return 1;
#endif
}
