/* Included by vehicle_arm.c: sole CAN task owns all bus calls. No new task,
 * serial start command, key shortcut, power toggle or fabricated feedback. */
#include "source_parallel_flow.h"
#include "source_pick_extension_lead_20261115.h"
#include "chassis_auto_test.h"
volatile uint32_t g_source_parallel_state,g_source_parallel_cause,g_source_parallel_started_mask;
volatile uint32_t g_source_parallel_done_mask,g_source_parallel_stop_mask,g_source_parallel_start_skew_ms;
#if !APP_MOTOR_OFFLINE_ONLY
extern volatile uint32_t g_dm_parallel_group_active,g_dm_parallel_group_go,g_dm_stage2_sync;
extern volatile uint32_t g_dm_initial_observe;
extern volatile uint32_t g_extension_home_reference_valid;
extern volatile float g_extension_home_reference_deg;
typedef struct {uint32_t phase,options,samples,flags,cycle,stall_active,stall_ms,position_tick,position_cycle,overshoot_active,overshoot_ms;int64_t position_raw;float before,goal,current;uint8_t move[15];} VSP_Axis;
static VSP_Axis vsp_axis[2];
static SPF_Group vsp_group,vsp_carry_place;
static uint32_t vsp_carry_leg,vsp_carry_sent,vsp_lift_start_cycle;
static uint32_t vsp_carry_origin_sampled;
volatile uint32_t g_vsp_carry_first_stage,g_vsp_carry_first_node,g_vsp_carry_first_command;
volatile uint32_t g_vsp_carry_first_exchange,g_vsp_carry_first_flags,g_vsp_carry_first_speed_raw;
volatile uint32_t g_vsp_carry_first_broadcast_result,g_vsp_carry_first_sequence;
volatile uint32_t g_vsp_carry_wait_speed_count,g_vsp_carry_wait_speed_node,g_vsp_carry_wait_speed_raw;
enum {VSP_CARRY_DIAG_CACHE=1,VSP_CARRY_DIAG_FLAGS,VSP_CARRY_DIAG_SPEED,
      VSP_CARRY_DIAG_ORIGIN,VSP_CARRY_DIAG_BROADCAST,VSP_CARRY_DIAG_TIMEOUT};
static void VSP_CarryFirst(uint32_t stage,uint32_t node,uint32_t cmd,uint32_t exchange,
                           uint32_t flags,uint32_t speed,uint32_t broadcast)
{
 if(g_vsp_carry_first_stage)return;
 g_vsp_carry_first_node=node;g_vsp_carry_first_command=cmd;
 g_vsp_carry_first_exchange=exchange;g_vsp_carry_first_flags=flags;
 g_vsp_carry_first_speed_raw=speed;g_vsp_carry_first_broadcast_result=broadcast;
 g_vsp_carry_first_sequence=vsp_group.command.sequence;g_vsp_carry_first_stage=stage;
}
extern volatile uint32_t g_dm_stage3_carry_target_feedback;
static SPF_Group *VSP_StepperGroup(void)
{return vsp_carry_leg?&vsp_carry_place:&vsp_group;}
static uint32_t vsp_fast_lift;
static uint32_t vsp_pick_lead_ms;
static uint32_t vsp_pick_sync,vsp_pick_queued;
static uint32_t vsp_loaded_sync_sent,vsp_loaded_z_cycle,vsp_loaded_dm_stop;
extern volatile uint32_t g_dm_stage3_empty_target_sent;
volatile uint32_t g_source_parallel_sync_tick;

volatile uint32_t g_vsp_prepare_miss,g_vsp_prepare_nonzero,g_vsp_prepare_speed_raw,g_vsp_prepare_wait_address;
static uint32_t vsp_dm_posted,vsp_dm_cycle,vsp_owned,vsp_dm_hold_tick;
static uint32_t VSP_Query(uint8_t address,uint8_t command,uint8_t count)
{uint8_t q[2];q[0]=command;q[1]=0x6b;return VehicleArm_Exchange(address,q,2,count,&s_rx)==1;}
static uint32_t VSP_Position(uint8_t address,VSP_Axis *a)
{
 uint32_t raw;if(!VSP_Query(address,0x36,7) || s_rx.Data[1]>1)return 0;
 raw=((uint32_t)s_rx.Data[2]<<24)|((uint32_t)s_rx.Data[3]<<16)|((uint32_t)s_rx.Data[4]<<8)|s_rx.Data[5];
 /* 2026-11-08 (ZDT X42S manual 5.3.10/5.5.13): the X-firmware 0xFD 15-byte frame is
  * the trapezoid position mode and its position field is 0.1 deg, not pulses:
  * 3200 units = 320.0 deg. The 0x36 readback uses the same 0.1 deg scale, so the
  * old *360/3200 inflated every position by 1.125x. */
 a->current=(s_rx.Data[1]?-1.0f:1.0f)*(float)raw/
  ((address==5U && (a->options&0x80U))?100.0f:10.0f);
 a->position_raw=s_rx.Data[1]?-(int64_t)raw:(int64_t)raw;
 a->position_tick=(uint32_t)xTaskGetTickCount();a->position_cycle++;
 if(address==5){g_vehicle_lift_current_deg=a->current;g_vehicle_lift_feedback_valid=1;}
 else {g_startup_extension_current_deg=a->current;g_startup_extension_feedback_valid=1;}
 return 1;
}
extern uint32_t Stage3_FastTransferStopPending(void);
uint32_t VehicleArm_Stage3EmptyHandoffActive(void)
{return vsp_owned && vsp_group.command.kind==SPF_CMD_STAGE3_EMPTY_HANDOFF;}
uint32_t VehicleArm_Stage3CarryActive(void)
{return vsp_owned && vsp_group.command.kind==SPF_CMD_STAGE3_CARRY_RISE;}
uint32_t VehicleArm_Stage3EmptyTargetPosted(float goal)
{return (VehicleArm_Stage3EmptyHandoffActive() || VehicleArm_Stage3CarryActive()) &&
 vsp_dm_posted && goal==vsp_group.command.goal.dm_deg;}
uint32_t VehicleArm_Stage3GroupStopPending(void)
{
 return vsp_owned && (vsp_group.command.kind==SPF_CMD_STAGE3_LOADED_RETURN ||
                      vsp_group.command.kind==SPF_CMD_STAGE3_EMPTY_HANDOFF ||
                      vsp_group.command.kind==SPF_CMD_STAGE3_CARRY_RISE) &&
  (vsp_group.state==4U || !SingleTeach_ReferencesReady() || Stage3_FastTransferStopPending());
}
static uint32_t VSP_Stage3SyncStopPending(void)
{
 return (vsp_group.command.kind==SPF_CMD_STAGE3_FAST_TRANSFER && Stage3_FastTransferStopPending()) ||
  VehicleArm_Stage3GroupStopPending();
}
static uint32_t VSP_LoadedReturnHeightReady(uint32_t now)
{
 const VSP_Axis *z=&vsp_axis[1];
 uint32_t scale=g_lift_ruler_controller.top_scale;
 int64_t depth;
 /* 广播发送前的准备位置不算；用原始反馈比较机械顶部到当前位置。 */
 if(!vsp_loaded_sync_sent || z->position_cycle==vsp_loaded_z_cycle ||
    (uint32_t)(now-z->position_tick)>150U || (scale!=10U && scale!=100U))return 0U;
 depth=(z->position_raw-g_lift_ruler_controller.top_position_raw)*3200;
 if(VEHICLE_LIFT_DOWN_SIGN<0.0f)depth=-depth;
 return depth>=0 && depth<=(int64_t)2000U*360U*scale;
}
static uint32_t S3_TouchdownAllowed(void);
static uint32_t VSP_TouchdownDelta(float delta,float down_sign,uint32_t scale)
{
 float down=delta*down_sign;
 float max_deg=(float)SPF_STAGE3_TOUCHDOWN_MAX_PULSES*360.0f/3200.0f;
 /* The two driver scales are fixed by node-5 status. Reject rounding to zero. */
 return (scale==10U || scale==100U) && down>0.0f && down<=max_deg &&
  down*(float)scale>=0.5f;
}
static uint32_t VSP_Prepare(void *context,uint32_t bit,const SPF_Command *c)
{
 VSP_Axis *a;uint8_t address,enable[5]={0xf3,0xab,1,0,0x6b};uint32_t units,speed;float delta;
 (void)context;
 if(bit==SPF_DM){
  if(VehicleArm_Stage3GroupStopPending())return 2U;
  if(!vsp_dm_posted){
   if((c->kind==SPF_CMD_STAGE3_LOADED_RETURN || c->kind==SPF_CMD_STAGE3_CARRY_RISE) &&
      !VSP_LoadedReturnHeightReady((uint32_t)xTaskGetTickCount()))return 0U;
   if(!g_dm_can2_feedback_valid || (uint32_t)xTaskGetTickCount()-g_dm_can2_feedback_tick>=100)return 0;
   g_dm_parallel_group_active=1;g_dm_parallel_group_go=0;
   g_dm_source_smooth=0;g_dm_point_enable=1;g_dm_posvel_speed_deg_s=c->dm_deg_s;
   g_dm_pick_target_deg=g_dm_place_target_deg=c->goal.dm_deg;
   g_dm_jog_request=c->goal.dm_deg>=g_dm_can2_logical_position_deg?4:5;
   vsp_dm_cycle=g_dm_can2_read_cycle;vsp_dm_hold_tick=g_dm_can2_feedback_tick;vsp_dm_posted=1;return 0;
  }
  if(g_dm_jog_first_fault || (g_dm_jog_state==8 && g_dm_jog_result!=2))return 2;
  /* 常使能直改目标：请求已被CAN任务消费并进入运动态即放行，不再等使能步骤。 */
  if(g_dm_jog_request)return 0;
  if(g_dm_jog_state==4U || g_dm_jog_state==5U)return 1;
  if(g_dm_jog_state==7U && g_dm_posvel_holding && g_dm_jog_result==1U &&
     fabsf(g_dm_can2_logical_position_deg-c->goal.dm_deg)<=3.0f)return 1;
  if(g_dm_stage2_sync && g_dm_jog_state==3U && g_dm_can2_motor_state==1U &&
     g_dm_can2_feedback_tick!=vsp_dm_hold_tick && (uint32_t)xTaskGetTickCount()-g_dm_can2_feedback_tick<100U)return 1U;
  return g_dm_jog_state==3 && g_dm_can2_motor_state==1 &&
   g_dm_can2_read_cycle!=vsp_dm_cycle && (uint32_t)xTaskGetTickCount()-g_dm_can2_feedback_tick<100;
 }
 address=bit==SPF_EXT?6:5;a=&vsp_axis[bit==SPF_EXT?0:1];
 switch(a->phase){
 case 0:
  if(!VSP_Query(address,0x1a,4))return 0;
  if(s_rx.Data[2]&3)return 2;
  a->options=s_rx.Data[2];
  if((vsp_fast_lift || c->kind==SPF_CMD_STAGE3_TOUCHDOWN ||
      c->kind==SPF_CMD_STAGE3_LOADED_RETURN || c->kind==SPF_CMD_STAGE3_EMPTY_HANDOFF ||
      c->kind==SPF_CMD_STAGE3_CARRY_RISE || c->kind==SPF_CMD_STAGE3_CARRY_PLACE) && address==5U &&
   ((a->options&0x80U)?100U:10U)!=g_lift_ruler_controller.top_scale)return 2;
  a->phase=1;return 0;
 case 1:
  if(!VSP_Position(address,a))return 0;a->before=a->current;
  if(c->kind==SPF_CMD_STAGE3_CARRY_PLACE &&
    (address==6U?fabsf(a->current-g_extension_home_reference_deg)>SOURCE_EXTENSION_ZERO_DONE_DEG:
     fabsf(a->current-(float)g_lift_ruler_controller.safe_position_raw/
          (float)g_lift_ruler_controller.top_scale)>SOURCE_TOP_TOLERANCE_DEG))return 2U;
  a->goal=address==6?g_extension_home_reference_deg+STARTUP_EXTENSION_EXTEND_SIGN*c->goal.extension_mm*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f:
   SingleTeach_TopDeg()+VEHICLE_LIFT_DOWN_SIGN*c->goal.lift_pulses*360.0f/3200.0f;
  delta=a->goal-a->current;
  if(c->kind==SPF_CMD_STAGE3_TOUCHDOWN &&
     !VSP_TouchdownDelta(delta,VEHICLE_LIFT_DOWN_SIGN,(a->options&0x80U)?100U:10U))return 2;
  /* 2026-11-08: the old 612.1 deg cap was 170mm at the wrong 32 pulses/mm scale.
   * With the measured 294.4 pulses/mm a full 170mm stroke is ~5630 deg, so derive the
   * cap from the same constants instead of a hard-coded angle. */
  if(!(fabsf(delta)<=(address==6?(170.0f*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f+0.1f):((float)(c->lift_limit_pulses?c->lift_limit_pulses:SPF_MAX_LIFT_PULSES)*360.0f/3200.0f+0.1f))))return 2;
  /* 2026-11-08 (ZDT X42S manual 5.3.10): X-firmware trapezoid position mode takes the
   * position field in 0.1 deg, so 360.0 deg -> 3600. The previous deg*3200/360 sent
   * only 3200 for the same 360 deg target, i.e. 320.0 deg, an 11.1% shortfall. */
  units=(uint32_t)(fabsf(delta)*((address==5U && (a->options&0x80U))?100.0f:10.0f)+0.5f);
  speed=(uint32_t)((address==6?c->extension_rpm:c->lift_rpm)*10.0f+.5f);
  memset(a->move,0,15);a->move[0]=0xfd;a->move[1]=delta<0;
  a->move[2]=(uint8_t)(c->stepper_accel_rpm_s>>8);
  a->move[3]=(uint8_t)c->stepper_accel_rpm_s;
  a->move[4]=(uint8_t)(c->stepper_accel_rpm_s>>8);a->move[5]=(uint8_t)c->stepper_accel_rpm_s;
  if(vsp_fast_lift && address==5U){
   a->move[2]=a->move[4]=(uint8_t)(45055U>>8);
   a->move[3]=a->move[5]=(uint8_t)45055U;
  }
  a->move[6]=(uint8_t)(speed>>8);a->move[7]=(uint8_t)speed;
  a->move[8]=(uint8_t)(units>>24);a->move[9]=(uint8_t)(units>>16);a->move[10]=(uint8_t)(units>>8);a->move[11]=(uint8_t)units;
  a->move[12]=2;a->move[14]=0x6b;a->phase=2;return 0;
 case 2:
  if(!VSP_Query(address,0x35,5)){
   g_vsp_prepare_miss++;g_vsp_prepare_wait_address=address;a->cycle=1U;return 0;
  }
  if(s_rx.Data[1]>1U)return 2;
  g_vsp_prepare_speed_raw=((uint32_t)s_rx.Data[2]<<8)|s_rx.Data[3];
  if(g_vsp_prepare_speed_raw){
   g_vsp_prepare_nonzero++;g_vsp_prepare_wait_address=address;a->cycle=1U;return 0;
  }
  /* A wait may have changed the relative-move origin. Re-read it once,
   * rebuild the same physical goal, then require zero speed again. */
  if(a->cycle){a->cycle=0U;a->phase=1U;return 0;}
  a->phase=3;return 0;
 case 3:
  if(VehicleArm_Exchange(address,enable,5,3,&s_rx)!=1)return 0;
  if(s_rx.Data[1]!=2)return 2;
  a->phase=4;return 0;
 default:
  if(!VSP_Query(address,0x3a,3))return 0;
  if(!(s_rx.Data[1]&1) || (s_rx.Data[1]&0x08))return 2;
  if(s_rx.Data[1]&4){
   if(address==5)return 2;
   if(!a->stall_active){a->stall_active=1;a->stall_ms=(uint32_t)xTaskGetTickCount();}
   else if((uint32_t)xTaskGetTickCount()-a->stall_ms>=SOURCE_EXTENSION_STALL_CONFIRM_MS)return 2;
  }else a->stall_active=0;
  return 1;
 }
}
static uint32_t VSP_PositionWindow(VSP_Axis *a,uint32_t settle_allowed)
{
 float lo=fminf(a->before,a->goal),hi=fmaxf(a->before,a->goal);
 float corridor=a==&vsp_axis[0]?SOURCE_EXTENSION_CORRIDOR_DEG:45.0f;
 (void)settle_allowed;
 return a->current>=lo-corridor && a->current<=hi+corridor;
}
/* Both different goals are cached first. One CAN broadcast releases X/Z.
 * Success here means transmitted, never measured physical simultaneity. */
extern uint32_t Stage3_FastTransferStopPending(void);
static uint32_t VSP_StartQueuedPick(uint32_t bit,VSP_Axis *a,uint8_t address)
{
 uint32_t r;
 SPF_Group *g=VSP_StepperGroup();
 if(VSP_Stage3SyncStopPending())return 0U;
 a->move[13]=1U;
 r=VehicleArm_Exchange(address,a->move,15U,3U,&s_rx);
 if(g->command.kind==SPF_CMD_STAGE3_CARRY_PLACE &&
    ((r!=1U && !(address==6U && r==5U)) ||
     (r==1U && s_rx.Data[1]!=2U && s_rx.Data[1]!=0x9fU)) && !VehicleArm_Stage3GroupStopPending())
  VSP_CarryFirst(VSP_CARRY_DIAG_CACHE,address,0xfdU,r,r==1U?s_rx.Data[1]:0U,0U,0U);
 if(address==6U){
  if(r!=1U && r!=5U)return 0U;
  /* 回收可沿用无应答兼容，但驱动明确拒绝时不得广播。 */
  if((g->command.kind==SPF_CMD_STAGE3_LOADED_RETURN || g->command.kind==SPF_CMD_STAGE3_CARRY_PLACE) && r==1U &&
     s_rx.Data[1]!=2U && s_rx.Data[1]!=0x9fU)return 0U;
 }else if(r!=1U || (s_rx.Data[1]!=2U && s_rx.Data[1]!=0x9fU))return 0U;
 vsp_pick_queued|=bit;
 /* 第二段先缓存；仅在旋转反馈放行时广播。 */
 if(g->command.kind==SPF_CMD_STAGE3_CARRY_PLACE)return 1U;
 if(vsp_pick_queued==(g->command.axes_mask&(SPF_EXT|SPF_LIFT))){
  if(VSP_Stage3SyncStopPending())return 0U;
  if(CAN1_StartQueuedSourceAxes()!=1U)return 0U;
  g_source_parallel_sync_tick=(uint32_t)xTaskGetTickCount();
  if(g->command.axes_mask&SPF_LIFT)vsp_lift_start_cycle=vsp_axis[1].position_cycle;
  if(vsp_group.command.kind==SPF_CMD_STAGE3_LOADED_RETURN){
   vsp_loaded_z_cycle=vsp_axis[1].position_cycle;vsp_loaded_sync_sent=1U;
  }
  if(g_dm_stage2_sync){g_dm_parallel_group_go=1U;g_dm_initial_observe=0U;}
 }
 return 1U;
}
static uint32_t VSP_Start(void *context,uint32_t bit,const SPF_Command *c)
{
 VSP_Axis *a;uint8_t address;(void)context;(void)c;
 if(bit==SPF_DM){
  if(VehicleArm_Stage3GroupStopPending())return 0U;
  if(!g_dm_stage2_sync)g_dm_parallel_group_go=1;return 1;
 }
 address=bit==SPF_EXT?6:5;a=&vsp_axis[bit==SPF_EXT?0:1];a->phase=0;
 if(c->kind==SPF_CMD_STAGE3_CARRY_PLACE)return VSP_StartQueuedPick(bit,a,address);
 if(c->kind==SPF_CMD_STAGE3_EMPTY_HANDOFF){
  uint32_t r;
  if(VehicleArm_Stage3GroupStopPending())return 0U;
  if(bit==SPF_EXT && fabsf(a->goal-a->before)<=.2f)return 1U;
  if(bit==SPF_LIFT)vsp_lift_start_cycle=a->position_cycle;
  r=VehicleArm_Exchange(address,a->move,15U,3U,&s_rx);
  return (r==1U && (s_rx.Data[1]==2U || s_rx.Data[1]==0x9fU)) || (bit==SPF_EXT && r==5U);
 }
 if(c->kind==SPF_CMD_STAGE3_CARRY_RISE){
  uint32_t r;
  if(VehicleArm_Stage3GroupStopPending())return 0U;
  /* 升降先直发，旋转真实目标发出后伸缩才直发。零位伸缩仍轮询到位。 */
  if(bit==SPF_EXT && fabsf(a->goal-a->before)<=.2f)return 1U;
  r=VehicleArm_Exchange(address,a->move,15U,3U,&s_rx);
  if(r!=1U || (s_rx.Data[1]!=2U && s_rx.Data[1]!=0x9fU))
   return bit==SPF_EXT && r==5U;
  if(bit==SPF_LIFT){
   vsp_lift_start_cycle=a->position_cycle;
   vsp_loaded_sync_sent=1U;vsp_loaded_z_cycle=a->position_cycle;
  }
  return 1U;
 }
 if(c->kind==SPF_CMD_STAGE3_TOUCHDOWN){
  /* The relative frame was prepared from this origin. Read it again before
   * start and do not transmit if it changed or a STOP arrived in the query. */
  if(address!=5U || !S3_TouchdownAllowed() || !VSP_Position(address,a) ||
     a->current!=a->before ||
     !VSP_TouchdownDelta(a->goal-a->current,VEHICLE_LIFT_DOWN_SIGN,(a->options&0x80U)?100U:10U) ||
     !S3_TouchdownAllowed())return 0U;
 }else{
  if(bit==SPF_LIFT)vsp_lift_start_cycle=a->position_cycle;
  if(vsp_pick_sync)return VSP_StartQueuedPick(bit,a,address);
  if(fabsf(a->goal-a->before)<=.2f)return 1;
 }
 {uint8_t n=15U;
  uint32_t r=VehicleArm_Exchange(address,a->move,n,3,&s_rx);
  /* 2026-11-08: for the Emm_V5 11-byte frame the drive does not always answer the
   * ack in the window we allow (r==5 means the frame was sent, ack missed). The
   * 0x36 position loop in VSP_Poll is the real proof of motion, so accept 'sent'
   * here and let the poll decide. Node5 keeps the strict ack check. */
  if(address==6U)return (r==1U || r==5U)?1U:0U;
  return r==1U && (s_rx.Data[1]==2 || s_rx.Data[1]==0x9f);}
}
static uint32_t VSP_TouchdownArrived(float before,float current,const uint8_t *move,uint32_t scale)
{
 uint32_t units=((uint32_t)move[8]<<24)|((uint32_t)move[9]<<16)|((uint32_t)move[10]<<8)|move[11];
 float moved=(current-before)*VEHICLE_LIFT_DOWN_SIGN;
 return units && moved>0.0f && fabsf(moved*(float)scale-(float)units)<=0.5f;
}
static uint32_t VSP_Poll(void *context,uint32_t bit)
{
 VSP_Axis *a;uint8_t address;uint32_t zero_return;(void)context;
 SPF_Group *g=bit==SPF_DM?&vsp_group:VSP_StepperGroup();
 if(g==&vsp_carry_place && !vsp_carry_sent)return 0U;
 if(bit==SPF_DM){
  if(g_dm_jog_first_fault || g_dm_jog_state==8)return 2;
  return g_dm_jog_state==7 && g_dm_jog_result==1 && g_dm_posvel_holding &&
   (uint32_t)xTaskGetTickCount()-g_dm_can2_feedback_tick<100 && fabsf(g_dm_can2_logical_position_deg-vsp_group.command.goal.dm_deg)<=3;
 }
 address=bit==SPF_EXT?6:5;a=&vsp_axis[bit==SPF_EXT?0:1];
 zero_return=(address==6U && g->command.goal.extension_mm==0.0f);
 if(a->phase==0){
  /* 2026-11-08: one missed poll while the axis is accelerating must not abort the
   * transfer (old code returned 2 -> fault 7 within ~3 s). Retry instead. */
  if(!VSP_Query(address,0x3a,3)){a->samples=0;return 0;}
  if(!(s_rx.Data[1]&1) || (s_rx.Data[1]&0x08))return 2;
  if(s_rx.Data[1]&4){
   if(address==5)return 2;
   if(!a->stall_active){a->stall_active=1;a->stall_ms=(uint32_t)xTaskGetTickCount();}
   else if((uint32_t)xTaskGetTickCount()-a->stall_ms>=SOURCE_EXTENSION_STALL_CONFIRM_MS)return 2;
  }else a->stall_active=0;
  a->flags=s_rx.Data[1];a->phase=1;return 0;
 }
 if(a->phase==1){
  float previous=a->current;
  if(!VSP_Position(address,a)){if(zero_return)a->samples=0;return 0;}
  if(fabsf(a->current-a->goal)+0.2f<fabsf(previous-a->goal)){
   g->progress_ms=a->position_tick;
   if(g!=&vsp_group)vsp_group.progress_ms=a->position_tick;
  }
  /* 2026-11-08: the +/-2 deg overshoot window (0.55 mm) was tighter than the axis can hold -- the extension lands within ~1.7 mm of target and was being reported as a fault. Widen to 8 deg (2.2 mm) which still catches a runaway (670 deg). */
  /* 2026-11-08: measured stop overshoot after the 294.4 pulses/mm recalibration is
   * ~33 deg (1.0 mm) on a 100 mm move. 60 deg still catches the 670 deg runaway seen
   * earlier. */
  if(!VSP_PositionWindow(a,zero_return || (address==6U && Low_ObserveReturnSettling())))return 2;
  a->phase=2;return 0;
 }
 if(!VSP_Query(address,0x35,5) || ((zero_return || vsp_group.command.kind==SPF_CMD_STAGE3_TOUCHDOWN) && s_rx.Data[1]>1U)){
  if(zero_return){a->samples=0;a->phase=0;}return 0;
 }
 /* Zero retraction must satisfy the same window used to admit rotation. */
 if(!s_rx.Data[2] && !s_rx.Data[3] && !(a->flags&0x0c) &&
    (vsp_group.command.kind==SPF_CMD_STAGE3_TOUCHDOWN?
     ((a->flags&2) && VSP_TouchdownArrived(a->before,a->current,a->move,(a->options&0x80U)?100U:10U)):
     (fabsf(a->current-a->goal)<=(zero_return?SOURCE_EXTENSION_ZERO_DONE_DEG:(address==6U?SOURCE_EXTENSION_DONE_DEG:22.5f)) &&
      (fabsf(a->goal-a->before)<=.2f || (a->flags&2)))))a->samples++;else a->samples=0;
 a->phase=0;return a->samples>=((vsp_group.command.kind==SPF_CMD_PICK || (address==5U && Low_Busy()))?2U:1U);
}
static uint32_t VSP_Stop(void *context,uint32_t bit)
{
 const uint8_t stop[4]={0xfe,0x98,0,0x6b};(void)context;
 if(bit==SPF_DM){
  if((vsp_group.command.kind==SPF_CMD_STAGE3_LOADED_RETURN ||
      vsp_group.command.kind==SPF_CMD_STAGE3_EMPTY_HANDOFF ||
      vsp_group.command.kind==SPF_CMD_STAGE3_CARRY_RISE) && vsp_dm_posted){
   /* 让 CAN2 先清未消费的回收请求，再确认停机；不重复打断 STOP。 */
   if(!vsp_loaded_dm_stop){vsp_loaded_dm_stop=1U;g_dm_jog_request=3U;return 0U;}
   if(g_dm_jog_request || g_dm_jog_state==6U)return 0U;
   if(!g_dm_posvel_holding && g_dm_jog_state==8U && g_dm_jog_stop_result==1U)return 1U;
   g_dm_jog_request=3U;return 0U;
  }
  /* 常使能：到位保持时不下0xFD，下一动作可直接改目标。 */
  if(g_dm_posvel_holding && g_dm_jog_state==7U && g_dm_jog_result==1U && !g_dm_jog_request)return 1;
  if(g_dm_jog_state==8 && g_dm_jog_stop_result==1 && !g_dm_posvel_holding)return 1;
  if(g_dm_stage2_sync && g_dm_jog_state==6U)return 0; /* Await the existing in-flight STOP response. */
  if(g_dm_jog_request!=3U)g_dm_jog_request=3U;return 0;
 }
 return VehicleArm_Exchange(bit==SPF_EXT?6:5,stop,4,3,&s_rx)==1 && s_rx.Data[1]==2;
}
static uint32_t VSP_Clock(void *context){(void)context;return (uint32_t)xTaskGetTickCount();}
static uint32_t VehicleSourceParallel_Owns(void){return vsp_owned;}
uint32_t VehicleArm_SourceParallelAllowsBody(void)
{return vsp_owned && vsp_group.command.kind==SPF_CMD_ROUTE_AND_STOW && vsp_group.state<4;}
static void VehicleSourceParallel_Cancel(void){g_dm_initial_observe=0U;if(vsp_owned)SPF_GroupCancel(&vsp_group,2);}
static void VSP_ReleasePickDescent(uint32_t now)
{
 if(vsp_pick_lead_ms && vsp_group.state==2U &&
    (vsp_group.started&SPF_EXT) && !(vsp_group.started&SPF_LIFT) &&
    (uint32_t)(now-vsp_group.first_start_ms)>=vsp_pick_lead_ms)
  vsp_group.release_mask|=SPF_LIFT;
}
static uint32_t VSP_CarryYawReady(uint32_t now)
{
 return g_dm_stage3_carry_target_feedback && g_dm_can2_feedback_valid &&
  (uint32_t)(now-g_dm_can2_feedback_tick)<100U && g_dm_can2_motor_state==1U &&
  !g_dm_jog_first_fault && g_dm_jog_state!=8U &&
  fabsf(g_dm_can2_logical_position_deg-vsp_group.command.goal.dm_deg)<=3.0f;
}
static uint32_t VSP_CarryOriginHealthy(uint8_t address)
{
 uint32_t speed,flags;
 if(!VSP_Query(address,0x3a,3))return 0U;
 if(VehicleArm_Stage3GroupStopPending())return 2U;
 flags=s_rx.Data[1];
 if(!(flags&1U) || (flags&0x0cU)){
  VSP_CarryFirst(VSP_CARRY_DIAG_FLAGS,address,0x3aU,1U,flags,0U,0U);return 2U;
 }
 if(!VSP_Query(address,0x35,5))return 0U;
 if(VehicleArm_Stage3GroupStopPending())return 2U;
 speed=((uint32_t)s_rx.Data[2]<<8)|s_rx.Data[3];
 if(s_rx.Data[1]>1U){VSP_CarryFirst(VSP_CARRY_DIAG_SPEED,address,0x35U,1U,flags,speed,0U);return 2U;}
 /* 合法非零速度表示尚未停稳；等待下一轮复读，不广播。 */
 if(speed){
  g_vsp_carry_wait_speed_count++;g_vsp_carry_wait_speed_node=address;g_vsp_carry_wait_speed_raw=speed;return 0U;
 }
 return 1U;
}
static uint32_t VSP_CarryOriginWithin(const VSP_Axis *a,float origin,float origin_window,
                                     float endpoint_window,uint32_t scale)
{
 uint32_t units=((uint32_t)a->move[8]<<24)|((uint32_t)a->move[9]<<16)|
                ((uint32_t)a->move[10]<<8)|a->move[11];
 float delta=(a->move[1]?-1.0f:1.0f)*(float)units/(float)scale;
 /* 缓存为相对位移：按最新原点换算落点，复用该轴原到位窗口。 */
 return fabsf(a->current-origin)<=origin_window &&
        fabsf(a->current+delta-a->goal)<=endpoint_window;
}
static void VSP_CarryService(void)
{
 uint32_t now=(uint32_t)xTaskGetTickCount();
 if(vsp_group.state==4U){
  if(vsp_group.fault==3U)VSP_CarryFirst(VSP_CARRY_DIAG_TIMEOUT,g_vsp_carry_wait_speed_node,
     0U,0U,0U,g_vsp_carry_wait_speed_raw,0U);
  return;
 }
 if((uint32_t)(now-vsp_group.progress_ms)>=vsp_group.timeout_ms){
  VSP_CarryFirst(VSP_CARRY_DIAG_TIMEOUT,g_vsp_carry_wait_speed_node,0U,0U,0U,g_vsp_carry_wait_speed_raw,0U);
  SPF_GroupCancel(&vsp_group,3U);return;
 }
 if(VehicleArm_Stage3GroupStopPending()){SPF_GroupCancel(&vsp_group,2U);return;}
 if(!vsp_carry_leg){
  if((vsp_group.done&(SPF_EXT|SPF_LIFT))!=(SPF_EXT|SPF_LIFT) || !VSP_CarryYawReady(now))return;
  /* X/Z 原命令已结束；下一段用新的读数计算相对位移。DM 继续由原组轮询。 */
  memset(vsp_axis,0,sizeof(vsp_axis));
  vsp_carry_leg=vsp_pick_sync=1U;vsp_pick_queued=vsp_carry_origin_sampled=0U;
  vsp_carry_place.begin_ms=vsp_carry_place.progress_ms=now;
 }
 SPF_GroupService(&vsp_carry_place);
 if(vsp_carry_place.state==4U){SPF_GroupCancel(&vsp_group,vsp_carry_place.fault);return;}
 if(!vsp_carry_sent && vsp_pick_queued==(SPF_EXT|SPF_LIFT) &&
    VSP_CarryYawReady((uint32_t)xTaskGetTickCount())){
  VSP_Axis *x=&vsp_axis[0],*z=&vsp_axis[1];uint32_t healthy,scale=g_lift_ruler_controller.top_scale;
  vsp_carry_origin_sampled=0U;
  healthy=VSP_CarryOriginHealthy(5U);
  if(VehicleArm_Stage3GroupStopPending()){SPF_GroupCancel(&vsp_group,2U);return;}
  if(!healthy)return;
  if(healthy==2U){SPF_GroupCancel(&vsp_group,6U);return;}
  healthy=VSP_CarryOriginHealthy(6U);
  if(VehicleArm_Stage3GroupStopPending()){SPF_GroupCancel(&vsp_group,2U);return;}
  if(!healthy)return;
  if(healthy==2U){SPF_GroupCancel(&vsp_group,6U);return;}
  /* 两轴确认零速后，才读取本次广播前的起点。 */
  healthy=VSP_Position(5U,z);
  if(VehicleArm_Stage3GroupStopPending()){SPF_GroupCancel(&vsp_group,2U);return;}
  if(!healthy)return;
  vsp_carry_origin_sampled|=SPF_LIFT;
  healthy=VSP_Position(6U,x);
  if(VehicleArm_Stage3GroupStopPending()){SPF_GroupCancel(&vsp_group,2U);return;}
  if(!healthy)return;
  vsp_carry_origin_sampled|=SPF_EXT;
  now=(uint32_t)xTaskGetTickCount();
  if((uint32_t)(now-z->position_tick)>150U || (uint32_t)(now-x->position_tick)>150U ||
     !VSP_CarryYawReady(now))return;
  if(!VSP_CarryOriginWithin(x,g_extension_home_reference_deg,SOURCE_EXTENSION_ZERO_DONE_DEG,
      vsp_carry_place.command.goal.extension_mm==0.0f?SOURCE_EXTENSION_ZERO_DONE_DEG:SOURCE_EXTENSION_DONE_DEG,10U)){
   VSP_CarryFirst(VSP_CARRY_DIAG_ORIGIN,6U,0x36U,1U,0U,0U,0U);SPF_GroupCancel(&vsp_group,6U);return;
  }
  if((scale!=10U && scale!=100U) ||
     !VSP_CarryOriginWithin(z,(float)g_lift_ruler_controller.safe_position_raw/(float)scale,
      SOURCE_TOP_TOLERANCE_DEG,22.5f,scale)){
   VSP_CarryFirst(VSP_CARRY_DIAG_ORIGIN,5U,0x36U,1U,0U,0U,0U);SPF_GroupCancel(&vsp_group,6U);return;
  }
  healthy=CAN1_StartQueuedSourceAxes();
  if(healthy!=1U){VSP_CarryFirst(VSP_CARRY_DIAG_BROADCAST,0U,0xffU,0U,0U,0U,healthy);
   SPF_GroupCancel(&vsp_group,6U);return;
  }
  vsp_carry_sent=1U;g_source_parallel_sync_tick=(uint32_t)xTaskGetTickCount();
 }
}
static void VehicleSourceParallel_Service(void)
{
 uint32_t state;
 if(!vsp_owned)return;
 if(!SingleTeach_ReferencesReady() && vsp_group.state!=4)SPF_GroupCancel(&vsp_group,8);
 if((vsp_group.command.kind==SPF_CMD_STAGE3_LOADED_RETURN || VehicleArm_Stage3CarryActive()) && vsp_group.state!=4U){
  if(VehicleArm_Stage3GroupStopPending())SPF_GroupCancel(&vsp_group,2U);
  else if(!vsp_dm_posted && vsp_loaded_sync_sent){
   uint32_t now=(uint32_t)xTaskGetTickCount();
   /* Z 到位后组不再轮询；等旋转反馈恢复时只读刷新位置。 */
   if((vsp_group.done&SPF_LIFT) && (uint32_t)(now-vsp_axis[1].position_tick)>150U &&
      (uint32_t)(now-vsp_group.progress_ms)<vsp_group.timeout_ms && g_dm_can2_feedback_valid &&
      (uint32_t)(now-g_dm_can2_feedback_tick)<100U)
    (void)VSP_Position(5U,&vsp_axis[1]);
   if(VehicleArm_Stage3GroupStopPending())SPF_GroupCancel(&vsp_group,2U);
   else if(VSP_LoadedReturnHeightReady((uint32_t)xTaskGetTickCount()))vsp_group.release_mask|=SPF_DM;
  }
 }
 if(VehicleArm_Stage3EmptyHandoffActive() && vsp_group.state!=4U){
  if(VehicleArm_Stage3GroupStopPending())SPF_GroupCancel(&vsp_group,2U);
  else if(vsp_group.done&SPF_LIFT){
   vsp_group.release_mask|=SPF_DM;
   if(vsp_group.done&SPF_DM)vsp_group.release_mask|=SPF_EXT;
  }
 }
 VSP_ReleasePickDescent((uint32_t)xTaskGetTickCount());
 SPF_GroupService(&vsp_group);
 if(VehicleArm_Stage3CarryActive())VSP_CarryService();
 state=vsp_group.state;
 if(VehicleArm_Stage3CarryActive() && state==3U && vsp_carry_place.state!=3U)state=2U;
 g_source_parallel_state=state;g_source_parallel_cause=vsp_group.fault;
 g_source_parallel_started_mask=vsp_group.started;
 g_source_parallel_done_mask=VehicleArm_Stage3CarryActive()?
  ((vsp_group.done&SPF_DM)|vsp_carry_place.done):vsp_group.done;
 g_source_parallel_stop_mask=vsp_group.stop_mask;
 g_source_parallel_start_skew_ms=vsp_group.last_start_ms-vsp_group.first_start_ms;
 if(state==3U){
  if(vsp_group.command.axes_mask&SPF_EXT)StartupAxes_SetExtensionHoldTarget(vsp_axis[0].goal);
  g_dm_initial_observe=g_dm_stage2_sync=g_dm_parallel_group_active=g_dm_parallel_group_go=0;
  g_vehicle_arm_axis_owned=g_vehicle_arm_dm_owned=0;g_vehicle_arm_state=2;vsp_owned=0;
 }else if(vsp_group.state==4){
  g_dm_initial_observe=0U;g_vehicle_arm_state=3;g_vehicle_arm_cause=vsp_group.fault;
  if(!vsp_group.stop_mask){g_vehicle_arm_axis_owned=g_vehicle_arm_dm_owned=0;g_dm_initial_observe=g_dm_stage2_sync=g_dm_parallel_group_active=g_dm_parallel_group_go=0;vsp_owned=0;}
 }
}
#else
static uint32_t VehicleSourceParallel_Owns(void){return 0;}
static void VehicleSourceParallel_Cancel(void){}
static void VehicleSourceParallel_Service(void){}
uint32_t VehicleArm_SourceParallelAllowsBody(void){return 0;}
uint32_t VehicleArm_Stage3EmptyHandoffActive(void){return 0U;}
uint32_t VehicleArm_Stage3CarryActive(void){return 0U;}
uint32_t VehicleArm_Stage3EmptyTargetPosted(float goal){(void)goal;return 0U;}
uint32_t VehicleArm_Stage3GroupStopPending(void){return 0U;}
#endif
uint32_t VehicleArm_BeginSourceParallel(const SPF_Command *command,uint32_t sweep_verified,uint32_t driving_stow_verified)
{
#if APP_MOTOR_OFFLINE_ONLY
 (void)command;(void)sweep_verified;(void)driving_stow_verified;return 0;
#else
 SPF_GroupIO io={0,VSP_Prepare,VSP_Start,VSP_Poll,VSP_Stop,VSP_Clock};ChassisAutoStatus body;
 if(!command || VehicleArm_Busy() || vsp_owned || !SingleTeach_ReferencesReady() ||
  !g_extension_home_reference_valid || !g_vehicle_lift_reference_valid)return 0;
 (void)ChassisAuto_ReadStatus(&body);
 if(body.active && !(command->kind==SPF_CMD_ROUTE_AND_STOW && driving_stow_verified==1))return 0;
 if(!body.active && body.cause)return 0;
 /* Planner/mission supplies verified trajectory, current run/profile and
  * references. No key or Watch path can grant these in this candidate. */
 /* 2026-11-08: a previous transfer fault left vsp_group.state==4 forever -- vsp_owned
  * was already cleared, so Cancel never ran and every later IT3 was rejected with
  * it_cause=7 until a full board reset. Reset the group instead of dead-ending. */
 if(vsp_group.state==4U){memset(&vsp_group,0,sizeof(vsp_group));}
 /* 2026-11-08: start skew was 20ms but a node6 frame + ack costs ~80ms, so every
 * transfer tripped SPF_GroupCancel(5). 90ms stays inside the SPF_GroupBegin limit
 * (<=100) while leaving room for the Emm_V5 ack. */
 if(!SPF_GroupBegin(&vsp_group,command,&io,sweep_verified,90,VEHICLE_AXIS_TIMEOUT_MS?VEHICLE_AXIS_TIMEOUT_MS:12000))return 0;
 vsp_pick_queued=g_source_parallel_sync_tick=g_dm_initial_observe=g_dm_stage2_sync=0U;
 vsp_pick_sync=command->kind==SPF_CMD_STAGE3_LOADED_RETURN;
 vsp_loaded_sync_sent=vsp_loaded_z_cycle=vsp_loaded_dm_stop=0U;
 vsp_carry_leg=vsp_carry_sent=vsp_lift_start_cycle=0U;
 memset(&vsp_carry_place,0,sizeof(vsp_carry_place));
 if(command->kind==SPF_CMD_STAGE3_CARRY_RISE)g_dm_stage3_carry_target_feedback=0U;
 if(command->kind==SPF_CMD_STAGE3_EMPTY_HANDOFF || command->kind==SPF_CMD_STAGE3_CARRY_RISE)g_dm_stage3_empty_target_sent=0U;
 vsp_pick_lead_ms=0U;memset(vsp_axis,0,sizeof(vsp_axis));vsp_dm_posted=0;vsp_fast_lift=0U;vsp_owned=1;
 g_source_parallel_state=1;g_source_parallel_cause=0;g_source_parallel_started_mask=0;
 g_source_parallel_done_mask=g_source_parallel_stop_mask=g_source_parallel_start_skew_ms=0;
 g_vehicle_arm_dm_owned=g_dm_integrated_owner=1;g_vehicle_arm_axis_owned=6;
 g_vehicle_arm_state=1;g_vehicle_arm_cause=0;return 1;
#endif
}

uint32_t VehicleArm_BeginStage3CarryPlace(const SPF_Command *rise,const SPF_Command *place)
{
#if APP_MOTOR_OFFLINE_ONLY
 (void)rise;(void)place;return 0U;
#else
 SPF_Group staged={0};SPF_GroupIO io={0,VSP_Prepare,VSP_Start,VSP_Poll,VSP_Stop,VSP_Clock};
 if(!rise || !place || rise->kind!=SPF_CMD_STAGE3_CARRY_RISE ||
    place->kind!=SPF_CMD_STAGE3_CARRY_PLACE || rise->sequence!=place->sequence ||
    rise->run_id!=place->run_id || rise->goal.dm_deg!=place->goal.dm_deg ||
    !(place->goal.lift_pulses>rise->goal.lift_pulses) ||
    !SPF_GroupBegin(&staged,place,&io,1U,90U,VEHICLE_AXIS_TIMEOUT_MS?VEHICLE_AXIS_TIMEOUT_MS:12000U) ||
    !VehicleArm_BeginSourceParallel(rise,1U,0U))return 0U;
 g_vsp_carry_first_node=g_vsp_carry_first_command=g_vsp_carry_first_exchange=0U;
 g_vsp_carry_first_flags=g_vsp_carry_first_speed_raw=g_vsp_carry_first_broadcast_result=0U;
 g_vsp_carry_first_sequence=g_vsp_carry_first_stage=0U;
 g_vsp_carry_wait_speed_count=g_vsp_carry_wait_speed_node=g_vsp_carry_wait_speed_raw=0U;
 vsp_carry_place=staged;vsp_fast_lift=1U;return 1U;
#endif
}

uint32_t VehicleArm_Stage3LiftDepthFresh(uint32_t sequence,float *depth)
{
#if APP_MOTOR_OFFLINE_ONLY
 (void)sequence;(void)depth;return 0U;
#else
 VSP_Axis *z=&vsp_axis[1];uint32_t scale=g_lift_ruler_controller.top_scale;
 if(!depth || vsp_group.command.sequence!=sequence || vsp_group.state==4U ||
    !(vsp_group.started&SPF_LIFT) || z->position_cycle==vsp_lift_start_cycle ||
    (uint32_t)((uint32_t)xTaskGetTickCount()-z->position_tick)>150U ||
    (scale!=10U && scale!=100U))return 0U;
 *depth=(float)(z->position_raw-g_lift_ruler_controller.top_position_raw)*
        VEHICLE_LIFT_DOWN_SIGN*(3200.0f/(360.0f*(float)scale));
 return 1U;
#endif
}

uint32_t VehicleArm_SourceParallelCanAllowed(uint8_t address,const uint8_t *q,uint32_t n)
{
#if APP_MOTOR_OFFLINE_ONLY
 (void)address;(void)q;(void)n;return 0U;
#else
 uint32_t bit;VSP_Axis *a;SPF_Group *g=VSP_StepperGroup();
 if(!vsp_owned || !q || !n)return 0U;
 if((address==0U || q[0]==0xfdU) && VehicleArm_Stage3GroupStopPending())return 0U;
 if(address==0U)return vsp_pick_sync && vsp_pick_queued &&
  vsp_pick_queued==(g->command.axes_mask&(SPF_EXT|SPF_LIFT)) &&
  ((g==&vsp_carry_place && !vsp_carry_sent && g->state==2U && g->started==(SPF_EXT|SPF_LIFT)) ||
   (g->state==1U && g->started==g->command.axes_mask) ||
   ((vsp_group.state==2U || (vsp_group.state==1U &&
     vsp_group.command.kind==SPF_CMD_STAGE3_LOADED_RETURN)) &&
    vsp_group.delayed_mask==SPF_DM && vsp_group.started==(SPF_EXT|SPF_LIFT))) && n==3U &&
  q[0]==0xffU && q[1]==0x66U && q[2]==0x6bU;
 if(address!=5U && address!=6U)return 0U;
 bit=address==6U?SPF_EXT:SPF_LIFT;a=&vsp_axis[address==6U?0:1];
 if(!(g->command.axes_mask&bit))return 0U;
 if(n==5U && q[0]==0xf3 && q[1]==0xab && q[2]==1 && q[3]==0 && q[4]==0x6b)
  return (g->state==1U || (g->delayed_mask && g->state==2U)) && a->phase==3U;
 if(address==6U && n==11U && q[0]==0xfdU)
  return g->state==1U && (g->started&bit) && !memcmp(q,a->move,11U);
 return n==15U && (g->state==1U || (g->delayed_mask && g->state==2U)) && (g->started&bit) && !memcmp(q,a->move,15U);
#endif
}
