/* Included by vehicle_arm.c: sole CAN task owns all bus calls. No new task,
 * serial start command, key shortcut, power toggle or fabricated feedback. */
#include "source_parallel_flow.h"
#include "chassis_auto_test.h"
volatile uint32_t g_source_parallel_state,g_source_parallel_cause,g_source_parallel_started_mask;
volatile uint32_t g_source_parallel_done_mask,g_source_parallel_stop_mask,g_source_parallel_start_skew_ms;
#if !APP_MOTOR_OFFLINE_ONLY
extern volatile uint32_t g_dm_parallel_group_active,g_dm_parallel_group_go;
extern volatile uint32_t g_extension_home_reference_valid;
extern volatile float g_extension_home_reference_deg;
typedef struct {uint32_t phase,options,samples,flags,cycle,stall_active,stall_ms;float before,goal,current;uint8_t move[15];} VSP_Axis;
static VSP_Axis vsp_axis[2];
static SPF_Group vsp_group;
static uint32_t vsp_dm_posted,vsp_dm_cycle,vsp_owned;
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
 a->current=(s_rx.Data[1]?-1.0f:1.0f)*(float)raw*0.1f;
 if(address==5){g_vehicle_lift_current_deg=a->current;g_vehicle_lift_feedback_valid=1;}
 else {g_startup_extension_current_deg=a->current;g_startup_extension_feedback_valid=1;}
 return 1;
}
static uint32_t VSP_Prepare(void *context,uint32_t bit,const SPF_Command *c)
{
 VSP_Axis *a;uint8_t address,enable[5]={0xf3,0xab,1,0,0x6b};uint32_t units,speed;float delta;
 (void)context;
 if(bit==SPF_DM){
  if(!vsp_dm_posted){
   if(!g_dm_can2_feedback_valid || (uint32_t)xTaskGetTickCount()-g_dm_can2_feedback_tick>=100)return 2;
   g_dm_parallel_group_active=1;g_dm_parallel_group_go=0;
   g_dm_source_smooth=0;g_dm_point_enable=1;g_dm_posvel_speed_deg_s=c->dm_deg_s;
   g_dm_pick_target_deg=g_dm_place_target_deg=c->goal.dm_deg;
   g_dm_jog_request=c->goal.dm_deg>=g_dm_can2_logical_position_deg?4:5;
   vsp_dm_cycle=g_dm_can2_read_cycle;vsp_dm_posted=1;return 0;
  }
  if(g_dm_jog_first_fault || (g_dm_jog_state==8 && g_dm_jog_result!=2))return 2;
  /* 常使能直改目标：请求已被CAN任务消费并进入运动态即放行，不再等使能步骤。 */
  if(g_dm_jog_request)return 0;
  if(g_dm_jog_state==4U || g_dm_jog_state==5U)return 1;
  if(g_dm_jog_state==7U && g_dm_posvel_holding && g_dm_jog_result==1U &&
     fabsf(g_dm_can2_logical_position_deg-c->goal.dm_deg)<=3.0f)return 1;
  return g_dm_jog_state==3 && g_dm_can2_motor_state==1 &&
   g_dm_can2_read_cycle!=vsp_dm_cycle && (uint32_t)xTaskGetTickCount()-g_dm_can2_feedback_tick<100;
 }
 address=bit==SPF_EXT?6:5;a=&vsp_axis[bit==SPF_EXT?0:1];
 switch(a->phase){
 case 0:
  if(!VSP_Query(address,0x1a,4) || (s_rx.Data[2]&3))return 2;
  a->options=s_rx.Data[2];a->phase=1;return 0;
 case 1:
  if(!VSP_Position(address,a))return 2;a->before=a->current;
  a->goal=address==6?g_extension_home_reference_deg+STARTUP_EXTENSION_EXTEND_SIGN*c->goal.extension_mm*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f:
   SingleTeach_TopDeg()+VEHICLE_LIFT_DOWN_SIGN*c->goal.lift_pulses*360.0f/3200.0f;
  delta=a->goal-a->current;
  /* 2026-11-08: the old 612.1 deg cap was 170mm at the wrong 32 pulses/mm scale.
   * With the measured 294.4 pulses/mm a full 170mm stroke is ~5630 deg, so derive the
   * cap from the same constants instead of a hard-coded angle. */
  if(!(fabsf(delta)<=(address==6?(170.0f*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f+0.1f):((float)SPF_MAX_LIFT_PULSES*360.0f/3200.0f+0.1f))))return 2;
  /* 2026-11-08 (ZDT X42S manual 5.3.10): X-firmware trapezoid position mode takes the
   * position field in 0.1 deg, so 360.0 deg -> 3600. The previous deg*3200/360 sent
   * only 3200 for the same 360 deg target, i.e. 320.0 deg, an 11.1% shortfall. */
  units=(uint32_t)(fabsf(delta)*10.0f+0.5f);
  speed=(uint32_t)((address==6?c->extension_rpm:c->lift_rpm)*10.0f+.5f);
  memset(a->move,0,15);a->move[0]=0xfd;a->move[1]=delta<0;
  a->move[2]=(uint8_t)(c->stepper_accel_rpm_s>>8);
  a->move[3]=(uint8_t)c->stepper_accel_rpm_s;
  a->move[4]=(uint8_t)(c->stepper_accel_rpm_s>>8);a->move[5]=(uint8_t)c->stepper_accel_rpm_s;
  a->move[6]=(uint8_t)(speed>>8);a->move[7]=(uint8_t)speed;
  a->move[8]=(uint8_t)(units>>24);a->move[9]=(uint8_t)(units>>16);a->move[10]=(uint8_t)(units>>8);a->move[11]=(uint8_t)units;
  a->move[12]=2;a->move[14]=0x6b;a->phase=2;return 0;
 case 2:
  if(!VSP_Query(address,0x35,5) || s_rx.Data[2] || s_rx.Data[3])return 2;
  a->phase=3;return 0;
 case 3:
  if(VehicleArm_Exchange(address,enable,5,3,&s_rx)!=1 || s_rx.Data[1]!=2)return 2;
  a->phase=4;return 0;
 default:
  if(!VSP_Query(address,0x3a,3) || !(s_rx.Data[1]&1) || (s_rx.Data[1]&0x08))return 2;
  if(s_rx.Data[1]&4){
   if(address==5)return 2;
   if(!a->stall_active){a->stall_active=1;a->stall_ms=(uint32_t)xTaskGetTickCount();}
   else if((uint32_t)xTaskGetTickCount()-a->stall_ms>=2000)return 2;
  }else a->stall_active=0;
  return 1;
 }
}
static uint32_t VSP_Start(void *context,uint32_t bit,const SPF_Command *c)
{
 VSP_Axis *a;uint8_t address;(void)context;(void)c;
 if(bit==SPF_DM){g_dm_parallel_group_go=1;return 1;}
 address=bit==SPF_EXT?6:5;a=&vsp_axis[bit==SPF_EXT?0:1];a->phase=0;
 if(fabsf(a->goal-a->before)<=.2f)return 1;
 {uint8_t n=15U;
  uint32_t r=VehicleArm_Exchange(address,a->move,n,3,&s_rx);
  /* 2026-11-08: for the Emm_V5 11-byte frame the drive does not always answer the
   * ack in the window we allow (r==5 means the frame was sent, ack missed). The
   * 0x36 position loop in VSP_Poll is the real proof of motion, so accept 'sent'
   * here and let the poll decide. Node5 keeps the strict ack check. */
  if(address==6U)return (r==1U || r==5U)?1U:0U;
  return r==1U && (s_rx.Data[1]==2 || s_rx.Data[1]==0x9f);}
}
static uint32_t VSP_Poll(void *context,uint32_t bit)
{
 VSP_Axis *a;uint8_t address;(void)context;
 if(bit==SPF_DM){
  if(g_dm_jog_first_fault || g_dm_jog_state==8)return 2;
  return g_dm_jog_state==7 && g_dm_jog_result==1 && g_dm_posvel_holding &&
   (uint32_t)xTaskGetTickCount()-g_dm_can2_feedback_tick<100 && fabsf(g_dm_can2_logical_position_deg-vsp_group.command.goal.dm_deg)<=3;
 }
 address=bit==SPF_EXT?6:5;a=&vsp_axis[bit==SPF_EXT?0:1];
 if(a->phase==0){
  /* 2026-11-08: one missed poll while the axis is accelerating must not abort the
   * transfer (old code returned 2 -> fault 7 within ~3 s). Retry instead. */
  if(!VSP_Query(address,0x3a,3)){a->samples=0;return 0;}
  if(!(s_rx.Data[1]&1) || (s_rx.Data[1]&0x08))return 2;
  if(s_rx.Data[1]&4){
   if(address==5)return 2;
   if(!a->stall_active){a->stall_active=1;a->stall_ms=(uint32_t)xTaskGetTickCount();}
   else if((uint32_t)xTaskGetTickCount()-a->stall_ms>=2000)return 2;
  }else a->stall_active=0;
  a->flags=s_rx.Data[1];a->phase=1;return 0;
 }
 if(a->phase==1){
  if(!VSP_Position(address,a))return 0;
  /* 2026-11-08: the +/-2 deg overshoot window (0.55 mm) was tighter than the axis can hold -- the extension lands within ~1.7 mm of target and was being reported as a fault. Widen to 8 deg (2.2 mm) which still catches a runaway (670 deg). */
  /* 2026-11-08: measured stop overshoot after the 294.4 pulses/mm recalibration is
   * ~33 deg (1.0 mm) on a 100 mm move. 60 deg still catches the 670 deg runaway seen
   * earlier. */
  if(a->current<fminf(a->before,a->goal)-60.0f || a->current>fmaxf(a->before,a->goal)+60.0f)return 2;
  a->phase=2;return 0;
 }
 if(!VSP_Query(address,0x35,5))return 0;
 if(!s_rx.Data[2] && !s_rx.Data[3] && !(a->flags&0x0c) && fabsf(a->current-a->goal)<=45.0f &&
  (fabsf(a->goal-a->before)<=.2f || (a->flags&2)))a->samples++;else a->samples=0;
 a->phase=0;return a->samples>=3;
}
static uint32_t VSP_Stop(void *context,uint32_t bit)
{
 const uint8_t stop[4]={0xfe,0x98,0,0x6b};(void)context;
 if(bit==SPF_DM){
  /* 常使能：到位保持时不下0xFD，下一动作可直接改目标。 */
  if(g_dm_posvel_holding && g_dm_jog_state==7U && g_dm_jog_result==1U)return 1;
  if(g_dm_jog_state==8 && g_dm_jog_stop_result==1 && !g_dm_posvel_holding)return 1;
  if(!g_dm_jog_request)g_dm_jog_request=3;return 0;
 }
 return VehicleArm_Exchange(bit==SPF_EXT?6:5,stop,4,3,&s_rx)==1 && s_rx.Data[1]==2;
}
static uint32_t VSP_Clock(void *context){(void)context;return (uint32_t)xTaskGetTickCount();}
static uint32_t VehicleSourceParallel_Owns(void){return vsp_owned;}
uint32_t VehicleArm_SourceParallelAllowsBody(void)
{return vsp_owned && vsp_group.command.kind==SPF_CMD_ROUTE_AND_STOW && vsp_group.state<4;}
static void VehicleSourceParallel_Cancel(void){if(vsp_owned)SPF_GroupCancel(&vsp_group,2);}
static void VehicleSourceParallel_Service(void)
{
 if(!vsp_owned)return;
 if(!SingleTeach_ReferencesReady() && vsp_group.state!=4)SPF_GroupCancel(&vsp_group,8);
 SPF_GroupService(&vsp_group);g_source_parallel_state=vsp_group.state;g_source_parallel_cause=vsp_group.fault;
 g_source_parallel_started_mask=vsp_group.started;g_source_parallel_done_mask=vsp_group.done;g_source_parallel_stop_mask=vsp_group.stop_mask;
 g_source_parallel_start_skew_ms=vsp_group.last_start_ms-vsp_group.first_start_ms;
 if(vsp_group.state==3){
  if(vsp_group.command.axes_mask&SPF_EXT)StartupAxes_SetExtensionHoldTarget(vsp_axis[0].goal);
  g_dm_parallel_group_active=g_dm_parallel_group_go=0;
  g_vehicle_arm_axis_owned=g_vehicle_arm_dm_owned=0;g_vehicle_arm_state=2;vsp_owned=0;
 }else if(vsp_group.state==4){
  g_vehicle_arm_state=3;g_vehicle_arm_cause=vsp_group.fault;
  if(!vsp_group.stop_mask){g_vehicle_arm_axis_owned=g_vehicle_arm_dm_owned=0;g_dm_parallel_group_active=g_dm_parallel_group_go=0;vsp_owned=0;}
 }
}
#else
static uint32_t VehicleSourceParallel_Owns(void){return 0;}
static void VehicleSourceParallel_Cancel(void){}
static void VehicleSourceParallel_Service(void){}
uint32_t VehicleArm_SourceParallelAllowsBody(void){return 0;}
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
 memset(vsp_axis,0,sizeof(vsp_axis));vsp_dm_posted=0;vsp_owned=1;
 g_source_parallel_state=1;g_source_parallel_cause=0;g_source_parallel_started_mask=0;
 g_source_parallel_done_mask=g_source_parallel_stop_mask=g_source_parallel_start_skew_ms=0;
 g_vehicle_arm_dm_owned=g_dm_integrated_owner=1;g_vehicle_arm_axis_owned=6;
 g_vehicle_arm_state=1;g_vehicle_arm_cause=0;return 1;
#endif
}

uint32_t VehicleArm_SourceParallelCanAllowed(uint8_t address,const uint8_t *q,uint32_t n)
{
#if APP_MOTOR_OFFLINE_ONLY
 (void)address;(void)q;(void)n;return 0U;
#else
 uint32_t bit;VSP_Axis *a;
 if(!vsp_owned || !q || (address!=5U && address!=6U))return 0U;
 bit=address==6U?SPF_EXT:SPF_LIFT;a=&vsp_axis[address==6U?0:1];
 if(!(vsp_group.command.axes_mask&bit))return 0U;
 if(n==5U && q[0]==0xf3 && q[1]==0xab && q[2]==1 && q[3]==0 && q[4]==0x6b)
  return vsp_group.state==1U && a->phase==3U;
 if(address==6U && n==11U && q[0]==0xfdU)
  return vsp_group.state==1U && (vsp_group.started&bit) && !memcmp(q,a->move,11U);
 return n==15U && vsp_group.state==1U && (vsp_group.started&bit) && !memcmp(q,a->move,15U);
#endif
}
