#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "source_parallel_flow.h"
#include "dm_can2.h"

#undef assert
#define assert(x) do {if(!(x)){fprintf(stderr,"fixture:%d: %s\n",__LINE__,#x);exit(1);}} while(0)

#define APP_MOTOR_OFFLINE_ONLY 0
#define CHASSIS_AUTO_TEST_H
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) (x)
#define portTICK_PERIOD_MS 1U
static uint32_t tick, stop, references=1U;
static uint32_t xTaskGetTickCount(void){return tick;}
static CanRxMsg s_rx;
static struct {uint32_t top_scale;int64_t top_position_raw,safe_position_raw;} g_lift_ruler_controller;
static uint32_t g_vehicle_lift_feedback_valid,g_startup_extension_feedback_valid,g_vehicle_lift_reference_valid;
static uint32_t g_vehicle_arm_axis_owned,g_vehicle_arm_dm_owned,g_vehicle_arm_state,g_vehicle_arm_cause;
extern volatile uint32_t g_dm_integrated_owner;
static float g_vehicle_lift_current_deg,g_startup_extension_current_deg;
volatile uint32_t g_extension_home_reference_valid;
volatile float g_extension_home_reference_deg;
static uint32_t SingleTeach_ReferencesReady(void){return references;}
static float SingleTeach_TopDeg(void){return (float)g_lift_ruler_controller.top_position_raw/g_lift_ruler_controller.top_scale;}
static uint32_t VehicleArm_Busy(void){return 0;}
static float extension_hold;
static void StartupAxes_SetExtensionHoldTarget(float goal){extension_hold=goal;}
static uint32_t Low_Busy(void){return 0;}
static uint32_t Low_ObserveReturnSettling(void){return 0;}
typedef struct {uint32_t active,cause;} ChassisAutoStatus;
static uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *out){memset(out,0,sizeof(*out));return 1;}
static uint32_t VehicleArm_Exchange(uint8_t,const uint8_t*,uint8_t,uint8_t,void*);
static uint32_t CAN1_StartQueuedSourceAxes(void);
uint32_t Stage3_FastTransferStopPending(void){return stop;}

#include "vehicle_source_parallel.inc"
static uint32_t S3_TouchdownAllowed(void){return 0;}

static uint32_t fail_send, inject_stop, x_raw, z_raw, x_speed, z_speed, z_queries;
static uint32_t x_cache_ack, x_cache_result;
static uint32_t direct_ext_ack;
static uint32_t x_flags,z_flags;
static uint32_t x_settled_raw;
static uint32_t cached, broadcasts, xz_stop, normal_moves;
static uint32_t VehicleArm_Exchange(uint8_t address,const uint8_t *q,uint8_t n,uint8_t reply_n,void *out)
{
 CanRxMsg *rx=out;uint32_t raw=address==5?z_raw:x_raw;
 uint32_t bit=address==5?SPF_LIFT:SPF_EXT;
 uint32_t speed=address==5?z_speed:x_speed;
 (void)reply_n;memset(rx,0,sizeof(*rx));
 switch(q[0]){
 case 0x1a: rx->Data[2]=0x80;break;
 case 0x36:
  if(address==5)z_queries++;
  rx->Data[2]=(uint8_t)(raw>>24);rx->Data[3]=(uint8_t)(raw>>16);
  rx->Data[4]=(uint8_t)(raw>>8);rx->Data[5]=(uint8_t)raw;break;
 case 0x35:
  if(address==6 && !speed && x_settled_raw){x_raw=x_settled_raw;x_settled_raw=0;}
  rx->Data[3]=(uint8_t)speed;break;
 case 0x3a:rx->Data[1]=(uint8_t)(address==5?z_flags:x_flags);break;
 case 0xf3:rx->Data[1]=2;break;
 case 0xfd:
  assert(n==15);rx->Data[1]=2;
  if(!q[13]){normal_moves|=bit;if(address==6)rx->Data[1]=(uint8_t)direct_ext_ack;break;}
  assert(VehicleArm_SourceParallelCanAllowed(address,q,n));cached|=bit;
  if(address==6)rx->Data[1]=(uint8_t)x_cache_ack;
  if(inject_stop==bit)stop=1;
  return address==6?x_cache_result:1U;
 case 0xfe:xz_stop|=bit;rx->Data[1]=2;break;
 default:assert(0);
 }
 return 1;
}
static uint32_t CAN1_StartQueuedSourceAxes(void)
{
 const uint8_t q[]={0xff,0x66,0x6b};
 assert(cached==(SPF_EXT|SPF_LIFT));
 assert(VehicleArm_SourceParallelCanAllowed(0,q,3));
 if(fail_send)return 4;
 broadcasts++;return 1;
}

#include "fixture_dm_motion.inc"

volatile uint32_t g_dm_can2_feedback_valid,g_dm_can2_feedback_tick,g_dm_can2_motor_state;
volatile uint32_t g_dm_can2_read_request,g_dm_can2_read_cycle,g_dm_can2_state,g_dm_can2_result;
volatile uint32_t g_dm_can2_valid_mask,g_dm_can2_range_mask,g_dm_can2_esc_id,g_dm_can2_master_id,g_dm_can2_ctrl_mode;
volatile uint32_t g_dm_can2_position_raw,g_dm_can2_velocity_raw,g_dm_can2_torque_raw;
volatile float g_dm_can2_pmax,g_dm_can2_vmax,g_dm_can2_tmax,g_dm_can2_position_rad,g_dm_can2_position_deg;
volatile float g_dm_can2_logical_position_deg,g_dm_can2_velocity_rad_s,g_dm_can2_torque_nm;
CanRxMsg g_dm_can2_last_rx;
FixtureCAN fixture_can2;
static uint8_t dm_last_tx;
static uint32_t dm_tx;
static uint8_t dm_tx_status=CAN_TxStatus_Ok;
static uint32_t dm_reply;
uint8_t CAN_Transmit(FixtureCAN *c,CanTxMsg *tx){(void)c;dm_tx++;dm_last_tx=tx->Data[0];return 0;}
uint8_t CAN_TransmitStatus(FixtureCAN *c,uint8_t mb){(void)c;(void)mb;return dm_tx_status;}
void CAN_CancelTransmit(FixtureCAN *c,uint8_t mb){(void)c;(void)mb;}
void CAN_Receive(FixtureCAN *c,uint8_t f,CanRxMsg *rx){(void)c;(void)f;(void)rx;}
uint8_t CAN_MessagePending(FixtureCAN *c,uint8_t f){(void)c;(void)f;return (uint8_t)dm_reply;}
uint8_t CAN_GetFlagStatus(FixtureCAN *c,uint32_t f){(void)c;(void)f;return 0;}
void CAN_ClearFlag(FixtureCAN *c,uint32_t f){(void)c;(void)f;}
void DM_CAN2_CancelRead(void){g_dm_can2_read_request=0;}
void DM_CAN2_Service(uint32_t r,uint32_t b){(void)r;(void)b;}
uint32_t DM_CAN2_BuildSwitch(uint8_t cmd,CanTxMsg *tx){tx->Data[0]=cmd;return 1;}
uint32_t DM_CAN2_BuildPosVel(float p,float v,CanTxMsg *tx){(void)p;(void)v;tx->Data[0]=0xbc;return 1;}
uint32_t DM_CAN2_BuildMode2(CanTxMsg *tx){tx->Data[0]=0xcc;return 1;}
uint32_t DM_CAN2_ParseStatus(const CanRxMsg *rx,float p,float v,float t,DM_CAN2_Feedback *f)
{(void)rx;(void)p;(void)v;(void)t;if(!dm_reply)return 0;dm_reply=0;f->motor_state=1U;return 1;}
uint32_t DM_CAN2_StartupCwDelta(float a,float b,float *p){(void)a;(void)b;*p=0;return 0;}
float SourcePrep_Duration(float a,float b,float c){(void)a;(void)b;(void)c;return 1.0f;}
float SourcePrep_Smooth(float f){return f;}

static SPF_Command command(uint32_t kind,uint32_t axes)
{
 SPF_Command c={0};c.kind=kind;c.axes_mask=axes;c.sequence=71;
 c.goal.dm_deg=36.5f;c.goal.lift_pulses=600;c.keep_vm_on=1;
 c.dm_deg_s=SPF_DM_DEG_S;c.extension_rpm=SPF_EXT_RPM;c.lift_rpm=SPF_LIFT_RPM;
#if TEST_RISE_FIX
 if(kind==SPF_CMD_STAGE3_CARRY_RISE)c.lift_rpm=SPF_STAGE3_CARRY_RISE_LIFT_RPM;
#endif
 c.stepper_accel_rpm_s=SPF_STEPPER_ACCEL;c.lift_limit_pulses=SPF_STAGE3_MAX_LIFT_PULSES;
 return c;
}
static void cycle(void)
{
 tick++;g_dm_can2_feedback_tick=tick;
 VehicleSourceParallel_Service();
}
static void reset(uint32_t kind,uint32_t axes)
{
 SPF_Command c=command(kind,axes);
 memset(&vsp_group,0,sizeof(vsp_group));
 vsp_owned=stop=fail_send=inject_stop=cached=broadcasts=xz_stop=normal_moves=0;
 z_queries=0;x_cache_ack=2U;x_cache_result=1U;
 direct_ext_ack=2U;dm_tx_status=CAN_TxStatus_Ok;dm_reply=0;
 x_flags=z_flags=3;x_settled_raw=0;
 dm_tx=dm_last_tx=s_queued_request=s_pending=s_enable_possible=0;
 g_dm_posvel_holding=g_dm_jog_state=g_dm_jog_request=g_dm_jog_first_fault=g_dm_jog_stop_result=0;
 g_dm_can2_motor_state=0;g_dm_can2_logical_position_deg=110.0f;
 g_dm_can2_feedback_valid=g_extension_home_reference_valid=g_vehicle_lift_reference_valid=1;
 g_lift_ruler_controller.top_scale=100;g_lift_ruler_controller.top_position_raw=1000;
 g_lift_ruler_controller.safe_position_raw=7750;
 x_raw=5000;z_raw=TEST_CARRY_PLACE?23500:91000;x_speed=z_speed=0;
#if TEST_CARRY_PLACE
 if(kind==SPF_CMD_STAGE3_CARRY_RISE){
  SPF_Command place=command(SPF_CMD_STAGE3_CARRY_PLACE,SPF_EXT|SPF_LIFT);
  place.goal.extension_mm=40;place.goal.lift_pulses=60000;
  assert(VehicleArm_BeginStage3CarryPlace(&c,&place));
  place.goal.extension_mm=60;assert(vsp_carry_place.command.goal.extension_mm==40);
 }else
#endif
 assert(VehicleArm_BeginSourceParallel(&c,1,0));
 c.goal.dm_deg=99;assert(vsp_group.command.goal.dm_deg==36.5f);
}
static void to_broadcast(void)
{
 unsigned i;for(i=0;i<16 && vsp_group.state==1;i++)cycle();
 assert(i<16 && !g_dm_jog_request);
}
static void height_gate(void)
{
 uint32_t i;
 reset(SPF_CMD_STAGE3_LOADED_RETURN,7);
 assert(!VSP_LoadedReturnHeightReady(tick));
 to_broadcast();assert(vsp_group.state==2 && broadcasts==1 && vsp_group.started==6);
 assert(!VSP_LoadedReturnHeightReady(tick));
 x_speed=z_speed=1;z_raw=TEST_CARRY_PLACE?23512:91012;
 for(i=0;i<5;i++)cycle();
 assert(!VSP_LoadedReturnHeightReady(tick) && !g_dm_jog_request);
 z_raw=TEST_CARRY_PLACE?23500:91000;
 for(i=0;i<6 && !g_dm_jog_request;i++)cycle();
 assert(i<6 && !(vsp_group.done&SPF_EXT) && !(vsp_group.started&SPF_DM));
 assert(g_dm_jog_request==5 && g_dm_place_target_deg==36.5f && vsp_group.command.axes_mask==7);
 stop=1;JogService(1,0);assert(!dm_tx && !g_dm_jog_request && g_dm_jog_stop_result==1);
 cycle();assert(vsp_group.state==4 && vsp_group.done==0 && xz_stop==6);
 JogService(1,0);cycle();assert(!vsp_owned);
}
static void stops_and_legacy(void)
{
 reset(SPF_CMD_STAGE3_LOADED_RETURN,7);stop=1;cycle();
 assert(cached==0 && !broadcasts && xz_stop==6 && vsp_group.state==4);
 reset(SPF_CMD_STAGE3_LOADED_RETURN,7);inject_stop=SPF_EXT;to_broadcast();
 assert(cached==SPF_EXT && !broadcasts && vsp_group.stop_mask==7);cycle();assert(xz_stop==6);
 reset(SPF_CMD_STAGE3_LOADED_RETURN,7);inject_stop=SPF_LIFT;to_broadcast();
 assert(cached==6 && !broadcasts && vsp_group.stop_mask==7);cycle();assert(xz_stop==6);
 reset(SPF_CMD_STAGE3_LOADED_RETURN,7);fail_send=1;to_broadcast();
 assert(!broadcasts && !VSP_LoadedReturnHeightReady(tick) && vsp_group.stop_mask==7);
 reset(SPF_CMD_STAGE3_LOADED_RETURN,7);to_broadcast();
 cycle();cycle();assert(VSP_LoadedReturnHeightReady(tick));
 assert(!VSP_LoadedReturnHeightReady(tick+151));
 stop=1;cycle();assert(g_dm_jog_request!=4 && g_dm_jog_request!=5 && vsp_group.state==4);
 reset(SPF_CMD_STAGE3_LOADED_RETURN,7);stop=1;s_queued_request=5;
 g_dm_jog_state=6;s_enable_possible=1;JogService(1,0);
 assert(!s_queued_request && !dm_tx && g_dm_jog_state==6);
 g_dm_jog_state=8;s_enable_possible=0;s_queued_request=5;JogService(1,0);
 assert(!s_queued_request && !dm_tx);
 reset(SPF_CMD_TRANSFER,6);stop=1;to_broadcast();
 assert(vsp_group.state==2 && normal_moves==6 && !cached && !broadcasts);
 g_dm_jog_state=8;s_queued_request=5;JogService(1,0);
 assert(g_dm_jog_state==1 && !s_queued_request);
}
static void all_axes_finish(void)
{
 unsigned i;
 reset(SPF_CMD_STAGE3_LOADED_RETURN,7);to_broadcast();
 x_speed=z_speed=1;
 for(i=0;i<8 && !g_dm_jog_request;i++)cycle();
 assert(g_dm_jog_request==5 && vsp_group.started==6);
 g_dm_jog_request=0;g_dm_jog_state=4;
 cycle();assert(vsp_group.prepared==7 && vsp_group.started==6);
 cycle();assert(vsp_group.started==7 && g_dm_parallel_group_go);
 g_dm_jog_state=7;g_dm_jog_result=1;g_dm_posvel_holding=1;g_dm_can2_logical_position_deg=36.5f;
 cycle();assert(vsp_group.done==1 && vsp_owned);
 x_raw=0;z_raw=7750;x_speed=z_speed=0;
 for(i=0;i<10 && vsp_owned;i++)cycle();
 assert(vsp_group.done==7 && !vsp_owned && g_source_parallel_state==3);
}
static void waits_for_dm_recovery(void)
{
 unsigned i;
 uint32_t before;
 reset(SPF_CMD_STAGE3_LOADED_RETURN,7);to_broadcast();
 g_dm_can2_feedback_valid=0;x_raw=0;z_raw=7750;x_speed=z_speed=0;
 for(i=0;i<10 && vsp_group.done!=6;i++)cycle();
 assert(vsp_group.done==6 && !g_dm_jog_request && !vsp_dm_posted);
 before=z_queries;
 tick+=151;VehicleSourceParallel_Service();
 assert(z_queries==before && !VSP_LoadedReturnHeightReady(tick) && !g_dm_jog_request);
 g_dm_can2_feedback_valid=1;cycle();
 assert(z_queries==before+1 && VSP_LoadedReturnHeightReady(tick));
 assert(g_dm_jog_request==5 && vsp_dm_posted);

 reset(SPF_CMD_STAGE3_LOADED_RETURN,7);to_broadcast();
 g_dm_can2_feedback_valid=0;x_raw=0;z_raw=7750;
 for(i=0;i<10 && vsp_group.done!=6;i++)cycle();
 assert(vsp_group.done==6 && !vsp_dm_posted);
 before=z_queries;tick+=151;g_dm_can2_feedback_valid=1;stop=1;cycle();
 assert(z_queries==before && vsp_group.state==4 && !vsp_dm_posted);
}
static void cache_ack_zero_move(void)
{
 const uint8_t *move;
 reset(SPF_CMD_STAGE3_LOADED_RETURN,7);x_raw=0;x_cache_ack=0xe2;
 to_broadcast();
 move=vsp_axis[0].move;
 assert(!move[8] && !move[9] && !move[10] && !move[11]);
 assert(cached==SPF_EXT && !broadcasts && vsp_group.stop_mask==7);
 reset(SPF_CMD_STAGE3_LOADED_RETURN,7);x_raw=0;x_cache_ack=0x9f;
 to_broadcast();assert(cached==6 && broadcasts==1);
 reset(SPF_CMD_STAGE3_LOADED_RETURN,7);x_raw=0;x_cache_result=5;x_cache_ack=0;
 to_broadcast();assert(cached==6 && broadcasts==1);
 reset(SPF_CMD_STAGE3_FAST_TRANSFER,6);x_raw=0;x_cache_ack=0xe2;
 vsp_pick_sync=1;to_broadcast();assert(cached==6 && broadcasts==1);
}
#if TEST_EMPTY_HANDOFF
static void empty_to_dm_post(void)
{
 unsigned int i;
 reset(SPF_CMD_STAGE3_EMPTY_HANDOFF,7);
 for(i=0;i<16 && !g_dm_jog_request;i++){
  if(!(normal_moves&SPF_LIFT))assert(!g_dm_jog_request && !vsp_dm_posted);
  cycle();
 }
 assert(g_dm_jog_request==5 && vsp_dm_posted && vsp_group.started==SPF_LIFT);
 assert(normal_moves==SPF_LIFT && vsp_group.done==0 && !g_dm_stage3_empty_target_sent);
 assert(!cached && !broadcasts && !(vsp_group.release_mask&SPF_EXT));
}
static void empty_dm_real_send(void)
{
 assert(g_dm_jog_request==5);
 g_dm_can2_state=4;g_dm_can2_result=1;g_dm_can2_ctrl_mode=DM_CAN2_MODE;
 g_dm_can2_motor_state=1;g_dm_can2_pmax=12.5f;
 g_dm_can2_position_rad=110.0f*JOG_RAD_PER_DEG;g_dm_can2_velocity_rad_s=0;
 g_dm_posvel_holding=1;g_dm_jog_state=7;
 JogService(1,0);
 assert(dm_last_tx==0xbc && dm_tx==1 && s_stage3_empty_target_pending);
 assert(!g_dm_stage3_empty_target_sent);
}
static void empty_handoff(void)
{
 unsigned int i;
 empty_to_dm_post();
 /* A preload/enable send and a transmit-pending target cannot release X. */
 g_dm_jog_state=3;s_point_goal=36.5f;s_target=1.0f;
 JogReference(0.5f,tick);assert(!s_stage3_empty_target_pending);(void)JogPoll(tick);
 assert(!g_dm_stage3_empty_target_sent);
 dm_tx=0;empty_dm_real_send();
 dm_tx_status=2U;(void)JogPoll(tick);
 cycle();cycle();assert(!(vsp_group.release_mask&SPF_EXT));
 assert(vsp_group.started==(SPF_LIFT|SPF_DM) && vsp_group.done==0);
 dm_tx_status=CAN_TxStatus_Ok;(void)JogPoll(tick);
 assert(g_dm_stage3_empty_target_sent);
 for(i=0;i<10 && !(normal_moves&SPF_EXT);i++)cycle();
 assert(normal_moves==6 && vsp_group.started==7 && !vsp_group.done && !broadcasts);
 g_dm_jog_state=7;g_dm_jog_result=1;g_dm_posvel_holding=1;g_dm_can2_logical_position_deg=36.5f;
 x_raw=0;z_raw=7750;
 for(i=0;i<10 && vsp_owned;i++)cycle();
 assert(vsp_group.done==7 && !vsp_owned);

 empty_to_dm_post();stop=1;JogService(1,0);assert(dm_last_tx!=0xbc);
 cycle();assert(vsp_group.state==4 && normal_moves==SPF_LIFT);
 empty_to_dm_post();empty_dm_real_send();stop=1;JogService(1,0);
 assert(!g_dm_stage3_empty_target_sent);cycle();assert(vsp_group.state==4 && normal_moves==SPF_LIFT);
 empty_to_dm_post();empty_dm_real_send();dm_tx_status=CAN_TxStatus_Failed;
 JogService(1,0);cycle();assert(vsp_group.state==4 && normal_moves==SPF_LIFT);
 empty_to_dm_post();empty_dm_real_send();(void)JogPoll(tick);
 direct_ext_ack=0xe2;
 for(i=0;i<10 && vsp_group.state!=4;i++)cycle();
 assert(vsp_group.state==4 && vsp_group.stop_mask==7);
}
#endif
#if TEST_CARRY_PLACE
static void carry_post(void)
{
 unsigned int i;
 reset(SPF_CMD_STAGE3_CARRY_RISE,7);x_raw=0;z_raw=23512;
 for(i=0;i<16 && !(normal_moves&SPF_LIFT);i++)cycle();
 assert(i<16 && normal_moves==SPF_LIFT && !g_dm_jog_request);
#if TEST_RISE_FIX
 assert(vsp_axis[1].move[1]==1 && vsp_axis[1].move[6]==0x27 && vsp_axis[1].move[7]==0x10);
 assert(vsp_axis[1].move[2]==0xaf && vsp_axis[1].move[3]==0xff && vsp_axis[1].move[4]==0xaf && vsp_axis[1].move[5]==0xff);
#endif
 for(i=0;i<6;i++)cycle();
 assert(!g_dm_jog_request && !vsp_carry_leg);
 z_raw=23500;
 for(i=0;i<6 && !g_dm_jog_request;i++)cycle();
 assert(g_dm_jog_request==5 && !vsp_carry_leg && !cached);
}
static void carry_confirm_dm(void)
{
 empty_dm_real_send();
 g_dm_can2_logical_position_deg=39.5f;
 (void)JogPoll(tick);
 assert(g_dm_stage3_empty_target_sent && !g_dm_stage3_carry_target_feedback);
 dm_reply=1;(void)JogPoll(tick);
 assert(g_dm_stage3_carry_target_feedback);
}
static void carry_pipeline(void)
{
 uint32_t i,x_sample,z_sample;
 carry_post();carry_confirm_dm();
 assert(!vsp_carry_leg);
 z_speed=1;z_raw=7750;
 for(i=0;i<5;i++)cycle();
 assert(!vsp_carry_leg && !cached);
 z_speed=0;
 for(i=0;i<6 && !vsp_carry_leg;i++)cycle();
 assert(vsp_carry_leg && !cached && !(g_source_parallel_done_mask&SPF_LIFT));
 g_dm_can2_feedback_valid=0;
 for(i=0;i<12 && cached!=6;i++)cycle();
 assert(cached==6 && !broadcasts && !(g_source_parallel_done_mask&SPF_EXT));
#if TEST_RISE_FIX
 assert(vsp_axis[1].move[1]==0 && vsp_axis[1].move[6]==0x48 && vsp_axis[1].move[7]==0x20);
#endif
 x_sample=vsp_axis[0].position_cycle;z_sample=vsp_axis[1].position_cycle;
 tick+=10U;
#if TEST_ORIGIN_FIX
 x_raw=2;z_raw=7749;
#endif
 g_dm_can2_feedback_valid=1;cycle();
 assert(broadcasts==1 && vsp_carry_sent && !g_source_parallel_done_mask);
 assert(vsp_axis[0].position_cycle!=x_sample && vsp_axis[1].position_cycle!=z_sample);
 assert(vsp_axis[1].before==77.5f && vsp_group.command.goal.lift_pulses==600);
 g_dm_jog_state=7;g_dm_jog_result=1;g_dm_posvel_holding=1;g_dm_can2_logical_position_deg=36.5f;
 cycle();assert(g_source_parallel_done_mask==SPF_DM && vsp_owned);
 x_raw=(uint32_t)(40.0f*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f*10.0f+.5f);
 z_raw=676000;
 for(i=0;i<8 && vsp_owned;i++)cycle();
 assert(!vsp_owned && g_source_parallel_done_mask==7 && g_source_parallel_state==3U);

 carry_post();carry_confirm_dm();z_raw=7750;
 for(i=0;i<8 && !vsp_carry_leg;i++)cycle();
 assert(vsp_carry_leg);
 g_dm_can2_feedback_valid=0;
 for(i=0;i<12 && cached!=6;i++)cycle();
 assert(cached==6 && !broadcasts);
 tick+=10U;x_raw=TEST_ORIGIN_FIX?178:1;g_dm_can2_feedback_valid=1;cycle();
 assert(!broadcasts && vsp_group.state==4 && vsp_group.fault==6U && vsp_group.stop_mask==7);

 carry_post();stop=1;JogService(1,0);cycle();
 assert(vsp_group.state==4 && !cached && !broadcasts && !g_dm_stage3_carry_target_feedback);
 carry_post();carry_confirm_dm();z_raw=7750;
 for(i=0;i<8 && !vsp_carry_leg;i++)cycle();
 assert(vsp_carry_leg);inject_stop=SPF_LIFT;
 for(i=0;i<16 && vsp_group.state!=4;i++)cycle();
 assert(vsp_group.state==4 && !broadcasts && g_source_parallel_stop_mask==7);
}
#if TEST_ORIGIN_FIX
static void origin_window(void)
{
 VSP_Axis x={0},z={0};
 const uint8_t x_frame[15]={0xfd,0,2,0x58,2,0x58,5,0xdc,0,0,0x3b,0xae,2,1,0x6b};
 const uint8_t z_frame[15]={0xfd,0,0xaf,0xff,0xaf,0xff,0x48,0x20,0,0x0a,0x81,0x77,2,1,0x6b};
 reset(SPF_CMD_STAGE3_CARRY_RISE,7);
 memcpy(x.move,x_frame,sizeof x_frame);memcpy(z.move,z_frame,sizeof z_frame);
 x.current=-1.4f;x.goal=1526.231201171875f;
 z.current=68.70f;z.goal=6953.740234375f;
 assert(VSP_CarryOriginWithin(&x,0,SOURCE_EXTENSION_ZERO_DONE_DEG,SOURCE_EXTENSION_DONE_DEG,10));
 assert(VSP_CarryOriginWithin(&z,68.7f,SOURCE_TOP_TOLERANCE_DEG,22.5f,100));
 z.current=91.19f;z.move[11]+=3;
 assert(!VSP_CarryOriginWithin(&z,68.7f,SOURCE_TOP_TOLERANCE_DEG,22.5f,100));
 x.current=17.5f;x.goal=0;x.move[1]=0;
 x.move[8]=x.move[9]=x.move[10]=0;x.move[11]=3;
 assert(!VSP_CarryOriginWithin(&x,0,SOURCE_EXTENSION_ZERO_DONE_DEG,SOURCE_EXTENSION_ZERO_DONE_DEG,10));
 x_flags=3;x_speed=1;assert(VSP_CarryOriginHealthy(6U)==(TEST_SETTLE_FIX?0U:2U));
 x_speed=0;x_flags=5;assert(VSP_CarryOriginHealthy(6U)==2U);
 x_flags=3;assert(VSP_CarryOriginHealthy(6U)==1U);
}
#endif
#if TEST_RISE_FIX
static void rise_speed_scope(void)
{
 SPF_Group test={0};
 SPF_GroupIO io={0,VSP_Prepare,VSP_Start,VSP_Poll,VSP_Stop,VSP_Clock};
 SPF_Command rise=command(SPF_CMD_STAGE3_CARRY_RISE,7);
 SPF_Command place=command(SPF_CMD_STAGE3_CARRY_PLACE,6);
 SPF_Command recovery=command(SPF_CMD_STAGE3_LOADED_RETURN,7);
 rise.lift_rpm=SPF_LIFT_RPM;assert(!SPF_GroupBegin(&test,&rise,&io,1,90,12000));
 place.lift_rpm=SPF_STAGE3_CARRY_RISE_LIFT_RPM;assert(!SPF_GroupBegin(&test,&place,&io,1,90,12000));
 recovery.lift_rpm=SPF_STAGE3_CARRY_RISE_LIFT_RPM;assert(!SPF_GroupBegin(&test,&recovery,&io,1,90,12000));
 reset(SPF_CMD_STAGE3_LOADED_RETURN,7);to_broadcast();
 assert(broadcasts==1 && vsp_axis[1].move[6]==0x48 && vsp_axis[1].move[7]==0x20);
 assert(vsp_axis[0].move[6]==0x05 && vsp_axis[0].move[7]==0xdc);
}
#endif
#if TEST_SETTLE_FIX
static void settle_to_cached(void)
{
 unsigned i;
 carry_post();carry_confirm_dm();z_raw=7750;
 for(i=0;i<8 && !vsp_carry_leg;i++)cycle();
 assert(vsp_carry_leg);g_dm_can2_feedback_valid=0;
 for(i=0;i<12 && cached!=6;i++)cycle();
 assert(cached==6 && !broadcasts);g_dm_can2_feedback_valid=1;
}
static void settle_wait(void)
{
 uint32_t x,z;
 settle_to_cached();x_speed=5;
 cycle();cycle();
 assert(vsp_group.state!=4 && !broadcasts && !vsp_carry_origin_sampled);
 assert(g_vsp_carry_wait_speed_count==2 && g_vsp_carry_wait_speed_node==6 && g_vsp_carry_wait_speed_raw==5);
 assert(!g_vsp_carry_first_stage);
 x=vsp_axis[0].position_cycle;z=vsp_axis[1].position_cycle;
 x_speed=0;cycle();assert(broadcasts==1 && vsp_carry_sent && !g_vsp_carry_first_stage);
 assert(vsp_axis[0].position_cycle!=x && vsp_axis[1].position_cycle!=z);
 settle_to_cached();x_speed=5;cycle();
 assert(!broadcasts && !g_vsp_carry_first_stage);
 /* The X origin changes as the drive first reports zero, after any stale pre-stop read. */
 x_speed=0;x_settled_raw=178;cycle();
 assert(!broadcasts && vsp_axis[0].position_raw==178 && vsp_group.fault==6 &&
       g_vsp_carry_first_stage==VSP_CARRY_DIAG_ORIGIN && g_vsp_carry_first_node==6);
 settle_to_cached();z_flags=5;cycle();
 assert(!broadcasts && vsp_group.fault==6 && g_vsp_carry_first_stage==VSP_CARRY_DIAG_FLAGS);
 assert(g_vsp_carry_first_node==5 && g_vsp_carry_first_command==0x3a &&
        g_vsp_carry_first_exchange==1 && g_vsp_carry_first_flags==5 && g_vsp_carry_first_sequence==71);
 settle_to_cached();x_speed=5;cycle();tick=vsp_group.begin_ms+vsp_group.timeout_ms;
 cycle();assert(!broadcasts && vsp_group.fault==3 && g_vsp_carry_first_stage==VSP_CARRY_DIAG_TIMEOUT);
 settle_to_cached();fail_send=1;cycle();
 assert(!broadcasts && vsp_group.fault==6 && g_vsp_carry_first_stage==VSP_CARRY_DIAG_BROADCAST &&
        g_vsp_carry_first_broadcast_result==4);
}
#endif
#endif
int main(void)
{
 height_gate();stops_and_legacy();all_axes_finish();waits_for_dm_recovery();cache_ack_zero_move();
 puts("PASS lower: immutable XYZ, fresh 8000p gate, STOP boundaries and DM queue, all-axis completion, stale DM recovery after Z done, X=0 cache ACK and legacy compatibility");
#if TEST_EMPTY_HANDOFF
 empty_handoff();
 puts("PASS empty handoff: Z then real DM CAN TX confirmation then X, independent axis completion, send failure, STOP and X rejection");
#endif
#if TEST_CARRY_PLACE
 carry_pipeline();
 puts("PASS carry/place: 2000p actual rise, DM TX and feedback, Z stop before handoff, stale yaw, post-cache origins unchanged or drift at 10ms, STOP and combined completion");
#endif
#if TEST_ORIGIN_FIX
 origin_window();
 puts("PASS origin fix: actual incident frames, endpoint window, X=0 tighter endpoint, health and zero speed");
#endif
#if TEST_RISE_FIX
 rise_speed_scope();
 puts("PASS carry rise rate: actual 0x2710 rise, 0x4820 placement and recovery, unchanged X rate and rise acceleration, strict admission by command kind");
#endif
#if TEST_SETTLE_FIX
 settle_wait();
 puts("PASS carry settle: moving polls wait, both zero checks precede fresh origins; at-zero drift blocks broadcast, unchanged origin permits it; flags, timeout and TX failure remain distinct");
#endif
 return 0;
}
