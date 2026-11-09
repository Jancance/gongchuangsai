
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#define DM_CAN2_H
#define DM_CAN2_MODE 2U
#define DM_CAN2_LEFT_ZERO_DEG (-296.544037F)
#define DM_CAN2_POSVEL_SPEED_DEG_S 300.0F
#define DM_CAN2_JOG_DEG 45.0F
#define CAN_TxStatus_NoMailBox 4U
#define CAN_TxStatus_Ok 1U
#define CAN_TxStatus_Failed 0U
#define CAN_FIFO0 0U
#define CAN_FLAG_FOV0 1U
#define CAN_ESR_BOFF 128U
#define RESET 0U
typedef struct {uint32_t cmd;float target,speed;} CanTxMsg;
typedef struct {uint8_t Data[8];} CanRxMsg;
typedef struct {uint32_t position_raw,velocity_raw,torque_raw,motor_state;float position_rad,velocity_rad_s,torque_nm;} DM_CAN2_Feedback;
static struct {uint32_t ESR;} fake_can;
#define CAN2 (&fake_can)
volatile uint32_t g_dm_can2_ctrl_mode;
volatile uint32_t g_dm_can2_esc_id;
volatile uint32_t g_dm_can2_feedback_tick;
volatile uint32_t g_dm_can2_feedback_valid;
CanRxMsg g_dm_can2_last_rx;
volatile float g_dm_can2_logical_position_deg;
volatile uint32_t g_dm_can2_master_id;
volatile uint32_t g_dm_can2_motor_state;
volatile float g_dm_can2_pmax;
volatile float g_dm_can2_position_deg;
volatile float g_dm_can2_position_rad;
volatile uint32_t g_dm_can2_position_raw;
volatile uint32_t g_dm_can2_range_mask;
volatile uint32_t g_dm_can2_read_cycle;
volatile uint32_t g_dm_can2_read_kind;
volatile uint32_t g_dm_can2_read_request;
volatile uint32_t g_dm_can2_result;
volatile uint32_t g_dm_can2_state;
volatile float g_dm_can2_tmax;
volatile float g_dm_can2_torque_nm;
volatile uint32_t g_dm_can2_torque_raw;
volatile uint32_t g_dm_can2_valid_mask;
volatile float g_dm_can2_velocity_rad_s;
volatile uint32_t g_dm_can2_velocity_raw;
volatile float g_dm_can2_vmax;
static uint32_t fixture_dm_fail;
static uint32_t tick=100,pending,motor,tx_count,enables,disables;
static float actual,last_target,last_speed;
uint32_t xTaskGetTickCount(void){return tick;}
uint32_t CAN_MessagePending(void *p,uint32_t fifo){return pending;}
void CAN_Receive(void *p,uint32_t fifo,CanRxMsg *rx){memset(rx,0,sizeof(*rx));pending=0;}
void CAN_ClearFlag(void *p,uint32_t flag){}
uint32_t CAN_GetFlagStatus(void *p,uint32_t flag){return 0;}
uint32_t CAN_TransmitStatus(void *p,uint32_t m){return CAN_TxStatus_Ok;}
void CAN_CancelTransmit(void *p,uint32_t m){pending=0;}
uint32_t CAN_Transmit(void *p,CanTxMsg *tx){
 tx_count++;pending=1;
 if(tx->cmd==0xfc){motor=1;enables++;}
 else if(tx->cmd==0xfd){motor=0;disables++;}
 else {last_target=tx->target;last_speed=tx->speed;actual=last_target;}
 return 0;
}
void DM_CAN2_CancelRead(void){g_dm_can2_read_request=0;}
uint32_t DM_CAN2_BuildSwitch(uint8_t c,CanTxMsg *tx){memset(tx,0,sizeof(*tx));tx->cmd=c;return 1;}
uint32_t DM_CAN2_BuildMode2(CanTxMsg *tx){memset(tx,0,sizeof(*tx));tx->cmd=0;return 1;}
uint32_t DM_CAN2_BuildPosVel(float p,float v,CanTxMsg *tx){memset(tx,0,sizeof(*tx));tx->cmd=1;tx->target=p;tx->speed=v;return 1;}
uint32_t DM_CAN2_ParseStatus(CanRxMsg *rx,float a,float b,float c,DM_CAN2_Feedback *f){
 memset(f,0,sizeof(*f));f->motor_state=fixture_dm_fail?9:motor;f->position_rad=actual;return 1;
}
void DM_CAN2_Service(uint32_t ready,uint32_t busy){
 if(!g_dm_can2_read_request)return;g_dm_can2_read_request=0;g_dm_can2_read_cycle++;
 g_dm_can2_state=4;g_dm_can2_result=1;g_dm_can2_read_kind=2;g_dm_can2_valid_mask=g_dm_can2_range_mask=7;
 g_dm_can2_feedback_valid=1;g_dm_can2_feedback_tick=tick;g_dm_can2_motor_state=motor;
 g_dm_can2_esc_id=1;g_dm_can2_master_id=0;g_dm_can2_ctrl_mode=2;
 g_dm_can2_pmax=12.5f;g_dm_can2_vmax=30;g_dm_can2_tmax=10;
 g_dm_can2_position_rad=actual;g_dm_can2_position_deg=actual*57.2957795f;g_dm_can2_velocity_rad_s=0;
}

#define s_cycle dm_cycle
#include "dm_can2_motion.c"
#undef s_cycle
#define APP_CONFIG_H
#define CHASSIS_AUTO_TEST_H
typedef struct{uint32_t active,result,cause,rotation_stop_mask;} ChassisAutoStatus;
static uint32_t fixture_holding=1,fixture_stopped=1,fixture_can_fail,fixture_protect,fixture_servo_fail,fixture_stuck;
static uint32_t moves,grips,source_tick,ext_first_tick,grip_first_tick,DM_first_tick,transition_seen;
static float ext_position,ext_goal,ext_velocity,last_grip=145;
volatile uint32_t g_startup_extension_feedback_valid=1,g_extension_hold_confirmed=1;
volatile float g_startup_extension_current_deg=0;
uint32_t StartupAxes_Holding(void){return fixture_holding;}
void StartupAxes_Cancel(void){fixture_holding=0;}
void StartupAxes_SetExtensionHoldTarget(float p){(void)p;}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *p){memset(p,0,sizeof(*p));p->active=!fixture_stopped;p->result=1;p->rotation_stop_mask=15;return 1;}
void Gripper_Init(void){}
uint8_t jiazhua(float v){if(fixture_servo_fail)return 0;if(!(v>=145 && v<=180))return 0;grips++;if(v>145 && !grip_first_tick)grip_first_tick=tick;last_grip=v;return 1;}
uint32_t CarrierPwm_Write(uint32_t p){(void)p;return 1;}
#include "vehicle_arm.c"
uint32_t VehicleArm_Exchange(uint8_t a,const uint8_t *q,uint8_t n,uint8_t rn,void *out){
 CanRxMsg *p=out;uint32_t raw;memset(p,0,sizeof(*p));p->Data[0]=q[0];p->Data[1]=2;
 if(fixture_can_fail)return 5;
 if(a!=6)return 5;
 if(q[0]==0x1a)p->Data[2]=0x80;
 if(q[0]==0x36){raw=(uint32_t)(fabsf(ext_position)*10+.5f);p->Data[1]=ext_position<0;p->Data[2]=raw>>24;p->Data[3]=raw>>16;p->Data[4]=raw>>8;p->Data[5]=raw;}
 if(q[0]==0x35){raw=fabsf(ext_velocity)>.2f?1:0;p->Data[3]=raw;}
 if(q[0]==0x3a)p->Data[1]=fixture_protect?9:(fabsf(ext_position-ext_goal)<.2f && fabsf(ext_velocity)<.2f?3:1);
 if(q[0]==0xfd){
  raw=((uint32_t)q[8]<<24)|((uint32_t)q[9]<<16)|((uint32_t)q[10]<<8)|q[11];
  if(n!=15 || q[12]!=2 || raw>18000 || ((((uint32_t)q[2]<<8)|q[3])!=60) || ((((uint32_t)q[6]<<8)|q[7])!=300))return 5;
  ext_goal=ext_position+(q[1]?-1:1)*(float)raw/100;ext_first_tick=ext_first_tick?ext_first_tick:tick;moves++;
 }
 if(q[0]==0xfe){ext_goal=ext_position;ext_velocity=0;}
 (void)rn;return 1;
}
static unsigned checks,failures;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL line%u check%u\n",__LINE__,checks);}}while(0)
static void fixture_step(unsigned dt){
 float remain=ext_goal-ext_position,desired=0,dv;
 if(!fixture_stuck){
  desired=(remain<0?-1:1)*fminf(180,sqrtf(720*fabsf(remain)));
  dv=desired-ext_velocity;float maxdv=360*(float)dt*.001f;
  ext_velocity+=dv>maxdv?maxdv:dv< -maxdv?-maxdv:dv;
  float step=ext_velocity*(float)dt*.001f;
  if(fabsf(step)>=fabsf(remain)){ext_position=ext_goal;ext_velocity=0;}else ext_position+=step;
 }
 tick+=dt;DM_CAN2_ModuleService(fixture_holding,0);
 if(g_dm_jog_state==8 && g_dm_jog_result==2 && s_queued_request){
  CK(DM_CAN2_SourceTransitionPending());transition_seen++;
  uint32_t saved=g_dm_jog_stop_result;g_dm_jog_stop_result=2;CK(!DM_CAN2_SourceTransitionPending());g_dm_jog_stop_result=saved;
 }
 if(g_dm_source_go && g_dm_jog_state==4 && !DM_first_tick)DM_first_tick=tick;
 if(VehicleArm_Busy() || g_vehicle_arm_stop_pending)VehicleArm_Service();
 DM_CAN2_ModuleService(fixture_holding,0);
}
static void reset(float home){
 tick=100;pending=motor=tx_count=enables=disables=0;actual=DM_CAN2_LEFT_ZERO_DEG*.01745329252f;
 g_dm_jog_state=g_dm_jog_result=g_dm_posvel_holding=g_dm_jog_request=g_dm_startup_go=0;
 s_enable_possible=s_pending=s_tx_done=s_queued_request=s_mode_pending=0;s_point_mode=0;
 g_dm_can2_read_cycle=g_dm_can2_read_request=0;g_dm_point_enable=1;g_dm_posvel_speed_deg_s=150;
 g_dm_jog_first_fault=g_dm_integrated_owner=g_dm_source_smooth=g_dm_source_go=0;fixture_dm_fail=0;
 fixture_holding=fixture_stopped=1;fixture_can_fail=fixture_protect=fixture_servo_fail=fixture_stuck=0;
 ext_position=ext_goal=home;ext_velocity=0;last_grip=145;moves=grips=source_tick=ext_first_tick=grip_first_tick=DM_first_tick=transition_seen=0;
 g_startup_extension_current_deg=home;g_startup_extension_feedback_valid=g_extension_hold_confirmed=1;
 g_source_prepare_state=g_source_prepare_cause=g_source_prepare_done_mask=g_vehicle_arm_state=g_vehicle_arm_axis_owned=g_vehicle_arm_stop_pending=0;
 g_dm_startup_go=1;g_dm_jog_request=6;
 for(unsigned i=0;i<200 && !g_dm_posvel_holding;i++)fixture_step(5);
 CK(g_dm_posvel_holding && fabsf(g_dm_can2_logical_position_deg)<.1f);
}
int main(void){
 FILE *trace=fopen("source-trace.csv","w");fprintf(trace,"case,t_ms,prepare_state,DM_reference_deg,extension_mm,grip_deg,done_mask\n");
 for(unsigned scenario=0;scenario<13;scenario++){
  reset(scenario==1?-31.5f:scenario==2?12.3f:-1.3f);
  if(scenario==3)fixture_stopped=0;
  CK(VehicleArm_BeginSourcePreparation()==(scenario==3?0:1));source_tick=tick;
  float previous_grip=145,previous_ref=actual*57.2957795f;uint32_t previous_tick=tick;
  if(scenario!=3)for(unsigned i=0;i<4000 && VehicleArm_Busy();i++){
   if(scenario==4 && g_source_prepare_state==1)fixture_can_fail=1;
   if(scenario==5 && g_source_prepare_state==2)fixture_dm_fail=1;
   if(scenario==6 && g_source_prepare_state==2)fixture_protect=1;
   if(scenario==7 && g_source_prepare_state==2)fixture_stuck=1;
   if(scenario==8 && g_source_prepare_state==2)fixture_servo_fail=1;
   if(scenario==9 && g_source_prepare_state==2)VehicleArm_Cancel();
   if(scenario==10 && g_source_prepare_state==2)fixture_holding=0;
   if(scenario==12 && g_source_prepare_state==1 && sp_phase==1)ext_position+=10;
   fixture_step(scenario==11 && g_source_prepare_state==2?151:10);
   CK(last_grip>=previous_grip-.001f && last_grip<=180.001f);
   if(scenario<=2 && g_dm_source_go){
    float ref=g_dm_jog_reference_deg;
    CK(ref>=previous_ref-.001f && fabsf(ref-previous_ref)<=90*(tick-previous_tick)*.001f+.01f);
    previous_ref=ref;
   }
   previous_grip=last_grip;previous_tick=tick;
   if(scenario<=2 && !(i%10))fprintf(trace,"%u,%u,%u,%.5f,%.5f,%.5f,%u\n",scenario,tick-source_tick,g_source_prepare_state,g_dm_jog_reference_deg-DM_CAN2_LEFT_ZERO_DEG,g_source_prepare_extension_mm,last_grip,g_source_prepare_done_mask);
  }
  if(scenario<=2){
   CK(g_source_prepare_state==3 && g_vehicle_arm_state==2 && g_source_prepare_done_mask==7 && fixture_holding);
   CK(moves==2 && fabsf(g_source_prepare_extension_mm-70)<.1f && fabsf(last_grip-180)<.01f && g_dm_posvel_holding);
   CK(transition_seen>0);
   CK(fabsf(ext_position-(g_startup_extension_current_deg-252))<.2f);
   CK(ext_first_tick==g_source_prepare_release_ms && grip_first_tick-g_source_prepare_release_ms<=50 && DM_first_tick-g_source_prepare_release_ms<=50);
  }else if(scenario!=3){CK(g_source_prepare_state==4 && !fixture_holding && g_vehicle_arm_state==3);}
  printf("SOURCE case%u state%u cause%u mask%u moves%u elapsed%u\n",scenario,g_source_prepare_state,g_source_prepare_cause,g_source_prepare_done_mask,moves,tick-source_tick);
 }
 fclose(trace);printf("SOURCE checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
