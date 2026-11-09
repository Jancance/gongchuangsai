
#include "source_parallel_flow.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#define APP_MOTOR_OFFLINE_ONLY 0
#define VEHICLE_EXTENSION_PULSES_PER_MM 32.0f
#define STARTUP_EXTENSION_EXTEND_SIGN (-1)
#define VEHICLE_LIFT_TOP_DEG 0.0f
#define VEHICLE_LIFT_DOWN_SIGN 1
#define VEHICLE_AXIS_TIMEOUT_MS 12000
static unsigned tick,holding=1,body_active,fail35,stop_retry;
static unsigned checks,failures;
#define CK(v) do{checks++;if(!(v)){failures++;printf("FAIL %d %s\n",__LINE__,#v);}}while(0)
typedef struct{unsigned char Data[8];} Reply;
static Reply s_rx;
volatile unsigned g_dm_parallel_group_active,g_dm_parallel_group_go,g_dm_can2_feedback_valid=1;
volatile unsigned g_extension_home_reference_valid=1,g_vehicle_lift_reference_valid=1;
volatile float g_extension_home_reference_deg=10,g_vehicle_lift_current_deg;
volatile unsigned g_vehicle_lift_feedback_valid,g_dm_can2_feedback_tick,g_dm_can2_read_cycle;
volatile unsigned g_dm_source_smooth,g_dm_point_enable,g_dm_jog_request,g_dm_jog_state=7,g_dm_jog_result=1;
volatile unsigned g_dm_jog_first_fault,g_dm_can2_motor_state=1,g_dm_posvel_holding=1,g_dm_jog_stop_result;
volatile float g_dm_posvel_speed_deg_s,g_dm_pick_target_deg,g_dm_place_target_deg,g_dm_can2_logical_position_deg;
volatile unsigned g_vehicle_arm_dm_owned,g_dm_integrated_owner,g_vehicle_arm_axis_owned,g_vehicle_arm_state,g_vehicle_arm_cause;
typedef struct{float current,goal;unsigned finish,started,move_count,stop_calls,speed,accel;} Driver;
static Driver motor[2];static float hold_target;static unsigned dm_finish;
unsigned xTaskGetTickCount(void){return tick;}
unsigned StartupAxes_Holding(void){return holding;}
void StartupAxes_SetExtensionHoldTarget(float value){hold_target=value;}
unsigned VehicleArm_Busy(void){return g_vehicle_arm_state==1;}
static unsigned exchange(unsigned char addr,const unsigned char *q,unsigned char n,unsigned char reply_n,void *out){
 Driver *a=&motor[addr==6?0:1];unsigned raw;Reply *r=out;(void)n;(void)reply_n;memset(r,0,sizeof(*r));r->Data[0]=q[0];
 if(q[0]==0x1a)r->Data[2]=0x80;
 else if(q[0]==0x36){raw=(unsigned)(fabsf(a->current)*100+.5f);r->Data[1]=a->current<0;r->Data[2]=raw>>24;r->Data[3]=raw>>16;r->Data[4]=raw>>8;r->Data[5]=raw;}
 else if(q[0]==0x35){r->Data[3]=a->started && tick<a->finish;}
 else if(q[0]==0x3a){r->Data[1]=3;if(fail35 && addr==5 && a->started)r->Data[1]|=8;}
 else if(q[0]==0xf3)r->Data[1]=2;
 else if(q[0]==0xfd){
  a->speed=((unsigned)q[6]<<8)|q[7];a->accel=((unsigned)q[2]<<8)|q[3];
  raw=((unsigned)q[8]<<24)|((unsigned)q[9]<<16)|((unsigned)q[10]<<8)|q[11];
  a->goal=a->current+(q[1]?-1:1)*(float)raw/100;a->started=1;
  a->finish=tick+(addr==6?800:1741);a->move_count++;r->Data[1]=2;
 }else if(q[0]==0xfe){a->stop_calls++;r->Data[1]=a->stop_calls>stop_retry?2:0;if(r->Data[1]==2)a->started=0;}
 else return 0;return 1;
}
unsigned VehicleArm_Exchange(unsigned char a,const unsigned char *q,unsigned char n,unsigned char r,void *out){return exchange(a,q,n,r,out);}
volatile float g_startup_extension_current_deg;
volatile unsigned g_startup_extension_feedback_valid;
static unsigned SingleTeach_ReferencesReady(void){return holding;}
static float SingleTeach_TopDeg(void){return 12.34f;}
#include "vehicle_source_parallel.inc"
int ChassisAuto_ReadStatus(ChassisAutoStatus *b){memset(b,0,sizeof(*b));b->active=body_active;b->result=1;b->rotation_stop_mask=15;return 1;}
static void step(void){
 tick+=5;g_dm_can2_feedback_tick=tick;g_dm_can2_read_cycle++;
 if(g_dm_jog_request==3){g_dm_jog_request=0;g_dm_jog_state=8;g_dm_jog_stop_result=1;g_dm_posvel_holding=0;}
 else if(g_dm_jog_request==4 || g_dm_jog_request==5){g_dm_jog_request=0;g_dm_jog_state=3;g_dm_posvel_holding=0;}
 if(g_dm_jog_state==3 && g_dm_parallel_group_go){g_dm_jog_state=4;dm_finish=tick+500;}
 if(g_dm_jog_state==4 && tick>=dm_finish){g_dm_can2_logical_position_deg=g_dm_pick_target_deg;g_dm_jog_state=7;g_dm_jog_result=1;g_dm_posvel_holding=1;}
 for(unsigned i=0;i<2;i++)if(motor[i].started && tick>=motor[i].finish)motor[i].current=motor[i].goal;
 VehicleSourceParallel_Service();
}
static void reset(void){
 memset(&vsp_group,0,sizeof(vsp_group));memset(motor,0,sizeof(motor));motor[0].current=10;motor[1].current=12.34f;
 vsp_owned=tick=g_vehicle_arm_state=g_vehicle_arm_cause=body_active=fail35=stop_retry=0;
 g_vehicle_lift_reference_valid=g_extension_home_reference_valid=g_dm_can2_feedback_valid=holding=1;
 g_dm_jog_state=7;g_dm_jog_result=g_dm_posvel_holding=g_dm_can2_motor_state=1;
 g_dm_jog_request=g_dm_jog_first_fault=g_dm_parallel_group_active=g_dm_parallel_group_go=0;
 g_dm_can2_feedback_tick=g_dm_can2_read_cycle=0;g_dm_can2_logical_position_deg=0;
}
int main(void){
 SPF_Command c={0};c.run_id=42;c.pose_version=7;c.kind=SPF_CMD_PICK;c.axes_mask=7;
 c.goal=(SPF_Pose){193.45f,164.07f,108400};c.dm_deg_s=300;c.extension_rpm=106;c.lift_rpm=1846.4f;c.stepper_accel_rpm_s=45055;c.keep_vm_on=1;
 reset();g_vehicle_lift_reference_valid=0;CK(!VehicleArm_BeginSourceParallel(&c,1,0));g_vehicle_lift_reference_valid=1;
 CK(!VehicleArm_BeginSourceParallel(&c,0,0));body_active=1;CK(!VehicleArm_BeginSourceParallel(&c,1,0));body_active=0;
 CK(VehicleArm_BeginSourceParallel(&c,1,0));CK(!VehicleArm_BeginSourceParallel(&c,1,0));
 for(unsigned i=0;i<800 && VehicleSourceParallel_Owns();i++)step();
 CK(!VehicleSourceParallel_Owns() && g_vehicle_arm_state==2 && g_source_parallel_done_mask==7);
 CK(motor[0].move_count==1 && motor[1].move_count==1);CK(motor[0].speed==1060 && motor[1].speed==18464);
 CK(motor[0].accel==45055 && motor[1].accel==45055);
 CK(fabsf(motor[0].goal-(10-164.07f*3.6f))<.02f);CK(fabsf(motor[1].goal-12207.34f)<.02f);
 CK(fabsf(hold_target-motor[0].goal)<.02f && g_vehicle_lift_feedback_valid);
 CK(!g_dm_parallel_group_active && !g_vehicle_arm_axis_owned);
 reset();c.kind=SPF_CMD_ROUTE_AND_STOW;body_active=1;CK(!VehicleArm_BeginSourceParallel(&c,1,0));
 CK(VehicleArm_BeginSourceParallel(&c,1,1));CK(VehicleArm_SourceParallelAllowsBody());
 VehicleSourceParallel_Cancel();CK(!VehicleArm_SourceParallelAllowsBody());
 for(unsigned i=0;i<50 && VehicleSourceParallel_Owns();i++)step();CK(!VehicleSourceParallel_Owns());
 reset();c.kind=SPF_CMD_PICK;CK(VehicleArm_BeginSourceParallel(&c,1,0));fail35=1;stop_retry=2;
 for(unsigned i=0;i<900 && VehicleSourceParallel_Owns();i++)step();
 CK(g_vehicle_arm_state==3 && !g_source_parallel_stop_mask && !VehicleSourceParallel_Owns());
 CK(motor[0].stop_calls>=3 && motor[1].stop_calls>=3 && g_dm_jog_stop_result==1);
 CK(!VehicleArm_BeginSourceParallel(&c,1,0));

 reset();vsp_owned=1;vsp_group.state=1;vsp_group.command.axes_mask=SPF_LIFT;
 {unsigned char enable[5]={0xf3,0xab,1,0,0x6b};unsigned char wrong[15]={0};
  vsp_axis[1].phase=3;CK(VehicleArm_SourceParallelCanAllowed(5,enable,5));
  CK(!VehicleArm_SourceParallelCanAllowed(6,enable,5));
  vsp_axis[1].phase=2;CK(!VehicleArm_SourceParallelCanAllowed(5,enable,5));
  memset(vsp_axis[1].move,0x55,15);vsp_group.started=SPF_LIFT;
  CK(VehicleArm_SourceParallelCanAllowed(5,vsp_axis[1].move,15));
  CK(!VehicleArm_SourceParallelCanAllowed(5,wrong,15));
  vsp_owned=0;CK(!VehicleArm_SourceParallelCanAllowed(5,vsp_axis[1].move,15));}
 printf("SOURCE_PARALLEL_REAL_PORT checks%u failures%u\n",checks,failures);return failures?1:0;
}
