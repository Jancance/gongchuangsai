"""Execute the actual DM motion driver with fake CAN hardware and identity reads."""
from pathlib import Path
from vehicle_check_paths import firmware_root
import json,re,subprocess
R=Path(__file__).resolve().parents[1];S=firmware_root()
O=R/'evidence/integration/2026-11-03/early-scan-startup';D=O/'dm-stubs';D.mkdir(exist_ok=True)
(D/'FreeRTOS.h').write_text('#include <stdint.h>\ntypedef uint32_t TickType_t;\n#define pdMS_TO_TICKS(x) (x)\n#define portTICK_PERIOD_MS 1U\n',encoding='utf-8')
(D/'task.h').write_text('uint32_t xTaskGetTickCount(void);\n',encoding='utf-8')
motion=(S/'HARDWARE/DM_CAN2/dm_can2_motion.c').read_text(encoding='utf-8')
float_names={'pmax','vmax','tmax','position_rad','position_deg','velocity_rad_s','torque_nm','logical_position_deg'}
declarations=[]
for name in sorted(set(re.findall(r'\bg_dm_can2_\w+',motion)) | {'g_dm_can2_read_kind'}):
    suffix=name.removeprefix('g_dm_can2_')
    if suffix=='last_rx':declarations.append('CanRxMsg '+name+';')
    else:declarations.append('volatile '+('float ' if suffix in float_names else 'uint32_t ')+name+';')
unit=r'''
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
__DECLARATIONS__
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
 memset(f,0,sizeof(*f));f->motor_state=motor;f->position_rad=actual;return 1;
}
void DM_CAN2_Service(uint32_t ready,uint32_t busy){
 if(!g_dm_can2_read_request)return;g_dm_can2_read_request=0;g_dm_can2_read_cycle++;
 g_dm_can2_state=4;g_dm_can2_result=1;g_dm_can2_read_kind=2;g_dm_can2_valid_mask=g_dm_can2_range_mask=7;
 g_dm_can2_feedback_valid=1;g_dm_can2_feedback_tick=tick;g_dm_can2_motor_state=motor;
 g_dm_can2_esc_id=1;g_dm_can2_master_id=0;g_dm_can2_ctrl_mode=2;
 g_dm_can2_pmax=12.5f;g_dm_can2_vmax=30;g_dm_can2_tmax=10;
 g_dm_can2_position_rad=actual;g_dm_can2_position_deg=actual*57.2957795f;g_dm_can2_velocity_rad_s=0;
}
#include "dm_can2_motion.c"
static uint32_t checks,failures;
#define C(x) do{checks++;if(!(x)){printf("fail line %u check %u\n",__LINE__,checks);failures++;}}while(0)
static void reset(float work){
 tick=100;pending=motor=tx_count=enables=disables=0;actual=(DM_CAN2_LEFT_ZERO_DEG+work)*0.01745329252f;
 g_dm_jog_state=g_dm_jog_result=g_dm_posvel_holding=g_dm_jog_request=g_dm_startup_go=0;
 s_enable_possible=s_pending=s_tx_done=s_queued_request=s_mode_pending=0;s_point_mode=0;
 g_dm_can2_read_cycle=g_dm_can2_read_request=0;g_dm_point_enable=1;g_dm_posvel_speed_deg_s=150;
 g_dm_jog_first_fault=g_dm_integrated_owner=0;
}
static void steps(uint32_t n){while(n--){tick+=5;DM_CAN2_ModuleService(1,0);}}
int main(void){float initial,goal;
 reset(180);initial=actual;g_dm_jog_request=6;steps(30);
 C(g_dm_jog_state==3 && enables==1 && !g_dm_posvel_holding);
 C(fabsf(actual-initial)<0.0001f && fabsf(last_target-initial)<0.0001f);
 C(fabsf(g_dm_posvel_latched_speed_deg_s-150)<0.001f);
 C(fabsf(g_dm_jog_target_deg-g_dm_jog_start_deg-180)<0.05f);
 g_dm_startup_go=1;steps(30);goal=initial+3.14159265359f;
 C(g_dm_jog_state==7 && g_dm_jog_result==1 && g_dm_posvel_holding);
 C(fabsf(actual-goal)<0.001f && fabsf(last_speed-2.617993878f)<0.001f);
 C(fabsf(g_dm_can2_logical_position_deg)<0.05f);
 steps(10);C(g_dm_posvel_holding && enables==1);
 g_dm_jog_request=3;steps(3);C(!g_dm_posvel_holding && g_dm_jog_stop_result==1 && disables==1);
 for(uint32_t i=0;i<3;i++){
  const float starts[3]={270.0f,315.0f,350.0f};
  reset(starts[i]);initial=actual;g_dm_jog_request=6;steps(10);
  C(g_dm_jog_state==3 && !g_dm_startup_go && fabsf(actual-initial)<0.001f);
  g_dm_startup_go=1;steps(150);
  C(g_dm_jog_state==7 && g_dm_posvel_holding && fabsf(actual-(DM_CAN2_LEFT_ZERO_DEG+360.0f)*0.01745329252f)<0.02f);
 }
 reset(90);g_dm_jog_request=6;steps(4);C(g_dm_jog_state==8 && g_dm_jog_result==14 && !enables);
 reset(0);initial=actual;g_dm_startup_go=1;g_dm_jog_request=6;steps(30);
 C(g_dm_jog_state==7 && fabsf(actual-initial)<0.001f); /* no additional full turn */
 reset(180);g_dm_jog_request=6;steps(10);tick+=101;DM_CAN2_ModuleService(1,0);
 C(g_dm_jog_first_fault==12 && !g_dm_posvel_holding);
 printf("actual DM driver checks=%u failures=%u\n",checks,failures);return failures?1:0;}
'''.replace('__DECLARATIONS__','\n'.join(declarations))
(O/'dm-driver-unit.c').write_text(unit,encoding='utf-8');exe=O/'dm-driver-unit.exe'
p=subprocess.run(['C:/mingw/mingw64/bin/gcc.exe','-DVEHICLE_STARTUP_GROUP_BEFORE_ROUTE=0','-DVEHICLE_STARTUP_GROUP_ONLY=0','-DVEHICLE_EXTENSION_HOME_ONLY=0','-DVEHICLE_DM_STARTUP_ONLY=0','-std=c99','-I'+str(D),'-I'+str(S/'HARDWARE/DM_CAN2'),str(O/'dm-driver-unit.c'),'-lm','-o',str(exe)],capture_output=True,text=True,errors='replace')
assert p.returncode==0,p.stderr[-6000:]
p=subprocess.run([str(exe)],capture_output=True,text=True);print(p.stdout);assert p.returncode==0,p.stdout
(O/'dm-driver-check.json').write_text(json.dumps(dict(passed=True,output=p.stdout,actual_driver=True,
    mocked_can=True,hardware_accessed=False),indent=2),encoding='utf-8')
