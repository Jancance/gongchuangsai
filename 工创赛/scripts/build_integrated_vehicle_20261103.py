"""Build the source-configured eight-stage field-tuning entry in a staged project."""
from pathlib import Path
import hashlib,json,xml.etree.ElementTree as E

R=Path(__file__).resolve().parents[1];S=R/'tmp/stage1_early_scan_startup_20261103'
EV=R/'evidence/integration/2026-11-03/early-scan-startup'

PARAMS=r'''#ifndef VEHICLE_PARAMETERS_H
#define VEHICLE_PARAMETERS_H
#include <stdint.h>
/* ================= 现场只改这一个文件，保存→Rebuild→Download =================
 * 调参单位：底盘距离cm、平移cm/s；机械臂Z为旧3200脉冲/圈的虚拟脉冲，
 * 伸缩mm仅沿用32脉冲/mm粗标；DM/舵机为角度deg、DM速度deg/s。
 * 本文件是工程E调试输入，坐标、路径、夹持和负载须现场逐段验证。
 */
#define VEHICLE_INTEGRATED_ENABLE 1U
#define VEHICLE_TEST_STAGE 1U       /* 1~8：本次只调哪一大步骤。 */
#define VEHICLE_RUN_ALL 0U          /* 0单段；1从第1段连续执行八段。 */
#define VEHICLE_REHEARSAL 0U        /* 八段现场分别通过后才设1，整场180秒。 */
#define VEHICLE_SCAN_BYPASS 1U      /* 当前按用户要求跳过扫码；不伪造任务码/计数。 */
#define VEHICLE_STAGE_TIMEOUT_MS 120000U
#define VEHICLE_REHEARSAL_MS 180000U

/* ---------- 上电停放：按键前DM/伸缩不自动运动 ---------- */
#ifndef STARTUP_GRIPPER_ANGLE_DEG
#define STARTUP_GRIPPER_ANGLE_DEG 140.0f
#endif
#ifndef STARTUP_CARRIER_ANGLE_DEG
#define STARTUP_CARRIER_ANGLE_DEG (-1.0f) /* 未定=-1不输出；填0~270后生效。 */
#endif
#define STARTUP_DM_SPEED_SCALE 0.5f /* 300×0.5=150deg/s，逆时针约180→360(显示0)。 */
/* 伸缩：先在本次有效机械缩回参考处读取驱动角，填ZERO及VALID。
 * VALID=0时按键只读位置并退出，不发DM/伸缩运动；不自动设置电机零点。
 * 手推、失电或位置故障后，必须重新确认参考，不承诺掉电机械绝对位置。
 */
#define STARTUP_EXTENSION_REFERENCE_VALID 0U
#define STARTUP_EXTENSION_ZERO_DEG 0.0f
#define STARTUP_EXTENSION_EXTEND_SIGN (-1)
#define STARTUP_EXTENSION_TARGET_MM 0.0f  /* 这里改按键后的固定伸出量，当前0~50粗标。 */
#define VEHICLE_EXTENSION_RPM 106.0f
#define VEHICLE_X_ACCEL_RPM_S 45055U
/* 升降机械顶部对应驱动角；未确认时禁止Z动作，不影响只调底盘路线。 */
#define VEHICLE_LIFT_REFERENCE_VALID 0U
#define VEHICLE_LIFT_TOP_DEG 0.0f
#define VEHICLE_LIFT_DOWN_SIGN 1
#define VEHICLE_LIFT_MAX_TARGET_PULSES 105000.0f
#define VEHICLE_LIFT_RPM 1846.4f
#define VEHICLE_DM_SPEED_DEG_S 300.0f
#define VEHICLE_GRIP_OPEN_DEG 180.0f
#define VEHICLE_GRIP_CLOSE_DEG 135.0f /* 上电140与动作闭合135分开。 */
#define VEHICLE_CARRIER_SLOT1_DEG 0.0f
#define VEHICLE_CARRIER_SLOT2_DEG 120.0f
#define VEHICLE_CARRIER_SLOT3_DEG 240.0f
#define VEHICLE_CARRIER_ARM_DEG 35.1079407f
#define VEHICLE_SOURCE_ARM_DEG 180.0f
#define VEHICLE_SOURCE_Z_PULSES 105000.0f
#define VEHICLE_CARRIER_Z_PULSES 28000.0f
#define VEHICLE_SAFE_Z_PULSES 0.0f
/* 后续工位实测后在这里填；-1拒绝执行，不能套用原料区高度。 */
#define VEHICLE_COARSE_Z_PULSES (-1.0f)
#define VEHICLE_STORAGE_Z_PULSES (-1.0f)
#define VEHICLE_STACK_Z_PULSES (-1.0f)
/* 工程运行范围与故障限值保留，普通调参不能用删除判断来绕过。 */
#define VEHICLE_MAX_TRANSLATION_CM_S 20.0f
#define VEHICLE_AXIS_TIMEOUT_MS 15000U
#define VEHICLE_MATERIAL_OBSERVATION_REQUIRED 1U
/* 真实成功观察接口尚待视觉/执行反馈接入；0仅用于无物料路线调试，
 * 不能把0写成已完成夹持/载台库存/正确计数。正式自主不允许旁路。 */

typedef struct {uint32_t direction;float distance_cm,speed_cm_s;} VehicleDriveRow;
/* 方向：0前进、1左移、2右移、3后退；相对本段起始车头，当前不转底盘朝向。 */
#define VF(cm,speed) {0U,cm,speed}
#define VL(cm,speed) {1U,cm,speed}
#define VR(cm,speed) {2U,cm,speed}
#define VB(cm,speed) {3U,cm,speed}
#define VSTOP {0U,0.0f,0.0f}
static const VehicleDriveRow vehicle_drive[8][8]={
 /* 1 启停区1→扫码区：已经观察方向正确，距离/停车仍要量。 */
 {VF(15.0f,5.0f),VL(105.0f,10.0f),VSTOP},
 /* 2 扫码区→原料区：以下2~8是场地图推导候选，未实车验收。 */
 {VF(90.0f,10.0f),VR(96.0f,10.0f),VSTOP},
 /* 3 原料区→粗加工区 */
 {VL(192.0f,10.0f),VSTOP},
 /* 4 粗加工区→暂存区 */
 {VR(96.0f,10.0f),VF(96.0f,10.0f),VSTOP},
 /* 5 暂存区→原料区 */
 {VB(96.0f,10.0f),VR(96.0f,10.0f),VSTOP},
 /* 6 原料区→粗加工区 */
 {VL(192.0f,10.0f),VSTOP},
 /* 7 粗加工区→暂存区 */
 {VR(96.0f,10.0f),VF(96.0f,10.0f),VSTOP},
 /* 8 暂存区→本轮启停区（用户已确认）。区2仅镜像1/8段的最后接入横移。 */
 {VB(96.0f,10.0f),VB(90.0f,10.0f),VR(105.0f,10.0f),VB(15.0f,5.0f),VSTOP}
};

/* ---------- 各段机械臂动作：每一行都可改目标、速度和停留 ----------
 * opcode：0结束，1升降目标(虚拟脉冲)，2DM角度，3伸缩目标mm，
 * 4夹爪角度，5载盘角度，6停留ms，7原料→载台实物观察，
 * 8载台→工位放稳观察，9工位→载台观察。观察缺失时不记成功、不运输。
 * 舵机无独立速度命令，speed列填0；settle_ms只表示等待，不冒充反馈。
 * 下面使用三件循环：item=0/1/2自动选对应槽位；同批实例仍需任务反馈。
 */
typedef struct {uint32_t op;float target,speed;uint32_t settle_ms;} VehicleArmRow;
#define VA_END {0U,0.0f,0.0f,0U}
#define VA_Z(p) {1U,p,VEHICLE_LIFT_RPM,0U}
#define VA_DM(d) {2U,d,VEHICLE_DM_SPEED_DEG_S,0U}
#define VA_EXT(mm) {3U,mm,VEHICLE_EXTENSION_RPM,0U}
#define VA_GRIP(d) {4U,d,0.0f,500U}
#define VA_SLOT {5U,-2.0f,0.0f,1500U} /* -2=本件槽位，其他数值为直接角。 */
#define VA_WAIT(ms) {6U,0.0f,0.0f,ms}
#define VA_OBSERVE(kind) {kind,0.0f,0.0f,0U}
static const uint32_t vehicle_arm_enable[8]={0U,1U,1U,1U,1U,1U,1U,0U};
static const uint32_t vehicle_arm_repeat[8]={1U,3U,3U,3U,3U,3U,3U,1U};
static const VehicleArmRow vehicle_arm_rows[8][32]={
 /* 第1段只调底盘；4310/伸缩并行准备由公共启动流程完成。 */
 {VA_END},
 /* 第2段：原料取一件→放对应车载槽→空爪安全高度→回运输角0。 */
 {VA_SLOT,VA_GRIP(VEHICLE_GRIP_OPEN_DEG),VA_DM(VEHICLE_SOURCE_ARM_DEG),
  VA_EXT(STARTUP_EXTENSION_TARGET_MM),VA_Z(VEHICLE_SOURCE_Z_PULSES),VA_GRIP(VEHICLE_GRIP_CLOSE_DEG),
  VA_Z(VEHICLE_SAFE_Z_PULSES),VA_DM(VEHICLE_CARRIER_ARM_DEG),VA_Z(VEHICLE_CARRIER_Z_PULSES),
  VA_GRIP(VEHICLE_GRIP_OPEN_DEG),VA_Z(VEHICLE_SAFE_Z_PULSES),VA_DM(0.0f),VA_OBSERVE(7U),VA_END},
 /* 第3段：载台→粗加工放稳→再取回载台；不接触地面后推移。 */
 {VA_SLOT,VA_DM(VEHICLE_CARRIER_ARM_DEG),VA_Z(VEHICLE_CARRIER_Z_PULSES),VA_GRIP(VEHICLE_GRIP_CLOSE_DEG),
  VA_Z(0.0f),VA_DM(180.0f),VA_Z(VEHICLE_COARSE_Z_PULSES),VA_GRIP(VEHICLE_GRIP_OPEN_DEG),VA_Z(0.0f),
  VA_OBSERVE(8U),VA_WAIT(500U),VA_Z(VEHICLE_COARSE_Z_PULSES),VA_GRIP(VEHICLE_GRIP_CLOSE_DEG),VA_Z(0.0f),
  VA_DM(VEHICLE_CARRIER_ARM_DEG),VA_Z(VEHICLE_CARRIER_Z_PULSES),VA_GRIP(VEHICLE_GRIP_OPEN_DEG),
  VA_Z(0.0f),VA_DM(0.0f),VA_OBSERVE(9U),VA_END},
 /* 第4段：第一批暂存，不把第四段粗加工位置当同色码垛位置。 */
 {VA_SLOT,VA_DM(VEHICLE_CARRIER_ARM_DEG),VA_Z(VEHICLE_CARRIER_Z_PULSES),VA_GRIP(VEHICLE_GRIP_CLOSE_DEG),
  VA_Z(0.0f),VA_DM(180.0f),VA_Z(VEHICLE_STORAGE_Z_PULSES),VA_GRIP(VEHICLE_GRIP_OPEN_DEG),VA_Z(0.0f),
  VA_DM(0.0f),VA_OBSERVE(8U),VA_END},
 /* 第5段：第二批取料。可单独改本段每个动作，不影响第2段。 */
 {VA_SLOT,VA_GRIP(180.0f),VA_DM(180.0f),VA_EXT(STARTUP_EXTENSION_TARGET_MM),
  VA_Z(VEHICLE_SOURCE_Z_PULSES),VA_GRIP(135.0f),VA_Z(0.0f),VA_DM(VEHICLE_CARRIER_ARM_DEG),
  VA_Z(VEHICLE_CARRIER_Z_PULSES),VA_GRIP(180.0f),VA_Z(0.0f),VA_DM(0.0f),VA_OBSERVE(7U),VA_END},
 /* 第6段：第二批粗加工，可独立改高度和停留。 */
 {VA_SLOT,VA_DM(VEHICLE_CARRIER_ARM_DEG),VA_Z(VEHICLE_CARRIER_Z_PULSES),VA_GRIP(135.0f),VA_Z(0.0f),
  VA_DM(180.0f),VA_Z(VEHICLE_COARSE_Z_PULSES),VA_GRIP(180.0f),VA_Z(0.0f),VA_OBSERVE(8U),
  VA_WAIT(500U),VA_Z(VEHICLE_COARSE_Z_PULSES),VA_GRIP(135.0f),VA_Z(0.0f),VA_DM(VEHICLE_CARRIER_ARM_DEG),
  VA_Z(VEHICLE_CARRIER_Z_PULSES),VA_GRIP(180.0f),VA_Z(0.0f),VA_DM(0.0f),VA_OBSERVE(9U),VA_END},
 /* 第7段：码垛高度独立；真实同色底层和落稳确认接入后才可全自动。 */
 {VA_SLOT,VA_DM(VEHICLE_CARRIER_ARM_DEG),VA_Z(VEHICLE_CARRIER_Z_PULSES),VA_GRIP(135.0f),VA_Z(0.0f),
  VA_DM(180.0f),VA_Z(VEHICLE_STACK_Z_PULSES),VA_GRIP(180.0f),VA_Z(0.0f),VA_DM(0.0f),VA_OBSERVE(8U),VA_END},
 {VA_END}
};
/* 对应每个工位/每件的伸缩目标；未示教=-1，先填实测再做带物料动作。
 * 原料区目前沿用启动固定伸出量；粗加工/暂存/码垛可分别配置。 */
static const float vehicle_station_extension_mm[8][3]={
 {0,0,0},{STARTUP_EXTENSION_TARGET_MM,STARTUP_EXTENSION_TARGET_MM,STARTUP_EXTENSION_TARGET_MM},
 {-1,-1,-1},{-1,-1,-1},{STARTUP_EXTENSION_TARGET_MM,STARTUP_EXTENSION_TARGET_MM,STARTUP_EXTENSION_TARGET_MM},
 {-1,-1,-1},{-1,-1,-1},{0,0,0}};
#endif
'''

ARM_H=r'''#ifndef VEHICLE_ARM_H
#define VEHICLE_ARM_H
#include "../../USER/vehicle_parameters.h"
extern volatile uint32_t g_vehicle_arm_state,g_vehicle_arm_cause,g_vehicle_arm_dm_owned,g_vehicle_arm_axis_owned;
extern volatile float g_vehicle_lift_current_deg;
uint32_t VehicleArm_Begin(const VehicleArmRow *row,uint32_t item);
void VehicleArm_Service(void);
void VehicleArm_Cancel(void);
uint32_t VehicleArm_Busy(void);
uint32_t VehicleArm_Exchange(uint8_t address,const uint8_t *q,uint8_t n,uint8_t reply_n,void *out);
#endif
'''

ARM_C=r'''#include "vehicle_arm.h"
#include "app_config.h"
#include "startup_axes.h"
#include "dm_can2.h"
#include "gripper.h"
#include "carrier_pwm.h"
#include "carrier_config.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>
volatile uint32_t g_vehicle_arm_state=0U,g_vehicle_arm_cause=0U;
volatile uint32_t g_vehicle_arm_dm_owned=0U,g_vehicle_arm_axis_owned=0U;
volatile float g_vehicle_lift_current_deg=0.0f;
static VehicleArmRow s_row;
static uint32_t s_begin,s_phase,s_item,s_samples,s_wait,s_cycle;
static float s_goal,s_before,s_current;
static uint8_t s_address,s_options,s_move[15];
static CanRxMsg s_rx;
extern volatile uint32_t g_dm_integrated_owner;
extern void StartupAxes_SetExtensionHoldTarget(float target);
uint32_t VehicleArm_Busy(void){return g_vehicle_arm_state==1U;}
static void arm_fail(uint32_t why)
{g_vehicle_arm_state=3U;g_vehicle_arm_cause=why;g_dm_jog_request=3U;StartupAxes_Cancel();}
void VehicleArm_Cancel(void)
{arm_fail(2U);}
static uint32_t query(uint8_t command,uint8_t n)
{uint8_t q[2]={command,0x6b};return VehicleArm_Exchange(s_address,q,2U,n,&s_rx)==1U;}
static uint32_t position(void)
{
    uint32_t raw;
    if(!query(0x36,7U) || s_rx.Data[1]>1U)return 0U;
    raw=((uint32_t)s_rx.Data[2]<<24)|((uint32_t)s_rx.Data[3]<<16)|((uint32_t)s_rx.Data[4]<<8)|s_rx.Data[5];
    s_current=(s_rx.Data[1]?-1.0f:1.0f)*(float)raw*0.1f;
    if(s_address==5U)g_vehicle_lift_current_deg=s_current;
    return 1U;
}
uint32_t VehicleArm_Begin(const VehicleArmRow *row,uint32_t item)
{
#if APP_MOTOR_OFFLINE_ONLY
    (void)row;(void)item;return 0U;
#else
    if(!row || VehicleArm_Busy() || !StartupAxes_Holding())return 0U;
    s_row=*row;s_item=item;s_phase=s_samples=s_wait=0U;s_cycle=0U;
    s_begin=(uint32_t)xTaskGetTickCount();g_vehicle_arm_cause=0U;
    g_vehicle_arm_state=1U;return 1U;
#endif
}
void VehicleArm_Service(void)
{
#if !APP_MOTOR_OFFLINE_ONLY
    uint32_t now=(uint32_t)xTaskGetTickCount(),pulse,units,speed;
    float target,delta;uint8_t q[5]={0xf3,0xab,1,0,0x6b};
    if(!VehicleArm_Busy())return;
    if(!StartupAxes_Holding() || now-s_begin>=VEHICLE_AXIS_TIMEOUT_MS){arm_fail(3U);return;}
    if(s_row.op==2U) {
        if(!s_phase) {
            if(!(s_row.target>=0 && s_row.target<=180 && s_row.speed>0 && s_row.speed<=300)){arm_fail(4U);return;}
            g_vehicle_arm_dm_owned=g_dm_integrated_owner=1U;
            g_dm_point_enable=1U;g_dm_posvel_speed_deg_s=s_row.speed;
            g_dm_pick_target_deg=g_dm_place_target_deg=s_row.target;
            g_dm_jog_request=s_row.target>=g_dm_can2_logical_position_deg?4U:5U;
            s_cycle=g_dm_can2_read_cycle;s_phase=1U;return;
        }
        if(g_dm_jog_state==8U && g_dm_jog_result!=2U){arm_fail(5U);return;}
        if(g_dm_can2_read_cycle!=s_cycle && g_dm_jog_state==7U && g_dm_jog_result==1U &&
           g_dm_posvel_holding && now-g_dm_can2_feedback_tick<100U) {
            g_vehicle_arm_dm_owned=0U;g_vehicle_arm_state=2U;
        }return;
    }
    if(s_row.op==4U || s_row.op==5U || s_row.op==6U) {
        if(!s_phase) {
            if(s_row.op==4U && !jiazhua(s_row.target)){arm_fail(4U);return;}
            if(s_row.op==5U) {
                target=s_row.target;
                if(target==-2.0f)target=s_item==0?VEHICLE_CARRIER_SLOT1_DEG:(s_item==1?VEHICLE_CARRIER_SLOT2_DEG:VEHICLE_CARRIER_SLOT3_DEG);
                if(!(target>=0 && target<=270)){arm_fail(4U);return;}
                pulse=CARRIER_PULSE_MIN_US+(uint32_t)(target*(CARRIER_PULSE_MAX_US-CARRIER_PULSE_MIN_US)/270.0f+0.5f);
                if(!CarrierPwm_Write(pulse)){arm_fail(4U);return;}
            }
            s_wait=now;s_phase=1U;
        }
        if(now-s_wait>=s_row.settle_ms)g_vehicle_arm_state=2U;
        return;
    }
    if(s_row.op!=1U && s_row.op!=3U){arm_fail(4U);return;}
    if(s_phase==0U) {
        if(s_row.op==1U) {
            s_address=5U;
            if(VEHICLE_LIFT_REFERENCE_VALID!=1U || !(s_row.target>=0 && s_row.target<=VEHICLE_LIFT_MAX_TARGET_PULSES))
            {g_vehicle_arm_axis_owned=5U;(void)position();arm_fail(6U);return;}
            s_goal=VEHICLE_LIFT_TOP_DEG+VEHICLE_LIFT_DOWN_SIGN*s_row.target*360.0f/3200.0f;
        } else {
            s_address=6U;
            if(!(s_row.target>=0 && s_row.target<=50)){arm_fail(4U);return;}
            s_goal=STARTUP_EXTENSION_ZERO_DEG+STARTUP_EXTENSION_EXTEND_SIGN*s_row.target*32.0f*360.0f/3200.0f;
        }
        if(!(s_row.speed>0 && s_row.speed<=(s_address==5U?VEHICLE_LIFT_RPM:VEHICLE_EXTENSION_RPM))){arm_fail(4U);return;}
        g_vehicle_arm_axis_owned=s_address;
        if(!query(0x1a,4U) || (s_rx.Data[2]&3U)){arm_fail(7U);return;}
        s_options=s_rx.Data[2];s_phase=1U;return;
    }
    if(s_phase==1U) {
        if(!position()){arm_fail(7U);return;}s_before=s_current;
        delta=s_goal-s_current;
        if(!(fabsf(delta)<=(s_address==5U?VEHICLE_LIFT_MAX_TARGET_PULSES*360.0f/3200.0f:180.0f))){arm_fail(8U);return;}
        units=(uint32_t)(fabsf(delta)*((s_options&0x80U)?100.0f:10.0f)+0.5f);
        speed=(uint32_t)(s_row.speed*10.0f+0.5f);
        memset(s_move,0,15);s_move[0]=0xfd;s_move[1]=delta<0;
        s_move[2]=s_move[4]=(uint8_t)(VEHICLE_X_ACCEL_RPM_S>>8);s_move[3]=s_move[5]=(uint8_t)VEHICLE_X_ACCEL_RPM_S;
        s_move[6]=(uint8_t)(speed>>8);s_move[7]=(uint8_t)speed;
        s_move[8]=units>>24;s_move[9]=units>>16;s_move[10]=units>>8;s_move[11]=units;
        s_move[12]=2;s_move[14]=0x6b;s_phase=2U;return;
    }
    if(s_phase==2U) {
        if(!query(0x35,5U) || s_rx.Data[2] || s_rx.Data[3]){arm_fail(7U);return;}
        s_phase=3U;return;
    }
    if(s_phase==3U) {
        if(VehicleArm_Exchange(s_address,q,5U,3U,&s_rx)!=1U || s_rx.Data[1]!=2U){arm_fail(7U);return;}
        s_phase=4U;return;
    }
    if(s_phase==4U) {
        if(fabsf(s_goal-s_before)>0.2f &&
           (VehicleArm_Exchange(s_address,s_move,15U,3U,&s_rx)!=1U || (s_rx.Data[1]!=2U && s_rx.Data[1]!=0x9fU)))
        {arm_fail(7U);return;}s_phase=5U;return;
    }
    if(s_phase==5U) {
        if(!query(0x3a,3U) || !(s_rx.Data[1]&1U) || (s_rx.Data[1]&0x0cU)){arm_fail(9U);return;}
        s_phase=6U;return;
    }
    if(s_phase==6U) {
        if(!position()){arm_fail(7U);return;}
        if(s_current<fminf(s_before,s_goal)-2.0f || s_current>fmaxf(s_before,s_goal)+2.0f){arm_fail(8U);return;}
        s_phase=7U;return;
    }
    if(!query(0x35,5U)){arm_fail(7U);return;}
    if(!s_rx.Data[2] && !s_rx.Data[3] && fabsf(s_current-s_goal)<=0.3f)s_samples++;else s_samples=0;
    if(s_samples>=3U) {
        if(s_address==6U)StartupAxes_SetExtensionHoldTarget(s_goal);
        g_vehicle_arm_axis_owned=0U;g_vehicle_arm_state=2U;
    } else s_phase=5U;
#endif
}
'''

MISSION=r'''/* 整车八阶段源码调参入口；按键只提交启动/取消，禁止Watch/无线修改业务参数。 */
volatile uint32_t g_vehicle_stage=VEHICLE_TEST_STAGE,g_vehicle_state=0U,g_vehicle_cause=0U;
volatile uint32_t g_vehicle_key=0U,g_vehicle_arm_row=0U,g_vehicle_item=0U,g_vehicle_stage_mask=0U;
volatile uint32_t g_vehicle_run_id=0U,g_vehicle_scan_bypassed=0U;
volatile uint32_t g_vehicle_correct_pick_count=0U,g_vehicle_correct_place_count=0U;
static uint32_t s_vehicle_start,s_vehicle_stage_start,s_vehicle_zone,s_vehicle_observation;
static uint32_t s_vehicle_scan_baseline,s_vehicle_code_valid,s_vehicle_display_done;
static char s_vehicle_task_code[16];
/* 真实观测端调用，必须绑定当前run/stage/item/row；不提供Watch成功开关。 */
uint32_t VehicleMission_Observe(uint32_t run,uint32_t stage,uint32_t item,uint32_t row,uint32_t kind)
{
    if(run!=g_vehicle_run_id || stage!=g_vehicle_stage || item!=g_vehicle_item ||
       row!=g_vehicle_arm_row || g_vehicle_state!=3U || kind<7U || kind>9U)return 0U;
    s_vehicle_observation=kind;return 1U;
}
static void VehicleMission_Fail(uint32_t cause)
{
    g_vehicle_cause=cause;g_vehicle_state=6U;g_stage2_ready=0U;
    MotionStop();VehicleArm_Cancel();StartupAxes_Cancel();
}
static uint32_t VehicleMission_Stopped(void)
{
    ChassisAutoStatus s;PowerCommandStatus p;
    (void)ChassisAuto_ReadStatus(&s);Power_ReadCommandStatus(&p);
    return g_combined_test.state==3U && !g_combined_test.cause && !s_motion_session_open &&
        !s.active && s.result==1U && !s.cause && s.rotation_stop_mask==15U &&
        !g_forward_hold.trip && !g_heading_exec_trip && !g_vm_test_request &&
        (!p.active || StartupAxes_Holding());
}
static uint32_t VehicleMission_Load(uint32_t stage)
{
    RouteRow rows[8];RouteCheck check;ChassisAutoStatus backend;
    ForwardHoldDebug forward={0};CombinedTestDebug combined={0};uint32_t n,i,direction;
    if(stage<1U || stage>8U)return 0U;
    for(n=0;n<8U && vehicle_drive[stage-1U][n].distance_cm>0;n++) {
        direction=vehicle_drive[stage-1U][n].direction;
        if(direction>3)return 0U;
        if(s_vehicle_zone==2U && ((stage==1U && n==1U) || (stage==8U && n==2U)))
            direction=direction==1U?2U:1U;
        rows[n].kind=MOTION_FORWARD_DISTANCE+direction;
        rows[n].amount=vehicle_drive[stage-1U][n].distance_cm;
        rows[n].speed=vehicle_drive[stage-1U][n].speed_cm_s;
    }
    if(!n || n>8U || !RoutePlan_Convert(rows,n,&g_chassis_geometry,s_motion_actions,&check))return 0U;
    if(g_combined_test.state==3U) {
        if(!VehicleMission_Stopped())return 0U;
        (void)ChassisAuto_ReadStatus(&backend);
        if(!ChassisAuto_RearmCompleted(backend.session))return 0U;
        g_combined_test=combined;g_forward_hold=forward;
        g_heading_exec_phase=HR_IDLE;g_heading_exec_state=HS_IDLE;g_heading_exec_request=0U;
        HeadingSession_Init(&s_heading_exec);
        s_motion_has_action=s_combined_turn=s_combined_powered_segment=s_combined_keep_vm=0U;
    }
    s_motion_action_count=n;g_debug_plan_id=DEBUG_PLAN_DEFAULT;g_route_check=check;
    g_motion_plan_valid=App_MotionPlanValid();if(!g_motion_plan_valid)return 0U;
    if(!App_SerialIdleReady((uint32_t)xTaskGetTickCount()))return 0U;
    for(i=0;i<9;i++){s_perimeter_targets[i][0]=s_perimeter_targets[i][1]=s_perimeter_targets[i][2]=0;}
    /* 各段里程仅作本段观测，不伪造全场真实X/Y或名义终点已到。 */
    g_stage1_stop_requested=g_stage1_motion_skipped=0U;
    s_serial_start_permit=1U;g_combined_test.request=1U;g_serial_motion_state=2U;g_serial_motion_result=2U;
    g_vehicle_stage=stage;g_vehicle_state=2U;s_vehicle_stage_start=(uint32_t)xTaskGetTickCount();
    return 1U;
}
static void VehicleMission_Advance(void)
{
    g_vehicle_stage_mask|=1UL<<(g_vehicle_stage-1U);
    if(!VEHICLE_RUN_ALL){g_vehicle_state=4U;return;}
    if(g_vehicle_stage==8U){g_vehicle_state=5U;return;}
    if(!VehicleMission_Load(g_vehicle_stage+1U))VehicleMission_Fail(7U);
}
static void VehicleMission_Service(void)
{
    uint32_t now=(uint32_t)xTaskGetTickCount(),key;const VehicleArmRow *row;
    taskENTER_CRITICAL();key=g_vehicle_key;g_vehicle_key=0U;taskEXIT_CRITICAL();
    if(key && g_vehicle_state!=0U && g_vehicle_state!=4U && g_vehicle_state!=5U) {
        VehicleMission_Fail(2U);g_vehicle_state=7U;return;
    }
    if(key && g_vehicle_state==0U) {
        if(VEHICLE_TEST_STAGE<1 || VEHICLE_TEST_STAGE>8 || !g_radar_zone_ready ||
           !App_SerialIdleReady(now)){g_vehicle_cause=1U;return;}
        s_vehicle_zone=RadarZoneSwitch_Lock();if(s_vehicle_zone<1 || s_vehicle_zone>2)return;
        g_vehicle_run_id++;if(!g_vehicle_run_id)g_vehicle_run_id=1U;
        s_vehicle_start=s_vehicle_stage_start=now;g_vehicle_stage_mask=0U;
        g_vehicle_scan_bypassed=VEHICLE_SCAN_BYPASS;s_vehicle_code_valid=s_vehicle_display_done=0U;
        s_vehicle_scan_baseline=g_saoma_accept_count;memset(s_vehicle_task_code,0,16);
        if(!StartupAxes_Begin()){VehicleMission_Fail(3U);return;}g_vehicle_state=1U;
    }
    if(g_vehicle_state>=1U && g_vehicle_state<=3U) {
        if(g_rtos_fault || g_safety_fault || g_can_ctrl_fault_latched || g_startup_axes_state>=5U)
        {VehicleMission_Fail(4U);return;}
        if(now-s_vehicle_stage_start>=VEHICLE_STAGE_TIMEOUT_MS ||
           (VEHICLE_REHEARSAL && now-s_vehicle_start>=VEHICLE_REHEARSAL_MS))
        {VehicleMission_Fail(5U);return;}
    }
    if(g_vehicle_state==1U && StartupAxes_Holding()) {
        if(!VehicleMission_Load(VEHICLE_RUN_ALL?1U:VEHICLE_TEST_STAGE)){VehicleMission_Fail(7U);return;}
    }
    if(g_vehicle_state==2U) {
        if(g_combined_test.state>=4U){VehicleMission_Fail(8U);return;}
        if(g_vehicle_stage==1U && !g_vehicle_scan_bypassed && !s_vehicle_code_valid &&
           g_saoma_accept_count!=s_vehicle_scan_baseline && g_saoma_last_result==1U &&
           (uint32_t)(g_saoma_frame_start_ms-s_vehicle_start)<0x80000000U &&
           (uint32_t)(now-g_saoma_frame_end_ms)<VEHICLE_STAGE1_SCAN_FRESH_MS && FieldMap_TaskCodeValid(SAOMA_Data)) {
            memcpy(s_vehicle_task_code,SAOMA_Data,16);s_vehicle_code_valid=1U;
            g_stage1_stop_requested=1U;
        }
        if(VehicleMission_Stopped()) {
            if(g_vehicle_stage==1U && !g_vehicle_scan_bypassed) {
                if(!s_vehicle_code_valid)return;
                if(!s_vehicle_display_done) {
                    g_scan_display_result=Screen_ShowTaskCode(s_vehicle_task_code);
                    if(g_scan_display_result!=SCREEN_OK){VehicleMission_Fail(9U);return;}
                    s_vehicle_display_done=1U;g_scan_display_count++;
                }
                /* 提前扫码会改变下一段起点；未接真实位姿重规划时禁止沿用名义路线。 */
                if(VEHICLE_RUN_ALL && g_combined_test.completed!=MOTION_ACTION_COUNT)
                {VehicleMission_Fail(10U);return;}
            }
            if(vehicle_arm_enable[g_vehicle_stage-1U]) {
                g_vehicle_arm_row=g_vehicle_item=0U;s_vehicle_observation=0U;g_vehicle_state=3U;
            } else VehicleMission_Advance();
        }
    }
    if(g_vehicle_state==3U) {
        if(g_vehicle_arm_row>=32U || g_vehicle_item>=3U){VehicleMission_Fail(11U);return;}
        row=&vehicle_arm_rows[g_vehicle_stage-1U][g_vehicle_arm_row];
        if(!row->op) {
            if(++g_vehicle_item<vehicle_arm_repeat[g_vehicle_stage-1U])g_vehicle_arm_row=0U;
            else VehicleMission_Advance();return;
        }
        if(row->op>=7U) {
            if(VEHICLE_MATERIAL_OBSERVATION_REQUIRED && s_vehicle_observation!=row->op)return;
            /* 没有真实观测不记正确次数；执行器DONE不能替代持料/落稳。 */
            s_vehicle_observation=0U;g_vehicle_arm_row++;return;
        }
        if(g_vehicle_arm_state==3U){VehicleMission_Fail(12U);return;}
        if(g_vehicle_arm_state==2U){g_vehicle_arm_state=0U;g_vehicle_arm_row++;return;}
        if(!VehicleArm_Busy() && !VehicleArm_Begin(row,g_vehicle_item))VehicleMission_Fail(13U);
    }
}
'''

def edit(rel,fn):
    p=S/rel;b=p.read_bytes()
    try:s=b.decode('utf-8');enc='utf-8'
    except UnicodeDecodeError:s=b.decode('gbk');enc='gbk'
    s=s.replace('\r\n','\n');p.write_bytes(fn(s).replace('\n','\r\n').encode(enc))

if __name__=='__main__':
    for rel,text in [('USER/vehicle_parameters.h',PARAMS),('CHASSIS/USER/vehicle_arm.h',ARM_H),
        ('CHASSIS/USER/vehicle_arm.c',ARM_C),('CHASSIS/USER/vehicle_mission.inc',MISSION)]:
        (S/rel).write_text(text,encoding='utf-8')
    (S/'USER/startup_pose_config.h').write_text('#ifndef STARTUP_POSE_CONFIG_H\n#define STARTUP_POSE_CONFIG_H\n#include "vehicle_parameters.h"\n#endif\n',encoding='utf-8')
    edit('CHASSIS/USER/app_config.h',lambda s:s.replace('#define APP_CONFIG_H',
        '#define APP_CONFIG_H\n#include "../../USER/vehicle_parameters.h"',1).replace(
        '#if !APP_IS_DEBUG || APP_DEBUG_MODE != 3U\n#error Perimeter_candidate_requires_debug_mode3\n#endif\n',''))
    edit('CHASSIS/USER/vehicle_tuning.h',lambda s:s.replace('#define VEHICLE_STAGE1_ENABLE 1U',
        '#define VEHICLE_STAGE1_ENABLE 1U\n#include "../../USER/vehicle_parameters.h"').replace(
        'VEHICLE_STAGE1_ZONE != 1U','(VEHICLE_STAGE1_ZONE < 1U || VEHICLE_STAGE1_ZONE > 2U)'))
    edit('CHASSIS/USER/route_config.h',lambda s:s.replace('#define ROUTE_CONFIG_H',
        '#define ROUTE_CONFIG_H\n#include "../../USER/vehicle_parameters.h"',1).replace(
        '#define APP_ROUTE_FORWARD_MAX_CM_S 20.0f','#define APP_ROUTE_FORWARD_MAX_CM_S VEHICLE_MAX_TRANSLATION_CM_S').replace(
        '#define APP_ROUTE_LEFT_MAX_CM_S    20.0f','#define APP_ROUTE_LEFT_MAX_CM_S VEHICLE_MAX_TRANSLATION_CM_S').replace(
        '#define APP_ROUTE_RIGHT_MAX_CM_S   20.0f','#define APP_ROUTE_RIGHT_MAX_CM_S VEHICLE_MAX_TRANSLATION_CM_S'))
    def main(s):
        s=s.replace('#include "startup_axes.h"','#include "startup_axes.h"\n#include "vehicle_arm.h"',1)
        pos=s.index('/* 任务A：')
        s=s[:pos]+ '#if VEHICLE_INTEGRATED_ENABLE\n#include "vehicle_mission.inc"\n#endif\n'+s[pos:]
        s=s.replace('        if(VEHICLE_STAGE1_ENABLE)\n',
            '        if(VEHICLE_INTEGRATED_ENABLE) VehicleMission_Service();\n        else if(VEHICLE_STAGE1_ENABLE)\n',1)
        s=s.replace('        if (VEHICLE_STAGE1_ENABLE)\n',
            '        if(VEHICLE_INTEGRATED_ENABLE)\n        {if(Key_GetEvent()!=0U){taskENTER_CRITICAL();g_vehicle_key=1U;taskEXIT_CRITICAL();}}\n        else if (VEHICLE_STAGE1_ENABLE)\n',1)
        # Entire deployed field entry uses code parameters; no serial/RAM business commands.
        s=s.replace('        App_SerialMotionSafety();',
            '        if(!VEHICLE_INTEGRATED_ENABLE)App_SerialMotionSafety();')
        # CAN task remains the sole owner of both buses. Refresh CAN2 around bounded CAN1 work.
        s=s.replace('                StartupAxes_Service(g_safety_ready && g_can_test_result==1U,g_can_guard_ready);',
            '                StartupAxes_Service(g_safety_ready && g_can_test_result==1U,g_can_guard_ready);\n'
            '                if(VehicleArm_Busy())VehicleArm_Service();\n'
            '                if(StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);')
        s=s.replace('#include "vehicle_arm.h"','#include "vehicle_arm.h"\n#include "dm_can2.h"',1)
        s=s.replace('                if(g_startup_axes_state!=1U &&',
            '                if(!VehicleArm_Busy() && g_startup_axes_state!=1U &&')
        s=s.replace('                    App_ComboIdlePoll();',
            '                    App_ComboIdlePoll();\n                    if(StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);')
        # Once accepted by serial the old command interpreter is inaccessible in integrated mode.
        marker='static void App_SerialMotionCommand('
        if marker in s:
            start=s.index('{',s.index(marker))+1;s=s[:start]+'\n    if(VEHICLE_INTEGRATED_ENABLE)return;\n'+s[start:]
        return s
    edit('CHASSIS/USER/main.c',main)
    # Reuse clean stop feedback between source-selected stages, while retaining independent arm VM.
    edit('CHASSIS/USER/chassis_auto_test.c',lambda s:s.replace('#include "chassis_auto_test.h"',
        '#include "chassis_auto_test.h"\n#include "startup_axes.h"',1).replace(
        '!g_chassis_auto_active && !g_chassis_auto_cut_vm && !power.active &&',
        '!g_chassis_auto_active && !g_chassis_auto_cut_vm && (!power.active || StartupAxes_Holding()) &&'))
    # Hold monitoring follows explicitly owned arm targets rather than fighting a legitimate action.
    def axes(s):
        s=s.replace('#include "startup_axes.h"','#include "startup_axes.h"\n#include "vehicle_arm.h"',1)
        s=s.replace('g_dm_jog_state==8U){fail(7U);return;}',
            'g_dm_jog_state==8U && !(g_vehicle_arm_dm_owned && g_dm_jog_result==2U)){fail(7U);return;}')
        s=s.replace('if((uint32_t)(now-s_poll_ms)>=50U)',
            'if(!VehicleArm_Busy() && (uint32_t)(now-s_poll_ms)>=50U)')
        s=s.replace('DM_CAN2_POSVEL_SPEED_DEG_S*0.5f','DM_CAN2_POSVEL_SPEED_DEG_S*STARTUP_DM_SPEED_SCALE')
        return s+'\nvoid StartupAxes_SetExtensionHoldTarget(float target){s_target=target;g_startup_extension_target_deg=target;}\n'
    edit('CHASSIS/USER/startup_axes.c',axes)
    # New owner-specific service budget, original standalone arm path keeps its original50ms.
    edit('HARDWARE/DM_CAN2/dm_can2_motion.c',lambda s:s.replace('volatile uint32_t g_dm_startup_go=0U;',
        'volatile uint32_t g_dm_startup_go=0U;\nvolatile uint32_t g_dm_integrated_owner=0U;').replace(
        's_point_mode==6U?100U:50U','(s_point_mode==6U || g_dm_integrated_owner)?100U:50U'))
    # Same address-limited transport for lift and extension; four-wheel Emm entry is unchanged.
    def transport(s):
        lo=s.index('uint32_t StartupAxis_Exchange(')
        before=s[:lo];part=s[lo:]
        part=part.replace('uint32_t StartupAxis_Exchange(const uint8_t *q',
            'uint32_t VehicleArm_Exchange(uint8_t address,const uint8_t *q',1)
        part=part.replace('if(!q || !out)return 7U;', 'if(!q || !out || (address!=5U && address!=6U))return 7U;')
        part=part.replace('CAN1_ExchangeSingle(6U,','CAN1_ExchangeSingle(address,')
        part=part.replace('tx.ExtId=0x0600U|packet;', 'tx.ExtId=((uint32_t)address<<8)|packet;')
        part=part.replace('rx.ExtId==0x600', 'rx.ExtId==((uint32_t)address<<8)')
        part+='\nuint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t n,uint8_t reply_n,void *out)\n{return VehicleArm_Exchange(6U,q,n,reply_n,out);}\n'
        return before+part
    edit('CHASSIS/SYSTEM/CAN/can.c',transport)
    tree=E.parse(S/'gongxun-f407.uvprojx')
    for t in tree.findall('Targets/Target'):
        active=t.findtext('TargetName').startswith('Chassis_')
        group=next(g for g in t.findall('Groups/Group') if g.findtext('GroupName')=='Chassis_USER')
        f=E.SubElement(group.find('Files'),'File')
        for tag,value in [('FileName','vehicle_arm.c'),('FileType','1'),('FilePath','.\\CHASSIS\\USER\\vehicle_arm.c')]:E.SubElement(f,tag).text=value
        op=E.SubElement(f,'FileOption');cp=E.SubElement(op,'CommonProperty');E.SubElement(cp,'IncludeInBuild').text='1' if active else '0'
        if t.findtext('TargetName')=='Chassis_Stage1_Emm':t.find('TargetName').text='Vehicle_Integrated'
        if t.findtext('TargetName')=='Chassis_Stage1_Offline':t.find('TargetName').text='Vehicle_Offline'
    tree.write(S/'gongxun-f407.uvprojx',encoding='utf-8',xml_declaration=True)
    m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    for p in S.rglob('*'):
        if p.is_file() and p.suffix.lower() in {'.c','.h','.s','.inc','.uvprojx'} and not {'Objects','Listings'}.intersection(p.relative_to(S).parts):
            rel=p.relative_to(S).as_posix();h=hashlib.sha256(p.read_bytes()).hexdigest()
            if m['original'].get(rel)!=h:m['changed'][rel]=h
    (EV/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
