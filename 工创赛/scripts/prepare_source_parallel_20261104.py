"""Stage source-area parallel preparation; no actual firmware writes or hardware access."""
from pathlib import Path
import shutil,json,hashlib
R=Path(__file__).resolve().parents[1];A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/source_parallel_20261104';E=R/'evidence/integration/2026-11-04/source-parallel'
E.mkdir(parents=True,exist_ok=True);assert not S.exists()
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','.git'));original={}
def edit(rel,fn):
 p=S/rel;b=(A/rel).read_bytes();original[rel]=hashlib.sha256(b).hexdigest()
 try:u=b.decode('utf-8');enc='utf-8'
 except UnicodeDecodeError:u=b.decode('gbk');enc='gbk'
 q=E/'before'/rel;q.parent.mkdir(parents=True,exist_ok=True);q.write_bytes(b)
 p.write_bytes(fn(u.replace('\r\n','\n')).replace('\n','\r\n').encode(enc))
def new(rel,src):
 assert not (A/rel).exists();original[rel]=None;(S/rel).write_bytes(src.replace('\n','\r\n').encode('utf-8'))
def params(u):
 marker='#define VEHICLE_STAGE2_ROUTE_ONLY 1U';assert u.count(marker)==1
 return u.replace(marker,marker+'''
/* 到原料区停车后，三个机构准备好后同轮放行；完成后只等视觉。 */
#define VEHICLE_SOURCE_PREP_ENABLE 1U
#define VEHICLE_SOURCE_PREP_DM_DEG 180.0f
#define VEHICLE_SOURCE_PREP_DM_SPEED_DEG_S 90.0f
#define VEHICLE_SOURCE_PREP_DM_ACCEL_DEG_S2 90.0f
#define VEHICLE_SOURCE_PREP_EXTENSION_MM 70.0f /* 用户已确认机械可用行程；比例仍粗标。 */
#define VEHICLE_SOURCE_PREP_EXTENSION_SEGMENT_MM 35.0f
#define VEHICLE_SOURCE_PREP_EXTENSION_RPM 30.0f
#define VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S 60U
#define VEHICLE_SOURCE_PREP_GRIP_DEG 180.0f
#define VEHICLE_SOURCE_PREP_GRIP_RAMP_MS 1500U
#define VEHICLE_SOURCE_PREP_GRIP_SETTLE_MS 300U
#define VEHICLE_SOURCE_PREP_TIMEOUT_MS 12000U
''')
edit('USER/vehicle_parameters.h',params)
new('CHASSIS/USER/source_prepare_math.h',r'''#ifndef SOURCE_PREPARE_MATH_H
#define SOURCE_PREPARE_MATH_H
#include <math.h>
static __inline float SourcePrep_Smooth(float u)
{if(u<=0)return 0;if(u>=1)return 1;return u*u*u*(10.0f+u*(-15.0f+6.0f*u));}
static __inline float SourcePrep_Duration(float delta,float speed,float accel)
{
 if(!(fabsf(delta)<=180 && speed>0 && speed<=300 && accel>0 && accel<=360))return -1;
 return fmaxf(.5f,fmaxf(fabsf(delta)*1.875f/speed,sqrtf(fabsf(delta)*5.774f/accel)));
}
#endif
''')
edit('HARDWARE/DM_CAN2/dm_can2.h',lambda u:u.replace('void DM_CAN2_ModuleService',
 '''extern volatile uint32_t g_dm_source_smooth,g_dm_source_go;
extern volatile float g_dm_source_accel_deg_s2;
void DM_CAN2_ModuleService''',1))
def dm(u):
 u='#include "../../CHASSIS/USER/source_prepare_math.h"\n'+u
 marker='volatile uint32_t g_dm_integrated_owner=0U;';assert u.count(marker)==1
 u=u.replace(marker,marker+'''
volatile uint32_t g_dm_source_smooth=0U,g_dm_source_go=0U;
volatile float g_dm_source_accel_deg_s2=90.0f;
static uint32_t s_source_smooth;
static float s_source_duration;
''')
 marker='        s_point_mode = (request == 4U || request == 5U || request == 6U) ? request : 0U;'
 assert u.count(marker)==1;u=u.replace(marker,marker+'''
        s_source_smooth=(g_dm_source_smooth && (request==4U || request==5U))?1U:0U;''')
 marker='        g_dm_jog_start_deg = s_start * JOG_DEG_PER_RAD;';assert u.count(marker)==1
 u=u.replace(marker,'''        if(s_source_smooth){
            s_source_duration=SourcePrep_Duration((s_target-s_start)*JOG_DEG_PER_RAD,
                g_dm_posvel_latched_speed_deg_s,g_dm_source_accel_deg_s2);
            if(!(s_source_duration>0)){JogStop(13U,now);return;}
        }
'''+marker)
 marker='        if(s_point_mode==6U && !g_dm_startup_go){JogReference(s_start,now);return;}'
 assert u.count(marker)==1;u=u.replace(marker,marker+'''
        if(s_source_smooth && !g_dm_source_go){JogReference(s_start,now);return;}''')
 marker='''        /* 与参考代码一致持续发最终位置和速度上限，未到位持续等待。 */
        JogReference(s_target, now);''';assert marker in u
 u=u.replace('if (s_settled >= 15U)','if (s_settled >= 15U && (!s_source_smooth || (float)g_dm_jog_elapsed_ms*.001f>=s_source_duration))')
 return u.replace(marker,'''        if(s_source_smooth){
            float ratio=(float)g_dm_jog_elapsed_ms*.001f/s_source_duration;
            JogReference(s_start+(s_target-s_start)*SourcePrep_Smooth(ratio),now);
        }else JogReference(s_target,now);''')
edit('HARDWARE/DM_CAN2/dm_can2_motion.c',dm)
edit('CHASSIS/USER/vehicle_arm.h',lambda u:u.replace('#endif','''uint32_t VehicleArm_BeginSourcePreparation(void);
extern volatile uint32_t g_source_prepare_state,g_source_prepare_cause,g_source_prepare_done_mask;
extern volatile uint32_t g_source_prepare_release_ms,g_source_prepare_elapsed_ms,g_source_prepare_segment;
extern volatile float g_source_prepare_extension_mm,g_source_prepare_grip_deg;
#endif'''))
new('CHASSIS/USER/source_prepare.inc',r'''/* 仅由vehicle_arm.c包含，所有CAN/PWM仍由原唯一CAN任务串行服务。 */
#include "chassis_auto_test.h"
#include "source_prepare_math.h"
volatile uint32_t g_source_prepare_state=0U,g_source_prepare_cause=0U,g_source_prepare_done_mask=0U;
volatile uint32_t g_source_prepare_release_ms=0U,g_source_prepare_elapsed_ms=0U,g_source_prepare_segment=0U;
volatile float g_source_prepare_extension_mm=0,g_source_prepare_grip_deg=STARTUP_GROUP_GRIPPER_ANGLE_DEG;
#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t sp_begin,sp_phase,sp_dm_cycle,sp_samples,sp_grip_tick;
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
   if(!query(0x1a,4) || (s_rx.Data[2]&3)){SourcePrep_Fail(22);return;}
   s_options=s_rx.Data[2];sp_phase=1;return;
  case 1:
   if(!position() || fabsf(s_current-sp_origin)>2){SourcePrep_Fail(23);return;}
   sp_goal=sp_origin+STARTUP_EXTENSION_EXTEND_SIGN*VEHICLE_SOURCE_PREP_EXTENSION_SEGMENT_MM*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f;
   sp_phase=2;return;
  case 2:
   if(VehicleArm_Exchange(6,enable,5,3,&s_rx)!=1 || s_rx.Data[1]!=2){SourcePrep_Fail(22);return;}
   sp_phase=3;return;
  case 3:
   if(!query(0x3a,3) || !(s_rx.Data[1]&1) || (s_rx.Data[1]&0x0c)){SourcePrep_Fail(24);return;}
   sp_phase=4;return;
  case 4:
   if(!query(0x35,5) || s_rx.Data[2] || s_rx.Data[3]){SourcePrep_Fail(24);return;}
   sp_phase=5;return;
  default:
   if(g_dm_jog_state!=3 || g_dm_can2_read_cycle==sp_dm_cycle || now-g_dm_can2_feedback_tick>=100)return;
   if(!position() || fabsf(s_current-sp_origin)>2){SourcePrep_Fail(23);return;}
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
   if(!query(0x3a,3) || !(s_rx.Data[1]&1) || (s_rx.Data[1]&0x0c)){SourcePrep_Fail(24);return;}
   s_flags=s_rx.Data[1];sp_phase=7;return;
  }
  if(sp_phase==7){
   if(!position() || s_current<fminf(sp_before,sp_goal)-2 || s_current>fmaxf(sp_before,sp_goal)+2){SourcePrep_Fail(23);return;}
   delta=(s_current-sp_origin)*STARTUP_EXTENSION_EXTEND_SIGN;
   g_source_prepare_extension_mm=delta*3200.0f/(VEHICLE_EXTENSION_PULSES_PER_MM*360.0f);sp_phase=8;return;
  }
  if(!query(0x35,5)){SourcePrep_Fail(22);return;}
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
 sp_origin=g_startup_extension_current_deg;
 if(!(sp_origin>=-100000 && sp_origin<=100000))return 0;
 sp_begin=(uint32_t)xTaskGetTickCount();sp_phase=sp_samples=0;
 g_source_prepare_cause=g_source_prepare_done_mask=g_source_prepare_segment=g_source_prepare_release_ms=0;
 g_source_prepare_extension_mm=0;g_source_prepare_grip_deg=STARTUP_GROUP_GRIPPER_ANGLE_DEG;
 g_source_prepare_state=g_vehicle_arm_state=1;g_vehicle_arm_cause=0;return 1;
#endif
}
''')
def arm(u):
 u=u.replace('g_vehicle_arm_state=3U;g_dm_jog_request=3U;',
 '''g_vehicle_arm_state=3U;g_dm_jog_request=3U;
 if(g_source_prepare_state==1U || g_source_prepare_state==2U){g_source_prepare_state=4U;g_source_prepare_cause=why;g_dm_source_go=0U;}''')
 marker='uint32_t VehicleArm_Begin(const VehicleArmRow *row,uint32_t item)';assert u.count(marker)==1
 u=u.replace(marker,'#include "source_prepare.inc"\n'+marker)
 marker='    if(!VehicleArm_Busy())return;';assert u.count(marker)==1
 return u.replace(marker,marker+'''
    if(g_source_prepare_state==1U || g_source_prepare_state==2U){SourcePrep_Service();return;}''')
edit('CHASSIS/USER/vehicle_arm.c',arm)
def mission(u):
 marker='    if(key && (g_vehicle_state==4U || g_vehicle_state==5U) && !VehicleMission_ResetCompleted())'
 assert marker in u
 u=u.replace(marker,'''    if(key && VEHICLE_SOURCE_PREP_ENABLE && g_source_prepare_state==3U &&
       (g_vehicle_state==4U || g_vehicle_state==5U)){g_vehicle_cause=19U;return;}
'''+marker)
 marker='''    if(g_vehicle_state==3U) {
        if(g_vehicle_arm_row>=32U''';assert marker in u
 u=u.replace(marker,'''    if(g_vehicle_state==3U && g_vehicle_stage==2U && VEHICLE_SOURCE_PREP_ENABLE){
        if(g_vehicle_arm_state==3U || g_source_prepare_state==4U){VehicleMission_Fail(12U);return;}
        if(g_source_prepare_state==3U && g_vehicle_arm_state==2U){g_vehicle_alignment_state=1U;VehicleMission_Advance();}
        return;
    }
    if(g_vehicle_state==3U) {
        if(g_vehicle_arm_row>=32U''')
 marker='''                /* Route DONE is distinct from visual alignment DONE. No synthetic correction. */'''
 assert u.count(marker)==1
 u=u.replace(marker,'''#if VEHICLE_SOURCE_PREP_ENABLE
                {
                    g_vehicle_alignment_state=0U;
                    if(!VehicleArm_BeginSourcePreparation()){VehicleMission_Fail(13U);return;}
                    g_vehicle_state=3U;return;
                }
#else
'''+marker)
 u=u.replace('                if(!VEHICLE_STAGE2_ROUTE_ONLY)return;','                if(!VEHICLE_STAGE2_ROUTE_ONLY)return;\n#endif')
 u=u.replace('static uint32_t last,slot,sequence[3];','static uint32_t last,slot,sequence[4];')
 u=u.replace('slot++%(g_vehicle_stage==2U?3U:2U)','slot++%(g_vehicle_stage==2U?4U:2U)')
 marker='''    }else{
        count=snprintf(frame,sizeof(frame),"VG3,''';assert u.count(marker)==1
 u=u.replace(marker,'''    }else if(kind==3U){
        count=snprintf(frame,sizeof(frame),"SP1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%ld,%ld,",
            (unsigned long)frame_seq,(unsigned long)now,(unsigned long)g_vehicle_run_id,
            (unsigned long)g_source_prepare_state,(unsigned long)g_source_prepare_cause,
            (unsigned long)g_source_prepare_done_mask,(unsigned long)g_source_prepare_release_ms,
            (unsigned long)g_source_prepare_elapsed_ms,(unsigned long)g_source_prepare_segment,
            (long)VehicleMission_MilliDeg(g_source_prepare_extension_mm),(long)VehicleMission_MilliDeg(g_source_prepare_grip_deg));
    }else{
        count=snprintf(frame,sizeof(frame),"VG3,''')
 return u
edit('CHASSIS/USER/vehicle_mission.inc',mission)
(E/'manifest.json').write_text(json.dumps(dict(actual=str(A),stage=str(S),original=original,changed={n:hashlib.sha256((S/n).read_bytes()).hexdigest() for n in original},installed=False,flashed=False),ensure_ascii=False,indent=2),encoding='utf-8');print(S)
