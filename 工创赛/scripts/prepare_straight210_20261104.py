"""Stage an isolated PC13 forward 210cm/40cm/s test; no motion or actual edits."""
from pathlib import Path
import shutil,json,hashlib
R=Path(__file__).resolve().parents[1];A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/straight210_20261104';E=R/'evidence/integration/2026-11-04/straight210'
E.mkdir(parents=True,exist_ok=True);assert not S.exists()
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','.git'));original={}
def edit(rel,fn):
 p=S/rel;b=(A/rel).read_bytes();original[rel]=hashlib.sha256(b).hexdigest()
 try:u=b.decode('utf-8');enc='utf-8'
 except UnicodeDecodeError:u=b.decode('gbk');enc='gbk'
 q=E/'before'/rel;q.parent.mkdir(parents=True,exist_ok=True);q.write_bytes(b)
 p.write_bytes(fn(u.replace('\r\n','\n')).replace('\n','\r\n').encode(enc))
def params(u):
 marker='#define VEHICLE_STAGE1_BLEND_ENABLE 1U';assert u.count(marker)==1
 return u.replace(marker,'''/* 独立直线测距：单按PC13，仅前210cm，参考上限40cm/s；本轮不启机构。 */
#define VEHICLE_STRAIGHT_TEST_ENABLE 1U
#define VEHICLE_STRAIGHT_TEST_DISTANCE_CM 210.0f
#define VEHICLE_STRAIGHT_TEST_SPEED_CM_S 40.0f
#define VEHICLE_STRAIGHT_TEST_TIMEOUT_MS 20000U
#define VEHICLE_STAGE1_BLEND_ENABLE 0U''')
edit('USER/vehicle_parameters.h',params)
rel='CHASSIS/USER/straight_test.inc';assert not (A/rel).exists();original[rel]=None
(S/rel).write_text('''/* 独立测距入口：只发布原CAN后端的一条平移动作，不启动任何机构。 */
static void VehicleMission_StraightService(uint32_t key,uint32_t now)
{
    RouteRow row;FieldPoint begin={FIELD_START1_X_MM,FIELD_START1_Y_MM},end=begin;
    if(key){
        if(g_vehicle_state==4U || g_vehicle_state==5U){g_vehicle_cause=19U;return;}
        if(g_vehicle_state!=0U){VehicleMission_Fail(2U);g_vehicle_state=7U;return;}
        if(!App_SerialIdleReady(now) || !g_safety_ready || g_rtos_fault || g_safety_fault || g_can_ctrl_fault_latched)
        {g_vehicle_cause=1U;return;}
        if(!(VEHICLE_STRAIGHT_TEST_DISTANCE_CM>=1 && VEHICLE_STRAIGHT_TEST_DISTANCE_CM<=210 &&
             VEHICLE_STRAIGHT_TEST_SPEED_CM_S>=1.21f && VEHICLE_STRAIGHT_TEST_SPEED_CM_S<=40 &&
             VEHICLE_STRAIGHT_TEST_TIMEOUT_MS>=10000U && VEHICLE_STRAIGHT_TEST_TIMEOUT_MS<=30000U))
        {VehicleMission_Fail(7U);return;}
        end.y_mm+=VEHICLE_STRAIGHT_TEST_DISTANCE_CM*10;
        if(!FieldMap_AxisClear(begin,end,FIELD_BODY_HALF_MM)){VehicleMission_Fail(7U);return;}
        s_vehicle_zone=1U;g_vehicle_run_id++;g_vehicle_stage=1U;g_vehicle_cause=0U;g_vehicle_stage_mask=0U;
        s_vehicle_start=s_vehicle_stage_start=now;g_vehicle_scan_bypassed=1U;
        RouteHeading_Reset(&s_route_heading_reference);
        row=(RouteRow){MOTION_FORWARD_DISTANCE,VEHICLE_STRAIGHT_TEST_DISTANCE_CM,VEHICLE_STRAIGHT_TEST_SPEED_CM_S};
        if(!VehicleMission_Queue(1U,&row,1U)){VehicleMission_Fail(7U);return;}
    }
    if(g_vehicle_state==2U){
        if(g_rtos_fault || g_safety_fault || g_can_ctrl_fault_latched){VehicleMission_Fail(4U);return;}
        if(now-s_vehicle_start>=VEHICLE_STRAIGHT_TEST_TIMEOUT_MS){VehicleMission_Fail(5U);return;}
        if(g_combined_test.state>=4U){VehicleMission_Fail(8U);return;}
        if(VehicleMission_Stopped()){g_vehicle_stage_mask=1U;g_vehicle_state=4U;}
    }
}
''',encoding='utf-8')
def mission(u):
 marker='static void VehicleMission_Service(void)';assert u.count(marker)==1
 u=u.replace(marker,'#include "straight_test.inc"\n'+marker)
 marker='    taskENTER_CRITICAL();key=g_vehicle_key;g_vehicle_key=0U;taskEXIT_CRITICAL();';assert u.count(marker)==1
 u=u.replace(marker,marker+'''
    {static const volatile uint32_t straight_test=VEHICLE_STRAIGHT_TEST_ENABLE;
     if(straight_test){VehicleMission_StraightService(key,now);return;}}
''')
 marker='''    }else if(kind==1U){
        count=snprintf(frame,sizeof(frame),"EX3,''';assert u.count(marker)==1
 u=u.replace(marker,'''    }else if(kind==1U && VEHICLE_STRAIGHT_TEST_ENABLE){
        count=snprintf(frame,sizeof(frame),"LT1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
            (unsigned long)frame_seq,(unsigned long)now,(unsigned long)g_vehicle_run_id,
            (unsigned long)g_vehicle_state,(unsigned long)g_vehicle_cause,
            (unsigned long)(VEHICLE_STRAIGHT_TEST_DISTANCE_CM*10000.0f),
            (unsigned long)(VEHICLE_STRAIGHT_TEST_SPEED_CM_S*10.0f),
            (unsigned long)(CHASSIS_FORWARD_DISTANCE_SCALE*1000000.0f));
    }else if(kind==1U){
        count=snprintf(frame,sizeof(frame),"EX3,''')
 return u
edit('CHASSIS/USER/vehicle_mission.inc',mission)
def main(u):
 old='if (g_combined_test.index==0U && phase==HR_RUN && !g_radar_servo_departure_done &&'
 assert u.count(old)==1
 return u.replace(old,'if (!VEHICLE_STRAIGHT_TEST_ENABLE && g_combined_test.index==0U && phase==HR_RUN && !g_radar_servo_departure_done &&')
edit('CHASSIS/USER/main.c',main)
(E/'manifest.json').write_text(json.dumps(dict(actual=str(A),stage=str(S),original=original,changed={n:hashlib.sha256((S/n).read_bytes()).hexdigest() for n in original},installed=False,flashed=False,restorable_integrated_version=True,distance_scale_changed=False),ensure_ascii=False,indent=2),encoding='utf-8');print(S)
