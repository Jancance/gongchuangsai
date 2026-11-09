"""Stage stop/read/replan legs: X1200 -> cell12 -> offset cell14, retaining current Y."""
from pathlib import Path
import shutil,json,hashlib
R=Path(__file__).resolve().parents[1];A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/dynamic_route_20261104';E=R/'evidence/integration/2026-11-04/dynamic-route'
E.mkdir(parents=True,exist_ok=True);assert not S.exists()
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','.git'));original={}
def edit(rel,fn):
 p=S/rel;b=(A/rel).read_bytes();original[rel]=hashlib.sha256(b).hexdigest()
 try:u=b.decode('utf-8');enc='utf-8'
 except UnicodeDecodeError:u=b.decode('gbk');enc='gbk'
 q=E/'before'/rel;q.parent.mkdir(parents=True,exist_ok=True);q.write_bytes(b)
 p.write_bytes(fn(u.replace('\r\n','\n')).replace('\n','\r\n').encode(enc))
def params(u):
 old='#define VEHICLE_STAGE1_FORWARD_DISTANCE_CM 10.0f';assert u.count(old)==1
 return u.replace(old,'#define VEHICLE_STAGE1_FORWARD_DISTANCE_CM 15.0f')
edit('USER/vehicle_parameters.h',params)
def mission(u):
 marker='static uint32_t s_vehicle_handoff_valid;';assert u.count(marker)==1
 u=u.replace(marker,marker+'''
/* 第2段：1仅横向对齐、2到12、3到14偏置、4行驶完成；每段停稳后重读编码器。 */
volatile uint32_t g_vehicle_route_phase=0U;
static FieldPoint s_vehicle_stage2_leg_target;
''')
 lo=u.index('static uint32_t VehicleMission_Stage2Rows(');hi=u.index('/* 真实观测端调用',lo)
 block='''/* 返回0拒绝，1生成一条动作，2本目标已在停车容差内。 */
static uint32_t VehicleMission_Stage2Rows(RouteRow rows[8],uint32_t *count,FieldPoint *goal)
{
    FieldPoint start,center,via,final,movement;float dx,dy,amount,tol=VEHICLE_HANDOFF_ALIGN_TOL_MM;
    if(!rows || !count || !goal || !s_vehicle_handoff_valid ||
       !FieldMap_CellCenter(2U,&center) || !FieldMap_CellCenter(12U,&via) ||
       !FieldMap_CellCenter(14U,&final) || !(tol>=VEHICLE_MIN_ROUTE_CM*10 && tol<=10) ||
       !(VEHICLE_STAGE2_GOAL_OFFSET_X_MM>=-200 && VEHICLE_STAGE2_GOAL_OFFSET_X_MM<=200))return 0;
    final.x_mm+=VEHICLE_STAGE2_GOAL_OFFSET_X_MM;*goal=final;*count=0;start=s_vehicle_handoff;
    if(g_vehicle_route_phase==1U){
        if(!(start.x_mm>=center.x_mm-FIELD_GRID_SIZE_MM*.5f && start.x_mm<center.x_mm+FIELD_GRID_SIZE_MM*.5f &&
             start.y_mm>=center.y_mm-FIELD_GRID_SIZE_MM*.5f && start.y_mm<center.y_mm+FIELD_GRID_SIZE_MM*.5f) ||
           !(fabsf(center.x_mm-start.x_mm)<=VEHICLE_HANDOFF_MAX_ALIGN_MM))return 0;
        s_vehicle_stage2_leg_target=(FieldPoint){center.x_mm,start.y_mm};
    }else if(g_vehicle_route_phase==2U){
        if(!(fabsf(start.x_mm-via.x_mm)<=tol))return 0;
        s_vehicle_stage2_leg_target=via;
    }else if(g_vehicle_route_phase==3U){
        if(!(fabsf(start.y_mm-final.y_mm)<=tol))return 0;
        s_vehicle_stage2_leg_target=final;
    }else return 0;
    dx=s_vehicle_stage2_leg_target.x_mm-start.x_mm;dy=s_vehicle_stage2_leg_target.y_mm-start.y_mm;
    if(fabsf(dx)<=tol && fabsf(dy)<=tol)return 2;
    movement=start;
    if(g_vehicle_route_phase==2U){
        amount=fabsf(dy);movement.y_mm=s_vehicle_stage2_leg_target.y_mm;
        rows[0]=(RouteRow){dy>0?MOTION_FORWARD_DISTANCE:MOTION_BACKWARD_DISTANCE,amount/10,VEHICLE_STAGE2_FORWARD_CM_S};
    }else{
        amount=fabsf(dx);movement.x_mm=s_vehicle_stage2_leg_target.x_mm;
        rows[0]=(RouteRow){dx>0?MOTION_RIGHT_DISTANCE:MOTION_LEFT_DISTANCE,amount/10,VEHICLE_STAGE2_LATERAL_CM_S};
    }
    if(!(amount>=VEHICLE_MIN_ROUTE_CM*10 && amount<=VEHICLE_MAX_ROUTE_CM*10) ||
       !FieldMap_AxisClear(start,movement,FIELD_BODY_HALF_MM))return 0;
    *count=1;return 1;
}

'''
 u=u[:lo]+block+u[hi:]
 marker='}static uint32_t VehicleMission_Load(uint32_t stage)';assert u.count(marker)==1
 u=u.replace(marker,'''}
static uint32_t VehicleMission_QueueNextStage2(void)
{
    RouteRow rows[8];uint32_t n,result;
    if(!g_vehicle_route_phase)g_vehicle_route_phase=1U;
    for(uint32_t i=0;i<3U && g_vehicle_route_phase<4U;i++){
        result=VehicleMission_Stage2Rows(rows,&n,&s_vehicle_stage2_goal);
        if(!result)return 0;
        if(result==1U)return VehicleMission_Queue(2U,rows,n);
        g_vehicle_route_phase++;
    }
    return g_vehicle_route_phase==4U?2U:0U;
}
static uint32_t VehicleMission_Load(uint32_t stage)''')
 marker='    if(stage<1U || stage>8U || !VehicleMission_PathClear(stage,s_vehicle_zone))return 0U;'
 assert u.count(marker)==1
 u=u.replace(marker,'''    if(stage==2U){
        uint32_t queued=VehicleMission_QueueNextStage2();
        if(queued==2U){g_vehicle_stage=2U;g_vehicle_state=2U;}
        return queued!=0U;
    }
'''+marker)
 marker='        s_vehicle_encoder_based=s_vehicle_handoff_valid=s_vehicle_handoff_session=0U;';assert u.count(marker)==1
 u=u.replace(marker,marker+'\n        g_vehicle_route_phase=0U;s_vehicle_stage2_leg_target=(FieldPoint){0,0};')
 marker='''            if(g_vehicle_stage==2U){
#if VEHICLE_SOURCE_PREP_ENABLE''';assert u.count(marker)==1
 u=u.replace(marker,'''            if(g_vehicle_stage==2U){
                /* 重读本轮最新停车端点后，才计算下一目标的距离。 */
                if(!VehicleMission_CaptureHandoff()){VehicleMission_Fail(10U);return;}
                if(g_vehicle_route_phase<4U){
                    if(!(fabsf(s_vehicle_handoff.x_mm-s_vehicle_stage2_leg_target.x_mm)<=VEHICLE_HANDOFF_ALIGN_TOL_MM &&
                         fabsf(s_vehicle_handoff.y_mm-s_vehicle_stage2_leg_target.y_mm)<=VEHICLE_HANDOFF_ALIGN_TOL_MM))
                    {VehicleMission_Fail(20U);return;}
                    g_vehicle_route_phase++;
                    if(g_vehicle_route_phase<4U){
                        uint32_t queued=VehicleMission_QueueNextStage2();
                        if(!queued){VehicleMission_Fail(7U);return;}
                        if(queued==1U)return;
                    }
                }
#if VEHICLE_SOURCE_PREP_ENABLE''')
 old='"VG3,'+','.join(['%lu']*16)+',"';assert u.count(old)==1
 u=u.replace(old,'"VG4,'+','.join(['%lu']*19)+',"')
 old='(unsigned long)VehicleMission_MilliDeg(s_vehicle_handoff.y_mm),(unsigned long)s_vehicle_handoff_valid);';assert u.count(old)==1
 u=u.replace(old,old[:-2]+''',
            (unsigned long)g_vehicle_route_phase,(unsigned long)s_vehicle_stage2_leg_target.x_mm,
            (unsigned long)s_vehicle_stage2_leg_target.y_mm);''')
 return u
edit('CHASSIS/USER/vehicle_mission.inc',mission)
(E/'manifest.json').write_text(json.dumps(dict(actual=str(A),stage=str(S),original=original,changed={n:hashlib.sha256((S/n).read_bytes()).hexdigest() for n in original},installed=False,flashed=False),ensure_ascii=False,indent=2),encoding='utf-8');print(S)
