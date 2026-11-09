"""Stage bounded stage1->2 chain with measured encoder handoff and -X goal offset."""
from pathlib import Path
import json,hashlib,shutil
R=Path(__file__).resolve().parents[1];A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/stage12_chain_20261104';E=R/'evidence/integration/2026-11-04/stage12-chain'
E.mkdir(parents=True,exist_ok=True);assert not S.exists()
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','.git'))
original={}
def edit(rel,fn):
 p=S/rel;b=p.read_bytes();original[rel]=hashlib.sha256(b).hexdigest()
 try:u=b.decode('utf-8');enc='utf-8'
 except UnicodeDecodeError:u=b.decode('gbk');enc='gbk'
 (E/'before'/rel).parent.mkdir(parents=True,exist_ok=True);(E/'before'/rel).write_bytes(b)
 p.write_bytes(fn(u.replace('\r\n','\n')).replace('\n','\r\n').encode(enc))
def params(u):
 u=u.replace('#define VEHICLE_TEST_STAGE 2U','#define VEHICLE_TEST_STAGE 1U')
 marker='#define VEHICLE_STAGE2_ROUTE_ONLY 1U';assert u.count(marker)==1
 return u.replace(marker,'''/* 本次只联调第1→2段；固定启停区1，完成第2段即停，不进入抓料。 */
#define VEHICLE_CHAIN_STAGE12 1U
#define VEHICLE_TEST_START_ZONE 1U /* 0沿用选区输入；1/2仅现场调试指定起点。 */
#define VEHICLE_STAGE2_GOAL_OFFSET_X_MM (-50.0f) /* 14格中心沿-X偏置。 */
#define VEHICLE_HANDOFF_ALIGN_TOL_MM 10.0f
#define VEHICLE_HANDOFF_MAX_ALIGN_MM 200.0f
#define VEHICLE_HANDOFF_ENDPOINT_MAX_AGE_MS 1000U
'''+marker+'''
#if VEHICLE_CHAIN_STAGE12 && (VEHICLE_RUN_ALL || VEHICLE_TEST_STAGE!=1U || !VEHICLE_STAGE2_ROUTE_ONLY)
#error Stage12_chain_requires_stage1_start_and_stage2_route_only_stop
#endif
#if VEHICLE_TEST_START_ZONE>2U
#error Test_start_zone_out_of_range
#endif
''')
edit('USER/vehicle_parameters.h',params)
def mission(u):
 lo=u.index('static uint32_t VehicleMission_Stage2Rows(');hi=u.index('/* 真实观测端调用',lo)
 block='''/* 轮编码器推算的接续起点，不是雷达/视觉测得的场地绝对定位。 */
static int64_t s_vehicle_encoder_start[4];
static uint32_t s_vehicle_encoder_based,s_vehicle_handoff_session;
static FieldPoint s_vehicle_handoff;
static uint32_t s_vehicle_handoff_valid;
static uint32_t VehicleMission_CaptureHandoff(void)
{
    ChassisPositionEndpoints e;ChassisAutoStatus b;float w[4],dx,dy;uint32_t i,found=0U;
    uint32_t now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    if(!VEHICLE_CHAIN_STAGE12)return 1U;
    if(g_stage1_motion_skipped)return 0U;
    (void)ChassisAuto_ReadStatus(&b);
    /* 端点表跨会话保留历史，必须按当前session选择，不把旧名义段当搜索段。 */
    for(i=0U;i<CHASSIS_POSITION_ENDPOINT_CAPACITY;i++){
        if(!ChassisAuto_ReadPositionEndpoints(i,&e))break;
        if(e.session==b.session){found=1U;break;}
    }
    if(!found)return 0U;
    if(e.session!=b.session || e.outcome!=1U || e.start_mask!=15U || e.end_mask!=15U ||
       b.active || b.result!=1U || b.cause || b.rotation_stop_mask!=15U)return 0U;
    for(i=0U;i<4U;i++){
        int64_t begin=(int64_t)e.start_raw[i],end=(int64_t)e.end_raw[i];
        if(e.start_sign[i]>1U || e.end_sign[i]>1U ||
           now-e.end_ms[i]>VEHICLE_HANDOFF_ENDPOINT_MAX_AGE_MS)return 0U;
        if(e.start_sign[i])begin=-begin;if(e.end_sign[i])end=-end;
        if(!s_vehicle_encoder_based)s_vehicle_encoder_start[i]=begin;
        w[i]=(float)(end-s_vehicle_encoder_start[i])*3.14159265358979323846f*g_chassis_geometry.wheel_diameter_mm/65536.0f;
        if(g_chassis_forward_direction[i])w[i]=-w[i];
    }
    dx=(-w[0]+w[1]+w[2]-w[3])*0.25f;dy=(w[0]+w[1]+w[2]+w[3])*0.25f;
    if(!Chassis_CalibrateDistanceMm(dx<0?1U:2U,dx,&dx) || !Chassis_CalibrateDistanceMm(dy<0?3U:0U,dy,&dy))return 0U;
    s_vehicle_handoff=(FieldPoint){(s_vehicle_zone==1U?FIELD_START1_X_MM:FIELD_START2_X_MM)+dx,FIELD_START1_Y_MM+dy};
    if(!(s_vehicle_handoff.x_mm>=0 && s_vehicle_handoff.x_mm<FIELD_SIZE_MM &&
         s_vehicle_handoff.y_mm>=0 && s_vehicle_handoff.y_mm<FIELD_SIZE_MM))return 0U;
    s_vehicle_encoder_based=s_vehicle_handoff_valid=1U;s_vehicle_handoff_session=b.session;return 1U;
}
static uint32_t VehicleMission_Stage2Rows(RouteRow rows[8],uint32_t *count,FieldPoint *goal)
{
    FieldPoint start={VEHICLE_STAGE2_START_X_MM,VEHICLE_STAGE2_START_Y_MM},aligned,center,previous,shifted;
    RouteRow built[8];uint32_t n=0U,prefix=0U,i;float align,offset=VEHICLE_STAGE2_GOAL_OFFSET_X_MM,signed_mm;
    if(VEHICLE_CHAIN_STAGE12){if(!s_vehicle_handoff_valid)return 0U;start=s_vehicle_handoff;}
    if(!FieldMap_CellCenter(VEHICLE_STAGE2_START_CELL,&center) ||
       !(start.x_mm>=center.x_mm-FIELD_GRID_SIZE_MM*0.5f && start.x_mm<center.x_mm+FIELD_GRID_SIZE_MM*0.5f &&
         start.y_mm>=center.y_mm-FIELD_GRID_SIZE_MM*0.5f && start.y_mm<center.y_mm+FIELD_GRID_SIZE_MM*0.5f) ||
       !(VEHICLE_HANDOFF_ALIGN_TOL_MM>=VEHICLE_MIN_ROUTE_CM*10 && VEHICLE_HANDOFF_ALIGN_TOL_MM<=10.0f) ||
       !(offset>=-200.0f && offset<=200.0f))return 0U;
    aligned=start;aligned.x_mm=center.x_mm;align=aligned.x_mm-start.x_mm;
    if(!FieldMap_AxisClear(start,aligned,FIELD_BODY_HALF_MM) || !(fabsf(align)<=VEHICLE_HANDOFF_MAX_ALIGN_MM))return 0U;
    if(fabsf(align)>VEHICLE_HANDOFF_ALIGN_TOL_MM){
        built[0]=(RouteRow){align>0?MOTION_RIGHT_DISTANCE:MOTION_LEFT_DISTANCE,fabsf(align)/10.0f,VEHICLE_STAGE2_LATERAL_CM_S};prefix=1U;
    }
    if(!GridRoute_Build(VEHICLE_MAP_VERSION,VEHICLE_STAGE2_START_CELL,aligned,
        VEHICLE_STAGE2_GOAL_CELL,vehicle_stage2_waypoints,sizeof(vehicle_stage2_waypoints),
        VEHICLE_STAGE2_FORWARD_CM_S,VEHICLE_STAGE2_LATERAL_CM_S,built+prefix,8U-prefix,&n,&shifted) || !n)return 0U;
    if(built[prefix+n-1U].kind!=MOTION_RIGHT_DISTANCE && built[prefix+n-1U].kind!=MOTION_LEFT_DISTANCE)return 0U;
    signed_mm=(built[prefix+n-1U].kind==MOTION_RIGHT_DISTANCE?1.0f:-1.0f)*built[prefix+n-1U].amount*10.0f;
    previous=shifted;previous.x_mm-=signed_mm;shifted.x_mm+=offset;signed_mm+=offset;
    if(!(fabsf(signed_mm)>=VEHICLE_MIN_ROUTE_CM*10 && fabsf(signed_mm)<=VEHICLE_MAX_ROUTE_CM*10) ||
       !FieldMap_AxisClear(previous,shifted,FIELD_BODY_HALF_MM))return 0U;
    built[prefix+n-1U].kind=signed_mm>0?MOTION_RIGHT_DISTANCE:MOTION_LEFT_DISTANCE;
    built[prefix+n-1U].amount=fabsf(signed_mm)/10.0f;
    for(i=0U;i<prefix+n;i++)rows[i]=built[i];*count=prefix+n;*goal=shifted;return 1U;
}

'''
 u=u[:lo]+block+u[hi:]
 old='''#if !VEHICLE_RUN_ALL
    g_vehicle_state=4U;return;''';assert old in u
 u=u.replace(old,'''#if !VEHICLE_RUN_ALL
    if(VEHICLE_CHAIN_STAGE12 && g_vehicle_stage==1U){if(!VehicleMission_Load(2U))VehicleMission_Fail(7U);return;}
    g_vehicle_state=4U;return;''')
 u=u.replace('if(VEHICLE_TEST_STAGE<1 || VEHICLE_TEST_STAGE>8 || !g_radar_zone_ready ||',
     'if(VEHICLE_TEST_STAGE<1 || VEHICLE_TEST_STAGE>8 || (!VEHICLE_TEST_START_ZONE && !g_radar_zone_ready) ||')
 u=u.replace('s_vehicle_zone=RadarZoneSwitch_Lock();if(s_vehicle_zone<1 || s_vehicle_zone>2)return;',
     's_vehicle_zone=VEHICLE_TEST_START_ZONE?VEHICLE_TEST_START_ZONE:RadarZoneSwitch_Lock();if(s_vehicle_zone<1 || s_vehicle_zone>2)return;')
 marker='        s_vehicle_start=s_vehicle_stage_start=now;g_vehicle_stage_mask=0U;';assert u.count(marker)==1
 u=u.replace(marker,marker+'''
        s_vehicle_encoder_based=s_vehicle_handoff_valid=s_vehicle_handoff_session=0U;
        s_vehicle_handoff=(FieldPoint){VEHICLE_STAGE2_START_X_MM,VEHICLE_STAGE2_START_Y_MM};''')
 marker='''        if(VehicleMission_Stopped()) {
            if(g_vehicle_stage==1U''';assert marker in u
 u=u.replace(marker,'''        if(VehicleMission_Stopped()) {
            if(g_vehicle_stage==1U && !VehicleMission_CaptureHandoff()){VehicleMission_Fail(10U);return;}
            if(g_vehicle_stage==1U''')
 u=u.replace('if(VEHICLE_RUN_ALL && (s_vehicle_search_used || g_combined_test.completed!=MOTION_ACTION_COUNT))',
             'if(VEHICLE_RUN_ALL && !VEHICLE_CHAIN_STAGE12 && (s_vehicle_search_used || g_combined_test.completed!=MOTION_ACTION_COUNT))')
 old='"VG2,'+','.join(['%lu']*13)+',"';assert old in u
 u=u.replace(old,'"VG3,'+','.join(['%lu']*16)+',"')
 old='(unsigned long)VehicleMission_MilliDeg(s_vehicle_stage2_goal.y_mm));'
 if old not in u:
  old='(unsigned long)s_vehicle_stage2_goal.y_mm);'
 assert old in u, u[u.index('"VG3,'):u.index('"VG3,')+1300]
 u=u.replace(old,old[:-2]+''',
            (unsigned long)VehicleMission_MilliDeg(s_vehicle_handoff.x_mm),
            (unsigned long)VehicleMission_MilliDeg(s_vehicle_handoff.y_mm),(unsigned long)s_vehicle_handoff_valid);''')
 return u
edit('CHASSIS/USER/vehicle_mission.inc',mission)
(E/'manifest.json').write_text(json.dumps(dict(actual=str(A),stage=str(S),original=original,changed={n:hashlib.sha256((S/n).read_bytes()).hexdigest() for n in original},installed=False,flashed=False),ensure_ascii=False,indent=2),encoding='utf-8')
print(S)
