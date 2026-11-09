"""Stage manual radar-shaped waypoints and a read-only visual alignment request."""
from pathlib import Path
import hashlib,json,shutil
R=Path(__file__).resolve().parents[1]
A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/stage2_grid_route_20261103'
E=R/'evidence/integration/2026-11-03/stage2-grid-route'
assert not S.exists()
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','.git'))
E.mkdir(parents=True,exist_ok=True)
changes={}
def edit(rel,fn):
 p=A/rel;b=p.read_bytes()
 try:s=b.decode('utf-8');enc='utf-8'
 except UnicodeDecodeError:s=b.decode('gbk');enc='gbk'
 out=fn(s.replace('\r\n','\n')).replace('\n','\r\n').encode(enc)
 q=E/'before-install'/rel;q.parent.mkdir(parents=True,exist_ok=True);q.write_bytes(b)
 (S/rel).write_bytes(out);changes[rel]=hashlib.sha256(b).hexdigest()
def replace(s,old,new):
 assert s.count(old)==1,old
 return s.replace(old,new)

def parameters(s):
 s=replace(s,'#define VEHICLE_TEST_STAGE 1U','#define VEHICLE_TEST_STAGE 2U')
 mark='/* 当前只做伸缩回零'
 idx=s.index(mark)
 s=s[:idx]+'''/* 第2段手填雷达同形节点：起点2，返回列表不含起点，即12、14。
 * 仅路线调试；到14格中心后停车并等待未来视觉毫米校准，不抓料。
 * 无雷达时来源是MANUAL，不伪造扫描结果。坐标用本项目+X右/+Y前。
 */
#define VEHICLE_STAGE2_ROUTE_ONLY 1U
#define VEHICLE_STAGE2_START_CELL 2U
#define VEHICLE_STAGE2_GOAL_CELL 14U
#define VEHICLE_STAGE2_FORWARD_CM_S 10.0f
#define VEHICLE_STAGE2_LATERAL_CM_S 10.0f
#define VEHICLE_STAGE2_START_X_MM FIELD_SCAN_X_MM
#define VEHICLE_STAGE2_START_Y_MM FIELD_SCAN_Y_MM
#ifdef VEHICLE_PARAMETER_TABLES
static const uint8_t vehicle_stage2_waypoints[]={12U,14U};
#endif
/* 毫米视觉尚未接入：只发布校准请求，不发送位置微调，不宣称校准完成。 */
#define VEHICLE_ALIGNMENT_POSITION_EXECUTOR_ENABLE 0U

'''+s[idx:]
 s=replace(s,'static const uint32_t vehicle_arm_enable[8]={0U,1U,1U,1U,1U,1U,1U,0U};','static const uint32_t vehicle_arm_enable[8]={0U,VEHICLE_STAGE2_ROUTE_ONLY?0U:1U,1U,1U,1U,1U,1U,0U};')
 return s
edit('USER/vehicle_parameters.h',parameters)

grid=r'''#ifndef GRID_ROUTE_H
#define GRID_ROUTE_H
#include "field_map.h"
#include "route_plan.h"
/* v1.35 EVT_PATH_RESULT: start is separate; nodes exclude start, include goal.
 * Protocol/sequence/zone/map-valid checks belong to a future radar adapter.
 * This module only validates project-layout geometry and builds motion rows.
 * Project x points right, y forward; radar raw XY must be explicitly mapped.
 */
#define GRID_ROUTE_SOURCE_MANUAL 1U
#define GRID_ROUTE_SOURCE_RADAR 2U
static __inline uint32_t GridRoute_Build(uint32_t layout_version,uint32_t start_cell,
    FieldPoint start,uint32_t goal,const uint8_t *nodes,uint32_t count,
    float forward_cm_s,float lateral_cm_s,RouteRow *rows,uint32_t capacity,
    uint32_t *row_count,FieldPoint *endpoint)
{
    FieldPoint center,next,current=start;RouteRow staged[MOTION_ACTION_CAPACITY];
    uint32_t i,previous=start_cell;float dx,dy,half=FIELD_GRID_SIZE_MM*0.5f;
    if(!nodes || !rows || !row_count || !endpoint || layout_version!=VEHICLE_MAP_VERSION ||
       !count || count>capacity || capacity>MOTION_ACTION_CAPACITY || goal>24U ||
       nodes[count-1U]!=goal || !FieldMap_CellCenter(start_cell,&center) ||
       !(start.x_mm>=center.x_mm-half && start.x_mm<center.x_mm+half &&
         start.y_mm>=center.y_mm-half && start.y_mm<center.y_mm+half))return 0U;
    if(!(forward_cm_s>=1.21f && forward_cm_s<=VEHICLE_MAX_TRANSLATION_CM_S &&
         lateral_cm_s>=1.21f && lateral_cm_s<=VEHICLE_MAX_TRANSLATION_CM_S))return 0U;
    for(i=0U;i<count;i++){
        if(nodes[i]==previous || !FieldMap_CellCenter(nodes[i],&next) ||
           !FieldMap_AxisClear(current,next,FIELD_BODY_HALF_MM))return 0U;
        dx=next.x_mm-current.x_mm;dy=next.y_mm-current.y_mm;
        if(dx!=0.0f && dy!=0.0f)return 0U;
        if(dx==0.0f && dy==0.0f)return 0U;
        staged[i].kind=dy>0.0f?MOTION_FORWARD_DISTANCE:(dy<0.0f?MOTION_BACKWARD_DISTANCE:
                       (dx>0.0f?MOTION_RIGHT_DISTANCE:MOTION_LEFT_DISTANCE));
        staged[i].amount=(dy!=0.0f?(dy<0.0f?-dy:dy):(dx<0.0f?-dx:dx))/10.0f;
        staged[i].speed=dy!=0.0f?forward_cm_s:lateral_cm_s;
        if(!(staged[i].amount>=VEHICLE_MIN_ROUTE_CM && staged[i].amount<=VEHICLE_MAX_ROUTE_CM))return 0U;
        current=next;previous=nodes[i];
    }
    for(i=0U;i<count;i++)rows[i]=staged[i];
    *row_count=count;*endpoint=current;return 1U;
}
#endif
'''
rel='CHASSIS/USER/grid_route.h';(S/rel).write_bytes(grid.replace('\n','\r\n').encode('ascii'));changes[rel]=None

def mission(s):
 s='#include "grid_route.h"\n#include "vehicle_mission.h"\n'+s
 mark='static char s_vehicle_task_code[16];'
 s=replace(s,mark,mark+'''
volatile uint32_t g_vehicle_alignment_state=0U;
static FieldPoint s_vehicle_stage2_goal;
static uint32_t VehicleMission_Stage2Rows(RouteRow rows[8],uint32_t *count,FieldPoint *goal)
{
    FieldPoint start={VEHICLE_STAGE2_START_X_MM,VEHICLE_STAGE2_START_Y_MM};
    return GridRoute_Build(VEHICLE_MAP_VERSION,VEHICLE_STAGE2_START_CELL,start,
        VEHICLE_STAGE2_GOAL_CELL,vehicle_stage2_waypoints,sizeof(vehicle_stage2_waypoints),
        VEHICLE_STAGE2_FORWARD_CM_S,VEHICLE_STAGE2_LATERAL_CM_S,rows,8U,count,goal);
}
''')
 s=replace(s,'g_vehicle_state=6U;g_stage2_ready=0U;','g_vehicle_state=6U;g_stage2_ready=0U;\n    if(g_vehicle_stage==2U)g_vehicle_alignment_state=3U;')
 mark='    a=starts[stage-1U];'
 s=replace(s,mark,'''    if(stage==2U){RouteRow rows[8];uint32_t count;return VehicleMission_Stage2Rows(rows,&count,&b);}
'''+mark)
 mark='    for(n=0;n<8U && vehicle_drive[stage-1U][n].distance_cm>0;n++) {'
 s=replace(s,mark,'''    if(stage==2U){
        if(!VehicleMission_Stage2Rows(rows,&n,&s_vehicle_stage2_goal))return 0U;
    }else
'''+mark)
 mark='        s_vehicle_scan_baseline=g_saoma_accept_count;'
 s=replace(s,mark,'        g_vehicle_alignment_state=0U;\n'+mark)
 mark='            if(vehicle_arm_enable[g_vehicle_stage-1U]) {'
 s=replace(s,mark,'''            if(g_vehicle_stage==2U){
                /* Route DONE is distinct from visual alignment DONE. No synthetic correction. */
                if(!g_vehicle_alignment_state)g_vehicle_alignment_state=1U;
                if(!VEHICLE_STAGE2_ROUTE_ONLY)return;
            }
'''+mark)
 idx=s.index('static int32_t VehicleMission_MilliDeg(')
 s=s[:idx]+'''/* Read-only future vision request: no acceptance/completion or motor writes. */
uint32_t VehicleMission_ReadAlignmentRequest(VehicleAlignmentRequest *request)
{
    uint32_t valid=0U;
    if(!request)return 0U;
    taskENTER_CRITICAL();
    if(g_vehicle_stage==2U && g_vehicle_alignment_state==1U &&
       (g_vehicle_state==2U || g_vehicle_state==4U) &&
       VehicleMission_Stopped() && StartupAxes_Holding()){
        request->run_id=g_vehicle_run_id;request->stage=2U;request->start_id=s_vehicle_zone;
        request->layout_version=VEHICLE_MAP_VERSION;request->goal_cell=VEHICLE_STAGE2_GOAL_CELL;
        request->nominal_target_x_mm=s_vehicle_stage2_goal.x_mm;
        request->nominal_target_y_mm=s_vehicle_stage2_goal.y_mm;
        request->position_executor_enabled=VEHICLE_ALIGNMENT_POSITION_EXECUTOR_ENABLE;
        valid=1U;
    }
    taskEXIT_CRITICAL();return valid;
}

'''+s[idx:]
 # Additional tagged telemetry; keep existing VM1 schema unchanged.
 mark='    (void)App_DebugSendString(frame);\n}'
 s=replace(s,mark,'''    (void)App_DebugSendString(frame);
    if(g_vehicle_stage==2U){
        count=snprintf(frame,sizeof(frame),"VG2,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
            (unsigned long)(sequence-1U),(unsigned long)now,(unsigned long)g_vehicle_run_id,
            (unsigned long)VEHICLE_MAP_VERSION,(unsigned long)GRID_ROUTE_SOURCE_MANUAL,
            (unsigned long)VEHICLE_STAGE2_START_CELL,(unsigned long)VEHICLE_STAGE2_GOAL_CELL,
            (unsigned long)sizeof(vehicle_stage2_waypoints),(unsigned long)g_combined_test.index,
            (unsigned long)g_vehicle_alignment_state,(unsigned long)VEHICLE_ALIGNMENT_POSITION_EXECUTOR_ENABLE,
            (unsigned long)s_vehicle_stage2_goal.x_mm,(unsigned long)s_vehicle_stage2_goal.y_mm);
        if(count<0 || (uint32_t)count>=sizeof(frame)-7U)return;
        crc=App_SerialCrc(frame,(uint32_t)count);
        (void)snprintf(frame+count,sizeof(frame)-(uint32_t)count,"%04X\\r\\n",crc);
        (void)App_DebugSendString(frame);
    }
}''')
 return s
edit('CHASSIS/USER/vehicle_mission.inc',mission)
def interface(s):
 return replace(s,'extern volatile uint32_t g_vehicle_stage,g_vehicle_state,g_vehicle_cause;',r'''extern volatile uint32_t g_vehicle_stage,g_vehicle_state,g_vehicle_cause;
/* Alignment state: 0 not reached, 1 awaiting calibrated metric vision,
 * 3 invalidated by fault/cancel. No DONE state or position executor yet.
 * Project frame: x right/y forward in mm, zero yaw is nominal +Y.
 */
extern volatile uint32_t g_vehicle_alignment_state;
typedef struct {
    uint32_t run_id,stage,start_id,layout_version,goal_cell;
    float nominal_target_x_mm,nominal_target_y_mm;
    uint32_t position_executor_enabled;
} VehicleAlignmentRequest;
uint32_t VehicleMission_ReadAlignmentRequest(VehicleAlignmentRequest *request);''')
edit('CHASSIS/USER/vehicle_mission.h',interface)
m=dict(actual=str(A),stage=str(S),original=changes,changed={rel:hashlib.sha256((S/rel).read_bytes()).hexdigest() for rel in changes},
       route_source='manual',radar_connected=False,vision_mm_connected=False,position_correction_enabled=False,
       installed=False,flashed=False)
(E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
print('Prepared',list(changes))
