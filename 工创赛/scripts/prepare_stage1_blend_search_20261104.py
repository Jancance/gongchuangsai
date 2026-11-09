"""Prepare stage1 two-axis motion and bounded mirrored scan search, no hardware."""
from pathlib import Path
import json,hashlib
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-04/stage1-blend-search'
M=E/'manifest.json';m=json.loads(M.read_text(encoding='utf-8'));A=Path(m['actual']);S=Path(m['stage'])
def edit(rel,fn):
 b=(A/rel).read_bytes()
 try:s=b.decode('utf-8');enc='utf-8'
 except UnicodeDecodeError:s=b.decode('gbk');enc='gbk'
 p=E/'before-install'/rel;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b)
 (S/rel).write_bytes(fn(s.replace('\r\n','\n')).replace('\n','\r\n').encode(enc));m['original'][rel]=hashlib.sha256(b).hexdigest()
def rep(s,old,new):
 assert s.count(old)==1,old[:120]
 return s.replace(old,new)
def parameters(s):
 s=rep(s,'#define VEHICLE_TEST_STAGE 2U','#define VEHICLE_TEST_STAGE 1U')
 s=rep(s,'#define VEHICLE_MAX_TRANSLATION_CM_S 40.0f','#define VEHICLE_MAX_TRANSLATION_CM_S 45.0f')
 s=rep(s,'#define VEHICLE_ROUTE_MAX_BASE_RPM 102U','#define VEHICLE_ROUTE_MAX_BASE_RPM 115U')
 s=rep(s,'#define VEHICLE_ROUTE_FEEDBACK_RPM (VEHICLE_ROUTE_MAX_BASE_RPM+28U)',
       '#define VEHICLE_ROUTE_FEEDBACK_RPM 130U /* 常规单轴反馈保护保持；斜移使用单独合成限值。 */')
 mark='#ifndef VEHICLE_SCAN_BYPASS'
 s=rep(s,mark,'''/* 第1段：同时前进/横移；纵向到150mm后横向继续，不在中途停车。 */
#define VEHICLE_STAGE1_BLEND_ENABLE 1U
#define VEHICLE_STAGE1_FORWARD_DISTANCE_CM 15.0f
#define VEHICLE_STAGE1_FORWARD_CM_S 20.0f
#define VEHICLE_STAGE1_LATERAL_CM_S 45.0f
#define VEHICLE_STAGE1_BLEND_MAX_RPM 175U /* 四轮合成边界，不是单轴提速。 */
#define VEHICLE_STAGE1_BLEND_FEEDBACK_RPM 200U
#define VEHICLE_STAGE1_TRACK_MIN_MM 50.0f
#define VEHICLE_STAGE1_TRACK_MARGIN_MM 20.0f
#define VEHICLE_STAGE1_TRACK_LAG_MS 100.0f
#define VEHICLE_STAGE1_TRACK_MAX_MM 80.0f
#define VEHICLE_STAGE1_FINAL_WAIT_MS 3000U
/* 名义扫码点两侧偏移：第一次15cm，第二次跨回30cm。 */
#define VEHICLE_SCAN_SEARCH_OFFSET_CM 15.0f
#define VEHICLE_SCAN_SEARCH_SPEED_CM_S 45.0f
#define VEHICLE_SCAN_SEARCH_TOTAL_MS 15000U
#if VEHICLE_STAGE1_BLEND_MAX_RPM>300U || VEHICLE_STAGE1_BLEND_FEEDBACK_RPM>300U || VEHICLE_STAGE1_BLEND_FEEDBACK_RPM<VEHICLE_STAGE1_BLEND_MAX_RPM
#error Stage1_blend_wheel_limits_invalid
#endif
'''+mark)
 s=rep(s,'{VF(15.0f,20.0f),VL(105.0f,40.0f),VSTOP}',
       '{VF(VEHICLE_STAGE1_FORWARD_DISTANCE_CM,VEHICLE_STAGE1_FORWARD_CM_S),VL(105.0f,VEHICLE_STAGE1_LATERAL_CM_S),VSTOP}')
 return s
edit('USER/vehicle_parameters.h',parameters)
edit('CHASSIS/USER/vehicle_tuning.h',lambda s:rep(s,'#define VEHICLE_STAGE1_FORWARD_CM 15.0f',
    '#define VEHICLE_STAGE1_FORWARD_CM VEHICLE_STAGE1_FORWARD_DISTANCE_CM'))
rel='CHASSIS/USER/stage1_blend.h';(S/rel).write_bytes((R/'src/stage1_blend.h').read_bytes());m['original'][rel]=None
edit('CHASSIS/USER/chassis_auto_test.h',lambda s:rep(s,'void ChassisAuto_Init(void);','''uint32_t ChassisAuto_BeginPreparedBlend(const ChassisAutoPreparedTranslation *prepared,uint32_t expected_session);
extern volatile uint32_t g_stage1_blend_active;
extern volatile float g_stage1_forward_mm,g_stage1_lateral_mm;
void ChassisAuto_Init(void);'''))
def backend(s):
 s=rep(s,'#include "motion_distance_limits.h"','#include "motion_distance_limits.h"\n#include "stage1_blend.h"')
 mark='static uint32_t s_vm_requested, s_used, s_mode;'
 s=rep(s,mark,'''volatile uint32_t g_stage1_blend_active=0U;
volatile float g_stage1_forward_mm=0.0f,g_stage1_lateral_mm=0.0f;
static Stage1Blend s_blend;
static float s_blend_x_rpm,s_blend_y_rpm;
static uint32_t s_blend_dirty;
'''+mark)
 s=rep(s,'    float mean, spread, low=0.0f, high=0.0f;','    float mean, spread, low=0.0f, high=0.0f,aligned[4],forward_raw=0.0f,forward_mm=0.0f;')
 s=s.replace('if(s_route_trial && s_translation_mode==1U && APP_LEFT_TRIM_ACTIVE)','if(!g_stage1_blend_active && s_route_trial && s_translation_mode==1U && APP_LEFT_TRIM_ACTIVE)')
 s=rep(s,'((ROUTE_TRIAL_FEEDBACK_RPM+30U)*65536U+59999U)',
       '(((g_stage1_blend_active?VEHICLE_STAGE1_BLEND_FEEDBACK_RPM:ROUTE_TRIAL_FEEDBACK_RPM)+30U)*65536U+59999U)')
 s=rep(s,'        mean+=mm*0.25f;','        aligned[i]=mm;mean+=mm*0.25f;')
 mark='    spread=high-low;'
 s=rep(s,mark,mark+'''
    if(g_stage1_blend_active){
        Stage1Blend_Project(s_translation_mode,aligned,&mean,&forward_raw,&spread);
        if(!Chassis_CalibrateDistanceMm(0U,forward_raw,&forward_mm)){Auto_DistanceFaultLocked(5U);return;}
    }
''')
 mark='    s_distance_progress_mm = mean;'
 s=rep(s,mark,mark+'''
    if(g_stage1_blend_active){
        float elapsed=(float)(TickType_t)(now-s_run_tick)*portTICK_PERIOD_MS*.001f;
        float observed=(float)(int32_t)(common_ms-(uint32_t)s_run_tick*portTICK_PERIOD_MS)*.001f;
        float xr=0,yr=0;uint32_t result;
        if(observed<0)observed=0;
        result=Stage1Blend_Step(&s_blend,elapsed,observed,mean,forward_mm,(uint32_t)now*portTICK_PERIOD_MS,&xr,&yr);
        g_stage1_forward_mm=forward_mm;g_stage1_lateral_mm=mean;
        if(result>=2U){Auto_DistanceFaultLocked(result==3U?9U:7U);return;}
        if(result==1U){s_distance_reached=1U;s_finish_requested=s_stop_due=1U;s_finish_tick=now;return;}
        if(xr!=s_blend_x_rpm || yr!=s_blend_y_rpm){s_blend_dirty=1U;Auto_BatchChangedLocked(now,3U);}
        s_blend_x_rpm=xr;s_blend_y_rpm=yr;
        s_translation_base_rpm=(uint32_t)(fabsf(xr)+.5f);
        (void)MotionProfile_Sample(&s_blend.x,elapsed,&s_distance_reference);
        s_profile_done=elapsed>=s_blend.duration;return;
    }
''')
 # The legacy wheel spread remains useful as residual after removing both planned axes.
 s=rep(s,'            if (s_turn_brake_trial) speed_limit=APP_TURN_BRAKE_FEEDBACK_RPM;',
       '            if(g_stage1_blend_active)speed_limit=VEHICLE_STAGE1_BLEND_FEEDBACK_RPM;\n            if (s_turn_brake_trial) speed_limit=APP_TURN_BRAKE_FEEDBACK_RPM;')
 # Reset on every ordinary begin/init; wrapper below enables only an explicit blend request.
 s=rep(s,'    g_chassis_auto_request = 0U;\n    g_chassis_auto_dry_run',
       '    g_stage1_blend_active=0U;s_blend_dirty=0U;\n    g_chassis_auto_request = 0U;\n    g_chassis_auto_dry_run')
 s=rep(s,'            s_translation_mode = prepared->motion_mode;',
       '            g_stage1_blend_active=0U;s_blend_dirty=0U;\n            s_translation_mode = prepared->motion_mode;')
 s=rep(s,'    uint32_t state, session, index, result, expected, i;',
       '    uint32_t state, session, index, result, expected, i,commands_ok;')
 s=rep(s,'(s_distance_mode && s_translation_base_rpm != s_translation_applied_base)))',
       '(s_distance_mode && s_translation_base_rpm != s_translation_applied_base) || (g_stage1_blend_active && s_blend_dirty)))')
 lo=s.index('        if (((s_forward_hold == 1U) ?');hi=s.index('        {\n            Auto_CutLocked(8U);',lo)
 expr=s[lo:hi].strip();assert expr.startswith('if (') and expr.endswith(' != 1U)')
 expr=expr[4:-len(' != 1U)')]
 s=s[:lo]+'''        if(g_stage1_blend_active)commands_ok=Stage1Blend_Commands(s_translation_mode,s_blend_x_rpm,s_blend_y_rpm,effective_heading,s_commands[2]);
        else commands_ok='''+expr+''';
        if(commands_ok!=1U)
'''+s[hi:]
 s=rep(s,'        s_heading_applied_rpm = effective_heading;',
       '        s_blend_dirty=0U;\n        s_heading_applied_rpm = effective_heading;')
 idx=s.index('static uint32_t Auto_BeginTranslation(')
 s=s[:idx]+'''/* Prepared and committed by the same CAN owner; no CAN I/O in this wrapper. */
uint32_t ChassisAuto_BeginPreparedBlend(const ChassisAutoPreparedTranslation *p,uint32_t session)
{
    static Stage1Blend planned;uint32_t accepted;
    if(!p || !Stage1Blend_Build(p->motion_mode,p->distance_mm,p->route_reference_mm_s,&planned))return 0U;
    taskENTER_CRITICAL();
    accepted=ChassisAuto_BeginPreparedTranslation(p,session);
    if(accepted){
        s_blend=planned;g_stage1_blend_active=1U;s_blend_x_rpm=s_blend_y_rpm=0;s_blend_dirty=0U;
        g_stage1_forward_mm=g_stage1_lateral_mm=0;
        s_distance_profile=planned.x;
        s_translation_run_ms=(uint32_t)(planned.duration*1000)+VEHICLE_STAGE1_FINAL_WAIT_MS+2000U;
    }
    taskEXIT_CRITICAL();return accepted;
}

'''+s[idx:]
 return s
edit('CHASSIS/USER/chassis_auto_test.c',backend)
def main(s):
 s=rep(s,'        accepted = ChassisAuto_BeginPreparedTranslation(&prepared,expected_session);',
 '''        accepted = (VEHICLE_STAGE1_BLEND_ENABLE && g_vehicle_stage==1U && g_vehicle_scan_search_state==0U) ?
            ChassisAuto_BeginPreparedBlend(&prepared,expected_session) : ChassisAuto_BeginPreparedTranslation(&prepared,expected_session);''')
 return s
edit('CHASSIS/USER/main.c',main)
edit('CHASSIS/USER/vehicle_mission.h',lambda s:rep(s,'extern volatile uint32_t g_vehicle_alignment_state;',
 'extern volatile uint32_t g_vehicle_alignment_state;\nextern volatile uint32_t g_vehicle_scan_search_state;'))
def field(s):
 s=rep(s,'uint32_t FieldMap_AxisClear(FieldPoint a,FieldPoint b,float half)',
       'uint32_t FieldMap_SweptBoxClear(FieldPoint a,FieldPoint b,float half)')
 s=rep(s,' ||\n        (a.x_mm!=b.x_mm && a.y_mm!=b.y_mm)) return 0U;',') return 0U;')
 idx=s.index('static uint32_t hash_word(')
 s=s[:idx]+'''uint32_t FieldMap_AxisClear(FieldPoint a,FieldPoint b,float half)
{return (a.x_mm==b.x_mm || a.y_mm==b.y_mm) && FieldMap_SweptBoxClear(a,b,half);}
'''+s[idx:]
 return s
edit('CHASSIS/USER/field_map.c',field)
edit('CHASSIS/USER/field_map.h',lambda s:rep(s,'uint32_t FieldMap_TaskCodeValid',
 'uint32_t FieldMap_SweptBoxClear(FieldPoint a,FieldPoint b,float half_mm);\nuint32_t FieldMap_TaskCodeValid'))
def mission(s):
 s='#include "stage1_blend.h"\n'+s
 s=rep(s,'static FieldPoint s_vehicle_stage2_goal;',
 '''static FieldPoint s_vehicle_stage2_goal;
/* 0 nominal blend,1 toward opposite zone,2 toward own zone,3 exhausted,4 code displayed. */
volatile uint32_t g_vehicle_scan_search_state=0U;
static uint32_t s_vehicle_search_begin,s_vehicle_search_used;
''')
 mark='    if(stage==2U){RouteRow rows[8];uint32_t count;return VehicleMission_Stage2Rows(rows,&count,&b);}'
 s=rep(s,mark,'''    if(stage==1U && VEHICLE_STAGE1_BLEND_ENABLE){
        a=(FieldPoint){zone==1U?FIELD_START1_X_MM:FIELD_START2_X_MM,FIELD_START1_Y_MM};
        b=(FieldPoint){FIELD_SCAN_X_MM,FIELD_SCAN_Y_MM};
        return FieldMap_SweptBoxClear(a,b,FIELD_BODY_HALF_MM);
    }
'''+mark)
 # Split load from common queue so search only queues its own bounded rows.
 lo=s.index('    if(!n || n>8U || !RoutePlan_Convert');hi=s.index('\nstatic void VehicleMission_Advance',lo)
 tail=s[lo:hi]
 loadlo=s.index('static uint32_t VehicleMission_Load(')
 loadhead=s[loadlo:lo]
 loadhead=loadhead.replace('    if(stage==2U){','    if(stage==1U && VEHICLE_STAGE1_BLEND_ENABLE){\n        rows[0]=(RouteRow){s_vehicle_zone==1U?MOTION_LEFT_DISTANCE:MOTION_RIGHT_DISTANCE,\n            fabsf((s_vehicle_zone==1U?FIELD_START1_X_MM:FIELD_START2_X_MM)-FIELD_SCAN_X_MM)/10.0f,VEHICLE_STAGE1_LATERAL_CM_S};n=1U;\n    }else if(stage==2U){',1)
 # Queue locals only; route validity checked by load/search caller.
 queue='''static uint32_t VehicleMission_Queue(uint32_t stage,const RouteRow *rows,uint32_t n)
{
    RouteCheck check;ChassisAutoStatus backend;
    ForwardHoldDebug forward={0};CombinedTestDebug combined={0};uint32_t i;
'''+tail
 mark='    if(g_combined_test.state==3U) {'
 queue=rep(queue,mark,'''    if(stage==1U && VEHICLE_STAGE1_BLEND_ENABLE && !g_vehicle_scan_search_state){
        Stage1Blend plan;
        if(!Stage1Blend_Build(s_vehicle_zone==1U?1U:2U,rows[0].amount*10,rows[0].speed*10,&plan))return 0U;
        s_motion_actions[0].run_ms=(uint32_t)(plan.duration*1000)+VEHICLE_STAGE1_FINAL_WAIT_MS+2000U;
    }
'''+mark)
 loadhead=loadhead.replace('RouteRow rows[8];RouteCheck check;ChassisAutoStatus backend;\n    ForwardHoldDebug forward={0};CombinedTestDebug combined={0};uint32_t n,i,direction;', 'RouteRow rows[8];uint32_t n,direction;')
 loadhead+='    return VehicleMission_Queue(stage,rows,n);\n}\n'
 search='''static uint32_t VehicleMission_Search(uint32_t step)
{
    RouteRow row;FieldPoint a={FIELD_SCAN_X_MM,FIELD_SCAN_Y_MM},b=a;
    float offset=VEHICLE_SCAN_SEARCH_OFFSET_CM*10.0f,sign=s_vehicle_zone==1U?-1.0f:1.0f;
    if(!(offset>=10 && offset<=300) || (step!=1U && step!=2U))return 0U;
    if(step==1U)b.x_mm+=sign*offset;
    else {a.x_mm+=sign*offset;b.x_mm-=sign*offset;}
    if(!FieldMap_AxisClear(a,b,FIELD_BODY_HALF_MM))return 0U;
    row=(RouteRow){b.x_mm>a.x_mm?MOTION_RIGHT_DISTANCE:MOTION_LEFT_DISTANCE,
        offset*(step==1U?1.0f:2.0f)/10.0f,VEHICLE_SCAN_SEARCH_SPEED_CM_S};
    s_vehicle_search_used=1U;g_vehicle_scan_search_state=step;return VehicleMission_Queue(1U,&row,1U);
}
'''
 s=s[:loadlo]+queue+loadhead+search+s[hi:]
 s=rep(s,'        g_vehicle_alignment_state=0U;',
       '        g_vehicle_scan_search_state=s_vehicle_search_begin=s_vehicle_search_used=0U;\n        g_vehicle_alignment_state=0U;')
 s=rep(s,'                if(!s_vehicle_code_valid)return;', '''                if(!s_vehicle_code_valid){
                    if(!g_vehicle_scan_search_state){s_vehicle_search_begin=now;if(!VehicleMission_Search(1U))VehicleMission_Fail(17U);}
                    else if(g_vehicle_scan_search_state==1U){if(!VehicleMission_Search(2U))VehicleMission_Fail(17U);}
                    else {g_vehicle_scan_search_state=3U;VehicleMission_Fail(16U);}
                    return;
                }''')
 s=rep(s,'                    s_vehicle_display_done=1U;g_scan_display_count++;',
       '                    s_vehicle_display_done=1U;g_scan_display_count++;g_vehicle_scan_search_state=4U;')
 mark='    if(g_vehicle_state==2U) {'
 s=rep(s,mark,'''    if(g_vehicle_state==2U && g_vehicle_stage==1U && g_vehicle_scan_search_state>=1U &&
       g_vehicle_scan_search_state<=2U && now-s_vehicle_search_begin>=VEHICLE_SCAN_SEARCH_TOTAL_MS){VehicleMission_Fail(18U);return;}
'''+mark)
 # Search alters next stage start even when both search movements report complete.
 s=rep(s,'if(VEHICLE_RUN_ALL && g_combined_test.completed!=MOTION_ACTION_COUNT)',
       'if(VEHICLE_RUN_ALL && (s_vehicle_search_used || g_combined_test.completed!=MOTION_ACTION_COUNT))')
 return s
edit('CHASSIS/USER/vehicle_mission.inc',mission)
m['changed']={rel:hashlib.sha256((S/rel).read_bytes()).hexdigest() for rel in m['original']}
M.write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8');print('Prepared',len(m['changed']),'files; no installation')
