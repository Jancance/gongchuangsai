"""Build a separate debug candidate; never edits the live USER or saved turn image."""
import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path

from p1_perimeter_preview import DEFAULT, evaluate
from p1_turn_fault_audit import function

ROOT=Path(__file__).resolve().parents[2]
LIVE=ROOT/'3.stm32f407工程'


def prepare(destination):
    dst=Path(destination).resolve()
    if dst.exists():raise ValueError('Use a new destination; preserve previous candidates.')
    cfg=json.loads(DEFAULT.read_text(encoding='utf8'));preview=evaluate(cfg)
    assert preview['closed'] and preview['boundary_geometry_pass']
    if len(cfg['actions'])!=12 or cfg['start_pose']!={'x_mm':2250,'y_mm':2250,'heading_deg':-90}:
        raise ValueError('This candidate adapter requires the reviewed 12-action start1/downward recipe')
    shutil.copytree(LIVE,dst,ignore=shutil.ignore_patterns('Objects','Listings','*.uvguix.*'))
    (dst/'Objects').mkdir();(dst/'Listings').mkdir()
    user=dst/'USER';changes=[]
    def edit(name,pairs):
        p=user/name;b=p.read_bytes();assert b.count(b'\r\n')==b.count(b'\n'),name
        text=b.decode('gbk')
        for old,new,count in pairs:
            old=old.replace('\n','\r\n');new=new.replace('\n','\r\n')
            assert text.count(old)==count,(name,old[:80],text.count(old),count)
            text=text.replace(old,new)
        p.write_bytes(text.encode('gbk'));changes.append(name)
    def pair(a,b,count=1):return a,b,count
    edit('app_config.h',[
        pair('#include "app_entry_config.h"','#include "app_entry_config.h"\n#if !APP_IS_DEBUG || APP_DEBUG_MODE != 3U\n#error Perimeter_candidate_requires_debug_mode3\n#endif\n#define APP_PERIMETER_TRIAL_ACTIVE 1U\n#define APP_PERIMETER_GROUP_MS 330000U'),
        pair('APP_TURN_BRAKE_TRIAL_ENABLE 0U','APP_TURN_BRAKE_TRIAL_ENABLE 1U'),
        pair('APP_TURN_BRAKE_TRIAL_DPS 15U','APP_TURN_BRAKE_TRIAL_DPS 30U')])
    config=user/'route_config.h';b=config.read_bytes();text=b.decode('gbk')
    text=text.replace('APP_ROUTE_USE_CUSTOM 0','APP_ROUTE_USE_CUSTOM 1').replace('APP_ROUTE_SPEED_TRIAL_ENABLE 0','APP_ROUTE_SPEED_TRIAL_ENABLE 1')
    rows=[]
    for a in cfg['actions']:rows.append(f"    X({a['kind']}, {a['amount']:.1f}f, {a['speed']:.1f}f)")
    table='#define APP_ROUTE_ROWS(X) \\\r\n'+' \\\r\n'.join(rows)+'\r\n'
    text=re.sub(r'#define APP_ROUTE_ROWS\(X\).*?(?=#endif)',lambda m:table,text,flags=re.S)
    config.write_bytes(text.encode('gbk'));changes.append('route_config.h')
    edit('motion_action_plan.h',[pair('#define MOTION_ACTION_CAPACITY 8U','#define MOTION_BACKWARD_DISTANCE 7U\n#define MOTION_ACTION_CAPACITY 12U')])
    edit('motion_control.h',[pair('#define MOVE_RIGHT   2U','#define MOVE_RIGHT   2U\n#define MOVE_BACKWARD 3U /* candidate: route-distance only, not timed public Move */')])
    edit('motion_route_trial.h',[
        pair('if(direction>2U)','if(direction>3U)'),pair('direction>2U ||','direction>3U ||'),
        pair('cm_s=direction==0U ?','cm_s=(direction==0U || direction==3U) ?')])
    edit('chassis_kinematics.c',[
        pair('(commands == NULL) || (mode > 2U)','(commands == NULL) || (mode > 3U)'),
        pair('if (mode == 0U)\n    {\n        logical.rpm[0] = base;', 'if (mode == 0U || mode == 3U)\n    {\n        if(mode==3U) base=-base;\n        logical.rpm[0] = base;'),
        pair('projection[3][4] = {{1,1,1,1},{1,-1,-1,1},{-1,1,1,-1}}','projection[4][4] = {{1,1,1,1},{1,-1,-1,1},{-1,1,1,-1},{-1,-1,-1,-1}}'),
        pair('if (mode > 2U) return 0U;','if (mode > 3U) return 0U;'),
        pair('if (mode > 2U || !Chassis_Finite','if (mode > 3U || !Chassis_Finite'),
        pair('if (mode == 0U) result *=','if (mode == 0U || mode == 3U) result *='),
        pair('for (mode = 0U; mode < 3U; mode++)','for (mode = 0U; mode < 4U; mode++)'),
        pair('body.vx_mm_s = (mode == 0U) ? base * per_rpm : 0.0f;', 'body.vx_mm_s = (mode == 0U || mode == 3U) ? (mode==3U ? -base : base)*per_rpm : 0.0f;'),
        pair('body.vy_mm_s = (mode == 0U) ? 0.0f :','body.vy_mm_s = (mode == 0U || mode == 3U) ? 0.0f :'),
        pair('Chassis_TranslationHoldCommands(3U, 0, actual)','Chassis_TranslationHoldCommands(4U, 0, actual)'),
        pair('    if (Chassis_TranslationPositionMm(3U,zero,zero,sign,raw,&mean,&spread)) return 0U;', '''    if (Chassis_TranslationPositionMm(4U,zero,zero,sign,raw,&mean,&spread)) return 0U;
    sign[0]=1U;sign[1]=0U;sign[2]=1U;sign[3]=0U;
    if(!Chassis_TranslationPositionMm(3U,zero,zero,sign,raw,&mean,&spread) ||
       Chassis_Abs(mean-241.90263f)>0.01f || spread>0.01f)return 0U;'''),
        pair('        Chassis_CalibrateDistanceMm(3U,300.0f,&mean) ||','        !Chassis_CalibrateDistanceMm(3U,300.0f,&mean) ||\n        Chassis_Abs(mean-300.0f*CHASSIS_FORWARD_DISTANCE_SCALE)>0.01f ||\n        Chassis_CalibrateDistanceMm(4U,300.0f,&mean) ||')])
    edit('chassis_auto_test.h',[
        pair('#define CHASSIS_POSITION_ENDPOINT_CAPACITY 8U','#define CHASSIS_POSITION_ENDPOINT_CAPACITY 12U'),
        pair('void ChassisAuto_ReadOdometrySamples(ChassisOdometrySamples *samples);','void ChassisAuto_ReadOdometrySamples(ChassisOdometrySamples *samples);\nvoid ChassisAuto_OdometryPoll(void);')])
    poll='''/* Only inactive, powered, successful gaps/preparation; one bounded CAN transaction. */
void ChassisAuto_OdometryPoll(void)
{
    uint32_t i=0U,j,oldest=0U,result,session,now;
    uint8_t sign=0U;uint32_t raw=0U;CanRxMsg reply={0};PowerCommandStatus power;
    taskENTER_CRITICAL();Power_ReadCommandStatus(&power);
    if(g_chassis_auto_active || !s_vm_requested || !power.active || !Auto_BaseHealthy() ||
       g_chassis_auto_cut_vm || (g_chassis_auto_result && g_chassis_auto_result!=1U))
    {taskEXIT_CRITICAL();return;}
    session=g_chassis_auto_session;now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    for(j=0U;j<4U;j++) {
        uint32_t age=now-s_odometry_samples.ms[j];
        if(!(s_odometry_samples.mask&(1UL<<j))) {i=j;oldest=400U;break;}
        if(age>oldest){oldest=age;i=j;}
    }
    if(oldest<40U){taskEXIT_CRITICAL();return;}
    taskEXIT_CRITICAL();
    result=CAN1_ReadPosition((uint8_t)(i+1U),&reply,&sign,&raw);
    taskENTER_CRITICAL();Power_ReadCommandStatus(&power);
    if(session==g_chassis_auto_session && !g_chassis_auto_active && s_vm_requested && power.active && Auto_BaseHealthy()) {
        Auto_OdometrySample(i,result,sign,raw,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
        if(result!=1U) {g_chassis_auto_cut_vm=1U;g_chassis_auto_cause=3U;}
    }
    taskEXIT_CRITICAL();
}

'''
    edit('chassis_auto_test.c',[
        pair('static const int8_t projection[3][4]={{1,1,1,1},{1,-1,-1,1},{-1,1,1,-1}}','static const int8_t projection[4][4]={{1,1,1,1},{1,-1,-1,1},{-1,1,1,-1},{-1,-1,-1,-1}}'),
        pair('s_translation_mode>2U','s_translation_mode>3U'),
        pair('motion_mode > 2U','motion_mode > 3U',2),
        pair('endpoint.translation_mode = 3U','endpoint.translation_mode = 4U'),
        pair('moving = (g_chassis_auto_state == CHASSIS_AUTO_RUN) ||\n        (s_distance_mode && s_run_started && g_chassis_auto_state == CHASSIS_AUTO_START);',
             'moving = (g_chassis_auto_state == CHASSIS_AUTO_RUN) ||\n        g_chassis_auto_state==CHASSIS_AUTO_VERIFY_ON || g_chassis_auto_state==CHASSIS_AUTO_VERIFY_OFF ||\n        g_chassis_auto_state==CHASSIS_AUTO_SETTLE ||\n        (s_distance_mode && s_run_started && g_chassis_auto_state == CHASSIS_AUTO_START);'),
        pair('if (s_distance_mode && moving)','if (moving)'),
        pair('age >= pdMS_TO_TICKS(180U)','age >= pdMS_TO_TICKS(100U)'),
        pair('!moving || s_stop_due || s_finish_requested ||','!moving || (g_chassis_auto_state!=CHASSIS_AUTO_SETTLE && (s_stop_due || s_finish_requested)) ||'),
        pair('         (s_distance_mode && s_run_started && g_chassis_auto_state == CHASSIS_AUTO_START)) && s_vm_requested)',
             '         (s_distance_mode && s_run_started && g_chassis_auto_state == CHASSIS_AUTO_START) ||\n         g_chassis_auto_state==CHASSIS_AUTO_VERIFY_ON || g_chassis_auto_state==CHASSIS_AUTO_VERIFY_OFF ||\n         g_chassis_auto_state==CHASSIS_AUTO_SETTLE) && s_vm_requested)'),
        pair('            if (g_chassis_auto_state == CHASSIS_AUTO_RUN ||\n                (s_distance_mode && s_run_started && g_chassis_auto_state == CHASSIS_AUTO_START))\n                s_position_due = i + 1U;', '            s_position_due = i + 1U;'),
        pair('uint32_t ChassisAuto_ReadPositions(ChassisPositionSnapshot *snapshot)',poll+'uint32_t ChassisAuto_ReadPositions(ChassisPositionSnapshot *snapshot)')])
    # Keep all ordinary public turn submission/runtime/controller functions byte-for-byte.
    old_main=(user/'main.c').read_bytes().decode('gbk')
    turn=function(old_main,'TurnStart')
    new_turn=turn.replace('uint32_t TurnStart(', 'static uint32_t App_PerimeterTurnStart(').replace('MOTION_ACTION_COUNT==1U','MOTION_ACTION_COUNT==12U')
    edit('route_plan.h',[
        pair('#define ROUTE_RIGHT MOTION_RIGHT_DISTANCE','#define ROUTE_RIGHT MOTION_RIGHT_DISTANCE\n#define ROUTE_BACKWARD MOTION_BACKWARD_DISTANCE'),
        pair('count==1U','(count==1U || count==12U)'),
        pair('row->kind<=MOTION_RIGHT_DISTANCE','row->kind<=MOTION_BACKWARD_DISTANCE'),
        pair('budget>120000U','budget>APP_PERIMETER_GROUP_MS',2)])
    # Coordinates for initial and 12 completed actions; observed values are never reset at corners.
    targets=[preview['start_pose']]+[r['end_pose'] for r in preview['rows']]
    checkpoints='''
typedef struct {uint32_t captured,tick_ms;ChassisOdometryStatus pose;} PerimeterRecord;
volatile PerimeterRecord g_perimeter_records[13];
static uint32_t s_perimeter_odom_started,s_perimeter_completed_seen;
static const float s_perimeter_targets[13][3]={
'''+',\n'.join('{'+','.join(f'{v:.3f}f' for v in p)+'}' for p in targets)+'''
};
'''
    edits=[
        pair('#define MOTION_ACTION_COUNT (s_motion_action_count)','#define MOTION_ACTION_COUNT (s_motion_action_count)\n'+checkpoints),
        pair('#define MOTION_TIMING_RECORD_CAPACITY 8U','#define MOTION_TIMING_RECORD_CAPACITY 12U'),
        pair('"PE2,%lu,%lu,%lu,%lu,%lu,%lu,%lu,"','"PE3,%lu,%lu,%lu,%lu,%lu,%lu,%lu,"'),
        pair('action->kind>MOTION_RIGHT_DISTANCE','action->kind>MOTION_BACKWARD_DISTANCE'),
        pair('action->kind <= MOTION_RIGHT_DISTANCE','action->kind <= MOTION_BACKWARD_DISTANCE',2),
        pair('APP_TURN_BRAKE_TRIAL_ACTIVE &&\n                MOTION_ACTION_COUNT==1U', 'APP_TURN_BRAKE_TRIAL_ACTIVE &&\n                (MOTION_ACTION_COUNT==1U || MOTION_ACTION_COUNT==12U)'),
        pair('return (budget <= 120000U)','return (budget <= APP_PERIMETER_GROUP_MS)'),
        pair('>= 120000U','>= APP_PERIMETER_GROUP_MS'),
        pair('static uint32_t App_CombinedLoadAction(uint32_t index)',new_turn+'\n\nstatic uint32_t App_CombinedLoadAction(uint32_t index)'),
        pair('accepted = TurnStart((action->relative_deg > 0.0f)', 'accepted = App_PerimeterTurnStart((action->relative_deg > 0.0f)'),
        pair('if (g_chassis_auto_active == 0U) App_CanPollStep();','if (g_chassis_auto_active == 0U) {\n                    if(g_combined_test.state==1U || g_combined_test.state==2U) ChassisAuto_OdometryPoll();\n                    else App_CanPollStep();\n                }'),
        pair('    ChassisAuto_ReadOdometrySamples(&samples);','''    if(!s_perimeter_odom_started && g_combined_test.state==1U && source.healthy) {
        if(ChassisOdometry_SetPose(&s_odometry,2250.0f,2250.0f,-90.0f,source.yaw_deg))
            s_perimeter_odom_started=1U;
    }
    ChassisAuto_ReadOdometrySamples(&samples);'''),
        pair('    taskENTER_CRITICAL(); g_chassis_odometry=s_odometry.status; taskEXIT_CRITICAL();','''    taskENTER_CRITICAL();
    g_chassis_odometry=s_odometry.status;
    if(s_perimeter_odom_started && !g_perimeter_records[0].captured && s_odometry.status.valid &&
       g_combined_test.completed==0U) {
        g_perimeter_records[0].captured=1U;g_perimeter_records[0].tick_ms=now;
        g_perimeter_records[0].pose=s_odometry.status;
    }
    if(g_combined_test.completed>s_perimeter_completed_seen && g_combined_test.completed<=12U) {
        uint32_t slot=g_combined_test.completed;
        g_perimeter_records[slot].captured=1U;g_perimeter_records[slot].tick_ms=now;
        g_perimeter_records[slot].pose=s_odometry.status;
        s_perimeter_completed_seen=slot;
    }
    taskEXIT_CRITICAL();'''),
        pair('if(APP_TURN_BRAKE_TRIAL_ACTIVE && g_debug_plan_id==DEBUG_PLAN_DEFAULT) {','if(APP_TURN_BRAKE_TRIAL_ACTIVE && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&\n       ((g_combined_test.state==0U && s_motion_actions[0].kind==MOTION_TURN) || s_combined_turn)) {')]
    edit('main.c',edits)
    source=(user/'main.c').read_bytes().decode('gbk')
    # Store checkpoint diagnostics after normal completion/fault; never drive from these records.
    telemetry='''static void App_PerimeterTelemetry(void)
{
    static uint32_t cursor,last;static char frame[256];
    PerimeterRecord record;uint32_t now=(uint32_t)xTaskGetTickCount();int n;uint16_t crc;
    if(g_combined_test.state<3U || cursor>12U || now-last<100U ||
       !App_SerialDiagnosticWindow(now,80U)) return;
    taskENTER_CRITICAL();record=g_perimeter_records[cursor];taskEXIT_CRITICAL();
    n=snprintf(frame,sizeof(frame),"OP1,%lu,%lu,%lu,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%ld,%ld,%ld,",
       (unsigned long)cursor,(unsigned long)record.captured,(unsigned long)record.tick_ms,
       (long)record.pose.x_mm,(long)record.pose.y_mm,(long)(record.pose.heading_deg*1000.0f),
       (unsigned long)record.pose.valid,(unsigned long)record.pose.reasons,
       (unsigned long)record.pose.continuity_lost,(unsigned long)record.pose.age_ms,
       (long)s_perimeter_targets[cursor][0],(long)s_perimeter_targets[cursor][1],
       (long)(s_perimeter_targets[cursor][2]*1000.0f));
    if(n<0 || (uint32_t)n>=sizeof(frame)-7U) return;
    crc=App_SerialCrc(frame,(uint32_t)n);
    (void)snprintf(frame+n,sizeof(frame)-(uint32_t)n,"%04X\\r\\n",crc);
    if(App_DebugSendString(frame)==1U){cursor++;last=now;}
}
'''
    edit('main.c',[pair('static void App_OdometryTelemetry(void)',telemetry+'\nstatic void App_OdometryTelemetry(void)'),pair('        App_OdometryTelemetry();','        App_OdometryTelemetry();\n        App_PerimeterTelemetry();')])
    final=(user/'main.c').read_bytes().decode('gbk')
    for name in ['TurnStart','App_HeadingRuntimeStep','App_HeadingRuntimeSafety','App_HeadingRuntimeTrip']:
        assert function(final,name)==function(old_main,name),name
    for name in ['heading_session.c','heading_session.h','turn_brake.h','heading_controller.c','heading_controller.h','hwt101.c','hwt101.h']:
        assert (user/name).read_bytes()==(LIVE/'USER'/name).read_bytes(),name
    # Hard identity for default-plan conversion; extra arbitrary high-speed rows are rejected.
    edit('route_plan.h',[pair('    for (i=0U;i<count;i++) {','''    {
        static const RouteRow expected[]={APP_ROUTE_ROWS(ROUTE_ROW)};
        if(count!=12U) {check->error=ROUTE_COUNT;return 0U;}
        for(i=0U;i<count;i++) if(rows[i].kind!=expected[i].kind ||
            rows[i].amount!=expected[i].amount || rows[i].speed!=expected[i].speed)
        {check->error=ROUTE_KIND;return 0U;}
    }
    for (i=0U;i<count;i++) {''')])
    # Record exact source provenance and a patch; no modification to the live project.
    import difflib
    patch=[]
    for name in sorted(set(changes)):
        patch.extend(difflib.unified_diff((LIVE/'USER'/name).read_bytes().decode('gbk').splitlines(True),(user/name).read_bytes().decode('gbk').splitlines(True),fromfile='live/USER/'+name,tofile='perimeter/USER/'+name))
    (dst/'candidate_changes.patch').write_text(''.join(patch),encoding='utf8')
    manifest={'purpose':'record-only perimeter debug candidate; not flashed','hardware_accessed':False,
              'group_deadline_ms':330000,'actions':cfg['actions'],'configuration_sha256':hashlib.sha256(DEFAULT.read_bytes()).hexdigest(),
              'unchanged_rotation_functions':['TurnStart','App_HeadingRuntimeStep','App_HeadingRuntimeSafety','App_HeadingRuntimeTrip'],
              'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in user.iterdir() if p.is_file()}}
    (dst/'candidate_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False),encoding='utf8')
    return dst


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);print(prepare(p.parse_args().out))
