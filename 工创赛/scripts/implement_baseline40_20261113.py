"""One-shot byte-preserving patch; before copies include all prior local work."""
from pathlib import Path
import hashlib, json
R=Path(__file__).resolve().parents[2]
E=R/'工创赛/evidence/navigation-planning/2026-11-13/baseline40-implementation'
U=Path('3.stm32f407工程/CHASSIS/USER')
paths=[U/n for n in ['main.c','app_config.h','vehicle_tuning.h','motion_trial20.h','motion_distance_limits.h','app_entry.h','app_entry.c','app_motion_runtime.c','chassis_auto_test.c','app_chassis_service.c','app_input_runtime.c','app_serial_runtime.c']]+[Path('3.stm32f407工程/CHASSIS/HARDWARE/ICM45686/icm45686_raw.h'),Path('工创赛/scripts/vehicle_uart_capture.py')]
before={}
for p in paths:
    data=(R/p).read_bytes(); q=E/'before'/p
    assert not q.exists(),str(q)
    q.parent.mkdir(parents=True,exist_ok=True); q.write_bytes(data)
    before[p.as_posix()]=hashlib.sha256(data).hexdigest()
E.mkdir(parents=True,exist_ok=True)
(E/'before_manifest.json').write_text(json.dumps(before,indent=2,ensure_ascii=False),'utf8')
def edit(name, pairs):
    p=R/(U/name); data=p.read_bytes(); assert data.count(b'\n')==data.count(b'\r\n'),name
    s=data.decode('cp936').replace('\r\n','\n')
    for old,new in pairs:
        assert s.count(old)==1,(name,old,s.count(old))
        s=s.replace(old,new)
    p.write_bytes(s.replace('\n','\r\n').encode('cp936'))
edit('app_config.h', [('#define APP_CONFIG_H\n','#define APP_CONFIG_H\n#ifndef APP_BASELINE40_ENABLE\n#define APP_BASELINE40_ENABLE 0U\n#endif\n#if APP_BASELINE40_ENABLE > 1U\n#error Baseline40_enable_invalid\n#endif\n#define APP_BASELINE40_ACTIVE (APP_BASELINE40_ENABLE && APP_IS_DEBUG && (APP_TEST_MODE == 3U))\n'), ('#define APP_PERIMETER_TRIAL_ACTIVE 1U','#if APP_BASELINE40_ENABLE && (!APP_IS_DEBUG || APP_TEST_MODE != 3U || VEHICLE_INTEGRATED_ENABLE)\n#error Baseline40_requires_debug3_and_no_integrated_entry\n#endif\n#if APP_BASELINE40_ENABLE && (!defined(VEHICLE_STAGE1_ENABLE) || VEHICLE_STAGE1_ENABLE)\n#error Baseline40_requires_explicit_stage1_disabled\n#endif\n#define APP_BASELINE40_RPM 100U\n#define APP_BASELINE40_MM 2100.0f\n#define APP_BASELINE40_MM_S 400.0f\n#define APP_BASELINE40_ACCEL_MM_S2 150.0f\n#define APP_BASELINE40_JERK_MM_S3 300.0f\n#define APP_BASELINE40_FEEDBACK_RPM 130U\n#define APP_BASELINE40_RUN_MS 12779U\n#define APP_PERIMETER_TRIAL_ACTIVE 1U')])
edit('vehicle_tuning.h',[('#define VEHICLE_STAGE1_ENABLE 1U','#ifndef VEHICLE_STAGE1_ENABLE\n#define VEHICLE_STAGE1_ENABLE 1U\n#endif')])
edit('main.c',[
('#define ICM45686_SAMPLE_ONLY 1U','#if APP_BASELINE40_ENABLE\n#define ICM45686_SAMPLE_ONLY 0U\n#else\n#define ICM45686_SAMPLE_ONLY 1U\n#endif'),
('#if !ICM45686_SAMPLE_ONLY\n','#if !ICM45686_SAMPLE_ONLY\n#if APP_BASELINE40_ACTIVE && ICM45686_SAMPLE_ONLY\n#error Baseline40_cannot_use_sample_only\n#endif\n'),
('    SAOMA_EnableReceive();','    if (!APP_BASELINE40_ACTIVE) SAOMA_EnableReceive();'),
('    AppMotion_InitStage1Plan();','    if (!APP_BASELINE40_ACTIVE) AppMotion_InitStage1Plan();'),
('        SAOMA_Update();','        if (!APP_BASELINE40_ACTIVE) SAOMA_Update();'),
('        if(VEHICLE_INTEGRATED_ENABLE){','        if(APP_BASELINE40_ACTIVE) { /* Dedicated chassis branch; no mission dispatch. */ }\n        else if(VEHICLE_INTEGRATED_ENABLE){'),
('        App_StartGateUpdate();','        if (!APP_BASELINE40_ACTIVE) App_StartGateUpdate();'),
('static void App_RadarTask(void *argument)','#if !APP_BASELINE40_ACTIVE\nstatic void App_RadarTask(void *argument)'),
('#endif /* !ICM45686_SAMPLE_ONLY */','#endif /* !APP_BASELINE40_ACTIVE */\n#endif /* !ICM45686_SAMPLE_ONLY */'),
('#if ICM45686_RAW_ENABLE || ICM45686_SAMPLE_ONLY','#if ICM45686_RAW_ENABLE || ICM45686_SAMPLE_ONLY || APP_BASELINE40_ACTIVE'),
('    StartupPose_Init();\n    RadarZoneSwitch_Init();\n    (void)RadarServo_Init();','#if !APP_BASELINE40_ACTIVE\n    StartupPose_Init();\n    RadarZoneSwitch_Init();\n    (void)RadarServo_Init();\n#endif'),
('    SAOMA_Init();','    if (!APP_BASELINE40_ACTIVE) SAOMA_Init();'),
('    configASSERT(xTaskCreate(App_RadarTask, "RadarObs", 768U, NULL, 2U, NULL) == pdPASS);','#if !APP_BASELINE40_ACTIVE\n    configASSERT(xTaskCreate(App_RadarTask, "RadarObs", 768U, NULL, 2U, NULL) == pdPASS);\n#endif'),
('#if ICM45686_RAW_ENABLE\n','#if ICM45686_RAW_ENABLE || APP_BASELINE40_ACTIVE\n'),
('    g_icm_task_create = xTaskCreate(App_IcmRawTask, "IcmRaw", 256U, NULL, 3U, NULL);\n#endif','    g_icm_task_create = xTaskCreate(App_IcmRawTask, "IcmRaw", 256U, NULL, 3U, NULL);\n    configASSERT(g_icm_task_create == pdPASS);\n#endif'),
('static void App_IcmRawTask(void *argument)','uint32_t AppIcm_ReadPublished(IcmRawStatus *out, uint32_t *publication)\n{\n    uint32_t before, after, attempt;\n    if (!out || !publication) return 0U;\n    for (attempt=0U;attempt<3U;attempt++) {\n        before=g_icm_publish_sequence;\n        if (before&1U) continue;\n        __DMB();\n        *out=g_icm_raw_status;\n        __DMB();\n        after=g_icm_publish_sequence;\n        if (before==after && !(after&1U)) { *publication=after;return 1U; }\n    }\n    return 0U;\n}\nstatic void App_IcmRawTask(void *argument)')])
edit('app_chassis_service.c',[
('                StartupAxes_Service(', '#if APP_BASELINE40_ACTIVE\n                App_CombinedTestStep();\n#else\n                StartupAxes_Service('),
('g_startup_axes_state!=3U) App_CombinedTestStep();','g_startup_axes_state!=3U) App_CombinedTestStep();\n#endif'),
('                    if(StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);','                    if(!APP_BASELINE40_ACTIVE && StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);'),
('        StartupAxes_Safety(!g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched);\n        if(g_startup_axes_state==5U && (g_combined_test.state==1U || g_combined_test.state==2U)) MotionStop();','#if !APP_BASELINE40_ACTIVE\n        StartupAxes_Safety(!g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched);\n        if(g_startup_axes_state==5U && (g_combined_test.state==1U || g_combined_test.state==2U)) MotionStop();\n#endif')])
edit('app_input_runtime.c', [('void AppInput_KeyStep(void)\n{','void AppInput_KeyStep(void)\n{\n#if APP_BASELINE40_ACTIVE\n    if (Key_GetEvent()!=0U) {\n        g_start_key_count++;\n        taskENTER_CRITICAL();g_combined_test.request=2U;taskEXIT_CRITICAL();\n    }\n#else'), ('void AppInput_KeyStep(void)\n{\n#if APP_BASELINE40_ACTIVE','void AppInput_KeyStep(void)\n{\n#if APP_BASELINE40_ACTIVE')])
# Close only the final KeyStep function; no other functions follow it.
p=R/U/'app_input_runtime.c';data=p.read_bytes();assert data.rstrip().endswith(b'}');pos=data.rfind(b'}');p.write_bytes(data[:pos]+b'#endif\r\n'+data[pos:])
edit('app_entry.h',[('#define DEBUG_PLAN_MAX DEBUG_PLAN_FORWARD210_20','#define DEBUG_PLAN_FORWARD210_40 8U\n#define DEBUG_PLAN_MAX DEBUG_PLAN_FORWARD210_40')])
edit('app_entry.c',[
('    if (id == DEBUG_PLAN_DEFAULT) {','    if (APP_BASELINE40_ACTIVE && id!=DEBUG_PLAN_FORWARD210_40) return 0U;\n    if (id == DEBUG_PLAN_FORWARD210_40) {\n        MotionAction trial={MOTION_FORWARD_DISTANCE,APP_BASELINE40_RPM,0U,0.0f,APP_BASELINE40_MM};\n        if (!MotionTrial40Allowed(0U,trial.base_rpm,trial.distance_mm)) return 0U;\n        n=1U;actions[0]=trial;\n    } else if (id == DEBUG_PLAN_DEFAULT) {')])
new=R/U/'motion_trial40.h';assert not new.exists();new.write_bytes('''#ifndef MOTION_TRIAL40_H
#define MOTION_TRIAL40_H
#include "app_config.h"
static __inline uint32_t MotionTrial40Allowed(uint32_t direction,uint32_t rpm,float mm)
{
    return APP_BASELINE40_ACTIVE && direction==0U && rpm==APP_BASELINE40_RPM && mm==APP_BASELINE40_MM;
}
static __inline float MotionDistanceAcceleration(uint32_t direction,uint32_t rpm,float mm)
{
    return MotionTrial40Allowed(direction,rpm,mm) ? APP_BASELINE40_ACCEL_MM_S2 : APP_DISTANCE_SCURVE_ACCEL_MM_S2;
}
static __inline float MotionDistanceJerk(uint32_t direction,uint32_t rpm,float mm)
{
    return MotionTrial40Allowed(direction,rpm,mm) ? APP_BASELINE40_JERK_MM_S3 : APP_DISTANCE_SCURVE_JERK_MM_S3;
}
#endif
'''.replace('\n','\r\n').encode('ascii'))
edit('motion_trial20.h',[
('#include "motion_route_trial.h"','#include "motion_route_trial.h"\n#include "motion_trial40.h"'),
('#define MOTION_PROFILE_BASE_CEILING ((APP_SPEED20_TRIAL_ACTIVE || APP_ROUTE_SPEED_TRIAL_ACTIVE) ? APP_SPEED20_TRIAL_RPM : 15U)','#define MOTION_PROFILE_BASE_CEILING (APP_BASELINE40_ACTIVE ? APP_BASELINE40_RPM : ((APP_SPEED20_TRIAL_ACTIVE || APP_ROUTE_SPEED_TRIAL_ACTIVE) ? APP_SPEED20_TRIAL_RPM : 15U))'),
('    if (MotionTrial20Allowed(direction,rpm,mm)) return APP_SPEED20_FEEDBACK_RPM;','    if (MotionTrial40Allowed(direction,rpm,mm)) return APP_BASELINE40_FEEDBACK_RPM;\n    if (MotionTrial20Allowed(direction,rpm,mm)) return APP_SPEED20_FEEDBACK_RPM;'),
('    if (MotionTrial20Allowed(direction,rpm,mm))\n','    if (MotionTrial40Allowed(direction,rpm,mm))\n        return ((APP_BASELINE40_FEEDBACK_RPM+30U)*65536U+59999U)/60000U;\n    if (MotionTrial20Allowed(direction,rpm,mm))\n'),
('    return APP_SPEED20_TRIAL_ACTIVE && rpm==APP_SPEED20_TRIAL_RPM ? 2.0f : APP_DISTANCE_SCURVE_FOLLOW_RPM;','    if (APP_BASELINE40_ACTIVE && rpm==APP_BASELINE40_RPM) return 2.0f;\n    return APP_SPEED20_TRIAL_ACTIVE && rpm==APP_SPEED20_TRIAL_RPM ? 2.0f : APP_DISTANCE_SCURVE_FOLLOW_RPM;'),
('    return APP_SPEED20_TRIAL_ACTIVE && rpm==APP_SPEED20_TRIAL_RPM ? 1.0f : APP_DISTANCE_SCURVE_FOLLOW_KP;','    if (APP_BASELINE40_ACTIVE && rpm==APP_BASELINE40_RPM) return 1.0f;\n    return APP_SPEED20_TRIAL_ACTIVE && rpm==APP_SPEED20_TRIAL_RPM ? 1.0f : APP_DISTANCE_SCURVE_FOLLOW_KP;'),
('    return speed;','    if (APP_BASELINE40_ACTIVE && rpm==APP_BASELINE40_RPM && speed>APP_BASELINE40_MM_S)\n        speed=APP_BASELINE40_MM_S;\n    return speed;')])
edit('motion_distance_limits.h',[
('    if (MotionTrial20Allowed(direction,rpm,mm)) return 1U;','    if (MotionTrial40Allowed(direction,rpm,mm)) return 1U;\n    if (APP_BASELINE40_ACTIVE) return 0U;\n    if (MotionTrial20Allowed(direction,rpm,mm)) return 1U;'),
('    if (MotionTrial20Allowed(direction,rpm,mm)) return APP_SPEED20_RUN_MS;','    if (MotionTrial40Allowed(direction,rpm,mm)) return APP_BASELINE40_RUN_MS;\n    if (MotionTrial20Allowed(direction,rpm,mm)) return APP_SPEED20_RUN_MS;')])
edit('app_motion_runtime.c',[
('g_debug_plan_id!=DEBUG_PLAN_FORWARD210_20)) return 0U;','(g_debug_plan_id!=DEBUG_PLAN_FORWARD210_20 &&\n                 !(g_debug_plan_id==DEBUG_PLAN_FORWARD210_40 && MotionTrial40Allowed(0U,action->base_rpm,action->distance_mm))))) return 0U;'),
('if (rpm>15U && (g_debug_plan_id!=DEBUG_PLAN_FORWARD210_20 || !s_motion_list_submit)) return 0U;','if (rpm>15U && (!s_motion_list_submit || (g_debug_plan_id!=DEBUG_PLAN_FORWARD210_20 &&\n            !(g_debug_plan_id==DEBUG_PLAN_FORWARD210_40 && MotionTrial40Allowed(direction,rpm,distance_mm))))) return 0U;'),
('        g_debug_plan_id != DEBUG_PLAN_DEFAULT) return 0U;','        (g_debug_plan_id != DEBUG_PLAN_DEFAULT &&\n         !(g_debug_plan_id==DEBUG_PLAN_FORWARD210_40 && MOTION_ACTION_COUNT==1U && index==0U &&\n           s_motion_actions[0].kind==MOTION_FORWARD_DISTANCE &&\n           MotionTrial40Allowed(0U,s_motion_actions[0].base_rpm,s_motion_actions[0].distance_mm)))) return 0U;'),
('        if (g_combined_test.index==0U && phase==HR_RUN','        if (!APP_BASELINE40_ACTIVE && g_combined_test.index==0U && phase==HR_RUN'),
('        (void)AppDebugPlan_Load(DEBUG_PLAN_DEFAULT,s_motion_actions,&s_motion_action_count);','        g_debug_plan_id=APP_BASELINE40_ACTIVE ? DEBUG_PLAN_FORWARD210_40 : DEBUG_PLAN_DEFAULT;\n        (void)AppDebugPlan_Load(g_debug_plan_id,s_motion_actions,&s_motion_action_count);'),
('    if(route_speed!=0.0f) {\n        if(!APP_POSITION_DIAGNOSTIC','    if(APP_BASELINE40_ACTIVE && !MotionTrial40Allowed(direction,rpm,distance_mm)) return 0U;\n    if(route_speed!=0.0f) {\n        if(!APP_POSITION_DIAGNOSTIC')])
edit('chassis_auto_test.c',[
('!MotionTrial20Allowed(motion_mode,base_rpm,distance_mm)))','!MotionTrial20Allowed(motion_mode,base_rpm,distance_mm) && !MotionTrial40Allowed(motion_mode,base_rpm,distance_mm)))'),
('!MotionTrial20Allowed(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm)))','!MotionTrial20Allowed(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) &&\n            !MotionTrial40Allowed(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm)))'),
('limits.acceleration = APP_DISTANCE_SCURVE_ACCEL_MM_S2 -','limits.acceleration = MotionDistanceAcceleration(motion_mode,base_rpm,distance_mm) -'),
('limits.jerk = APP_DISTANCE_SCURVE_JERK_MM_S3 -','limits.jerk = MotionDistanceJerk(motion_mode,base_rpm,distance_mm) -'),
('prepared->profile.limits.acceleration != APP_DISTANCE_SCURVE_ACCEL_MM_S2 -','prepared->profile.limits.acceleration != MotionDistanceAcceleration(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) -'),
('prepared->profile.limits.jerk != APP_DISTANCE_SCURVE_JERK_MM_S3 -','prepared->profile.limits.jerk != MotionDistanceJerk(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) -')])
print('Baseline40 control/isolation patches installed; RAW protocol follows separately.')
