#include "app_entry.h"
#include "motion_trial20.h"
#include "route_plan.h"
#include "field_map.h"
volatile RouteCheck g_route_check = {0};
volatile CompetitionEntryStatus g_competition_entry = {0};
uint32_t AppDebugPlan40Allowed(uint32_t id,uint32_t direction,uint32_t rpm,float mm)
{
    return MotionTrial40Allowed(direction,rpm,mm) &&
        ((((id==DEBUG_PLAN_FORWARD210_40 && direction==0U) ||
           (id==DEBUG_PLAN_LEFT210_40 && direction==1U) ||
           (id==DEBUG_PLAN_RIGHT210_40 && direction==2U) ||
           (id==DEBUG_PLAN_BACKWARD210_40 && direction==3U)) && mm==APP_BASELINE40_MM) ||
         (id==DEBUG_PLAN_RIGHT150_40 && direction==2U && mm==1500.0f) ||
         (id==DEBUG_PLAN_RIGHT30_40 && direction==2U && mm==300.0f) ||
         (id==DEBUG_PLAN_FORWARD150_40 && direction==0U && mm==1500.0f) ||
         (id==DEBUG_PLAN_FORWARD30_40 && direction==0U && mm==300.0f) ||
         (id==DEBUG_PLAN_BACKWARD150_40 && direction==3U && mm==1500.0f) ||
         (id==DEBUG_PLAN_BACKWARD30_40 && direction==3U && mm==300.0f) ||
         (id==DEBUG_PLAN_LEFT150_40 && direction==1U && mm==1500.0f) ||
         (id==DEBUG_PLAN_LEFT30_40 && direction==1U && mm==300.0f));
}
uint32_t AppDebugPlan_Load(uint32_t id, MotionAction *actions, uint32_t *count)
{
#if !APP_ROUTE_USE_CUSTOM || (APP_TEST_MODE != 3U)
    static const MotionAction defaults[] = APP_MOTION_ACTIONS;
#endif
    static const MotionAction route[] = {
        {MOTION_LEFT_DISTANCE,15U,0U,0.0f,300.0f},
        {MOTION_FORWARD_DISTANCE,15U,0U,0.0f,300.0f},
        {MOTION_RIGHT_DISTANCE,15U,0U,0.0f,300.0f}
    };
    uint32_t i,n;
    if (!APP_IS_DEBUG || !actions || !count || id > DEBUG_PLAN_MAX || id == DEBUG_PLAN_FORWARD237) return 0U;
    if (APP_BASELINE40_ACTIVE && id!=DEBUG_PLAN_FORWARD210_40 && id!=DEBUG_PLAN_LEFT210_40 &&
        id!=DEBUG_PLAN_RIGHT210_40 && id!=DEBUG_PLAN_BACKWARD210_40 && id!=DEBUG_PLAN_RIGHT150_40 && id!=DEBUG_PLAN_RIGHT30_40 && id!=DEBUG_PLAN_FORWARD150_40 && id!=DEBUG_PLAN_FORWARD30_40 && id!=DEBUG_PLAN_BACKWARD150_40 && id!=DEBUG_PLAN_BACKWARD30_40 && id!=DEBUG_PLAN_LEFT150_40 && id!=DEBUG_PLAN_LEFT30_40) return 0U;
    if (id == DEBUG_PLAN_FORWARD210_40 || id == DEBUG_PLAN_FORWARD150_40 || id == DEBUG_PLAN_FORWARD30_40) {
        MotionAction trial={MOTION_FORWARD_DISTANCE,APP_BASELINE40_RPM,0U,0.0f,APP_BASELINE40_MM};
        if(id==DEBUG_PLAN_FORWARD150_40) trial.distance_mm=1500.0f;
        else if(id==DEBUG_PLAN_FORWARD30_40) trial.distance_mm=300.0f;
        if (!AppDebugPlan40Allowed(id,0U,trial.base_rpm,trial.distance_mm)) return 0U;
        n=1U;actions[0]=trial;
    } else if (id == DEBUG_PLAN_LEFT210_40 || id == DEBUG_PLAN_LEFT150_40 || id == DEBUG_PLAN_LEFT30_40) {
        MotionAction trial={MOTION_LEFT_DISTANCE,APP_BASELINE40_LEFT_RPM,0U,0.0f,APP_BASELINE40_MM};
        if(id==DEBUG_PLAN_LEFT150_40) trial.distance_mm=1500.0f;
        else if(id==DEBUG_PLAN_LEFT30_40) trial.distance_mm=300.0f;
        if (!AppDebugPlan40Allowed(id,1U,trial.base_rpm,trial.distance_mm)) return 0U;
        n=1U;actions[0]=trial;
    } else if (id == DEBUG_PLAN_RIGHT210_40 || id == DEBUG_PLAN_BACKWARD210_40 || id == DEBUG_PLAN_RIGHT150_40 || id == DEBUG_PLAN_RIGHT30_40 || id == DEBUG_PLAN_BACKWARD150_40 || id == DEBUG_PLAN_BACKWARD30_40) {
        uint32_t direction=(id==DEBUG_PLAN_BACKWARD210_40 || id==DEBUG_PLAN_BACKWARD150_40 || id==DEBUG_PLAN_BACKWARD30_40) ? 3U : 2U;
        MotionAction trial={MOTION_RIGHT_DISTANCE,APP_BASELINE40_LEFT_RPM,0U,0.0f,APP_BASELINE40_MM};
        if(direction==3U) {trial.kind=MOTION_BACKWARD_DISTANCE;trial.base_rpm=APP_BASELINE40_RPM;}
        if(id==DEBUG_PLAN_RIGHT150_40 || id==DEBUG_PLAN_BACKWARD150_40) trial.distance_mm=1500.0f;
        else if(id==DEBUG_PLAN_RIGHT30_40 || id==DEBUG_PLAN_BACKWARD30_40) trial.distance_mm=300.0f;
        if (!AppDebugPlan40Allowed(id,direction,trial.base_rpm,trial.distance_mm)) return 0U;
        n=1U;actions[0]=trial;
    } else if (id == DEBUG_PLAN_DEFAULT) {
#if VEHICLE_STAGE1_ENABLE
        FieldStage1Plan p;RouteRow rows[2];RouteCheck check;
        if(!FieldMap_Stage1(&p)) return 0U;
        rows[0].kind=MOTION_FORWARD_DISTANCE;rows[0].amount=p.forward_cm;rows[0].speed=p.entry_speed_cm_s;
        rows[1].kind=MOTION_LEFT_DISTANCE;rows[1].amount=p.left_cm;rows[1].speed=p.speed_cm_s;
        if(!RoutePlan_Convert(rows,2U,&g_chassis_geometry,actions,&check))
        {g_route_check=check;return 0U;}
        g_route_check=check;*count=2U;return 1U;
#else

#if APP_ROUTE_USE_CUSTOM && (APP_TEST_MODE == 3U)
        static const RouteRow configured[] = { APP_ROUTE_ROWS(ROUTE_ROW) };
        RouteCheck check;
        n=sizeof(configured)/sizeof(configured[0]);
        if (!RoutePlan_Convert(configured,n,&g_chassis_geometry,actions,&check))
        { g_route_check=check; return 0U; }
        g_route_check=check; *count=n; return 1U;
#else
        n = sizeof(defaults)/sizeof(defaults[0]);
        if (!n || n > MOTION_ACTION_CAPACITY) return 0U;
        for (i=0U;i<n;i++) actions[i]=defaults[i];
#endif
#endif
    } else if (id == DEBUG_PLAN_ROUTE_A) {
        n=3U; for(i=0U;i<n;i++) actions[i]=route[i];
    } else if (id == DEBUG_PLAN_FORWARD210_20) {
        MotionAction trial={MOTION_FORWARD_DISTANCE,APP_SPEED20_TRIAL_RPM,0U,0.0f,APP_SPEED20_TRIAL_MM};
        if (!MotionTrial20Allowed(0U,trial.base_rpm,trial.distance_mm)) return 0U;
        n=1U; actions[0]=trial;
    } else if (id == DEBUG_PLAN_FORWARD210) {
        MotionAction long_forward={MOTION_FORWARD_DISTANCE,15U,0U,0.0f,2100.0f};
        n=1U; actions[0]=long_forward;
    } else {
        n=1U; actions[0]=route[(id==DEBUG_PLAN_LEFT30)?0U:((id==DEBUG_PLAN_FORWARD30)?1U:2U)];
    }
    *count=n; return 1U;
}
void AppCompetitionEntry_Submit(const char *code, uint32_t platform_ready)
{
    uint32_t i;
    if (!APP_IS_COMPETITION || g_competition_entry.state != COMP_ENTRY_IDLE) return;
    g_competition_entry.state=COMP_ENTRY_BLOCKED;
    g_competition_entry.reason=COMP_REASON_INVALID_CODE;
    if (!code) return;
    for(i=0U;i<15U;i++) {
        if ((i==3U || i==7U || i==11U) ? (code[i]!='+') : (code[i]<'1' || code[i]>'6')) return;
    }
    if(code[15]!='\0') return;
    for(i=0U;i<16U;i++) g_competition_entry.task_code[i]=code[i];
    g_competition_entry.reason=platform_ready ? COMP_REASON_PLAN_UNIMPLEMENTED : COMP_REASON_PLATFORM;
    /* 不伪造完整任务已实现，不直接调用底层运动，也不改VM。 */
}

/* 任务调度和场地状态尚未接入；统一许可保持拒绝，不借用调试表。 */
uint32_t AppCompetitionEntry_MotionAllowed(void)
{
    return 0U;
}
