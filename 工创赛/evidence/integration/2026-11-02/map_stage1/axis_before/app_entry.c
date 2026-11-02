#include "app_entry.h"
#include "motion_trial20.h"
#include "route_plan.h"
#include "field_map.h"
volatile RouteCheck g_route_check = {0};
volatile CompetitionEntryStatus g_competition_entry = {0};
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
    if (id == DEBUG_PLAN_DEFAULT) {
#if VEHICLE_STAGE1_ENABLE
        FieldStage1Plan p;RouteRow row;RouteCheck check;
        if(!FieldMap_Stage1(&p)) return 0U;
        row.kind=MOTION_LEFT_DISTANCE;row.amount=p.left_cm;row.speed=p.speed_cm_s;
        if(!RoutePlan_Convert(&row,1U,&g_chassis_geometry,actions,&check))
        {g_route_check=check;return 0U;}
        g_route_check=check;*count=1U;return 1U;
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
