/* Offline adapter: actual table converter, preset loader and extracted main
 * validation/dispatch. Motor requests are captured stubs, never hardware IO. */
#include "src/route_plan.h"
#include "src/motion_distance_limits.h"
#include "src/app_entry.c"
#include "src/motion_control.h"
static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];
static uint32_t s_motion_action_count, s_motion_list_submit;
static uint32_t g_debug_plan_id;
#define MOTION_ACTION_COUNT s_motion_action_count
static uint32_t dispatch_calls, dispatch_kind, dispatch_direction, dispatch_rpm;
static uint32_t dispatch_accept=1U;
static float dispatch_amount;
static uint32_t App_MoveSubmit(uint32_t direction,uint32_t rpm,uint32_t ms,float mm,float reference)
{ (void)ms;(void)reference;dispatch_calls++;dispatch_kind=1U;dispatch_direction=direction;dispatch_rpm=rpm;dispatch_amount=mm/10.0f;return dispatch_accept; }
static uint32_t HWT101_TimingBegin(void) { return 0U; }
static void HWT101_TimingEnd(uint32_t a,uint32_t b) { (void)a;(void)b; }
uint32_t MoveStart(uint32_t direction,uint32_t rpm,uint32_t ms)
{ dispatch_calls++;dispatch_kind=0U;dispatch_direction=direction;dispatch_rpm=rpm;dispatch_amount=(float)ms;return dispatch_accept; }
uint32_t MoveDistanceStart(uint32_t direction,uint32_t rpm,float cm)
{ dispatch_calls++;dispatch_kind=1U;dispatch_direction=direction;dispatch_rpm=rpm;dispatch_amount=cm;return dispatch_accept; }
uint32_t TurnStart(uint32_t direction,float angle,uint32_t rpm)
{ dispatch_calls++;dispatch_kind=2U;dispatch_direction=direction;dispatch_rpm=rpm;dispatch_amount=angle;return dispatch_accept; }
/* LIVE_FUNCTIONS */
/* PARITY_CASES */
static uint32_t checks;
#define CHECK(x) do { checks++; if (!(x)) return 0x80000000U|__LINE__; } while(0)
static float bits_float(uint32_t bits) { union { uint32_t bits; float number; } u;u.bits=bits;return u.number; }
uint32_t test_entry(void)
{
    MotionAction actions[8];
    RouteRow row, rows[9];
    RouteCheck check;
    uint32_t i,n,ok;
    /* Emulator enters test_entry directly, without C runtime RW copy. */
    dispatch_accept=1U;
    for(i=0U;i<sizeof(parity)/sizeof(parity[0]);i++) {
        row.kind=parity[i][0];row.amount=bits_float(parity[i][1]);row.speed=bits_float(parity[i][2]);
        actions[0].kind=99U;
        ok=RoutePlan_Convert(&row,1U,&g_chassis_geometry,actions,&check);
        CHECK(check.error==parity[i][3]);
        CHECK(ok==(check.error==ROUTE_OK));
        if(ok) { CHECK(actions[0].base_rpm==parity[i][4]);CHECK(actions[0].kind==row.kind); }
        else CHECK(actions[0].kind==99U);
    }
    row.kind=MOTION_FORWARD_DISTANCE;row.amount=10.0f;row.speed=5.0f;
    for(i=0U;i<9U;i++) rows[i]=row;
    CHECK(!RoutePlan_Convert(rows,0U,&g_chassis_geometry,actions,&check)&&check.error==ROUTE_COUNT);
    CHECK(!RoutePlan_Convert(rows,9U,&g_chassis_geometry,actions,&check)&&check.error==ROUTE_COUNT);
    CHECK(RoutePlan_Convert(rows,3U,&g_chassis_geometry,actions,&check)&&check.budget_ms==105500U);
    actions[0].kind=99U;
    CHECK(!RoutePlan_Convert(rows,4U,&g_chassis_geometry,actions,&check)&&check.error==ROUTE_BUDGET&&check.segment==4U);
    CHECK(actions[0].kind==99U);
    CHECK(!RoutePlan_Convert(rows,1U,0,actions,&check)&&check.error==ROUTE_GEOMETRY);
    n=77U;actions[0].kind=99U;
    ok=AppDebugPlan_Load(DEBUG_PLAN_DEFAULT,actions,&n);
#if APP_IS_COMPETITION
    CHECK(!ok&&n==77U&&actions[0].kind==99U);
    CHECK(!AppCompetitionEntry_MotionAllowed());
#elif EXPECT_INVALID
    CHECK(!ok&&n==77U&&actions[0].kind==99U);
    CHECK(g_route_check.error==ROUTE_SPEED&&g_route_check.segment==1U);
#else
    CHECK(ok);
#if APP_ROUTE_USE_CUSTOM && APP_TEST_MODE==3U
    CHECK(n==3U&&actions[0].kind==MOTION_FORWARD_DISTANCE);
    CHECK(actions[0].base_rpm==12U&&actions[0].distance_mm==300.0f);
    CHECK(actions[1].base_rpm==7U&&actions[1].distance_mm==200.0f);
    CHECK(actions[2].base_rpm==4U&&actions[2].relative_deg==90.0f);
#else
    CHECK(n==4U&&actions[0].kind==MOTION_FORWARD&&actions[0].run_ms==8000U);
    CHECK(actions[2].kind==MOTION_TURN&&actions[2].base_rpm==0U);
#endif
#endif
    /* Presets other than PLAN0 stay fixed, including rejection of disabled PLAN7. */
    if(APP_IS_DEBUG) {
        CHECK(AppDebugPlan_Load(DEBUG_PLAN_ROUTE_A,actions,&n)&&n==3U&&actions[0].base_rpm==15U);
        CHECK(AppDebugPlan_Load(DEBUG_PLAN_FORWARD210,actions,&n)&&n==1U&&actions[0].distance_mm==2100.0f);
        CHECK(!AppDebugPlan_Load(DEBUG_PLAN_FORWARD237,actions,&n));
        CHECK(!AppDebugPlan_Load(DEBUG_PLAN_FORWARD210_20,actions,&n));
    }
#if APP_TEST_MODE==3U
    s_motion_action_count=1U;g_debug_plan_id=DEBUG_PLAN_DEFAULT;
    s_motion_actions[0].kind=MOTION_TURN;s_motion_actions[0].base_rpm=4U;
    s_motion_actions[0].run_ms=0U;s_motion_actions[0].distance_mm=0.0f;
    s_motion_actions[0].relative_deg=-30.0f;
    CHECK(App_MotionPlanValid());
    CHECK(App_CombinedLoadAction(0U));
    CHECK(dispatch_kind==2U&&dispatch_direction==TURN_CW&&dispatch_rpm==4U&&dispatch_amount==30.0f);
    CHECK(s_motion_list_submit==0U);
    s_motion_actions[0].base_rpm=0U;s_motion_actions[0].relative_deg=90.0f;
    CHECK(App_MotionPlanValid()&&App_CombinedLoadAction(0U));
    CHECK(dispatch_direction==TURN_CCW&&dispatch_rpm==5U);
    s_motion_actions[0].base_rpm=6U;CHECK(!App_MotionPlanValid());
    s_motion_actions[0].kind=MOTION_LEFT_DISTANCE;s_motion_actions[0].base_rpm=7U;
    s_motion_actions[0].relative_deg=0.0f;s_motion_actions[0].distance_mm=200.0f;
    CHECK(App_MotionPlanValid()&&App_CombinedLoadAction(0U));
    CHECK(dispatch_kind==1U&&dispatch_direction==MOVE_LEFT&&dispatch_rpm==7U&&dispatch_amount==20.0f);
    dispatch_accept=0U;CHECK(!App_CombinedLoadAction(0U)&&s_motion_list_submit==0U);
    i=dispatch_calls;CHECK(!App_CombinedLoadAction(1U)&&dispatch_calls==i);
    s_motion_actions[0].base_rpm=52U;CHECK(!App_MotionPlanValid());
#endif
    return checks;
}
