/* Offline only. Actual backend, route converter and extracted frontend;
 * CAN/VM/RTOS below are stubs. RUN setup is seeded, not hardware validation. */
#include "src/app_entry.c"
#include "src/chassis_auto_test.c"
#include "src/motion_control.h"
#include <math.h>
#include <string.h>
uint32_t test_tick,test_depth,test_primask;
volatile uint32_t g_rtos_fault,g_safety_fault,g_safety_ready,g_can_guard_ready;
volatile uint32_t g_motion_diag_fault_latched,g_motion_auto_stop_state,g_motion_auto_stop_cut_vm;
volatile uint32_t g_wheel_test_active,g_wheel_test_cut_vm,g_can_ctrl_fault_latched;
volatile uint32_t g_can_poll_result[4],g_can_poll_flags[4],g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
static uint32_t checks,first_failure,model_mode,model_step,model_fault,model_delayed,model_count;
static float model_gain,position[4],velocity[4],target[4],history[4][32];
volatile uint32_t failure_context[6];
volatile uint32_t model_results[60][9];
volatile float trim_model_positions[60][3];
volatile uint32_t parity_outputs[160][4];
#define CHECK(x) do {checks++;if(!(x)&&!first_failure){first_failure=__LINE__;failure_context[0]=model_mode;failure_context[1]=model_fault;failure_context[2]=model_step;failure_context[3]=g_chassis_auto_cause;failure_context[4]=s_distance_fault_reason;failure_context[5]=model_count;}}while(0)
void Power_ReadCommandStatus(PowerCommandStatus *p){p->active=1U;}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg*r){(void)a;r->Data[1]=1;return 1;}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg*r,int32_t*v)
{
    int32_t logical=(int32_t)(velocity[a-1]>=0 ? velocity[a-1]+0.5f : velocity[a-1]-0.5f);
    (void)r;
    if(model_fault==1U&&a==2U&&model_step>=100U)logical=81;
    if(model_fault==2U&&a==3U&&model_step>=100U)return 0U;
    if(model_fault==5U&&a==4U&&model_step>=100U)logical=-logical;
    *v=g_chassis_forward_direction[a-1] ? -logical : logical;return 1U;
}
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg*r,uint8_t*sign,uint32_t*raw)
{
    float driver=position[a-1]*(g_chassis_forward_direction[a-1] ? -1.0f : 1.0f);
    (void)r;
    if(model_fault==3U&&a==1U&&model_step>=100U)return 0U;
    *sign=driver<0;*raw=(uint32_t)(fabsf(driver)*65536.0f/(3.14159265358979323846f*77.0f)+0.5f);return 1U;
}
uint32_t CAN1_SetMotorEnable(uint8_t a,uint8_t b,CanRxMsg*r){(void)a;(void)b;(void)r;return 1U;}
uint32_t CAN1_SetEmmSpeedRaw(uint8_t a,uint8_t direction,uint16_t rpm,uint8_t accel,CanRxMsg*r)
{
    (void)r;CHECK(test_depth==0U);CHECK(rpm<=54U);CHECK(accel==0U);
    target[a-1]=(direction==g_chassis_forward_direction[a-1]) ? (float)rpm : -(float)rpm;
    return 1U;
}
uint32_t CAN1_StopMotor(uint8_t a,CanRxMsg*r)
{
    (void)r;CHECK(test_depth==0U);
    if(model_fault==4U&&a==2U)return 0U;
    target[a-1]=0;return 1U;
}
/* FRONT_TYPE */
static ForwardHoldDebug g_forward_hold;
static uint32_t s_combined_turn,s_combined_mode,s_combined_base_rpm,s_combined_run_ms;
static float s_combined_distance_mm,s_combined_reference_mm_s;
static uint32_t g_forward_hold_trace_count,g_motion_fault_diag_valid,s_motion_has_action;
static uint32_t s_motion_list_submit,g_debug_plan_id,s_motion_action_count;
static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];
#define MOTION_ACTION_COUNT s_motion_action_count
#define HR_IDLE 0U
#define HR_DONE 3U
static uint32_t mock_accept;
/* Only HeadingSession initialization is stubbed; TurnStart admission and
 * translation-parameter reset are extracted from the actual main.c. */
static uint32_t s_motion_turn_max_rpm,s_heading_exec,g_heading_stop_trace[4];
static uint32_t g_heading_exec_trip,g_heading_exec_phase,g_heading_exec_state,g_heading_exec_request;
static float g_heading_exec_relative_deg;
#define HS_IDLE 0U
static void HeadingSession_Init(uint32_t *session){*session=0U;}
static uint32_t App_MotionAccept(void){return mock_accept;}
static void HWT101_DiagnosticArm(void){}
static uint32_t HWT101_TimingBegin(void){return 0U;}
static void HWT101_TimingEnd(uint32_t a,uint32_t b){(void)a;(void)b;}
/* FRONT_FUNCTIONS */
/* PARITY_CASES */
#if APP_ROUTE_SPEED_TRIAL_ACTIVE
static void backend_reset(void)
{
    uint32_t i,j;
    test_tick=1000U;test_depth=test_primask=0U;
    g_rtos_fault=g_safety_fault=g_can_ctrl_fault_latched=0U;
    g_safety_ready=g_can_guard_ready=1U;
    g_motion_diag_fault_latched=g_motion_auto_stop_state=g_motion_auto_stop_cut_vm=0U;
    g_wheel_test_active=g_wheel_test_cut_vm=0U;
    for(i=0;i<4;i++){position[i]=velocity[i]=target[i]=0;for(j=0;j<32;j++)history[i][j]=0;}
    ChassisAuto_Init();CHECK(ChassisAuto_BeginSequence());ChassisAuto_SafetyUpdate(1U);
}
static void front_reset(void)
{
    ForwardHoldDebug empty={0};g_forward_hold=empty;mock_accept=1U;
    s_motion_has_action=s_motion_list_submit=0U;g_debug_plan_id=0U;
}
static void model(uint32_t mode,float mm,float speed,uint32_t delayed,uint32_t fault)
{
    MotionProfile profile;uint32_t rpm,run_ms,w,step;
    ChassisAutoPreparedTranslation p;
    model_mode=mode;model_fault=fault;model_delayed=delayed;model_step=0;
    model_gain=delayed ? 0.97f : 1.03f;
    backend_reset();
    CHECK(MotionRoute_Build(mode,mm,speed,&profile,&rpm,&run_ms));
    CHECK(ChassisAuto_PrepareRouteTranslation(mode,rpm,run_ms,mm,speed,&p));
    CHECK(ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session));
    CHECK(s_route_trial&&s_translation_base_rpm==0U&&s_profile_active);
    g_chassis_auto_state=CHASSIS_AUTO_RUN;s_run_started=1U;s_run_tick=s_profile_follow_tick=test_tick;
    s_status_valid=s_speed_valid=s_position_valid=15U;s_endpoints[s_endpoint_index].start_mask=15U;
    for(w=0;w<4;w++){
        s_status[w]=1U;s_status_tick[w]=s_speed_tick[w]=s_position_tick[w]=test_tick;
        s_position.sign[w]=s_position.raw[w]=0U;
        s_distance_seen_ms[w]=test_tick;s_distance_seen_raw[w]=s_distance_seen_sign[w]=0U;
    }
    for(step=0;step<6500U&&g_chassis_auto_active;step++) {
        MotionProfileSample sample;
        model_step=step;test_tick=1000U+step*10U;
        for(w=0;w<4;w++) {
            uint32_t lag=delayed ? 4U*w+4U : 0U;
            float demand=step>lag ? history[w][(step+31U-lag)%32U] : 0.0f;
            velocity[w]+=(demand*model_gain-velocity[w])*(0.01f/(delayed ? 0.08f : 0.02f));
            position[w]+=velocity[w]*(3.14159265358979323846f*77.0f/60.0f)*0.01f;
        }
        if(!(fault==6U&&step>=100U))s_heading_command_tick=test_tick;
        if((fault==4U||fault==7U)&&step==100U)CHECK(ChassisAuto_RequestStop());
        Auto_CheckLocked(test_tick);ChassisAuto_CanStep();
        for(w=0;w<4;w++)history[w][step%32U]=target[w];
        CHECK(test_depth==0U);CHECK(s_translation_base_rpm<=rpm);
        CHECK(MotionProfile_Sample(&profile,step*0.01f,&sample));
        CHECK(sample.speed_mm_s>=-0.001f&&sample.speed_mm_s<=speed+0.01f);
        CHECK(fabsf(sample.acceleration_mm_s2)<=profile.limits.acceleration+0.01f);
        CHECK(fabsf(sample.jerk_mm_s3)<=profile.limits.jerk+0.01f);
    }
    CHECK(step<6500U);
    if(!fault) {CHECK(g_chassis_auto_result==1U);CHECK(s_distance_reached);CHECK(g_chassis_auto_stop_mask[2]==15U);}
    else if(fault==7U) {CHECK(g_chassis_auto_result==1U);CHECK(!s_distance_reached);CHECK(g_chassis_auto_stop_mask[2]==15U);}
    else {CHECK(g_chassis_auto_result==2U);CHECK(g_chassis_auto_cut_vm);CHECK(!s_distance_reached);}
    if(mode==1U && !fault) {
        float y=(position[0]-position[1]-position[2]+position[3])*0.25f;
        float x=(position[0]+position[1]+position[2]+position[3])*0.25f;
        trim_model_positions[model_count][0]=y*0.99761905f;
        trim_model_positions[model_count][1]=x;
        trim_model_positions[model_count][2]=x+y*0.99761905f*0.01670644f;
#if APP_LEFT_TRIM_ACTIVE
        if(mm==2100.0f && speed==200.0f) CHECK(fabsf(trim_model_positions[model_count][2])<15.0f);
#endif
    }
    CHECK(model_count<60U);
    model_results[model_count][0]=mode;model_results[model_count][1]=(uint32_t)mm;
    model_results[model_count][2]=(uint32_t)speed;model_results[model_count][3]=delayed;
    model_results[model_count][4]=fault;model_results[model_count][5]=g_chassis_auto_result;
    model_results[model_count][6]=g_chassis_auto_cause;model_results[model_count][7]=s_distance_fault_reason;
    model_results[model_count][8]=step*10U;model_count++;
}
#endif
uint32_t test_entry(void)
{
    MotionProfile profile;uint32_t rpm=0,run_ms=0;
    CHECK(!MotionRoute_Build(3U,300,200,&profile,&rpm,&run_ms));
#if APP_ROUTE_SPEED_TRIAL_ACTIVE
    uint32_t i,d,s,k,count;ChassisAutoPreparedTranslation p,bad;
#if APP_LEFT_TRIM_ACTIVE
    {
        uint32_t b,w;int32_t h;ChassisMotorCommand cmd[4];float progress;
        CHECK(Chassis_CalibrateDistanceMm(1U,2100.0f,&progress));
        CHECK(fabsf(progress-2095.0f)<0.01f);
        for(b=0U;b<=52U;b++)for(h=-2;h<=2;h++) {
            float forward=0.0f,left=0.0f;
            uint32_t trim=(uint32_t)((float)b*APP_LEFT_BACKWARD_RATIO+0.5f);
            CHECK(Chassis_LeftTrimCommands(b,h,cmd));
            for(w=0;w<4;w++) {
                float logical=cmd[w].direction==g_chassis_forward_direction[w] ? cmd[w].speed_raw : -(float)cmd[w].speed_raw;
                CHECK(cmd[w].speed_raw<=54U);CHECK(cmd[w].address==w+1U);
                forward+=logical*0.25f;left+=logical*((w==0U||w==3U)?0.25f:-0.25f);
            }
            CHECK(forward==-(float)trim);CHECK(left==(float)b);
        }
        CHECK(!Chassis_LeftTrimCommands(53U,0,cmd));
        CHECK(!Chassis_LeftTrimCommands(30U,3,cmd));
        /* Unexplained synchronized 60mm wheel residual still stops. */
        backend_reset();Auto_PositionReset(1U);
        s_distance_mode=s_route_trial=s_run_started=1U;s_translation_mode=1U;
        s_distance_target_mm=2100.0f;s_distance_progress_mm=0.0f;
        s_run_tick=test_tick;s_position_valid=15U;
        s_endpoints[s_endpoint_index].start_mask=15U;
        g_chassis_auto_active=1U;g_chassis_auto_state=CHASSIS_AUTO_RUN;
        for(w=0;w<4;w++) {
            s_position_tick[w]=s_distance_seen_ms[w]=test_tick;
            Auto_PositionRemember(w,test_tick,w==2U?1060.0f:1000.0f);
        }
        Auto_DistanceCheckLocked(test_tick);
        CHECK(s_distance_fault_reason==5U);CHECK(g_chassis_auto_cut_vm);

    }
#endif

    MotionAction actions[8];RouteRow rows[8];RouteCheck report;
    for(i=0;i<sizeof(parity)/sizeof(parity[0]);i++) {
        uint32_t ok=MotionRoute_Build(parity[i][0],(float)parity[i][1],(float)parity[i][2],&profile,&rpm,&run_ms);
        CHECK(ok==parity[i][3]);
        if(ok) {
            CHECK(rpm==parity[i][4]);CHECK(fabsf((float)run_ms-(float)parity[i][5])<=2.0f);
            CHECK(fabsf(profile.peak_speed-(float)parity[i][6]*0.001f)<=0.01f);
            parity_outputs[i][0]=rpm;parity_outputs[i][1]=run_ms;
            parity_outputs[i][2]=(uint32_t)(profile.peak_speed*1000.0f);
            parity_outputs[i][3]=(uint32_t)(profile.duration_s*1000000.0f);
        }
    }
    /* Direction caps must remain independent, including altered-cap builds. */
    for(d=0;d<3;d++) {
        CHECK(!MotionRoute_Build(d,300,MotionRoute_MaxSpeed(d)+1.0f,&profile,&rpm,&run_ms));
        CHECK(!MotionRoute_Build(d,99,100,&profile,&rpm,&run_ms));
        CHECK(!MotionRoute_Build(d,2101,100,&profile,&rpm,&run_ms));
        CHECK(!MotionRoute_Build(d,300,-1,&profile,&rpm,&run_ms));
        CHECK(MotionRoute_Build(d,100,MotionRoute_MaxSpeed(d),&profile,&rpm,&run_ms));
        CHECK(profile.peak_speed<MotionRoute_MaxSpeed(d));
    }
    for(i=0;i<8;i++) {rows[i].kind=MOTION_FORWARD_DISTANCE;rows[i].amount=10;rows[i].speed=10;}
    CHECK(RoutePlan_Convert(rows,7,&g_chassis_geometry,actions,&report));
    CHECK(report.budget_ms<=120000U);
    for(i=0;i<8;i++)rows[i].amount=210;
    CHECK(!RoutePlan_Convert(rows,8,&g_chassis_geometry,actions,&report)&&report.error==ROUTE_BUDGET);
    CHECK(AppDebugPlan_Load(0,actions,&count)&&count==3U);
#if FULL20_TABLE
    CHECK(actions[0].reference_mm_s==200.0f&&actions[1].reference_mm_s==200.0f&&actions[2].reference_mm_s==200.0f);
    CHECK(actions[0].kind==4U&&actions[1].kind==5U&&actions[2].kind==6U);
#else
    CHECK(actions[0].reference_mm_s==50.0f);
#endif
    for(d=0;d<3;d++) {
        backend_reset();CHECK(MotionRoute_Build(d,300,100,&profile,&rpm,&run_ms));
        CHECK(ChassisAuto_PrepareRouteTranslation(d,rpm,run_ms,300,100,&p));
        bad=p;bad.route_reference_mm_s=201;CHECK(!ChassisAuto_BeginPreparedTranslation(&bad,g_chassis_auto_session));
        bad=p;bad.base_rpm++;CHECK(!ChassisAuto_BeginPreparedTranslation(&bad,g_chassis_auto_session));
        bad=p;bad.run_ms++;CHECK(!ChassisAuto_BeginPreparedTranslation(&bad,g_chassis_auto_session));
        bad=p;bad.profile.limits.speed+=1;CHECK(!ChassisAuto_BeginPreparedTranslation(&bad,g_chassis_auto_session));
        CHECK(!ChassisAuto_PrepareTranslation(d,52,run_ms,300,&bad));
        CHECK(!ChassisAuto_PrepareRouteTranslation(d,rpm,run_ms,300,0,&bad));
        CHECK(!ChassisAuto_PrepareRouteTranslation(d,rpm,run_ms,0,100,&bad));
        front_reset();CHECK(!App_MoveSubmit(d,rpm,run_ms,300,100));
        s_motion_list_submit=1;g_debug_plan_id=4;CHECK(!App_MoveSubmit(d,rpm,run_ms,300,100));
        g_debug_plan_id=0;g_forward_hold.trip=1;CHECK(!App_MoveSubmit(d,rpm,run_ms,300,100));
        g_forward_hold.trip=0;mock_accept=0;CHECK(!App_MoveSubmit(d,rpm,run_ms,300,100));
        mock_accept=1;CHECK(App_MoveSubmit(d,rpm,run_ms,300,100));
        CHECK(s_combined_reference_mm_s==100&&s_combined_mode==d&&s_combined_run_ms==run_ms);
        front_reset();s_motion_action_count=1;
        s_motion_actions[0]=(MotionAction){MOTION_FORWARD_DISTANCE+d,rpm,run_ms,0,300,100};
        CHECK(App_MotionPlanValid());CHECK(App_CombinedLoadAction(0));
        CHECK(!s_motion_list_submit&&s_combined_reference_mm_s==100);
    }
    front_reset();g_heading_exec_trip=g_heading_exec_phase=0U;
    s_combined_distance_mm=300.0f;s_combined_reference_mm_s=200.0f;s_combined_run_ms=9000U;
    CHECK(TurnStart(TURN_CCW,90.0f,4U));
    CHECK(s_combined_turn==1U&&s_motion_turn_max_rpm==4U);
    CHECK(s_combined_distance_mm==0.0f&&s_combined_reference_mm_s==0.0f&&s_combined_run_ms==0U);
#if !ALTERED_CAPS && !FULL20_TABLE
    for(d=0;d<3;d++)model(d,100,200,1,0);
    for(d=0;d<3;d++)for(s=0;s<3;s++)for(k=0;k<2;k++)for(i=0;i<2;i++)
        model(d,k ? 2100.0f : 300.0f,100.0f+50.0f*s,i,0);
    for(d=0;d<3;d++)for(k=1;k<=7;k++)model(d,2100,200,1,k);
#endif
#else
    CHECK(!MotionRoute_Build(0,300,200,&profile,&rpm,&run_ms));
    CHECK(!MotionRoute_Build(1,300,200,&profile,&rpm,&run_ms));
    CHECK(!MotionRoute_Build(2,300,200,&profile,&rpm,&run_ms));
#endif
    return first_failure ? 0x80000000U|first_failure : checks;
}
