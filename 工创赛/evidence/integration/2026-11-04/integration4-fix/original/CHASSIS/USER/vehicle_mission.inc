#include "route_heading_reference.h"
#include "stage1_blend.h"
#include "vehicle_mission.h"
/* Map source tag kept for the VG4 telemetry frame (was in grid_route.h). */
#define GRID_ROUTE_SOURCE_MANUAL 1U
/* 整车八阶段源码调参入口；按键只提交启动/取消，禁止Watch/无线修改业务参数。 */
volatile uint32_t g_vehicle_stage=VEHICLE_TEST_STAGE,g_vehicle_state=0U,g_vehicle_cause=0U;
volatile uint32_t g_vehicle_key=0U,g_vehicle_arm_row=0U,g_vehicle_item=0U,g_vehicle_stage_mask=0U;
volatile uint32_t g_vehicle_run_id=0U,g_vehicle_scan_bypassed=0U;
volatile uint32_t g_vehicle_correct_pick_count=0U,g_vehicle_correct_place_count=0U;
static uint32_t s_vehicle_start,s_vehicle_stage_start,s_vehicle_zone,s_vehicle_observation;
static uint32_t s_vehicle_scan_baseline,s_vehicle_code_valid,s_vehicle_display_done;
static char s_vehicle_task_code[16];
volatile uint32_t g_vehicle_alignment_state=0U;
static FieldPoint s_vehicle_stage2_goal;
/* 0 nominal blend,1 toward opposite zone,2 toward own zone,3 exhausted,4 code displayed. */
volatile uint32_t g_vehicle_scan_search_state=0U;
static uint32_t s_vehicle_search_begin,s_vehicle_search_used;
/* queue_state:0无等待、1待准入、2已发布、3截止/取消；计时只覆盖本次交接。 */
static uint32_t s_vehicle_queue_pending,s_vehicle_queue_begin;
volatile uint32_t g_vehicle_queue_state,g_vehicle_queue_first_mask,g_vehicle_queue_last_mask;
volatile uint32_t g_vehicle_queue_wait_ms,g_vehicle_queue_attempts,g_vehicle_queue_stage;


/* 轮编码器推算的接续起点，不是雷达/视觉测得的场地绝对定位。 */
static int64_t s_vehicle_encoder_start[4];
static uint32_t s_vehicle_encoder_based,s_vehicle_handoff_session;
static FieldPoint s_vehicle_handoff;
static uint32_t s_vehicle_handoff_valid;
/* 第2段：1仅横向对齐、2到12、3到14偏置、4行驶完成；每段停稳后重读编码器。 */
volatile uint32_t g_vehicle_route_phase=0U;
static FieldPoint s_vehicle_stage2_leg_target;

static uint32_t VehicleMission_CaptureHandoff(void)
{
    ChassisPositionEndpoints e;ChassisAutoStatus b;float w[4],dx,dy;uint32_t i,found=0U;
    uint32_t now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    if(!VEHICLE_CHAIN_STAGE12)return 1U;
    if(g_stage1_motion_skipped)return 0U;
    (void)ChassisAuto_ReadStatus(&b);
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
    s_vehicle_handoff=(FieldPoint){(s_vehicle_zone==1U?FIELD_START1_X_MM:FIELD_START2_X_MM)+dx,(s_vehicle_zone==1U?FIELD_START1_Y_MM:FIELD_START2_Y_MM)+dy};
    if(!(s_vehicle_handoff.x_mm>=0 && s_vehicle_handoff.x_mm<FIELD_SIZE_MM &&
         s_vehicle_handoff.y_mm>=0 && s_vehicle_handoff.y_mm<FIELD_SIZE_MM))return 0U;
    s_vehicle_encoder_based=s_vehicle_handoff_valid=1U;s_vehicle_handoff_session=b.session;return 1U;
}
/* 返回0拒绝，1生成一条动作，2本目标已在停车容差内。 */
static uint32_t VehicleMission_Stage2Rows(RouteRow rows[8],uint32_t *count,FieldPoint *goal)
{
    FieldPoint start,center,via,final,movement;float dx,dy,amount,tol=VEHICLE_HANDOFF_ALIGN_TOL_MM;
    if(!rows || !count || !goal || !s_vehicle_handoff_valid ||
       !FieldMap_CellCenter(2U,&center) || !FieldMap_CellCenter(12U,&via) ||
       !FieldMap_CellCenter(14U,&final) || !(tol>=VEHICLE_MIN_ROUTE_CM*10 && tol<=40) ||
       !(VEHICLE_STAGE2_GOAL_OFFSET_X_MM>=-200 && VEHICLE_STAGE2_GOAL_OFFSET_X_MM<=200))return 0;
    final.x_mm+=VEHICLE_STAGE2_GOAL_OFFSET_X_MM;*goal=final;*count=0;start=s_vehicle_handoff;
    if(g_vehicle_route_phase==1U){
        /* explicit start window: the old cell-center/half-cell form left only
         * 180 mm of Y margin, so a small forward overshoot aborted the stage. */
        if(!(start.x_mm>=VEHICLE_STAGE2_ALIGN_X_MIN_MM && start.x_mm<=VEHICLE_STAGE2_ALIGN_X_MAX_MM &&
             start.y_mm>=VEHICLE_STAGE2_ALIGN_Y_MIN_MM && start.y_mm<=VEHICLE_STAGE2_ALIGN_Y_MAX_MM) ||
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

/* 真实观测端调用，必须绑定当前run/stage/item/row；不提供Watch成功开关。 */
uint32_t VehicleMission_Observe(uint32_t run,uint32_t stage,uint32_t item,uint32_t row,uint32_t kind)
{
    uint32_t accepted=0U;
    taskENTER_CRITICAL();
    if(stage>=1U && stage<=8U && row<32U && item<3U &&
       run==g_vehicle_run_id && stage==g_vehicle_stage && item==g_vehicle_item &&
       row==g_vehicle_arm_row && g_vehicle_state==3U && kind>=7U && kind<=9U &&
       vehicle_arm_rows[stage-1U][row].op==kind)
    {s_vehicle_observation=kind;accepted=1U;}
    taskEXIT_CRITICAL();return accepted;
}
static void VehicleMission_Fail(uint32_t cause)
{
    if((g_vehicle_state!=6U && g_vehicle_state!=7U) || !g_vehicle_cause)g_vehicle_cause=cause;g_vehicle_state=6U;g_stage2_ready=0U;
    if(g_vehicle_stage==2U)g_vehicle_alignment_state=3U;
    s_vehicle_queue_pending=0U;if(g_vehicle_queue_state==1U)g_vehicle_queue_state=3U;
    MotionStop();VehicleArm_Cancel();StartupAxes_Cancel();
}
static uint32_t VehicleMission_Stopped(void)
{
    ChassisAutoStatus s;PowerCommandStatus p;
    (void)ChassisAuto_ReadStatus(&s);Power_ReadCommandStatus(&p);
    return g_combined_test.state==3U && !g_combined_test.cause && !s_motion_session_open &&
        !s.active && !s.cause && ((g_stage1_motion_skipped && s.state==CHASSIS_AUTO_IDLE && !s.result) ||
        (s.result==1U && s.rotation_stop_mask==15U)) &&
        !g_forward_hold.trip && !g_heading_exec_trip && !g_vm_test_request &&
        (!p.active || StartupAxes_Holding());
}
static uint32_t VehicleMission_PathClear(uint32_t stage,uint32_t zone)
{
    static const FieldPoint starts[8]={{2250,150},{1200,300},{2160,1200},{240,1200},{1200,2160},{2160,1200},{240,1200},{1200,2160}};
    FieldPoint a,b;uint32_t i,d;
    if(stage<1U || stage>8U || zone<1U || zone>2U)return 0U;
    if(stage==1U && VEHICLE_STAGE1_BLEND_ENABLE){
        a=(FieldPoint){zone==1U?FIELD_START1_X_MM:FIELD_START2_X_MM,FIELD_START1_Y_MM};
        b=(FieldPoint){FIELD_SCAN_X_MM,FIELD_SCAN_Y_MM};
        return FieldMap_SweptBoxClear(a,b,FIELD_BODY_HALF_MM);
    }
    if(stage==2U){RouteRow rows[8];uint32_t count;return VehicleMission_Stage2Rows(rows,&count,&b);}
    a=starts[stage-1U];if(zone==2U && stage==1U)a.x_mm=150.0f;
    for(i=0U;i<8U && vehicle_drive[stage-1U][i].distance_cm>0;i++) {
        d=vehicle_drive[stage-1U][i].direction;
        if(zone==2U && ((stage==1U && i==1U) || (stage==8U && i==2U)))d=d==1U?2U:1U;
        b=a;
        if(d==0U)b.y_mm+=vehicle_drive[stage-1U][i].distance_cm*10.0f;
        else if(d==1U)b.x_mm-=vehicle_drive[stage-1U][i].distance_cm*10.0f;
        else if(d==2U)b.x_mm+=vehicle_drive[stage-1U][i].distance_cm*10.0f;
        else if(d==3U)b.y_mm-=vehicle_drive[stage-1U][i].distance_cm*10.0f;
        else return 0U;
        if(!FieldMap_AxisClear(a,b,FIELD_BODY_HALF_MM))return 0U;a=b;
    }
    return i>0U;
}
static uint32_t VehicleMission_ArmProfileReady(uint32_t stage)
{
    uint32_t i,j;float target;const VehicleArmRow *r;
    static const volatile uint32_t lift_verified=VEHICLE_LIFT_REFERENCE_VALID,carrier_verified=VEHICLE_CARRIER_REFERENCE_VALID;
    if(stage<1U || stage>8U)return 0U;
    if(!vehicle_arm_enable[stage-1U])return 1U;
    if(VEHICLE_X_ACCEL_RPM_S==0U || VEHICLE_X_ACCEL_RPM_S>65535U || vehicle_arm_repeat[stage-1U]<1U || vehicle_arm_repeat[stage-1U]>3U)return 0U;
    for(i=0U;i<32U;i++) {
        r=&vehicle_arm_rows[stage-1U][i];if(!r->op)return 1U;
        if(r->op==1U && (lift_verified!=1U || !(r->target>=0 && r->target<=VEHICLE_LIFT_MAX_TARGET_PULSES && r->speed>0 && r->speed<=VEHICLE_LIFT_RPM)))return 0U;
        if(r->op==2U && !(r->target>=0 && r->target<=180 && r->speed>0 && r->speed<=300))return 0U;
        if(r->op==3U)for(j=0U;j<vehicle_arm_repeat[stage-1U];j++) {
            target=r->target==-2.0f?vehicle_station_extension_mm[stage-1U][j]:r->target;
            if(!(target>=0 && target<=50 && r->speed>0 && r->speed<=VEHICLE_EXTENSION_RPM))return 0U;
        }
        if(r->op==4U && !(r->target>=0 && r->target<=270))return 0U;
        if(r->op==5U && (carrier_verified!=1U || !(VEHICLE_CARRIER_SLOT1_DEG>=0 && VEHICLE_CARRIER_SLOT1_DEG<=270) ||
           !(VEHICLE_CARRIER_SLOT2_DEG>=0 && VEHICLE_CARRIER_SLOT2_DEG<=270) ||
           !(VEHICLE_CARRIER_SLOT3_DEG>=0 && VEHICLE_CARRIER_SLOT3_DEG<=270)))return 0U;
        if(r->op>9U)return 0U;
    }return 0U;
}
static uint32_t VehicleMission_PublishQueued(uint32_t stage)
{
    uint32_t i;
    for(i=0;i<9;i++){s_perimeter_targets[i][0]=s_perimeter_targets[i][1]=s_perimeter_targets[i][2]=0;}
    /* 各段里程仅作本段观测，不伪造全场真实X/Y或名义终点已到。 */
    g_stage1_stop_requested=g_stage1_motion_skipped=0U;
    s_serial_start_permit=1U;g_combined_test.request=1U;g_serial_motion_state=2U;g_serial_motion_result=2U;
    g_vehicle_stage=stage;g_vehicle_state=2U;s_vehicle_stage_start=(uint32_t)xTaskGetTickCount();
    s_vehicle_queue_pending=0U;g_vehicle_queue_state=2U;return 1U;
 }
static uint32_t VehicleMission_Queue(uint32_t stage,const RouteRow *rows,uint32_t n)
{
    RouteCheck check;ChassisAutoStatus backend;
    ForwardHoldDebug forward={0};CombinedTestDebug combined={0};
    if(!n || n>8U || !RoutePlan_Convert(rows,n,&g_chassis_geometry,s_motion_actions,&check))return 0U;
    if(stage==1U && VEHICLE_STAGE1_BLEND_ENABLE && (g_vehicle_scan_search_state!=1U && g_vehicle_scan_search_state!=2U)){
        Stage1Blend plan;
        if(!Stage1Blend_Build(s_vehicle_zone==1U?1U:2U,rows[0].amount*10,rows[0].speed*10,&plan))return 0U;
        s_motion_actions[0].run_ms=(uint32_t)(plan.duration*1000)+VEHICLE_STAGE1_FINAL_WAIT_MS+2000U;
    }
    if(g_combined_test.state==3U) {
        if(!VehicleMission_Stopped())return 0U;
        /* Synchronize the already completed session before immediate search rearm. */
        ChassisAuto_SafetyUpdate(0U);
        (void)ChassisAuto_ReadStatus(&backend);
        if(!ChassisAuto_RearmCompleted(backend.session))return 0U;
        g_combined_test=combined;g_forward_hold=forward;
        g_heading_exec_phase=HR_IDLE;g_heading_exec_state=HS_IDLE;g_heading_exec_request=0U;
        HeadingSession_Init(&s_heading_exec);
        s_motion_has_action=s_combined_turn=s_combined_powered_segment=s_combined_keep_vm=0U;
    }
    s_motion_timing_count=s_motion_timing_cursor=0U;
    s_motion_action_count=n;g_debug_plan_id=DEBUG_PLAN_DEFAULT;g_route_check=check;
    g_motion_plan_valid=App_MotionPlanValid();if(!g_motion_plan_valid)return 0U;
    {uint32_t mask=App_SerialIdleBlocked((uint32_t)xTaskGetTickCount());
     g_vehicle_queue_stage=stage;g_vehicle_queue_first_mask=g_vehicle_queue_last_mask=mask;
     g_vehicle_queue_wait_ms=0U;g_vehicle_queue_attempts=1U;
     if(mask){
        if(mask&~(32U|256U)){g_vehicle_queue_state=3U;return 0U;}
        if(VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS<100U || VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS>2000U)return 0U;
        s_vehicle_queue_pending=stage;s_vehicle_queue_begin=(uint32_t)xTaskGetTickCount();g_vehicle_queue_state=1U;return 1U;
     }}
    return VehicleMission_PublishQueued(stage);
}
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
static uint32_t VehicleMission_Load(uint32_t stage)
{
    RouteRow rows[8];uint32_t n,direction;
    if(stage==2U){
        uint32_t queued=VehicleMission_QueueNextStage2();
        if(queued==2U){g_vehicle_stage=2U;g_vehicle_state=2U;}
        return queued!=0U;
    }
    if(stage<1U || stage>8U || !VehicleMission_PathClear(stage,s_vehicle_zone))return 0U;
    if(stage==1U && VEHICLE_STAGE1_BLEND_ENABLE){
        rows[0]=(RouteRow){s_vehicle_zone==1U?MOTION_LEFT_DISTANCE:MOTION_RIGHT_DISTANCE,
            fabsf((s_vehicle_zone==1U?FIELD_START1_X_MM:FIELD_START2_X_MM)-FIELD_SCAN_X_MM)/10.0f,VEHICLE_STAGE1_LATERAL_CM_S};n=1U;
    }else if(stage==2U){
        if(!VehicleMission_Stage2Rows(rows,&n,&s_vehicle_stage2_goal))return 0U;
    }else
    for(n=0;n<8U && vehicle_drive[stage-1U][n].distance_cm>0;n++) {
        direction=vehicle_drive[stage-1U][n].direction;
        if(direction>3)return 0U;
        if(s_vehicle_zone==2U && ((stage==1U && n==1U) || (stage==8U && n==2U)))
            direction=direction==1U?2U:1U;
        rows[n].kind=MOTION_FORWARD_DISTANCE+direction;
        rows[n].amount=vehicle_drive[stage-1U][n].distance_cm;
        rows[n].speed=vehicle_drive[stage-1U][n].speed_cm_s;
    }
    return VehicleMission_Queue(stage,rows,n);
}
static uint32_t VehicleMission_Search(uint32_t step)
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

static void VehicleMission_Advance(void)
{
    g_vehicle_stage_mask|=1UL<<(g_vehicle_stage-1U);
#if !VEHICLE_RUN_ALL
    if(VEHICLE_CHAIN_STAGE12 && g_vehicle_stage==1U){if(!VehicleMission_Load(2U))VehicleMission_Fail(7U);return;}
    g_vehicle_state=4U;return;
#else
    if(g_vehicle_stage==8U){g_vehicle_state=5U;return;}
    if(!VehicleMission_Load(g_vehicle_stage+1U))VehicleMission_Fail(7U);
#endif
}
static uint32_t VehicleMission_ResetCompleted(void)
{
    ChassisAutoStatus backend;ForwardHoldDebug forward={0};CombinedTestDebug combined={0};
    if(!VehicleMission_Stopped())return 0U;ChassisAuto_SafetyUpdate(0U);(void)ChassisAuto_ReadStatus(&backend);
    if(!ChassisAuto_RearmCompleted(backend.session))return 0U;
    g_combined_test=combined;g_forward_hold=forward;s_motion_has_action=s_motion_session_open=0U;
    g_heading_exec_phase=HR_IDLE;g_heading_exec_state=HS_IDLE;g_heading_exec_request=0U;
    HeadingSession_Init(&s_heading_exec);g_serial_motion_state=g_serial_motion_result=0U;g_vehicle_state=0U;return 1U;
}
static void VehicleMission_Service(void)
{
    uint32_t now=(uint32_t)xTaskGetTickCount(),key;const VehicleArmRow *row;
    taskENTER_CRITICAL();key=g_vehicle_key;g_vehicle_key=0U;taskEXIT_CRITICAL();
    /* 机构单项调试：伸缩回零或4310启动；完成后不进入扫码/路线/库存。 */
    if(VEHICLE_EXTENSION_HOME_ONLY || VEHICLE_DM_STARTUP_ONLY || VEHICLE_STARTUP_GROUP_ONLY){
        if(key){
            if(g_vehicle_state==0U){
                if(!g_safety_ready || g_rtos_fault || g_safety_fault || g_can_ctrl_fault_latched){g_vehicle_cause=1U;return;}
                if(!StartupAxes_Begin()){g_vehicle_state=6U;g_vehicle_cause=3U;return;}
                g_vehicle_run_id++;g_vehicle_stage=1U;g_vehicle_state=1U;g_vehicle_cause=0U;
            }else{StartupAxes_Cancel();g_vehicle_state=7U;g_vehicle_cause=2U;return;}
        }
        if(g_vehicle_state==1U && g_startup_axes_state==4U)g_vehicle_state=4U;
        if((g_vehicle_state==1U || g_vehicle_state==4U) &&
           (g_rtos_fault || g_safety_fault || g_can_ctrl_fault_latched || g_startup_axes_state>=5U)){
            if(g_startup_axes_state<5U)StartupAxes_Cancel();
            g_vehicle_state=6U;g_vehicle_cause=4U;
        }
    }else{
    if(key && VEHICLE_SOURCE_PREP_ENABLE && g_source_prepare_state==3U &&
       (g_vehicle_state==4U || g_vehicle_state==5U)){g_vehicle_cause=19U;return;}
    if(key && (g_vehicle_state==4U || g_vehicle_state==5U) && !VehicleMission_ResetCompleted()){g_vehicle_cause=7U;return;}
    if(key && g_vehicle_state!=0U && g_vehicle_state!=4U && g_vehicle_state!=5U) {
        VehicleMission_Fail(2U);g_vehicle_state=7U;return;
    }
    if(key && g_vehicle_state==0U) {
        if(VEHICLE_TEST_STAGE<1 || VEHICLE_TEST_STAGE>8 || (!VEHICLE_TEST_START_ZONE && !g_radar_zone_ready) ||
           !App_SerialIdleReady(now)){g_vehicle_cause=1U;return;}
        s_vehicle_zone=VEHICLE_TEST_START_ZONE?VEHICLE_TEST_START_ZONE:RadarZoneSwitch_Lock();if(s_vehicle_zone<1 || s_vehicle_zone>2)return;
        g_vehicle_stage=VEHICLE_RUN_ALL?1U:VEHICLE_TEST_STAGE;
        RouteHeading_Reset(&s_route_heading_reference);
        g_vehicle_run_id++;if(!g_vehicle_run_id)g_vehicle_run_id=1U;
        s_vehicle_start=s_vehicle_stage_start=now;g_vehicle_stage_mask=0U;
        s_vehicle_queue_pending=s_vehicle_queue_begin=0U;
        g_vehicle_queue_state=g_vehicle_queue_first_mask=g_vehicle_queue_last_mask=0U;
        g_vehicle_queue_wait_ms=g_vehicle_queue_attempts=g_vehicle_queue_stage=0U;
        s_vehicle_encoder_based=s_vehicle_handoff_valid=s_vehicle_handoff_session=0U;
        g_vehicle_route_phase=0U;s_vehicle_stage2_leg_target=(FieldPoint){0,0};
        s_vehicle_handoff=(FieldPoint){VEHICLE_STAGE2_START_X_MM,VEHICLE_STAGE2_START_Y_MM};
        g_vehicle_scan_bypassed=VEHICLE_SCAN_BYPASS;s_vehicle_code_valid=s_vehicle_display_done=0U;
        g_vehicle_scan_search_state=s_vehicle_search_begin=s_vehicle_search_used=0U;
        g_vehicle_alignment_state=0U;
        /* 每次启动重新打开扫码（上一轮锁码后是关着的） */
        SAOMA_EnableReceive();
        s_vehicle_scan_baseline=g_saoma_accept_count;memset(s_vehicle_task_code,0,16);
        if(!StartupAxes_Holding() && !StartupAxes_Begin()){VehicleMission_Fail(3U);return;}g_vehicle_state=1U;
    }
    if(g_vehicle_state>=1U && g_vehicle_state<=3U) {
        if(g_rtos_fault || g_safety_fault || g_can_ctrl_fault_latched || g_startup_axes_state>=5U)
        {VehicleMission_Fail(4U);return;}
        if(now-s_vehicle_stage_start>=VEHICLE_STAGE_TIMEOUT_MS ||
           (VEHICLE_REHEARSAL && now-s_vehicle_start>=VEHICLE_REHEARSAL_MS))
        {VehicleMission_Fail(5U);return;}
    }
    if(s_vehicle_queue_pending){
        g_vehicle_queue_wait_ms=now-s_vehicle_queue_begin;
        if(g_vehicle_queue_wait_ms>=VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS){g_vehicle_queue_state=3U;VehicleMission_Fail(7U);return;}
        g_vehicle_queue_attempts++;g_vehicle_queue_last_mask=App_SerialIdleBlocked(now);
        if(g_vehicle_queue_last_mask&~(32U|256U)){g_vehicle_queue_state=3U;VehicleMission_Fail(7U);return;}
        if(!g_vehicle_queue_last_mask)(void)VehicleMission_PublishQueued(s_vehicle_queue_pending);
        return;
    }
        if((g_vehicle_state==1U || g_vehicle_state==2U) && g_vehicle_stage==1U && !g_vehicle_scan_bypassed && !s_vehicle_code_valid &&
           g_saoma_accept_count!=s_vehicle_scan_baseline && g_saoma_last_result==1U &&
           (uint32_t)(g_saoma_frame_start_ms-s_vehicle_start)<0x80000000U &&
           (uint32_t)(now-g_saoma_frame_end_ms)<VEHICLE_STAGE1_SCAN_FRESH_MS && FieldMap_TaskCodeValid(SAOMA_Data)) {
            /* 本轮只锁首个合法码；当前名义/搜索动作走完，再停车交接。 */
            memcpy(s_vehicle_task_code,SAOMA_Data,16);s_vehicle_code_valid=1U;
            /* 只认第一个合法码：立刻关接收，后面再来码也不处理、不重复上屏 */
            SAOMA_DisableReceive();
            /* 读到就显示，不等车停到名义扫码点 */
            HWT101_Update();App_HeadingSourcePublish();
            g_scan_display_result=Screen_ShowTaskCode(s_vehicle_task_code);
            HWT101_Update();App_HeadingSourcePublish();
            s_vehicle_display_done=1U;
            if(g_scan_display_result==SCREEN_OK){g_scan_display_count++;g_vehicle_scan_search_state=4U;}

        }
    if(g_vehicle_state==1U && StartupAxes_Holding()) {
        if(!VehicleMission_Load(VEHICLE_RUN_ALL?1U:VEHICLE_TEST_STAGE)){VehicleMission_Fail(7U);return;}

    }
    if(g_vehicle_state==2U && g_vehicle_stage==1U && g_vehicle_scan_search_state>=1U &&
       g_vehicle_scan_search_state<=2U && now-s_vehicle_search_begin>=VEHICLE_SCAN_SEARCH_TOTAL_MS){VehicleMission_Fail(18U);return;}
    if(g_vehicle_state==2U) {
        if(g_combined_test.state>=4U){VehicleMission_Fail(8U);return;}
        if(VehicleMission_Stopped()) {
            if(g_vehicle_stage==1U && !VehicleMission_CaptureHandoff()){VehicleMission_Fail(10U);return;}
            if(g_vehicle_stage==1U && !g_vehicle_scan_bypassed) {
                if(!s_vehicle_code_valid){
                    if(!g_vehicle_scan_search_state){s_vehicle_search_begin=now;if(!VehicleMission_Search(1U))VehicleMission_Fail(17U);}
                    else if(g_vehicle_scan_search_state==1U){if(!VehicleMission_Search(2U))VehicleMission_Fail(17U);}
                    else {g_vehicle_scan_search_state=3U;VehicleMission_Fail(16U);}
                    return;
                }
                /* 码在捕获时就已经上屏，这里不再等屏幕，直接进第二段。 */
                /* 提前扫码会改变下一段起点；未接真实位姿重规划时禁止沿用名义路线。 */
                if(VEHICLE_RUN_ALL && !VEHICLE_CHAIN_STAGE12 && (s_vehicle_search_used || g_combined_test.completed!=MOTION_ACTION_COUNT))
                {VehicleMission_Fail(10U);return;}
            }
            if(g_vehicle_stage==2U){
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
#if VEHICLE_SOURCE_PREP_ENABLE
                {
                    g_vehicle_alignment_state=0U;
                    if(!VehicleArm_BeginSourcePreparation()){VehicleMission_Fail(13U);return;}
                    g_vehicle_state=3U;return;
                }
#else
                /* Route DONE is distinct from visual alignment DONE. No synthetic correction. */
                if(!g_vehicle_alignment_state)g_vehicle_alignment_state=1U;
                if(!VEHICLE_STAGE2_ROUTE_ONLY)return;
#endif
            }
            if(vehicle_arm_enable[g_vehicle_stage-1U]) {
                if(!VehicleMission_ArmProfileReady(g_vehicle_stage)){VehicleMission_Fail(15U);return;}
                g_vehicle_arm_row=g_vehicle_item=0U;s_vehicle_observation=0U;g_vehicle_state=3U;
            } else VehicleMission_Advance();
        }
    }
    if(g_vehicle_state==3U && g_vehicle_stage==2U && VEHICLE_SOURCE_PREP_ENABLE){
        if(g_vehicle_arm_state==3U || g_source_prepare_state==4U){VehicleMission_Fail(12U);return;}
        if(g_source_prepare_state==3U && g_vehicle_arm_state==2U){g_vehicle_alignment_state=1U;VehicleMission_Advance();}
        return;
    }
    if(g_vehicle_state==3U) {
        if(g_vehicle_arm_row>=32U || g_vehicle_item>=3U){VehicleMission_Fail(11U);return;}
        row=&vehicle_arm_rows[g_vehicle_stage-1U][g_vehicle_arm_row];
        if(!row->op) {
            if(++g_vehicle_item<vehicle_arm_repeat[g_vehicle_stage-1U])g_vehicle_arm_row=0U;
            else VehicleMission_Advance();return;
        }
        if(row->op>=7U) {
            if(VEHICLE_MATERIAL_OBSERVATION_REQUIRED && s_vehicle_observation!=row->op)return;
            /* 没有真实观测不记正确次数；执行器DONE不能替代持料/落稳。 */
            taskENTER_CRITICAL();s_vehicle_observation=0U;g_vehicle_arm_row++;taskEXIT_CRITICAL();return;
        }
        if(g_vehicle_arm_state==3U){VehicleMission_Fail(12U);return;}
        if(g_vehicle_arm_state==2U){g_vehicle_arm_state=0U;g_vehicle_arm_row++;return;}
        if(!VehicleArm_Busy()) {
            VehicleArmRow resolved=*row;
            if(resolved.op==3U && resolved.target==-2.0f)resolved.target=vehicle_station_extension_mm[g_vehicle_stage-1U][g_vehicle_item];
            if(!VehicleArm_Begin(&resolved,g_vehicle_item))VehicleMission_Fail(13U);
        }
    }
    }
}

/* Read-only future vision request: no acceptance/completion or motor writes. */
uint32_t VehicleMission_ReadAlignmentRequest(VehicleAlignmentRequest *request)
{
    uint32_t valid=0U;
    if(!request)return 0U;
    taskENTER_CRITICAL();
    if(g_vehicle_stage==2U && g_vehicle_alignment_state==1U &&
       (g_vehicle_state==2U || g_vehicle_state==4U) &&
       VehicleMission_Stopped() && StartupAxes_Holding() &&
       !g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched){
        request->run_id=g_vehicle_run_id;request->stage=2U;request->start_id=s_vehicle_zone;
        request->layout_version=VEHICLE_MAP_VERSION;request->goal_cell=VEHICLE_STAGE2_GOAL_CELL;
        request->nominal_target_x_mm=s_vehicle_stage2_goal.x_mm;
        request->nominal_target_y_mm=s_vehicle_stage2_goal.y_mm;
        request->position_executor_enabled=VEHICLE_ALIGNMENT_POSITION_EXECUTOR_ENABLE;
        valid=1U;
    }
    taskEXIT_CRITICAL();return valid;
}

static int32_t VehicleMission_MilliDeg(float value)
{
    if(!(value>=-2000000.0f && value<=2000000.0f))return (-2147483647L-1L);
    return (int32_t)(value*1000.0f);
}
static void VehicleMission_Telemetry(void)
{
    static uint32_t last,slot,sequence[4];static char frame[256];int count;uint16_t crc;
    uint32_t now=(uint32_t)xTaskGetTickCount(),frame_seq,kind;
    if(now-last<500U || !App_SerialDiagnosticWindow(now,20U))return;last=now;
    /* The debug TX mailbox accepts one frame at a time. Rotate frames instead
     * of dropping the second diagnostic while VM1 is still transmitting. */
    kind=slot++%(g_vehicle_stage==2U?4U:2U);frame_seq=sequence[kind]++;
    if(kind==0U){
        count=snprintf(frame,sizeof(frame),"VM2,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%ld,%ld,%lu,%lu,%lu,%lu,%lu,%lu,",
        (unsigned long)frame_seq,(unsigned long)now,(unsigned long)g_vehicle_run_id,
        (unsigned long)g_vehicle_stage,(unsigned long)g_vehicle_state,(unsigned long)g_vehicle_cause,
        (unsigned long)g_startup_axes_state,(unsigned long)g_startup_axes_cause,
        (unsigned long)g_vehicle_arm_row,(unsigned long)g_vehicle_item,(unsigned long)g_vehicle_arm_state,
        (unsigned long)g_vehicle_arm_cause,(unsigned long)(g_startup_extension_feedback_valid|(g_vehicle_lift_feedback_valid<<1)),
        (long)VehicleMission_MilliDeg(g_startup_extension_current_deg),(long)VehicleMission_MilliDeg(g_vehicle_lift_current_deg),
        (unsigned long)g_vehicle_queue_state,(unsigned long)g_vehicle_queue_stage,
        (unsigned long)g_vehicle_queue_first_mask,(unsigned long)g_vehicle_queue_last_mask,
        (unsigned long)g_vehicle_queue_wait_ms,(unsigned long)g_vehicle_queue_attempts);
    }else if(kind==1U){
        count=snprintf(frame,sizeof(frame),"EX3,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
        (unsigned long)frame_seq,(unsigned long)now,(unsigned long)g_vehicle_run_id,
        (unsigned long)g_eh_first_reason,(unsigned long)g_eh_first_command,(unsigned long)g_eh_first_exchange,
        (unsigned long)g_eh_first_flags,(unsigned long)g_eh_first_service_gap,(unsigned long)g_eh_first_phase,
        (unsigned long)g_eh_first_tick,(unsigned long)g_extension_hold_enable_count,
        (unsigned long)g_extension_hold_samples,(unsigned long)g_extension_hold_confirmed,
        (unsigned long)g_extension_hold_ready_tick,(unsigned long)g_extension_hold_delta_mdeg,
        (unsigned long)g_eh_first_step,(unsigned long)g_eh_speed_sign,(unsigned long)g_eh_speed_rpm);
    }else if(kind==3U){
        count=snprintf(frame,sizeof(frame),"SP1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%ld,%ld,",
            (unsigned long)frame_seq,(unsigned long)now,(unsigned long)g_vehicle_run_id,
            (unsigned long)g_source_prepare_state,(unsigned long)g_source_prepare_cause,
            (unsigned long)g_source_prepare_done_mask,(unsigned long)g_source_prepare_release_ms,
            (unsigned long)g_source_prepare_elapsed_ms,(unsigned long)g_source_prepare_segment,
            (long)VehicleMission_MilliDeg(g_source_prepare_extension_mm),(long)VehicleMission_MilliDeg(g_source_prepare_grip_deg));
    }else{
        count=snprintf(frame,sizeof(frame),"VG4,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
            (unsigned long)frame_seq,(unsigned long)now,(unsigned long)g_vehicle_run_id,
            (unsigned long)VEHICLE_MAP_VERSION,(unsigned long)GRID_ROUTE_SOURCE_MANUAL,
            (unsigned long)VEHICLE_STAGE2_START_CELL,(unsigned long)VEHICLE_STAGE2_GOAL_CELL,
            (unsigned long)sizeof(vehicle_stage2_waypoints),(unsigned long)g_combined_test.index,
            (unsigned long)g_vehicle_alignment_state,(unsigned long)VEHICLE_ALIGNMENT_POSITION_EXECUTOR_ENABLE,
            (unsigned long)s_vehicle_stage2_goal.x_mm,(unsigned long)s_vehicle_stage2_goal.y_mm,
            (unsigned long)VehicleMission_MilliDeg(s_vehicle_handoff.x_mm),
            (unsigned long)VehicleMission_MilliDeg(s_vehicle_handoff.y_mm),(unsigned long)s_vehicle_handoff_valid,
            (unsigned long)g_vehicle_route_phase,(unsigned long)s_vehicle_stage2_leg_target.x_mm,
            (unsigned long)s_vehicle_stage2_leg_target.y_mm);
    }
    if(count<0 || (uint32_t)count>=sizeof(frame)-7U)return;
    crc=App_SerialCrc(frame,(uint32_t)count);
    (void)snprintf(frame+count,sizeof(frame)-(uint32_t)count,"%04X\r\n",crc);
    (void)App_DebugSendString(frame);
}
