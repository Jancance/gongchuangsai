/* 整车八阶段源码调参入口；按键只提交启动/取消，禁止Watch/无线修改业务参数。 */
volatile uint32_t g_vehicle_stage=VEHICLE_TEST_STAGE,g_vehicle_state=0U,g_vehicle_cause=0U;
volatile uint32_t g_vehicle_key=0U,g_vehicle_arm_row=0U,g_vehicle_item=0U,g_vehicle_stage_mask=0U;
volatile uint32_t g_vehicle_run_id=0U,g_vehicle_scan_bypassed=0U;
volatile uint32_t g_vehicle_correct_pick_count=0U,g_vehicle_correct_place_count=0U;
static uint32_t s_vehicle_start,s_vehicle_stage_start,s_vehicle_zone,s_vehicle_observation;
static uint32_t s_vehicle_scan_baseline,s_vehicle_code_valid,s_vehicle_display_done;
static char s_vehicle_task_code[16];
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
static uint32_t VehicleMission_Load(uint32_t stage)
{
    RouteRow rows[8];RouteCheck check;ChassisAutoStatus backend;
    ForwardHoldDebug forward={0};CombinedTestDebug combined={0};uint32_t n,i,direction;
    if(stage<1U || stage>8U || !VehicleMission_PathClear(stage,s_vehicle_zone))return 0U;
    for(n=0;n<8U && vehicle_drive[stage-1U][n].distance_cm>0;n++) {
        direction=vehicle_drive[stage-1U][n].direction;
        if(direction>3)return 0U;
        if(s_vehicle_zone==2U && ((stage==1U && n==1U) || (stage==8U && n==2U)))
            direction=direction==1U?2U:1U;
        rows[n].kind=MOTION_FORWARD_DISTANCE+direction;
        rows[n].amount=vehicle_drive[stage-1U][n].distance_cm;
        rows[n].speed=vehicle_drive[stage-1U][n].speed_cm_s;
    }
    if(!n || n>8U || !RoutePlan_Convert(rows,n,&g_chassis_geometry,s_motion_actions,&check))return 0U;
    if(g_combined_test.state==3U) {
        if(!VehicleMission_Stopped())return 0U;
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
    if(!App_SerialIdleReady((uint32_t)xTaskGetTickCount()))return 0U;
    for(i=0;i<9;i++){s_perimeter_targets[i][0]=s_perimeter_targets[i][1]=s_perimeter_targets[i][2]=0;}
    /* 各段里程仅作本段观测，不伪造全场真实X/Y或名义终点已到。 */
    g_stage1_stop_requested=g_stage1_motion_skipped=0U;
    s_serial_start_permit=1U;g_combined_test.request=1U;g_serial_motion_state=2U;g_serial_motion_result=2U;
    g_vehicle_stage=stage;g_vehicle_state=2U;s_vehicle_stage_start=(uint32_t)xTaskGetTickCount();
    return 1U;
}
static void VehicleMission_Advance(void)
{
    g_vehicle_stage_mask|=1UL<<(g_vehicle_stage-1U);
#if !VEHICLE_RUN_ALL
    g_vehicle_state=4U;return;
#else
    if(g_vehicle_stage==8U){g_vehicle_state=5U;return;}
    if(!VehicleMission_Load(g_vehicle_stage+1U))VehicleMission_Fail(7U);
#endif
}
static uint32_t VehicleMission_ResetCompleted(void)
{
    ChassisAutoStatus backend;ForwardHoldDebug forward={0};CombinedTestDebug combined={0};
    if(!VehicleMission_Stopped())return 0U;(void)ChassisAuto_ReadStatus(&backend);
    if(!ChassisAuto_RearmCompleted(backend.session))return 0U;
    g_combined_test=combined;g_forward_hold=forward;s_motion_has_action=s_motion_session_open=0U;
    g_heading_exec_phase=HR_IDLE;g_heading_exec_state=HS_IDLE;g_heading_exec_request=0U;
    HeadingSession_Init(&s_heading_exec);g_serial_motion_state=g_serial_motion_result=0U;g_vehicle_state=0U;return 1U;
}
static void VehicleMission_Service(void)
{
    uint32_t now=(uint32_t)xTaskGetTickCount(),key;const VehicleArmRow *row;
    taskENTER_CRITICAL();key=g_vehicle_key;g_vehicle_key=0U;taskEXIT_CRITICAL();
    /* 用户当前只调伸缩回零；不进入扫码、路线、机械臂或库存状态。 */
    if(VEHICLE_EXTENSION_HOME_ONLY){
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
    if(key && (g_vehicle_state==4U || g_vehicle_state==5U) && !VehicleMission_ResetCompleted()){g_vehicle_cause=7U;return;}
    if(key && g_vehicle_state!=0U && g_vehicle_state!=4U && g_vehicle_state!=5U) {
        VehicleMission_Fail(2U);g_vehicle_state=7U;return;
    }
    if(key && g_vehicle_state==0U) {
        if(VEHICLE_TEST_STAGE<1 || VEHICLE_TEST_STAGE>8 || !g_radar_zone_ready ||
           !App_SerialIdleReady(now)){g_vehicle_cause=1U;return;}
        s_vehicle_zone=RadarZoneSwitch_Lock();if(s_vehicle_zone<1 || s_vehicle_zone>2)return;
        g_vehicle_stage=VEHICLE_RUN_ALL?1U:VEHICLE_TEST_STAGE;
        g_vehicle_run_id++;if(!g_vehicle_run_id)g_vehicle_run_id=1U;
        s_vehicle_start=s_vehicle_stage_start=now;g_vehicle_stage_mask=0U;
        g_vehicle_scan_bypassed=VEHICLE_SCAN_BYPASS;s_vehicle_code_valid=s_vehicle_display_done=0U;
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
        if((g_vehicle_state==1U || g_vehicle_state==2U) && g_vehicle_stage==1U && !g_vehicle_scan_bypassed && !s_vehicle_code_valid &&
           g_saoma_accept_count!=s_vehicle_scan_baseline && g_saoma_last_result==1U &&
           (uint32_t)(g_saoma_frame_start_ms-s_vehicle_start)<0x80000000U &&
           (uint32_t)(now-g_saoma_frame_end_ms)<VEHICLE_STAGE1_SCAN_FRESH_MS && FieldMap_TaskCodeValid(SAOMA_Data)) {
            memcpy(s_vehicle_task_code,SAOMA_Data,16);s_vehicle_code_valid=1U;
            if(g_vehicle_state==2U)g_stage1_stop_requested=1U;
        }
    if(g_vehicle_state==1U && StartupAxes_Holding()) {
        if(!VehicleMission_Load(VEHICLE_RUN_ALL?1U:VEHICLE_TEST_STAGE)){VehicleMission_Fail(7U);return;}
        if(g_vehicle_stage==1U && s_vehicle_code_valid && !g_vehicle_scan_bypassed)g_stage1_stop_requested=1U;
    }
    if(g_vehicle_state==2U) {
        if(g_combined_test.state>=4U){VehicleMission_Fail(8U);return;}
        if(VehicleMission_Stopped()) {
            if(g_vehicle_stage==1U && !g_vehicle_scan_bypassed) {
                if(!s_vehicle_code_valid)return;
                if(!s_vehicle_display_done) {
                    g_scan_display_result=Screen_ShowTaskCode(s_vehicle_task_code);
                    if(g_scan_display_result!=SCREEN_OK){VehicleMission_Fail(9U);return;}
                    s_vehicle_display_done=1U;g_scan_display_count++;
                }
                /* 提前扫码会改变下一段起点；未接真实位姿重规划时禁止沿用名义路线。 */
                if(VEHICLE_RUN_ALL && g_combined_test.completed!=MOTION_ACTION_COUNT)
                {VehicleMission_Fail(10U);return;}
            }
            if(vehicle_arm_enable[g_vehicle_stage-1U]) {
                if(!VehicleMission_ArmProfileReady(g_vehicle_stage)){VehicleMission_Fail(15U);return;}
                g_vehicle_arm_row=g_vehicle_item=0U;s_vehicle_observation=0U;g_vehicle_state=3U;
            } else VehicleMission_Advance();
        }
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

static int32_t VehicleMission_MilliDeg(float value)
{
    if(!(value>=-2000000.0f && value<=2000000.0f))return (-2147483647L-1L);
    return (int32_t)(value*1000.0f);
}
static void VehicleMission_Telemetry(void)
{
    static uint32_t last,sequence;static char frame[192];int count;uint16_t crc;
    uint32_t now=(uint32_t)xTaskGetTickCount();
    if(now-last<500U || !App_SerialDiagnosticWindow(now,20U))return;last=now;
    count=snprintf(frame,sizeof(frame),"VM1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%ld,%ld,",
        (unsigned long)sequence++,(unsigned long)now,(unsigned long)g_vehicle_run_id,
        (unsigned long)g_vehicle_stage,(unsigned long)g_vehicle_state,(unsigned long)g_vehicle_cause,
        (unsigned long)g_startup_axes_state,(unsigned long)g_startup_axes_cause,
        (unsigned long)g_vehicle_arm_row,(unsigned long)g_vehicle_item,(unsigned long)g_vehicle_arm_state,
        (unsigned long)g_vehicle_arm_cause,(unsigned long)(g_startup_extension_feedback_valid|(g_vehicle_lift_feedback_valid<<1)),
        (long)VehicleMission_MilliDeg(g_startup_extension_current_deg),(long)VehicleMission_MilliDeg(g_vehicle_lift_current_deg));
    if(count<0 || (uint32_t)count>=sizeof(frame)-7U)return;
    crc=App_SerialCrc(frame,(uint32_t)count);(void)snprintf(frame+count,sizeof(frame)-(uint32_t)count,"%04X\r\n",crc);
    (void)App_DebugSendString(frame);
}
