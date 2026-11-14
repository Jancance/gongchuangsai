/* Fixture only: raw reference/common-mean inputs, not a fitted vehicle plant. */
volatile uint32_t distance_control_trace[32][12];
static void distance_inputs(uint32_t now,float mean)
{
    uint32_t w;float wheel_mm=mean/CHASSIS_FORWARD_DISTANCE_SCALE;
    clock_ms=now;s_position_valid=15U;s_endpoint_index=0U;s_endpoint_count=1U;
    s_endpoints[0].start_mask=15U;
    for(w=0;w<4;w++) {
        s_position_tick[w]=now;s_distance_seen_ms[w]=now;
        s_position_history_count[w]=2U;s_position_history_head[w]=2U;
        s_position_history[w][0].ms=now-100U;s_position_history[w][0].mm=wheel_mm;
        s_position_history[w][1].ms=now;s_position_history[w][1].mm=wheel_mm;
    }
}
static uint32_t observe_distance_control(void)
{
    uint32_t k;MotionProfileSample sample;
    CHECK(entry_start());s_run_tick=1000U;s_profile_follow_tick=1000U;
    s_run_started=1U;g_chassis_auto_state=CHASSIS_AUTO_RUN;
    s_stop_due=s_finish_requested=0U;
    for(k=0;k<32;k++) {
        float elapsed=.25f*k,mean;uint32_t now=1000U+250U*k;
        CHECK(MotionProfile_Sample(&s_distance_profile,elapsed,&sample));
        mean=sample.position_mm+(k%3U==0U ? 1.0f : -1.0f);
        if(mean<0)mean=0;distance_inputs(now,mean);
        Auto_DistanceCheckLocked(now);
        distance_control_trace[k][0]=s_translation_base_rpm;
        distance_control_trace[k][1]=s_translation_applied_base;
        distance_control_trace[k][2]=s_distance_reached;
        distance_control_trace[k][3]=s_stop_due;
        distance_control_trace[k][4]=s_finish_requested;
        distance_control_trace[k][5]=s_distance_fault_reason;
        memcpy((void*)&distance_control_trace[k][6],&s_distance_progress_mm,4);
        memcpy((void*)&distance_control_trace[k][7],&s_distance_reference.position_mm,4);
        memcpy((void*)&distance_control_trace[k][8],&s_distance_reference.speed_mm_s,4);
        memcpy((void*)&distance_control_trace[k][9],&s_profile_follow_rpm,4);
        memcpy((void*)&distance_control_trace[k][10],&s_profile_follow_rate,4);
        distance_control_trace[k][11]=s_profile_follow_tick;
        CHECK(!s_distance_fault_reason && g_chassis_auto_active);
    }
    cases++;return 0U;
}

#ifdef CHASSIS_TAIL_PROFILE_ACTIVE
volatile uint8_t snapshot_observation_blob[176];
static uint32_t check_tail_snapshots(void)
{
    ChassisAutoTailSnapshots a,saved;ChassisAutoPreparedTranslation p,bad;
    uint32_t last_follow_tick,old_freezes;float last_follow,last_rate;
    CHECK(sizeof(a)==176U && sizeof(a.slot[0])==48U);
    CHECK((uint32_t)((char*)&a.slot-(char*)&a)==32U);
    CHECK((uint32_t)((char*)&a.slot[0].common_mean_mm-(char*)&a.slot[0])==16U);
    CHECK((uint32_t)((char*)&a.slot[0].pending_base_rpm-(char*)&a.slot[0])==36U);
    CHECK((uint32_t)((char*)&a.slot[0].flags-(char*)&a.slot[0])==44U);
    CHECK(entry_start());ChassisAuto_ReadTailSnapshots(&a);
    CHECK(a.session==g_chassis_auto_session && a.format_version==1U && !a.valid_mask);
    s_run_tick=1000U;s_profile_follow_tick=8200U;s_run_started=1U;
    s_profile_follow_rpm=-1.0f;s_profile_follow_rate=.1f;
    g_chassis_auto_state=CHASSIS_AUTO_RUN;s_stop_due=s_finish_requested=0U;
    s_distance_progress_mm=1975.0f;distance_inputs(8300U,1985.0f);
    Auto_DistanceCheckLocked(8300U);ChassisAuto_ReadTailSnapshots(&a);
    CHECK(a.valid_mask==1U && a.entry_count==1U && a.slot[0].now_ms==8300U);
    CHECK(a.slot[0].follow_tick_ms==8200U && a.slot[0].follow_rpm==-1.0f);
    CHECK(a.slot[0].common_ms==8300U && a.slot[0].common_mean_mm>1984.9f);
    saved=a;distance_inputs(8310U,1986.0f);Auto_DistanceCheckLocked(8310U);
    CHECK(memcmp((const void*)&g_chassis_auto_tail_snapshots.slot[0],&saved.slot[0],48)==0);
    /* A genuine accepted command freeze; ACK completion is not claimed. */
    input_od.pose.valid=0U;s_lateral_vy=0;s_lateral_applied_deci=0U;
    s_translation_applied_base=99U;s_heading_applied_rpm=s_heading_pending_rpm=0;
    s_monitor_between=s_motor_index=0;g_chassis_auto_state=CHASSIS_AUTO_RUN;
    clock_ms=8311U;ChassisAuto_CanStep();ChassisAuto_ReadTailSnapshots(&a);
    CHECK(a.valid_mask==3U && a.freeze_count==1U);
    CHECK(a.slot[1].now_ms==8311U && a.slot[1].applied_base_rpm==a.slot[1].pending_base_rpm);
    CHECK(g_chassis_auto_state==CHASSIS_AUTO_START && s_motor_index==1U);
    old_freezes=a.freeze_count;clock_ms++;ChassisAuto_CanStep();
    CHECK(g_chassis_auto_tail_snapshots.freeze_count==old_freezes);
    /* Unchanged requested/applied batch cannot generate a freeze record. */
    g_chassis_auto_state=CHASSIS_AUTO_RUN;s_motor_index=s_monitor_between=0U;
    clock_ms++;ChassisAuto_CanStep();CHECK(g_chassis_auto_tail_snapshots.freeze_count==old_freezes);
    last_follow=s_profile_follow_rpm;last_rate=s_profile_follow_rate;last_follow_tick=s_profile_follow_tick;
    distance_inputs(9050U,2104.0f);Auto_DistanceCheckLocked(9050U);ChassisAuto_ReadTailSnapshots(&a);
    CHECK(a.valid_mask==7U && a.arrival_count==1U && a.slot[2].now_ms==9050U);
    CHECK(a.slot[2].reference_rpm>14.0f && a.slot[2].reference_rpm<16.0f);
    CHECK(a.slot[2].common_mean_mm>2103.9f && a.slot[2].run_tick_ms==1000U);
    CHECK(a.slot[2].follow_rpm==last_follow && a.slot[2].follow_rate_rpm_s==last_rate);
    CHECK(a.slot[2].follow_tick_ms==last_follow_tick);
    CHECK((a.slot[2].flags&(8U|16U|32U))==0U);
    CHECK((a.slot[2].flags&(1U|4U|64U|128U))==(1U|4U|64U|128U));
    CHECK(s_distance_reached && s_stop_due && s_finish_requested && s_finish_tick==9050U);
    memcpy((void*)snapshot_observation_blob,&a,176U);saved=a;
    ChassisAuto_CanStep();ChassisAuto_ReadTailSnapshots(&a);CHECK(memcmp(&a,&saved,176)==0);
    ChassisAuto_SafetyUpdate(0U);ChassisAuto_ReadTailSnapshots(&a);CHECK(memcmp(&a,&saved,176)==0);
    ChassisAuto_ReadTailSnapshots(0);CHECK(!lock_depth);
    CHECK(ChassisAuto_PrepareTranslation(0U,100U,APP_BASELINE40_RUN_MS,2100.0f,&p));
    bad=p;bad.profile.valid=0U;
    CHECK(!ChassisAuto_BeginPreparedTranslation(&bad,g_chassis_auto_session));
    CHECK(memcmp((const void*)&g_chassis_auto_tail_snapshots,&saved,176)==0);
    CHECK(!ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session+1U));
    CHECK(memcmp((const void*)&g_chassis_auto_tail_snapshots,&saved,176)==0);
    /* Accepted nonspecial action retains previous snapshot with its own session. */
    g_chassis_auto_active=g_chassis_auto_cut_vm=s_used=0U;g_chassis_auto_state=CHASSIS_AUTO_IDLE;s_vm_requested=1U;
    g_debug_plan_id=DEBUG_PLAN_DEFAULT;s_motion_action_count=1U;s_endpoint_count=0U;
    CHECK(ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session));CHECK(!s_lateral_enabled);
    CHECK(memcmp((const void*)&g_chassis_auto_tail_snapshots,&saved,176)==0);
    Auto_TailSnapshotRecordLocked(2U,clock_ms);CHECK(memcmp((const void*)&g_chassis_auto_tail_snapshots,&saved,176)==0);
    CHECK(entry_start());ChassisAuto_ReadTailSnapshots(&a);CHECK(a.valid_mask==0 && a.session==g_chassis_auto_session);
    /* No cross-session capture; exact C unsigned wrap for common freshness. */
    s_profile_active=s_run_started=g_chassis_auto_active=s_lateral_enabled=1U;
    s_alignment_seen=1U;s_alignment_ms=0xFFFFFFF0U;s_run_tick=0xFFFFF000U;s_profile_follow_tick=0xFFFFFFFAU;
    s_distance_progress_mm=1990;s_distance_reference.speed_mm_s=80;s_profile_mm_per_rpm_s=4;
    Auto_TailSnapshotRecordLocked(0U,5U);ChassisAuto_ReadTailSnapshots(&a);
    CHECK(a.slot[0].now_ms==5U && a.slot[0].common_ms==0xFFFFFFF0U);
    CHECK((a.slot[0].flags&128U)!=0U && a.slot[0].reference_rpm==20.0f);
    saved=a;g_chassis_auto_session++;Auto_TailSnapshotRecordLocked(1U,6U);
    CHECK(memcmp((const void*)&g_chassis_auto_tail_snapshots,&saved,176)==0);
    cases+=18U;return 0U;
}
#endif
