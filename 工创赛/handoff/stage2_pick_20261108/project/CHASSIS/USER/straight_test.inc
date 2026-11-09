/* 独立测距入口：只发布原CAN后端的一条平移动作，不启动任何机构。 */
static void VehicleMission_StraightService(uint32_t key,uint32_t now)
{
    RouteRow row;FieldPoint begin={FIELD_START1_X_MM,FIELD_START1_Y_MM},end=begin;
    if(key){
        if(g_vehicle_state==4U || g_vehicle_state==5U){g_vehicle_cause=19U;return;}
        if(g_vehicle_state!=0U){VehicleMission_Fail(2U);g_vehicle_state=7U;return;}
        if(!App_SerialIdleReady(now) || !g_safety_ready || g_rtos_fault || g_safety_fault || g_can_ctrl_fault_latched)
        {g_vehicle_cause=1U;return;}
        if(!(VEHICLE_STRAIGHT_TEST_DISTANCE_CM>=1 && VEHICLE_STRAIGHT_TEST_DISTANCE_CM<=210 &&
             VEHICLE_STRAIGHT_TEST_SPEED_CM_S>=1.21f && VEHICLE_STRAIGHT_TEST_SPEED_CM_S<=40 &&
             VEHICLE_STRAIGHT_TEST_TIMEOUT_MS>=10000U && VEHICLE_STRAIGHT_TEST_TIMEOUT_MS<=30000U))
        {VehicleMission_Fail(7U);return;}
        end.y_mm+=VEHICLE_STRAIGHT_TEST_DISTANCE_CM*10;
        if(!FieldMap_AxisClear(begin,end,FIELD_BODY_HALF_MM)){VehicleMission_Fail(7U);return;}
        s_vehicle_zone=1U;g_vehicle_run_id++;g_vehicle_stage=1U;g_vehicle_cause=0U;g_vehicle_stage_mask=0U;
        s_vehicle_start=s_vehicle_stage_start=now;g_vehicle_scan_bypassed=1U;
        RouteHeading_Reset(&s_route_heading_reference);
        row=(RouteRow){MOTION_FORWARD_DISTANCE,VEHICLE_STRAIGHT_TEST_DISTANCE_CM,VEHICLE_STRAIGHT_TEST_SPEED_CM_S};
        if(!VehicleMission_Queue(1U,&row,1U)){VehicleMission_Fail(7U);return;}
    }
    if(g_vehicle_state==2U){
        if(g_rtos_fault || g_safety_fault || g_can_ctrl_fault_latched){VehicleMission_Fail(4U);return;}
        if(now-s_vehicle_start>=VEHICLE_STRAIGHT_TEST_TIMEOUT_MS){VehicleMission_Fail(5U);return;}
        if(g_combined_test.state>=4U){VehicleMission_Fail(8U);return;}
        if(VehicleMission_Stopped()){g_vehicle_stage_mask=1U;g_vehicle_state=4U;}
    }
}
