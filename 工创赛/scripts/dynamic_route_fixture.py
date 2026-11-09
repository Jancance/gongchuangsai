"""Exercise feedback-replanned one-leg routing and preserved source preparation."""
def adapt_dynamic(u):
    u=u.replace('scenario<37','scenario<40')
    u=u.replace('if(scenario>=30)', 'if(scenario>=30 && scenario<37)')
    u=u.replace('stop_sent=0;model_gate_started=', 'uint32_t test_max_queue_wait=0,test_first_queue_mask=0,test_stage2_corrupted=0;\n  stop_sent=0;model_gate_started=')
    mark='   if(scenario==13 && g_vehicle_stage==2 && moving)g_vehicle_key=1;'
    assert mark in u
    u=u.replace(mark,mark+r'''
   if(scenario>=37 && g_vehicle_stage==2 && g_combined_test.state==3 && !test_stage2_corrupted){
    if(scenario==37)for(unsigned i=0;i<4;i++)s_endpoints[s_endpoint_index].end_ms[i]=test_tick-1001;
    if(scenario==38)s_endpoints[s_endpoint_index].end_mask=7;
    if(scenario==39)s_endpoints[s_endpoint_index].session++;
    test_stage2_corrupted=1;
   }
''')
    mark='   if(g_combined_test.request)stop_sent=0;'
    u=u.replace(mark,mark+r'''
   if(g_vehicle_queue_wait_ms>test_max_queue_wait)test_max_queue_wait=g_vehicle_queue_wait_ms;
   if(g_vehicle_queue_first_mask)test_first_queue_mask=g_vehicle_queue_first_mask;
''')
    u=u.replace('g_vehicle_queue_wait_ms>0 && g_vehicle_queue_wait_ms<1000', 'test_max_queue_wait>0 && test_max_queue_wait<1000')
    u=u.replace('g_vehicle_queue_first_mask==256 && g_vehicle_queue_wait_ms>=100', 'test_first_queue_mask==256 && test_max_queue_wait>=100')
    u=u.replace('  VehicleAlignmentRequest req;', '  if(scenario>=37)ck(test_stage2_corrupted && g_vehicle_cause==10 && !g_source_prepare_release_ms && !fixture_vm);\n  VehicleAlignmentRequest req;')
    old='''ck(a->reference_mm_s==400 && s_motion_action_count>=2 && s_motion_action_count<=3);
   if(g_combined_test.index==s_motion_action_count-1)ck(a->kind==MOTION_RIGHT_DISTANCE && fabsf(a->distance_mm-860)<.01f);'''
    assert old in u
    u=u.replace(old,r'''
   ck(a->reference_mm_s==400 && s_motion_action_count==1 && g_combined_test.index==0);
   float dx=s_vehicle_stage2_leg_target.x_mm-s_vehicle_handoff.x_mm;
   float dy=s_vehicle_stage2_leg_target.y_mm-s_vehicle_handoff.y_mm;
   if(g_vehicle_route_phase==2)ck(a->kind==(dy>0?MOTION_FORWARD_DISTANCE:MOTION_BACKWARD_DISTANCE) && fabsf(a->distance_mm-fabsf(dy))<.01f);
   else ck(a->kind==(dx>0?MOTION_RIGHT_DISTANCE:MOTION_LEFT_DISTANCE) && fabsf(a->distance_mm-fabsf(dx))<.01f);
''')
    u=u.replace('if(g_vehicle_state==4)ck(g_source_prepare_state==3', 'if(g_vehicle_state==4)ck(g_vehicle_route_phase==4 && g_source_prepare_state==3')
    fn=r'''
static void dynamic_geometry_checks(void){
 RouteRow rows[8];FieldPoint goal;uint32_t n;uint32_t original_calls=hardware_calls;
 ck(VEHICLE_STAGE1_FORWARD_DISTANCE_CM==15 && FIELD_SCAN_Y_MM==300);
 s_vehicle_handoff_valid=1;g_vehicle_route_phase=1;s_vehicle_handoff=(FieldPoint){1320,390};
 ck(VehicleMission_Stage2Rows(rows,&n,&goal)==1 && n==1 && rows[0].kind==MOTION_LEFT_DISTANCE && fabsf(rows[0].amount-12)<.001f);
 ck(s_vehicle_stage2_leg_target.x_mm==1200 && s_vehicle_stage2_leg_target.y_mm==390);
 /* 横移后的Y读回改变，下一条距离必须随之改变，不能沿用首轮计划。 */
 g_vehicle_route_phase=2;s_vehicle_handoff=(FieldPoint){1200,395};
 ck(VehicleMission_Stage2Rows(rows,&n,&goal)==1 && rows[0].kind==MOTION_FORWARD_DISTANCE && fabsf(rows[0].amount-80.5f)<.001f);
 s_vehicle_handoff=(FieldPoint){1200,390};
 ck(VehicleMission_Stage2Rows(rows,&n,&goal)==1 && fabsf(rows[0].amount-81)<.001f);
 s_vehicle_handoff=(FieldPoint){1200,300};
 ck(VehicleMission_Stage2Rows(rows,&n,&goal)==1 && fabsf(rows[0].amount-90)<.001f);
 s_vehicle_handoff=(FieldPoint){1200,240};
 ck(VehicleMission_Stage2Rows(rows,&n,&goal)==1 && fabsf(rows[0].amount-96)<.001f);
 g_vehicle_route_phase=3;s_vehicle_handoff=(FieldPoint){1205,1203};
 ck(VehicleMission_Stage2Rows(rows,&n,&goal)==1 && rows[0].kind==MOTION_RIGHT_DISTANCE && fabsf(rows[0].amount-85.5f)<.001f && goal.x_mm==2060 && goal.y_mm==1200);
 s_vehicle_handoff=(FieldPoint){1200,1230};ck(!VehicleMission_Stage2Rows(rows,&n,&goal));
 s_vehicle_handoff=(FieldPoint){2060,1200};ck(VehicleMission_Stage2Rows(rows,&n,&goal)==2 && !n);
 g_vehicle_route_phase=1;s_vehicle_handoff=(FieldPoint){1208,390};ck(VehicleMission_Stage2Rows(rows,&n,&goal)==2);
 s_vehicle_handoff=(FieldPoint){1080,390};ck(VehicleMission_Stage2Rows(rows,&n,&goal)==1 && rows[0].kind==MOTION_RIGHT_DISTANCE && fabsf(rows[0].amount-12)<.001f);
 s_vehicle_handoff=(FieldPoint){1450,300};ck(!VehicleMission_Stage2Rows(rows,&n,&goal));
 s_vehicle_handoff_valid=0;ck(!VehicleMission_Stage2Rows(rows,&n,&goal));
 ck(hardware_calls==original_calls);
}
'''
    u=u.replace('int main(void){',fn+'\nint main(void){\n dynamic_geometry_checks();')
    return u
