"""Ensure early/nominal/search recognition never issues a stop and cannot overwrite code."""
def adapt_scan_continue(u):
 u=u.replace('scenario<40','scenario<42')
 u=u.replace('scenario>=30 && scenario<37','scenario>=30 && scenario<37')
 u=u.replace('if(scenario>=37)', 'if(scenario>=37 && scenario<40)')
 u=u.replace('scenario>=20 && scenario<30 && stopped','scenario>=20 && scenario<30 && stopped')
 u=u.replace('scenario>=7 && scenario!=28)', 'scenario>=7 && scenario!=28 && scenario<40)')
 u=u.replace('scenario==27 || scenario==28 || scenario==29){', 'scenario==27 || scenario==28 || scenario==29 || scenario==4 || scenario==40 || scenario==41){')
 # Old early-scan refusal is now a full-length nominal move and normal handoff.
 u=u.replace('   if(scenario==4)ck(!second_stage_seen && g_vehicle_cause==7);','')
 mark='   if(g_vehicle_stage==2){second_stage_seen=1;ck(s_vehicle_handoff_valid);}'
 assert mark in u
 u=u.replace(mark,r'''
   if(!sent && ((scenario==40 && g_vehicle_state==1) ||
       (scenario==41 && g_vehicle_stage==1 && g_chassis_auto_active && s_distance_progress_mm>400))){
    strcpy(SAOMA_Data,"156+123+516+231");g_saoma_accept_count++;g_saoma_last_result=1;
    g_saoma_frame_start_ms=g_saoma_frame_end_ms=test_tick;sent=1;
   }
   if(scenario==41 && s_vehicle_code_valid){
    strcpy(SAOMA_Data,"216+231+612+123");g_saoma_accept_count++;
    g_saoma_frame_start_ms=g_saoma_frame_end_ms=test_tick;
   }
   if(s_vehicle_code_valid && g_vehicle_stage==1){
    ck(!g_stage1_stop_requested && !g_stage1_motion_skipped && !stop_sent);
    ck(!strcmp(s_vehicle_task_code,"156+123+516+231"));
   }
'''+mark)
 mark='  VehicleAlignmentRequest req;'
 u=u.replace(mark,r'''
  if(scenario==0 || scenario==3 || scenario==4 || scenario==40 || scenario==41)ck(!seen1 && !seen2);
  if(scenario==1)ck(seen1 && !seen2);
  if(scenario==2)ck(seen1 && seen2);
'''+mark)
 return u
