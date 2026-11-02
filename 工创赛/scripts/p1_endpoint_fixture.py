"""Correct the offline endpoint injection contract without editing firmware."""

HELPER = r'''
/* Test-only injection evidence; driver sign and acknowledged 0.1RPM target. */
volatile float endpoint_injections[56][10];
static uint32_t EndpointInjectionAllowed(uint32_t a)
{
 const ChassisDirectionGuard *d=&s_direction_guard[a-1U];
 uint32_t now=test_tick;
 if(!d->valid || !d->target_deci ||
    (uint32_t)(now-d->direction_ms)<500U) return 0U;
 if(micro_case==4U) {
  /* Deliberately outside the 10mm endpoint allowance. */
  return s_distance_progress_mm>=s_distance_target_mm-50.0f &&
         s_distance_progress_mm<s_distance_target_mm-10.0f;
 }
 return s_forward_hold && s_distance_mode && s_route_trial && s_profile_active &&
     !s_finish_requested && !s_stop_due && !s_distance_reached &&
     s_translation_base_rpm<=2U && s_distance_target_mm>=100.0f &&
     s_distance_progress_mm>=s_distance_target_mm-10.0f &&
     s_distance_progress_mm<=s_distance_target_mm &&
     s_distance_reference.position_mm>=s_distance_target_mm*.5f &&
     s_distance_reference.acceleration_mm_s2<=0.0f &&
     d->target_deci>=-DIRECTION_TARGET_MAX_DECI &&
     d->target_deci<=DIRECTION_TARGET_MAX_DECI;
}
'''

GUARD_CHECKS = r'''
static void endpoint_guard_contract_checks(void)
{
 ChassisDirectionGuard d;uint32_t i;int32_t sign;
 for(sign=-1;sign<=1;sign+=2) {
  for(i=1;i<=20;i++) {
   DirectionGuard_Reset(&d);DirectionGuard_Ack(&d,sign*(int32_t)i,1000);
   CHECK(DirectionGuard_Check(&d,1600,1600,-sign*5,1)==0);
   CHECK(d.pending && d.reverse_samples==1);
   /* The same cached reverse sample does not count as another event. */
   CHECK(DirectionGuard_Check(&d,1610,1600,-sign*5,1)==0);
   CHECK(d.reverse_samples==1);
   CHECK(DirectionGuard_Check(&d,1620,1620,sign,1)==0);CHECK(!d.pending);
  }
  DirectionGuard_Reset(&d);DirectionGuard_Ack(&d,sign*21,1000);
  CHECK(DirectionGuard_Check(&d,1600,1600,-sign*3,1)==DIRECTION_REVERSE);
  DirectionGuard_Reset(&d);DirectionGuard_Ack(&d,sign*20,1000);
  CHECK(DirectionGuard_Check(&d,1600,1600,-sign*6,1)==DIRECTION_REVERSE);
  DirectionGuard_Reset(&d);DirectionGuard_Ack(&d,sign*20,1000);
  CHECK(DirectionGuard_Check(&d,1600,1600,-sign*3,0)==DIRECTION_REVERSE);
  DirectionGuard_Reset(&d);DirectionGuard_Ack(&d,sign*20,1000);
  CHECK(DirectionGuard_Check(&d,1600,1600,-sign*3,1)==0);
  CHECK(DirectionGuard_Check(&d,1620,1620,-sign*3,1)==DIRECTION_PERSISTENT);
  DirectionGuard_Reset(&d);DirectionGuard_Ack(&d,sign*20,1000);
  CHECK(DirectionGuard_Check(&d,1600,1600,-sign*3,1)==0);
  CHECK(!DirectionGuard_Expired(&d,2199));CHECK(DirectionGuard_Expired(&d,2200));
  CHECK(DirectionGuard_Check(&d,2200,2200,sign,1)==DIRECTION_PERSISTENT);
 }
}
'''


def correct_endpoint_fixture(source):
    old = '''    s_distance_progress_mm>=s_distance_target_mm-(micro_case==4U?50.0f:3.0f) &&
    (micro_case==4U || s_translation_base_rpm<=2U) &&'''
    assert source.count(old) == 1
    source=source.replace(old, '    EndpointInjectionAllowed(a) &&')
    old=''' if(micro_case && a==1U && s_translation_mode==2U && (g_chassis_auto_state==CHASSIS_AUTO_RUN || g_chassis_auto_state==CHASSIS_AUTO_START) &&
    EndpointInjectionAllowed(a) &&
    (micro_case==2U || !micro_injected)) {'''
    assert source.count(old)==1
    source=source.replace(old,''' if((micro_case==2U && micro_injected && a==1U && s_translation_mode==2U && g_chassis_auto_active) ||
    (micro_case && a==1U && s_translation_mode==2U && (g_chassis_auto_state==CHASSIS_AUTO_RUN || g_chassis_auto_state==CHASSIS_AUTO_START) &&
     EndpointInjectionAllowed(a) && !micro_injected)) {''')
    old='  raw=micro_case==3U?0.6f:0.3f;micro_injected++;'
    assert source.count(old) == 1
    source=source.replace(old, r'''  /* A persistent fault stays present through STOP/SETTLE until cut-off.
   * Do not accidentally stop injecting when the endpoint gate closes. */
  raw=(micro_case==2U && micro_injected)?endpoint_injections[scenario][1]:
      (s_direction_guard[a-1U].target_deci>0?-1.0f:1.0f)*(micro_case==3U?0.6f:0.3f);
  if(!micro_injected) {
   endpoint_injections[scenario][0]=(float)s_direction_guard[a-1U].target_deci*.1f;
   endpoint_injections[scenario][1]=raw;
   endpoint_injections[scenario][2]=(float)s_translation_base_rpm;
   endpoint_injections[scenario][3]=s_distance_progress_mm;
   endpoint_injections[scenario][4]=s_distance_target_mm-s_distance_progress_mm;
   endpoint_injections[scenario][5]=(float)(test_tick-s_direction_guard[a-1U].ack_ms);
   endpoint_injections[scenario][6]=s_distance_reference.position_mm;
   endpoint_injections[scenario][7]=s_distance_reference.acceleration_mm_s2;
   endpoint_injections[scenario][8]=(float)micro_case;
  }
  micro_injected++;endpoint_injections[scenario][9]=(float)micro_injected;''')
    anchor='static float micro_frozen[4];'
    assert source.count(anchor) == 1
    source=source.replace(anchor,anchor+'\n'+HELPER)
    anchor='uint32_t test_entry(void){'
    assert source.count(anchor) == 1
    source=source.replace(anchor,GUARD_CHECKS+'\n'+anchor+'\n endpoint_guard_contract_checks();')
    return source
