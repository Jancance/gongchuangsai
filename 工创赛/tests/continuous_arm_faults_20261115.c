#define main previous_owner_main
#include "all-regions-owner.c"
#undef main

static void reach_phase(unsigned desired)
{
 unsigned loops=0;
 start_region(0U,1U);
 while(g_low_phase!=desired && ++loops<300U){
  if(g_low_phase==6U){tick=pulse_apply_at+50U;Low_Service(tick);tick+=50U;Low_Service(tick);}
  else if(owned)finish_group();
  else {tick+=10U;Low_Service(tick);}
 }
 CHECK(loops<300U && Low_Busy() && g_low_phase==desired);
}

int main(void)
{
 unsigned p,k,previous;
 const unsigned phases[]={1U,4U,5U,12U,6U,7U};
 for(p=0U;p<sizeof(phases)/sizeof(phases[0]);p++){
  reach_phase(phases[p]);previous=recorded;g_low_request=9U;Low_Service(tick);
  CHECK(g_low_state==9U && g_low_fault==2U && failures==1U && recorded==previous);
  for(k=0U;k<2U;k++){
   reach_phase(phases[p]);previous=recorded;
   if(k==0U)platform=0U;
   else if(owned)g_source_parallel_state=4U;
   else g_dm_jog_first_fault=1U;
   Low_Service(tick);
   CHECK(g_low_state==9U && failures==1U && recorded==previous);
  }
 }
 reach_phase(5U);
 CHECK(vsp_group.command.axes_mask==6U && vsp_group.command.goal.extension_mm==0.0f);
 CHECK(records[recorded-1U].before_depth==29700.0f);
 CHECK(records[recorded-1U].before_extension>0.0f);
 previous=recorded;
 /* Lift-only completion must not turn DM, cancel/reissue moving X, or
  * trigger the old remaining-retract segment. Wait for the same group. */
 depth=1800.0f;vsp_axis[1].position_cycle=1U;
 vsp_axis[1].current=depth*360.0f/3200.0f;vsp_axis[1].position_tick=tick;
 g_source_parallel_done_mask=SPF_LIFT;Low_Service(tick);
 CHECK(g_low_phase==5U && recorded==previous && owned);
 finish_group();CHECK(g_low_phase==12U && extension==0.0f);
 printf("PASS continuous owner fault exits: %u checks;6 phases STOP/platform/group faults, full retract starts before top, wait for both axes\n",checks);
 return 0;
}
