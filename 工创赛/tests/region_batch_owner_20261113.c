#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define SPF_DM 1U
#define SPF_EXT 2U
#define VEHICLE_EXTENSION_PULSES_PER_MM 315.5f
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static unsigned checks;
static uint32_t g_it_request,g_it_state,g_it_sequence,g_it_action,g_it_axes,g_it_cause,g_it_rejected;
static uint32_t g_it_lead5_grip_done;
static uint32_t g_lift_ruler_request,g_lift_ruler_stop_pending;
static uint32_t g_vis_material_valid,g_vis_mat_cycle,g_vis_mat_tick;
static uint32_t g_vis_mat_left,g_vis_mat_right,g_vis_mat_down;
static float g_it_dm_deg,g_it_extension_mm,g_it_carrier_auto_deg,g_it_lift_pulses;
static float g_startup_extension_current_deg,g_extension_home_reference_deg,g_dm_can2_logical_position_deg;
static uint32_t tick,platform,top,body,fresh,grips,releases,failures;
static uint32_t xTaskGetTickCount(void){return tick;}
static uint32_t it_platform(void){return platform;}
static uint32_t it_at_top(void){return top;}
static uint32_t it_dm_fresh(void){return fresh;}
static uint32_t SingleTeach_ReferencesReady(void){return 1U;}
static uint32_t SourceBodyStep_Busy(void){return body;}
static uint32_t SourceBodyStep_WheelWindow(void){return body;}
static uint32_t VehicleArm_Busy(void){return 0U;}
static uint32_t Stage2Fast_IsActive(void){return 0U;}
static uint32_t SourceStage2_IsActive(void){return 0U;}
static void it_fail(uint32_t cause){failures++;g_it_cause=cause;g_it_state=9U;}
static uint32_t jiazhua(float angle){if(angle==140.0f)grips++;else if(angle==210.0f)releases++;else CHECK(0);return 1U;}
static uint32_t Region_Busy(void);
#include "integration13_immediate.inc"
#include "pzq_near_cycle.inc"
#include "pzq_region_batch.inc"

static void reset(void)
{
 unsigned i;g_region_state=g_region_phase=g_region_item=g_region_fault=0U;
 g_region_request=g_region_rejected=g_region_accepted_sequence=0U;
 g_region_sequence=1U;g_region_observation_cycle=g_vis_mat_cycle=7U;
 g_it_request=g_it_sequence=g_it_cause=g_it_rejected=g_it_auto_state=g_pzq_near_state=0U;
 g_lift_ruler_request=g_lift_ruler_stop_pending=g_it_lead5_grip_done=0U;
 g_it_state=2U;tick=g_vis_mat_tick=1000U;g_vis_material_valid=1U;
 g_vis_mat_left=g_region_colors[0]=5U;g_vis_mat_right=g_region_colors[1]=2U;
 g_vis_mat_down=g_region_colors[2]=1U;
 g_region_task_colors[0]=2U;g_region_task_colors[1]=1U;g_region_task_colors[2]=5U;
 g_region_yaw_deg[0]=194.5516f;g_region_yaw_deg[1]=163.5323f;g_region_yaw_deg[2]=180.0f;
 g_region_extension_mm[0]=162.7926f;g_region_extension_mm[1]=158.3957f;g_region_extension_mm[2]=0.0f;
 for(i=0;i<3;i++)g_region_release_ms[i]=0U;
 g_extension_home_reference_deg=0.0f;g_startup_extension_current_deg=-65.0f*315.5f*360.0f/3200.0f;
 g_dm_can2_logical_position_deg=180.0f;platform=top=fresh=1U;
 body=grips=releases=failures=0U;
}
static void complete(void)
{
 uint32_t request=g_it_request;CHECK(request==3U || request==14U || request==15U);
 g_it_request=0U;g_it_action=request;g_it_sequence++;tick+=500U;
 if(request==3U){
  if(g_it_axes&SPF_DM)g_dm_can2_logical_position_deg=g_it_dm_deg;
  if(g_it_axes&SPF_EXT)g_startup_extension_current_deg=-g_it_extension_mm*315.5f*360.0f/3200.0f;
 }else if(request==14U){
  CHECK(Region_PoseAllowed());CHECK(top);CHECK(g_it_lift_pulses==118800.0f);
  CHECK(fabsf(g_dm_can2_logical_position_deg-g_it_dm_deg)<0.01f);
  g_startup_extension_current_deg=-g_it_extension_mm*315.5f*360.0f/3200.0f;top=0U;
 }else{
  CHECK(Region_PoseAllowed());CHECK(g_it_lift_pulses==7200.0f);
  g_startup_extension_current_deg=0.0f;g_dm_can2_logical_position_deg=37.0f;top=1U;
 }
 g_it_state=2U;Region_Service(tick);
}
static void run_batch(unsigned first_region)
{
 unsigned item,i;uint32_t order[3];
 reset();order[0]=g_region_colors[first_region];
 order[1]=g_region_colors[(first_region+1U)%3U];order[2]=g_region_colors[(first_region+2U)%3U];
 for(i=0;i<3;i++)g_region_task_colors[i]=order[i];
 g_region_request=1U;Region_Service(tick);CHECK(Region_Busy());CHECK(g_it_request==3U);
 CHECK(!it_auto_start(tick));CHECK(!PZQNear_TryStart(1U,2U,tick));
 it_auto_service(tick);PZQNear_Service(tick);
 if(first_region==2U){CHECK(g_it_axes==SPF_EXT);CHECK(!Region_ObserveYawAllowed());}
 else {CHECK(g_it_axes==SPF_DM);CHECK(Region_ObserveYawAllowed());}
 g_region_request=1U;Region_Service(tick+1U);
 CHECK(g_region_rejected==1U && !g_it_rejected && !failures);
 /* Producer edits after acceptance cannot redirect the latched trajectory. */
 g_region_yaw_deg[0]=0.0f;g_region_extension_mm[0]=170.0f;
 for(item=0;item<3;item++){
  CHECK(g_region_item==item);CHECK(g_it_carrier_auto_deg==(float)item*120.0f);
  complete();CHECK(g_it_request==14U);
  if(item==0U && first_region!=2U)CHECK(Region_ObservePickAllowed());
  complete();CHECK(g_it_request==15U);CHECK(grips==item+1U);
  complete();CHECK(!g_it_request && releases==item+1U && g_region_phase==4U);
  CHECK(g_region_release_ms[item]>0U);
  tick=region_phase_tick+49U;Region_Service(tick);CHECK(!g_it_request);
  tick++;Region_Service(tick);CHECK(g_it_request==3U);
 }
 CHECK(g_region_phase==5U);complete();CHECK(g_it_request==3U && g_it_extension_mm==65.0f);
 complete();CHECK(g_region_state==2U && g_region_phase==7U && grips==3U && releases==3U);
 CHECK(g_dm_can2_logical_position_deg==180.0f && Region_AtObserveExtension());
 CHECK(!Region_PoseAllowed() && !Region_ObserveYawAllowed() && !failures);
 g_region_request=1U;Region_Service(tick);CHECK(g_region_rejected==2U && !g_it_request);
}
int main(void)
{
 unsigned i;for(i=0;i<3;i++)run_batch(i);
 reset();g_region_observation_cycle=6U;CHECK(!Region_TryStart(tick) && !g_it_request);
 reset();CHECK(!Region_TryStart(tick+1001U) && !g_it_request);
 reset();g_vis_material_valid=0U;CHECK(!Region_TryStart(tick));
 reset();g_region_colors[0]=2U;CHECK(!Region_TryStart(tick));
 reset();g_region_task_colors[1]=2U;CHECK(!Region_TryStart(tick));
 reset();g_region_yaw_deg[0]=NAN;CHECK(!Region_TryStart(tick));
 reset();g_region_extension_mm[0]=170.01f;CHECK(!Region_TryStart(tick));
 reset();g_region_extension_mm[2]=1.0f;CHECK(!Region_TryStart(tick));
 reset();g_region_sequence=0U;CHECK(!Region_TryStart(tick));
 reset();g_it_auto_state=1U;CHECK(!Region_TryStart(tick));
 reset();body=1U;CHECK(!Region_TryStart(tick));
 reset();top=0U;CHECK(!Region_TryStart(tick));
 reset();fresh=0U;CHECK(!Region_TryStart(tick));
 reset();g_startup_extension_current_deg=0.0f;CHECK(!Region_TryStart(tick));
 reset();CHECK(Region_TryStart(tick));g_it_request=0U;
 Region_Service(tick+100U);CHECK(!g_it_request && g_region_phase==1U); /* old DONE */
 reset();CHECK(Region_TryStart(tick));platform=0U;Region_Service(tick+1U);
 CHECK(g_region_state==9U && failures==1U && g_it_state==9U);
 reset();CHECK(Region_TryStart(tick));g_region_request=9U;Region_Service(tick+1U);
 CHECK(g_region_state==9U && g_it_request==9U);
 reset();CHECK(Region_TryStart(tick));Region_Service(tick+16000U);
 CHECK(g_region_state==9U && failures==1U);
 printf("PASS actual region owner: %u checks, all first regions, latched targets, 49/50ms, real sequence, stop/fault\n",checks);
 return 0;
}
