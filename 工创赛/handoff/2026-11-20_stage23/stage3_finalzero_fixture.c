#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#undef assert
#define assert(c) do { if(!(c)) {fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);exit(1);} } while(0)
#define SPF_DM 1U
#define SPF_EXT 2U
#define S3_RECOVER_FIRST_PHASE 17U

static uint32_t clock_ms, top, fault, poses;
static uint32_t g_stage3_state, g_stage3_phase, g_stage3_item, g_stage3_recovered_count;
static uint32_t g_stage3_fast_recover_tick, g_stage3_fast_timing_mask;
static uint32_t s3_pending, s3_round_trip, s3_first, s3_count, s3_cycle_mode, s3_tray_valid;
static uint32_t g_stage3_tray_prefetch_item;
static uint32_t g_it_axes, g_it_request, g_it_rejected, g_it_state, g_it_action, g_it_elapsed_ms;
static uint32_t it_stage3_grip100, it_stage3_grip_tick;
static float g_it_dm_deg, g_it_extension_mm, g_it_lift_pulses, g_it_carrier_auto_deg;

static uint32_t xTaskGetTickCount(void){return clock_ms;}
static uint32_t it_at_top(void){return top;}
static uint32_t Stage3_Owns(void){return g_stage3_state==5U;}
static uint32_t S3_AnticipatoryOwned(uint32_t phase){(void)phase;return 0U;}
static void S3_Fail(uint32_t cause){fault=cause;g_stage3_state=9U;}
static void S3_Pose(void){poses++;}
static void S3_IndexItemSlot(void){assert(0);}
static void it_done(void){g_it_state=2U;}
/* EXTRACT_COMMANDS */

static void service(void){
/* EXTRACT_RECOVERY */
}

static void grip_service(void){
 switch(g_it_action){
/* EXTRACT_GRIP */
 default:assert(0);
 }
}

static void setup(uint32_t item){
 clock_ms=1000;top=1;poses=fault=0;
 s3_first=0;s3_count=3;s3_round_trip=s3_cycle_mode=s3_tray_valid=1;
 g_stage3_state=5;g_stage3_phase=24;g_stage3_item=g_stage3_recovered_count=item;
 g_stage3_tray_prefetch_item=item;
 g_stage3_fast_timing_mask=3;g_stage3_fast_recover_tick=0;
 s3_pending=g_it_axes=g_it_request=g_it_rejected=g_it_state=g_it_action=g_it_elapsed_ms=0;
 g_it_dm_deg=36.5f;
}

static void accept(uint32_t action){
 assert(g_it_request==action && s3_pending==action);
 g_it_request=0;g_it_action=action;g_it_state=1;
}

static void assert_incomplete(void){
 assert(g_stage3_state==5 && g_stage3_fast_timing_mask==3 && !g_stage3_fast_recover_tick && !poses);
}

static void release(uint32_t item){
 service();assert(g_stage3_phase==25 && !g_it_request);
 service();assert(g_stage3_phase==26 && !g_it_request);
 service();accept(17);
 it_stage3_grip_tick=clock_ms;it_stage3_grip100=1; /* Backend admitted successful PWM. */
 clock_ms+=99;grip_service();service();
 assert(g_stage3_phase==26 && g_it_state==1 && !g_it_request && g_stage3_recovered_count==item);
 assert_incomplete();
 clock_ms++;grip_service();service();
 assert(g_stage3_phase==28 && g_it_state==2 && !g_it_request);
 assert_incomplete();
}

static void post_zero(void){
 setup(2);release(2);service();
 assert(g_stage3_phase==31 && g_stage3_item==3 && g_stage3_recovered_count==3 && !g_it_request);
 assert_incomplete();
 service();assert(g_it_axes==SPF_DM && g_it_dm_deg==0.0f && g_stage3_phase==31);
 service();assert(g_it_request==3 && g_it_action==17);assert_incomplete();
 accept(3);
}

int main(void){
 uint32_t i;
 for(i=0;i<2U;i++){
  setup(i);release(i);service();
  assert(g_stage3_phase==S3_RECOVER_FIRST_PHASE && g_stage3_item==i+1 &&
         g_stage3_recovered_count==i+1 && !g_it_request && g_it_dm_deg==36.5f);
  assert_incomplete();
 }
 puts("PASS first two releases complete their 100 ms dwell and go to the next item without zeroing");

 post_zero();
 for(i=0;i<3U;i++){clock_ms+=100;service();assert_incomplete();assert(!g_it_request && g_stage3_phase==31);}
 g_it_state=2;g_it_action=17;service();assert_incomplete();assert(g_stage3_phase==31);
 g_it_action=3;clock_ms++;service();
 assert(g_stage3_phase==32 && g_stage3_state==6 && g_stage3_fast_timing_mask==7 &&
        g_stage3_fast_recover_tick==clock_ms && !s3_pending && poses==1);
 puts("PASS final release waits 100 ms, submits DM-only 0, then timestamps only the matching DONE");

 post_zero();g_it_rejected=6;g_it_state=2;service();
 assert(g_stage3_state==9 && fault==6 && !poses && g_stage3_fast_timing_mask==3 && !g_stage3_fast_recover_tick);
 setup(2);service();service();service();accept(17);g_it_rejected=7;service();
 assert(g_stage3_state==9 && fault==7 && !poses && !g_it_request && g_stage3_recovered_count==2 &&
        g_stage3_fast_timing_mask==3 && !g_stage3_fast_recover_tick && g_it_dm_deg==36.5f);
 puts("PASS either release or DM rejection follows the existing failure path without completion timing");
 return 0;
}
