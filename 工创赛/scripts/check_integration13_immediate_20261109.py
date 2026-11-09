"""Exercise the exact owner-chain C, mocked motion completions only."""
from pathlib import Path
import subprocess
R=Path(__file__).resolve().parents[1];P=R/'tmp/stage2_sub20_20261109/project/CHASSIS/USER'
OUT=P.parents[2];E=R/'evidence/integration/2026-11-07/stage2-mainline'
unit=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "source_parallel_flow.h"
#undef assert
#define assert(x) do {if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t g_it_axes,g_it_sequence,g_it_request,g_it_state=2,g_it_rejected,g_it_cause;
static uint32_t g_lift_ruler_request,g_lift_ruler_stop_pending,g_it_action;
static float g_it_dm_deg,g_it_extension_mm,g_it_carrier_auto_deg,g_it_lift_pulses;
static float g_startup_extension_current_deg,g_extension_home_reference_deg;
static uint32_t platform=1,grip_count,open_count,fail_count;
static uint32_t g_it_lead5_grip_done;
static uint32_t it_platform(void){return platform;}
static uint32_t it_at_top(void){return 1;}
static uint32_t SingleTeach_ReferencesReady(void){return 1;}
static uint32_t VehicleArm_Busy(void){return 0;}
static uint32_t SourceStage2_IsActive(void){return 0;}
static uint32_t Stage2Fast_IsActive(void){return 0;}
static void it_fail(uint32_t why){fail_count++;g_it_cause=why;g_it_state=9;}
static uint32_t jiazhua(float deg){if(deg==140)grip_count++;else if(deg==210)open_count++;else assert(0);return 1;}
#include "integration13_immediate.inc"
static void complete(uint32_t now){
 uint32_t request=g_it_request;assert(request);g_it_request=0;
 g_it_action=request;g_it_sequence++;g_it_state=2;
 it_auto_service(now);
}
int main(void){
 assert(it_auto_start(1000) && g_it_request==3 && g_it_dm_deg==180);
 assert(!it_auto_start(1001));
 it_auto_service(1002);assert(g_it_request==3 && !grip_count); /* stale DONE */
 for(uint32_t item=0;item<3;item++){
  uint32_t now=2000+item*3000;
  complete(now);assert(g_it_request==14 && g_it_extension_mm==(item==2?160:(item==1?156:0)));
  complete(now+1000);assert(g_it_request==15 && grip_count==item+1);
  complete(now+2000);assert(!g_it_request && open_count==item+1 && g_it_auto_phase==6);
  it_auto_service(now+2199);assert(!g_it_request && g_it_auto_item==item);
  it_auto_service(now+2200);assert(g_it_request==3);
  assert(g_it_auto_release_ms[item]==now+2000-1000);
 }
 assert(g_it_dm_deg==0 && g_it_auto_phase==4 && !fail_count);
 complete(11000);assert(g_it_auto_state==2 && g_it_auto_elapsed_ms==10000);
 g_it_auto_state=0;g_it_state=2;g_it_cause=0;g_it_request=0;g_it_rejected=0;
 assert(it_auto_start(20000));platform=0;it_auto_service(20001);
 assert(g_it_auto_state==9 && g_it_state==9 && fail_count==1 && !g_it_request);
 puts("PASS: actual chain: three fresh move cycles, grip->return immediate, release->next yaw waits199/200ms boundary including final zero, stale completion rejected, fault cancels. SIMULATED COMPLETIONS ONLY.");
 return 0;
}
'''
src=OUT/'integration13-immediate-test.c';exe=OUT/'integration13-immediate-test.exe';src.write_text(unit)
r=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(P),str(src),'-o',str(exe)],capture_output=True,text=True)
log=r.stdout+r.stderr
if r.returncode==0:
 r=subprocess.run([str(exe)],capture_output=True,text=True,timeout=10);log+=r.stdout+r.stderr
(E/'integration13-immediate-host-20261109.log').write_text(log,encoding='utf-8');print(log);assert r.returncode==0
