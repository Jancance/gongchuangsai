"""Compile the entire actual IT3 guard, including the formerly missed branch."""
from prepare_three_frame_warm_20261118 import P,BASE,E
from check_stage2_extension_mirror_20261117 import function
from check_three_frame_warm_c_20261118 import run
import json


def guard(text):
    start=text.index('}else if(request==3U){')
    start=text.index('    if(!SingleTeach_ReferencesReady()',start)
    end=text.index('g_it_rejected=6U;',start)+len('g_it_rejected=6U;')
    return text[start:end]


def main():
    old=(BASE/'CHASSIS/USER/integration10_complete.inc').read_text(encoding='utf-8')
    new=(P/'CHASSIS/USER/integration10_complete.inc').read_text(encoding='utf-8')
    helper=function((P/'CHASSIS/USER/source_mcu_cycle.inc').read_text(encoding='utf-8'),'static uint32_t SC_InitialObserveAllowed(')
    code=r'''
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "source_parallel_flow.h"
#include "vehicle_parameters.h"
static uint32_t g_source_cycle_state,g_source_cycle_prep,g_it_axes,top=1,refs=1,low,region;
static uint32_t g_it_place_depth_limit_pulses;
static float g_it_dm_deg=180,g_it_extension_mm=65,g_it_lift_pulses=29700;
static float g_startup_extension_current_deg,g_extension_home_reference_deg,g_dm_can2_logical_position_deg;
static unsigned checks;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t it_at_top(void){return top;}
static uint32_t SingleTeach_ReferencesReady(void){return refs;}
static uint32_t Low_XYAllowed(void){return low;}
static uint32_t Region_ObserveYawAllowed(void){return region;}
'''+helper+'\n'
    for name,src in [('old',guard(old)),('new',guard(new))]:
        code+='static unsigned '+name+'_allowed(void){unsigned axes=g_it_axes,stage3_scope=0,lift_limit=SPF_MAX_LIFT_PULSES,g_it_rejected=0;\n'+src+'\nreturn !g_it_rejected;}\n'
    code+=r'''
int main(void){unsigned s,p;g_it_axes=3;
 for(s=0;s<10;s++)for(p=0;p<6;p++){
  g_source_cycle_state=s;g_source_cycle_prep=p;
  CHECK(!old_allowed());CHECK(new_allowed()==(s==1 && p==4));
 }
 g_source_cycle_state=1;g_source_cycle_prep=4;
 CHECK(new_allowed());top=0;CHECK(!new_allowed());top=1;
 refs=0;CHECK(!new_allowed());refs=1;
 g_startup_extension_current_deg=2.01f;CHECK(!new_allowed());g_startup_extension_current_deg=0;
 g_it_extension_mm=164;CHECK(!new_allowed());g_it_extension_mm=65;
 g_it_dm_deg=195;CHECK(!new_allowed());g_it_dm_deg=180;
 g_it_axes=1;g_it_extension_mm=0;CHECK(old_allowed() && new_allowed());
 g_it_axes=2;g_it_extension_mm=65;CHECK(old_allowed() && new_allowed());
 g_it_axes=3;g_it_extension_mm=164;low=1;CHECK(old_allowed() && new_allowed());
 low=0;g_source_cycle_state=6;CHECK(!new_allowed());
 printf("PASS actual complete IT3 guard %u checks: exact first180/65 XY exception, existing top/zero/ref retained, all unrelated scopes unchanged\n",checks);return 0;}
'''
    result=run('actual-complete-initial-IT3-guard',code)
    (E/'initial-guard-checks.json').write_text(json.dumps(dict(passed=True,result=result,hardware_access=False),indent=2)+'\n')


if __name__=='__main__':main()
