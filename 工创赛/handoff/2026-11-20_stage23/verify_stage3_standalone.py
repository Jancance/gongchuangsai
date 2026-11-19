"""Compile Stage3 admission and key dispatch from the actual candidate source."""
import json
from pathlib import Path
import subprocess
import tempfile

WORK = Path(__file__).resolve().parent
ROOT = WORK / 'stage3_standalone_20261120/project'

STUBS = r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define assert(c) do { if(!(c)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);exit(1);} }while(0)
#define SPF_STAGE3_MAX_LIFT_PULSES 82000U
#define SOURCE_LIFT_PULSES_PER_MM 400.0f
static uint32_t platform=1,wheel_window,pose_calls,replay_calls;
static uint32_t g_it_request,g_it_action,g_it_state;
static uint32_t g_low_wheel_hold_ready,g_low_wheel_hold_stage,g_low_wheel_hold_request;
static uint32_t S3_Platform(void){return platform;}
static uint32_t SourceBodyStep_WheelWindow(void){return wheel_window;}
static void S3_ReadActual(void){}
static void S3_Teach(void){}
static void S3_ReplayBegin(void){replay_calls++;}
static void S3_Pose(void){pose_calls++;}
'''

TEST = r'''
static void S3_Fail(uint32_t cause){g_stage3_fault=cause;g_stage3_state=9;g_it_request=9;}
int main(void){
 stage3_admit();assert(g_stage3_config_ready==VEHICLE_STAGE3_STANDALONE_ENABLE);
 assert(!g_stage3_state && !g_stage3_run && !g_stage3_request && !g_it_request && !pose_calls);
#if VEHICLE_STAGE3_STANDALONE_ENABLE
 assert(g_stage3_cycle_mode==1 && g_stage3_task_run==1 && g_stage3_task_batch==1 &&
        !g_stage3_test_first_item && g_stage3_test_count==3 && !g_stage3_replay_enable &&
        !g_stage3_first_mapping_valid && g_stage3_integrated_compensation==1);
 /* INSERT_CHECKS */
 Stage3_KeyEvent();assert(g_stage3_request==1 && !g_it_request && !g_stage3_state);
 stage3_admit();assert(g_stage3_run==1 && g_stage3_state==1 && replay_calls==1 && pose_calls==1);
 g_stage3_state=6;Stage3_KeyEvent();stage3_admit();
 assert(g_stage3_run==2 && g_stage3_state==1 && replay_calls==2 && pose_calls==2);
 Stage3_KeyEvent();assert(g_stage3_request==9);stage3_admit();
 assert(g_stage3_state==9 && g_stage3_fault==2 && g_it_request==9);
 Stage3_KeyEvent();assert(!g_stage3_request && g_stage3_fault==2);
 for(uint32_t i=1;i<=11;i++)if(i!=6 && i!=9){
  g_stage3_state=i;g_stage3_request=0;Stage3_KeyEvent();assert(g_stage3_request==9);
 }
 g_stage3_state=g_stage3_fault=g_stage3_request=g_it_request=0;
 g_stage3_pick_depth=-1;Stage3_KeyEvent();stage3_admit();
 assert(!g_stage3_state && !g_it_request && !g_stage3_config_ready && g_stage3_run==2);
 g_stage3_pick_depth=VEHICLE_STAGE3_STANDALONE_PICK_PULSES;
 platform=0;Stage3_KeyEvent();stage3_admit();assert(!g_stage3_state && g_stage3_run==2);
 platform=1;wheel_window=1;Stage3_KeyEvent();stage3_admit();assert(!g_stage3_state && g_stage3_run==2);
 puts("PASS standalone: exact compiled preset; idle boot, first and repeat button, STOP/fault and start admission");
#else
 assert(!g_stage3_cycle_mode && !g_stage3_task_run && g_stage3_pick_depth==-1.f);
 Stage3_KeyEvent();assert(g_stage3_request==10);stage3_admit();assert(g_stage3_state==10);
 g_stage3_state=11;Stage3_KeyEvent();assert(g_stage3_request==1);
 g_stage3_state=7;Stage3_KeyEvent();assert(g_stage3_request==2);
 g_it_request=18;Stage3_KeyEvent();assert(g_stage3_request==9);
 g_stage3_state=6;g_stage3_request=0;Stage3_KeyEvent();assert(!g_stage3_request);
 puts("PASS disabled build: original load, bench, touchdown STOP and complete key routes");
#endif
 return 0;
}
'''


def main():
    s = (ROOT/'CHASSIS/USER/stage3_cycle.inc').read_text(encoding='latin1')
    globals_ = s[s.index('#include "../../USER/vehicle_parameters.h"'):s.index('uint32_t Stage3_Owns(void)')]
    globals_ = globals_.replace('"../../USER/vehicle_parameters.h"','"vehicle_parameters.h"',1)
    header = (ROOT/'CHASSIS/USER/stage3_cycle.h').read_text(encoding='latin1')
    offset = next(line for line in header.splitlines() if line.startswith('#define S3_FAST_HANDOFF_OFFSET_PULSES '))
    key = s[s.index('void Stage3_KeyEvent(void)'):s.index('void Stage3_Arrive(void)')]
    config = s[s.index('static uint32_t S3_TargetPosition('):s.index('static uint32_t S3_TeachReady(')]
    start = s.index('static void Stage3_Service(uint32_t now)')
    admission = s[start:s.index(' if(request==32U){',start)].replace('Stage3_Service(uint32_t now)','stage3_admit(void)')+'}\n'
    c = json.loads((WORK/'stage3_standalone_validation.json').read_text())['config']
    checks = []
    for name,values in dict(colors=[2,1,5],positions=c['mission_rings'],slot_colors=c['slots'],
                            target_yaw=c['yaw'], target_x=c['x'],target_depth=c['z_place'],
                            recovery_depth=c['z_recover']).items():
        for i,value in enumerate(values):
            checks.append(f'assert(g_stage3_{name}[{i}]=={float(value)}f);')
    checks.extend(f'assert(g_stage3_{name}=={value}f);' for name,value in
                  [('carrier_yaw',c['carrier_deg']),('pick_depth',c['pick_pulses']),('observe_yaw',180.55)])
    fixture = '\n'.join((STUBS, offset, globals_, key, config, 'static void S3_Fail(uint32_t cause);',
                         admission, TEST.replace('/* INSERT_CHECKS */','\n '.join(checks))))
    with tempfile.TemporaryDirectory(dir=WORK) as temp:
        p = Path(temp)/'stage3_standalone.c'
        p.write_text(fixture,encoding='latin1')
        for flags in ([],['-DVEHICLE_STAGE3_STANDALONE_ENABLE=0U']):
            exe = p.with_suffix('.exe')
            subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-variable',
                            '-I',str(ROOT/'USER'),*flags,str(p),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)


if __name__ == '__main__':
    main()
