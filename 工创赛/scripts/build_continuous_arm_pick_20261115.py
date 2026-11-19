"""Validate actual owner/VSP payload and build both isolated firmware targets."""
from pathlib import Path
import json
import shutil
import subprocess
import sys

import build_integration18_ext150_20261115 as build

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-15/continuous-arm-pick'
P = R / 'tmp/source_continuous_arm_pick_20261115/project'
BASE = R / 'tmp/source_mcu_rimpark_20261115/project'


def run(name, source):
    path = E / (name + '.c')
    path.write_text(source, encoding='utf-8')
    exe = E / (name + '.exe')
    subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
        '-Wno-misleading-indentation', '-I'+str(E), '-I'+str(R/'tests'),
        '-I'+str(P/'CHASSIS/USER'), '-I'+str(P/'USER'), str(path), '-lm', '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], cwd=E, capture_output=True, text=True)
    (E / (name + '.log')).write_text(result.stdout + result.stderr, encoding='utf-8')
    print(result.stdout + result.stderr, flush=True)
    assert result.returncode == 0


def main():
    proof = json.loads((E/'matlab-result.json').read_text())
    assert proof['passed'] and proof['local_matlab']
    old = R / 'evidence/integration/2026-11-15/source-rim-reacquire'
    for name in ('can.h', 'Buzzer.h', 'tested-key-branch.inc', 'static-cycle-fixture.c',
                 'tested-wheel-hold.inc', 'tested-close-held.inc'):
        shutil.copyfile(old/name, E/name)
    for source in old.glob('python-continuous-*.txt'):
        shutil.copyfile(source, E/source.name)
    owner = (P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes()
    (E/'tested-commit-owner.inc').write_bytes(owner.replace(b'#include "low_readonly_diag.inc"', b'/* separate diagnostic fixture */'))
    fixture = (R/'tests/overlap_commit_owner_20261114.c').read_text()
    fixture = fixture.replace('records[16]', 'records[32]').replace('recorded<16', 'recorded<32')
    fixture = fixture.replace('if(g_low_phase==2)CHECK(vsp_group.command.axes_mask==6 && vsp_group.command.goal.lift_pulses==15600);',
        'if(g_low_phase==4)CHECK(vsp_group.command.axes_mask==6 && vsp_group.command.goal.lift_pulses==29700);')
    fixture = fixture.replace('vsp_group.command.goal.extension_mm==fmaxf(0,g_low_target_extension_mm-60)',
                               'vsp_group.command.goal.extension_mm==0')
    fixture = fixture.replace('extension==65 && fabsf(depth-600)<.1f && g_dm_can2_logical_position_deg==180',
        'extension==(slot==3?0:65) && fabsf(depth-600)<.1f && g_dm_can2_logical_position_deg==(slot==3?0:180)')
    # Assert no intermediate hover command or post-top retract exists.
    fixture = fixture.replace('CHECK(g_low_group_count==recorded);',
        'CHECK(g_low_group_count==recorded);\n for(loops=0;loops<recorded;loops++){CHECK(records[loops].phase!=2 && records[loops].phase!=11);'
        'if(records[loops].phase==5)CHECK(records[loops].command.goal.extension_mm==0);}')
    (E/'base-commit-fixture.c').write_text(fixture, encoding='utf-8')
    (E/'all-regions-owner.c').write_text(fixture, encoding='utf-8')
    previous = (R/'tests/source_continuous_owner_20261115.c').read_text()
    (E/'prior-continuous-test.c').write_text(previous.replace('int main(void){',
        'static int prior_continuous_test_main(void){', 1), encoding='utf-8')
    run('final-park-continuous-owner', (R/'tests/source_rimpark_owner_20261115.c').read_text())
    run('continuous-owner-faults', (R/'tests/continuous_arm_faults_20261115.c').read_text())
    # Compare C timing calculation to independently generated local MATLAB vectors.
    cases = '\n'.join(' CK(fabsf(SourcePick_ContinuousRPM(%.9ff,%.9ff,%.9ff)-%.4ff)<0.11f);' % (
        abs(v['goal_x']-v['initial_x'])*315.5/3200,
        (29700-v['initial_z'])/3200, (15600-v['initial_z'])/3200, v['lift_rpm'])
        for v in proof['comparison_vectors'])
    run('matlab-c-timing', '#include <stdio.h>\n#include <stdlib.h>\n#include "source_parallel_flow.h"\n'
        '#include "source_continuous_descent_20261115.h"\nstatic unsigned checks;\n'
        '#define CK(x) do{checks++;if(!(x)){printf("FAIL line%u\\n",__LINE__);exit(1);}}while(0)\n'
        'int main(void){\n'+cases+'\nprintf("PASS MATLAB/C timing: %u cases\\n",checks);return 0;}')
    # Actual VSP_Prepare function, not a duplicate packet encoder.
    backend = (P/'CHASSIS/USER/vehicle_source_parallel.inc').read_text(encoding='utf-8')
    axis = backend[backend.index('typedef struct {uint32_t phase'):backend.index('static VSP_Axis')]
    prepare = backend[backend.index('static uint32_t VSP_Prepare('):backend.index('static uint32_t VSP_PositionWindow(')]
    prefix = r'''
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "source_parallel_flow.h"
#include "vehicle_parameters.h"
#include "source_continuous_descent_20261115.h"
static unsigned checks;
#define CK(x) do{checks++;if(!(x)){printf("FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
'''+axis+r'''
static VSP_Axis vsp_axis[2];
static struct {uint8_t Data[8];}s_rx;
static struct{uint32_t top_scale;}g_lift_ruler_controller;
static uint32_t vsp_fast_lift=1,vsp_dm_posted,vsp_dm_cycle,g_dm_can2_feedback_valid=1,g_dm_can2_feedback_tick;
static uint32_t g_dm_parallel_group_active,g_dm_parallel_group_go,g_dm_source_smooth,g_dm_point_enable;
static uint32_t g_dm_jog_request,g_dm_can2_read_cycle,g_dm_jog_first_fault,g_dm_jog_state,g_dm_jog_result,g_dm_posvel_holding,g_dm_can2_motor_state;
static uint32_t g_vsp_prepare_miss,g_vsp_prepare_wait_address,g_vsp_prepare_speed_raw,g_vsp_prepare_nonzero;
static float g_dm_posvel_speed_deg_s,g_dm_pick_target_deg,g_dm_place_target_deg,g_dm_can2_logical_position_deg,g_extension_home_reference_deg;
static uint32_t xTaskGetTickCount(void){return 1;}
static uint32_t low_active=1;
static uint32_t Low_Busy(void){return low_active;}
static float SingleTeach_TopDeg(void){return 0;}
static uint32_t VSP_Query(uint8_t a,uint8_t c,uint8_t n){(void)a;(void)c;(void)n;return 1;}
static uint32_t VSP_Position(uint8_t a,VSP_Axis *v){(void)a;(void)v;return 1;}
static uint32_t VehicleArm_Exchange(uint8_t a,uint8_t*q,uint32_t n,uint32_t count,void*reply){(void)a;(void)q;(void)n;(void)count;(void)reply;s_rx.Data[1]=2;return 1;}
'''
    # Include up to VSP_Prepare only (the source also contains position-window
    # helper before Start; stopping at its declaration preserves dependencies).
    tests = r'''
int main(void){SPF_Command c={0};VSP_Axis *z=&vsp_axis[1];unsigned speed;
 c.kind=SPF_CMD_PICK;c.axes_mask=6;c.goal.lift_pulses=29700;c.goal.extension_mm=162;
 c.lift_rpm=SPF_LIFT_RPM;c.extension_rpm=150;c.stepper_accel_rpm_s=600;
 vsp_axis[0].before=-65*315.5f*360/3200;vsp_axis[0].goal=-162*315.5f*360/3200;
 z->phase=1;z->current=600*360.0f/3200;CK(VSP_Prepare(0,4,&c)==0);
 speed=((unsigned)z->move[6]<<8)|z->move[7];CK(speed>0 && speed<18464);
 CK(z->move[2]==(45055>>8) && z->move[3]==(45055&255));
 CK(z->move[12]==2 && z->move[14]==0x6b);
 CK(speed==(unsigned)(SourcePick_ContinuousRPM(97*315.5f/3200,29100.0f/3200,15000.0f/3200)*10+.5f));
 low_active=0;z->phase=1;CK(VSP_Prepare(0,4,&c)==0);
 CK((((unsigned)z->move[6]<<8)|z->move[7])==18464);low_active=1;
 c.kind=SPF_CMD_TRANSFER;c.goal.lift_pulses=1800;z->phase=1;
 CK(VSP_Prepare(0,4,&c)==0);CK((((unsigned)z->move[6]<<8)|z->move[7])==18464);
 printf("PASS actual VSP prepare: %u checks; matched descent, original return rate/acceleration/payload\n",checks);return 0;}
'''
    run('actual-vsp-descent-prepare', prefix+prepare+tests)
    # Existing driver/poll/STOP checks for unchanged arrival and fault behaviour.
    checker = (R/'scripts/check_integration18_ext150_20261115.py').read_text()
    checker = checker.replace('SOURCE_EXTENSION_DONE_DEG==71', 'fabsf(SOURCE_EXTENSION_DONE_DEG-177.46875f)<.001f')
    checker = checker.replace('a->current=29;', 'a->current=100.0f-177.46875f+.01f;').replace(
        'a->current=28;', 'a->current=100.0f-177.46875f-.01f;')
    scope={'__name__':'continuous_arm_poll','__file__':str(R/'scripts/check_integration18_ext150_20261115.py')}
    exec(compile(checker,scope['__file__'],'exec'),scope)
    scope.update(P=P,E=E,EXPECTED_CORRIDOR_DEG=354.9375);scope['main']()
    source = (R/'scripts/build_integration18_ext150_20261115.py').read_text()
    source = source.replace("str(ROOT/'tests'/source)", "str(OWNER_TEST if name=='owner' else ROOT/'tests'/source)")
    scope={'__name__':'continuous_arm_build','__file__':build.__file__}
    exec(compile(source,build.__file__,'exec'),scope)
    scope.update(PROJECT=P,BASE=BASE,E=E,OWNER_TEST=E/'all-regions-owner.c');scope['main']()


if __name__ == '__main__':
    main()
