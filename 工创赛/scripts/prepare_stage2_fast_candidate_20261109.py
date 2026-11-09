"""Prepare/build a separate offline candidate. Never attach, reset or flash."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tmp/stage2_three_overlap_20261109/project'
OUT = ROOT / 'tmp/stage2_sub20_20261109'
PROJECT = OUT / 'project'
E = ROOT / 'evidence/integration/2026-11-07/stage2-mainline'


def replace(data, old, new):
    old, new = old.encode(), new.encode()
    assert data.count(old) == 1, old[:100]
    return data.replace(old, new, 1)


def main():
    assert json.loads((E / 'stage2-fast-gates-matlab-20261109.json').read_text())['passed']
    assert not PROJECT.exists(), 'Do not overwrite an existing candidate'
    assert hashlib.sha256((BASE / 'Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf').read_bytes()).hexdigest() == \
        '4aa68a090da08bcebeb28e3a8b6b57ee897a85b48ee7f1fd0c73eebb158d18dd'
    shutil.copytree(BASE, PROJECT, ignore=shutil.ignore_patterns('Objects', 'Listings', '*.uvguix.*', '*.bak'))
    user = PROJECT / 'CHASSIS/USER'
    for name in ('stage2_fast_cycle.h', 'stage2_fast_cycle.inc'):
        shutil.copy2(ROOT / 'software_interfaces/stage2_fast_cycle' / name, user / name)
    # Existing files are patched as bytes; preserve all untouched GBK and mixed
    # CRLF/LF content. New ASCII additions use the neighbouring newline style.
    p = user / 'source_parallel_flow.h'
    p.write_bytes(replace(p.read_bytes(),
        ' uint32_t first_start_ms,last_start_ms,max_start_skew_ms,timeout_ms;',
        ' uint32_t first_start_ms,last_start_ms,max_start_skew_ms,timeout_ms;\n'
        ' /* Opt-in position-gated release; legacy groups keep both zero. */\n'
        ' uint32_t delayed_mask,release_mask;'))
    p = user / 'source_parallel_flow.c'
    staged = '''
 if(g->delayed_mask){
  /* Prepare only released axes: preparing DM can queue a retarget. */
  for(bit=1U;bit<=4U;bit<<=1)if((g->command.axes_mask&bit) && !(g->started&bit) &&
     (g->release_mask&bit)){
   if(!(g->prepared&bit)){
    result=g->io.prepare(g->io.context,bit,&g->command);
    if(result==2U){SPF_GroupCancel(g,4U);return;}
    if(result==1U)g->prepared|=bit;
    break;
   }
   if(!g->started)g->first_start_ms=now;
   g->started|=bit;
   if(g->io.start(g->io.context,bit,&g->command)!=1U){SPF_GroupCancel(g,6U);return;}
   g->last_start_ms=g->io.clock_ms(g->io.context);g->state=2U;break;
  }
  /* Keep servicing moving axes while a later release waits for height. */
  for(bit=1U;bit<=4U;bit<<=1)if((g->started&bit) && !(g->done&bit)){
   result=g->io.poll(g->io.context,bit);
   if(result==2U){SPF_GroupCancel(g,7U);return;}
   if(result==1U)g->done|=bit;
  }
  if(g->done==g->command.axes_mask)g->state=3U;
  return;
 }
'''
    p.write_bytes(replace(p.read_bytes(), ' if(g->state==1){\n', staged + ' if(g->state==1){\n'))
    p = user / 'vehicle_source_parallel.inc'
    d = p.read_bytes()
    d = replace(d, 'static SPF_Group vsp_group;', 'static SPF_Group vsp_group;\nstatic uint32_t vsp_fast_lift;')
    d = replace(d, 'phase,options,samples,flags,cycle,stall_active,stall_ms;',
                'phase,options,samples,flags,cycle,stall_active,stall_ms,position_tick,position_cycle;')
    d = replace(d, ' a->current=(s_rx.Data[1]?-1.0f:1.0f)*(float)raw*0.1f;',
                ' a->current=(s_rx.Data[1]?-1.0f:1.0f)*(float)raw/\n'
                '  ((address==5U && (a->options&0x80U))?100.0f:10.0f);\n'
                ' a->position_tick=(uint32_t)xTaskGetTickCount();a->position_cycle++;')
    d = replace(d, '  a->options=s_rx.Data[2];a->phase=1;return 0;',
                '  a->options=s_rx.Data[2];\n'
                '  if(vsp_fast_lift && address==5U &&\n'
                '   ((a->options&0x80U)?100U:10U)!=g_lift_ruler_controller.top_scale)return 2;\n'
                '  a->phase=1;return 0;')
    d = replace(d, '  units=(uint32_t)(fabsf(delta)*10.0f+0.5f);',
                '  units=(uint32_t)(fabsf(delta)*((address==5U && (a->options&0x80U))?100.0f:10.0f)+0.5f);')
    d = replace(d, '  a->move[6]=(uint8_t)(speed>>8);a->move[7]=(uint8_t)speed;',
                '  if(vsp_fast_lift && address==5U){\n'
                '   a->move[2]=a->move[4]=(uint8_t)(45055U>>8);\n'
                '   a->move[3]=a->move[5]=(uint8_t)45055U;\n'
                '  }\n'
                '  a->move[6]=(uint8_t)(speed>>8);a->move[7]=(uint8_t)speed;')
    # Permit staged preparation while an earlier axis is already moving.
    d = replace(d, 'return vsp_group.state==1U && a->phase==3U;',
                'return (vsp_group.state==1U || (vsp_group.delayed_mask && vsp_group.state==2U)) && a->phase==3U;')
    d = replace(d, 'return n==15U && vsp_group.state==1U && (vsp_group.started&bit)',
                'return n==15U && (vsp_group.state==1U || (vsp_group.delayed_mask && vsp_group.state==2U)) && (vsp_group.started&bit)')
    d = replace(d, ' memset(vsp_axis,0,sizeof(vsp_axis));vsp_dm_posted=0;vsp_owned=1;',
                ' memset(vsp_axis,0,sizeof(vsp_axis));vsp_dm_posted=0;vsp_fast_lift=0U;vsp_owned=1;')
    d = replace(d, 'if(g_dm_posvel_holding && g_dm_jog_state==7U && g_dm_jog_result==1U)return 1;',
                'if(g_dm_posvel_holding && g_dm_jog_state==7U && g_dm_jog_result==1U && !g_dm_jog_request)return 1;')
    d = replace(d, 'if(!g_dm_jog_request)g_dm_jog_request=3;return 0;',
                'if(g_dm_jog_request!=3U)g_dm_jog_request=3U;return 0;')
    p.write_bytes(d)
    p = user / 'vehicle_arm.c'
    p.write_bytes(replace(p.read_bytes(), '#include "integration10_complete.inc"\r\n',
                         '#include "integration10_complete.inc"\r\n#include "stage2_fast_cycle.inc"\r\n'))
    p = user / 'integration10_complete.inc'
    d = p.read_bytes()
    d = replace(d, '#include "usart.h"\r\n', '#include "usart.h"\r\n#include "stage2_fast_cycle.h"\r\n')
    d = replace(d, 'if(request==9U){SourceStage2_Cancel(2U);',
                'if(request==9U){Stage2Fast_Cancel(2U);SourceStage2_Cancel(2U);')
    d = replace(d, 'if(SourceStage2_IsActive() || g_it_state==1U',
                'if(Stage2Fast_IsActive() || SourceStage2_IsActive() || g_it_state==1U')
    d = replace(d, ' SourceStage2_Service(now);\r\n',
                ' SourceStage2_Service(now);\r\n Stage2Fast_Service((uint32_t)xTaskGetTickCount());\r\n')
    p.write_bytes(d)
    manifest = {'base': BASE.relative_to(ROOT).as_posix(), 'matlab_passed': True,
                'deployed': False, 'hardware_motion': False, 'under20_demonstrated': False,
                'default_start_provider': None, 'images': {}, 'changed': {}}
    for p in user.iterdir():
        old = BASE / 'CHASSIS/USER' / p.name
        if p.is_file() and (not old.exists() or p.read_bytes() != old.read_bytes()):
            manifest['changed'][p.name] = hashlib.sha256(p.read_bytes()).hexdigest()
    (E / 'stage2-fast-candidate-20261109.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(PROJECT, flush=True)


if __name__ == '__main__':
    main()
