"""Authorized Stage2-only delta after the local MATLAB contract passes."""
from pathlib import Path
import shutil,json
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/stage3_parallel_preclose_20261116/project'
P=R/'tmp/stage2_sync_opt_20261117/project'
E=R/'evidence/integration/2026-11-17/stage2-sync-opt'
def edit(name,pairs):
 b=(P/name).read_bytes()
 for old,new in pairs:
  old=old.replace('\n','\r\n').encode('ascii');new=new.replace('\n','\r\n').encode('ascii')
  assert b.count(old)==1,(name,old,b.count(old));b=b.replace(old,new)
 (P/name).write_bytes(b)
def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed']
 assert not P.exists(),'Preserve candidate'
 shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 edit('HARDWARE/DM_CAN2/dm_can2_motion.c',[
 ('volatile uint32_t g_dm_parallel_group_active=0U,g_dm_parallel_group_go=0U;',
  'volatile uint32_t g_dm_parallel_group_active=0U,g_dm_parallel_group_go=0U;\nvolatile uint32_t g_dm_stage2_sync=0U;'),
 ('    g_dm_fast_retarget_count++;\n    if (!DM_CAN2_BuildPosVel(s_target, s_speed, &tx)',
  '    g_dm_fast_retarget_count++;\n    /* Stage2 caches the real goal; existing state3 holds the current position. */\n    if(g_dm_stage2_sync && g_dm_parallel_group_active && !g_dm_parallel_group_go){\n        s_enable_possible=1U;g_dm_jog_state=3U;JogReference(s_start,now);return 1U;\n    }\n    if (!DM_CAN2_BuildPosVel(s_target, s_speed, &tx)'),
])
 edit('CHASSIS/USER/vehicle_source_parallel.inc',[
 ('  if(g_dm_jog_request!=3U)g_dm_jog_request=3U;return 0;', '  if(g_dm_stage2_sync && g_dm_jog_state==6U)return 0; /* Await the existing in-flight STOP response. */\n  if(g_dm_jog_request!=3U)g_dm_jog_request=3U;return 0;'),
 ('static uint32_t vsp_dm_posted,vsp_dm_cycle,vsp_owned;', 'static uint32_t vsp_dm_posted,vsp_dm_cycle,vsp_owned,vsp_dm_hold_tick;'),
 ('vsp_dm_cycle=g_dm_can2_read_cycle;vsp_dm_posted=1;return 0;', 'vsp_dm_cycle=g_dm_can2_read_cycle;vsp_dm_hold_tick=g_dm_can2_feedback_tick;vsp_dm_posted=1;return 0;'),
 ('  return g_dm_jog_state==3 && g_dm_can2_motor_state==1 &&', '  if(g_dm_stage2_sync && g_dm_jog_state==3U && g_dm_can2_motor_state==1U &&\n     g_dm_can2_feedback_tick!=vsp_dm_hold_tick && (uint32_t)xTaskGetTickCount()-g_dm_can2_feedback_tick<100U)return 1U;\n  return g_dm_jog_state==3 && g_dm_can2_motor_state==1 &&'),
 ('extern volatile uint32_t g_dm_parallel_group_active,g_dm_parallel_group_go;',
  'extern volatile uint32_t g_dm_parallel_group_active,g_dm_parallel_group_go,g_dm_stage2_sync;'),
 ('  g_source_parallel_sync_tick=(uint32_t)xTaskGetTickCount();',
  '  g_source_parallel_sync_tick=(uint32_t)xTaskGetTickCount();\n  if(g_dm_stage2_sync)g_dm_parallel_group_go=1U;'),
 (' if(bit==SPF_DM){g_dm_parallel_group_go=1;return 1;}',
  ' if(bit==SPF_DM){if(!g_dm_stage2_sync)g_dm_parallel_group_go=1;return 1;}'),
 (' vsp_pick_sync=vsp_pick_queued=g_source_parallel_sync_tick=0U;',
  ' vsp_pick_sync=vsp_pick_queued=g_source_parallel_sync_tick=g_dm_stage2_sync=0U;'),
 (' if(address==0U)return vsp_pick_sync && vsp_pick_queued==6U &&\n  vsp_group.state==1U && vsp_group.started==6U && n==3U &&',
  ''' if(address==0U)return vsp_pick_sync && vsp_pick_queued==6U &&
  ((vsp_group.state==1U && vsp_group.started==vsp_group.command.axes_mask) ||
   (vsp_group.state==2U && vsp_group.delayed_mask==SPF_DM && vsp_group.started==6U)) && n==3U &&''')])
 # Clear scope only on genuine group ownership release, including STOP completion.
 b=(P/'CHASSIS/USER/vehicle_source_parallel.inc').read_bytes()
 b=b.replace(b'g_dm_parallel_group_active=g_dm_parallel_group_go=0;',b'g_dm_stage2_sync=g_dm_parallel_group_active=g_dm_parallel_group_go=0;')
 (P/'CHASSIS/USER/vehicle_source_parallel.inc').write_bytes(b)
 edit('CHASSIS/USER/pzq_commit_overlap.inc',[
 (''' if(phase==4U && axes==(SPF_EXT|SPF_LIFT)){
  if(g_low_region<2U){vsp_pick_sync=1U;}else{
   vsp_pick_lead_ms=SourcePick_ExtensionLeadMs(g_low_region);
   vsp_group.delayed_mask=SPF_LIFT;vsp_group.release_mask=SPF_EXT;
  }
 }''', ''' /* Explicit Stage2 scope only: two steppers cache before one broadcast.
  * Loaded transfer retains the existing delayed-DM clearance gate. */
 if(phase==4U || phase==5U || (phase==7U && low_slot<3U)){
  vsp_pick_sync=((axes&(SPF_EXT|SPF_LIFT))==(SPF_EXT|SPF_LIFT));
  g_dm_stage2_sync=vsp_pick_sync && phase!=5U && (axes&SPF_DM);
 }'''),
 (' g_low_pick_overlap_axes=SPF_EXT|SPF_LIFT;\n return Low_Group(4U,SPF_EXT|SPF_LIFT,low_yaw,low_extension,(float)SOURCE_PICK_PULSES);',
  ' g_low_pick_overlap_axes=SPF_EXT|SPF_LIFT|(g_low_region<2U?SPF_DM:0U);\n return Low_Group(4U,g_low_pick_overlap_axes,low_yaw,low_extension,(float)SOURCE_PICK_PULSES);'),
 (' if(g_low_region==2U){if(!Low_EmptyApproach())Low_Fail(8U);}\n else if(!Low_Group(1U,SPF_DM,low_yaw,65.0f,depth))Low_Fail(8U);',
  ' if(!Low_EmptyApproach())Low_Fail(8U);'),
 ('   low_release_applied=1U;low_pulse_tick=now;g_low_release_pulse_tick=now;g_low_release_ms=now-low_begin;',
  '   now=(uint32_t)xTaskGetTickCount();\n   low_release_applied=1U;low_pulse_tick=now;g_low_release_pulse_tick=now;g_low_release_ms=now-low_begin;'),
 ('  g_low_phase=12U; /* All axes of the original group are already done. */\n  break;\n case 12U:',
  '  g_low_phase=12U; /* All axes done: fall through to the real tray window. */\n  /* fall through */\n case 12U:')])
 edit('USER/vehicle_parameters.h',[('#define SOURCE_RELEASE_DWELL_MS 300U','#define SOURCE_RELEASE_DWELL_MS 100U')])
 print('Prepared Stage2 candidate after MATLAB; no hardware access.')
if __name__=='__main__':main()
