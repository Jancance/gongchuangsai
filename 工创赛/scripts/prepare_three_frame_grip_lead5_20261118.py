"""Restore region-latched existing lead5 preclose, no new waiting."""
from pathlib import Path
import shutil,json
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/three_frame_initial5_20261118/project'
P=R/'tmp/three_frame_grip_lead5_20261118/project'
E=R/'evidence/integration/2026-11-18/three-frame-grip-lead5'

def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed']
 assert not P.exists()
 shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 p=P/'CHASSIS/USER/pzq_commit_overlap.inc';b=p.read_bytes()
 old=b' /* Close at the completed selected pick pose, not 5mm before it. */'
 new=''' /* Restore the prior live-feedback lead5 close for this latched pick depth. */
 if(g_low_phase==4U && !g_it_lead5_grip_done && g_source_parallel_state!=4U &&
    vsp_group.command.sequence==low_group_sequence &&
    (vsp_group.command.axes_mask&SPF_LIFT) && (vsp_group.started&SPF_LIFT) &&
    vsp_axis[1].position_cycle && (uint32_t)(now-vsp_axis[1].position_tick)<=150U &&
    (vsp_axis[1].current-SingleTeach_TopDeg())*(3200.0f/360.0f)>=low_pick_depth-5.0f*SOURCE_LIFT_PULSES_PER_MM){
  if(jiazhua(140.0f)!=1U){Low_Fail(7U);return;}
  g_low_grip_command_ms=(uint32_t)xTaskGetTickCount()-low_begin;g_it_lead5_grip_done=1U;
 }'''.replace('\n','\r\n').encode('ascii')
 assert b.count(old)==1;b=b.replace(old,new,1)
 old=b'  if(jiazhua(140.0f)!=1U){Low_Fail(7U);return;}\r\n  g_low_grip_command_ms=(uint32_t)xTaskGetTickCount()-low_begin;g_it_lead5_grip_done=1U;\r\n  g_low_lift_retract_axes='
 new=b'  if(!g_it_lead5_grip_done){\r\n   if(jiazhua(140.0f)!=1U){Low_Fail(7U);return;}\r\n   g_low_grip_command_ms=(uint32_t)xTaskGetTickCount()-low_begin;g_it_lead5_grip_done=1U;\r\n  }\r\n  g_low_lift_retract_axes='
 assert b.count(old)==1;b=b.replace(old,new,1);p.write_bytes(b)
 print('PASS independent candidate prepared: only pzq_commit_overlap.inc')
if __name__=='__main__':main()
