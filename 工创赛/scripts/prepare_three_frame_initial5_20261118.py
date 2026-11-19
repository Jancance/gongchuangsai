"""Authorized initial -5 only, after MATLAB, from frozen cb425232 source."""
from pathlib import Path
import json,shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/three_frame_warm_20261118/project'
P=R/'tmp/three_frame_initial5_20261118/project'
E=R/'evidence/integration/2026-11-18/three-frame-initial5'

def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed']
 assert not P.exists()
 shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 def patch(name,edits):
  p=P/name;b=p.read_bytes()
  for old,new in edits:
   a=old.encode('utf-8');z=new.encode('utf-8')
   assert b.count(a)==1,(name,old[:80],b.count(a))
   b=b.replace(a,z,1)
  p.write_bytes(b)
 patch('USER/vehicle_parameters.h',[
  ('#define VEHICLE_SOURCE_PREP_DM_DEG 180.0f',
   '#define SOURCE_INITIAL_DM_START_MIN_DEG (-5.0f) /* 用户2026-11-18确认：仅首次180度/65mm并行起点下限，后续仍为-2度。 */\r\n#define VEHICLE_SOURCE_PREP_DM_DEG 180.0f')])
 patch('HARDWARE/DM_CAN2/dm_can2_motion.c',[
  ('volatile uint32_t g_dm_stage2_sync=0U;',
   'volatile uint32_t g_dm_stage2_sync=0U;\r\nvolatile uint32_t g_dm_initial_observe=0U;'),
  ('static void JogStop(uint32_t reason, TickType_t now)\r\n{\r\n    CanTxMsg tx;',
   'static void JogStop(uint32_t reason, TickType_t now)\r\n{\r\n    CanTxMsg tx;\r\n    if(reason>2U)g_dm_initial_observe=0U;'),
  ('if (request == 3U) s_queued_request = 0U;',
   'if (request == 3U) {s_queued_request = 0U;g_dm_initial_observe=0U;}'),
  ('static uint32_t JogFastRetarget(uint32_t request, TickType_t now)',
   '''/* Only the exact first180/65 owner may use the user-approved -5 start.
 * Later commands retain -2; raw range, direction and motion checks stay intact. */
static uint32_t JogPointStartAllowed(float work, float goal)
{
    float lower=-2.0F;
    if(g_dm_initial_observe && g_dm_stage2_sync && g_dm_parallel_group_active &&
       !g_dm_parallel_group_go && goal==180.0F)lower=SOURCE_INITIAL_DM_START_MIN_DEG;
    return work>=lower && work<=(g_dm_parallel_group_active?272.0F:182.0F);
}
static uint32_t JogFastRetarget(uint32_t request, TickType_t now)'''.replace('\n','\r\n')),
  ('if (!(current_work_deg >= -2.0F && current_work_deg <= (g_dm_parallel_group_active ? 272.0F : 182.0F)))',
   'if (!JogPointStartAllowed(current_work_deg,goal))'),
  ('if (!(current_work_deg >= -2.0F && current_work_deg <= (g_dm_parallel_group_active?272.0F:182.0F)) ||',
   'if (!JogPointStartAllowed(current_work_deg,s_point_goal) ||'),
 ])
 p=P/'CHASSIS/USER/vehicle_source_parallel.inc';b=p.read_bytes()
 a=b'extern volatile uint32_t g_dm_parallel_group_active,g_dm_parallel_group_go,g_dm_stage2_sync;'
 assert b.count(a)==1;b=b.replace(a,a+b'\r\nextern volatile uint32_t g_dm_initial_observe;',1)
 a=b'g_dm_stage2_sync=g_dm_parallel_group_active=g_dm_parallel_group_go=0;'
 assert b.count(a)==2;b=b.replace(a,b'g_dm_initial_observe='+a)
 a=b'vsp_pick_sync=vsp_pick_queued=g_source_parallel_sync_tick=g_dm_stage2_sync=0U;'
 assert b.count(a)==1;b=b.replace(a,b'vsp_pick_sync=vsp_pick_queued=g_source_parallel_sync_tick=g_dm_initial_observe=g_dm_stage2_sync=0U;',1)
 a=b'static void VehicleSourceParallel_Cancel(void){if(vsp_owned)SPF_GroupCancel(&vsp_group,2);}'
 assert b.count(a)==1;b=b.replace(a,b'static void VehicleSourceParallel_Cancel(void){g_dm_initial_observe=0U;if(vsp_owned)SPF_GroupCancel(&vsp_group,2);}',1)
 a=b'if(g_dm_stage2_sync)g_dm_parallel_group_go=1U;'
 assert b.count(a)==1;b=b.replace(a,b'if(g_dm_stage2_sync){g_dm_parallel_group_go=1U;g_dm_initial_observe=0U;}',1)
 a=b'g_vehicle_arm_state=3;g_vehicle_arm_cause=vsp_group.fault;'
 assert b.count(a)==1;b=b.replace(a,b'g_dm_initial_observe=0U;'+a,1)
 p.write_bytes(b)
 patch('CHASSIS/USER/integration10_complete.inc',[
  ('it_command.goal.extension_mm==65.0f){vsp_pick_sync=1U;g_dm_stage2_sync=1U;}',
   'it_command.goal.extension_mm==65.0f){vsp_pick_sync=1U;g_dm_stage2_sync=1U;g_dm_initial_observe=SC_InitialObserveAllowed();}')])
 print('PASS candidate prepared: exact four-file delta from cb; no startup/reference changes')

if __name__=='__main__':main()
