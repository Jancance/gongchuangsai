"""MATLAB-gated handoff proof; preserve cb425232 and all non-scoped limits."""
from pathlib import Path
import json,shutil,hashlib
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/three_frame_warm_20261118/project'
P=R/'tmp/three_frame_handoff_20261118/project'
E=R/'evidence/integration/2026-11-18/three-frame-handoff'

def patch(name,edits):
    p=P/name;data=p.read_bytes()
    for old,new in edits:
        a=old.encode('ascii');b=new.encode('ascii')
        assert data.count(a)==1,(name,old[:90],data.count(a))
        data=data.replace(a,b,1)
    p.write_bytes(data)

def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    assert not P.exists(),'Do not overwrite an existing candidate'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    patch('HARDWARE/DM_CAN2/dm_can2_motion.c',[
      ('volatile uint32_t g_dm_stage2_sync=0U;','volatile uint32_t g_dm_stage2_sync=0U;\r\nvolatile uint32_t g_dm_initial_observe_handoff=0U;\r\nstatic uint32_t s_startup_hold_valid;\r\nstatic float s_startup_hold_target;'),
      ('static void JogStop(uint32_t reason, TickType_t now)\r\n{\r\n    CanTxMsg tx;',
       'static void JogStop(uint32_t reason, TickType_t now)\r\n{\r\n    CanTxMsg tx;\r\n    if(reason>2U){s_startup_hold_valid=0U;g_dm_initial_observe_handoff=0U;}'),
      ('static uint32_t JogFastRetarget(uint32_t request, TickType_t now)',
       '''/* One completed request6 target may hand off to exact initial180/65 only.
 * Reuse its existing 3-degree completion relation; other starts keep -2.
 * Raw pmax, direction, freshness and all motion corridor checks still apply. */
static uint32_t JogInitialObservePending(float goal)
{
    return g_dm_initial_observe_handoff && g_dm_stage2_sync &&
        g_dm_parallel_group_active && !g_dm_parallel_group_go &&
        g_dm_integrated_owner && goal==180.0F && s_startup_hold_valid;
}
static uint32_t JogPointStartAllowed(float raw, float work, float goal)
{
    if(work>=-2.0F && work<=(g_dm_parallel_group_active?272.0F:182.0F))return 1U;
    return JogInitialObservePending(goal) && work<=272.0F &&
        JogAbs(raw-s_startup_hold_target)<=3.0F*JOG_RAD_PER_DEG;
}
static uint32_t JogFastRetarget(uint32_t request, TickType_t now)'''.replace('\n','\r\n')),
      ('if (!(current_work_deg >= -2.0F && current_work_deg <= (g_dm_parallel_group_active ? 272.0F : 182.0F)))',
       'if (!JogPointStartAllowed(g_dm_can2_position_rad,current_work_deg,goal))'),
      ('s_target = s_start + delta_deg * JOG_RAD_PER_DEG;\r\n    if (!(s_start',
       's_target = s_start + delta_deg * JOG_RAD_PER_DEG;\r\n    s_startup_hold_valid=0U; /* New target locked; no reuse by another action. */\r\n    if (!(s_start'),
      ('g_dm_jog_request = 0U;\r\n    if (request == 3U) s_queued_request = 0U;',
       '''g_dm_jog_request = 0U;
    if(g_dm_parallel_group_go){s_startup_hold_valid=0U;g_dm_initial_observe_handoff=0U;}
    if(request==3U || request==6U ||
       (request && !((request==4U || request==5U) &&
          JogInitialObservePending(request==4U?g_dm_pick_target_deg:g_dm_place_target_deg)))){
        s_startup_hold_valid=0U;g_dm_initial_observe_handoff=0U;
    }
    if (request == 3U) s_queued_request = 0U;'''.replace('\n','\r\n')),
      ('if (!(current_work_deg >= -2.0F && current_work_deg <= (g_dm_parallel_group_active?272.0F:182.0F)) ||',
       'if (!JogPointStartAllowed(s_start,current_work_deg,s_point_goal) ||'),
      ('s_target = s_start + delta_deg * JOG_RAD_PER_DEG;\r\n        }',
       's_target = s_start + delta_deg * JOG_RAD_PER_DEG;\r\n            s_startup_hold_valid=0U;\r\n        }'),
      ('g_dm_posvel_holding = 1U; JogReference(s_target, now); return;',
       'g_dm_posvel_holding = 1U;\r\n            if(s_point_mode==6U){s_startup_hold_target=s_target;s_startup_hold_valid=1U;}\r\n            JogReference(s_target, now); return;'),
    ])
    patch('CHASSIS/USER/vehicle_source_parallel.inc',[
      ('extern volatile uint32_t g_dm_parallel_group_active,g_dm_parallel_group_go,g_dm_stage2_sync;',
       'extern volatile uint32_t g_dm_parallel_group_active,g_dm_parallel_group_go,g_dm_stage2_sync;\r\nextern volatile uint32_t g_dm_initial_observe_handoff;'),
      ('if(g_dm_stage2_sync)g_dm_parallel_group_go=1U;',
       'if(g_dm_stage2_sync){g_dm_parallel_group_go=1U;g_dm_initial_observe_handoff=0U;}'),
      ('static void VehicleSourceParallel_Cancel(void){if(vsp_owned)SPF_GroupCancel(&vsp_group,2);}',
       'static void VehicleSourceParallel_Cancel(void){g_dm_initial_observe_handoff=0U;if(vsp_owned)SPF_GroupCancel(&vsp_group,2);}'),
      ('g_dm_stage2_sync=g_dm_parallel_group_active=g_dm_parallel_group_go=0;\r\n  g_vehicle_arm_axis_owned',
       'g_dm_initial_observe_handoff=g_dm_stage2_sync=g_dm_parallel_group_active=g_dm_parallel_group_go=0;\r\n  g_vehicle_arm_axis_owned'),
      ('g_vehicle_arm_state=3;g_vehicle_arm_cause=vsp_group.fault;',
       'g_dm_initial_observe_handoff=0U;g_vehicle_arm_state=3;g_vehicle_arm_cause=vsp_group.fault;'),
      ('vsp_pick_sync=vsp_pick_queued=g_source_parallel_sync_tick=g_dm_stage2_sync=0U;',
       'vsp_pick_sync=vsp_pick_queued=g_source_parallel_sync_tick=g_dm_initial_observe_handoff=g_dm_stage2_sync=0U;'),
    ])
    patch('CHASSIS/USER/integration10_complete.inc',[
      ('it_command.goal.extension_mm==65.0f){vsp_pick_sync=1U;g_dm_stage2_sync=1U;}',
       'it_command.goal.extension_mm==65.0f){vsp_pick_sync=1U;g_dm_stage2_sync=1U;g_dm_initial_observe_handoff=1U;}'),
    ])
    print('Prepared independent candidate after MATLAB; three localized files')

if __name__=='__main__':main()
