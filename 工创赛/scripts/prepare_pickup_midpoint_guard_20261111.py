"""Prepare a guarded midpoint descent; preserve200/600 extension and2500/45055 lift."""
from pathlib import Path
import json
import shutil

R = Path(__file__).resolve().parents[1]
B = R / 'tmp/pickup_driver_fault_read_20261111/project'
P = R / 'tmp/pickup_midpoint_guard_20261111/project'
E = R / 'evidence/integration/2026-11-11/midpoint-guard'


def edit(relative, old, new):
    p = P / relative
    data = p.read_bytes()
    old, new = old.encode('utf-8'), new.encode('utf-8')
    assert data.count(old) == 1, (relative, old[:100])
    p.write_bytes(data.replace(old, new, 1))


def main():
    assert not P.exists()
    E.mkdir(parents=True, exist_ok=True)
    shutil.copytree(B, P, ignore=shutil.ignore_patterns('Objects', 'Listings'))
    guard = r'''/* Only profile2 side pickup. No independent task or CAN owner.
 * 半程高度先保持，伸缩真实到位后再下降；抓取终点不增加固定停顿。 */
enum {PMG_OFF,PMG_TO_MID,PMG_WAIT_EXT,PMG_PREP_FINAL,PMG_TO_FINAL};
static uint32_t vsp_pick_gate_enabled;
volatile uint32_t g_pick_midpoint_stage,g_pick_midpoint_ready_tick,g_pick_midpoint_final_start_tick;
volatile float g_pick_midpoint_before_deg,g_pick_midpoint_hold_deg,g_pick_midpoint_final_deg;
static uint32_t VSP_Prepare(void *,uint32_t,const SPF_Command *);
static uint32_t VSP_Start(void *,uint32_t,const SPF_Command *);
static void PMG_Reset(void)
{
 vsp_pick_gate_enabled=0U;g_pick_midpoint_stage=PMG_OFF;
 g_pick_midpoint_ready_tick=g_pick_midpoint_final_start_tick=0U;
 g_pick_midpoint_before_deg=g_pick_midpoint_hold_deg=g_pick_midpoint_final_deg=0.0f;
}
static float PMG_LiftGoal(VSP_Axis *z,float final_goal)
{
 if(!vsp_pick_gate_enabled)return final_goal;
 if(g_pick_midpoint_stage==PMG_OFF){
  float span=final_goal-z->before;
  if(!(span>0.0f && span<=(float)SPF_MAX_LIFT_PULSES*360.0f/3200.0f+0.1f))return NAN;
  g_pick_midpoint_before_deg=z->before;g_pick_midpoint_final_deg=final_goal;
  g_pick_midpoint_hold_deg=z->before+(final_goal-z->before)*0.5f;
  g_pick_midpoint_stage=PMG_TO_MID;
 }
 return g_pick_midpoint_stage<=PMG_WAIT_EXT?g_pick_midpoint_hold_deg:final_goal;
}
static uint32_t PMG_LiftCompletion(uint32_t completed)
{
 if(!vsp_pick_gate_enabled)return completed;
 if(g_pick_midpoint_stage==PMG_TO_MID && completed){
  g_pick_midpoint_stage=PMG_WAIT_EXT;g_pick_midpoint_ready_tick=(uint32_t)xTaskGetTickCount();
 }
 /* Midpoint is never reported as the final lift completion. */
 return g_pick_midpoint_stage==PMG_TO_FINAL?completed:0U;
}
static uint32_t PMG_Service(void)
{
 VSP_Axis *z=&vsp_axis[1];uint32_t r,now=(uint32_t)xTaskGetTickCount();
 if(!vsp_pick_gate_enabled || vsp_group.state!=2U ||
    vsp_group.command.kind!=SPF_CMD_PICK)return 1U;
 if(g_pick_midpoint_stage==PMG_WAIT_EXT){
  if(!(vsp_group.done&SPF_EXT))return 1U;
  if(!z->position_cycle || now-z->position_tick>150U || z->samples<3U)return 1U;
  if((z->flags&0x0fU)!=3U || !(fabsf(z->current-g_pick_midpoint_hold_deg)<=60.0f))return 0U;
  z->phase=z->samples=z->cycle=z->stall_active=z->overshoot_active=0U;
  g_pick_midpoint_stage=PMG_PREP_FINAL;
 }
 if(g_pick_midpoint_stage!=PMG_PREP_FINAL)return 1U;
 r=VSP_Prepare(0,SPF_LIFT,&vsp_group.command);
 if(r==2U)return 0U;
 if(r!=1U)return 1U;
 /* Prepare refreshed relative origin and confirmed zero speed again. */
 if(!z->position_cycle || (uint32_t)xTaskGetTickCount()-z->position_tick>150U ||
    !(fabsf(z->before-g_pick_midpoint_hold_deg)<=60.0f) ||
    !(fabsf(z->goal-g_pick_midpoint_final_deg)<=0.1f))return 0U;
 if(VSP_Start(0,SPF_LIFT,&vsp_group.command)!=1U)return 0U;
 g_pick_midpoint_stage=PMG_TO_FINAL;
 g_pick_midpoint_final_start_tick=(uint32_t)xTaskGetTickCount();return 1U;
}
'''
    (P / 'CHASSIS/USER/pickup_midpoint_guard.inc').write_bytes(guard.replace('\n', '\r\n').encode('utf-8'))
    v = 'CHASSIS/USER/vehicle_source_parallel.inc'
    edit(v, '#include "pickup_s_curve_adapter.inc"', '#include "pickup_s_curve_adapter.inc"\r\n#include "pickup_midpoint_guard.inc"')
    edit(v, '  delta=a->goal-a->current;', '  if(address==5U)a->goal=PMG_LiftGoal(a,a->goal);\r\n  delta=a->goal-a->current;')
    edit(v, ' zero_return=(address==6U && vsp_group.command.goal.extension_mm==0.0f);', ' zero_return=(address==6U && vsp_group.command.goal.extension_mm==0.0f);\r\n if(address==5U && vsp_pick_gate_enabled && g_pick_midpoint_stage==PMG_PREP_FINAL)return 0U;')
    edit(v, ' a->phase=0;return a->samples>=3;}', ' a->phase=0;return address==5U?PMG_LiftCompletion(a->samples>=3U):(a->samples>=3U);}')
    edit(v, ' if(!VSP_SCurveService())SPF_GroupCancel(&vsp_group,9U);', ' if(!VSP_SCurveService())SPF_GroupCancel(&vsp_group,9U);\r\n if(!PMG_Service())SPF_GroupCancel(&vsp_group,7U);')
    edit(v, ' memset(vsp_axis,0,sizeof(vsp_axis));vsp_dm_posted=0;vsp_fast_lift=0U;vsp_owned=1;', ' memset(vsp_axis,0,sizeof(vsp_axis));PMG_Reset();vsp_dm_posted=0;vsp_fast_lift=0U;vsp_owned=1;')
    it = 'CHASSIS/USER/integration10_complete.inc'
    edit(it, ' vsp_fast_lift=1U;g_it_carrier_auto_done=0U;g_it_lead5_grip_done=0U;', ' vsp_fast_lift=1U;g_it_carrier_auto_done=0U;g_it_lead5_grip_done=0U;\r\n /* 第二档侧件：保持原速度，在半程高度等伸缩完成，禁止低位横推。 */\r\n vsp_pick_gate_enabled=(g_pick_trial_profile==2U && (it_command.axes_mask&SPF_EXT))?1U:0U;')
    marker = ' span=z->goal-z->before;'
    addition = r''' if(vsp_pick_gate_enabled){
  if(g_pick_midpoint_stage!=PMG_WAIT_EXT || z->samples<3U || (z->flags&0x0fU)!=3U)return;
  if(!(fabsf(z->current-g_pick_midpoint_hold_deg)<=45.0f)){it_fail(8U);return;}
  g_it_half_lift_before=g_pick_midpoint_before_deg;g_it_half_lift_goal=g_pick_midpoint_final_deg;
  g_it_half_lift_at_release=z->current;g_it_half_extension_release_tick=now;
  vsp_group.release_mask|=SPF_EXT;g_it_half_extension_released=1U;return;
 }
'''
    edit(it, marker, addition.replace('\n', '\r\n') + marker)
    (E / 'scope.json').write_text(json.dumps(dict(base=B.relative_to(R).as_posix(), project=P.relative_to(R).as_posix(),
        profile=2, extension_rpm=200, extension_accel_decel=600, lift_rpm=2500, lift_accel_decel=45055,
        extension_one_native_move=True, midpoint_hold_until_extension_done=True,
        no_boundary_or_fault_clear_change=True, hardware_access=False, deployed=False, physical_stall_resolved=False), indent=2)+'\n', encoding='utf-8')
    build = (R / 'scripts/build_pickup_driver_fault_read_20261111.py').read_text(encoding='utf-8')
    build = build.replace('tmp/pickup_driver_fault_read_20261111/project', 'tmp/pickup_midpoint_guard_20261111/project')
    build = build.replace('2026-11-11/driver-fault-read', '2026-11-11/midpoint-guard')
    build = build.replace('driver_fault_readonly_admission_fix', 'pickup_midpoint_hold_until_extension_done')
    build = build.replace('tmp/pickup_driver_fifo_fix_20261111/project', 'tmp/pickup_driver_fault_read_20261111/project')
    build = build.replace('9641f3d692f167da3396a7633a744b689536a856ab5467e274e823aa8225eab1', '81801798f81df0a4e358a08730248e0754ba729c63862eb17ac6c81fb98285df')
    (R / 'scripts/build_pickup_midpoint_guard_20261111.py').write_text(build, encoding='utf-8')
    print('Prepared profile2 midpoint guard; no hardware access.')


if __name__ == '__main__':
    main()
