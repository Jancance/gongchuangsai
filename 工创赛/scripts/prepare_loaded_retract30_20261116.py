"""Isolated loaded-transfer scheduling delta; no installation or hardware IO."""
from pathlib import Path
import hashlib,json,shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/source_observe210_tray36p5_20261116/project'
P=R/'tmp/source_loaded_retract30_20261116/project'
E=R/'evidence/integration/2026-11-16/loaded-retract30-gate'

GATE='''/* This runs in the existing sole CAN owner, never a second reader/task.
 * DM prepare can retarget immediately: do not release it before actual gates. */
static void Low_ReleaseLoadedTransfer(uint32_t now)
{
#if !APP_MOTOR_OFFLINE_ONLY
 VSP_Axis *z=&vsp_axis[1],*x=&vsp_axis[0];float depth;
 if(!Low_Loaded() || g_low_phase!=5U || !VehicleSourceParallel_Owns() ||
    vsp_group.command.sequence!=low_group_sequence || vsp_group.state!=2U ||
    vsp_group.delayed_mask!=SPF_DM || (vsp_group.release_mask&SPF_DM) ||
    !(vsp_group.done&SPF_LIFT) || !SingleTeach_ReferencesReady())return;
 /* SPF stops polling completed axes. Refresh only those, without restarting
  * them; an absent EXT axis still needs actual home feedback in this group. */
 if(!z->position_cycle || (uint32_t)(now-z->position_tick)>150U){
  if(!VSP_Position(5U,z))return;
 }
 now=(uint32_t)xTaskGetTickCount();
 if(!(vsp_group.command.axes_mask&SPF_EXT) || (vsp_group.done&SPF_EXT)){
  if(!x->position_cycle || (uint32_t)(now-x->position_tick)>150U){
   if(!VSP_Position(6U,x))return;
  }
 }
 now=(uint32_t)xTaskGetTickCount();
 if(!z->position_cycle || !x->position_cycle ||
    (uint32_t)(now-z->position_tick)>150U || (uint32_t)(now-x->position_tick)>150U)return;
 depth=(z->current-SingleTeach_TopDeg())*(3200.0f/360.0f);
 if(!(depth>=0.0f && fabsf(depth-(float)SOURCE_RETURN_PULSES)<=SOURCE_TOP_TOLERANCE_DEG*(3200.0f/360.0f)) ||
    !(fabsf(x->current-g_extension_home_reference_deg)<=30.0f*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f))return;
 if(Low_TransferGate(now,depth,1U))vsp_group.release_mask|=SPF_DM;
#else
 (void)now;
#endif
}

'''

def main():
 proof=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))
 assert proof['passed'] and proof['local_matlab'] and proof['checks']==5533
 assert not P.exists(),'Preserve previous candidate'
 shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 path=P/'CHASSIS/USER/pzq_commit_overlap.inc';old=path.read_bytes();data=old
 def replace(a,b):
  nonlocal data
  a=a.replace('\n','\r\n').encode();b=b.replace('\n','\r\n').encode()
  assert data.count(a)==1,a
  data=data.replace(a,b,1)
 replace('static void Low_Fail(uint32_t why)',GATE+'static void Low_Fail(uint32_t why)')
 replace(' vsp_fast_lift=(axes&SPF_LIFT)?1U:0U;', ' vsp_fast_lift=(axes&SPF_LIFT)?1U:0U;\n if(phase==5U){vsp_group.delayed_mask=SPF_DM;vsp_group.release_mask=axes&~SPF_DM;}')
 replace('#if !APP_MOTOR_OFFLINE_ONLY\n if(Low_Loaded()', '#if !APP_MOTOR_OFFLINE_ONLY\n Low_ReleaseLoadedTransfer(now);\n now=(uint32_t)xTaskGetTickCount();\n if(Low_Loaded()')
 replace('g_low_lift_retract_axes=SPF_LIFT|','g_low_lift_retract_axes=SPF_DM|SPF_LIFT|')
 replace('Low_Group(5U,g_low_lift_retract_axes,low_yaw,','Low_Group(5U,g_low_lift_retract_axes,36.5f,')
 replace('  if(!g_low_clear_valid){\n   float depth;if(!Low_ReadDepth(&depth) || !Low_TransferGate((uint32_t)xTaskGetTickCount(),depth,1U))return;\n  }\n  Low_PublishLift(1U);\n  if(!Low_Group(12U,SPF_DM,36.5f,0.0f,(float)SOURCE_RETURN_PULSES))Low_Fail(8U);', '  Low_PublishLift(1U);\n  g_low_phase=12U; /* All axes of the original group are already done. */')
 path.write_bytes(data)
 assert old.count(b'\n')==old.count(b'\r\n') and data.count(b'\n')==data.count(b'\r\n')
 changes={}
 for path in P.rglob('*'):
  if path.is_file() and path.read_bytes()!=(BASE/path.relative_to(P)).read_bytes():changes[path.relative_to(P).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
 assert set(changes)=={'CHASSIS/USER/pzq_commit_overlap.inc'}
 (E/'prepared.json').write_text(json.dumps(dict(changes=changes,flashed=False,hardware_access=False),indent=2)+'\n',encoding='utf-8')
 print('Prepared one-file isolated delta, preserved CRLF and all other source bytes')
if __name__=='__main__':main()
