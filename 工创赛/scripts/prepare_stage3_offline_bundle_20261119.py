"""Assemble selected 13/5 offline artifacts without connecting to any device."""
import copy
import hashlib
import json
from pathlib import Path
from prepare_stage3_align13_offline_20261119 import prepare

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output/stage3_offline_13_5_20261119'
CYCLE=ROOT/'tmp/stage3_offline_cycle_20261119/stage3-offline-cycle-compatible.json'

def write(name,value):
    p=OUT/name
    p.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return {'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    matched=prepare(5,CYCLE)
    taught=json.loads((ROOT/'calibration/stage3_ring_teach_forward11_20261119.json').read_text(encoding='utf-8'))
    archived=json.loads((ROOT/'calibration/stage3_roughing_ring_teach_20261118.json').read_text(encoding='utf-8'))
    left,right=taught['ring_points']['1'],taught['ring_points']['3']
    depths=[left['lift_depth_pulses'],taught['placement']['ring_2_trial_reference']['estimated_release_depth_pulses'],right['candidate_release_depth_pulses']]
    yaws=[left['dm_yaw_deg'],180.55,right['dm_yaw_deg']]
    xs=[left['extension_mm'],0,right['extension_mm']]
    release={str(i+1):dict(yaw_deg=yaws[i],extension_mm=xs[i],release_depth_pulses=depths[i],evidence=None) for i in range(3)}
    shared=dict(schema_version=1,task_code='215+213+512+321',start_id=1,
        slot_colors=[2,1,5],carrier_pick=dict(yaw_deg=36.5,depth_pulses=8003.47,evidence=None),
        evidence=dict(same_fixed_pose=False,pi=None,stage3_live=None),
        note='Offline numeric candidate only. Populate fresh alignment evidence after deployment; not field acceptance.')
    place=dict(copy.deepcopy(shared),mode='three-place',batch=1,rings=copy.deepcopy(release))
    recovery=dict(copy.deepcopy(shared),mode='rough-round-trip',batch=1,rings=copy.deepcopy(release))
    for key,p in recovery['rings'].items():
        p['recovery_depth_pulses']=archived['per_ring_pick_depth_pulses'][key]
        p['recovery_calibration_status']='historical candidate; new 13/5 pose not physically accepted'
    recovery['carrier_return']='actual lift top; original slot; release160 completed before next action; no descent'
    resume=dict(schema_version=1,expected_run=None,expected_pose=None,
        recovery_depth_by_ring=archived['per_ring_pick_depth_pulses'],
        note='Fill current successful placement run before --recover-placed. Same boot/reference only; no replacement placement or alignment.')
    stack=dict(copy.deepcopy(shared),mode='storage-stack',batch=2,
        verified_stack_base_colors=None,rings={str(i):dict(yaw_deg=None,extension_mm=None,
            ground_release_depth_pulses=None,evidence=None) for i in (1,2,3)})
    stack['evidence']['verified_stack_bases']=None
    stack['note']='Independent storage measurements required. Existing three base materials before alignment; firmware subtracts 24000 pulses once. Do not copy rough-area XY/J as measured storage calibration.'
    entries={name:write(name,data) for name,data in (
        ('placement-profile.json',place),('recovery-profile-candidate.json',recovery),
        ('recover-placed-template.json',resume),('stack-profile-template.json',stack))}
    selected=json.loads(Path(matched['manifest']).read_text(encoding='utf-8'))
    entries['matched_manifest']=write('stage3-offline-compatible.json',selected)
    summary=dict(offline_only=True,hardware_access=False,flashed=False,run_count=0,
        selected_compensation=dict(forward_mm=13,right_mm=5,target_pixel=matched['target_pixel']),
        next_trial_mode='three-place; one run only',profiles=entries,
        stack_desktop_package=str(ROOT/'output/stage3_stack_desktop_offline_20261119'),
        recovery='three placed -> same-order pickup -> actual top -> retract -> DM36.5 -> original slot -> release160 -> next',
        stack='three real bases -> independent material-plane visual alignment -> same-color targets -> ground Z minus 60mm once',
        open_items=['physical 13/5 placement acceptance','new-pose ring pickup heights (historical candidates supplied)',
            'top-release tray landing stability','storage material-plane response and arm points','stack physical acceptance'])
    write('bundle.json',summary)
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
