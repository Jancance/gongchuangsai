"""Name the accepted immutable mechanical snapshot pzq; no firmware edits."""
from pathlib import Path
import hashlib,json,shutil
R=Path(__file__).resolve().parents[1]
source=R/'handoff/integration13_final_20261109';target=R/'handoff/pzq'
original=json.loads((source/'manifest.json').read_text(encoding='utf-8'))
assert original['online_sha256']=='8da8171cb852556eaa8669ee200c18979195d078d084a0a195c78655bf587808'
assert not (target/'project').exists(),'Preserve an existing named baseline'
for name,h in original['files'].items():
 assert hashlib.sha256((source/name).read_bytes()).hexdigest()==h,name
for folder in ('project','images','verification'):shutil.copytree(source/folder,target/folder)
shutil.copy2(source/'candidate-manifest.json',target/'candidate-manifest.json')
shutil.copy2(source/'README.md',target/'ORIGINAL_INTEGRATION13_README.md')
shutil.copy2(source/'manifest.json',target/'origin-manifest.json')
identity={
 'name':'pzq','named_by':'user','named_on':'2026-11-09',
 'source_snapshot':'handoff/integration13_final_20261109',
 'project':'project/gongxun-f407.uvprojx','target':'Vehicle_LiftRuler',
 'online_axf_sha256':original['online_sha256'],'flash_bytes_verified':166752,
 'firmware_bytes_changed_by_naming':False,'motion_performed_by_naming':False,
 'pick_yaw_deg':{'down':180,'left':195,'right':165},
 'extension_mm':{'down':0,'left':156,'right':160},
 'slot_yaw_deg':37,'carrier_slot_deg':[0,120,240],'gripper_deg':{'preclose':160,'close':140,'open':210},
 'release_before_next_yaw_min_ms':200,'last_grip_depth_lead_mm':5,'rise_before_retract_mm':5,
 'rise_target_lead_before_yaw_mm':5,'yaw_and_retraction_overlap':True,
 'dm_deg_s':300,'extension_rpm':106,'extension_accel_rpm_s':600,
 'lift_rpm':1846.4,'lift_accel_rpm_s':45055,
 'field_evidence':'verification/integration13-three-immediate-once-20261109-213859.json',
 'third_release_command_s':17.987,'zero_complete_s':19.185,
 'exact_material_stable_timestamp_available':False,'three_slots_loaded_operator_confirmed':True,
 'startup_reference_in_timing':False,'route3_tested':False,
 'future_sequence':['arrive_source','real_vision_metric_alignment','chassis_correct_and_hold',
                    'vision_select_task_target','pzq_mechanical_pick_place','verified_loaded_then_route3'],
 'future_sequence_end_to_end_connected':False,'current_fixed_teach_order':['down','left','right'],
 'team_handoff':'../../docs/PZQ_CHASSIS_TEAM_HANDOFF_2026-11-09.md'
}
(target/'pzq.json').write_text(json.dumps(identity,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
files={}
for p in sorted(target.rglob('*')):
 if p.is_file():files[p.relative_to(target).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
(target/'source-manifest.json').write_text(json.dumps({'name':'pzq','files':files,'count':len(files),
 'online_axf_sha256':identity['online_axf_sha256'],'original_snapshot_verified':True},indent=2)+'\n',encoding='utf-8')
for name,h in files.items():assert hashlib.sha256((target/name).read_bytes()).hexdigest()==h,name
assert (target/'images/Vehicle_LiftRuler.axf').read_bytes()==(source/'images/Vehicle_LiftRuler.axf').read_bytes()
print(json.dumps({'name':'pzq','files_verified':len(files),'source_unchanged':True,'target':str(target)},indent=2))
