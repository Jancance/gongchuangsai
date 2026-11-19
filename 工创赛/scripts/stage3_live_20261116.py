"""Local manifest by default. Explicit actions only; no flash/reset/health writes."""
from pathlib import Path
from datetime import datetime
import argparse,hashlib,json,struct,sys,time,math,re
R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-16/stage3'

ERROR_FIELDS=('g_stage3_fault','g_it_cause','g_it_rejected','g_source_body_step_reason',
              'g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched',
              'g_startup_axes_cause','g_lift_ruler_cause','g_dm_jog_first_fault')
EXTRA_FIELDS=('g_it_action','g_it_sequence','g_it_elapsed_ms','g_source_body_step_submitted',
              'g_source_align_rejected','g_lift_ruler_request','g_lift_ruler_stop_unconfirmed',
              'g_vehicle_arm_state','g_dm_jog_request','g_dm_jog_state','g_dm_jog_stop_result',
              'g_dm_posvel_holding','g_dm_can2_motor_state',
              'g_lift_diag_cycle','g_recovery_diag_cycle','g_dm_can2_read_cycle')
DM_MANUAL_FIELDS=('g_dm_can2_read_request','g_dm_can2_state','g_dm_can2_result',
                  'g_dm_can2_feedback_valid','g_dm_can2_feedback_tick',
                  'g_dm_jog_result','g_dm_jog_tx_count','g_dm_jog_rx_count')
RELEASE_FIELDS=('g_vehicle_lift_feedback_valid','g_startup_extension_feedback_valid',
                'g_vm_test_request','g_lift_ruler_state')
TOUCHDOWN_REQUEST=18
TOUCHDOWN_MAX_STEP=1200
REPLAY_FIELDS=('g_stage3_replay_enable','g_stage3_replay_state','g_stage3_replay_step')
INTEGRATED_COMPENSATION_FIELD='g_stage3_integrated_compensation'


def replay_arm_admission(image,s,starting=False):
 if image.get('capabilities',{}).get('stage3_ring2_replay') is not True:
  raise ValueError('Selected manifest does not declare the Stage3 ring2 replay latch')
 if not (s['g_stage3_state'] in (0,11) and s['g_region_dispatch_area']==3 and
         s['g_region_dispatch_test_enable']==1 and not s['g_stage3_request'] and
         s['g_stage3_replay_state'] in (0,5) and s['g_stage3_replay_enable']==int(starting) and
         not s['g_it_request'] and s['g_it_state']!=1 and
         not s['g_chassis_auto_active'] and not s['g_source_body_step_request'] and
         s['g_source_body_step_state'] in (0,3) and not any(s.get(k) for k in ERROR_FIELDS)):
  latch='armed' if starting else 'clear'
  raise ValueError(f'Ring2 replay requires an idle Stage3 bench with {latch} latch, no pending request or fault')

def replay_started(before,s):
 return (not s['g_stage3_request'] and not s['g_stage3_replay_enable'] and
         s['g_stage3_run']!=before['g_stage3_run'] and s['g_stage3_run']!=0 and
         s['g_stage3_replay_state'] in (1,2,3,4,5) and
         s.get('g_stage3_replay_run',s['g_stage3_run'])==s['g_stage3_run'])

def replay_status(s):
 status={n:s[n] for n in REPLAY_FIELDS}
 completed_step=9
 status['compensation_mode']='nine_step_replay'
 if INTEGRATED_COMPENSATION_FIELD in s:
  marker=s[INTEGRATED_COMPENSATION_FIELD]
  status[INTEGRATED_COMPENSATION_FIELD]=marker
  if type(marker) is int and marker==1:
   status['compensation_mode']='integrated_visual'
   completed_step=0
  elif type(marker) is not int or marker!=0:
   status['compensation_mode']='invalid_marker'
   completed_step=None
 status['compensation_ready']=(completed_step is not None and s['g_stage3_replay_state']==5 and
   s['g_stage3_replay_step']==completed_step and
   not s['g_stage3_replay_enable'] and s['g_stage3_state']==7 and s['g_stage3_align_done']==1 and
   s['g_low_wheel_hold_ready']==1 and s['wheel_hold_results']==[1]*4 and
   not s['g_chassis_auto_active'] and not s['g_source_body_step_request'] and
   not s['g_stage3_request'] and not any(s.get(k) for k in ERROR_FIELDS))
 return status


def touchdown_target(target,image):
 if image.get('capabilities',{}).get('stage3_ring2_touchdown') is not True:
  raise ValueError('Selected manifest does not declare the Stage3 ring2 touchdown action')
 if not math.isfinite(target) or not 3200<=target<=image['stage3_lift_max']:
  raise ValueError('Touchdown requires an in-range absolute positive-down lift target')
 return target

def touchdown_admission(target,s):
 if not (s['g_region_dispatch_test_enable']==1 and s['g_region_dispatch_area']==3 and
         s['g_stage3_state']==7 and s['g_stage3_align_done']==1 and
         not s['g_stage3_request'] and not s['g_stage3_manual_state'] and
         s['g_low_wheel_hold_ready']==1 and s['wheel_hold_results']==[1]*4 and
         not s['g_source_align_chain_active'] and not s['g_chassis_auto_active'] and
         not s['g_source_body_step_request'] and s['g_source_body_step_state'] in (0,3) and
         # At low Z, normal descent invalidates the backoff pose, not the measured top reference.
         s['g_startup_axes_state']==4 and s['g_lift_ruler_state']==4 and
         s['g_extension_home_reference_valid']==1 and s['g_extension_hold_confirmed']==1 and
         s['g_vm_test_request']==1 and s['g_lift_compare_top_valid']==1 and
         s['g_vehicle_lift_feedback_valid']==1 and
         s['g_it_state']==2 and not s['g_it_request'] and not any(s.get(k) for k in ERROR_FIELDS)):
  raise ValueError('Touchdown requires the referenced, aligned Stage3 bench with wheels held and owners idle')
 if not 0<target-s['lift_depth_pulses']<=TOUCHDOWN_MAX_STEP:
  raise ValueError('Touchdown must descend by >0 and <=1200 virtual pulses from cached lift feedback')


def action_metadata(image,teach_kind=None,measured_x=None,dm_action=False):
 if teach_kind is not None:
  assert image.get('teach_feedback_mode')=='measured-axis-v1','Selected manifest lacks measured-axis-v1 teaching'
  if teach_kind in (2,4,6,7):
   assert image.get('teach_measured_x') is True,'Selected manifest lacks user-measured extension input'
   assert isinstance(measured_x,(int,float)) and math.isfinite(measured_x) and 0<=measured_x<=170,'Side teaching requires --measured-x-mm 0..170'
   if teach_kind in (6,7):assert teach_kind in image.get('teach_geometry_only_kinds',[]),'Selected manifest lacks geometry-only teaching kind'
  else:assert measured_x is None,'--measured-x-mm applies only to side teaching kinds 2/4/6/7'
 else:assert measured_x is None,'--measured-x-mm requires side teaching'
 if dm_action:assert image.get('dm_manual_teach_existing_requests') is True,'Selected manifest lacks explicit DM teaching requests'


def teach_completed(s):
 return (not s['g_stage3_teach_request'] and s['g_stage3_teach_result']==1 and
         s['g_stage3_teach_saved_sequence']==s['g_stage3_teach_sequence'])


def dm_released(s):
 return (not s['g_dm_jog_request'] and not s['g_dm_posvel_holding'] and
         not s['g_dm_can2_motor_state'] and s['g_dm_jog_state']==8 and
         s['g_dm_jog_result']==2 and s['g_dm_jog_stop_result']==1)


def dm_read_completed(s):
 return (not s['g_dm_can2_read_request'] and s['g_dm_can2_state']==4 and
         s['g_dm_can2_result']==1 and s['g_dm_can2_feedback_valid']==1)


def dm_request(action,before):
 assert not before['g_dm_jog_request'] and not before['g_dm_can2_read_request'],'DM request still pending'
 if action=='dm_release':return 'g_dm_jog_request',3
 assert action=='dm_read'
 assert not before['g_dm_posvel_holding'] and not before['g_dm_can2_motor_state'],'DM is enabled/holding; explicitly release first, then hand-position and read'
 assert before['g_dm_jog_state'] not in range(1,7) and before['g_dm_can2_state'] not in (1,2,3),'DM owner busy; wait for its existing operation'
 return 'g_dm_can2_read_request',2


def dm_release_pose(s,limits):
 """Reuse firmware it_at_top/SingleTeach references and StartupAxes zero tolerance."""
 c=s['lift_controller']
 assert (c['top_valid'] and c['backoff_valid'] and c['top_scale'] and
         s['g_vehicle_lift_feedback_valid'] and
         abs(s['g_vehicle_lift_current_deg']-c['safe_position_raw']/c['top_scale'])<=limits['top_deg']),'DM release requires the existing measured safe-top reference'
 assert (s['g_lift_ruler_state']==4 and not s['g_lift_ruler_cause'] and
         s['g_startup_axes_state']==4 and not s['g_startup_axes_cause'] and
         s['g_extension_home_reference_valid'] and s['g_extension_hold_confirmed'] and
         s['g_startup_extension_feedback_valid'] and s['g_vm_test_request']==1),'DM release requires existing axis references'
 assert abs(s['g_startup_extension_current_deg']-s['g_extension_home_reference_deg'])<=limits['x_zero_deg'],'DM release requires extension at existing home tolerance'
 assert (not s['g_source_body_step_request'] and s['g_source_body_step_state'] in (0,3) and
         not s['g_chassis_auto_active'] and s['g_it_state']==2),'DM release requires existing motion owners idle'


def controller_layout(elf):
 """Read actual compiler member offsets, including 64-bit safe_position_raw."""
 dwarf=elf.get_dwarf_info()
 for cu in dwarf.iter_CUs():
  for die in cu.iter_DIEs():
   name=die.attributes.get('DW_AT_name')
   if die.tag=='DW_TAG_typedef' and name and name.value==b'LiftRulerController':
    target=die.get_DIE_from_attribute('DW_AT_type')
    while target.tag!='DW_TAG_structure_type':target=target.get_DIE_from_attribute('DW_AT_type')
    from elftools.dwarf.dwarf_expr import DWARFExprParser
    parser=DWARFExprParser(dwarf.structs)
    offsets={}
    for child in target.iter_children():
     if child.tag!='DW_TAG_member':continue
     value=child.attributes['DW_AT_data_member_location'].value
     if not isinstance(value,int):
      operations=parser.parse_expr(value)
      assert len(operations)==1 and operations[0].op_name=='DW_OP_plus_uconst','Unsupported actual member expression'
      value=operations[0].args[0]
     offsets[child.attributes['DW_AT_name'].value.decode()]=value
    needed=('top_valid','backoff_valid','top_scale','safe_position_raw')
    assert all(type(offsets.get(k)) is int for k in needed),'Actual compiler member offsets unavailable'
    return {k:(offsets[k],8 if k=='safe_position_raw' else 4) for k in needed}
 raise AssertionError('Actual LiftRulerController debug type unavailable; no release permitted')


def calibration_profile(sample):
 return {n:sample[n] for n in ('g_stage3_carrier_yaw','g_stage3_pick_depth','g_stage3_target_yaw','g_stage3_target_x','g_stage3_target_depth')}

def it_completed(s,request):
 return not s['g_it_request'] and s['g_it_action']==request and s['g_it_state']==2

def touchdown_completed(s):
 return (it_completed(s,TOUCHDOWN_REQUEST) and s['g_vehicle_lift_feedback_valid']==1 and
         math.isfinite(s['lift_depth_pulses']))

def grip_request(kind,image):
 if kind=='preclose':
  assert image.get('preclose_request')==17,'Selected firmware manifest does not declare preclose17'
  return 17
 return {'open':6,'close':5}[kind]

def calibration_values(saved,image):
 allowed={'g_stage3_carrier_yaw','g_stage3_pick_depth','g_stage3_target_yaw','g_stage3_target_x','g_stage3_target_depth'}
 if image.get('stage3_observe_yaw_parameter'):allowed.add('g_stage3_observe_yaw')
 assert saved and set(saved)<=allowed,'Calibration file must contain only explicit configuration fields'
 limit=image['stage3_lift_max']
 for name,value in saved.items():
  array=name.startswith('g_stage3_target_')
  values=value if array else [value]
  assert isinstance(values,list) and (not array or len(values)==3),name
  upper=270 if name.endswith('yaw') else 170 if name.endswith('_x') else limit
  lower=3200 if name.endswith('depth') else 0
  for v in values:
   assert isinstance(v,(int,float)) and math.isfinite(v) and (v==-1 or lower<=v<=upper),name
  if not image.get('stage3_center_measured_xy'):
   if name=='g_stage3_target_yaw':assert value[1]==180
   if name=='g_stage3_target_x':assert value[1]==0
 return saved

def round_trip_profile(task_code,batch,slot_colors,start_id,image,cycle_mode=None):
 assert image.get('stage3_round_trip'),'Selected firmware has no round-trip mode'
 cycle=image.get('capabilities',{}).get('stage3_cycle_modes') is True
 assert cycle_mode in (None,'place-only','recover'),'Invalid cycle mode'
 assert cycle_mode is None or cycle,'Explicit cycle mode requires cycle-mode firmware'
 sys.path.insert(0,str(R/'src'))
 from smart_carrier.task_materials import resolve_preliminary_task
 mission=resolve_preliminary_task(task_code,start_id,R/'config/task_materials_preliminary_2027.json')
 targets=mission.batch_1 if batch==1 else mission.batch_2
 colors=[t.color_code for t in targets]
 assert sorted(slot_colors)==sorted(colors),'Actual slot colors must match this task batch'
 if image.get('capabilities',{}).get('stage3_three_place'):
  taught=json.loads((R/'calibration/stage3_ring_teach_forward11_20261119.json').read_text(encoding='utf-8'))
  left,right=(taught['ring_points'][str(n)] for n in (1,3))
  center=taught['placement']['ring_2_trial_reference']
  calibration=calibration_values(dict(g_stage3_carrier_yaw=36.5,g_stage3_pick_depth=8003.47,
   g_stage3_target_yaw=[left['dm_yaw_deg'],180.55,right['dm_yaw_deg']],
   g_stage3_target_x=[left['extension_mm'],0.0,right['extension_mm']],
   g_stage3_target_depth=[left['lift_depth_pulses'],center['estimated_release_depth_pulses'],
                         right['candidate_release_depth_pulses']]),image)
  mission_fields=dict(g_stage3_colors=colors,g_stage3_positions=[t.rough_ring for t in targets],
   g_stage3_slot_colors=list(slot_colors),g_stage3_task_batch=batch,
   g_stage3_test_first_item=0,g_stage3_test_count=3)
  if cycle:
   mission_fields.update(g_stage3_cycle_mode=int(cycle_mode=='recover'),g_stage3_first_mapping_valid=0)
   if cycle_mode=='recover':
    archived=json.loads((R/'calibration/stage3_roughing_ring_teach_20261118.json').read_text(encoding='utf-8'))
    recovery=[archived['ring_points'][str(n)]['lift_target_pulses'] for n in (1,2,3)]
    assert all(3200<=v<=image['stage3_lift_max'] for v in recovery)
    calibration['g_stage3_recovery_depth']=recovery
  return calibration,mission_fields
 taught=json.loads((R/'calibration/stage3_roughing_ring_teach_20261118.json').read_text(encoding='utf-8'))
 points=[taught['ring_points'][str(n)] for n in (1,2,3)]
 assert taught['measurement_round']==2 and all(p['angle_evidence'] for p in points)
 calibration=calibration_values(dict(g_stage3_carrier_yaw=taught['vehicle_carrier_pick_yaw_deg'],
   g_stage3_pick_depth=8000.0,g_stage3_target_yaw=[p['dm_yaw_deg'] for p in points],
   g_stage3_target_x=[p['extension_mm'] for p in points],g_stage3_target_depth=[60206.0]*3),image)
 recovery=[p['lift_target_pulses'] for p in points]
 assert all(3200<=v<=image['stage3_lift_max'] for v in recovery)
 calibration['g_stage3_recovery_depth']=recovery
 return calibration,dict(g_stage3_colors=colors,g_stage3_positions=[t.rough_ring for t in targets],
   g_stage3_slot_colors=list(slot_colors),g_stage3_task_batch=batch,
   g_stage3_test_first_item=0,g_stage3_test_count=3)

PROFILE_CONTEXT=('g_stage3_run','g_stage3_pose','g_low_wheel_hold_cycle','g_stage3_observe_yaw')
PROFILE_CAPS=('stage3_storage_only','stage3_clearance_depth','stage3_single_center_single_slot')
PLACE_CLEARANCE='g_stage3_place_clearance_depth'
RECOVERY_CLEARANCE='g_stage3_recovery_clearance_depth'
STACK_RISE_PULSES=60*400

def profile_json(path):
 def unique(pairs):
  data={}
  for k,v in pairs:
   if k in data:raise ValueError('Duplicate profile field: '+k)
   data[k]=v
  return data
 return json.loads(path.read_text(encoding='utf-8-sig'),object_pairs_hook=unique)

def stage_profile(saved,image):
 """Compile explicit measurements, with all arrays addressed by physical ring."""
 def require(test,message):
  if not test:raise ValueError(message)
 def number(value,lo,hi):
  return type(value) in (int,float) and math.isfinite(value) and lo<=value<=hi
 modes=('single-red','three-place','rough-round-trip','storage-stack')
 mode=saved.get('mode')
 require(type(saved.get('schema_version')) is int and saved['schema_version']==1 and mode in modes,
         'Expected schema_version 1 and mode '+', '.join(modes))
 single=mode=='single-red';storage=mode=='storage-stack'
 caps=image.get('capabilities',{})
 cycle=caps.get('stage3_cycle_modes') is True
 require(mode!='three-place' or cycle,'Three-place explicit profile requires cycle-mode firmware')
 require(single or (image.get('stage3_round_trip') and (cycle or all(caps.get(k) is True for k in PROFILE_CAPS[:2]))),
         'Three-item explicit profiles require the compatible Stage3 manifest')
 slots=saved.get('slot_colors')
 require(type(slots) is list and len(slots)==3 and all(type(c) is int and c in range(7) for c in slots),
         'Provide three physical slot colors; 0 means empty')
 if single:
  require(slots==[0,1,0] or (slots[1]==1 and len(set(slots))==3 and 0 not in slots),
          'Single red requires physical slot 2 red; use actual other colors or leave both empty')
  # Legacy validators require a full permutation. The extra labels are never
  # indexed by the one-item bench run, and are not reported as observed inventory.
  board_slots=slots if slots!=[0,1,0] or caps.get(PROFILE_CAPS[2]) else [2,1,5]
  colors=list(board_slots) if 0 not in board_slots else [2,1,5]
  positions=[1,2,3]
 else:
  batch=saved.get('batch')
  require(type(batch) is int and batch in ((2,) if storage else (1,2)),'Invalid batch for selected mode')
  start=saved.get('start_id')
  require(type(start) is int and start in (1,2),'Explicit preliminary start_id 1 or 2 required')
  sys.path.insert(0,str(R/'src'))
  from smart_carrier.task_materials import resolve_preliminary_task
  require(type(saved.get('task_code')) is str and bool(saved['task_code']),'Explicit preliminary task_code required')
  mission=resolve_preliminary_task(saved['task_code'],start,R/'config/task_materials_preliminary_2027.json')
  targets=mission.batch_1 if batch==1 else mission.batch_2
  colors=[t.color_code for t in targets]
  positions=[t.temporary_ring if storage else t.rough_ring for t in targets]
  require(sorted(slots)==sorted(colors),'Actual slot colors must uniquely match this task batch')
  board_slots=slots
  if storage:
   first=saved.get('verified_stack_base_colors')
   expected=[0]*3
   for t in mission.batch_1:expected[t.temporary_ring-1]=t.color_code
   require(type(first) is list and all(type(c) is int for c in first) and first==expected,
           'Confirm correctly placed first-batch colors by physical storage ring')
 rings=saved.get('rings')
 keys={'2'} if single else {'1','2','3'}
 require(isinstance(rings,dict) and set(rings)==keys,'Required measured ring numbers: '+', '.join(sorted(keys)))
 limit=image['stage3_lift_max']
 carrier=saved.get('carrier_pick')
 require(isinstance(carrier,dict) and number(carrier.get('yaw_deg'),0,270) and
         number(carrier.get('depth_pulses'),3200,limit),'Need measured carrier yaw and depth in range')
 if caps.get('stage3_integrated_visual_compensation') is True:
  require(carrier['yaw_deg']==36.5,'Integrated Stage3 carrier pickup requires 36.5 degrees')
 yaw=[-1.0]*3;x=yaw.copy();z=yaw.copy();recovery=yaw.copy();pc=yaw.copy();rc=yaw.copy()
 def clearance(point,key,endpoint):
  value=point.get(key,-1)
  require(number(value,3200,limit) and value<endpoint or type(value) in (int,float) and value==-1,
          'Clearance must be shallower than the phase target, or -1 for top-only')
  return value
 for key,point in rings.items():
  i=int(key)-1
  require(isinstance(point,dict) and number(point.get('yaw_deg'),0,270) and
          number(point.get('extension_mm'),0,170),'Ring '+key+': need measured yaw and extension')
  field='ground_release_depth_pulses' if storage else 'release_depth_pulses'
  base=point.get(field)
  require(number(base,3200,limit),'Ring '+key+': need measured '+field)
  effective=base-STACK_RISE_PULSES if storage else base
  require(number(effective,3200,limit),'Ring '+key+': computed release depth outside lift range')
  depth=base if cycle else effective
  yaw[i],x[i],z[i]=point['yaw_deg'],point['extension_mm'],depth
  pc[i]=clearance(point,'place_clearance_depth_pulses',effective)
  if mode=='rough-round-trip':
   require(number(point.get('recovery_depth_pulses'),3200,limit),'Ring '+key+': need separate recovery depth')
   recovery[i]=point['recovery_depth_pulses']
   rc[i]=clearance(point,'recovery_clearance_depth_pulses',recovery[i])
  else:
   require(not {'recovery_depth_pulses','recovery_clearance_depth_pulses'}&point.keys(),'Place-only mode cannot include recovery depth')
 if not caps.get('stage3_clearance_depth'):
  require(all(v==-1 for v in pc+rc),'Selected firmware has no explicit clearance depth')
 if single and not image.get('stage3_center_measured_xy'):
  require(yaw[1]==180 and x[1]==0,'Selected firmware requires fixed center 180 deg / 0 mm')
 floats=dict(g_stage3_carrier_yaw=carrier['yaw_deg'],g_stage3_pick_depth=carrier['depth_pulses'],
             g_stage3_target_yaw=yaw,g_stage3_target_x=x,g_stage3_target_depth=z)
 ints=dict(g_stage3_colors=colors,g_stage3_positions=positions,g_stage3_slot_colors=board_slots,
           g_stage3_test_first_item=int(single),g_stage3_test_count=1 if single else 3)
 if image.get('stage3_round_trip'):
  floats['g_stage3_recovery_depth']=recovery
  ints['g_stage3_task_batch']=0 if single else batch
 if caps.get('stage3_storage_only'):ints['g_stage3_storage_only']=int(storage)
 if caps.get('stage3_clearance_depth'):floats.update({PLACE_CLEARANCE:pc,RECOVERY_CLEARANCE:rc})
 if cycle:
  ints.update(g_stage3_cycle_mode=2 if storage else int(mode=='rough-round-trip'),
              g_stage3_first_mapping_valid=int(storage))
  if storage:
   ints['g_stage3_first_colors']=[t.color_code for t in mission.batch_1]
   ints['g_stage3_first_positions']=[t.temporary_ring for t in mission.batch_1]
 return floats,ints

def profile_evidence(saved,image):
 """Explicit local evidence references, not assertions of physical clearance."""
 ref=saved.get('evidence')
 if not isinstance(ref,dict) or ref.get('same_fixed_pose') is not True:
  raise ValueError('Explicit evidence and unchanged chassis/observation confirmation required')
 def existing(value):
  if type(value) is not str or not value.strip() or not (R/value).is_file():
   raise ValueError('Missing local Stage3 evidence file: '+str(value))
  return R/value
 existing(ref.get('pi'))
 existing(saved['carrier_pick'].get('evidence'))
 if saved['mode']=='storage-stack':existing(ref.get('verified_stack_bases'))
 for point in saved['rings'].values():existing(point.get('evidence'))
 evidence=profile_json(existing(ref.get('stage3_live')))
 after=evidence.get('after',{})
 if (evidence.get('image',{}).get('sha256')!=image['sha256'] or
     not all(after.get(k) for k in (*PROFILE_CONTEXT[:3],'g_stage3_align_done','g_low_wheel_hold_ready')) or
     not isinstance(after.get(PROFILE_CONTEXT[3]),(int,float)) or
     after.get('g_stage3_state')!=7 or after.get('wheel_hold_results')!=[1]*4 or
     type(after.get('g_src_now_ms')) is not int or after['g_src_now_ms']<0 or not evidence.get('when')):
  raise ValueError('Alignment evidence must match the selected firmware and show real Stage3 alignment/hold')
 datetime.fromisoformat(evidence['when'])
 return evidence

def require_profile_pose(evidence,sample,now):
 old=evidence['after']
 age_ms=(now-datetime.fromisoformat(evidence['when'])).total_seconds()*1000
 if (any(sample.get(k)!=old[k] for k in PROFILE_CONTEXT) or
     sample['g_src_now_ms']<old['g_src_now_ms'] or
     not 0<=age_ms<=sample['g_src_now_ms']):
  raise ValueError('Stage3 profile evidence is from a different run, observation, hold or boot')


def recover_placed_depths(saved,image):
 if (image.get('capabilities',{}).get('stage3_recover_placed') is not True or
     image.get('stage3_recover_placed_request')!=32):
  raise ValueError('Selected firmware has no independent placed-item recovery')
 if any(type(saved.get(k)) is not int or saved[k]<=0 for k in ('expected_run','expected_pose')):
  raise ValueError('Recovery requires explicit positive expected_run and expected_pose')
 points=saved.get('recovery_depth_by_ring')
 if not isinstance(points,dict) or set(points)!={'1','2','3'}:
  raise ValueError('Recovery requires independent depths for physical rings 1, 2, 3')
 values=[points[str(n)] for n in (1,2,3)]
 if not all(type(v) in (int,float) and math.isfinite(v) and 3200<=v<=image['stage3_lift_max'] for v in values):
  raise ValueError('Recovery depth outside lift range')
 return values


def recover_placed_admission(saved,s):
 if not (s['g_stage3_state']==6 and s['g_stage3_placed_count']==3 and
         s['g_stage3_recovered_count']==0 and s['g_stage3_run']==saved['expected_run'] and
         s['g_stage3_locked_run']==s['g_stage3_run'] and s['g_stage3_pose']==saved['expected_pose'] and
         s['g_region_dispatch_area']==3 and s['g_region_dispatch_test_enable']==1 and
         s['g_low_wheel_hold_ready']==1 and s['wheel_hold_results']==[1]*4 and
         not s['g_stage3_request'] and not s['g_stage3_manual_state'] and
         not s['g_it_request'] and s['g_it_state']==2 and
         not s['g_chassis_auto_active'] and not s['g_source_body_step_request'] and
         not any(s.get(k) for k in ERROR_FIELDS)):
  raise ValueError('Recovery requires the same locked completed three-place run, zero recovered items and idle held owners')


def recover_placed_completed(before,s):
 return (not s['g_stage3_request'] and s['g_stage3_state']==6 and
         s['g_stage3_run']==before['g_stage3_run'] and s['g_stage3_locked_run']==before['g_stage3_locked_run'] and
         s['g_stage3_placed_count']==3 and s['g_stage3_recovered_count']==3 and
         s['g_stage3_pose']!=before['g_stage3_pose'])


def enter_teach_completed(s):
 return (not s['g_stage3_request'] and s['g_stage3_state']==7 and not s['g_stage3_align_done'] and
         s['g_low_wheel_hold_ready']==1 and s['g_low_wheel_hold_stage']==13)

def continue_completed(before,s):
 return s['g_stage3_state'] in ((1,2) if not before['g_stage3_align_done'] else (5,6))

def refresh_it_evidence(s,read):
 # Base snapshots read IT state before the extra sequence. Re-read state AFTER
 # the sequence so a concurrent admission cannot combine old DONE with new seq.
 for n in ('g_it_sequence','g_it_action','g_it_elapsed_ms','g_it_cause','g_it_rejected',
           'g_it_state','g_it_request'):
  s[n]=read(n)
 return s

class ActionOutcome:
 """Evaluate actual sampled MCU evidence; an old terminal state is insufficient."""
 def __init__(self,action,before,done,it_request=None):
  self.action,self.before,self.done=action,before,done
  self.it_request=it_request
  self.first_error=None;self.it_busy_seen=False;self.completed=False

 def observe(self,s):
  errors={n:s[n] for n in ERROR_FIELDS if s.get(n)}
  for n in ('g_stage3_rejected','g_source_align_rejected'):
   if s.get(n,0)!=self.before.get(n,0):errors[n]=s[n]
  # User STOP intentionally enters fault states. Keep their reason in samples,
  # but wait for backend acknowledgments instead of treating state9 as stopped.
  if self.action=='stop':
   for n,v in [('g_stage3_fault',2),('g_it_cause',2),('g_source_body_step_reason',9)]:
    if errors.get(n)==v:errors.pop(n)
  if errors and self.first_error is None:self.first_error=dict(values=errors,sample=s.copy())
  fresh=True
  if self.action in ('it_axis','it_return','touchdown_z'):
   req={'it_axis':3,'it_return':4,'touchdown_z':TOUCHDOWN_REQUEST}[self.action]
   if not s['g_it_request'] and s['g_it_action']==req and s['g_it_state']==1:self.it_busy_seen=True
   fresh=(s['g_it_sequence']!=self.before['g_it_sequence'] if req in (3,TOUCHDOWN_REQUEST) else
          self.it_busy_seen or s['g_it_elapsed_ms']!=self.before['g_it_elapsed_ms'])
  elif self.action in ('carrier_slot','grip'):
   req=self.it_request
   consumed=not s['g_it_request'] and s['g_it_action']==req
   if consumed and s['g_it_state']==1:self.it_busy_seen=True
   fresh=consumed and (self.before['g_it_action']!=req or self.it_busy_seen or
                      s['g_it_elapsed_ms']!=self.before['g_it_elapsed_ms'])
  elif self.action in ('body_axis','turn'):
   fresh=not s['g_stage3_request'] and s['g_source_align_command_seq']!=self.before['g_source_align_command_seq']
  elif self.action=='teach_kind':
   fresh=s['g_stage3_teach_sequence']!=self.before['g_stage3_teach_sequence']
  elif self.action=='dm_release':
   fresh=all(s[n]!=self.before[n] for n in ('g_dm_can2_feedback_tick','g_dm_jog_tx_count','g_dm_jog_rx_count'))
  elif self.action=='dm_read':
   fresh=all(s[n]!=self.before[n] for n in ('g_dm_can2_read_cycle','g_dm_can2_feedback_tick'))
  elif self.action in ('continue_cycle','new_round','enter_teach'):
   fresh=not s['g_stage3_request']
  self.completed=bool(self.first_error is None and fresh and self.done(s))
  return self.completed

def stop_acknowledged(before,s):
 # Acknowledged software STOP is not a measurement of physical stillness.
 if (s['g_stage3_request'] or s['g_stage3_state']!=9 or s['g_it_request'] or
     s['g_it_state']==1 or s['g_source_body_step_request'] or
     s['g_source_body_step_state'] in (1,2,8) or s['g_chassis_auto_active'] or
     s['g_vehicle_arm_state']==1 or s['g_startup_axes_state'] in (1,2,3) or
     any(s[n] for n in ('g_vehicle_arm_stop_pending','g_lift_ruler_stop_pending',
                       'g_lift_ruler_request','g_lift_ruler_stop_unconfirmed','g_source_parallel_stop_mask'))):return False
 wheel_stop_needed=(before['g_chassis_auto_active'] or before['g_source_body_step_submitted'] or
                    before['g_source_body_step_state'] in (1,2,8))
 if wheel_stop_needed and s['body_stop_masks'][2]!=15:return False
 # An idle STOP may leave an existing position hold enabled. A cancelled IT
 # command, in contrast, must finish the existing DM disable/STOP handshake.
 if before['g_it_state']==1 or before['g_it_request']:
  if (s['g_dm_jog_request'] or s['g_dm_posvel_holding'] or s['g_dm_can2_motor_state'] or
      s['g_dm_jog_stop_result']!=1 or s['g_dm_jog_state']!=8):return False
 return True

def manifest(path=None):
 c=profile_json(Path(path) if path else E/'candidate.json');assert c['build_passed']
 for name,digest in {**c['changes'],**c['unchanged_sha256']}.items():
  assert hashlib.sha256((R/c['project']/name).read_bytes()).hexdigest()==digest,name
 image=dict(c['images']['Vehicle_LiftRuler'])
 assert hashlib.sha256((R/image['axf']).read_bytes()).hexdigest()==image['sha256']
 image['stage3_lift_max']=c.get('stage3_lift_max',29700)
 image['teach_entry_request']=c.get('teach_entry_request')
 image['preclose_request']=c.get('preclose_request')
 for key in ('teach_feedback_mode','teach_measured_x','dm_manual_teach_existing_requests','teach_geometry_only_kinds'):image[key]=c.get(key)
 for key in ('stage3_center_measured_xy','stage3_manual_axes_teach'):image[key]=c.get(key,False)
 for key in ('stage3_reobserve_request','stage3_observe_yaw_parameter'):image[key]=c.get(key)
 image['stage3_low_teach_request']=c.get('stage3_low_teach_request')
 image['stage3_recover_placed_request']=c.get('stage3_recover_placed_request')
 image['stage3_round_trip']=c.get('stage3_round_trip',False)
 image['capabilities']=c.get('capabilities',{})
 if image['dm_manual_teach_existing_requests']:
  header=(R/c['project']/'USER/vehicle_parameters.h').read_text(encoding='latin1')
  image['manual_teach_limits']={key:float(re.search(r'^#define\s+'+macro+r'\s+([\d.]+)f?\b',header,re.M).group(1))
      for key,macro in [('top_deg','SOURCE_TOP_TOLERANCE_DEG'),('x_zero_deg','SOURCE_EXTENSION_ZERO_DONE_DEG')]}
 return image

def main():
 global E
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--manifest',type=Path,default=E/'candidate.json')
 p.add_argument('--snapshot',action='store_true');p.add_argument('--watch',type=float,metavar='SECONDS')
 group=p.add_mutually_exclusive_group()
 group.add_argument('--center-only',action='store_true');group.add_argument('--all-three',action='store_true')
 group.add_argument('--teach-kind',type=int,choices=range(1,8))
 group.add_argument('--it-axis',choices=['dm','x','z']);group.add_argument('--it-return',action='store_true')
 group.add_argument('--touchdown-z',type=float,metavar='ABS_PULSES',
                    help='Stage3 ring2 bench: one bounded downward lift step; no automatic grip')
 group.add_argument('--arm-ring2-replay',action='store_true',
                    help='Arm the bench ring2 replay latch only; requires a separate normal start')
 group.add_argument('--start-ring2-replay',action='store_true',
                    help='Start one already-armed bench replay with the tray stocked')
 group.add_argument('--body-axis',choices=['right','forward','left','backward']);group.add_argument('--turn',type=float)
 group.add_argument('--teach-body-axis',choices=['right','forward','left','backward'])
 group.add_argument('--teach-body-finish',action='store_true')
 group.add_argument('--teach-wheel-hold',action='store_true')
 group.add_argument('--continue',dest='continue_cycle',action='store_true')
 group.add_argument('--new-round',action='store_true');group.add_argument('--stop',action='store_true')
 group.add_argument('--enter-teach',action='store_true')
 group.add_argument('--carrier-slot',type=int,choices=(1,2,3))
 group.add_argument('--grip',choices=('open','close','preclose'))
 group.add_argument('--restore-calibration',type=Path)
 group.add_argument('--configure-round-trip',metavar='TASK_CODE')
 group.add_argument('--recover-placed',type=Path,metavar='JSON',
                    help='Recover the same completed three-place run with explicit ring depths; never re-place or realign')
 p.add_argument('--cycle-mode',choices=('place-only','recover'),
                help='Cycle-mode firmware only: explicit recovery opt-in; configuration never starts motion')
 group.add_argument('--configure-profile',type=Path,help='Explicit measured Stage3 profile; publish only, never start')
 p.add_argument('--check',action='store_true',help='Validate and print profile locally; never attach')
 p.add_argument('--slot-colors',type=int,nargs=3,metavar=('SLOT1','SLOT2','SLOT3'))
 p.add_argument('--batch',type=int,choices=(1,2),default=1)
 p.add_argument('--start-id',type=int,choices=(1,2),default=1)
 group.add_argument('--dm-release',action='store_true',help='Existing DM request3 disable with real acknowledgment; no automatic recovery')
 group.add_argument('--manual-teach-release',action='store_true',help='Enter dual-axis bench teach on the dedicated firmware')
 group.add_argument('--manual-teach-low',action='store_true',help='Release XY at the completed powered low lift position')
 group.add_argument('--manual-teach-exit',action='store_true',help='Exit bench teach and rehome the extension')
 group.add_argument('--reobserve',action='store_true',help='Restart bench visual alignment through the firmware request')
 group.add_argument('--dm-read',action='store_true',help='Existing CAN read request2 only; release holding before manually positioning')
 p.add_argument('--measured-x-mm',type=float,help='Explicit ruler-measured target extension for side teaching kinds 2/4/6/7')
 p.add_argument('--value',type=float);p.add_argument('--distance-mm',type=float)
 p.add_argument('--timeout',type=float,default=65)
 a=p.parse_args();image=manifest(a.manifest);E=a.manifest.resolve().parent
 if a.grip is not None:grip_request(a.grip,image)
 if a.restore_calibration:calibration_values(profile_json(a.restore_calibration),image)
 action_metadata(image,a.teach_kind,a.measured_x_mm,a.dm_release or a.dm_read)
 action=next((k for k in ['center_only','all_three','teach_kind','it_axis','it_return','touchdown_z','arm_ring2_replay','start_ring2_replay','body_axis','turn','teach_body_axis','teach_body_finish','teach_wheel_hold','continue_cycle','new_round','stop','enter_teach','carrier_slot','grip','restore_calibration','configure_round_trip','configure_profile','recover_placed','dm_release','dm_read','manual_teach_release','manual_teach_low','manual_teach_exit','reobserve'] if getattr(a,k) is not None and getattr(a,k) is not False),None)
 has_replay=image.get('capabilities',{}).get('stage3_ring2_replay') is True
 if action in ('arm_ring2_replay','start_ring2_replay') and not has_replay:
  p.error('Selected manifest does not declare the Stage3 ring2 replay latch')
 if action=='touchdown_z':
  try:touchdown_target(a.touchdown_z,image)
  except ValueError as exc:p.error(str(exc))
 profile=profile_source=profile_report=None
 if a.cycle_mode is not None and action!='configure_round_trip':
  p.error('--cycle-mode requires --configure-round-trip; explicit profiles select their own mode')
 if a.check and (action not in ('configure_profile','configure_round_trip','recover_placed') or a.watch is not None or a.snapshot):
  p.error('--check requires a profile/configuration/recovery action alone')
 recovery_source=recovery_depths=None
 if action=='recover_placed':
  try:
   recovery_source=profile_json(a.recover_placed)
   recovery_depths=recover_placed_depths(recovery_source,image)
  except (ValueError,OSError,KeyError,TypeError) as exc:p.error(str(exc))
  if a.check:
   print(json.dumps(dict(local_check=True,hardware_access=False,motion_allowed=False,
                        recovery_depth_by_ring=recovery_source['recovery_depth_by_ring'],
                        expected_run=recovery_source['expected_run'],expected_pose=recovery_source['expected_pose'])));return
 if action=='configure_profile':
  try:
   profile_source=profile_json(a.configure_profile)
   profile=stage_profile(profile_source,image)
   profile_report=profile_evidence(profile_source,image)
  except (ValueError,OSError,KeyError,TypeError) as exc:p.error(str(exc))
  if a.check:
   print(json.dumps(dict(local_check=True,hardware_access=False,motion_allowed=False,
                         calibration=profile[0],mission=profile[1],actual_slot_colors=profile_source['slot_colors'],
                         evidence={k:profile_report['after'][k] for k in PROFILE_CONTEXT}),indent=2));return
 if action=='configure_round_trip':
  if image.get('capabilities',{}).get('stage3_clearance_depth'):
   p.error('Compatible firmware requires --configure-profile; legacy archive is not a new measurement')
  if not a.slot_colors:p.error('--configure-round-trip requires confirmed --slot-colors in physical slot order')
  print('Task preset uses archival taught geometry and release depths; not freshly bound evidence.',file=sys.stderr)
  profile=round_trip_profile(a.configure_round_trip,a.batch,a.slot_colors,a.start_id,image,a.cycle_mode)
  if a.cycle_mode=='recover':
   print('Recovery uses historical taught Z; ring 2/3 depths remain physically unverified.',file=sys.stderr)
  if a.check:
   print(json.dumps(dict(local_check=True,hardware_access=False,motion_allowed=False,
                        calibration=profile[0],mission=profile[1]),indent=2));return
 if action=='manual_teach_low':assert image.get('stage3_low_teach_request')==3
 if action=='reobserve':assert image.get('stage3_reobserve_request')==31
 if action in ('manual_teach_release','manual_teach_exit'):
  assert image.get('stage3_manual_axes_teach'),'Selected firmware has no dual-axis teach interface'
 if not action and not a.snapshot and not a.watch:
  print(json.dumps(dict(local_check=True,hardware_access=False,image=image,actions='explicit CLI only')));return
 import gripper210_attach_read_20261105 as live
 from source_body_probe_20261111 import attach
 sys.path.insert(0,str(R/'src'))
 from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard
 live.IMAGE=R/image['axf'];symbols,segments=live.load_image()
 from elftools.elf.elffile import ELFFile
 with live.IMAGE.open('rb') as stream:
  elf=ELFFile(stream);objects={s.name:(int(s['st_value']),int(s['st_size'])) for s in elf.get_section_by_name('.symtab').iter_symbols() if s['st_info']['type']=='STT_OBJECT'}
  lift_layout=controller_layout(elf) if action=='dm_release' else None
 from stage3_distance_capture_20261116 import load_elf as load_distance_elf,snapshot as distance_snapshot
 distance_symbols,_=load_distance_elf(live.IMAGE)
 floats={'g_stage3_carrier_yaw','g_stage3_pick_depth','g_stage3_target_yaw','g_stage3_target_x','g_stage3_target_depth','g_stage3_actual_yaw','g_stage3_actual_x','g_stage3_actual_depth','g_stage3_actual_z_mm','g_stage3_teach_measured_x','g_dm_can2_position_deg','g_dm_can2_logical_position_deg'}
 floats.update(('g_stage3_observe_yaw','g_stage3_recovery_depth',PLACE_CLEARANCE,RECOVERY_CLEARANCE))
 signed={'g_stage3_right_um','g_stage3_forward_um','g_stage3_yaw_mdeg','g_stage3_error_yaw_mdeg'}
 names=[n for n in objects if n.startswith('g_stage3_') or n.startswith('g_region_dispatch_') or n in EXTRA_FIELDS]
 if image.get('stage3_round_trip'):
  names.extend(('g_source_parallel_sync_tick','g_source_parallel_started_mask','g_source_parallel_done_mask'))
 if action in ('dm_release','dm_read') or image.get('teach_feedback_mode')=='measured-axis-v1':
  assert all(n in objects for n in DM_MANUAL_FIELDS),'Required DM feedback absent from image'
  names.extend(n for n in (*DM_MANUAL_FIELDS,'g_dm_can2_position_deg','g_dm_can2_logical_position_deg') if n in objects and n not in names)
 if action=='teach_kind':
  assert all(n in objects for n in ('g_stage3_teach_sequence','g_stage3_teach_saved_sequence','g_stage3_teach_measured_x')),'Required teaching publication absent from image'
 if action=='dm_release':
  assert all(n in objects for n in (*RELEASE_FIELDS,'g_lift_ruler_controller')),'Actual release pose evidence absent from image'
  names.extend(n for n in RELEASE_FIELDS if n not in names)
 if action=='touchdown_z':
  assert all(n in symbols and n in objects and objects[n][1]==4 for n in
             ('g_it_axes','g_it_lift_pulses','g_it_request','g_vehicle_lift_feedback_valid',
              'g_lift_ruler_state','g_vm_test_request')), \
             'Touchdown command and lift feedback symbols are required'
  names.extend(n for n in ('g_vehicle_lift_feedback_valid','g_lift_ruler_state','g_vm_test_request') if n not in names)
 if has_replay:
  assert all(n in symbols and n in objects and objects[n][1]==4 for n in REPLAY_FIELDS), \
         'Replay latch and status symbols are required'
 assert all(n in objects for n in EXTRA_FIELDS),'Required action evidence absent from image'
 if action in ('configure_profile','configure_round_trip'):
  fields={**profile[0],**profile[1]}
  if image.get('stage3_round_trip'):fields['g_stage3_task_run']=0
  assert all(n in symbols and n in objects and objects[n][1]==4*(len(v) if isinstance(v,list) else 1)
             for n,v in fields.items()),'Profile symbols or sizes do not match the selected image'
 writes=[];samples=[]
 with attach() as session:
  board=CommittedPickBoard(session.target,symbols,segments);t=session.target
  def snap():
   s=board.snapshot()
   for n in names:
    addr,size=objects[n];raw=list(t.read_memory_block32(addr,size//4))
    if n in floats:raw=[struct.unpack('<f',struct.pack('<I',v))[0] for v in raw]
    elif n in signed:raw=[v-(1<<32) if v&(1<<31) else v for v in raw]
    s[n]=raw[0] if len(raw)==1 else raw
   s['body_distance_evidence']=distance_snapshot(t,distance_symbols)
   if lift_layout is not None:
    base=objects['g_lift_ruler_controller'][0]
    s['lift_controller']={n:int.from_bytes(bytes(t.read_memory_block8(base+offset,size)),'little',signed=n=='safe_position_raw')
                          for n,(offset,size) in lift_layout.items()}
   return refresh_it_evidence(s,lambda n:t.read32(symbols[n]))
  def write(name,value,index=0,floating=False):
   if floating:value=struct.unpack('<I',struct.pack('<f',value))[0]
   t.write32(symbols[name]+4*index,int(value));writes.append(dict(symbol=name,index=index,value=value))
  before=snap();samples.append(before)
  def idle():
   assert before['g_stage3_state'] in (0,6,7,11),'Calibration/profile requires idle or pending configuration'
   assert not before['g_it_request'] and before['g_it_state']!=1
   assert not before['g_safety_fault'] and not before['g_rtos_fault'] and not before['g_can_ctrl_fault_latched']
  done=lambda s:True
  if action=='arm_ring2_replay':
   replay_arm_admission(image,before)
   write('g_stage3_replay_enable',1)
   done=lambda s:(s['g_stage3_replay_enable']==1 and s['g_stage3_state']==before['g_stage3_state'] and
                   s['g_stage3_replay_state']==before['g_stage3_replay_state'] and not s['g_stage3_request'])
  elif action=='start_ring2_replay':
   replay_arm_admission(image,before,starting=True)
   write('g_stage3_request',1)
   done=lambda s:replay_started(before,s)
  elif action=='recover_placed':
   recover_placed_admission(recovery_source,before)
   assert all(n in objects and objects[n][1]==size for n,size in
              (('g_stage3_recovery_depth',12),('g_stage3_locked_run',4))), 'Recovery symbols do not match firmware'
   for i,value in enumerate(recovery_depths):write('g_stage3_recovery_depth',value,i,True)
   recover_placed_admission(recovery_source,snap())
   write('g_stage3_request',image['stage3_recover_placed_request'])
   done=lambda s:recover_placed_completed(before,s)
  elif action in ('center_only','all_three'):
   idle();write('g_stage3_test_first_item',1 if a.center_only else 0);write('g_stage3_test_count',1 if a.center_only else 3)
  elif action=='teach_kind':
   idle();assert before['g_stage3_state'] in (0,7,11)
   assert not before['g_stage3_teach_request'],'Teaching request still pending'
   if a.teach_kind in (2,4,6,7):write('g_stage3_teach_measured_x',a.measured_x_mm,floating=True)
   write('g_stage3_teach_request',a.teach_kind)
   done=teach_completed
  elif action=='reobserve':
   idle();assert before['g_stage3_state']==7 and not before['g_stage3_config_ready']
   assert not before['g_stage3_manual_state'] and before['g_low_wheel_hold_ready']
   assert before['g_it_state']==2 and not before['g_stage3_request']
   prior=before['g_stage3_pose'];write('g_stage3_request',31)
   done=lambda s:not s['g_stage3_request'] and s['g_stage3_pose']!=prior and s['g_stage3_state'] in (1,2,3,4,7)
  elif action in ('manual_teach_release','manual_teach_low','manual_teach_exit'):
   idle();assert before['g_stage3_state']==7 and not before['g_stage3_manual_request']
   if action in ('manual_teach_release','manual_teach_low'):
    assert before['g_stage3_manual_state']==0 and before['g_low_wheel_hold_ready']==1
    write('g_stage3_manual_request',3 if action=='manual_teach_low' else 1)
    done=lambda s:s['g_stage3_manual_state']==3 and s['g_stage3_manual_x']==2 and s['g_dm_can2_motor_state']==0 and not s['g_dm_posvel_holding']
   else:
    assert before['g_stage3_manual_state']==3
    write('g_stage3_manual_request',2)
    done=lambda s:s['g_stage3_manual_state']==0 and s['g_extension_home_reference_valid']==1 and s['g_extension_hold_confirmed']==1
  elif action in ('dm_release','dm_read'):
   idle();assert before['g_stage3_state']==7,'DM manual teaching requires teaching state'
   if action=='dm_release':
    dm_release_pose(before,image['manual_teach_limits'])
    print('DM release: confirm the gripper is empty and hand-positioning clearance is available; software does not measure these facts.',file=sys.stderr)
   symbol,value=dm_request(action,before);write(symbol,value)
   done=dm_released if action=='dm_release' else dm_read_completed
  elif action=='touchdown_z':
   idle()
   touchdown_admission(a.touchdown_z,before)
   write('g_it_axes',4)
   write('g_it_lift_pulses',a.touchdown_z,floating=True)
   write('g_it_request',TOUCHDOWN_REQUEST)
   done=touchdown_completed
  elif action in ('it_axis','it_return'):
   idle();assert before['g_stage3_state']==7 and before['g_it_state']==2
   if a.it_axis:
    assert a.value is not None
    low,high={'dm':(0,270),'x':(0,170),'z':(3200,image['stage3_lift_max'])}[a.it_axis];assert low<=a.value<=high
    write('g_it_axes',{'dm':1,'x':2,'z':4}[a.it_axis]);write({'dm':'g_it_dm_deg','x':'g_it_extension_mm','z':'g_it_lift_pulses'}[a.it_axis],a.value,floating=True);req=3
   else:req=4
   write('g_it_request',req)
   done=lambda s:it_completed(s,req)
  elif action in ('teach_body_axis','teach_body_finish','teach_wheel_hold'):
   idle()
   assert before['g_stage3_state']==7 and not before['g_stage3_config_ready']
   assert before['g_stage3_align_done'] and before['g_it_state']==2 and not before['g_it_cause'] and not before['g_it_rejected']
   assert not before['g_chassis_auto_active'] and before['g_source_body_step_state']==3
   assert not before['g_source_body_step_request'] and not before['g_source_body_step_reason']
   assert abs(before['g_stage3_actual_yaw']-180)<=1 and abs(before['g_stage3_actual_x'])<=0.5
   assert before['g_lift_compare_top_valid'] and before['g_lift_compare_backoff_valid']
   assert abs(before['lift_depth_pulses']-600)<=22.5*3200/360
   if action=='teach_body_axis':
    assert a.distance_mm is not None and 1<=a.distance_mm<=20
    assert before['g_source_align_chain_active'] or before['g_low_wheel_hold_ready']
    prior=before['g_source_align_command_seq']
    write('g_source_align_direction',{'right':2,'forward':0,'left':1,'backward':3}[a.teach_body_axis])
    write('g_source_align_distance_mm',a.distance_mm,floating=True)
    write('g_source_body_step_request',2)
    done=lambda s:s['g_source_align_command_seq']!=prior and s['g_source_body_step_state']==3 and not s['g_chassis_auto_active']
   elif action=='teach_body_finish':
    assert before['g_source_align_chain_active']
    write('g_source_body_step_request',7)
    done=lambda s:not s['g_source_align_chain_active'] and not s['g_source_body_step_request']
   else:
    assert not before['g_source_align_chain_active']
    prior=before['g_low_wheel_hold_cycle'];write('g_low_wheel_hold_request',1)
    done=lambda s:s['g_low_wheel_hold_cycle']!=prior and s['g_low_wheel_hold_stage']==13 and s['g_low_wheel_hold_ready']==1
  elif action in ('body_axis','turn'):
   assert before['g_stage3_state']==2 and before['g_it_state']==2 and not before['g_source_body_step_request']
   old=before['g_stage3_pose']
   if a.body_axis:
    assert a.distance_mm is not None and 1<=a.distance_mm<=30
    write('g_source_align_direction',{'right':2,'forward':0,'left':1,'backward':3}[a.body_axis]);write('g_source_align_distance_mm',a.distance_mm,floating=True);req=20
   else:
    assert 1<abs(a.turn)<=10;write('g_source_align_turn_deg',a.turn,floating=True);req=21
   write('g_stage3_request',req)
   done=lambda s:s['g_stage3_state']==2 and s['g_stage3_pose']!=old and not s['g_source_body_step_request']
  elif action=='continue_cycle':
   idle();assert before['g_stage3_state']==7;write('g_stage3_request',2)
   done=lambda s:continue_completed(before,s)
  elif action=='enter_teach':
   assert image['teach_entry_request']==30,'Selected firmware manifest does not declare teaching entry'
   assert before['g_stage3_state']==2 and before['g_it_state']==2
   assert not before['g_stage3_request'] and not before['g_it_request']
   write('g_stage3_request',30);done=enter_teach_completed
  elif action in ('carrier_slot','grip'):
   idle();assert before['g_stage3_state']==7 and before['g_it_state']==2
   if action=='carrier_slot':
    write('g_it_carrier_deg',{1:0.0,2:120.0,3:240.0}[a.carrier_slot],floating=True);req=10
   else:req=grip_request(a.grip,image)
   write('g_it_request',req)
   done=lambda s:it_completed(s,req)
  elif action=='new_round':
   idle();assert before['g_stage3_state']==6;write('g_stage3_request',1)
   done=lambda s:s['g_stage3_state'] in (1,2)
  elif action=='stop':
   write('g_stage3_request',9);done=lambda s:stop_acknowledged(before,s)
  elif action in ('configure_round_trip','configure_profile'):
   idle()
   three_place_preset=(action=='configure_round_trip' and
    image.get('capabilities',{}).get('stage3_three_place') and before['g_stage3_state']==0)
   assert before['g_region_dispatch_area']==3 and (three_place_preset or before['g_stage3_state']==7)
   assert before['g_region_dispatch_test_enable'] and not before['g_stage3_manual_state']
   assert not before['g_stage3_request'] and not before['g_stage3_fault']
   if three_place_preset:
    assert not before['g_stage3_run'] and before['g_it_state']==0
    assert not before['g_stage3_replay_enable'] and not any(before.get(n) for n in ERROR_FIELDS)
   else:
    assert before['g_stage3_run'] and before['g_stage3_align_done']
    assert before['wheel_hold_results']==[1]*4 and before['g_low_wheel_hold_ready']
    assert not before['g_source_align_chain_active'] and before['g_lift_compare_top_valid']
    assert abs(before['lift_depth_pulses']-600)<=22.5*3200/360
   calibration,mission=profile
   if profile_report is not None:
    require_profile_pose(profile_report,before,datetime.now())
    assert before['g_it_state']==2 and not any(before.get(n) for n in ERROR_FIELDS)
   task_run=1 if three_place_preset else (0 if profile_source and profile_source['mode']=='single-red'
                                        else before['g_stage3_run'])
   # Write pick depth last so interrupted publication leaves the profile unready.
   write('g_stage3_pick_depth',-1.0,floating=True)
   if 'g_stage3_task_run' in objects:write('g_stage3_task_run',0)
   for fields,floating in ((calibration,True),(mission,False)):
    for name,value in fields.items():
     if name=='g_stage3_pick_depth':continue
     for i,v in enumerate(value if isinstance(value,list) else [value]):write(name,v,i,floating)
   if profile_report is not None:
    require_profile_pose(profile_report,snap(),datetime.now())
   if 'g_stage3_task_run' in objects and task_run:write('g_stage3_task_run',task_run)
   write('g_stage3_pick_depth',calibration['g_stage3_pick_depth'],floating=True)
   def done(s):
    return (s['g_stage3_state']==before['g_stage3_state'] and s['g_stage3_config_ready'] and s.get('g_stage3_task_run',0)==task_run and
     (profile_report is None or all(s.get(k)==before.get(k) for k in PROFILE_CONTEXT)) and
     all(s[n]==v for n,v in mission.items()) and
     all(s[n]==([struct.unpack('<f',struct.pack('<f',x))[0] for x in v] if isinstance(v,list) else
                 struct.unpack('<f',struct.pack('<f',v))[0]) for n,v in calibration.items()))
  elif action=='restore_calibration':
   idle();saved=profile_json(a.restore_calibration)
   for n in calibration_values(saved,image):
    values=saved[n] if isinstance(saved[n],list) else [saved[n]]
    for i,v in enumerate(values):write(n,v,i,True)
  outcome=ActionOutcome(action,before,done,req if action in ('carrier_slot','grip') else None)
  deadline=time.monotonic()+(a.watch if a.watch else a.timeout)
  while True:
   after=snap();samples.append(after)
   outcome.observe(after)
   if action and action!='stop' and outcome.first_error:break
   if not a.watch and outcome.completed:break
   if time.monotonic()>=deadline:break
   time.sleep(.15)
  report=dict(when=datetime.now().isoformat(),image=image,verified_flash_bytes=board.verified_flash_bytes,
              action=action,writes=writes,before=before,after=after,samples=samples,
              task_code=profile_source.get('task_code') if profile_source else a.configure_round_trip,
              configured_profile=str(a.configure_profile.resolve()) if a.configure_profile else None,
              profile_mode=profile_source['mode'] if profile_source else None,
              cycle_mode=profile[1].get('g_stage3_cycle_mode') if profile else None,
              recovery_depth_source=('calibration/stage3_roughing_ring_teach_20261118.json; ring 2/3 physically unverified'
                                     if action=='configure_round_trip' and a.cycle_mode=='recover' else None),
              profile_evidence=profile_source.get('evidence') if profile_source else None,
              independent_recovery=recovery_source,
              actual_slot_colors=profile_source['slot_colors'] if profile_source else None,
              completed=outcome.completed,first_error=outcome.first_error,
              completion_scope=('accepted_and_started' if action in ('continue_cycle','new_round','start_ring2_replay') else
                                'lift_target_feedback_only' if action=='touchdown_z' else
                                'replay_latch_armed_only' if action=='arm_ring2_replay' else
                                'software_stop_acknowledged' if action=='stop' else 'request_completed'),
              physical_stop_verified=False,reset=False,flashed=False)
  if action=='touchdown_z':
   report.update(physical_contact_verified=False,touchdown_target_pulses=a.touchdown_z)
  if has_replay:report['ring2_replay']=replay_status(after)
  stamp=datetime.now().strftime('%Y%m%d-%H%M%S');out=E/('stage3-live-'+stamp+'.json')
  out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
  if action=='teach_kind' and report['completed']:
   config=calibration_profile(after)
   (E/'stage3-calibration.json').write_text(json.dumps(config,indent=2)+'\n')
  summary={k:report[k] for k in ('when','image','action','completed','completion_scope','physical_stop_verified')}
  summary['first_error']=outcome.first_error['values'] if outcome.first_error else None
  summary['after']={k:after[k] for k in ('g_stage3_state','g_stage3_phase','g_stage3_item',
       'g_stage3_fault','g_it_state','g_it_action','g_it_request','g_it_sequence','g_it_cause',
       'g_it_rejected','g_source_body_step_state','g_vehicle_arm_stop_pending','g_lift_ruler_stop_pending')}
  summary['evidence']=str(out);summary['samples']=len(samples)
  if has_replay:summary['ring2_replay']=report['ring2_replay']
  if action=='touchdown_z':
   summary.update(physical_contact_verified=False,touchdown_target_pulses=a.touchdown_z,
                  lift_depth_pulses=after['lift_depth_pulses'])
  if action=='stop':summary['software_stop_acknowledged']=stop_acknowledged(before,after)
  if action in ('dm_release','dm_read'):
   summary['dm_feedback']={n:after[n] for n in (*DM_MANUAL_FIELDS,'g_dm_can2_position_deg','g_dm_can2_logical_position_deg') if n in after}
  print(json.dumps(summary,ensure_ascii=False,indent=2))
  if action and not report['completed']:raise SystemExit('Action did not complete; first observed error retained, no automatic retry')
if __name__=='__main__':main()
