"""Read-only helpers for an EXISTING probe session; never attaches or writes."""
import struct,json,sys,argparse
from pathlib import Path
from datetime import datetime
R=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(R/'tmp/gripper_debug_deps'))

def decode_lrc(path,raw):
 from elftools.elf.elffile import ELFFile
 with open(path,'rb') as stream:
  dwarf=ELFFile(stream).get_dwarf_info()
  for cu in dwarf.iter_CUs():
   for die in cu.iter_DIEs():
    name=die.attributes.get('DW_AT_name')
    if die.tag=='DW_TAG_typedef' and name and name.value==b'LiftRulerController':
     typ=die.get_DIE_from_attribute('DW_AT_type')
     result={}
     for field in typ.iter_children():
      if field.tag!='DW_TAG_member':continue
      key=field.attributes['DW_AT_name'].value.decode()
      if key not in ('state','cause','top_valid','top_scale','top_position_raw','safe_position_raw','backoff_valid'):continue
      offset=field.attributes['DW_AT_data_member_location'].value
      if not isinstance(offset,int):
       from elftools.dwarf.dwarf_expr import DWARFExprParser
       ops=DWARFExprParser(dwarf.structs).parse_expr(offset)
       assert len(ops)==1 and ops[0].op_name=='DW_OP_plus_uconst'
       offset=ops[0].args[0]
      size=8 if key.endswith('_raw') else 4
      result[key]=int.from_bytes(bytes(raw[offset:offset+size]),'little',signed=size==8)
     assert 'top_position_raw' in result and result['top_scale'] in (10,100)
     return result
 raise ValueError('LRC type absent in DWARF')

def decode_vsp(path,raw):
 from elftools.elf.elffile import ELFFile
 from elftools.dwarf.dwarf_expr import DWARFExprParser
 with open(path,'rb') as stream:
  dwarf=ELFFile(stream).get_dwarf_info()
  for cu in dwarf.iter_CUs():
   for die in cu.iter_DIEs():
    name=die.attributes.get('DW_AT_name')
    if die.tag!='DW_TAG_typedef' or not name or name.value!=b'VSP_Axis':continue
    typ=die.get_DIE_from_attribute('DW_AT_type');size=typ.attributes['DW_AT_byte_size'].value
    assert len(raw)==2*size
    axes=[]
    for i in range(2):
     result={}
     for field in typ.iter_children():
      if field.tag!='DW_TAG_member':continue
      key=field.attributes['DW_AT_name'].value.decode()
      if key=='move':continue
      offset=field.attributes['DW_AT_data_member_location'].value
      if not isinstance(offset,int):
       ops=DWARFExprParser(dwarf.structs).parse_expr(offset)
       assert len(ops)==1 and ops[0].op_name=='DW_OP_plus_uconst'
       offset=ops[0].args[0]
      result[key]=struct.unpack_from('<f' if key in ('before','goal','current') else '<I',bytes(raw),i*size+offset)[0]
     axes.append(result)
    return {'extension':axes[0],'lift':axes[1]}
 raise ValueError('VSP_Axis type absent')

UINTS=('g_lift_diag_request','g_lift_diag_cycle','g_lift_diag_done',
 'g_recovery_diag_request','g_recovery_diag_cycle','g_recovery_diag_done',
 'g_dm_can2_read_request','g_dm_can2_read_cycle','g_dm_can2_state','g_dm_can2_result',
 'g_dm_can2_feedback_valid','g_dm_can2_feedback_tick','g_dm_posvel_holding',
 'g_dm_can2_motor_state','g_dm_jog_state','g_dm_jog_result',
 'g_vehicle_lift_feedback_valid','g_startup_extension_feedback_valid',
 'g_lift_ruler_state','g_lift_ruler_cause','g_lift_ruler_top_valid',
 'g_startup_axes_state','g_startup_axes_cause','g_extension_home_reference_valid',
 'g_extension_hold_confirmed','g_vm_test_request','g_src_now_ms',
 'g_it_sequence','g_it_action','g_it_state','g_it_request','g_it_cause','g_it_rejected',
 'g_source_parallel_state','g_source_parallel_cause','g_source_parallel_done_mask','vsp_owned',
 's3_teach_pending','s3_teach_z_cycle','s3_teach_x_cycle','s3_teach_dm_cycle')
FLOATS=('g_vehicle_lift_current_deg','g_startup_extension_current_deg',
 'g_extension_home_reference_deg','g_dm_can2_logical_position_deg',
 'g_stage3_actual_depth','g_stage3_actual_x','g_stage3_actual_yaw','g_stage3_pick_depth')
BLOCKS=('g_lift_diag_result','g_recovery_diag_result','g_lift_diag_raw',
 'g_recovery_diag_raw','g_lift_ruler_controller')
BLOCKS=BLOCKS+('vsp_axis',)

def objects_from_elf(path):
 from elftools.elf.elffile import ELFFile
 with open(path,'rb') as stream:
  elf=ELFFile(stream)
  return {s.name:(int(s['st_value']),int(s['st_size'])) for s in elf.get_section_by_name('.symtab').iter_symbols() if s['st_info']['type']=='STT_OBJECT'}

def snapshot(target,objects,image=None):
 result={'hardware_writes':[],'physical_motion_requested':False,'missing_symbols':[]}
 for name in UINTS+FLOATS+BLOCKS:
  if name not in objects:
   result['missing_symbols'].append(name);continue
  address,size=objects[name]
  if name in UINTS:result[name]=target.read32(address)
  elif name in FLOATS:result[name]=struct.unpack('<f',struct.pack('<I',target.read32(address)))[0]
  else:
   raw=list(target.read_memory_block8(address,size));result[name]=raw
   if name.endswith('_result') or (name=='g_lift_diag_raw' and size==192):
    result[name]=[struct.unpack('<I',bytes(raw[i:i+4]))[0] for i in range(0,size,4)]
    if name=='g_lift_diag_raw':result['lift_diag_element_bytes']=4
 # Decode source Z transaction, without assuming the diagnostic succeeded.
 if all(k in result for k in ('g_lift_diag_raw','g_lift_diag_result')):
  raw=result['g_lift_diag_raw'];position=raw[24:32];options=raw[32:40]
  if len(position)==8 and len(options)==8:
   scale=100 if options[2]&0x80 else 10
   result['lift_diag_decoded']={'position_deg':(-1 if position[1] else 1)*int.from_bytes(bytes(position[2:6]),'big')/scale,
    'scale':scale,'position_reply':position,'options_reply':options,
    'transaction_valid':result.get('g_lift_diag_done')==1 and result['g_lift_diag_result'][3:5]==[1,1]}
 if image and 'g_lift_ruler_controller' in result:
  result['lrc_decoded']=decode_lrc(image,result['g_lift_ruler_controller'])
  z=result.get('lift_diag_decoded');ref=result['lrc_decoded']
  if z and z['transaction_valid'] and ref['top_valid']:
   result['measured_pick_depth']=(z['position_deg']-ref['top_position_raw']/ref['top_scale'])*(3200/360)
 if image and 'vsp_axis' in result:
  result['vsp_decoded']=decode_vsp(image,result['vsp_axis'])
  ref=result['lrc_decoded'];axis=result['vsp_decoded']['lift']
  result['last_completed_vsp_pick_depth']=(axis['current']-ref['top_position_raw']/ref['top_scale'])*(3200/360)
 return result

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--manifest',type=Path,required=True)
 p.add_argument('--snapshot',action='store_true')
 p.add_argument('--save-pick-config',action='store_true')
 p.add_argument('--accepted-position-record',type=Path)
 a=p.parse_args()
 from stage3_live_20261116 import manifest,calibration_values
 image=manifest(a.manifest);axf=R/image['axf'];objects=objects_from_elf(axf)
 if not a.snapshot:
  print(json.dumps({'local_only':True,'hardware_access':False,'symbols':len(objects)}));return
 from source_body_probe_20261111 import attach
 with attach() as session:data=snapshot(session.target,objects,axf)
 data.update(when=datetime.now().isoformat(),image=image,firmware_teach_succeeded=False,
  source='Read existing completed Z/X diagnostic replies plus live references; no new MCU requests')
 out=a.manifest.resolve().parent/('teach-readonly-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
 out.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
 summary={k:data.get(k) for k in ('measured_pick_depth','last_completed_vsp_pick_depth','vsp_decoded','lift_diag_decoded','lrc_decoded','g_dm_can2_read_cycle','g_dm_posvel_holding','s3_teach_pending')}
 summary['evidence']=str(out)
 if a.save_pick_config:
  depth=data.get('measured_pick_depth')
  if depth is None and a.accepted_position_record:
   prior=json.loads(a.accepted_position_record.read_text(encoding='utf-8'))
   old=prior['after']
   assert prior['completed'] and not prior['first_error'] and prior['action']=='it_axis'
   assert data['g_it_sequence']==old['g_it_sequence'] and data['g_it_action']==old['g_it_action']==3
   assert data['g_it_state']==2 and data['g_it_request']==0 and not data['g_it_cause'] and not data['g_it_rejected']
   assert data['g_source_parallel_state']==3 and not data['g_source_parallel_cause'] and not data['vsp_owned']
   assert data['g_source_parallel_done_mask']&4 and data['vsp_decoded']['lift']['position_cycle']>0
   depth=data['last_completed_vsp_pick_depth']
   def f32(value):return struct.unpack('<f',struct.pack('<f',value))[0]
   ref=data['lrc_decoded'];current=data['vsp_decoded']['lift']['current']
   assert ref['top_valid']==1 and current==old['g_vehicle_lift_current_deg']
   matched=f32(f32(current-f32(ref['top_position_raw']/ref['top_scale']))*f32(3200/360))
   assert matched==old['g_stage3_actual_depth'],'Live VSP actual and reference do not reproduce accepted historical float measurement'
   data['accepted_position_source']={'record':str(a.accepted_position_record.resolve()),'recorded_actual_depth':old['g_stage3_actual_depth'],
    'recomputed_actual_depth':depth,'difference':depth-old['g_stage3_actual_depth'],
    'fresh_poll':False,'description':'Failed Z diagnostic unused. Existing last completed real VSP feedback at the user accepted height; actual current and matching IT sequence agree with accepted record'}
   out.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
   summary['accepted_position_source']=data['accepted_position_source']
  if depth is None:
   summary['configuration_saved']=False
   summary['unavailable_reason']='Z diagnostic not completed successfully; no accepted completed VSP position record supplied'
   print(json.dumps(summary));return
  assert data['g_lift_ruler_cause']==0 and data['g_lift_ruler_state']==4
  config=calibration_values({'g_stage3_pick_depth':depth},image)
  config_path=out.with_name(out.stem+'-pick-config.json')
  config_path.write_text(json.dumps(config,indent=2)+'\n',encoding='utf-8')
  summary['pick_config']=str(config_path)
 print(json.dumps(summary))

if __name__=='__main__':main()
