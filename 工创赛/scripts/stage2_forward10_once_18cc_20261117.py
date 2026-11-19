"""One explicitly authorized 18cc forward10 step. Default is local only."""
from pathlib import Path
from datetime import datetime
import argparse,json,sys,time,struct
import watch_stage2_debug215_readonly_20261117 as checked
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-17/stage2-settle-ring'
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--forward10',action='store_true');a=p.parse_args()
 checked.E=E;image=checked.manifest()
 sys.path.insert(0,str(R/'src'))
 import gripper210_attach_read_20261105 as live
 from source_body_probe_20261111 import attach
 from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard
 live.IMAGE=Path(image['axf']);symbols,segments=live.load_image()
 from elftools.elf.elffile import ELFFile
 with live.IMAGE.open('rb') as stream:
  elf=ELFFile(stream);objects={s.name:(int(s['st_value']),int(s['st_size'])) for s in elf.get_section_by_name('.symtab').iter_symbols() if s['st_info']['type']=='STT_OBJECT'}
 scalar_names=('g_source_cycle_state g_source_cycle_fault g_source_cycle_pose g_source_body_step_session '
  'g_chassis_auto_session g_can_test_result g_source_align_rejected s_motion_has_action s_combined_turn '
  'g_vehicle_lift_reference_valid g_vehicle_lift_feedback_valid g_lift_ruler_request '
  'g_source_align_direction g_source_align_distance_mm g_source_body_step_request').split()
 for name in scalar_names:
  assert name in objects and objects[name][1]==4,('Missing/non-scalar ELF object',name)
  symbols[name]=objects[name][0]
 assert 'g_forward_hold' in objects and objects['g_forward_hold'][1]>=16,'Missing ForwardHold header'
 symbols['g_forward_hold']=objects['g_forward_hold'][0]
 if not a.forward10:print(json.dumps({'local_check':True,'hardware_access':False,'sha256':image['sha256'],'one_explicit_forward10_required':True,'required_elf_objects_checked':len(scalar_names)+1}));return
 dest=E/('forward10-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
 report={'image_sha256':image['sha256'],'reset':False,'flash':False,'writes':[],'complete':False,'trace':[]}
 def save():dest.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
 save()
 try:
  with attach() as session:
   t=session.target;board=CommittedPickBoard(t,symbols,segments);report['verified_flash_bytes']=board.verified_flash_bytes
   def snap():
    s=board.snapshot()
    for k in ('g_source_cycle_state','g_source_cycle_fault','g_source_cycle_pose','g_source_body_step_session','g_chassis_auto_session','g_can_test_result','g_source_align_rejected','s_motion_has_action','s_combined_turn','g_vehicle_lift_reference_valid','g_vehicle_lift_feedback_valid','g_lift_ruler_request'):
     s[k]=t.read32(symbols[k])
    s['forward_hold_header']=list(t.read_memory_block32(symbols['g_forward_hold'],4));return s
   b=snap();report['before']=b;save()
   assert board.pose_ready(b) and b['g_source_cycle_state']==2 and b['g_source_cycle_fault']==0 and b['g_can_test_result']==1
   assert b['g_vehicle_lift_reference_valid']==1 and b['g_vehicle_lift_feedback_valid']==1 and not b['g_lift_ruler_request']
   assert not any(b[k] for k in ('g_lift_ruler_stop_pending','g_vehicle_arm_stop_pending','g_source_parallel_stop_mask','g_source_body_step_reason'))
   assert not b['g_source_body_step_request'] and not b['g_chassis_auto_active'] and b['g_source_body_step_state'] in (0,3)
   if b['g_source_body_step_state']==3:assert b['body_stop_masks'][2]==15 and b['g_chassis_auto_result']==1
   beforeseq=b['g_source_align_command_seq'];before_session=b['g_chassis_auto_session'];rejected=b['g_source_align_rejected']
   # The existing adapter writes only direction0, distance10.0 and SBS request2.
   report['request_attempted']=True;save();seq=board.begin_body_step(0,10);assert seq==beforeseq
   report['writes']=[['g_source_align_direction',0],['g_source_align_distance_mm',10.0],['g_source_body_step_request',2]];save()
   end=time.monotonic()+14
   while time.monotonic()<end:
    s=snap();report['trace'].append(s)
    faults=('g_source_cycle_fault','g_source_body_step_reason','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_it_cause','g_startup_axes_cause','g_lift_ruler_cause','g_dm_jog_first_fault')
    if any(s[k] for k in faults) or s['g_source_align_rejected']!=rejected or s['g_source_body_step_state'] in (8,9):report['first_fault']=s;raise RuntimeError('Existing controller fault/rejection; no repeat request')
    expected=(beforeseq+1)&0xffffffff;expected=expected or 1
    done=s['g_source_align_command_seq']==expected and s['g_source_body_step_state']==3 and not s['g_chassis_auto_active'] and s['g_chassis_auto_result']==1 and s['body_stop_masks'][2]==15 and s['g_source_body_step_session']==s['g_chassis_auto_session'] and s['g_chassis_auto_session']!=before_session
    if done:
     # MotionGetState uses has_action, non-turn and ForwardHold phase HR_DONE=3.
     assert s['s_motion_has_action']==1 and s['s_combined_turn']==0 and s['forward_hold_header'][0]!=1 and s['forward_hold_header'][1]==3 and s['forward_hold_header'][2]==0
     report.update(complete=True,software_motion_done=True,backend_result=1,stop_mask=15,completed_host_time=datetime.now().isoformat(),completed_host_monotonic_s=time.monotonic(),after=s,physical_displacement_pending_user=True);break
    time.sleep(.08)
   else:report['monitor_deadline_reached']=True;report['controller_may_continue']=True;raise TimeoutError('No confirmed DONE; no retry or synthetic stop')
 except BaseException as ex:report['error']=repr(ex);raise
 finally:save();print(dest,flush=True)
if __name__=='__main__':main()
