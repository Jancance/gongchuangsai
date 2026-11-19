"""One explicit fixed-near pzq trigger; passive host monitoring, no per-step commands."""
import argparse,json,hashlib,struct,time
from pathlib import Path
from datetime import datetime
import gripper210_attach_read_20261105 as live
from source_body_probe_20261111 import attach
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-13/near-pzq'
def run(slot):
 m=json.loads((E/'candidate.json').read_text());assert m['build_passed'] and json.loads((E/'checks.json').read_text())['passed']
 im=m['images']['Vehicle_LiftRuler'];live.IMAGE=R/im['axf'];assert hashlib.sha256(live.IMAGE.read_bytes()).hexdigest()==im['sha256']
 syms,segs=live.load_image();out=E/('near-pzq-slot'+str(slot)+'-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
 d={'slot':slot,'image_sha256':im['sha256'],'complete':False,'one_board_trigger':True,'flashed':False,'reset':False,'physical_placement_pending':True,'trace':[]}
 def save():out.write_text(json.dumps(d,indent=2),encoding='utf-8')
 save();wrote=False
 try:
  with attach() as session:
   t=session.target;assert t.get_state().name=='RUNNING'
   count=0
   for a,b in segs:
    for i in range(0,len(b),4096):
     q=b[i:i+4096];assert bytes(t.read_memory_block8(a+i,len(q)))==q,'wrong image';count+=len(q)
   d['verified_flash_bytes']=count
   keys='g_pzq_near_request g_pzq_near_state g_pzq_near_rejected g_pzq_near_accepted_seq g_it_request g_it_state g_it_cause g_it_rejected g_it_sequence g_it_auto_state g_it_auto_phase g_it_auto_fault g_it_auto_elapsed_ms g_safety_ready g_safety_fault g_rtos_fault g_can_ctrl_fault_latched g_can_test_result g_lift_ruler_cause g_lift_ruler_stop_pending g_source_parallel_cause g_source_parallel_stop_mask g_vehicle_arm_cause g_vehicle_arm_stop_pending g_startup_axes_cause g_dm_jog_first_fault g_chassis_auto_active g_lift_compare_top_valid g_lift_compare_backoff_valid g_extension_hold_confirmed g_extension_home_reference_valid'.split()
   floats='g_dm_can2_logical_position_deg g_startup_extension_current_deg g_extension_home_reference_deg'.split()
   def snap():
    pairs=sorted((syms[k],k) for k in keys+floats);groups=[];v={}
    for address,key in pairs:
     if not groups or address-groups[-1][-1][0]>12 or address-groups[-1][0][0]>128:groups.append([])
     groups[-1].append((address,key))
    for group in groups:
     start=group[0][0];words=t.read_memory_block32(start,(group[-1][0]-start)//4+1)
     for address,key in group:
      x=words[(address-start)//4];v[key]=struct.unpack('<f',struct.pack('<I',x))[0] if key in floats else x
    v['release_ms']=list(t.read_memory_block32(syms['g_it_auto_release_ms'],3));v['pe2']=(t.read32(0x40021010)>>2)&1;v['gripper_pwm_us']=t.read32(0x40000034);return v
   faults='g_it_cause g_it_rejected g_it_auto_fault g_safety_fault g_rtos_fault g_can_ctrl_fault_latched g_lift_ruler_cause g_lift_ruler_stop_pending g_source_parallel_cause g_source_parallel_stop_mask g_vehicle_arm_cause g_vehicle_arm_stop_pending g_startup_axes_cause g_dm_jog_first_fault g_chassis_auto_active'.split()
   def healthy(v):return not any(v[k] for k in faults) and v['g_safety_ready']==1 and v['g_can_test_result']==1 and v['pe2']==1
   before=snap();d['before']=before;save();assert healthy(before) and before['g_it_state']==2 and before['g_it_auto_state'] in (0,2)
   assert before['g_pzq_near_state'] in (0,2) and not before['g_pzq_near_request'] and not before['g_it_request']
   assert before['g_lift_compare_top_valid']==1 and before['g_lift_compare_backoff_valid']==1 and before['g_extension_hold_confirmed']==1 and before['g_extension_home_reference_valid']==1
   assert abs(before['g_dm_can2_logical_position_deg']-180)<1 and abs(before['g_startup_extension_current_deg']-before['g_extension_home_reference_deg'])<=2 and before['gripper_pwm_us']==2056
   seq=(before['g_pzq_near_accepted_seq']+1)&0xffffffff;seq=seq or 1;d['command_seq']=seq
   try:
    t.write32(syms['g_pzq_near_slot'],slot);t.write32(syms['g_pzq_near_command_seq'],seq);wrote=True;t.write32(syms['g_pzq_near_request'],1)
    start=time.monotonic();last_phase=None
    while time.monotonic()-start<55:
     v=snap();d['trace'].append(v)
     if not healthy(v) or v['g_pzq_near_state']==9:raise RuntimeError('near pzq actual fault')
     if v['g_pzq_near_rejected']!=before['g_pzq_near_rejected']:raise RuntimeError('near pzq request rejected')
     phase=(v['g_pzq_near_state'],v['g_it_auto_phase'])
     if phase!=last_phase:print('board near/phase',phase,'elapsed_ms',v['g_it_auto_elapsed_ms'],flush=True);last_phase=phase
     if v['g_pzq_near_state']==2 and v['g_pzq_near_accepted_seq']==seq and v['g_it_auto_state']==2:
      assert v['release_ms'][slot-1]>0 and v['g_it_state']==2 and v['gripper_pwm_us']==2056
      assert abs(v['g_dm_can2_logical_position_deg']-180)<1 and abs(v['g_startup_extension_current_deg']-v['g_extension_home_reference_deg'])<=2
      d.update(complete=True,after=v,release_elapsed_ms=v['release_ms'][slot-1],board_elapsed_ms=v['g_it_auto_elapsed_ms']);break
     time.sleep(.12)
    else:raise TimeoutError('single near pzq command')
   except BaseException as error:
    d['error']=repr(error)
    if wrote:
     try:
      t.write32(syms['g_pzq_near_request'],9);t.write32(syms['g_it_request'],9);d['stop_requested']=True
      for _ in range(12):
       time.sleep(.15);v=snap();d['after_stop']=v
       if not v['g_lift_ruler_stop_pending'] and not v['g_source_parallel_stop_mask'] and not v['g_vehicle_arm_stop_pending']:break
     except BaseException as err:d['stop_error']=repr(err)
    raise
   finally:
    try:d['last']=snap()
    except BaseException as err:d['last_error']=repr(err)
 except BaseException as err:d.setdefault('error',repr(err));raise
 finally:save();print(out,flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--once-slot',type=int,choices=[1,2,3]);a=p.parse_args()
 if a.once_slot:run(a.once_slot)
 else:print('CHECK only. Explicit --once-slot N sends one board pzq trigger; no automatic flash/reset/body.')
