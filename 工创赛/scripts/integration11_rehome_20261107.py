"""One supervised reset and native homing; never clear fault/reference RAM fields."""
import sys,json,time,hashlib
import single_pickup_to_top_20261107 as p
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target
assert json.loads((p.O/'integration11-resume-matlab.json').read_text())['passed']
assert hashlib.sha256(p.IMAGE.read_bytes()).hexdigest()==p.EXPECTED
p.transport.live.IMAGE=p.IMAGE;n,segs=p.transport.live.load_image()
o={'target_override':'cortex_m','connect_mode':'attach','frequency':1000000,'auto_unlock':False,'resume_on_disconnect':False,'no_config':True,'cmsis_dap.prefer_v1':False,'cmsis_dap.limit_packets':True}
Session(None,options=o);a=ConnectHelper.get_all_connected_probes(blocking=False)
assert len(a)==1 and a[0].unique_id=='6afaae24'
p.transport.synchronous_interface(a[0]._link._interface);s=Session(a[0],options=o)
r={'writes':[],'flash':False,'reset':False,'native_home_done':False};t=None;requested=False
with (p.O/'integration11-rehome-02-samples.jsonl').open('x',encoding='utf-8') as log:
 try:
  s.open();t=s.target;assert t.get_state()==Target.State.RUNNING
  for addr,b in segs:
   for k in range(0,len(b),4096):assert bytes(t.read_memory_block8(addr+k,len(b[k:k+4096])))==b[k:k+4096]
  def snap():
   d={k:t.read32(n[k]) for k in p.NAMES}
   d['PE2']=bool(t.read32(0x40021014)&4)
   d['gripper_pwm']=t.read32(0x40000034)
   return d
  r['before_reset']=d=snap()
  assert d['g_vm_test_request']==0 and not d['PE2'] and d['g_it_lift_permit']==0
  assert d['g_lift_ruler_stop_pending']==0 and d['g_dm_jog_request']==0 and d['g_chassis_auto_active']==0
  t.reset(reset_type=Target.ResetType.SYSRESETREQ);r['reset']=True
  time.sleep(3);assert t.get_state()==Target.State.RUNNING
  r['after_reset']=d=snap()
  for k in ['g_it_state','g_it_request','g_it_cause','g_lift_ruler_state','g_lift_ruler_cause','g_lift_compare_top_valid','g_startup_axes_state','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched']:assert d[k]==0,k
  assert t.read32(n['g_safety_ready'])==1 and t.read32(n['g_can_test_result'])==1
  # Only the existing native homing request; no ordinary position/rotation request.
  t.write32(n['g_it_request'],1);requested=True;r['writes'].append(['g_it_request',1])
  start=time.monotonic()
  while time.monotonic()-start<65:
   d=snap();d['elapsed']=time.monotonic()-start;log.write(json.dumps(d)+'\n');log.flush();r['last']=d
   if any(d[k] for k in ['g_it_cause','g_lift_ruler_cause','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched']):raise RuntimeError('native homing fault; no automatic retry')
   if d['g_it_state']==2 and d['g_lift_ruler_state']==4 and d['g_lift_compare_top_valid']==1:
    r['native_home_done']=True;break
   time.sleep(.05)
  assert r['native_home_done'],'homing timeout'
 except Exception as e:r['error']=repr(e)
 finally:
  if t is not None and r.get('reset') and not r['native_home_done']:
   if requested:
    t.write32(n['g_lift_ruler_request'],2);r['writes'].append(['g_lift_ruler_request',2]);time.sleep(.2)
   t.write32(n['g_lift_ruler_controller']+4,0);t.write32(n['g_startup_axes_vm_request'],0);t.write32(n['g_it_lift_permit'],0)
   for i in range(30):
    time.sleep(.1)
    if t.read32(n['g_vm_test_request'])==0 and not t.read32(0x40021014)&4:break
   r['failure_VM_off']=t.read32(n['g_vm_test_request'])==0 and not t.read32(0x40021014)&4
  s.close();(p.O/'integration11-rehome-02-result.json').write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r),flush=True)
