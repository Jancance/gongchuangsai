"""One low-speed escape from user-confirmed empty top; never home or fake origin."""
import sys,json,time,hashlib
import single_pickup_to_top_20261107 as p
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target
IMAGE=p.R/'tmp/integration11_release_step_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
EXPECTED='1a8c827c2a3339685308bb37544fb676bbab0b87ae64641567c57b00ac5daaf5'
assert json.loads((p.O/'integration11-release-step-matlab.json').read_text())['passed']
assert hashlib.sha256(IMAGE.read_bytes()).hexdigest()==EXPECTED
p.transport.live.IMAGE=IMAGE;n,segs=p.transport.live.load_image()
o={'target_override':'cortex_m','connect_mode':'attach','frequency':1000000,'auto_unlock':False,'resume_on_disconnect':False,'no_config':True,'cmsis_dap.prefer_v1':False,'cmsis_dap.limit_packets':True}
Session(None,options=o);a=ConnectHelper.get_all_connected_probes(blocking=False);assert len(a)==1 and a[0].unique_id=='6afaae24';p.transport.synchronous_interface(a[0]._link._interface);s=Session(a[0],options=o)
r={'image_sha256':EXPECTED,'writes':[],'home_requests':0,'user_confirmed_empty_top':True,'physical_mm_verified':False};t=None;requested=False
with (p.O/'integration11-release-step-01-samples.jsonl').open('x',encoding='utf-8') as log:
 try:
  s.open();t=s.target;assert t.get_state()==Target.State.RUNNING
  for addr,b in segs:
   for k in range(0,len(b),4096):assert bytes(t.read_memory_block8(addr+k,len(b[k:k+4096])))==b[k:k+4096]
  r['Flash_matches']=True
  def get(k):return t.read32(n[k])
  def put(k,v):t.write32(n[k],v);r['writes'].append([k,v])
  def snap():
   d={k:get(k) for k in p.NAMES};d['PE2']=bool(t.read32(0x40021014)&4)
   d['flags']=t.read32(n['g_lift_ruler_controller']+64);d['moves_sent']=t.read32(n['g_lift_ruler_controller']+72)
   d['speed_raw']=t.read32(n['g_lift_ruler_controller']+68);return d
  def diag():
   old=get('g_lift_diag_cycle');put('g_lift_diag_request',1);start=time.monotonic()
   while time.monotonic()-start<4:
    if get('g_lift_diag_cycle')!=old:break
    time.sleep(.05)
   assert get('g_lift_diag_cycle')!=old and get('g_lift_diag_done')==1
   results=t.read_memory_block32(n['g_lift_diag_result'],6);raw=t.read_memory_block32(n['g_lift_diag_raw'],48)
   assert results==[1]*6,results
   return {'results':results,'raw':[raw[i*8:i*8+8] for i in range(6)]}
  r['before']=d=snap()
  for k in ['g_lift_ruler_state','g_lift_ruler_request','g_it_state','g_it_request','g_startup_axes_state','g_dm_jog_state','g_dm_jog_request','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched']:assert d[k]==0,k
  assert not d['PE2'] and get('g_vm_test_request')==0
  t.write32(n['g_lift_ruler_controller']+4,1);r['writes'].append(['actual lift VM',1]);start=time.monotonic()
  while time.monotonic()-start<6:
   if get('g_vm_test_request')==1 and t.read32(0x40021014)&4 and get('g_can_guard_ready'):break
   time.sleep(.1)
  assert get('g_can_guard_ready')==1 and t.read32(0x40021014)&4
  r['fresh_before']=fresh=diag();raw=fresh['raw']
  assert not raw[0][1]&12,'actual driver protection present; no motion'
  assert (raw[1][1]&3)==3 and not raw[1][1]&0x3c,'home not inactive/healthy'
  assert (raw[2][2]<<8|raw[2][3])==0,'actual motor not stationary'
  put('g_it_lift_permit',1);put('g_lift_ruler_request',4);requested=True;start=time.monotonic()
  while time.monotonic()-start<18:
   d=snap();d['elapsed']=time.monotonic()-start;log.write(json.dumps(d)+'\n');log.flush();r['last']=d
   assert not any(d[k] for k in ['g_lift_ruler_cause','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched']),'native operation fault'
   if d['g_lift_ruler_state']==4 and d['g_lift_compare_operation']==4 and d['g_lift_ruler_samples']==1:
    r['software_release_done']=True;break
   time.sleep(.05)
  assert r.get('software_release_done'),'escape timeout'
  assert d['g_lift_compare_top_valid']==0 and d['g_lift_compare_home_reported']==0
  put('g_it_lift_permit',0);time.sleep(1.5);r['fresh_after']=diag();r['final']=snap()
 except Exception as e:r['error']=repr(e)
 finally:
  if t is not None and not r.get('software_release_done'):
   if requested:put('g_lift_ruler_request',2);time.sleep(.2)
   t.write32(n['g_lift_ruler_controller']+4,0);t.write32(n['g_startup_axes_vm_request'],0);put('g_it_lift_permit',0)
   for i in range(30):
    time.sleep(.1)
    if not get('g_vm_test_request') and not t.read32(0x40021014)&4:break
   r['failure_VM_off']=not get('g_vm_test_request') and not t.read32(0x40021014)&4
  if t is not None:
   r['first_exchange']={k:get(k) for k in ['g_lift_exchange_first_valid','g_lift_exchange_first_cmd','g_lift_exchange_first_result','g_lift_exchange_first_phase']};r['first_reply']=t.read_memory_block32(n['g_lift_exchange_first_raw'],8)
  s.close();(p.O/'integration11-release-step-01-result.json').write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps({k:v for k,v in r.items() if k not in ['before','last']}),flush=True)
