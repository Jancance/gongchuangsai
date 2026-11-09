"""Read actual node5 state and native return setting; no motion/enable/home requests."""
import sys,json,time,hashlib
import single_pickup_to_top_20261107 as base
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target
IMAGE=base.R/'tmp/integration11_home_diag_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
EXPECTED='82ace45279289e77f1ab859ce303b1b692e2100ab50ba9b3574c082d6118e687'
assert hashlib.sha256(IMAGE.read_bytes()).hexdigest()==EXPECTED
base.transport.live.IMAGE=IMAGE;n,segs=base.transport.live.load_image()
o={'target_override':'cortex_m','connect_mode':'attach','frequency':1000000,'auto_unlock':False,'resume_on_disconnect':False,'no_config':True,'cmsis_dap.prefer_v1':False,'cmsis_dap.limit_packets':True}
Session(None,options=o);a=ConnectHelper.get_all_connected_probes(blocking=False);assert len(a)==1 and a[0].unique_id=='6afaae24';base.transport.synchronous_interface(a[0]._link._interface);s=Session(a[0],options=o)
r={'writes':[],'reset':False,'flash':False,'motion_requests':0,'image_sha256':EXPECTED};t=None;powered=False
try:
 s.open();t=s.target;assert t.get_state()==Target.State.RUNNING
 for addr,b in segs:
  for k in range(0,len(b),4096):assert bytes(t.read_memory_block8(addr+k,len(b[k:k+4096])))==b[k:k+4096]
 r['Flash_matches']=True
 for key in ['g_it_request','g_it_state','g_lift_ruler_request','g_lift_ruler_state','g_startup_axes_state','g_dm_jog_request','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched']:assert t.read32(n[key])==0,key
 t.write32(n['g_lift_ruler_controller']+4,1);r['writes'].append(['actual lift VM request',1]);powered=True
 start=time.monotonic()
 while time.monotonic()-start<6:
  if t.read32(n['g_vm_test_request'])==1 and t.read32(0x40021014)&4 and t.read32(n['g_can_guard_ready']):break
  time.sleep(.1)
 assert t.read32(0x40021014)&4 and t.read32(n['g_can_guard_ready'])
 cycle=t.read32(n['g_lift_diag_cycle']);t.write32(n['g_lift_diag_request'],1);r['writes'].append(['read-only diagnostic request',1]);start=time.monotonic()
 while time.monotonic()-start<5:
  if t.read32(n['g_lift_diag_cycle'])!=cycle:break
  time.sleep(.05)
 assert t.read32(n['g_lift_diag_cycle'])!=cycle
 r['done']=t.read32(n['g_lift_diag_done']);r['results']=t.read_memory_block32(n['g_lift_diag_result'],6)
 raw=t.read_memory_block32(n['g_lift_diag_raw'],48);r['raw']=[raw[i*8:i*8+8] for i in range(6)]
 if r['results'][5]==1:
  value=(r['raw'][5][1]<<8)|r['raw'][5][2];r['O_SL_RP_raw']=value;r['O_SL_RP']=('BaseOnMa' if value==0 else str(value/10)+' degrees')
 r['controller_state']=t.read32(n['g_lift_ruler_state']);r['home_started']='no home request issued'
 assert all(t.read32(n[k])==0 for k in ['g_lift_ruler_state','g_it_state','g_startup_axes_state','g_dm_jog_request'])
except Exception as e:r['error']=repr(e)
finally:
 if powered and t is not None:
  t.write32(n['g_lift_ruler_controller']+4,0);t.write32(n['g_startup_axes_vm_request'],0);r['writes'].extend([['actual lift VM request',0],['actual startup VM request',0]])
  for i in range(30):
   time.sleep(.1)
   if t.read32(n['g_vm_test_request'])==0 and not t.read32(0x40021014)&4:break
  r['VM_after']=t.read32(n['g_vm_test_request']);r['PE2_after']=bool(t.read32(0x40021014)&4)
 s.close();(base.O/'integration11-home-readonly-result.json').write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r))
