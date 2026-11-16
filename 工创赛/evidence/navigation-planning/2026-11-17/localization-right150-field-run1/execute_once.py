from pathlib import Path
import json,hashlib,datetime,sys,subprocess,os,importlib.util
E=Path(__file__).resolve().parent
R=E.parents[4]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(name,value):
 with (E/name).open('x',encoding='utf8') as f:json.dump(value,f,indent=2)
b=json.loads((E/'execution_binding.json').read_text('utf8'));assert b['execution_permitted'] and b['site_confirmation'] and b['root_release'] and b['plan_id']==12;IMAGE=Path(b['image']);EXPECTED=b['candidate_sha256']
assert sha(IMAGE)==EXPECTED
installed=json.loads(Path(b['download_receipt']).read_text('utf8'));assert (installed.get('success') is True or installed.get('user_reported_completed') is True) and installed['candidate_sha256']==EXPECTED
save('execution_once_started.json',dict(issued_by='root',authorization=b['authorization'],automatic_retry=False,started_at=datetime.datetime.now().astimezone().isoformat()))
sys.path.insert(0,str(R/'工创赛/tmp/turn_probe_deps'));sys.path.insert(0,str(R/'工创赛/scripts'));sys.path.insert(0,str(E))
from elftools.elf.elffile import ELFFile
from source_stage2_readonly_sync_probe_20261105 import synchronous_interface
from pyocd.target.pack.pack_target import PackTargets
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from decode_capture import inspect_header
with IMAGE.open('rb') as f:
 elf=ELFFile(f);names=['g_chassis_auto_active','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_fusion_capture_a'];symbols={s.name:(int(s['st_value']),int(s['st_size'])) for s in elf.get_section_by_name('.symtab').iter_symbols() if s.name in names}
assert len(symbols)==len(names)
PackTargets.populate_targets_from_pack('D:/keil/Arm/Packs/Keil/STM32F4xx_DFP/1.0.8')
def connect():
 probes=ConnectHelper.get_all_connected_probes(blocking=False);assert len(probes)==1 and probes[0].unique_id.lower()==b['probe_uid'].lower()
 probe=probes[0];synchronous_interface(probe._link._interface)
 s=Session(probe,options={'target_override':'stm32f407ve','connect_mode':'attach','frequency':1000000,'auto_unlock':False,'resume_on_disconnect':False,'no_config':True,'dap_protocol':'swd','cmsis_dap.prefer_v1':False,'cmsis_dap.limit_packets':True});s.open();assert s.target.get_state().name=='RUNNING';return s
reset={'candidate_sha256':EXPECTED,'reset_attempted':False,'reset':False,'flash_read':False,'serial_TX_bytes':0}
session=None
try:
 session=connect();t=session.target
 reset['states_before']={n:t.read32(a) for n,(a,z) in symbols.items() if n!='g_fusion_capture_a'}
 assert all(v==0 for v in reset['states_before'].values()),'Active motion or fault; no reset'
 reset['reset_attempted']=True
 (E/'reset_intent.json').write_text(json.dumps(reset,indent=2),'utf8');t.reset();reset['reset']=True
finally:
 if session:session.close()
 save('reset_once_receipt.json',reset)
CLIENT=E/'p1_serial_motion_turn90_keepalive3s.py'
cmd=[sys.executable,str(CLIENT),'--port',b['wireless_port'],'--mode','3','--action','monitor','--seconds','3','--output',str(E/'fresh_monitor')]
with (E/'fresh_monitor.log').open('xb') as f:cp=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=12)
assert cp.returncode==0
files=list((E/'fresh_monitor').glob('*.summary.json'));assert len(files)==1;mon=json.loads(files[0].read_text('utf8'));assert mon['sent_commands']==0 and mon['valid_frames']>0
spec=importlib.util.spec_from_file_location('client',CLIENT);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);ctl=m.Controller('run-once',3,12,expected_complete_powered=True);bf=mon['last_baseline40'];ctl.baseline_config=bf[2:];ctl.baseline_tick=bf[1];assert ctl._healthy(mon['last']) and all(v==0 for v in mon['max_abs_rpm'])
session=None
try:
 session=connect();t=session.target;header=inspect_header(bytes(t.read_memory_block8(symbols['g_fusion_capture_a'][0],256)))
 assert header['state']==0 and header['session']==0 and header['pre_count']==100 and header['post_count']==0 and header['quality']==0 and header['last_raw_state']==2 and header['last_raw_error']==0
finally:
 if session:session.close()
save('fresh_ready_receipt.json',{'accepted':True,'initialization_policy':'one_explicit_reset_after_download','reset_count':1,'capture_header':header,'monitor_valid_frames':mon['valid_frames'],'monitor_TX_commands':mon['sent_commands']})
cmd=[sys.executable,str(CLIENT),'--port',b['wireless_port'],'--mode','3','--action','run-once','--plan','right150_40','--expect-powered-done','--seconds','40','--output',str(E/'serial')]
save('run_once_started.json',{'command':cmd,'maximum_START':1,'automatic_retry':False,'started_at':datetime.datetime.now().astimezone().isoformat()})
print('RESET1_FRESH_READY_STARTING_ONE_RUN',flush=True)
with (E/'run_client.log').open('xb') as f:cp=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=65)
files=list((E/'serial').glob('*.summary.json'));assert len(files)==1;summary=json.loads(files[0].read_text('utf8'));summary['returncode']=cp.returncode;save('result_summary.json',summary)
print(json.dumps({'returncode':cp.returncode,'last':summary.get('last'),'last_position':summary.get('last_position')},indent=2),flush=True)
