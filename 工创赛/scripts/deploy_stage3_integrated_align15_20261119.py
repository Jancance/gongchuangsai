"""Install only a matched Stage3 target config while MCU is idle; no MCU writes."""
import argparse
import base64
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time
import zlib
from stage23_pi_install_20261116 import ROOT, remote

CONFIG = 'stage3_candidate/config/stage3.json'
SCRIPT = r'''
from pathlib import Path
import os,json,time,hashlib,signal,subprocess
root=Path('/home/orangepi/vision_tools/stage23_desktop_20261116')
binding=BINDING
result={'status':'PRECHECK','mcu_writes':False,'motion_requests':False,'term_sent':False}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def apps():
 out=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   a=[v.decode() for v in (p/'cmdline').read_bytes().split(b'\0') if v]
   if 'source_fast_cycle_view_20261115.py' in a:
    out.append({'pid':int(p.name),'argv':a,'cwd':os.readlink(p/'cwd')})
  except OSError:pass
 return out
def owners():
 out=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   for f in (p/'fd').iterdir():
    try:
     dev=os.readlink(f)
     if dev in ('/dev/video0','/dev/ttyUSB0'):out.append((int(p.name),dev))
    except OSError:pass
  except OSError:pass
 return out
def latest(a):
 relative=a[a.index('--session-dir')+1]
 assert relative.startswith('captures/run-') and '..' not in Path(relative).parts
 p=root/relative/'observations.jsonl'
 assert time.time()-p.stat().st_mtime<3
 with p.open('rb') as f:
  f.seek(max(0,p.stat().st_size-65536));lines=f.read().splitlines()
 o=json.loads(lines[-1]);s=o['mcu_status']
 assert o['region']=='stage3' and s['state']==0 and s['run']==binding['run'] and s['pose']==binding['pose']
 assert not o.get('replay') and not o.get('motor_commands_sent')
 return o
def atomic(p,data):
 q=p.with_name(p.name+'.integrated-'+str(time.time_ns()))
 with q.open('xb') as f:f.write(data);f.flush();os.fsync(f.fileno())
 os.replace(q,p)
 fd=os.open(str(p.parent),os.O_RDONLY|os.O_DIRECTORY)
 try:os.fsync(fd)
 finally:os.close(fd)
try:
 rows=apps();assert len(rows)==1 and rows[0]['pid']==binding['pid']
 old=rows[0];assert old['cwd']==str(root) and old['argv']==binding['argv'] and '--stage3-only' in old['argv']
 before=latest(old['argv']);manifest_path=root/'manifest.json';manifest=json.loads(manifest_path.read_text())
 assert all(not Path(n).is_absolute() and '..' not in Path(n).parts and sha(root/n)==h for n,h in manifest.items())
 config_path=root/binding['config'];assert sha(config_path)==binding['old_sha']
 document=json.loads(config_path.read_text());assert document==binding['old_config']
 preserved={n:h for n,h in manifest.items() if n!=binding['config']}
 backup=root.parent/('stage3-integrated-align15-backup-'+str(time.time_ns()));backup.mkdir()
 (backup/'stage3.json').write_bytes(config_path.read_bytes());(backup/'manifest.json').write_bytes(manifest_path.read_bytes())
 result.update(backup=str(backup),before=before,old_pid=old['pid'])
 os.kill(old['pid'],signal.SIGTERM);result['term_sent']=True
 end=time.monotonic()+12
 while time.monotonic()<end and (apps() or owners()):time.sleep(.2)
 assert not apps() and not owners(),'Existing owners did not exit'
 document['ring_row_alignment']['target_center_px']=binding['target']
 document['ring_row_alignment']['integrated_compensation']={'forward_mm':binding['forward_mm'],'right_mm':binding['right_mm'],'reference_center_px':[960,540],'post_alignment_replay':False}
 data=(json.dumps(document,indent=2)+'\n').encode()
 atomic(config_path,data);manifest[binding['config']]=hashlib.sha256(data).hexdigest()
 atomic(manifest_path,(json.dumps(manifest,indent=2)+'\n').encode());os.sync()
 args=list(old['argv']);args[args.index('--session-dir')+1]='captures/run-'+time.strftime('%Y%m%d-%H%M%S')+'-'+str(time.time_ns()%1000000000)
 logfile=root/(args[args.index('--session-dir')+1]+'.log')
 logfile.parent.mkdir(exist_ok=True)
 environment=dict(os.environ)
 environment.setdefault('DISPLAY',':0')
 environment.setdefault('XAUTHORITY','/home/orangepi/.Xauthority')
 environment.setdefault('QT_QPA_PLATFORM','xcb')
 with logfile.open('xb') as stream:
  process=subprocess.Popen(args,cwd=root,env=environment,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
 result['new_pid']=process.pid
 end=time.monotonic()+15;observed=None
 while time.monotonic()<end:
  try:
   current=apps();assert len(current)==1 and current[0]['pid']==process.pid
   observed=latest(current[0]['argv']);break
  except (AssertionError,OSError,ValueError,KeyError):time.sleep(.2)
 assert observed is not None,'New viewer state not verified'
 assert all(sha(root/n)==h for n,h in manifest.items()) and all(manifest[n]==h for n,h in preserved.items())
 assert json.loads(config_path.read_text())==document
 own=owners();assert len(own)==2 and {x[1] for x in own}=={'/dev/video0','/dev/ttyUSB0'}
 assert next(x[0] for x in own if x[1]=='/dev/ttyUSB0')==process.pid
 result.update(status='DEPLOYED',config_sha256=sha(config_path),config=document,latest=observed,owners=own,code_files_preserved=len(preserved),session=args[args.index('--session-dir')+1])
except Exception as error:result.update(status='UNCERTAIN_INSPECT_NO_RETRY',error=repr(error))
print('INTEGRATED_DEPLOY_JSON='+json.dumps(result))
'''

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--deploy',action='store_true');p.add_argument('--mcu-proof',type=Path);p.add_argument('--pi-proof',type=Path)
 p.add_argument('--manifest',type=Path,required=True);a=p.parse_args()
 m=json.loads(a.manifest.read_text(encoding='utf-8'))
 assert m['build_passed'] and m['capabilities']['stage3_integrated_visual_compensation']
 compensation=m['integrated_visual_compensation']
 target=compensation['target_pixel']
 forward,right=compensation['forward_mm'],compensation['right_mm']
 model=json.loads((ROOT/'output/stage3_row_bridge_20261118/stage3-row-config.json').read_text())['ring_row_alignment']
 response=model['response_pixel_per_right_forward_ccw']
 expected=[base+response[i][0]*right+response[i][1]*forward for i,base in enumerate((960,540))]
 assert len(target)==2 and all(abs(x-y)<1e-8 for x,y in zip(target,expected))
 if not a.deploy:print(json.dumps({'local_only':True,'target_pixel':target,'forward_mm':forward,'right_mm':right,'changed_file':CONFIG}));return
 assert a.mcu_proof and a.pi_proof
 assert all(0<=time.time()-q.stat().st_mtime<180 for q in (a.mcu_proof,a.pi_proof))
 proof=json.loads(a.mcu_proof.read_text(encoding='utf-8'));s=proof['after'];pi=json.loads(a.pi_proof.read_text(encoding='utf-8'))
 assert proof['image']['sha256']==m['images']['Vehicle_LiftRuler']['sha256'] and proof['verified_flash_bytes']>0
 assert s['g_stage3_state']==0 and s['g_region_dispatch_area']==3 and s['g_stage3_integrated_compensation']==1
 assert all(not s.get(k) for k in ('g_stage3_fault','g_safety_fault','g_it_request','g_it_cause','g_rtos_fault','g_can_ctrl_fault_latched'))
 assert not pi['manifest_mismatches'] and len(pi['apps'])==1
 app=pi['apps'][0];c=pi['stage3_config'];row=c['ring_row_alignment']
 assert c['observation_mode']=='cyqm_ring_station' and row['response_pixel_per_right_forward_ccw']==model['response_pixel_per_right_forward_ccw']
 assert row['nominal_image_to_station_affine']==model['nominal_image_to_station_affine'] and row['yaw_reference']=='vehicle_edge'
 bind=dict(pid=app['pid'],argv=app['argv'],run=s['g_stage3_run'],pose=s['g_stage3_pose'],config=CONFIG,old_sha=pi['stage3_config_sha256'],old_config=c,target=target,forward_mm=forward,right_mm=right)
 code=SCRIPT.replace('binding=BINDING','binding='+repr(bind),1);compile(code,'remote-deploy','exec')
 encoded=base64.b64encode(zlib.compress(code.encode())).decode()
 output=remote(ROOT/'tmp/integration_mirror_ssh_20261117.py',"import zlib,base64;exec(zlib.decompress(base64.b64decode("+repr(encoded)+")))")
 rows=[v.split('INTEGRATED_DEPLOY_JSON=',1)[1] for v in output.splitlines() if v.startswith('INTEGRATED_DEPLOY_JSON=')]
 assert len(rows)==1,'Uncertain remote result; inspect before retry'
 result=json.loads(rows[0]);out=ROOT/'evidence/integration/2026-11-18/stage3-resume'/('integrated-align15-deploy-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
 out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
 print(json.dumps({'evidence':str(out),'status':result['status'],'error':result.get('error'),'target':result.get('config',{}).get('ring_row_alignment',{}).get('target_center_px')}))
 assert result['status']=='DEPLOYED',result

if __name__=='__main__':main()
