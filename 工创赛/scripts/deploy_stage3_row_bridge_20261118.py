"""Default local; explicit Pi overlay deployment, existing O31 observations only."""
from pathlib import Path
import argparse,base64,datetime,hashlib,json,sys,time,zlib
from stage23_pi_install_20261116 import ROOT,remote

P=ROOT/'output/stage3_row_bridge_20261118'
E=ROOT/'evidence/integration/2026-11-18/stage3-resume'
SCRIPT=r'''
from pathlib import Path
import os,signal,subprocess,time,json,hashlib,datetime,base64
root=Path('/home/orangepi/vision_tools/stage23_desktop_20261116')
payload=PAYLOAD
expected=EXPECTED
binding=BINDING
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def syncdir(p):
 fd=os.open(str(p),os.O_RDONLY|os.O_DIRECTORY)
 try:os.fsync(fd)
 finally:os.close(fd)
def write_new(p,data):
 with p.open('xb') as f:f.write(data);f.flush();os.fsync(f.fileno())
 syncdir(p.parent)
def atomic(p,data):
 tmp=p.with_name('.'+p.name+'.row-bridge-new');write_new(tmp,data);os.replace(tmp,p);syncdir(p.parent)
def processes():
 rows=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   a=[x.decode() for x in (p/'cmdline').read_bytes().split(b'\0') if x]
   if 'source_fast_cycle_view_20261115.py' in a:rows.append(dict(pid=int(p.name),argv=a,cwd=os.readlink(p/'cwd')))
  except OSError:pass
 return rows
def owners():
 rows=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   parent=int(next(x.split()[1] for x in (p/'status').read_text().splitlines() if x.startswith('PPid:')))
   for fd in (p/'fd').iterdir():
    try:
     dev=os.readlink(fd)
     if dev.startswith(('/dev/video','/dev/ttyUSB','/dev/ttyACM')):rows.append(dict(pid=int(p.name),parent=parent,device=dev))
    except OSError:pass
  except OSError:pass
 return rows
def last(proc):
 a=proc['argv'];s=Path(a[a.index('--session-dir')+1]);assert not s.is_absolute() and '..' not in s.parts
 path=root/s/'observations.jsonl'
 with path.open('rb') as f:f.seek(max(0,path.stat().st_size-262144));rows=f.read().splitlines()
 return json.loads(rows[-1]),str(s)
report={'MCU_reset':False,'MCU_requests_sent':False,'Pi_reboot':False,'term_sent':False,'files_written':[],
        'physical_calibration_verified':False,'status':'PRECHECK'}
try:
 rows=processes();assert len(rows)==1 and rows[0]['pid']==binding['pid'],rows
 old=rows[0];assert old['cwd']==str(root) and '--preview-only' not in old['argv']
 obs,session=last(old);assert session==binding['session']
 s=obs['mcu_status'];assert obs['region']=='stage3' and s['run']==binding['run'] and s['pose']==binding['pose'] and s['state']==2 and s['config_ready']==0 and s['align_done']==0,s
 assert time.monotonic()-obs['host_read_completed_monotonic_s']<1
 owned=owners();assert len(owned)==2 and {x['device'] for x in owned}=={'/dev/video0','/dev/ttyUSB0'}
 assert all(x['pid']==old['pid'] or x['parent']==old['pid'] for x in owned)
 manifest=json.loads((root/'manifest.json').read_text());assert all(sha(root/n)==h for n,h in manifest.items())
 assert all(sha(root/n)==h for n,h in expected.items())
 if 'ring_row_alignment.py' in payload:assert not (root/'ring_row_alignment.py').exists()
 for n,encoded in payload.items():
  data=base64.b64decode(encoded)
  if n.endswith('.py'):compile(data,str(root/n),'exec')
  else:json.loads(data)
 preserved={n:sha(root/n) for n in manifest if n not in payload}
 ext=Path('/home/orangepi/vision_tools/material_pickup_top65_20261114.json');extsha=sha(ext)
 backup=root.parent/('stage3-row-backup-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));backup.mkdir()
 for n in [*expected,'manifest.json']:
  dest=backup/n;dest.parent.mkdir(parents=True,exist_ok=True);write_new(dest,(root/n).read_bytes())
 write_new(backup/'preserved.json',json.dumps(preserved).encode());syncdir(backup);syncdir(backup.parent)
 # Bind the last observed state once more before stopping the original owner.
 assert processes()==[old];o,_=last(old);assert o['mcu_status']==s
 os.kill(old['pid'],signal.SIGTERM);report['term_sent']=True
 deadline=time.monotonic()+12
 while time.monotonic()<deadline and (processes() or owners()):time.sleep(.2)
 assert not processes() and not owners(),'Owner not released; no replacement'
 changed={}
 for n,encoded in payload.items():
  data=base64.b64decode(encoded);atomic(root/n,data);changed[n]=hashlib.sha256(data).hexdigest();report['files_written'].append(n)
 updated=dict(manifest);updated.update(changed)
 atomic(root/'manifest.json',(json.dumps(updated,indent=2)+'\n').encode())
 assert all(sha(root/n)==h for n,h in updated.items());assert all(sha(root/n)==h for n,h in preserved.items());assert sha(ext)==extsha
 os.sync()
 subprocess.Popen(['bash',str(root/'start-stage2-debug215-trial.sh')],cwd=root,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
 deadline=time.monotonic()+25;latest=None;new=None
 while time.monotonic()<deadline:
  rows=processes()
  if len(rows)==1 and rows[0]['pid']!=old['pid']:
   try:
    o,ns=last(rows[0]);latest=o;new=rows[0]
    if o.get('alignment_response_ready') and o.get('region')=='stage3' and o.get('mcu_status',{}).get('state') in (2,3,4,7,9):break
   except (OSError,ValueError,IndexError):pass
  time.sleep(.2)
 assert latest is not None and latest.get('alignment_response_ready'),'New row response startup unconfirmed; do not retry'
 report.update(status='DEPLOYED',backup=str(backup),before=old,after=new,new_session=ns,latest=latest,owners=owners(),
  changed_sha256=changed,preserved_manifest_files=len(preserved),external_config_preserved=sha(ext)==extsha,
  old_status=s,file_and_dir_fsync=True,os_sync=True,code_and_config_manifest_verified=True)
except Exception as error:report.update(status='UNCERTAIN_INSPECT_NO_AUTOMATIC_RETRY',error=repr(error))
print('ROW_DEPLOY_JSON='+json.dumps(report))
'''

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--deploy',action='store_true');ap.add_argument('--mcu-proof',type=Path)
    ap.add_argument('--pi-proof',type=Path);ap.add_argument('--sequence-only',action='store_true');args=ap.parse_args()
    payload={'region_desktop.py':base64.b64encode((P/'region_desktop.py').read_bytes()).decode(),
             'ring_row_alignment.py':base64.b64encode((P/'ring_row_alignment.py').read_bytes()).decode(),
             'stage3_candidate/config/stage3.json':base64.b64encode((P/'stage3-row-config.json').read_bytes()).decode()}
    old=ROOT/'output/stage23_desktop_20261116'
    expected={n:hashlib.sha256((old/n).read_bytes()).hexdigest() for n in ('region_desktop.py','stage3_candidate/config/stage3.json')}
    if args.sequence_only:
        releases=[json.loads(p.read_text(encoding='utf-8')) for p in sorted(E.glob('row-bridge-deploy-*.json'))]
        deployed=[r for r in releases if r.get('status')=='DEPLOYED']
        assert deployed,'No previous confirmed row deployment'
        prior=deployed[-1]['changed_sha256']
        payload={'region_desktop.py':payload['region_desktop.py']}
        expected={'region_desktop.py':prior['region_desktop.py']}
    if not args.deploy:
        print(json.dumps(dict(local_only=True,changes=list(payload),base_sha256=expected)));return
    assert args.mcu_proof and args.pi_proof
    assert time.time()-args.mcu_proof.stat().st_mtime<180 and time.time()-args.pi_proof.stat().st_mtime<180
    m=json.loads(args.mcu_proof.read_text(encoding='utf-8'));pi=json.loads(args.pi_proof.read_text(encoding='utf-8'))
    assert m['flash_matches'] and m['verified_flash_bytes']==197368 and not m.get('error')
    g={n:v.get('value') for n,v in m['globals'].items()}
    assert g['g_stage3_state']==2 and g['g_stage3_config_ready']==0 and g['g_region_dispatch_area']==3
    for n in ('g_stage3_fault','g_stage3_request','g_it_request','g_it_cause','g_dm_jog_first_fault','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched'):assert g[n]==0,(n,g[n])
    assert not pi['manifest_mismatches'] and len(pi['apps'])==1
    app=pi['apps'][0];s=app['latest']['mcu_status'];assert s['pose']==g['g_stage3_pose'] and s['state']==2 and s['config_ready']==0
    binding=dict(pid=app['pid'],session=str(Path(app['session']).as_posix()).split('stage23_desktop_20261116/')[-1],run=s['run'],pose=s['pose'])
    script=SCRIPT.replace('payload=PAYLOAD','payload='+repr(payload),1).replace('expected=EXPECTED','expected='+repr(expected),1).replace('binding=BINDING','binding='+repr(binding),1)
    encoded=base64.b64encode(zlib.compress(script.encode(),9)).decode()
    assert len(encoded)<20000
    output=remote(ROOT/'tmp/integration_mirror_ssh_20261117.py',"import base64,zlib;exec(zlib.decompress(base64.b64decode("+repr(encoded)+")))")
    stamp=datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    (E/('row-bridge-deploy-output-'+stamp+'.txt')).write_text(output,encoding='utf-8')
    rows=[x.split('ROW_DEPLOY_JSON=',1)[1] for x in output.splitlines() if x.startswith('ROW_DEPLOY_JSON=')];assert len(rows)==1
    result=json.loads(rows[0]);(E/('row-bridge-deploy-'+stamp+'.json')).write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result));assert result['status']=='DEPLOYED'

if __name__=='__main__':main()
