"""Explicit three-code overlay + preview/trial restart. Never sends MCU commands."""
from pathlib import Path
import argparse,base64,hashlib,json,zlib
from prepare_three_frame_warm_20261118 import R,PI,E
from stage23_pi_install_20261116 import remote

SCRIPT=r'''
from pathlib import Path
import base64,datetime,hashlib,json,os,signal,stat,subprocess,time,py_compile
root=Path('/home/orangepi/vision_tools/stage23_desktop_20261116')
mode=MODE
c=CANDIDATE
payload=PAYLOAD
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def procs():
 out=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   a=[x.decode() for x in (p/'cmdline').read_bytes().split(b'\0') if x]
   if any(Path(x).name=='source_fast_cycle_view_20261115.py' for x in a):out.append(dict(pid=int(p.name),argv=a,cwd=os.readlink(p/'cwd')))
  except OSError:pass
 return out
def owners():
 out=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   dev=[]
   for fd in (p/'fd').iterdir():
    try:
     name=os.readlink(fd)
     if name.startswith(('/dev/video','/dev/ttyUSB','/dev/ttyACM')):dev.append(name)
    except OSError:pass
   if dev:
    parent=int(next(x.split()[1] for x in (p/'status').read_text().splitlines() if x.startswith('PPid:')))
    out.append(dict(pid=int(p.name),ppid=parent,devices=sorted(set(dev))))
  except OSError:pass
 return out
current=procs();owned=owners()
names=set(c['pi_base_hashes'])|set(c['pi_changes'])|set(c['pi_unchanged_dependencies'])
if mode=='inspect':
 print('THREE_FRAME_JSON='+json.dumps(dict(processes=current,owners=owned,hashes={n:sha(root/n) if (root/n).exists() else None for n in names},body_profile_sha256=sha(root/'body-profile.json'))))
else:
 assert len(current)<=1,current
 before=current[0] if current else None
 assert not before or before['cwd']==str(root),before
 if not before:assert not owned,'An unrelated process owns a camera/UART'
 else:assert all(v['pid']==before['pid'] or v['ppid']==before['pid'] for v in owned),owned
 for n,h in c['pi_unchanged_dependencies'].items():assert sha(root/n)==h,('Dependency drift',n)
 preserved={str(p):sha(p) for p in root.glob('*.json') if p.name!='manifest.json'}
 for p in (Path('/home/orangepi/vision_tools/material_pickup_top65_20261114.json'),Path('/home/orangepi/p2-source-observer-20261106/vision_app/code/config/material_color_thresholds.json'),root/'start.sh',root/'start-trial.sh',root/'start-preview.sh'):
  assert p.is_file(),str(p)
  preserved[str(p)]=sha(p)
 manifest_path=root/'manifest.json';manifest=json.loads(manifest_path.read_text())
 if mode=='deploy':
  for n,h in c['pi_base_hashes'].items():assert sha(root/n)==h,('Base code drift',n)
  assert not (root/'source_pickup_window.py').exists(),'Already/partially deployed; inspect before recovery'
 else:
  assert mode=='trial' and before and '--preview-only' in before['argv'],'Expected current new preview'
  for n,h in c['pi_changes'].items():assert sha(root/n)==h,('New code drift',n)
 backup=Path('/home/orangepi/vision_tools')/('three-frame-'+mode+'-backup-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));backup.mkdir()
 (backup/'state.json').write_text(json.dumps(dict(before=before,owners=owned,preserved=preserved,mode=mode,autostart_modified=False),indent=2))
 if mode=='deploy':
  (backup/'manifest.json').write_bytes(manifest_path.read_bytes())
  for n in c['pi_changes']:
   if (root/n).exists():(backup/n).write_bytes((root/n).read_bytes())
 os.sync()
 if before:
  os.kill(before['pid'],signal.SIGTERM)
  end=time.monotonic()+12
  while time.monotonic()<end and (procs() or owners()):time.sleep(.2)
  assert not procs() and not owners(),'Old owner did not release; no new process launched'
 if mode=='deploy':
  new_manifest=dict(manifest)
  for n,encoded in payload.items():
   assert n in c['pi_changes'];data=base64.b64decode(encoded);assert hashlib.sha256(data).hexdigest()==c['pi_changes'][n]
   p=root/n;tmp=p.with_name(p.name+'.three-frame-new');assert not tmp.exists()
   m=stat.S_IMODE(p.stat().st_mode) if p.exists() else 0o644
   tmp.write_bytes(data);tmp.chmod(m);os.replace(tmp,p);py_compile.compile(str(p),doraise=True);new_manifest[n]=sha(p)
  tmp=root/'manifest.three-frame-new.json';tmp.write_text(json.dumps(new_manifest,indent=2)+'\n');os.replace(tmp,manifest_path)
  assert all(new_manifest[n]==h for n,h in manifest.items() if n not in payload)
  os.sync()
 assert all(sha(Path(p))==h for p,h in preserved.items()),'Runtime configuration changed'
 oldlogs=set((root/'captures').glob('run-*.log'));launcher=root/('start.sh' if mode=='deploy' else 'start-trial.sh')
 subprocess.Popen(['bash',str(launcher)],cwd=root,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
 end=time.monotonic()+14;after=config=log=None
 while time.monotonic()<end:
  rows=procs();logs=set((root/'captures').glob('run-*.log'))-oldlogs
  if len(rows)==1 and logs:
   log=max(logs,key=lambda p:p.stat().st_mtime)
   for line in log.read_text(errors='replace').splitlines():
    if line.startswith('CONFIG '):config=json.loads(line[7:])
   expected=(mode=='trial')
   if config and config.get('observation_tx_enabled')==expected and config.get('boot_handshake_tx_enabled') and config.get('mcu_status_rx_enabled'):
    own=owners()
    if sum('/dev/ttyUSB0' in v['devices'] for v in own)==1 and sum('/dev/video0' in v['devices'] for v in own)==1:after=rows[0];break
  time.sleep(.2)
 assert after is not None,('Expected one camera/serial owner and new CONFIG; inspect current state before retry',str(log))
 assert ('--preview-only' in after['argv'])==(mode=='deploy')
 assert all(sha(root/n)==h for n,h in c['pi_changes'].items())
 assert all(sha(Path(p))==h for p,h in preserved.items())
 print('THREE_FRAME_JSON='+json.dumps(dict(mode=mode,before=before,after=after,backup=str(backup),owners=owners(),config=config,hashes={n:sha(root/n) for n in c['pi_changes']},preserved=preserved,log=str(log),autostart_modified=False,serial_control_commands_sent=False,completed_local=datetime.datetime.now().isoformat())))
'''


def main():
    ap=argparse.ArgumentParser();g=ap.add_mutually_exclusive_group();g.add_argument('--inspect',action='store_true');g.add_argument('--deploy',action='store_true');g.add_argument('--trial',action='store_true')
    ap.add_argument('--ssh-helper',type=Path,default=R/'tmp/integration_mirror_ssh_20261117.py');a=ap.parse_args()
    c=json.loads((E/'candidate.json').read_text());assert c['build_passed'] and c['self_review_passed']
    if not any((a.inspect,a.deploy,a.trial)):
        print(json.dumps(dict(local_only=True,files=list(c['pi_changes']),hardware_access=False)));return
    mode='inspect' if a.inspect else 'deploy' if a.deploy else 'trial'
    files={n:base64.b64encode((PI/n).read_bytes()).decode() for n in c['pi_changes']} if a.deploy else {}
    assert all(hashlib.sha256((PI/n).read_bytes()).hexdigest()==h for n,h in c['pi_changes'].items())
    script=SCRIPT.replace('mode=MODE','mode='+repr(mode),1).replace('c=CANDIDATE','c='+repr(c),1).replace('payload=PAYLOAD','payload='+repr(files),1)
    # Windows process command lines are bounded; compress code, never credentials.
    encoded=base64.b64encode(zlib.compress(script.encode('utf-8'),9)).decode('ascii')
    command="import zlib,base64;exec(zlib.decompress(base64.b64decode("+repr(encoded)+")))"
    result=remote(a.ssh_helper,command);(E/('pi-'+mode+'-output.txt')).write_text(result,encoding='utf-8')
    lines=[x.split('THREE_FRAME_JSON=',1)[1] for x in result.splitlines() if x.startswith('THREE_FRAME_JSON=')]
    assert len(lines)==1,'No verified Pi result; inspect saved output/current processes before retry'
    report=json.loads(lines[0]);(E/('pi-'+mode+'.json')).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False))


if __name__=='__main__':main()
