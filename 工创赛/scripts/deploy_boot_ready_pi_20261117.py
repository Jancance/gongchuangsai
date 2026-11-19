"""Explicit two-code-file overlay into the current preview. Default is local only."""
from pathlib import Path
import sys,json,hashlib,base64
from stage23_pi_install_20261116 import remote,ROOT
PI=ROOT/'output/stage23_boot_ready_overlay_20261117'
E=ROOT/'evidence/integration/2026-11-17/boot-ready-beep'
NAMES=('source_fast_cycle_view_20261115.py','boot_ready_handshake.py')
SCRIPT=r'''
from pathlib import Path
import os,json,hashlib,base64,signal,subprocess,time,datetime,stat
root=Path('/home/orangepi/vision_tools/stage23_desktop_20261116')
files=PAYLOAD
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def procs():
 found=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   a=[v.decode()for v in(p/'cmdline').read_bytes().split(b'\0')if v]
   if 'source_fast_cycle_view_20261115.py'in a:found.append(dict(pid=int(p.name),argv=a,cwd=os.readlink(p/'cwd')))
  except OSError:pass
 return found
def owners():
 found=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   paths=[]
   for fd in(p/'fd').iterdir():
    try:
     value=os.readlink(fd)
     if value.startswith(('/dev/video','/dev/ttyUSB','/dev/ttyACM')):paths.append(value)
    except OSError:pass
   if paths:found.append(dict(pid=int(p.name),devices=sorted(set(paths))))
  except OSError:pass
 return found
current=procs();assert len(current)==1,current
before=current[0];assert before['pid']==3417 and before['cwd']==str(root) and '--preview-only'in before['argv'],before
owned=owners();assert len([v for v in owned if '/dev/ttyUSB0'in v['devices']])==1,owned
assert next(v for v in owned if '/dev/ttyUSB0'in v['devices'])['pid']==before['pid']
assert sha(root/'source_fast_cycle_view_20261115.py')=='20814ac5f8d38c16efefd7291ee1be910b8dc73ae100dc7543cd14fbc4f40fd4'
assert not(root/'boot_ready_handshake.py').exists(),'New module already exists; do not blindly redeploy'
manifest_path=root/'manifest.json';old_manifest=json.loads(manifest_path.read_text())
preserved={}
for argument in ('--body-profile','--material-pickup-config'):
 name=before['argv'][before['argv'].index(argument)+1];path=root/name;preserved[str(path)]=sha(path)
for index,argument in enumerate(before['argv'][:-1]):
 if argument.startswith('--')and(argument.endswith('config')or argument.endswith('profile')):
  path=root/before['argv'][index+1]
  if path.is_file():preserved[str(path)]=sha(path)
for path in(root/'source-alignment-settle.json',Path('/home/orangepi/p2-source-observer-20261106/vision_app/code/config/material_color_thresholds.json')):
 assert path.is_file(),str(path)
 preserved[str(path)]=sha(path)
stamp=datetime.datetime.now().strftime('%Y%m%d-%H%M%S');backup=Path('/home/orangepi/vision_tools')/('boot-ready-backup-'+stamp);backup.mkdir()
(backup/'source_fast_cycle_view_20261115.py').write_bytes((root/'source_fast_cycle_view_20261115.py').read_bytes())
(backup/'manifest.json').write_bytes(manifest_path.read_bytes())
(backup/'state.json').write_text(json.dumps(dict(before=before,owners=owned,preserved=preserved,new_module_previously_absent=True,autostart_modified=False),indent=2))
os.sync()
os.kill(before['pid'],signal.SIGTERM)
deadline=time.monotonic()+12
while time.monotonic()<deadline and(procs()or owners()):time.sleep(.2)
assert not procs()and not owners(),'Prior owner did not release; no file write or new launch'
new_manifest=dict(old_manifest);changed={}
for name,encoded in files.items():
 assert name in('source_fast_cycle_view_20261115.py','boot_ready_handshake.py')
 data=base64.b64decode(encoded);path=root/name;temp=root/(name+'.boot-ready-tmp')
 mode=stat.S_IMODE(path.stat().st_mode)if path.exists()else 0o644
 temp.write_bytes(data);temp.chmod(mode);os.replace(temp,path);changed[name]=sha(path);new_manifest[name]=changed[name]
temp=root/'manifest.boot-ready-tmp.json';temp.write_text(json.dumps(new_manifest,indent=2)+'\n');os.replace(temp,manifest_path)
os.sync()
assert all(sha(Path(p))==v for p,v in preserved.items())
assert all(new_manifest[k]==v for k,v in old_manifest.items()if k not in files)
oldlogs=set((root/'captures').glob('run-*.log'))
subprocess.Popen(['bash',str(root/'start.sh')],cwd=root,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
deadline=time.monotonic()+12;after=config=log=None
while time.monotonic()<deadline:
 p=procs();logs=set((root/'captures').glob('run-*.log'))-oldlogs
 if len(p)==1 and '--preview-only'in p[0]['argv']and logs:
  log=max(logs,key=lambda p:p.stat().st_mtime)
  for line in log.read_text(errors='replace').splitlines():
   if line.startswith('CONFIG '):config=json.loads(line[7:])
  if config and config.get('boot_handshake_tx_enabled')and config.get('mcu_status_rx_enabled')and config.get('observation_tx_enabled')is False:after=p[0];break
 time.sleep(.2)
assert after is not None,'New preview did not report RX + boot BE echo only'
owned=owners();assert len([v for v in owned if '/dev/ttyUSB0'in v['devices']])==1 and next(v for v in owned if '/dev/ttyUSB0'in v['devices'])['pid']==after['pid'],owned
assert all(sha(Path(p))==v for p,v in preserved.items())
print('BOOT_DEPLOY_JSON='+json.dumps(dict(before=before,after=after,backup=str(backup),config=config,owners=owned,changed=changed,manifest_sha256=sha(manifest_path),preserved_profile_sha256=preserved,autostart_modified=False,observation_commands_sent=False,serial_start_stop_sent=False,preview=True,completed_local=datetime.datetime.now().isoformat(),log=str(log)),ensure_ascii=False))
'''

def main():
 if '--deploy'not in sys.argv:
  print(json.dumps(dict(local_only=True,hardware_access=False,explicit_deploy_required=True,expected_owner=3417,changes=list(NAMES),preserve_runtime_profiles=True)));return
 c=json.loads((E/'candidate.json').read_text());assert c['review_passed']and c['build_passed']
 files={name:base64.b64encode((PI/name).read_bytes()).decode()for name in NAMES}
 for name in NAMES:assert hashlib.sha256((PI/name).read_bytes()).hexdigest()==c['pi_changes'][name]
 script=SCRIPT.replace('files=PAYLOAD','files='+repr(files),1)
 result=remote(ROOT/'tmp/integration_mirror_ssh_20261117.py',script)
 lines=[v.split('BOOT_DEPLOY_JSON=',1)[1]for v in result.splitlines()if v.startswith('BOOT_DEPLOY_JSON=')]
 if not lines:
  (E/'pi-deploy-error.txt').write_text(result,encoding='utf-8');raise RuntimeError('No verified deployment result; inspect backup/process state; do not blindly retry')
 report=json.loads(lines[-1]);(E/'pi-deploy.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps(report,ensure_ascii=False))
if __name__=='__main__':main()
