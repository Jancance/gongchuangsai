"""Install the Stage3-only Orange Pi desktop startup and restart its viewer."""
from datetime import datetime
import json
from stage23_pi_install_20261116 import ROOT, remote

SCRIPT = r'''
from pathlib import Path
import hashlib, json, os, signal, subprocess, time
root=Path.home()/'vision_tools/stage23_desktop_20261116'
viewer='source_fast_cycle_view_20261115.py'
manifest_path=root/'manifest.json'
desktop=Path.home()/'.config/autostart/source-rimpark.desktop'
launcher='start-stage3-round-trip.sh'
manifest=json.loads(manifest_path.read_text())
def digest(data): return hashlib.sha256(data).hexdigest()
assert all(digest((root/name).read_bytes())==value for name,value in manifest.items())
assert 'start-stage2-debug215-trial.sh' in desktop.read_text()
source=(root/viewer).read_text()
for old,new in [
 ('    parser.add_argument(\'--stage3-config\'', '    parser.add_argument(\'--stage3-only\',action=\'store_true\')\n    parser.add_argument(\'--stage3-config\''),
 ('    region_ui=RegionDesktop(args.stage3_config)', '    region_ui=RegionDesktop(args.stage3_config)\n    if args.stage3_only:region_ui.region=\'stage3\''),
 ('                if boot_handshake.receive(line,link.write):continue', '                if not args.stage3_only and boot_handshake.receive(line,link.write):continue'),
 ('                try:latest=decode_status(line)', '                if args.stage3_only:continue\n                try:latest=decode_status(line)')]:
 assert source.count(old)==1,old
 source=source.replace(old,new)
compile(source,viewer,'exec')
start=f'#!/bin/bash\nset -eu\nexec /bin/bash {root}/start-trial.sh --stage3-only "$@"\n'
subprocess.run(['/bin/bash','-n'],input=start,text=True,check=True)
entry=f'[Desktop Entry]\nType=Application\nName=Stage3 round trip vision\nExec=/bin/bash {root}/{launcher}\nPath={root}\nTerminal=false\nX-GNOME-Autostart-enabled=true\n'
def viewers():
 result=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   args=(p/'cmdline').read_bytes().split(b'\x00')
   if viewer.encode() in args:
    assert Path(os.readlink(p/'cwd'))==root
    result.append((int(p.name),args))
  except OSError:pass
 return result
old=viewers();assert len(old)<=1
backup=root.parent/('stage3-startup-backup-'+str(time.time_ns()));backup.mkdir()
for p in (root/viewer,manifest_path,desktop):(backup/p.name).write_bytes(p.read_bytes())
def atomic(path,data):
 tmp=path.with_name(path.name+'.stage3-new')
 with tmp.open('xb') as f:
  f.write(data);f.flush();os.fsync(f.fileno())
 tmp.replace(path)
if old:os.kill(old[0][0],signal.SIGTERM)
deadline=time.monotonic()+12
while viewers() and time.monotonic()<deadline:time.sleep(.2)
assert not viewers(),'Viewer failed to release devices'
for name,data in ((viewer,source.encode()),(launcher,start.encode())):
 atomic(root/name,data);manifest[name]=digest(data)
atomic(manifest_path,(json.dumps(manifest,indent=2)+'\n').encode())
atomic(desktop,entry.encode());os.sync()
assert all(digest((root/name).read_bytes())==value for name,value in manifest.items())
assert desktop.read_text()==entry
subprocess.Popen(['/bin/bash',str(root/launcher)],cwd=root,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
deadline=time.monotonic()+12
latest=None
while time.monotonic()<deadline:
 rows=viewers()
 if len(rows)==1 and b'--stage3-only' in rows[0][1]:
  args=rows[0][1];f=root/args[args.index(b'--session-dir')+1].decode()/'observations.jsonl'
  if f.exists() and f.stat().st_size:
   try:latest=json.loads(f.read_bytes().splitlines()[-1]);break
   except ValueError:pass
 time.sleep(.25)
assert latest and latest.get('region')=='stage3',latest
assert latest.get('motor_commands_sent') is False
print('DEPLOY_JSON='+json.dumps(dict(backup=str(backup),desktop=str(desktop),exec=str(root/launcher),pid=rows[0][0],region=latest['region'],mcu_status=latest.get('mcu_status'),manifest_verified=True,boot_retested=False)))
'''

def main():
    output=remote(ROOT/'tmp/integration_mirror_ssh_20261117.py',SCRIPT)
    rows=[s.removeprefix('DEPLOY_JSON=') for s in output.splitlines() if s.startswith('DEPLOY_JSON=')]
    if len(rows)!=1:raise RuntimeError(output[-4500:])
    result=json.loads(rows[0]); evidence=ROOT/'evidence/integration/2026-11-18/stage3-resume'
    path=evidence/('pi-stage3-startup-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
    with path.open('x',encoding='utf-8') as f:json.dump(result,f,indent=2)
    print(json.dumps({'evidence':str(path),**result}))

if __name__=='__main__':main()
