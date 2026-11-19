"""Read current Pi viewer and bounded status tail; no camera/serial/process writes."""
from pathlib import Path
from datetime import datetime
import json
from stage23_pi_install_20261116 import ROOT, remote

SCRIPT = r'''
from pathlib import Path
import os,json,time,hashlib
root=Path('/home/orangepi/vision_tools/stage23_desktop_20261116')
out={'read_only':True,'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
     'uptime':Path('/proc/uptime').read_text().strip(),'apps':[],'owners':[]}
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  a=[x.decode() for x in (p/'cmdline').read_bytes().split(b'\0') if x]
  for fd in (p/'fd').iterdir():
   try:
    dev=os.readlink(fd)
    if dev in ('/dev/video0','/dev/ttyUSB0'):out['owners'].append({'pid':int(p.name),'device':dev})
   except OSError:pass
  if 'source_fast_cycle_view_20261115.py' not in a:continue
  row={'pid':int(p.name),'cwd':os.readlink(p/'cwd'),'argv':a};out['apps'].append(row)
  session=root/a[a.index('--session-dir')+1];f=session/'observations.jsonl'
  with f.open('rb') as stream:
   stream.seek(max(0,f.stat().st_size-131072));lines=stream.read().decode().splitlines()
  row.update(session=str(session),latest=json.loads(lines[-1]),file_age_s=time.time()-f.stat().st_mtime)
 except (OSError,ValueError):pass
manifest=json.loads((root/'manifest.json').read_text())
out['manifest_count']=len(manifest)
out['manifest_mismatches']=[n for n,h in manifest.items() if hashlib.sha256((root/n).read_bytes()).hexdigest()!=h]
config=root/'stage3_candidate/config/stage3.json'
out['stage3_config']=json.loads(config.read_text());out['stage3_config_sha256']=hashlib.sha256(config.read_bytes()).hexdigest()
print('STATUS_JSON='+json.dumps(out))
'''

if __name__ == '__main__':
    output = remote(ROOT/'tmp/integration_mirror_ssh_20261117.py', SCRIPT)
    rows = [s.split('STATUS_JSON=',1)[1] for s in output.splitlines() if s.startswith('STATUS_JSON=')]
    assert len(rows)==1, 'No unique structured result'
    result=json.loads(rows[0])
    path=ROOT/'evidence/integration/2026-11-18/stage3-resume'/('pi-continuation-readonly-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.json')
    with path.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
    print(json.dumps({'evidence':str(path),'result':result}))
