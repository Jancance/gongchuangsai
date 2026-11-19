"""Read-only Pi process/config/ownership preflight. No capture or serial open."""
from pathlib import Path
import sys,json,re
from stage23_pi_install_20261116 import remote,ROOT
E=ROOT/'evidence/integration/2026-11-17/stage2-pi-preflight'
SCRIPT=r'''
from pathlib import Path
import os,json,hashlib,socket,datetime
root=Path('/home/orangepi/vision_tools/stage23_desktop_20261116')
out={'local_time':datetime.datetime.now().isoformat(),'hostname':socket.gethostname(),'processes':[],'device_owners':[],'files':{},'sessions':[],'hardware_commands':False}
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  argv=(p/'cmdline').read_bytes().split(b'\0');argv=[x.decode(errors='replace') for x in argv if x]
  owners=[]
  for fd in (p/'fd').iterdir():
   try:
    target=os.readlink(fd)
    if target.startswith(('/dev/video','/dev/ttyUSB','/dev/ttyACM')):owners.append(target)
   except OSError:pass
  if owners:out['device_owners'].append({'pid':int(p.name),'argv':argv,'devices':sorted(set(owners))})
  if any('source_fast_cycle_view_20261115.py' in a or 'source_observe' in a or 'stage23' in a for a in argv) and int(p.name)!=os.getpid() and len(' '.join(argv))<3000:
   out['processes'].append({'pid':int(p.name),'argv':argv,'cwd':os.readlink(p/'cwd')})
 except (OSError,PermissionError):pass
paths=[root/'start.sh',root/'manifest.json',root/'body-profile.json',root/'prediction-config.json',root/'orbit-config.json',root/'rim-detector.json',root/'stage3_candidate/config/stage3.json',Path('/home/orangepi/vision_tools/material_pickup_top65_20261114.json'),Path('/home/orangepi/.config/autostart/source-rimpark.desktop')]
for p in paths:
 if not p.is_file():out['files'][str(p)]={'exists':False};continue
 b=p.read_bytes();item={'exists':True,'bytes':len(b),'sha256':hashlib.sha256(b).hexdigest()}
 if p.suffix in ('.sh','.desktop'):item['text']=b.decode(errors='replace')
 elif p.suffix=='.json' and p.name!='manifest.json':item['config']=json.loads(b)
 out['files'][str(p)]=item
if (root/'manifest.json').is_file():
 m=json.loads((root/'manifest.json').read_text());out['manifest_check']={'entries':len(m),'mismatched':[n for n,h in m.items() if not (root/n).is_file() or hashlib.sha256((root/n).read_bytes()).hexdigest()!=h]}
cap=root/'captures'
if cap.is_dir():
 for p in sorted(cap.glob('run-*'),key=lambda p:p.stat().st_mtime,reverse=True)[:5]:
  if p.is_file():out['sessions'].append({'path':str(p),'mtime':p.stat().st_mtime,'tail':p.read_text(errors='replace')[-18000:]})
  else:
   files=sorted([x for x in p.iterdir() if x.is_file()],key=lambda x:x.stat().st_mtime,reverse=True)
   info={'path':str(p),'files':[{'name':x.name,'bytes':x.stat().st_size,'mtime':x.stat().st_mtime} for x in files[:12]]}
   for x in files:
    if x.name=='session.json':info['session']=json.loads(x.read_text())
   out['sessions'].append(info)
print('PREFLIGHT_JSON='+json.dumps(out,ensure_ascii=False))
'''
def main():
 E.mkdir(parents=True,exist_ok=True)
 try:result=remote(ROOT/'tmp/integration22_ssh.py',SCRIPT)
 except Exception as exc:
  (E/'error.txt').write_text(str(exc),encoding='utf-8');raise
 (E/'ssh-readonly-output.txt').write_text(result,encoding='utf-8')
 lines=[x.split('PREFLIGHT_JSON=',1)[1] for x in result.splitlines() if x.startswith('PREFLIGHT_JSON=')]
 if not lines:raise RuntimeError('No preflight JSON returned; inspect saved helper error once, do not retry automatically')
 report=json.loads(lines[-1]);(E/'preflight.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps({k:report[k] for k in ['local_time','hostname','processes','device_owners','manifest_check'] if k in report},ensure_ascii=False))
 print('Saved full config/session evidence: '+str(E/'preflight.json'))
if __name__=='__main__':main()
