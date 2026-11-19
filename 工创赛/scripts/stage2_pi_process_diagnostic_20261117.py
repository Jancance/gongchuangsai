"""Explicit read-only process/boot diagnostic, without video or UART opens."""
from pathlib import Path
import json,sys
from stage23_pi_install_20261116 import remote,ROOT
E=ROOT/'evidence/integration/2026-11-17/stage2-pi-preflight'
SCRIPT=r'''
from pathlib import Path
import os,json,datetime
root=Path('/home/orangepi/vision_tools/stage23_desktop_20261116')
r={'read_time':datetime.datetime.now().isoformat(),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'uptime':Path('/proc/uptime').read_text().strip(),'processes':[],'owners':[]}
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  a=[x.decode(errors='replace') for x in (p/'cmdline').read_bytes().split(b'\0') if x]
  if 'source_fast_cycle_view_20261115.py' in a:
   stat=(p/'stat').read_text().rsplit(')',1)[1].split();ppid=int(stat[1]);parent=Path('/proc')/str(ppid)
   item={'pid':int(p.name),'argv':a,'cwd':os.readlink(p/'cwd'),'start_ticks':int(stat[19]),'ppid':ppid,'parent_argv':[x.decode(errors='replace') for x in (parent/'cmdline').read_bytes().split(b'\0') if x]}
   session=root/a[a.index('--session-dir')+1];log=Path(str(session)+'.log');item['session']=str(session);item['log_tail']=log.read_text(errors='replace')[-4000:]
   r['processes'].append(item)
  devices=[]
  for fd in (p/'fd').iterdir():
   try:
    d=os.readlink(fd)
    if d.startswith(('/dev/video','/dev/ttyUSB','/dev/ttyACM')):devices.append(d)
   except OSError:pass
  if devices:r['owners'].append({'pid':int(p.name),'devices':sorted(set(devices))})
 except OSError:pass
desktop=Path('/home/orangepi/.config/autostart/source-rimpark.desktop')
r['autostart']=desktop.read_text() if desktop.is_file() else None
print('PROCESS_JSON='+json.dumps(r,ensure_ascii=False))
'''
if '--read' not in sys.argv:
 print(json.dumps({'local_only':True,'hardware_access':False,'explicit_read_required':True}));raise SystemExit(0)
result=remote(ROOT/'tmp/integration22_ssh.py',SCRIPT)
lines=[x.split('PROCESS_JSON=',1)[1] for x in result.splitlines() if x.startswith('PROCESS_JSON=')]
if not lines:
 (E/'process-diagnostic-error.txt').write_text(result,encoding='utf-8');raise RuntimeError('Read failed; no automatic retry')
r=json.loads(lines[-1]);(E/'process-diagnostic.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(r,ensure_ascii=False))
