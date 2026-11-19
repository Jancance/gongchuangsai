"""Explicit authorized preview -> existing trial switch; no UART commands."""
from pathlib import Path
import json,sys,base64
from stage23_pi_install_20261116 import remote,ROOT
E=ROOT/'evidence/integration/2026-11-17/stage2-pi-preflight'
SCRIPT=r'''
from pathlib import Path
import os,json,hashlib,subprocess,time,signal,datetime,base64
root=Path('/home/orangepi/vision_tools/stage23_desktop_20261116')
def procs():
 found=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   a=[x.decode() for x in (p/'cmdline').read_bytes().split(b'\0') if x]
   if 'source_fast_cycle_view_20261115.py' in a:found.append({'pid':int(p.name),'argv':a,'cwd':os.readlink(p/'cwd')})
  except OSError:pass
 return found
def owners():
 found=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   dev=[]
   for fd in (p/'fd').iterdir():
    try:
     s=os.readlink(fd)
     if s.startswith(('/dev/video','/dev/ttyUSB','/dev/ttyACM')):dev.append(s)
    except OSError:pass
   if dev:found.append({'pid':int(p.name),'devices':sorted(set(dev))})
  except OSError:pass
 return found
current=procs();assert len(current)==1,current
before=current[0];assert before['cwd']==str(root) and '--preview-only' in before['argv'],before
beforeOwners=owners();assert len([x for x in beforeOwners if '/dev/ttyUSB0' in x['devices']])==1,beforeOwners
assert next(x for x in beforeOwners if '/dev/ttyUSB0' in x['devices'])['pid']==before['pid']
stamp=datetime.datetime.now().strftime('%Y%m%d-%H%M%S');backup=Path('/home/orangepi/.cache')/('stage2-preview-before-trial-'+stamp+'.json')
backup.write_text(json.dumps({'process':before,'owners':beforeOwners,'restore_argv':before['argv'],'cwd':str(root),'autostart_modified':False},indent=2))
os.kill(before['pid'],signal.SIGTERM)
deadline=time.monotonic()+12
while time.monotonic()<deadline and (procs() or owners()):time.sleep(.2)
assert not procs() and not owners(),'Old process/device owner did not release; no trial launched'
oldlogs=set((root/'captures').glob('run-*.log'))
launch=subprocess.Popen(['bash',str(root/'start-trial.sh')],cwd=root,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
deadline=time.monotonic()+10;new=None;log=None;config=None
while time.monotonic()<deadline:
 p=procs();logs=set((root/'captures').glob('run-*.log'))-oldlogs
 if len(p)==1 and '--preview-only' not in p[0]['argv'] and logs:
  log=max(logs,key=lambda p:p.stat().st_mtime)
  for line in log.read_text(errors='replace').splitlines():
   if line.startswith('CONFIG '):config=json.loads(line[7:])
  deviceOwners=owners()
  if config and len([x for x in deviceOwners if '/dev/video0'in x['devices']])==1 and len([x for x in deviceOwners if '/dev/ttyUSB0'in x['devices']])==1:new=p[0];break
 time.sleep(.2)
assert new is not None,'Trial did not reach one owner/config within startup window'
assert config['mcu_status_rx_enabled'] and config['observation_tx_enabled']
session=root/new['argv'][new['argv'].index('--session-dir')+1]
records=[];obs=session/'observations.jsonl'
if obs.is_file():
 for line in obs.read_text(errors='replace').splitlines()[-3:]:
  try:records.append(json.loads(line))
  except ValueError:pass
out={'switched_local':datetime.datetime.now().isoformat(),'before':before,'backup':str(backup),'after':new,'owners':owners(),'config':config,'log':str(log),'session':str(session),'latest_records':records,'autostart_modified':False,'uart_control_commands_sent':False,'log_tail':log.read_text(errors='replace')[-1500:]}
try:
 os.environ['DISPLAY']=':0';os.environ['XAUTHORITY']='/home/orangepi/.Xauthority'
 import gi;gi.require_version('Gdk','3.0');from gi.repository import Gdk
 w=Gdk.get_default_root_window();pix=Gdk.pixbuf_get_from_window(w,0,0,w.get_width(),w.get_height());ok,data=pix.save_to_bufferv('png',[],[]);out['desktop_png_base64']=base64.b64encode(data).decode()
except Exception as e:out['desktop_error']=str(e)
print('TRIAL_JSON='+json.dumps(out,ensure_ascii=False))
'''
if '--switch' not in sys.argv:
 print(json.dumps({'local_only':True,'hardware_access':False,'explicit_switch_required':True}));raise SystemExit(0)
result=remote(ROOT/'tmp/integration22_ssh.py',SCRIPT)
lines=[x.split('TRIAL_JSON=',1)[1] for x in result.splitlines() if x.startswith('TRIAL_JSON=')]
if not lines:
 (E/'trial-switch-error.txt').write_text(result,encoding='utf-8');raise RuntimeError('Trial switch did not return JSON; inspect evidence; do not blindly retry')
r=json.loads(lines[-1]);image=r.pop('desktop_png_base64',None)
if image:(E/'desktop-trial.png').write_bytes(base64.b64decode(image));r['desktop_local_file']=str(E/'desktop-trial.png')
(E/'trial-switch.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:r.get(k) for k in ['switched_local','before','backup','after','owners','config','log','session','desktop_local_file']},ensure_ascii=False))
