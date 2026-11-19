"""Read latest recorded MCU status and desktop pixels; never opens video/UART."""
import json,base64
from pathlib import Path
from stage23_pi_install_20261116 import remote,ROOT
E=ROOT/'evidence/integration/2026-11-17/stage2-pi-preflight'
SCRIPT=r'''
from pathlib import Path
import json,hashlib,datetime,os,base64
root=Path('/home/orangepi/vision_tools/stage23_desktop_20261116')
active=[]
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit():continue
 try:
  args=[x.decode() for x in (proc/'cmdline').read_bytes().split(b'\0') if x]
  if 'source_fast_cycle_view_20261115.py' in args:active.append(args)
 except OSError:pass
assert len(active)==1,active
cap=root/active[0][active[0].index('--session-dir')+1]/'observations.jsonl'
r={'read_time':datetime.datetime.now().isoformat(),'source':str(cap)}
with cap.open('rb') as f:
 f.seek(max(0,cap.stat().st_size-200000));lines=f.read().splitlines()
records=[]
for line in lines[-8:]:
 try:records.append(json.loads(line))
 except ValueError:pass
r['latest_records']=records[-3:]
code=Path('/home/orangepi/p2-source-observer-20261106/vision_app/code')
names=['material_color_thresholds.json','camera_processing.json','camera_processing_config.json','source_metric_profile.json','source_plane_calibration.json']
r['external_configs']={}
for p in [code/'config'/n for n in names]:
 if p.is_file():
  b=p.read_bytes();r['external_configs'][str(p)]={'sha256':hashlib.sha256(b).hexdigest(),'config':json.loads(b)}
r['code_config_files']=[p.name for p in (code/'config').glob('*.json')]
try:
 os.environ['DISPLAY']=':0';os.environ['XAUTHORITY']='/home/orangepi/.Xauthority'
 import gi;gi.require_version('Gdk','3.0');from gi.repository import Gdk
 w=Gdk.get_default_root_window();pix=Gdk.pixbuf_get_from_window(w,0,0,w.get_width(),w.get_height())
 ok,data=pix.save_to_bufferv('png',[],[]);r['desktop_png_base64']=base64.b64encode(data).decode();r['desktop_capture_time']=datetime.datetime.now().isoformat()
except Exception as e:r['desktop_capture_error']=str(e)
print('STATUS_JSON='+json.dumps(r,ensure_ascii=False))
'''
result=remote(ROOT/'tmp/integration22_ssh.py',SCRIPT)
lines=[x.split('STATUS_JSON=',1)[1] for x in result.splitlines() if x.startswith('STATUS_JSON=')]
if not lines:
 (E/'status-helper-output.txt').write_text(result,encoding='utf-8');raise RuntimeError('No status JSON; saved helper output, no retry')
r=json.loads(lines[-1]);image=r.pop('desktop_png_base64',None)
if image:(E/'desktop-current.png').write_bytes(base64.b64decode(image));r['desktop_local_file']=str(E/'desktop-current.png')
(E/'status-current.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({'read_time':r['read_time'],'records':len(r['latest_records']),'desktop_local_file':r.get('desktop_local_file'),'desktop_capture_error':r.get('desktop_capture_error'),'external_configs':{p:x['sha256'] for p,x in r['external_configs'].items()}},ensure_ascii=False))
