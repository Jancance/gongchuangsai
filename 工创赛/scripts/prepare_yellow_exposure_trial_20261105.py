"""Run on Orange Pi: make an independent copy for an exposure trial, no hardware I/O."""
from pathlib import Path
import shutil,json,hashlib
src=Path('/home/orangepi/p2-generic135-lightblue1-20261027/vision_app')
dst=Path('/home/orangepi/p2-yellow-exposure50-20261105/vision_app')
assert src.is_dir() and not dst.exists(),'Source missing or trial directory already exists; refuse overwrite'
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','diagnostics','color-follow-audits'))
p=dst/'code/config/generic_135_camera.json';d=json.loads(p.read_text(encoding='utf-8'));before=dict(d)
assert d['width']==1920 and d['height']==1080 and not d.get('motion_allowed',False)
d.update(exposure_absolute=50,gain=32)
p.write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf-8')
changed=[]
for q in src.rglob('*'):
 if q.is_file() and q.suffix in {'.py','.json','.yaml'} and (dst/q.relative_to(src)).exists():
  other=dst/q.relative_to(src)
  if q.read_bytes()!=other.read_bytes():changed.append(q.relative_to(src).as_posix())
assert changed==['code/config/generic_135_camera.json'],changed
(dst.parent/'exposure-trial.json').write_text(json.dumps(dict(source=str(src),destination=str(dst),before=before,after=d,changed_files=changed,opened_camera=False,motion_commands_sent=False),ensure_ascii=False,indent=2),encoding='utf-8')
print('Prepared',dst,'only exposure50/gain32 changed; no camera/serial opened')
