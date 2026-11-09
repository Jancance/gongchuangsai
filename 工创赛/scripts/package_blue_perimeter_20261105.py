from pathlib import Path
import shutil,json,zipfile,hashlib,subprocess
R=Path(__file__).resolve().parents[1];src=R/'output/p2_vision_generic135_lightblue1_20261027/vision_app';base=R/'output/blue-perimeter-trial-20261105';dst=base/'vision_app'
assert not dst.exists(),'Refuse existing output'
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','diagnostics'))
shutil.copy2(R/'output/blue-perimeter-candidate-20261105/project_detector.py',dst/'code/project_detector.py')
cp=dst/'code/config/generic_135_camera.json';d=json.loads(cp.read_text(encoding='utf-8'));d.update(exposure_absolute=50,gain=32);cp.write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf-8')
cp=dst/'code/config/material_color_thresholds.json';d=json.loads(cp.read_text(encoding='utf-8'));d['blue_projected_silhouette_candidate']['perimeter_denoise_candidate']=True;cp.write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf-8')
changes=[]
for p in src.rglob('*'):
 if p.is_file() and p.suffix in {'.py','.json','.yaml'} and (dst/p.relative_to(src)).exists():
  if p.read_bytes()!=(dst/p.relative_to(src)).read_bytes():changes.append(p.relative_to(src).as_posix())
assert sorted(changes)==sorted(['code/project_detector.py','code/config/generic_135_camera.json','code/config/material_color_thresholds.json'])
o=R/'output/blue-perimeter-delivery';o.mkdir(exist_ok=True);zpath=o/'p2_blue_perimeter_20261105.zip'
with zipfile.ZipFile(zpath,'w',zipfile.ZIP_DEFLATED) as z:
 for p in dst.rglob('*'):
  if p.is_file() and '__pycache__' not in p.parts:z.write(p,p.relative_to(base).as_posix())
with zipfile.ZipFile(zpath) as z:assert z.testzip() is None
report=dict(file=str(zpath),sha256=hashlib.sha256(zpath.read_bytes()).hexdigest(),bytes=zpath.stat().st_size,changed_files=changes,camera_opened=False,serial_commands_sent=False)
(o/'manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(report,ensure_ascii=False))
