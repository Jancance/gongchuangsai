from pathlib import Path
import zipfile,json,hashlib
R=Path(__file__).resolve().parents[1];D=R/'output/hsv-mask-candidate-20261105';E=R/'evidence/p2-camera/2026-11-05/hsv-mask-candidate'
r=json.loads((E/'replay.json').read_text(encoding='utf-8'));assert r['passed'] and r['HSV_values_exhaustively_checked']==256**3 and r['images']==151
s=(R/'output/blue-cache-candidate-20261105/benchmark.py').read_text(encoding='utf-8')
s=s.replace("sys.path.insert(0,str(APP));cv2.setNumThreads(8)","sys.path[:0]=[str(HERE),str(APP)];cv2.setNumThreads(8)")
s=s.replace("after_cache","after_hsv_masks").replace("before_cache","before_hsv_masks")
(D/'benchmark.py').write_text(s,encoding='utf-8')
O=R/'output/hsv-benchmark-delivery';O.mkdir(exist_ok=True);p=O/'p2_hsv_benchmark_20261105.zip'
manifest={n:hashlib.sha256((D/n).read_bytes()).hexdigest() for n in ['project_detector.py','project_hsv_masks.py','benchmark.py']}
with zipfile.ZipFile(p,'w',zipfile.ZIP_DEFLATED) as z:
 for n in manifest:z.write(D/n,n)
 z.writestr('manifest.json',json.dumps(manifest,indent=2))
with zipfile.ZipFile(p) as z:assert z.testzip() is None
d=dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),file=p.name,contents=manifest,camera_opened=False,installed_program_modified=False)
(O/'delivery.json').write_text(json.dumps(d,indent=2),encoding='utf-8');print(json.dumps(d))
