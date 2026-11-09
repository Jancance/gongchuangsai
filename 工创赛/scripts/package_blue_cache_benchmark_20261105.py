from pathlib import Path
import zipfile,json,hashlib,subprocess,sys
R=Path(__file__).resolve().parents[1];D=R/'output/blue-cache-candidate-20261105';O=R/'output/blue-cache-benchmark-delivery';O.mkdir(exist_ok=True)
r=json.loads((R/'evidence/p2-camera/2026-11-05/blue-cache-candidate/replay.json').read_text(encoding='utf-8'));assert r['passed'] and r['images']==151
manifest={n:hashlib.sha256((D/n).read_bytes()).hexdigest() for n in ['project_detector.py','benchmark.py']}
archive=O/'p2_blue_cache_benchmark_20261105.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
 for n in manifest:z.write(D/n,n)
 z.writestr('manifest.json',json.dumps(manifest,indent=2))
with zipfile.ZipFile(archive) as z:assert z.testzip() is None
result=dict(file=archive.name,bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),contents=manifest,scope='offline only, no installed program changed')
(O/'delivery.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))
