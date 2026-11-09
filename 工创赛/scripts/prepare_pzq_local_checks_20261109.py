"""Prepare a NEW local verification workspace from frozen pzq, no device I/O.

Run from the project's 工创赛 directory after cloning. Never overwrites a
candidate. Restores the legacy tmp layout required by the archived check tools;
the immutable pzq sources and original AXFs are not edited.
"""
from pathlib import Path
import hashlib,json,shutil
R=Path(__file__).resolve().parents[1];S=R/'handoff/pzq';P=R/'tmp/stage2_sub20_20261109/project'
manifest=json.loads((S/'source-manifest.json').read_text(encoding='utf-8'))
for name,h in manifest['files'].items():
 assert hashlib.sha256((S/name).read_bytes()).hexdigest()==h,name
assert not P.exists(),'Existing candidate preserved; choose another clone/workspace'
shutil.copytree(S/'project',P)
m=json.loads((S/'candidate-manifest.json').read_text(encoding='utf-8'))
for target in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
 folder=P/'Objects'/target;folder.mkdir(parents=True)
 src=S/'images'/(target+'.axf');dst=folder/src.name;shutil.copy2(src,dst)
 assert hashlib.sha256(dst.read_bytes()).hexdigest()==m['images'][target]['sha256']
 (P/'Listings'/target).mkdir(parents=True)
print('Prepared byte-verified pzq local check workspace:',P)
print('No hardware access. Next: python scripts/check_integration13_immediate_20261109.py')
print('and python scripts/check_stage2_fast_port_20261109.py; gcc is required.')
