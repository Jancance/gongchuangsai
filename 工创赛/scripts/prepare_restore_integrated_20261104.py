"""Restore the three pre-straight-test files into a checked candidate only."""
from pathlib import Path
import json,hashlib,shutil
R=Path(__file__).resolve().parents[1];A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/restore_integrated_20261104';E=R/'evidence/integration/2026-11-04/restore-integrated';T=R/'evidence/integration/2026-11-04/straight210'
E.mkdir(parents=True,exist_ok=True);assert not S.exists()
test=json.loads((T/'manifest.json').read_text(encoding='utf-8'));fix=json.loads((R/'evidence/integration/2026-11-04/straight-dispatch-fix/manifest.json').read_text(encoding='utf-8'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files=['USER/vehicle_parameters.h','CHASSIS/USER/vehicle_mission.inc','CHASSIS/USER/main.c'];original={};changed={}
for n in files:
 assert sha(A/n)==fix['changed'].get(n,test['changed'][n]),('Actual source changed',n)
 assert sha(T/'before'/n)==test['original'][n],('Backup changed',n)
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','.git'))
for n in files:
 original[n]=sha(A/n);p=E/'before-test'/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes((A/n).read_bytes())
 (S/n).write_bytes((T/'before'/n).read_bytes());changed[n]=sha(S/n)
prior=json.loads((R/'evidence/integration/2026-11-04/dynamic-route/actual-final/result.json').read_text(encoding='utf-8'))
differences={n:dict(prior=h,restored=sha(S/n)) for n,h in prior['source_sha256'].items() if sha(S/n)!=h}
assert set(differences)=={'USER/vehicle_parameters.h'}
m=dict(actual=str(A),stage=str(S),original=original,changed=changed,installed=False,flashed=False,
 restored_from='straight210/before',straight_test_inc_left_unreferenced=True,
 prior_integration_source_differences=differences,
 retained_backup_delta='Unused legacy vehicle_drive stage2 lateral table entry 20cm/s; dedicated dynamic lateral parameter remains40cm/s',
 forward_scale_changed=False,compensation_applied=False)
(E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8');print(S)
