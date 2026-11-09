"""Stage two user-requested distances; do not remove unidentified fault protections."""
from pathlib import Path
import hashlib,json,shutil
R=Path(__file__).resolve().parents[1];A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/vm_review_retune_20261104';E=R/'evidence/integration/2026-11-04/vm-review-retune'
E.mkdir(parents=True,exist_ok=True);assert not S.exists()
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','.git'))
rel='USER/vehicle_parameters.h';p=S/rel;b=p.read_bytes();q=b
for old,new in [(b'#define VEHICLE_STAGE1_FORWARD_DISTANCE_CM 15.0f',b'#define VEHICLE_STAGE1_FORWARD_DISTANCE_CM 10.0f'),(b'#define VEHICLE_STAGE2_GOAL_OFFSET_X_MM (-50.0f)',b'#define VEHICLE_STAGE2_GOAL_OFFSET_X_MM (-100.0f)')]:
 assert q.count(old)==1,old;q=q.replace(old,new)
p.write_bytes(q);(E/'before-vehicle_parameters.h').write_bytes(b)
(E/'manifest.json').write_text(json.dumps(dict(actual=str(A),stage=str(S),original={rel:hashlib.sha256(b).hexdigest()},changed={rel:hashlib.sha256(q).hexdigest()},installed=False,flashed=False,protection_changes=[],latest_fault_user_reset=True,latest_fault_cause_unknown=True,user_requested_forward_cm=10,goal_mm=[2060,1200]),ensure_ascii=False,indent=2),encoding='utf-8');print(S)
