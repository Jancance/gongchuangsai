"""Stage only stage selection; preserve all accepted stage1 and extension fixes."""
from pathlib import Path
import shutil,json,hashlib
R=Path(__file__).resolve().parents[1]
A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/stage2_retest_20261104';E=R/'evidence/integration/2026-11-04/stage2-retest'
E.mkdir(parents=True,exist_ok=True);assert not S.exists()
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','.git'))
rel='USER/vehicle_parameters.h';p=S/rel;b=p.read_bytes()
assert b.count(b'#define VEHICLE_TEST_STAGE 1U')==1
q=b.replace(b'#define VEHICLE_TEST_STAGE 1U',b'#define VEHICLE_TEST_STAGE 2U')
p.write_bytes(q);(E/'before-vehicle_parameters.h').write_bytes(b)
(E/'manifest.json').write_text(json.dumps(dict(actual=str(A),stage=str(S),original={rel:hashlib.sha256(b).hexdigest()},changed={rel:hashlib.sha256(q).hexdigest()},installed=False,flashed=False),ensure_ascii=False,indent=2),encoding='utf-8')
print(S)
