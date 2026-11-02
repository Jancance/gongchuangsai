"""Verify prepared builds and immutable sources. No hardware operations."""
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'工创赛/scripts'))
from p1_longitudinal_feedforward_sim import hashes
from p1_runtime_route import load_route
from p1_turn_fault_audit import function

out=Path(__file__).resolve().parent;firmware=out/'firmware'
before=json.loads((out/'live_before.json').read_text(encoding='utf8'))
assert hashes(ROOT/'3.stm32f407工程')==before
changed=[]
for path,sha in before.items():
    f=firmware/path
    if f.is_file() and path.startswith(('USER\\','SYSTEM\\','HARDWARE\\')) and hashlib.sha256(f.read_bytes()).hexdigest()!=sha:
        changed.append(path)
assert changed==['USER\\chassis_kinematics.c'],changed
manifest=json.loads((out/'manifest.json').read_text(encoding='utf8'))
builds=[]
for p in manifest['projects']:
    log=out/('build_'+p['variant']+'_'+p['kind']+'.log')
    text=log.read_bytes().decode('gbk',errors='replace')
    assert '0 Error(s), 0 Warning(s).' in text
    image=firmware/p['output_directory']/'gongxun-f407.axf'
    builds.append(dict(**p,image=str(image.relative_to(out)),sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
                       build_log=log.name,errors=0,warnings=0))
gates=json.loads((out/'compile_gates/report.json').read_text())
assert len(gates)==7 and all(g['passed'] for g in gates)
models=json.loads((out/'eligible_wheel_check/models_ff005_forward_only/report.json').read_text())
assert models['passed'] and len(models['models'])==40
assert models['models'][28]['completed_actions']==8
assert all(models['models'][i]['completed_actions']==1 for i in (29,30,31))
route=load_route(out/'forward210.json');assert route==[(4,21000,200,35000)]
old=ROOT/'工创赛/evidence/p1-foundation/2026-11-02/longitudinal_feedforward_v4/candidate/USER'
new=(firmware/'USER/chassis_kinematics.c').read_bytes().decode('gbk')
previous=(old/'chassis_kinematics.c').read_bytes().decode('gbk')
assert new==previous
a=(firmware/'USER/longitudinal_feedforward.h').read_bytes().decode('gbk')
b=(old/'longitudinal_feedforward.h').read_bytes().decode('gbk')
assert a[a.index('static __inline float LongitudinalFeedforward'):]==b[b.index('static __inline float LongitudinalFeedforward'):]
for f in (firmware/'USER/chassis_kinematics.c',firmware/'USER/longitudinal_feedforward.h'):
    raw=f.read_bytes();raw.decode('gbk');assert b'\n' not in raw.replace(b'\r\n',b'')
result=dict(hardware_accessed=False,downloaded=False,driven=False,github_pushed=False,
            live_unchanged=True,changed_existing_firmware_sources=changed,
            same_ff_math_as_reviewed_candidate=True,source_encoding='GBK/CRLF',
            forward_ratio=.005,backward_enabled=False,baseline_available=True,
            sampling_changed=False,protections_changed=False,builds=builds,
            actual_c_models=40,assertions=models['assertions'],compile_gates=7,
            endpoint_test_wheels=models['endpoint_wheel'][28:32],
            endpoint_injections=models['endpoint_injections'],
            actual_c_passed=models['passed'],route=route,ready_for_field_validation=True,
            vehicle_validated=False,notes='Prepare only. No repeated MATLAB simulation, no flashing or hardware access. Field condition and baseline validation remain pending.')
(out/'final_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(result,ensure_ascii=False,indent=2))
