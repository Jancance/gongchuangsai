"""Integrity/contract checks only; mathematical acceptance comes from MATLAB."""
from pathlib import Path
import hashlib
import json

out=Path(__file__).resolve().parent
root=out.parents[4]
entries=json.loads((out/'input_manifest.json').read_text(encoding='utf-8'))
for item in entries:
    p=root/item['path']
    assert hashlib.sha256(p.read_bytes()).hexdigest()==item['sha256'],item['path']
result=json.loads((out/'matlab_results.json').read_text(encoding='utf-8'))
receipt=json.loads((out/'matlab_receipt.json').read_text(encoding='utf-8'))
snap=json.loads((out/'snapshot_matlab_results.json').read_text(encoding='utf-8'))
assert receipt['exit_code']==0
assert result['status']=='B0_FAIL_NOT_IDENTIFIABLE' and result['B1_executed'] is False
assert len(result['runs'])==4
assert all(not r['last_F_reconstruction_tolerance_pass'] and r['last_F_reconstruction_error_rpm'] is None for r in result['runs'])
assert all(r['internal_follow_state_identifiable'] is False for r in result['runs'])
assert snap['status']=='PASS_APPLICABLE_DIAGNOSTIC_PREREQUISITE'
assert snap['total_bytes']==176 and snap['C_ABI_sizeof_verified'] is False
assert snap['product_code_written'] is False and snap['B1_enabled'] is False
assert b'B0_FAIL_NOT_IDENTIFIABLE' in (out/'matlab_native_stdout.bin').read_bytes()
manifest=[]
for p in sorted(out.rglob('*')):
    if p.is_file() and p.name not in {'evidence_manifest.json','verification_result.json'} and '__pycache__' not in p.parts:
        manifest.append(dict(path=str(p.relative_to(out)),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
(out/'evidence_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
summary=dict(status='PASS_EVIDENCE_CONTRACT',scope='original B0 and MATLAB snapshot prerequisite; product has independent snapshot-candidate/final_verification.json',frozen_input_hashes_checked=len(entries),local_artifacts_hashed=len(manifest),B0='FAIL_NOT_IDENTIFIABLE',B1_executed=False,matlab_exit_code=0,snapshot_matlab_prerequisite='PASS',B0_model_script_wrote_product_source=False)
(out/'verification_result.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps(summary,indent=2))
