"""Four exact-byte local source restores only. No Git, network or hardware."""
from pathlib import Path
import hashlib,json

R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER'
F=E.parent/'float-timing-20261115';B=E.parent/'bounded-lateral-enhancement'
H=R/'工创赛/evidence/navigation-planning/2026-11-14/baseline40-field/field-candidate-conservative-ff'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
restore={
 'chassis_auto_test.c':(B/'chassis_auto_test.before.c','4434979a40f940d8ee582378e5b57b45da8caeefb29bdde1558d1b17e85dac19'),
 'chassis_auto_test.h':(F/'chassis_auto_test.h.before','e940c64b4a5126f63a6cc6e4d814be2ef3096bd0d59f2e049d6a53257033b764'),
 'hwt101.c':(F/'hwt101.c.before','690b864c1144baf247a93cffb8be585675a0612002333edaf006026b5dbb3c90'),
 'hwt101.h':(F/'hwt101.h.before','eb01b7feb7e18c73495c472b09497514ab14de6941013683315b61f929b56429')}
preserve={'app_motion_runtime.c':'2cded70cccf56c9464a0787be6656e81aad5e9a9d9334620d3c5a958b5eef611','app_serial_runtime.c':'d99860e5266f1e56156f41f88452895aff55d20a35459649ca97f72facdfbca4'}
assert sha(U/'chassis_auto_test.c')=='ada294ab54d230e1c791a701b99c9007dc3c15ee7711633d63fe9855b4de122b'
for name,(path,digest) in restore.items():assert sha(path)==digest,(name,sha(path))
for name,digest in preserve.items():assert sha(U/name)==digest
build=json.loads((H/'build_summary.json').read_text('utf8'))
assert build['source_sha256']['chassis_auto_test.c']==restore['chassis_auto_test.c'][1]
oldAXF=build['builds']['baseline40_integrated']['axf_sha256']
runs=json.loads((H/'four_ff_field_runs_summary.json').read_text('utf8'))
assert runs['candidate_sha256']==oldAXF=='2661174f8e54f31285cb6666f2ddd1dada110e53f005201b92c5d3b85e29c1e7'
assert all(x['source_sha256']==restore['chassis_auto_test.c'][1] and x['candidate_sha256']==oldAXF for x in runs['successful_runs'])
auths=[]
for n in range(1,5):
 p=H.parent/f'conservative-ff-run{n}/authorization.json';a=json.loads(p.read_text('utf8'))
 assert a['source_sha256']==restore['chassis_auto_test.c'][1]
 auths.append(dict(path=str(p.relative_to(R)),sha256=sha(p),source_sha256=a['source_sha256']))
measure=json.loads((H/'four_ff_field_runs_compact.json').read_text('utf8'))
offset=[x['operator_measurement']['signed_right_mm'] for x in measure]
assert offset==[3.5,-.5,10,7.5]
prior=json.loads((B/'preparation_input_manifest.json').read_text('utf8'))['source_files']
before={path:sha(R/path) for path in prior if (R/path).is_file()}
archive=E/'recover_previous_diag';archive.mkdir(exist_ok=True)
report={}
for name,(path,digest) in restore.items():
 current=(U/name).read_bytes();(archive/name).write_bytes(current)
 report[name]=dict(previous_sha256=hashlib.sha256(current).hexdigest(),restore_path=str(path.relative_to(R)),restored_sha256=digest,previous_bytes=len(current),restored_bytes=path.stat().st_size)
for name,(path,digest) in restore.items():(U/name).write_bytes(path.read_bytes())
after={path:sha(R/path) for path in before}
changed=[path for path in before if before[path]!=after[path]]
expected=['3.stm32f407工程/CHASSIS/USER/'+n for n in restore]
assert sorted(changed)==sorted(expected),changed
for name,digest in preserve.items():assert sha(U/name)==digest
for name,(path,digest) in restore.items():assert (U/name).read_bytes()==path.read_bytes()
record=dict(status='LOCAL_SOURCE_RESTORE_COMPLETE',label='half geometric FF control + current normal-DONE VM retention and serial handshake fixes',restored=report,preserved=preserve,source_dependencies_checked=len(before),changed_source_paths=changed,old_half_build_AXF=oldAXF,new_AXF_is_not_claimed_old_identical=True,historical_half_signed_right_mm=offset,historical_half_ruler_MAE_mm=sum(abs(x) for x in offset)/4,historical_auths=auths,hardware_accessed=False,git_accessed=False,new_hardware_runs_authorized=False)
(E/'source_restore_receipt.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),'utf8')
(E/'dependency_source_manifest.json').write_text(json.dumps({'before':before,'after':after},ensure_ascii=False,indent=2),'utf8')
print(json.dumps({k:record[k] for k in ['status','restored','preserved','source_dependencies_checked','changed_source_paths','historical_half_ruler_MAE_mm']},ensure_ascii=False,indent=2))
