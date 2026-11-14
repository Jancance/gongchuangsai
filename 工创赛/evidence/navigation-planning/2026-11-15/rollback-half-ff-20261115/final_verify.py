from pathlib import Path
import hashlib,json,re,struct,subprocess

R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER'
F=E.parent/'float-timing-20261115';B=E.parent/'bounded-lateral-enhancement'
H=R/'工创赛/evidence/navigation-planning/2026-11-14/baseline40-field/field-candidate-conservative-ff'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
load=lambda p:json.loads(p.read_text('utf8'))
restore=load(E/'source_restore_receipt.json');deps=load(E/'dependency_source_manifest.json')
for rel,digest in deps['after'].items():assert sha(R/rel)==digest,rel
for name,record in restore['restored'].items():
 assert sha(U/name)==record['restored_sha256']
 assert (U/name).read_bytes()==(R/record['restore_path']).read_bytes()
 assert sha(E/'recover_previous_diag'/name)==record['previous_sha256']
for name,digest in restore['preserved'].items():assert sha(U/name)==digest
encoding={}
for name in restore['restored']:
 b=(U/name).read_bytes();assert b.decode('gbk').encode('gbk')==b
 encoding[name]=dict(lf=b.count(b'\n'),crlf=b.count(b'\r\n'),sha256=sha(U/name))
 if name=='chassis_auto_test.h':assert b.count(b'\r\n')==1
 else:assert b.count(b'\n')==b.count(b'\r\n')

def allocated(path):
 b=path.read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b);out=[]
 for i in range(h[12]):
  _,kind,flags,address,offset,size,*_=struct.unpack_from('<IIIIIIIIII',b,h[6]+i*h[11])
  if flags&2 and size:out.append([kind,flags,address,size,hashlib.sha256(bytes(size) if kind==8 else b[offset:offset+size]).hexdigest()])
 return out

build=load(E/'build_summary.json');prior=load(F/'build_summary.json');oldHalf=load(H/'build_summary.json')
checks={}
for name,record in build['builds'].items():
 assert record['returncode']==0 and sha(Path(record['axf']))==record['axf_sha256']
 log=(E/(name+'.log')).read_text('gbk');assert '0 Error(s), 0 Warning(s)' in log
 same=allocated(Path(record['axf']))==allocated(Path(prior['builds'][name]['axf']))
 checks[name]=dict(program_size=record['program_size'],axf=record['axf'],axf_sha256=record['axf_sha256'],allocated_equal_to_before_rollback=same,old_half_AXF_sha256=oldHalf['builds'][name]['axf_sha256'],equals_old_half_whole_AXF=record['axf_sha256']==oldHalf['builds'][name]['axf_sha256'])
 symbols=subprocess.check_output(['D:/Keil_v5/ARM/arm5_compiler/Bin/fromelf.exe','--text','-s',record['axf']]).decode(errors='replace')
 assert 'g_chassis_auto_tail_snapshots' not in symbols and 'g_chassis_auto_lateral_timing' not in symbols
 if name=='baseline_off_integrated':assert same

actual=load(E/'actual_c_check.json');historical=load(H/'actual_c_check.json')
assert actual['all_passed'] and actual['tested_auto_sha256']==sha(U/'chassis_auto_test.c')
assert actual['entry_observation']==historical['entry_observation'] and actual['ff_observation']==historical['ff_observation']
halfMatlab=load(H/'matlab_summary.json');vmMatlab=load(B/'vm_matlab_summary.json')
assert halfMatlab['completed'] and halfMatlab['selected_FF_fraction']==.5
assert next(x for x in halfMatlab['results'] if x['id']==halfMatlab['selected_id'])['passed']
assert vmMatlab['passed'] and vmMatlab['cases']==249
vmC=load(B/'vm_actual_c_check.json');serialC=load(B/'serial_actual_c_check.json')
assert vmC['all_passed'] and vmC['source_sha256']==sha(U/'app_motion_runtime.c')
assert serialC['all_passed'] and serialC['source_sha256']==sha(U/'app_motion_runtime.c') and serialC['serial_source_sha256']==sha(U/'app_serial_runtime.c')
header=(U/'hwt101.h').read_text('gbk')
assert '#define HWT_TIMING_SCOPES 3U' in header and '#define HWT_TIMING_FIELDS 28U' in header
assert 'HWT101_TimingEnabled' not in header
refs=[H/'matlab_summary.json',H/'matlab_process.json',B/'vm_matlab_summary.json',B/'vm_actual_c_check.json',B/'serial_actual_c_check.json',H/'four_ff_field_runs_summary.json']
result=dict(status='PASS_LOCAL_ROLLBACK_ONLY',label='half geometric FF control + current normal-DONE VM retention and serial handshake fixes',
 restored_sources={n:sha(U/n) for n in restore['restored']},preserved_sources=restore['preserved'],
 dependency_sources_checked=len(deps['after']),changed_sources=restore['changed_source_paths'],encoding=encoding,
 actual_ARM=dict(cases=actual['cases'],assertions=actual['assertions'],half_entry_observation_identical=True,half_FF_observation_identical=True,external_calls_stubbed=True),
 builds=checks,legacy_protocol=dict(scopes=3,fields=28,new_timing_and_tail_symbols_absent=True),
 reused_MATLAB=dict(half_selected_FF=.5,half_completed=True,normal_done_VM_cases=249,numerical_control_not_new=True),
 reference_evidence=[dict(path=str(p.relative_to(R)),sha256=sha(p)) for p in refs],
 rollback_reversible=dict(previous_diag_sources=str((E/'recover_previous_diag').relative_to(R)),automatically_reapplied=False),
 hardware_accessed=False,downloaded=False,reset=False,serial_accessed=False,git_accessed=False,
 limitations='New isolated build is half control plus newer VM/serial fixes, not the original 2661174f whole AXF; source/ARM/build evidence is not a fresh physical pass. User cancelled further physical tests; no startup-fault repair attempted.')
(E/'final_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),'utf8')
print(json.dumps({k:result[k] for k in ['status','restored_sources','preserved_sources','actual_ARM','builds','legacy_protocol']},ensure_ascii=False,indent=2))
