from pathlib import Path
import hashlib,json
E=Path('D:/Desktop/工创赛/工创赛/evidence/navigation-planning/2026-11-17/right1-forward-trim-prep/medium');U=Path('D:/Desktop/工创赛/3.stm32f407工程/CHASSIS/USER')
p=E/'check_right1_c.py';s=p.read_text('utf8')
addition="\nreport['source_sha256'].update({name:hashlib.sha256((U/name).read_bytes()).hexdigest() for name in ['app_config.h','chassis_auto_test.c','chassis_kinematics.h','motion_profile.c']})\nreport['fixture_sha256']=hashlib.sha256((T/'unit.c').read_bytes()).hexdigest()\nreport['fixture_scope']='Actual prepare and precritical validation only; real right selector condition plus actual AppDebugPlan40Allowed; no RTOS backend'\n(E/'actual_c_checks.json').write_text(json.dumps(report,indent=2),'utf8')\n"
assert addition not in s;p.write_text(s+addition,'utf8')
q=E/'actual_c_checks.json';report=json.loads(q.read_text('utf8'));report['source_sha256'].update({name:hashlib.sha256((U/name).read_bytes()).hexdigest() for name in ['app_config.h','chassis_auto_test.c','chassis_kinematics.h','motion_profile.c']});report['fixture_sha256']=hashlib.sha256((E/'actual-c/unit.c').read_bytes()).hexdigest();report['fixture_scope']='Actual prepare and precritical validation only; real right selector condition plus actual AppDebugPlan40Allowed; no RTOS backend';q.write_text(json.dumps(report,indent=2),'utf8')
expected=json.loads((E/'source_after_manifest.json').read_text('utf-8-sig'))
assert all(hashlib.sha256((U/x['file']).read_bytes()).hexdigest()==x['sha256'].lower() for x in expected)
print(json.dumps(dict(status='PASS_FROZEN_SOURCE_UNCHANGED',fixture_sha256=report['fixture_sha256'],source_files=len(expected))))