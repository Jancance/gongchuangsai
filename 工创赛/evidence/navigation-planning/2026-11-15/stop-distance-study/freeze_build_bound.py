"""Correct dependency provenance; keep the historical git-object copies intact."""
from pathlib import Path
import hashlib,json

out=Path(__file__).resolve().parent;root=out.parents[4];base=out.parent
prep=base/'bounded-lateral-enhancement/preparation_input_manifest.json'
build=base/'s25-field-candidate/build_summary.json'
prepData=json.loads(prep.read_text(encoding='utf-8'));buildData=json.loads(build.read_text(encoding='utf-8'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
dest=out/'frozen_build_bound';dest.mkdir(exist_ok=True)
names=['app_config.h','motion_trial20.h','motion_trial40.h','motion_profile.c','motion_profile.h','motion_distance_limits.h','chassis_kinematics.c','chassis_kinematics.h','vehicle_parameters.h']
rows=[]
for name in names:
    rel='3.stm32f407工程/CHASSIS/USER/'+name
    p=root/rel
    if not p.exists():
        rel='3.stm32f407工程/USER/'+name;p=root/rel
    expected=prepData['source_files'][rel]['sha256'];actual=sha(p);assert actual==expected,(rel,expected,actual)
    if name in buildData['source_sha256']:assert actual==buildData['source_sha256'][name]
    (dest/name).write_bytes(p.read_bytes())
    old=out/'frozen_source'/name
    rows.append(dict(source_path=rel,frozen_path=str((dest/name).relative_to(root)),sha256=actual,preparation_manifest_match=True,S25_build_source_match=actual==buildData['source_sha256'].get(name) if name in buildData['source_sha256'] else 'not listed by S25 abbreviated source map; inherited unchanged preparation dependency',historical_git_copy_sha256=sha(old) if old.exists() else None,historical_copy_matches=sha(old)==actual if old.exists() else None))
record=dict(status='CORRECTED_DEPENDENCY_PROVENANCE',auto_origin='only chassis_auto_test.c from 86ad3415 is bound by S25 build SHA; 86ad3415 is NOT the full S25 checkout',preparation_manifest=dict(path=str(prep.relative_to(root)),sha256=sha(prep),source_file_count=len(prepData['source_files'])),S25_build_summary=dict(path=str(build.relative_to(root)),sha256=sha(build),auto_sha256=buildData['source_sha256']['chassis_auto_test.c'],AXF_sha256=buildData['builds']['baseline40_integrated']['axf_sha256']),dependencies=rows,numeric_spec_changed=False,reason='Current unchanged dependencies match saved preparation1731 source manifest; S25 changed-source map and its build define baseline40 binding. Historical unbound app_config/trial20 cannot authorize numerical parameters.')
(out/'build_bound_source_manifest.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
manifestPath=out/'input_manifest.json';archive=out/'input_manifest_before_dependency_correction.json'
if not archive.exists():archive.write_bytes(manifestPath.read_bytes())
items=json.loads(archive.read_text(encoding='utf-8'))
for item in items:
    if 'frozen_source' in item['path'] and not item['path'].endswith('/chassis_auto_test.c'):
        item['role']='historical 86ad3415 git object; NOT assumed S25 dependency; compare corrected build-bound manifest'
for p in [prep,build,out/'build_bound_source_manifest.json']+list(dest.iterdir()):
    items.append(dict(path=str(p.relative_to(root)),sha256=sha(p),role='corrected build-bound dependency provenance; numeric spec unchanged'))
manifestPath.write_text(json.dumps(items,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(dict(status=record['status'],checked=len(rows),prep_sources=len(prepData['source_files']),numeric_spec_changed=False),indent=2))
