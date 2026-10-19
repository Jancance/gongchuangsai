"""Archive current static desk evidence; no camera/config changes."""
from collections import Counter
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
source=ROOT/'evidence/20261006-201311-289701.zip'
previous=ROOT/'evidence/20261006-195642-102168.zip'
out=ROOT/'output/v9-static-pair-baseline-20261006';out.mkdir(parents=True,exist_ok=True)
with ZipFile(source) as archive,ZipFile(previous) as old,ZipFile(ROOT/'output/follow_speed_patch_v9_2026-10-06.zip') as package:
    assert archive.testzip() is None
    rows=[json.loads(s) for s in archive.read('observations.jsonl').decode().splitlines()]
    hashes=json.loads(archive.read('module_source_hashes.json'))
    manifest=json.loads(package.read('PATCH_MANIFEST.json'))
    matched={Path(name).name:hashes[Path(name).name]==entry['after_sha256'] for name,entry in manifest['files'].items()}
    configs={name:old.read(name)==archive.read(name) for name in ('material_color_thresholds.json','camera_processing.json','project_profile.json')}
    labels=Counter(t['color'] for r in rows for t in r['targets'])
    ids={color:sorted({t['id'] for r in rows for t in r['targets'] if t['color']==color}) for color in labels}
    checks={'v9_modules_match':all(matched.values()),'settings_match_previous_run':all(configs.values()),
            'all_states_following':all(r['state']=='FOLLOWING_PIXEL' for r in rows),
            'each_result_one_blue_one_lightblue':all(sorted(t['color'] for t in r['targets'])==['blue','lightblue'] for r in rows),
            'ids_do_not_change':all(len(values)==1 for values in ids.values()),
            'all_result_host_ages_within_150ms':all(0<=r['result_host_age_ms']<=150 for r in rows),
            'all_action_gates_off':all(not r['ready'] and not r['motion_allowed'] for r in rows)}
    report={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'run':json.loads(archive.read('run.json')),'module_hashes':hashes,
            'module_matches':matched,'settings_unchanged':configs,'checks':checks,
            'duration_between_first_and_last_result_s':rows[-1]['timestamp']-rows[0]['timestamp'],
            'observed_label_counts':dict(labels),'ids_by_color':ids,
            'detector_ms_p50_p95_max':np.percentile([r['detector_call_ms'] for r in rows],[50,95,100]).tolist(),
            'result_host_age_ms_p50_p95_max':np.percentile([r['result_host_age_ms'] for r in rows],[50,95,100]).tolist(),
            'scope':'Current static desk placement/lighting only; labels in logs and sampled original images. Not moving/grasp/field acceptance.'}
    for name in ('material_color_thresholds.json','camera_processing.json','project_profile.json','module_source_hashes.json'):
        (out/name).write_bytes(archive.read(name))
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
assert all(checks.values()),checks
print('PASS static desk result checks:',len(rows),'results')
print(out/'report.json')
