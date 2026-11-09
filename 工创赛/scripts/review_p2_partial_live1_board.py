"""复核live1现场来源/逐帧状态并拆分时效，不修改现场原包。"""
from collections import Counter
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'evidence/p2-camera/2026-10-27/partial-live1-board'


def main():
    archive=ROOT/'20261027-214551-920305.zip'
    manifest=ROOT/'output/p2_partial_ring_live1_20261027/LIVE_MANIFEST.json'
    with ZipFile(archive) as z:
        assert z.testzip() is None
        for name in z.namelist():
            assert Path(name).name==name and z.read(name)==(BASE/name).read_bytes()
    source=json.loads((BASE/'source.json').read_text(encoding='utf8'))
    assert source['manifest_sha256']==hashlib.sha256(manifest.read_bytes()).hexdigest()
    assert source['files']==json.loads(manifest.read_text(encoding='utf8'))['files']
    rows=[json.loads(s) for s in (BASE/'observations.jsonl').read_text(encoding='utf8').splitlines()]
    run=json.loads((BASE/'run.json').read_text(encoding='utf8'))
    assert len(rows)==run['frames']==87
    assert dict(Counter(row['status']['state'] for row in rows))==run['states']
    assert np.all(np.diff([r['host_read_time'] for r in rows])>0)
    for row in rows:
        state=row['status']
        assert not state['motion_allowed'] and not state['metric_valid'] and state['serial']=='DISABLED'
        assert row['detector_result_age_ms']==(row['detector_finished']-row['host_read_time'])*1000
        assert row['candidate']['accepted']
        assert row['candidate']['method']=='partial_arc_template_v2'
        if state['state']=='STALE':assert state['candidate_centers_px']==state['stable_centers_px']==[]
        assert not state['pixel_ready']
    values=dict(processing=[r['candidate']['processing_ms'] for r in rows],
                before_detector_estimated=[r['detector_result_age_ms']-r['candidate']['processing_ms'] for r in rows],
                detector_result_age=[r['detector_result_age_ms'] for r in rows],
                after_detector_to_display_submission=[r['display_submission_age_ms']-r['detector_result_age_ms'] for r in rows],
                display_submission_age=[r['display_submission_age_ms'] for r in rows])
    report=dict(archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),source_manifest_match=True,
                frames=len(rows),states=run['states'],quality_accepted=sum(r['candidate']['accepted'] for r in rows),
                fresh_at_detection=sum(r['detector_result_age_ms']<=150 for r in rows),
                fresh_at_display_submission=sum(r['display_submission_age_ms']<=150 for r in rows),
                stage_min_p50_p95_max_ms={name:np.percentile(v,[0,50,95,100]).tolist() for name,v in values.items()},
                processing_stage_medians_ms={k:float(np.median([r['candidate']['stages_ms'][k] for r in rows])) for k in rows[0]['candidate']['stages_ms']},
                limits=['Before-detector duration is estimated from detector result age minus candidate timer; no explicit start stamp in live1.',
                        'After-detector duration combines result-queue scheduling, gate, rendering and imshow; live1 cannot uniquely attribute it to imshow.',
                        'Candidate quality acceptance is not independently measured circle-center truth.',
                        'Host timestamps and imshow submission do not measure exposure or monitor presentation.'],
                metric_valid=False,motion_allowed=False,hardware_acceptance=False)
    (BASE/'verified-review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
