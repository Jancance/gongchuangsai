"""复核lightblue1现场包及各坐标来源分布；不覆盖上一轮review.json。"""
import contextlib
import importlib.util
import io
import json
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def main():
    spec=importlib.util.spec_from_file_location('shared_review',ROOT/'scripts/review_camera135_blue2_20261027.py')
    review=importlib.util.module_from_spec(spec);spec.loader.exec_module(review)
    review.BASE=ROOT/'evidence/p2-camera/2026-10-27/lightblue'
    review.RUN=review.BASE/'run-20261027-183210-098652'
    review.APP=ROOT/'output/p2_vision_generic135_lightblue1_20261027/vision_app/code'
    filename='lightblue1-board-review.json'
    with contextlib.redirect_stdout(io.StringIO()):
        review.main('20261027-183210-098652.zip',filename)
    report=json.loads((review.BASE/filename).read_text(encoding='utf8'))
    rows=[json.loads(s) for s in (review.RUN/'observations.jsonl').read_text().splitlines()]
    good=[r for r in rows if r['state']!='STALE_RESULT']
    report['source_coordinate_stats']={}
    for color in ('blue','lightblue'):
        targets=[t for r in good for t in r['targets'] if t['color']==color]
        stats={}
        for source in sorted({t['proposal_source'] for t in targets}):
            pts=np.array([t['center_px'] for t in targets if t['proposal_source']==source])
            stats[source]={'count':len(pts),'median_xy_px':np.median(pts,axis=0).tolist(),
                           'range_xy_px':np.ptp(pts,axis=0).tolist()}
        report['source_coordinate_stats'][color]={'geometry_reference_changes':sum(t.get('geometry_reference_changed',False) for t in targets),
                                                 'by_source':stats}
    report['all_stale_detector_had_two_colors']=all(sorted(t['label'] for t in r['detector_regions'] if t['label']!='UNKNOWN')==['blue','lightblue']
                                                  for r in rows if r['state']=='STALE_RESULT')
    report['auxiliary_helper_hash_recorded']='project_pale_component.py' in json.loads((review.RUN/'module_source_hashes.json').read_text())
    report['provenance_limit']='Original recorder did not hash the imported pale helper; four main modules, two diagnostic modules and three configs match. New track1 recorder adds both helper hashes.'
    (review.BASE/filename).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'frames':report['frames'],'normal':len(good),'normal_lightblue_missing':len(report['colors']['lightblue']['normal_missing_frames']),
                      'lightblue_center_range':report['colors']['lightblue']['center_range_xy_px'],
                      'hash_limit':report['provenance_limit']},ensure_ascii=False))


if __name__=='__main__':
    main()
