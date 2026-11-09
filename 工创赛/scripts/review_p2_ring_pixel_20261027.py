"""复算首轮R圆环像素证据、核对来源及报告；只读取现场数据并写review.json。"""
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'evidence/p2-camera/2026-10-27/ring-pixel-audit'
APP=ROOT/'output/p2_vision_generic135_lightblue1_20261027/vision_app/code'


def main(run_name='20261027-185938-451099', report_name='review.json'):
    run=BASE/('run-'+run_name)
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    rows=[json.loads(s) for s in (run/'observations.jsonl').read_text(encoding='utf8').splitlines()]
    summary=json.loads((run/'run.json').read_text(encoding='utf8'))
    archive=BASE/(run_name+'.zip')
    with ZipFile(archive) as z:
        assert z.testzip() is None
        for name in z.namelist():assert z.read(name)==(run/name).read_bytes()
    hashes=json.loads((run/'module_source_hashes.json').read_text())
    sources={name:sha(APP/name)==value for name,value in hashes.items() if name!='diagnostic_entry'}
    sources['diagnostic_entry']=sha(ROOT/'output/p2_ring_pixel_audit_20261027.py')==hashes['diagnostic_entry']
    assert all(sources.values())
    configs={}
    for name in ('ring_station_detector.json','camera_processing.json','project_profile.json'):
        source=json.loads((run/(name+'.source.json')).read_text())
        configs[name]={'release_hash':sha(APP/'config'/name)==source['sha256'],
                       'effective_equal':json.loads((run/name).read_text())==json.loads((APP/'config'/name).read_text())}
    assert all(all(v.values()) for v in configs.values())
    stamps=np.array([r['host_read_time'] for r in rows])
    usable=[r for r in rows if r['usable_for_pixel_statistics']]
    assert len(rows)==summary['frames'] and len(usable)==summary['usable_frames']
    assert np.all(np.diff(stamps)>0)
    assert all(r['metric_valid'] is False and r['motion_allowed'] is False and r['serial']=='DISABLED' for r in rows)
    assert all(r['usable_for_pixel_statistics']==(r['detected'] and len(r['centers_px'])==3 and 0<=r['result_host_age_ms']<=150) for r in rows)
    points=np.array([r['usable_centers_px'] for r in usable])
    computed=[]
    for i in range(3):
        p=points[:,i,:];median=np.median(p,axis=0)
        result={'index':i+1,'median_xy_px':median.tolist(),'range_xy_px':np.ptp(p,axis=0).tolist(),
                'p95_abs_deviation_from_median_xy_px':np.percentile(np.abs(p-median),95,axis=0).tolist()}
        assert result==summary['ring_statistics'][i]
        computed.append(result)
    report={'source_zip':archive.name,'zip_sha256':sha(archive),'crc_and_members_match':True,
            'source_matches':sources,'config_matches':configs,'frames':len(rows),'usable_frames':len(usable),
            'usable_fraction':len(usable)/len(rows),'read_completion_span_s':float(stamps[-1]-stamps[0]),
            'read_completion_interval_p50_p95_max_ms':np.percentile(np.diff(stamps)*1000,[50,95,100]).tolist(),
            'result_host_age_p50_p95_max_ms':np.percentile([r['result_host_age_ms'] for r in rows],[50,95,100]).tolist(),
            'detector_processing_p50_p95_max_ms':np.percentile([r['detector_processing_ms'] for r in rows],[50,95,100]).tolist(),
            'output_statistics':computed,
            'coordinate_reference':'Three ring centers, ordered by config. Raw 1920x1080 image origin upper left, X right/Y down.',
            'manual_paper_label_mapping':{'left_output_1':'printed 3','middle_output_2':'printed 2','right_output_3':'printed 1'},
            'mapping_source':'Visual review of first-three-centers-overlay.png, not OCR or general digit acceptance.',
            'temporal_filter':'Original algorithm uses three-frame coordinate median when quality gates pass; these are output statistics, not independent raw localization accuracy.',
            'maximum_output_axis_range_px':float(np.ptp(points,axis=0).max()),
            'maximum_output_axis_p95_abs_deviation_px':max(max(r['p95_abs_deviation_from_median_xy_px']) for r in computed),
            'metric_valid':False,'motion_allowed':False,
            'scope':'One stationary scene and position, 262 host-read timestamps; no independent exposure, residual distortion, motion, metric or physical placement acceptance.',
            'timing_limit':'Maximum processing-record gap is reported. Snapshot encoding may disturb timing; causality not established.'}
    (BASE/report_name).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({k:report[k] for k in ('frames','usable_frames','read_completion_span_s','output_statistics',
                                          'maximum_output_axis_range_px','maximum_output_axis_p95_abs_deviation_px')},ensure_ascii=False,indent=2))


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-name',default='20261027-185938-451099')
    parser.add_argument('--report-name',default='review.json')
    args=parser.parse_args()
    main(args.run_name,args.report_name)
