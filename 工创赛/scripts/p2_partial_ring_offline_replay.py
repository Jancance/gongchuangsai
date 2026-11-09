"""独立已存图片复核；不打开相机、不导入main、不访问串口或修改应用。"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

import cv2
import numpy as np

from p2_partial_ring_candidate import detect_candidate, partial_candidate
import ring_station_detector as module


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    base=Path(__file__).resolve().parent
    manifest=json.loads((base/'manifest.json').read_text(encoding='utf8'))
    for name,digest in manifest['files'].items():
        path=(base/name).resolve()
        assert path.is_relative_to(base),name
        assert hashlib.sha256(path.read_bytes()).hexdigest()==digest,name
    print('source_and_fixture_hashes=PASS camera_opened=False serial=DISABLED motion_allowed=False',flush=True)
    if args.check:
        return
    cv2.ocl.setUseOpenCL(False)
    config=module.load_ring_station_config(base/'ring_station_detector.json')
    out=base/('offline-results-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    out.mkdir()
    records=[]
    for case in manifest['cases']:
        frame=cv2.imdecode(np.fromfile(base/'inputs'/(case['name']+'.png'),np.uint8),1)
        assert frame is not None,case['name']
        detect=partial_candidate if case['name']=='synthetic-partial_arc' else detect_candidate
        result=detect(frame,module,config)
        passed=result['accepted']==case['expected_accepted']
        records.append(dict(name=case['name'],passed=passed,result=result))
        print(case['name'], 'PASS' if passed else 'DIFF',result['reason'],
              'offline_cpu_ms=%.1f'%result['processing_ms'],flush=True)
        for point in result['centers_px']:
            cv2.drawMarker(frame,tuple(round(v) for v in point),(0,190,255),cv2.MARKER_CROSS,24,2)
        cv2.putText(frame,'OFFLINE CANDIDATE ONLY - NO LIVE FRAME',(20,45),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,255),2)
        cv2.imencode('.png',frame)[1].tofile(out/(case['name']+'.png'))
    report=dict(opencv=cv2.__version__,cases=records,passed=sum(c['passed'] for c in records),total=len(records),
                camera_opened=False,serial='DISABLED',motion_allowed=False,metric_valid=False,hardware_acceptance=False,
                source_manifest_sha256=hashlib.sha256((base/'manifest.json').read_bytes()).hexdigest(),
                scope='Archived-image replay only; timing is offline CPU processing, not live result age or exposure latency.')
    # 已知变换检查保持与本地一致；没有绝对实物真值。
    reference=next(r for r in records if r['name'].startswith('pixel1-material-occlusion-') and 'first-frame' in r['name'])
    if reference['result']['accepted']:
        for name,truth in [('real-derived-translation',np.array(reference['result']['centers_px'])+[21,-17]),
                           ('real-derived-mirror',(np.array(reference['result']['centers_px'])*[-1,1]+[1919,0])[::-1])]:
            row=next(r for r in records if r['name']==name)
            errors=np.linalg.norm(np.array(row['result']['centers_px'])-truth,axis=1).tolist() if row['result']['accepted'] else None
            row['relative_transform_error_px']=errors
            row['passed']=row['passed'] and errors is not None and max(errors)<2
    else:
        for row in records:
            if row['name'] in ('real-derived-translation','real-derived-mirror'):
                row['passed']=False
    report['passed']=sum(c['passed'] for c in records)
    (out/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    archive=out.with_suffix('.zip')
    with ZipFile(archive,'x',ZIP_DEFLATED) as z:
        for path in out.iterdir():
            z.write(path,path.name)
    print('passed=%d/%d'%(report['passed'],report['total']))
    print('Evidence ZIP:',archive)
    if report['passed']!=report['total']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
