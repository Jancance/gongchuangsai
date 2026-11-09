"""验证v2与冻结v1的语义逐字段一致，记录同图交替计时；不是硬件验收。"""
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'output/p2_partial_ring_offline_20261027'))
sys.path.insert(0,str(ROOT/'scripts'))
import p2_partial_ring_candidate as v1
import p2_partial_ring_candidate_v2 as v2
import ring_station_detector as module
from p2_ring_raw_shared import extract_with_edges,raw_from_candidates


def semantic(result):
    return {k:v for k,v in result.items() if k not in ('processing_ms','method','stages_ms')}


def main():
    base=ROOT/'output/p2_partial_ring_offline_20261027'
    config=module.load_ring_station_config(base/'ring_station_detector.json')
    manifest=json.loads((base/'manifest.json').read_text(encoding='utf8'))
    cv2.ocl.setUseOpenCL(False)
    records=[]
    for case in manifest['cases']:
        frame=cv2.imdecode(np.fromfile(base/'inputs'/(case['name']+'.png'),np.uint8),1)
        observations,_,_=extract_with_edges(frame,config)
        candidates=module._cluster_observations(observations,config)
        assert raw_from_candidates(candidates,frame.shape[1],config,module)==module._raw_detection(frame,config)
        a=(v1.partial_candidate if case['name']=='synthetic-partial_arc' else v1.detect_candidate)(frame,module,config)
        b=(v2.partial_candidate if case['name']=='synthetic-partial_arc' else v2.detect_candidate)(frame,module,config)
        assert semantic(a)==semantic(b),case['name']
        assert b['accepted']==case['expected_accepted']
        records.append(dict(name=case['name'],raw_exact_match=True,semantic_exact_match=True,
                            v1_ms=a['processing_ms'],v2_ms=b['processing_ms']))
    benchmark=[]
    for case in manifest['cases'][:3]:
        frame=cv2.imdecode(np.fromfile(base/'inputs'/(case['name']+'.png'),np.uint8),1)
        times={'v1':[],'v2':[]}
        for i in range(5):
            for name,model in ([('v1',v1),('v2',v2)] if i%2==0 else [('v2',v2),('v1',v1)]):
                result=model.detect_candidate(frame,module,config)
                times[name].append(result['processing_ms'])
        benchmark.append(dict(name=case['name'],times_ms=times,
                              v1_p50_p95_ms=np.percentile(times['v1'],[50,95]).tolist(),
                              v2_p50_p95_ms=np.percentile(times['v2'],[50,95]).tolist()))
    out=ROOT/'output/partial-ring-v2-validation-20261027'
    out.mkdir(exist_ok=True)
    report=dict(cases=records,benchmark=benchmark,passed=len(records),opencv=cv2.__version__,
                hardware_acceptance=False,metric_valid=False,motion_allowed=False)
    (out/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'passed':len(records),'benchmark':benchmark},indent=2))


if __name__=='__main__':
    main()
