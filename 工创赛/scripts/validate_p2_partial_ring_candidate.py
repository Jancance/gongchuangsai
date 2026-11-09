"""离线开发回归：真实归档图 + 已知中心合成遮挡/负样本；非板端验收。"""
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

from p2_partial_ring_candidate import detect_candidate, partial_candidate

ROOT=Path(__file__).resolve().parents[1]
APP=ROOT/'output/p2_vision_generic135_pixel1_20261027/vision_app/code'
OUT=ROOT/'output/partial-ring-candidate-validation-20261027'
sys.path.insert(0,str(APP))
import ring_station_detector as module


def synthetic(kind):
    image=np.full((1080,1920,3),235,np.uint8)
    centers=[(500,610),(950,610),(1400,610)]
    for center in centers[:2]:
        for radius in range(85,141,10):
            cv2.circle(image,center,radius,(20,20,20),3,cv2.LINE_AA)
    if kind=='partial_arc':
        for radius in range(85,141,10):
            cv2.ellipse(image,centers[2],(radius,radius),0,50,310,(20,20,20),3,cv2.LINE_AA)
    elif kind in ('occluded','short_arc','full'):
        for radius in range(85,141,10):
            cv2.circle(image,centers[2],radius,(20,20,20),3,cv2.LINE_AA)
        if kind=='occluded':
            cv2.rectangle(image,(1400,580),(1630,800),(130,45,20),-1)
        elif kind=='short_arc':
            cv2.rectangle(image,(1290,450),(1600,800),(235,235,235),-1)
    elif kind=='single_ring':
        cv2.circle(image,centers[2],130,(20,20,20),5,cv2.LINE_AA)
    elif kind=='solid_disk':
        cv2.circle(image,centers[2],130,(30,30,30),-1,cv2.LINE_AA)
    elif kind=='clutter':
        rng=np.random.default_rng(20261027)
        for _ in range(100):
            p=rng.integers([1250,450],[1560,770],size=(2,2))
            cv2.line(image,tuple(p[0]),tuple(p[1]),(20,20,20),2)
    elif kind=='extra_anchor':
        for radius in range(85,141,10):
            cv2.circle(image,(950,250),radius,(20,20,20),3,cv2.LINE_AA)
    return image,centers


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'inputs').mkdir(exist_ok=True)
    cv2.ocl.setUseOpenCL(False)
    config=module.load_ring_station_config(APP/'config/ring_station_detector.json')
    cases=[]
    base=ROOT/'evidence/p2-camera/2026-10-27'
    # 每张文件只作为快照；SHA去重，不能把副本算独立成功样本。
    seen=set()
    for folder,expect in [('pixel1-material-occlusion/run-20261027-205050-650002',True),
                          ('ring-pixel-audit/run-20261027-185938-451099',True),
                          ('ring-pixel-audit/run-20261027-190527-433108',True),
                          ('ring-shadow/run-20261027-194314-063293',True),
                          ('ring-empty/run-20261027-194859-020849',False)]:
        for path in sorted((base/folder).glob('*-raw.png')):
            digest=hashlib.sha256(path.read_bytes()).hexdigest()
            if digest in seen:
                continue
            seen.add(digest)
            frame=cv2.imdecode(np.fromfile(path,np.uint8),1)
            cases.append((folder.split('/')[0]+'-'+folder.split('/')[-1]+'-'+path.stem,frame,expect,None,str(path.relative_to(ROOT)),digest))
    for kind in ('full','occluded','partial_arc','short_arc','single_ring','solid_disk','missing','clutter','extra_anchor'):
        frame,truth=synthetic(kind)
        # 等间距弧的次优解分数接近，当前保守门预期拒绝；保留此反例，不为放行而降间隔门。
        cases.append(('synthetic-'+kind,frame,kind in ('full','occluded'),truth if kind in ('full','occluded') else None,'synthetic',None))
    real=cv2.imdecode(np.fromfile(base/'pixel1-material-occlusion/run-20261027-205050-650002/first-frame-raw.png',np.uint8),1)
    shifted=cv2.warpAffine(real,np.array([[1.,0.,21.],[0.,1.,-17.]]),(1920,1080),borderValue=(235,235,235))
    cases.append(('real-derived-translation',shifted,True,None,'derived from current occlusion image: +21px,-17px',None))
    cases.append(('real-derived-mirror',cv2.flip(real,1),True,None,'derived from current occlusion image: horizontal mirror',None))
    erased=real.copy()
    erased[420:800,1240:1720]=235
    cases.append(('real-derived-missing-right',erased,False,None,'current image with entire right ring erased',None))
    records=[]
    for name,frame,expected,truth,source,digest in cases:
        input_file=OUT/'inputs'/(name+'.png')
        if digest:
            input_file.write_bytes((ROOT/source).read_bytes())
        else:
            cv2.imencode('.png',frame)[1].tofile(input_file)
        result=(partial_candidate if name=='synthetic-partial_arc' else detect_candidate)(frame,module,config)
        ok=result['accepted']==expected
        original=module._raw_detection(frame,config)
        if result['method']=='original_quality_candidate':
            assert np.array_equal(result['centers_px'],original['centers'])
        assert not result['stable'] and not result['metric_valid'] and not result['motion_allowed'] and result['serial']=='DISABLED'
        error=None
        if truth and result['accepted']:
            error=np.linalg.norm(np.array(result['centers_px'])-truth,axis=1).tolist()
            ok=ok and max(error)<=3
        records.append(dict(name=name,passed=bool(ok),expected_accepted=expected,source=source,source_sha256=digest,
                            input_image_sha256=hashlib.sha256(input_file.read_bytes()).hexdigest(),
                            known_center_error_px=error,result=result))
        display=frame.copy()
        for point in result['centers_px']:
            p=tuple(round(v) for v in point)
            cv2.drawMarker(display,p,(0,190,255),cv2.MARKER_CROSS,24,2)
        cv2.putText(display,'OFFLINE CANDIDATE ONLY | '+result['reason'],(20,45),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,255),2)
        cv2.imencode('.png',display)[1].tofile(OUT/(name+'.png'))
        print(name,ok,result['accepted'],result['reason'],round(result['processing_ms'],1),error,flush=True)
    report=dict(cases=records,passed=sum(r['passed'] for r in records),total=len(records),
                hardware_acceptance=False,metric_valid=False,motion_allowed=False,
                code_sha256=hashlib.sha256((ROOT/'scripts/p2_partial_ring_candidate.py').read_bytes()).hexdigest(),
                original_detector_sha256=hashlib.sha256((APP/'ring_station_detector.py').read_bytes()).hexdigest(),
                limits=['Development regression only; actual occlusion images were used during tuning.',
                        'Only two complete adjacent anchors plus one outer partial ring supported.',
                        'No same-pose real ground truth or full 55-frame image sequence.',
                        'No actual belt raw image in this dataset; synthetic single ring does not replace that gate.',
                        'Host offline CPU durations do not measure Orange Pi performance or exposure latency.',
                        'Arc fractions measure edge support, not physical visible surface; distance residual is inlier-conditioned.',
                        'No temporal stability, tracking or main-program integration.',
                        'Regular synthetic partial arcs have a close runner-up and are intentionally rejected; recall is limited.'])
    # 已知平移/镜像关系检查相对几何一致性；这不是实物绝对误差真值。
    reference=next(r for r in records if r['name'].startswith('pixel1-material-occlusion-') and 'first-frame' in r['name'])
    for name,expected_points in [('real-derived-translation',np.array(reference['result']['centers_px'])+[21,-17]),
                                 ('real-derived-mirror',(np.array(reference['result']['centers_px'])*[-1,1]+[1919,0])[::-1])]:
        row=next(r for r in records if r['name']==name)
        error=np.linalg.norm(np.array(row['result']['centers_px'])-expected_points,axis=1) if row['result']['accepted'] else np.array([float('inf')])
        row['known_transform_consistency_error_px']=error.tolist()
        row['passed']=row['passed'] and bool(max(error)<2)
    report['passed']=sum(r['passed'] for r in records)
    (OUT/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    assert report['passed']==report['total'], 'See review.json for failing cases'


if __name__=='__main__':
    main()
