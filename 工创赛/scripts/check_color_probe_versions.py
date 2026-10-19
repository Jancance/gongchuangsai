"""Repeatable development regression on archived images, not a holdout score."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
import p2_color_probe as v0
import p2_color_probe_v1 as v1

ROOT=Path(__file__).resolve().parents[1]
def main():
    base=ROOT/'evidence'
    samples=[]
    for n,e in [
        ('red-e50-g30.png',['red']),('yellow-e50-g30.png',['yellow']),
        ('blue-e50-g30.png',['blue']),('blue-day-e50-g30.png',['blue']),
        ('lightblue-e50-g30.png',['lightblue']),('green-day-e50-g30.png',['green']),
        ('black-day-e50-g30.png',['black']),('empty-day-e50-g30.png',[])]:
        samples.append((base/'six_color_crosscheck_2026-09-24/inputs'/n,e))
    for folder,n,e in [
        ('black_blue_pair_holdout_2026-09-24','black-blue-pair-e50-g30.png',['black','blue']),
        ('blue_lightblue_pair_holdout_2026-09-25','blue-lightblue-pair-e50-g30.png',['black','lightblue']),
        ('blue_lightblue_confirmed_holdout_2026-09-25','blue-lightblue-confirmed-e50-g30.png',['blue','lightblue'])]:
        samples.append((base/folder/n,e))
    for folder,e in [
        ('green_black_audit_20260925_202633',['green','black']),
        ('blue_lightblue_audit_20260925_203640',['blue','lightblue']),
        ('red_yellow_audit_20260925_204538',['red','yellow']),
        ('empty_audit_20260925_205230',[])]:
        for n in ('first.png','last.png'):samples.append((base/folder/n,e))
    out=base/'color_probe_v1_regression';out.mkdir(exist_ok=True)
    rows=[];fixtures={}
    for i,(path,expected) in enumerate(samples):
        raw=path.read_bytes();frame=cv2.imdecode(np.frombuffer(raw,np.uint8),1)
        if frame is None:raise RuntimeError(str(path))
        fixtures[path.name]=frame
        old,_=v0.analyze(frame);new,drawing=v1.analyze(frame)
        labels=[r['label'] for r in new['regions'] if r['label']!='UNKNOWN']
        row=dict(file=str(path.relative_to(ROOT)),sha256=hashlib.sha256(raw).hexdigest(),expected=expected,
                 actual=labels,labels_match=Counter(labels)==Counter(expected),
                 v0_unknown=sum(r['label']=='UNKNOWN' for r in old['regions']),
                 v1_unknown=sum(r['label']=='UNKNOWN' for r in new['regions']))
        rows.append(row)
        ok,data=cv2.imencode('.png',drawing)
        if not ok:raise RuntimeError('Encode failed')
        data.tofile(out/f'{i:02d}-overlay.png')
        print(path.parent.name+'/'+path.name,'labels',labels,'UNKNOWN',row['v0_unknown'],'->',row['v1_unknown'])
    def labels(frame):return [r['label'] for r in v1.analyze(frame)[0]['regions'] if r['label']!='UNKNOWN']
    blue=fixtures['blue-day-e50-g30.png'];black=fixtures['black-day-e50-g30.png']
    checks={
        'blue_horizontal_flip':labels(cv2.flip(blue,1))==['blue'],
        'two_separated_objects':Counter(labels(np.hstack([blue,black])))==Counter(['blue','black']),
        'cropped_blue_rejected':not labels(blue[:,650:]),
        'empty_no_known_label':not labels(fixtures['empty-day-e50-g30.png']),
    }
    cropped=v1.analyze(blue[:,650:])[0]
    checks['cropped_rejection_explicit']=any(r['reason']=='TOUCHES_BORDER' for r in cropped['regions'])
    summary=dict(scope='19 saved development images only, NOT 599 original frames or independent acceptance.',
                 probe_v0_sha256=hashlib.sha256(Path(v0.__file__).read_bytes()).hexdigest(),
                 probe_v1_sha256=hashlib.sha256(Path(v1.__file__).read_bytes()).hexdigest(),
                 images=len(rows),known_label_matches=sum(r['labels_match'] for r in rows),
                 v0_unknown_total=sum(r['v0_unknown'] for r in rows),
                 v1_unknown_total=sum(r['v1_unknown'] for r in rows),checks=checks,results=rows)
    (out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print('SUMMARY',json.dumps({k:v for k,v in summary.items() if k!='results'}))
    if not all(r['labels_match'] for r in rows) or not all(checks.values()):raise SystemExit(1)

if __name__=='__main__':main()
