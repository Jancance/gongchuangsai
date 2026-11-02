"""Export captured OD1 trajectory and OP1 checkpoints. Read-only, no serial API."""
import argparse
import csv
import json
import math
from pathlib import Path


def export(source,out):
    source=Path(source).resolve();out=Path(out).resolve()
    if out.exists():raise ValueError('Choose a new output directory')
    records=[json.loads(line) for line in source.read_text(encoding='utf8').splitlines() if line.strip()]
    trajectory=[r for r in records if r.get('direction')=='RX' and r.get('frame_type')=='OD1']
    checkpoints={r['checkpoint']:r for r in records if r.get('direction')=='RX' and r.get('frame_type')=='OP1'}
    out.mkdir(parents=True)
    def csv_file(name,rows,fields):
        with (out/name).open('w',newline='',encoding='utf-8-sig') as f:
            writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    csv_file('xy_trajectory.csv',trajectory,['host_utc','tick_ms','sample_ms','x_mm','y_mm','heading_mdeg','valid','reasons','continuity_lost','age_ms'])
    rows=[]
    for index in range(13):
        row=dict(checkpoints.get(index,{'checkpoint':index,'captured':0,'valid':0}))
        if row.get('captured') and row.get('valid'):
            row['error_x_mm']=row['x_mm']-row['target_x_mm'];row['error_y_mm']=row['y_mm']-row['target_y_mm']
            row['error_heading_deg']=((row['heading_mdeg']-row['target_heading_mdeg']+180000)%360000-180000)/1000
        rows.append(row)
    csv_file('checkpoints.csv',rows,['checkpoint','captured','tick_ms','x_mm','y_mm','heading_mdeg','valid','reasons','continuity_lost','age_ms','target_x_mm','target_y_mm','target_heading_mdeg','error_x_mm','error_y_mm','error_heading_deg'])
    summary={'source':str(source),'trajectory_records':len(trajectory),'invalid_trajectory_records':sum(not r['valid'] for r in trajectory),
             'all_checkpoints_present_and_valid':all(r.get('captured') and r.get('valid') for r in rows),
             'position_compensation_enabled':False,'vehicle_return_proven':False}
    if summary['all_checkpoints_present_and_valid']:
        start,end=rows[0],rows[-1]
        summary['estimated_return_error']={'dx_mm':end['x_mm']-start['x_mm'],'dy_mm':end['y_mm']-start['y_mm'],
            'distance_mm':math.hypot(end['x_mm']-start['x_mm'],end['y_mm']-start['y_mm']),
            'heading_deg':((end['heading_mdeg']-start['heading_mdeg']+180000)%360000-180000)/1000}
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--jsonl',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();print(json.dumps(export(a.jsonl,a.out),ensure_ascii=False,indent=2))
