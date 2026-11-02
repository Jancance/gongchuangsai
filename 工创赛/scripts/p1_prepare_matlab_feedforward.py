"""Export CRC-checked saved vehicle logs for MATLAB. No hardware imports."""
import argparse
import bisect
import csv
import hashlib
import json
from pathlib import Path
import statistics

from p1_x_load_telemetry import LoadAccumulator, parse_load_telemetry, FORWARD_SIGN
from p1_longitudinal_feedforward_sim import ROOT, hashes


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    out=a.out.resolve()
    if not out.is_relative_to(ROOT):raise ValueError('Output must be in repository')
    out.mkdir(parents=True,exist_ok=False)
    base=ROOT/'工创赛/evidence/p1-foundation/2026-11-01/x_load_diag'
    commands=[];samples=[];live=[];runs=[];source_hashes={}
    for run,(folder,review_name) in enumerate([
        ('run_pd_once','vehicle_pd_result_review.json'),
        ('run_pd_repeat_once','vehicle_pd_repeat_result_review.json')],1):
        raw=next((base/folder).glob('*.raw'));jsonl=next((base/folder).glob('*.jsonl'))
        summary_path=next((base/folder).glob('*.summary.json'));review_path=base/review_name
        for f in [raw,jsonl,summary_path,review_path]:
            source_hashes[f.relative_to(ROOT).as_posix()]=hashlib.sha256(f.read_bytes()).hexdigest()
        acc=LoadAccumulator()
        for line in raw.read_bytes().splitlines():
            if line.startswith(b'XD1,'):acc.add(parse_load_telemetry(line))
        assert acc.complete,acc.summary()
        rows=[json.loads(line) for line in jsonl.read_text(encoding='utf8').splitlines()]
        other=LoadAccumulator()
        for r in rows:
            if r.get('frame_type')=='XD1':other.add(r)
        assert acc.records==other.records,'raw CRC decoding differs from saved JSON'
        records=[acc.records[i] for i in sorted(acc.records)]
        assert all(b['tick_ms']>=a['tick_ms'] for a,b in zip(records,records[1:]))
        cmd=[r for r in records if r['kind'] in ('target_ack','stop_ack') and r['result']==1]
        t0=min(r['tick_ms'] for r in cmd)
        for r in cmd:
            value=0 if r['kind']=='stop_ack' else r['target_rpm']
            commands.append([run,r['address'],(r['tick_ms']-t0)/1000,FORWARD_SIGN[r['address']]*value])
        per_wheel=[]
        for wheel in range(1,5):
            wc=[r for r in cmd if r['address']==wheel]
            ticks=[r['tick_ms'] for r in wc]
            valid=[];pairing_errors=0
            for r in records:
                if not (r['address']==wheel and r['kind']=='sample' and r['result']==1 and
                        r['target_valid'] and not r['speed_clipped'] and r['tick_ms']>=t0):continue
                k=bisect.bisect_right(ticks,r['tick_ms'])-1
                ack=wc[k] if k>=0 else None
                if not ack or ack['tick_ms']!=r['target_ack_ms'] or ack['target_rpm']!=r['target_rpm']:
                    pairing_errors+=1;continue
                sign=FORWARD_SIGN[wheel]
                valid.append(r)
                samples.append([run,wheel,(r['tick_ms']-t0)/1000,sign*r['actual_rpm'],
                                sign*r['target_rpm'],r['target_ack_age_ms'],r['phase_current_ma'],
                                sign*r['position_deg'],r['backend_state']])
            assert pairing_errors==0,(run,wheel,pairing_errors)
            gaps=[b['tick_ms']-a['tick_ms'] for a,b in zip(valid,valid[1:])]
            per_wheel.append(dict(wheel=wheel,samples=len(valid),median_gap_ms=statistics.median(gaps),
                                  max_gap_ms=max(gaps),ack_pairing_errors=pairing_errors))
        tm=[r for r in rows if r.get('telemetry_schema')=='TM7' and r.get('source_valid') and
            r.get('distance_mode')==1 and t0<=r['tick_ms']<=records[-1]['tick_ms']]
        assert all(b['tick_ms']>a['tick_ms'] for a,b in zip(tm,tm[1:]))
        for r in tm:
            live.append([run,(r['tick_ms']-t0)/1000,r['yaw_mdeg']/1000,r['target_mdeg']/1000,
                         r['rate_mdps']/1000,r['correction_rpm'],r['distance_base_rpm'],
                         r['distance_progress_milli_mm']/1000,r['backend_state'],r['backend_active']])
        summary=json.loads(summary_path.read_text(encoding='utf8'))
        review=json.loads(review_path.read_text(encoding='utf8'))
        assert summary['status']=='software_complete' and summary['invalid_frames']==0
        runs.append(dict(run=run,source=folder,history=acc.summary(),wheels=per_wheel,
                         tm_rows=len(tm),duration_s=(records[-1]['tick_ms']-t0)/1000,
                         physical_measurement=review['physical_measurement'],
                         physical_stop_confirmed=False,direction='forward',
                         control=dict(P=1,D=.25,deadband_deg=.3,integral=False)))
    for filename,header,data in [
        ('commands.csv',['run','wheel','t_s','target_logical_rpm'],commands),
        ('samples.csv',['run','wheel','t_s','actual_logical_rpm','target_logical_rpm','ack_age_ms','current_ma','position_logical_deg','backend_state'],samples),
        ('telemetry.csv',['run','t_s','yaw_deg','target_deg','rate_dps','correction_rpm','base_rpm','progress_mm','backend_state','active'],live)]:
        with (out/filename).open('w',newline='',encoding='utf8') as f:
            w=csv.writer(f);w.writerow(header);w.writerows(data)
    result=dict(hardware_accessed=False,runs=runs,source_sha256=source_hashes,
                backward_measured=False,ground_trajectory_measured=False,
                limits=['Only two forward runs; backward parameters must remain unvalidated extrapolation.',
                        'Per-wheel ~0.5s sampling cannot resolve fast inner-loop dynamics or separate short delay from tau.',
                        'Two endpoint ruler measurements do not identify lateral slip dynamics or true ground trajectories.'])
    (out/'data_review.json').write_text(json.dumps(result,ensure_ascii=True,indent=2),encoding='ascii')
    metadata={k:v for k,v in result.items() if k!='source_sha256'}
    metadata['source_hash_manifest']='data_review.json'
    (out/'matlab_metadata.json').write_text(json.dumps(metadata,ensure_ascii=True,indent=2),encoding='ascii')
    (out/'live_before.json').write_text(json.dumps(hashes(ROOT/'3.stm32f407工程'),indent=2),encoding='utf8')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
