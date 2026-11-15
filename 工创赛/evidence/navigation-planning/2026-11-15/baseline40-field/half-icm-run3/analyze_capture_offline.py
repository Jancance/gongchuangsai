# Derived from bounded-run4-retry1 analyzer; only authorization identity-key lookups adapted. Original SHA256 a30eb7063993c95d4093c93460487747c87ad870192ee2d7650d20bde5f4133b
"""Decode completed conservative FF captures only. No hardware, simulation or tuning."""
from pathlib import Path
import collections
import csv
import hashlib
import importlib.util
import json
import sys

OUT=Path(__file__).parent
ROOT=OUT.parents[5]
FIELD=OUT.parent
sys.path.insert(0,str(ROOT/'工创赛/scripts'))
from p1_x_load_telemetry import LoadAccumulator,parse_load_telemetry
from vehicle_uart_capture import parse_wheel
CLIENT=OUT/'p1_serial_motion_powered_done.py'
spec=importlib.util.spec_from_file_location('capture_parser',CLIENT)
client=importlib.util.module_from_spec(spec);spec.loader.exec_module(client)


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def wrap(a):return (a+180)%360-180


def save(folder,name,rows):
    assert rows,name
    with (folder/name).open('w',newline='',encoding='utf8') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def capture_files(folder):
    raw=list((folder/'serial').glob('*.raw'));journal=list((folder/'serial').glob('*.jsonl'))
    assert len(raw)==len(journal)==1
    return raw[0],journal[0]


def analyze(number):
    folder=OUT;raw,journal=capture_files(folder)
    rows=[json.loads(line) for line in journal.read_text('utf8').splitlines()]
    result=json.loads((folder/'result_summary.json').read_text('utf8'));auth=json.loads((folder/'authorization.json').read_text('utf8'))
    assert result['status']=='software_complete' and sha(folder/'p1_serial_motion_powered_done.py')==auth['client_sha256']
    acc=LoadAccumulator();errors=collections.Counter();decoded=collections.Counter();od_by_sample={};raw_tm={};fragments=[]
    for line in raw.read_bytes().splitlines(keepends=True):
        tag=line.split(b',',1)[0]
        if not line.endswith((b'\r',b'\n')):
            fragments.append(dict(tag=tag.decode('ascii',errors='replace'),bytes=len(line)));continue
        try:
            if tag==b'XD1':acc.add(parse_load_telemetry(line))
            elif tag==b'OD1':
                r=client.parse_odometry_telemetry(line)
                if r['valid'] and not r['reasons'] and not r['local_origin'] and r['sample_ms'] not in od_by_sample:
                    od_by_sample[r['sample_ms']]=dict(tick_ms=r['tick_ms'],sample_ms=r['sample_ms'],along_mm=2100-r['y_mm'],right_mm=2100-r['x_mm'],local_heading_left_deg=r['heading_mdeg']/1000+90,packet_age_ms=r['age_ms'])
            elif tag==b'WF1':parse_wheel(line)
            elif tag in (b'TMQ',b'TM1'):
                r=client.parse_telemetry(line);raw_tm[(r['tick_ms'],r['seq'])]=r
            else:continue
            decoded[tag.decode()]+=1
        except ValueError as ex:errors[tag.decode()+':'+str(ex)]+=1
    assert acc.complete and not errors
    records=[acc.records[i] for i in sorted(acc.records)]
    (folder/'xd1_records.json').write_text(json.dumps(records,indent=2),'utf8')
    tm=[r for r in rows if r.get('wire_tag') in ('TMQ','TM1')]
    for r in tm:assert all(r[k]==v for k,v in raw_tm[(r['tick_ms'],r['seq'])].items())
    od=sorted(od_by_sample.values(),key=lambda r:r['sample_ms']);save(folder,'od_samples.csv',od)
    batches=[];batch=[]
    for r in records:
        if r['kind']!='target_ack':continue
        assert r['result']==1
        if r['address']==1:batch=[r]
        elif batch and r['address']==len(batch)+1:batch.append(r)
        else:batch=[]
        if len(batch)!=4:continue
        q=[round(r['target_rpm']*10)*(1 if r['address'] in (1,3) else -1) for r in batch]
        F,L,C,N=sum(q)/4,(q[0]-q[1]-q[2]+q[3])/4,(-q[0]+q[1]-q[2]+q[3])/4,(q[0]+q[1]-q[2]-q[3])/4
        assert F%10==0 and N==0 and abs(L)<=30 and abs(C)<=20 and abs(L)+abs(C)<=50 and abs(L)+abs(C)<=F
        assert q==[F+L-C,F-L+C,F-L-C,F+L+C] and min(q)>=0 and max(q)<=F+50
        t=batch[-1]['tick_ms'];prior=next((o for o in reversed(od) if o['sample_ms']<=t),None)
        b=dict(first_ack_ms=batch[0]['tick_ms'],last_ack_ms=t,ack_span_ms=t-batch[0]['tick_ms'],F_deci=int(F),L_deci=int(L),C_deci=int(C),N_deci=int(N),F_rpm=F/10,L_rpm=L/10,C_rpm=C/10,
               preceding_common_OD_along_mm=prior['along_mm'] if prior else None,preceding_common_OD_age_ms=t-prior['sample_ms'] if prior else None)
        for i,v in enumerate(q,1):b['q'+str(i)+'_deci']=v
        batches.append(b);batch=[]
    save(folder,'command_batches.csv',batches)
    fresh=lambda r:r['source_valid']==1 and r['angle_age_ms']<100
    startidx=next(i for i,r in enumerate(rows) if r.get('direction')=='TX' and ',START,' in r['text'])
    start=[r for r in rows[:startidx] if r.get('wire_tag') in ('TMQ','TM1') and fresh(r) and r['phase']==0 and not r['vm_request']][-1]
    moving=[r for r in tm if fresh(r) and r['backend_active'] and r['distance_mode']==1]
    done=[r for r in tm if fresh(r) and r['phase']==3 and r['vm_request']==1 and r['pe2_active']==1]
    assert moving and done
    end=done[-1];reference=start['yaw_mdeg']/1000
    assert set(r['target_mdeg']/1000 for r in moving)=={reference}
    stops=[r for r in records if r['kind']=='stop_ack'];assert [r['address'] for r in stops]==[1,2,3,4]
    first,last=stops[0]['tick_ms'],stops[-1]['tick_ms'];before=max((r for r in tm if fresh(r) and r['tick_ms']<first),key=lambda r:r['tick_ms'])
    tail=[r for r in done if end['tick_ms']-r['tick_ms']<=1000]
    window=[dict(tick_ms=r['tick_ms'],relative_first_stop_ack_ms=r['tick_ms']-first,yaw_deg=r['yaw_mdeg']/1000,left_from_start_deg=wrap(r['yaw_mdeg']/1000-reference),angle_age_ms=r['angle_age_ms'],phase=r['phase'],backend_active=r['backend_active'],progress_mm=r['distance_progress_milli_mm']/1000) for r in tm if fresh(r) and first-1000<=r['tick_ms']<=last+1200]
    save(folder,'hwt_stop_window.csv',window)
    tx=[r for r in rows if r.get('direction')=='TX'];counts=collections.Counter(r['text'].split(',')[3] for r in tx)
    post=sum(r['host_utc']>done[0]['host_utc'] for r in tx);assert counts['PLAN8']==counts['ARM']==counts['START']==1 and post==0
    mid=[r for r in od if 600<=r['along_mm']<=1800];peak=max(r['right_mm'] for r in mid);abspeak=max(abs(r['right_mm']) for r in mid)
    final=result['last'];assert final['vm_request']==final['pe2_active']==1 and final['backend_active']==final['trip']==0
    measurement=json.loads((folder/'operator_measurement.json').read_text('utf8')) if (folder/'operator_measurement.json').exists() else None
    report=dict(run_id='2026-11-15_'+OUT.name,software_status=result['status'],candidate_sha256=auth['candidate_sha256'],source_sha256=auth['all_changed_source_sha256']['3.stm32f407工程/CHASSIS/USER/chassis_auto_test.c'],kinematics_sha256=auth['all_changed_source_sha256']['3.stm32f407工程/CHASSIS/USER/chassis_kinematics.c'],raw_sha256=sha(raw),journal_sha256=sha(journal),xd1=acc.summary(),parse_errors=dict(errors),crc_checked_frames=dict(decoded),excluded_final_incomplete_fragments=fragments,
        tx_commands=dict(counts),post_complete_tx=post,complete_batches=len(batches),exact_FLCN_remix_checks=len(batches),max_batch_ack_span_ms=max(b['ack_span_ms'] for b in batches),
        lateral_peak_rpm=max(abs(b['L_rpm']) for b in batches),lateral_cap30_complete_batches=sum(abs(b['L_deci'])==30 for b in batches),lateral_above_old24_complete_batches=sum(abs(b['L_deci'])>24 for b in batches),heading_peak_rpm=max(abs(b['C_rpm']) for b in batches),heading_cap20_complete_batches=sum(abs(b['C_deci'])==20 for b in batches),
        od=od,middle_samples=len(mid),middle_sample_peak_abs_mm=abspeak,middle_sample_min_right_mm=min(r['right_mm'] for r in mid),middle_sample_peak_right_mm=peak,middle_peak_observations=[r for r in mid if r['right_mm']==peak],
        hwt_start_deg=reference,hwt_end_deg=end['yaw_mdeg']/1000,hwt_delta_deg=wrap(end['yaw_mdeg']/1000-reference),moving_sampled_peak_right_yaw_deg=max(-wrap(r['yaw_mdeg']/1000-reference) for r in moving),
        first_stop_ack_ms=first,last_stop_ack_ms=last,stop_ack_span_ms=last-first,pre_stop_hwt_tick_ms=before['tick_ms'],pre_stop_gap_to_ack_ms=first-before['tick_ms'],pre_stop_hwt_deg=before['yaw_mdeg']/1000,pre_stop_left_from_start_deg=wrap(before['yaw_mdeg']/1000-reference),post_pre_stop_yaw_change_deg=wrap((end['yaw_mdeg']-before['yaw_mdeg'])/1000),first_complete_hwt_deg=done[0]['yaw_mdeg']/1000,
        last_1s_hwt_samples=len(tail),last_1s_hwt_span_deg=(max(r['yaw_mdeg'] for r in tail)-min(r['yaw_mdeg'] for r in tail))/1000,
        operator_measurement=measurement,software_final={k:final[k] for k in ['vm_request','pe2_active','backend_active','trip','stop_mask','can_fault','safety_fault','rtos_fault','rpm1','rpm2','rpm3','rpm4','distance_progress_milli_mm','feedback_limit_rpm']},
        limits=['Complete-batch targets are acknowledged commands, not simultaneous actual body speeds.','OD is sparse wheel-odometry, not a measured ground path; operator gave endpoints only.','Batch counts are not time fractions. Common-sample OD association is descriptive offline alignment, not an online availability claim.','Live HWT yaw used, not frozen controller error; STOP markers are ACK timestamps.','No new motion authorized after these completed captures.'])
    (folder/'capture_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf8')
    return report


def main():
    reports=[analyze(i) for i in [1,2,3,4]]
    out=dict(candidate_sha256=reports[0]['candidate_sha256'],parameters=dict(Ky=.75,FF_fraction=.5,L_limit_RPM=3,speed_cm_s=40,distance_cm=210),successful_runs=reports,no_more_start_authorization=True,hardware_accessed_by_analysis=False)
    (OUT/'four_ff_field_runs_summary.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),'utf8')
    compact=[{k:r[k] for k in ['run_id','xd1','complete_batches','lateral_peak_rpm','lateral_cap30_complete_batches','heading_peak_rpm','middle_samples','middle_sample_peak_abs_mm','middle_sample_peak_right_mm','hwt_start_deg','hwt_end_deg','hwt_delta_deg','moving_sampled_peak_right_yaw_deg','pre_stop_left_from_start_deg','post_pre_stop_yaw_change_deg','last_1s_hwt_span_deg','operator_measurement']} for r in reports]
    (OUT/'four_ff_field_runs_compact.json').write_text(json.dumps(compact,ensure_ascii=False,indent=2),'utf8')
    print(json.dumps(compact,ensure_ascii=True,indent=2))
if __name__=='__main__':
 import json
 report=analyze(1)
 print(json.dumps({k:report[k] for k in ['middle_sample_peak_abs_mm','middle_samples','lateral_peak_rpm','hwt_start_deg','hwt_end_deg','hwt_delta_deg','pre_stop_left_from_start_deg','xd1','post_complete_tx','complete_batches']}))
