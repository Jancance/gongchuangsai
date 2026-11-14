"""Offline only: arrival common-time wheel progress vs saved final PE3 endpoint.

Use the matching half+observer ELF, coherent snapshot and the same run's saved
PM1/OD1/PE3 JSONL plus XD-derived records. Does not open hardware. Differences
are encoder-wheel quantities at different times, never ground stopping distance.
"""
from pathlib import Path
import argparse,hashlib,json,math
from parse_snapshot import parse

def elapsed(now,then):return (int(now)-int(then))&0xffffffff

def calculate(blob,serial_records,xd_records,ruler_mm=None):
    snapshot=parse(blob);arrival=snapshot['slots'][2]
    if not arrival['record_valid']:raise ValueError('No valid arrival snapshot; zero/invalid is not a measurement')
    session=snapshot['session'];now=arrival['now_ms']
    endpoints=[r for r in serial_records if r.get('frame_type')=='PE3' and r.get('session')==session and r.get('start_mask')==15 and r.get('end_mask')==15 and elapsed(r['tick_ms'],now)<0x80000000]
    if not endpoints:raise ValueError('No matching complete same-session PE3 after arrival')
    pe=min(endpoints,key=lambda r:elapsed(r['tick_ms'],now))
    wheel=[]
    for i in range(1,5):
        start=pe[f'start_raw{i}']*(1-2*pe[f'start_sign{i}'])
        end=pe[f'end_raw{i}']*(1-2*pe[f'end_sign{i}'])
        wheel.append((end-start)*(1 if i in (1,3) else -1)*math.pi*77/65536)
    final=sum(wheel)/4*1.02182540
    stops=[r for r in xd_records if r.get('kind')=='stop_ack' and r.get('session',session)==session and elapsed(r['tick_ms'],now)<0x80000000]
    stop=min(stops,key=lambda r:elapsed(r['tick_ms'],now)) if stops else None
    result=dict(label='wheel endpoint difference with explicit sample timing; NOT physical sliding or ground stopping distance',
      session=session,arrival=dict(control_ms=now,common_ms=arrival['common_ms'],common_age_ms=arrival['common_age_ms'],common_progress_mm=arrival['common_mean_mm'],reference_rpm=arrival['reference_rpm'],follow_state_age_ms=arrival['follow_state_age_ms']),
      first_STOP_ACK_ms=stop['tick_ms'] if stop else None,
      STOP_ACK_after_arrival_ms=elapsed(stop['tick_ms'],now) if stop else None,
      XD_session_binding='per-record session available' if stop and 'session' in stop else 'derived XD lacks per-record session; caller must bind matching raw capture/hash',
      PE3=dict(packet_ms=pe['tick_ms'],end_sample_ms=[pe[f'end_ms{i}'] for i in range(1,5)],start_sample_ms=[pe[f'start_ms{i}'] for i in range(1,5)],logical_wheel_unscaled_mm=wheel,calibrated_mean_mm=final,
        end_samples_after_arrival_ms=[elapsed(pe[f'end_ms{i}'],now) for i in range(1,5)],
        end_samples_after_STOP_ACK_ms=[elapsed(pe[f'end_ms{i}'],stop['tick_ms']) for i in range(1,5)] if stop else None),
      PE_minus_arrival_common_progress_mm=final-arrival['common_mean_mm'],
      ruler_forward_mm=ruler_mm,PE_minus_ruler_mm=final-ruler_mm if ruler_mm is not None else None,
      ruler_minus_arrival_common_progress_mm=ruler_mm-arrival['common_mean_mm'] if ruler_mm is not None else None,
      measurement_limits=['PE wheels are sampled asynchronously; arrival progress is at common_ms, not arrival now_ms',
        'STOP ACK confirms a protocol response, not mechanical stopping',
        'Encoder response, sample age, wheel-ground slip and body measurement-point rotation are not separable here',
        'Ruler input is optional external final measurement, not automatically synchronized or same body point'],
      hardware_provenance_established=False,physical_stopping_distance_established=False)
    # Preserve nearby existing channel context without mixing their coordinate frames.
    for tag in ['PM1','OD1']:
        records=[r for r in serial_records if r.get('frame_type')==tag and elapsed(now,r['tick_ms'])<0x80000000]
        result['latest_saved_'+tag+'_before_arrival']=min(records,key=lambda r:elapsed(now,r['tick_ms'])) if records else None
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('snapshot',type=Path);p.add_argument('serial_jsonl',type=Path);p.add_argument('xd_records',type=Path);p.add_argument('--ruler-mm',type=float);a=p.parse_args()
    records=[json.loads(x) for x in a.serial_jsonl.read_text('utf8').splitlines()]
    result=calculate(a.snapshot.read_bytes(),records,json.loads(a.xd_records.read_text('utf8')),a.ruler_mm)
    result['input_sha256']={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [a.snapshot,a.serial_jsonl,a.xd_records]}
    print(json.dumps(result,ensure_ascii=False,indent=2))
