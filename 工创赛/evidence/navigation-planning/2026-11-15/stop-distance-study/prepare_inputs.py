"""Freeze saved evidence only. No serial, hardware, network, or Git mutation."""
from pathlib import Path
import collections
import hashlib
import json
import subprocess

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[4]
BASE = OUT.parent
manifest = []

def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def register(p, role):
    manifest.append(dict(path=str(p.relative_to(ROOT)), sha256=digest(p), role=role))

src = OUT / 'frozen_source'
src.mkdir(exist_ok=True)
for name in ['chassis_auto_test.c', 'motion_profile.c', 'motion_profile.h', 'motion_trial20.h', 'app_config.h', 'chassis_kinematics.h', 'app_serial_runtime.c']:
    rel = '3.stm32f407工程/CHASSIS/USER/' + name
    data = subprocess.check_output(['git', 'show', '86ad3415:' + rel], cwd=ROOT)
    dest = src / name
    dest.write_bytes(data)
    register(dest, 'read-only historical git object 86ad3415; ONLY auto is S25-bound; other dependencies require freeze_build_bound.py')
assert digest(src/'chassis_auto_test.c') == '5742f22ba37b9fc211c5129d6a8dab607fdf8f376e5bc6fcc31070ec18208c1b'

summary_path = BASE/'s25-four-analysis/summary.json'
summary = json.loads(summary_path.read_text(encoding='utf-8'))
register(summary_path, 'four-run published facts cross-check')
runs = []
for n in range(1,5):
    folder = BASE/'baseline40-field'/f's25-run{n}'
    log = next((folder/'serial').glob('*.jsonl'))
    raw = next((folder/'serial').glob('*.raw'))
    xdpath = folder/'xd1_records.json'
    batchpath = folder/'command_batches.csv'
    for p in [log,raw,xdpath,batchpath,folder/'operator_measurement.json']:
        register(p,'saved field evidence, immutable input')
    records = [json.loads(line) for line in log.read_text(encoding='utf-8').splitlines()]
    xd = json.loads(xdpath.read_text(encoding='utf-8'))
    tm = [x for x in records if x.get('wire_tag')=='TMQ']
    pm = [x for x in records if x.get('frame_type')=='PM1' and x.get('session')==1]
    pe = next(x for x in records if x.get('frame_type')=='PE3')
    first_tx = next(x for x in xd if x['kind']=='target_tx')
    stop = next(x for x in xd if x['kind']=='stop_ack')
    b = summary['runs'][n-1]
    start_lo = max(pe[f'start_ms{i}'] for i in range(1,5))
    movingpm=[x for x in pm if start_lo<=x['tick_ms']<=stop['tick_ms']]
    omitted_min=0
    for a,z in zip(movingpm,movingpm[1:]):
        changed=sum(a[f'sample_ms{i}']!=z[f'sample_ms{i}'] for i in range(1,5))
        omitted_min+=max(0,z['sample_sequence']-a['sample_sequence']-changed)
    rawtags=collections.Counter(x.split(b',',1)[0].decode('ascii',errors='replace') for x in raw.read_bytes().splitlines())
    runs.append(dict(run=n,tmq=tm,pm1=pm,pe3=pe,xd1=xd,summary=b,
        run_tick_lower_ms=start_lo,run_tick_upper_ms=first_tx['tick_ms'],
        run_tick_note='source order bound: all start endpoint samples precede s_run_tick; first target_tx follows it; neither equals an observed internal tick',
        missing_position_updates_lower_bound=omitted_min,
        moving_pm_snapshots=len(movingpm),raw_tags=dict(rawtags)))

spec=dict(schema='stop-distance-study-v1',source_commit='86ad3415',scope='A diagnostics only; B1 gated off unless identifiable B0 passes',
    wheel_diameter_mm=77,forward_scale=1.02182540,distance_mm=2100,max_rpm=100,
    requested_speed_mm_s=400,requested_acceleration_mm_s2=150,requested_jerk_mm_s3=300,
    follow_cap_rpm=2,follow_kp=1,follow_omega=1,follow_min_dt_ms=10,follow_max_dt_ms=100,
    approach_floor_rpm=1,arrival_mm=2,track_mm=50,final_wait_ms=2000,maxrun_ms=12779,
    B0=dict(last_complete_F_tolerance_rpm=1,require_internal_state_identifiable=True,require_arrival_common_mean_reproduced=True),
    B1=dict(enabled=False,tail_mm=120,lookahead_s=.15,scope='speed-only smoothstep; original position/scale/follower/floor/STOP unchanged'),
    rule_ids=dict(R=['RUN-01','PWR-01','MECH-01','MECH-02'],E=['LOC-01','LOC-02','LOC-03','LOC-05','LOC-07','MOT-02','MOT-04','RUN-06'],Q=['FIELD-05','OBS-03','QR-04']),
    assumptions=['no plant fitted or ground trajectory inferred','single-precision profile arithmetic follows source; prepare integration double','first TX is a time bound, not asserted internal profile origin','reached compares calibrated common-time raw mean BEFORE common-time reference compensation and follower'])
(OUT/'frozen_spec.json').write_text(json.dumps(spec,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'matlab_inputs.json').write_text(json.dumps(dict(spec=spec,runs=runs),ensure_ascii=False),encoding='utf-8')
register(OUT/'frozen_spec.json','pre-run frozen numerical/acceptance specification')
register(OUT/'matlab_inputs.json','prepared data; MATLAB performs mathematical diagnostics')
(OUT/'input_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(dict(source_sha256=digest(src/'chassis_auto_test.c'),runs=[{k:r[k] for k in ['run','run_tick_lower_ms','run_tick_upper_ms','missing_position_updates_lower_bound','moving_pm_snapshots','raw_tags']} for r in runs]),ensure_ascii=False,indent=2))
