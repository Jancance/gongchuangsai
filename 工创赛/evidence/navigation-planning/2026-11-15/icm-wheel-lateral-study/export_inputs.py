"""Decode saved captures only. No hardware, dynamics simulation, or tuning."""
from pathlib import Path
import hashlib
import importlib.util
import json
import statistics
import sys

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[4]
FIELD = OUT.parent / 'baseline40-field'
sys.path.insert(0, str(ROOT / '工创赛/scripts'))
from vehicle_uart_capture import parse_baseline40


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def unique(rows):
    # Repeated snapshots are not extra observations; last at a timestamp wins.
    return [v for _, v in sorted({row[0]: row for row in rows}.items())]


def main():
    runs, manifest, audits = [], {}, []
    for number in [5, 6, 7]:
        folder = FIELD / f'half-stop-run{number}'
        raw = next((folder / 'serial').glob('*.raw'))
        journal = next((folder / 'serial').glob('*.jsonl'))
        rows = [json.loads(line) for line in journal.read_text('utf8').splitlines()]
        icm, crc_errors = [], []
        for line in raw.read_bytes().splitlines(keepends=True):
            if line.startswith(b'IR1,'):
                try:
                    item = parse_baseline40(line)
                    assert item['whoami'] == 233 and item['accel_config'] == 56
                    assert item['gyro_config'] == 40 and item['fresh'] == 1
                    assert item['last_error'] == item['transfer_errors'] == 0
                    icm.append(item)
                except (ValueError, AssertionError) as error:
                    crc_errors.append(str(error))
        assert icm and not crc_errors, crc_errors
        snap = json.loads((folder / 'tail_parsed.json').read_text('utf8'))
        receipt = json.loads((folder / 'tail_read_receipt.json').read_text('utf8'))
        assert snap['trusted_acquisition']['accepted'] and receipt['accepted']
        t0 = snap['raw_parser_output']['slots'][0]['run_tick_ms']
        end = next(x for x in rows if x.get('frame_type') == 'PE3')
        te = min(end[f'end_ms{i}'] for i in range(1, 5))
        wheels = []
        for i in range(1, 5):
            values = [[x[f'sample_ms{i}'], x[f'raw{i}'] * (1 - 2*x[f'sign{i}']), x['tick_ms']]
                      for x in rows if x.get('frame_type') == 'PM1'
                      and x['session'] == 1 and x['valid_mask'] & (1 << (i-1))]
            for kind in ['start', 'end']:
                values.append([end[f'{kind}_ms{i}'], end[f'{kind}_raw{i}'] *
                               (1-2*end[f'{kind}_sign{i}']), end['tick_ms']])
            wheels.append(unique(values))
        tm = [x for x in rows if 'seq' in x and x.get('direction') == 'RX'
              and x.get('source_valid') == 1]
        heading = unique([[x['tick_ms']-x['angle_age_ms'], x['yaw_mdeg']/1000]
                          for x in tm if x['angle_age_ms'] < 100])
        rate = unique([[x['tick_ms']-x['rate_age_ms'], x['rate_mdps']/1000]
                       for x in tm if x['rate_age_ms'] < 100])
        reference = json.loads((folder/'preflight_verified.json').read_text('utf8'))['last']['yaw_mdeg']/1000
        gaps = [b['sample_tick_ms']-a['sample_tick_ms'] for a,b in zip(icm,icm[1:])]
        audit = dict(run=number,IR1_records=len(icm),crc_errors=crc_errors,
            sample_dt_min_ms=min(gaps),sample_dt_median_ms=statistics.median(gaps),
            sample_dt_max_ms=max(gaps),age_max_ms=max(x['age_ms'] for x in icm),
            sequence_steps=sorted(set(b['sample_sequence']-a['sample_sequence'] for a,b in zip(icm,icm[1:]))),
            max_skipped_edges=max(x['skipped_edges'] for x in icm),
            max_read_races=max(x['read_races'] for x in icm),
            configured_sensor_Hz=200,export_nominal_Hz=5,t0_ms=t0,te_ms=te,
            pre_bias_samples=sum(t0-2200 <= x['sample_tick_ms'] <= t0-200 for x in icm))
        audits.append(audit)
        runs.append(dict(name=f'run{number}',number=number,t0_ms=t0,te_ms=te,
            reference_left_deg=reference,ir=icm,wheel1=wheels[0],wheel2=wheels[1],
            wheel3=wheels[2],wheel4=wheels[3],heading=heading,rate=rate,
            ruler=json.loads((folder/'operator_measurement.json').read_text('utf8')),
            od=[x for x in rows if x.get('frame_type')=='OD1' and x['valid'] and x['reasons']==0]))
        for p in [raw,journal,folder/'tail_parsed.json',folder/'tail_read_receipt.json',
                  folder/'operator_measurement.json',folder/'authorization.json']:
            manifest[str(p.relative_to(ROOT))] = sha(p)
    product_paths = ['CHASSIS/USER/chassis_odometry.c','CHASSIS/USER/chassis_odometry.h',
        'CHASSIS/USER/chassis_kinematics.c','CHASSIS/USER/chassis_kinematics.h',
        'CHASSIS/USER/app_serial_runtime.c','CHASSIS/USER/app_odometry.c',
        'CHASSIS/HARDWARE/ICM45686/icm45686_raw.c','CHASSIS/USER/chassis_auto_test.c']
    for rel in product_paths:
        p = ROOT/'3.stm32f407工程'/rel
        manifest[str(p.relative_to(ROOT))] = sha(p)
    payload=dict(runs=runs,policy='offline alignment only; pre-run bias; no endpoint fitting',
                 taus_s=[0.1,0.3,1,3],g_mm_s2=9806.65,accel_lsb_g=8192,gyro_lsb_dps=32.8)
    (OUT/'inputs.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2),'utf8')
    (OUT/'input_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),'utf8')
    (OUT/'audit.json').write_text(json.dumps(audits,ensure_ascii=False,indent=2),'utf8')
    print(json.dumps(audits,ensure_ascii=True))


if __name__ == '__main__':
    main()
