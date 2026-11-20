"""Recompute saved S2.5 four-run data only; no hardware, tuning or simulation.

Run from any directory with Python 3. Writes only beside this script.
Raw CRC/decoder results are reused only after matching raw/journal SHA256.
"""
from pathlib import Path
import collections
import binascii
import csv
import hashlib
import json
import math
import re
import statistics

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[4]
FIELD = OUT.parent / 'baseline40-field'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path):
    return json.loads(path.read_text('utf-8'))


def read_csv(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return [{k: float(v) if v else None for k, v in row.items()}
                for row in csv.DictReader(stream)]


def wrap(angle):
    return (angle + 180) % 360 - 180


def distribution(values):
    return dict(values=values, mean=statistics.mean(values),
                mean_absolute=statistics.mean(abs(x) for x in values),
                median=statistics.median(values), minimum=min(values), maximum=max(values),
                sample_sd=statistics.stdev(values), max_absolute=max(abs(x) for x in values))


def main():
    manifest = {}

    def register(path):
        manifest[str(path.relative_to(ROOT))] = dict(sha256=sha(path), bytes=path.stat().st_size)

    source = ROOT / '3.stm32f407工程/CHASSIS/USER/chassis_kinematics.c'
    header = source.with_suffix('.h')
    params = ROOT / '3.stm32f407工程/USER/vehicle_parameters.h'
    scale = float(re.search(r'#define CHASSIS_FORWARD_DISTANCE_SCALE\s+([0-9.]+)f', header.read_text('gbk'))[1])
    diameter = float(re.search(rb'#define VEHICLE_WHEEL_DIAMETER_MM\s+([0-9.]+)f', params.read_bytes())[1])
    source_text = source.read_text('gbk')
    assert 'g_chassis_forward_direction[CHASSIS_WHEEL_COUNT] = {0U, 1U, 0U, 1U}' in source_text
    assert 'if (mode == 0U || mode == 3U) result *= CHASSIS_FORWARD_DISTANCE_SCALE;' in source_text
    for path in [source, header, params, Path(__file__).resolve(),
                 ROOT / '工创赛/docs/competition_requirements_2027.md',
                 FIELD / 'four-bounded-analysis/summary.json',
                 FIELD / 'four-bounded-analysis/analyze_four.py',
                 OUT.parent / 'bounded-lateral-enhancement/analyze_distance.py',
                 FIELD / 'bounded-run4-retry1/analyze_capture.py']:
        register(path)
    old = read_json(FIELD / 'four-bounded-analysis/summary.json')['runs']
    runs = []
    for number in range(1, 5):
        folder = FIELD / f's25-run{number}'
        paths = {name: folder / name for name in [
            'capture_check.json', 'operator_measurement.json', 'result_summary.json',
            'od_samples.csv', 'hwt_stop_window.csv', 'command_batches.csv',
            'analyze_once.py', 'authorization.json', 'xd1_records.json',
            'p1_serial_motion_powered_done.py']}
        raw, = (folder / 'serial').glob('*.raw')
        journal, = (folder / 'serial').glob('*.jsonl')
        saved_summary, = (folder / 'serial').glob('*.summary.json')
        for path in list(paths.values()) + [raw, journal, saved_summary]:
            register(path)
        check = read_json(paths['capture_check.json'])
        measurement = read_json(paths['operator_measurement.json'])
        result = read_json(paths['result_summary.json'])
        full = read_json(saved_summary)
        assert sha(raw) == check['raw_sha256'] and sha(journal) == check['journal_sha256']
        assert not check['parse_errors']
        assert result['status'] == 'software_complete' and result['powered_tail_valid']
        assert result['powered_tail_frames'] >= 190
        assert sha(source) == check['kinematics_sha256']
        rows = [json.loads(line) for line in journal.read_text('utf-8').splitlines()]
        tm = [r for r in rows if r.get('wire_tag') in ('TMQ', 'TM1')]
        fresh = [r for r in tm if r['source_valid'] == 1 and r['angle_age_ms'] < 100]
        done = [r for r in fresh if r['phase'] == 3 and r['backend_state'] == 9 and r['backend_result'] == 1]
        assert done
        tx = [r for r in rows if r.get('direction') == 'TX']
        counts = collections.Counter(r['text'].split(',')[3] for r in tx)
        assert counts['PLAN8'] == counts['ARM'] == counts['START'] == 1
        assert counts['STOP'] == 0 and check['post_complete_tx'] == 0
        assert all(r['host_utc'] <= done[0]['host_utc'] for r in tx)
        assert all(r['vm_request'] == r['pe2_active'] == 1 and r['trip'] == r['backend_active'] == 0
                   and not (r['can_fault'] or r['rtos_fault'] or r['safety_fault']) for r in done)
        od = read_csv(paths['od_samples.csv'])
        batches = read_csv(paths['command_batches.csv'])
        stop = check['first_stop_ack_ms']
        origin = check['hwt_start_deg']
        before = [r for r in fresh if r['tick_ms'] < stop][-1]
        early = [r for r in fresh if 1300 <= r['tick_ms'] - stop <= 2000]
        assert len(early) >= 5
        early_rel = statistics.median(wrap(r['yaw_mdeg'] / 1000 - origin) for r in early)
        late = [r for r in fresh if r['tick_ms'] >= stop + 2000]
        final_second = [r for r in done if r['tick_ms'] >= done[-1]['tick_ms'] - 1000]
        pre_od = [r for r in od if r['sample_ms'] < stop and r['tick_ms'] < stop]
        middle = [r for r in od if 600 <= r['along_mm'] <= 1800]
        pre_window = [r for r in pre_od if r['along_mm'] >= 600]
        middle_batches = [b for b in batches if b['preceding_common_OD_along_mm'] is not None
                          and 600 <= b['preceding_common_OD_along_mm'] <= 1800]
        assert len(middle) == check['middle_samples']
        assert max(abs(r['right_mm']) for r in middle) == check['middle_sample_peak_abs_mm']
        assert all(r['sample_ms'] < stop and r['tick_ms'] < stop for r in pre_window)
        ep, = full['endpoints']
        assert ep['start_mask'] == ep['end_mask'] == 15 and ep['translation_mode'] == 0
        endpoint_lines = [line.rstrip(b'\r\n') for line in raw.read_bytes().splitlines()
                          if line.startswith((b'PE2,', b'PE3,'))]
        endpoint_line, = endpoint_lines
        prefix, _, checksum = endpoint_line.rpartition(b',')
        assert binascii.crc_hqx(prefix + b',', 0xFFFF) == int(checksum, 16)
        endpoint_names = ['tick_ms', 'session', 'outcome', 'start_mask', 'end_mask', 'translation_mode', 'distance_mode']
        endpoint_names += [f'{stage}_{field}{i}' for stage in ['start', 'end']
                           for i in range(1, 5) for field in ['sign', 'raw', 'ms']]
        endpoint_values = [int(x) for x in prefix.split(b',')[1:]]
        assert len(endpoint_names) == len(endpoint_values)
        assert all(ep[key] == value for key, value in zip(endpoint_names, endpoint_values))
        logical = []
        for i in range(1, 5):
            first = ep[f'start_raw{i}'] * (-1 if ep[f'start_sign{i}'] else 1)
            last = ep[f'end_raw{i}'] * (-1 if ep[f'end_sign{i}'] else 1)
            logical.append((last - first) * (1 if i in (1, 3) else -1) * math.pi * diameter / 65536)
        unscaled = sum(logical) / 4
        assert abs(unscaled - ep['translation_travel_mm']) < 1e-8
        encoder = unscaled * scale
        software = result['last']['distance_progress_milli_mm'] / 1000
        assert software == [2104.655, 2101.789, 2102.141, 2102.677][number - 1]
        assert measurement['right_mm'] == [8, 10, 7.5, 10][number - 1]
        assert measurement['forward_error_mm'] == [5, 7.5, 5, 12.5][number - 1]
        first_final = [r for r in tm if r['distance_progress_milli_mm'] == result['last']['distance_progress_milli_mm']][0]
        pre_rel = wrap(before['yaw_mdeg'] / 1000 - origin)

        def hwt_record(record):
            return dict(tick_ms=record['tick_ms'], relative_STOP_ACK_ms=record['tick_ms'] - stop,
                        yaw_deg=record['yaw_mdeg'] / 1000,
                        left_from_start_deg=wrap(record['yaw_mdeg'] / 1000 - origin),
                        angle_age_ms=record['angle_age_ms'],
                        estimated_angle_sample_ms=record['tick_ms'] - record['angle_age_ms'])

        snapshots = {}
        for offset in [100, 300, 1000, 2000]:
            following = [r for r in fresh if 0 <= r['tick_ms'] - stop - offset <= 150]
            snapshots[f'first_at_or_after_{offset}ms'] = hwt_record(following[0]) if following else None
        lastbatch = batches[-1]
        pure_f = [b for b in batches if b['L_deci'] == b['C_deci'] == b['N_deci'] == 0 and b['F_deci'] > 0]

        def peak_batches(key):
            maximum = max(abs(b[key]) for b in batches)
            return [dict(first_ack_ms=b['first_ack_ms'], last_ack_ms=b['last_ack_ms'],
                         relative_STOP_ACK_ms=b['last_ack_ms'] - stop, F_rpm=b['F_rpm'],
                         L_rpm=b['L_rpm'], C_rpm=b['C_rpm'],
                         preceding_common_OD_along_mm=b['preceding_common_OD_along_mm'])
                    for b in batches if abs(b[key]) == maximum]

        r = dict(run=number, folder=folder.name,
            ruler_forward_mm=measurement['forward_mm'], ruler_forward_error_mm=measurement['forward_error_mm'],
            ruler_right_mm=measurement['right_mm'], software_arrival_progress_mm=software,
            software_progress_first_seen_tick_ms=first_final['tick_ms'],
            software_progress_first_seen_from_STOP_ACK_ms=first_final['tick_ms'] - stop,
            software_minus_ruler_mm=software - measurement['forward_mm'],
            final_encoder_from_PE_raw_calibrated_mm=encoder,
            final_encoder_minus_frozen_progress_mm=encoder - software,
            final_encoder_minus_ruler_mm=encoder - measurement['forward_mm'],
            PE_unscaled_encoder_mean_mm=unscaled, PE_logical_wheel_mm=logical,
            PE_endpoint=ep, PE_start_time_span_ms=[min(ep[f'start_ms{i}'] for i in range(1, 5)), max(ep[f'start_ms{i}'] for i in range(1, 5))],
            PE_end_time_from_firstSTOP_ms=[min(ep[f'end_ms{i}'] for i in range(1, 5)) - stop, max(ep[f'end_ms{i}'] for i in range(1, 5)) - stop],
            middle_OD_peak_mm=max(abs(o['right_mm']) for o in middle), middle_OD_point_count=len(middle),
            middle_OD_min_right_mm=min(o['right_mm'] for o in middle), middle_OD_max_right_mm=max(o['right_mm'] for o in middle),
            middle_OD_recorded_both_signs=any(o['right_mm'] < 0 for o in middle) and any(o['right_mm'] > 0 for o in middle),
            middle_OD_records=middle, preSTOP_OD_continuous_saved_window=pre_window,
            last_preSTOP_OD_right_mm=pre_od[-1]['right_mm'], last_preSTOP_OD_along_mm=pre_od[-1]['along_mm'],
            last_preSTOP_OD_gap_to_STOP_ms=stop - pre_od[-1]['sample_ms'],
            last_preSTOP_OD_packet_gap_to_STOP_ms=stop - pre_od[-1]['tick_ms'],
            last_OD_right_mm=od[-1]['right_mm'], last_OD_along_mm=od[-1]['along_mm'],
            last_OD_sample_after_STOP_ms=od[-1]['sample_ms'] - stop,
            last_OD_packet_after_STOP_ms=od[-1]['tick_ms'] - stop,
            lateral_peak_RPM=max(abs(b['L_rpm']) for b in batches), heading_peak_RPM=max(abs(b['C_rpm']) for b in batches),
            L30_cap_batches=sum(abs(b['L_deci']) == 30 for b in batches),
            C20_cap_batches=sum(abs(b['C_deci']) == 20 for b in batches),
            middle_associated_L30_cap_batches=sum(abs(b['L_deci']) == 30 for b in middle_batches),
            middle_associated_C20_cap_batches=sum(abs(b['C_deci']) == 20 for b in middle_batches),
            middle_associated_batch_count=len(middle_batches),
            lateral_peak_batches=peak_batches('L_rpm'), heading_peak_batches=peak_batches('C_rpm'),
            last_complete_batch=lastbatch, last_pure_F_batch=pure_f[-1] if pure_f else None,
            tail_last8_complete_batches=batches[-8:],
            pure_F_tail_associated_batches=sum(b['L_deci'] == b['C_deci'] == 0 for b in batches
                if b['preceding_common_OD_along_mm'] is not None and b['preceding_common_OD_along_mm'] >= 1800),
            last_complete_batch_F_RPM=lastbatch['F_rpm'], last_complete_batch_L_RPM=lastbatch['L_rpm'],
            last_complete_batch_C_RPM=lastbatch['C_rpm'],
            last_complete_batch_nominal_forward_mm_s=lastbatch['F_rpm'] * math.pi * diameter / 60,
            last_complete_batch_final_ACK_to_STOP_ms=stop - lastbatch['last_ack_ms'],
            last_telemetry_base_RPM=result['last']['base_rpm'], first_STOP_ACK_ms=stop,
            STOP_ACK_span_ms=check['stop_ack_span_ms'], HWT_start_deg=origin,
            HWT_before_STOP_left_deg=pre_rel, HWT_before_STOP_record=hwt_record(before),
            HWT_first_complete_left_deg=wrap(done[0]['yaw_mdeg'] / 1000 - origin),
            HWT_early_1p3_2s_left_deg=early_rel, HWT_change_pre_to_early_left_deg=wrap(early_rel - pre_rel),
            HWT_early_1p3_2s_samples=len(early), HWT_snapshots=snapshots,
            HWT_final_left_deg=wrap(done[-1]['yaw_mdeg'] / 1000 - origin),
            HWT_late_tail_yaw_span_after2s_deg=(max(t['yaw_mdeg'] for t in late) - min(t['yaw_mdeg'] for t in late)) / 1000,
            HWT_last1s_samples=len(final_second),
            HWT_last1s_span_deg=(max(t['yaw_mdeg'] for t in final_second) - min(t['yaw_mdeg'] for t in final_second)) / 1000,
            HWT_last1s_mean_left_deg=statistics.mean(wrap(t['yaw_mdeg'] / 1000 - origin) for t in final_second),
            xd_expected=check['xd1']['expected_records'], xd_received=check['xd1']['received_records'], xd_dropped=check['xd1']['dropped'],
            telemetry_queue_dropped=result['last']['telemetry_dropped'], complete_target_batches=len(batches),
            powered_tail_frames=result['powered_tail_frames'], powered_tail_s=result['powered_tail_last_elapsed_s'],
            post_complete_TX=check['post_complete_tx'], tx_commands=dict(counts),
            raw_crc_reused_counts=check['crc_checked_frames'],
            raw_PE_CRC_and_endpoint_fields_verified=True,
            excluded_final_incomplete_fragments=check['excluded_final_incomplete_fragments'],
            next_run_explicitly_authorized=measurement['next_run_explicitly_authorized'])
        assert r['xd_expected'] == r['xd_received'] and r['xd_dropped'] == 0
        runs.append(r)
    assert runs[-1]['next_run_explicitly_authorized'] is False
    metrics = ['ruler_right_mm', 'ruler_forward_error_mm', 'middle_OD_peak_mm',
               'software_minus_ruler_mm', 'final_encoder_minus_ruler_mm',
               'final_encoder_minus_frozen_progress_mm', 'HWT_early_1p3_2s_left_deg',
               'HWT_change_pre_to_early_left_deg']
    comparison = {label: {key: distribution([r[key] for r in group]) for key in metrics}
                  for label, group in [('S3_actual_four', old), ('S2p5_actual_four', runs)]}
    limits = [
        'Raw decoder/CRC evidence is reused from analyze_once/capture_check only after raw and journal SHA256 verification. PE CRC and every endpoint field are additionally checked directly against the raw frame. This script does not re-decode every raw frame.',
        'Ruler endpoint is operator ground measurement after stopping; unknown ruler reference offset and reset heading alignment prevent rigid causal/scale conclusions.',
        'Frozen software arrival progress is wheel distance at arrival, not final stopped wheel distance. First-seen telemetry time is not exact internal latch time.',
        'PE is signed raw count difference averaged with logical wheel directions, multiplied by pi*77/65536 and the existing 1.02182540 forward scale; double-precision reconstruction is not bitwise firmware float replay.',
        'PE four-wheel start/end samples are asynchronous. PE end is after STOP ACK and includes sample lag and braking evolution; PE-minus-progress is not pure ground coasting.',
        'OD is sparse integer wheel odometry, not independent ground motion. No negative saved sample does not exclude real intersample crossing or overshoot.',
        'preSTOP OD window ends with sample and packet before first STOP ACK; missing final subsecond is not silently interpolated.',
        'Complete ACK batches are commanded wheel targets, not simultaneous wheel/body measured speed. Middle association uses most recent common sample retrospectively and is descriptive, not online freshness proof.',
        'STOP markers are first/last motor acknowledgement timestamps, not physical stationary instants. HWT fresh telemetry uses live yaw, not frozen controller error.',
        'Early HWT 1.3..2s is primary endpoint comparison; later tail and final 1s are separate, so late handling cannot improve early stop yaw.',
        'For a front ruler point d mm forward from the pose reference, right-coordinate rotational contribution is -d*sin(delta_left) if initial axes coincide and lateral lever arm is zero. Unknown d/axes/reference mean no numeric correction is applied; general rigid transform requires both lever-arm components.',
        'Four runs per group are descriptive, not randomized paired trials; small differences cannot identify a unique cause or justify fitting a new distance coefficient.',
        'No new firmware, MATLAB model, download, reset, hardware access, network or further run is performed/authorized by this analysis.'
    ]
    summary = dict(schema='s25-four-saved-data-v1', date='2026-11-15', runs=runs,
                   previous_S3_runs=old, comparison=comparison,
                   calibration=dict(wheel_diameter_mm=diameter, counts_per_revolution=65536,
                                    forward_scale=scale, changed=False),
                   requirements_checked=dict(R=['FIELD-06', 'RUN-01', 'PWR-01'],
                                             E=['LOC-02', 'LOC-07', 'MOT-02', 'MOT-04', 'RUN-06'],
                                             Q='Unchanged; official ambiguities remain pending technical clarification.'),
                   scope='Saved data arithmetic and aggregation only. No product/control recommendation.', limits=limits)
    scalar_fields = [key for key, value in runs[0].items() if not isinstance(value, (list, dict))]
    with (OUT / 'four_trials.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=scalar_fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(runs)
    (OUT / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), 'utf-8')
    (OUT / 'input_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), 'utf-8')
    report = ['# S2.5 四趟数据汇总（仅数据口径）', '',
              '与已实测 S3 四趟比较；不是随机配对试验，不作方案结论。单位 mm；尺量右偏为正，HWT 左转为正。', '',
              '|趟|尺量前进|尺量超距|尺量右偏|冻结到位进度|进度−尺量|最终PE现系数换算|PE−尺量|PE−冻结|中段OD峰/点数|',
              '|---|---|---|---|---|---|---|---|---|---|']
    for r in runs:
        report.append(f"|{r['run']}|{r['ruler_forward_mm']:.1f}|{r['ruler_forward_error_mm']:.1f}|{r['ruler_right_mm']:.1f}|{r['software_arrival_progress_mm']:.3f}|{r['software_minus_ruler_mm']:.3f}|{r['final_encoder_from_PE_raw_calibrated_mm']:.3f}|{r['final_encoder_minus_ruler_mm']:.3f}|{r['final_encoder_minus_frozen_progress_mm']:.3f}|{r['middle_OD_peak_mm']:.0f}/{r['middle_OD_point_count']}|")
    report += ['', '|口径|S3 已实测四趟|S2.5 本轮四趟|', '|---|---|---|']
    for key in metrics:
        a, b = comparison['S3_actual_four'][key], comparison['S2p5_actual_four'][key]
        report.append(f"|{key}|{a['values']}；均值 {a['mean']:.4f}；MAE {a['mean_absolute']:.4f}|{b['values']}；均值 {b['mean']:.4f}；MAE {b['mean_absolute']:.4f}|")
    report += ['', '## 时基、采样与动作核验', '']
    for r in runs:
        report += [f"- 第{r['run']}趟：首STOP ACK={r['first_STOP_ACK_ms']} ms，四轮ACK跨度{r['STOP_ACK_span_ms']} ms；PE末样本相对首ACK={r['PE_end_time_from_firstSTOP_ms']} ms。末个有效preSTOP OD：along={r['last_preSTOP_OD_along_mm']:.0f}、right={r['last_preSTOP_OD_right_mm']:.0f}，样本早于ACK {r['last_preSTOP_OD_gap_to_STOP_ms']:.0f} ms。最终有效OD {r['last_OD_along_mm']:.0f}/{r['last_OD_right_mm']:.0f}的样本在ACK后 {r['last_OD_sample_after_STOP_ms']:.0f} ms。",
                   f"  - 最后完整批F/L/C={r['last_complete_batch_F_RPM']}/{r['last_complete_batch_L_RPM']}/{r['last_complete_batch_C_RPM']} RPM，末ACK距STOP {r['last_complete_batch_final_ACK_to_STOP_ms']:.0f} ms；L/C峰={r['lateral_peak_RPM']}/{r['heading_peak_RPM']} RPM，L30/C20 cap批={r['L30_cap_batches']}/{r['C20_cap_batches']}（按中段OD回溯关联的cap={r['middle_associated_L30_cap_batches']}/{r['middle_associated_C20_cap_batches']}）。F是解混合后的纯前进分量；尾段L=C=0的实际批数={r['pure_F_tail_associated_batches']}，不能把解混合F等同实际四轮纯前进批。最后8批与峰值批逐时刻在summary.json。",
                   f"  - HWT起始绝对yaw={r['HWT_start_deg']:.3f}°；停前相对左转={r['HWT_before_STOP_left_deg']:.3f}°；停后1.3..2s中位左转={r['HWT_early_1p3_2s_left_deg']:.3f}°，相对停前又变化{r['HWT_change_pre_to_early_left_deg']:.3f}°。最后1s左转均值={r['HWT_last1s_mean_left_deg']:.3f}°、跨度={r['HWT_last1s_span_deg']:.3f}°；不替代早期窗口。+.1/.3/1/2s最近后继新鲜样本见summary.json。",
                   f"  - XD={r['xd_received']}/{r['xd_expected']}，XD drop={r['xd_dropped']}；TM队列累计drop={r['telemetry_queue_dropped']}（不同通道）；尾窗{r['powered_tail_frames']}帧、最后有效elapsed={r['powered_tail_s']:.3f}s；完成后TX={r['post_complete_TX']}。排除的末尾不完整片段={r['excluded_final_incomplete_fragments']}。",
                   f"  - 中段OD右偏范围[{r['middle_OD_min_right_mm']:.0f},{r['middle_OD_max_right_mm']:.0f}]，保存点两侧越零={r['middle_OD_recorded_both_signs']}；这只描述已采样点，不排除采样间真实越零。preSTOP连续保存窗(along,right)={[(int(o['along_mm']), int(o['right_mm'])) for o in r['preSTOP_OD_continuous_saved_window']]}；窗尾以前的记录完整保留，缺失停车前亚秒段不插值。"]
    report += ['', '## 边界与未闭环', ''] + ['- ' + item for item in limits]
    report += ['', 'R已核 FIELD-06/RUN-01/PWR-01；E已核 LOC-02/LOC-07/MOT-02/MOT-04/RUN-06。R/Q不修改；正式答疑未明项继续待技术答疑确认。',
               '复算：`python 工创赛/evidence/navigation-planning/2026-11-15/s25-four-analysis/analyze_four.py`。输入SHA256/大小见input_manifest.json；脚本写入仅限本目录。']
    (OUT / 'report_data.md').write_text('\n'.join(report) + '\n', 'utf-8')
    print(json.dumps(dict(output=str(OUT), runs=len(runs), inputs=len(manifest), comparison=comparison), ensure_ascii=False))


if __name__ == '__main__':
    main()
