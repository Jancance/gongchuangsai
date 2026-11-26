"""Read-only field log reduction. Wheel projection is odometry, not ground truth."""
from pathlib import Path
import csv
import hashlib
import json
import re
import numpy as np

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent.parent
SIGN = np.array([1, -1, 1, -1])  # LR RR LF RF, motor to logical
MM_RPM = 77 * np.pi / 60


def profile(direction):
    text = (REPO / '3.stm32f407工程/CHASSIS/USER/app_speed_trial160.inc').read_text(encoding='gbk')
    text = text.split('static const uint16_t s_trial_rpm[][4] = {')[1].split('};')[0]
    rows = lambda s: [[int(v) for v in x.split(',')] for x in re.findall(r'\{\s*([\d\s,]+)\}', s)]
    if direction >= 2:
        return rows(text.split('#elif')[0])
    return rows(text.split('#else')[1])


def parse(path):
    segments, current, ready = [], None, False
    malformed = []
    for line in path.read_text(encoding='ascii', errors='replace').splitlines():
        if line == 'F407 FreeRTOS READY':
            ready = True
        m = re.fullmatch(r'T160_(BEGIN|END|C|W|H|I),([\d,\-]+)', line)
        if not m:
            if re.match(r'T160_(BEGIN|END|C|W|H|I),', line):
                malformed.append(line)
            continue
        kind, values = m.group(1), [int(x) for x in m.group(2).split(',')]
        expected = {'BEGIN': 7, 'END': 5, 'C': 7, 'W': 5, 'H': 5, 'I': 9}
        if len(values) != expected[kind]:
            malformed.append(line)
            continue
        if kind == 'BEGIN':
            current = {'BEGIN': values, 'C': [], 'W': [], 'H': [], 'I': [], 'ready_before_begin': ready}
            segments.append(current)
            ready = False
        elif current is not None:
            if kind == 'END':
                current['END'] = values
                current = None
            else:
                current[kind].append(values)
    return segments, malformed


def unique(rows, key):
    return [v for _, v in sorted({key(row): row for row in rows}.items())]


def reduce_run(name, segment, file):
    result = {'run_id': name, 'file': file.name, 'file_sha256': hashlib.sha256(file.read_bytes()).hexdigest(),
              'begin': segment['BEGIN'], 'end': segment.get('END'), 'ready_before_begin': segment['ready_before_begin']}
    direction = segment['BEGIN'][0]
    end = segment.get('END')
    result['normal'] = segment['BEGIN'][1:5] == [0, 9, 0, 1]
    result['parsed_counts'] = [len(segment[k]) for k in ('C', 'W', 'H', 'I')]
    result['reported_counts'] = end[:4] if end else None
    result['counts_match'] = bool(end and result['parsed_counts'] == end[:4])
    result['overflow'] = end[4] if end else None
    c, w = np.array(segment['C']), np.array(segment['W'])
    hrows = unique([r for r in segment['H'] if len(r) == 5 and r[4] & 3 == 3], lambda r: r[0])
    result['unique_heading_frames'] = len(hrows)
    if not result['normal'] or not len(c) or not len(w) or not hrows:
        return result, None, []
    h = np.array(hrows, dtype=float)
    groups = {int(g): c[c[:, 0] == g][np.argsort(c[c[:, 0] == g, 1])] for g in np.unique(c[:, 0])}
    first = min(g for g, q in groups.items() if np.any(q[:, 2] != 0))
    slot = float(groups[first][0, 3])
    # First profile is computed after preceding generation ACK, before its slot.
    # Last H <= that ACK is a reproducible proxy, not a captured target variable.
    decision = float(np.max(groups[first - 1][:, 5])) if first - 1 in groups else slot
    eligible = h[h[:, 0] <= decision]
    target = float((eligible[-1] if len(eligible) else h[0])[2] / 100)
    yaw = np.unwrap(np.deg2rad(h[:, 2] / 100 - target)) * 180 / np.pi
    yaw -= 360 * round(yaw[0] / 360)
    result.update(first_profile_generation=first, first_profile_slot_ms=slot,
                  target_yaw_proxy_deg=target, target_proxy_decision_ms=decision,
                  target_is_inferred=True, yaw_min_deg=float(yaw.min()), yaw_max_deg=float(yaw.max()),
                  yaw_last_deg=float(yaw[-1]), wz_last_dps=float(h[-1, 3] / 100),
                  generation_count=len(groups), command_peak_rpm_by_wheel=[int(np.max(abs(c[c[:, 1] == i, 2]))) for i in range(1, 5)])
    batches = []
    for ix, base in enumerate(profile(direction)):
        gen = first + ix
        if gen not in groups or len(groups[gen]) != 4:
            break
        q = groups[gen]
        trims = []
        for i in range(4):
            reverse = (direction & 1) ^ (direction >= 2 and i in (1, 2))
            motor_sign = SIGN[i] * (1 - 2 * reverse)
            mix = 1 if ((i & 1) ^ reverse) else -1
            trims.append((q[i, 2] / motor_sign - base[i]) / mix)
        limit = min(20, min(base))
        batches.append({'run_id': name, 'generation': gen, 'slot_ms': int(q[0, 3]),
                        'trim_rpm': float(np.median(trims)), 'trim_spread': float(np.ptp(trims)),
                        'limit_rpm': limit, 'at_limit': bool(limit > 0 and abs(np.median(trims)) >= limit)})
    active = [b for b in batches if b['limit_rpm'] > 0]
    result['trim_at_limit_fraction'] = sum(b['at_limit'] for b in active) / len(active) if active else None
    result['trim_peak_abs_rpm'] = max(abs(b['trim_rpm']) for b in batches)
    result['trim_max_spread'] = max(b['trim_spread'] for b in batches)
    wheels = [np.array(unique([r for r in segment['W'] if r[0] == a], lambda r: r[1]), dtype=float) for a in range(1, 5)]
    t0 = max(slot, h[0, 0], *(q[0, 1] for q in wheels))
    t1 = min(h[-1, 0], *(q[-1, 1] for q in wheels))
    t = np.arange(np.ceil(t0 / 10) * 10, np.floor(t1 / 10) * 10 + 1, 10)
    assert len(t) > 2
    rpm = np.column_stack([np.interp(t, q[:, 1], q[:, 2] / 10) * SIGN[i] for i, q in enumerate(wheels)])
    angle = np.interp(t, h[:, 0], yaw)
    f = rpm.mean(axis=1) * MM_RPM
    l = (rpm[:, 0] - rpm[:, 1] - rpm[:, 2] + rpm[:, 3]) * MM_RPM / 4
    wf = f * np.cos(np.deg2rad(angle)) - l * np.sin(np.deg2rad(angle))
    wl = f * np.sin(np.deg2rad(angle)) + l * np.cos(np.deg2rad(angle))
    integrate = lambda v: float(np.trapezoid(v, t / 1000))
    counts = np.column_stack([np.interp(t, q[:, 1], q[:, 4] * (1 - 2 * q[:, 3])) * SIGN[i] for i, q in enumerate(wheels)])
    delta = (counts[-1] - counts[0]) * 77 * np.pi / 65536
    result.update(common_window_ms=[float(t[0]), float(t[-1])], body_forward_mm=integrate(f), body_left_mm=integrate(l),
                  world_forward_mm=integrate(wf), world_left_mm=integrate(wl),
                  yaw_projection_forward_mm=integrate(-l * np.sin(np.deg2rad(angle))),
                  yaw_projection_left_mm=integrate(f * np.sin(np.deg2rad(angle))),
                  body_peak_forward_mm_s=float(max(abs(f))), body_peak_left_mm_s=float(max(abs(l))),
                  wheel_count_delta_mm=delta.tolist(), count_body_forward_mm=float(delta.mean()),
                  count_body_left_mm=float((delta[0]-delta[1]-delta[2]+delta[3])/4),
                  max_wheel_sample_gap_ms=max(float(max(np.diff(q[:, 1]))) for q in wheels))
    grid = {'run_id': name, 'time_ms': t.tolist(), 'logical_rpm': rpm.tolist(), 'relative_yaw_deg': angle.tolist()}
    return result, grid, batches


def main():
    results, grids, batches = [], [], []
    for file in sorted(ROOT.glob('*.bin')):
        if not re.fullmatch(r'(left\d+|right\d+|forward\d+|reverse\d+)\.bin', file.name):
            continue
        segments, malformed = parse(file)
        if not segments:
            results.append({'run_id': file.stem, 'file': file.name, 'status': 'no BEGIN / no telemetry', 'bytes': file.stat().st_size})
        for ix, segment in enumerate(segments):
            name = 'right01' if file.stem == 'left06' and segment['BEGIN'][0] == 3 else file.stem
            if len(segments) > 1 and file.stem != 'left06':
                name += '_segment' + str(ix + 1)
            result, grid, detail = reduce_run(name, segment, file)
            result['malformed_CWHI_lines_in_file'] = malformed
            results.append(result)
            if grid:
                grids.append(grid)
            batches.extend(detail)
    (ROOT / 'analysis_metrics.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    (ROOT / 'analysis_projection_grid.json').write_text(json.dumps(grids), encoding='utf-8')
    for name, rows in [('analysis_metrics.csv', results), ('analysis_batches.csv', batches)]:
        keys = list(dict.fromkeys(k for row in rows for k in row))
        with (ROOT / name).open('w', newline='', encoding='utf-8-sig') as stream:
            writer = csv.DictWriter(stream, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rows)
    for r in results:
        print(r['run_id'], r.get('counts_match'), r.get('normal'), r.get('yaw_min_deg'), r.get('yaw_max_deg'), r.get('world_forward_mm'), r.get('world_left_mm'))


if __name__ == '__main__':
    main()
