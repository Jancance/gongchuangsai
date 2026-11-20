import pathlib, json, hashlib, re

OUT = pathlib.Path(__file__).resolve().parent
DAY = OUT.parent
OLD = DAY.parent / '2026-11-15/fusion-b-mahony-comparison'
RUNS = ['inertial-field-run1', 'inertial-field-run2', 'inertial-field-run3-retry1']
def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

manifest = {}
telemetry = []
for name in RUNS:
    root = DAY / name
    decoded = root / 'capture/decoded.json'
    d = json.loads(decoded.read_text('utf-8-sig'))
    trigger = d['header']['trigger_ms']
    log = next((root/'serial').glob('*.jsonl'))
    rows = [json.loads(s) for s in log.read_text('utf-8-sig').splitlines() if s.strip()]
    tm = [r for r in rows if r.get('wire_tag') == 'TMQ' and r.get('tick_ms', 0) >= trigger]
    done = next(r for r in tm if r['backend_active'] == 0 and r['backend_result'] == 1)
    zero = next(r for i,r in enumerate(tm) if all(all(abs(z.get('rpm'+str(j),1))==0 for j in range(1,5)) for z in tm[i:]))
    result = json.loads((root/'result_summary.json').read_text('utf-8-sig'))
    # These are source-time encoder observations after capture, not inertial extrapolation.
    endpoint = result['endpoints'][0]
    pm = [r for r in rows if r.get('frame_type') == 'PM1' and r.get('session') == d['header']['session'] and r.get('valid_mask') == 15]
    pos = []
    for j in range(1,5):
        vals = {}
        for w in d['wheel']:
            if w['source']=='position' and w['wheel']==j-1 and w['valid']:
                vals[w['source_ms']] = (-1 if w['sign'] else 1)*w['value']
        for r in pm:
            vals[r['sample_ms'+str(j)]] = (-1 if r['sign'+str(j)] else 1)*r['raw'+str(j)]
        vals[endpoint['end_ms'+str(j)]] = (-1 if endpoint['end_sign'+str(j)] else 1)*endpoint['end_raw'+str(j)]
        pos.append({'time_s':[(t-trigger)/1000 for t in sorted(vals)], 'counts':[vals[t] for t in sorted(vals)]})
    telemetry.append({'name': name, 'done_s': (done['tick_ms']-trigger)/1000,
        'persistent_reported_zero_rpm_s': (zero['tick_ms']-trigger)/1000,
        'done_rpm': [done['rpm'+str(j)] for j in range(1,5)],
        'positions':pos,
        'ruler_mm': {'inertial-field-run1':2100,'inertial-field-run2':2102.5}.get(name),
        'ruler_uncertainty_mm':None, 'same_window_reference_verified':False,
        'physical_standstill_verified':False})
    for file in [decoded,log,root/'result_summary.json',root/'field_execution_summary.json']:
        manifest[str(file)] = sha(file)
    measure = root/'operator_measurement.json'
    if measure.exists(): manifest[str(measure)] = sha(measure)
for file in [OLD/'compare_mahony_b.m', OLD/'frozen_plan.json']:
    manifest[str(file)] = sha(file)
(OUT/'input_sha256_before.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False),encoding='utf-8')
(OUT/'telemetry_context.json').write_text(json.dumps(telemetry,indent=2),encoding='utf-8')
source=(OLD/'compare_mahony_b.m').read_text('utf-8')
# Copy exact original numerical functions; no old synthetic scenarios are called.
parts=[]
for name in ['front','exp_rotation','rotz3','prepare_wheel','empty_events','filter_intervals','rebuild','skew','write_json']:
    pattern=re.compile(r'^function[^\n]*\b'+name+r'(?:\(|\s|$)',re.M)
    m=pattern.search(source)
    assert m,name
    nextfun=re.search(r'^function',source[m.end():],re.M)
    end=m.end()+nextfun.start() if nextfun else len(source)
    parts.append(source[m.start():end])
top=(OUT/'analysis_main.txt').read_text('utf-8')
(OUT/'replay_forward_three.m').write_text(top+'\n'+'\n'.join(parts),encoding='utf-8')
print('Prepared replay_forward_three.m',sha(OUT/'replay_forward_three.m'))
