"""This run's saved bytes only; no connections, tuning or motion commands."""
from pathlib import Path
import collections,csv,hashlib,importlib.util,json,statistics,sys
E=Path(__file__).resolve().parent;R=E.parents[5]
S=E.parents[1]/'rollback-half-ff-20261115/stop-distance-observer'
sys.path[:0]=[str(R/'工创赛/scripts'),str(S)]
from vehicle_uart_capture import parse_baseline40
from calculate_stop_wheel_distance import calculate
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def summarize(values):
    return dict(count=len(values),minimum=min(values),median=statistics.median(values),maximum=max(values)) if values else None
def ir_audit(records):
    files=[];selected=[];selected_steps=[];boundaries=[]
    for key in dict.fromkeys((x['source'],x['file']) for x in records):
        rows=[x for x in records if (x['source'],x['file'])==key]
        segments=[[]]
        for x in rows:
            if segments[-1]:
                p=segments[-1][-1]
                if x['sample_tick_ms']<=p['sample_tick_ms'] or x['sample_sequence']<=p['sample_sequence']:
                    boundaries.append(dict(source=key[0],file=key[1],before_raw_line=p['raw_line'],after_raw_line=x['raw_line'],
                        before_tick_ms=p['sample_tick_ms'],after_tick_ms=x['sample_tick_ms'],before_sample=p['sample_sequence'],after_sample=x['sample_sequence']))
                    segments.append([])
            segments[-1].append(x)
        segment_summaries=[]
        for index,segment in enumerate(segments):
            steps=[dict(tick_delta_ms=b['sample_tick_ms']-a['sample_tick_ms'],sample_delta=b['sample_sequence']-a['sample_sequence'],edge_delta=b['edge_sequence']-a['edge_sequence']) for a,b in zip(segment,segment[1:])]
            segment_summaries.append(dict(index=index,frames=len(segment),first_raw_line=segment[0]['raw_line'],last_raw_line=segment[-1]['raw_line'],first_tick_ms=segment[0]['sample_tick_ms'],last_tick_ms=segment[-1]['sample_tick_ms'],first_sample=segment[0]['sample_sequence'],last_sample=segment[-1]['sample_sequence'],export_dt_ms=summarize([x['tick_delta_ms'] for x in steps]),sample_jumps=dict(collections.Counter(str(x['sample_delta']) for x in steps)),used_for_current_summary=index==len(segments)-1))
            if index==len(segments)-1:selected.extend(segment);selected_steps.extend(steps)
        files.append(dict(source=key[0],file=key[1],frames=len(rows),first_tick_ms=rows[0]['sample_tick_ms'],last_tick_ms=rows[-1]['sample_tick_ms'],segments=segment_summaries))
    audit=dict(files=files,nonmonotonic_boundaries=boundaries,selected_steps=selected_steps,
        reset_receipt_started_at=json.loads((E/'reset_once_receipt.json').read_text('utf8'))['started_at'],
        policy='Keep raw order. No sorting/dedup or modulo conversion at backward boundary. Main timing summary uses the last monotonic segment in each file; all preceding segments remain documented.',
        interpretation='Backwards boundary occurs within preflight raw, not the preflight/serial file join. Old buffered serial prefix across the explicit reset is plausible but not proven; this alone does not establish another MCU reset.')
    (E/'ir1_export_audit.json').write_text(json.dumps(audit,indent=2),'utf8')
    return selected,selected_steps,audit
def main():
    auth=json.loads((E/'authorization.json').read_text('utf8'))
    tail=json.loads((E/'tail_parsed.json').read_text('utf8'))
    receipt=json.loads((E/'tail_read_receipt.json').read_text('utf8'))
    cap=json.loads((E/'capture_check.json').read_text('utf8'))
    io_path=E/'io1_decoded_complete.json' if (E/'io1_decoded_complete.json').exists() else E/'io1_decoded.json'
    io=json.loads(io_path.read_text('utf8'))
    measurement=json.loads((E/'operator_measurement.json').read_text('utf8')) if (E/'operator_measurement.json').exists() else None
    assert receipt['accepted'] and receipt['candidate_sha256']==auth['candidate_sha256']
    raw=next((E/'serial').glob('*.raw'));journal=next((E/'serial').glob('*.jsonl'))
    rows=[json.loads(x) for x in journal.read_text('utf8').splitlines()]
    tm=[r for r in rows if r.get('wire_tag') in ['TMQ','TM1']]
    first_done=min(r['tick_ms'] for r in tm if r['phase']==3 and r['backend_state']==9)
    arrival=tail['raw_parser_output']['slots'][2];start=arrival['run_tick_ms']
    def phase(tick):
        if tick<start:return 'before_run'
        if tick<arrival['now_ms']:return 'movement_before_arrival'
        if tick<first_done:return 'STOP_until_first_DONE_telemetry'
        return 'normal_powered_tail'
    ir=[];ir_errors=[];incomplete=[]
    for name in ['preflight','serial']:
        for path in (E/name).glob('*.raw'):
            for raw_line,line in enumerate(path.read_bytes().splitlines(keepends=True)):
                if not line.endswith((b'\r',b'\n')):
                    incomplete.append(dict(source=name,tag=line.split(b',',1)[0].decode('ascii',errors='replace'),bytes=len(line)));continue
                if not line.startswith(b'IR1,'):continue
                try:ir.append(dict(source=name,file=path.name,raw_line=raw_line,**parse_baseline40(line)))
                except Exception as exc:ir_errors.append(repr(exc))
    grouped=collections.defaultdict(list);valid=[]
    for frame in io['frames']:
        d=frame['data'];group=phase(d['estimate_ms']);grouped[group].append(d)
        if d['valid']:
            v={key:d[key] for key in ['tick_ms','estimate_ms','publication_sequence','sample_sequence','session','t0_ms','t1_ms','wheel_age_ms','hwt_age_ms','history_samples','gaps','flags']}
            v['phase']=group;v['delta_ms']=(d['t1_ms']-d['t0_ms'])&0xffffffff
            v['print_minus_estimate_ms']=(d['tick_ms']-d['estimate_ms'])&0xffffffff
            for axis in ['f','r']:
                for kind in ['wheel','position']:
                    v[f'{kind}_{axis}_mm']=d[f'{kind}_{axis}_milli_mm']/1000
                v[f'velocity_{axis}_mm_s']=d[f'velocity_{axis}_milli_mm_s']/1000
                v[f'accel_{axis}_mm_s2']=d[f'accel_{axis}_milli_mm_s2']/1000
                v[f'aux_minus_old_wheel_{axis}_mm']=v[f'position_{axis}_mm']-v[f'wheel_{axis}_mm']
            valid.append(v)
    (E/'valid_io_records.json').write_text(json.dumps(valid,indent=2),'utf8')
    if valid:
        with (E/'valid_io_records.csv').open('w',newline='',encoding='utf8') as f:
            w=csv.DictWriter(f,fieldnames=list(valid[0]));w.writeheader();w.writerows(valid)
    stop=calculate((E/'tail_read1.bin').read_bytes(),rows,json.loads((E/'xd1_records.json').read_text('utf8')),measurement['forward_mm'] if measurement else None)
    stop['hardware_provenance_established']=True
    stop['trusted_acquisition_receipt_sha256']=sha(E/'tail_read_receipt.json')
    stop['candidate_sha256']=auth['candidate_sha256']
    stop['input_sha256']={p.name:sha(p) for p in [raw,journal,E/'tail_read1.bin',E/'xd1_records.json']}
    (E/'stop_wheel_distance.json').write_text(json.dumps(stop,ensure_ascii=False,indent=2),'utf8')
    valid_steps=[dict(estimate_delta_ms=b['estimate_ms']-a['estimate_ms'],publication_delta=(b['publication_sequence']-a['publication_sequence'])&0xffffffff,
                      sample_delta=(b['sample_sequence']-a['sample_sequence'])&0xffffffff)
                 for a,b in zip(valid,valid[1:])]
    selected_ir,ir_steps,ir_file_audit=ir_audit(ir)
    io_ticks=[f['data']['tick_ms'] for f in io['frames'] if f['source']=='serial']
    report=dict(status='COMPLETED_RUN_AND_OFFLINE_OBSERVER_ANALYSIS',candidate_sha256=auth['candidate_sha256'],operator_measurement=measurement,
        hardware_actions=dict(downloads=auth['downloads_authorized'],resets=1,PLAN8=1,ARM=1,START=1,automatic_retry=False),
        software_status='software_complete',tail_stable_double_read=True,tail_session=1,tail_mask=7,
        io1=dict(total=io['total'],valid=len(valid),decode_errors=io['errors'],
            phase_definition=dict(run_start_ms=start,arrival_event_ms=arrival['now_ms'],first_DONE_telemetry_ms=first_done),
            phases={k:dict(frames=len(v),valid=sum(x['valid'] for x in v),reasons=dict(collections.Counter(str(x['reasons']) for x in v))) for k,v in grouped.items()},
            reasons={'16':'No wheel reference before start, expected diagnostic','32':'Original wheel pose unavailable/stale; pre-run/after-DONE diagnostic, not new control fault'},
            cal_sample_counts=sorted(set(f['data']['cal_samples'] for f in io['frames'])),
            observed_gap_counters=sorted(set(f['data']['gaps'] for f in io['frames'])),
            serial_wire_intervals_ms=summarize([b-a for a,b in zip(io_ticks,io_ticks[1:])]),
            valid_publication_endpoint_deltas=valid_steps,
            frequency_limit='Low-priority <=2Hz attempts; captured IO1 contains long TX gaps. Endpoint publication deltas are consistent with20ms updates but do not prove every internal50Hz calculation or its latency.'),
        ir1=dict(frames=len(ir),frames_by_source=dict(collections.Counter(x['source'] for x in ir)),export_intervals_ms=summarize([x['tick_delta_ms'] for x in ir_steps]),exported_sample_jumps=dict(collections.Counter(str(x['sample_delta']) for x in ir_steps)),decode_errors=ir_errors,excluded_final_incomplete_fragments=incomplete,age_ms=summarize([x['age_ms'] for x in ir]),
            sample_sequence_min=min(x['sample_sequence'] for x in ir),sample_sequence_max=max(x['sample_sequence'] for x in ir),
            skipped_edges=sorted(set(x['skipped_edges'] for x in ir)),read_races=sorted(set(x['read_races'] for x in ir)),
            transfer_errors=sorted(set(x['transfer_errors'] for x in ir)),
            sample_edge_step_disagreements=sum(x['sample_delta']!=x['edge_delta'] for x in ir_steps),
            last_monotonic_segment_frames=len(selected_ir),
            file_segment_summaries=ir_file_audit['files'],
            nonmonotonic_boundaries=ir_file_audit['nonmonotonic_boundaries'],
            timing_summary_policy=ir_file_audit['policy'],
            observed_rate_Hz=summarize([1000*x['sample_delta']/x['tick_delta_ms'] for x in ir_steps if x['tick_delta_ms']>0]),
            limit='IR1 exports sparse snapshots, not all200Hz samples; counter evidence cannot reconstruct unexported acceleration/gyro trajectory.'),
        icm_stack=dict(sampled_minimum_free_words=receipt['icm_statistics']['g_icm_stack_words'],sampled_minimum_free_bytes=4*receipt['icm_statistics']['g_icm_stack_words'],allocated_bytes=1024,
            limit='One saved run watermark only; not worst-case CPU/stack guarantee.'),
        wheel_summary=dict(arrival_common_mm=stop['arrival']['common_progress_mm'],arrival_common_age_ms=stop['arrival']['common_age_ms'],
            PE3_mean_mm=stop['PE3']['calibrated_mean_mm'],PE_minus_arrival_mm=stop['PE_minus_arrival_common_progress_mm'],PE_minus_ruler_mm=stop['PE_minus_ruler_mm'],physical_stop_distance_established=False),
        control_summary={k:cap[k] for k in ['xd1','complete_batches','lateral_peak_rpm','hwt_start_deg','hwt_end_deg','hwt_delta_deg','post_complete_tx']},
        limitations=['No control change or slip correction from IO1. Ruler is external measured endpoint and remains authoritative for its stated quantity.',
          'Auxiliary position minus old wheel position mixes extrapolation time, motion and IMU error, not ground estimation accuracy.',
          'No new motion or hardware access performed by this analysis.'])
    (E/'observer_field_summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf8')
    print(json.dumps({k:report[k] for k in ['status','operator_measurement','wheel_summary','icm_stack']},ensure_ascii=False))
if __name__=='__main__':main()
