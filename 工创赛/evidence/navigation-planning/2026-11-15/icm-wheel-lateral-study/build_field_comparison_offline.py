"""Summarize completed same-image runs and keep communication failure separate."""
from pathlib import Path
import collections,csv,datetime,hashlib,json
R=Path.cwd();E=Path(__file__).resolve().parent;F=E.parent/'baseline40-field'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text('utf8'))
successful=[];manifest={}
for name in ['half-icm-run1','half-icm-run3']:
    d=F/name;s=read(d/'observer_field_summary.json');result=read(d/'result_summary.json')
    io=read(d/'io1_decoded_complete.json');valid=read(d/'valid_io_records.json')
    last=valid[-1];wheel=s['wheel_summary'];m=s['operator_measurement'];ir=s['ir1']
    row=dict(run=name,candidate_sha256=s['candidate_sha256'],ruler_right_mm=m['lateral_right_mm'],
        ruler_forward_mm=m['forward_mm'],ruler_forward_error_mm=m['forward_error_mm'],
        arrival_common_mm=wheel['arrival_common_mm'],arrival_common_age_ms=wheel['arrival_common_age_ms'],
        PE3_final_wheel_mm=wheel['PE3_mean_mm'],PE_minus_arrival_mm=wheel['PE_minus_arrival_mm'],PE_minus_ruler_mm=wheel['PE_minus_ruler_mm'],
        IO1_complete_frames=io['total'],IO1_valid_frames=io['valid'],IO1_complete_CRC_errors=len(io['errors']),
        IO1_incomplete_final_fragments=len(io['excluded_final_incomplete_fragments']),
        last_valid_IO1_estimate_ms=last['estimate_ms'],last_valid_IO1_wheel_age_ms=last['wheel_age_ms'],
        last_valid_IO1_old_wheel_F_mm=last['wheel_f_mm'],last_valid_IO1_old_wheel_R_mm=last['wheel_r_mm'],
        last_valid_IO1_aux_F_mm=last['position_f_mm'],last_valid_IO1_aux_R_mm=last['position_r_mm'],
        valid_aux_minus_old_wheel_F_min_mm=min(x['aux_minus_old_wheel_f_mm'] for x in valid),
        valid_aux_minus_old_wheel_F_max_mm=max(x['aux_minus_old_wheel_f_mm'] for x in valid),
        valid_aux_minus_old_wheel_R_min_mm=min(x['aux_minus_old_wheel_r_mm'] for x in valid),
        valid_aux_minus_old_wheel_R_max_mm=max(x['aux_minus_old_wheel_r_mm'] for x in valid),
        IR1_all_complete_frames=ir['frames'],IR1_serial_frames=ir['frames_by_source']['serial'],
        IR1_current_segment_frames=ir['last_monotonic_segment_frames'],
        IR1_current_export_interval_ms=ir['export_intervals_ms'],IR1_current_sample_jumps=ir['exported_sample_jumps'],
        IR1_preflight_backward_boundaries=ir['nonmonotonic_boundaries'],
        driver_skips=ir['skipped_edges'],driver_races=ir['read_races'],driver_transfer_errors=ir['transfer_errors'],
        IO1_observer_gaps=s['io1']['observed_gap_counters'],
        ICM_sampled_min_free_stack_bytes=s['icm_stack']['sampled_minimum_free_bytes'],
        TM_status_dropped=result['last']['telemetry_dropped'],IO1_low_priority_dropped=max(x['data']['telemetry_dropped'] for x in io['frames']),
        IO1_max_successful_wire_gap_ms=s['io1']['serial_wire_intervals_ms']['maximum'],
        HWT_start_deg=s['control_summary']['hwt_start_deg'],HWT_end_deg=s['control_summary']['hwt_end_deg'],
        HWT_delta_deg=s['control_summary']['hwt_delta_deg'],XD=s['control_summary']['xd1'],
        normal_tail_frames=result['powered_tail_frames'],normal_tail_seconds=result['powered_tail_last_elapsed_s'])
    successful.append(row)
    for pattern in ['*.json','*.bin','serial/*.raw','serial/*.jsonl','preflight/*.raw']:
        for p in d.glob(pattern):manifest[str(p.relative_to(R))]=sha(p)
assert len({x['candidate_sha256'] for x in successful})==1
failed_dir=F/'half-icm-run2';failure=read(failed_dir/'failure_read_receipt.json')
failed_result=read(failed_dir/'result_summary.json');campaign=read(failed_dir/'campaign_receipt.json')
journal=next((failed_dir/'serial').glob('*.jsonl'))
tx=[json.loads(x) for x in journal.read_text('utf8').splitlines() if json.loads(x).get('direction')=='TX']
failure_summary=dict(run='half-icm-run2',counted_for_precision=False,client_status=failed_result['status'],
    transmitted_commands=dict(collections.Counter(x['text'].split(',')[3] for x in tx)),
    last_telemetry_is_stale=True,read_only_capture_at=failure['started_at'],
    delay_after_failed_campaign_seconds=(datetime.datetime.fromisoformat(failure['started_at'])-datetime.datetime.fromisoformat(campaign['finished_at'])).total_seconds(),
    live_core_before=failure['core_before'],live_core_after=failure['core_after'],
    live_before=failure['state_before'],live_after=failure['state_after'],registers=failure['registers'],pe2=failure['pe2'],
    conclusion='Later running-core read confirms START accepted3/RX67, link_lost1/result8; task/watchdog/TX still advance, VM/PE2 now0 and backend idle/session0. Subsequent KEEP/STOP did not advance MCU RX consumer. USB/COM/USART cause remains unresolved; no automatic retry.')
(failed_dir/'failure_analysis.json').write_text(json.dumps(failure_summary,indent=2),'utf8')
for pattern in ['*.json','serial/*.raw','serial/*.jsonl']:
    for p in failed_dir.glob(pattern):manifest[str(p.relative_to(R))]=sha(p)
old=read(F/'half-stop-run7/result_summary.json')
report=dict(status='TWO_SUCCESSFUL_SAME_IMAGE_RUNS_ONE_SEPARATE_COMMUNICATION_FAILURE',
    successful=successful,failed=failure_summary,
    assessment=dict(functional_evidence='Live independent observer produced timestamped F/R estimates with200-sample startup mean ready, known driver counters0 and observed own gaps0. Both normal runs completed and new-address176B reads were coherent.',
      forward_extrapolation='Old wheel anchors are77..180ms behind these IO1 estimates; added forward displacement reaches about50..55mm during motion, consistent with latency extrapolation. This is not ground-error reduction.',
      lateral_benefit='No demonstrated correction of ground lateral slip or improved ground accuracy. Observer depends on the same wheel anchors, has only9 valid exported snapshots per run and no synchronized ground trajectory.',
      motion_accuracy='Two ruler endpoints are right5/11mm and forward+7.5/+10mm; the observer was not fed to control. Run-to-run variation does not establish improvement or degradation from observer logging.',
      frequency='Valid endpoint publication deltas are50 counts over1000/1005ms; raw sample deltas200/201 over the same span. Consistent with20ms observation/200Hz sampling, but sparse outputs cannot certify every execution, full chronology or latency.',
      log_load=dict(old_half_stop_run7_TM_drop=old['last']['telemetry_dropped'],new_runs_TM_drop=[x['TM_status_dropped'] for x in successful],new_runs_IO1_busy_drop=[x['IO1_low_priority_dropped'] for x in successful],
        statement='Status drops were7 in the referenced old run and2/8 in the two new runs; IO1 low-priority own busy drops40/40 and up to13.001/13.050s successful-output gaps. Different runs, no causal queue/CPU load conclusion.'),
      stationary_prediction='Last exported STOP-region estimates include forward velocity−27.445/−36.743mm/s. They are unvalidated model outputs, not evidence the chassis physically reversed or proof of a particular vibration/bias cause.'),
    limits=['No additional hardware access, tuning or new test authorization from this analysis.',
      'Ruler endpoints, asynchronous encoder endpoints, delayed common wheel poses and different-time IO1 estimates are different quantities; no IO1-minus-ruler endpoint accuracy is calculated.',
      'Only final monotonic segment within each raw file enters IR timing aggregates; earlier preflight segments and their backwards boundaries remain explicitly documented, not sorted away.',
      'Full-time ground trajectory, attitude calibration/lever compensation, long-term drift and worst-case timing/stack remain unclosed.'],
    evidence_manifest_sha256=None)
(E/'field_evidence_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),'utf8')
report['evidence_manifest_sha256']=sha(E/'field_evidence_manifest.json')
(E/'field_comparison_summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf8')
with (E/'field_comparison_table.csv').open('w',newline='',encoding='utf8') as f:
    keys=['run','ruler_right_mm','ruler_forward_error_mm','arrival_common_mm','arrival_common_age_ms','PE3_final_wheel_mm','PE_minus_arrival_mm','IO1_complete_frames','IO1_valid_frames','IR1_serial_frames','TM_status_dropped','IO1_low_priority_dropped','HWT_delta_deg','ICM_sampled_min_free_stack_bytes']
    w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows({k:x[k] for k in keys} for x in successful)
print(json.dumps(dict(status=report['status'],successful_runs=len(successful),failed_runs=1,evidence_manifest_sha256=report['evidence_manifest_sha256'])))
