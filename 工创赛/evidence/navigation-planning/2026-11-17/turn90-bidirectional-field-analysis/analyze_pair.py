from pathlib import Path
import json,collections,hashlib
R=Path(__file__).resolve().parents[5]; B=Path(__file__).resolve().parent.parent; OUT=Path(__file__).resolve().parent
wrap=lambda x:(x+180)%360-180
results=[]
for name,sign in [('turn90_90-field-retry2-keepalive3s',1),('turn90_90-field-cw1-keepalive3s',-1)]:
 E=B/name;d=json.loads((E/'capture/decoded.json').read_text('utf8'));s=json.loads((E/'result_summary.json').read_text('utf8'));rr=json.loads((E/'capture/read_receipt.json').read_text('utf8'));rows=[json.loads(v) for v in next((E/'serial').glob('*.jsonl')).read_text('utf8').splitlines()];rx=[r for r in rows if r.get('wire_tag')=='TMQ'];tx=collections.Counter(r['text'].split(',')[3] for r in rows if r.get('direction')=='TX');hw=d['hwt'];initial=hw[0]['yaw_deg'];goal=initial+sign*90; progress=[sign*wrap(r['yaw_deg']-initial) for r in hw];peak=max(hw,key=lambda r:abs(r['rate_dps']));moving=next(r for r in hw if abs(r['rate_dps'])>=1);stable=None;settled=None
 for r,p in zip(hw,progress):
  t=max(r['angle_ms'],r['rate_ms'])
  if abs(90-p)<=.8 and abs(r['rate_dps'])<=1:
   if stable is None:stable=t
   if t-stable>=700 and settled is None:settled=t
  else:stable=None
 terminal=[r for r in rx if r.get('phase')==3];tail=[r for r in terminal if r['tick_ms']>=terminal[0]['tick_ms']];tail_angles=[wrap(r['yaw_mdeg']/1000-goal) for r in tail];last=s['last'];final_progress=sign*wrap(last['yaw_mdeg']/1000-initial)
 assert tx['START']==1 and tx['ARM']==1 and tx['PLAN'+str(20 if sign>0 else 21)]==1 and tx['STOP']==0
 assert rr['accepted'] and rr['stable_double_read'] and d['header']['quality']==0 and not d['header']['truncated']
 assert all(r['healthy'] and r['valid_mask']==3 for r in hw)
 assert all(last[k]==0 for k in ['backend_active','safety_fault','rtos_fault','can_fault','link_lost','remote_rejected']) and last['stop_mask']==15 and last['backend_result']==1
 result=dict(directory=name,direction='CCW' if sign>0 else 'CW',goal_relative_deg=sign*90,initial_RAW_yaw_deg=initial,target_from_RAW_start_deg=wrap(goal),raw_last_relative_deg=sign*progress[-1],last_serial_relative_deg=sign*final_progress,final_shortfall_deg=90-final_progress,sampled_positive_overshoot_deg=max(0,max(progress)-90),nearest_RAW_shortfall_deg=90-max(progress),peak_sampled_abs_rate_deg_s=abs(peak['rate_dps']),peak_rate_source_ms=peak['rate_ms'],peak_excess_over90_percent=(abs(peak['rate_dps'])/90-1)*100,first_motion_rate_ms=moving['rate_ms'],first_sampled700ms_stable_ms=settled,motion_to_sampled_stable_s=(settled-moving['rate_ms'])/1000 if settled else None,stable_semantics='retained approx20Hz HWT snapshots with separately timestamped angle/rate; not all internal controller observations',max_angle_age_ms=max(r['angle_age_ms'] for r in hw),max_rate_age_ms=max(r['rate_age_ms'] for r in hw),max_retained_angle_gap_ms=max(b['angle_ms']-a['angle_ms'] for a,b in zip(hw,hw[1:])),max_retained_rate_gap_ms=max(b['rate_ms']-a['rate_ms'] for a,b in zip(hw,hw[1:])),max_angle_rate_time_skew_ms=max(abs(r['angle_ms']-r['rate_ms']) for r in hw),terminal_tail_duration_s=(tail[-1]['tick_ms']-tail[0]['tick_ms'])/1000,terminal_tail_range_deg=max(tail_angles)-min(tail_angles),TX_counts=dict(tx),valid_serial_frames=s['valid_frames'],bad_serial_frames=s['invalid_frames'],telemetry_dropped_end=last['telemetry_dropped'],RAW_samples=d['header']['raw_count'],HWT_snapshots=len(hw),source_drops={k:d['header'][k] for k in ['wheel_dropped','hwt_dropped','od_dropped']},capture_quality=d['header']['quality'],software_success=True,vm_request_last=last['vm_request'],pe2_last=last['pe2_active'],ground90_verified=False,physical_standstill_independently_observed=False,rotation_center_translation_known=False)
 results.append(result)
(OUT/'metrics.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),'utf8');print(json.dumps(results,ensure_ascii=False,indent=2))
