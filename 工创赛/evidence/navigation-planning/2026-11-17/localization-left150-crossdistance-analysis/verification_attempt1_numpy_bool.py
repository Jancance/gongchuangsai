from pathlib import Path
import json,hashlib,collections,binascii
import numpy as np
R=Path.cwd();B=R/'工创赛/evidence/navigation-planning/2026-11-17';OUT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
P=json.loads((B/'four-direction-short-analysis/parameters.json').read_text('utf8'));original=B/'localization-left210-field-run3/fresh_two_run_residual_candidate_frozen.json';expected=sha(original);results=[];maxdiff=0
for n in [1,2,3]:
 E=B/f'localization-left150-crossdistance-run{n}';b=json.loads((E/'execution_binding.json').read_text('utf8'));m=json.loads((E/'holdout_prediction.json').read_text('utf8'));g=json.loads((E/'operator_measurement.json').read_text('utf8'));rr=json.loads((E/'capture/read_receipt.json').read_text('utf8'));d=json.loads((E/'capture/decoded.json').read_text('utf8'));h=d['header'];assert sha(E/'frozen_left210_coordinate_candidate.json')==expected
 assert not b['root_release'] and not b['execution_permitted'] and b['START_sent']==1
 assert rr['accepted'] and rr['stable_double_read'] and h['quality']==0 and not h['truncated'] and h['terminal_tail_completed'];assert all(h[k]==0 for k in ['wheel_dropped','hwt_dropped','od_dropped']);assert len(d['raw'])==h['raw_count'];assert all(y['sample_sequence']-x['sample_sequence']==1 for x,y in zip(d['raw'],d['raw'][1:]));assert all(y['edge_sequence']-x['edge_sequence']==1 for x,y in zip(d['raw'],d['raw'][1:]))
 for bank in ['bank_a','bank_b','tail']:
  x=(E/f'capture/{bank}_read1.bin').read_bytes();y=(E/f'capture/{bank}_read2.bin').read_bytes();assert x==y
  if bank!='tail':assert hashlib.sha256(x).hexdigest()==rr['raw_sha256'][bank]
 rows=[json.loads(v) for v in next((E/'serial').glob('*.jsonl')).read_text('utf8').splitlines()];tx=collections.Counter(x['text'].split(',')[3] for x in rows if x.get('direction')=='TX');assert tx['PLAN18']==tx['ARM']==tx['START']==1 and tx['STOP']==0
 rx=[x for x in rows if x.get('wire_tag')=='TMQ'];last=rx[-1];assert last['phase']==3 and last['backend_state']==9 and last['backend_result']==1 and last['backend_active']==0 and last['stop_mask']==15;assert all(max(x[k] for x in rx)==0 for k in ['safety_fault','rtos_fault','can_fault','link_lost','remote_rejected'])
 ts=[];xs=[];lo=0;hi=(d['raw'][-1]['service_ms']-h['trigger_ms'])/1000
 for j in range(4):
  a=[x for x in d['wheel'] if x['source']=='position' and x['valid'] and x['wheel']==j];by={x['source_ms']:x for x in a};a=[by[t] for t in sorted(by)];t=np.array([(x['source_ms']-h['trigger_ms'])/1000 for x in a]);z=np.array([x['value']*(1-2*x['sign'])*P['motor_forward_sign'][j]*np.pi*P['wheel_diameter_mm']/P['counts_per_rev'] for x in a]);ts.append(t);xs.append(z);lo=max(lo,t[0]);hi=min(hi,t[-1])
 hw=[x for x in d['hwt'] if x['healthy'] and x['valid_mask']&1];by={x['angle_ms']:x for x in hw};hw=[by[t] for t in sorted(by)];ht=np.array([(x['angle_ms']-h['trigger_ms'])/1000 for x in hw]);hy=np.unwrap(np.deg2rad([x['yaw_deg'] for x in hw]));lo=max(lo,ht[0]);hi=min(hi,ht[-1]);assert lo<=0
 q=np.arange(int(np.floor((hi+1e-10)/.005))+1)*.005;pos=np.column_stack([np.interp(q,t,z) for t,z in zip(ts,xs)]);yaw=np.interp(q,ht,hy);dw=np.diff(pos,axis=0);F=dw.mean(axis=1)*P['forward_scale'];L=(dw[:,0]-dw[:,1]-dw[:,2]+dw[:,3])/4;half=np.diff(yaw)/2;mid=(yaw[1:]+yaw[:-1])/2-np.deg2rad(h['path_heading_deg']-h['heading_offset_deg']);sinc=np.sinc(half/np.pi);path=np.array([np.sum(sinc*(F*np.cos(mid)-L*np.sin(mid))),np.sum(sinc*(F*np.sin(mid)+L*np.cos(mid)))]);err=max(abs(path-[m['cross_forward_mm'],m['main_left_mm']]));maxdiff=max(maxdiff,float(err));assert err<1e-8
 c=json.loads((E/'frozen_left210_coordinate_candidate.json').read_text('utf8'));corrected=np.array([path[0]-c['cross_model_minus_ground_per_main_beta']*path[1],c['main_scale_gamma']*path[1]]);assert max(abs(corrected-[m['corrected_forward_prediction_mm'],m['corrected_left_prediction_mm']]))<1e-8
 io=[];bad=0
 for l in next((E/'serial').glob('*.raw')).read_bytes().splitlines():
  if not l.startswith(b'IO1,'):continue
  try:
   p,_,crc=l.rpartition(b',');assert binascii.crc_hqx(p+b',',0xffff)==int(crc,16);v=[int(x) for x in p.split(b',')[1:]];assert len(v)==27;io.append(v)
  except Exception:bad+=1
 valid=[v for v in io if v[5]==1 and v[6]==0];assert bad==0
 v=dict(run=n,ground_left_mm=g['actual_left_mm'],ground_forward_mm=g['forward_mm'],baseline_left_mm=float(path[1]),baseline_forward_mm=float(path[0]),corrected_left_mm=float(corrected[1]),corrected_forward_mm=float(corrected[0]),baseline_main_apparent_error_mm=float(path[1]-g['actual_left_mm']),corrected_main_apparent_error_mm=float(corrected[1]-g['actual_left_mm']),baseline_cross_apparent_error_mm=float(path[0]-g['forward_mm']),corrected_cross_apparent_error_mm=float(corrected[0]-g['forward_mm']),cross_worsened=abs(corrected[0]-g['forward_mm'])>abs(path[0]-g['forward_mm']),raw_count=h['raw_count'],TX=dict(tx),common_end_s=float(q[-1]),terminal_gap_s=m['terminal_minus_common_end_s'],IO1_legal=len(io),IO1_bad=bad,IO1_valid=len(valid),IO1_final_valid=bool(io[-1][5]) if io else None,shared_wheel_not_independent_ground=True,coefficients_refit=False,ground_time_aligned=False)
 results.append(v)
summary=dict(status='PASS_SAVED_EVIDENCE_AND_NUMERIC_CROSSCHECK',review_role='root separate NumPy formula crosscheck; not completed new GPT-6.1 high agent review',frozen_candidate_sha256=expected,numpy_vs_MATLAB_max_mm=maxdiff,runs=results,metrics={key:float(np.mean([abs(v[key]) for v in results])) for key in ['baseline_main_apparent_error_mm','corrected_main_apparent_error_mm','baseline_cross_apparent_error_mm','corrected_cross_apparent_error_mm']},cross_worsened_runs=[v['run'] for v in results if v['cross_worsened']],new_measurements_used_for_training=False,new_compensation_deployed=False,hardware_access=False)
(OUT/'verification_and_scores.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),'utf8');print(json.dumps(summary,ensure_ascii=False,indent=2))
