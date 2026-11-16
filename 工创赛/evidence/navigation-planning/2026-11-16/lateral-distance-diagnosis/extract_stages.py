from pathlib import Path
import json,math,csv,hashlib
R=Path.cwd();D=Path(__file__).parent;BASE=D.parent
measurement=dict(user_exact='前偏9.5cm左移少走3.5cm',forward_cross_track_mm=95,left_shortfall_interpretation_mm=35,left_displacement_interpretation_mm=2065,source='user report; cm converted to mm; 2100-35=2065',measurement_uncertainty_mm=None,synchronization_uncertainty_ms=None,strict_common_endpoint_error_established=False)
(BASE/'lateral-field-run2/operator_measurement.json').write_text(json.dumps(measurement,ensure_ascii=False,indent=2),encoding='utf8')
results=[];odrows=[];tmrows=[]
for run in [1,2]:
 P=BASE/f'lateral-field-run{run}';p=P/'capture/decoded.json';d=json.loads(p.read_text('utf8'));h=d['header'];trigger=h['trigger_ms']
 logs=[]
 for q in (P/'serial').glob('*.jsonl'):
  logs += [json.loads(x) for x in q.read_text('utf8').splitlines() if x.strip()]
 tm=[x for x in logs if x.get('direction')=='RX' and x.get('debug_plan_id')==9 and 'backend_state' in x]
 ep=[x for x in logs if x.get('frame_type')=='EP1'];endpoint=ep[-1] if ep else None
 states=[]
 for x in tm:
  key=(x['backend_state'],x['stop_mask'],x['distance_reached'],x['sequence_state'])
  if not states or key!=states[-1]['key']:
   states.append(dict(key=key,time_s=(x['tick_ms']-trigger)/1000,record=x))
 for x in states:tmrows.append(dict(run=run,time_s=x['time_s'],backend_state=x['key'][0],stop_mask=x['key'][1],distance_reached=x['key'][2],sequence_state=x['key'][3],progress_mm=x['record']['distance_progress_milli_mm']/1000,base_rpm=x['record']['distance_base_rpm'],rpm=[x['record'][f'rpm{i}'] for i in range(1,5)]))
 theta=math.radians(h['path_heading_deg']);od=[]
 for x in d['od']:
  dx=x['x_mm']-h['origin_x_mm'];dy=x['y_mm']-h['origin_y_mm']
  row=dict(run=run,time_s=(x['observed_ms']-trigger)/1000,source_time_s=(x['sample_ms']-trigger)/1000,F_mm=dx*math.cos(theta)+dy*math.sin(theta),L_mm=-dx*math.sin(theta)+dy*math.cos(theta),heading_delta_deg=x['heading_deg']-h['path_heading_deg'],state=x['state']);od.append(row);odrows.append(row)
 wheel=[]
 for i in range(4):
  a=[x for x in d['wheel'] if x['source']=='position' and x['valid'] and x['wheel']==i]
  a.sort(key=lambda x:x['source_ms']);wheel.append(a)
 lo=max(a[0]['source_ms'] for a in wheel);hi=min(a[-1]['source_ms'] for a in wheel)
 def value(a,ms):
  for k in range(1,len(a)):
   if a[k]['source_ms']>=ms:
    b,c=a[k-1],a[k];sg=lambda x:(-1 if x['sign'] else 1)*x['value'];return sg(b)+(sg(c)-sg(b))*(ms-b['source_ms'])/(c['source_ms']-b['source_ms'])
  return (-1 if a[-1]['sign'] else 1)*a[-1]['value']
 def project(ms):
  signs=[1,-1,1,-1];w=[(value(wheel[i],ms)-value(wheel[i],lo))*signs[i]*math.pi*77/65536 for i in range(4)]
  return dict(source_time_s=(ms-trigger)/1000,F_unscaled_mm=sum(w)/4,L_mm=(w[0]-w[1]-w[2]+w[3])/4,wheel_logical_mm=w)
 stopms=h['stop_ms'];supported_stop=min(hi,max(lo,stopms));last=project(hi);stop=project(supported_stop)
 parts=[]
 for t in [0,1,3,5,7,(stopms-trigger)/1000,(h['first_terminal_receiver_ms']-trigger)/1000,(hi-trigger)/1000]:
  ms=trigger+t*1000
  if lo<=ms<=hi:parts.append(project(ms))
 stage_od=[min(od,key=lambda x:abs(x['time_s']-t)) for t in [3,5,7,9.5]]
 yaw=[x['yaw_deg'] for x in d['hwt']];res=dict(run=run,raw_count=len(d['raw']),raw_input_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),header=h,telemetry_state_changes=states,od_stages=stage_od,encoder_common_start_s=(lo-trigger)/1000,encoder_common_end_s=(hi-trigger)/1000,encoder_stages=parts,encoder_stop_to_last=dict(F_unscaled_mm=last['F_unscaled_mm']-stop['F_unscaled_mm'],L_mm=last['L_mm']-stop['L_mm']),yaw_span_deg=max(yaw)-min(yaw),endpoint=endpoint)
 results.append(res)
(D/'stage_summary.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf8')
for name,rows in [('od_full_rate_snapshots.csv',odrows),('telemetry_state_changes.csv',tmrows)]:
 with (D/name).open('w',newline='',encoding='utf8') as f:w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
for r in results:print(json.dumps({k:r[k] for k in ['run','raw_count','encoder_common_start_s','encoder_common_end_s','encoder_stop_to_last','yaw_span_deg','od_stages']},ensure_ascii=False))
