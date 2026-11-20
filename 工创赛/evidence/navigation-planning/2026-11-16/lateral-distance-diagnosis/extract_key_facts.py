from pathlib import Path
import json,math,hashlib
E=Path(__file__).resolve().parent
DAY=E.parent
out={}
for name in ['lateral-field-run1','lateral-field-run2']:
 p=DAY/name;s=json.loads((p/'result_summary.json').read_text('utf8'));d=json.loads((p/'capture/decoded.json').read_text('utf8'));h=d['header'];end=s['endpoints'][0]
 rows=[json.loads(x) for x in next((p/'serial').glob('*.jsonl')).read_text('utf8').splitlines()]
 tm=[x for x in rows if 'backend_state' in x and 'distance_progress_milli_mm' in x]
 stages={}
 for state in [5,6,7,8,9]:
  q=[x for x in tm if x['backend_state']==state]
  if q:
   x=q[0];stages[str(state)]={k:x.get(k) for k in ['tick_ms','backend_state','distance_progress_milli_mm','distance_base_rpm','rpm1','rpm2','rpm3','rpm4','yaw_mdeg','target_mdeg']}
  else:stages[str(state)]=None
 last=s['last']; wheels=end['logical_wheel_mm'];ff=sum(wheels)/4;ll=(wheels[0]-wheels[1]-wheels[2]+wheels[3])/4
 out[name]={'trigger_ms':h['trigger_ms'],'stop_state_observed_ms':h['stop_ms'],'terminal_ms':h['first_terminal_receiver_ms'],'stop_to_terminal_ms':h['first_terminal_receiver_ms']-h['stop_ms'],'stop_decision_progress_mm':last['distance_progress_milli_mm']/1000,'later_endpoint_projection_mm':ll,'endpoint_minus_latched_progress_mm':ll-last['distance_progress_milli_mm']/1000,'endpoint_body_forward_mm':ff,'endpoint_times_ms':[end['end_ms'+str(i)] for i in range(1,5)],'peak_heading_error_deg':last['peak_error_mdeg']/1000,'final_heading_error_deg':last['error_mdeg']/1000,'first_observed_TM_by_state':stages,'endpoint_interpretation':'Later asynchronously sampled wheel projection; difference includes feedback age and post-decision motion, NOT isolated ground stopping distance','inputs':{str(q.relative_to(DAY)):hashlib.sha256(q.read_bytes()).hexdigest() for q in [p/'result_summary.json',p/'capture/decoded.json']}}
(E/'key_facts.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(out,ensure_ascii=True,indent=2))