"""Read-only current deployed60 session summaries; default performs no SSH."""
from pathlib import Path
import argparse,base64,zlib,json,datetime
from stage23_pi_install_20261116 import remote
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-18/pickup60-ui'
SCRIPT=r'''
from pathlib import Path
from collections import Counter,deque
import json,os
bound=BINDING
pid=bound['pid'];session=Path(bound['session']).resolve();root=Path(bound['root']).resolve()
a=[x.decode() for x in Path('/proc/'+str(pid)+'/cmdline').read_bytes().split(b'\0') if x]
assert any(Path(x).name=='source_fast_cycle_view_20261115.py' for x in a)
v=Path(a[a.index('--session-dir')+1]);cwd=Path(os.readlink('/proc/'+str(pid)+'/cwd'))
if not v.is_absolute():v=cwd/v
assert v.resolve()==session and root in session.parents,'Deployed process/session changed; do not inspect a substituted session'
p=session/'observations.jsonl';size=p.stat().st_size;start=max(0,size-32*1024*1024);items={};states=Counter();parsed=partial=0;last={};recent=deque(maxlen=18)
with p.open('rb') as f:
 f.seek(start)
 if start:f.readline()
 while f.tell()<size:
  line=f.readline(size-f.tell())
  if not line:break
  try:x=json.loads(line)
  except (ValueError,UnicodeError):partial+=1;continue
  parsed+=1;s=x.get('mcu_status') or {};states[str(s.get('state'))]+=1;item=s.get('item');run=s.get('run')
  if item not in (0,1,2):continue
  key=str(run)+':'+str(item);w=x.get('pickup_window') or {};rot=x.get('initial_rotation') or {};stop=x.get('stop_confirmation') or {};pred=x.get('direct_frame_trigger')
  row=items.setdefault(key,dict(run=run,item=item,state_counts={},max_stable_frames=0,max_abs_net_deg=0,signed_net_at_max_abs_deg=0,required_deg=rot.get('required_deg'),max_stable_evidence=None,angle_evidence=None,window_reasons={},OC2=[]))
  state=str(s.get('state'));row['state_counts'][state]=row['state_counts'].get(state,0)+1
  reason=str(w.get('reason'));row['window_reasons'][reason]=row['window_reasons'].get(reason,0)+1
  t=x.get('host_read_completed_monotonic_s');stable=w.get('stable_frames',0);net=rot.get('observed_net_deg',0)
  small=dict(frame_sequence=x.get('frame_sequence'),host_monotonic_s=t,state=s.get('state'),pose=s.get('pose'),stable_frames=stable,required_frames=stop.get('required_frames'),net_signed_deg=net,net_abs_deg=abs(net),required_deg=rot.get('required_deg'),rotation_ready=rot.get('ready'),rotation_gate_enabled=rot.get('rotation_gate_enabled'),prediction=pred,window_reason=w.get('reason'))
  if stable>row['max_stable_frames']:row['max_stable_frames']=stable;row['max_stable_evidence']=small
  if abs(net)>row['max_abs_net_deg']:row['max_abs_net_deg']=abs(net);row['signed_net_at_max_abs_deg']=net;row['angle_evidence']=small
  sig=(s.get('state'),w.get('reason'),stable,rot.get('ready'),bool(pred))
  if last.get(key)!=sig:recent.append(dict(run=run,item=item,**small));last[key]=sig
  sent=x.get('last_observation_tx') or {};wire=sent.get('wire','')
  fields=wire.split(',')
  current_pose_wire=len(fields)>3 and fields[0]=='OC2' and fields[1]==str(run) and fields[2]==str(s.get('pose'))
  if current_pose_wire and (not row['OC2'] or row['OC2'][-1]['wire']!=wire):
   row['OC2'].append(dict(wire=wire,sent_monotonic_s=sent.get('sent_monotonic_s'),capture_host_age_ms=sent.get('host_capture_age_ms'),observed_row=small));row['OC2']=row['OC2'][-6:]
print('PICKUP60_TRIAL_JSON='+json.dumps(dict(read_only=True,pid=pid,session=str(session),path=str(p),file_size=size,tail_start=start,tail_limit_bytes=32*1024*1024,parsed_rows=parsed,partial_rows=partial,state_counts=states,items=items,recent_gate_changes=list(recent),limit='Recorded visual net estimates and command times only; no exposure/physical-angle/actual grip proof. State0/8 at current time does not discard earlier item rows.'),ensure_ascii=False))
'''
def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--extract',action='store_true');a=ap.parse_args();d=json.loads((E/'pi-deploy.json').read_text())
 bound=dict(pid=d['after']['pid'],session=d['session'],root=d['after']['cwd'])
 if not a.extract:print(json.dumps(dict(local_only=True,ssh_executed=False,binding=bound,max_tail_bytes=32*1024*1024)));return
 code=SCRIPT.replace('bound=BINDING','bound='+repr(bound),1);encoded=base64.b64encode(zlib.compress(code.encode(),9)).decode();out=remote(R/'tmp/integration_mirror_ssh_20261117.py',"import zlib,base64;exec(zlib.decompress(base64.b64decode("+repr(encoded)+")))")
 rows=[x.split('PICKUP60_TRIAL_JSON=',1)[1] for x in out.splitlines() if x.startswith('PICKUP60_TRIAL_JSON=')];assert len(rows)==1;report=json.loads(rows[0]);p=E/('pi-trial-observations-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'.json');p.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(report=str(p),state_counts=report['state_counts'],item_keys=list(report['items']),read_only=True)))
if __name__=='__main__':main()
