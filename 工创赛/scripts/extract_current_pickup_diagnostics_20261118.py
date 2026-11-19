"""Local by default; --extract only reads the exact audited Pi session JSONL."""
from pathlib import Path
import argparse,base64,zlib,json,datetime
from stage23_pi_install_20261116 import remote
R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-18/pi-persistence'
SCRIPT=r'''
from pathlib import Path
from collections import Counter,deque
import json,os
binding=BINDING
pid=binding['pid'];root=Path(binding['root']).resolve();session=Path(binding['session']).resolve()
assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()==binding['boot_id'],'New boot: re-audit first'
argv=[x.decode() for x in Path('/proc/'+str(pid)+'/cmdline').read_bytes().split(b'\0') if x]
assert any(Path(a).name=='source_fast_cycle_view_20261115.py' for a in argv)
assert '--session-dir' in argv
value=Path(argv[argv.index('--session-dir')+1]);cwd=Path(os.readlink('/proc/'+str(pid)+'/cwd'))
if not value.is_absolute():value=cwd/value
assert value.resolve()==session and root in session.parents,'Process/session changed: re-audit'
path=session/'observations.jsonl';size=path.stat().st_size
windows=[(0,min(size,8*1024*1024))]
if size>8*1024*1024:windows.append((max(8*1024*1024,size-24*1024*1024),size))
counts=Counter();reasons=Counter();first=[];recent=deque(maxlen=24);tx=deque(maxlen=10);lastsig=None;parsed=bad=0;max_abs_rotation=0;max_abs_rotation_signed=0
def subset(d,keys):return {k:d[k] for k in keys if k in d} if isinstance(d,dict) else {}
with path.open('rb') as f:
 for start,end in windows:
  f.seek(start)
  if start:f.readline()
  while f.tell()<end:
   line=f.readline()
   if not line:break
   try:x=json.loads(line)
   except (ValueError,UnicodeError):bad+=1;continue
   parsed+=1;s=x.get('mcu_status') or {};w=x.get('pickup_window') or {};rot=x.get('initial_rotation') or w.get('first_rotation') or {};stop=x.get('stop_confirmation') or {};pred=x.get('direct_frame_trigger')
   state=s.get('state');counts[str(state)]+=1
   if state not in (2,4,5,6):continue
   reasons[str(w.get('reason'))]+=1
   net=rot.get('observed_net_deg')
   if isinstance(net,(int,float)) and abs(net)>max_abs_rotation:max_abs_rotation=abs(net);max_abs_rotation_signed=net
   row=dict(frame_sequence=x.get('frame_sequence'),host_read_completed_monotonic_s=x.get('host_read_completed_monotonic_s'),mcu=subset(s,('run','state','pose','item','colors','accepted','done')),
      pickup_window=w,first_rotation=rot,stop_confirmation=stop,prediction=pred,stationary=x.get('stationary'),frame_epoch=x.get('frame_epoch'),recognition_enabled=x.get('recognition_enabled'))
   det=x.get('detection') or {};row['detection']=dict(calibrated=det.get('calibrated'),slot_colors=det.get('slot_colors'),regions=[subset(v,('color_id','center_x_px','center_y_px','region','motion_state','speed_px_s')) for v in det.get('regions',[])[:3]])
   sig=(s.get('run'),state,s.get('pose'),s.get('item'),w.get('reason'),w.get('stable_frames'),rot.get('ready'),bool(pred))
   if sig!=lastsig:
    if len(first)<8:first.append(row)
    recent.append(row);lastsig=sig
   sent=x.get('last_observation_tx') or {}
   if sent.get('wire','').startswith('OC2') and (not tx or tx[-1].get('wire')!=sent['wire']):tx.append(subset(sent,('wire','sent_monotonic_s','host_capture_age_ms','capture_metadata')))
print('PICKUP_DIAGNOSTICS_JSON='+json.dumps(dict(read_only=True,pid=pid,boot_id=binding['boot_id'],session=str(session),observations_file=str(path),file_size=size,byte_windows_scanned=windows,middle_omitted=size>32*1024*1024,parsed_rows=parsed,malformed_or_partial_rows=bad,state_counts=counts,pickup_window_reason_counts=reasons,max_abs_observed_first_rotation_deg=max_abs_rotation,signed_rotation_at_max_abs_deg=max_abs_rotation_signed,first_gate_changes=first,recent_gate_changes=list(recent),recent_OC2=list(tx),no_eligible_rows_means='This exact running session may contain only completed/idle states; do not invent earlier-run gate evidence',diagnostic_limit='window.reason is direct; stable_frames/rotation are recorded diagnostics, not a newly run detector or proof of sensor exposure timing'),ensure_ascii=False))
'''
def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--audit-json',type=Path,default=E/'audit-20261118-040246.json');ap.add_argument('--extract',action='store_true');a=ap.parse_args()
 audit=json.loads(a.audit_json.read_text());assert len(audit['main_processes'])==1 and len(audit['current_session_logs'])==1
 binding=dict(pid=audit['main_processes'][0]['pid'],root=audit['root'],session=audit['current_session_logs'][0]['session_dir'],boot_id=audit['boot_id'])
 if not a.extract:print(json.dumps(dict(local_only=True,ssh_executed=False,binding=binding,maximum_scan_bytes=32*1024*1024),ensure_ascii=False));return
 code=SCRIPT.replace('binding=BINDING','binding='+repr(binding),1);encoded=base64.b64encode(zlib.compress(code.encode(),9)).decode()
 result=remote(R/'tmp/integration_mirror_ssh_20261117.py',"import zlib,base64;exec(zlib.decompress(base64.b64decode("+repr(encoded)+")))")
 lines=[x.split('PICKUP_DIAGNOSTICS_JSON=',1)[1] for x in result.splitlines() if x.startswith('PICKUP_DIAGNOSTICS_JSON=')];assert len(lines)==1
 report=json.loads(lines[0]);out=E/('pickup-diagnostics-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'.json');out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(report=str(out),read_only=True,state_counts=report['state_counts'],reasons=report['pickup_window_reason_counts']),ensure_ascii=False))
if __name__=='__main__':main()
