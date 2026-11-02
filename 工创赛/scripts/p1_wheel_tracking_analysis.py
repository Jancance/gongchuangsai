"""Offline command/encoder interval alignment; no hardware or motion access."""
import json,csv,statistics
from pathlib import Path
FORWARD={1:1,2:-1,3:1,4:-1}
def signed32(x):return x if x<0x80000000 else x-0x100000000
def magnitude(value,sign):return -value if sign else value
def command_average(commands,start,end):
 if end<=start:raise ValueError('nonpositive interval')
 before=[x for x in commands if x[0]<=start]
 if not before:return None
 value=before[-1][1];tick=start;area=0
 for ms,new in commands:
  if start<ms<end:area+=(ms-tick)*value;tick=ms;value=new
 return (area+(end-tick)*value)/(end-start)
def analyze(dump):
 if dump['dropped']:raise ValueError('Incomplete trace: dropped events; cannot conclude about end segment')
 events=dump['events'];sessions=sorted({r['session'] for r in events});assert len(sessions)==1,'Expected one diagnostic session'
 assert all(b['ms']>=a['ms'] for a,b in zip(events,events[1:])),'Clock discontinuity'
 rows=[];summary={}
 for wheel in range(1,5):
  sign=FORWARD[wheel];e=[r for r in events if r['address']==wheel]
  commands=[(r['ms'],sign*magnitude(r['value'],r['detail'])/10) for r in e if r['kind']==1]
  commands +=[(r['ms'],0) for r in e if r['kind']==4];commands.sort()
  assert commands
  peak_ms=max(commands,key=lambda r:abs(r[1]))[0]
  positions=[(r['ms'],sign*magnitude(r['value'],r['detail'])) for r in e if r['kind']==3]
  feedback=[(r['ms'],sign*signed32(r['value'])) for r in e if r['kind']==2]
  anchor=None;wheelrows=[]
  for ms,pos in positions:
   if anchor is None:anchor=(ms,pos);continue
   dt=ms-anchor[0]
   if dt<150:continue
   if dt>400:anchor=(ms,pos);continue
   target=command_average(commands,anchor[0],ms)
   if target is None:anchor=(ms,pos);continue
   actual=(pos-anchor[1])*60000/(65536*dt)
   f=[x for x in feedback if x[0]<=ms];fb=f[-1] if f else None
   row={'wheel':wheel,'start_ms':anchor[0],'end_ms':ms,'dt_ms':dt,'target_mean_rpm':target,'encoder_mean_rpm':actual,'encoder_minus_target_rpm':actual-target,'integer_feedback_rpm':fb[1] if fb and ms-fb[0]<=500 else None,'feedback_age_ms':ms-fb[0] if fb else None,'late_low':int(ms>peak_ms and 3<=target<15)}
   rows.append(row);wheelrows.append(row);anchor=(ms,pos)
  low=[r for r in wheelrows if r['late_low']];duration=sum(r['dt_ms'] for r in low)
  summary[str(wheel)]={'name':{1:'LR',2:'RR',3:'LF',4:'RF'}[wheel],'command_events':len(commands),'speed_samples':len(feedback),'position_samples':len(positions),'low_windows':len(low),'low_duration_ms':duration,'low_command_mean_rpm':sum(r['target_mean_rpm']*r['dt_ms'] for r in low)/duration if duration else None,'low_encoder_mean_rpm':sum(r['encoder_mean_rpm']*r['dt_ms'] for r in low)/duration if duration else None,'low_encoder_minus_target_mean_rpm':sum(r['encoder_minus_target_rpm']*r['dt_ms'] for r in low)/duration if duration else None}
 return {'per_wheel':summary,'windows':rows,'limits':['Encoder derivative is motor rotation over150..400ms, not ground motion or instantaneous0.1RPM accuracy','Targets are ACKed requests; mechanical response delay not measured','Per-wheel windows have different endpoints, so do not call their averages simultaneous','No wheel current/torque measurement; tracking difference alone does not identify root cause']}
def main():
 import argparse
 p=argparse.ArgumentParser();p.add_argument('dump',type=Path);a=p.parse_args();d=json.loads(a.dump.read_text(encoding='utf8'));r=analyze(d);out=a.dump.parent
 (out/'wheel_tracking_analysis.json').write_text(json.dumps(r,indent=2),encoding='utf8')
 with (out/'wheel_tracking_windows.csv').open('w',encoding='utf-8-sig',newline='') as f:
  w=csv.DictWriter(f,fieldnames=r['windows'][0].keys());w.writeheader();w.writerows(r['windows'])
 print(json.dumps(r['per_wheel'],indent=2))
if __name__=='__main__':main()
