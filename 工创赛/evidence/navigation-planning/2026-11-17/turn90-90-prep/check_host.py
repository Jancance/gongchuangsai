from pathlib import Path
import importlib.util,json,binascii
E=Path(__file__).resolve().parent;p=E/'p1_serial_motion_turn90.py';sp=importlib.util.spec_from_file_location('newclient',p);m=importlib.util.module_from_spec(sp);sp.loader.exec_module(m);checks=0
def ck(v):
 global checks;checks+=1;assert v,checks
def row(plan):
 r=dict.fromkeys(m.FIELDS,0);r.update(debug_plan_id=plan,motion_mode=3,source_valid=1,selftest=1,protocol_selftest=1,sequence_count=1,action_kind=3,base_rpm=82,feedback_limit_rpm=130,tick_ms=1000,seq=1)
 return r
for plan in [20,21]:
 c=m.Controller('run-once',3,plan,expected_complete_powered=True);r=row(plan);ck(c._speed_profile_healthy(r))
 for key,val in [('debug_plan_id',21 if plan==20 else 20),('base_rpm',81),('feedback_limit_rpm',129),('action_kind',4),('sequence_count',2),('distance_mode',1)]:
  q=r.copy();q[key]=val;ck(not c._speed_profile_healthy(q))
 c.start_sent=True;c.stage='running';r.update(phase=2,correction_rpm=81.4);ck(c.consume(r,1)==[] and c.status is None)
 q=r.copy();q.update(seq=2,correction_rpm=82);ck(c.consume(q,2) and c.status=='turn90_profile_changed');ck(c.tick(3)==[])
 d=m.Controller('run-once',3,plan,expected_complete_powered=True);d.start_sent=True;d.stage='running';r=row(plan);r.update(phase=3,backend_state=9,backend_result=1,stop_mask=15,vm_request=1,pe2_active=1,sequence_state=3,sequence_completed=1);ck(m.powered_done_record(r));d.consume(r,1);ck(d.status=='software_complete')
 f=m.command('PLAN'+str(plan),(3,900));a,b=f.rstrip().rsplit(b',',1);ck(int(b,16)==binascii.crc_hqx(a+b',',0xffff))
 try:m.command('PLAN22');raise AssertionError('accepted22')
 except ValueError:checks+=1
for plan in [20,21]:
 c=m.Controller('run-once',3,plan,expected_complete_powered=True);frames=[];tick=1000;seq=0
 def feed(r):
  global tick,seq
  tick+=100;seq+=1;r.update(tick_ms=tick,seq=seq)
  if r['debug_plan_id']==8:c.baseline_config=m.baseline40_config(8);c.baseline_tick=tick-10
  frames.extend(c.consume(r,tick/1000))
 for _ in range(3):
  r=row(8);r.update(base_rpm=100,action_kind=4,target_theoretical_milli_cm_s=40000);feed(r)
 ck(c.stage=='plan_wait' and sum((',PLAN%d,'%plan).encode() in f for f in frames)==1)
 r=row(plan);r['remote_accepted']=1;feed(r);ck(c.plan_selected and c.stage=='observe')
 for _ in range(3):r=row(plan);r['remote_accepted']=1;feed(r)
 ck(c.stage=='arm_wait' and sum(b',ARM,' in f for f in frames)==1)
 r=row(plan);r.update(remote_state=1,remote_accepted=2,token_seq=c.token[0],token_tick=c.token[1]);feed(r)
 ck(c.start_sent and sum(b',START,' in f for f in frames)==1)
 r=row(plan);r.update(phase=2,remote_state=2,remote_accepted=3,token_seq=c.token[0],token_tick=c.token[1]);feed(r);ck(c.stage=='running')
 r=row(plan);r.update(phase=3,backend_state=9,backend_result=1,stop_mask=15,vm_request=1,pe2_active=1,sequence_state=3,sequence_completed=1);feed(r);ck(c.status=='software_complete')
 ck(sum(b',START,' in f for f in frames)==1)
(E/'host_checks.json').write_text(json.dumps(dict(status='PASS',checks=checks,hardware_access=False),indent=2),encoding='utf-8');print('Host',checks,'checks passed')
