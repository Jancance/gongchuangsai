from pathlib import Path
import hashlib,importlib.util,json,subprocess,sys
sys.dont_write_bytecode=True
R=Path.cwd();E=R/'工创赛/evidence/navigation-planning/2026-11-17/turn90-keepalive3s-prep';V=E/'independent-review';U=R/'3.stm32f407工程/CHASSIS/USER'
V.mkdir(exist_ok=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def extract(s,sig):
 b=s.index(sig);i=s.index('{',b)+1;d=1
 while d:d+=(s[i]=='{')-(s[i]=='}');i+=1
 return s[b:i]
serial=(U/'app_serial_runtime.c').read_bytes().decode('gbk').replace('\r\n','\n');motion=(U/'app_motion_runtime.c').read_bytes().decode('gbk').replace('\r\n','\n')
unit=(E/'actual-c-on/unit.c').read_bytes().decode('gbk').replace('\r\n','\n')
sigs=['uint16_t App_SerialCrc(','static uint32_t App_SerialUInt(','static void App_SerialReject(','static void App_SerialMotionCommand(','void App_SerialMotionByte(','void App_SerialMotionSafety(void)\n{']
for sig in sigs:assert unit.count(extract(serial,sig))==1,sig
for sig in ['void App_CombinedTestStop(','void App_CombinedTestSafety(']:assert unit.count(extract(motion,sig))==1,sig
runner=(E/'check_keepalive.py').read_text('utf-8-sig')
runner=runner.replace("E=Path(__file__).resolve().parent","E=R/'工创赛/evidence/navigation-planning/2026-11-17/turn90-keepalive3s-prep'",1)
runner=runner.replace("T=E/(('red-'","T=E/'independent-review'/(('red-'",1)
runner=runner.replace("(E/'checks.json').write_text","(E/'independent-review'/'checks.json').write_text",1)
(V/'rerun_keepalive.py').write_text(runner,encoding='utf8')
cp=subprocess.run([sys.executable,'-B','-X','utf8',str(V/'rerun_keepalive.py')],cwd=R,capture_output=True,text=True,encoding='utf8',timeout=120)
(V/'rerun.log').write_text(cp.stdout+cp.stderr,encoding='utf8');assert cp.returncode==0,cp.stdout+cp.stderr
checks=0
def ck(b):
 global checks
 checks+=1
 assert b,checks
spec=importlib.util.spec_from_file_location('review_turn_client',E/'p1_serial_motion_turn90_keepalive3s.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
def ctl(plan,stage='start_wait'):
 c=m.Controller('run-once',3,plan,expected_complete_powered=True);c.stage=stage;c.stage_tick=0;c.last_rx=0;c.last_keep=0;c.start_sent=stage in ['start_wait','running'];c.token=(49,4721);return c
def row(c,**kw):
 r=dict.fromkeys(m.FIELDS,0);r.update(seq=1,tick_ms=3000,phase=2,debug_plan_id=c.plan,base_rpm=82,feedback_limit_rpm=130,sequence_count=1,action_kind=3,distance_mode=0,remote_state=2,token_seq=49,token_tick=4721);r.update(kw);return r
for plan in [20,21]:
 for t in [1.0,2.0,2.01,3.0,3.999]:
  c=ctl(plan);ck(c.tick(t)==[m.command('KEEP',c.token)] and c.status is None)
 c=ctl(plan);ck(c.tick(4.0)==[m.command('STOP',c.token)] and c.status=='ack_timeout')
 ck(c.consume(row(c),4.1)==[] and c.tick(4.2)==[] and c.status=='ack_timeout')
 for stage in ['plan_wait','arm_wait','stop_wait']:
  c=ctl(plan,stage);ck(c.tick(1.999)==[] and c.status is None);ck(c.tick(2.0)==[m.command('STOP',c.token)] and c.status=='ack_timeout')
 c=ctl(plan);ck(c.tick(2.5)==[m.command('KEEP',c.token)]);ck(c.consume(row(c),2.6)==[] and c.stage=='running')
 ck(c.tick(6.0)==[m.command('KEEP',c.token)] and c.status is None)
 c=ctl(plan,'running');ck(c.tick(4.999)==[m.command('KEEP',c.token)]);ck(c.tick(5.0)==[m.command('STOP',c.token)] and c.status=='telemetry_timeout')
 ck(c.consume(row(c),5.1)==[] and c.tick(5.2)==[] and c.status=='telemetry_timeout')
 for phase in [4,5]:
  c=ctl(plan,'running');ck(c.consume(row(c,phase=phase,trip=1),.1)==[m.command('STOP',c.token)] and c.status=='motion_fault_or_cancelled')
  ck(c.consume(row(c,seq=2),.2)==[] and c.tick(.3)==[] and c.status=='motion_fault_or_cancelled')
for plan in range(8,20):
 c=ctl(plan);ck(c.tick(2.0)==[m.command('STOP',c.token)] and c.status=='ack_timeout')
old=E.parent/'turn90_90-field-retry1/p1_serial_motion_powered_done.py'
ck(sha(old)==sha(E/'p1_serial_motion_powered_done.py.before'))
before=json.loads((E/'snapshot_before.json').read_text('utf-8-sig'))
dirty={p:sha(R/p)==h for p,h in before['original_dirty'].items()};ck(all(dirty.values()))
build=json.loads((E/'build_receipt.json').read_text('utf-8-sig'));ck(build['success'] and sha(Path(build['axf']))==build['axf_sha256'])
data=(U/'app_serial_runtime.c').read_bytes();ck(data.count(b'\n')==data.count(b'\r\n'))
rr=json.loads((V/'checks.json').read_text('utf8'));ck(rr['source_sha256']==sha(U/'app_serial_runtime.c'))
report=dict(status='PASS',hardware_accessed=False,product_files_written=False,source_sha256=sha(U/'app_serial_runtime.c'),exact_function_slices=8,actual_c=rr['actual_c'],upstream_host_checks=rr['host_checks'],independent_host_and_preservation_checks=checks,original_dirty_unchanged=dirty,old_client_unchanged=True,build_success=True,axf_sha256=build['axf_sha256'],limits='Timing/dispatch only. RTOS, physical stop and RF recovery are not proven; one-way no-RX with arriving KEEP can approach 5+3 seconds before board cancellation if STOP is lost.')
(V/'review_verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8');print(json.dumps(report,ensure_ascii=False))

