from pathlib import Path
import importlib.util, json, binascii, ast, hashlib

E=Path(__file__).resolve().parent
p=E/'p1_serial_motion_powered_done.py'
ast.parse(p.read_text('utf-8'))
spec=importlib.util.spec_from_file_location('short_candidate',p)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
checks=0
def ck(value):
    global checks
    checks+=1
    assert value

expected={8:(100,4,2100),9:(102,5,2100),10:(102,6,2100),11:(100,7,2100),12:(102,6,1500)}
ck(m.BASELINE40_SPECS==expected)
def idle(plan,tick=1000):
    rpm,kind,mm=expected[plan]
    r=dict.fromkeys(m.FIELDS,0)
    r.update(debug_plan_id=plan,sequence_count=1,target_theoretical_milli_cm_s=40000,
        base_rpm=rpm,feedback_limit_rpm=130,tick_ms=tick,action_kind=kind,
        motion_mode=3,source_valid=1,selftest=1,protocol_selftest=1)
    return r
for plan,(rpm,kind,mm) in expected.items():
    c=m.Controller('run-once',3,plan,expected_complete_powered=True)
    c.baseline_config=m.baseline40_config(plan);c.baseline_tick=900
    r=idle(plan)
    ck(c._speed_profile_healthy(r))
    for key,value in [('action_kind',3),('feedback_limit_rpm',129),
                      ('sequence_count',2),('target_theoretical_milli_cm_s',39999)]:
        q=r.copy();q[key]=value;ck(not c._speed_profile_healthy(q))
    for wrong_mm in {1499,1500,1501,2099,2100,2101}-{mm}:
        c.baseline_config=[plan,rpm,wrong_mm,400,150,300,130,12779]
        ck(not c._speed_profile_healthy(r))
    c.baseline_config=m.baseline40_config(plan)
    c.start_sent=True;c.stage='running'
    r.update(seq=1,phase=2,distance_mode=1,distance_base_rpm=rpm,
             distance_target_milli_mm=mm*1000)
    ck(c.consume(r,1)==[] and c.status is None)
    q=r.copy();q['seq']=2;q['distance_target_milli_mm']=(2100 if mm==1500 else 1500)*1000
    ck(c.consume(q,2)!=[] and c.status=='baseline40_profile_changed')
    frame=m.command('PLAN'+str(plan),(3,900))
    prefix,crc=frame.rstrip().rsplit(b',',1)
    ck(int(crc,16)==binascii.crc_hqx(prefix+b',',0xffff))

# A positive simulated protocol path must reach ARM and exactly one START.
# No WindowsSerial object is constructed; returned frames stay in memory.
c=m.Controller('run-once',3,12,expected_complete_powered=True)
frames=[];tick=1000;seq=0;accepted=0
def feed(r):
    global tick,seq
    tick+=100;seq+=1;r.update(tick_ms=tick,seq=seq)
    c.baseline_config=m.baseline40_config(r['debug_plan_id'])
    c.baseline_tick=tick-10
    out=c.consume(r,tick/1000)
    frames.extend(out)
    return out
for _ in range(3): feed(idle(8))
ck(any(b',PLAN12,' in f for f in frames) and c.stage=='plan_wait')
r=idle(12);r['remote_accepted']=1;feed(r)
ck(c.plan_selected and c.stage=='observe')
for _ in range(3):
    r=idle(12);r['remote_accepted']=1;feed(r)
ck(c.stage=='arm_wait' and sum(b',ARM,' in f for f in frames)==1)
r=idle(12);r.update(remote_state=1,remote_accepted=2,
                   token_seq=c.token[0],token_tick=c.token[1])
feed(r)
ck(c.start_sent and c.stage=='start_wait')
ck(sum(b',START,' in f for f in frames)==1)
r=idle(12);r.update(phase=2,remote_state=2,remote_accepted=3,
        token_seq=c.token[0],token_tick=c.token[1],distance_mode=1,
        distance_base_rpm=50,distance_target_milli_mm=1500000,base_rpm=50)
feed(r);ck(c.stage=='running' and c.status is None)
r.update(phase=3,backend_state=9,backend_result=1,stop_mask=15,
        vm_request=1,pe2_active=1,sequence_state=3,sequence_completed=1,
        distance_reached=1)
feed(r);ck(c.status=='software_complete' and m.powered_done_record(r))
ck(sum(b',START,' in f for f in frames)==1)

# Wrong target and action must fail over enough healthy time to ARM otherwise.
for wrong_kind in [4,5,7,8]:
    c=m.Controller('run-once',3,12,expected_complete_powered=True)
    c.plan_selected=True;c.baseline_config=m.baseline40_config(12)
    emitted=[]
    for i in range(8):
        r=idle(12,1000+i*100);r.update(seq=i+1,action_kind=wrong_kind)
        c.baseline_tick=r['tick_ms']-10
        emitted+=c.consume(r,1+i*.1)
    ck(not any(b',ARM,' in f or b',START,' in f for f in emitted))
for verb in ['PLAN13','PLAN012','PLAN12x','PLAN120','PLAN+12','PLAN']:
    try:m.command(verb)
    except ValueError:ck(True)
    else:ck(False)
report={'status':'PASS','checks':checks,'hardware_accessed':False,
    'serial_opened':False,'serial_TX_bytes':0,'simulated_START_frames':1,
    'scope':'Explicit 8..12 specs, wrong target/kind refusal, CRC, positive PLAN12-ARM-START-DONE path',
    'client_sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
(E/'host_checks.json').write_text(json.dumps(report,indent=2),'utf-8')
print(json.dumps(report))
