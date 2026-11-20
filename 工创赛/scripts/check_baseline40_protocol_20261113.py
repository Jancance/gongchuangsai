"""Narrow no-hardware protocol, single-shot client and UART-slot checks."""
from pathlib import Path
import binascii,csv,json,tempfile
import p1_serial_motion as p
import vehicle_uart_capture as capture
R=Path(__file__).resolve().parents[2]
E=R/'工创赛/evidence/navigation-planning/2026-11-13/baseline40-implementation'
def frame(tag,values):
    b=(tag+','+','.join(map(str,values))+',').encode()
    return b+f'{binascii.crc_hqx(b,0xffff):04X}\r\n'.encode()
checks=[]
def check(name,condition):
    assert condition,name
    checks.append(name)
def main():
    values=[0]*31
    values[:4]=[1,1000,2,42]
    values[4]=2;values[7]=0xe9;values[15:18]=[990,10,1];values[23]=168000000
    values[24:]=[-32768,32767,1,-1,0,100,200]
    ir=frame('IR1',values)
    check('IR1 valid signed raw and uint cycle',capture.decode(ir)[1]['accel_x_raw']==-32768)
    bad_cases=[]
    for index,value in [(2,3),(17,2),(24,-32769),(16,11),(23,0)]:
        v=values.copy();v[index]=value;bad_cases.append(frame('IR1',v))
    bad_cases += [ir[:-8]+b'BAD\r\n',frame('IR1',values[:-1])]
    for i,bad in enumerate(bad_cases):
        try:capture.decode(bad)
        except (ValueError,OverflowError):check(f'IR1 invalid case {i} rejected',True)
        else:raise AssertionError(i)
    check('BF1 exact config',capture.decode(frame('BF1',[0,1000,8,100,2100,400,150,300,130,12779]))[1]['plan_id']==8)
    with tempfile.TemporaryDirectory(dir=R/'工创赛/tmp',prefix='baseline40-protocol-') as t:
        out=Path(t)/'capture';c=capture.Capture(out,0)
        for seq in [42,42,45,44]:
            v=values.copy();v[3]=seq;c.feed(frame('IR1',v))
        report=c.close('offline_test')
        rows=list(csv.DictReader(next(out.glob('IR1_*.csv')).open(encoding='utf-8-sig')))
        check('IR1 duplicate gap reorder exported',rows[1]['sample_duplicate']=='True' and rows[2]['sample_sequence_gap']=='2' and rows[3]['sample_out_of_order']=='True')
        check('capture sends no commands',report['serial_commands_sent']==0)
    fields=p.FIELDS+p.SEQUENCE_FIELDS+p.CAN_DIAG_FIELDS+p.ACTION_FIELDS+p.DISPLAY_FIELDS+p.DISTANCE_FIELDS+['distance_fault_reason','can_stack_words','debug_plan_id']
    r=dict.fromkeys(fields,0)
    r.update(source_valid=1,selftest=1,protocol_selftest=1,motion_mode=3,base_rpm=100,
             feedback_limit_rpm=130,sequence_count=1,debug_plan_id=8,target_theoretical_milli_cm_s=40000)
    ctl=p.Controller('run-once',3,8)
    for i in range(3):r.update(seq=i,tick_ms=i*100);commands=ctl.consume(r.copy(),i*.1)
    check('explicit PLAN8 only after paced healthy frames',len(commands)==1 and b',PLAN8,' in commands[0])
    r.update(seq=3,tick_ms=300,remote_accepted=1);ctl.consume(r.copy(),.3)
    for i in range(4,7):r.update(seq=i,tick_ms=i*100);commands=ctl.consume(r.copy(),i*.1)
    check('ARM after PLAN8 acknowledgement and health',len(commands)==1 and b',ARM,' in commands[0])
    r.update(seq=7,tick_ms=700,remote_state=1,token_seq=6,token_tick=600)
    commands=ctl.consume(r.copy(),.7)
    check('one START after matching token',len(commands)==1 and b',START,' in commands[0])
    check('cached telemetry cannot resend START',ctl.consume(r.copy(),.71)==[])
    r.update(seq=8,tick_ms=800,remote_state=2,phase=2);check('running acknowledgement',ctl.consume(r.copy(),.8)==[])
    check('existing KEEP cadence',any(b',KEEP,' in x for x in ctl.tick(1.0)))
    r.update(seq=9,tick_ms=1100,phase=3,backend_result=1,stop_mask=15,sequence_state=3,sequence_completed=1)
    ctl.consume(r.copy(),1.1);check('software complete has no restart',ctl.status=='software_complete' and ctl.tick(2)==[])
    for plan in [None,0,6,7]:
        other=p.Controller('run-once',3,plan)
        check(f'40 identity rejected by old/default plan {plan}',not other._speed_profile_healthy(r))
    ordinary=r|dict(debug_plan_id=6,base_rpm=15,feedback_limit_rpm=30)
    speed20=r|dict(debug_plan_id=7,base_rpm=52,feedback_limit_rpm=80)
    check('ordinary profile preserved',p.Controller('run-once',3,6)._speed_profile_healthy(ordinary))
    check('20 profile preserved',p.Controller('run-once',3,7)._speed_profile_healthy(speed20))
    check('default monitor never commands',p.Controller().consume(r,1)==[])
    # Assumed frame sizes; call order and attempt/success cadence follow C.
    # No BT/CT/fault/endpoint/OD slots here: this is the active straight segment.
    # PM1 retries only in baseline; normal firmware preserves the old attempt cadence.
    busy_until=0;status_last=None;wheel_last=0;ir_last=0;bf_last=0;pm_last=0
    sent=dict.fromkeys(['TM1','WF1','IR1','BF1','PM1'],0);wait_fail=0
    for now in range(0,5000,10):
        def send(tag,size):
            nonlocal busy_until
            if now<busy_until:return False
            busy_until=now+size*10000/115200;sent[tag]+=1;return True
        if status_last is None or now-status_last>=100:
            if send('TM1',500):status_last=now
        if status_last is None:continue
        phase=now-status_last
        if now-wheel_last>=200 and 40<=phase<100:
            if send('WF1',337):wheel_last=now  # baseline-only new success cadence
        if now-ir_last>=200 and 70<=phase<100:
            if send('IR1',323):ir_last=now
            else:wait_fail+=1
        if (not sent['BF1'] or now-bf_last>=1000) and 40<=phase<100:
            if send('BF1',160):bf_last=now
        if now-pm_last>=200 and 40<=phase<100:
            if send('PM1',512):pm_last=now  # baseline-only success cadence
    check('same-phase TM500 leaves WF1 and IR1 sending opportunities',sent['IR1']>=15 and sent['WF1']>=15)
    check('BF1 parameter frame has sending opportunities',sent['BF1']>=4)
    check('PM1 position frame has sending opportunities',sent['PM1']>=10)
    report={'checks':checks,'passed':len(checks),'hardware_accessed':False,
            'slot_schedule_assumptions':{'baud':115200,'format':'8N1','TM1_bytes':500,'WF1_bytes':337,'IR1_bytes':323,'BF1_bytes':160,'PM1_bytes':512,'scope':'active straight segment, no fault/history/OD competition; PM baseline success cadence'},
            'slot_schedule_counts':sent,'IR1_busy_retries':wait_fail,
            'IR1_worst_field_length_bytes':4+24*11+7*7+6,
            'IR1_max_nominal_bytes_per_second':(4+24*11+7*7+6)*5}
    (E/'protocol_checks.json').write_text(json.dumps(report,indent=2),'utf8')
    print('BASELINE40_PROTOCOL_OK',len(checks),sent)
if __name__=='__main__':main()
