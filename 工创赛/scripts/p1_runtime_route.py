"""Bounded runtime route upload. Default is read-only monitor; never flashes."""
import argparse
import binascii
import json
from decimal import Decimal
from pathlib import Path
import time

import p1_x_main_transport as wire
from p1_x_load_telemetry import parse_load_telemetry, LoadAccumulator

KINDS={"FORWARD":4,"LEFT":5,"RIGHT":6,"TURN":3}
ACK_FIELDS=['route_id','op','result','mask','committed_id','digest','count','ready','consumed']
STATUS_FIELDS=['route_id','digest','state','index','completed','count','segment_elapsed_ms',
               'progress_milli_mm','fault','stage_id','mask','last_result']

def packet(tag, values):
    prefix=(tag+','+','.join(str(v) for v in values)+',').encode('ascii')
    return prefix+f'{binascii.crc_hqx(prefix,0xFFFF):04X}\n'.encode('ascii')

def command(op,token,route_id,*values):
    if op not in {'BEGIN','ROW','COMMIT','GO','ABORT'}:raise ValueError('unknown route command')
    if not 1<=route_id<=0xFFFFFFFF or not all(0<=x<=0xFFFFFFFF for x in token):raise ValueError('route/token range')
    return packet('RQ1',[*token,route_id,op,*values])

def parse_route(line):
    line=line.rstrip(b'\r\n')
    if not line.startswith((b'RA1,',b'RS1,')):return None
    if len(line)>=256:raise ValueError('route status too long')
    prefix,_,crc=line.rpartition(b',')
    if len(crc)!=4 or binascii.crc_hqx(prefix+b',',0xFFFF)!=int(crc,16):raise ValueError('route CRC')
    names=ACK_FIELDS if line.startswith(b'RA1,') else STATUS_FIELDS
    values=[int(v) for v in prefix.split(b',')[1:]]
    if len(values)!=len(names):raise ValueError('route field count')
    for name,v in zip(names,values):
        if name=='progress_milli_mm':
            if not -(2**31)<=v<2**31:raise ValueError('route progress range')
        elif not 0<=v<=0xFFFFFFFF:raise ValueError('route uint32')
    r=dict(zip(names,values));r['frame_type']='RA1' if names is ACK_FIELDS else 'RS1'
    if r['count']>8 or r['mask']>255:raise ValueError('route count/mask')
    if names is ACK_FIELDS:
        if r['op']>5 or r['result']>10 or r['ready']>1 or r['consumed']>1:raise ValueError('route ACK flags')
    elif r['state']>6 or r['completed']>r['count'] or r['index']>=max(1,r['count']):raise ValueError('route state')
    return r

def integer(value,scale):
    d=Decimal(str(value))*scale
    if not d.is_finite() or d!=d.to_integral_value():raise ValueError('value cannot be represented exactly')
    return int(d)

def load_route(path):
    cfg=json.loads(Path(path).read_text(encoding='utf8'))
    if cfg.get('schema')!='p1-runtime-route-v1' or cfg.get('frame')!='body-relative':raise ValueError('route schema/frame')
    rows=[]
    for a in cfg['actions']:
        k=KINDS[a['kind']];turn=k==3
        amount=integer(a['angle_deg'] if turn else a['distance_cm'],1000 if turn else 100)
        speed=integer(a['speed_deg_s'] if turn else a['speed_cm_s'],1000 if turn else 10)
        deadline=integer(a['timeout_ms'],1)
        if not 1000<=deadline<=75000:raise ValueError('deadline range')
        if turn:
            if abs(amount)!=90000 or speed!=30000:raise ValueError('turn range')
        elif not 1000<=amount<=21000 or not 13<=speed<=200:raise ValueError('translation range')
        rows.append((k,amount,speed,deadline))
    if not 1<=len(rows)<=8 or 5000+sum(r[3]+1000 for r in rows)>300000:raise ValueError('route capacity/budget')
    return rows

def digest(route_id,rows):
    h=2166136261
    for v in [route_id,len(rows)]+[v for row in rows for v in row]:
        for b in (v&0xFFFFFFFF).to_bytes(4,'little'):
            h=((h^b)*16777619)&0xFFFFFFFF
    return h

class RouteUpload:
    """Stop-and-wait upload. Only fresh TMQ creates tokens; RA1 never arms."""
    def __init__(self,route_id,rows):
        self.id,self.rows=route_id,rows;self.digest=digest(route_id,rows)
        self.commands=[('BEGIN',(len(rows),))]+[('ROW',(i,*r)) for i,r in enumerate(rows)]+[('COMMIT',(self.digest,))]
        self.index=0;self.sent_at=None;self.retries=0;self.complete=False;self.error=None
        self.latest=None;self.last_seq=None;self.healthy_since=None;self.healthy_tick=None;self.healthy_count=0

    def observe(self,r,now):
        if r.get('telemetry_schema')!='TMQ':return
        if self.last_seq is not None:
            delta=(r['seq']-self.last_seq)&0xFFFFFFFF
            if delta==0:return
            if delta>=0x80000000:
                if self.sent_at is not None:self.error='telemetry_session_changed'
                self.healthy_count=0;self.healthy_since=None
        self.last_seq=r['seq'];self.latest=(r,now)
        editable=(r['phase']==0 and r['remote_state']==0 and not r['backend_result']) or (
            r['phase']==3 and r['remote_state']==3 and r['remote_result']==2 and r['backend_result']==1 and r['stop_mask']==15)
        healthy=editable and not any(r[k] for k in ['trip','backend_active','vm_request','pe2_active','can_fault','rtos_fault','safety_fault']) and (
            r['source_valid']==1 and r['selftest']==1 and r['protocol_selftest']==1 and
            0<=r['angle_age_ms']<100 and 0<=r['rate_age_ms']<100 and abs(r['rate_mdps'])<=3000)
        if not healthy:
            self.healthy_since=None;self.healthy_count=0
            if self.sent_at is not None and not self.complete:self.error='health_changed_during_upload'
        else:
            if self.healthy_since is None:self.healthy_since,self.healthy_tick=now,r['tick_ms']
            if abs(((r['tick_ms']-self.healthy_tick)&0xFFFFFFFF)-(now-self.healthy_since)*1000)>150:
                self.healthy_since,self.healthy_tick,self.healthy_count=now,r['tick_ms'],0
            self.healthy_count+=1

    def consume_ack(self,r):
        if self.complete or self.error or self.sent_at is None or r['route_id']!=self.id:return
        op,values=self.commands[self.index]
        if r['op']!={'BEGIN':1,'ROW':2,'COMMIT':3}[op]:return
        if r['result'] not in (0,1):self.error=f"route_rejected_{r['result']}";return
        if op=='ROW' and not r['mask']&(1<<values[0]):return
        if op=='COMMIT':
            if (r['committed_id'],r['digest'],r['count'])!=(self.id,self.digest,len(self.rows)):
                self.error='commit_identity_mismatch';return
            if not r['ready'] or r['consumed']:self.error='route_already_consumed';return
            self.complete=True
        else:self.index+=1;self.sent_at=None;self.retries=0

    def tick(self,now):
        if self.complete or self.error or self.latest is None:return []
        r,received=self.latest
        if now-received>=0.4 or self.healthy_count<3 or self.healthy_since is None or now-self.healthy_since<0.15:return []
        if self.sent_at is not None:
            if now-self.sent_at<0.6:return []
            self.retries+=1
            if self.retries>3:self.error='upload_ack_timeout';return []
        op,values=self.commands[self.index];self.sent_at=now
        return [command(op,(r['seq'],r['tick_ms']),self.id,*values)]

class RouteController(wire.Controller):
    def __init__(self,route_id,rows):
        super().__init__('run-once',3,None)
        self.id,self.rows,self.digest=route_id,rows,digest(route_id,rows)
        self.identity_time=None;self.now=0

    def route_status(self,r,now):
        if (r['route_id'],r['digest'],r['count'])!=(self.id,self.digest,len(self.rows)):
            if self.start_sent:return self.abort('route_identity_changed')
            return []
        if r['state']==1:self.identity_time=now
        return []

    def _speed_profile_healthy(self,r):
        turn=self.rows[0][0]==3
        return (self.identity_time is not None and 0<=self.now-self.identity_time<0.6 and
            r.get('debug_plan_id')==0 and r.get('sequence_count')==len(self.rows) and not r.get('distance_mode') and
            r['feedback_limit_rpm']==(40 if turn else 80) and (r['base_rpm']==27 if turn else 3<=r['base_rpm']<=52))

    def shape(self,r):
        i=r.get('sequence_index',-1)
        if r.get('sequence_count')!=len(self.rows) or not 0<=i<len(self.rows) or r.get('debug_plan_id')!=0:return False
        kind,amount,speed,deadline=self.rows[i];turn=kind==3
        if r.get('action_kind')!=kind or r['feedback_limit_rpm']!=(40 if turn else 80):return False
        if turn:
            if not 0<=r['correction_rpm']*(1 if amount>0 else -1)<=27:return False
        elif not 0<=r['base_rpm']<=52:return False
        if r['backend_active']:
            if turn and r.get('distance_mode'):return False
            if not turn and (not r.get('distance_mode') or abs(r.get('distance_target_milli_mm',-1)-amount*100)>1):return False
        return True

    def consume(self,r,now):
        self.now=now
        if r.get('telemetry_schema')!='TMQ':return []
        if self.last_seq is not None and ((r['seq']-self.last_seq)&0xFFFFFFFF)>=0x80000000:return super().consume(r,now)
        fresh=self.last_seq is None or 0<((r['seq']-self.last_seq)&0xFFFFFFFF)<0x80000000
        if fresh and self.start_sent and r['phase'] in (2,3) and not self.shape(r):return self.abort('runtime_route_shape_changed')
        if fresh and self.start_sent and r['phase']==3 and r.get('sequence_completed')!=len(self.rows):return self.abort('runtime_route_incomplete')
        messages=super().consume(r,now)
        return [command('GO',self.token,self.id,self.digest) if b',START,' in m else m for m in messages]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',default='COM17');p.add_argument('--action',choices=['monitor','upload','run-once','stop'],default='monitor')
    p.add_argument('--route',type=Path);p.add_argument('--route-id',type=int)
    p.add_argument('--seconds',type=float,default=300);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not 1<=a.seconds<=450:p.error('seconds must be 1..450')
    if a.action in ('upload','run-once') and a.route is None:p.error('--route is required')
    rows=load_route(a.route) if a.route else None
    route_id=a.route_id if a.route_id is not None else int(time.time())&0xFFFFFFFF
    if not 1<=route_id<=0xFFFFFFFF:p.error('route ID must be nonzero uint32')
    upload=RouteUpload(route_id,rows) if a.action in ('upload','run-once') else None
    ctl=RouteController(route_id,rows) if a.action=='run-once' else wire.Controller('stop' if a.action=='stop' else 'monitor',3)
    a.output.mkdir(parents=True,exist_ok=True);prefix=a.output/(time.strftime('%Y%m%d_%H%M%S')+f'_{time.time_ns()%1000000:06d}')
    report=dict(action=a.action,route_id=route_id,digest=digest(route_id,rows) if rows else None,
                route_rows=rows,valid_frames=0,invalid_frames=0,sent_commands=0,last=None,last_route=None,last_fault=None,error=None)
    history=LoadAccumulator();serial=None;pending=b'';started=time.monotonic();drain=None;progress=0;last_rx=None
    with prefix.with_suffix('.raw').open('wb') as raw,prefix.with_suffix('.jsonl').open('w',encoding='utf8') as out:
        def send(messages):
            for m in messages:
                serial.write(m);report['sent_commands']+=1
                out.write(json.dumps(dict(direction='TX',host_monotonic=time.monotonic(),text=m.decode().strip()))+'\n');out.flush()
        try:
            serial=wire.WindowsSerial(a.port)
            while time.monotonic()-started<a.seconds:
                now=time.monotonic()
                if drain is not None and (now>=drain or (history.complete and now>=drain-89)):break
                chunk=serial.read()
                if chunk:raw.write(chunk);raw.flush();pending+=chunk
                if len(pending)>8192:raise ValueError('serial framing overflow')
                while b'\n' in pending:
                    line,pending=pending.split(b'\n',1);line+=b'\n'
                    try:
                        record=parse_route(line)
                        if record is not None:
                            if record['frame_type']=='RA1' and upload:upload.consume_ack(record)
                            elif record['frame_type']=='RS1':
                                report['last_route']=record
                                if isinstance(ctl,RouteController) and upload.complete:send(ctl.route_status(record,now))
                        elif line.startswith(b'TMQ,'):
                            record=wire.parse_telemetry(line,b'TMQ');report['last']=record;report['valid_frames']+=1;last_rx=now
                            if upload:upload.observe(record,now)
                            if upload is None or upload.complete:send(ctl.consume(record,now))
                            if now-progress>=1:
                                print(json.dumps(dict(event='telemetry',tick_ms=record['tick_ms'],phase=record['phase'],trip=record['trip'],
                                    index=record.get('sequence_index'),completed=record.get('sequence_completed'),count=record.get('sequence_count'),
                                    progress_mm=record.get('distance_progress_milli_mm',0)/1000,vm=record['pe2_active'])),flush=True);progress=now
                        elif line.startswith(b'XD1,'):
                            record=parse_load_telemetry(line);history.add(record);record=dict(frame_type='XD1',**record)
                        else:
                            record=None
                            parsers=[('HF3',wire.parse_fault_telemetry),('BTQ',lambda v:wire.parse_batch_telemetry(v,b'BTQ')),
                                     ('PE3',wire.parse_position_endpoints),('PM1',wire.parse_position_telemetry),
                                     ('OP1',wire.parse_perimeter_checkpoint),('OD1',wire.parse_odometry_telemetry),('CT',wire.parse_timing_telemetry)]
                            for tag,fn in parsers:
                                value=fn(line)
                                if value is not None:
                                    record=dict(frame_type=tag,**value)
                                    if tag=='HF3':report['last_fault']=record
                                    break
                        if record is not None:
                            out.write(json.dumps(dict(direction='RX',host_monotonic=now,**record))+'\n');out.flush()
                    except (ValueError,UnicodeError):report['invalid_frames']+=1
                if upload and not upload.complete:
                    send(upload.tick(now))
                    if upload.error:raise ValueError(upload.error)
                    if now-started>20:raise ValueError('upload_not_ready_timeout')
                if a.action=='upload' and upload.complete:break
                send(ctl.tick(now))
                if ctl.status is not None and drain is None:drain=now+(90 if a.action=='run-once' else 1)
                if ctl.start_sent and last_rx is not None and now-last_rx>=0.6:send(ctl.abort('telemetry_timeout'))
                time.sleep(0.002)
            if a.action in ('run-once','stop') and ctl.status is None:send(ctl.abort('duration_expired'))
            if upload and not upload.complete and report['error'] is None:report['error']='upload_incomplete'
        except (Exception,KeyboardInterrupt) as exc:
            report['error']=str(exc) or 'interrupted'
            if serial is not None and ctl.action!='monitor':
                try:send(ctl.abort('client_error'))
                except Exception:pass
        finally:
            if serial is not None:serial.close()
    report['status']='uploaded' if a.action=='upload' and upload.complete else ctl.status or 'monitor_finished'
    report['load_diagnostic']=history.summary();report['capture_prefix']=str(prefix)
    report['limits']='Software observations; no physical distance/stop proof. XD1 covers final backend session only.'
    prefix.with_suffix('.summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(report,ensure_ascii=True,indent=2))
    return 0 if report['error'] is None and report['status'] in ('uploaded','software_complete','stop_acknowledged','monitor_finished') else 2

if __name__=='__main__':raise SystemExit(main())
