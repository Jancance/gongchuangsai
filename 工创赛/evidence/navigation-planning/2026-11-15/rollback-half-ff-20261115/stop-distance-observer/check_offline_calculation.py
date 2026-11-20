from pathlib import Path
import json,math,struct
from calculate_stop_wheel_distance import calculate
from parse_snapshot import parse

E=Path(__file__).resolve().parent
blob=(E/'synthetic_snapshot_from_ARM.bin').read_bytes();snap=parse(blob);session=snap['session'];now=snap['slots'][2]['now_ms']
def pe_at(ticks):
 p=dict(frame_type='PE3',session=session,start_mask=15,end_mask=15,tick_ms=(ticks[-1]+17)&0xffffffff)
 for i,count in enumerate([562000,562100,561900,562050],1):
  p.update({f'start_sign{i}':0,f'start_raw{i}':0,f'start_ms{i}':1000+i,f'end_sign{i}':0 if i in (1,3) else 1,f'end_raw{i}':count,f'end_ms{i}':ticks[i-1]})
 return p
pe=pe_at([now+1010,now+1021,now+1032,now+1043]);xd=[dict(kind='stop_ack',session=session,tick_ms=now+10,address=1)]
records=[dict(frame_type='PM1',tick_ms=now-10),dict(frame_type='OD1',tick_ms=now-20),pe]
r=calculate(blob,records,xd,2105)
expected=sum([562000,562100,561900,562050])/4*math.pi*77/65536*1.02182540
assert abs(r['PE3']['calibrated_mean_mm']-expected)<1e-9
assert r['STOP_ACK_after_arrival_ms']==10
assert r['PE3']['end_samples_after_STOP_ACK_ms']==[1000,1011,1022,1033]
assert abs(r['PE_minus_arrival_common_progress_mm']-(expected-snap['slots'][2]['common_mean_mm']))<1e-9
assert r['physical_stopping_distance_established'] is False and r['hardware_provenance_established'] is False
assert abs(r['PE_minus_ruler_mm']-(expected-2105))<1e-9
wrap=bytearray(blob);struct.pack_into('<4I',wrap,32+2*48,5,0xfffff000,0xfffffff0,0xfffffffa)
wr=calculate(wrap,[pe_at([20,31,42,53])],[dict(kind='stop_ack',tick_ms=6,address=1)],None)
assert wr['arrival']['common_age_ms']==21 and wr['STOP_ACK_after_arrival_ms']==1
assert wr['PE3']['end_samples_after_STOP_ACK_ms']==[14,25,36,47]
assert 'lacks per-record session' in wr['XD_session_binding']
without=calculate(blob,[pe],[]);assert without['first_STOP_ACK_ms'] is None and without['PE3']['end_samples_after_STOP_ACK_ms'] is None
for invalid_blob,invalid_records in [(bytes(176),records),(blob,[dict(pe,session=session+1)])]:
 try:calculate(invalid_blob,invalid_records,xd)
 except ValueError:pass
 else:raise AssertionError('Invalid/other-session evidence must not produce a measurement')
(E/'synthetic_stop_wheel_difference.json').write_text(json.dumps(dict(provenance='synthetic ARM snapshot plus constructed PE/STOP records, NOT hardware',calculation=r),indent=2),'utf8')
(E/'offline_calculation_check.json').write_text(json.dumps(dict(all_passed=True,synthetic_cases=5,hardware_accessed=False,ground_distance_established=False,checks=['PE signed counts/calibration','different sample times','uint32 rollover','unknown STOP and XD session explicit','invalid/other session rejected']),indent=2),'utf8')
print('Offline wheel difference: 5 synthetic cases PASS; no ground-distance claim')
