from pathlib import Path
import hashlib,json,re,struct,subprocess
from parse_snapshot import parse
R=Path.cwd();E=Path(__file__).parent;U=R/'3.stm32f407工程/CHASSIS/USER'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def allocated(p):
 b=p.read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b);out=[]
 for i in range(h[12]):
  _,kind,flags,addr,off,size,*_=struct.unpack_from('<IIIIIIIIII',b,h[6]+i*h[11])
  if flags&2 and size:out.append([kind,flags,addr,size,hashlib.sha256(bytes(size) if kind==8 else b[off:off+size]).hexdigest()])
 return out
old=json.loads((E.parent/'s25-field-candidate/build_summary.json').read_text('utf8'))
new=json.loads((E/'build_summary.json').read_text('utf8'))
comparisons={}; symbols={}
for n,v in new['builds'].items():
 assert v['returncode']==0 and sha(Path(v['axf']))==v['axf_sha256']
 comparisons[n]={'before':old['builds'][n]['program_size'],'after':v['program_size'],
   'allocated_identical':allocated(Path(v['axf']))==allocated(Path(old['builds'][n]['axf']))}
 s=subprocess.check_output(['D:/Keil_v5/ARM/arm5_compiler/Bin/fromelf.exe','--text','-s',v['axf']]).decode(errors='replace')
 m=re.search(r'\bg_chassis_auto_lateral_timing\s+(0x[0-9a-fA-F]+)\s+Gb\s+\d+\s+Data\s+\w+\s+(0x[0-9a-fA-F]+)',s)
 if n.startswith('baseline40'):
  assert m,s[s.find('g_chassis_auto_lateral_timing')-30:s.find('g_chassis_auto_lateral_timing')+180]
  assert int(m[2],16)==104
  symbols[n]={'axf':v['axf'],'axf_sha256':v['axf_sha256'],'symbol':'g_chassis_auto_lateral_timing','address':m[1],'size_bytes':104,'read_before_reset':True,'coherence':'future authorized acquisition only: same session DONE/backend inactive/no fault before and after two identical 104-byte reads; no halt/reset/VM change/IRQ masking/getter; otherwise unstable and not accepted','getter_linked':'ChassisAuto_ReadLateralTiming' in s}
 else: assert not m
assert comparisons['baseline_off_integrated']['allocated_identical']
data=bytearray(104);assert all(not s['valid'] and s['mean_us'] is None for s in parse(data)['scopes'])
struct.pack_into('<II',data,0,1,84000000)
struct.pack_into('<5I',data,8,1,3,0,0xffffffff,0xffffffff)
struct.pack_into('<QIII',data,32,0xffffffff+32,0xfffffff0,16,32)
p=parse(data);assert p['scopes'][0]['valid'] and p['scopes'][0]['sum_cycles']==4294967327
struct.pack_into('<II',data,0,0,84000000);assert parse(data)['scopes'][0]['mean_us'] is None
struct.pack_into('<II',data,0,1,0);assert parse(data)['scopes'][0]['mean_us'] is None
# Source equivalence: strip only known instrumentation, then require exact bytes.
s=(U/'chassis_auto_test.c').read_bytes().decode('gbk');b=(E/'chassis_auto_test.c.before').read_bytes().decode('gbk')
s=s.replace('#include "hwt101.h"\r\n','')
begin=s.index('#if APP_BASELINE40_ACTIVE\r\nvolatile ChassisAutoLateralTiming')
end=s.index('volatile uint32_t g_chassis_auto_request',begin);s=s[:begin]+s[end:]
s=s.replace('    uint32_t lateral_timing_enabled,lateral_timing_start,lateral_timing_end,lateral_step_ok;\r\n','')
s=s.replace('    lateral_timing_enabled=HWT101_TimingEnabled();\r\n    lateral_timing_start=HWT101_TimingBegin();\r\n','')
s=s.replace('        lateral_timing_end=HWT101_TimingBegin();\r\n','').replace('    lateral_timing_end=HWT101_TimingBegin();\r\n','')
s=s.replace('        if(lateral_valid) Auto_LateralTimingRecordLocked(0U,lateral_timing_enabled,\r\n            lateral_timing_start,lateral_timing_end,lateral_now_ms);\r\n','')
s=s.replace('        lateral_timing_start=HWT101_TimingBegin();\r\n','')
s=s.replace('        lateral_step_ok=Auto_LateralStepLocked','        if(!Auto_LateralStepLocked')
s=s.replace('                s_heading_pending_rpm,&heading_deci,&lateral_deci);\r\n        Auto_LateralTimingRecordLocked(1U,lateral_timing_enabled,lateral_timing_start,\r\n            lateral_timing_end,(uint32_t)now*portTICK_PERIOD_MS);\r\n        if(!lateral_step_ok)','                s_heading_pending_rpm,&heading_deci,&lateral_deci))')
import difflib
if s!=b: print(''.join(difflib.unified_diff(b.splitlines(True),s.splitlines(True))))
assert s==b,'Non-observation semantic/source difference'
encoding={}
for n in ['chassis_auto_test.c','chassis_auto_test.h','hwt101.c','hwt101.h']:
 data=(U/n).read_bytes();oldb=(E/(n+'.before')).read_bytes();assert data.decode('gbk').encode('gbk')==data
 encoding[n]={'before_lf':oldb.count(b'\n'),'before_crlf':oldb.count(b'\r\n'),'after_lf':data.count(b'\n'),'after_crlf':data.count(b'\r\n')}
 if n=='chassis_auto_test.h':assert data.count(b'\r\n')==oldb.count(b'\r\n')==1
 else:assert data.count(b'\n')==data.count(b'\r\n')
h=(U/'hwt101.h').read_text('gbk');assert '#define HWT_TIMING_SCOPES 3U' in h and '#define HWT_TIMING_FIELDS 28U' in h
report={'all_passed':True,'chip_cycles_measured':False,'hardware_accessed':False,'source_control_equivalence':True,'builds':comparisons,'debug_symbols':symbols,'scope_layout':{'header':{'enabled':0,'SystemCoreClock':4},'scope0':8,'scope1':56,'scope_bytes':48,'fields':{'valid':0,'count':4,'min_cycles':8,'last_cycles':12,'max_cycles':16,'sum_cycles_u64':24,'first_ms':32,'last_ms':36,'window_ms':40}},'encoding':encoding,'parser_synthetic_cases_passed':4,'matlab_process_exit_code':0}
(E/'final_verification.json').write_text(json.dumps(report,indent=2),'utf8')
print(json.dumps({'all_passed':True,'debug_symbols':symbols,'builds':comparisons},indent=2))
