from pathlib import Path
import hashlib,html,json,re,struct,subprocess
from parse_snapshot import parse

R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def allocated(path):
 b=path.read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b);ans=[]
 for i in range(h[12]):
  _,kind,flags,addr,off,size,*_=struct.unpack_from('<IIIIIIIIII',b,h[6]+i*h[11])
  if flags&2 and size:ans.append([kind,flags,addr,size,hashlib.sha256(bytes(size) if kind==8 else b[off:off+size]).hexdigest()])
 return ans

# Remove only exact recorded diagnostic insertions, require original bytes.
changes=json.loads((E/'source_changes.json').read_text('utf8'))
encoding={}
for name in ['chassis_auto_test.c','chassis_auto_test.h']:
 now=(U/name).read_bytes();old=(E/(name+'.before')).read_bytes();restored=now
 for change in reversed(changes):
  if change['file']==name:
   new=bytes.fromhex(change['new_hex']);assert restored.count(new)==1;restored=restored.replace(new,bytes.fromhex(change['old_hex']))
 assert restored==old,'Unexpected product control/source change'
 assert now.decode('gbk').encode('gbk')==now
 if name.endswith('.c'):assert now.count(b'\n')==now.count(b'\r\n')
 else:assert now.count(b'\r\n')==old.count(b'\r\n')==1
 encoding[name]=dict(lf=now.count(b'\n'),crlf=now.count(b'\r\n'),sha256=sha(U/name))

before=json.loads((E.parents[1]/'float-timing-20261115/build_summary.json').read_text('utf8'))
after=json.loads((E/'build_summary.json').read_text('utf8'));builds={};symbols={}
for name,record in after['builds'].items():
 assert record['returncode']==0 and sha(Path(record['axf']))==record['axf_sha256']
 prior=before['builds'][name]
 sizes=lambda v:[int(x) for x in re.search(r'Code=(\d+) RO-data=(\d+) RW-data=(\d+) ZI-data=(\d+)',v['program_size']).groups()]
 delta=[a-b for a,b in zip(sizes(record),sizes(prior))]
 builds[name]=dict(before=prior['program_size'],after=record['program_size'],size_delta=delta,allocated_identical=allocated(Path(record['axf']))==allocated(Path(prior['axf'])))
 sym=subprocess.check_output(['D:/Keil_v5/ARM/arm5_compiler/Bin/fromelf.exe','--text','-s',record['axf']]).decode(errors='replace')
 match=re.search(r'\bg_chassis_auto_tail_snapshots\s+(0x[0-9a-fA-F]+)\s+Gb\s+\d+\s+Data\s+\w+\s+(0x[0-9a-fA-F]+)',sym)
 if name.startswith('baseline40'):
  assert match and int(match[2],16)==176
  objectZI=lambda path:int(re.search(r'^\s+\d+\s+\d+\s+\d+\s+\d+\s+(\d+)\s+\d+\s+chassis_auto_test.o\s*$',path.read_text('gbk'),re.M)[1])
  paddingZI=lambda path:sum(int(x) for x in re.findall(r'^\s+\d+\s+\d+\s+\d+\s+\d+\s+(\d+)\s+\d+\s+\(incl. Padding\)',path.read_text('gbk'),re.M))
  newMap=Path(record['axf']).with_suffix('.map');oldMap=Path(prior['axf']).with_suffix('.map')
  objDelta=objectZI(newMap)-objectZI(oldMap);padDelta=paddingZI(newMap)-paddingZI(oldMap)
  assert objDelta==176 and delta[2]==0 and delta[3]==objDelta+padDelta
  builds[name]['RAM_detail']=dict(new_symbol_bytes=176,auto_object_ZI_delta=objDelta,linker_padding_ZI_delta=padDelta,whole_image_ZI_delta=delta[3])
  symbols[name]=dict(symbol='g_chassis_auto_tail_snapshots',address=match[1],bytes=176,axf=record['axf'],axf_sha256=record['axf_sha256'],getter_linked='ChassisAuto_ReadTailSnapshots' in sym,
    acquisition='Future explicitly authorized only: same session DONE/backend inactive/fault0 before and after two equal full176B reads; no halt/reset/VM/IRQ/getter; unstable otherwise')
 else:assert not match and builds[name]['allocated_identical']

a=json.loads((E/'actual_c_check.json').read_text('utf8'));b=json.loads((E/'actual_c_before_check.json').read_text('utf8'))
assert a['all_passed'] and b['all_passed']
assert a['tested_auto_sha256']==sha(U/'chassis_auto_test.c') and b['tested_auto_sha256']==sha(E/'chassis_auto_test.c.before')
assert a['source_sha256']['chassis_auto_test.c']==a['tested_auto_sha256'] and b['source_sha256']['chassis_auto_test.c']==b['tested_auto_sha256']
assert a['source_sha256']['chassis_auto_test.h']==b['source_sha256']['chassis_auto_test.h']==sha(U/'chassis_auto_test.h')
assert a['entry_observation']==b['entry_observation'] and a['ff_observation']==b['ff_observation']
assert (E/'before_distance_control.bin').read_bytes()==(E/'after_distance_control.bin').read_bytes()
zero=parse(bytes(176));assert not any(x['record_valid'] for x in zero['slots'])
dump=(E/'synthetic_snapshot_from_ARM.bin').read_bytes();decoded=parse(dump)
assert decoded['valid_mask']==7 and all(x['record_valid'] for x in decoded['slots'])
arrival=decoded['slots'][2];assert 14<arrival['reference_rpm']<16
assert not any(arrival['flags_decoded'][x] for x in ['reached_before_event','stop_due_before_event','finish_requested_before_event'])
assert arrival['follow_state_age_ms']>0
wrap=bytearray(dump);struct.pack_into('<4I',wrap,32,5,0xfffffff0,0xfffffffa,0xffffffff)
p=parse(wrap);assert p['slots'][0]['elapsed_ms']==21 and p['slots'][0]['common_age_ms']==11 and p['slots'][0]['follow_state_age_ms']==6
nan=bytearray(dump);struct.pack_into('<I',nan,32+16,0x7fc00000);assert not parse(nan)['slots'][0]['record_valid']
stack={}
for stage,summary in [('before',before),('after',after)]:
 p=Path(summary['builds']['baseline40_integrated']['axf']).with_suffix('.htm');text=html.unescape(re.sub('<[^>]+>',' ',p.read_text('latin1')));text=re.sub(r'[ \t]+',' ',text)
 stack[stage]={}
 for name in ['ChassisAuto_BeginPreparedTranslation','Auto_DistanceCheckLocked','ChassisAuto_CanStep','Auto_TailSnapshotsResetLocked','Auto_TailSnapshotRecordLocked']:
  m=re.search(re.escape(name)+r' \(Thumb, \d+ bytes, Stack size (\d+) bytes[^\n]*\n(?: \[Stack\] Max Depth = (\d+) Call Chain = ([^\n]+))?',text)
  if m:stack[stage][name]=dict(local_bytes=int(m[1]),max_depth_bytes=int(m[2]) if m[2] else int(m[1]),call_chain=m[3])
for name in stack['before']:assert stack['before'][name]['local_bytes']==stack['after'][name]['local_bytes'] and stack['before'][name]['max_depth_bytes']==stack['after'][name]['max_depth_bytes']
assert stack['after']['Auto_TailSnapshotsResetLocked']['local_bytes']==0
(E/'synthetic_snapshot_from_ARM.decoded.json').write_text(json.dumps(decoded,indent=2),'utf8')
report=dict(all_passed=True,source_control_bytes_equivalent=True,hardware_accessed=False,B0='FAIL_NOT_IDENTIFIABLE',B1_enabled=False,
 actual_ARM=dict(candidate_cases=a['cases'],candidate_assertions=a['assertions'],baseline_cases=b['cases'],baseline_assertions=b['assertions'],distance_control_fields_identical=32*12,entry_FLC_equal=True,ff_observation_equal=True,external_calls_stubbed=True),
 layout=dict(header_bytes=32,slot_bytes=48,total_bytes=176,C_sizeof_pass=True,ELF_symbol_bytes=176),builds=builds,debug_symbols=symbols,encoding=encoding,
 parser_synthetic_tests=5,static_stack=stack,hardware_stack_watermark_verified=False,limits='ARM execution with external CAN/power/RTOS/source readiness stubs, no M4 wall-time or actual response/ground validation')
(E/'final_verification.json').write_text(json.dumps(report,indent=2),'utf8');print(json.dumps(report,indent=2))
