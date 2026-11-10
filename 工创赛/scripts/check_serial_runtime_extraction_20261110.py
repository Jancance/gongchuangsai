"""Main-D serial owner extraction: exact source/target equivalence and ARM replay."""
from pathlib import Path
import ctypes, hashlib, json, re, subprocess, sys, xml.etree.ElementTree as ET
R=Path(__file__).resolve().parents[2];U=R/'3.stm32f407工程/CHASSIS/USER';T=R/'工创赛/tmp/serial_runtime_main_d';E=R/'工创赛/evidence/navigation-planning/2026-11-10/main-d';B=Path(r'D:\Keil_v5\ARM\arm5_compiler\Bin');BASE='1f272dd8cae82bea606f4f779573ecf5e22d4cf1'
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,32768);return b.value
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def functions(text):
 found={}
 for m in re.finditer(r'^(?:static\s+)?(?:__inline\s+)?(?:void|uint32_t|uint16_t|uint8_t|int32_t|float|int)\s+(\w+)\([^;]*?\)\s*\{',text,re.M):
  end=text.index('{',m.start())+1;depth=1
  while depth:depth+=(text[end]=='{')-(text[end]=='}');end+=1
  found[m[1]]=text[m.start():end]
 return found
def norm(s):return re.sub(r'\s+','',s).removeprefix('static')
def restore(s,units):
 for n in ['AppSerial_Init','AppSerial_Telemetry']:
  f=units[n];s=s.replace(n+'();',f[f.index('{')+1:f.rindex('}')])
 return s
def main():
 old=(T/'main.c').read_bytes().decode('gbk');main=(U/'main.c').read_bytes().decode('gbk');serial=(U/'app_serial_runtime.c').read_bytes().decode('gbk')
 baseline=subprocess.check_output(['git','show',BASE+':3.stm32f407工程/CHASSIS/USER/main.c'],cwd=R)
 assert baseline.replace(b'\r\n',b'\n')==(T/'main.c').read_bytes().replace(b'\r\n',b'\n')
 before=functions(old);units=functions(main)|functions(serial)
 for name,body in before.items():assert norm(restore(units[name],units))==norm(body),('raw function',name)
 # Entire original declaration/function block, not just selected variables, remains exact.
 original_block=''.join(old.splitlines(True)[180:851])
 restored_serial=serial.replace('void App_SerialMotionByte','static void App_SerialMotionByte').replace('void App_SerialMotionSafety','static void App_SerialMotionSafety')
 assert original_block in restored_serial
 assert ''.join(old.splitlines(True)[108:117]) in serial
 owned=sorted(set(re.findall(r'\bs_serial_\w+|\bg_serial_motion_\w+',original_block)))
 for name in owned:assert name not in main,('owner still in main',name)
 targets=[];replays=[]
 for target in ET.parse(U.parents[1]/'gongxun-f407.uvprojx').getroot().findall('./Targets/Target'):
  name=target.findtext('TargetName');v=target.find('./TargetOption/TargetArmAds/Cads/VariousControls')
  opts=['--cpu','Cortex-M4.fp.sp','--c99']+['-D'+d for d in v.findtext('Define').split(',')]
  for p in v.findtext('IncludePath').split(';'):opts+=['-I',short((U.parents[1]/p).resolve())]
  parsed=[];preprocessed=[]
  for label,p in [('old',T/'main.c'),('main',U/'main.c'),('serial',U/'app_serial_runtime.c')]:
   q=subprocess.run([str(B/'armcc.exe')]+opts+['-E',short(p)],cwd=T,capture_output=True,timeout=60);assert q.returncode==0,q.stderr.decode(errors='replace')
   text=re.sub(r'^#.*$','',q.stdout.decode('gbk'),flags=re.M);(T/(name+'_'+label+'.i')).write_text(text,'utf8');preprocessed.append(text);parsed.append(functions(text))
  merged=parsed[1]|parsed[2];metadata=[]
  for n,b in parsed[0].items():
   a=restore(merged[n],merged)
   if n=='main':
    aa=re.findall(r'App_RtosAssertFailed\((\d+)\)',a);bb=re.findall(r'App_RtosAssertFailed\((\d+)\)',b);assert len(aa)==len(bb)==8
    metadata=[int(x)-int(y) for x,y in zip(aa,bb)];assert len(set(metadata))==1
    a=re.sub(r'App_RtosAssertFailed\((\d+)\)',lambda m:'App_RtosAssertFailed('+str(int(m[1])-metadata[0])+')',a)
   assert norm(a)==norm(b),(name,n)
  entries=[x.findtext('FilePath') for x in target.findall('./Groups/Group/Files/File')]
  for fn in ['main.c','app_serial_runtime.c']:assert entries.count('.\\CHASSIS\\USER\\'+fn)==1
  mapfile=T/'final'/(name+'.map');maps=mapfile.read_bytes().decode('gbk')
  mapowners={}
  for symbol in owned:
   matches=re.findall(r'^\s+'+symbol+r'\s+0x[0-9a-fA-F]+\s+(?:Data|Number)\s+\d+\s+(\S+)',maps,re.M)
   if matches:assert len(matches)==1 and matches[0].startswith('app_serial_runtime.o('),(name,symbol,matches);mapowners[symbol]=matches[0]
  for required in ['s_serial_tx_seq','s_serial_tx_tick','s_serial_token_seq','g_serial_motion_state']:assert required in mapowners,(name,required)
  targets.append(dict(target=name,functions=len(parsed[0]),assert_source_line_deltas=metadata,map_owners=mapowners,map_sha256=sha(mapfile)))
  import check_serial_runtime_arm_20261110 as arm
  replays.append(dict(target=name,fixture=False,**arm.replay(sys.modules[__name__],old,serial,preprocessed[2],opts,name,False)))
  if name=='Vehicle_Integrated':replays.append(dict(target='Vehicle_Integrated compile options + isolated nonintegrated command fixture',fixture=True,**arm.replay(sys.modules[__name__],old,serial,preprocessed[2],opts,name+'_protocol_fixture',True)))
 manifest=T/'before_hashes.json';original=json.loads(manifest.read_text('utf8'));allowed=['3.stm32f407工程/CHASSIS/USER/main.c','3.stm32f407工程/gongxun-f407.uvprojx']
 assert len(original)==10394 and sha(T/'main.c')==original[allowed[0]] and sha(T/'project.uvprojx')==original[allowed[1]]
 for p,h in original.items():
  if p not in allowed:assert (R/p).exists() and sha(R/p)==h,('protected',p)
 for n in ['stage1_runtime.inc','vehicle_mission.inc']:assert (T/n).read_bytes()==(U/n).read_bytes()
 paths=[U/'main.c',U/'app_serial_runtime.c',U/'app_serial_runtime.h',U.parents[1]/'gongxun-f407.uvprojx']
 report=dict(ok=True,baseline=BASE,raw_functions=len(before),moved_functions=len(functions(serial))-2,entire_original_serial_block_preserved=True,owned_global_symbols=len(owned),targets=targets,arm=replays,protected_existing_files=len(original)-2,protected_manifest_sha256=sha(manifest),inc_bytes_unchanged=True,source_sha256={str(p.relative_to(R)).replace('\\','/'):sha(p) for p in paths},script_sha256={str(p.relative_to(R)).replace('\\','/'):sha(p) for p in [Path(__file__),R/'工创赛/scripts/check_serial_runtime_arm_20261110.py']},hardware_accessed=False,limits='Actual serial C and formatting; controlled backend/clock/UART/other-owner stubs. Actual Integrated/Offline plus explicitly separate nonintegrated fixture for dormant protocol branches. No physical timing, stack, bus or vehicle validation.')
 (E/'serial_runtime_check.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),'utf8');print(json.dumps(report,ensure_ascii=False))
if __name__=='__main__':main()
