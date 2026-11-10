"""Main-F: original function/init/global tokens and real-target preprocessing."""
from pathlib import Path
import json,re,subprocess,sys,xml.etree.ElementTree as ET
from check_debug_runtime_extraction_20261110 import tokens,norm,functions,globals,short,sha
R=Path(__file__).resolve().parents[2];U=R/'3.stm32f407工程/CHASSIS/USER';T=R/'工创赛/tmp/input_chassis_main_f';E=R/'工创赛/evidence/navigation-planning/2026-11-10/main-f';B=Path(r'D:\Keil_v5\ARM\arm5_compiler\Bin');BASE='d4d23490dfe74023c7043f209200fc29a6b60a75'
WRAPPERS={'AppChassis_CanInit':'','AppChassis_CanStep':'','AppChassis_SafetyCheck':'','AppInput_KeyStep':'','AppInput_ScanDisplay':'screen_start_tick'}
def restore(s,units):
 for name,arg in WRAPPERS.items():
  f=units[name];s=s.replace(name+'('+arg+');',f[f.index('{')+1:f.rindex('}')])
 return s
def main():
 old=(T/'main.c').read_bytes().decode('gbk').replace('\r\n','\n');texts={n:(U/n).read_bytes().decode('gbk').replace('\r\n','\n') for n in ['main.c','app_input_runtime.c','app_chassis_service.c']}
 assert subprocess.check_output(['git','show',BASE+':3.stm32f407工程/CHASSIS/USER/main.c'],cwd=R).replace(b'\r\n',b'\n')==old.encode('gbk')
 before=functions(old);units={n:b for s in texts.values() for n,b in functions(s).items()};assert set(units)==set(before)|set(WRAPPERS)
 for n,b in before.items():assert norm(restore(units[n],units))==norm(b),('function',n)
 decl=globals(old);after=[d for s in texts.values() for d in globals(s)]
 for _ in range(2):after.remove(tokens('extern volatile uint32_t g_vm_test_request;'))
 assert sorted(decl)==sorted(after),'global/initialization changed'
 owned=sorted(set(re.findall(r'\bg_start_\w+|\bs_start_\w+|\bg_scan_\w+|\bg_chassis_key_\w+|\bs_chassis_key_\w+|\bg_hwt_turn_\w+|\bs_hwt_turn_\w+',texts['app_input_runtime.c'])))
 for n in owned:assert n not in texts['main.c'],('owner remains main',n)
 # Exact project delta: add these two C units to each existing target, no settings rewrite.
 project=(T/'project.uvprojx').read_bytes();pattern=rb'(?P<block>\s*<File>\s*<FileName>app_debug_runtime.c</FileName>\s*<FileType>1</FileType>\s*<FilePath>[^<]*app_debug_runtime.c</FilePath>\s*</File>)'
 project,count=re.subn(pattern,lambda m:m[0]+m['block'].replace(b'app_debug_runtime.c',b'app_input_runtime.c')+m['block'].replace(b'app_debug_runtime.c',b'app_chassis_service.c'),project);assert count==2 and project==(U.parents[1]/'gongxun-f407.uvprojx').read_bytes()
 targets=[];arm=[]
 for target in ET.parse(U.parents[1]/'gongxun-f407.uvprojx').getroot().findall('./Targets/Target'):
  name=target.findtext('TargetName');v=target.find('./TargetOption/TargetArmAds/Cads/VariousControls');opts=['--cpu','Cortex-M4.fp.sp','--c99']+['-D'+d for d in v.findtext('Define').split(',')]
  for p in v.findtext('IncludePath').split(';'):opts+=['-I',short((U.parents[1]/p).resolve())]
  parsed=[];pp=[]
  for label,p in [('old',T/'main.c')]+[(n[:-2],U/n) for n in texts]:
   q=subprocess.run([str(B/'armcc.exe')]+opts+['-E',short(p)],cwd=T,capture_output=True,timeout=60);assert q.returncode==0,q.stderr.decode(errors='replace')
   s=re.sub(r'^#.*$','',q.stdout.decode('gbk'),flags=re.M);(T/(name+'_'+label+'.i')).write_text(s,'utf8');pp.append(s);parsed.append(functions(s))
  merged={n:b for p in parsed[1:] for n,b in p.items()};deltas=[]
  for n,b in parsed[0].items():
   a=restore(merged[n],merged)
   if n=='main':
    aa=re.findall(r'App_RtosAssertFailed\((\d+)\)',a);bb=re.findall(r'App_RtosAssertFailed\((\d+)\)',b);assert len(aa)==len(bb)==8;deltas=[int(x)-int(y) for x,y in zip(aa,bb)];assert len(set(deltas))==1
    a=re.sub(r'App_RtosAssertFailed\((\d+)\)',lambda m:'App_RtosAssertFailed('+str(int(m[1])-deltas[0])+')',a)
   assert norm(a)==norm(b),(name,n)
  entries=[x.findtext('FilePath') for x in target.findall('./Groups/Group/Files/File')]
  for fn in texts:assert entries.count('.\\CHASSIS\\USER\\'+fn)==1
  mp=T/'final'/name/(name+'.map');maps=mp.read_bytes().decode('gbk');owners={}
  for n in owned:
   matches=re.findall(r'^\s+'+n+r'\s+0x[0-9a-fA-F]+\s+(?:Data|Number)\s+\d+\s+(\S+)',maps,re.M)
   if matches:assert len(matches)==1 and matches[0].startswith('app_input_runtime.o('),(name,n,matches);owners[n]=matches[0]
  targets.append(dict(target=name,preprocessed_functions=len(parsed[0]),assert_line_deltas=deltas,input_map_owners=owners,map_sha256=sha(mp)))
  import check_input_chassis_arm_20261110 as replay
  arm.append(dict(target=name,fixture=False,**replay.replay(sys.modules[__name__],old,texts,pp,opts,name,'actual')))
  if name=='Vehicle_Integrated':
   for fixture in ['ordinary','competition','heading','stage1','hwt_turn','chassis_key','uart_static']:
    arm.append(dict(target=name+' isolated '+fixture+' fixture',fixture=True,**replay.replay(sys.modules[__name__],old,texts,pp,opts,name+'_'+fixture,fixture)))
 original=json.loads((T/'before_hashes.json').read_text('utf8'));allowed=['3.stm32f407工程/CHASSIS/USER/main.c','3.stm32f407工程/gongxun-f407.uvprojx'];assert len(original)==10397
 for p,h in original.items():
  if p not in allowed:assert (R/p).exists() and sha(R/p)==h,('protected',p)
 for n in ['stage1_runtime.inc','vehicle_mission.inc']:assert (T/n).read_bytes()==(U/n).read_bytes()
 assert project==(U.parents[1]/'gongxun-f407.uvprojx').read_bytes(),'active project changed during verification'
 paths=[U/n for n in ['main.c','app_input_runtime.c','app_input_runtime.h','app_chassis_service.c','app_chassis_service.h']]+[U.parents[1]/'gongxun-f407.uvprojx']
 for p in paths:
  b=p.read_bytes();assert b.count(b'\n')==b.count(b'\r\n'),(p,'CRLF')
  if p.name.startswith(('app_input_','app_chassis_')):assert b.endswith(b'\r\n') and not b.endswith(b'\r\n\r\n');assert not re.search(rb'[ \t]+\r\n',b)
 scripts=[Path(__file__),R/'工创赛/scripts/check_input_chassis_arm_20261110.py',R/'工创赛/scripts/build_input_chassis_20261110.py',R/'工创赛/scripts/check_debug_runtime_extraction_20261110.py']
 report=dict(ok=True,baseline=BASE,raw_functions=len(before),main_initializer_and_task_creation_tokens_equal=True,original_global_declarations=len(decl),original_global_definitions=sum(d[0]!='extern' for d in decl),input_owner_symbols=len(owned),literal_preserving_tokens=True,main_lines_before=old.count('\n'),main_lines_after=texts['main.c'].count('\n'),targets=targets,arm=arm,protected_existing_files=len(original)-2,protected_manifest_sha256=sha(T/'before_hashes.json'),inc_bytes_unchanged=True,project_exact_additions_only=True,source_sha256={str(p.relative_to(R)).replace('\\','/'):sha(p) for p in paths},script_sha256={str(p.relative_to(R)).replace('\\','/'):sha(p) for p in scripts},hardware_accessed=False,limits='Actual Integrated/Offline mode3 and separately labeled extracted-C fixtures. Controlled clock, input, radar, backend, mechanics, safety and serial stubs; call/order and state equivalence, not full RTOS/stack/hardware or MATLAB dynamics. Historical missing runtime inputs unchanged, no assertion that complete legacy profiles build.')
 (E/'input_chassis_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf8');print(json.dumps(report,ensure_ascii=False))
if __name__=='__main__':main()
