"""Main-E exact owner extraction; preserve literal text in token comparisons."""
from pathlib import Path
import ctypes,hashlib,json,re,subprocess,sys,xml.etree.ElementTree as ET
R=Path(__file__).resolve().parents[2];U=R/'3.stm32f407工程/CHASSIS/USER';T=R/'工创赛/tmp/debug_runtime_main_e';E=R/'工创赛/evidence/navigation-planning/2026-11-10/main-e';B=Path(r'D:\Keil_v5\ARM\arm5_compiler\Bin');BASE='3676d2dacd4889d7439663eeb691c061be387acb'
TOKEN=re.compile(r'(?P<comment>/\*.*?\*/|//[^\r\n]*)|(?P<literal>"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\')|(?P<word>[A-Za-z_][A-Za-z_0-9]*|(?:\d|\.\d)[A-Za-z_0-9.]*|>>=|<<=|\+\+|--|->|==|!=|<=|>=|&&|\|\||<<|>>|\+=|-=|\*=|/=|%=|&=|\|=|\^=|[^\s])',re.S)
def tokens(s):return [m[0] for m in TOKEN.finditer(s) if m.lastgroup!='comment']
def norm(s):
 t=tokens(s)
 return t[1:] if t and t[0]=='static' else t
def function_spans(s):
 out={}
 for m in re.finditer(r'^(?:static\s+)?(?:__inline\s+)?(?:void|uint32_t|uint16_t|uint8_t|int32_t|float|int)\s+(\w+)\([^;]*?\)\s*\{',s,re.M):
  start=s.index('{',m.start());depth=0
  for q in TOKEN.finditer(s,start):
   if q.lastgroup=='comment':continue
   depth+=(q[0]=='{')-(q[0]=='}')
   if depth==0:out[m[1]]=(m.start(),q.end());break
 return out
def functions(s):return {n:s[a:b] for n,(a,b) in function_spans(s).items()}
def globals(s):
 for a,b in sorted(function_spans(s).values(),reverse=True):s=s[:a]+s[b:]
 s=re.sub(r'^\s*#.*$','',s,flags=re.M);t=tokens(s);out=[];start=0
 for i,v in enumerate(t):
  if v==';':out.append(t[start:i+1]);start=i+1
 assert not t[start:],t[start:]
 return out
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,32768);return b.value
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def restore(s,units):
 f=units['AppDebug_HeadingObserve'];return s.replace('AppDebug_HeadingObserve();',f[f.index('{')+1:f.rindex('}')])
def blocks(old):
 watch=old[old.index('#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST\n/* 以下仅显示'):];watch=watch[:watch.index('#endif')+len('#endif\n')]
 heading=old[old.index('/* 仅虚拟数据的软件自检：'):old.index('/* 任务A：每100毫秒计数一次 */')]
 uart=old[old.index('#if APP_UART_LINK_TEST\n/* 仅测试数据：'):old.index('/* 独立接收任务：')]
 observe=old[old.index('                App_HeadingSourceObserve();'):];observe=observe[:observe.index('        #endif')]
 return watch,heading,uart,observe
def main():
 old=(T/'main.c').read_bytes().decode('gbk').replace('\r\n','\n');main=(U/'main.c').read_bytes().decode('gbk').replace('\r\n','\n');debug=(U/'app_debug_runtime.c').read_bytes().decode('gbk').replace('\r\n','\n')
 assert subprocess.check_output(['git','show',BASE+':3.stm32f407工程/CHASSIS/USER/main.c'],cwd=R).replace(b'\r\n',b'\n')==old.encode('gbk')
 before=functions(old);units=functions(main)|functions(debug)
 assert set(units)==set(before)|{'AppDebug_HeadingObserve'}
 for n,b in before.items():assert norm(restore(units[n],units))==norm(b),('raw function',n)
 declarations=globals(old);after=globals(main)+globals(debug)
 after.remove(tokens('extern volatile uint32_t g_vm_test_request;'))
 assert sorted(declarations)==sorted(after),'global declaration or initializer changed'
 watch,heading,uart,observe=blocks(old)
 restored=debug
 for n in ['App_HeadingDryRun','App_HeadingLiveStep','App_UartLinkByte','App_UartLinkPoll']:restored=restored.replace('void '+n,'static void '+n)
 for b in [watch,heading]:assert b in restored,'whole original block changed'
 assert uart.rstrip('\n') in restored,'original UART block changed except trailing empty lines'
 owned=sorted(set(re.findall(r'\bg_heading_(?:bus|publish|dry|live)_\w+|\bs_heading_live_\w+|\bg_uart_link_\w+|\bs_link_\w+',watch+heading+uart)))
 for n in owned:assert n not in main,('owner left in main',n)
 assert 'AppDebug_HeadingObserve();' in main
 guard='#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST\n                AppDebug_HeadingObserve();\n        #endif'
 assert guard in main
 targets=[];arm=[]
 for target in ET.parse(U.parents[1]/'gongxun-f407.uvprojx').getroot().findall('./Targets/Target'):
  name=target.findtext('TargetName');v=target.find('./TargetOption/TargetArmAds/Cads/VariousControls');opts=['--cpu','Cortex-M4.fp.sp','--c99']+['-D'+d for d in v.findtext('Define').split(',')]
  for p in v.findtext('IncludePath').split(';'):opts+=['-I',short((U.parents[1]/p).resolve())]
  parsed=[];preprocessed=[]
  for label,p in [('old',T/'main.c'),('main',U/'main.c'),('debug',U/'app_debug_runtime.c')]:
   q=subprocess.run([str(B/'armcc.exe')]+opts+['-E',short(p)],cwd=T,capture_output=True,timeout=60);assert q.returncode==0,q.stderr.decode(errors='replace')
   text=re.sub(r'^#.*$','',q.stdout.decode('gbk'),flags=re.M);(T/(name+'_'+label+'.i')).write_text(text,'utf8');preprocessed.append(text);parsed.append(functions(text))
  merged=parsed[1]|parsed[2];line_deltas=[]
  for n,b in parsed[0].items():
   a=merged[n]
   if n=='main':
    aa=re.findall(r'App_RtosAssertFailed\((\d+)\)',a);bb=re.findall(r'App_RtosAssertFailed\((\d+)\)',b);assert len(aa)==len(bb)==8
    line_deltas=[int(x)-int(y) for x,y in zip(aa,bb)];assert len(set(line_deltas))==1
    a=re.sub(r'App_RtosAssertFailed\((\d+)\)',lambda m:'App_RtosAssertFailed('+str(int(m[1])-line_deltas[0])+')',a)
   assert norm(a)==norm(b),(name,n)
  # Current configurations deliberately do not enable legacy UART/static preview.
  for n in ['App_HeadingLiveStep','App_UartLinkByte','App_UartLinkPoll']:assert n not in parsed[0] and n not in merged
  assert 'App_HeadingDryRun' in merged
  entries=[x.findtext('FilePath') for x in target.findall('./Groups/Group/Files/File')]
  for fn in ['main.c','app_debug_runtime.c']:assert entries.count('.\\CHASSIS\\USER\\'+fn)==1
  maps=T/'final'/name/(name+'.map');text=maps.read_bytes().decode('gbk');mapowners={}
  for n in owned:
   matches=re.findall(r'^\s+'+n+r'\s+0x[0-9a-fA-F]+\s+(?:Data|Number)\s+\d+\s+(\S+)',text,re.M)
   if matches:assert len(matches)==1 and matches[0].startswith('app_debug_runtime.o('),(name,n,matches);mapowners[n]=matches[0]
  assert set(mapowners)=={n for n in owned if n.startswith('g_heading_dry_')}
  targets.append(dict(target=name,preprocessed_functions=len(parsed[0]),assert_line_deltas=line_deltas,watch_map_owners=mapowners,map_sha256=sha(maps)))
  import check_debug_runtime_arm_20261110 as replay
  arm.append(dict(target=name,fixture=False,**replay.replay(sys.modules[__name__],old,debug,preprocessed[2],opts,name,False)))
  if name=='Vehicle_Integrated':arm.append(dict(target='Integrated compiler options + isolated UART/static preview fixture',fixture=True,**replay.replay(sys.modules[__name__],old,debug,preprocessed[2],opts,name+'_debug_fixture',True)))
 original=json.loads((T/'before_hashes.json').read_text('utf8'));allowed=['3.stm32f407工程/CHASSIS/USER/main.c','3.stm32f407工程/gongxun-f407.uvprojx'];assert len(original)==10396
 for p,h in original.items():
  if p not in allowed:assert (R/p).exists() and sha(R/p)==h,('protected',p)
 for n in ['stage1_runtime.inc','vehicle_mission.inc']:assert (T/n).read_bytes()==(U/n).read_bytes()
 for n in ['main.c','app_debug_runtime.c','app_debug_runtime.h']:
  b=(U/n).read_bytes();assert b.count(b'\n')==b.count(b'\r\n');assert b.decode('gbk').encode('gbk')==b
 paths=[U/'main.c',U/'app_debug_runtime.c',U/'app_debug_runtime.h',U.parents[1]/'gongxun-f407.uvprojx']
 scripts=[Path(__file__),R/'工创赛/scripts/check_debug_runtime_arm_20261110.py',R/'工创赛/scripts/build_debug_runtime_20261110.py']
 report=dict(ok=True,baseline=BASE,raw_functions=len(before),original_global_declarations=len(declarations),original_global_definitions=sum(d[0]!='extern' for d in declarations),owned_symbols=len(owned),literal_preserving_tokens=True,main_lines_before=old.count('\n'),main_lines_after=main.count('\n'),targets=targets,arm=arm,protected_existing_files=len(original)-2,protected_manifest_sha256=sha(T/'before_hashes.json'),inc_bytes_unchanged=True,source_sha256={str(p.relative_to(R)).replace('\\','/'):sha(p) for p in paths},script_sha256={str(p.relative_to(R)).replace('\\','/'):sha(p) for p in scripts},hardware_accessed=False,limits='Actual mode3 Integrated/Offline: dry self-test active; UART/static preview excluded by original macros. Separate extracted-C fixture enables those dormant bodies and supplies missing historical value.valid_mask input only in harness. Does not repair or validate a complete legacy profile. Actual controller and kinematics; controlled clock/publish/read/UART stubs. No MATLAB dynamics, RTOS scheduling/stack, physical bus or vehicle validation.')
 (E/'debug_runtime_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf8');print(json.dumps(report,ensure_ascii=False))
if __name__=='__main__':main()
