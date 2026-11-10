"""C2c behavior-preserving motion/route owner extraction checks (no hardware)."""
from pathlib import Path
import ctypes,hashlib,json,re,subprocess,sys,xml.etree.ElementTree as ET
R=Path(__file__).resolve().parents[2];U=R/'3.stm32f407工程/CHASSIS/USER';T=R/'工创赛/tmp/motion_runtime_main_c2c';E=R/'工创赛/evidence/navigation-planning/2026-11-10/main-c2c';B=Path(r'D:\Keil_v5\ARM\arm5_compiler\Bin');BASE='5111ab98019619e1b14a97c0b409f759a9662e1c'
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,32768);return b.value
def functions(text):
 found={}
 for m in re.finditer(r'^(?:static\s+)?(?:__inline\s+)?(?:void|uint32_t|uint16_t|uint8_t|int32_t|float|int)\s+(\w+)\([^;]*?\)\s*\{',text,re.M):
  e=text.index('{',m.start())+1;d=1
  while d:d+=(text[e]=='{')-(text[e]=='}');e+=1
  found[m[1]]=text[m.start():e]
 return found
FIELDS={'action_count':'s_motion_action_count','session_open':'s_motion_session_open','has_action':'s_motion_has_action','turn':'s_combined_turn','mode':'s_combined_mode','base_rpm':'s_combined_base_rpm','distance_mm':'s_combined_distance_mm','reference_mm_s':'s_combined_reference_mm_s','begin_ms':'s_combined_begin_ms','end_ms':'s_combined_end_ms','stage_ms':'s_combined_stage_ms'}
def restore(a,units,pp=False):
 for k,v in FIELDS.items():a=a.replace('(*AppMotion_Live()->'+k+')','('+v+')' if pp and k=='action_count' else v)
 a=a.replace('AppMotion_Live()->actions','s_motion_actions').replace('AppMotion_Live()->perimeter_targets','s_perimeter_targets')
 a=a.replace('AppRuntimeRoute_Live()->','s_runtime_route.')
 for n in ['token_seq','token_tick','arm_ms','tx_valid','tx_seq']:a=a.replace('(*AppSerial_Live()->'+n+')','s_serial_'+n)
 a=re.sub(r'AppMotion_SetStartPermit\(([01]U)\);',r's_serial_start_permit=\1;',a)
 a=a.replace('AppSerial_SetKeepalive(now);','s_serial_keepalive_ms=now;').replace('AppSerial_ResetToken();','s_serial_token_seq=s_serial_token_tick=s_serial_arm_ms=s_serial_keepalive_ms=0U;')
 a=a.replace('AppRuntimeRoute_Install(p);','RuntimeRoute_Install(&s_runtime_route,p);').replace('AppRuntimeRoute_Init();','RuntimeRoute_Init(&s_runtime_route);')
 for new,old in [('AppMotion_Stage1Service','App_Stage1Service'),('AppMotion_Stage1Display','App_Stage1Display'),('AppMotion_Stage1Telemetry','App_Stage1Telemetry'),('AppMotion_Stage1CodeTelemetry','App_Stage1CodeTelemetry'),('AppMotion_MissionService','VehicleMission_Service'),('AppMotion_MissionTelemetry','VehicleMission_Telemetry')]:a=a.replace(new+'(',old+'(')
 for name in ['AppMotion_InitOdometry','AppMotion_StepOdometry','AppMotion_LoadDefaultPlan','AppMotion_InitStage1Plan']:
  if name+'();' in a:
   fn=units[name];body=fn[fn.index('{')+1:fn.rindex('}')]
   if name=='AppMotion_InitOdometry':body=re.sub(r'\s*s_odometry_inputs.runtime_route_id = &AppRuntimeRoute_Live\(\)->committed.id;','',body)
   if not pp:body=re.sub(r'^#(?:if|endif).*\n','',body,flags=re.M)
   a=a.replace(name+'();',body)
 if pp:
  a=a.replace('((s_combined_distance_mm > 0.0f) ? (4U + s_combined_mode) : s_combined_mode)', '(((s_combined_distance_mm) > 0.0f) ? (4U + (s_combined_mode)) : (s_combined_mode))')
 if not pp:
  a=a.replace('(*AppMotion_Live()->action_count)','MOTION_ACTION_COUNT')
  # The restored expression is the exact expansion of the original local macros.
  for expr,var in [('App_StageDistanceMm()','s_combined_distance_mm'),('App_TestTranslationMode()','s_combined_mode')]:a=a.replace(expr,var)
 return a
def norm(a):return re.sub(r'\s+','',a).removeprefix('static')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def ownership(old,texts):
 # Compare original storage/initializers, including the state omitted by function checks.
 types='uint32_t|float|MotionAction|RuntimeRoute|RuntimeRoutePrepared|RouteHeadingReference|AppHeadingRuntimeInputs|AppTranslationRuntimeInputs|AppOdometryInputs|CombinedTestDebug'
 pattern=r'^(?:static\s+)?(?:const\s+)?(?:volatile\s+)?(?:'+types+r')\s+([^;]+);'
 def declarations(s):
  result={}
  for m in re.finditer(pattern,s,re.M):
   first=re.match(r'(\w+)',m[1])
   if not first or '(' in m[1].split('=')[0]:continue
   if first[1].startswith(('s_','g_')):result[first[1]]=m[0]
  return result
 before=declarations(old);motion=declarations(texts[1][:texts[1].index('static void App_TimingRecordCompleted')]);route=declarations(texts[2][:texts[2].index('uint32_t App_RuntimeRouteEditable')])
 for name,d in (motion|route).items():
  b=before[name]
  if name=='s_odometry_inputs':b=b.replace('&s_runtime_route.committed.id','NULL')
  assert re.sub(r'\s+','',d)==re.sub(r'\s+','',b),('state initializer',name)
 header=(U/'app_motion_runtime.h').read_bytes().decode('gbk')
 typedef=re.search(r'typedef struct\s*\{[^}]+\}\s*CombinedTestDebug;',old)[0]
 assert norm(typedef)==norm(re.search(r'typedef struct\s*\{[^}]+\}\s*CombinedTestDebug;',header)[0])
 declaration='static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];'
 assert texts[1].count(declaration)==1 and 's_motion_actions' not in header
 assert 's_motion_actions' not in texts[0] and 's_motion_actions' not in texts[2]
 assert '#if APP_HEADING_SESSION_TEST' in texts[1][:texts[1].index(declaration)]
 assert texts[1].index(declaration)<texts[1].index('static uint32_t s_motion_action_count;')
 maps=[]
 for target in ['Vehicle_Integrated','Vehicle_Offline']:
  p=T/'final'/(target+'.map');matches=re.findall(r'^\s+s_motion_actions\s+0x[0-9a-fA-F]+\s+Data\s+(\d+)\s+(\S+)',p.read_bytes().decode('gbk'),re.M)
  assert matches==[('192','app_motion_runtime.o(.bss)')],(target,matches)
  maps.append(dict(target=target,owner='app_motion_runtime.o',bytes=192,map_sha256=sha(p)))
 return dict(state_declarations=len(motion|route),original_storage_initializers=True,action_array_unique_owner=maps)
def main():
 old=(T/'main.c').read_bytes().decode('gbk');texts=[(U/n).read_bytes().decode('gbk') for n in ['main.c','app_motion_runtime.c','app_runtime_route.c']];units={k:v for s in texts for k,v in functions(s).items()};before=functions(old)
 gitold=subprocess.check_output(['git','show',BASE+':3.stm32f407工程/CHASSIS/USER/main.c'],cwd=R)
 assert gitold.replace(b'\r\n',b'\n')==(T/'main.c').read_bytes().replace(b'\r\n',b'\n'),'baseline main identity'
 for n,b in before.items():
  a=restore(units[n],units)
  # Explicit only: original local aliases expand to the same owner variables.
  a=a.replace('MOTION_ACTION_COUNT','s_motion_action_count')
  b=b.replace('MOTION_ACTION_COUNT','s_motion_action_count').replace('App_StageDistanceMm()','s_combined_distance_mm').replace('App_TestTranslationMode()','s_combined_mode')
  if n in ['App_TestTaskA','App_CanTask']:
   b=re.sub(r'^#(?:if|endif).*\n','',b,flags=re.M);a=re.sub(r'^#(?:if|endif).*\n','',a,flags=re.M)
  assert norm(a)==norm(b),('raw',n)
 pp=[];arm=[]
 for target in ET.parse(U.parents[1]/'gongxun-f407.uvprojx').getroot().findall('./Targets/Target'):
  name=target.findtext('TargetName');ctrl=target.find('./TargetOption/TargetArmAds/Cads/VariousControls');opts=['--cpu','Cortex-M4.fp.sp','--c99']+['-D'+d for d in ctrl.findtext('Define').split(',')]
  for inc in ctrl.findtext('IncludePath').split(';'):opts+=['-I',short((U.parents[1]/inc).resolve())]
  parsed=[]
  for tag,p in [('old',T/'main.c'),('main',U/'main.c'),('motion',U/'app_motion_runtime.c'),('route',U/'app_runtime_route.c')]:
   run=subprocess.run([str(B/'armcc.exe')]+opts+['-E',short(p)],cwd=T,capture_output=True,timeout=60);assert run.returncode==0,run.stderr.decode(errors='replace');s=re.sub(r'^#.*$','',run.stdout.decode('gbk'),flags=re.M);(T/(name+'_'+tag+'.i')).write_text(s,'utf8');parsed.append(functions(s))
  merged={k:v for d in parsed[1:] for k,v in d.items()};meta=[]
  for n,b in parsed[0].items():
   a=restore(merged[n],merged,True)
   if n=='main':
    aa=re.findall(r'App_RtosAssertFailed\((\d+)\)',a);bb=re.findall(r'App_RtosAssertFailed\((\d+)\)',b);assert len(aa)==len(bb)==8;meta=[int(x)-int(y) for x,y in zip(aa,bb)];a=re.sub(r'App_RtosAssertFailed\((\d+)\)',lambda m:'App_RtosAssertFailed('+bb[aa.index(m[1])]+')',a)
   assert norm(a)==norm(b),(name,n)
  pp.append(dict(target=name,functions=len(parsed[0]),assert_source_line_deltas=meta))
  import check_motion_runtime_arm_20261110 as replay
  result=replay.replay(sys.modules[__name__],old,texts,opts);arm.append(dict(target=name,**result))
 for n in ['stage1_runtime.inc','vehicle_mission.inc']:
  assert (T/n).read_bytes()==(U/n).read_bytes(),n
  baseline=subprocess.check_output(['git','show',BASE+':3.stm32f407工程/CHASSIS/USER/'+n],cwd=R)
  assert baseline.replace(b'\r\n',b'\n')==(T/n).read_bytes().replace(b'\r\n',b'\n'),('baseline inc',n)
 state=ownership(old,texts)
 manifest=T/'before_hashes.json';original=json.loads(manifest.read_text('utf8'))
 allowed=['3.stm32f407工程/CHASSIS/USER/main.c','3.stm32f407工程/gongxun-f407.uvprojx']
 assert len(original)==10389 and all(p in original for p in allowed)
 assert sha(T/'main.c')==original[allowed[0]] and sha(T/'project.uvprojx')==original[allowed[1]],'original manifest/snapshot identity'
 for p,h in original.items():
  if p not in allowed:assert (R/p).exists() and sha(R/p)==h,('protected file',p)
 sources={str(p.relative_to(R)).replace('\\','/'):sha(p) for p in [U/n for n in ['main.c','app_motion_runtime.c','app_motion_runtime.h','app_runtime_route.c','app_runtime_route.h','app_serial_internal.h','stage1_runtime.inc','vehicle_mission.inc']]+[U.parents[1]/'gongxun-f407.uvprojx']}
 report=dict(ok=True,baseline=BASE,raw_main_functions=len(before),preprocessed_targets=pp,arm_differential=arm,ownership=state,protected_existing_files=len(original)-len(allowed),protected_manifest_sha256=sha(manifest),original_inc_bytes_unchanged=True,source_sha256=sources,hardware_accessed=False,limits='Actual selected application C and exact API bridges; backend, RTOS/clock, plan generation and action submission are controlled stubs. Both real mode3 targets only; no MATLAB dynamics, physical timing, stack or vehicle validation.')
 (E/'motion_runtime_check.json').write_text(json.dumps(report,indent=2),'utf8');print(json.dumps(report))
if __name__=='__main__':main()
