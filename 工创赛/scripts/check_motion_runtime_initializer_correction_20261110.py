"""C2c initializer correction on frozen 5111ab9/1f272dd inputs; never use D main."""
from pathlib import Path
from types import SimpleNamespace
import ctypes,hashlib,json,re,subprocess,xml.etree.ElementTree as ET
import check_motion_runtime_extraction_20261110 as source_check
import check_motion_runtime_arm_20261110 as arm

R=Path(__file__).resolve().parents[2];BASE='5111ab98019619e1b14a97c0b409f759a9662e1c';CANDIDATE='1f272dd8cae82bea606f4f779573ecf5e22d4cf1'
HIST=R/'工创赛/tmp/motion_runtime_main_c2c';OLD_E=R/'工创赛/evidence/navigation-planning/2026-11-10/main-c2c'
T=R/'工创赛/tmp/motion_runtime_initializer_correction_20261110';E=OLD_E/'initializer-correction-20261110';B=Path(r'D:\Keil_v5\ARM\arm5_compiler\Bin')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def digest(b):return hashlib.sha256(b).hexdigest()
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,32768);return b.value
def gitbytes(commit,path):return subprocess.check_output(['git','show',commit+':'+path],cwd=R)
def normal(b):return b.replace(b'\r\n',b'\n')
def tokens(s):return re.findall(r'"(?:\\.|[^"\\])*"|\w+|[^\s]',s)

def main():
 T.mkdir(exist_ok=True);E.mkdir(exist_ok=True)
 historical_paths=[OLD_E/n for n in ['motion_runtime_check.json','build_summary.json','review.json','independent_review.json']]+[HIST/'unit/unit.axf',HIST/'unit/unit.c']
 historical={str(p.relative_to(R)).replace('\\','/'):sha(p) for p in historical_paths}
 d_e=R/'工创赛/evidence/navigation-planning/2026-11-10/main-d';d_check=json.loads((d_e/'serial_runtime_check.json').read_text('utf8'))
 protected_d={**d_check['source_sha256'],**d_check['script_sha256']}
 protected_d.update({str(p.relative_to(R)).replace('\\','/'):sha(p) for p in d_e.iterdir() if p.is_file()})
 for p,h in protected_d.items():assert sha(R/p)==h,('D baseline',p)
 old_report=json.loads((OLD_E/'motion_runtime_check.json').read_text('utf8'));raw_baseline=json.loads((HIST/'before_hashes.json').read_text('utf8'))
 old=(HIST/'main.c').read_bytes();assert sha(HIST/'main.c')==raw_baseline['3.stm32f407工程/CHASSIS/USER/main.c']
 assert normal(old)==normal(gitbytes(BASE,'3.stm32f407工程/CHASSIS/USER/main.c'))
 (T/'baseline_main.c').write_bytes(old)
 candidate=T/'candidate';candidate.mkdir(exist_ok=True);frozen={}
 for p,h in old_report['source_sha256'].items():
  source=R/p
  if p.endswith('/main.c'):source=R/'工创赛/tmp/serial_runtime_main_d/main.c'
  if p.endswith('/gongxun-f407.uvprojx'):source=R/'工创赛/tmp/serial_runtime_main_d/project.uvprojx'
  data=source.read_bytes();assert digest(data)==h,('frozen C2c raw identity',p)
  assert normal(data)==normal(gitbytes(CANDIDATE,p)),('candidate Git identity',p)
  dest=candidate/p;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(data);frozen[p]=h
 u=candidate/'3.stm32f407工程/CHASSIS/USER'
 # Validate every tracked header/inc plus the one separately compiled helper.
 # Raw bytes also match C2c's original/candidate manifests; Git CRLF conversion
 # is explicitly separated from the exact raw-byte identity.
 tracked=subprocess.check_output(['git','ls-tree','-r','-z','--name-only',CANDIDATE,'--','3.stm32f407工程'],cwd=R).decode('utf8').strip('\0').split('\0')
 paths=[p for p in tracked if p.endswith(('.h','.inc')) or p.endswith('/CHASSIS/USER/heading_controller.c')]
 assert paths and any(p.endswith('/heading_controller.c') for p in paths),'missing dependency manifest'
 requests=''.join(CANDIDATE+':'+p+'\n' for p in paths).encode('utf8')
 raw=subprocess.check_output(['git','cat-file','--batch'],input=requests,cwd=R);offset=0;dependencies={}
 for p in paths:
  end=raw.index(b'\n',offset);head=raw[offset:end].split();assert head[1]==b'blob';size=int(head[2]);blob=raw[end+1:end+1+size];offset=end+1+size+1
  data=(R/p).read_bytes();assert normal(data)==normal(blob),('dependency Git mismatch',p)
  expected=old_report['source_sha256'].get(p,raw_baseline.get(p));assert expected and digest(data)==expected,('dependency raw mismatch',p)
  dependencies[p]={'raw_sha256':digest(data),'git_blob':head[0].decode(),'git_normalized_sha256':digest(normal(blob))}
 helper='3.stm32f407工程/CHASSIS/USER/heading_controller.c';(u/'heading_controller.c').write_bytes((R/helper).read_bytes());frozen[helper]=sha(u/'heading_controller.c')
 (T/'dependency_manifest.json').write_text(json.dumps(dependencies,indent=2,ensure_ascii=False),'utf8')
 texts=[(u/n).read_bytes().decode('gbk') for n in ['main.c','app_motion_runtime.c','app_runtime_route.c']]
 target_results=[];saved_pp={}
 for target in ET.parse(candidate/'3.stm32f407工程/gongxun-f407.uvprojx').getroot().findall('./Targets/Target'):
  name=target.findtext('TargetName');v=target.find('./TargetOption/TargetArmAds/Cads/VariousControls');opts=['--cpu','Cortex-M4.fp.sp','--c99']+['-D'+d for d in v.findtext('Define').split(',')]
  opts+=['-I',short(u)]
  for include in v.findtext('IncludePath').split(';'):opts+=['-I',short((R/'3.stm32f407工程'/include).resolve())]
  target_dir=T/name;target_dir.mkdir(exist_ok=True)
  for label,p in [('old',T/'baseline_main.c'),('main',u/'main.c'),('motion',u/'app_motion_runtime.c'),('route',u/'app_runtime_route.c')]:
   prior=HIST/(name+'_'+label+'.i');saved_pp[str(prior.relative_to(R)).replace('\\','/')]=sha(prior)
   run=subprocess.run([str(B/'armcc.exe')]+opts+['-E',short(p)],cwd=target_dir,capture_output=True,timeout=60);assert run.returncode==0,run.stderr.decode(errors='replace')
   current=re.sub(r'^#.*$','',run.stdout.decode('gbk'),flags=re.M)
   assert tokens(current)==tokens(prior.read_text('utf8')),('saved preprocessing changed',name,label)
   (target_dir/(label+'.i')).write_text(current,'utf8')
  context=SimpleNamespace(R=R,U=u,T=target_dir,B=B,short=short,functions=source_check.functions)
  result=arm.replay(context,old.decode('gbk'),texts,opts)
  assert sha(target_dir/'unit/unit.c')==sha(HIST/'unit/unit.c'),'original C scenarios changed'
  prior_result=next(row for row in old_report['arm_differential'] if row['target']==name)
  assert all(result[k]==val for k,val in prior_result.items() if k!='target'),('original scenario/count regression',name)
  result.update(target=name,artifact=str((target_dir/'unit/unit.axf').relative_to(R)).replace('\\','/'),artifact_sha256=sha(target_dir/'unit/unit.axf'),generated_c_sha256=sha(target_dir/'unit/unit.c'))
  target_results.append(result)
 for p,h in historical.items():assert sha(R/p)==h,('historical evidence modified',p)
 for p,h in protected_d.items():assert sha(R/p)==h,('D modified',p)
 scripts=[Path(__file__),R/'工创赛/scripts/check_motion_runtime_arm_20261110.py',R/'工创赛/scripts/check_motion_runtime_extraction_20261110.py']
 assert normal(scripts[2].read_bytes())==normal(gitbytes(CANDIDATE,'工创赛/scripts/check_motion_runtime_extraction_20261110.py')),'source parser helper changed'
 report={'ok':True,'baseline_commit':BASE,'candidate_commit':CANDIDATE,'correction':'Previous program-header-only loading omitted startup RAM copy initialization. The old dynamic results used an incorrect initial state; this separate two-target replay replaces those dynamic claims. Product source is unchanged.','supersedes_dynamic_claims':['motion_runtime_check.json.arm_differential','review.json ARM differential item','independent_review.json.read_only_arm_replay'],'unchanged_valid_evidence':['raw/preprocessed source equivalence and original global declarations','protected file hashes and two original inc byte identity','single action array owner and formal two-target Keil builds'],'historical_evidence_preserved_sha256':historical,'frozen_input_sha256':frozen,'baseline_main_sha256':digest(old),'saved_preprocessed_inputs_sha256':saved_pp,'dependency_count':len(dependencies),'dependency_manifest':str((T/'dependency_manifest.json').relative_to(R)).replace('\\','/'),'dependency_manifest_sha256':sha(T/'dependency_manifest.json'),'dependency_identity':'Raw bytes match the original C2c manifests; separately compared against candidate Git blobs after CRLF normalization. Only frozen candidate source/header copies and verified unchanged dependencies compile.','script_sha256':{str(p.relative_to(R)).replace('\\','/'):sha(p) for p in scripts},'targets':target_results,'D_product_and_evidence_unchanged':True,'D_preserved_sha256':protected_d,'prior_independent_probe':{'scope':'last-target retained artifact only, independently reported by root/high before this correction','artifact_sha256':'2c2753b9775c83a67bdd40e42100b15b63ec998f0414212b7e726ca04a907818','initialized_RW_bytes':636,'nonzero_RW_bytes':126,'cases':672,'assertions':44418},'self_review':'passed; helper changes confined to execution-address initialization and initializer/owner checks; C harness/scenarios unchanged','independent_review':'pending root high supplemental review','formal_firmware_rebuilt':False,'hardware_accessed':False,'network_accessed':False,'requirements_R':['RUN-01','RUN-02','FIELD-06','PWR-01','MECH-01','MECH-02'],'requirements_E':['LOC-03','LOC-05','MOT-02','RUN-06','SRC-04'],'open_Q':['FIELD-05: 待技术答疑确认','OBS-03: 待技术答疑确认'],'limits':['Same narrow actual ARM C with controlled backend/RTOS/clock/plan-generation/action-submission stubs.','Correct initialization is an offline validator correction, not MATLAB dynamics, hardware/RTOS timing/stack or vehicle validation.','No historical stages outside C2c are revalidated by this report.']}
 (E/'correction_report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),'utf8')
 print(json.dumps({'ok':True,'dependency_count':len(dependencies),'targets':[{k:v for k,v in x.items() if k!='loader_initialization'} for x in target_results],'historical_and_D_unchanged':True},ensure_ascii=False))
if __name__=='__main__':main()
