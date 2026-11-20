"""Main-E: two real target Rebuilds, outputs isolated under the E workspace."""
from pathlib import Path
import ctypes,hashlib,json,re,subprocess
R=Path(__file__).resolve().parents[2];T=R/'工创赛/tmp/debug_runtime_main_e';E=R/'工创赛/evidence/navigation-planning/2026-11-10/main-e';P=R/'3.stm32f407工程/gongxun-f407.uvprojx'
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,32768);return b.value
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 before=R/'工创赛/evidence/navigation-planning/2026-11-10/main-d/build_summary.json';d=json.loads(before.read_text('utf8'))
 for p,h in d['final_source_sha256'].items():
  frozen=T/('main.c' if p.endswith('/main.c') else 'project.uvprojx') if p.endswith(('/main.c','.uvprojx')) else R/p
  assert sha(frozen)==h,(p,'D final baseline mismatch')
 paths=[P]+[P.parent/'CHASSIS/USER'/n for n in ['main.c','app_debug_runtime.c','app_debug_runtime.h']]
 initial={str(p.relative_to(R)).replace('\\','/'):sha(p) for p in paths};targets={};projects={}
 for target in ('Vehicle_Integrated','Vehicle_Offline'):
  directory=T/'final'/target;directory.mkdir(parents=True,exist_ok=True)
  s=P.read_text('utf-8-sig').replace('.\\',short(P.parent)+'\\')
  for tag in ('OutputDirectory','ListingPath'):s=re.sub('<'+tag+'>.*?</'+tag+'>',lambda m:'<'+tag+'>final\\'+target+'\\</'+tag+'>',s)
  project=T/('build_final_'+target+'.uvprojx');project.write_text(s,'utf8');projects[str(project.relative_to(R)).replace('\\','/')]=sha(project)
  log=E/('final_'+target+'.log')
  cmd=[r'D:\Keil_v5\UV4\UV4.exe','-r',str(project),'-t',target,'-o',str(log)]
  p=subprocess.run(cmd,timeout=240);text=log.read_bytes().decode('gbk');assert p.returncode==0 and '0 Error(s), 0 Warning(s)' in text,(target,text)
  sizes=re.search(r'Program Size: Code=(\d+) RO-data=(\d+) RW-data=(\d+) ZI-data=(\d+)',text);assert sizes
  targets[target]=dict(before=d['targets'][target]['after'],after=dict(zip(['Code','RO','RW','ZI'],map(int,sizes.groups()))),errors=0,warnings=0,command=cmd,log_sha256=sha(log),artifact_sha256={str((directory/(target+x)).relative_to(R)).replace('\\','/'):sha(directory/(target+x)) for x in ['.axf','.map']})
  print(target,targets[target]['after'],flush=True)
 assert initial=={str(p.relative_to(R)).replace('\\','/'):sha(p) for p in paths}
 report=dict(baseline='3676d2dacd4889d7439663eeb691c061be387acb',before_reused='Main-D final Rebuild; raw main/project and serial owner SHA verified against D report',before_evidence=str(before.relative_to(R)).replace('\\','/'),before_evidence_sha256=sha(before),final_source_sha256=initial,targets=targets,build_driver_sha256=sha(Path(__file__)),build_projects_sha256=projects,hardware_accessed=False)
 (E/'build_summary.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),'utf8')
if __name__=='__main__':main()
