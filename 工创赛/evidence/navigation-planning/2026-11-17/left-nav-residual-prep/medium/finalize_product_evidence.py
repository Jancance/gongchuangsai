from pathlib import Path
import hashlib,json,re,subprocess
R=Path('D:/Desktop/工创赛');U=R/'3.stm32f407工程/CHASSIS/USER';E=R/'工创赛/evidence/navigation-planning/2026-11-17/left-nav-residual-prep/medium';files=['app_config.h','app_odometry.c','app_odometry.h','app_icm_observer.c']
report=[]
for name in files:
 b=(U/name).read_bytes();old=(E/'product_source_before'/name).read_bytes();assert b.count(b'\n')==b.count(b'\r\n');assert b.decode('cp936').encode('cp936')==b
 report.append(dict(file=name,before_sha256=hashlib.sha256(old).hexdigest(),after_sha256=hashlib.sha256(b).hexdigest(),bytes=len(b),crlf=b.count(b'\r\n'),encoding='GBK exact original bytes and local ASCII edits'))
(E/'product_source_manifest.json').write_text(json.dumps(report,indent=2),'utf8')
paths=[str((U/n).relative_to(R)) for n in files];cp=subprocess.run(['git','-c','core.quotePath=false','diff','--',*paths],cwd=R,capture_output=True,check=True);(E/'product_source_diff.txt').write_bytes(cp.stdout)
subprocess.run(['git','-c','core.whitespace=cr-at-eol','diff','--check','--',*paths],cwd=R,check=True)
def function_bytes(b,name):
 t=b.decode('latin1');m=re.search(r'(?m)^void '+name+r'\(',t);assert m;i=t.index('{',m.end());depth=1;j=i+1
 while depth:
  if t[j]=='{':depth+=1
  if t[j]=='}':depth-=1
  j+=1
 return t[m.start():j]
before=(E/'product_source_before/app_odometry.c').read_bytes();after=(U/'app_odometry.c').read_bytes();assert function_bytes(before,'AppOdometry_ReadBaseline40')==function_bytes(after,'AppOdometry_ReadBaseline40')
b=(E/'product_source_before/app_icm_observer.c').read_bytes();a=(U/'app_icm_observer.c').read_bytes();assert b.replace(b'AppOdometry_ReadBaseline40(&wheel);',b'AppOdometry_ReadNavigation40(&wheel);')==a
manifest=json.loads((E/'input_sha256_before.json').read_text('utf8'));protected={p:h for p,h in manifest.items() if Path(p).name not in files};assert all(hashlib.sha256((R/p).read_bytes()).hexdigest()==h for p,h in protected.items())
fromelf='D:/Keil_v5/ARM/arm5_compiler/Bin/fromelf.exe';storage={}
for mode in ['nav_on','nav_off']:
 text=subprocess.check_output([fromelf,'--text','-s',str(E/'actual-c'/(mode+'.axf'))]).decode(errors='replace');lines=[s for s in text.splitlines() if 's_navigation40 ' in s or 's_navigation_left_scope ' in s];storage[mode]=lines
 if mode=='nav_on':assert len(lines)==2 and any('0x40' in s for s in lines) and any(s.endswith('0x4') for s in lines)
 else:assert not lines
checks=dict(status='PASS',raw_getter_unchanged=True,icm_only_getter_change=True,protected_inputs_unchanged=len(protected),snapshot_size_c_verified=64,storage=storage,hardware_accessed=False,git_written=False)
(E/'final_source_checks.json').write_text(json.dumps(checks,indent=2),'utf8');print(json.dumps(dict(product_files=report,checks=checks),indent=2))