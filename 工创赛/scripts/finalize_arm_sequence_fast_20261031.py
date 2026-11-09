"""最终单文件修订：中止/故障/复位撤销夹持高度准入；仅实际增量构建。"""
from pathlib import Path
import hashlib,json,shutil,subprocess,re
ROOT=Path(__file__).resolve().parents[1]
A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=ROOT/'tmp/arm_sequence_fast_20261031/project'
EV=ROOT/'evidence/p2-arm/2026-10-31/arm-sequence-fast'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
m=json.loads((EV/'installed.json').read_text(encoding='utf-8'))
h=json.loads((EV/'host-review.json').read_text(encoding='utf-8'))
assert m['actual_build_pass'] and h['passed']
for e in m['files']:assert sha(A/e['file'])==e['after'],e['file']
for n,v in m['protected'].items():assert sha(A/n)==v,n
for e in m['files']:
    actual=sha(S/e['file'])
    assert actual==h['source_sha256'][e['file']]
    if e['file']!='USER/arm_sequence.c':assert actual==e['after']
assert not (EV/'installed-before-height-invalidation.json').exists()
shutil.copyfile(EV/'installed.json',EV/'installed-before-height-invalidation.json')
shutil.copyfile(A/'USER/arm_sequence.c',EV/'arm-sequence-before-height-invalidation.c')
shutil.copyfile(S/'USER/arm_sequence.c',A/'USER/arm_sequence.c')
for e in m['files']:e['after']=sha(S/e['file'])
m['stage_initial_revision_build_pass']=True
m['stage_build_pass']=False
m['final_revision']='fault/abort/reset invalidate height-ready; current height-ready withdrawal aborts'
m['actual_build_pass']=False
(EV/'installed.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
p=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(A/'gongxun-f407.uvprojx'),'-j0','-o',str(EV/'actual-final-build.log')],capture_output=True,timeout=120)
t=(EV/'actual-final-build.log').read_text(encoding='utf-8',errors='replace')
assert p.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',t),t[-4000:]
for e in m['files']:assert sha(A/e['file'])==e['after'],e['file']
for n,v in m['protected'].items():assert sha(A/n)==v,n
m['actual_build_pass']=True
m['final_native_scenarios']=h['new_sequence_scenarios']+h['new_gripper_scenarios']
m['final_revision_build_scope']='actual incremental build only; final native scenarios passed; no repeated full baseline suites'
for n in ['manifest.json','installed.json']:
 (EV/n).write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(t[-750:],flush=True)
print('Final safety invalidation installed;no flash/no hardware;old baseline suites not repeated.',flush=True)
