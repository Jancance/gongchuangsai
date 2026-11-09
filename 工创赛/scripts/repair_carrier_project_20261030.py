"""恢复被旧工程文件替换的carrier引用；仅改uvprojx，不改源或烧录。"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
EV=ROOT/'evidence/p2-arm/2026-10-30/carrier-project-repair'
PREVIOUS=ROOT/'evidence/p2-arm/2026-10-30/carrier-pb1'
STAGE=ROOT/'tmp/carrier_pb1_20261030/project/gongxun-f407.uvprojx'
ATTACHMENT=Path('D:/Codex/home/attachments/49bf3123-9c1e-46ba-8e04-20610229c39c/已粘贴的文本.txt')

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(name,value):
    (EV/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

m=json.loads((PREVIOUS/'installed.json').read_text(encoding='utf-8'))
entry=next(e for e in m['files'] if e['file']=='gongxun-f407.uvprojx')
p=ACTUAL/'gongxun-f407.uvprojx'
assert sha(STAGE)==entry['after'],'Validated project changed'
source_checks={e['file']:sha(ACTUAL/e['file'])==e['after'] for e in m['files']
               if e['file']!='gongxun-f407.uvprojx'}
assert all(source_checks.values()),source_checks
tree=ET.fromstring(STAGE.read_bytes())
includes=tree.findall('.//Cads/VariousControls/IncludePath')
assert any(r'.\HARDWARE\CARRIER' in (n.text or '').split(';') for n in includes)
for name in ['carrier.c','carrier_pwm.c','carrier.h','carrier_pwm.h','carrier_config.h']:
    assert sum(n.text==name for n in tree.findall('.//FileName'))==1,name

if '--repair' not in sys.argv:
    assert sha(p)==entry['before'],'Current project not identical to the known prior version'
    EV.mkdir(parents=True,exist_ok=True)
    assert not (EV/'original-gongxun-f407.uvprojx').exists()
    shutil.copyfile(p,EV/'original-gongxun-f407.uvprojx')
    shutil.copyfile(ATTACHMENT,EV/'user-pasted-main.txt')
    logfile=ACTUAL/'Objects/gongxun-f407.build_log.htm'
    if logfile.exists():shutil.copyfile(logfile,EV/'user-failed-build.htm')
    save('diagnosis.json',{
        'date':'2026-10-30','classification':'E project integration repair',
        'project_sha256_before':sha(p),'equals_pre_carrier_project':True,
        'validated_project_sha256':entry['after'],
        'carrier_module_and_main_match_previous_install':source_checks,
        'user_attachment_sha256':sha(ATTACHMENT),
        'user_main_equals_actual_after_decoding_and_newline_normalization':
            ATTACHMENT.read_text(encoding='utf-8-sig').replace('\r\n','\n').strip()==
            (ACTUAL/'USER/main.c').read_bytes().decode('gb18030').replace('\r\n','\n').strip(),
        'missing_project_entries':['C compiler include HARDWARE/CARRIER','five module references including carrier.c and carrier_pwm.c'],
        'how_project_was_reverted':'unknown; stale IDE save is a possibility, not proven',
        'planned_only_write':'gongxun-f407.uvprojx',
        'source_changes':False,'flash_performed':False,'hardware_commands_sent':False})
    print('Diagnosis saved; known validated replacement ready. No actual files written.')
    raise SystemExit(0)

d=json.loads((EV/'diagnosis.json').read_text(encoding='utf-8'))
assert sha(p)==d['project_sha256_before'],'Project changed after diagnosis; do not overwrite'
source_before={s.relative_to(ACTUAL).as_posix():sha(s) for s in ACTUAL.rglob('*')
               if s.is_file() and s.suffix.lower() in ('.c','.h')}
shutil.copyfile(STAGE,p)
assert sha(p)==entry['after']
result=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(p),'-j0','-o',str(EV/'actual-rebuild.log')],
                      capture_output=True,timeout=120)
text=(EV/'actual-rebuild.log').read_text(encoding='utf-8',errors='replace')
source_after={s.relative_to(ACTUAL).as_posix():sha(s) for s in ACTUAL.rglob('*')
              if s.is_file() and s.suffix.lower() in ('.c','.h')}
passed=(result.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',text) is not None
        and 'compiling carrier.c...' in text and 'compiling carrier_pwm.c...' in text
        and sha(p)==entry['after'] and source_before==source_after)
save('review.json',{
    'rebuild_pass':passed,'compiler_returncode':result.returncode,'project_sha256_after':sha(p),
    'project_matches_prior_validated':sha(p)==entry['after'],
    'source_files_preserved':len(source_before),'source_bytes_unchanged':source_before==source_after,
    'carrier_modules_compiled':['carrier.c','carrier_pwm.c'],
    'reran_motion_state_tests':False,'reason':'No C or H behavior changes; existing 44-case evidence retained.',
    'flash_performed':False,'hardware_commands_sent':False,'physical_trial_accepted':False,
    'requirements_R':['RUN-01','RUN-02','RUN-03','PWR-01','MECH-01','MECH-02'],
    'requirements_E':['RUN-06','LOC-03','CAL-05'],'official_Q':'unchanged; pending official clarification'})
assert passed,text[-4000:]
print(text[-800:],flush=True)
print('Fixed project only; all source bytes preserved. Close old project without saving and reopen this uvprojx.',flush=True)
