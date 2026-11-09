"""旧uvprojx反复恢复旧版时，建立独立工程入口，共用原源码，不烧录。"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
EV=ROOT/'evidence/p2-arm/2026-10-30/carrier-project-repair'
STAGE=ROOT/'tmp/carrier_pb1_20261030/project/gongxun-f407.uvprojx'
NEW=ACTUAL/'gongxun-f407-carrier.uvprojx'
OLD=ACTUAL/'gongxun-f407.uvprojx'
OPTIONS=ACTUAL/'gongxun-f407.uvoptx'
NEW_OPTIONS=ACTUAL/'gongxun-f407-carrier.uvoptx'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert not NEW.exists() and not NEW_OPTIONS.exists(),'Do not overwrite a user entry'
previous=json.loads((EV/'review.json').read_text(encoding='utf-8'))
assert sha(STAGE)==previous['project_sha256_after']
source_before={p.relative_to(ACTUAL).as_posix():sha(p) for p in ACTUAL.rglob('*')
               if p.is_file() and p.suffix.lower() in ('.c','.h')}
old_hash=sha(OLD)
shutil.copyfile(OLD,EV/'project-reverted-again.uvprojx')
shutil.copyfile(STAGE,NEW)
options_hash=None
if OPTIONS.exists():
    options_hash=sha(OPTIONS)
    shutil.copyfile(OPTIONS,EV/'original-debug-options.uvoptx')
    shutil.copyfile(OPTIONS,NEW_OPTIONS)
result=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(NEW),'-j0','-o',str(EV/'independent-entry-rebuild.log')],
                      capture_output=True,timeout=120)
text=(EV/'independent-entry-rebuild.log').read_text(encoding='utf-8',errors='replace')
source_after={p.relative_to(ACTUAL).as_posix():sha(p) for p in ACTUAL.rglob('*')
              if p.is_file() and p.suffix.lower() in ('.c','.h')}
passed=(result.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',text) is not None
        and 'compiling carrier.c...' in text and 'compiling carrier_pwm.c...' in text
        and sha(NEW)==sha(STAGE) and source_before==source_after)
summary={
    'independent_entry_rebuild_pass':passed,'current_project_entry':str(NEW),
    'new_project_sha256':sha(NEW),'validated_template_sha256':sha(STAGE),
    'old_project_reverted_again_after_successful_rebuild':old_hash!=previous['project_sha256_after'],
    'old_project_hash_before_entry_creation':old_hash,'old_project_hash_after_entry_creation':sha(OLD),
    'old_project_written_by_this_script':False,'new_options_path':str(NEW_OPTIONS),
    'debug_options_copied_from':str(OPTIONS),'debug_options_sha256_before_copy':options_hash,
    'source_files_preserved':len(source_before),'source_bytes_unchanged':source_before==source_after,
    'C_runtime_evidence':'prior 44 cases; no C/H changes',
    'flash_performed':False,'hardware_commands_sent':False,'physical_trial_accepted':False,
    'requirements_R':['RUN-01','RUN-02','RUN-03','PWR-01','MECH-01','MECH-02'],
    'requirements_E':['RUN-06','LOC-03','CAL-05'],'official_Q':'unchanged; pending official clarification'
}
(EV/'independent-entry-review.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
assert passed,text[-4000:]
print(text[-800:],flush=True)
print('Independent entry rebuilt; existing source shared and unchanged; no flash.',flush=True)
