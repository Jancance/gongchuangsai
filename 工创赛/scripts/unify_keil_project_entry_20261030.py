"""按用户要求恢复唯一gongxun-f407入口；归档重复入口，保留新调试设置。"""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import re

ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
EV=ROOT/'evidence/p2-arm/2026-10-30/project-entry-unification'
STAGE=ROOT/'tmp/project_entry_unification_20261030/project'
MAIN='gongxun-f407'
EXTRA='gongxun-f407-carrier'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(name,data):
    (EV/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def sources(folder):
    return {p.relative_to(folder).as_posix():sha(p) for p in folder.rglob('*')
            if p.is_file() and p.suffix.lower() in ('.c','.h')}
def validate_project(path):
    tree=ET.fromstring(path.read_bytes())
    includes=tree.findall('.//Cads/VariousControls/IncludePath')
    assert any(r'.\HARDWARE\CARRIER' in (p.text or '').split(';') for p in includes)
    for name in ['carrier.c','carrier_pwm.c','carrier.h','carrier_pwm.h','carrier_config.h']:
        assert sum(p.text==name for p in tree.findall('.//FileName'))==1,name
    # All referenced files must resolve in this project directory.
    for p in tree.findall('.//FilePath'):
        rel=(p.text or '').replace('\\','/')
        assert (path.parent/rel).is_file(),rel
def build(folder,log):
    result=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(folder/(MAIN+'.uvprojx')),
                           '-j0','-o',str(log)],capture_output=True,timeout=120)
    text=log.read_text(encoding='utf-8',errors='replace')
    assert result.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',text),text[-4000:]
    assert 'compiling carrier.c...' in text and 'compiling carrier_pwm.c...' in text
    print(text[-650:],flush=True)

if '--install' not in sys.argv:
    assert not STAGE.exists()
    EV.mkdir(parents=True,exist_ok=True)
    a=ACTUAL/(MAIN+'.uvprojx');b=ACTUAL/(EXTRA+'.uvprojx')
    validate_project(a);validate_project(b)
    assert a.read_bytes()==b.read_bytes(),'Review project differences instead of silently overwriting'
    # Preserve initial evidence; final live settings are backed up again at install.
    initial=EV/'initial'
    initial.mkdir(exist_ok=True)
    for p in ACTUAL.glob('gongxun-f407*'):
        if p.is_file():shutil.copyfile(p,initial/p.name)
    shutil.copytree(ACTUAL,STAGE,ignore=shutil.ignore_patterns(
        'Objects','Listings','.git','.vscode','*.uvguix.*'))
    # Stage uses the recent carrier debug options under the one canonical name.
    shutil.copyfile(ACTUAL/(EXTRA+'.uvoptx'),STAGE/(MAIN+'.uvoptx'))
    m={'canonical_project':MAIN+'.uvprojx','project_sha256':sha(a),
       'initial_duplicate_project_identical':True,'source_sha256':sources(ACTUAL),
       'settings_source':EXTRA+'.uvoptx','stage_rebuild_pass':False,
       'assistant_flash_performed':False,'assistant_hardware_commands':False}
    save('manifest.json',m)
    build(STAGE,EV/'stage-rebuild.log')
    assert sources(STAGE)==m['source_sha256']
    validate_project(STAGE/(MAIN+'.uvprojx'))
    m['stage_rebuild_pass']=True;save('manifest.json',m)
    print('Single-entry stage rebuilt; original project and live settings untouched.',flush=True)
    raise SystemExit(0)

m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
assert m['stage_rebuild_pass']
current_sources=sources(ACTUAL)
if current_sources!=m['source_sha256']:
    assert current_sources.keys()==m['source_sha256'].keys(),'Source file set changed; review required'
    changed=[n for n,h in current_sources.items() if h!=m['source_sha256'][n]]
    for n in changed:
        old=(STAGE/n).read_bytes();new=(ACTUAL/n).read_bytes()
        assert old.replace(b'\r',b'')==new.replace(b'\r',b''),('User code changed beyond line endings',n)
    save('manifest-before-ide-save.json',m)
    save('ide-save-line-ending-audit.json',{'files':changed,'only_CR_bytes_changed':True,
                                           'user_file_preserved':True})
    for n in changed:
        before=EV/'before-ide-line-ending-save'/n
        after=EV/'after-ide-line-ending-save'/n
        before.parent.mkdir(parents=True,exist_ok=True);after.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(STAGE/n,before);shutil.copyfile(ACTUAL/n,after)
        shutil.copyfile(ACTUAL/n,STAGE/n)
    m['source_sha256']=current_sources
    m['stage_rebuild_pass']=False
    save('manifest.json',m)
    build(STAGE,EV/'stage-after-ide-save-rebuild.log')
    assert sources(STAGE)==current_sources
    m['stage_rebuild_pass']=True
    save('manifest.json',m)
assert sources(ACTUAL)==m['source_sha256'],'User source changed; re-audit before merge'
a=ACTUAL/(MAIN+'.uvprojx');b=ACTUAL/(EXTRA+'.uvprojx')
validate_project(a);validate_project(b)
assert a.read_bytes()==b.read_bytes(),'User project entries diverged; review instead of overwrite'
assert sha(a)==m['project_sha256'],'Project changed after staged build'
backup=EV/'before-merge';archive=EV/'archived-duplicate-entry'
backup.mkdir(exist_ok=True);archive.mkdir(exist_ok=True)
entries=[p for p in ACTUAL.glob('gongxun-f407*') if p.is_file()]
for p in entries:
    target=backup/p.name
    assert not target.exists(),'Do not overwrite merge backups'
    shutil.copyfile(p,target)
copies=[]
for suffix in ['.uvoptx','.uvguix.Tenderness']:
    source=ACTUAL/(EXTRA+suffix)
    if source.exists():
        ET.fromstring(source.read_bytes())
        target=ACTUAL/(MAIN+suffix)
        shutil.copyfile(source,target)
        copies.append({'from':source.name,'to':target.name,'sha256':sha(target)})
# Move only explicitly enumerated files, with absolute source/destination containment checked.
moved=[]
for source in entries:
    if not source.name.startswith(EXTRA+'.'):continue
    src=source.resolve();dst=(archive/source.name).resolve()
    assert src.parent==ACTUAL.resolve() and src.is_file()
    assert dst.parent==archive.resolve() and not dst.exists()
    original=sha(src)
    shutil.move(str(src),str(dst))
    assert sha(dst)==original and not src.exists()
    moved.append({'name':source.name,'sha256':original,'archive':str(dst)})
assert [p.name for p in ACTUAL.glob('*.uvprojx')]==[MAIN+'.uvprojx']
assert not list(ACTUAL.glob(EXTRA+'.*'))
build(ACTUAL,EV/'actual-rebuild.log')
validate_project(a)
assert sha(a)==m['project_sha256']
assert sources(ACTUAL)==m['source_sha256']
assert [p.name for p in ACTUAL.glob('*.uvprojx')]==[MAIN+'.uvprojx']
review={'date':'2026-10-30','single_root_project':MAIN+'.uvprojx',
        'project_sha256':sha(a),'source_files_unchanged':len(m['source_sha256']),
        'source_bytes_unchanged':True,'stage_rebuild_pass':True,'actual_rebuild_pass':True,
        'settings_copied':copies,'duplicate_files_archived':moved,
        'runtime_tests_repeated':False,'reason':'No C/H source or compiler project change; project filenames and debug settings only',
        'nominal_slot_targets_preserved':[0,120,240],
        'flash_performed':False,'hardware_commands_sent':False,
        'physical_zero_slot_alignment_verified':False,
        'requirements_R':['RUN-01','RUN-02','RUN-03','PWR-01','MECH-01','MECH-02'],
        'requirements_E':['RUN-06','LOC-03','CAL-05'],'official_Q':'unchanged; pending official clarification'}
save('review.json',review)
print('One canonical entry remains; latest debug settings preserved; duplicate entry archived. Not flashed.',flush=True)
