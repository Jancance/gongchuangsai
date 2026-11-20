from pathlib import Path
import json,hashlib,subprocess
R=Path.cwd();E=R/'工创赛/evidence/navigation-planning/2026-11-17/turn90-90-prep';U=R/'3.stm32f407工程/CHASSIS/USER';(E/'source_before').mkdir(exist_ok=True)
names=['app_config.h','app_entry.h','app_entry.c','app_motion_runtime.c','app_serial_runtime.c','turn_brake.h','heading_session.c'];enc={}
for n in names:
 b=(U/n).read_bytes();(E/'source_before'/n).write_bytes(b)
 try:b.decode('utf-8');enc[n]='utf-8'
 except UnicodeDecodeError:b.decode('gbk');enc[n]='gbk'
 assert b.count(b'\r\n')==b.count(b'\n')
(E/'encoding_before.json').write_text(json.dumps(enc,indent=2),encoding='utf-8')
dirty=subprocess.check_output(['git','diff','--name-only','-z']).decode('utf-8').strip('\0').split('\0');(E/'original_dirty_sha256.json').write_text(json.dumps({p:hashlib.sha256((R/p).read_bytes()).hexdigest() for p in dirty},ensure_ascii=False,indent=2),encoding='utf-8')
print(enc)
