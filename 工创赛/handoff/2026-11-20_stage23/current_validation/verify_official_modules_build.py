from pathlib import Path
import sys,json,hashlib,re
sys.path.insert(0,'D:/Codex/工创赛/tmp/gripper_debug_deps')
from elftools.elf.elffile import ELFFile
w=Path('work');d=w/'stage3_modules_20261120';m=json.loads((d/'stage3-modules-compatible.json').read_text(encoding='utf-8-sig'));actual=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
records=m['unchanged_sha256']|m['changes'];bad=[n for n,h in records.items() if hashlib.sha256((actual/n).read_bytes()).hexdigest()!=h];assert not bad,bad
images={}
for target in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
 p=actual/'Objects'/target/(target+'.axf');log=d/('official-'+target+'.log');t=log.read_text(encoding='gbk',errors='replace');assert '0 Error(s), 0 Warning(s).' in t
 with p.open('rb') as f:
  elf=ELFFile(f);segments=[(int(s['p_paddr']),s.data()) for s in elf.iter_segments() if s['p_type']=='PT_LOAD' and s['p_filesz'] and 0x08000000<=int(s['p_paddr'])<0x08100000]
 images[target]={'axf':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'flash_bytes':sum(len(data) for _,data in segments),'flash_sha256':hashlib.sha256(b''.join(data for _,data in segments)).hexdigest(),'build_log':str(log.resolve()),'errors':0,'warnings':0,'candidate_axf_sha256':m['images'][target]['sha256']}
r={'official_project':str(actual),'source_inventory_count':len(records),'source_mismatches':bad,'images':images,'flashed':False,'reset':False,'physical_acceptance':False};(d/'official-build-verification.json').write_text(json.dumps(r,indent=2,ensure_ascii=False),encoding='utf-8');print(json.dumps(r,ensure_ascii=False))
