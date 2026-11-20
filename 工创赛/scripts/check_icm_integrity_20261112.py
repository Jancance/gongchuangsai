from pathlib import Path
import hashlib,json,subprocess
r=Path.cwd();e=r/'工创赛/evidence/navigation-planning/2026-11-12/icm-raw';old=json.loads((r/'工创赛/tmp/icm_raw_20261112/before/manifest.json').read_text('utf8'));changed=[n for n,h in old.items() if hashlib.sha256((r/n).read_bytes()).hexdigest()!=h];assert set(changed)=={'3.stm32f407工程/CHASSIS/USER/main.c','3.stm32f407工程/gongxun-f407.uvprojx'},changed
m=(r/'3.stm32f407工程/CHASSIS/USER/main.c').read_bytes();u=(r/'3.stm32f407工程/gongxun-f407.uvprojx').read_bytes();assert m.count(b'\n')==m.count(b'\r\n') and b'\r' not in u;m.decode('gbk');u.decode('utf8')
symbols={}
for target,name in [('Vehicle_Integrated','Vehicle_Integrated'),('Vehicle_Offline','Vehicle_Offline')]:
 p=r/'工创赛/tmp/icm_raw_20261112/build'/target/(name+'.axf');s=subprocess.check_output(['D:/Keil_v5/ARM/arm5_compiler/bin/fromelf.exe','--text','-s',str(p)]).decode(errors='replace');assert 'App_IcmRawTask' not in s and 'IcmPort_Init' not in s;symbols[target]='no ICM task or port init symbol retained'
(e/'integrity_result.json').write_text(json.dumps({'baseline_inputs':len(old),'changed':changed,'all_other_inputs_unchanged':True,'main_gbk_crlf':True,'uvprojx_utf8_lf':True,'default_off_symbols':symbols,'hardware':False},ensure_ascii=False,indent=2),'utf8');print('integrity',len(old),'inputs; only main/project changed; default off symbols absent')
