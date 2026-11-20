from pathlib import Path
import re,ctypes,subprocess,json,hashlib
r=Path.cwd();p=r/'3.stm32f407工程/gongxun-f407.uvprojx';s=p.read_bytes().decode('utf8');assert 'RADAR_V6' in s
t=r/'工创赛/tmp/radar_speed80_20261111/build';e=r/'工创赛/evidence/navigation-planning/2026-11-11/radar-zone2/speed80';
inputs=list((r/'3.stm32f407工程/CHASSIS/HARDWARE/RADAR_V6').glob('*.[ch]'))+[r/n for n in ['3.stm32f407工程/CHASSIS/HARDWARE/RADAR_SERVO/radar_servo_config.h','3.stm32f407工程/CHASSIS/HARDWARE/RADAR_SERVO/radar_servo.c','3.stm32f407工程/CHASSIS/HARDWARE/RADAR_SERVO/radar_lift.c','3.stm32f407工程/CHASSIS/HARDWARE/RADAR_SERVO/radar_lift.h','3.stm32f407工程/CHASSIS/USER/main.c','3.stm32f407工程/gongxun-f407.uvprojx']]
initial={str(p.relative_to(r)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
report={'source_sha256':initial,'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'targets':{}}
for target in ['Vehicle_Integrated','Vehicle_Offline']:
 out=t/target;out.mkdir(exist_ok=True)
 text=s.replace('.\\',str(p.parent)+'\\')
 for tag in ['OutputDirectory','ListingPath']:
  text=re.sub('<'+tag+'>.*?</'+tag+'>',lambda m:'<'+tag+'>'+target+'\\</'+tag+'>',text)
 q=t/(target+'.uvprojx');q.write_bytes(text.encode('utf8'))
 log=e/(target+'.log');proc=subprocess.run(['D:/Keil_v5/UV4/UV4.exe','-r',str(q),'-t',target,'-o',str(log)],timeout=240)
 data=log.read_bytes().decode('gbk');assert proc.returncode==0 and '0 Error(s), 0 Warning(s)' in data,(target,data[-1500:])
 report['targets'][target]={'returncode':proc.returncode,'program_size':re.search(r'Program Size:.*',data)[0],'log_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),'axf_sha256':hashlib.sha256((out/(target+'.axf')).read_bytes()).hexdigest()}
 print(target,report['targets'][target]['program_size'],flush=True)
assert initial=={str(p.relative_to(r)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
(e/'build_summary.json').write_text(json.dumps(report,indent=2),'utf8')
