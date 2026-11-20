from pathlib import Path
import ctypes,hashlib,json,subprocess,xml.etree.ElementTree as ET
R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER';T=E/'actual-c';B=Path('D:/Keil_v5/ARM/arm5_compiler/Bin')
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,len(b));return b.value
control=ET.parse(R/'3.stm32f407工程/gongxun-f407.uvprojx').getroot().find('./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls')
opts=['--cpu','Cortex-M3','--c99','--gnu','-DAPP_VM_CALIBRATION_ONLY_ENABLE=0','-DAPP_VM_ONLY_ENABLE=0','-DAPP_WHEEL_ENABLE_HOLD_ENABLE=0','-DVEHICLE_INTEGRATED_ENABLE=0','-DVEHICLE_STAGE1_ENABLE=0']
opts+=['-D'+d for d in control.findtext('Define').split(',')]
for inc in control.findtext('IncludePath').split(';'):opts+=['-I',short((R/'3.stm32f407工程'/inc).resolve())]
unit=T/'guard_unit.c';unit.write_text('#include <stdint.h>\n#include "app_config.h"\nuint32_t candidate_default(void){return APP_RIGHT40_FORWARD_TRIM_ENABLE;}\n','ascii')
results={}
for name,baseline,right,expected in [('default_off',1,None,''),('enabled_debug',1,1,''),('enable2_rejected',1,2,'Right40_forward_trim_enable_invalid'),('baseline_inactive_rejected',0,1,'Right40_forward_trim_requires_baseline40_debug')]:
 cmd=[str(B/'armcc.exe'),*opts,'-DAPP_BASELINE40_ENABLE='+str(baseline)]
 if right is not None:cmd+=['-DAPP_RIGHT40_FORWARD_TRIM_ENABLE='+str(right)]
 cmd+=['-c',short(unit),'-o',str(T/(name+'.o'))];cp=subprocess.run(cmd,capture_output=True,timeout=60)
 output=cp.stdout+cp.stderr;(T/(name+'.log')).write_bytes(output)
 passed=(cp.returncode!=0 and expected.encode() in output) if expected else cp.returncode==0
 results[name]=dict(returncode=cp.returncode,expected_error=expected,status='PASS' if passed else 'FAIL');assert passed,(name,output)
report=dict(status='PASS',hardware_accessed=False,source_sha256=hashlib.sha256((U/'app_config.h').read_bytes()).hexdigest(),cases=results)
(E/'negative_compile_checks.json').write_text(json.dumps(report,indent=2),'utf8');print(json.dumps(report))