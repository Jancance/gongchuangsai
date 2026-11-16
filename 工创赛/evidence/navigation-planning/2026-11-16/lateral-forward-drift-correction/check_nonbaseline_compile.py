from pathlib import Path
import ctypes,subprocess,json,xml.etree.ElementTree as ET
R=Path.cwd();E=Path(__file__).parent;P=R/'3.stm32f407工程';U=P/'CHASSIS/USER'
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,len(b));return b.value
c=ET.parse(P/'gongxun-f407.uvprojx').getroot().find('./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls')
cmd=['D:/Keil_v5/ARM/arm5_compiler/Bin/armcc.exe','--cpu','Cortex-M4','--fpu','FPv4-SP','--c99','--gnu','--split_sections','-O1','-DAPP_BASELINE40_ENABLE=0','-DVEHICLE_INTEGRATED_ENABLE=0','-DVEHICLE_STAGE1_ENABLE=0']
cmd+=['-D'+d for d in c.findtext('Define').split(',')]
for inc in c.findtext('IncludePath').split(';'):cmd+=['-I',short((P/inc).resolve())]
cmd+=['-c',short(U/'chassis_auto_test.c'),'-o',str(E/'actual-c/nonbaseline_chassis_auto_test.o')]
p=subprocess.run(cmd,capture_output=True,timeout=60);(E/'nonbaseline_compile.log').write_bytes(p.stdout+p.stderr)
(E/'nonbaseline_compile.json').write_text(json.dumps(dict(returncode=p.returncode,command=cmd,hardware_accessed=False,scope='actual chassis_auto_test.c with BASELINE40=0 and newmacro default0'),indent=2),encoding='utf8')
print(p.stdout.decode('gbk',errors='replace')+p.stderr.decode('gbk',errors='replace'));assert p.returncode==0
