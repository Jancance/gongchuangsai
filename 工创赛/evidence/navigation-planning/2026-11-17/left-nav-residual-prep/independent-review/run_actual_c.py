"""Actual application/raw odometry/ICM C offline review. I/O interfaces are stubs."""
from pathlib import Path
import hashlib,json,re,struct,subprocess,sys
OUT=Path(__file__).resolve().parent
ROOT=next(p for p in OUT.parents if (p/'3.stm32f407工程/gongxun-f407.uvprojx').is_file())
SRC=ROOT/'3.stm32f407工程/CHASSIS/USER';ARM=Path('D:/Keil_v5/ARM/arm5_compiler/bin')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda n:(SRC/n).read_bytes().decode('cp936')
names=['app_odometry.c','app_odometry.h','app_icm_observer.c','app_icm_observer.h','chassis_odometry.c','chassis_odometry.h','chassis_kinematics.h','route_heading_reference.h','heading_source.h','app_entry.h','app_entry_config.h','motion_action_plan.h','motion_trial40.h','app_config.h','app_motion_runtime.c','app_motion_runtime.h','chassis_auto_test.c','app_fusion_capture.c']
hashes={str((SRC/n).relative_to(ROOT)):sha(SRC/n) for n in names}
for n in names[:13]:
 if n!='app_config.h':(OUT/n).write_bytes((SRC/n).read_bytes())
# Bind actual candidate macro values; do not model or replace tested C functions.
config=read('app_config.h');macros='\n'.join(re.findall(r'^#define APP_LEFT_NAV_RESIDUAL_(?:COEFFICIENT|WEIGHT) .+$',config,re.M))
assert '0.0179055385f' in macros and '0.5f' in macros
(OUT/'app_config.h').write_text("""#ifndef APP_CONFIG_H
#define APP_CONFIG_H
#include "app_entry_config.h"
#define APP_BASELINE40_ACTIVE 1U
#define APP_POSITION_DIAGNOSTIC 1U
#define APP_HEADING_SESSION_TEST 1U
#define VEHICLE_STAGE1_ENABLE 0U
#define APP_BASELINE40_LEFT_RPM 102U
#define APP_BASELINE40_RPM 100U
#define APP_BASELINE40_MM 2100.0f
#ifndef APP_LEFT_NAV_RESIDUAL_ENABLE
#define APP_LEFT_NAV_RESIDUAL_ENABLE 0U
#endif
#define APP_BASELINE40_ACCEL_MM_S2 150.0f
#define APP_BASELINE40_JERK_MM_S3 300.0f
#define APP_DISTANCE_SCURVE_ACCEL_MM_S2 100.0f
#define APP_DISTANCE_SCURVE_JERK_MM_S3 200.0f
"""+macros+'\n#endif\n','ascii')
def typedef(text,name):return re.search(r'typedef\s+struct\s*\{[^}]*\}\s*'+name+r'\s*;',text,re.S).group(0)
ca=read('chassis_auto_test.h')
(OUT/'chassis_auto_test.h').write_text('#ifndef TEST_CHASSIS_AUTO_H\n#define TEST_CHASSIS_AUTO_H\n#include <stdint.h>\n'+typedef(ca,'ChassisAutoStatus')+'\n'+typedef(ca,'ChassisOdometrySamples')+'\nextern volatile uint32_t g_chassis_auto_active,g_wheel_test_active;\nvoid ChassisAuto_ReadOdometrySamples(ChassisOdometrySamples *v);\nuint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *v);\n#endif\n','cp936')
mo=typedef(read('app_motion_runtime.h'),'AppMotionObservation')
(OUT/'app_motion_runtime.h').write_text('#ifndef TEST_MOTION_H\n#define TEST_MOTION_H\n#include "motion_action_plan.h"\n'+mo+'\nextern volatile uint32_t g_debug_plan_id;\nconst AppMotionObservation *AppMotion_Live(void);\n#endif\n','cp936')
entry=re.search(r'uint32_t AppDebugPlan40Allowed\([^\n]*\)\s*\{.*?\n\}',read('app_entry.c'),re.S).group(0)
(OUT/'entry_scope.c').write_text('#include "app_entry.h"\n#include "motion_trial40.h"\n'+entry+'\n','ascii')
(OUT/'FreeRTOS.h').write_text('#ifndef TEST_RTOS_H\n#define TEST_RTOS_H\n#include <stdint.h>\ntypedef uint32_t TickType_t;\n#define portTICK_PERIOD_MS 1U\n#endif\n','ascii')
(OUT/'task.h').write_text('uint32_t xTaskGetTickCount(void);\nvoid TestEnter(void);void TestExit(void);\n#define taskENTER_CRITICAL() TestEnter()\n#define taskEXIT_CRITICAL() TestExit()\n','ascii')
for n,var in [('freertos_hooks.h','g_rtos_fault'),('safety.h','g_safety_fault'),('can.h','g_can_ctrl_fault_latched')]:
 (OUT/n).write_text('#include <stdint.h>\nextern volatile uint32_t '+var+';\n','ascii')
(OUT/'vehicle_tuning.h').write_text('#include "app_config.h"\n','ascii')
(OUT/'heading_controller.h').write_text('float HeadingController_WrapDeg(float x);\nfloat HeadingController_ErrorDeg(float a,float b);\n','ascii')
(OUT/'power.h').write_bytes((ROOT/'3.stm32f407工程/CHASSIS/HARDWARE/POWER/power.h').read_bytes())
(OUT/'icm45686_raw.h').write_bytes((ROOT/'3.stm32f407工程/CHASSIS/HARDWARE/ICM45686/icm45686_raw.h').read_bytes())
(OUT/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }\n','ascii')
logs=[];results=[]
def run(cmd,label):
 p=subprocess.run(cmd,cwd=OUT,capture_output=True,text=True,errors='replace');logs.append(dict(label=label,command=cmd,returncode=p.returncode,output=p.stdout+p.stderr));(OUT/'build.json').write_text(json.dumps(logs,ensure_ascii=False,indent=2),'utf8')
 if p.returncode:raise RuntimeError(label+'\n'+p.stdout+p.stderr)
 return p.stdout
sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
import unicorn as uc
from unicorn import arm_const as reg
for enabled in (0,1):
 objects=[]
 for n in ['unit','app_odometry','chassis_odometry','app_icm_observer','entry_scope']:
  objects.append(n+'.o');run([str(ARM/'armcc.exe'),'--cpu','Cortex-M4','--c99','--split_sections','-O1','-I','.',f'-DAPP_LEFT_NAV_RESIDUAL_ENABLE={enabled}','-c',n+'.c','-o',n+'.o'],f'enable{enabled}_{n}')
 run([str(ARM/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct']+objects+['--library_type=microlib','-o',f'unit{enabled}.axf'],f'enable{enabled}_link')
 blob=(OUT/f'unit{enabled}.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',blob)
 m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
 for i in range(h[12]):
  _,kind,flags,addr,off,size,*_=struct.unpack_from('<IIIIIIIIII',blob,h[6]+i*h[11])
  if flags&2 and kind!=8 and size:m.mem_write(addr,blob[off:off+size])
 stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(h[4]|1,stop,count=100000000)
 value=m.reg_read(reg.UC_ARM_REG_R0);symbols=run([str(ARM/'fromelf.exe'),'--text','-s',f'unit{enabled}.axf'],f'enable{enabled}_symbols')
 def addr(n):return int(re.search(r'\b'+n+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
 def u(n):return struct.unpack('<I',m.mem_read(addr(n),4))[0]
 result=dict(enabled=enabled,passed=not(value&0x80000000) and m.reg_read(reg.UC_ARM_REG_PC)==stop,assertions=value if not(value&0x80000000) else None,failure_line=value&0x7fffffff if value&0x80000000 else None,pc=hex(m.reg_read(reg.UC_ARM_REG_PC)),scope_cases=u('scope_cases'),observer_cases=u('observer_cases'),rawF_navF_observerWheelF_observerPredictF=list(struct.unpack('<8f',m.mem_read(addr('values'),32)))[:4])
 results.append(result);(OUT/'actual_c_report.json').write_text(json.dumps(dict(status='RUNNING',results=results),indent=2),'utf8')
 if not result['passed']:raise RuntimeError(str(result))
assert all(sha(ROOT/n)==digest for n,digest in hashes.items())
before=OUT.parent/'before';preserved={}
for n in ['app_fusion_capture.c','chassis_auto_test.c','app_motion_runtime.c']:
 preserved[n]=(SRC/n).read_bytes()==(before/n).read_bytes();assert preserved[n],n
assert 'AppOdometry_ReadBaseline40(&lateral_od)' in read('chassis_auto_test.c')
assert read('app_fusion_capture.c').count('AppOdometry_ReadBaseline40(')==2
observer=read('app_icm_observer.c');assert observer.count('AppOdometry_ReadNavigation40(')==1 and 'AppOdometry_ReadBaseline40(' not in observer
assert observer.replace('AppOdometry_ReadNavigation40','AppOdometry_ReadBaseline40').encode('cp936')==(before/'app_icm_observer.c').read_bytes()
report=dict(status='PASS_ACTUAL_APPLICATION_AND_OBSERVER_C',results=results,source_sha256=hashes,raw_control_capture_sources_unchanged=preserved,struct_AppOdometryBaseline40_bytes=64,full_sources_byte_identical=all((OUT/n).read_bytes()==(SRC/n).read_bytes() for n in ['app_odometry.c','chassis_odometry.c','app_icm_observer.c']),hardware_accessed=False,product_modified=False,limits='Actual full app_odometry, chassis_odometry and ICM Step execute in ARM emulator; RTOS critical sections, clock, motor sample source, power and motion owner interfaces stubbed with actual typedef layouts. Candidate k/weight macros bound from actual config. No scheduler race, CAN/electrical, hardware flash or physical-navigation accuracy test. Capture ABI is type-size plus unchanged capture/control source, not executing fixed-address hardware capture.')
(OUT/'actual_c_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf8');print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},ensure_ascii=False,indent=2))
