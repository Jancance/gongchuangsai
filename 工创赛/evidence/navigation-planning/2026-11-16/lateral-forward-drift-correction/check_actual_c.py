from pathlib import Path
import ctypes,hashlib,json,re,struct,subprocess,sys,xml.etree.ElementTree as ET
R=Path.cwd();E=Path(__file__).parent;U=R/'3.stm32f407工程/CHASSIS/USER';B=Path('D:/Keil_v5/ARM/arm5_compiler/Bin');T=E/'actual-c';T.mkdir(exist_ok=True)
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,len(b));return b.value
prefix=r'''
#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"
#include "app_entry.h"
#include "motion_distance_limits.h"
#include "app_fusion_capture.h"
#include "app_odometry.h"
#include "chassis_auto_test.h"
#include "heading_source.h"
#include "power.h"
static uint32_t clock_ms;
static ChassisAutoStatus motion;
static ChassisWheelLog wheel;
static ChassisOdometrySamples position;
#if APP_BASELINE40_ACTIVE
static AppOdometryBaseline40 od;
#endif
static AppHeadingSource heading;
volatile uint32_t g_vm_test_request,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
uint32_t SystemCoreClock=168000000U;
TickType_t xTaskGetTickCount(void){return clock_ms;}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *v){*v=motion;return 1U;}
uint32_t ChassisAuto_ReadWheelLog(ChassisWheelLog *v){*v=wheel;return 1U;}
void ChassisAuto_ReadOdometrySamples(ChassisOdometrySamples *v){*v=position;}
#if APP_BASELINE40_ACTIVE
void AppOdometry_ReadBaseline40(AppOdometryBaseline40 *v){*v=od;}
#endif
uint32_t App_HeadingSourceRead(AppHeadingSource *v){*v=heading;return 1U;}
'''
tests=r'''
volatile uint32_t review_assertions;
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
#if APP_BASELINE40_ACTIVE
static IcmRawStatus rr;
static void sample(uint32_t ms){
 clock_ms=ms;DWT->CYCCNT=ms*168000U;rr.state=ICM_RAW_READY;rr.last_error=ICM_RAW_OK;rr.fresh=1;
 rr.sample_sequence++;rr.edge_sequence++;rr.sample_tick_ms=ms;rr.edge_cycle=ms*168000U;
 rr.read_start_cycle=rr.edge_cycle+20;rr.read_end_cycle=rr.edge_cycle+100;
 AppFusionCapture_Step(&rr);
}
static void reset_capture(void){
 memset(&g_fusion_capture_a,0,sizeof(g_fusion_capture_a));memset(&g_fusion_capture_b,0,sizeof(g_fusion_capture_b));
 memset(&rr,0,sizeof(rr));memset(&motion,0,sizeof(motion));motion.session=1;
}
static uint32_t capture_check(uint32_t terminal,uint32_t kind,uint32_t expected){
 uint32_t ms;reset_capture();
 for(ms=0;ms<=14000;ms+=5){
  motion.state=ms<500 ? CHASSIS_AUTO_IDLE : (ms==500 ? CHASSIS_AUTO_START : CHASSIS_AUTO_RUN);
  if(ms>=8550)motion.state=CHASSIS_AUTO_STOP;
  if(ms>=terminal)motion.state=kind;
  sample(ms);if(g_fusion_capture_a.h.state==2)break;
 }
 CK(g_fusion_capture_a.h.reason==expected);
 CK(g_fusion_capture_a.h.raw_count<=2100 && g_fusion_capture_a.h.post_count<=2000);
 if(expected==1){
  CK(g_fusion_capture_a.h.reserved[4]==1 && g_fusion_capture_a.h.reserved[5]==terminal);
  CK(g_fusion_capture_a.h.freeze_ms==terminal+500);
 }else{CK(g_fusion_capture_a.h.freeze_ms==10500 && g_fusion_capture_a.h.post_count==2000);}
 if(terminal==9700){CK(g_fusion_capture_a.h.stop_ms==8550 && g_fusion_capture_a.h.post_count==1940);}
 return 0;
}
#endif
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t id,d,r,n,b,i,z;MotionAction a[8];ChassisMotorCommand cmd[4];
 for(id=8;id<=9;id++)for(d=0;d<4;d++)for(r=99;r<=103;r++){
  CK(AppDebugPlan40Allowed(id,d,r,2100)==(APP_BASELINE40_ACTIVE && ((id==8&&d==0&&r==100)||(id==9&&d==1&&r==102))));
 }
 CK(!AppDebugPlan40Allowed(9,1,102,2099));CK(!AppDebugPlan40Allowed(9,1,102,2101));
 CK(AppDebugPlan_Load(8,a,&n)==APP_BASELINE40_ACTIVE);
 if(APP_BASELINE40_ACTIVE){CK(n==1 && a[0].kind==4 && a[0].base_rpm==100);}
 CK(AppDebugPlan_Load(9,a,&n)==APP_BASELINE40_ACTIVE);
 if(APP_BASELINE40_ACTIVE){CK(n==1 && a[0].kind==5 && a[0].base_rpm==102 && a[0].distance_mm==2100);}
#if APP_BASELINE40_ACTIVE
 CK(MOTION_PROFILE_BASE_CEILING==102 && MotionDistanceFollowRpm(102)==2 && MotionDistanceFollowKp(102)==1);
 CK(MotionDistanceReferenceSpeed(3.14159265358979323846f*77/60,102)==400);
 CK(MotionDistanceAcceleration(1,102,2100)==(APP_LEFT40_DRIFT_TRIM_ENABLE ? 150.0f/1.04f : 150.0f));
 CK(MotionDistanceJerk(1,102,2100)==(APP_LEFT40_DRIFT_TRIM_ENABLE ? 300.0f/1.04f : 300.0f));
 CK(MotionDistanceAcceleration(0,100,2100)==150 && MotionDistanceJerk(0,100,2100)==300);
 CK(MotionDistanceFeedbackLimit(1,102,2100)==130 && MotionDistanceRunMsFor(1,102,2100)==12779);
 CK(MotionDistanceJumpCountsPerMs(1,102,2100)==175);
 for(b=0;b<=102;b++)for(z=0;z<3;z++){
  float correction=(float)((int32_t)z*2-2);float eff=correction;
  CK(Chassis_TranslationProfileCommands(1,b,correction,cmd));
  for(i=0;i<4;i++)CK(cmd[i].speed_raw<=1040);
  if(b>2){CK(cmd[0].direction==0 && cmd[1].direction==0 && cmd[2].direction==1 && cmd[3].direction==1);}
  if(b==0){for(i=0;i<4;i++)CK(cmd[i].speed_raw==0);}
 }

 for(b=0;b<=102;b++)for(z=0;z<3;z++){
  ChassisMotorCommand old[4];int32_t logical[4];float h=(float)((int32_t)z*2-2);uint32_t trim=(uint32_t)(.04f*b+.5f);
  CK(Chassis_Left40DriftCommands(b,h,cmd));
  CK(Chassis_TranslationProfileCommands(1,b,h,old));
  for(i=0;i<4;i++){
   logical[i]=cmd[i].direction ? -(int32_t)cmd[i].speed_raw : (int32_t)cmd[i].speed_raw;
   if(i==1 || i==3)logical[i]=-logical[i];
   CK(cmd[i].speed_raw<=1080);
  }
  CK(logical[0]+logical[1]+logical[2]+logical[3]==-(int32_t)trim*40);
  CK(logical[0]-logical[1]-logical[2]+logical[3]==(int32_t)b*40);
  if(!trim)CK(!memcmp(cmd,old,sizeof(cmd)));
 }
 CK(!Chassis_Left40DriftCommands(103,0,cmd));
 CK(!Chassis_Left40DriftCommands(102,3,cmd));
 CK(!Chassis_Left40DriftCommands(102,0,0));
 CK(!Chassis_TranslationProfileCommands(1,103,0,cmd));
 CK(Chassis_Baseline40Commands(100,0,0,cmd));
 CK(!Chassis_Baseline40Commands(102,0,0,cmd));
 z=capture_check(9700,CHASSIS_AUTO_DONE,1);if(z)return z;
 z=capture_check(13500,CHASSIS_AUTO_DONE,2);if(z)return z;
 z=capture_check(1500,CHASSIS_AUTO_FAULT,1);if(z)return z;
 z=capture_check(1500,CHASSIS_AUTO_CANCELLED,1);if(z)return z;
#else
 CK(!MotionTrial40Allowed(1,102,2100));
#endif
 return 0;
}
'''
source=(U/'app_fusion_capture.c').read_text('ascii');(T/'unit.c').write_text(prefix+source+tests,encoding='ascii')
control=ET.parse(U.parents[1]/'gongxun-f407.uvprojx').getroot().find('./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls')
opts=['--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','--diag_suppress=177,550','-DCAPTURE_UNIT_TEST=1','-DVEHICLE_INTEGRATED_ENABLE=0','-DVEHICLE_STAGE1_ENABLE=0']
opts += ['-D'+d for d in control.findtext('Define').split(',')]
for inc in control.findtext('IncludePath').split(';'):opts+=['-I',short((U.parents[1]/inc).resolve())]
def run(cmd,log):
 cp=subprocess.run(cmd,cwd=T,capture_output=True,timeout=90);(T/log).write_bytes(cp.stdout+cp.stderr);assert cp.returncode==0,(cp.stdout+cp.stderr).decode('gbk',errors='replace')
sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
from unicorn import arm_const as reg
report={'hardware_accessed':False,'software_model_only':True,'cases':{},'source_sha256':{f:hashlib.sha256((U/f).read_bytes()).hexdigest() for f in ['app_entry.c','app_entry.h','chassis_kinematics.c','app_fusion_capture.c','motion_trial20.h','motion_trial40.h']}}
for mode,flag in [('trim_on',1),('trim_off',1),('inactive',0)]:
 objects=[]
 for p in [T/'unit.c',U/'app_entry.c',U/'chassis_kinematics.c',U/'motion_profile.c']:
  obj=mode+'_'+p.stem+'.o';objects.append(obj);run([str(B/'armcc.exe'),*opts,'-DAPP_LEFT40_DRIFT_TRIM_ENABLE='+str(int(mode=='trim_on')),'-DAPP_BASELINE40_ENABLE='+str(flag),'-c',short(p),'-o',obj],obj+'.log')
 (T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { '+objects[0]+' (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x50000 { *(+RW,+ZI) } }\n')
 run([str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct',*objects,'--library_type=microlib','-o',mode+'.axf'],mode+'_link.log')
 data=(T/(mode+'.axf')).read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',data)
 machine=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
 for addr,size in [(0x08000000,0x100000),(0x20000000,0x50000),(0xE0000000,0x100000)]:machine.mem_map(addr,size)
 for n in range(elf[12]):
  _,kind,flags,addr,offset,size,*_=struct.unpack_from('<IIIIIIIIII',data,elf[6]+n*elf[11])
  if flags&2 and size:machine.mem_write(addr,b'\0'*size if kind==8 else data[offset:offset+size])
 stop=0x080FF000;machine.reg_write(reg.UC_ARM_REG_SP,0x2004FFF0);machine.reg_write(reg.UC_ARM_REG_LR,stop|1)
 machine.emu_start(elf[4]|1,stop,timeout=60000000,count=200000000);assert machine.reg_read(reg.UC_ARM_REG_PC)==stop
 result=machine.reg_read(reg.UC_ARM_REG_R0)
 symbols=subprocess.check_output([str(B/'fromelf.exe'),'--text','-s',mode+'.axf'],cwd=T).decode(errors='replace')
 addr=int(re.search(r'\breview_assertions\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
 count=struct.unpack('<I',machine.mem_read(addr,4))[0];report['cases'][mode]={'assertions':count,'failed_line':result,'status':'PASS' if result==0 else 'FAIL'}
 (E/'actual_c_checks.json').write_text(json.dumps(report,indent=2),encoding='utf8')
 if result:print((prefix+source+tests).splitlines()[result-1]);raise SystemExit(result)
print(json.dumps(report['cases']))
