"""Run the actual servo driver with peripheral stubs, and actual zone/angle logic on ARM."""
from pathlib import Path
import json,re,struct,subprocess,sys
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[4];FW=ROOT/'3.stm32f407工程';SRC=FW/'HARDWARE/RADAR_SERVO'
ARM=Path('D:/Keil_v5/ARM/arm5_compiler/bin')
UNIT=r'''
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "radar_servo.h"
#include "radar_servo_logic.h"
#include "zdt_x_config.h"
#include <string.h>
volatile unsigned checks,failures,hw_calls,failure_checks[16];
static uint32_t test_pclk=42000000U,psc,period,c1,c2;
static void ck(unsigned good){checks++;if(!good){if(failures<16)failure_checks[failures]=checks;failures++;}}
void RCC_GetClocksFreq(RCC_ClocksTypeDef *c){c->PCLK1_Frequency=test_pclk;}
void RCC_AHB1PeriphClockCmd(uint32_t p,FunctionalState s){hw_calls++;}
void RCC_APB1PeriphClockCmd(uint32_t p,FunctionalState s){hw_calls++;}
void GPIO_ResetBits(GPIO_TypeDef *p,uint16_t pins){hw_calls++;}
void GPIO_StructInit(GPIO_InitTypeDef *p){memset(p,0,sizeof(*p));}
void GPIO_Init(GPIO_TypeDef *p,GPIO_InitTypeDef *c){hw_calls++;}
void GPIO_PinAFConfig(GPIO_TypeDef *p,uint16_t source,uint8_t af){hw_calls++;}
void TIM_Cmd(TIM_TypeDef *p,FunctionalState s){hw_calls++;}
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef *p){memset(p,0,sizeof(*p));}
void TIM_TimeBaseInit(TIM_TypeDef *p,TIM_TimeBaseInitTypeDef *c){psc=c->TIM_Prescaler;period=c->TIM_Period;hw_calls++;}
void TIM_OCStructInit(TIM_OCInitTypeDef *p){memset(p,0,sizeof(*p));}
void TIM_OC1Init(TIM_TypeDef *p,TIM_OCInitTypeDef *c){c1=c->TIM_Pulse;hw_calls++;}
void TIM_OC2Init(TIM_TypeDef *p,TIM_OCInitTypeDef *c){c2=c->TIM_Pulse;hw_calls++;}
void TIM_OC1PreloadConfig(TIM_TypeDef *p,uint16_t s){hw_calls++;}
void TIM_OC2PreloadConfig(TIM_TypeDef *p,uint16_t s){hw_calls++;}
void TIM_ARRPreloadConfig(TIM_TypeDef *p,FunctionalState s){hw_calls++;}
void TIM_SetCounter(TIM_TypeDef *p,uint32_t c){hw_calls++;}
void TIM_GenerateEvent(TIM_TypeDef *p,uint16_t e){hw_calls++;}
void TIM_SetCompare1(TIM_TypeDef *p,uint32_t c){c1=c;hw_calls++;}
void TIM_SetCompare2(TIM_TypeDef *p,uint32_t c){c2=c;hw_calls++;}
void test_entry(void){
 uint32_t i,saved;uint16_t pulse=1234,previous=0;RadarZoneFilter f;union {uint32_t u;float f;}nan;
 nan.u=0x7fc00000U;
 for(i=0;i<=1800;i++){
  ck(RadarServo_Encode(1,(float)i/10.0f,&pulse)==1);
  ck(pulse>=500 && pulse<=2500);ck(pulse>=previous);previous=pulse;
 }
 ck(RadarServo_Encode(2,90.0f,&pulse)==1 && pulse==1500);
 ck(!RadarServo_Encode(0,0.0f,&pulse));ck(!RadarServo_Encode(3,0.0f,&pulse));
 ck(!RadarServo_Encode(1,-0.1f,&pulse));ck(!RadarServo_Encode(1,180.1f,&pulse));
 ck(!RadarServo_Encode(1,nan.f,&pulse));ck(!RadarServo_Encode(1,0.0f,0));
 RadarZoneFilter_Init(&f);ck(!RadarZoneFilter_Lock(&f));
 RadarZoneFilter_Sample(&f,0,10);RadarZoneFilter_Sample(&f,0,59);ck(!f.ready);
 RadarZoneFilter_Sample(&f,0,60);ck(f.ready && f.zone==1 && RadarZoneFilter_Lock(&f)==1);
 RadarZoneFilter_Sample(&f,1,65);ck(!f.ready && RadarZoneFilter_Lock(&f)==1);
 RadarZoneFilter_Sample(&f,0,70);RadarZoneFilter_Sample(&f,0,119);ck(!f.ready);
 RadarZoneFilter_Sample(&f,0,120);ck(f.ready && f.zone==1);
 RadarZoneFilter_Sample(&f,1,200);RadarZoneFilter_Sample(&f,1,250);
 ck(f.zone==2 && RadarZoneFilter_Lock(&f)==1);
 RadarZoneFilter_Unlock(&f);ck(RadarZoneFilter_Lock(&f)==2);
 RadarZoneFilter_Init(&f);RadarZoneFilter_Sample(&f,1,0xfffffff0U);
 RadarZoneFilter_Sample(&f,1,0x21U);ck(!f.ready);
 RadarZoneFilter_Sample(&f,1,0x22U);ck(f.ready && f.zone==2);
#if APP_X_OFFLINE_ONLY
 ck(!RadarServo_Init() && g_radar_servo_state==2 && hw_calls==0);
 ck(!RadarServo_SetAngle(1,90.0f) && hw_calls==0);
#else
 test_pclk=42000000U; /* Direct emulation entry does not run C RW startup. */
 RCC->CFGR=RCC_CFGR_PPRE1_DIV4;
 ck(RadarServo_Init()==1 && psc==83 && period==19999 && c1==500 && c2==500);
 ck(RadarServo_SetAngle(1,90.0f)==1 && c1==1500 && c2==500);
 ck(RadarServo_SetAngle(2,180.0f)==1 && c1==1500 && c2==2500);
 saved=hw_calls;ck(!RadarServo_SetAngle(3,90.0f));ck(!RadarServo_SetAngle(1,nan.f));
 ck(!RadarServo_SetAngle(1,180.1f));ck(!RadarServo_SetAngle(1,-1.0f));ck(hw_calls==saved && c1==1500 && c2==2500);
 ck(RadarServo_Init()==1 && hw_calls==saved && c1==1500 && c2==2500);
 g_radar_servo_state=0;test_pclk=16000000U;RCC->CFGR=0;
 ck(RadarServo_Init()==1 && psc==15);
 g_radar_servo_state=0;test_pclk=16500000U;saved=hw_calls;
 ck(!RadarServo_Init() && g_radar_servo_state==3 && hw_calls==saved);
#endif

 g_radar_servo_test_id=1;g_radar_servo_test_angle_deg=10.0f;g_radar_servo_test_request=1;
#if APP_X_OFFLINE_ONLY
 RadarServo_DebugService(1);ck(g_radar_servo_test_result==2 && g_radar_servo_test_sequence==1 && hw_calls==0);
#else
 test_pclk=16000000U;g_radar_servo_state=0;ck(RadarServo_Init()==1);
 RadarServo_DebugService(1);ck(g_radar_servo_test_result==1 && g_radar_servo_test_sequence==1 && c1==611 && c2==500);
 saved=hw_calls;RadarServo_DebugService(1);ck(g_radar_servo_test_sequence==1 && hw_calls==saved);
 g_radar_servo_test_angle_deg=16.0f;g_radar_servo_test_request=1;RadarServo_DebugService(1);
 ck(g_radar_servo_test_result==3 && g_radar_servo_test_sequence==2 && hw_calls==saved);
 g_radar_servo_test_id=3;g_radar_servo_test_angle_deg=10.0f;g_radar_servo_test_request=1;RadarServo_DebugService(1);
 ck(g_radar_servo_test_result==3 && g_radar_servo_test_sequence==3 && hw_calls==saved);
 g_radar_servo_test_id=1;g_radar_servo_test_request=1;RadarServo_DebugService(0);
 ck(g_radar_servo_test_result==2 && g_radar_servo_test_sequence==4 && hw_calls==saved);
 g_radar_servo_test_request=2;RadarServo_DebugService(1);
 ck(g_radar_servo_test_result==3 && g_radar_servo_test_sequence==5 && hw_calls==saved);
 g_radar_servo_test_id=2;g_radar_servo_test_request=1;RadarServo_DebugService(1);
 ck(g_radar_servo_test_result==1 && g_radar_servo_test_sequence==6 && c1==611 && c2==611);
#endif
}
'''
reports=[]
for offline in (0,1):
 out=OUT/('actual_c_offline_'+str(offline));out.mkdir(exist_ok=True)
 (out/'unit.c').write_text(UNIT)
 (out/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }')
 includes=[FW/'CORE',FW/'USER',FW/'StdPeriph_Driver/inc',SRC]
 commands=[]
 for src,obj in [('unit.c','unit.o'),(str(SRC/'radar_servo.c'),'driver.o'),(str(SRC/'radar_servo_logic.c'),'logic.o')]:
  commands.append([str(ARM/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-DSTM32F40_41xxx','-DUSE_STDPERIPH_DRIVER','-DAPP_X_OFFLINE_ONLY='+str(offline),*[arg for p in includes for arg in ('-I',str(p))],'-c',src,'-o',obj])
 commands.append([str(ARM/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','driver.o','logic.o','--library_type=microlib','-o','unit.axf'])
 logs=[]
 for command in commands:
  p=subprocess.run(command,cwd=out,capture_output=True,text=True,errors='replace',timeout=60)
  logs.append({'command':command,'returncode':p.returncode,'output':p.stdout+p.stderr});(out/'build.json').write_text(json.dumps(logs,indent=2))
  assert p.returncode==0,p.stdout+p.stderr
 sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
 import unicorn as uc
 from unicorn import arm_const as reg
 m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
 m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000);m.mem_map(0x40000000,0x30000)
 b=(out/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
 for i in range(h[10]):
  kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
  if kind==1 and size:m.mem_write(addr,b[off:off+size])
 symbols=subprocess.check_output([str(ARM/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out,text=True,errors='replace')
 def address(name):return int(re.search(r'\b'+name+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
 stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1)
 m.emu_start(address('test_entry')|1,stop,count=5000000)
 checks=struct.unpack('<I',m.mem_read(address('checks'),4))[0];failures=struct.unpack('<I',m.mem_read(address('failures'),4))[0]
 failed=list(struct.unpack('<16I',m.mem_read(address('failure_checks'),64)))[:failures]
 assert m.reg_read(reg.UC_ARM_REG_PC)==stop and checks>5400 and failures==0,(checks,failures,failed)
 reports.append({'offline':offline,'checks':checks,'failures':failures,'peripherals':'stubs, not hardware'})
(OUT/'actual_c_report.json').write_text(json.dumps(reports,indent=2));print(json.dumps(reports))
