from pathlib import Path
import subprocess,re,struct,json,sys,hashlib
R=Path.cwd();T=R/'工创赛/tmp/icm_raw_20261112/port_test';T.mkdir(parents=True,exist_ok=True);B=Path('D:/Keil_v5/ARM/arm5_compiler/bin');D=R/'3.stm32f407工程';E=R/'工创赛/evidence/navigation-planning/2026-11-12/icm-raw'
source='''#include "icm45686_port.c"
uint32_t SystemCoreClock=168000000U;
void GPIO_Init(GPIO_TypeDef *g,GPIO_InitTypeDef *i){(void)g;(void)i;}
void GPIO_PinAFConfig(GPIO_TypeDef *g,uint16_t p,uint8_t a){(void)g;(void)p;(void)a;}
void GPIO_ResetBits(GPIO_TypeDef *g,uint16_t p){(void)g;(void)p;}
void GPIO_SetBits(GPIO_TypeDef *g,uint16_t p){(void)g;(void)p;}
void GPIO_StructInit(GPIO_InitTypeDef *i){(void)i;}
void RCC_AHB1PeriphClockCmd(uint32_t p,FunctionalState s){(void)p;(void)s;}
void RCC_APB2PeriphClockCmd(uint32_t p,FunctionalState s){(void)p;(void)s;}
void vTaskDelay(TickType_t t){(void)t;}
TickType_t xTaskGetTickCount(void){return 0;}

unsigned test_entry(void) {
 uint32_t loops=20; uint8_t byte;
 SPI1->SR=2; DWT->CYCCNT=168001;
 if(wait_flag(2,1,0,&loops)==0) return 1;
 DWT->CYCCNT=10;
 if(wait_flag(2,1,0,&loops)!=0) return 2;
 SPI1->SR=0; loops=3;
 if(wait_flag(2,1,0,&loops)==0 || loops!=0) return 3;
 SPI1->SR=0x42; loops=20;
 if(wait_flag(2,1,10,&loops)==0) return 4;
 SPI1->SR=1; DWT->CYCCNT=0;
 if(IcmPort_Read(0,&byte,1)==0 || SPI1->SR!=0 || !(SPI1->CR1&64)) return 5;
 SPI1->SR=0x40;
 if(IcmPort_Read(0,&byte,1)==0 || SPI1->SR!=0) return 6;
 SPI1->SR=2;loops=20;
 if(wait_flag(2,1,0,&loops)!=0) return 7;
 return 0;
}
'''
(T/'unit.c').write_text(source,'utf8');(T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }')
s=(D/'gongxun-f407.uvprojx').read_text('utf8');paths=next(x for x in re.findall('<IncludePath>(.*?)</IncludePath>',s) if x)
args=['-I'+str(D/Path(p.replace('\\','/'))) for p in paths.split(';')]
cmds=[[str(B/'armcc.exe'),'--cpu','Cortex-M4.fp','--c99','--gnu','--split_sections','-O1','-DUSE_STDPERIPH_DRIVER','-DSTM32F40_41xxx',*args,'-c','unit.c','-o','unit.o'],[str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]
logs=[]
for cmd in cmds:
 p=subprocess.run(cmd,cwd=T,capture_output=True);logs.append((p.stdout+p.stderr).decode(errors='replace'));assert p.returncode==0,logs[-1]
(T/'compile.log').write_text('\n'.join(logs),'utf8')
sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
from unicorn import arm_const as reg
m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
for a,n in [(0x08000000,0x100000),(0x20000000,0x20000),(0x40000000,0x30000),(0xe0000000,0x100000)]:m.mem_map(a,n)
blob=(T/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',blob)
for i in range(elf[12]):
 _,kind,flags,addr,off,size,_,_,_,_=struct.unpack_from('<IIIIIIIIII',blob,elf[6]+i*elf[11])
 if flags&2 and size:
  if kind==1:m.mem_write(addr,blob[off:off+size])
  elif kind==8:m.mem_write(addr,b'\0'*size)
def reset_hook(machine,access,address,size,value,user):
 if address==0x40023824 and value&4096: machine.mem_write(0x40013000,b'\0'*0x24)
m.hook_add(uc.UC_HOOK_MEM_WRITE,reset_hook)
m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,0x080ff001);m.emu_start(elf[4]|1,0x080ff000,count=1000000);result=m.reg_read(reg.UC_ARM_REG_R0);assert result==0,result
(E/'port_arm_result.json').write_text(json.dumps({'passed':True,'actual_port_source':True,'harness_sha256':hashlib.sha256((T/'unit.c').read_bytes()).hexdigest(),'axf_sha256':hashlib.sha256((T/'unit.axf').read_bytes()).hexdigest(),'cases':7,'source_sha256':{str(p.relative_to(R)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in (D/'CHASSIS/HARDWARE/ICM45686').glob('*.[ch]')},'peripheral_reset_emulated':True,'checks':['ready flag after transaction deadline rejects','ready flag within deadline succeeds','stalled DWT plus missing flag bounded by loop budget','OVR with ready flag rejects','stale RXNE triggers peripheral reset','OVR triggers peripheral reset','next ready flag clean after reset'],'hardware':False},indent=2),'utf8');print('port result',result,'cases 7')
