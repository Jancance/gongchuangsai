"""Actual UART5 parser, task setter, MCU gate/STOP and dual Keil build; no hardware."""
from pathlib import Path
import hashlib,json,shutil,subprocess,sys
from prepare_source_real_qr_20261116 import R,BASE,P,E

STUB=r'''
#include <stdint.h>
typedef uint32_t TickType_t;typedef int BaseType_t;
#define pdFALSE 0
#define pdPASS 1
#define portTICK_PERIOD_MS 1
#define pdMS_TO_TICKS(x) (x)
#define taskENTER_CRITICAL() do{critical++;}while(0)
#define taskEXIT_CRITICAL() do{CHECK(critical);critical--;}while(0)
#define configASSERT(x) CHECK(x)
#define portYIELD_FROM_ISR(x) ((void)(x))
static unsigned critical;
typedef struct{unsigned SR,DR;} USART_TypeDef;
static USART_TypeDef uart;
#define UART5 (&uart)
typedef struct {unsigned GPIO_Pin,GPIO_Mode,GPIO_PuPd;} GPIO_InitTypeDef;
typedef struct {unsigned USART_BaudRate,USART_Mode;} USART_InitTypeDef;
typedef struct {unsigned NVIC_IRQChannel,NVIC_IRQChannelPreemptionPriority,NVIC_IRQChannelSubPriority,NVIC_IRQChannelCmd;} NVIC_InitTypeDef;
#define GPIOD 0
#define GPIO_PinSource2 2
#define GPIO_AF_UART5 5
#define GPIO_Pin_2 4
#define GPIO_Mode_AF 2
#define GPIO_PuPd_UP 1
#define RCC_AHB1Periph_GPIOD 1
#define RCC_APB1Periph_UART5 1
#define ENABLE 1
#define DISABLE 0
#define USART_Mode_Rx 1
#define USART_IT_RXNE 1
#define UART5_IRQn 5
#define USART_FLAG_RXNE 1
#define USART_FLAG_ORE 2
#define USART_FLAG_FE 4
#define USART_FLAG_NE 8
#define USART_FLAG_PE 16
static unsigned irq_enabled;
#define RCC_AHB1PeriphClockCmd(a,b) ((void)(a),(void)(b))
#define RCC_APB1PeriphClockCmd(a,b) ((void)(a),(void)(b))
#define GPIO_PinAFConfig(a,b,c) ((void)(a),(void)(b),(void)(c))
#define GPIO_StructInit(a) memset(a,0,sizeof(*(a)))
#define GPIO_Init(a,b) ((void)(a),(void)(b))
#define USART_StructInit(a) memset(a,0,sizeof(*(a)))
#define USART_Init(a,b) ((void)(a),(void)(b))
#define USART_ITConfig(a,b,c) ((void)(a),(void)(b),irq_enabled=(c))
#define NVIC_Init(a) ((void)(a))
#define USART_Cmd(a,b) ((void)(a),(void)(b))
static uint32_t xTaskGetTickCountFromISR(void){return tick;}
typedef struct{unsigned rd,wr,n;struct{uint32_t tick;uint8_t data;}v[128];}Queue;
typedef Queue* QueueHandle_t;static Queue queue;
static QueueHandle_t xQueueCreate(unsigned n,unsigned size){CHECK(n==128 && size==sizeof(queue.v[0]));return &queue;}
static int xQueueReset(QueueHandle_t q){q->rd=q->wr=q->n=0;return 1;}
static int xQueueSendFromISR(QueueHandle_t q,const void *p,BaseType_t *w){(void)w;if(q->n==128)return 0;memcpy(&q->v[q->wr++%128],p,sizeof(q->v[0]));q->n++;return 1;}
static int xQueueReceive(QueueHandle_t q,void *p,unsigned timeout){(void)timeout;if(!q->n)return 0;memcpy(p,&q->v[q->rd++%128],sizeof(q->v[0]));q->n--;return 1;}
static unsigned uxQueueMessagesWaiting(QueueHandle_t q){return q->n;}
volatile uint32_t g_scan_display_result,g_scan_display_count;
static unsigned displays;static char display_code[16];
uint8_t Screen_ShowTaskCode(const char *code){CHECK(!critical && strlen(code)==15);memcpy(display_code,code,16);displays++;return 1;}
static unsigned preempt_lock;
static void SourceCycle_Service(uint32_t);
static void SC_TestPreempt(void){if(preempt_lock){preempt_lock=0;SourceCycle_Service(tick);}}
'''

TEST=r'''
static void feed(const char *code){unsigned i;for(i=0;i<strlen(code);i++){if(irq_enabled){uart.SR=USART_FLAG_RXNE;uart.DR=(uint8_t)code[i];UART5_IRQHandler();}tick++;}}
static void a(void){SourceCycle_ScanService();SAOMA_Update();}
static void round_reset(void){
 reset_cycle();g_it_request=0;g_vis_pending_seq=0;g_vis_task_valid=0;
 g_source_scan_owner_run=g_source_scan_locked_run=g_source_scan_pending=g_source_scan_displayed_run=0;
 g_source_scan_rejected=0;sc_scan_seen=0;g_saoma_locked=0;
 SAOMA_Init();SAOMA_EnableReceive();
}
static void start_wait(void){
 SourceCycle_KeyEvent();SourceCycle_Service(tick);
 CHECK(g_source_cycle_run==1 && g_source_cycle_pose==1 && g_source_cycle_state==1 && g_source_cycle_prep==0);
 CHECK(!g_it_request && !g_source_body_step_request && !g_low_request && !g_source_scan_locked_run);
 CHECK(!g_source_cycle_colors[0] && !g_source_cycle_colors[1] && !g_source_cycle_colors[2]);
}
int main(void){
 const char *codes[]={"126+123+621+231","162+231+621+312","216+312+621+123","261+132+621+213","612+213+621+132","621+321+621+321"};
 unsigned i,j,x,y,z,before;char held[16],code[16];
 for(x=1;x<=6;x++)for(y=1;y<=6;y++)for(z=1;z<=6;z++)if(x!=y && y!=z && x!=z){
  round_reset();start_wait();a();snprintf(code,sizeof(code),"%u%u%u+123+%u%u%u+321",x,y,z,z,x,y);
  feed(code);feed("\r");a();a();SourceCycle_Service(tick);
  CHECK(g_source_scan_locked_run==1 && g_source_cycle_colors[0]==x && g_source_cycle_colors[1]==y && g_source_cycle_colors[2]==z);
 }
 for(i=0;i<6;i++){
  round_reset();feed("215+123+512+321\r");SAOMA_Update();CHECK(SAOMA_DataReady);
  feed("215+12");SAOMA_Update();CHECK(s_active);
  start_wait();a();CHECK(!s_active && !queue.n && !SAOMA_DataReady);
  SourceCycle_Service(tick);CHECK(!g_it_request);
  feed("3+512+321\r");a();a();SourceCycle_Service(tick);CHECK(!g_it_request && !g_source_scan_locked_run);
  feed("111+123+111+123\r");a();a();SourceCycle_Service(tick);CHECK(g_source_scan_rejected==1 && !g_it_request);
  feed("126+112+621+231\r");a();a();SourceCycle_Service(tick);CHECK(g_source_scan_rejected==2 && !g_it_request);
  feed("126+123+625+231\r");a();a();SourceCycle_Service(tick);CHECK(g_source_scan_rejected==3 && !g_it_request);
  feed(codes[i]);feed("\r");a();a();CHECK(g_source_scan_pending);
  memcpy(held,g_source_scan_code,16);feed("215+123+512+321\r");SAOMA_Update();
  CHECK(!memcmp(held,g_source_scan_code,16));
  /* Deterministic preemption after TaskA's first locked_run read: CAN consumes
   * pending first code and releases mailbox before TaskA's publication check. */
  preempt_lock=1;SourceCycle_ScanService();CHECK(!preempt_lock && !memcmp(held,g_source_scan_code,16));
  CHECK(g_source_scan_locked_run==1 && g_it_request==1);
  CHECK(g_vis_task_valid && !g_vis_loaded_mask && !g_vis_batch_complete && !g_vis_pending_seq);
  for(j=0;j<3;j++)CHECK(g_source_cycle_colors[j]==(unsigned)(codes[i][j]-'0'));
  memcpy(held,g_source_scan_code,16);before=displays;a();CHECK(g_saoma_locked && !irq_enabled && g_source_scan_displayed_run==1 && displays==before+1);
  a();CHECK(displays==before+1 && !memcmp(display_code,held,16));
  feed("215+123+512+321\r");a();SourceCycle_Service(tick);CHECK(!memcmp(held,g_source_scan_code,16));
  CHECK(g_source_scan_accept>0 && g_source_scan_result==1 && g_source_scan_frame_end_ms>=g_source_scan_frame_start_ms);
  SourceCycle_KeyEvent();SourceCycle_Service(tick);CHECK(g_source_cycle_state==9 && g_source_cycle_fault==2);
 }
 /* STOP while waiting cannot be bypassed by an already-published valid code. */
 round_reset();start_wait();a();feed(codes[0]);feed("\r");a();a();CHECK(g_source_scan_pending);
 SourceCycle_KeyEvent();SourceCycle_Service(tick);CHECK(g_source_cycle_state==9 && !g_it_request && !g_source_scan_locked_run);
 /* Gap-delimited real frame is also accepted; incomplete/noisy UART frame is not. */
 round_reset();start_wait();a();feed("126+123");a();tick+=50;a();a();SourceCycle_Service(tick);CHECK(!g_it_request);
 feed(codes[0]);a();tick+=50;a();a();SourceCycle_Service(tick);CHECK(g_source_scan_locked_run==1 && g_it_request==1);
 CHECK(!critical);
 printf("PASS actual UART5 IRQ/parser + CAN gate/task setter: %u checks;120ordered colors, stale complete/partial, invalid rescan, pending+locked duplicate, deterministic CAN preemption, screen once, gaps, STOP, inventory not fabricated\n",checks);return 0;
}
'''

def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed']
 old=R/'evidence/integration/2026-11-16/loaded-retract30-gate'
 for p in old.iterdir():
  if p.suffix in ('.h','.inc','.c','.txt'):shutil.copyfile(p,E/p.name)
 # Stub MCU registers, RTOS queues and clock only; parser and owner are actual C.
 for h in ('stm32f4xx.h','FreeRTOS.h','task.h','queue.h'):(E/h).write_text('/* fixture platform supplied in translation unit */\n')
 (E/'screen.h').write_text('#include <stdint.h>\n#define SCREEN_OK 1U\nuint8_t Screen_ShowTaskCode(const char *);\n')
 shutil.copyfile(P/'CHASSIS/HARDWARE/SAOMA/saoma.h',E/'saoma.h')
 shutil.copyfile(P/'CHASSIS/HARDWARE/SAOMA/saoma.c',E/'tested-saoma.c')
 integration=(P/'CHASSIS/USER/integration10_complete.inc').read_text(encoding='utf-8')
 task=integration[integration.index('volatile uint32_t g_vis_task_valid'):integration.index('/* Only the existing mission/CAN owner')]
 fixture=(old/'static-cycle-fixture.c').read_text()
 cycle=(P/'CHASSIS/USER/source_mcu_cycle.inc').read_text()
 boundary=' if(state!=1U || !run || g_source_scan_owner_run!=run || g_source_scan_pending)return;'
 assert cycle.count(boundary)==1
 (E/'tested-source-cycle.inc').write_text(cycle.replace(boundary,' SC_TestPreempt(); /* only fixture scheduling hook; no production change */\n'+boundary))
 fixture=fixture.replace('#include "source_mcu_cycle.inc"',STUB+'\n#include "tested-saoma.c"\n'+task+'\n#include "tested-source-cycle.inc"')
 fixture=fixture.replace('#define taskENTER_CRITICAL() ((void)0)\n#define taskEXIT_CRITICAL() ((void)0)','')
 # The old fixture's static prepare is retained but never invoked; current gate is tested below.
 (E/'actual-real-qr.c').write_text(fixture+'\n'+TEST)
 exe=E/'actual-real-qr.exe'
 subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Wno-unused-function','-Wno-misleading-indentation','-I'+str(E),'-I'+str(P/'CHASSIS/USER'),'-I'+str(P/'USER'),str(E/'actual-real-qr.c'),str(P/'CHASSIS/USER/source_parallel_flow.c'),'-lm','-o',str(exe)],check=True)
 r=subprocess.run([str(exe)],capture_output=True,text=True);(E/'actual-real-qr.log').write_text(r.stdout+r.stderr);print(r.stdout+r.stderr,flush=True);assert r.returncode==0
 if '--c-only' in sys.argv:
  p=E/'checks.json';checks=json.loads(p.read_text());checks['actual_c']=r.stdout.strip();p.write_text(json.dumps(checks,indent=2)+'\n');return
 changes={}
 unchanged={}
 for path in P.rglob('*'):
  if not path.is_file() or any(x in ('Objects','Listings') for x in path.relative_to(P).parts):continue
  rel=path.relative_to(P).as_posix();before=(BASE/rel).read_bytes();data=path.read_bytes()
  if before!=data:
   assert before.count(b'\n')==before.count(b'\r\n') and data.count(b'\n')==data.count(b'\r\n')
   changes[rel]=hashlib.sha256(data).hexdigest()
 assert set(changes)=={'CHASSIS/USER/source_mcu_cycle.inc','CHASSIS/USER/main.c','CHASSIS/USER/vehicle_arm.h'}
 for name in ('CHASSIS/USER/pzq_commit_overlap.inc','CHASSIS/USER/source_body_step.inc','CHASSIS/USER/vehicle_source_parallel.inc','CHASSIS/USER/source_parallel_flow.c','CHASSIS/USER/integration10_complete.inc','CHASSIS/HARDWARE/SAOMA/saoma.c','USER/vehicle_parameters.h'):
  unchanged[name]=hashlib.sha256((P/name).read_bytes()).hexdigest()
 candidate=dict(base=BASE.relative_to(R).as_posix(),project=P.relative_to(R).as_posix(),changes=changes,unchanged_sha256=unchanged,images={},build_passed=False,flashed=False,hardware_access=False,matlab_prior_to_implementation=True)
 for target in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
  for folder in ('Objects','Listings'):(P/folder/target).mkdir(parents=True,exist_ok=True)
  log=E/(target+'-build.log')
  r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
  data=log.read_bytes();print(data.decode('gb18030',errors='replace')[-1100:],flush=True)
  assert r.returncode==0 and b'0 Error(s), 0 Warning(s)' in data
  axf=P/'Objects'/target/(target+'.axf');candidate['images'][target]=dict(axf=axf.relative_to(R).as_posix(),sha256=hashlib.sha256(axf.read_bytes()).hexdigest())
 candidate['build_passed']=True
 (E/'candidate.json').write_text(json.dumps(candidate,indent=2)+'\n')
 checks=dict(passed=True,actual_c=(E/'actual-real-qr.log').read_text().strip(),matlab_checks=json.loads((E/'matlab-result.json').read_text())['checks'],dual_build=True,hardware_access=False,flashed=False,physical_scan_and_three_piece_open=True)
 (E/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
 print(json.dumps(candidate['images']),flush=True)
if __name__=='__main__':main()
