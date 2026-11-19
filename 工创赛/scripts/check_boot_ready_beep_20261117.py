"""Actual C boot module and Pi echo regression; no hardware imports or access."""
import sys,json,subprocess,hashlib,textwrap
from pathlib import Path
from prepare_boot_ready_beep_20261117 import R,BASE,P,PI,PIBASE,E

def ctest():
 stub=E/'stubs';stub.mkdir(exist_ok=True)
 (stub/'stm32f4xx_rcc.h').write_text('#define RCC_AHB2Periph_RNG 1\n#define ENABLE 1\nstatic void RCC_AHB2PeriphClockCmd(unsigned p,unsigned e){(void)p;(void)e;}\n')
 pre=r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define RNG_CR_RNGEN 4
#define RNG_SR_DRDY 1
#define RNG_SR_CECS 2
#define RNG_SR_SECS 4
#define STARTUP_CARRIER_ANGLE_DEG (-1.0f)
#define APP_INIT_BEEP_COUNT 2
static struct {uint32_t CR,SR,DR;} fake_rng;
#define RNG (&fake_rng)
static unsigned checks,depth,beeps,busy,txbusy,tick,send_calls;static char wire[100];
static uint32_t g_can_test_result,g_safety_ready,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched,g_watchdog_init_result,g_screen_init_done,g_radar_servo_state;
volatile uint32_t g_startup_gripper_result,g_startup_carrier_result,g_source_cycle_state,g_source_cycle_request,g_stage3_state,g_stage3_request;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL line%u %s\n",__LINE__,#x);exit(1);}}while(0)
#define taskENTER_CRITICAL() (++depth)
#define taskEXIT_CRITICAL() (--depth)
static unsigned xTaskGetTickCount(void){return tick;}
static unsigned Buzzer_IsBusy(void){return busy;}
static void Buzzer_Start(unsigned count){CHECK(depth && count==2);beeps++;busy=1;}
static unsigned USART6_VisionSend(const uint8_t *data,uint16_t length){send_calls++;if(txbusy)return 0;CHECK(length<sizeof(wire));memcpy(wire,data,length);wire[length]=0;return 1;}
'''
 actual=(P/'CHASSIS/USER/boot_ready_link.inc').read_text(encoding='ascii')
 post=r'''
static void reset(void){
 g_boot_ready_rng_state=g_boot_ready_rng_error=br_rng_words=g_boot_ready_link_ok=g_boot_ready_beep_started=0;
 g_boot_ready_tx=g_boot_ready_rx=g_boot_ready_rx_bad=g_boot_ready_sequence=br_last_tx=br_active=br_used=0;br_line_start=1;
 g_boot_main_init_done=g_boot_app_init_done=g_can_test_result=g_safety_ready=g_watchdog_init_result=g_screen_init_done=g_radar_servo_state=g_startup_gripper_result=1;
 g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=g_startup_carrier_result=0;
 g_source_cycle_state=g_source_cycle_request=g_stage3_state=g_stage3_request=0;beeps=busy=txbusy=send_calls=depth=tick=0;fake_rng.CR=fake_rng.SR=fake_rng.DR=0;}
static void feed(const char*s){while(*s)BootReady_SharedByte((uint8_t)*s++,tick++,0);}
static void echo(uint32_t hi,uint32_t lo,uint32_t seq){char b[100];int n=snprintf(b,sizeof(b),"BE1,%lu,%lu,%lu,",(unsigned long)hi,(unsigned long)lo,(unsigned long)seq);snprintf(b+n,sizeof(b)-n,"%04X\r\n",BR_Crc(b,n));feed(b);}
static void ready(void){reset();BootReady_LinkPoll(0);CHECK(g_boot_ready_rng_state==1 && !send_calls);
 fake_rng.SR=1;fake_rng.DR=123;BootReady_LinkPoll(1);CHECK(br_rng_words==1 && !send_calls);
 fake_rng.DR=456;BootReady_LinkPoll(2);CHECK(g_boot_ready_rng_state==2 && g_boot_ready_tx==1 && g_boot_ready_sequence==1);}
int main(void){unsigned i;uint32_t*flags[7]={&g_boot_main_init_done,&g_boot_app_init_done,&g_can_test_result,&g_safety_ready,&g_watchdog_init_result,&g_screen_init_done,&g_radar_servo_state};
#if APP_MOTOR_OFFLINE_ONLY
 reset();BootReady_LinkPoll(0);BootReady_BeepStep();CHECK(g_boot_ready_offline_skipped && !send_calls && !beeps && !fake_rng.CR);puts("PASS actual boot Offline: explicit SKIPPED no RNG/UART/beep output");return 0;
#else
 reset();for(i=0;i<10;i++){BootReady_LinkPoll(i);BootReady_BeepStep();}CHECK(!beeps && !g_boot_ready_link_ok && !send_calls);
 reset();BootReady_LinkPoll(0);for(i=0;i<8;i++)if(i!=1){fake_rng.SR=i;BootReady_LinkPoll(i+1);CHECK(!br_rng_words && !send_calls);}CHECK(!g_safety_fault);
 ready();CHECK(BR_Crc("123456789",9)==0x29b1);feed("random RX\nOC1,1,2,B3,FFFF\n");CHECK(!g_boot_ready_link_ok);
 echo(122,456,1);echo(123,455,1);echo(123,456,0);echo(123,456,2);CHECK(!g_boot_ready_link_ok);
 feed("BE1,123,456,1,0000\n");CHECK(!g_boot_ready_link_ok && g_boot_ready_rx_bad);
 {char text[100];int n=snprintf(text,sizeof(text),"BE1,123,456,1,");snprintf(text+n,sizeof(text)-n,"%04X\n",BR_Crc(text,n));memmove(text+7,text+6,strlen(text+6)+1);text[6]='\r';feed(text);CHECK(!g_boot_ready_link_ok);}
 echo(123,456,1);CHECK(g_boot_ready_link_ok && g_boot_ready_rx==1);BootReady_BeepStep();CHECK(beeps==1 && g_boot_ready_beep_started && !depth);
 busy=0;for(i=0;i<5;i++)BootReady_BeepStep();CHECK(beeps==1);
 for(i=0;i<7;i++){ready();echo(123,456,1);*flags[i]=0;BootReady_BeepStep();CHECK(!beeps);*flags[i]=1;BootReady_BeepStep();CHECK(beeps==1);}
 ready();echo(123,456,1);g_startup_gripper_result=0;BootReady_BeepStep();CHECK(!beeps);g_startup_gripper_result=1;g_startup_carrier_result=2;BootReady_BeepStep();CHECK(!beeps);
 ready();echo(123,456,1);busy=1;BootReady_BeepStep();CHECK(!beeps);busy=0;g_source_cycle_state=5;BootReady_BeepStep();CHECK(!beeps);g_source_cycle_state=0;g_source_cycle_request=1;BootReady_BeepStep();CHECK(!beeps);g_source_cycle_request=0;g_stage3_request=1;BootReady_BeepStep();CHECK(!beeps);g_stage3_request=0;BootReady_BeepStep();CHECK(beeps==1);
 ready();txbusy=1;BootReady_LinkPoll(600);CHECK(g_boot_ready_sequence==1);txbusy=0;BootReady_LinkPoll(601);CHECK(g_boot_ready_sequence==2);echo(123,456,1);CHECK(!g_boot_ready_link_ok);echo(123,456,2);CHECK(g_boot_ready_link_ok);
 ready();BootReady_SharedByte('B',10,1);feed("E1,123,456,1,FFFF\n");CHECK(!g_boot_ready_link_ok);echo(123,456,1);CHECK(g_boot_ready_link_ok);
 ready();BootReady_SharedByte('B',0,0);BootReady_SharedByte('E',300,0);feed("1,123,456,1,0000\n");CHECK(!g_boot_ready_link_ok);echo(123,456,1);CHECK(g_boot_ready_link_ok);
 CHECK(!g_source_cycle_request && !g_stage3_request && !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched);
 printf("PASS actual boot module: %u checks; RNG/CRC/session/seq/txbusy/binary/split/init/once/business-cue coexistence; no motion writes\n",checks);return 0;
#endif
}
'''
 path=E/'actual-boot-ready.c';path.write_text(pre+actual+post,encoding='utf-8');results={}
 for offline in (0,1):
  name='actual-boot-ready'+('-offline'if offline else '');exe=E/(name+'.exe')
  r=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Wno-unused-function','-Wno-unused-variable','-Wno-misleading-indentation','-DAPP_MOTOR_OFFLINE_ONLY='+str(offline),'-I'+str(stub),'-I'+str(P/'CHASSIS/USER'),str(path),'-o',str(exe)],capture_output=True,text=True)
  (E/(name+'-build.log')).write_text(r.stdout+r.stderr);assert not r.returncode,r.stderr
  r=subprocess.run([str(exe)],capture_output=True,text=True);(E/(name+'.log')).write_text(r.stdout+r.stderr);print(r.stdout+r.stderr,flush=True);assert not r.returncode;results[name]=r.stdout.strip()
 return results

def pitest():
 sys.path.insert(0,str(PI));from boot_ready_handshake import BootReadyHandshake
 from binascii import crc_hqx
 def frame(prefix,hi=123,lo=456,seq=1):
  b=f'{prefix},{hi},{lo},{seq},'.encode();return b+f'{crc_hqx(b,0xffff):04X}'.encode()
 checks=0;sent=[];h=BootReadyHandshake()
 def write(data):sent.append(data);return len(data)
 assert not h.receive(b'SC2,unrelated',write);assert not sent;checks+=1
 for bad in (b'BH1,1,2,3,0000',frame('BH1',seq=0),frame('BH1',hi=-1),frame('BH1',lo=2**32),b'BH1,'+b'9'*90+b',1,1,0000'):
  assert h.receive(bad,write) and not sent;checks+=1
 assert h.receive(frame('BH1')+b'\r',write);assert sent==[frame('BE1')+b'\r\n'];checks+=1
 try:h.receive(frame('BH1'),lambda data:0)
 except OSError:pass
 else:raise AssertionError('short write accepted')
 checks+=1
 # Execute the actual entry's receive dispatcher fragment, including preview mode.
 s=(PI/'source_fast_cycle_view_20261115.py').read_text(encoding='utf-8')
 start=s.index("            while b'\\n' in buffer:");end=s.index('        now=time.monotonic()',start)
 block=textwrap.indent(textwrap.dedent(s[start:end]),'    ')
 received=[]
 class Region:
  def receive(self,line,now):received.append(line);return True
 class Link:pass
 Link.write=staticmethod(write)
 class Dummy:
  def invalidate(self,*a):pass
  def reset(self):pass
 import time
 env=dict(boot_handshake=h,region_ui=Region(),link=Link(),time=time,frame_epoch=Dummy(),gate=Dummy(),direct=Dummy(),initial_rotation=Dummy(),alignment_settle=Dummy())
 code='def dispatcher(buffer):\n'+block+'    return buffer\n';exec(code,env)
 sent.clear();remain=env['dispatcher'](frame('BH1')+b'\r\n'+b'SC2,after\n')
 assert len(sent)==1 and received==[b'SC2,after'] and remain==b'';checks+=1
 assert s.index('boot_handshake.receive(line,link.write)')<s.index('if region_ui.receive(')
 assert "'boot_handshake_tx_enabled':True"in s and "'observation_tx_enabled':not args.preview_only"in s;checks+=1
 (E/'pi-checks.json').write_text(json.dumps({'passed':True,'checks':checks,'actual_dispatcher':True,'preview_echo_separate_from_OC':True,'hardware_access':False},indent=2)+'\n');print('PASS actual Pi boot echo/entry dispatcher:',checks,flush=True)
 return checks

def scope():
 for n in ('CHASSIS/USER/source_mcu_cycle.inc','CHASSIS/USER/source_body_step.inc','CHASSIS/USER/region_dispatch.inc','CHASSIS/USER/startup_axes.c','CHASSIS/USER/extension_home_only.inc','USER/vehicle_parameters.h','CHASSIS/SYSTEM/CAN/can.c','CHASSIS/HARDWARE/BUZZER/buzzer.c'):
  assert (BASE/n).read_bytes()==(P/n).read_bytes(),n
 print('PASS original PC13/SC state5 80ms cues/mirror controls/buzzer code byte-identical',flush=True)

def main():
 results=ctest();pi=pitest();scope();(E/'checks.json').write_text(json.dumps({'passed':True,'actual_c':results,'pi_checks':pi,'business80ms_PC13_unchanged':True,'hardware_access':False},indent=2)+'\n')
if __name__=='__main__':main()
