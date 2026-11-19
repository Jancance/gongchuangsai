/* Independent boot cue only. No motion request, health write, fault or PC13 gate. */
#include "boot_ready_link.h"
#include "stm32f4xx_rcc.h"
extern volatile uint32_t g_source_cycle_state,g_source_cycle_request,g_stage3_state,g_stage3_request;
extern volatile uint32_t g_startup_gripper_result,g_startup_carrier_result;
volatile uint32_t g_boot_main_init_done,g_boot_app_init_done,g_boot_app_selftests_skipped;
volatile uint32_t g_boot_ready_init_mask,g_boot_ready_link_ok,g_boot_ready_beep_started,g_boot_ready_beep_tick;
volatile uint32_t g_boot_ready_nonce_hi,g_boot_ready_nonce_lo,g_boot_ready_sequence,g_boot_ready_tx,g_boot_ready_rx,g_boot_ready_rx_bad;
volatile uint32_t g_boot_ready_rng_state,g_boot_ready_rng_error,g_boot_ready_offline_skipped;
static uint32_t br_rng_words,br_last_tx,br_active,br_used,br_last_byte,br_line_start=1U;
static char br_input[80];
static uint16_t BR_Crc(const char *data,uint32_t length)
{
 uint16_t crc=0xffffU;uint32_t i,j;
 for(i=0U;i<length;i++){crc^=(uint16_t)((uint16_t)(uint8_t)data[i]<<8);
  for(j=0U;j<8U;j++)crc=(uint16_t)((crc&0x8000U)?((crc<<1)^0x1021U):(crc<<1));}
 return crc;
}
static uint32_t BR_UInt(const char **text,uint32_t *out)
{
 uint32_t n=0U,d;const char*p=*text;if(*p<'0'||*p>'9')return 0U;
 while(*p>='0'&&*p<='9'){d=(uint32_t)(*p-'0');if(n>(0xffffffffU-d)/10U)return 0U;n=n*10U+d;p++;}
 if(*p!=',')return 0U;*text=p+1;*out=n;return 1U;
}
static void BR_Line(void)
{
 const char*p;uint32_t hi,lo,sequence,crc=0U,i,d;
 i=br_used;if(i && br_input[i-1U]=='\r')i--;br_input[i]='\0';p=br_input;
 if(strncmp(p,"BE1,",4U)){g_boot_ready_rx_bad++;return;}p+=4;
 if(!BR_UInt(&p,&hi)||!BR_UInt(&p,&lo)||!BR_UInt(&p,&sequence)){g_boot_ready_rx_bad++;return;}
 if(strlen(p)!=4U){g_boot_ready_rx_bad++;return;}
 for(i=0U;i<4U;i++){char c=p[i];if(c>='0'&&c<='9')d=(uint32_t)(c-'0');
  else if(c>='A'&&c<='F')d=(uint32_t)(c-'A'+10);else if(c>='a'&&c<='f')d=(uint32_t)(c-'a'+10);else{g_boot_ready_rx_bad++;return;}
  crc=(crc<<4)|d;}
 if(BR_Crc(br_input,(uint32_t)(p-br_input))!=crc){g_boot_ready_rx_bad++;return;}
 if(g_boot_ready_rng_state!=2U || !g_boot_ready_tx || hi!=g_boot_ready_nonce_hi || lo!=g_boot_ready_nonce_lo ||
    sequence!=g_boot_ready_sequence || !sequence)return;
 g_boot_ready_rx++;g_boot_ready_link_ok=1U;
}
uint32_t BootReady_SharedByte(uint8_t byte,uint32_t now,uint32_t binary_active)
{
 if(br_active && now-br_last_byte>=200U){br_active=br_used=0U;br_line_start=1U;}
 if(binary_active){br_active=br_used=0U;br_line_start=1U;return 0U;}
 if(!br_active){
  if(!br_line_start || byte!='B'){br_line_start=byte=='\n';return 0U;}
  br_active=1U;br_used=0U;
 }
 br_line_start=byte=='\n';br_last_byte=now;
 if(byte=='\n'){BR_Line();br_active=br_used=0U;return 1U;}
 if((byte<32U && byte!='\r') || br_used>=sizeof(br_input)-1U){br_active=br_used=0U;g_boot_ready_rx_bad++;return 1U;}
 br_input[br_used++]=(char)byte;return 1U;
}
void BootReady_LinkPoll(uint32_t now)
{
#if APP_MOTOR_OFFLINE_ONLY
 (void)now;g_boot_ready_offline_skipped=1U;return;
#else
 char wire[80];int n;uint32_t next,status;
 if(g_boot_ready_link_ok)return;
 if(!g_boot_ready_rng_state){RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG,ENABLE);RNG->CR|=RNG_CR_RNGEN;g_boot_ready_rng_state=1U;return;}
 if(g_boot_ready_rng_state==1U){
  status=RNG->SR;g_boot_ready_rng_error=status&(RNG_SR_CECS|RNG_SR_SECS);
  if(g_boot_ready_rng_error || !(status&RNG_SR_DRDY))return;
  if(!br_rng_words){g_boot_ready_nonce_hi=RNG->DR;br_rng_words=1U;return;}
  g_boot_ready_nonce_lo=RNG->DR;g_boot_ready_rng_state=2U;RNG->CR&=~RNG_CR_RNGEN;
 }
 if(g_boot_ready_tx && now-br_last_tx<500U)return;
 next=g_boot_ready_sequence+1U;if(!next)next=1U;
 n=snprintf(wire,sizeof(wire),"BH1,%lu,%lu,%lu,",(unsigned long)g_boot_ready_nonce_hi,
  (unsigned long)g_boot_ready_nonce_lo,(unsigned long)next);
 if(n<=0 || (uint32_t)n>=sizeof(wire)-7U)return;
 snprintf(wire+n,sizeof(wire)-(uint32_t)n,"%04X\r\n",BR_Crc(wire,(uint32_t)n));
 if(USART6_VisionSend((const uint8_t*)wire,(uint16_t)(n+6))){g_boot_ready_sequence=next;br_last_tx=now;g_boot_ready_tx++;}
#endif
}
static uint32_t BR_ReadyMask(void)
{
 uint32_t mask=0U;
 if(g_boot_main_init_done==1U)mask|=1U;
 if(g_boot_app_init_done==1U)mask|=2U;
 if(g_can_test_result==1U)mask|=4U;
 if(g_safety_ready && !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched)mask|=8U;
 if(g_watchdog_init_result==1U)mask|=16U;
 if(g_screen_init_done==1U)mask|=32U;
#if !APP_MOTOR_OFFLINE_ONLY
 if(g_radar_servo_state==1U && g_startup_gripper_result==1U &&
    ((STARTUP_CARRIER_ANGLE_DEG==-1.0f && g_startup_carrier_result==0U) ||
     (STARTUP_CARRIER_ANGLE_DEG!=-1.0f && g_startup_carrier_result==1U)))mask|=64U;
#endif
 return mask;
}
void BootReady_BeepStep(void)
{
 g_boot_ready_init_mask=BR_ReadyMask();
#if APP_MOTOR_OFFLINE_ONLY
 g_boot_ready_offline_skipped=1U;return;
#else
 if(g_boot_ready_beep_started || !g_boot_ready_link_ok || g_boot_ready_init_mask!=127U)return;
 /* The existing business80ms double cue owns its original sequence. Do not
  * replace any active cue or insert boot audio into an active/pending run. */
 taskENTER_CRITICAL();
 if(!g_boot_ready_beep_started && g_boot_ready_link_ok && BR_ReadyMask()==127U && !Buzzer_IsBusy() &&
    !g_source_cycle_request && (!g_source_cycle_state || g_source_cycle_state>=8U) &&
    !g_stage3_request && (!g_stage3_state || g_stage3_state==6U || g_stage3_state==11U)){
  g_boot_ready_beep_started=1U;g_boot_ready_beep_tick=(uint32_t)xTaskGetTickCount();Buzzer_Start(APP_INIT_BEEP_COUNT);
 }
 taskEXIT_CRITICAL();
#endif
}
