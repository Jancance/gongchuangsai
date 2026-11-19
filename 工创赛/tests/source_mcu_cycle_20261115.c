/* Exercise the real MCU cycle and real committed-piece owner together. */
#define main previous_owner_fixture_main
#include "base-commit-fixture.c"
#undef main
static uint32_t g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched,g_safety_ready=1,g_can_test_result=1;
static uint32_t g_source_body_step_request,g_source_body_step_state,g_source_body_step_reason;
static uint32_t g_it_axes;
volatile uint32_t g_source_align_direction,g_source_align_command_seq,g_source_align_chain_active;
volatile float g_source_align_distance_mm;
static uint32_t beep_until,beep_calls,beep_count,uart_frames;
static uint8_t USART6_VisionSend(const uint8_t *p,uint16_t n){CHECK(n>8 && p[0]=='S');uart_frames++;return 1;}
static void Buzzer_StartTimed(uint32_t count,uint32_t on,uint32_t gap){
 CHECK(count==2 && on==80 && gap==80);beep_calls++;beep_count=count;beep_until=tick+240;
}
static uint32_t Buzzer_IsBusy(void){return tick<beep_until;}
#include "source_mcu_cycle.inc"
static uint32_t g_start_key_count,g_vehicle_key,key_event;
static uint8_t Key_GetEvent(void){uint8_t value=(uint8_t)key_event;key_event=0;return value;}
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static void press_actual_key(void){key_event=1;
#include "tested-key-branch.inc"
}

static void sample(uint32_t pose,uint32_t seq,uint32_t age,uint32_t norm,int right,uint32_t stationary,
 uint32_t left,uint32_t right_color,uint32_t down,uint32_t corrupt){
 char wire[192];unsigned i;int n=snprintf(wire,sizeof(wire),"OC1,%u,%u,%u,1,%d,0,%u,%u,%u,%u,%u,%u,",
 g_source_cycle_run,pose,seq,right,norm,stationary,left,right_color,down,age);
 snprintf(wire+n,sizeof(wire)-(unsigned)n,"%04X\r\n",SC_Crc(wire,(uint32_t)n));
 if(corrupt)wire[n]='Z';
 for(i=0;i<strlen(wire);i++)CHECK(SourceCycle_SharedByte((uint8_t)wire[i],tick,0));
}
static void reset_cycle(void){
 reset();g_it_state=0;g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=0;
 g_source_cycle_request=g_source_cycle_state=g_source_cycle_fault=g_source_cycle_run=0;
 g_source_cycle_pose=g_source_cycle_item=g_source_cycle_prep=0;
 g_source_cycle_rejected=g_source_cycle_body_steps=0;
 g_source_align_direction=g_source_align_command_seq=g_source_align_chain_active=0;
 g_source_body_step_request=g_source_body_step_state=g_source_body_step_reason=0;
 sc_pending=sc_frame=sc_valid=sc_used=sc_active=sc_beep_started=0;
 beep_calls=beep_count=0;
}
static void prepare(void){
 static const uint32_t requests[6]={1,2,7,6,3,3};unsigned i;
 press_actual_key();SourceCycle_Service(tick);CHECK(g_source_cycle_state==1 && g_it_request==1 && !g_vehicle_key);
 for(i=0;i<6;i++){
  CHECK(g_it_request==requests[i]);
  if(i==4)CHECK(g_it_axes==1 && g_it_dm_deg==180 && g_it_extension_mm==0);
  if(i==5)CHECK(g_it_axes==2 && g_it_dm_deg==180 && g_it_extension_mm==65);
  g_it_state=2;g_it_request=0;g_it_action=requests[i];tick+=100;SourceCycle_Service(tick);
 }
 CHECK(g_source_cycle_state==2 && g_source_cycle_prep==6);
}
static void finish_piece(void){
 unsigned loops=0;
 while(Low_Busy() && ++loops<300){
  if(g_low_phase==6){tick=pulse_apply_at+50;Low_Service(tick);tick+=50;Low_Service(tick);}
  else if(owned)finish_group();
  else {tick+=10;Low_Service(tick);}
  SourceCycle_Service(tick);
 }
 CHECK(loops<300 && g_low_state==2 && !g_low_fault);
}
int main(void){
 uint32_t pose;unsigned item;reset_cycle();prepare();
 pose=g_source_cycle_pose;
 sample(pose-1,1,0,10000,12000,0,0,0,0,0);SourceCycle_Service(tick);CHECK(g_source_cycle_state==2);
 sample(pose,1,201,10000,12000,0,0,0,0,0);SourceCycle_Service(tick);CHECK(g_source_cycle_state==2);
 sample(pose,1,0,10000,12000,0,0,0,0,1);SourceCycle_Service(tick);CHECK(g_source_cycle_state==2 && g_source_cycle_rx_bad);
 {FILE *f=fopen("python-observation.txt","rb");int ch;CHECK(f!=0);
  while((ch=fgetc(f))!=EOF)CHECK(SourceCycle_SharedByte((uint8_t)ch,tick,0));fclose(f);}
 SourceCycle_Service(tick);
 CHECK(g_source_cycle_state==3 && g_source_body_step_request==2 && g_source_align_direction==2 && g_source_align_distance_mm==12);
 CHECK(!beep_calls && !g_low_accepted_sequence);
 g_source_body_step_request=0;g_source_body_step_state=3;g_source_align_chain_active=1;g_source_align_command_seq++;
 tick+=100;SourceCycle_Service(tick);CHECK(g_source_cycle_state==2);
 sample(g_source_cycle_pose,1,0,4000,1000,0,0,0,0,0);SourceCycle_Service(tick);
 CHECK(g_source_cycle_state==4 && g_source_body_step_request==7 && !beep_calls);
 g_source_body_step_request=g_source_align_chain_active=0;SourceCycle_Service(tick);CHECK(g_source_cycle_state==5);
 lock_wheels();SourceCycle_Service(tick);CHECK(beep_calls==1 && beep_count==2 && g_source_cycle_state==5);
 tick=beep_until-1;SourceCycle_Service(tick);CHECK(g_source_cycle_state==5);
 tick++;SourceCycle_Service(tick);CHECK(g_source_cycle_state==6);
 for(item=0;item<3;item++){
  sample(g_source_cycle_pose,1,0,0,0,1,5,item==0?2:0,item<2?1:0,0);
  SourceCycle_Service(tick);CHECK(g_source_cycle_state==7 && Low_Busy());
  CHECK(g_low_slot==item+1 && g_low_target_yaw==(item==0?165.0f:item==1?180.0f:195.0f));
  /* No new sample during the entire committed motion. */
  sc_valid=0;finish_piece();CHECK(g_source_cycle_item==item+1);
 }
 CHECK(g_source_cycle_state==8 && g_low_done_sequence==3 && beep_calls==1);
 CHECK(g_source_cycle_body_steps==1 && g_source_cycle_beeps==2);
 SourceCycle_Telemetry(tick);CHECK(uart_frames==1);
 g_source_cycle_request=1;SourceCycle_Service(tick);CHECK(g_source_cycle_state==8 && g_source_cycle_rejected==1);
 reset_cycle();prepare();g_source_cycle_state=6;lock_wheels();
 sample(g_source_cycle_pose,1,0,0,0,1,5,2,1,0);SourceCycle_Service(tick);CHECK(Low_Busy());
 g_safety_fault=1;SourceCycle_Service(tick);CHECK(g_source_cycle_state==9 && g_low_request==9);
 Low_Service(tick);CHECK(g_low_state==9 && g_it_state==9 && g_safety_fault==1);
 reset_cycle();prepare();SourceCycle_KeyEvent();SourceCycle_Service(tick);CHECK(g_source_cycle_state==9 && g_source_cycle_fault==2);
 printf("PASS actual MCU cycle and committed owner: %u checks;6real-prepare requests, CRC/pose/age, STOP-before-beeps,3pieces, camera-free execution, no auto-retry\n",checks);
 return 0;
}
