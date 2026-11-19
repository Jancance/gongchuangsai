#include "static-cycle-fixture.c"

static void ready(void){
 reset_cycle();g_source_cycle_scheduled=0;prepare();
 sample(g_source_cycle_pose,1,0,4000,1000,0,0,0,0,0);SourceCycle_Service(tick);
 CHECK(g_source_cycle_state==5);lock_wheels();SourceCycle_Service(tick);
 tick=beep_until;SourceCycle_Service(tick);CHECK(g_source_cycle_state==6 && g_source_cycle_pose==4);
}
static void python_packet(unsigned index){
 char name[64];FILE *f;int ch;snprintf(name,sizeof(name),"python-continuous-%u.txt",index);
 f=fopen(name,"rb");CHECK(f!=0);while((ch=fgetc(f))!=EOF)CHECK(SourceCycle_SharedByte((uint8_t)ch,tick,0));fclose(f);
}
int main(void){
 unsigned item;ready();
 /* Old stopped-disc packets cannot start the continuously rotating-disc owner. */
 sample(g_source_cycle_pose,1,0,0,0,1,5,2,1,0);SourceCycle_Service(tick);
 CHECK(g_source_cycle_state==6 && !Low_Busy() && !g_low_accepted_sequence);
 for(item=3;item<=6;item++){
  python_packet(item);SourceCycle_Service(tick);
  CHECK(g_source_cycle_state==6 && !Low_Busy() && !g_low_accepted_sequence);
 }
 CHECK(g_source_cycle_rx_bad>0);
 for(item=0;item<3;item++){
  unsigned before=recorded;
  python_packet(item);SourceCycle_Service(tick);
  CHECK(g_source_cycle_state==7 && g_source_cycle_scheduled==1 && !Low_Busy());
  CHECK(g_source_cycle_planned_region==(item==0?1:item==1?2:0));
  CHECK(recorded==before && g_source_cycle_due_tick==tick+100);
  sc_valid=0;tick+=99;SourceCycle_Service(tick);CHECK(!Low_Busy() && recorded==before);
  tick++;SourceCycle_Service(tick);CHECK(Low_Busy() && !g_source_cycle_scheduled);
  CHECK(g_low_target_yaw==(item==0?165.0f:item==1?180.0f:195.0f));
  CHECK(g_low_slot==item+1);finish_piece();CHECK(g_source_cycle_grip_command_ms[item]>0);
 }
 CHECK(g_source_cycle_state==8 && g_low_done_sequence==3 && beep_calls==1);
 ready();python_packet(0);SourceCycle_Service(tick);CHECK(g_source_cycle_scheduled);
 SourceCycle_KeyEvent();SourceCycle_Service(tick);
 CHECK(g_source_cycle_state==9 && g_source_cycle_fault==2 && !g_source_cycle_scheduled && !Low_Busy());
 printf("PASS actual continuous MCU owner: %u checks; PythonOC2, realPC13, scheduled3pieces, actual colours separate, no stopped-disc trigger, existing STOP\n",checks);
 return 0;
}
