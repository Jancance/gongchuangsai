#include "prior-continuous-test.c"
int main(void){
 unsigned item;CHECK(prior_continuous_test_main()==0);ready();
 for(item=0;item<3;item++){
  python_packet(item);SourceCycle_Service(tick);CHECK(g_source_cycle_scheduled);
  tick=g_source_cycle_due_tick;SourceCycle_Service(tick);CHECK(Low_Busy());
  finish_piece();CHECK(depth==600 && g_low_return_overlap_axes==7);
  if(item<2)CHECK(g_dm_can2_logical_position_deg==180 && extension==65 && g_source_cycle_state==6);
  else{
   Record *last=&records[recorded-1];
   CHECK(g_source_cycle_state==8 && g_dm_can2_logical_position_deg==0 && extension==0);
   CHECK(last->before_yaw==37 && last->command.goal.dm_deg==0 && last->command.goal.extension_mm==0);
   CHECK(last->command.goal.dm_deg-last->before_yaw==-37);
  }
  CHECK(g_source_cycle_error_milli_mm==4000); /* OC2 isn't a body measurement. */
 }
 printf("PASS final-park actual owner: %u checks; slots1/2 return180/65, slot3 directCW37->0/X0/top600,50ms release and STOP preserved\n",checks);
 return 0;
}
