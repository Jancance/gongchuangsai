#include <stdio.h>
#include <stdlib.h>
#include "driver_config_reply.h"
static unsigned checks;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t feed(DriverConfigReply*s,const uint8_t raw[36])
{
 uint8_t frame[8];uint32_t i,result;
 result=DriverConfigReply_Push(s,0,raw,8);CHECK(!result);
 for(i=1;i<5;i++){
  frame[0]=0x42;memcpy(frame+1,raw+8+(i-1)*7,7);
  result=DriverConfigReply_Push(s,i,frame,8);
  if(i<4)CHECK(!result);
 }
 return result;
}
int main(void)
{
 DriverConfigReply state={0};uint8_t raw[36]={0x42,0x25,0x18},frame[8]={0x42};unsigned i;
 for(i=3;i<35;i++)raw[i]=(uint8_t)i;raw[35]=0x6b;
 CHECK(feed(&state,raw)==1);CHECK(state.length==36 && state.packet==5 && !memcmp(raw,state.data,36));
 memset(&state,0,sizeof(state));CHECK(DriverConfigReply_Push(&state,1,raw,8)==2);
 memset(&state,0,sizeof(state));CHECK(DriverConfigReply_Push(&state,0,raw,9)==2);
 memset(&state,0,sizeof(state));raw[1]=0x24;CHECK(feed(&state,raw)==2);raw[1]=0x25;
 memset(&state,0,sizeof(state));raw[35]=0;CHECK(feed(&state,raw)==2);raw[35]=0x6b;
 memset(&state,0,sizeof(state));CHECK(!DriverConfigReply_Push(&state,0,raw,8));
 CHECK(DriverConfigReply_Push(&state,0,frame,8)==2);
 memset(&state,0,sizeof(state));state.length=35;CHECK(DriverConfigReply_Push(&state,0,frame,8)==2);
 CHECK(DriverConfigReply_Push(0,0,frame,8)==2);
 printf("PASS read-only config parser: %u checks; five packets, count/header/checksum/order/overflow\n",checks);
 return 0;
}
