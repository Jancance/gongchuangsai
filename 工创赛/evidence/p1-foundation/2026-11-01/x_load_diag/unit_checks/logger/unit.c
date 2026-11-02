#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "src/x_load_diag.h"
uint32_t g_chassis_auto_active,g_chassis_auto_session,g_chassis_auto_state;
static uint32_t depth;
#define taskENTER_CRITICAL() (++depth)
#define taskEXIT_CRITICAL() (--depth)
#define CHECK(x) do{if(!(x))return 0x80000000U|__LINE__;}while(0)

#include "src/x_load_diag.c"

uint32_t test_entry(void){XSystemSnapshot s={0};XLoadRecord q;XLoadStatistics st;uint32_t i;
 g_chassis_auto_active=1;g_chassis_auto_session=1;g_chassis_auto_state=5;
 s.speed_decirpm=100;s.phase_ma=2800;s.bus_mv=24000;s.home_flags=0x33;s.motor_flags=0x0C;s.position_sign=1;s.position_deci_deg=1234;
 XLoad_RecordCommand(3,200,XLOAD_TX,0,10);XLoad_RecordSnapshot(3,1,&s,11);
 g_chassis_auto_active=0;CHECK(XLoad_Read(1,&q,&st));CHECK(!(q.flags&XLOAD_TARGET_VALID));
 g_chassis_auto_active=1;XLoad_RecordCommand(3,200,XLOAD_ACK,6,12);XLoad_RecordSnapshot(3,1,&s,13);
 g_chassis_auto_active=0;CHECK(XLoad_Read(3,&q,&st));CHECK(!(q.flags&XLOAD_TARGET_VALID));
 g_chassis_auto_active=1;XLoad_RecordCommand(3,200,XLOAD_ACK,1,14);XLoad_RecordCommand(3,300,XLOAD_TX,0,15);
 XLoad_RecordCommand(3,300,XLOAD_STAGED_ACK,1,16);XLoad_RecordSnapshot(3,1,&s,17);
 g_chassis_auto_active=0;CHECK(XLoad_Read(7,&q,&st));CHECK(q.target_decirpm==200 && q.target_ack_ms==14 && (q.flags&2));CHECK(q.faults==0x3F && q.phase_ma==2800 && q.position_deci_deg==1234 && (q.flags&1));
 g_chassis_auto_active=1;XLoad_RecordCommand(3,0,XLOAD_STOP_ACK,1,18);XLoad_RecordSnapshot(3,1,&s,19);
 g_chassis_auto_active=0;CHECK(XLoad_Read(9,&q,&st));CHECK(q.target_decirpm==0 && q.target_ack_ms==18);
 g_chassis_auto_active=1;g_chassis_auto_session=2;
 for(i=0;i<XLOAD_CAPACITY+17;i++)XLoad_RecordSnapshot(2,1,&s,100+i);
 CHECK(!XLoad_Read(0,&q,&st));g_chassis_auto_active=0;CHECK(XLoad_Read(0,&q,&st));CHECK(st.count==XLOAD_CAPACITY && st.dropped==17 && q.tick_ms==100);
 g_chassis_auto_active=1;g_chassis_auto_session=3;s.speed_decirpm=65535;XLoad_RecordSnapshot(1,1,&s,2);
 g_chassis_auto_active=0;CHECK(XLoad_Read(0,&q,&st));CHECK(st.count==1 && st.dropped==0 && q.actual_decirpm==32767 && (q.flags&4));
 XLoad_RecordCommand(0,1,XLOAD_ACK,1,3);XLoad_RecordCommand(5,1,XLOAD_ACK,1,3);CHECK(!depth);return 100U;
}
