#define WIRE "\115\101\061\054\061\054\062\054\062\054\061\054\062\054\061\064\054\061\060\054\062\060\054\063\060\054\064\060\054\065\060\054\062\054\061\060\054\062\060\054\071\071\060\054\061\054\070\060\060\060\054\055\064\060\060\060\054\060\054\104\060\062\067\015\012"

#include <stdio.h>
#include <string.h>
#include "source_observer.h"
#include "vehicle_mission.h"
static uint32_t ready=1;static unsigned checks,failures;
static VehicleAlignmentRequest current={2,2,1,2,14,2060,1200,0};
uint32_t VehicleMission_ReadAlignmentRequest(VehicleAlignmentRequest*out){if(!ready)return 0;*out=current;return 1;}
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL line%u\n",__LINE__);}}while(0)
static void feed(const char*p,uint32_t now){while(*p)SourceObserver_Byte((uint8_t)*p++,now);}
int main(void){
 MetricAlignmentProfile p={1,10,20,30,40,50,200,900,100000,10000};char request[256];unsigned i;
 SourceObserver_Init(0);SourceObserver_Service(100);
 CK(g_source_observer_reason==3 && !g_source_observer_accepted && !SourceObserver_Request(request,sizeof(request)));
 feed(WIRE,110);SourceObserver_Service(120);CK(!g_source_observer_accepted);
 SourceObserver_Init(&p);SourceObserver_Service(100);
 CK(SourceObserver_Request(request,sizeof(request))>0);printf("REQUEST:%s",request);
 CK(!SourceObserver_Request(request,sizeof(request)));
 feed(WIRE,125);SourceObserver_Service(140);
 CK(g_source_observer_accepted && g_source_observer_dx_milli_mm==8000 && g_source_observer_dy_milli_mm==-4000);
 CK(g_source_observer_frames==1);
 SourceObserver_Service(321);CK(!g_source_observer_accepted && !g_source_observer_dx_milli_mm);
 SourceObserver_Service(1100);CK(g_source_observer_requests==2);
 feed(WIRE,1110);SourceObserver_Service(1120);CK(!g_source_observer_accepted && g_source_observer_reason==6);
 SourceObserver_Init(&p);SourceObserver_Service(100);
 feed(WIRE,125);feed(WIRE,125);CK(g_source_observer_dropped==1);SourceObserver_Service(140);CK(g_source_observer_accepted);
 ready=0;SourceObserver_Service(150);CK(!g_source_observer_accepted);
 ready=1;SourceObserver_Init(&p);SourceObserver_Service(100);
 for(i=0;i<700;i++)SourceObserver_Byte('A',101);SourceObserver_Byte('\n',101);
 CK(g_source_observer_dropped>0);feed(WIRE,125);SourceObserver_Service(140);CK(g_source_observer_accepted);
 SourceObserver_Init(&p);SourceObserver_Service(100);
 feed("MA1,1,2,",125);SourceObserver_Byte('X',400);SourceObserver_Byte('\n',401);SourceObserver_Service(402);
 CK(!g_source_observer_accepted);
 SourceObserver_Init(&p);SourceObserver_Service(0xfffffff0U);
 feed(WIRE,5);SourceObserver_Service(24);CK(g_source_observer_accepted);
 printf("SOURCE_STREAM checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
