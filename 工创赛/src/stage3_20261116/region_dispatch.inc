/* Exactly one region owns mechanical commands and the text UART observation. */
volatile uint32_t g_region_dispatch_area,g_region_dispatch_rejected;
volatile uint32_t g_region_dispatch_test_enable=1U;
uint32_t RegionDispatch_Arrive(uint32_t area)
{
 if((area!=1U && area!=3U) ||
    (g_source_cycle_state && g_source_cycle_state!=8U) ||
    (g_stage3_state && g_stage3_state!=6U && g_stage3_state!=11U) ||
    g_it_request || g_it_state==1U || g_it_state==9U || Low_Busy() ||
    SourceBodyStep_Busy() || SourceBodyStep_WheelWindow() || g_chassis_auto_active){
  g_region_dispatch_rejected++;return 0U;
 }
 g_region_dispatch_area=area;
 if(area==1U)g_source_cycle_request=1U;else Stage3_Arrive();
 return 1U;
}
void RegionDispatch_KeyEvent(void)
{
 if(g_region_dispatch_area==1U){SourceCycle_KeyEvent();return;}
 if(g_region_dispatch_test_enable){g_region_dispatch_area=3U;Stage3_KeyEvent();}
}
void RegionDispatch_ScanService(void)
{if(g_region_dispatch_area==1U)SourceCycle_ScanService();}
static void RegionDispatch_Service(uint32_t now)
{if(g_region_dispatch_area==1U)SourceCycle_Service(now);else if(g_region_dispatch_area==3U)Stage3_Service(now);}
static uint32_t RegionDispatch_SharedByte(uint8_t b,uint32_t now,uint32_t binary_active)
{return g_region_dispatch_area==3U?Stage3_SharedByte(b,now,binary_active):SourceCycle_SharedByte(b,now,binary_active);}
static void RegionDispatch_Telemetry(uint32_t now)
{if(g_region_dispatch_area==3U)Stage3_Telemetry(now);else SourceCycle_Telemetry(now);}
