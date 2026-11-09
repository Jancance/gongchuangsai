"""Bounded handoff admission without relaxing source/ownership/stop checks."""
from pathlib import Path
import shutil,json,hashlib
R=Path(__file__).resolve().parents[1];A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/stage12_admission_20261104';E=R/'evidence/integration/2026-11-04/stage12-admission'
E.mkdir(parents=True,exist_ok=True);assert not S.exists()
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','.git'));original={}
def edit(rel,fn):
 p=S/rel;b=p.read_bytes();original[rel]=hashlib.sha256(b).hexdigest()
 try:u=b.decode('utf-8');enc='utf-8'
 except UnicodeDecodeError:u=b.decode('gbk');enc='gbk'
 q=E/'before'/rel;q.parent.mkdir(parents=True,exist_ok=True);q.write_bytes(b)
 p.write_bytes(fn(u.replace('\r\n','\n')).replace('\n','\r\n').encode(enc))
def params(u):
 marker='#define VEHICLE_HANDOFF_ENDPOINT_MAX_AGE_MS 1000U';assert u.count(marker)==1
 return u.replace(marker,marker+'''\n#define VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS 1000U /* 已停车后等待原准入条件，禁止无限等待。 */''')
edit('USER/vehicle_parameters.h',params)
def main(u):
 lo=u.index('static uint32_t App_SerialIdleReady(uint32_t now)\n{');hi=u.index('/*',u.index('\n}',lo)+2)
 old=u[lo:hi]
 assert 'App_HeadingRuntimeSource(now)' in old and 'static uint32_t' not in old[30:]
 new='''/* 位掩码只细分原有准入条件，不放宽任何门槛。 */
static uint32_t App_SerialIdleBlocked(uint32_t now)
{
#if APP_MOTOR_OFFLINE_ONLY
    (void)now;return 1U;
#else
    ChassisAutoStatus status;PowerCommandStatus power;uint32_t mask=0U;
    if(!CAN1_EmmSettingsConfirmed())mask|=1U;
    (void)ChassisAuto_ReadStatus(&status);Power_ReadCommandStatus(&power);
#if APP_TEST_MODE == 3U
    if(g_combined_test.state || g_combined_test.request || g_motion_plan_valid!=1U)mask|=2U;
#endif
    if(g_forward_hold.phase!=HR_IDLE || g_forward_hold.request || g_vm_test_request ||
       (power.active && !StartupAxes_Holding()))mask|=4U;
    if(status.active || status.result || status.cut_vm)mask|=8U;
    if(g_translation_hold_selftest!=1U || s_serial_proto_ok!=1U)mask|=16U;
    if(!App_HeadingRuntimeSource(now))mask|=32U;
    if(!App_HeadingRuntimeExclusive())mask|=64U;
    if(g_can_ctrl_fault_latched)mask|=128U;
    if(!(s_heading_source.rate_dps>=-3.0f && s_heading_source.rate_dps<=3.0f))mask|=256U;
    return mask;
#endif
}
static uint32_t App_SerialIdleReady(uint32_t now)
{return App_SerialIdleBlocked(now)==0U;}
'''
 return u[:lo]+new+u[hi:]
edit('CHASSIS/USER/main.c',main)
def mission(u):
 marker='static uint32_t s_vehicle_search_begin,s_vehicle_search_used;';assert u.count(marker)==1
 u=u.replace(marker,marker+'''
/* queue_state:0无等待、1待准入、2已发布、3截止/取消；计时只覆盖本次交接。 */
static uint32_t s_vehicle_queue_pending,s_vehicle_queue_begin;
volatile uint32_t g_vehicle_queue_state,g_vehicle_queue_first_mask,g_vehicle_queue_last_mask;
volatile uint32_t g_vehicle_queue_wait_ms,g_vehicle_queue_attempts,g_vehicle_queue_stage;
''')
 marker='    MotionStop();VehicleArm_Cancel();StartupAxes_Cancel();';assert u.count(marker)==1
 u=u.replace(marker,'''    s_vehicle_queue_pending=0U;if(g_vehicle_queue_state==1U)g_vehicle_queue_state=3U;
'''+marker)
 lo=u.index('    for(i=0;i<9;i++){s_perimeter_targets',u.index('static uint32_t VehicleMission_Queue'))
 hi=u.index('}static uint32_t VehicleMission_Load',lo)
 publish=u[lo:hi]
 publish=publish.replace('    return 1U;','    s_vehicle_queue_pending=0U;g_vehicle_queue_state=2U;return 1U;')
 fn='''static uint32_t VehicleMission_PublishQueued(uint32_t stage)
{
    uint32_t i;
'''+publish+''' }
'''
 # The gate is sampled once; successful publication always uses that same verdict.
 u=u[:lo]+'''    {uint32_t mask=App_SerialIdleBlocked((uint32_t)xTaskGetTickCount());
     g_vehicle_queue_stage=stage;g_vehicle_queue_first_mask=g_vehicle_queue_last_mask=mask;
     g_vehicle_queue_wait_ms=0U;g_vehicle_queue_attempts=1U;
     if(mask){
        if(mask&~(32U|256U)){g_vehicle_queue_state=3U;return 0U;}
        if(VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS<100U || VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS>2000U)return 0U;
        s_vehicle_queue_pending=stage;s_vehicle_queue_begin=(uint32_t)xTaskGetTickCount();g_vehicle_queue_state=1U;return 1U;
     }}
    return VehicleMission_PublishQueued(stage);
'''+u[hi:]
 # Remove the original fail-fast gate and move publication ahead of Queue.
 u=u.replace('    if(!App_SerialIdleReady((uint32_t)xTaskGetTickCount()))return 0U;\n','')
 marker='static uint32_t VehicleMission_Queue(uint32_t stage,const RouteRow *rows,uint32_t n)'
 assert u.count(marker)==1;u=u.replace(marker,fn+marker)
 u=u.replace('ForwardHoldDebug forward={0};CombinedTestDebug combined={0};uint32_t i;',
             'ForwardHoldDebug forward={0};CombinedTestDebug combined={0};')
 marker='        s_vehicle_start=s_vehicle_stage_start=now;g_vehicle_stage_mask=0U;'
 assert u.count(marker)==1
 u=u.replace(marker,marker+'''
        s_vehicle_queue_pending=s_vehicle_queue_begin=0U;
        g_vehicle_queue_state=g_vehicle_queue_first_mask=g_vehicle_queue_last_mask=0U;
        g_vehicle_queue_wait_ms=g_vehicle_queue_attempts=g_vehicle_queue_stage=0U;''')
 marker='        if((g_vehicle_state==1U || g_vehicle_state==2U) && g_vehicle_stage==1U && !g_vehicle_scan_bypassed'
 assert u.count(marker)==1
 u=u.replace(marker,'''    if(s_vehicle_queue_pending){
        g_vehicle_queue_wait_ms=now-s_vehicle_queue_begin;
        if(g_vehicle_queue_wait_ms>=VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS){g_vehicle_queue_state=3U;VehicleMission_Fail(7U);return;}
        g_vehicle_queue_attempts++;g_vehicle_queue_last_mask=App_SerialIdleBlocked(now);
        if(g_vehicle_queue_last_mask&~(32U|256U)){g_vehicle_queue_state=3U;VehicleMission_Fail(7U);return;}
        if(!g_vehicle_queue_last_mask)(void)VehicleMission_PublishQueued(s_vehicle_queue_pending);
        return;
    }
'''+marker)
 # Retain queue verdicts after publication/failure in VM2, compatible parser extension.
 old='"VM1,'+','.join(['%lu']*13+['%ld']*2)+',"';assert u.count(old)==1
 u=u.replace(old,'"VM2,'+','.join(['%lu']*13+['%ld']*2+['%lu']*6)+',"')
 old='(long)VehicleMission_MilliDeg(g_startup_extension_current_deg),(long)VehicleMission_MilliDeg(g_vehicle_lift_current_deg));'
 assert u.count(old)==1
 return u.replace(old,old[:-2]+''',
        (unsigned long)g_vehicle_queue_state,(unsigned long)g_vehicle_queue_stage,
        (unsigned long)g_vehicle_queue_first_mask,(unsigned long)g_vehicle_queue_last_mask,
        (unsigned long)g_vehicle_queue_wait_ms,(unsigned long)g_vehicle_queue_attempts);''')
edit('CHASSIS/USER/vehicle_mission.inc',mission)
(E/'manifest.json').write_text(json.dumps(dict(actual=str(A),stage=str(S),original=original,changed={n:hashlib.sha256((S/n).read_bytes()).hexdigest() for n in original},installed=False,flashed=False),ensure_ascii=False,indent=2),encoding='utf-8');print(S)
