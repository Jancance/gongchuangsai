from pathlib import Path
import json,hashlib
R=Path.cwd(); E=Path(__file__).parent
old=R/'工创赛/evidence/navigation-planning/2026-11-15/bounded-lateral-enhancement'
s=(old/'build_isolated.py').read_text('utf8').replace('2026-11-15/bounded-lateral-enhancement','2026-11-15/float-timing-20261115').replace('工创赛/tmp/bounded-lateral-build-20261115','工创赛/tmp/float-timing-build-20261115')
s=s.replace("'app_serial_runtime.c']","'app_serial_runtime.c','chassis_auto_test.h','hwt101.c','hwt101.h']")
(E/'build_isolated.py').write_text(s,'utf8')
s=(old/'check_actual_entry.py').read_text('utf8').replace('2026-11-15/bounded-lateral-enhancement','2026-11-15/float-timing-20261115').replace('工创赛/tmp/bounded-lateral-c-check-20261115','工创赛/tmp/float-timing-c-check-20261115')
s=s.replace("auto.index('volatile uint32_t g_chassis_auto_request')","auto.index('volatile ChassisAutoLateralTiming')")
s=s.replace("globals_=auto[","globals_='#if APP_BASELINE40_ACTIVE\\n'+auto[")
s=s.replace('#include "app_entry.h"','#include "hwt101.h"\nuint32_t SystemCoreClock=84000000U;\nstatic uint32_t test_timing_enabled=1U,test_cycles;\n#include "app_entry.h"',1)
s=s.replace('uint32_t HWT101_TimingBegin(void){return 0U;}','uint32_t HWT101_TimingEnabled(void){return test_timing_enabled;}\nuint32_t HWT101_TimingBegin(void){test_cycles+=32U;return test_timing_enabled?test_cycles:0U;}')
s=s.replace('static const int32_t ex[4]={0,5,10,20};','static const int32_t ex[4]={0,5,11,20};')
timing_tests=r'''
 {
 ChassisAutoLateralTiming a,z; uint32_t previous;
 CHECK(sizeof(a)==104U && sizeof(a.scope[0])==48U);
 CHECK((uint32_t)((char*)&a.scope[0].sum_cycles-(char*)&a)==32U && (uint32_t)((char*)&a.scope[1]-(char*)&a)==56U);
 memset((void*)&g_chassis_auto_lateral_timing,0,sizeof(g_chassis_auto_lateral_timing));
 test_timing_enabled=0U; ChassisAuto_ReadLateralTiming(&z); CHECK(!z.enabled && !z.scope[0].valid && !z.scope[0].count);
 Auto_LateralTimingRecordLocked(0U,0U,1U,2U,100U);ChassisAuto_ReadLateralTiming(&a);CHECK(!a.scope[0].count);
 test_timing_enabled=1U;
 Auto_LateralTimingRecordLocked(0U,1U,0xFFFFFFF0U,16U,0xFFFFFFF0U);
 Auto_LateralTimingRecordLocked(0U,1U,0U,0U,5U);
 Auto_LateralTimingRecordLocked(0U,1U,0U,0xFFFFFFFFU,16U);
 ChassisAuto_ReadLateralTiming(&a);CHECK(a.enabled && a.core_clock_hz==84000000U);
 CHECK(a.scope[0].valid && a.scope[0].count==3U && a.scope[0].min_cycles==0U);
 CHECK(a.scope[0].last_cycles==0xFFFFFFFFU && a.scope[0].max_cycles==0xFFFFFFFFU);
 CHECK(a.scope[0].sum_cycles==((uint64_t)0xFFFFFFFFU+32U) && a.scope[0].window_ms==32U);
 CHECK(!a.scope[1].valid && !a.scope[1].count);
 Auto_LateralTimingRecordLocked(0U,1U,1U,2U,17U);CHECK(a.scope[0].count==3U);
 g_chassis_auto_lateral_timing.scope[0].count=0xFFFFFFFFU;
 z=g_chassis_auto_lateral_timing;Auto_LateralTimingRecordLocked(0U,1U,0U,0xFFFFFFFFU,18U);
 CHECK(memcmp(&z,(const void*)&g_chassis_auto_lateral_timing,sizeof(z))==0);
 memset((void*)&g_chassis_auto_lateral_timing,0,sizeof(g_chassis_auto_lateral_timing));
 ff_setup(-1,4,0);memset((void*)&g_chassis_auto_lateral_timing,0,sizeof(g_chassis_auto_lateral_timing));ChassisAuto_CanStep();ChassisAuto_ReadLateralTiming(&a);
 CHECK(a.scope[0].count==1U);
 CHECK(a.scope[1].count==1U);
 CHECK(a.scope[0].last_cycles==32U);
 CHECK(a.scope[1].last_cycles==32U);
 test_timing_enabled=0U;ChassisAuto_CanStep();ChassisAuto_ReadLateralTiming(&z);
 CHECK(!z.enabled && z.scope[0].count==1U && z.scope[1].count==1U);
 test_timing_enabled=1U;g_chassis_auto_state=CHASSIS_AUTO_RUN;s_motor_index=s_monitor_between=0U;input_od.pose.valid=0U;ChassisAuto_CanStep();ChassisAuto_ReadLateralTiming(&z);
 CHECK(z.scope[0].count==1U && z.scope[1].count==2U);
 test_timing_enabled=1U;s_stop_due=1U;previous=z.scope[1].count;ChassisAuto_CanStep();ChassisAuto_ReadLateralTiming(&z);
 CHECK(z.scope[0].count==1U && z.scope[1].count==previous);
 g_chassis_auto_active=0U;ChassisAuto_CanStep();ChassisAuto_ReadLateralTiming(&z);
 CHECK(z.scope[0].count==1U && z.scope[1].count==previous);
 ChassisAuto_ReadLateralTiming(0);CHECK(lock_depth==0U);cases+=12U;
 }
'''
# Inject after all existing runner transformations, immediately before unit assembly.
s=s.replace("unit=prefix+","tests=tests.replace(' uint32_t saved_tick,saved_sends,k,j;AppOdometryBaseline40 od;', ' uint32_t saved_tick,saved_sends,k,j;AppOdometryBaseline40 od;'+"+repr(timing_tests)+")\nunit=prefix+",1)
(E/'check_actual_entry.py').write_text(s,'utf8')
print('prepared isolated build and actual-C runners')
