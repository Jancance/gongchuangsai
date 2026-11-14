from pathlib import Path
import hashlib, json
R=Path.cwd(); U=R/'3.stm32f407工程/CHASSIS/USER'; E=Path(__file__).parent
assert json.loads((E/'matlab_summary.json').read_text())['all_passed']
names=['chassis_auto_test.c','chassis_auto_test.h','hwt101.c','hwt101.h']
before={n:((E/(n+'.before')).read_bytes() if (E/(n+'.before')).exists() else (U/n).read_bytes()) for n in names}
assert hashlib.sha256(before[names[0]]).hexdigest()=='5742f22ba37b9fc211c5129d6a8dab607fdf8f376e5bc6fcc31070ec18208c1b'
def replace(n,a,b):
 s=current[n]; assert s.count(a)==1,(n,a); current[n]=s.replace(a,b)
current={n:b.decode('gbk') for n,b in before.items()}
replace('hwt101.h','uint32_t HWT101_TimingBegin(void);','uint32_t HWT101_TimingEnabled(void);\r\nuint32_t HWT101_TimingBegin(void);')
replace('hwt101.c','uint32_t HWT101_TimingBegin(void)','uint32_t HWT101_TimingEnabled(void)\r\n{ return s_timing_enabled; }\r\nuint32_t HWT101_TimingBegin(void)')
header='''#if APP_BASELINE40_ACTIVE
/* Boot-lifetime debug RAM only. scope0: target+FF; scope1: Step/budget/quantize.
 * Elapsed cycles include IRQ preemption; window_ms is last_ms-first_ms.
 * valid=0/count=0 means no measurement, not zero cost. No UART wire changes. */
typedef struct
{
    uint32_t valid, count, min_cycles, last_cycles, max_cycles;
    uint64_t sum_cycles;
    uint32_t first_ms, last_ms, window_ms;
} ChassisAutoLateralScopeTiming;
typedef struct
{
    uint32_t enabled, core_clock_hz;
    ChassisAutoLateralScopeTiming scope[2];
} ChassisAutoLateralTiming;
extern volatile ChassisAutoLateralTiming g_chassis_auto_lateral_timing;
void ChassisAuto_ReadLateralTiming(ChassisAutoLateralTiming *snapshot);
#endif

'''.replace('\n','\r\n')
replace('chassis_auto_test.h','extern volatile uint32_t g_chassis_auto_request;',header.replace('\r\n','\n')+'extern volatile uint32_t g_chassis_auto_request;')
replace('chassis_auto_test.c','#include "heading_source.h"','#include "heading_source.h"\r\n#include "hwt101.h"')
code='''#if APP_BASELINE40_ACTIVE
volatile ChassisAutoLateralTiming g_chassis_auto_lateral_timing;
/* Called only under the existing CanStep lock; statistics are outside scopes. */
static void Auto_LateralTimingRecordLocked(uint32_t scope, uint32_t enabled,
    uint32_t start, uint32_t end, uint32_t ms)
{
    volatile ChassisAutoLateralScopeTiming *s=&g_chassis_auto_lateral_timing.scope[scope];
    uint32_t cycles=end-start;
    g_chassis_auto_lateral_timing.enabled=enabled;
    g_chassis_auto_lateral_timing.core_clock_hz=SystemCoreClock;
    /* Freeze at UINT32_MAX: count and sum remain mutually consistent. */
    if(!enabled || s->count==0xFFFFFFFFU) return;
    if(!s->count) { s->first_ms=ms; s->min_cycles=cycles; }
    s->count++; s->last_cycles=cycles;
    if(cycles<s->min_cycles) s->min_cycles=cycles;
    if(cycles>s->max_cycles) s->max_cycles=cycles;
    s->sum_cycles+=(uint64_t)cycles; s->last_ms=ms;
    s->window_ms=ms-s->first_ms; s->valid=1U;
}
void ChassisAuto_ReadLateralTiming(ChassisAutoLateralTiming *snapshot)
{
    if(!snapshot) return;
    taskENTER_CRITICAL();
    *snapshot=g_chassis_auto_lateral_timing;
    snapshot->enabled=HWT101_TimingEnabled(); snapshot->core_clock_hz=SystemCoreClock;
    taskEXIT_CRITICAL();
}
#endif

'''.replace('\n','\r\n')
replace('chassis_auto_test.c','volatile uint32_t g_chassis_auto_request = 0U;',code+'volatile uint32_t g_chassis_auto_request = 0U;')
replace('chassis_auto_test.c','    float lateral_target,lateral_ff_per_rpm=0.0f;', '    float lateral_target,lateral_ff_per_rpm=0.0f;\r\n    uint32_t lateral_timing_enabled,lateral_timing_start,lateral_timing_end,lateral_step_ok;')
replace('chassis_auto_test.c','    lateral_target=Auto_LateralTarget(&lateral_od,lateral_begin,lateral_now_ms,&lateral_valid);','    lateral_timing_enabled=HWT101_TimingEnabled();\r\n    lateral_timing_start=HWT101_TimingBegin();\r\n    lateral_target=Auto_LateralTarget(&lateral_od,lateral_begin,lateral_now_ms,&lateral_valid);')
replace('chassis_auto_test.c','        &lateral_heading,lateral_heading_ready,lateral_now_ms);\r\n#endif\r\n    taskENTER_CRITICAL();\r\n    now = xTaskGetTickCount(); Auto_CheckLocked(now);','        &lateral_heading,lateral_heading_ready,lateral_now_ms);\r\n    lateral_timing_end=HWT101_TimingBegin();\r\n#endif\r\n    taskENTER_CRITICAL();\r\n#if APP_BASELINE40_ACTIVE\r\n    Auto_LateralTimingRecordLocked(0U,lateral_timing_enabled,lateral_timing_start,\r\n        lateral_timing_end,lateral_now_ms);\r\n#endif\r\n    now = xTaskGetTickCount(); Auto_CheckLocked(now);')
replace('chassis_auto_test.c','        if(!Auto_LateralStepLocked((uint32_t)now*portTICK_PERIOD_MS,lateral_target,\r\n                s_heading_pending_rpm,&heading_deci,&lateral_deci))','        lateral_timing_start=HWT101_TimingBegin();\r\n        lateral_step_ok=Auto_LateralStepLocked((uint32_t)now*portTICK_PERIOD_MS,lateral_target,\r\n                s_heading_pending_rpm,&heading_deci,&lateral_deci);\r\n        lateral_timing_end=HWT101_TimingBegin();\r\n        Auto_LateralTimingRecordLocked(1U,lateral_timing_enabled,lateral_timing_start,\r\n            lateral_timing_end,(uint32_t)now*portTICK_PERIOD_MS);\r\n        if(!lateral_step_ok)')
# Record scope0 only after existing active/session/fresh-OD admission.
replace('chassis_auto_test.c','#if APP_BASELINE40_ACTIVE\r\n    Auto_LateralTimingRecordLocked(0U,lateral_timing_enabled,lateral_timing_start,\r\n        lateral_timing_end,lateral_now_ms);\r\n#endif\r\n','')
replace('chassis_auto_test.c','        if(lateral_valid) {\r\n            float half=', '        if(lateral_valid) Auto_LateralTimingRecordLocked(0U,lateral_timing_enabled,\r\n            lateral_timing_start,lateral_timing_end,lateral_now_ms);\r\n        if(lateral_valid) {\r\n            float half=')
for n in names:
 b=current[n].encode('gbk'); assert n=='chassis_auto_test.h' or b.count(b'\n')==b.count(b'\r\n')
 (E/(n+'.before')).write_bytes(before[n]); (U/n).write_bytes(b)
manifest={n:{'before':hashlib.sha256(before[n]).hexdigest(),'after':hashlib.sha256((U/n).read_bytes()).hexdigest()} for n in names}
(E/'source_manifest.json').write_text(json.dumps(manifest,indent=2),'utf8')
print(json.dumps(manifest,indent=2))
