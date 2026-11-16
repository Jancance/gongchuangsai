
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "app_config.h"
#include "app_entry.h"
#include "motion_control.h"
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define RQ_LINE_MAX 240U
#define HR_WAIT 1U
#define HR_RUN 2U
#define HR_IDLE 0U
#define HR_DONE 3U
static uint32_t clock_ms,permit,abort_count,power_disable_count,heading_trip_count,forward_trip_count;
static uint32_t xTaskGetTickCount(void){return clock_ms;}
static struct {uint32_t state,request,index,cause,completed;} g_combined_test;
static struct {uint32_t phase,request,trip;} g_forward_hold;
static uint32_t s_motion_session_open,s_combined_keep_vm,s_combined_end_ms,s_combined_turn;
static uint32_t g_vm_test_request,g_heading_exec_request;
static uint32_t s_serial_token_seq,s_serial_token_tick,s_serial_arm_ms,s_serial_keepalive_ms,s_serial_tx_seq;
static uint32_t g_serial_motion_state,g_serial_motion_result,g_serial_motion_rejected,g_serial_motion_accepted,g_serial_motion_link_lost;
volatile uint32_t g_debug_plan_id;
static uint32_t action_count=1;
static struct {const uint32_t *action_count;} live={&action_count};
static struct {uint32_t staging,ready,consumed;struct {uint32_t id,count;struct {uint32_t timeout_ms;} rows[8];} committed;} route;
#define AppMotion_Live() (&live)
#define AppRuntimeRoute_Live() (&route)
static uint32_t s_combined_begin_ms,s_combined_stage_ms;
static uint32_t g_can_ctrl_fault_latched,g_safety_ready,g_can_guard_ready,source_healthy=1;
static const uint32_t s_heading_runtime_inputs=0,s_translation_runtime_inputs=0;
static uint32_t App_HeadingRuntimeSource(uint32_t now){return source_healthy;}
static uint32_t App_HeadingRuntimeExclusive(void){return 1;}
static void App_HeadingRuntimeSafety(const void *p){}
static void App_ForwardHoldSafety(const void *p){}
uint32_t MotionGetState(void){return MOTION_BUSY;}
static uint32_t FieldMap_LocalMode(void){return 0;}
static void App_RuntimeRouteCommand(char *p){(void)p;}
static uint32_t App_SerialIdleReady(uint32_t now){(void)now;return 1;}
static uint32_t App_SerialCompletedStopped(void){return 0;}
static uint32_t AppMotion_SetStartPermit(uint32_t p){permit=p;return 1;}
static uint32_t App_DebugSelectPlan(uint32_t id){MotionAction a[8];uint32_t n;if(!AppDebugPlan_Load(id,a,&n))return 0;g_debug_plan_id=id;return 1;}
static void App_HeadingRuntimeTrip(uint32_t reason){heading_trip_count++;}
static void App_ForwardHoldTrip(uint32_t reason){forward_trip_count++;}
static void ChassisAuto_Abort(void){abort_count++;}
static void Power_Disable(void){g_vm_test_request=0;power_disable_count++;}
uint16_t App_SerialCrc(const char *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i, bit;
    for (i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)((uint16_t)(uint8_t)data[i] << 8);
        for (bit = 0U; bit < 8U; bit++)
            crc = (uint16_t)((crc & 0x8000U) ? ((crc << 1) ^ 0x1021U) : (crc << 1));
    }
    return crc;
}
static uint32_t App_SerialUInt(const char **text, uint32_t *out)
{
    uint32_t n = 0U, digit;
    const char *p = *text;
    if (*p < '0' || *p > '9') return 0U;
    while (*p >= '0' && *p <= '9')
    {
        digit = (uint32_t)(*p - '0');
        if (n > (0xFFFFFFFFU - digit) / 10U) return 0U;
        n = n * 10U + digit; p++;
    }
    if (*p != ',') return 0U;
    *text = p + 1; *out = n; return 1U;
}
static void App_SerialReject(uint32_t reason)
{ g_serial_motion_rejected++; g_serial_motion_result = reason; }
void App_CombinedTestStop(uint32_t cause)
{
    /* STOP是幂等的；已完成/失败不覆盖首因，不清原单段trip。 */
    if (g_combined_test.state >= 3U)
    { g_vm_test_request = 0U; Power_Disable(); return; }
    s_motion_session_open = s_combined_keep_vm = 0U;
    s_combined_end_ms = (uint32_t)xTaskGetTickCount();
    g_combined_test.cause = cause;
    g_combined_test.state = (cause == 1U) ? 5U : 4U;
    g_forward_hold.request = 0U;
    g_heading_exec_request = 0U;
    if (s_combined_turn) App_HeadingRuntimeTrip(1U);
    else App_ForwardHoldTrip(1U);
    ChassisAuto_Abort();
    g_vm_test_request = 0U;
    Power_Disable();
}
void App_CombinedTestSafety(void)
{
    uint32_t now = (uint32_t)xTaskGetTickCount();
    taskENTER_CRITICAL();
    if ((g_combined_test.state == 1U) || (g_combined_test.state == 2U))
    {
        if (g_combined_test.request == 2U) App_CombinedTestStop(1U);
        else if ((uint32_t)(now - s_combined_begin_ms) >= APP_PERIMETER_GROUP_MS)
            App_CombinedTestStop(2U);
        else if (AppRuntimeRoute_Live()->committed.id && AppRuntimeRoute_Live()->consumed &&
                 g_combined_test.state==1U && g_combined_test.index<AppRuntimeRoute_Live()->committed.count &&
                 (uint32_t)(now-s_combined_stage_ms)>=AppRuntimeRoute_Live()->committed.rows[g_combined_test.index].timeout_ms)
            App_CombinedTestStop(2U);
        else if (!App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
                 g_can_ctrl_fault_latched) App_CombinedTestStop(3U);
        else if ((g_combined_test.state == 2U) &&
                 ((g_vm_test_request != 1U) || (g_safety_ready != 1U) ||
                  (g_can_guard_ready != 1U))) App_CombinedTestStop(3U);
        else if (g_combined_test.state == 1U)
        {
            if (s_combined_turn) App_HeadingRuntimeSafety(&s_heading_runtime_inputs);
            else App_ForwardHoldSafety(&s_translation_runtime_inputs);
        }
    }
    else if (s_motion_session_open)
    {
        if (!App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
            g_can_ctrl_fault_latched) App_CombinedTestStop(3U);
        else if (MotionGetState() == MOTION_BUSY)
        {
            if (s_combined_turn) App_HeadingRuntimeSafety(&s_heading_runtime_inputs);
            else App_ForwardHoldSafety(&s_translation_runtime_inputs);
        }
        else if (g_vm_test_request != 1U || g_can_guard_ready != 1U)
            App_CombinedTestStop(3U);
    }
    else { g_vm_test_request = 0U; }
    taskEXIT_CRITICAL();
}
static void App_SerialMotionCommand(char *line)
{
#if APP_VM_ONLY_ENABLE
    App_SerialReject(6U);
#else
    static const volatile uint32_t source_parameter_entry=VEHICLE_INTEGRATED_ENABLE;
    if(source_parameter_entry)return;

    if(FieldMap_LocalMode()) { App_SerialReject(9U);return; }
    if(strncmp(line,"RQ1,",4U)==0) {
#if APP_BASELINE40_ACTIVE
        App_SerialReject(6U);return;
#else
        App_RuntimeRouteCommand(line);return;
#endif
    }
    char *last = strrchr(line, ',');
    const char *p;
    uint32_t seq, tick, crc = 0U, i, digit, now;
    if ((strncmp(line, "RC1,", 4U) != 0) || (last == NULL) || strlen(last + 1) != 4U)
    { App_SerialReject(4U); return; }
    for (i = 1U; i <= 4U; i++)
    {
        char c = last[i];
        if (c >= '0' && c <= '9') digit = (uint32_t)(c - '0');
        else if (c >= 'A' && c <= 'F') digit = (uint32_t)(c - 'A') + 10U;
        else { App_SerialReject(4U); return; }
        crc = (crc << 4) | digit;
    }
    if ((uint32_t)App_SerialCrc(line, (uint32_t)(last - line + 1)) != crc)
    { App_SerialReject(4U); return; }
    *last = '\0'; p = line + 4;
    if (!App_SerialUInt(&p, &seq) || !App_SerialUInt(&p, &tick))
    { App_SerialReject(4U); return; }
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    if (strcmp(p, "STOP") == 0)
    {
        AppMotion_SetStartPermit(0U);
#if APP_TEST_MODE == 3U
        App_CombinedTestStop(1U);
#else
        if ((g_forward_hold.phase == HR_WAIT) || (g_forward_hold.phase == HR_RUN) ||
            (g_forward_hold.request == 1U)) g_forward_hold.request = 2U;
#endif
        g_serial_motion_state = 3U; g_serial_motion_result = 3U;
        g_serial_motion_accepted++;
    }
#if APP_TEST_MODE == 3U
    else if ((strncmp(p,"PLAN",4U)==0 && p[4]>='0' && p[4]<='9' && p[5]=='\0') ||
             strcmp(p,"PLAN10")==0 || strcmp(p,"PLAN11")==0 || strcmp(p,"PLAN12")==0 || strcmp(p,"PLAN13")==0 ||
             strcmp(p,"PLAN14")==0 || strcmp(p,"PLAN15")==0 || strcmp(p,"PLAN16")==0 || strcmp(p,"PLAN17")==0 || strcmp(p,"PLAN18")==0 || strcmp(p,"PLAN19")==0 ||
              strcmp(p,"PLAN20")==0 || strcmp(p,"PLAN21")==0)
    {
        if (!s_serial_tx_seq || (uint32_t)(s_serial_tx_seq-1U-seq)>5U ||
            (uint32_t)(now-tick)>=500U) App_SerialReject(5U);
        else if (g_serial_motion_state || AppRuntimeRoute_Live()->staging || AppRuntimeRoute_Live()->committed.id || !App_SerialIdleReady(now) ||
                 !App_DebugSelectPlan(p[5] ? ((uint32_t)(p[4]-'0')*10U+(uint32_t)(p[5]-'0')) : (uint32_t)(p[4]-'0'))) App_SerialReject(6U);
        else g_serial_motion_accepted++;
    }
#endif
    else if (strcmp(p, "ARM") == 0)
    {
        if ((s_serial_tx_seq == 0U) || ((uint32_t)(s_serial_tx_seq - 1U - seq) > 5U) ||
            ((uint32_t)(now - tick) >= 500U)) App_SerialReject(5U);
        else if ((g_serial_motion_state != 0U) || !App_SerialIdleReady(now) ||
                 (!APP_BASELINE40_ACTIVE && (!AppRuntimeRoute_Live()->ready || AppRuntimeRoute_Live()->staging || AppRuntimeRoute_Live()->consumed)) ||
                 (APP_BASELINE40_ACTIVE && ((g_debug_plan_id!=DEBUG_PLAN_FORWARD210_40 && g_debug_plan_id!=DEBUG_PLAN_LEFT210_40 &&
                  g_debug_plan_id!=DEBUG_PLAN_RIGHT210_40 && g_debug_plan_id!=DEBUG_PLAN_BACKWARD210_40 && g_debug_plan_id!=DEBUG_PLAN_RIGHT150_40 && g_debug_plan_id!=DEBUG_PLAN_RIGHT30_40 &&
                   g_debug_plan_id!=DEBUG_PLAN_FORWARD150_40 && g_debug_plan_id!=DEBUG_PLAN_FORWARD30_40 && g_debug_plan_id!=DEBUG_PLAN_BACKWARD150_40 && g_debug_plan_id!=DEBUG_PLAN_BACKWARD30_40 && g_debug_plan_id!=DEBUG_PLAN_LEFT150_40 && g_debug_plan_id!=DEBUG_PLAN_LEFT30_40 && !AppDebugTurn90Plan(g_debug_plan_id)) ||
                  *AppMotion_Live()->action_count!=1U))) App_SerialReject(6U);
        else
        {
            s_serial_token_seq = seq; s_serial_token_tick = tick; s_serial_arm_ms = now;
            g_serial_motion_state = 1U; g_serial_motion_result = 1U;
            g_serial_motion_accepted++;
        }
    }
    else if ((seq != s_serial_token_seq) || (tick != s_serial_token_tick)) App_SerialReject(5U);
    else if (strcmp(p, "START") == 0)
    {
#if APP_TEST_MODE == 3U && !APP_BASELINE40_ACTIVE
        App_SerialReject(6U);
#else
        if (g_serial_motion_state == 2U) { /* 重复帧只确认，不重新启动或续期。 */ }
        else if ((g_serial_motion_state != 1U) || ((uint32_t)(now - s_serial_arm_ms) >= 2000U) ||
            !App_SerialIdleReady(now)) App_SerialReject(6U);
        else
        {
            AppMotion_SetStartPermit(1U); s_serial_keepalive_ms = now;
#if APP_TEST_MODE == 3U
            g_combined_test.request = 1U;
#else
            g_forward_hold.request = 1U;
#endif
            g_serial_motion_state = 2U; g_serial_motion_result = 2U;
            g_serial_motion_accepted++;
        }
#endif
    }
    else if (strcmp(p, "KEEP") == 0)
    {
        if (g_serial_motion_state == 2U)
        { s_serial_keepalive_ms = now; g_serial_motion_accepted++; }
        else if (App_SerialCompletedStopped())
        { g_serial_motion_accepted++; } /* 同令牌已在上方核对；终态只确认，不续期。 */
        else App_SerialReject(6U);
    }
    else App_SerialReject(4U);
    taskEXIT_CRITICAL();
#endif
}
void App_SerialMotionByte(uint8_t data)
{
    static char line[RQ_LINE_MAX];
    static uint32_t used, invalid, last_ms;
    uint32_t now = (uint32_t)xTaskGetTickCount();
    if ((uint32_t)(now - last_ms) >= 200U) { used = 0U; invalid = 0U; }
    last_ms = now;
    if (data == '\r') return;
    if (data == '\n')
    {
        if (!invalid && used) { line[used] = '\0'; App_SerialMotionCommand(line); }
        used = invalid = 0U; return;
    }
    if (invalid) return;
    if (data < 32U || data > 126U || used >= sizeof(line) - 1U)
    { invalid = 1U; App_SerialReject(4U); return; }
    line[used++] = (char)data;
}
void App_SerialMotionSafety(void)
{
    uint32_t now;
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    if ((g_serial_motion_state == 1U) && ((uint32_t)(now - s_serial_arm_ms) >= 2000U))
    { g_serial_motion_state = 0U; g_serial_motion_result = 7U; AppMotion_SetStartPermit(0U); }
    if (g_serial_motion_state == 2U)
    {
#if APP_TEST_MODE == 3U
        if (g_combined_test.state >= 3U) g_serial_motion_state = 3U;
        else if (!VEHICLE_STAGE1_ENABLE && (uint32_t)(now - s_serial_keepalive_ms) >=
            (AppDebugTurn90Plan(g_debug_plan_id) ? 3000U : 1000U))
        {
            AppMotion_SetStartPermit(0U); g_serial_motion_link_lost = 1U;
            g_serial_motion_result = 8U; g_serial_motion_state = 3U;
            App_CombinedTestStop(1U);
        }
#else
        if ((g_forward_hold.phase == HR_IDLE) && (g_forward_hold.request == 2U))
        {
            AppMotion_SetStartPermit(0U);
            g_serial_motion_state = 3U; g_serial_motion_result = 3U;
            App_ForwardHoldTrip(1U);
        }
        else if (g_forward_hold.phase >= HR_DONE) g_serial_motion_state = 3U;
        else if (!VEHICLE_STAGE1_ENABLE && (uint32_t)(now - s_serial_keepalive_ms) >= 1000U)
        {
            AppMotion_SetStartPermit(0U); g_serial_motion_link_lost = 1U;
            g_serial_motion_result = 8U; g_serial_motion_state = 3U;
            App_ForwardHoldTrip(1U);
        }
#endif
    }
    taskEXIT_CRITICAL();
}
volatile uint32_t review_assertions;
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
static void send_frame(const char *verb,uint32_t seq,uint32_t tick,uint32_t corrupt,uint32_t incomplete){
 char line[128];uint32_t n,i;uint16_t crc;
 n=(uint32_t)sprintf(line,"RC1,%lu,%lu,%s,",(unsigned long)seq,(unsigned long)tick,verb);crc=App_SerialCrc(line,n);
 n+=(uint32_t)sprintf(line+n,"%04X\n",corrupt?(crc^1U):crc);
 for(i=0;i<n-(incomplete?1U:0U);i++)App_SerialMotionByte((uint8_t)line[i]);
}
static void reset(uint32_t plan,uint32_t running){
 App_SerialMotionByte('\n');
 memset(&g_combined_test,0,sizeof(g_combined_test));memset(&g_forward_hold,0,sizeof(g_forward_hold));memset(&route,0,sizeof(route));
 clock_ms=10000;s_combined_begin_ms=s_combined_stage_ms=10000;source_healthy=1;g_can_ctrl_fault_latched=0;g_safety_ready=g_can_guard_ready=1;g_debug_plan_id=plan;g_serial_motion_state=running?2:0;g_serial_motion_result=running?2:0;
 s_serial_token_seq=7;s_serial_token_tick=9000;s_serial_arm_ms=s_serial_keepalive_ms=10000;s_serial_tx_seq=8;
 g_serial_motion_rejected=g_serial_motion_accepted=g_serial_motion_link_lost=0;
 permit=running;abort_count=power_disable_count=heading_trip_count=forward_trip_count=0;
 g_combined_test.state=running;g_vm_test_request=running;s_motion_session_open=s_combined_keep_vm=running;s_combined_turn=1;
}
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t id,boundary,origin,last,i;MotionAction a[8];uint32_t count;
 for(id=8;id<=21;id++){
  boundary=AppDebugTurn90Plan(id)?3000:1000;
  reset(id,1);clock_ms+=boundary-1;App_SerialMotionSafety();CK(g_serial_motion_state==2 && permit==1 && !abort_count);
  clock_ms++;App_SerialMotionSafety();CK(g_serial_motion_state==3 && !permit && g_serial_motion_result==8 && g_serial_motion_link_lost==1);
  CK(abort_count==1 && power_disable_count==1 && heading_trip_count==1 && !g_vm_test_request && g_combined_test.state==5);
  send_frame("KEEP",7,9000,0,0);send_frame("START",7,9000,0,0);CK(g_serial_motion_state==3 && !permit && abort_count==1 && !g_vm_test_request);
 }
 for(id=20;id<=21;id++){
  boundary=AppDebugTurn90Plan(id)?3000:1000;
  reset(id,1);clock_ms+=boundary-1;send_frame("KEEP",7,9000,0,0);CK(s_serial_keepalive_ms==clock_ms && g_serial_motion_state==2);
  clock_ms+=boundary-1;App_SerialMotionSafety();CK(g_serial_motion_state==2);clock_ms++;App_SerialMotionSafety();CK(g_serial_motion_result==8 && !permit);
  for(i=0;i<4;i++){
   reset(id,1);last=s_serial_keepalive_ms;clock_ms+=boundary-1;
   if(i==0)send_frame("KEEP",6,9000,0,0);
   if(i==1)send_frame("KEEP",7,9000,1,0);
   if(i==2)send_frame("KEEP",7,9000,0,1);
   if(i==3)send_frame("START",7,9000,0,0);
   CK(s_serial_keepalive_ms==last);clock_ms++;App_SerialMotionSafety();CK(g_serial_motion_result==8 && !permit && abort_count==1);
  }
  reset(id,1);clock_ms+=1;send_frame("STOP",123,0,0,0);CK(g_serial_motion_state==3 && g_serial_motion_result==3 && !permit && abort_count==1 && !g_vm_test_request);
  reset(id,0);clock_ms=10000;send_frame(id==20?"PLAN20":"PLAN21",7,9999,0,0);CK((g_serial_motion_accepted==1)==AppDebugTurn90Plan(id));
  reset(id,0);send_frame("ARM",7,9999,0,0);CK((g_serial_motion_state==1)==AppDebugTurn90Plan(id));
  reset(id,1);clock_ms++;source_healthy=0;App_CombinedTestSafety();CK(abort_count==1 && !g_vm_test_request && g_combined_test.state==4);
  reset(id,1);clock_ms++;g_can_ctrl_fault_latched=1;App_CombinedTestSafety();CK(abort_count==1 && !g_vm_test_request && g_combined_test.state==4);
 }
 /* ARM lifetime stays exactly 2000 ms, independently of keepalive profile. */
 for(i=0;i<2;i++){
  reset(8,0);send_frame("ARM",7,9999,0,0);CK(g_serial_motion_state==1 && s_serial_token_seq==7 && s_serial_token_tick==9999);
  clock_ms+=i?2000:1999;send_frame("START",7,9999,0,0);
  CK(i?(g_serial_motion_state==1 && !permit && g_serial_motion_result==6):(g_serial_motion_state==2 && permit));
 }
 reset(20,0);g_serial_motion_state=1;permit=1;clock_ms+=1999;App_SerialMotionSafety();CK(g_serial_motion_state==1 && permit);
 clock_ms++;App_SerialMotionSafety();CK(g_serial_motion_state==0 && !permit && g_serial_motion_result==7);
 /* ARM token freshness is strict 500 ms. */
 reset(8,0);send_frame("ARM",7,9501,0,0);CK(g_serial_motion_state==1);
 reset(8,0);send_frame("ARM",7,9500,0,0);CK(g_serial_motion_state==0 && g_serial_motion_result==5);
 /* Unsigned keepalive age still handles wrap. */
 for(id=20;id<=21;id++){
  reset(id,1);boundary=AppDebugTurn90Plan(id)?3000:1000;s_serial_keepalive_ms=0xFFFFFF00U;clock_ms=s_serial_keepalive_ms+boundary-1;App_SerialMotionSafety();CK(g_serial_motion_state==2);
  clock_ms++;App_SerialMotionSafety();CK(g_serial_motion_state==3 && !permit);
 }
#if !APP_TURN90_90_ACTIVE
 CK(!AppDebugPlan_Load(20,a,&count) && !AppDebugPlan_Load(21,a,&count));
#endif
 return 0;
}
