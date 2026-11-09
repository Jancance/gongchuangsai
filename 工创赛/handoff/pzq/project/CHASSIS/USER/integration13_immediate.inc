/* Supervised fixed-pose THREE-item commissioning, sole CAN task only.
 * Command completion is not physical grasp/placement evidence. No ledger or
 * route3 is updated. The operator judges the whole trial after it stops. */
volatile uint32_t g_it_auto_state,g_it_auto_phase,g_it_auto_item,g_it_auto_fault;
volatile uint32_t g_it_auto_elapsed_ms,g_it_auto_release_ms[3],g_it_auto_phase_ms;
static uint32_t it_auto_begin,it_auto_phase_begin,it_auto_sequence;

static void it_auto_aim(uint32_t now)
{
 static const float yaw[3]={180.0f,195.0f,165.0f};
 g_it_axes=SPF_DM;g_it_dm_deg=yaw[g_it_auto_item];g_it_extension_mm=0.0f;
 g_it_carrier_auto_deg=(float)g_it_auto_item*120.0f;
 it_auto_sequence=g_it_sequence;g_it_request=3U;
 g_it_auto_phase=1U;it_auto_phase_begin=now;
}
static uint32_t it_auto_start(uint32_t now)
{
 if(g_it_auto_state==1U || g_it_auto_state==9U || g_it_request ||
  g_it_state!=2U || !it_platform() || !it_at_top() || !SingleTeach_ReferencesReady() ||
  VehicleArm_Busy() || SourceStage2_IsActive() || Stage2Fast_IsActive() ||
  g_it_cause || g_lift_ruler_request || g_lift_ruler_stop_pending ||
  fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)>2.0f)return 0U;
 g_it_auto_state=1U;g_it_auto_item=g_it_auto_fault=g_it_auto_elapsed_ms=0U;
 g_it_auto_release_ms[0]=g_it_auto_release_ms[1]=g_it_auto_release_ms[2]=0U;
 it_auto_begin=now;it_auto_aim(now);return 1U;
}
static void it_auto_service(uint32_t now)
{
 uint32_t expected;
 if(g_it_auto_state!=1U)return;
 g_it_auto_elapsed_ms=now-it_auto_begin;g_it_auto_phase_ms=now-it_auto_phase_begin;
 if(!it_platform() || g_it_rejected || g_it_cause || g_it_state==9U ||
  g_it_auto_phase_ms>=16000U){
  g_it_auto_fault=g_it_cause?g_it_cause:12U;g_it_auto_state=9U;
  g_it_request=0U;it_fail(g_it_auto_fault);return;
 }
 if(g_it_auto_phase==6U){
  /* User-specified release interval starts at the actual PWM write, not at
   * host observation. No yaw/next-item request is published before200ms. */
  if(g_it_auto_phase_ms<200U)return;
  if(g_it_auto_item<2U){g_it_auto_item++;it_auto_aim(now);return;}
  g_it_axes=SPF_DM;g_it_dm_deg=0.0f;g_it_extension_mm=0.0f;
  it_auto_sequence=g_it_sequence;g_it_request=3U;g_it_auto_phase=4U;
  it_auto_phase_begin=now;return;
 }
 if(g_it_state!=2U || g_it_request || g_it_sequence==it_auto_sequence)return;
 expected=g_it_auto_phase==2U?14U:(g_it_auto_phase==3U?15U:3U);
 if(g_it_action!=expected)return;
 switch(g_it_auto_phase){
 case 1U:
  g_it_extension_mm=g_it_auto_item==2U?160.0f:(g_it_auto_item==1U?156.0f:0.0f);g_it_lift_pulses=118800.0f;
  it_auto_sequence=g_it_sequence;g_it_request=14U;g_it_auto_phase=2U;break;
 case 2U:
  /* The approach already preclosed to160. Publish140 and return preparation
   * now; no fixed150/600ms pause. Actual servo/motor motion is field-tested. */
  if(!g_it_lead5_grip_done && jiazhua(140.0f)!=1U){g_it_auto_fault=7U;g_it_auto_state=9U;it_fail(7U);return;}
  g_it_lift_pulses=7200.0f;it_auto_sequence=g_it_sequence;g_it_request=15U;
  g_it_auto_phase=3U;break;
 case 3U:
  if(jiazhua(210.0f)!=1U){g_it_auto_fault=7U;g_it_auto_state=9U;it_fail(7U);return;}
  g_it_auto_release_ms[g_it_auto_item]=now-it_auto_begin;
  g_it_auto_phase=6U;break;
 case 4U:g_it_auto_state=2U;g_it_auto_phase=5U;break;
 default:g_it_auto_state=9U;g_it_auto_fault=13U;it_fail(13U);return;
 }
 it_auto_phase_begin=now;
}
