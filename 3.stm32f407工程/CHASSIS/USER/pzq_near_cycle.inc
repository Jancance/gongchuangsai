/* Single fixed-near pzq trigger. No detector, body or fake physical ledger. */
volatile uint32_t g_pzq_near_request,g_pzq_near_slot=1U,g_pzq_near_command_seq;
volatile uint32_t g_pzq_near_accepted_seq,g_pzq_near_state,g_pzq_near_rejected;
uint32_t PZQNear_TryStart(uint32_t slot,uint32_t sequence,uint32_t now)
{
 if(Low_Busy() || Region_Busy() || !sequence || sequence==g_pzq_near_accepted_seq || slot<1U || slot>3U ||
    g_pzq_near_state==1U || g_pzq_near_state==9U ||
    SourceBodyStep_Busy() || SourceBodyStep_WheelWindow() || !it_dm_fresh() ||
    fabsf(g_dm_can2_logical_position_deg-180.0f)>1.0f)return 0U;
 if(!it_auto_start_mode(now,1U,slot-1U))return 0U;
 g_pzq_near_accepted_seq=sequence;g_pzq_near_slot=slot;
 g_pzq_near_state=1U;g_pzq_near_rejected=0U;return 1U;
}
static void PZQNear_Service(uint32_t now)
{
 uint32_t request=g_pzq_near_request;g_pzq_near_request=0U;
 if(g_pzq_near_state==1U){
  if(g_it_auto_state==9U || g_it_state==9U)g_pzq_near_state=9U;
  else if(g_it_auto_state==2U && g_it_state==2U && !g_it_request &&
          g_it_auto_release_ms[g_pzq_near_slot-1U])g_pzq_near_state=2U;
 }
 if(request==9U){if(g_pzq_near_state==1U){g_it_request=9U;g_pzq_near_state=9U;}return;}
 if(request==1U && !PZQNear_TryStart(g_pzq_near_slot,g_pzq_near_command_seq,now))
  g_pzq_near_rejected++;
}
