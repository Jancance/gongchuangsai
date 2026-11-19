#ifndef PICKUP_TRIAL_PROFILE_H
#define PICKUP_TRIAL_PROFILE_H
#include <stdint.h>
/* 0=pzq baseline,1..3=the user's paired trials. Selected only while idle and
 * latched for the entire three-piece run. No arbitrary speed input. */
extern volatile uint32_t g_pick_trial_profile,g_pick_trial_requested,g_pick_trial_scurve;
static float PickupTrial_LiftRpm(void)
{return g_pick_trial_profile==0U?1846.4f:(g_pick_trial_profile==1U?2100.0f:(g_pick_trial_profile==2U?2500.0f:(g_pick_trial_profile==3U?2900.0f:0.0f)));}
static float PickupTrial_ExtRpm(void)
{return g_pick_trial_profile==0U?106.0f:(g_pick_trial_profile==1U?200.0f:(g_pick_trial_profile==2U?300.0f:(g_pick_trial_profile==3U?400.0f:0.0f)));}
static uint32_t PickupTrial_ExtAccel(void)
{return g_pick_trial_profile<=1U?600U:(g_pick_trial_profile==2U?1200U:(g_pick_trial_profile==3U?2000U:0U));}
#endif
