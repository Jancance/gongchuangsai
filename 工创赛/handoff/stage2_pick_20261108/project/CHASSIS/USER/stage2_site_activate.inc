#include "stage2_site_profile.h"
/* Stage2 site activation glue (offline scaffold).
 * Gives Stage2Body_Configure a real caller so the stage2 body port is not
 * permanently unconfigured. While the site table is unverified the call is
 * rejected and nothing changes: no fake profile, no fake success.
 * state:  0 = not attempted, 1 = table rejected, 2 = configured
 */
volatile uint32_t g_stage2_site_state = 0U;
volatile uint32_t g_stage2_site_reason = 0U;

uint32_t Stage2Site_ActivateOnce(void)
{
    static uint32_t done = 0U;
    if (done)
    {
        g_stage2_site_state = 2U;
        return 1U;
    }
    if (!Stage2Body_Configure(&g_stage2_site_profile, &g_stage2_site_limits))
    {
        g_stage2_site_reason = 1U;
        g_stage2_site_state = 1U;
        return 0U;
    }
    g_stage2_site_reason = 0U;
    g_stage2_site_state = 2U;
    done = 1U;
    return 1U;
}
