"""Complete the requested initial XY path without opening other XY paths."""
import json
from prepare_three_frame_warm_20261118 import P,E,edit


def main():
    assert json.loads((E/'matlab-initial-gate.json').read_text())['passed']
    p=P/'CHASSIS/USER/source_mcu_cycle.inc'
    edit(p,'static uint32_t sc_beep_started,sc_last_tx;', '''static uint32_t SC_InitialObserveAllowed(void)
{
 return g_source_cycle_state==1U && g_source_cycle_prep==4U &&
  g_it_axes==(SPF_DM|SPF_EXT) && g_it_dm_deg==180.0f && g_it_extension_mm==65.0f &&
  it_at_top() && fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)<=2.0f;
}
static uint32_t sc_beep_started,sc_last_tx;'''.replace('\n','\r\n'))
    p=P/'CHASSIS/USER/integration10_complete.inc'
    edit(p,'((axes&1U) && !Low_XYAllowed() && !Region_ObserveYawAllowed() &&',
           '((axes&1U) && !Low_XYAllowed() && !Region_ObserveYawAllowed() && !SC_InitialObserveAllowed() &&')
    print('Scoped initial-observe XY gate repaired; no hardware.',flush=True)


if __name__=='__main__':main()
