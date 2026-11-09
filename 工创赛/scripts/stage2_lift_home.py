import time
from pathlib import Path
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target
R = Path('.').resolve()
transport.live.IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
names, _ = transport.live.load_image()
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
s.open(); t = s.target
g = lambda k: t.read32(names[k])
print('before lift=%d cause=%d it=%d' % (g('g_lift_ruler_state'), g('g_lift_ruler_cause'), g('g_it_state')))
t.write32(names['g_it_request'], 1)
t0 = time.time()
while time.time() - t0 < 60:
    time.sleep(0.5)
    st = g('g_it_state'); ca = g('g_it_cause'); rj = g('g_it_rejected')
    ls = g('g_lift_ruler_state'); lc = g('g_lift_ruler_cause'); ph = g('g_lift_ruler_phase')
    tv = g('g_lift_compare_top_valid'); bv = g('g_lift_compare_backoff_valid')
    op = g('g_lift_compare_operation')
    print('t=%5.1f it=%d/%d rej=%d lift=%d phase=%d cause=%d top=%d backoff=%d op=%d'
          % (time.time() - t0, st, ca, rj, ls, ph, lc, tv, bv, op))
    if st == 2 and ls == 4:
        print('HOME_OK')
        break
    if st == 9 or ls in (5, 6):
        print('HOME_FAIL')
        break
