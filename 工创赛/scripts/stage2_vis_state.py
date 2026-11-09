"""Snapshot of all vision-related globals on the board. Read-only."""
from pathlib import Path
import json
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
transport.live.IMAGE = IMAGE
names, _ = transport.live.load_image()
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
s.open(); t = s.target
assert t.get_state() == Target.State.RUNNING
keys = sorted([k for k in names if k.startswith('g_vis') or k.startswith('g_uart6') or k.startswith('g_source')])
for k in keys:
    v = t.read32(names[k])
    sv = v - (1 << 32) if v >= (1 << 31) else v
    print('%-28s %d' % (k, sv))
