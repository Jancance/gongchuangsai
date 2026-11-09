import re
from pathlib import Path
import source_stage2_readonly_sync_probe_20261105 as transport
R = Path('.').resolve()
B = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
transport.live.IMAGE = B
names, _ = transport.live.load_image()
src = (R / 'scripts/integration11_v8_threepiece_20261107.py').read_text(encoding='utf-8')
used = sorted(set(re.findall(r"'(g_[a-z0-9_]+)'", src)))
missing = [k for k in used if k not in names]
print('symbols used by v8: %d   missing in integration12 AXF: %d' % (len(used), len(missing)))
for k in missing:
    print('   MISSING', k)
