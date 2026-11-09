import hashlib
from pathlib import Path
import source_stage2_readonly_sync_probe_20261105 as transport
R = Path('.').resolve()
A = R / 'tmp/integration11_stream_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
B = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
print('A sha', hashlib.sha256(A.read_bytes()).hexdigest())
print('B sha', hashlib.sha256(B.read_bytes()).hexdigest())
def syms(p):
    transport.live.IMAGE = p
    n, _ = transport.live.load_image()
    return n
na, nb = syms(A), syms(B)
keys = ['g_it_request','g_it_state','g_it_cause','g_it_axes','g_it_extension_mm','g_it_carrier_deg',
        'g_lift_ruler_request','g_lift_ruler_state','g_lift_ruler_target_raw','g_dm_jog_request',
        'g_dm_can2_read_request','g_startup_axes_state','g_vis_rx_bytes','g_vis_material_valid',
        'g_vis_center_x','g_vis_mm_per_px','g_vis_taught_mask','g_vis_ref_x','g_vis_pick_region']
diff = 0
for k in keys:
    a = na.get(k); b = nb.get(k)
    flag = '' if a == b else '   <-- DIFF'
    if a != b: diff += 1
    print('%-26s A=%-12s B=%-12s%s' % (k, hex(a) if a else 'missing', hex(b) if b else 'missing', flag))
print('symbols A=%d B=%d  diffs=%d' % (len(na), len(nb), diff))
common = set(na) & set(nb)
mismatch = sum(1 for k in common if na[k] != nb[k])
print('common=%d mismatched_addrs=%d' % (len(common), mismatch))
