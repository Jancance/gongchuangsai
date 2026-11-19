"""Read the current verified near-candidate material RX fields; no target writes."""
import json
import argparse
import sys
from datetime import datetime
from pathlib import Path
import gripper210_attach_read_20261105 as live
from source_body_probe_20261111 import attach
import source_alignment_live_20261112 as alignment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', nargs='?', default='material-check')
    parser.add_argument('--region-candidate', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    name = 'region-batch' if args.region_candidate else 'near-pzq'
    evidence = root / 'evidence/integration/2026-11-13' / name
    alignment.E = evidence
    alignment.P = root / ('tmp/source_region_batch_20261113/project' if args.region_candidate
                          else 'tmp/source_near_pzq_20261113/project')
    _, image = alignment.manifest()
    live.IMAGE = root / image['axf']
    symbols, segments = live.load_image()
    report = dict(readonly=True, image_sha256=image['sha256'],
                  phase=args.phase)
    with attach() as session:
        target = session.target
        report['core_state'] = target.get_state().name
        count = 0
        for address, data in segments:
            for offset in range(0, len(data), 4096):
                part = data[offset:offset + 4096]
                assert bytes(target.read_memory_block8(address + offset, len(part))) == part
                count += len(part)
        report['flash_bytes_verified'] = count
        keys = ('g_vis_rx_bytes g_vis_rx_bad g_uart6_rx_errors g_vis_material_valid '
                'g_vis_mat_cycle g_vis_mat_left g_vis_mat_right g_vis_mat_down '
                'g_vis_mat_tick g_src_now_ms g_it_state g_it_cause g_it_rejected '
                'g_pzq_near_state g_it_auto_state g_safety_fault g_rtos_fault '
                'g_chassis_auto_active').split()
        report['values'] = {key: target.read32(symbols[key]) for key in keys}
    path = evidence / ('material-readonly-' + datetime.now().strftime('%Y%m%d-%H%M%S') + '.json')
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    print(path)


if __name__ == '__main__':
    main()
