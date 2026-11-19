"""Build the independent Pi-side committed-pick observer, without deployment."""
from pathlib import Path
R=Path(__file__).resolve().parents[1]

def main():
    s=(R/'output/top_material_stream_20261114.py').read_text(encoding='utf-8')
    def change(a,b):
        nonlocal s
        assert a in s,a;s=s.replace(a,b)
    change("value['state'] not in ('OBSERVE', 'CONFIRM')","value['state'] not in ('OBSERVE', 'ALIGN')")
    change("not remaining or len(set(remaining)) != len(remaining)",
           "(not remaining and value['state'] != 'ALIGN') or len(set(remaining)) != len(remaining)")
    change("parser.add_argument('--replay', type=Path)",
           "parser.add_argument('--replay', type=Path)\n    parser.add_argument('--body-profile', type=Path, required=True)")
    change("    import cv2\n    import main as app",
           "    import cv2\n    from source_body_registration import load_profile, estimate\n    body_profile = load_profile(args.body_profile)\n    import main as app")
    change('    profile_since = 0.0','    profile_since = 0.0\n    profile_frames = 0')
    change('nonlocal packets, last_send, current_profile, profile_since, motion_seen, stop_since, stop_sequence',
           'nonlocal packets, last_send, current_profile, profile_since, motion_seen, stop_since, stop_sequence, profile_frames')
    change('            profile_since = now','            profile_since = now\n            profile_frames = 0')
    change('        detection = app.material_pickup_detector.detect(frame, timestamp=now)',
           '        profile_frames += 1\n        detection = app.material_pickup_detector.detect(frame, timestamp=now)')
    change('        if active:\n            stopped = gate.update',
           "        if active and control['state'] == 'OBSERVE':\n            stopped = gate.update")
    change('        settled = active and len(gate.history) >= 2',
           '        settled = active and profile_frames >= 2')
    change('        if bridge is not None:\n            bridge.publish(',
           "        body_result = None\n        if active and control['state'] == 'ALIGN' and settled:\n            try:\n                body_result = dict(available=True, **estimate(frame, body_profile))\n            except (ValueError, ArithmeticError) as error:\n                body_result = dict(available=False, error=str(error), motion_allowed=False)\n        if bridge is not None:\n            bridge.publish(")
    change("                material_packets_sent=packets))", "                material_packets_sent=packets, body=body_result))")
    s=s.replace('Low source vision - OBSERVATIONS ONLY','Committed source pickup - observations')
    s=s.replace('TOP VIEW','COMMIT TOP').replace('TOP_POSE_CONFIG','COMMIT_POSE_CONFIG').replace('TOP_MATERIAL_SENT','COMMIT_MATERIAL_SENT')
    s=s.replace('low-transport.json','commit-transport.json')
    (R/'output/commit_material_stream_20261114.py').write_text(s,encoding='utf-8')
    print('Prepared Pi observer and fixed-rim bridge locally; nothing deployed.')

if __name__=='__main__':main()
