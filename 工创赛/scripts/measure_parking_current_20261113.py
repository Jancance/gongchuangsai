"""Extract contact arcs from this session's real image; no actuator access."""
from pathlib import Path
import argparse
import hashlib
import json
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'evidence/integration/2026-11-13/align-new-20261113-capture'


def main():
    global OUT
    parser = argparse.ArgumentParser()
    parser.add_argument('--session-dir', type=Path)
    parser.add_argument('--colors', type=int, nargs=3, default=[5, 2, 1], choices=range(1, 7))
    parser.add_argument('--centers', type=float, nargs=6,
                        default=[642, 185, 1348, 218, 973, 806])
    args = parser.parse_args()
    if args.session_dir is not None:
        OUT = args.session_dir.resolve()
    path = OUT / 'last-raw.png'
    image = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert image is not None and image.shape[:2] == (1080, 1920)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    overlay = image.copy()
    arcs = []
    # Approximate search centers selected from this actual image, not old slots.
    for index, name in enumerate(['left', 'right', 'near']):
        color = args.colors[index]
        cx, cy = args.centers[index * 2:index * 2 + 2]
        direction = np.arctan2(540 - cy, 960 - cx)
        points = []
        for angle in direction + np.deg2rad(np.linspace(-60, 60, 241)):
            radius = np.arange(101 - 22, 101 + 22.01, .25)
            x, y = cx + radius * np.cos(angle), cy + radius * np.sin(angle)
            values = cv2.remap(gray, x.astype('float32')[None],
                               y.astype('float32')[None], cv2.INTER_LINEAR)[0].astype(float)
            gradient = (values[8:] - values[:-8]) / 2
            index = int(np.argmax(gradient)) + 4
            if 6 < index < len(radius) - 7 and gradient[index - 4] > 10:
                points.append([float(x[index]), float(y[index])])
        arcs.append(dict(region=name, color_id=color, boundary_px=points,
                         search_center_px=[cx, cy], manual_current_image_search_prior=True))
        for point in points[::3]:
            cv2.circle(overlay, tuple(np.rint(point).astype(int)), 2, (0, 255, 255), 1)
    report = dict(image_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  source_session=OUT.name, arcs=arcs, motion_allowed=False)
    (OUT / 'measurement-input.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    (OUT / 'contact-arcs.png').write_bytes(cv2.imencode('.png', overlay)[1].tobytes())
    print(json.dumps({'arc_counts': [len(a['boundary_px']) for a in arcs],
                      'image_sha256': report['image_sha256']}))


if __name__ == '__main__':
    main()
