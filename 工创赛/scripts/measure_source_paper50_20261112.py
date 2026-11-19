"""Extract current paper line evidence only; never activates calibration."""
from pathlib import Path
import json, hashlib
import cv2
import numpy as np
from measure_source_paper50_20261111 import read, line

R = Path(__file__).resolve().parents[1]
B = R / 'evidence/integration/2026-11-11/source-view-entry'
S = B / 'source-observe-20261112-010846'
O = B / 'paper50-current-20261112'

def main():
    O.mkdir(exist_ok=True)
    im = read(S / 'last-raw.png')
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    initial = [[808,478],[1009,493],[997,695],[795,681]]
    edges = [line(gray, initial[i], initial[(i+1)%4]) for i in range(4)]
    corners = []
    for i in range(4):
        p = np.cross(edges[(i-1)%4][0], edges[i][0])
        corners.append((p[:2]/p[2]).tolist())
    before = cv2.cvtColor(read(B / 'source-observe-20261112-010523/last-raw.png'), cv2.COLOR_BGR2GRAY)
    patches = []
    for x,y in [(1005,330),(945,374),(1014,403),(953,449),(1021,480),(1074,359),(996,239)]:
        response = cv2.matchTemplate(gray[y-24:y+25,x-24:x+25], before[y-12:y+13,x-12:x+13], cv2.TM_CCOEFF_NORMED)
        _, score, _, where = cv2.minMaxLoc(response)
        patches.append(dict(before_px=[x,y], correlation=score, integer_shift_px=[where[0]-12,where[1]-12]))
    session = json.loads((S/'session.json').read_text(encoding='utf-8'))
    rows = [json.loads(x) for x in (S/'observations.jsonl').read_text(encoding='utf-8').splitlines() if x.strip()]
    assert session['frames'] == len(rows) == 1 and session['error'] is None
    d = dict(paper_width_mm=50, paper_height_mm=50, size_basis='retained user-measured 50mm square',
        image_size=[im.shape[1],im.shape[0]], corners_ABCD_px=corners,
        border_line_support_counts=[len(e[1]) for e in edges], border_line_rms_px=[e[2] for e in edges],
        frame_boundary_points_px=[x['boundary_points_px'] for x in rows], rim_points_px=rows[-1]['boundary_points_px'], fixed_surface_patches=patches,
        serial=session['serial'], motion_allowed=False, frames=len(rows), single_frame_repeatability_verified=False,
        independent_metric_truth_verified=False,
        sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in S.iterdir() if f.is_file()})
    (O/'measurement-input.json').write_text(json.dumps(d,indent=2),encoding='utf-8')
    for name, p in zip('ABCD', corners):
        xy=tuple(np.rint(p).astype(int));cv2.circle(im,xy,5,(0,0,255),2)
        cv2.putText(im,name,(xy[0]+8,xy[1]-8),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,255),2)
    (O/'corners-overlay.png').write_bytes(cv2.imencode('.png',im)[1].tobytes())
    print(json.dumps(dict(corners=corners,line_rms=d['border_line_rms_px'],patches=patches),indent=2))

if __name__ == '__main__':
    main()
