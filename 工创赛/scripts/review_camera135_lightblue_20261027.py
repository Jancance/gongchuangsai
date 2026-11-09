"""复核浅蓝现场包并在已保存漏检图上记录原圆弧拒绝门，不访问硬件。"""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'evidence/p2-camera/2026-10-27/lightblue'
RUN = BASE / 'run-20261027-181512-402452'
APP = ROOT / 'output/p2_vision_generic135_blue2_20261026/vision_app/code'


def main():
    spec = importlib.util.spec_from_file_location('shared_review', ROOT/'scripts/review_camera135_blue2_20261027.py')
    review = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(review)
    review.BASE, review.RUN = BASE, RUN
    with contextlib.redirect_stdout(io.StringIO()):
        review.main('20261027-181512-402452.zip')
    sys.path.insert(0, str(APP))
    import project_detector as detector
    frame = cv2.imdecode(np.frombuffer((RUN/'first-nonstale-lightblue-missing-raw.png').read_bytes(),np.uint8),1)
    config = json.loads((RUN/'material_color_thresholds.json').read_text(encoding='utf8'))
    cv2.setNumThreads(8)
    original = detector.current_pale_circle
    details = []

    def inspect(frame, masks, parent, hsv, value, cache=None):
        result = original(frame,masks,parent,hsv,value,cache)
        x,y,w,h = cv2.boundingRect(parent)
        if result is None:
            palette = dict(masks)
            palette[6] = cv2.inRange(hsv,(90,25,100),(115,189,255))
            palette[3] = cv2.bitwise_and(masks[3],cv2.bitwise_not(palette[6]))
            diagnostics = []
            detector.hinted_color_circles(frame,palette,[parent],[],[{'color':'lightblue','bbox_px':[x,y,w,h]}],value,diagnostics)
            details.append({'parent_bbox_px':[x,y,w,h],'circle_diagnostics':diagnostics})
        return result

    detector.current_pale_circle = inspect
    try:
        result,_ = detector.analyze(frame,config,draw_overlay=False)
    finally:
        detector.current_pale_circle = original
    assert not any(r['label']=='lightblue' for r in result['regions'])
    report = json.loads((BASE/'review.json').read_text(encoding='utf8'))
    report['saved_frame3_replay'] = {'scope':'Only the stored first nonstale missing frame; not all 461 missing frames.',
                                   'current_pale_circle_rejections':details,
                                   'frame_complete_visible_visual_review':True,
                                   'old_arc_threshold':.75}
    (BASE/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'frames':report['frames'],'normal_lightblue_missing':len(report['colors']['lightblue']['normal_missing_frames']),
                      'replay':report['saved_frame3_replay']},ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
