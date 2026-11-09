"""从保存的原尺寸图点选标记中心，不打开相机或串口。"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("markers", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.resolve() in (args.image.resolve(), args.markers.resolve()):
        parser.error("输出须为新文件，保留输入原件")
    frame = cv2.imdecode(np.fromfile(args.image, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        parser.error("无法读取原图")
    data = json.loads(args.markers.read_text(encoding="utf-8-sig"))
    height, width = frame.shape[:2]
    if data["context"]["image_size"] != [width, height]:
        parser.error("图像与标定模板分辨率不一致；请用原图并核对模板")
    selected = []
    name = "Source markers - click in ID order; U undo; S save; Q cancel"
    def mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(selected) < len(data["markers"]) and 0 <= x < width and 0 <= y < height:
            selected.append([x, y])
    cv2.namedWindow(name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow(name, 1140, max(300, round(1140*height/width)))
    cv2.setMouseCallback(name, mouse)
    print("点选顺序: " + " -> ".join(m["id"] for m in data["markers"]))
    try:
        while True:
            view = frame.copy()
            for marker, point in zip(data["markers"], selected):
                cv2.drawMarker(view, tuple(point), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                cv2.putText(view, marker["id"], (point[0]+10, point[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            next_id = data["markers"][len(selected)]["id"] if len(selected) < len(data["markers"]) else "S to save"
            cv2.putText(view, "Next: " + next_id, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow(name, view)
            key = cv2.waitKey(30) & 0xff
            if key in (ord('q'), 27) or cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1:
                return 1
            if key == ord('u') and selected:
                selected.pop()
            if key == ord('s') and len(selected) == len(data["markers"]):
                for marker, point in zip(data["markers"], selected):
                    marker["image_px"] = point
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
                print(f"像素已保存至 {args.output}；仍须填写实测毫米/姿态/高度")
                return 0
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
