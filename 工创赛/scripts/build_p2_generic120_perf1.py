"""从已部署120度候选生成隔离性能修订，不覆盖工作版本。"""
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'output/p2_vision_generic120_20261023/vision_app'
APP = ROOT / 'output/p2_vision_generic120_perf1_20261023/vision_app'


def change(path, old, new):
    text = path.read_text(encoding='utf-8')
    if text.count(old) != 1:
        raise RuntimeError('源版本/替换位置不唯一: ' + str(path))
    path.write_text(text.replace(old, new), encoding='utf-8')


def build():
    shutil.copytree(BASE, APP, dirs_exist_ok=True, ignore=shutil.ignore_patterns('__pycache__','*.pyc','diagnostics'))
    code = APP / 'code'
    change(code/'颜色标定.py', '''    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for hsv_range in ranges:
        lower = np.asarray(hsv_range["lower"], dtype=np.uint8)
        upper = np.asarray(hsv_range["upper"], dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    return mask''', '''    # 首段本身已是完整蒙版，无须再与全零图做OR；后续段原位合并。
    mask = None
    for hsv_range in ranges:
        lower = np.asarray(hsv_range["lower"], dtype=np.uint8)
        upper = np.asarray(hsv_range["upper"], dtype=np.uint8)
        current = cv2.inRange(hsv, lower, upper)
        if mask is None:
            mask = current
        else:
            cv2.bitwise_or(mask, current, dst=mask)
    return mask if mask is not None else np.zeros(hsv.shape[:2], dtype=np.uint8)''')
    change(code/'project_detector.py', '''        mask=np.zeros(frame.shape[:2],np.uint8)
        for m in calibrated_masks.values():mask=cv2.bitwise_or(mask,m)''', '''        mask=next(iter(calibrated_masks.values())).copy()
        for m in list(calibrated_masks.values())[1:]:cv2.bitwise_or(mask,m,dst=mask)''')
    change(code/'project_detector.py', '''                    scores=[]
                    for delta in offsets:
                        xx=np.rint(cx+ax*cos+delta*nx).astype(int)-left
                        yy=np.rint(cy+ay*sin+delta*ny).astype(int)-top
                        good=(xx>=0)&(xx<gx.shape[1])&(yy>=0)&(yy<gx.shape[0])
                        sx=np.zeros(72);sy=np.zeros(72)
                        sx[good]=gx[yy[good],xx[good]];sy[good]=gy[yy[good],xx[good]]
                        magnitude=np.hypot(sx,sy)
                        alignment=np.abs(sx*nx+sy*ny)/np.maximum(magnitude,1.)
                        scores.append(np.where(good&(magnitude>=40)&(alignment>=.7),magnitude*alignment,0.))
                    scores=np.array(scores);edge=scores.max(axis=0)>0;fraction=float(edge.mean())''', '''                    # 同样的9组偏移、72角度与双精度运算，只合并Python循环。
                    xx=np.rint(cx+ax*cos+offsets[:,None]*nx).astype(int)-left
                    yy=np.rint(cy+ay*sin+offsets[:,None]*ny).astype(int)-top
                    good=(xx>=0)&(xx<gx.shape[1])&(yy>=0)&(yy<gx.shape[0])
                    sx=np.zeros(xx.shape);sy=np.zeros(yy.shape)
                    sx[good]=gx[yy[good],xx[good]];sy[good]=gy[yy[good],xx[good]]
                    magnitude=np.hypot(sx,sy)
                    alignment=np.abs(sx*nx+sy*ny)/np.maximum(magnitude,1.)
                    scores=np.where(good&(magnitude>=40)&(alignment>=.7),magnitude*alignment,0.)
                    edge=scores.max(axis=0)>0;fraction=float(edge.mean())''')
    change(code/'project_detector.py', '''                    fractions={names[k]:float((m[ry:ry+rh,rx:rx+rw][valid]>0).mean()) for k,m in masks.items()}
                    if disjoint:''', '''                    # 原门要求黑票>=65%；不足时竞争票不会改变拒绝结论。
                    black_fraction=float((masks[5][ry:ry+rh,rx:rx+rw][valid]>0).mean())
                    if black_fraction<.65:
                        colour_rejections+=1
                        continue
                    fractions={names[k]:(black_fraction if k==5 else float((m[ry:ry+rh,rx:rx+rw][valid]>0).mean())) for k,m in masks.items()}
                    if disjoint:''')
    change(code/'project_follow.py', '''        self.history=history
        return {'state':'FOLLOWING_PIXEL' if targets else 'SEARCHING',''', '''        self.history=history
        result={'state':'FOLLOWING_PIXEL' if targets else 'SEARCHING',''')
    change(code/'project_follow.py', '''                'circle_diagnostics':detection.get('circle_diagnostics',[])}

def draw_follow''', '''                'circle_diagnostics':detection.get('circle_diagnostics',[])}
        # 跟踪/预测/组装也占时间，最终交付前再次检查，不延长原150ms门。
        if time.monotonic()-now>.15:
            self.reset()
            result.update(state='STALE_RESULT',targets=[],unknown_candidates=0,retired_ids=[])
        return result

def draw_follow''')
    changes=[]
    for path in sorted(code.rglob('*')):
        if not path.is_file() or '__pycache__' in path.parts:continue
        old=BASE/'code'/path.relative_to(code)
        if old.exists() and old.read_bytes()!=path.read_bytes():
            changes.append({'file':'code/'+path.relative_to(code).as_posix(),
                            'before_sha256':hashlib.sha256(old.read_bytes()).hexdigest(),
                            'after_sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    report={'base':'p2_vision_generic120_20261023', 'hardware_acceptance':False,
            'motion_allowed':False,'thresholds_changed':False,'changes':changes}
    (APP/'PERF1_MANIFEST.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (APP/'PERF1_README.md').write_text(
        '# 120度性能候选perf1\n\n基于已部署候选，仅改3个源码文件，见PERF1_MANIFEST.json。'
        '相机/HSV/分辨率/线程数不变，原150ms门保留并补最终输出检查。'
        '保留黑色恢复、所有候选数量与判定门；本机等价回归不代表板端提速/六色/毫米或抓放验收。'
        '\n\n独立解压至~/p2-generic120-perf1-20261024，原目录保留。'
        '先python3 ~/p2-generic120-perf1-20261024/vision_app/run.py --check。'
        '下一项复用原诊断脚本的--app指定新目录，再按真实输出决定是否采用。\n',encoding='utf-8')
    readme=APP/'README.md'
    readme.write_text('> 2026-10-24：此目录是perf1性能候选，先读PERF1_DEPLOY.md；下文部署命令保留基础版本历史。\n\n'+readme.read_text(encoding='utf-8'),encoding='utf-8')
    shutil.copy2(ROOT/'docs/P2_GENERIC120_PERF1_2026-10-24.md',APP/'PERF1_DEPLOY.md')
    return report


if __name__=='__main__':print(json.dumps(build(),ensure_ascii=False,indent=2))
