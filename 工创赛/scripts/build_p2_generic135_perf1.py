"""135°blue1的等价计算优化：稀疏蒙版轮廓裁剪及逐帧浅蓝缓存。"""
from pathlib import Path
import hashlib
import json
import shutil
from zipfile import ZipFile, ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'output/p2_vision_generic135_blue1_20261026/vision_app'
APP=ROOT/'output/p2_vision_generic135_perf1_20261026/vision_app'

def change(text,old,new):
    if old not in text:raise RuntimeError('Source mismatch: '+old[:70])
    return text.replace(old,new)

def build():
    if APP.exists():raise RuntimeError('Refuse to overwrite existing candidate')
    shutil.copytree(BASE,APP,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    p=APP/'code/project_detector.py';s=p.read_text(encoding='utf-8')
    helper='''def sparse_external_contours(mask):
    """只裁掉全零边缘，保持原图坐标及RETR_EXTERNAL/CHAIN_APPROX_SIMPLE。"""
    x,y,w,h=cv2.boundingRect(mask)
    if not w or not h:
        return [],None
    return cv2.findContours(mask[y:y+h,x:x+w],cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE,offset=(x,y))


'''
    s=change(s,'def bright_round_face(',helper+'def bright_round_face(')
    s=change(s,'def current_pale_circle(frame,masks,parent,hsv,value):','def current_pale_circle(frame,masks,parent,hsv,value,cache=None):')
    s=change(s,'''    palette=dict(masks)
    palette[6]=cv2.inRange(hsv,(90,25,100),(115,189,255))
    if cv2.countNonZero(palette[6][y:y+h,x:x+w])<325:return None
    palette[3]=cv2.bitwise_and(masks[3],cv2.bitwise_not(palette[6]))''','''    # 缓存仅存活于一次analyze，不能跨帧复用；蒙版/阈值和后续圆弧门不变。
    palette=None if cache is None else cache.get('pale_palette')
    if palette is None:
        # 少于325票必定达不到500有效像素×65%色票，先在本候选ROI精确排除。
        local=cv2.inRange(hsv[y:y+h,x:x+w],(90,25,100),(115,189,255))
        if cv2.countNonZero(local)<325:return None
        palette=dict(masks)
        palette[6]=cv2.inRange(hsv,(90,25,100),(115,189,255))
        palette[3]=cv2.bitwise_and(masks[3],cv2.bitwise_not(palette[6]))
        if cache is not None:cache['pale_palette']=palette
    if cv2.countNonZero(palette[6][y:y+h,x:x+w])<325:return None''')
    s=change(s,'hsv=None,blue_settings=None):','hsv=None,blue_settings=None,pale_cache=None):')
    s=change(s,'contours,_=cv2.findContours(geometry,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)',
               'contours,_=sparse_external_contours(geometry)')
    s=change(s,'face=current_pale_circle(frame,masks,refined,hsv,value)',
               'face=current_pale_circle(frame,masks,refined,hsv,value,cache=pale_cache)')
    s=change(s,'rejected=[];accepted=[];circle_diagnostics=[]',
               'rejected=[];accepted=[];circle_diagnostics=[];pale_cache={}')
    s=change(s,"color_config.get('round_face_color_candidate'),hsv=hsv):",
               "color_config.get('round_face_color_candidate'),hsv=hsv,pale_cache=pale_cache):")
    s=change(s,'hsv=hsv,blue_settings=blue_settings):',
               'hsv=hsv,blue_settings=blue_settings,pale_cache=pale_cache):')
    p.write_text(s,encoding='utf-8')
    manifest={'base':'generic135 blue1 2026-10-26','changed_file':'code/project_detector.py',
              'before_sha256':hashlib.sha256((BASE/'code/project_detector.py').read_bytes()).hexdigest(),
              'after_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),
              'motion_allowed':False,'hardware_acceptance':False,
              'scope':'Exact sparse-mask contour cropping and same-frame-only pale palette cache; no threshold/gate changes'}
    (APP/'GENERIC135_PERF1_MANIFEST.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    shutil.copy2(ROOT/'docs/P2_GENERIC135_PERF1_2026-10-26.md',APP/'PERF1_DEPLOY.md')
    compare=APP.parent/'p2_generic135_perf1_saved_compare.py'
    shutil.copy2(ROOT/'output/p2_generic135_perf1_saved_compare.py',compare)
    archive=APP.parent.with_suffix('.zip')
    with ZipFile(archive,'w',ZIP_DEFLATED) as z:
        for path in sorted(APP.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix!='.pyc':z.write(path,path.relative_to(APP.parent).as_posix())
        z.write(compare,compare.name)
    print(archive,hashlib.sha256(archive.read_bytes()).hexdigest())

if __name__=='__main__':build()
