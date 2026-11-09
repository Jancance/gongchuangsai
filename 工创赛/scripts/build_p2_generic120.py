"""从已核验板端快照构建隔离的120°候选；不修改原程序或板端。"""
from pathlib import Path
import hashlib
import json
import shutil
import sys
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'tmp/p2_board_source_20261023'
APP = ROOT / 'output/p2_vision_generic120_20261023/vision_app'
CODE = APP / 'code'


def replace(path, old, new):
    text = path.read_text(encoding='utf-8')
    if old not in text:
        raise RuntimeError(f'预期源内容不存在: {path}: {old[:60]}')
    path.write_text(text.replace(old, new), encoding='utf-8')


def build():
    manifest = json.loads((ROOT / 'evidence/p2-camera/2026-10-23/board-source/source_manifest.json').read_text(encoding='utf-8'))
    for name, digest in manifest['files'].items():
        if hashlib.sha256((SOURCE / name).read_bytes()).hexdigest() != digest:
            raise RuntimeError('板端快照哈希不符: ' + name)
    shutil.copytree(SOURCE, APP, dirs_exist_ok=True)
    guide = ROOT / 'docs/P2_GENERIC120_MAIN_DEPLOY_2026-10-23.md'
    shutil.copy2(guide, APP / 'DEPLOY_120_CAMERA.md')
    readme = APP / 'README.md'
    readme.write_text('> 2026-10-23：本目录为板端快照衍生的120度候选，先读 DEPLOY_120_CAMERA.md。下文旧DECXIN参数仅保留历史，不用于本候选。\n\n' + readme.read_text(encoding='utf-8'), encoding='utf-8')
    legacy = APP / 'reference_config/board_decxin_20261023'
    shutil.copytree(SOURCE / 'code/config', legacy, dirs_exist_ok=True)
    for name in ('camera120_startup_check.py', 'config/generic_120_camera.json'):
        shutil.copy2(ROOT / 'vision_app/code' / name, CODE / name)

    path = CODE / 'project_camera.py'
    text = path.read_text(encoding='utf-8')
    start = text.index('DEFAULT_DEVICE=')
    end = text.index('\ndef open_project_camera', start)
    text = text[:start] + '''import json
from camera120_startup_check import initialize_camera, verify_identity

CAMERA_PROFILE_PATH=Path(__file__).resolve().parent/'config/generic_120_camera.json'
CAMERA_PROFILE=json.loads(CAMERA_PROFILE_PATH.read_text(encoding='utf-8'))
DEFAULT_DEVICE=CAMERA_PROFILE['device']
DEFAULT_WIDTH=CAMERA_PROFILE['width']
DEFAULT_HEIGHT=CAMERA_PROFILE['height']
DEFAULT_FPS=float(CAMERA_PROFILE['fps_request'])
DEFAULT_CALIBRATION=Path(__file__).resolve().parent/'config/generic_120_unconfigured.yaml'

def prepare_camera(cap,device,width=DEFAULT_WIDTH,height=DEFAULT_HEIGHT,fps=DEFAULT_FPS,lock_controls=True):
    """仅接受已识别设备与候选模式；每次打开/重连重新调整并关闭AWB。"""
    cv2.ocl.setUseOpenCL(False)
    try:
        if (str(device),width,height,fps)!=(DEFAULT_DEVICE,DEFAULT_WIDTH,DEFAULT_HEIGHT,DEFAULT_FPS):
            raise RuntimeError('120度候选只接受独立profile中的设备和1920x1080 MJPG30请求')
        if not lock_controls:
            raise RuntimeError('120度候选不得绕过启动锁参流程')
        verify_identity(CAMERA_PROFILE)
        cap.set(cv2.CAP_PROP_BUFFERSIZE,4)
        initialize_camera(cap,CAMERA_PROFILE)
        return cap
    except Exception:
        cap.release()
        raise
''' + text[end:]
    text = text.replace('if (width,height)!=(1280,960) or fps!=30:', 'if (str(device),width,height,fps)!=(DEFAULT_DEVICE,DEFAULT_WIDTH,DEFAULT_HEIGHT,DEFAULT_FPS):')
    text = text.replace('持续采集配置目前只验证1280x960 MJPG30', '120度持续采集仅接受profile候选模式')
    text = text.replace('self._slot=LatestSlot(ctx);', 'self._slot=LatestSlot(ctx,shape=(height,width,3));')
    text = text.replace('deadline=time.monotonic()+25', 'deadline=time.monotonic()+40')
    text = text.replace('本项目DECXIN相机适配', '本项目120度相机候选适配')
    path.write_text(text, encoding='utf-8')

    path = CODE / 'project_capture.py'
    text = path.read_text(encoding='utf-8')
    start = text.index('        cap=cv2.VideoCapture(device,cv2.CAP_V4L2)')
    end = text.index('        controls=subprocess.run(', start)
    text = text[:start] + '''        from project_camera import prepare_camera, DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_FPS
        cap=cv2.VideoCapture(device,cv2.CAP_V4L2)
        prepare_camera(cap,device,DEFAULT_WIDTH,DEFAULT_HEIGHT,DEFAULT_FPS)
        buffer_setting_accepted=None  # 启动函数设置4；不把返回值当零积压证明。
        if stop.is_set():return
''' + text[end:]
    path.write_text(text, encoding='utf-8')

    # 主程序及四工具使用独立分辨率；算法阈值保持板端值，尚未做新相机调参。
    files = ['main.py', '颜色标定.py', '转盘参数设置.py', '物料抓取区域标定.py',
             '叠放位直线标定.py', 'material_pickup_detector.py']
    for name in files:
        path = CODE / name
        text = path.read_text(encoding='utf-8')
        text = text.replace('1280', '1920').replace('960', '1080')
        text = text.replace('decxin_camera_unconfigured.yaml', 'generic_120_unconfigured.yaml')
        text = text.replace('DECXIN', 'Generic120')
        path.write_text(text, encoding='utf-8')

    (CODE / 'config/generic_120_unconfigured.yaml').write_text(
        '%YAML:1.0\n---\ncamera_name: generic_10bb_2b08_unconfigured\nimage_width: 1920\nimage_height: 1080\ncalibrated: 0\n', encoding='utf-8')
    profile = json.loads((CODE / 'config/generic_120_camera.json').read_text(encoding='utf-8'))
    for name in ('material_color_thresholds.json', 'material_pickup_detector.json'):
        path = CODE / 'config' / name
        data = json.loads(path.read_text(encoding='utf-8'))
        data.update(image_width=1920, image_height=1080)
        if name.startswith('material_color'):
            data.update(camera_device=profile['device'], calibration_file='config/generic_120_unconfigured.yaml')
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    path = CODE / 'config/project_profile.json'
    data = json.loads(path.read_text(encoding='utf-8'))
    data.update(camera_model='Generic_USB_CAMERA_10bb_2b08_user120deg', image_size=[1920,1080],
                camera_profile=profile['profile_id'], camera_verified=False, geometry_verified=False,
                ring_geometry_verified=False, stack_geometry_verified=False, serial_enabled=False,
                follow_grasp_enabled=False)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    path = CODE / 'config/camera_processing.json'
    path.write_text(json.dumps({'undistort_enabled': False, 'reason': '120度新相机未标定，禁止沿用旧内参'}, ensure_ascii=False, indent=2), encoding='utf-8')
    report = {'source_archive_sha256': manifest['source_archive_sha256'],
              'hardware_acceptance': False, 'motion_allowed': False, 'changed': [], 'preserved': []}
    for name, digest in manifest['files'].items():
        current = hashlib.sha256((APP / name).read_bytes()).hexdigest()
        report['preserved' if current == digest else 'changed'].append(name)
    (APP / 'ADAPTATION_MANIFEST.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    return report


if __name__ == '__main__':
    report = build()
    print(json.dumps({'changed': report['changed'], 'preserved_count': len(report['preserved'])}, ensure_ascii=False))
