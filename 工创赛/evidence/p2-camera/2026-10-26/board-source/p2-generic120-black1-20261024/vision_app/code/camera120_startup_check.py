"""120°相机独立启动取证：不加载旧标定，不连接串口或执行机构。"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import cv2

PROFILE = Path(__file__).resolve().parent / 'config' / 'generic_120_camera.json'
CONTROL_NAMES = ('exposure_auto', 'exposure_absolute', 'gain',
                 'white_balance_temperature_auto', 'white_balance_temperature')


def command(arguments):
    return subprocess.run(arguments, check=True, capture_output=True,
                          text=True, timeout=5).stdout


def verify_identity(profile):
    device = profile['device']
    if not Path(device).exists():
        raise RuntimeError('未找到指定120°相机，请检查USB连接；不自动改用video0')
    raw = command(['udevadm', 'info', '--query=property', '--name=' + device])
    properties = dict(line.split('=', 1) for line in raw.splitlines() if '=' in line)
    for key, expected in [('ID_VENDOR_ID', profile['usb_vendor_id']),
                          ('ID_MODEL_ID', profile['usb_model_id']),
                          ('ID_SERIAL', profile['usb_serial'])]:
        if properties.get(key) != expected:
            raise RuntimeError('相机身份不匹配: ' + key)
    return properties


def set_control(device, setting):
    command(['v4l2-ctl', '-d', device, '--set-ctrl=' + setting])


def read_for(cap, seconds, shape):
    deadline = time.monotonic() + seconds
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            raise RuntimeError('相机读帧失败')
        if frame.shape != shape:
            raise RuntimeError('实际解码尺寸/通道不符: ' + str(frame.shape))
        if time.monotonic() >= deadline:
            return frame


def initialize_camera(cap, profile):
    """每次新开采集都初始化；关闭AWB后不写手动色温、不重开。"""
    device = profile['device']
    shape = (profile['height'], profile['width'], 3)
    if not cap.isOpened():
        raise RuntimeError('无法打开相机，请关闭其他预览')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, profile['width'])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, profile['height'])
    cap.set(cv2.CAP_PROP_FPS, profile['fps_request'])
    read_for(cap, 0, shape)
    for setting in ('exposure_auto=1',
                    'exposure_absolute=' + str(profile['exposure_absolute']),
                    'gain=' + str(profile['gain']),
                    'white_balance_temperature_auto=1'):
        set_control(device, setting)
    read_for(cap, profile['awb_adjust_seconds'], shape)
    set_control(device, 'white_balance_temperature_auto=0')
    frame = read_for(cap, profile['locked_capture_seconds'], shape)
    raw = command(['v4l2-ctl', '-d', device, '--get-ctrl=' + ','.join(CONTROL_NAMES)])
    actual = {}
    for line in raw.splitlines():
        if ':' in line:
            name, value = line.split(':', 1)
            if name.strip() in CONTROL_NAMES:
                actual[name.strip()] = int(value.strip().split()[0])
    expected = {'exposure_auto': 1, 'exposure_absolute': profile['exposure_absolute'],
                'gain': profile['gain'], 'white_balance_temperature_auto': 0}
    if any(actual.get(name) != value for name, value in expected.items()):
        raise RuntimeError('控制读回不符合候选设置: ' + raw)
    return frame, raw


def capture_once(profile, output, label):
    identity = verify_identity(profile)
    cv2.ocl.setUseOpenCL(False)
    cap = cv2.VideoCapture(profile['device'], cv2.CAP_V4L2)
    try:
        frame, controls = initialize_camera(cap, profile)
        output.mkdir(parents=True, exist_ok=True)
        path = output / (label + '.png')
        if not cv2.imwrite(str(path), frame):
            raise RuntimeError('图像保存失败')
        report = {
            'profile': profile, 'identity': identity,
            'decoded_size': [frame.shape[1], frame.shape[0]],
            'controls': controls, 'image_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'opencv_fps_readback': cap.get(cv2.CAP_PROP_FPS),
            'measured_fps': None, 'awb_convergence_verified': False,
            'color_repeatability_verified': False, 'motion_allowed': False,
            'note': '等待照片对照；控制字段符合不等于颜色准确或启动复现通过。'
        }
        path.with_suffix('.json').write_text(json.dumps(report, ensure_ascii=False, indent=2),
                                            encoding='utf-8')
        print(str(path), flush=True)
        print(controls, flush=True)
    finally:
        cap.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='证据输出目录，默认自动生成时间目录')
    parser.add_argument('--single', choices=['startup1', 'startup2'], help=argparse.SUPPRESS)
    args = parser.parse_args()
    profile = json.loads(PROFILE.read_text(encoding='utf-8'))
    output = args.output or Path.home() / ('camera120_startup_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    if args.single:
        capture_once(profile, output, args.single)
        return
    # 父进程限制整个子进程时长；包括阻塞的VideoCapture/open/read。
    print('将独立打开相机两次，每次重新调整并关闭自动白平衡。', flush=True)
    try:
        for label in ('startup1', 'startup2'):
            subprocess.run([sys.executable, str(Path(__file__).resolve()),
                            '--single', label, '--output', str(output)],
                           check=True, timeout=40)
    except (subprocess.SubprocessError, KeyboardInterrupt) as exc:
        raise SystemExit('测试中止，已有文件只作部分证据：' + str(exc)) from exc
    print('请回传两张PNG及终端输出；保存成功不等于画质通过。', flush=True)
    print('证据目录: ' + str(output), flush=True)


if __name__ == '__main__':
    main()
