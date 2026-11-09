"""Build a separate hardware-auto-exposure preview; never access camera or serial."""
from pathlib import Path
import hashlib
import json
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[1]
AUTO_FUNCTION = '''
def initialize_auto_controls(cap, profile):
    """硬件自动曝光；预热仅为等待，不证明曝光或白平衡已收敛。"""
    import re
    device = profile['device']
    wb = profile.get('white_balance_mode', 'startup_lock')
    if wb not in ('auto', 'startup_lock'):
        raise RuntimeError('未知白平衡模式: ' + str(wb))
    menus = command(['v4l2-ctl', '-d', device, '--list-ctrls-menus'])
    exposure_menu = re.search(r'^\\s*exposure_auto\\s+0x[^\\n]*\\n((?:[ \\t]+[0-9]+:[^\\n]*\\n?)+)', menus, re.M)
    if not exposure_menu or not re.search(r'^\\s*3:\\s*Aperture Priority Mode\\s*$', exposure_menu[1], re.M):
        raise RuntimeError('相机未报告已支持的自动曝光模式3')
    for name in ('exposure_auto_priority', 'white_balance_temperature_auto'):
        if not re.search(r'^\\s*' + name + r'\\s+0x', menus, re.M):
            raise RuntimeError('相机缺少控制: ' + name)
    for setting in ('exposure_auto_priority=0', 'exposure_auto=3',
                    'white_balance_temperature_auto=1'):
        set_control(device, setting)
    shape = (profile['height'], profile['width'], 3)
    read_for(cap, profile['awb_adjust_seconds'], shape)
    if wb == 'startup_lock':
        set_control(device, 'white_balance_temperature_auto=0')
    frame = read_for(cap, profile['locked_capture_seconds'], shape)
    names = CONTROL_NAMES + ('exposure_auto_priority',)
    raw = command(['v4l2-ctl', '-d', device, '--get-ctrl=' + ','.join(names)])
    actual = {}
    for line in raw.splitlines():
        if ':' in line:
            name, value = line.split(':', 1)
            if name.strip() in names:
                actual[name.strip()] = int(value.strip().split()[0])
    expected = dict(exposure_auto=3, exposure_auto_priority=0,
                    white_balance_temperature_auto=int(wb == 'auto'))
    if any(actual.get(k) != v for k, v in expected.items()):
        raise RuntimeError('自动控制读回不匹配: ' + raw)
    print('AUTO CAMERA: AE=3, variable-FPS=0, WB=' + wb +
          '; convergence/colour/freshness NOT validated', flush=True)
    print(raw, flush=True)
    return frame, raw

'''


def build():
    src = ROOT/'output/hsv-live-trial-20261105/vision_app'
    base = ROOT/'output/auto-camera-trial-20261105'
    dst = base/'vision_app'
    if base.exists():
        raise RuntimeError('Refuse existing trial directory')
    profile = json.loads((src/'code/config/project_profile.json').read_text(encoding='utf-8'))
    assert profile['serial_enabled'] is False and profile['follow_grasp_enabled'] is False
    shutil.copytree(src, dst, ignore=shutil.ignore_patterns('__pycache__', 'diagnostics'))
    p = dst/'code/camera135_startup_check.py'
    source = p.read_text(encoding='utf-8')
    anchor = '    read_for(cap, 0, shape)\n'
    assert source.count(anchor) == 1
    source = source.replace(anchor, anchor + "    if profile.get('exposure_mode', 'manual') == 'auto':\n        return initialize_auto_controls(cap, profile)\n", 1)
    assert source.count('\ndef capture_once(') == 1
    source = source.replace('\ndef capture_once(', AUTO_FUNCTION + '\ndef capture_once(', 1)
    p.write_text(source, encoding='utf-8')
    p = dst/'code/config/generic_135_camera.json'
    config = json.loads(p.read_text(encoding='utf-8'))
    config.update(exposure_mode='auto', white_balance_mode='auto',
                  note='AE3/持续AWB观察候选；手动exposure/gain字段不用于自动模式。未验证收敛、色彩、实际帧率及新鲜度。')
    p.write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding='utf-8')
    changed = sorted(p.relative_to(src).as_posix() for p in src.rglob('*') if p.is_file()
                     and '__pycache__' not in p.parts and p.read_bytes() != (dst/p.relative_to(src)).read_bytes())
    assert changed == ['code/camera135_startup_check.py', 'code/config/generic_135_camera.json']
    out = ROOT/'output/auto-camera-delivery'
    out.mkdir(exist_ok=True)
    archive = out/'p2_auto_camera_20261105.zip'
    hashes = {p.relative_to(base).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in dst.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
        for name in hashes:
            z.write(base/name, name)
        z.writestr('auto-camera-manifest.json', json.dumps(hashes, indent=2))
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
    result = dict(changed_files=changed, camera_opened=False, serial_opened=False,
                  archive=str(archive), sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                  hardware_acceptance='pending', gain_policy='not manually written in auto mode; readback only')
    (out/'delivery.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    build()
