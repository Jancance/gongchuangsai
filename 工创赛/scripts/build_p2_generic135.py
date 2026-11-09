"""从本轮取回的现场perf2快照生成135°隔离候选，不修改源目录。"""
from pathlib import Path
import hashlib
import json
import shutil
from zipfile import ZipFile, ZIP_DEFLATED

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'evidence/p2-camera/2026-10-26/board-source/p2-generic120-perf2-20261024/vision_app'
APP = ROOT / 'output/p2_vision_generic135_20261026/vision_app'
PROFILE_ID = 'generic_135_1920x1080_20261026_v1'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def replace(path, old, new):
    value = path.read_text(encoding='utf-8')
    if old not in value:
        raise RuntimeError('源内容不匹配: ' + str(path) + ': ' + old[:60])
    path.write_text(value.replace(old, new), encoding='utf-8')


def build():
    review = json.loads((SOURCE.parents[1] / 'source_review.json').read_text(encoding='utf-8'))
    files = review['versions']['perf2']['files']
    for name, expected in files.items():
        if digest(SOURCE / name) != expected:
            raise RuntimeError('现场源哈希不符: ' + name)
    if APP.exists():
        raise RuntimeError('候选目录已存在，拒绝覆盖: ' + str(APP))
    shutil.copytree(SOURCE, APP, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    code = APP / 'code'
    cfg = code / 'config'
    # 旧相机运行配置完整保留，但不作为135°的标定来源。
    shutil.copytree(cfg, APP / 'reference_config/board_generic120_20261026')
    profile = json.loads((cfg / 'generic_120_camera.json').read_text(encoding='utf-8'))
    profile.update(profile_id=PROFILE_ID, gain=64, awb_adjust_seconds=7,
                   lens_nominal_degrees=135, lens_fov_axis='unverified',
                   identity_distinguishes_lens=False,
                   note='135°为用户镜头标称；USB身份与旧120°相同。固定等待不是AWB收敛检测；本轮仅静态启动及10秒主机读取检查。')
    write_json(cfg / 'generic_135_camera.json', profile)
    (cfg / 'generic_135_unconfigured.yaml').write_text(
        '%YAML:1.0\n---\ncamera_name: generic135_unconfigured\n'
        f'camera_profile_id: {PROFILE_ID}\nimage_width: 1920\nimage_height: 1080\ncalibrated: 0\n', encoding='utf-8')
    startup = code / 'camera135_startup_check.py'
    startup.write_text((code / 'camera120_startup_check.py').read_text(encoding='utf-8')
                       .replace('generic_120_camera.json', 'generic_135_camera.json')
                       .replace('120°', '135°').replace('camera120_startup_', 'camera135_startup_'), encoding='utf-8')
    for name in ('project_camera.py', 'main.py', '颜色标定.py', '转盘参数设置.py', '物料抓取区域标定.py'):
        path = code / name
        value = path.read_text(encoding='utf-8')
        value = value.replace('generic_120_camera.json', 'generic_135_camera.json')
        value = value.replace('generic_120_unconfigured.yaml', 'generic_135_unconfigured.yaml')
        value = value.replace('from camera120_startup_check import', 'from camera135_startup_check import')
        value = value.replace('Generic120', 'Generic135').replace('120度', '135度')
        path.write_text(value, encoding='utf-8')

    # 分辨率相同也不能接受另一镜头的内参；主程序和四工具共用此校验。
    guard = code / 'camera135_calibration.py'
    guard.write_text('''"""135°候选的标定归属校验，不将同USB编号视为同镜头。"""
EXPECTED_PROFILE_ID = "generic_135_1920x1080_20261026_v1"

def validate_calibration_storage(storage):
    if storage.getNode('camera_profile_id').string() != EXPECTED_PROFILE_ID:
        raise RuntimeError('标定不属于当前135°profile，禁止沿用120°/DECXIN/原作者内参')
    flag = storage.getNode('calibrated')
    if flag.empty() or flag.real() != 1:
        raise RuntimeError('135°相机尚未完成内参标定')
    size = (int(storage.getNode('image_width').real()), int(storage.getNode('image_height').real()))
    if size != (1920, 1080):
        raise RuntimeError('135°标定分辨率不一致')
''', encoding='utf-8')
    replace(code / 'project_camera.py', 'import json\n', 'import json\nfrom camera135_calibration import validate_calibration_storage\n')
    replace(code / 'project_camera.py', '    try:\n        if not fs.isOpened()',
            '    try:\n        validate_calibration_storage(fs)\n        if not fs.isOpened()')
    replace(code / 'main.py', 'def load_calibration(path):',
            'from camera135_calibration import validate_calibration_storage\n\ndef load_calibration(path):')
    replace(code / 'main.py', '    try:\n        width = int(storage.getNode("image_width").real())',
            '    try:\n        validate_calibration_storage(storage)\n        width = int(storage.getNode("image_width").real())')
    replace(code / 'main.py', '"metric_geometry_verified":False},ensure_ascii=False)',
            '"metric_geometry_verified":False,"camera_profile":"' + PROFILE_ID + '","motion_allowed":False},ensure_ascii=False)')
    write_json(cfg / 'camera_processing.json', {'undistort_enabled': False, 'reason': '135°新机独立标定未完成，不加载旧内参'})
    project = json.loads((cfg / 'project_profile.json').read_text(encoding='utf-8'))
    project.update(camera_model='Generic_USB_CAMERA_user135deg', camera_profile=PROFILE_ID,
                   camera_verified=False, geometry_verified=False, ring_geometry_verified=False,
                   stack_geometry_verified=False, serial_enabled=False, follow_grasp_enabled=False,
                   raw_turntable_height_mm=None, height_version=None,
                   note='本轮静态采集通过不等于完整相机验收；工作面高度及相机/夹爪映射待实测。')
    write_json(cfg / 'project_profile.json', project)
    # 检测参数保留为待核的像素预览起点；旧坐标不能作为有效ROI或毫米结果。
    for name in ('turntable_detector.json', 'turntable_material_seed.json',
                 'ring_station_detector.json', 'stacked_ring_detector.json', 'material_pickup_detector.json'):
        value = json.loads((cfg / name).read_text(encoding='utf-8'))
        value.update(camera_profile_id=PROFILE_ID, parameters_verified_for_this_camera=False,
                     placeholder_note='继承算法结构和候选搜索参数供像素预览；不是135°实测ROI、尺寸或毫米标定。')
        for key in ('calibration_verified', 'calibrated'):
            if key in value:
                value[key] = False
        if name == 'material_pickup_detector.json':
            value['regions'] = {key: None for key in value['regions']}
        if name == 'ring_station_detector.json':
            value['profile_name'] = 'generic135_unconfigured_station'
        write_json(cfg / name, value)
    guide = ROOT / 'docs/P2_GENERIC135_DEPLOY_2026-10-26.md'
    shutil.copy2(guide, APP / 'DEPLOY_135_CAMERA.md')
    readme = APP / 'README.md'
    readme.write_text('> 本目录是135°隔离候选，先读DEPLOY_135_CAMERA.md。以下120°/DECXIN说明仅为历史来源。\n\n' + readme.read_text(encoding='utf-8'), encoding='utf-8')
    changed = [name for name in files if digest(APP / name) != files[name]]
    added = [p.relative_to(APP).as_posix() for p in APP.rglob('*') if p.is_file() and p.relative_to(APP).as_posix() not in files]
    write_json(APP / 'GENERIC135_MANIFEST.json', {
        'source_archive_sha256': review['archive_sha256'], 'source': 'board perf2 snapshot 2026-10-26',
        'source_files': files, 'changed_source_files': changed, 'added_files': added,
        'profile_id': PROFILE_ID, 'motion_allowed': False,
        'note': '旧manifest为来源记录；以本manifest和实际新文件为准。配置检查不签署硬件通过。'})
    archive = APP.parent.with_suffix('.zip')
    with ZipFile(archive, 'w', ZIP_DEFLATED) as output:
        for path in sorted(APP.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix != '.pyc':
                output.write(path, path.relative_to(APP.parent).as_posix())
    print(json.dumps({'app': str(APP), 'zip': str(archive), 'sha256': digest(archive),
                      'changed': changed}, ensure_ascii=False))


if __name__ == '__main__':
    build()
