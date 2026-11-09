"""完整保留lightblue1，从已验证源码建立普通主程序像素展示候选。"""
from pathlib import Path
import hashlib
import json
import struct
import zlib
from zipfile import ZipFile, ZipInfo, ZIP_DEFLATED

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'output/p2_vision_generic135_lightblue1_20261027/vision_app'
APP = ROOT / 'output/p2_vision_generic135_pixel1_20261027/vision_app'


def build():
    if APP.exists() and not (APP/'PIXEL1_MANIFEST.json').exists():
        raise RuntimeError('拒绝覆盖非本构建器生成的目录')
    original = {p.relative_to(BASE).as_posix(): p.read_bytes() for p in BASE.rglob('*')
                if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'}
    files = dict(original)
    source = original['code/main.py'].decode('utf8').replace('\r\n', '\n')
    def replace(old, new):
        nonlocal source
        assert source.count(old) == 1, old[:100]
        source = source.replace(old, new)
    replace('from project_geometry import metric_verified, draw_pixels',
            'from project_geometry import metric_verified, draw_pixels\nfrom project_ring_pixel import RingPixelPresenter, draw_ring_pixels, format_ring_status, parse_paper_labels')
    replace('ring_station_detector = None', 'ring_station_detector = None\nring_pixel_presenter = None')
    replace('    turntable_detector.reset()\n',
            '    if ring_pixel_presenter is not None:\n        ring_pixel_presenter.reset()\n    turntable_detector.reset()\n')
    old = '''    else:
        draw_pixels(frame,result,'ring')
    send_ring_if_ready(result)'''
    new = '''    else:
        ring_pixel_presenter.observe(result, frame.shape,
            getattr(camera, 'last_read_completed_time', None), time.monotonic())
        draw_ring_pixels(frame, current_ring_pixel_status())
    send_ring_if_ready(result)'''
    replace(old, new)
    replace('def process_ring_station(frame):', '''def current_ring_pixel_status():
    """展示时再次检查时效；失效样本不能留在原检测器的稳定窗口。"""
    status = ring_pixel_presenter.current(time.monotonic())
    if status['state'] == 'STALE':
        ring_station_detector.reset()
    return status


def process_ring_station(frame):''')
    replace('    return parser.parse_args()', '''    parser.add_argument('--start-ring', action='store_true', help='实时启动直接进入R三圆环模式，默认仍为FOLLOW')
    parser.add_argument('--ring-paper-labels', type=parse_paper_labels, help='按检测器输出顺序显式指定当前人工标签，例如3,2,1；不是OCR')
    return parser.parse_args()''')
    replace('    if current_mode == Mode.FOLLOW:\n        print(', '''    if current_mode == Mode.RING_STATION and not metric_verified('ring'):
        print(format_ring_status(current_ring_pixel_status(), measured_fps))
        return
    if current_mode == Mode.FOLLOW:
        print(''')
    replace('    """断线期间继续显示待请求状态和任务码统计。"""\n    print(',
            '''    """断线期间继续显示待请求状态和任务码统计。"""
    if current_mode == Mode.RING_STATION and not metric_verified('ring'):
        print(format_ring_status(current_ring_pixel_status(), 0.))
    print(''')
    replace('    global current_mode, camera_next_retry, camera_last_error, window_name\n',
            '    global current_mode, camera_next_retry, camera_last_error, window_name, ring_pixel_presenter\n')
    replace('    ring_station_detector = RingStationDetector(ring_station_config)',
            '''    ring_station_detector = RingStationDetector(ring_station_config)
    ring_pixel_presenter = RingPixelPresenter(ring_station_config.minimum_confidence,
        paper_labels=args.ring_paper_labels, ring1_side=ring_station_config.ring1_side)''')
    replace('    current_mode = Mode.FOLLOW\n    reset_detection_history()',
            '    current_mode = Mode.RING_STATION if args.start_ring else Mode.FOLLOW\n    reset_detection_history()')
    replace("            if args.mode in ('ring','stack'):data['stable']=False",
            """            if args.mode in ('ring','stack'):data['stable']=False
            if args.mode == 'ring':
                data['pixel_status'] = current_ring_pixel_status()
                data['pixel_status']['scope'] = 'OFFLINE_NO_LIVE_CAMERA_TIMESTAMP'""")
    replace('"motion_allowed":False},ensure_ascii=False))',
            '"motion_allowed":False,"ring_pixel_status_version":1,"ring_paper_labels":list(args.ring_paper_labels) if args.ring_paper_labels else None,"start_mode":current_mode.value},ensure_ascii=False))')
    files['code/main.py'] = source.encode('utf8')
    files['code/project_ring_pixel.py'] = (ROOT/'scripts/templates/project_ring_pixel.py').read_bytes()
    files['PIXEL1_DEPLOY.md'] = (ROOT/'docs/P2_GENERIC135_PIXEL1_2026-10-27.md').read_bytes()
    compile(source, 'main.py', 'exec')
    compile(files['code/project_ring_pixel.py'], 'project_ring_pixel.py', 'exec')
    changed = [name for name in original if files[name] != original[name]]
    assert changed == ['code/main.py']
    profile = json.loads(files['code/config/project_profile.json'])
    assert profile['serial_enabled'] is False and profile['geometry_verified'] is False
    sha = lambda data: hashlib.sha256(data).hexdigest()
    manifest = dict(base=str(BASE.relative_to(ROOT)),
                    source_files={n: sha(d) for n,d in sorted(original.items())},
                    changed_files=changed, added_code_files=['code/project_ring_pixel.py'],
                    added_files=['code/project_ring_pixel.py','PIXEL1_DEPLOY.md'],
                    candidate_files={n:sha(files[n]) for n in changed+['code/project_ring_pixel.py','PIXEL1_DEPLOY.md']},
                    motion_allowed=False, hardware_acceptance=False,
                    manual_labels_are_not_ocr=True,
                    preserved='All original application files; only main.py changed, new display module added. No config/camera/detector/serial edits.')
    files['PIXEL1_MANIFEST.json'] = json.dumps(manifest, ensure_ascii=False, indent=2).encode('utf8')
    APP.mkdir(parents=True, exist_ok=True)
    for name, data in files.items():
        p = APP/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
    archive = APP.parent.with_suffix('.zip')
    with ZipFile(archive, 'w', ZIP_DEFLATED) as z:
        for name, data in sorted(files.items()):
            full_name='vision_app/'+name
            info=ZipInfo(full_name)
            if not full_name.isascii():
                encoded=full_name.encode('utf8')
                payload=b'\x01'+struct.pack('<I',zlib.crc32(encoded)&0xffffffff)+encoded
                info.extra=struct.pack('<HH',0x7075,len(payload))+payload
            z.writestr(info,data,compress_type=ZIP_DEFLATED)
    with ZipFile(archive) as z:
        assert z.testzip() is None
        for name, data in files.items():assert z.read('vision_app/'+name) == data == (APP/name).read_bytes()
    print(json.dumps(dict(app=str(APP), archive=str(archive), changed_files=changed,
                         original_files=len(original),sha256=sha(archive.read_bytes())), ensure_ascii=False))


if __name__ == '__main__':
    build()
