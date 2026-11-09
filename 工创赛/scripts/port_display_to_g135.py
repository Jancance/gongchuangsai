# -*- coding: utf-8 -*-
"""Port the --display/--display-every preview switch onto the Generic135 main.py."""
import sys, py_compile
from pathlib import Path

src = Path('tmp/pi_sync/main_pi_20261106.py').read_text(encoding='utf-8')
out = src
tags = []

def rep(old, new, tag):
    global out
    n = out.count(old)
    if n != 1:
        print('FAIL %-12s occurrences=%d' % (tag, n))
        sys.exit(1)
    out = out.replace(old, new)
    tags.append(tag)

rep('# 相机与显示：断线时camera=None，达到重试时间后再打开。\nsettings = None\n',
    '# 相机与显示：断线时camera=None，达到重试时间后再打开。\nsettings = None\n'
    '# 预览开关：window = 本机窗口（VNC 里能看到画面）；off = 不建窗口、不绘制、不传图。\n'
    '# 两种模式走同一套检测流程，只差显示通路，关掉只提帧率、不改识别结果。\n'
    'display_mode = "window"\n'
    'display_every = 1\n'
    'display_counter = 0\n', 'globals')

rep('    parser.add_argument("--fps", type=float, default=30.0)\n',
    '    parser.add_argument("--fps", type=float, default=30.0)\n'
    '    parser.add_argument(\n'
    '        "--display",\n'
    '        choices=("window", "off"),\n'
    '        default="window",\n'
    '        help="window=窗口预览（VNC 可见）；off=无窗口无绘制（比赛/无人值守用）",\n'
    '    )\n'
    '    parser.add_argument(\n'
    '        "--display-every",\n'
    '        type=int,\n'
    '        default=1,\n'
    '        help="窗口每 N 帧刷新一次；>1 可大幅降低 VNC 带宽占用，只影响显示",\n'
    '    )\n', 'args')

rep('    global current_mode, camera_next_retry, camera_last_error, window_name\n',
    '    global current_mode, camera_next_retry, camera_last_error, window_name, display_mode, display_every\n',
    'globaldecl')

rep('    window_name = "Undistorted Camera" if undistort_enabled else "Raw Camera"\n',
    '    window_name = "Undistorted Camera" if undistort_enabled else "Raw Camera"\n'
    '    display_mode = getattr(args, "display", "window")\n'
    '    display_every = max(1, int(getattr(args, "display_every", 1) or 1))\n'
    '    print("Preview: %s (every %d frame(s))" % (display_mode, display_every))\n', 'init-set')

rep('def run(args):\n',
    'def show_image(name, image, force=False):\n'
    '    """按开关决定是否真的 imshow；off 模式下完全不碰 GUI。"""\n'
    '    global display_counter\n'
    '    if display_mode != "window":\n'
    '        return\n'
    '    if not force:\n'
    '        display_counter += 1\n'
    '        if display_every > 1 and (display_counter % display_every):\n'
    '            return\n'
    '    cv2.imshow(name, image)\n'
    '\n'
    '\n'
    'def pump_ui(delay_ms):\n'
    '    """窗口模式用 waitKey 抽事件；off 模式退化成 sleep，保持同样的循环节拍。"""\n'
    '    if display_mode == "window":\n'
    '        return cv2.waitKey(delay_ms) & 0xFF\n'
    '    if delay_ms > 0:\n'
    '        time.sleep(delay_ms / 1000.0)\n'
    '    return 0xFF\n'
    '\n'
    '\n'
    'def run(args):\n', 'helpers')

rep('''        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, args.width, args.height)
        cv2.namedWindow(task_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(task_window_name, 760, 320)
        displayed_task_code = serial_link.last_task_code
        cv2.imshow(task_window_name, draw_task_code_window(displayed_task_code))
        cv2.setMouseCallback(window_name, handle_mouse)
''',
    '''        if display_mode == "window":
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, args.width, args.height)
            cv2.namedWindow(task_window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(task_window_name, 760, 320)
        displayed_task_code = serial_link.last_task_code
        show_image(task_window_name, draw_task_code_window(displayed_task_code), force=True)
        if display_mode == "window":
            cv2.setMouseCallback(window_name, handle_mouse)
''', 'namedwindow')

rep('                cv2.imshow(task_window_name, draw_task_code_window(displayed_task_code))\n',
    '                show_image(task_window_name, draw_task_code_window(displayed_task_code))\n', 'taskcode-loop')

rep('''                waiting = np.zeros((args.height, args.width, 3), dtype=np.uint8)
                draw_status(waiting, current_mode, measured_fps, frame_count)
                draw_camera_waiting(waiting, f"Device: {args.device} | retry every {args.camera_reconnect_interval:.1f}s")
                if current_mode == Mode.MEASUREMENT:
                    draw_measurement(waiting)
                cv2.imshow(window_name, waiting)
''',
    '''                if display_mode == "window":
                    waiting = np.zeros((args.height, args.width, 3), dtype=np.uint8)
                    draw_status(waiting, current_mode, measured_fps, frame_count)
                    draw_camera_waiting(waiting, f"Device: {args.device} | retry every {args.camera_reconnect_interval:.1f}s")
                    if current_mode == Mode.MEASUREMENT:
                        draw_measurement(waiting)
                    cv2.imshow(window_name, waiting)
''', 'waiting')

rep('                if not handle_key(cv2.waitKey(50) & 0xFF):\n',
    '                if not handle_key(pump_ui(50)):\n', 'waitkey50')

rep('''            draw_status(frame, current_mode, measured_fps, frame_count)
            if current_mode == Mode.MEASUREMENT:
                draw_measurement(frame)
            cv2.imshow(window_name, frame)
''',
    '''            if display_mode == "window":
                draw_status(frame, current_mode, measured_fps, frame_count)
                if current_mode == Mode.MEASUREMENT:
                    draw_measurement(frame)
                cv2.imshow(window_name, frame)
''', 'frame-show')

rep('            if not handle_key(cv2.waitKey(1) & 0xFF):\n',
    '            if not handle_key(pump_ui(1)):\n', 'waitkey1')

rep('''        serial_link.close()
        close_camera()
        cv2.destroyAllWindows()
''',
    '''        serial_link.close()
        close_camera()
        if display_mode == "window":
            cv2.destroyAllWindows()
''', 'destroy')

dst = Path('tmp/pi_sync/main_g135_display.py')
dst.write_text(out, encoding='utf-8')
py_compile.compile(str(dst), doraise=True)
print('OK edits=%d  lines %d -> %d' % (len(tags), src.count(chr(10)) + 1, out.count(chr(10)) + 1))
print('tags:', ','.join(tags))

