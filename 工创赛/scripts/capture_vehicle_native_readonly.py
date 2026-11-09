"""使用既有Win32后端只读采集；无启动/停止/KEEP或其他串口写入。"""
import argparse
import json
import time
from pathlib import Path

from p1_serial_motion import WindowsSerial
from vehicle_uart_capture import Capture, source_manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--stage', type=int, choices=range(0, 9), default=1)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    assert 0 < args.seconds <= 60
    capture = Capture(args.out, args.stage)
    stream = None
    reason = 'finished'
    try:
        manifest = dict(port=args.port, baud=115200, stage=args.stage, read_only=True,
                        native_win32_backend=True, **source_manifest())
        (args.out / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
        stream = WindowsSerial(args.port)
        started = time.monotonic()
        while time.monotonic() - started < args.seconds:
            capture.feed(stream.read())
    except Exception as exc:
        reason = f'{type(exc).__name__}:{exc}'
        raise
    finally:
        if stream is not None:
            stream.close()
        print(json.dumps(dict(output=str(args.out), **capture.close(reason)), ensure_ascii=False))


if __name__ == '__main__':
    main()
