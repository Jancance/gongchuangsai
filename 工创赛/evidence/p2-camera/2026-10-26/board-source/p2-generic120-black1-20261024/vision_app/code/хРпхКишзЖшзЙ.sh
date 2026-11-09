#!/usr/bin/env bash
# 桌面快捷方式和登录自启动共用入口，不修改视觉模式或串口行为。
set -u

vision_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || exit 1
vision_python=/usr/bin/python3
vision_lock_dir="${XDG_CACHE_HOME:-$HOME/.cache}/cyqm-vision"

already_running() {
    printf '%s\n' '视觉程序已经在运行，不重复启动。'
    if command -v notify-send >/dev/null 2>&1; then
        notify-send --app-name='视觉程序' '视觉程序已经运行' '请切换到已有的摄像头窗口。' || true
    fi
}

mkdir -p -- "$vision_lock_dir" || exit 1
exec 9>"$vision_lock_dir/main.lock" || exit 1
if ! flock -n 9; then
    already_running
    exit 0
fi

# IDE直接运行main.py时不会持有上面的锁，因此额外按实际脚本路径检查。
"$vision_python" - "$vision_dir/main.py" <<'PY'
import os
from pathlib import Path
import sys

target = Path(sys.argv[1]).resolve()
for process in Path('/proc').iterdir():
    if not process.name.isdigit() or int(process.name) == os.getpid():
        continue
    try:
        arguments = (process / 'cmdline').read_bytes().split(b'\0')
        if not Path(os.fsdecode(arguments[0])).name.startswith('python'):
            continue
        # 只检查Python脚本参数，不把-c内容或其他程序的参数误认为运行实例。
        for argument in arguments[1:]:
            text = os.fsdecode(argument)
            if not text:
                continue
            if text in ('-c', '-m', '-'):
                break
            if text.startswith('-'):
                continue
            candidate = Path(text)
            if not candidate.is_absolute():
                candidate = (process / 'cwd').resolve() / candidate
            if candidate.resolve() == target:
                raise SystemExit(0)
            break
    except (OSError, ValueError):
        continue
raise SystemExit(1)
PY
vision_check=$?
if [ "$vision_check" -eq 0 ]; then
    already_running
    exit 0
elif [ "$vision_check" -ne 1 ]; then
    printf '%s\n' '检查已有视觉进程失败，未启动新实例。'
    exit "$vision_check"
fi

cd -- "$vision_dir/.." || exit 1
printf '%s\n' '正在启动视觉程序；在摄像头窗口按 Q 或 Esc 退出。'
"$vision_python" -u "$vision_dir/main.py" "$@"
vision_result=$?
if [ "$vision_result" -ne 0 ] && [ -t 0 ]; then
    printf '\n程序异常退出，退出码：%s。请查看上方错误信息。\n' "$vision_result"
    read -r -p '按回车关闭此窗口……' _vision_reply || true
fi
exit "$vision_result"
