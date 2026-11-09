#!/usr/bin/env bash
# 本项目安全桌面入口；原完整防重复启动脚本也保留于code/启动视觉.sh。
set -euo pipefail
app_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$app_dir/run.py" "$@"
