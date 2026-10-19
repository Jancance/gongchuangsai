#!/usr/bin/env bash
set -Eeuo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${project_root}"

mkdir -p diagnostics
robot-health --config config | tee diagnostics/health.json
robot-camera-audit --output diagnostics/camera-audit.json
python3 scripts/camera_stress.py --config config/cameras.yaml --duration-hours "${CAMERA_HOURS:-8}"
python3 scripts/uart_loopback_stress.py --device "${UART_DEVICE:-/dev/ttyS6}" --frames "${UART_FRAMES:-100000}"
stress-ng --cpu 8 --vm 2 --vm-bytes 60% --timeout "${STRESS_DURATION:-6h}" --metrics-brief |& tee diagnostics/stress-ng.txt
sensors | tee diagnostics/temperature-after-stress.txt

echo "Automated checks passed. Complete the manual cold-boot and fault-injection checklist in docs/ACCEPTANCE.md."

