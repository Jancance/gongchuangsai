#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo bash scripts/orangepi/bootstrap.sh" >&2
  exit 1
fi

source_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ ! -f "${source_root}/pyproject.toml" ]]; then
  echo "Cannot locate project root" >&2
  exit 1
fi

architecture="$(dpkg --print-architecture)"
if [[ "${architecture}" != "arm64" ]]; then
  echo "This deployment script is for the Orange Pi arm64 system, got ${architecture}" >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
  git cmake build-essential pkg-config python3-venv python3-pip \
  python3-opencv python3-numpy python3-yaml python3-serial \
  v4l-utils ffmpeg gstreamer1.0-tools usbutils stress-ng lm-sensors \
  minicom logrotate rsync

if ! id robot >/dev/null 2>&1; then
  useradd --create-home --shell /bin/bash robot
fi
usermod -aG dialout,video robot

install -d -o robot -g robot /opt/smart-carrier /var/log/smart-carrier
rsync -a --delete \
  --exclude .git --exclude .venv --exclude logs --exclude diagnostics \
  "${source_root}/" /opt/smart-carrier/
python3 -m venv --system-site-packages /opt/smart-carrier/.venv
/opt/smart-carrier/.venv/bin/python -m pip install --no-deps -e /opt/smart-carrier
chown -R robot:robot /opt/smart-carrier /var/log/smart-carrier

install -m 0644 /opt/smart-carrier/systemd/robot-health.service /etc/systemd/system/robot-health.service
install -m 0644 /opt/smart-carrier/systemd/robot-main.service /etc/systemd/system/robot-main.service
install -m 0644 /opt/smart-carrier/systemd/smart-carrier-logrotate /etc/logrotate.d/smart-carrier
systemctl daemon-reload

echo "Installed but NOT enabled. Configure cameras, calibration and UART first."
echo "Then run: sudo systemctl enable --now robot-health.service robot-main.service"

