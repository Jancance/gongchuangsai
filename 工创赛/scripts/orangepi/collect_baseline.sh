#!/usr/bin/env bash
set -Eeuo pipefail

output_dir="${1:-./diagnostics/baseline-$(date +%Y%m%d-%H%M%S)}"
mkdir -p "${output_dir}"

capture() {
  local name="$1"
  shift
  { echo "COMMAND: $*"; "$@"; } >"${output_dir}/${name}.txt" 2>&1 || true
}

capture os-release cat /etc/os-release
capture uname uname -a
capture cpu lscpu
capture memory free -h
capture block lsblk -o NAME,SIZE,TYPE,FSTYPE,MOUNTPOINTS,MODEL
capture disk df -hT
capture network ip address show
capture usb lsusb -tv
capture v4l2 v4l2-ctl --list-devices
capture temperature sensors
capture packages dpkg-query -W
capture kernel-modules lsmod
capture boot-log journalctl -b --no-pager
capture dmesg dmesg -T

if [[ -r /sys/kernel/debug/rknpu/version ]]; then
  capture rknpu cat /sys/kernel/debug/rknpu/version
fi
sha256sum /boot/* >"${output_dir}/boot-sha256.txt" 2>/dev/null || true
echo "Baseline written to ${output_dir}"

