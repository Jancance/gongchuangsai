#!/bin/bash
set -eu
cd /home/orangepi/vision_tools/stage23_desktop_20261116
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-/home/orangepi/.Xauthority}
export QT_QPA_PLATFORM=xcb
exec 9>/home/orangepi/.cache/source-rimpark.lock
flock -n 9 || exit 0
session="captures/run-$(date +%Y%m%d-%H%M%S-%N)"
mkdir -p captures
exec /usr/bin/python3 -u source_fast_cycle_view_20261115.py \
 --body-profile body-profile.json \
 --material-pickup-config /home/orangepi/vision_tools/material_pickup_top65_20261114.json \
 --session-dir "$session" --continuous-pick --prediction-config prediction-config.json \
 --orbit-config orbit-config.json --rim-detector-config rim-detector.json --preview-only "$@" >> "$session.log" 2>&1
