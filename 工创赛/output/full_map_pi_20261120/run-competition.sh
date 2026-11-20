#!/bin/bash
set -eu
cd "$(dirname "$(realpath "$0")")"
. /home/orangepi/.config/full-map-vision/full-map.env
export VISION_APP_CODE
mkdir -p captures /home/orangepi/.cache
exec 9>/home/orangepi/.cache/source-rimpark.lock
flock -n 9
session="captures/run-$(date +%Y%m%d-%H%M%S-%N)"
exec /usr/bin/python3 -u source_fast_cycle_view_20261115.py --full-map --headless \
  --camera-size 1920 1080 \
  --body-profile body-profile.json --material-pickup-config "$VISION_PICKUP" \
  --material-color-config "$VISION_HSV" --storage-config "$VISION_STORAGE_ALIGN" \
  --storage-observation-config "$VISION_STORAGE_OBSERVATION" --session-dir "$session" \
  --continuous-pick --prediction-config prediction-config.json \
  --orbit-config orbit-config.json --rim-detector-config rim-detector.json
