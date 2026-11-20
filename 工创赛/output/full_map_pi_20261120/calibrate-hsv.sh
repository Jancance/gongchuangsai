#!/bin/bash
set -eu
. /home/orangepi/.config/full-map-vision/full-map.env
service=full-map-vision.service
sudo systemctl stop "$service"
trap 'sudo systemctl start "$service"' EXIT
(
  mkdir -p /home/orangepi/.cache
  exec 9>/home/orangepi/.cache/source-rimpark.lock
  flock -n 9
  /usr/bin/python3 "$VISION_APP_CODE/颜色标定.py" --config "$VISION_HSV"
)
