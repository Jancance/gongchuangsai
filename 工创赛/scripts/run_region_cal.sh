#!/bin/bash
cd /home/orangepi/p2-source-observer-20261106/vision_app/code || exit 1
export DISPLAY=:0
pkill -f 物料抓取区域标定 2>/dev/null; sleep 0.5
nohup python3 物料抓取区域标定.py --calibration config/generic_135_unconfigured.yaml --config config/material_pickup_detector.json > /tmp/region_cal.log 2>&1 &
echo LAUNCHED
