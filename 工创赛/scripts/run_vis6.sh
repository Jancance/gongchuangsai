#!/bin/bash
cd /home/orangepi/p2-source-observer-20261106/vision_app/code || exit 1
pkill -f "main.py --display off" 2>/dev/null; sleep 0.5
nohup timeout 90 python3 -u main.py --display off --serial-device /dev/ttyUSB0 > /tmp/vis_run7.log 2>&1 &
echo STARTED
