#!/bin/bash
cd /home/orangepi/p2-source-observer-20261106/vision_app/code || exit 1
pkill -f "main.py --display off" 2>/dev/null; sleep 0.3
nohup timeout 120 python3 main.py --display off --serial-device /dev/ttyUSB0 > /tmp/vis_run4.log 2>&1 &
echo STARTED
