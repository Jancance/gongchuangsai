#!/bin/bash
cd /home/orangepi/p2-source-observer-20261106/vision_app/code || exit 1
nohup timeout 45 python3 main.py --display off --serial-device /dev/ttyUSB0 > /tmp/vis_run2.log 2>&1 &
echo STARTED
