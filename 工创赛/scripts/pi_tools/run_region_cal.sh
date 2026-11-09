#!/bin/bash
cd /home/orangepi/p2-source-observer-20261106/vision_app/code || exit 1
echo "=== 物料抓取区域标定 ==="
echo "拖动顺序: 物料活动区 -> 左 -> 右 -> 下 ; 按 S 保存, U 撤销, R 重来, Q 退出"
echo
python3 物料抓取区域标定.py --calibration config/generic_135_unconfigured.yaml --config config/material_pickup_detector.json
echo
echo "程序已退出，按回车关闭窗口"
read
