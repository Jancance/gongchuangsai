#!/bin/bash
cd /home/orangepi/p2-source-observer-20261106/vision_app/code || exit 1
echo "=== 颜色阈值标定 (六色 HSV) ==="
echo "数字键 1~6 选颜色，然后在物料上拖框采样；滑条可微调；按 S 保存, Q 退出"
echo
python3 颜色标定.py --calibration config/generic_135_unconfigured.yaml --config config/material_color_thresholds.json
echo
echo "程序已退出，按回车关闭窗口"
read
