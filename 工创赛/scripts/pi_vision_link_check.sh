#!/bin/bash
# 香橙派侧：CH340 -> STM32 USART6(PC6/PC7) 视觉链路 一键检查 + 修复
# 用法：拷到香橙派后   bash pi_vision_link_check.sh
set -u

echo "===== 1. USB 枚举 ====="
if lsusb | grep -qi 1a86; then
  lsusb | grep -i 1a86
else
  echo "[X] 没有 1a86:7523 —— CH340 没插好，或这根是纯充电线"
fi

echo "===== 2. 设备节点 ====="
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "[X] 无 /dev/ttyUSB* —— 驱动没绑"

echo "===== 3. 修复驱动 ====="
sudo modprobe ch341 2>/dev/null || sudo modprobe usbserial 2>/dev/null
sudo systemctl stop brltty 2>/dev/null
sudo systemctl disable brltty 2>/dev/null
sleep 1
ls -l /dev/ttyUSB* 2>/dev/null || { echo "[X] modprobe 后仍无节点，看 dmesg:"; sudo dmesg | tail -12; }

echo "===== 4. 固定设备名 /dev/ttyVISION ====="
printf '%s\n' 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", SYMLINK+="ttyVISION"' | sudo tee /etc/udev/rules.d/99-vision-ch340.rules >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger
sleep 1
ls -l /dev/ttyVISION 2>/dev/null || echo "[!] 无 /dev/ttyVISION，下面退回用 /dev/ttyUSB0"

echo "===== 5. 被动监听 5 秒（等主控发 AA xx BB）====="
python3 - <<'PY'
import os, sys, time
try:
    import serial
except ImportError:
    print("[X] 缺 pyserial: sudo apt install -y python3-pip ; pip3 install pyserial")
    sys.exit(2)
dev = None
for cand in ("/dev/ttyVISION", "/dev/ttyUSB0", "/dev/ttyACM0"):
    if os.path.exists(cand):
        dev = cand
        break
if not dev:
    print("[X] 没有任何串口节点")
    sys.exit(2)
s = serial.Serial(dev, 115200, timeout=0.2)
s.reset_input_buffer()
print("监听", dev, "115200 ...")
t0 = time.time()
buf = b""
while time.time() - t0 < 5:
    buf += s.read(256)
s.close()
print("收到 %d 字节" % len(buf))
if len(buf) == 0:
    print("[!] 完全安静 —— 若主控此刻没在请求就正常；至少说明没接错到调试口")
elif b"TMQ," in buf or b"PM1," in buf or b"LR1," in buf or b"OD1," in buf:
    print("[X] 收到 CSV 遥测 = 接在调试口 USART1(PA9/PA10) 上，数据线要挪到 PC6/PC7")
elif b"\xaa" in buf and b"\xbb" in buf:
    print("[OK] 收到主控帧:", buf[:24].hex(" "), "—— 引脚和波特率都对")
else:
    print("[?] 未知数据:", buf[:60].hex(" "))
PY

echo "===== 6. 视觉程序应使用的串口参数 ====="
echo "python3 main.py --serial-device /dev/ttyVISION --serial-baud 115200"
