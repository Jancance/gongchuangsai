# Orange Pi 5 Pro 上板指南

## 1. 镜像与烧录

1. 只从 Orange Pi 5 Pro 官方页面下载 Ubuntu 22.04 Jammy Server 的 Linux 6.1 分支镜像。
2. 保存镜像文件名、发布日期和 SHA256；用 balenaEtcher 写入并完成校验。
3. 使用固定 5V/5A 电源和主动散热。该板不依赖 PD 协商，错误电压可能损坏板卡。
4. 首次启动使用千兆网和 SSH；另备 3.3V USB-TTL，禁止使用 5V TTL。

## 2. 账户加固

首次登录后立即完成：

```bash
sudo adduser robotadmin
sudo usermod -aG sudo robotadmin
mkdir -m 700 ~/.ssh
# 将你自己的公钥加入 ~/.ssh/authorized_keys 后先在另一个终端验证密钥登录
sudoedit /etc/ssh/sshd_config.d/10-smart-carrier.conf
```

配置文件内容：

```text
PermitRootLogin no
PasswordAuthentication no
PubkeyAuthentication yes
```

必须先验证密钥登录成功，再执行 `sudo systemctl restart ssh`，否则可能把自己锁在系统外。设置主机名、`Asia/Shanghai` 时区并确认 `timedatectl` 已同步。

## 3. 安装工程

把本仓库传到板端后：

```bash
sudo bash scripts/orangepi/bootstrap.sh
bash scripts/orangepi/collect_baseline.sh
bash scripts/orangepi/configure_uart6.sh
```

`bootstrap.sh` 不会自动启用机器人服务，也不会改设备树。先完成下面的硬件确认。

## 4. 相机确认

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/
robot-camera-audit --output diagnostics/camera-audit.json
lsusb -t
```

- 主工业相机插 USB3，前视相机插 USB2。
- 把真实 `/dev/v4l/by-id/` 路径填入 `config/cameras.yaml`。
- 依据枚举结果选择格式，不按商品页猜测。优先 MJPEG 降低 USB2 带宽。
- 使用 `v4l2-ctl -d <设备> --set-ctrl=...` 锁定曝光、增益和白平衡；不同相机的控制名不同，以审计结果为准。

## 5. 标定门

默认配置故意使用 `front-unconfigured`，所以真实服务不能启动。完成以下工作后才能替换：

1. 固定最终镜头、焦距、分辨率、安装高度和俯角。
2. 拍摄覆盖画面的棋盘格/ChArUco，计算内参和畸变并记录重投影 RMS。
3. 用至少四个独立地面毫米点计算图像到地面的单应矩阵。
4. 另外用未参与求解的点验证毫米误差和 X/Y 方向。
5. 同时更新 `config/cameras.yaml` 与 `config/vision.yaml` 的相同 `calibration_id`。

可复制 `config/calibration-points.example.yaml` 并填入实测点，然后运行：

```bash
python3 scripts/calibrate_ground.py my-points.yaml --max-rms-mm 3
```

工具只输出候选矩阵，不会自动覆盖正式配置；必须通过独立验证点后手工采用。

## 6. UART

在 `orangepi-config -> System -> Hardware` 启用 `uart6-m1`，重启后确认节点。40Pin 16 为 Orange Pi TX，18 为 RX；与 STM32 交叉连接并共地。第一次只做 TX-RX 短接：

```bash
python3 scripts/uart_loopback_stress.py --device /dev/ttyS6 --frames 100000
```

实际设备节点可能随镜像不同；必须根据设备树和 `/dev/ttyS*` 核验后修改配置，不能机械照抄。

## 7. 启用服务

```bash
robot-health --config config
sudo systemctl enable --now robot-health.service
sudo systemctl enable --now robot-main.service
systemctl status robot-health.service robot-main.service
journalctl -u robot-main.service -f
```

硬件看门狗默认关闭。先完成进程崩溃、相机断开和 STM32 断线测试，再由负责人启用；否则配置错误会造成反复重启。

## 8. 版本冻结

平台验收通过后保存：基线目录、`dpkg-query -W`、启动分区 SHA256、配置、标定、RKNN版本矩阵和一份完整 TF 镜像。比赛前一周禁止升级内核、U-Boot、设备树、OpenCV 和 RKNN。
