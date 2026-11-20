# 整图竞赛视觉

本机候选继承队友提交 `f0393093` 的 `stage23_desktop_current_20261120`，正式入口是 `run-competition.sh`。原压缩包的说明和脚本是上游历史。正式入口在本地引导后由 systemd 启动，现场调色使用原有 `颜色标定.py` 和同一份 HSV 文件。

## 串口

数据字段为十进制整数。CRC-16/CCITT-FALSE 初值 FFFF，覆盖末尾逗号，四位大写十六进制，帧尾 CR LF。

```text
VS1,run,request,batch,mode,pose,ring,color,CRC
VO1,run,request,batch,mode,pose,seq,ring,color,valid,occupied,stable,age_ms,CRC
```

模式 0 待机或行驶，1 原料，2 粗加工，3 暂存平放，4 暂存码垛。每次 VS1 后紧跟同 `run` 和 `pose` 的 SC2（原料）或 S31（其他工位）。未配对、超出 200 ms、行驶、切帧或换相机进程时停用观测。原料输出 OC1/OC2，工位对位在 S31 state 2 输出 O31。

放稳观察要求执行释放、收回挡住目标的部件并停稳后，把 VS1 从 `ring=0,color=0` 切为实际目标并配对 S31 state 5。下一件重新建立观测围栏。VO1 需要现地标定、三帧独立稳定观测与不超过 200 ms 的主机保守帧龄。码垛须在目前相机视角验证取样区域可区分底层和顶层。收到动作完成报文不会产生占位确认。主机帧围栏以连续相机读取完成时间为依据，真实曝光时效须在板上验收。

## 安装和调色

1. 安装日前先核对旧自启与现场程序，备份旧入口和当前 HSV。Pi 需保留现有 1920×1080 源区相机运行时、原物料抓取配置、本地 Python3、OpenCV、NumPy、PySerial、Pillow 和已有设备权限。
2. 在 Pi 使用 `install-local.py --destination /home/orangepi/vision_tools/full_map_pi_20261120` 安装包。将 `full-map.env`、`storage-align.json` 和 `storage-observation.json` 的初始副本放入 `/home/orangepi/.config/full-map-vision/`。已有设备配置保留。核对 `VISION_APP_CODE` 与 `VISION_HSV` 对应现场正在使用的文件。
3. 停用旧自启并确认原相机和串口进程已正常退出。安装 `full-map-vision.service` 到 `/etc/systemd/system/`，用 systemd 重读配置并启用此服务。此服务无网络依赖。正式入口和旧入口共用 `source-rimpark.lock`。
4. 将 `full-map-hsv.desktop` 安装到现场账号的本地应用菜单。待机时在 HDMI 桌面打开“比赛视觉 HSV 调色”。窗口通过本机 sudo 停止服务，沿用原工具预览六色。用 S 保存，Q 退出。退出后脚本再次启动服务。服务日志的 `hsv_config_path`、`hsv_config_sha256` 可对照保存文件及断网重启。

## 现场与软件证据

原料使用既有配置。粗加工和暂存复用三环中心与车体边缘的纠偏方法。暂存模板以粗加工数值作为标定起点，格 22 实拍日志会记录三环圆心和相对车体的角度。确认格 22 的目标点、车体边缘取样和车体微调响应后，填写现场数值并启用 `target_confirmed_by_user` 和 `response_measured_on_vehicle`。模式 2、3、4 的占位记录都需各自三个环位样本。每个样本填写 `offset_px`、`radius_px`、`empty_color_max`、`occupied_color_min`、`empty_background_min`、`empty_hsv_low` 和 `empty_hsv_high`。码垛还要求现场验证取样可区分上下层后填 `top_height_separates_base`。样本未齐时保留 `verified=false`，只在现场验收之后启用。

前置 MATLAB 270 项通过。正式程序和真实采集循环接相机、串口、OpenCV 及检测器替身回放，两批共 88 帧通过。现场须核对相机曝光新鲜度、粗加工及暂存放稳放行、服务冷启动、HDMI 保存后重启、蓝和浅蓝实际识别支路以及首个有效帧耗时。均未部署或实测。

规则 R 核对 VIS-01、SRC-02、FIELD-06、RUN-01/02、PWR-01、MECH-01/02。工程 E 核对 VIS-03/04/07/10/12/14/15、RUN-06。规则原文保持不变，QR-04 待技术答疑确认。
