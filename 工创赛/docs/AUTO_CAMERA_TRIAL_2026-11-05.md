# 整合5：自动曝光与白平衡观察候选

> 原包复核及读回更新（2026-11-05，E）：已取回24,475,866字节、SHA256 f23f4d4e8ca62382bf0662dbc48aac59eb33986341fcebbd286e615c08d0625e，ZIP CRC通过，已通知用户关闭8765临时服务。616帧/609非过期/7过期（1.136%），7过期均清空目标；结果年龄p50/p95/max=111.73/136.94/197.40ms，检测调用96.31/109.80/175.87ms。609非过期帧中蓝100、缺蓝509，红539、黄442、浅蓝18、绿1；96帧仅红黄蓝，不能将4帧红黄蓝+浅蓝当正确三色。切换时刻未知，整包不能等同连续自动模式验收；7/616与旧39/266仅各包描述，非同条件收益结论。原四模块SHA与HSV副本匹配，包未记录新增helper SHA，未知项保留。用户本轮读回AE3/priority0/AWB1/gain32/曝光50/色温4600，确认模式，曝光字段不作为实际逐帧快门。首次蓝丢失原图的末尾1px去噪蓝轮廓圆度0.4188/凸实度0.9055，触发原0.60圆度门；正常恢复图可到0.6561/0.9520。最长已结束蓝缺失区间9.332s，其事件原图是恢复帧，不能误称缺失帧；左侧桌面浅蓝候选bbox174,656,66,40为可见背景误检。8图回放/追踪见scripts/review_auto_camera_board_20261105.py及board-review/offline-regions/blue-fallback-trace.json。保持自动控制候选和质量门，不签稳定识别/抓取。

> 部署准备（E）：现有p2_auto_camera_20261105.zip实现重启/重连时AE3+持续AWB，而非每次重启后再人工执行v4l2。电脑临时HTTP只服务output/auto-camera-delivery、绑定10.252.117.235:8766。准备交付独立p2-auto-camera-20261105目录，旧HSV目录保留；尚未获板端下载/启动证据。该包仅解决自动控制持久接入，不包含蓝色漏检/背景误检修复；下载并读取manifest后须逐文件核对，实际启动检查控制读回。板端将启动新版本，因此启动输出和画面验证有必要，不要求重跑旧固定曝光测试。

> 实时回传更新（2026-11-05，E）：用户已在p2-hsv-live副本启动F诊断，附终端完整输出与自动控制命令无错误返回截图。新预览较前图明显亮，红黄有框、蓝无框；不能据单图认定稳定识别或把蓝失检归因AWB。日志主循环FPS由非零首值15.4到末8.5，属于处理/显示吞吐，不是传感器帧率证据；记录蓝出现/丢失/换号、额外green/lightblue及重复yellow/lightblue事件，事件名不等于误检根因。源码已核first-blue-with-red/yellow等仅指监视蓝色的运行中出现该颜色，不保证蓝同时存在。终端确认Serial=DISABLED/motion_allowed=false、正常保存ZIP：/home/orangepi/p2-hsv-live-20261105/color-follow-audits/20261105-161032-971488.zip。尚未取回ZIP、自动控制当前读回和精确切换时刻；不做配置相同时效对比，不放宽阈值。来源归档evidence/p2-camera/2026-11-05/auto-camera/hsv-live-user-console.txt及auto-camera-user-preview.png，结构化摘要user-run-intake.json。下一项只读取当前控制并取回该精确ZIP，不先重跑/调阈值；独立自动启动候选仍仅本机准备未部署。下文“待用户执行”是本次回传前的历史状态。

来源：用户2026-11-05暗场预览截图及本机v4l2-ctl --list-ctrls-menus完整终端输出。截图表明当前画面暗，不能从截图独立证明现场照度变化的原因。实际控制读回exposure_auto=1（Manual）、exposure_absolute=50、gain=32、white_balance_temperature_auto=0、temperature=4600。明确支持exposure_auto菜单1 Manual/3 Aperture Priority，exposure_auto_priority布尔，AWB布尔；power_line_frequency=1（50Hz）。

## 工程决定（E）

固定曝光50/增益32无法持续适应变化环境。先观察硬件AE3+持续AWB1，exposure_auto_priority保持0，不请求通过降低帧率延长曝光。此设置不是最大曝光上限或实际30FPS证明；自动曝光不保证无运动模糊/不削顶，持续AWB可能改变HSV分布，六色需实测。

Linux V4L2说明：模式3自动曝光时间；priority=0禁止自动变化帧率；自动模式中手动曝光值不应同时写入。来源https://docs.kernel.org/userspace-api/media/v4l/ext-ctrls-camera.html 。白平衡用于色温/偏色，不能替代曝光补亮。

当前预览已开启时可在另一终端执行（尚待用户执行/回传）：

```bash
v4l2-ctl -d /dev/v4l/by-id/usb-Generic_USB_CAMERA_EP.20CC54K01-video-index0 --set-ctrl=exposure_auto_priority=0,exposure_auto=3,white_balance_temperature_auto=1 &&
v4l2-ctl -d /dev/v4l/by-id/usb-Generic_USB_CAMERA_EP.20CC54K01-video-index0 --get-ctrl=exposure_auto,exposure_auto_priority,exposure_absolute,gain,white_balance_temperature_auto,white_balance_temperature
```

保持当前预览，不重启旧程序，5～10秒后检查暗区、黄色削顶、三色/背景误检、FPS。固定等待不等于收敛确认。旧版每次启动仍写固定设置，所以该命令仅为当前预览临时切换；独立持久候选另外准备，不混称部署完成。若色彩漂移，观察调节后单独关闭AWB（white_balance_temperature_auto=0），AE保持自动。读回不符/画面异常先保存证据，不据此开放运动。旧固定设置可用exposure_auto=1,exposure_absolute=50,gain=32,white_balance_temperature_auto=0恢复，但AWB已改变色温，关闭AWB不等于恢复此前色温；此前4600读数只适用于先前会话，不保证是当前现场正确色温。

## 本机实现与验证

scripts/prepare_auto_camera_trial_20261105.py从已准备HSV实时副本建立output/auto-camera-trial-20261105/vision_app。仅camera135_startup_check.py和generic_135_camera.json改变；检测器、HSV、跟踪、150ms期限、曝光50对照及原程序保留。自动分支验证实际菜单3、AWB及priority能力；设置后读回核对。不写手动曝光/增益/色温，增益由本机当前状态/固件策略决定，仅记录，不宣称自动增益。profile白平衡可选auto或startup_lock；原手动分支保持。

本机--check退出0，1920×1080、serial DISABLED、metric_geometry_verified=false、motion_allowed=false。scripts/check_auto_camera_trial_20261105.py共7项模拟检查0失败：持续自动、启动AWB锁定、未支持模式、缺少控制、错误读回、写入异常、非法模式。模拟检查不等于相机实测。

ZIP：output/auto-camera-delivery/p2_auto_camera_20261105.zip；SHA256=58aad0035c9126f30053416f4ed96610639df298309cc08255f228f595ac66c0，ZIP CRC通过。尚未传输/安装/板端运行。原启动测试工具中的历史“关闭AWB”文案不作为自动分支的验收声明；本轮以实际控制读回为准。

未闭环：当前AE/AWB切换的实际画面、自动控制的暗亮过渡/收敛、黄色削顶和蓝浅蓝/黑误检、动态拖影、实际帧率/帧年龄/ID连续、六色及几何/毫米/抓放。原90秒HSV采集若跨越本次控制切换，应标记混合设置，不作为同设置优化前后时效验收。

R已核VIS-01、QR-01/05/06、DSP-01、RUN-01/02/04/07、PWR-01、MECH-01/02；E已核VIS-02/03/09/11/12/15、CAL-02/03、RUN-06。本次自适应策略属E，VIS-03可锁定能力保留，并非官方禁止自动。R无变化，官方Q保持**待技术答疑确认**。
