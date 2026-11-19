# 第三段圆环观察独立包（本地候选，未部署）

本包只做第三段预览/采图/搜索标定记录和S31→O31观测桥。未集成原料算法，没有伪造source模式；未来同进程按区域切换尚未完成。本轮215物料、123槽位属于人工测试装盘，不是完整扫码结果。

MATLAB前置：`evidence/integration/2026-11-16/stage3/matlab-result.json`，12组/92故障注入/36排列通过。仿真、离线软件与真实抓放分别验收。本包未连接硬件。追加MATLAB标定数学验证6组/60仿射例通过，证据为`calibration-matlab-result.json`。A2/B3 fit草稿曾在该追加验证前写出、未执行；保留原草稿审计，最终实现于验证通过后重审修订，不能声称这一部分全程满足“MATLAB先于代码”。

## 现场先采图

先关闭正在运行的原料预览/识别程序，释放唯一相机和串口。不要同时启动两个程序。使用RealVNC终端，无需本机SSH登录。将本目录整个复制到Pi的`~/stage3_ring_pi_20261116`后，确认相机仍采用现场实际设备路径及分辨率。

```bash
cd ~/vision_tools/stage3_ring_pi_20261116
python3 stage3.py capture --output records/initial
python3 stage3.py preview --output records/preview
```

默认继承已核对的1920×1080和Generic相机`/dev/v4l/by-id/usb-Generic_USB_CAMERA_EP.20CC54K01-video-index0`。capture只采一张，不开串口、不发运动。preview按C保存原图和JSON，Q退出。记录含原图SHA、配置绝对路径/SHA、尺寸、主机时间；时间是主机read完成时间，不是传感器曝光时间。未标定可正常预览，显示INVALID METRIC。

Windows本机查看已有图，无需相机/串口：

```powershell
python stage3.py preview --image D:/path/current-raw.png --headless --output records/local
```

输出`preview.png`和`observations.jsonl`。环境需要Python3.10+、OpenCV、numpy；桥接额外需要pyserial。Pi可用系统`python3-opencv python3-numpy python3-serial`，也可在已有虚拟环境安装requirements。

## 尺寸来源与标定记录

官方名义尺寸已查明：初赛物料最大φ50（R），表3公式得到最外6环外轮廓95（E公式推导）；工位580×150、1到3号中心跨度300（R）；相邻150及上下边界距中环75是图3对称/等距布局推导（E），不是额外实测值。来源为官方《竞赛命题与运行》p5图3/表3和p6图5；证据见仓库`evidence/integration/2026-11-16/stage3/manual-dimensions/README.md`。75只适用于图中的真实矩形长边，不能把预览画布或其他线当基准。95为最外轮廓，黑线中心轮廓对应93.5；应确认算法选中边界，避免约1.6%尺度差。这些尺寸不要求用户重新尺量。

相机/TCP/轴向/底盘纠偏矩阵仍须本车验证。先拍当前DM180°、伸缩0、顶部观察姿态图像；`--select X Y`可指定中环像素候选。工具要求真实上下两条线都有效才记录，绝不会因保存搜索ROI而标为有效毫米。

```bash
python3 stage3.py record-calibration --image records/initial/实际原图-raw.png --select 960 540 --output records/calibration
```

选点示例必须按真实图像调整。`--reference-confirmed`仅用于用户已摆到期望TCP/底盘参考位且三环均实际识别的参考候选。`--matrices measured.json`可导入已独立测得的`image_to_station_affine`(2×2)和`chassis_image_to_motion`(3×3)，输出坐标约定为右+/前+/逆时针+。保存后始终`calibration_verified=false`。单帧记录不会被称为稳定标定。

要启用有效观测，先完成真实参考图、矩阵和实际轴向/TCP验证，再用以下命令导入本车矩阵、确认并设置原有`calibration_verified`标记，同时保存矩阵路径/SHA记录。本包默认false，不会自行把离线检查当实物验证：

```bash
python3 stage3.py confirm-calibration --matrices measured.json --tcp-axes-confirmed
```

`measured.json`包含实测`image_to_station_affine`和`chassis_image_to_motion`两个矩阵；工具检查尺寸/有限值/可逆性。不能用单位矩阵或上游矩阵代替本车实测后直接确认。无需用户手改复杂JSON或另建强制验收schema。

## 本车A2/B3拟合工具

先用capture分别保存参考位及少量独立右移、前移、逆时针转动前后的原图；这些动作由已授权的主控调试/现场操作完成，本工具不发运动命令。保持相机、分辨率、观察姿态和TCP一致。前三类必须独立覆盖三轴；另留正反方向或组合动作作独立检查。所有`command_right_forward_ccw`依次是右mm、前mm、逆时针度，记录实际采用的命令量；这不替代真实位移尺量。每对before/after对应该命令前后图像，独立检查数据标`validation_only: true`。

```json
{
  "reference_image": "reference.png",
  "pairs": [
    {"before": "reference.png", "after": "right.png", "command_right_forward_ccw": [10, 0, 0]},
    {"before": "reference.png", "after": "forward.png", "command_right_forward_ccw": [0, 10, 0]},
    {"before": "reference.png", "after": "ccw.png", "command_right_forward_ccw": [0, 0, 2]},
    {"before": "reference.png", "after": "check.png", "command_right_forward_ccw": [-5, 5, -1], "validation_only": true}
  ]
}
```

以上仅格式示例，绝不是建议立即发车的动作幅度。图像路径相对于pairs.json；参考图必须真实检出三个圆环，每个配对必须检出中环及真实上下基准线。

```bash
python3 stage3.py fit-calibration --measurements records/calibration/pairs.json --output records/fit
```

输出`records/fit/measured.json`含A2初估、B3、每张原图SHA、训练残差和独立检查残差，平移norm以mm、夹角以度分别报告。B3由实车命令与图像变化测得，可包含轴向符号、比例和耦合。A2是局部仿射初估，不能覆盖大透视、畸变、相机松动或任意90/180°转向；每次转区重新观察。当前主控采用先yaw后XY策略，因此不能仅凭B3可逆就声称任意强耦合响应均能闭环，须现场核对实际响应与该策略相容。输出不会修改配置或设置verified。

当车已处于真实期望落点的TCP参考位，记录参考图：

```bash
python3 stage3.py record-calibration --image records/calibration/reference.png --reference-confirmed --matrices records/fit/measured.json --output records/reference
```

现场核对真实边界、右/前/逆时针方向、独立毫米误差、TCP落点后，才由`confirm-calibration --matrices records/fit/measured.json --tcp-axes-confirmed`写入原有标记。本轮安装后只允许先capture/preview，不启动有效bridge；不能用synthetic结果或直接改true替代现场证据。

## 串口桥（尚未启动）

```bash
python3 stage3.py bridge --serial /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --baud 115200 --output records/bridge
```

设备路径/分辨率/波特率按实际接线核对。此命令会打开串口，供主控第三段闭环使用；本轮只准备，尚未部署或执行。唯一准入为上层S31 state2 observe，run/pose与MCU精确绑定，新epoch重置检测历史，旧frame不重用，status/观测主机年龄沿原200ms围栏。其余状态仅无效观测。固件到区和人工两次PC13装盘/搬车调度归主控，本包不宣布到区。

`S31,run,state,pose,item,color0,color1,color2,align_done,config_ready,CRC16\r\n`

`O31,run,pose,seq,valid,right_um,forward_um,yaw_mdeg,error_norm_um,error_yaw_mdeg,age_ms,CRC16\r\n`

CRC16 CCITT-FALSE initFFFF poly1021，覆盖CRC前末逗号，大写4hex；123456789→29B1。seq递增，motion是经过本车A2/B3矩阵转换的期望纠偏量，绝不直接把图像角当旋转指令。error_norm/error_yaw是B3变换前的真实平面norm和真实基准夹角残差，供MCU独立验收2mm/1°。config_ready只代表MCU机械配置，不拦初始视觉定位，也不能替代Pi标定。总龄从read开始算到发送前，包含读取和处理；图像时间为主机围栏，不冒充硬件曝光同步。`CAP_PROP_BUFFERSIZE=1`只向后端请求单帧缓冲，尚未验证后端是否执行；read开始围栏和200ms只约束主机读取/处理年龄，不能证明曝光时间或排除V4L旧帧。真实链路时效尚未验收，因此本轮只安装文件、先独立capture/preview，默认标定false。

验收需求：R FIELD-06、RUN-01/02；E VIS-09/12/14/15、CAL-02～05。官方Q保持待技术答疑。未闭环：真实相机图、所选外边界、真实上下边界、A2/B3/轴向/TCP、视野覆盖、实时链路和实车三件放料。
