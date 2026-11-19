# 伸缩机构镜像安装：独立本地候选

2026-11-17用户确认：镜像安装后伸缩方向全部相反；0mm仍为完全缩回且有机械挡块，接线和驱动菜单未改，机构已停稳。目标是保留原动作和速度，只修正镜像方向及使用该方向的反馈判据。本轮严格离线：未连接探针、SSH、串口或相机，未下载、复位、清故障、发车或操作机构，未提交或推送GitHub。

冻结基线为 `tmp/stage2_settle_ring_20261117/project`，原在线AXF SHA256为 `18cc11eec2bf28b2642dbbb31c7c25807d651317a20464ca29997584262f293d`。独立候选为 `tmp/stage2_extension_mirror_20261117/project`；未覆盖基线和实际实车工程。工程选择E：逻辑伸出量仍为正毫米，`STARTUP_EXTENSION_EXTEND_SIGN=+1`，缩回碰撞回零方向为1。

## 实现范围

候选相对冻结基线仅7个固件文件有变化：

- `USER/vehicle_parameters.h`：仅两个方向常量，HOME_DIRECTION由0改1，EXTEND_SIGN由-1改+1。速度、加速度、毫米标定、位置/零速门限、故障、停止和超时参数逐字节不变。
- `CHASSIS/USER/extension_home_only.inc`：活动启动分支原先将回零方向硬限定为0，现核对其与伸出符号的对应关系；原参数核验保留。完成原状态、使能及精确零速检查后，调用仅节点6的临时回零方向设置，再进入原9A回零。ACK后的状态提交和取消检查处于短临界区，避免把已取消的状态6覆盖为运行状态3。
- `CHASSIS/SYSTEM/CAN/can.c`：新增节点6、state2、VM允许且无既有故障时的专用回零参数方向设置。固定save=0，读回完整17字节，保存原mode及其余参数，仅改方向。方向已匹配时不发送4C，仍作真实完整读回。实际4C命令去地址长19字节，CAN分包地址600/601/602，DLC8/8/5，每包首字节4C。严格匹配4C/02/6B ACK；失败或完整读回不匹配不得发9A，不盲目重发参数写。每个4C分包和9A最终入队前以短临界区重核owner，等待ACK/读取均在临界区外。
- `CHASSIS/USER/integration10_complete.inc`：抓取/返回位置判据及原48×3.6旧判据只改方向表达式，保留各自原量纲和门限，不趁机重标旧路径。
- `CHASSIS/USER/pzq_commit_overlap.inc`：Low_ExtensionAt使用统一有符号目标。
- `CHASSIS/USER/pzq_region_batch.inc`：65mm观察位置判据使用统一有符号目标。
- `CHASSIS/USER/stage3_cycle.inc`：反馈角度转实际伸出毫米使用统一符号；不代表第三段获准实车运行。

已审计但保持原样：`vehicle_source_parallel.inc`、`vehicle_arm.c`、`source_prepare.inc`及实际include链；它们的目标/行程转换已使用统一EXTEND_SIGN。非活动 `HARDWARE/EXTENSION/extension_home.*` 等历史路径不批量修改。原CAN/停止、故障/超时与唯一CAN任务所有权保留；不写轮1～4参数，不增加速度或新保护门限。

共享host模块 `src/smart_carrier/vision_manipulation/committed_pick_board.py` 的静态 `pose_ready` 仅增加可选方向参数，默认仍-1，旧18cc工具保持原行为。新 `mirror_committed_pick_board.py` 的 `MirrorCommittedPickBoard` 明确使用+1，构造必须提供本候选candidate manifest，先校验完整Flash段地址/长度/内容摘要，再保留原基类完整板载Flash核验。输入generator会先固化，避免二次遍历跳过核验。未批量改历史现场工具；未来使用本候选工具时必须显式选择新类及匹配manifest。

## 前置仿真与实际软件证据

证据目录统一为 `evidence/integration/2026-11-17/stage2-extension-mirror/`。

- 本机 `D:/MATLAB1/bin/matlab.exe` 实际运行：初始929检查于08:18:56通过后才写对应实现。覆盖符号、正负原点、全逻辑行程、往返、错误方向不放行、save0参数构造/读回及既有owner模型。初次证据为会话工具实际stdout和`matlab-result.json`；初次未生成单独log文件，不以后补日志冒充。
- 高审发现取消竞态后，实际MATLAB补充11检查于08:25:51通过后才写对应短临界区修复，含取消/提交交错和3包bus模型；见`matlab-cancel-result.json`、`matlab-cancel.log`。补充没有冒充初始前置结果。
- 真C抽取实际CAN参数helper和活动回零include执行279检查：捕获真实600/601/602及DLC/数据、save0、同向不写、17字节逐项破坏拒绝、ACK拒绝、取消、原参数错误、非零速不得回零、真实abort/STOP及临界区不等待。见`actual-mirror-home.c/.log`。
- 真C抽取实际Low/Region/S3函数112检查：不同原点、0～170mm、错误方向及反馈毫米转换，见`actual-mirror-gates.c/.log`。
- 真C抽取实际CAN1_ExchangeSingle执行2检查：FIFO清理期间取消后不得入队9A，仍拥有权限时原9A成功；见`actual-mirror-enqueue.c/.log`。
- host86检查：旧默认-1和镜像+1各自接受正确方向、拒绝错误方向；构造generator仍执行完整Flash读取；改动字节/sign/home字段在任何target动作前拒绝。见`host-checks.json`。
- 双Keil实际重构建均0错误0警告，最终在线 `30994d4a7fb445ea45e101425beeb73ca82b02ef74f9d893eaea133f7fe75a61`，194292字节Flash段；Offline `9f5f38bfef58f9ae4cf629f2658f537fc8722c1feaf60ba5527799af8f57a409`，157528字节。实际日志为`Vehicle_LiftRuler-build.log`和`Vehicle_LiftRuler_Offline-build.log`。Online为未来实车镜像；Offline禁输出，仅供离线目标检查，不能烧作实车版本。
- `candidate.json`列出最终双镜像和Flash段摘要；`source-manifest.json`列出2051源码文件摘要。`encoding-line-endings.json`记录7文件原UTF8/GB18030与行尾：所有非ASCII字节相同，原混合行尾中的裸LF数量不变，新增行使用其目标段原CRLF，无批量转码。

构建和软件模型通过不等于机械回零或实车通过。save0只表示不保存到驱动非易失存储；已设置的方向会在本次驱动供电期间保持，不能声称复位MCU即恢复旧方向。回退旧固件前必须重新核对或按旧固件核验恢复驱动当次参数，不能凭目录名直接继续运动。

独立high最终审查已通过：复核7文件局部delta、参数完整帧和严格ACK/读回、取消原子提交、host默认兼容与manifest绑定、279+112+2真C及86host检查、两次真实MATLAB前置时序、双Keil日志，并独立重算7源码和2AXF摘要匹配candidate。该结论仅覆盖软件，硬件验收仍未进行。

## R/E/Q与未闭环

已检查R：RUN-01、RUN-02、PWR-01、MECH-01、MECH-02；不改变自主/运输/电源/初始包络及展开规则。已检查E：RUN-06、CAL-05、VIS-15；方向镜像、临时参数和软件回归属于工程实现，不写成竞赛正式规则。官方未决Q继续保持“待技术答疑确认”，本次无新官方资料或规则覆盖。

未闭环：新机械方向首次实际验证、完全缩回挡块承受情况、重新建立真实原点、0→65mm观察→0往返、原料区首抓/三件和负载stall原因。后续需由主代理按新鲜现场条件和明确动作授权安排，不能继承旧原点/旧run或软件测试替代实际验收；第三段保持原暂停安排。本轮只本地完成，未下载/未实测/未上传。
