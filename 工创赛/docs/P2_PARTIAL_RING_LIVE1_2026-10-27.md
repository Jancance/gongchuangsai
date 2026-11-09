# 摄像头1.3：v2板端复核完成与live1独立实时取证

最新结果：live1实机ZIP已复核，87条全部质量候选通过，但84过期/3候选/0稳定；检测结束67条新鲜，显示提交仅3条，实时门未通过。live2针对前置帧龄/显示开销和质量历史与显示TTL混用作独立修订，下一项live2同场景10秒，尚未板端运行。[完整复核与工程差异记录](P2_PARTIAL_RING_LIVE2_2026-10-27.md)。下文“live1尚未执行”及显示TTL清历史为历史；原源/包不覆盖。

## 最新结果（E）

已读回 `offline-results-20261027-213000-557734.zip`（11549030字节）：CRC通过，报告manifest哈希匹配已发布v2；OpenCV4.5.4、19/19且逐项v1/v2语义一致。报告保存 `evidence/p2-camera/2026-10-27/partial-v2-offline-board/review.json`。

三张真实遮挡图初次处理121.6/115.2/111.6ms；各5次交替计时v2中位110.91/112.13/107.60ms，最大114.93/114.39/113.72ms。对应v1中位258.34/259.25/259.72ms。等距合成歧义样本拒绝分支192.1ms仍超门；不能把“三张图快了”写成任意输入的150ms保证。完整报告保持候选、非稳定、非毫米和非动作。

## live1交付（E）

新增独立入口 `p2_partial_ring_live_audit.py`，嵌入完整pixel1应用作为相机配置/依赖来源，103个原文件逐字节保留，不修改main.py、原检测器、原UI、配置或串口模块。旧pixel1、v1/v2离线工具继续可用。新入口不导入main或串口，不作为完整应用的替代品。

live1采用已验的v2候选，直接复用原 `LatestProjectCamera`、135°独立profile与启动控制流程；不更改曝光、增益、白平衡、分辨率和几何配置。启动仍会按该既有流程锁参，不代表启动参数被本轮重新验收。

采集、检测、GUI分离：原采集进程持续维护最新帧，读线程与检测线程分别使用单槽队列，不累积FIFO旧帧。主线程独立检查150ms主机read完成年龄；候选本身质量已通过后，再检查原图范围、新增时间戳、三帧/1.5px短窗口、方法切换和过期/失检/故障。失效清候选及稳定坐标；完整/残缺分支切换、观测间隔过大或显示状态过期时清窗口。GUI由原图重画，不在已标记图上重复画旧十字。

短窗口不表示独立物理停稳，没有OCR、毫米目标、串口或动作。保存检测完成年龄和imshow提交前后年龄；没有显示器实际呈现时间或相机曝光时间戳，不能据此签端到端硬实时。

默认记录10秒，首次结果后计时；无处理结果500ms或相机读失败退出并归档。Q提前退出如实标记completed_duration=false，采样结束时间在清理/PNG写入前截取。原图快照先存有限内存引用，结束采样后再PNG编码和写盘，避免同步写图影响采样窗口。每帧JSONL、GUI失效事件、相机事件、源哈希、首帧/首次候选/首次窗口稳定/失检/过期快照及汇总ZIP分别保存。

重要工程限制：单次处理约112ms低于150ms，不代表每个显示间隔均持续新鲜。若旧结果在新结果完成前已经过期，窗口会被保守清除，可能只出现候选/过期、没有绿色窗口稳定。这不是可以调高150ms或用历史坐标掩盖的问题；本轮先取实机日志，再按实际排队、处理和显示年龄定位。

## 本地验证证据

- `python -m unittest discover -s tests -p test_partial_ring_live_gate.py`：5项通过，覆盖新鲜窗口、过期/失检/故障撤销、重复/未来时间戳、边界、方法切换、抖动和观测间隔。
- `python scripts/validate_p2_partial_ring_live.py`：4项模拟入口通过，覆盖3秒自动结束、Q提前退出、读异常和处理停顿500ms退出/归档，未打开真实相机/GUI；结果 `tmp/partial-live-validation/review.json`。模拟正常分支出现过期/候选，无稳定结论，保留实测语义不冒充真实板端。
- `python scripts/build_p2_partial_ring_live.py`：103原文件及包内字节校验通过。
- 发布包 `--check`：来源与profile通过，camera_opened=false、maximum_host_age_ms=150。

这些是软件验证；live1板端尚未执行，未签相机、GUI、持续时效、稳定或遮挡新场景验收。

## 下一单项：live1真实相机10秒

包：`output/p2_partial_ring_live1_20261027.zip`，305073字节，SHA256 `a4f87d06f91f8c8441570adf03c8109d68654bfbae8e0425d9c1d7e672751231`。新目录，不覆盖pixel1。

Windows PowerShell上传：

```powershell
scp "C:\Users\Tenderness\Documents\ChatGPT\工创赛\output\p2_partial_ring_live1_20261027.zip" orangepi@robot-opi5pro.local:/home/orangepi/
```

上传后在香橙派VNC终端解压并先检查：

```bash
python3 -m zipfile -e /home/orangepi/p2_partial_ring_live1_20261027.zip /home/orangepi/
python3 /home/orangepi/p2_partial_ring_live1_20261027/p2_partial_ring_live_audit.py --check
```

确认PASS后，先在原预览按Q退出，避免同时占相机。保留当前纸、物料/遮挡、机位和光照，不需要重新摆图；当前真实场景以新原图为准，不假定仍与旧ZIP完全同位姿。在VNC终端运行：

```bash
python3 /home/orangepi/p2_partial_ring_live1_20261027/p2_partial_ring_live_audit.py --seconds 10
```

程序按既有流程启动相机，10秒自动结束。回传完整终端（至少最后汇总）和 `Evidence ZIP:` 实际完整路径；有条件保留GUI截图，但不以截图代替原图/日志。取回命令待实际路径出现后给出。不先换参数/轻遮挡，不以未见绿色认定算法失检；需区分质量、时效、窗口和显示过期。

## 需求与未闭环

R：SCORE-01（本轮不证明物理落环分数）、FIELD-07（既有垂直补光约束不变，本轮不改灯）。

E：VIS-04/05/07（像素候选，编号/毫米映射未验）、VIS-12（当前新鲜度/失效撤销）、VIS-13（沿用已验的显式可见边缘与歧义拒绝）、VIS-15（检测/窗口/动作分离）、CAL-02/03（像素证据不代替几何或物理精度）、RUN-06（超时/错误退出归档）。方案与门槛是工程实现，不新增官方规则。

Q：官方未明确项继续待技术答疑确认。

未闭环：live1实机10秒结果、连续新鲜窗口/失效恢复及持续性能、独立困难场景/真实同步带新分支、任意编号、几何/手眼/毫米/残余误差、物理落环和执行器。几何继续暂缓，动作禁用。
