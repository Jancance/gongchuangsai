# 码垛底层三物料视觉：离线候选

2026-11-19，仅本地准备，未连接相机、串口、调试器、Pi，未安装、烧录、复位或发动作。同步状态由主任务统一记录；此分工不修改共享旧文件。

## 规则事实、工程实现与未知

- R：QR-01/05/06/07规定六色编码、四段任务与第二批码垛在第一批正确放置的同色物料上。第二批暂存位置使用第一批颜色位置映射，不能沿用第四段粗加工位置。RUN-01/02/03、FIELD-06、PWR-01保持。
- E：复用现有 `project_detector.analyze` 及 `material_color_thresholds.json`，只适配其当帧颜色与轮廓中心，不改颜色阈值、检测算法或增加颜色置信门限。配置任务215+213+512+321/start_id1，物料平面响应未绑定；不得直接作为实车完成版。
- E：三个不同颜色与第一批物理暂存位置逐一对应，三件必须均为当帧实际检测；不重建缺失中心。必须填写实测physical_image_order的图像x/y轴和沿正轴的物理环号顺序，缺省不假定左右方向；同色三件齐全但颜色放错环时仍无效。沿用每次独立观测5帧、原中心抖动1.5px和原Session 200ms新鲜度，切换run/pose/state即清空历史。这里的物料轮廓中心不是已经验收的顶面中心或TCP。
- Q：官方已有待答疑项不变，继续待技术答疑确认。独立物料观察姿态、工作面高度、中心定义、实际HSV配置和本区响应标定均待实物闭环，不得用工程候选替代。

## 原S模式核对结论

`vision_app/code/main.py:379`调用`StackedRingDetector`。其文档明确为“中间圆环＋上下长直线”，并重建三个圆心（`stacked_ring_detector.py:4`）。原S并非“三件底层物料识别”。本次只借鉴局部观测与沿用同一观测协议的组织方式；不照搬其空环定位，不用旧环平面J、历史抓放点或已部署前15/右5目标。

## 新增文件与接口

`scripts/prepare_stage3_stack_vision_offline_20261119.py`提供：

1. `StackMaterialPipeline(config,size)`：实际图像→既有物料检测器→按第一批暂存环排序→5帧稳定。`detect/reset/alignment_ready/measured_motion/verified`用于独立观察模式适配。
2. `OfflineStackBridge(pipeline)`：接收CRC有效的既有S31，管理观测epoch；`process`只返回既有O31字节，永不写串口。重复帧、旧epoch和过期帧不能计入5帧稳定。有效观测要求同一epoch的新鲜三件观测及独立物料平面响应。
3. 仅离线CLI：`--sequence`读取本地图像与记录时序，生成报告。没有设备参数、网络调用或启动硬件选项。

配置为`config/stage3_stack_materials_offline_20261119.json`。未标定状态输出`alignment_response_ready=false`及O31 valid0，不会假装毫米标定完成。补齐独立物料平面J、仿射、参考像素/yaw及证据后，代码可计算候选运动观测；`verified()`仍为false，表示未取得物理精度验收。配置SHA、当前颜色配置SHA和相机标定SHA改变使候选响应失效。

host不做任何码垛高度扣除，报告固定`stack_height_subtractions_in_host=0`。由本次固件的cycle mode2负责一次60mm高度差；现有`stage3_live_20261116.py`的旧storage-stack减高逻辑不能与新固件重复使用，主任务另行适配。

### 已生成desktop独立分支（尚未安装）

`scripts/package_stage3_stack_desktop_offline_20261119.py`生成完整本地候选`output/stage3_stack_desktop_offline_20261119/`。原source/ring代码保留为`_stack_base_region_desktop.py`，新region_desktop在`stack-material-config.json`的`observation_mode=stack_base_materials`时实际创建StackMaterialPipeline/OfflineStackBridge。包内带现有颜色检测、初赛任务解析与协议依赖，已部署文件未改。

本地已导入生成包并以1920×1080空白图、无metadata、preview_only执行新分支，确认pipe类型、无O31输出、UI为400×960；未启动相机或串口。选择该配置才启用分支，包内原启动脚本保留原参数；不自动把比赛入口切到未标定候选。包manifest记录本地文件SHA。

现有`output/stage23_desktop_20261116/region_desktop.py`固定创建原`Pipeline`并按`verified()`计算有效观测，直接替换类名不足以启用本候选。必须在用户明确选择storage-stack观察模式时分支接入，原放料/回收模式仍保留原分支：

```python
from prepare_stage3_stack_vision_offline_20261119 import StackMaterialPipeline, OfflineStackBridge
pipe = StackMaterialPipeline(stack_config_path, actual_camera_size)
bridge = OfflineStackBridge(pipe)
# 唯一串口owner收到S31时：
bridge.accept_status(original_s31_bytes, status_received_monotonic)
# 原相机owner取得连续generation/sequence且已建立read-start下界时：
row = bridge.process(raw_bgr, frame_id, acquisition_start_lower_bound,
                     read_completed_monotonic, now_monotonic)
# row['wire']为候选O31；由现有唯一owner按原preview_only策略决定是否发送。
# 本次离线入口只记录row，永不执行serial.write。
```

若capture metadata缺失、相机generation变化或序列不连续，调用`pipe.reset()`并跳过该帧；不得给未知采集起点补造时间戳。源区模式切回时重新绑定session并清空历史。必须使用新物料profile而非旧ring-row profile。生成包已包含分支，尚未安装或启动进程。

离线序列每行为JSON，字段`image`（相对序列目录）、`frame_id/acquired_after/completed_at/now`（同一单调时钟，秒）；更新状态时另提供`status`（含CRC及CRLF的原S31文本）和`status_at`。同一图像重复回放仅是合成/离线软件证据，不能称为新鲜相机实测。

```powershell
python scripts/prepare_stage3_stack_vision_offline_20261119.py
python scripts/prepare_stage3_stack_vision_offline_20261119.py --sequence PATH_TO_LOCAL_JSONL
python scripts/package_stage3_stack_desktop_offline_20261119.py
python -m unittest discover -s tests -p test_stage3_stack_vision_offline_20261119.py -v
```

## 验证证据与未闭环

- MATLAB前置：`scripts/validate_stage3_stack_vision_offline_20261119.m`，本机D:/MATLAB1执行，`tmp/stage3-stack-vision-matlab-20261119.log`记录128例通过。覆盖合成不同符号/耦合J、平移反解和无向yaw处理；所有矩阵均为合成测试，不是本车标定。
- Python：6项单元测试通过，含实际既有检测器对合成RGB三物料图运行→第1～4帧O31 invalid→第5帧valid，CRC解析得到预期候选平移；另覆盖三件色位交换必须拒绝、缺件、重复色、未知件、空场无环fallback、抖动、清历史、缺标定、配置变化及过期Session。
- 默认CLI已运行，`output/stage3_stack_vision_offline_20261119/report.json`明确devices_opened空、物理标定false、独立物料响应unready。
- opencv-pzq知识审计已执行，仅作审查线索，未修改skill。

缺少真实三底层物料新图、用户实际保存的HSV配置、物料平面响应标定和参考像素、实际三环堆叠点位/工作高度，以及desktop选择模式安装和实物验收。离线测试不能证明层数、倒塌/倾斜识别或最终落稳，VIS-10尚未闭环；已有检测返回的是轮廓中心，不承诺顶面/TCP几何精度。未新增自动停车、永久锁定或故障门限。

已检查：R QR-01/05/06/07、RUN-01/02/03、FIELD-06、PWR-01；E VIS-09/10/12/15、CAL-02/03/05、RUN-06。软件准备完成、现场标定与物理验收未完成，未提交推送。
