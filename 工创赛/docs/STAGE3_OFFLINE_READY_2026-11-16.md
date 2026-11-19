# 第三段断电期离线交付（2026-11-16）

当前用户明确已断电。本轮仅修改Windows本地文件、运行MATLAB/C/Python与Keil构建、生成本地包；没有probe、SSH、串口、相机、烧录、复位、电机动作、远端安装或现场自启修改，没有提交/推送GitHub。a54eb0ab曾实际烧录、槽2红料160预收和取高被认可属于历史证据；断电后运行参考、保持状态、实时位置和历史PID均失效，不能作为当前live状态。

## 已完成的软件

独立候选入口：`tmp/stage3_offline_ready_20261116/project/gongxun-f407.uvprojx`。旧`tmp/stage3_preclose160_20261116/project`保留。仅修改四个源码：`CHASSIS/USER/stage3_cycle.inc`、`stage3_cycle.h`、`integration10_complete.inc`、`source_vision_runtime.h`，镜像保存在`src/stage3_offline_ready_20261116/`。

- S3Teach按所需轴采集，不再依赖原generic Z/X诊断及其四轮guard准入。没有修改generic诊断驱动或声称所有GPIO/诊断问题已修复。Z取已有IT完成链转发的真实VSP/LRC反馈与真实顶部参考，不能取目标值；DM保持时等原Jog owner下一真实反馈tick，失能时走原CAN只读request2，不并发第二个CAN/FIFO owner。真实`g_lift_diag_raw`外部声明改回`uint32_t[6][8]`/192字节。
- 结果`1=saved`、`2=pending real DM feedback`、`3=not-ready/cancelled`。missing仅诊断：1状态/参考未就绪、2缺真实Z、4缺尺量X、8等DM。新请求序号与saved序号证明本次完成；重复请求不覆盖pending，离开示教/收到STOP取消旧pending；无新超时、保护门限、永久锁或自动动作。侧尺量X使用后回到−1，不改编码器反馈或参考。
- 任务颜色→目标环与库存颜色→实际槽独立。`Stage3_BindMission(Stage3_Task*, slot_colors)`验证并绑定任务run/batch、三色/目标环和库存；动作开始锁定三表，按颜色查唯一槽位再转0/120/240，按目标环查yaw/X/Z，不再把item、槽号与环号当同一件事。当前215/123/槽215明确是调试对象。
- `Vision_ReadLoadedTask`只读便利接口仍要求原真实三件commit完成。当前SourceCycle Low快路径没有生成该严格commit，因此本轮没有实现或宣称真实扫码→跨区路线自动交接；不能伪造loaded/健康状态绕过。已有mission/CAN owner可将真实任务与库存填入结构，先BindMission成功再调用原`Stage3_Arrive()`，不调用`Vision_SetTask`清库存。
- 仍为原第三段普通速度Z1846.4RPM、X106RPM、DM300°/s及原加速度，160预收/140夹紧/210释放与原STOP保留。原料29700/X150及原驱动文件逐字节不变。

| teach-kind | 保存内容 | 数据依赖 |
| --- | --- | --- |
| 1 | 取料Z | 完成的真实Z+参考 |
| 2 / 4 | 环1 / 环3完整DM、尺量X、Z | 真实Z、下一真实DM、显式尺量X |
| 3 | 环2放料Z | 真实Z；环2固定180°/X0 |
| 5 | 载盘取料DM角 | 下一真实DM |
| 6 / 7 | 环1 / 环3的DM与尺量X | 下一真实DM、显式尺量X；不要求新Z、不覆盖未知或已有放料Z |

## 本地证据与构建

证据目录：`evidence/integration/2026-11-16/stage3-offline-ready/`。`candidate.json`列出四文件差异、九个关键原料/驱动文件保真hash及双AXF，`checks.json`记录实际C与构建，`rebuild-check.json`记录prepare+finish在新目录重新生成四文件逐字节相同。

| 离线证据 | 结果与范围 |
| --- | --- |
| MATLAB | 8组；包括216任务/环/库存排列、单CAN owner、真实测量公式、STOP与几何6/7。是离散数据/流程模型，不是实车动力学或净空验收 |
| 实际C | owner/示教/映射5223、只读getter14、IT预收46、deep143、真实LRC56、home560、原料group10检查通过 |
| Keil | `Vehicle_LiftRuler`、`Vehicle_LiftRuler_Offline`完整重建均0错误0警告 |
| 在线AXF SHA256 | `967c9140cf78c1b937955a9dabd849c8c6c696dad7316853935a122e5e13a6d7` |
| Offline AXF SHA256 | `43fd2f0a66d4c2f7827809f121dedca7c7ea1f650b4a4ff6abad25e7da8bce89` |
| host | 22项测试、py_compile和新manifest无动作离线检查通过；最终高档审查已通过，不代表硬件通过 |

MATLAB时序必须按增量解读：15:51:24的teach依赖/CAN所有权5组先于teach实现；15:54:37的216排列先于任务绑定实现；16:02:19的几何6/7模型先于其实现。纯软件sequence/saved-sequence发布逻辑先写入固件，15:59:44模型随后复核且早于host实现，因此不能宣称最后8组全部先于全部源码；manifest已明确该时序，没有伪造前置结果。

主机`stage3_live_20261116.py`默认仅本地manifest/hash检查，不连接硬件。显式动作才连接。示教使用新seq+savedseq匹配+result1，不再要求三轴diagcycle全部变化；侧2/4/6/7必须传`--measured-x-mm`。`--dm-release`仅复用原jog_request3，要求已有顶部/Xhome/参考及IT/SBS空闲，等新真实反馈、jog收发和失能ACK；`--dm-read`仅CAN read_request2，保持中拒绝并提示先失能，不发jog_request2、不写0xFE零位、不写健康或holding。

固件967c9140、host 22项测试对应最终脚本、Pi 64项/e1a41fc7三项最终高档审查均已通过（本会话审查结论）；现场尚未部署或验证。完整增量包为`output/stage3_offline_ready_20261116.zip`，内含本指南、最终host/测试、四源码及校验依赖、双AXF、八组MATLAB证明、配置/provenance和Pi ZIP。包内`README_OFFLINE_DELIVERY.md`说明完整工作区基线依赖与本地核验命令，`MANIFEST.sha256.json`逐项记录内部文件hash（不含自身）；外部`evidence/integration/2026-11-16/stage3-offline-ready/delivery-report.json`记录两个ZIP的hash，避免自引用。包是增量交付，不能当作独立完整工程。打包仅处理本地文件，不自动安装或连接硬件。

## 配置保存和估算边界

`calibration-config.json`仅包含restore允许的五个配置字段，可以作为将来显式配置恢复的输入；`calibration-provenance.json`单独保存来源、调试三表及估算说明，不向运行参考写入旧值。

已认可取高来自06:36完成IT/sequence6的真实current903.8099975585938°与当时top342/100，保存为约8003.466645p，float存储约8003.466797p。与历史actual8003.467285差远不足1p；不是目标8000冒充反馈。原始证据`stage3-preclose160/teach-readonly-20261116-065232-pick-config.json`保留。无需重复测量已认可取高；上电后仍须原流程重建真实机械参考，再显式恢复该配置。

环2固定DM180°/X0，放料Z仍−1；左右角/X/Z仍−1。205mm为顶部指尖距地实量，物料底低指尖约50mm是估计，所以155mm相对下降与`600+400×155=62600p`只为估算候选，不是已确认放料目标，不自动下发。真实放料高度、两侧几何及带料净空仍待现场闭环。

## 统一桌面与上电后的最短路径（当前不执行）

统一桌面包`output/stage23_desktop_20261116.zip`共64项，SHA256 `e1a41fc7596f9fba98a11bcbbb144cbe23b57068cc3733ed9f0ef6b68bd2fe3f`，操作指南见`output/stage23_desktop_20261116/OFFLINE_READY.md`。默认preview是状态RX开启/有效观测TX关闭；未标定trial可能发valid0，不能说串口完全无TX。未启标定、未恢复自启。SC2与S31只做区域显示分派，S31缺目标环协议时显示“按主控任务映射”，不猜颜色等于环号。

1. 用户上电并确认当前实际承托、停稳空爪、可断电、行程空间与215槽位。先核实际版本/现场程序，不继承断电前位置和PID；若选择新候选，之后才安排显式下载/读回。
2. 按既有启动流程重建真实参考与观察姿态，显式恢复纯配置文件，核槽2及已认可取高配置。B3/TCP仍未认证，不启动自动纠偏或三件抓放。
3. 侧几何在顶部/Xhome/空爪条件下，显式DM失能并取得真实motor0 ACK后才手摆；`--dm-read`取新角，尺量X，用kind6/7记录几何，保留放料Z未知。恢复运动只使用原IT3，不写零位。
4. 环2放高用kind3保存真实Z；两侧最终放高可用kind2/4完整示教。先中心单件，再两侧/带料净空，再三件流程；一次动作失败保留首故障，不自动重复循环。
5. Pi独立目录安装、preview验屏和人工自启切换按其OFFLINE_READY执行；本轮只交离线包，不执行这些步骤。

R已检查RUN-01/02、FIELD-06、PWR-01、MECH-01/02；E已检查CAL-02/03/05、VIS-09/12/14/15、RUN-06。示教、三表与预览模式属于工程实现，不改正式R；官方Q继续“待技术答疑确认”。未闭环：新固件/host/桌面现场验证、实际DM失能/手摆/恢复、真实放高与两侧点、B3/TCP与曝光时效、带料扫掠/净空、中心单件与完整三件、真实扫码跨区交接。软件完成、仿真通过、编译通过、下载和实物验收分开记录。本次仅本地完成。
