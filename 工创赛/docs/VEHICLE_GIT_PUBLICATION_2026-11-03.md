# 整合1：Git上传接续记录

日期2026-11-03。接续“整合1”最后明确请求：将本次内容更新到Git仓库。此前本地交付完成，最后上传仅执行了bare clone，因额度中断；远端main仍为842efa52c3dfdd878583c87a02976a511cebaa44。此次以该提交为父提交整理差量，保留已有远端历史，不使用强制推送。

## 本次交付

- 仓库根`3.stm32f407工程/`同步实际工程源码与Keil工程文件，目标Vehicle_Integrated、Vehicle_Offline。旧独立源仍保留，编译入口以uvprojx为准。
- 统一参数`3.stm32f407工程/USER/vehicle_parameters.h`：八段距离/速度、逐行机构动作、上电角与参考有效性。保护处置和现场步骤见VEHICLE_FIELD_TUNING_2026-11-03.md。
- 软件桩脚本支持直接验证仓库整合源码，环境变量VEHICLE_SOURCE_ROOT可显式指定源码根；Windows GCC固定安装位置见脚本。
- 上传检查脚本、构建日志、桩检查报告、原件差分备份和源码SHA证据；不加入本次生成的exe、Objects、Listings、个人IDE会话、安装包或整个tmp目录。原历史报告中的git=false描述上传前状态，不回写冒充当时已发布。

## 验证与边界

实际工程30个改动源码与既有manifest一致；uvprojx的差异核对为Keil重排和六个新文件默认选项展开，目标引用及IncludeInBuild一致。待上传源码逐文件复制并记录SHA256。复跑六组软件检查及比赛profile误用调试旁路的编译拒绝检查，结果见git-publication/与early-scan-startup/test-suite.json。本次待上传源码两目标重新Rebuild均0错误0警告，Integrated的Code/RO/RW/ZI为124128/6612/3272/118888，Offline为115076/6604/3112/118804，与既有实际构建一致；接续工作不改固件逻辑。

构建和桩检查不关闭机械坐标参考、反半圈扫掠、工位高度/伸出量、载盘角、第一段距离/屏显、后七段/双起点、真实任务/物料/库存、实时XY/障碍、带载速度、包络/净距和180秒整场演练。未下载、复位或向硬件发送动作。

## R/E/Q

检查R：QR-01/05/06/07、DSP-01/02、FIELD-01/02/06、OBS-01/04、RUN-01～04/07、PWR-01、MECH-01/02。检查E：LOC-03/05/06/07、RUN-05/06、MOT-01～04、CAL-02～05、MECH-06、VIS-09/12/14/15。本次为E类工程发布，不改变R；FIELD-05、OBS-03、QR-04等Q仍**待技术答疑确认**。证据位于`evidence/integration/2026-11-03/git-publication/`及原early-scan-startup目录。
