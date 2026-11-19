# 第二段 / 第三段统一桌面候选（2026-11-16，本机离线）

当前已断电，先看`OFFLINE_READY.md`：包含可复现打包、离线SC2→S31→SC2回放、只安装不启动、preview启动/回滚和自启模板操作。原59项描述为此前候选，本次交付项数以manifest为准；新包未现场安装，自启未改。

工程选择 E：同一个 source capture.process、相机工作进程和唯一串口；有效 SC2/SC1 显示原料区左纠偏/右抓取，有效 S31 显示第三段左纠偏/右物料放置。区域由 MCU 协议决定，不看图猜区，不启动两个程序。原两包不覆盖。本候选未部署、不改自启、不连接硬件。

源区 fast process 中原 active/status 至 callback 结束的检测、HSV、Observe120、五帧停稳、OC1/OC2、渲染代码文本完全一致；新增处理只在其上方分派第三段。S31 用原 stage3 Pipeline、Session、O31，不写入运动命令，不更改其标定 false。第三段原始图未标定仍可显示，标定不齐不会输出 valid=1。右侧显示车载槽颜色与原始 MCU item，目标圆环显示“按主控任务映射”：S31 没有 positions/first/count，不推断实际当前槽、目标圈或已放置成功。state6 仅“本轮软件动作完成”。

部署入口（只准备，由主任务审查后执行）：start.sh 默认 --preview-only；start-preview.sh 同样只预览；start-trial.sh 保留原桥接行为，需要明确选择且有效标定才有第三段有效观测。三个脚本均指向本候选目录。仍使用旧 source-rimpark.lock，避免和原料程序同时持有资源；不能绕过旧锁另开第三段 bridge。

相机边界：只在同 worker 连续 sequence 时以前一 read completion 为当前 read-start 的保守下界；首次/换 worker/跳号不发该帧。Session 原有新 pose/state fence、200ms 状态/总年龄继续工作。此证据仅为主机时间围栏，不能证明硬件曝光新鲜或 V4L2 无旧缓冲。返回 SC2 清 Session/检测历史；进入 S31 清源区观测与旋转历史；不因切区发送任何 motor 命令。

前置 MATLAB evidence/validate_region_flow.m 已先通过后写实现：单 owner、SC2/source 保真分派、S31 state2 才可有效、epoch 和未配置无 valid、零 motor。日志 evidence/matlab.log。Python evidence/test_region.py 4项和 stage3 原测试8项通过；mock PNG 为黑色输入的布局模型，不是相机实拍/实车通过。源区、第三段 PNG 已目视检查。

已检查需求：R FIELD-06、RUN-01/02；E VIS-09/12/14/15、CAL-02～05、RUN-06。R 不变；官方 Q 继续待技术答疑确认。未闭环：真实部署与状态切换、相机时效、第三段实际参考圆/上下边界、A2/B3/TCP/轴向、实物抓取与放置。无硬件验证声明。

清单 manifest.json 为本候选 SHA256（排除清单自身、pycache）；upstream-integrity.json 逐文件核对 source/stage3 复制保真与仅预期入口改动。旧两目录未被本任务写入。

## 预览 RX/TX 与摘要说明（本机补证）

`--preview-only` 原有行为是唯一私有状态串口持续 RX，只禁 source/third 观测 TX；capture 摘要中的 `serial: DISABLED` 指 app.serial_link dummy，为硬编码摘要，不能据此推断 MCU 状态串口关闭，`metric:null` 也不代表 callback 未运行。新增 CONFIG 明确两个通道角色，S31 状态改变输出 MCU_STAGE3_STATUS。第三段每个新状态上下文第二个渲染帧保存 session/views-current.jpg，频率与源区原做法一致。未改变 RX/TX 行为、检测算法或阈值。

补充 MATLAB validate_preview_rx.m 已先通过，之后完成日志/截图修正；真实入口 test_entry_preview.py 执行真实 entry.main → capture.main → callback（硬件和源区检测器替身），输入 S31 state2 两帧再 SC2 state2 一帧，唯一串口 open1/read3/write0，真实保存 views-current.jpg。适配器单测的已标定 state2/epoch/preview 分别保留验证。现场是否实际收到 S31 仍须看 observations.jsonl 的 region/mcu_status 或 MCU_STAGE3_STATUS，不以摘要推断。
