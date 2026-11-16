# TURN90 keepalive 3 s — 本地编码交接

2026-11-17。用户授权有限放宽；本任务未操作硬件、COM、探针、下载、复位或发车，未 fetch/push/commit。根交接由 root 维护，本工作者未编辑。

## 唯一产品改动

`app_serial_runtime.c` 仅 MODE3 运行保活超时从固定 1000ms 改为 `AppDebugTurn90Plan(g_debug_plan_id) ? 3000U : 1000U`。关闭候选/其它计划仍 1 秒；非 MODE3、ARM 2 秒、token 500ms、HWT/CAN 和停止路径未修改。GBK/CRLF 保留。

隔离新客户端 `p1_serial_motion_turn90_keepalive3s.py` 复制自已冻结的 `turn90_90-field-retry1/p1_serial_motion_powered_done.py`。仅 PLAN20/21 的 `start_wait` ACK 从 2s 放宽到 4s。其它 ACK 2s、无 RX 5s、KEEP 0.2s、30s 总时长和成功 20s 尾窗保持；旧实测客户端原字节不变。

## 验证顺序和证据

1. 产品修改前，实际本机 MATLAB R2020a 运行 `validate_keepalive_timing.m`：75 项边界检查 PASS，结果 `matlab_summary.json`。只验证时序边界；无动力学改变，复用既有 54 组动力学证据，不声称这次重新验证动力学。
2. `check_keepalive.py` 从实际 C 源提取 CRC、数字解析、字节行累积、MotionCommand、SerialMotionSafety、CombinedTestStop、CombinedTestSafety，使用真实 app_entry/kinematics/profile 对象及正确初始化 ELF RAM 的 ARM+Unicorn 夹具执行：候选开 100 断言、关 101 断言通过。
3. 覆盖 TURN20/21 age2999 保持、3000 取消；其它计划 999/1000 保持；超时许可清零、result8/linklost1、真实停止分发调用 Abort/VM0；合法 KEEP 在 deadline−1 续期；错 token/CRC/不完整行/重复 START 不续期；超时后迟 KEEP/START 不复活；ARM1999/2000 与 token499/500 原界；tick 回绕。候选关时 PLAN20/21 和 ARM 拒绝。人工 STOP、严重 HWT 源失效/CAN 锁存故障在 +1ms 即进入原停止分支，与 3s 保活无关。
4. 真实新客户端 24 项通过：start_wait2.01/3.99 继续等待并 KEEP、4.0 单 STOP；其余 ACK 阶段/其它计划仍 2s；无 RX 5s 保持。旧产品和旧客户端使用同样门槛期望的红测失败已复现，见 `red_regression.json` / `before_timeout_red.log`。
5. `preservation_checks.json`：原六 dirty、旧客户端、非本次保护 owner 原样；两处修改逆向替换可精确还原冻结字节；新客户端语法及 scoped diff 检查通过。

## 限制与下一步

仅软件边界测试。App_SerialIdleReady、HWT健康输入和底层 CAN/VM 物理效果使用受控桩；未模拟 RTOS 调度、无线传输、实际停止时间或板上栈。真实入口/分支与故障分发被执行，不可扩大为全 RTOS 或实车验收。尤其单向丢下行而上行 KEEP 仍到时，客户端可能 5s 才判无 RX，若其 STOP 也丢失，板再等约 3s，总取消命令窗口可约 8s；不能写为所有通信故障 3s 必停。3s 也不能保证修复无线问题。

已生成但未执行 `build_candidate.py`，隔离目录 `工创赛/tmp/turn90-keepalive3s-build-20261117`，name `turn90_90_keepalive3s`，flags 沿用入口修复候选，source_checkpoint `6d380311`。root 统一做 high 收口和完整 Keil 构建。一次实测 START 已用，本轮没有新硬件重试授权。

已核 R RUN-01/PWR-01/MECH-01/02；E LOC-02/03/05/07、MOT-02/04、RUN-06。此次为 E 工程时序参数，不改 R。Q FIELD-05/OBS-03/QR-04 保持待技术答疑确认。真实90度/峰速、无线可靠性、实车停止与扫掠空间未闭环。
