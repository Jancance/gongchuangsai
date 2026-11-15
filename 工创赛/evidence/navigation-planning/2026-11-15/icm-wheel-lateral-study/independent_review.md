# 冻结候选独立 high 审查

2026-11-15，检查者 `/root/observer_review_high`。协调者正式 FROZEN_READY 后核对当前候选；结论：**本地独立观测候选软件审查通过，接受两处断言源行号元数据的窄例外，无阻断发现**。这不是下载、实车或定位精度放行。

## 冻结与隔离

- C `ef3c62a442fc7b893736cc6ec7da8bf6d7d1d6dd6e20ff50d2c7d24098dff163`；Integrated AXF `0ebf9e4e5d89e507cafd9e5cd17ce66570ec861804d38535e61ee89bbad874b3`。其余源/工程/parser/镜像完整 SHA 见同名 JSON。
- 独立重新散列前置 1937 项清单：只有既有 `main.c`、`app_serial_runtime.c`、唯一 `gongxun-f407.uvprojx` 三项改变，其余 1934 项不变、无缺失。parser 单独核对；新增仅 observer C/H。核对四份前后差异：main 仅 include 和同 ICM owner hook，serial 仅 include/最低优先级 IO1，工程只在双目标加入该单元，parser 仅 IO1 字段/解析与分发。
- 原 auto/kinematics/odom/HWT/控制参数、到位/STOP/VM/健康/保护/看门狗源保持原 SHA。新输出只被记录读取；没有控制写回、恢复权限、新停车条件、新任务/优先级/栈/动态内存。
- main/serial 保留 GBK 可逆和 CRLF，parser 保留 UTF-8/CRLF，工程保留 UTF-8/LF；新增 ASCII C/H 为 CRLF。无批量转码或无关格式变化。

## 数学、生命周期与实际 C 证据

- 独立读最新 MATLAB 脚本、summary、日志及实际 exit 0 回执：268 项局部核；另 20 ms/50 Hz 调度回执 exit 0。旧失败/205 项阶段原件保留；未在本审查重复运行 MATLAB，不把合成数值当传感器噪声/地面精度。
- 代数核对分段线性裁段积分及 `u+C+D`、`p1+u*h+h*C+E`，局部 uint32 时间、无 future 插值、无全程 I/J/杆臂。200 样本原始 counts 均值有界累加，径向均值差与归一重力分离；单静止姿态不能识别真实横向 bias。
- 已独立只读重执行绑定最终源码的现有 ARM ELF：**R0=0，98 用例、395 断言**。测试为实际 C 核/IO1 函数配合合成 getter/时钟/TX stub，在 Unicorn Cortex-M3 soft-float 下执行；未独立重新编译，不能替代生产 M4 FPU/RTOS/SPI/UART 时序。
- 覆盖异步裁段、1 ms/250 ms 接受、0/251 ms 拒绝、h=0/200 接受及 201 拒绝、uint32 回绕、重复/缺样、累计计数 gap、参考/session/continuity 改变、无效 pose/HWT/RAW、起前非单位重力均值与非零 gyro、运动后禁止补静止均值、DONE 冻锚过期。HWT 比 IMU 晚时不回填旧历史；重力跨 gap 诊断持久保留；参考改变立即 invalid，不拼历史或伪造可信零值。
- IO1 最多每 500 ms **尝试**一次，phase40..55 ms、在既有状态/诊断之后，忙则丢帧不重试。最大格式 315 B（含 CRLF、不含 NUL）小于本帧 384 B/既有 USART 768 B；uint32 元字段与 signed 计量字段分开格式化，无效计量 sentinel 保留。
- 独立解析 ARM 真正生成的三条 IO1：WAIT_CAL/UINT_MAX age、>=2^31 tick、t0/t1 跨回绕均通过，长度 164/119/95 B；实际 CRC/sentinel/2100 mm 缩放路径通过。源码与 fixture 源/ELF及 parser 的 SHA 均一致。

## 三构建、内存及非专项窄例外

- 三份实际 Keil 构建日志均 0 errors / 0 warnings；独立验证日志、源、工程副本与 AXF 的绑定，未重跑覆盖原件的构建脚本。
- 独立解析旧/新 baseline-off 全部 SHF_ALLOC 节：地址/尺寸相同，ER_IROM1=12444 B、RW_IRAM1=25576 B。仅 RO offset6808/6830 两字节不同；独立反汇编确认 `MOVW r0,#401→405` 与 `#471→475`，对应 before/current main 中 ICM 任务创建失败断言与调度器返回 `configASSERT(0)` 的源行移动。BL/分支目标与其余字节相同。**不是 bit-identical**；接受这两处诊断源行元数据例外，不做行号技巧重构，也未放宽原控制隔离。
- 链接表：128 点历史 1536 B；observer 模块 RW96+ZI1828=1924 B；IO1 静态缓冲 384 B。ICM task 原分配 1024 B，已知静态调用链 592 B（Step frame328 B）；IO1 链 376 B+printf 未知，TaskA 历史最大仍 768 B+未知。数字不包含异常/上下文开销，不宣称动态余栈或时序通过。

## 修复、交付边界与未闭环

预审的 IO1 busy 重试和 uint32 打印问题已修正；实际 C 测试同时发现并关闭起前 float 均值累积误差及参考改变后旧 valid 暂留问题。最终源/回执对应这些修复，没有遗留阻断项。

本次是**局部补丁回退点，非完整工程 checkout checkpoint**：四份接线文件原已含大量既有 dirty 前提，直接分块暂存也无法使 HEAD 获得这些前提。保留真实工作区接线及 before/after/base64/精确 integration.patch；独立核 base64 与当前/前件所有字节和 SHA 一致。拟提交新 C/H、准确局部补丁及证据/本次交接增量，不混入旧 dirty，不声称 checkout 此提交即可独立构建。本检查者没有 Git 操作。另读协调者 `checkpoint_replay_verification.json`：新 tmp 副本精确复放及实际工作区 reverse --check 均 exit 0、六件 SHA 与 after 一致；本检查者未重复执行该复放。命令必须带单次 `git -c core.autocrlf=false`，base64 原字节不依赖 checkout 换行。首轮默认 autocrlf 只在 tmp 将工程 LF 转 CRLF 的失败尝试原件保留，不算精确回放通过，未修改用户 Git 配置。

已核 R RUN-01/PWR-01/MECH-01/02，E LOC-01/02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04 继续待技术答疑确认。未闭环：实物三轴符号/完整标定、起前静止真实性、200 Hz 完整性/50 Hz 真实开销/串口负载/余栈、长期 idle/session 姿态漂移、振动/滑移与地面沿程真值/收益。本版不修累计滑移；无杆臂且不能用与轮位置接近证明实尺右偏消失。新镜像未下载；旧固定 SHA/地址 176 B reader 不得直接用于新候选。
