# P1 独立 M4F 复核

2026-11-18，复核源码 SHA256 前缀：`fusion_motion.c` 67a9efc4，`fusion_motion.h` 1d4c882c。以 ARMCC Cortex-M4.fp.sp 编译实际模块与实际 `chassis_kinematics.c`，在 Unicorn Cortex-M4 上执行，编译、链接命令退出 0，日志为空。读取 attempt7 的 22 个 MATLAB 向量，完成 1249 项检查，0 个差异。包括持续时间、制动弧长、采样 q/a/j、真实逆解 scale=1、四轮方向及 0.1 RPM 编码。独立夹具的编译、执行和两次源码哈希绑定见 `final.json`。

- 修前 P1，前缀少计反馈误差。q=a=0，目标 +1 mm/s，dt=200 ms，前方 3 mm，反馈误差上界 10 mm/s 时，修前返回 ADVANCE，前缀与制动共可达 4.2 mm。最终返回 BRAKE，并核后向、侧向及角度的对应边界。
- 修前 P1，先细分绝对速度控制点再差分造成 J 检查失真。vx=398，ax=−3.5980224609375，目标 vx=397.9721374511719，dt=20 ms 时，修前接受 20 ms 曲线，M4F 样点初始 jerk 为 301.6662 mm/s³。最终选 40 ms，记录见 `final.json`。另复核 398 mm/s、0.001 mm/s² 小斜率的起点加速度保持。
- 修前 P2，精确限速点向外小加速度被舍入掩盖，合法 400 mm/s 常量分量也可因插值超过限速并误触发 BRAKE。最终拒绝正负限速向外及收紧轮速的单 ULP 反例，允许向内。常量 400 mm/s 逐毫秒样本达峰 400，原 17 ms 假拒绝返回 ADVANCE。

最终默认纯内核未见未闭环 P0/P1。模型的反馈误差上界须覆盖“本批持旧指令与理想连续前缀”的差额。即便旧、新定点轮速相同，理想速度曲线仍可在中间变化，有界混合掩码误差不能单独覆盖该差额。P2 核查真实唯一 CAN owner 的持令时序、量化、最坏耗时、反馈、STOP/停稳与栈余量。P3 及后续负责站点、扫掠包络和实车闭环。本轮未操作硬件。

在项目根目录执行 `python 工创赛/evidence/navigation-planning/2026-11-18/chassis-completion-grill/p1-motion/independent/review_final.py` 可重放，生成的对象和报告位于临时目录。

