# 平移启动临界区局部优化（2026-10-25）

当前已本地实现、烧录并完成4轮同路线实测，板为优化版最后一轮成功完成态、VM关闭。取消的转向后两段不恢复。下文实现时的未测说明保留为历史限制，实测更新见末节。

## 实现及边界

`App_ForwardHoldStep`在HR_WAIT就绪后锁内捕获动作参数和后台会话，退出临界区，调用`ChassisAuto_PrepareTranslation`完成原基本命令、平移编码及距离数学自检。该函数拒绝BASEPRI/PRIMASK屏蔽状态，仅写局部结果，没有CAN/VM副作用。

重新加锁后刷新tick、姿态观察和安全检查，核对等待阶段、故障、会话、参数及CAN/电源许可。失去就绪则留在原有界等待；阶段改变或故障则不提交。`ChassisAuto_BeginPreparedTranslation`继续在短锁中核后台状态、session、VM和原Auto_BaseHealthy门禁、端点容量；通过后复用原初始化和VERIFY_ON续段分支。没有放宽100ms邮箱/源龄、位置400ms、单段30秒及组120秒等保护。

旧接口仍可用，当前仅前端平移启动改用两阶段。旧转向路径及动作装载锁区未全局优化。新增局部准备缓冲需要现场核CAN任务栈余量。

CT2格式和故障HF2/CT1不变。scope0计每一独立临界区间，锁外准备不计入；scope1现仅为准备后提交调用，不能与旧84us的完整Begin当作同等工作量对比。scope2不变。比较应关注scope0最大屏蔽区间及是否出现ORE；嵌套区间不能相加。原105~107us仅为优化前实测，优化后暂未量到，不能据编译或模拟宣称ORE根因闭环。

## 验证与证据

同日evidence/p1-foundation目录的`startup_prepare_review.json`记录生产源SHA和三份构建：route_a左/前/右各300mm为59064/768/2020/34172；single模式2为51096/708/1948/33932；manual模式0为41848/652/1836/27892（Code/RO/RW/ZI字节）。三份日志均0错0警告。隔离组合版仅motion_action_plan.h为测试三段表；主工程默认表保留。

实际后台C及运动学通过140项断言，涵盖三方向命令等价、锁外准备无共享/IO副作用、非法/NaN参数、会话改变、VM/健康门禁丢失、活动后取消、重复提交、端点容量、续段VERIFY_ON及旧接口。源位于tmp/startup_prepare_unit_20261025/src，桩及unit.c在同目录。使用ARMCC5以Cortex-M3/SoftVFP编译unit.c和chassis_kinematics.c，armlink按unit.sct链接，再在该目录执行`python run_arm.py`；需本机visualizations下arm_test_deps的Unicorn2.1.4。ARM模拟是行为验证，RTOS/CAN被桩替代，只做顺序状态交错，不证明F407浮点时序或真实任务并发。结果见startup_prepare_arm_checks.json。

`python -m unittest discover -s 工创赛/scripts -p test_p1_serial_motion.py`共81项通过。生产三个源与各隔离构建逐字节相同，模拟相关源也核同，GBK/CRLF保持、局部git diff --check通过。

## 下一步

现场重新就位确认后，复用tmp/startup_prepare_route_a_20261025已通过的构建烧录，只跑一次同路线，核CT2三段齐全、两次保电换段、最终停车/VM0、HF2/ORE及栈余量。没有新源变化不重复全矩阵编译。不自动提交或推送。

核对R RUN-01/PWR-01及E LOC-03/05/07、RUN-06、MOT-02/04。保护与比赛规则未变；Q保留待答疑。

## 15:22四轮实测更新

已同一固件依次烧录并完成4轮左/前/右各30cm，12段、8次保电换段、最终停车及灯灭均已确认。前端最大屏蔽区间累计最大32.1369us，较优化前105~107us缩短约70%；backend提交9.8929us、动作装载最大40.1012us。本次无HF2/ORE复现，不能扩大成可靠性全部闭环。CAN任务栈余量及地面实际定位精度未验。

后两轮终态各2次命令拒绝，首次均见于完成3/3、VM0；在途KEEP及完成遥测延迟是待核解释，未修改代码。遥测丢弃和捕获无效帧均保留。四轮完整汇总见同日evidence/p1-foundation/2026-10-25/startup_prepare_four_runs_review.json，详见根交接最新章节。当前不再发车，建议后续只读核结束边界拒绝与CAN任务栈。未提交推送。
