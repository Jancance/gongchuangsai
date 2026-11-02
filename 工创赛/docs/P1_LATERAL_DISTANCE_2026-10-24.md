# 前进与左右横移距离接口（2026-10-24，仅本地代码完成）

## 调用

仅包含USER/motion_control.h。APP_TEST_MODE=3且APP_POSITION_DIAGNOSTIC开启，单一上层调用者与动作表互斥；不在中断内调用。请求只提交一次，原CAN任务执行，不新增任务。

```c
MoveDistanceStart(MOVE_FORWARD, 15U, 50.0f); // 前进50cm
MoveDistanceStart(MOVE_LEFT,    15U, 50.0f); // 左移50cm
MoveDistanceStart(MOVE_RIGHT,   15U, 50.0f); // 右移50cm
```

以上为三种独立调用示例，不能连续三句同时提交；每个接受后要等DONE再提交下一段。参数为方向、最高基础RPM3..15、正的厘米数10..100，无后退入口。纠偏仍沿原±2RPM叠加，因此最高基础15不代表各轮实际命令绝不超过15，原量化/限幅/30RPM反馈保护不改。返回1仅请求接受，0为拒绝，不自动重发。MoveForwardDistanceStart(max_rpm,cm)保留为通用接口MOVE_FORWARD包装。

MotionReadStatus保留厘米target/progress、reached、base_rpm和故障字段，新增distance_direction：仅distance_mode1时解释为MOVE_FORWARD0/LEFT1/RIGHT2。排队时目标/方向由新请求锁存，进度等仍须匹配后端本会话，不能串用上次结果。reached只是触发停止，DONE且底层停稳/成功/无故障才完成。继续下一段可保电，整组最后MotionFinish并等VM请求/PE2关闭；取消或故障MotionStop走原路径，不清故障续跑，当前仍按复位新会话边界。不可用模式通用API返回0。

## 横移行程计算

先将四轮带符号电机位置增量按65536/圈和名义77mm轮径换算，再按既有前进方向0/1/0/1统一为逻辑轮子行程d1..d4，数组顺序地址1左后、2右后、3左前、4右前。

- 前进进度：(d1+d2+d3+d4)/4。
- 左移进度：(d1-d2-d3+d4)/4。
- 右移进度：(-d1+d2+d3-d4)/4。

与现有X形45度麦轮正解一致，不把横移四轮直接求和导致抵消成0。对于选定方向，各轮投影后正值为朝目标方向；轮间差保护也比较投影后的行程，而不是比较横移时本就正负相反的原值。旧Chassis_ForwardPositionMm保持前进包装，新增通用纯数学Chassis_TranslationPositionMm(mode,...)；无CAN/电源副作用。

HWT继续维持起始车头朝向。剩120mm/40mm沿原8/3基础RPM减速档，估算到距触发停止；起步前/停稳后批次、int64增量、真实采样时间、400ms位置保护、最旧位置调度、跳变/倒退/轮间50mm差/超时/故障及原状态/速度/停止/看门狗全部保留。不会通过清故障或改阈值签横移通过。低速长行程可能达30秒截止，超时失败不算到距。

这些是理想轮子滚动推算，不是独立实测横向位移。滚子角度/打滑、承重和地面都会影响实际横移；不添加手量89cm对应的前进或横移补偿，不输出世界X/Y定位。当前几何77/左右212/前后206沿用户确认值。

## 动作表与遥测

MotionAction五字段保留，新增MOTION_LEFT_DISTANCE5、MOTION_RIGHT_DISTANCE6，前进距离仍4。

```c
{MOTION_LEFT_DISTANCE,  15U, 0U, 0.0f, 300.0f} // 左移目标300mm
{MOTION_RIGHT_DISTANCE, 15U, 0U, 0.0f, 300.0f} // 右移目标300mm
```

动作表用mm，公开API用cm；run_ms与relative_deg均填0。范围/预算/1至8段校验仍生效。默认APP_MOTION_ACTIONS原四段定时路线未改，横移/混合表只在隔离构建中替换，均未运行。

TM1仍64字段，action_kind4/5/6区分前/左/右距离；主机输出distance_direction0/1/2，不新增控制命令。PM1不变。终点帧升级为PE2，明确把本段方向锁存在历史记录内，避免根据另一段当前状态猜方向：

```
PE2,tick_ms,session,outcome,start_mask,end_mask,translation_mode,distance_mode,
 start_sign1,start_raw1,start_ms1,...,start_sign4,start_raw4,start_ms4,
 end_sign1,end_raw1,end_ms1,...,end_sign4,end_raw4,end_ms4,CRC
```

实际单行，31个uint32字段，CRC规则与PE1一致。translation_mode0前/1左/2右/3非平移，distance_mode0或1；距离动作不能标成非平移。准备时记录初始化为非平移，平移成功创建会话后再锁存方向/是否距离，不从其它任务临时读取。前8段历史容量、成功齐全mask15才输出、关VM后保留历史规则不变。

主机兼容旧PE1和新PE2：保留delta_counts及logical_wheel_mm；新PE2另算requested_wheel_mm和translation_travel_mm（指定方向投影后的理论平均），转向不编造平移距离；旧PE1没有方向信息，translation_travel_mm保持None，不默认猜作前进。JSONL按实际PE1/PE2标记，均不送入控制状态机或产生发车令牌。使用新固件应搭配新主机才能看懂PE2，旧记录仍可读取。

## 验证与状态

58项主机测试通过，包括原50项与新增8项PE2左/右一圈、错误方向/标志/不完整、CRC/协议隔离、旧PE1不猜方向等。新增C距离自检覆盖左右已知符号向量、反方向与非法mode，已编译并保留启动门禁；本轮没有运行MCU，不能称该自检已在硬件执行。源码投影表与既有正解/横移命令矩阵对照一致。

9个隔离完整构建均0错0警告：默认原组合、前进距离、左移距离、右移距离、前进/左移/右移混合距离、原单段定时、无线、手动、诊断关闭。公开头文件调用示例ARMCC编译通过。源与构建比对、GBK/CRLF和差异检查通过。证据lateral_distance_*_build.log与lateral_distance_review.json。主机解码/数学投影测试不能代替MCU调度、轮向/真实距离、停稳和VM实车验证。

本轮未下载、未打开COM、未发车、未上传GitHub。板上仍此前单87cm前进固件（旧210/210几何及PE1），新通用接口、PE2和212/206参数未上板；不能把历史定时横移通过扩大为本版横移距离控制通过。历史HWT偶发失效、减速cause8、距离精度和右偏仍未闭环。下一项若安排实测，只选择一段短横移（例如30cm）隔离表，先监听健康并确认方向、告知用户观察后才发车；当前未安排运动。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-01/02/04，官方Q不变。
