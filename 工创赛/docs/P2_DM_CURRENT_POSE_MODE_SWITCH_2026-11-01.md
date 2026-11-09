# 当前姿态首次切位置速度模式并保持（E，2026-11-01）

用户图read4/result1/valid1、ctrl_mode1、logical77.1824951°、motor0、speed300、mode_change0/holding0/jog0。接受新鲜读数，模式仍MIT1，尚未实际切2或启用保持；carrier4仅沿用软件0保持证据，不冒充精密槽对齐。

下一项保持其他轴和实物姿态不动，先填g_dm_pick_target_deg77.1825，再point_enable1，最后jog_request4一次，保持CPU运行，完成本版首次失能切模式/读回/当前位置预置/使能及原位保持。77.1825是当前姿态的临时保持目标，不是右侧物料最终坐标，不安排旧177.833679或新的45度旋转。速度上限300沿用，当前位置目标并不要求以300度/秒转动。

软件候选结果：ctrl_mode2、mode_change_state3，jog_state7/result1、holding1/motor_state1，stop_result0表示没有执行失能；实际姿态保持和有无抖动仍需用户观察。若明显偏转/异常，jog_request3并保持运行执行失能，回原Watch，不改状态或重复连发。完成后继续右侧取料几何示教，未有实物对准证据不保存为最终取料点。

证据dm-posvel/current-pose-mode-switch/intake.json及原图SHA，本轮无固件/构建/助手动作。已核R RUN-01～04、PWR-01、MECH-01/02；E RUN-06（用户无期限动作等待决定保持）、LOC-03、CAL-05；官方Q待技术答疑确认。未闭环：首次切2/原位保持实物、300速度实际表现/平滑度、取料点与完整转运。
