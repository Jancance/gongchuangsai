# 位置速度版Watch整理与下一次只读（E，2026-11-01）

用户新图：VM1/lift4、carrier4；DM read state/result/valid0，point_enable/request0、角0、jog0。本次尚无DM有效读取，角0不当机械原点，motor_state0在valid0时也不当新鲜失能确认。carrier4证明软件已进入启动0保持，不单独证明物理槽对齐或最终二进制版本。原图/SHA在dm-posvel/watch-cleanup/intake.json。

当前只需Watch1；保留/新增列表见同目录watch1-variables.txt。重复g_carrier_request和g_dm_point_enable各留一行。Watch2可留取放target、原始角、运动起止/参考/误差和伸缩变量备用；Memory1、Core Registers、Call Stack+Locals可关闭，当前不查内存/寄存器/调用栈。移除Watch表达式只是界面整理，不改变量，也不用清旧状态。

下一项只写g_dm_can2_read_request2一次，保持F5运行约1秒。VM/lift条件已见，不重复设置；point_enable0/jog_request0保持，carrier4保持0度，不为释放会话request2。回DM state/result/feedback_valid/ctrl_mode/logical_position_deg/motor_state。成功候选4/1/valid1；只读可能显示mode1（首次动作入口才临时切2）或mode2，不凭mode1认定失败。之后依新角继续右侧实际点位；本次不旋转/下探/伸缩/回零，不拿历史177.833679当已示教目标。

本轮仅更新Watch清单和接续记录，无代码/编译/助手硬件命令。已核R RUN-01～04、PWR-01、MECH-01/02；E RUN-06、LOC-03、CAL-05，用户动作无期限等待决定保持，官方Q待技术答疑确认。未闭环：新鲜DM角/模式、实际mode2切换与300速度/保持效果、真实取料点与完整转运。
