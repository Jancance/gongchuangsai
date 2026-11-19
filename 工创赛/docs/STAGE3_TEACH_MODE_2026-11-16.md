# 第三段显式机械示教模式（2026-11-16，本地软件候选）

当前基线为已部署deep-Z fdbb4d2f。本轮仅修改新隔离候选stage3_cycle.inc，原工程、source流程和deep manifest保留。新候选 `tmp/stage3_teach_mode_20261116/project`，prepare/build脚本同名20261116，证据 `evidence/integration/2026-11-16/stage3-teach-mode/`。

E：显式req30仅在观察state2、IT/SBS无活跃请求及owner无pending时进入。清旧视觉pose、新建pose、align_done保持0，完成既有bodychain MotionFinish后重建wheel hold，经state4到state7配置。即使配置已全，也不能自动执行。state7继续且未真正视觉对准时，不要求配置齐全，清旧wheel-hold，复用准备state1依实际反馈回顶、伸缩参考和舵机/夹爪准备，最后X0后DM180返回state2；不从残留低Z/伸出/DM36.5直接假装观察。真align_done1继续仍按旧配置锁存执行。保留现有STOP及互锁，不增加保护阈值或永久锁。PC13和人工标定属于调试，不是正式比赛规则。

MATLAB前置05:45:52四组拓扑通过后才写实现。实际C143/owner3580/真实owner-LRC56/home80p560/group10全过；owner新增chain0/1、配置0/1进入不自动执行、旧hold13重建、低Z且伸出退出top→X0→DM180、pending/忙拒绝、STOP和原aligned路径。两Keil目标均0错0警告；仅1文件变化、src字节一致、CRLF保留。在线AXF SHA fa7b7e6f2aa6a11d19eea67a7392be7929381b88ab50ce0871df4615ace07b96；Offline17dddfe060f46446abac779542155f42302e0037c1245d531d9b30b729b690bd。manifest/checks与build.log保存在新证据目录。

本实现代理未访问硬件。是否已部署及实物结果由主代理另行记录；不能用软件模型/编译替代真实重建、深度、净距、落点验收。B3/TCP未认证，不伪O31=0通过对准；真实取放高度和中心/左右/三件仍开放。核R RUN-01/02/03/04、FIELD-06、PWR-01、MECH-01/02；E VIS-04/07/12/15、CAL-02/03/05、RUN-06；官方Q待技术答疑确认不变。仅本地，无提交推送。

## 2026-11-16 05:52 · teach版已部署，显式机械标定到槽2顶部

- 主代理05:49:40烧录teach `fa7b7e6f2aa6a11d19eea67a7392be7929381b88ab50ce0871df4615ace07b96` Verify OK；05:49:56完整193516字节读回匹配state0/fault0，05:50:22只写center-only first_item1/count1。证据目录 `evidence/integration/2026-11-16/stage3-teach-mode/`，对应时间stage3-live JSON。
- 用户完成第三版两次PC13并确认180观察无碰擦；05:51:15 watch state2/phase6/fault0。05:51:47显式enter-teach成功state7/align_done0/wheel_hold_ready1，未伪造视觉对准；05:52:12槽2载盘120° IT10完成，05:52:34 DM36.5° IT3 sequence3完成，故障0。尚未下降或夹紧；当前物料仍在载盘槽，红料实际对准确认待用户答。
- 当前软件已具示教入口，但B3/TCP、深Z实际下降及返回、实际取放高度、中心/左右/三件仍未验收。前述fdbb和815只保留历史。用户后续明确160°预收修正，进入下一独立候选的MATLAB/实现/构建/审查流程；本时刻不提前宣称已安装或执行160预收。

