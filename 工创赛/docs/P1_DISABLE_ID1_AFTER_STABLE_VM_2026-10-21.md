# 稳定供电后单独验证1号失能准备命令

前置：通用模式四宏0、四轮接回且机构VM继续断开；用户实测VM22V和灯持续亮，原Watch两fault0/guard ready1/cause0/valid0F/flags全03。证据vm_four_wheel_22V_steady_user_pass.json及vm_four_wheel_static_guard_ready.png/json。此为静态供电局部通过，不签运动/满载峰值或热设计。

本步不改代码、不下载、不触发速度/位置/自动会话，只用当前唯一CAN任务中的g_can_disable_request入口，地址1为左后。实际main已核：请求消费后检查VM/两fault/Safety ready/CAN guard和原运动诊断互锁，通过才调用CAN1_SetMotorEnable(address,0)，原F3 AB 00 00 6B帧、非02返回8、保护不改。此接口会松轴，车保持平整地面防止溜动。

1. Watch2补g_can_disable_address/request/result/reply、g_can_poll_flags、g_can_guard_ready；旧诊断可保留。若当前VM已关闭，在Run中确认两fault0、Safety ready1，按下保持外部VM开关，将g_vm_test_request写1一次，连续Run至少3秒；确认灯持续/guard ready1/valid0F，否则结束供电发图，不进入失能。
2. 保持Run和供电，确认g_can_disable_address=1、request=0，只把g_can_disable_request写1一次，等约2秒。不写enable/jog/auto/heading或其他CAN动作请求，不按PC13。
3. 在仍供电时截图disable result、展开reply的ExtId/IDE/RTR/DLC/Data，以及poll_flags四项、guard和两fault。预期result1、ExtId0100/DLC3/Data F3 02 6B，随后poll_flags[0]最低位为0；不强求其他位固定，也不凭02判真实失能完成。失败/保护不复位重试，截图首次锁存及原帧；出现异常先松开外部VM开关。
4. 记录后在Run中请求VM0并松开开关，不用Stop代替断电，不以关电后flags判断命令生效。

R核RUN-01/PWR-01；E LOC-03/05/07、RUN-06、MOT-04，官方Q待技术答疑确认。用户实物结果待回传；原F3 E2根因与恢复SESSION后的真实转角/停车和X/Y仍未闭环。助手未修改实际源/操作硬件。