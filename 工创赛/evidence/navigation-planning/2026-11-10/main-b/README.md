# main-B定位适配拆分证据

本轮本地基线4a90de1；用户取消重新联网核对。活动工程只有根目录3.stm32f407工程/gongxun-f407.uvprojx，冻结包不构建、不编辑。

- baseline.json：操作前HEAD、三项允许修改文件原始SHA及6324保护文件摘要。完整原始清单仅留本机工创赛/tmp/odometry_main_b/raw_baseline.json，不提交大清单。
- before/after_Vehicle_Integrated.log、before/after_Vehicle_Offline.log：实际Keil Rebuild原始日志，均0错0警告。保留原始日志行尾空格及CRCRLF，不按源码空白规则重写日志。
- build_summary.json：从四份日志提取Code/RO/RW/ZI。
- app_odometry_check.json：运行工创赛/scripts/check_app_odometry_extraction_20261110.py生成；89个main函数还原、两处Reset调用与原始C对照。脚本复用main-A函数提取辅助，不执行或改写main-A历史断言。
- review.json：编码agent自查与边界；independent_review.json为另起gpt-6.1-sol/high agent的独立审查结论和源码SHA绑定，未发现可行动问题。

本机重建使用工创赛/tmp/build_main_b.py before/after，临时uvprojx仅把源码/IncludePath改为活动目录绝对路径、产物与listing改为tmp下相对目录；原工程编译配置和源文件引用顺序保持。执行D:/Keil_v5/UV4/UV4.exe -r 临时工程 -t Vehicle_Integrated或Vehicle_Offline -o 对应日志。最终源工程仅每目标各增加app_odometry.c/.h一次。生成的目标文件/镜像/仿真unit仅在tmp，不提交二进制。

实际C检查仅stub外部航向、供电、轮样本与临界区，执行新旧适配和未改纯里程计，验证Init、Watch接收/拒绝、99/100/101ms、缺轮/故障、记录0~8/重复完成、Reset不改位姿/Watch，以及航向/供电/样本/发布临界区钩子改变live共享状态后的读取顺序。两种Reset调用另作源码逐字节还原检查。stage1=0/1各2584场景、23770断言通过。

纯行为拆分不涉及新控制模型，无需MATLAB动力学验证。没有操作硬件；离线等价和构建不等于RTOS真实延迟、栈水位、烧录、实车或定位性能通过。场地起点、continuity恢复等旧问题保持开放。R/E/Q检查及剩余项见review.json及根目录交接。
