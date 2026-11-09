# F407工程入口统一（E）

日期：2026-10-30。用户要求“为什么有俩个？给我修改好”，本轮按唯一入口整理。

## 当前唯一入口

`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`

工程根目录只保留这一个uvprojx。`gongxun-f407-carrier.uvprojx`及同名uvoptx、uvguix.Tenderness已移出工程目录，归档到`evidence/p2-arm/2026-10-30/project-entry-unification/archived-duplicate-entry/`，没有删除其内容。

两个入口此前用于绕过旧项目回写，用户本次明确要求合一。合并前实读两份项目字节相同，均有CARRIER包含路径、carrier.c/carrier_pwm.c和头文件引用，全部源路径存在。用户关闭Keil后继续整理，避免旧窗口再次写回；最新carrier版的调试选项及Watch/窗口布局复制到原名uvoptx、uvguix.Tenderness，旧设置和新设置均有备份。原lenovo布局保留。

Keil关闭保存时main.c仅统一了CR换行字节，去除CR后与初始快照完全相同。该变化已审计并保留用户保存版本，副本重新构建；助手没有修改实际C/H源码内容。合并时的817个C/H文件前后SHA保持，0/120/240°标称候选、默认请求0及其他机构逻辑保持。

## 验证及证据

副本、关闭Keil保存后的副本及最终实际唯一入口完整Rebuild均0错误0警告，Code59724/RO608/RW2264/ZI26016，明确编译carrier.c及carrier_pwm.c。实际目录最终只剩gongxun-f407.uvprojx，重复入口相关文件均已归档；现用项目和配置头SHA完成复查。

证据目录`evidence/p2-arm/2026-10-30/project-entry-unification/`保存initial、before-merge、重复入口归档、换行审计、manifest/review及构建日志。无C/H行为变化，不重复49项状态机检查；旧49项逻辑证据继续保留，不当新硬件验收。助手未烧录或发送硬件命令。

当前接着执行`P2_CARRIER_ZERO_SLOTS_2026-10-30.md`的0°安装步骤，只从原名工程编译/下载。旧独立入口的历史修复记录仅供追溯，不再作为打开工程的指引。当前准备脚本默认项目名也已改为原名。

R已检查RUN-01/02/03、PWR-01、MECH-01/02；E已检查RUN-06、LOC-03、CAL-05。本次是工程入口/调试设置整理，不修改规则或运动参数。尚未闭环：用户侧重新打开统一入口、本版0°运动/机械安装对齐、120/240°实际槽位/方向/范围/反馈、盘臂与负载/抓放及其他轴原有开放项。官方Q保持待技术答疑确认。
