# carrier.h构建错误及项目引用恢复（E）

> 2026-10-30当前已按用户要求合并为唯一原名gongxun-f407.uvprojx。双入口的项目内容目前相同，最新carrier调试选项/Watch布局已迁至原名，重复carrier项目及同名设置移出实际工程目录并归档，817个C/H保持，最终原名完整Rebuild0错误0警告Code59724。现用步骤及证据见P2_SINGLE_KEIL_ENTRY_2026-10-30.md；下方创建独立入口是历史处理，不再作为当前打开入口。无烧录/助手硬件动作，官方Q保持待技术答疑确认。

> 最新交付：恢复旧入口并成功Rebuild后，复查发现旧uvprojx再次回到缺少carrier引用的版本。因此新增独立入口`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407-carrier.uvprojx`及从当前旧工程复制的同名uvoptx调试选项，直接共用原目录源码。新入口完整Rebuild0错误0警告，明确编译两个carrier源；817个C/H文件字节保持，未烧录/助手发硬件命令。当前应使用新入口，旧入口持续回写的来源仍未独立查明，不再把首次恢复旧入口当作最终可交付状态。最终证据`independent-entry-review.json`、`independent-entry-rebuild.log`，旧入口再次回写快照`project-reverted-again.uvprojx`和原调试选项备份均在本页证据目录。

日期：2026-10-30。

用户Rebuild报告`USER/main.c(23): cannot open source input file "carrier.h"`，且日志无carrier源文件编译。读取用户粘贴文本后，与实际main按GB18030解码及换行归一化比较一致；实际main及5个carrier文件SHA均与上轮已安装版本一致。当前`gongxun-f407.uvprojx`则与添加carrier前的备份完全一致，丢失C编译器包含路径`HARDWARE/CARRIER`及5个文件引用（其中`carrier.c`、`carrier_pwm.c`参与编译）。这是本次错误的直接原因。项目文件如何恢复旧版未独立查明，旧IDE窗口保存回写仅为可能原因。

本次先备份当前项目、用户粘贴main和本机失败日志，再仅恢复上轮通过检查的项目文件；没有修改C/H源码或覆盖用户main。实际Keil完整Rebuild成功，明确编译`carrier.c`及`carrier_pwm.c`，最终0错误0警告、Code59732/RO608/RW2264/ZI26016。所有C/H源码修复前后SHA一致，未重复运行44项状态机检查；现有逻辑检查证据仍保留，不冒充新硬件验收。

备份、原因核对及构建证据：`evidence/p2-arm/2026-10-30/carrier-project-repair/diagnosis.json`、`original-gongxun-f407.uvprojx`、`user-pasted-main.txt`、`user-failed-build.htm`、`actual-rebuild.log`、`review.json`。修复脚本`scripts/repair_carrier_project_20261030.py`只允许已核对旧SHA匹配时替换项目，若用户又改项目则拒绝覆盖。

## 当前操作

1. 用户已关闭Keil并完成入口合并；现在只打开`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`。
2. 项目包含carrier.c/carrier_pwm.c及CARRIER头文件路径，最新调试/Watch设置保留；独立carrier入口已归档，不再从最近工程记录中打开它。
3. 继续`P2_CARRIER_ZERO_SLOTS_2026-10-30.md`的0°摆位/1号安装步骤，不重复早期135°试验。助手未烧录或触发硬件；0°及三槽实物对齐待验。

R已检查RUN-01/02/03、PWR-01、MECH-01/02；E已检查RUN-06、LOC-03、CAL-05。本次仅工程文件修复，不改正式R，官方Q保持待技术答疑确认。PB1波形/舵机实际运动与停稳、三槽角度/机械范围/到位、盘臂互锁实物与抓放，以及其他轴既有行程/精度等开放项继续待验。
