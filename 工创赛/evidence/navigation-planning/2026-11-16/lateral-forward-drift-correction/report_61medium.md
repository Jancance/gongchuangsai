# PLAN9向后4%候选：本地软件完成，未下载/实测

2026-11-16，实际6.1-sol medium执行。按xhigh冻结计划，先实际MATLAB通过再改产品源码；没有新增下载、复位、串口或发车，没有fetch/commit/push。原已下载0d5e2b左移候选、两趟2032RAW原件和d7d66e前进候选完整保留。

## 改动与边界

五个文件保持GBK/CRLF：app_config.h、motion_trial40.h、chassis_kinematics.h/c、chassis_auto_test.c。source_before逐字节备份，source_changes.json记录前后SHA；CRLF-aware diff --check通过。

- 独立APP_LEFT40_DRIFT_TRIM_ENABLE默认0；候选隔离构建显式1，不新增UART协议或运行配置。
- 仅baseline40、PLAN9、方向1、102预算、2100目标、非route路径进入新混轮；前进PLAN8/100与旧20cm/s LeftTrim不改。实际generic命令分支显式#if隔离，新macro关闭或非baseline时无新调用。
- t=round(.04*b)，b=0..102；输出[b−t−h,−b−t+h,−b−t−h,b−t+h]，HWT仍原有±min(b,2)。这是随基速变化的向后前馈，不是地面前后位置闭环。t峰4RPM、单轮峰108RPM，反馈限130保持。
- 总连续预算先除1.04再扣原随动余量，Aref=128.10392694/Jref=240.08101160；左参考400/目标2100、距离scale和到距/STOP规则不改。公共向后项在当前左轮投影中抵消，不扣命令积分，不增加spread保护或停车条件。
- 旧Chassis_Baseline40Budget/Commands/Chassis_LeftTrimCommands切片字节一致，protected_functions_check.json保存证据；高独审另核距离计算函数切片一致。

## 实际验证

MATLAB R2020a最后补充执行13:24:00.591–13:24:10.425 exit0，actual_matlab_receipt.json含最终脚本SHA。完整2032RAW/趟均读入且检查时序/quality，轮+HWT分段与原始加计均值另列，不积分冒充真值。

- 左参考峰400、七段积分2100、起终v/a为0，profile约8.90605s，+2s<12.779s。原profile约8.76995s，仅旁证内部加减速变慢。
- b0..102与heading−2/0/+2共309边界，左投影保持b、前向=−t、峰108；共同时间正交累计轮位移代数检查通过。
- integer_trim_jumps.csv有8个trim变化、每次1RPM；adjacent_command_changes.csv和integer_profile_wheels.csv对照原/新整数命令。h=0、trim阈值相邻b增1时，某些新轮增2RPM而旧轮增1RPM。低基速纠偏本身也有变化；这些整数台阶没有连续有限jerk保证，不能用连续预算掩盖。未建立地面动力学模型，不能声称85/95mm前偏已改善。
- actual_c_checks.json：真实入口/运动helper/kinematics/capture源码ARMCC+Unicorn，trim_on4443、trim_off4443、非baseline45断言全PASS。核PLAN8/9错配拒绝、向后公共项/左投影、base/heading/非法输入/零速、旧前向100接受102拒绝、原STOP/terminal尾窗。fixture未真实运行RTOS/CAN/电机；实际STOP仍原CAN1_StopMotor分支，未新增补退。
- nonbaseline_compile.json/log：实际chassis_auto_test.c以BASELINE40=0、新macro默认0单源ARMCC编译return0，无warning，核默认入口编译边界。

## 候选和回退证据

正式ON候选：`D:/Desktop/工创赛/工创赛/tmp/lateral-drift-trim-build-20261116/left210_trim4/Vehicle_Integrated.axf`，SHA256 `73b4fad295ac044af9002e93e65858d7bc87165543557cd034a7a758406224ce`。单一Vehicle_Integrated隔离构建0错0警告，Program Size Code108884/RO6596/RW3064/ZI188972；offline_elf_check.json确认bank A34020/B33600、固定地址及尾176B不变。

defaultOFF同Integrated构建0错0警告；defaultoff_load_comparison.json证明PT_LOAD与此前0d5e2b候选逐字节相同，长度115340、加载段SHA `ddc6dd63e925ec44921290f726359bb12377b78736916a1927dfd572dd2680f5`。AXF整体SHA可因调试元数据不同而不同，不误称整个文件字节相同。早期OFF加载段+8B的局部恒false分支问题已用预处理隔离修正，最终比较为true；无需改旧固件/证据。

## 尚未闭环

本候选未下载、未实车。beta4%是两趟同向前偏量级支持的首候选，不是最优标定；手量不确定度和同窗仍未知。左欠距35/20mm与停止延迟没有本轮改动，未承诺新前馈改善左距；KF/Mahony保持不变。必要后续实车授权与现场条件由root统一处理，当前无新硬件授权。

已检查R RUN-01/PWR-01/MECH-01/02；E LOC-01/02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04仍待技术答疑确认。所有4%参数、108RPM和A/J分配属E，不是正式规则。
