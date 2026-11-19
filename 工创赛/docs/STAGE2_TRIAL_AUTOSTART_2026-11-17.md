# 本轮第二段215测试自启选择修复

用户说明全断电重启后已按一次PC13却不纠偏。此前只临时把当前preview切成trial，自启仍是preview；本轮明确授权修复当前及重启后的本次测试选择，不再要求第二次按键。

## 已取得事实

02:07:19只读SSH：boot ID `42b23e92-731e-4a86-831f-6ccadf3d56c2`、uptime225.45s。实际主PID2235、相机worker2386；主进程父为`xfce4-session` PID1674，cwd统一`stage23_desktop_20261116`，argv包含`--preview-only`。有效autostart的Exec仍为原`start.sh`，Comment明确观测TX关闭。

02:07:59当前新session的真实SC2为run1/state2/prep6/item0/colors215，Pi measurement_packets_sent=0；盘缘观测有效。主代理同时读MCU rx0/state2/fault0。该证据说明重启后的preview阻止了观测发送，不能把“CSI/相机未运行”或“按键没启动”当原因。

## 最小已审变更与回滚

本地两文件在 `output/stage2_debug215_pi_trial_20261117/`，已由高档审查：

- 新`start-stage2-debug215-trial.sh`只exec原`start-trial.sh`，不发MCU start/KEEP/STOP命令。
- 原`source-rimpark.desktop`只改Name/Comment/Exec，明确当前是Stage2 debug215 trial，Exec指向该新wrapper。

02:11:03已先备份desktop、原`start.sh`/`start-trial.sh`及若存在的同名wrapper，位置：`/home/orangepi/.config/stage23-autostart-backups/stage2-trial-20261117-021103`。原preview入口`start.sh`保留且SHA `7534d7534fded0127c44e5064b4773f1eb4bbf62044aa63c2c0c94970cbd6280`不变；原`start-trial.sh` SHA `ff092b9061e1159b9b5410db4f16a24a5b76cd895c759e9bbd093205ae769317`不变。

恢复preview时，在另行明确的切换时机从上述备份恢复原desktop并按唯一owner流程切回`start.sh`；不要在机械运动中盲目重启进程。本轮没有修改HSV、top65、body-profile、算法、MCU请求或S3配置。

这只是**当前用户授权215上车测试的启动选择**，wrapper本身没有强制region过滤。S3仍暂停，实际stage3配置SHA `24cd70aa7da70c58eb53c063f1c5d1432f27e8a002a6d3b0eb3c8eb82c2760e8`、calibration_verified=false、reference/affine为null，没有启用有效O31。将来S3配置启用或改变测试范围时须重新审查选择或恢复preview，不能称为永久S2-only入口。

## 本次实际恢复与首故障

02:11:06正常TERM已重新核对的实际preview PID2235，确认旧相机/串口owner释放后启动原trial。新主3531唯一持有ttyUSB0，worker3541唯一video0，CONFIG RX=true/TX=true、motor_commands_sent=false；body-profile SHA `9e98008f…`、top65 SHA `15a93e31…`保持。

新session为`/home/orangepi/vision_tools/stage23_desktop_20261116/captures/run-20261117-021104-702488800`。恢复观测后原run1自动继续，**没有再次按PC13或发送串口启动命令**。Pi观测发送计数实际从0增长到20，SC2 state2/3多次交替，说明发送链和主控纠偏流程已推进；不将此写成纠偏精度或三件验收通过。

02:11:50 Pi记录state9/pose19/item0/accepted0/done0，未进入抓取。首故障前盘缘`error_norm_mm`约38.51→39.23→17.83→15.28→17.30→16.67→18.01→16.46，圆心y在约348/462间来回，后期x约944～949；观测有效但未收敛。该范数是像素映射误差；`right_forward_mm`是不同映射的底盘命令分量，不能混作同一范数。主控真正故障原因由主代理唯一probe取证，本子任务没有清错、重试、改保护、调参或提前认定根因。

## 证据及未闭环

证据在 `evidence/integration/2026-11-17/stage2-pi-preflight/`：`process-diagnostic.json`、`autostart-repair.json`、`autostart-repair-output.txt`、`status-after-autostart-repair-021150.json`、`first-fault-trial-021104.json`及当前桌面图。`scripts/stage2_pi_autostart_trial_repair_20261117.py`默认不联网，显式`--repair`才执行该已授权切换；只读monitor默认仅本地manifest，`--watch --duration45`由主代理独占probe调用，到期不会停车，快照非原子。

当前会话试运行已证，**尚未再次断电验收新的自启选择**；不为验自启而自行重启硬件。当前纠偏不收敛/首故障、实际同步起动差、三件夹稳放稳及100ms余量仍开放。R核RUN-01/02、PWR-01、MECH-01/02、QR-01/05/06、DSP-01；本测试选择为用户工程决定E，R未改，官方Q继续待技术答疑确认。仅本地文件和已授权Pi配置修复，无GitHub提交推送。
