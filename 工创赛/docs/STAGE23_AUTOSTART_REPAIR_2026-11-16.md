# 统一桌面现场自启修复（2026-11-16 21:12）

用户上电后确认停稳、空爪和空间，明确要求修复重启仍进旧原料区界面的问题。本次只部署已审桌面包、替换桌面进程和自启入口；未操作主控、复位、烧录或发送运动/有效纠偏观测，未修改HSV、检测算法、TCP或矩阵，不将车身当前位置建立为零误差参考。

## 根因和最小修复

活动入口 `/home/orangepi/.config/autostart/source-rimpark.desktop` 仍指向 `/home/orangepi/vision_tools/source_observe_ready_each120_pi_20261116/start-trial.sh`。PID2239从该旧目录运行并持有ttyUSB0，worker2392持有video0。统一目录当时仍为55项历史包。用户具有dialout/video组权限；没有发现权限导致连接失败的证据。

已通过复用既有授权SSH helper，单次stdin heredoc上传并核对64项ZIP SHA256 `e1a41fc7596f9fba98a11bcbbb144cbe23b57068cc3733ed9f0ef6b68bd2fe3f`，调用包内install-local.py安装并逐项hash验证。旧目录完整备份为 `/home/orangepi/vision_tools/stage23_desktop_20261116-backup55-20261116-211129`。原source目录保持原样。

原desktop及整个sessions目录保存在 `/home/orangepi/.config/stage23-autostart-backups/20261116-211217/`。只将原有活动desktop内容替换为已审模板，Exec现在为 `/bin/bash /home/orangepi/vision_tools/stage23_desktop_20261116/start.sh`，没有另增第二个活动入口。原目录内历史`.bak`不以`.desktop`结尾。已查实际`~/.cache/sessions`文件：仅xfwm4窗口状态，没有xfce4-session客户端RestartCommand文件；没有删除任何会话缓存。

精确核对旧PID2239的cwd/cmdline后发送TERM，等待该进程退出及串口/相机全部释放，再启动统一preview。无强杀、无删除lock。新PID3585唯一持有ttyUSB0，相机worker3592唯一持有video0，均来自统一目录。

## 现场软件证据

- 当前session：`/home/orangepi/vision_tools/stage23_desktop_20261116/captures/run-20261116-211218-286209543`。
- CONFIG：`mcu_status_rx_enabled=true`、`observation_tx_enabled=false`、`motor_commands_sent=false`；启动命令有`--preview-only`。
- 真实收到S31：run2/state2/pose4/item0/颜色215/align_done0/config_ready0。observations记录region=stage3、calibration_verified=false、preview_only=true、observation_wire=null。不能把界面“视觉纠偏中”解释为已经发出纠偏。
- 双视图截图已下载并人工查看：第三段左“视觉纠偏”、右“物料放置”、目标环“按主控任务映射”，底部明确未标定仅预览；实际当前三环黑白图像可见。未将颜色当目标环号。
- 64项manifest无不匹配；包外top65 SHA256 `15a93e31c5f268958c98ab3dd8e8a70e8570b2fc072b94252e78607d4d84c910`；实际入口固定引用的 `.../vision_app/code/config/material_color_thresholds.json` SHA256 `ddba739f6a3aed97a58c37ce13e52709be75b3b14859105cb983a6f0eebc664a`。本轮仅记录hash，未修改这些文件，不将top65.calibrated替代第三段标定。
- 本地证据目录：`evidence/integration/2026-11-16/stage23-offline-ready/`，包括`autostart-before.txt`、`autostart-switch.txt`、`autostart-after.json`和`live-stage3-preview.jpg`。

未执行香橙派重启，因此持久入口已改及当前运行通过，与冷启动验收分开。真实SC2往返切换本次没有触发；离线已审回放保留，不假造现场SC2。日志有单次Corrupt JPEG提示，画面和帧处理继续，不据此扩大改算法或相机配置。

## 回滚和后续

如明确选择回滚，先核当前PID/cwd，用包内rollback-preview.sh正常结束统一进程并启动旧source的绝对start-preview.sh；不能启动旧trial或同时占用相机/串口。持久入口回滚可从上述备份恢复source-rimpark.desktop，但它历史内容为trial，恢复前须明确选择；不自动恢复。55项备份独立保留，不对当前目录盲目覆盖。

下一步仍为TCP/A2/B3和当前场地新图闭环，不能直接放行有效O31或放料。中心放高、左右几何/净空、三件流程及真实扫码跨区交接仍未闭环。纯启动配置沿用已通过的MATLAB区域模型，不改变控制拓扑，无新增动力学实现。

R已核FIELD-06、RUN-01/02、PWR-01、MECH-01/02；E已核VIS-09/12/14/15、CAL-02～05、RUN-06。本修复为E，不改R；官方Q继续待技术答疑确认。仅本地文档和远端部署完成，未提交/推送GitHub。根目录Git无提交/remote，无法在此根副本执行GitHub同步；本次没有修改共享源码。
