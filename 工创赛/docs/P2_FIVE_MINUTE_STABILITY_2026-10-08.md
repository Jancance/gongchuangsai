# 地面白纸背景5分钟连续运行检查

> 2026-10-08结果更新：130216-931830 ZIP实际覆盖239.189秒/5210条，全部green/T001，无失检、换号或超时；最长间隔115ms。当前只签署约4分钟观测范围，不写满5分钟。配套console未下载，下一项只补原日志，不立即重做相机测试，见[结果与下载命令](P2_STABILITY_REVIEW_2026-10-08.md)。以下保留原测试目标。

前置证据：V13六色有限样本/交叉及空场已归档，071416轮单次USB断连进入等待并自动恢复FOLLOW/green。当前仍地面A4白纸，无实物转盘、机械臂安装或执行器动作。依用户决定保留触摸/强光困难项，不继续补丁。

本项是计划C独立采集稳定性工程检查。目的为查找正常连续运行的意外断流、V4L/JPEG解码错误、超时拒绝和无操作ID重置，不作为官方5分钟规则或真实抓取验收。使用板端已有工具，无需安装：

```bash
python3 -u /home/orangepi/p2-integrated-20261003/p2_color_follow_pair_audit.py --watch-color green --sample-every 30 --max-samples 12 2>&1 | tee -a /home/orangepi/p2-integrated-20261003/follow-stability-console-20261008.log
```

1. 绿色物料完整入镜、放在地面A4白纸中部。启动后确认Mode: FOLLOW及green框；只有模式不对才点击Raw Camera按一次F。稳定后开始计时。
2. 保持物料、相机、灯及USB连接不动，连续运行5分钟。计时阶段不切模式，避免人为清历史与意外重置混淆。若有闪框、错误或等待画面，记大概第几分钟发生，保留终端输出。
3. 5分钟后点击Raw Camera按Q退出，回传末尾Evidence ZIP路径/截图及是否出现异常。后续下载该ZIP和follow-stability-console-20261008.log一并核对。无需再次拔插。

后台每30秒保存一组原图，最多12组；全部有效FOLLOW结果仍写observations。等待阶段无行，必须结合console和观测时间间隙。日志tee -a可能包含多轮，应按Diagnostic folder隔离。除已有候选建立阶段外，检查完整green数量、标签/ID、时效拒绝、意外Camera unavailable/重连、JPEG警告与进程正常退出；若不满足则保留失败范围，不因平均FPS正常签署全部通过。

已检查R：FIELD-07，只允许垂直向下补光，本项保持现有灯具。E：VIS-03、VIS-12/15、CAL-02/03、RUN-06，5分钟仅工程测试长度。Q：QR-04仍待技术答疑确认，其他正式Q不变。尚未闭环：早期额外ID重置原因、正常运行稳定性、旧帧/真实曝光与端到端时延、触摸/强光/边缘/手部误检、多实例、毫米/运动补偿、机构与场地；不宣称转盘/安装/相机verified通过。

Serial=DISABLED、ready=False、motion_allowed=False保持。进入正式相机安装/标定、机械臂或真实场地前先提醒用户；本轮不需要搭转盘或安装。无代码/配置修改，本项物理结果待用户运行。
