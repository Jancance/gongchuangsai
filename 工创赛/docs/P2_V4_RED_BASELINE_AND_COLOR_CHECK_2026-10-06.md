# V4红色桌面版本归档与其他颜色复核

用户已认可红色效果；本次读取20261006-083236-468596.zip完成归档，不继续要求同一红色问题反复调参。原ZIP SHA256：`a1ad04556b42e0e5ff6205ae4e8bc7556e14ca7eff388b46624d3da7a18267b1`，CRC通过，四项模块/诊断SHA均与已交付V4包一致。版本包和阈值不改，配置快照/统计见`output/p2-v4-red-baseline-20261006/summary.json`。

## 这轮直接证据

约39.222秒检测时间跨度，456个结果：443个FOLLOWING_PIXEL、12个SEARCHING、1个启动STALE_RESULT；所有可见红色观测使用T001，没有换号。6次短失检，最长0.373817秒，均在500ms内部暂存期内恢复；结果95分位主机年龄118.050ms，唯一启动超时183.999ms被拒绝。检测调用95分位84.529ms，取证提交最大7.224ms。ready/motion_allowed始终False、串口DISABLED。

同ID保留不等于已证明物理身份连续，短失检后仍有identity_continuity_unverified标记，预测受到限制。当前只认可用户需要的红色桌面工作效果，不签署零失检、所有角度、完整六色、传感器延迟/毫米或真实跟随抓取。其他颜色的复核必须继续。

## 新颜色记录入口

`scripts/p2_color_follow_audit.py`（传输副本output同名）复用板端V4的p2_integrated_follow_audit.py记录器；新入口按--watch-color选择red/yellow/blue/green/black/lightblue触发取证。目标实际标签保留，不把被错分的黑色替换成蓝色。保存首帧以覆盖“启动就错分/一直未知”、首次可见/失检/恢复/换号及最长段；记录模块和基础记录器SHA、配置快照，输出color-follow-audits内ZIP。检测器/跟随器/相机/阈值/时间门均不改。

新增2项记录器回归通过：蓝色→黑色错分→蓝色恢复事件留存真实标签/编号，并保存最长蓝色失检；关注浅蓝但一直输出黑色时仍保存首帧。新入口本地--check通过。未重复既有90项视觉算法测试，因为本轮没有修改算法；新工具尚未在板端运行。

新入口SHA256：`ac985a2f327a7cb31f22f67e6a253122af45e7523d61839a1bf847ae116ecdd8`。必须与已有p2_integrated_follow_audit.py同目录运行，用户板端该文件已由V4补丁更新。

## 当前下一步：单蓝色

先只放一个用户确认的深蓝色物料，移走红色和其他物料；保持相机、灯和参数不变。Windows PowerShell：

```powershell
scp "D:\Codex\工创赛\output\p2_color_follow_audit.py" "orangepi@10.31.154.211:/home/orangepi/p2-integrated-20261003/"
```

RealVNC香橙派终端：

```bash
python3 -u /home/orangepi/p2-integrated-20261003/p2_color_follow_audit.py --watch-color blue
```

静止5秒，保持顶面可见，接触侧面慢推2～3cm后松手，静止5秒，Q退出。关注标签是否blue、是否错成black/lightblue、框/编号是否变化；回传窗口截图/观察及Evidence ZIP。此步单蓝色通过限定场景后，再复核浅蓝/黑及其他颜色，不把旧版六色结果直接移作V4六色验收。

已检查VIS-01（R），VIS-02/03/09/11/12/15、RUN-06（E）。QR-04（Q）仍待技术答疑；新工作台六色/困难集、真实身份、毫米/时钟/高度、机构/P1与场地仍未闭环。保持桌面阶段，安装、机械臂和场地切换前提醒，当前两组分工按GROUP_RESPONSIBILITIES_2026-10-06执行。
