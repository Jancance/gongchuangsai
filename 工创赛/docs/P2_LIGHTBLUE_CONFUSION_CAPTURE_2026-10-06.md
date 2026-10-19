# V6浅蓝偶发蓝色的直接取证

输入20261006-094547-039797.zip，CRC通过，SHA256 `760a63736fd29e1e1e2187e47b107a80e8705284c5a76350a736441f1fc0e2dc`；检测器与配置字节均匹配V6，确认已应用。统计存于`output/lightblue-v6-live-review-20261006/report.json`。

697结果中603次lightblue、45次blue、14次额外red候选；41个SEARCHING、1个启动STALE_RESULT。用户认可多数情况效果，但蓝/浅蓝未完全分离，7次浅蓝可见编号变化、最长浅蓝缺失4.148秒，不能冻结为完整连续性通过。首个异色JSON和图是手部red区域，不是变blue的时刻。保存的最长段前后图也不能替代45个blue结果的原图，不从颜色票数猜测所有错分的可见顶面/侧面。

本轮仅改`scripts/p2_color_follow_audit.py`：除原首个异色图外，为每个不同实际候选颜色单独保存first-<watch>-with-<other>，因此red候选先出现后仍能保存first-lightblue-with-blue。事件是候选共现记录，不自动断言多物料场景的其他颜色错误；当前用户单浅蓝场景由原图人工判定。实际标签、原图、阈值、检测/跟踪程序与150ms/500ms门均不修改。

3项记录器测试通过（新增red先出现不吞掉blue原图回归），颜色入口--check正常。未重复此前98项算法测试，因为本轮未改算法或颜色配置。新传输副本SHA256 `1f167db00a3cb1ff670cde56e718eb290ee80135b18e4e46877fd9345d39ff36`，仅更新取证入口，未改变V6候选包。

Windows PowerShell：

```powershell
scp "D:\Codex\工创赛\output\p2_color_follow_audit.py" "orangepi@10.31.154.211:/home/orangepi/p2-integrated-20261003/"
```

关闭旧预览，RealVNC香橙派终端：

```bash
python3 -u /home/orangepi/p2-integrated-20261003/p2_color_follow_audit.py --watch-color lightblue
```

只放同一浅蓝物料，保持底面贴桌面、相机/灯/参数不变。静止5秒，再复现此前会变blue的侧面轻推方向。首次看到blue后按Q退出；后台记录器会保存对应原图，无需用户赶着截图。回传Evidence ZIP。若这次没有复现，不伪造或人为拿深蓝替换，保留该轮无事件的日志。

已核对VIS-01（R）、VIS-02/03/09/11/12/15、RUN-06（E）；QR-04（Q）仍待技术答疑。直接蓝错色原图、六色困难集、手部负样本、真实身份/毫米/机构/场地未闭环。此步仅桌面取证，无新的识别阈值调整或机械动作。
