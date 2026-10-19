# 红色物料接触失检诊断

日期：2026-10-06。来源为用户对集成F移动测试的反馈：框在手触碰物料时消失，手离开恢复，移动后停止时编号加1；用户明确手没有遮住顶面。此为观察事实，不能直接写成反光根因。

## 诊断边界

当前project_detector将六色掩膜合并后筛选轮廓，碰边、形状、非饱和像素数量和颜色票数均可拒绝候选。手不遮顶面仍可能影响可见侧面、阴影、掩膜连通与圆形比例，但尚无瞬间原图。project_follow还会以150ms主机年龄拒绝输入/结果；内部跟踪最多暂存500ms，重新匹配失败、过期或时序重置可分配新编号。这些是代码路径，不是本次失检原因判定。不能通过延长时间门或无证据保留编号宣布解决。

## 新交付

独立入口`scripts/p2_integrated_follow_audit.py`，可传输副本为`output/p2_integrated_follow_audit.py`。默认加载已部署的`/home/orangepi/p2-integrated-20261003/vision_app`，直接使用原MaterialFollower及原颜色配置；只在检测后、绘图前记录诊断，不改变阈值/轮廓/ID/预测/150ms门及DisabledSerialLink。普通工作台全部原功能不变，诊断时保持F模式。

每次运行建立独立follow-audits目录；保存逐帧observations.jsonl（目标ID、状态、候选原因、内部暂存轨迹、主机年龄与检测耗时）、颜色/相机处理/几何配置快照和哈希。首次可见、失检、恢复及换号各存原图和前后JSON，换号可跨过空目标帧识别。只保存首次每类事件的图像，后续事件仍计数并记录逐帧结果。退出后生成run.json与同名ZIP，输出Evidence ZIP路径。

同步图像保存会增加诊断开销，记录diagnostic_io_ms；本工具的速度不能当正式实时性能验收。主机read完成时间不是传感器曝光时间；输出继续是桌面像素观测，不开真实串口或执行机构。没有新鲜帧时主程序的waiting/重连由原入口处理，此包装器只记录实际调用F检测的帧，不伪造等待期间图像。

副本SHA256：`9a1ee47418516ea37953d979b279f3a8ba9f1e9cbe45444e20c152f3c2ae71f4`。

## 用户执行

先退出普通相机预览。在香橙派用hostname -I查当前IPv4，Windows PowerShell运行：

```powershell
$opiAddress = Read-Host "请输入香橙派当前IPv4地址"
scp "D:\Codex\工创赛\output\p2_integrated_follow_audit.py" "orangepi@${opiAddress}:/home/orangepi/p2-integrated-20261003/"
```

然后在RealVNC香橙派终端运行：

```bash
python3 -u /home/orangepi/p2-integrated-20261003/p2_integrated_follow_audit.py
```

一个红色物料先静止5秒，保持相机与灯不变；手指接触侧面、顶面保持可见，慢推2～3厘米后松手，再静止5秒，点击Raw Camera窗口按Q退出。只做一次，以首次事件和前后原图为证据。回传Captured和Evidence ZIP输出，随后将生成ZIP取回PC供分析。不要求装机/机械臂/场地，下一步是原因判定后再选择算法或补光修订。

## 验证与基线

新增4项无硬件回归通过：原图在主程序后续绘制后仍保持before/after正确、拒绝原因和检测结果未被改写；失检跨帧换号和逐帧记录；同ID恢复与换号区分、STALE_RESULT留证；启动无红色不产生假失检、拒绝覆盖已有运行目录。证据ZIP CRC及raw内容一致检查通过。脚本编译与本地通过现有工作台--check均通过；尚未在用户香橙派实际运行诊断，不宣称根因已定位或识别已修复。

已检查VIS-01（R）、VIS-03/09/11/12/14/15和RUN-06（E）；反光或手部连通原因、连续身份、实际性能、毫米/时间/机械臂安全与正式场地均未闭环。QR-04（Q）仍待技术答疑确认。此为诊断工具，不改变R规则、Q状态或安装提醒门禁。
