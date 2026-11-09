# 原R圆环模式的像素坐标取证

## 当前现场证据与日志问题（E）

用户2026-10-27提供VNC图：RING_STATION模式，三绿色十字落在三张纸面圆环中心附近，约22.9FPS；纸面可见数字从左到右为3/2/1，当前circle1/2/3只是配置排序，尚非数字OCR或最终编号映射。随后终端日志R模式持续约22.5～23.2FPS，仅输出Metric=UNVERIFIED，未输出Centers。截图支持三环像素检出，不构成逐帧稳定性或毫米精度验收。

已查明main.py的print_runtime_status在ring/stack未标定时直接打印简化状态并return，后面的Centers格式化分支不会执行。此前要求用户在这种状态下寻找Centers不适用；不要求将ring_geometry_verified/calibration_verified改为true来开启日志，也不把UNVERIFIED当三环未检出的证据。

## 独立脚本

scripts/p2_ring_pixel_audit.py，发布副本output/p2_ring_pixel_audit_20261027.py。它导入用户已部署的完整lightblue1程序，在初始化结束后切原R模式，并在原process_ring_station前后记录原图、叠加图、原检测结果。原程序目录/代码/配置、原Gitee圆环算法、相机参数、串口禁用及动作许可不变；脚本不保存毫米/纠偏量或声称原stable标志通过。

预热后从首个R处理结果开始计时，默认10秒，结束自动退出，Q/Esc可提前退出；其他模式键在这次有界取证中忽略。逐帧记录read完成时间、检测耗时、结果主机年龄、三个候选圆心及统计可用性。只将三圆心齐全、有限且在图像内、主机read完成时间严格递增、结果主机年龄0～150ms的记录纳入像素统计。失检/过期/时间戳缺失或倒退时不输出旧可用坐标。150ms是工程统计门，不是曝光到结果总延迟或官方竞赛规则。

文件包含observations.jsonl、run.json（每个圆心X/Y中位、极差、相对中位绝对偏差95分位）、主模块/记录脚本哈希、圆环/相机/项目配置及来源哈希。最多四类事件各保存首组raw/overlay原图，总量有界。图片编码发生在当前结果年龄测量后，仍可能扰动后续采集时序；不把取证开销当零。完成记录自动打包ZIP。排序仍取原ring1_side配置，不从上下颠倒的数字假造编号识别。

5项软件检查通过：新鲜坐标统计与ZIP字节、过期/缺失/倒退时间戳拒绝、失败/非有限/越界坐标不沿用、计时/图像数量有界、原入口确实切R并调用原检测函数/打印Centers。独立--check通过，1920×1080/135°profile/serial=DISABLED/motion_allowed=false。tests/test_ring_pixel_audit.py可重跑；软件检查不是板端实测结论。

## 用户下一项

此前主程序已退出。纸、相机、灯光保持不变，物料已移开，完整三个圆环可见。

Windows PowerShell上传一个小脚本：

```powershell
scp "C:\Users\Tenderness\Documents\ChatGPT\工创赛\output\p2_ring_pixel_audit_20261027.py" orangepi@10.252.117.211:/home/orangepi/
```

香橙派VNC终端完整启动：

```bash
python3 /home/orangepi/p2_ring_pixel_audit_20261027.py --app /home/orangepi/p2-generic135-lightblue1-20261027/vision_app --seconds 10
```

不需按R，预热后自动进入R并记录10秒。保持纸/机位不动，结束回传终端Evidence ZIP精确路径；记录输出目录/home/orangepi/camera135-ring-pixel-diagnostics。若文件缺失/程序异常，保留报错，不重装或改标定开关。

已检查SCORE-01（R）、VIS-04/05/07/12/13/15、CAL-02/03/04、RUN-06（E）。原项目为工程实现来源，不新增规则；官方Q继续待技术答疑确认。未闭环：板端完整像素统计、圆环编号映射、不同位置/光照/遮挡、内参及工作平面毫米/方向、真实落环。当前只推进圆环坐标，不推进已停用的物料中心track1候选。
