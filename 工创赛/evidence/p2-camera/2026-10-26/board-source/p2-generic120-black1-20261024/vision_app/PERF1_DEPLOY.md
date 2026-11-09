# 120°相机性能修订perf1：隔离候选与板端诊断

## 原因和证据

2026-10-23用户回传香橙派对已存原图进行15次分析的cProfile结果：总2.708秒，black_edge_recovery累计1.252秒（约46.2%），六色_mask_from_hsv累计0.588秒（约21.7%）；默认OpenCV8线程，图像1080×1920×3。这是重复单张图、无历史提示的离线剖析，含剖析开销，不等于实时帧率或端到端时延。此前线程对照8线程12/20过期、单线程9/20过期且尾部未改善，不将线程数作为已验证修复。

## 本次修改（E）

从已部署p2_vision_generic120_20261023衍生隔离目录output/p2_vision_generic120_perf1_20261023/vision_app，交付日期为2026-10-24。只改三个源码文件：

- project_detector.py：黑色边缘的9偏移×72角度计算使用相同公式合并数组计算；黑票低于原65%门时直接计入原颜色拒绝计数，不再计算无法改变拒绝结论的竞争色票；六色联合蒙版使用副本原位OR。
- 颜色标定.py：首段inRange直接作为蒙版，避免与全零图做OR；后续原位OR。包括空范围行为仍保留，所有HSV阈值不变。
- project_follow.py：保留分析后150ms门，并在跟踪/预测/结果组装之后再次检查，过期清轨清目标。最终检查之后仍有函数返回/调用方开销，不把它宣称为传感器曝光至结果的硬实时保证。

不缩放输入、不裁掉画面、不删除黑色/蓝浅蓝恢复、碰边与歧义拒绝、不延长门限，不设置单线程。相机/颜色/所有其他配置逐字节保持，全部模式/工具与串口禁用保留。原目录/源包不覆盖。PERF1_MANIFEST.json列三文件前后哈希。

## 软件验证和实物边界

本机证据在evidence/p2-camera/2026-10-24/perf1/validation.json、regression.log：9项测试，覆盖六色蒙版、两张新相机原图、历史六色、真实黑色反光/接触恢复、黑蓝票重叠、蓝色高光不增黑、双黑歧义、空场旧框、颜色策略开关及最终时效复核；比较优化前后完整检测结果相等。历史960P图只是算法回归，不代表新1080P标定。另含六模式离线冒烟、四工具帮助、--check、编译及ZIP解压检查。Windows离线计时只用于本机对照，不能迁为香橙派提速成绩。

## 当前唯一板端门：同场景20次诊断

Windows PowerShell上传：

```powershell
scp "D:\Codex\工创赛\output\p2_vision_generic120_perf1_20261024.zip" orangepi@10.124.145.211:/home/orangepi/
```

IP是用户上一轮回传值；若已换网络先用hostname -I确认，不以该地址长期固定。香橙派终端执行：

```bash
python3 -m zipfile -e ~/p2_vision_generic120_perf1_20261024.zip ~/p2-generic120-perf1-20261024
python3 ~/p2-generic120-perf1-20261024/vision_app/run.py --check
python3 -u ~/p2_camera120_follow_diagnostic.py --app ~/p2-generic120-perf1-20261024/vision_app
```

若新目标目录已存在现场修改，改用新目录名避免覆盖。关闭其他相机程序，保持相同物料/安装/环境光/补光OFF，诊断20次自动退出。期望--check仍1920×1080、undistort=false、serial=DISABLED、metric_geometry_verified=false。回传诊断完整输出，检查实际state/耗时/目标，失败如实保留；不据本机速度预期通过。该诊断无GUI/运动，不能验收实际带GUI帧率、长期稳定、困难六色和曝光至结果时延。

保留~/p2-generic120-20261023原工作版本。回退时退出新候选，按原命令启动旧目录；不能同时占用相机。先诊断，不直接进入抓放。

## 需求检查和未闭环

R：VIS-01六色、FIELD-07补光方向。E：VIS-03/09/12/13/15、CAL-02/03、RUN-06，保留质量/歧义/新鲜度与分层证据。Q：QR-04等官方未决项仍为“待技术答疑确认”，不因性能优化修改规则。

板端perf1实时耗时/带GUI预览、长期稳定性、新相机六色困难集、原始板端profile文件取回、内参/毫米/随臂补偿和动态抓放均未闭环。所有硬件验收和动作权限保持未放行。构建脚本scripts/build_p2_generic120_perf1.py，验证/打包脚本scripts/validate_p2_generic120_perf1.py。
