# 135°相机独立候选部署

来源是2026-10-26用户取回的perf2现场完整源码；不是从旧简化程序重写。现场black1/perf2与本地发布源逐文件相同；perf1相较后两者的代码/活动配置差异只有检测器和颜色配置新增候选分支。保留六模式、四工具、六色HSV、跟踪、150ms主机年龄拒绝、500ms无新read退出和禁用串口。

## 候选与边界（E）

候选output/p2_vision_generic135_20261026/vision_app，ZIP同名。原包和三个现场快照保留。GENERIC135_MANIFEST.json记录来源哈希和变更；原120°配置另存reference_config/board_generic120_20261026。旧manifest/README保留作历史来源，新运行参数以135°配置为准。

独立profile generic_135_1920x1080_20261026_v1，请求1920×1080 MJPG30；曝光166/增益64，同一次采集中开启AWB读取7秒→关闭AWB再读2秒，不手动写色温4600。USB身份不能区分旧120°与新135°镜头，必须由使用者确认连接的实物。图像暂不旋转，以保持已采证据及像素坐标一致。两次静态启动及10秒读取30.11fps证据不代表主程序性能或任意光照稳定性。

新内参文件只有calibrated=0元数据；未标定默认undistort=false。主程序/四工具在请求矫正时必须校验135°profile ID、完成标记和分辨率，不接受同尺寸旧镜头内参。物料ROI清空。圆环/转盘/叠放像素搜索结构与算法候选参数仅供预览，全部标记未验证，不把旧参考矩阵、像素尺度或工程默认尺寸当本机几何。毫米/串口/抓取许可仍为false。六色HSV暂保留供对照，待135°实物困难集重验。

## 唯一下一项：板端配置检查

Windows PowerShell上传：

```powershell
scp "D:\Codex\工创赛\output\p2_vision_generic135_20261026.zip" orangepi@10.252.117.211:/home/orangepi/
```

在香橙派仅当目标目录不存在时解压（已有目录时不覆盖，使用新名字）：

```bash
test ! -e /home/orangepi/p2-generic135-20261026 && python3 -m zipfile -e /home/orangepi/p2_vision_generic135_20261026.zip /home/orangepi/p2-generic135-20261026
python3 /home/orangepi/p2-generic135-20261026/vision_app/run.py --check
```

输出应为image_size=[1920,1080]、serial=DISABLED、undistort=false、metric_geometry_verified=false、motion_allowed=false及135°profile。该命令不打开相机/GUI/串口，回传完整输出后再安排实时预览。禁止同时运行两个相机程序。

## 需求与待验

R核VIS-01（六色保留）、FIELD-07（如启用补光仅垂直向下）；E核VIS-02/03/05/09/11/12/13/15、CAL-02/03/05、RUN-06。官方QR-04等Q仍待技术答疑确认。未闭环：板端配置检查、实时GUI、六色及新机像素搜索参数、处理时效/真实故障恢复、内参/残余畸变/毫米与随臂变换、固定动作组前置验收及实际抓放。证据参见同日board-source/source_review.json及135°validation.json；软件验证不签署以上实物通过。
