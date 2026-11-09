# pixel1导入颜色标定失败：候选目录最小修复

当前结果：用户板端截图已确认四个正确中文.py恢复，configs_modified=false，随后普通候选--check通过（1920×1080、135°profile、ring_pixel_status_version=1、serial/motion禁用）。证据pixel1-startup/filename-repair-board-check.png及board-check-review.json。无需重复修复；下一项启动普通R预览并回传GUI/终端，实际相机/GUI尚未验收。下文为修复过程。

用户截图：普通run.py在code/main.py第29行from颜色标定导入阶段报ModuleNotFoundError。程序尚未进入相机/GUI初始化，不改曝光、颜色阈值或标定开关。

本地发布原ZIP SHA256 0bfc2ac6e6ea4db91f0ec1747eae83e3a9b8676a99f99f2e0bfecf24710b115a，CRC通过；vision_app/code/颜色标定.py存在、34244字节，与源逐字节一致，Zip UTF-8 flag=0x800。实际板端文件清单未取得，不能仅凭截图断言CP866根因；当前按正确名称缺失处理，若已存在则保留并反馈。

参考项目早期同类部署记录的可逆做法，创建scripts/p2_restore_pixel1_names.py并另发output/p2_pixel1_restore_missing_names_20261027.py。修复先检查候选目录main.py与ZIP manifest版本、CRC、四个中文模块来源哈希、目标路径安全；全部检查后仅以排他创建模式写缺失的中文.py。现存模块即使与ZIP不同也不覆盖；原配置和乱码文件保留。输出restored与existing_preserved可区分缺失/已存在及内容一致性。

在本地临时候选目录模拟UTF-8名误解码为CP866名，4个正确中文模块均恢复；用户配置、现有用户模块和乱码文件未覆盖。4项测试通过；这些是本地修复证据，不是当前板端恢复成功。

Windows PowerShell上传小修复工具：

```powershell
scp "C:\Users\Tenderness\Documents\ChatGPT\工创赛\output\p2_pixel1_restore_missing_names_20261027.py" orangepi@robot-opi5pro.local:/home/orangepi/
```

香橙派终端运行：

```bash
python3 /home/orangepi/p2_pixel1_restore_missing_names_20261027.py --check
```

默认读取板端已有/home/orangepi/p2_vision_generic135_pixel1_20261027.zip，只修/home/orangepi/p2-generic135-pixel1-20261027/vision_app/code中缺失的中文.py，然后自动执行普通候选--check --start-ring --ring-paper-labels 3,2,1。该检查不打开相机、串口或窗口。回传整个输出；有版本/现存文件不一致或仍导入失败时，先保留错误与报告，不反复整包解压。

未来首次部署改用Python zipfile到新目录；已有目录不整包覆盖。构建器给非ASCII ZIP成员新增Info-ZIP Unicode Path字段，原程序/配置字节不因ZIP元数据更改而变化；新ZIP是否兼容板端unzip仍需实际证据，不作为当前修复前提。

核SCORE-01（R）、VIS-03/07/12/15、CAL-02/03、RUN-06（E）。本轮是部署工程修复，R与官方Q不变，待技术答疑确认项保留。几何标定仍暂缓；板端模块恢复、普通预览与性能尚未闭环。
