# 可复用校准测试程序检查点（2026-11-17）

实测镜像：right1-forward-trim-prep/frozen/Vehicle_Integrated.axf，SHA256 3043c4a98aed1b04eb0c80fd4b2ea819b2c70b52a2c0dbe60613a969bca48ad3。冻结工程与下载选项在同目录，均已Git跟踪；候选原功能提交03528388，最新导航功能3ed61df7，实测AXF不等于当前默认源码重建结果。需要复现该镜像时应从原功能提交在隔离目录恢复依赖及冻结配置，不能只拿旧工程指向今后的活动源码就声称相同镜像。

日常唯一工程入口仍3.stm32f407工程/gongxun-f407.uvprojx；冻结工程仅作候选对照。右1%仅右向PLAN10/12/13生效，左2%和前后原控制保持；未安装左nav50%合并版。

右210单次夹具及RAW读取器见right1-forward-trim-field-run1..5（fa14d5fe）；前进PLAN8见right1-forward210-field-run1..4（f317a900）；后退PLAN11见right1-backward210-field-run1..3（31cb7b1c）。这些已包含execute_once.py、read_capture_once.py、decode_capture.py、冻结串口客户端、执行绑定及回执。每次校准必须新建独立运行目录/绑定，核实际无线COM和UID、候选哈希及摆位；旧执行门全关闭，不能重用旧token/session或直接重跑已消费目录。

单次reset/PLAN/ARM/START，不自动重试；软件完成、零反馈后直接只读保存RAW双bank，读取稳定后再复位或摆车。原始尺量与排除标记分别保留。当前全部测试已结束，不触发硬件。本检查点仅本地提交，不fetch/push，不包含原6dirty或历史无关未跟踪文件。
