# DAPLink下载配置补齐

当前硬件为DAPLink（CMSIS-DAP），已枚举COM17/FAED:4873/SER6AFAAE24，与历史绑定一致。新组合工程只有uvprojx，未配uvoptx；旧右向工程成功下载使用CMSIS_AGDI.dll/nTsel3、Horco CMSIS-DAP v1、UID6afaae24。新工程首次下载return2，Internal DLL Error/Target DLL has been cancelled，没有Programming/Verify/Run成功标志。

本阶段只从旧已用选项复制Vehicle_Integrated的TargetOption和元数据，去掉文件树与其他target，改ListingPath，生成同名新uvoptx。未修改固件或原工程。XML/驱动/UID与AXF不变检查通过；补齐文件是确认事实，是否为唯一失败原因尚待下一次授权下载证实。不伪称已修复下载或已装新板版。

当前download尝试1/失败1，显式reset0/START0，不构成后退运动样本。失败下载可能影响目标运行状态，不假定板仍正常运行旧版；下一次需成功安装与fresh检查。原失败记录保留，无自动retry。prepare_daplink_options.py可重建新工程旁的选项，仅写本地文件、不访问硬件。

按项目单次失败不自动重试约定，继续下载及发车需用户再次明确；该约束来自AGENTS.md第12节。R RUN-01/PWR-01/MECH-01/02，E LOC-03/05/07/MOT-02/04/RUN-06；Q FIELD-05/OBS-03/QR-04待技术答疑确认。软件配置本地提交，不推送。
