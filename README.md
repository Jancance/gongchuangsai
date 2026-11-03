# 工创赛整车整合工程

2026-11-03整合1交付：八段现场调车、DM4310/伸缩并行启动与统一源码参数入口。

- Keil工程：[gongxun-f407.uvprojx](3.stm32f407工程/gongxun-f407.uvprojx)，目标Vehicle_Integrated；离线目标Vehicle_Offline。
- 日常参数：[vehicle_parameters.h](3.stm32f407工程/USER/vehicle_parameters.h)，Keil分组00_现场参数，中文注释。
- [现场调参、保护保留/清理及八段步骤](工创赛/docs/VEHICLE_FIELD_TUNING_2026-11-03.md)。
- [本次上传记录与验证边界](工创赛/docs/VEHICLE_GIT_PUBLICATION_2026-11-03.md)。
- [规则基线：R/E/Q](工创赛/docs/competition_requirements_2027.md)。

现场仍使用用户指定的D盘实际工程。本仓库是该整合源码快照；请Rebuild当前目标，不使用旧历史构建产物。当前伸缩/升降参考与后续工位参数仍需现场填写，真实物料/视觉/库存、八段实测和180秒演练尚未完成。软件检查及构建通过不代表整车验收；官方Q继续待技术答疑确认。
