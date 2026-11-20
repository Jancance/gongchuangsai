# 旧反偏示例标注更正（2026-11-15）

本目录 `inspect_reverse_witness.m` 传入 `[3 2.5]`，而 `early_controller_cases.m` 固定将第1列解释为 noFF、第2列为 half，增强控制器仅从第3列启用。因此旧 `reverse_witness.json/csv/png` 中 `current_S3/early_S2_5` 实际是 noFF/half；不能用旧1.430796m或1.787605mm作为S3对S2.5的证据。原文件保留追溯，不继续引用其S3/S2.5标签。

完整 `run_early_batch.m` 正确使用七列 `[3 3 3 2.5 2 1.5 1]`，该矩阵及原筛选失败保留，不因单独示例脚本误标而失效。

正确示例在相邻 `s25-field-candidate/correct_reverse_witness.m`（四列取3/4），已实际MATLAB运行exit0；同一假设响应/偏置场景，S3/S2.5中段左越线1.871998/2.103615mm，绝对峰4.059300/3.639667mm。两版都可能模型越零，不是已测地面反偏。更多见 `s25-field-candidate/report.md` 与 `independent_review.md`。
