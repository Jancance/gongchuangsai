# 区2圆角位姿滞后复核

2026-11-18，本机 MATLAB R2020a 退出 0。复跑源 `validate_p4_corners.m` 得到 279 项断言、25 个非零圆角和 8 段模型，再用 `check_position_lag.m` 复核 9 个场内冻结圆角。MATLAB 首次启动有用户首选项权限和工具箱路径缓存警告，两个批处理均完成。

- 冻结圆角用 80 ms 晚反应做包络膨胀。P2 运动模型用当前 tick 前 120 ms 的四轮/HWT 共同位姿。只增加 40 ms 的膨胀项时，段 4、5、7 在 (1200,1200) 的净距下界由 2.8306 mm 变为 −0.6781 mm。结果表示原证明失效，轨迹本身尚需重新验证。
- 以 120 ms 位姿滞后加 80 ms 反应合计 200 ms 进行保守敏感性检查。保留原曲线，按整批增加圆角通过及制动时间，重新扣除入角、弯内、出角延长和量化反馈余量。9 个场内圆角有 8 个存在正下界，三个中央角的候选各为 86 个 80 ms 批次，角内速度下界 24.27 mm/s、净距下界约 0.22 mm。段 1 在 (1200,240) 的短出段未找到同时满足净距、制动延长及量化反馈至少 12 mm/s 的候选。
- 这是冻结几何、量化和时序假设的保守证明筛查。运行接入仍须将曲线参考与 `sample_ms` 对齐，联验实际逐轮错时施加、轮响应、位置计数和 HWT；再核圆角任意前缀的跟踪与制动。候选未作为固件参数使用。

复现命令在 MATLAB 中从 `chassis-completion-grill` 目录运行。

```matlab
addpath('p4-corners')
vectors = fullfile(tempdir, 'zone2_p4_corner_vectors.json');
validate_p4_corners(fullfile('p3-path','p3_path_vectors.json'), vectors)
check_position_lag(vectors)
```

R 核 FIELD-06、OBS-04、RUN-04、RUN-07。E 核 FIELD-04、LOC-05/06、MOT-01～04。FIELD-05、OBS-03 待技术答疑确认。无设备操作。
