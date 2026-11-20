## 2026-11-17 前210四次完整结束，已转后退采集（最新）

- 同已装右1%单独3043...8ad3镜像，前进原PD/100RPM预算/400mm每秒保持，不重复下载、不改固件或控制。用户明确先测量并逐趟摆好授权；初组3次后用户另授权加测第4，全部保留不挑样本。
- 前距/右偏mm依次2105/+25、2100/-5、2112.5/+15、2107.5/+14。四次均software_complete/result1/stop_mask15/终态四轮零反馈，系统fault/linklost/rejected0；reset/PLAN8 ARM START各4、KEEP175、STOP0。全部运动后直接保存RAW，不再等停稳确认，2026/2025/2025/2025双bank稳定双读、quality0/源drop0/终态尾和前向tail mask7完整。
- 四次均值前2106.25mm/右12.25mm只是实测描述，未做新模型拟合/参数修改。证据工创赛/evidence/navigation-planning/2026-11-17/right1-forward210-field-run1/stage_complete4.json及run1..4；初组三次stage_checkpoint.json保留。四次执行门全false。
- 用户要求后退210并确认已回原起点、后侧约2.6m足够。后退第1次已完成且直接保存RAW2021；实尺后2102.5mm/右10mm（原话0.25c'm按cm），用户已摆好授权第2次。后退数据在right1-backward210-field-run1及后续目录，尚未满3次，不把方向或版本混组；以各自结果文件为实际运行进度。
- 右阶段本地提交fa14d5fe；本前进完整阶段单独本地commit，不fetch/push。原6dirty/原编码换行保持。用户要求运动完成直接保存RAW继续有效。R RUN-01/PWR-01/MECH-01/02；E LOC-02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04待技术答疑。未做完整轮/HWT残差拟合或ICM精度验收。
