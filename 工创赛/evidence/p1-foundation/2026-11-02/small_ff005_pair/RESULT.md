# 前进0.5%小前馈：A/B软件已准备，未实车执行

2026-11-02。用户要求下次测试加小前馈，澄清此前仿真已经做过。本轮准备独立固件对照，不重复MATLAB参数扫描，不把模型曲线作为实车精度保证。

- A：无前馈；B：前进0.5%、后退关闭。两者共同基线均为当前主工程X/TMQ/BTQ/HF3，P1/D0.25/死区0.3°/I关闭。
- 主工程1314文件SHA256不变。副本仅既有chassis_kinematics.c改变，新增longitudinal_feedforward.h；前馈数学实现与已审查候选一致。GBK/CRLF保持。
- 新增实车编译显式许可APP_FF_VEHICLE_OPT_IN，源码默认0，两个方向的前馈默认关闭。仅B实车工程明确置1；非mode3调试入口拒绝启用。它只解除候选的编译禁用门，不代替原健康/VM/启动许可或运行保护。
- Safety、Power、Watchdog、CAN、main和原控制后端逐字节不变；未放宽超时、反向或任何运行保护。
- 四个独立构建A/B×offline/vehicle均0错误0警告；7项编译门禁符合预期。
- 仅补新0.5%前进/后退关闭配置的实际C回归：40场景、1,157,215条断言符合预期，包括原S曲线/限幅/反向/停止及故障场景。
- 固定1号轮的首轮夹具未找到合法末端采样点，严格报“未注入”，原报告保留在models_ff005_forward_only。随后在本次独立检查内按实际ACK目标选择可注入轮，不改固件；4号轮目标+1.7RPM注入-0.3RPM一次，路线完成8/8。持续反号2次、大反号-0.6RPM及窗口外反号均第二段保护停。最终报告位于eligible_wheel_check/models_ff005_forward_only。
- 无串口/探针/硬件访问，没有下载、复位、供电或发车，没有提交推送GitHub。

| 待实车验证版本 | Keil工程 | AXF SHA256 |
|---|---|---|
| A 无前馈 | firmware/gongxun-f407-baseline_vehicle.uvprojx | f8462206ca1fcdb22b5e18bd9980d3dc8dba11622fb3ae70b082fa816a0e3630 |
| B 前进0.5% | firmware/gongxun-f407-ff005_vehicle.uvprojx | b36d68f26118381868ee9d56e99c2915dae13791f6691e58288b6bd8b5324304 |

offline两工程保持APP_X_OFFLINE_ONLY=1，不作实车下载。所有项目、宏、独立产物目录与SHA256见manifest.json、final_review.json和build_*.log。

下一步按[待执行测试表](TEST_PLAN.md)现场重新确认状态后，先新A基线，再同条件B单次210cm，保留尺量与固定机位视频。使用forward210.json单段路线，不使用旧默认绕图路线。新主工程本轮也未实车验证，不能省略A直接把B与板上旧TM9镜像比较。

本轮未提高采样频率，两版采样一致；原约0.5秒反馈仍不足以精确辨识快速动态。后退关闭；高频记录优化、后退基线、前馈实际收益均未闭环。

已核对R FIELD-01/06、RUN-01、PWR-01；E LOC-03/05、MOT-02/04、RUN-06，R/Q不变。当前仅本地准备完成。
