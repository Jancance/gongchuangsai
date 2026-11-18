# 0x43 轮位置接入

2026-11-18。仅修改 `chassis_auto_test.c` 的现有 `Auto_Monitor` 速度分支。用同一条 `0x43` 的原始 sign/count 及末帧接收时间补入现有里程计接口。保留原 rpm 换算、方向保护、独立位置诊断、`0x3A` 状态轮询及异常出口。样本发布要求前后均为有效实车会话和 VM 命令，且电源使能代相同。位置样本须在 request/reply/now 时序内，并严格晚于已有的同轮样本。

- 本机 MATLAB R2020a 前置验证 `validate_monitor_odometry checks=306 PASS MATLAB 2020a`，退出码 0。模型 SHA256 `f3d6066a0e4b5672d44b291b78c541210d38551e1e8b5d1564e1849b03967d7c`。产品变更前源码 SHA256 `c86d93fce27db82889d762e546ca323624306fb35861064757d07d8310a84194`。
- 实际 Cortex-M4 `Auto_Monitor` 回放在两组编译配置下各 51 项通过，见 `monitor-arm-attempt2/actual_can_report.json` 和 `inputs.json`。监控代码和里程计样本出口为编译后的产品 C，CAN/VM 为测试桩。第一次链接缺少四个未改分支的桩，未运行。完整 0x43 协议解析见上一阶段 `ACTUAL_ARM_TRANSPORT_STOP.md`。
- 原 STOP 回放补齐速度接口测试桩后仍在两组配置各 51 项通过，见 `stop-arm-attempt4/actual_can_report.json` 和 `inputs.json`。
- 产品 SHA256 `4d6357ab44fdfa89893fce49c1ab6bc5bd4a3362c40fdaa95188c453cac45265`。GBK/CRLF 保留。R 核对 RUN-04/07、PWR-01；E 核对 LOC-03/05、MOT-02/03、RUN-05/06。FIELD-05、OBS-03、QR-04 待技术答疑确认。真实设备取样到末帧的滞后、运行频率和地图定位误差待实车测量。
