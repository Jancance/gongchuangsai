# 本机MATLAB离线仿真能力检查

检查日期：2026-11-02。本轮只做本机环境诊断和合成数据测试，无串口、下载器、外部模式或硬件访问。未修改MATLAB工具箱、编译器配置或项目固件，未提交推送。

- 实际运行版本：MATLAB 9.1.0.441655，R2016b，64位；安装位置 `D:\matlab`。
- MATLAB数值积分实际通过：dy/dt=-y，y(0)=1，t=1结果0.36787944238，与exp(-1)一致。
- Control System Toolbox 10.1实际通过：1/(s+1)单位阶跃，t=1结果0.63212055883。
- Simulink 8.8实际通过：脚本创建Step→Transfer Fcn→To Workspace，仿真1秒，结果0.63212055883。模型保存在本目录codex_matlab_environment_probe.slx。
- System Identification Toolbox 9.5实际通过：合成输入输出数据创建iddata并运行arx，得到idpoly模型。这是工具功能测试，不是实车参数辨识。
- 已安装并通过license test：Simscape、Simscape Multibody、Simscape Driveline、Simulink Coder、Embedded Coder等。本轮未实际运行这些机械模型或代码生成，不将安装/授权检查说成运行验证通过。
- MATLAB的MEX C编译器：Selected=0、Installed=0，未识别到兼容编译器。因此未执行C MEX编译；这不表示电脑上完全没有其他C编译器。未运行mex setup，也未安装或调整软件。

结论：本机具备基础数值、控制系统、Simulink闭环及系统辨识仿真能力，可用于后续PD/前馈数学模型与日志分析。若要在MATLAB/Simulink内直接编译复用当前C控制代码，还需配置与R2016b兼容的MEX C编译器并按旧版接口实现；不能将现代版本示例直接视为本机可运行。

证据：environment.json、environment.mat、matlab_probe.log、probe_matlab.m和生成的slx模型。first_probe.log保留初次探测脚本因带连字符的字段名不合法而提前退出的记录；已修正字段转换并完整重跑，以environment.json及最终日志的MATLAB_PROBE_FINISHED为准。
