# 左移2100 mm：旧系数预冻的独立预测

本机MATLAB R2020a实际运行一次，exit0。旧系数于04:52:44先冻结，数值脚本没有读取本趟地面尺量；整份报告及MATLAB回执落盘晚于用户测量消息，不声称全部预测证据在测量揭晓前已落盘。没有产品、参数、硬件或Git操作。

旧6b4c42de左向四趟（left150 run1/2/3与left30 run1）训练CSV SHA为4105456A3441157CA68A43C2F841399B85C85CBB423F6AC6F32D5E76736C838B；2026-11-17 04:52:44+08:00先冻结参数再读取新原始轨迹。固定残差均值-11.8893624958mm；比例beta=-0.009967950358。残差b=model_cross-ground_cross，修正=model_cross-b预测。全部系数与训练名字在frozen_prior_prediction_coefficients.json，未依据本趟改动。

|本趟同源估计|mm|
|---|---:|
|左主程|2112.041161|
|前旁程|-46.231215|
|本体F|-43.717025|
|本体L|2112.167760|
|航向旋转对主程贡献|-0.126599|
|航向旋转对旁程贡献|-2.514190|
|固定残差修正后旁程预测|-34.341853|
|比例残差修正后旁程预测|-25.178494|

前为正，负值为后偏。这些是估计与预先声明模型的预测，不是真实地面位移。训练版6b4与新实车79e74不同，即使translation参数被保留，也需记录版本分布差异。

四轮position用source_ms，HWT用angle_ms，解缠后逐步中点/sinc旋转，F比例一次。RAW合法共同窗至9.115s，STOP接收8.073s、首终态接收9.151s；共同末窗早36ms，不宣称物理尾程零。OD同窗主轴差+0.001823mm、旁轴差+0.079021mm。完整分解见holdout_decomposition.csv，数值见holdout_prediction.json。

输入哈希before命令因PowerShell数组表达式串接失败，未取得本代理的before快照；实际MATLAB运行成功。现4项after哈希与实际脚本/系数SHA保存在holdout_input_sha256_after.json及holdout_matlab_receipt.json，不能把after冒称before。原始采集及双读稳定证据由根代理维护。

R RUN-01/PWR-01/MECH-01/02；E LOC-02/03/05/07、MOT-02/04、RUN-06。Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。未闭环：本趟尺量、起始摆位和尺量误差、同步地面轨迹、版本差异影响；不部署残差模型，不根据本次新尺量再拟合。
