## 2026-11-01 修正左前采集零位置符号误触发

用户表示准备好了后，先导出上轮capture：194条有效回复、2.509秒、cause1，读回左前P_Pul0。但转速/位置始终0，发现触发代码把零位置的sign翻转当位移，不能称捕捉到扰动或振荡改善；原数据dynamic_analysis.json/dynamic_samples.csv保留。已向用户纠正此前“捕捉到扰动”的判断。

独立tmp/x_lf_puloff_capture_v2_20261101只修触发：先以int64计算有符号位置差，再比较±0.2度，忽略正负零，0.2RPM速度触发保持。原PID/保护/60秒等待/触发后2秒关闭/30RPM和30度附加阈值/P_Pul核对门禁均不改。抽取实际候选C表达式编译至ARM并Unicorn执行9项回归，0失败，含正负零、小位移、跨零、速度与uint32极值。Keil增量0错0警告。烧录075d4c336a457e5175a8095fc038ffd40aa5fddfc34c88039e83a049369bf8a9，43908字节回读一致，VM默认0/PE2低，主控无故障。证据capture_v2/download_vm_off.json、trigger_regression_result.json、build.log。即将单次重新采集，尚无动态结论。仅本地未上传。


本轮复查：采集已结束，cause1、trigger_tick49275，总1981条中环形保留512条，触发后155条约2秒；VM请求0/PE2低，主控无故障。修正后实际读得转速-2.1~+1.8RPM、位置-0.5~+0.3度、目标始终0、位置误差-0.34~+0.58度、相电流5~156mA，最后0.5秒仍有同范围往复。关闭P_Pul后仍观测到往复，不能声称已改善。用户是否在该窗口拨轴尚待确认，不将该数据自动归类为手动扰动试验通过或单一定因。证据capture_state.json、samples.bin、quick_analysis.json、decoded_samples.json。未重新启动采集或改参数。
