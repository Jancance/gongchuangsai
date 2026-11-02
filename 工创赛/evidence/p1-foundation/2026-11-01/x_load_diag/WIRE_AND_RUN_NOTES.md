# X负载诊断记录格式与后续步骤

XD1,session,first_index,count,total,dropped,hex_payload,CRC16

payload为count条24字节小端记录，每包1~8条；struct格式<IIIhhHHBBBB。依次：tick_ms、target_ack_ms、position_deci_deg(幅值)、target_decirpm、actual_decirpm、phase_ma、bus_mv、address_kind、result、flags、faults。address_kind低4位地址(1左后/2右后/3左前/4右前)，高4位类型(1待发送/2立即ACK/3系统样本/4STOP ACK/5缓存ACK)。flags位0位置负号、位1目标已确认、位2日志速度被截断、位3~6后端状态。faults位0编码器就绪、1校准就绪、2过热、3过流、4堵转、5堵转保护。当前目标仅在立即执行ACK或STOP成功时更新；SAMPLE类型才可把target字段当成当前已确认目标；TX/失败ACK仅为请求记录。

IG1,session,tick_ms,enabled,ki_milli,limit_millirpm,peak_millirpm,last_millirpm,samples,nonzero_samples,CRC16

两类CRC均为CRC16-CCITT初值FFFF，计算覆盖最后CRC前的逗号，行尾CRLF。IG1只在I诊断版本停车VM关闭后发送。无XD1完整历史或dropped>0时禁止给完整负载结论；IG1缺失时不能确认I实际使用。

现场尚未就绪：现在不要执行以下命令。待用户明确就绪，先运行受限下载助手（会核对旧镜像与故障），再监视TM8待机身份。保持相同起点、对齐、载物与四台P_Pul_OFF。

后续单次主机入口：p1_x_load_integral_20261101.py --port COM17 --action run-once --mode 3 --plan default --route-trial --seconds 150 --output <本次新证据目录>。

运动结束VM关闭后继续等待数据导出，主机在完整收到XD1/IG1后提前结束，不需通过探针读RAM。期间不要复位/重新上电。STOP入口仍为同客户端--action stop，不自动重试START。轨迹结果、完整诊断数据、尺量和现场抖动观察分别确认。
