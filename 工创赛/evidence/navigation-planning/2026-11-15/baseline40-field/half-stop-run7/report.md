# half-stop-run7：同half版第三次成功，尺量右12mm/多7mm

2026-11-15 07:51（Asia/Shanghai）。用户另行要求再测并确认摆好，仅工具复位一次/fresh HWT −0.005°/40cm/s210cm一趟。无改固件/参数/下载或自动重试。

- software_complete；20秒保VM尾收，199有效尾帧，最后有效帧相对完成19.938秒，powered_tail_valid=true；VMreq/PE2=1、backend state9/result1/inactive/fault0。完成后TX0；XD934/934完整，103完整批，TM累计drop7另列。
- 中段稀疏OD峰6mm/3点，横向指令峰2.2RPM；HWT首尾−0.005→+0.390°（+0.395°），停止前相对起点左0.043°。均非地面横偏真值。
- 同脚本尾收后立即读176B，双读一致SHA bbd9e8a50b971e39f10f8d727cf87e2efb6d476caac0653d77b5465ba4bf4c57；session1/mask7/RUNNING/VM1/fault0前后不变。读取结束已通知用户可复位。没有halt/reset/VM写。
- arrival now14060ms/common13946ms/age114ms，mean2102.637695mm/reference15.110434RPM；最终PE3校准mean2114.248248mm，与arrival差11.610552mm。不同样点的轮端差值，不是地面停距。
- 用户原话“右偏1.2cm多走0.7cm”：实际前进2107mm/前进误差+7mm/右偏12mm。PE减尺量+7.248248mm，尺量减arrival +4.362305mm。不能用轮端接近到位值否认地面误差。

同版run5/6/7实尺右20/12.5/12mm，多12.5/10/7mm；描述性横偏均14.833mm、前进多9.833mm，三趟未改参数且非严格配对，不能按顺序认定改善，也不能宣称串口故障修复。此前half-stop-run1右15/少5mm为另一次成功，run2/3/4失败不混入精度统计。

源/已下AXF3966ed37…1113d保持。R RUN-01/PWR-01；E LOC-01/02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04待技术答疑确认。本轮无新控制/模型或MATLAB需求。用户现在转向ICM辅助编码器/横向计算离线研究，不再自动发车；HWT主航向、ICM只记录，先已有日志和MATLAB，不直接接控制。未fetch/push，其他dirty保留。
