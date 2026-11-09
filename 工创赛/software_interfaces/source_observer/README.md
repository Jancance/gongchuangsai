# 原料区观测收件接口

该接口只接收和保存观测，不发送电机命令，不设置对准DONE、抓放成功或第三段放行。

`source_observer.c/h`复用`stage2_metric_alignment`的CRC、版本/轮次、停车、有效期和范围校验。Task RX将完整MA1帧放入一个256字节槽，Task A消费；队列占用、超长或坏帧计数，不因此停车。RTOS临界区只保护队列复制。Task A通过真实`VehicleMission_ReadAlignmentRequest`取得当前请求，生成带CRC的MR1，沿现有非阻塞发送通路上报。新包默认profile全0，保持MA_UNCALIBRATED，不虚构ID或实际标定。

MR1包含18个非负十进制字段：request/run/stage/start/map/goal、timeout、camera/calibration/mount/height/target、max_age/min_confidence/max_delta/max_yaw、nominal_x/nominal_y，末尾CRC16/CCITT-FALSE和CRLF。SBC端`metric_alignment_protocol.decode_request`对应解码。MA1沿用既有19字段，不改旧像素协议。

`source_metric_preview.py`先变换实际外缘点，再在物理平面拟合圆心；`source_metric_bridge.observation_packet`仅在实际几何、版本、曝光时效和纯平移/保持语义都确认时编码观测。调用它不打开串口。圆本身没有航向信息，不能把包中的0旋转纠偏说成视觉测得偏航为0。

完整候选位于`tmp/source_observer_20261106/project/`，尚未安装/烧录。候选接到当前USART1调试接收链，真实香橙派串口归属仍需现场核对；未改USART6接线或用端口名假定已联通。主状态机位置执行器保持关闭，普通参考/载台新高度未确认时继续拒绝抓放。包含本轮升降108400候选和5号Scale解码修正，旧载台28000被留空(-1)，不能直接当正式自动抓放版下载。

验证入口`scripts/check_source_observer_20261106.py`，本机MATLAB15检查先于实现；几何/协议Python32检查、C收件16、几何→Python帧→C收件16、真实正式position函数32检查通过；两个完整Keil目标0错0警告。设备、时钟、主状态机在主机测试中部分为桩，不代替RTOS时序、现场坐标/通信或物理动作。
