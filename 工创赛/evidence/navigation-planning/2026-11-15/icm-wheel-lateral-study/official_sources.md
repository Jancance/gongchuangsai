# 官方资料核对（2026-11-15）

- TDK InvenSense DS-000577 ICM-45686，rev1.0（2024-07-25），https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf 。±4g档为8192LSB/g，±1000°/s档表列32.8LSB/(°/s)。当前IR1 accel_config=0x38、gyro_config=0x28与驱动写入匹配；不把产品额定噪声/偏差当本车标定值。
- TDK AN-000257 Understanding IMU Sensor Offset，rev1.3，https://invensense.tdk.com/wp-content/uploads/documentation/an-000257_understanding-imu-sensor-offset.pdf 。芯片级偏置和装板后偏置不同；板应力/焊接及温度等影响会改变偏置，官方特定测试板指标不能直接套本车。

本研究使用标准重力9806.65mm/s²作为换算常数，不是现场测量当地重力。重力与加速度分离、杆臂及短时预测公式均是明确假设下的工程模型，不是厂商对地面位移精度的保证。
