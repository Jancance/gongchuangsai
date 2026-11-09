# 原料区视觉对准：软件接口候选

用户要求“进入下一步”，随后明确“先做软件接口，设备稍后接”。本轮仅开发软件候选，不操作相机/香橙派/主控，不改D盘实际固件或串口接线。

已核当前视觉project_profile仍tracking_units=pixels、camera/geometry未确认、工作平面高度null，generic相机候选calibrated=false。它们不是当前在线硬件证明，也不能用旧像素误差当毫米偏差。

完成C毫米观测inbox、调用实际VehicleMission_ReadAlignmentRequest声明的桥接候选、Python MA1帧编解码及默认未标定profile。观测绑定request/run/stage/start/map/goal及camera/calibration/mount/height/target版本，校验CRC/整数范围、时间、序号、单位、置信度和偏差上界；仅在实际停车接口允许时打开/提交/读取，取消/轮次改变失效。观测接受不标记对准DONE、不发电机命令。原AA/CC像素协议/视觉main.py未改，新模块未挂接当前串口或固件；请求线上握手仍待设备归属确定。

实现和完整字段/时间契约见[软件接口说明](../software_interfaces/stage2_metric_alignment/README.md)。默认calibrated=false/ID0，缺乏实测时拒绝事务；没有伪造标定、图像或毫米纠偏。

验证：`scripts/check_metric_alignment_interface_20261104.py`执行Python13项及C137项，0失败；包含Python生成帧→C解析/接受、CRC/截断/超长字段、像素拒绝、旧轮次/标定变更、超时/传输积压、重复序号、int32极值/NaN和取消后结果失效。GCC警告视为错误通过，ARMCC Cortex-M4单元编译0诊断；不是完整固件构建/RTOS调度或硬件验证。证据metric-interface/result.json、metric-check.log、armcc-compile.log、源码SHA及技能知识审计knowledge-audit.json。技能只用于项目检查，未修改技能或声称自动更新。

已检查R RUN-01/02/04、FIELD-02/06、PWR-01、MECH-01/02、QR-01、DSP-01；E VIS-09/12/14/15、CAL-02/03/04/05、LOC-02/03/05/06/07、RUN-06、MECH-06。本接口/候选限幅/时效均为E，不新增竞赛规则；FIELD-05、OBS-03、QR-04等Q仍**待技术答疑确认**。

未闭环：新10cm/−X10cm轨迹本轮实物结果、偶发撤VM最新首因、设备身份与图像采集/缓存时效、内参/外参/工作平面高度/毫米验证、跨重启唯一request_id、线上请求握手/串口集成、原生位置微调/复测收敛、抓料及正确次数。当前车端镜像仍vm-review-retune版，本轮无烧录、复位或运动。
