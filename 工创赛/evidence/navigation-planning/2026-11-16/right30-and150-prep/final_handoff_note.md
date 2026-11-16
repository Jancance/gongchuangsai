## 2026-11-16 已commit150检查点，30＋150组合软件完成未部署（最新）

- 用户授权先commit当前150cm代码、再准备30cm、以后一起测；随后强调比赛运动函数需参数化方向/距离/峰速等，并正确运用以前各方向参数。已先本地提交607368de0ce5eeb087f675b9183fbee4cb761f35（27文件，150阶段当前固件源及选定验证/交接，含默认关闭VM诊断脚手架，不表示该支路实车通过）。不fetch/push；30新增和本条交接保持本地未提交，不自动再commit。无关GUI/遗留全局host/历史证据dirty仍保。
- 新证据工创赛/evidence/navigation-planning/2026-11-16/right30-and150-prep/implementation_report.md。同一候选PLAN12/right150_40=右1500mm，PLAN13/right30_40=右300mm；均mode2/kind6/102RPM预算/参考上限400mm/s。旧8前/9左/10右/11后2100与12保持，默认仍8。13不是旧PLAN3/right30；ID/方向/RPM/距离精确匹配，不开放无约束串口参数或旧PLAN0旋转。
- 本机R2020a21:38:50—59实际exit0先于21:42:56产品write。复现实际通用motion_profile.c短程分支：300生成参考峰167.930894808mm/s（约16.79cm/s），无匀速、总3.572898249s；1500/2100仍400峰、总7.269949/8.769949s，巡航.230051/1.730051s。300是短程起停工况，不是与150/210同峰速标定；实际速度/滑移/距离仍待测，两个长度也不唯一判比例。
- 已复用既有MotionProfile_Prepare(distance,&limits(speed,A,J))，没有30cm新控制器。数学规划层已经参数化与短程退峰；比赛调用层仍有route20cm/s/入口/物理工况约束，不能称任意距离速度或正式比赛全开放。parameterized_motion_interface_note.md与direction_parameter_matrix.md说明现有prepare/Begin分离、单位/时标/方向参数、真实完成语义和未来调用合同。
- 方向参数分别保持：前/后主纵向倍率1.0218254，后为继承非独立标定；右主距1，横移旁向F里程仍用现有纵向换算。四向共用航向PD；前专用横偏修正仅PLAN8/mode0，左2%仅PLAN9/mode1，右10/12/13和后11不套。有效A/J随方向换算与补偿预算而不同，不把右向数值套全部方向；普通/BASELINE40/旧route左修正分支不混用。
- 相对607检查点产品仅5files：app_entry.c/h、motion_trial40.h、app_motion_runtime.c、app_serial_runtime.c，1931输入其余1926原字节，GBK/CRLF保持；curve/chassis_auto/kinematics/app_config/heading_controller/比例/STOP/capture/保护/IMU七项均未改。真实ARM C/Unicorn trimON10998/OFF10998/inactive3863 PASS；host120内存PASS含12/13分别唯一模拟START，当前模型独立91检查与源码/方向/MATLAB/build/ELF审阅PASS_WITH_LIMITATIONS，非6.1-sol high。
- 正确工程工创赛/tmp/right30-and150-build-20261116/right30_150_40.uvprojx，目标Vehicle_Integrated，相对输出right30_150_40\，21:54:00实际构建0错误0警告。最终AXF3f8fe3bf732637ebe1e9765b5ebf686c7042d536bcfd1898dc163033d260b3cd，加载116424B/SHAb8cd6790ed133de5aa2c5ebee0c87e3793a14bc85ce72891cc8ae4a74ef458e7；normal/frozen一致，旧15038a7和旧210821镜像均未覆盖。完整身份见frozen_verification_manifest.json。
- 当前download/reset/START/COM/SWD全部0，execution_permitted=false。以后只需组合候选安装一次，按摆位逐趟采集150和30，每趟沿reset1/START1、供电尾与首次RAW安全保存；不先下150再下30，不自动连跑。当前没有新现场确认/板端动作，最后已知板载仍旧210候选；用新目录独立client，旧150client不支持13。
- 已核R RUN-01/PWR-01/MECH-01/02；E LOC-02/03/05/07、MOT-02/04/RUN-06；Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。未闭环组合板身份、150/30实车主距/旁偏/实际速度、不同曲线下误差解释、正式比赛参数接口与其它运动工况。根本条外旧文逐字节保留，HEAD607368de未再改写，不push、不声称远端同步，未提交其它无关成果。

