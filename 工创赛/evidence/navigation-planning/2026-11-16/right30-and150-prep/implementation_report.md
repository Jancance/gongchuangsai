# 右移150cm＋30cm软件候选完成（未部署）

用户要求先commit已有150cm代码，再准备30cm，稍后一起测试；并要求未来比赛使用方向/距离/峰速等参数的通用运动函数，保留此前各方向参数。已先本地提交150cm检查点607368de0ce5eeb087f675b9183fbee4cb761f35（27文件，当前固件源及相关验证/交接；包含默认关闭的VM诊断脚手架，未宣称其实车验收），未fetch/push。30cm增量保持未提交，不自动再commit。GUI布局、旧全局host脚本及历史证据等无关dirty仍保留。

## 同一候选的入口

- PLAN12/right150_40：右mode2/kind6，1500mm，102RPM预算，400mm/s参考上限。
- PLAN13/right30_40：右mode2/kind6，300mm，同102RPM预算与400mm/s参考上限。
- 原8前/9左/10右/11后2100mm和12全部保留。13不等于旧right30/PLAN3；严格串口仅加入PLAN13，跨距离/方向/RPM拒绝。默认仍PLAN8，不自动启动。

两者使用既有MotionProfile_Prepare通用曲线，不新建30cm控制器。不改运动参数、比例/PID/前专用横偏/左2%/STOP/capture/保护。方向矩阵和parameterized_motion_interface_note.md明确哪些来自已测工况；普通/BASELINE40/旧route分支不混用，后退继承比例不冒称独立标定。比赛高层参数接口仍未完全泛化，保留prepare与Begin运动许可分离。

## MATLAB必须先于实现：已满足

本机E:/Program Files/Polyspace/R2020a/bin/matlab.exe，21:38:50—59实际exit0，产品首次write21:42:56之后。按实际motion_profile.c短程分支复现，包含float32对照。

|右向目标|速度上限mm/s|生成参考峰mm/s|参考巡航s|参考总时长s|
|---|---:|---:|---:|---:|
|300mm|400|167.930894808|0|3.572898249|
|1500mm|400|400|0.230050978|7.269949022|
|2100mm|400|400|1.730050978|8.769949022|

右向有效A133.873157712/J251.619473135相同；其它方向仍按自身换算与补偿计算，不把右向有效A/J强套四向。300 nominal42RPM、加跟随base44、加航向轮界46；全预算组合<=104<反馈130。三者加2s仍小于原12.779s。只签参考模型/预算，不代表实际峰速、离散阶跃物理A/J、轮地滑移或地面到距。300不能称与1500/2100同峰速倍率校准。

## 验证结果

- 1931项输入仅app_entry.c/h、motion_trial40.h、app_motion_runtime.c、app_serial_runtime.c五文件变化，其余1926原字节；GBK/CRLF保。
- 真实ARM C/Unicorn局部检查：trimON10998/OFF10998/inactive3863断言PASS，含精确8..13白名单、旧方向边界、真实通用曲线对300/1500/2100采样/末态。
- host120项纯内存检查PASS：12和13分别PLAN→ARM→唯一模拟START→DONE、错误kind/target拒绝，未开Serial对象；不是自动连跑器或实板协议验收。
- 当前模型独立审查91项内存检查及源码/方向矩阵、MATLAB、ARM/host、完整构建/ELF核读PASS_WITH_LIMITATIONS。不是6.1-sol high审查，用户已明确允许服务不足后由当前模型接替。
- 最终Vehicle_Integrated重建返回0、0错误0警告，离线ELF文件检查通过。旧150冻结38a7...和旧210现场镜像文件821f...精确SHA保持；本轮没有板端验证或改动。

## 正确工程与镜像

正常可重建工程：工创赛/tmp/right30-and150-build-20261116/right30_150_40.uvprojx，目标Vehicle_Integrated，相对输出right30_150_40\。
最终AXF SHA256：3f8fe3bf732637ebe1e9765b5ebf686c7042d536bcfd1898dc163033d260b3cd。
Flash加载116424B/SHA256：b8cd6790ed133de5aa2c5ebee0c87e3793a14bc85ce72891cc8ae4a74ef458e7。
冻结副本：本目录frozen/Vehicle_Integrated.axf。独立新client为本目录p1_serial_motion_powered_done.py；旧150client不支持13，不覆盖它。

## 硬件与未闭环

当前download/reset/START/COM/SWD全部0，execution_permitted=false，不自动发车/重试。用户说以后一起测，只表示把两工况备在同版；后续现场就绪再逐趟执行。最后已知板载是旧210测试版，本轮未更新，不将文件构建成功说成已下载。此前已确认同一测点、固定起点地面横线、直边/两点初始对齐的测量方式继续沿用。

已核R RUN-01/PWR-01/MECH-01/02；E LOC-02/03/05/07、MOT-02/04/RUN-06；Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。未闭环：组合候选实际下载/板身份、1500/300地面主距/旁偏/轮速/负载与重复性、不同速度过程下误差解释；正式比赛参数接口与旋转/斜向等未在本轮放开。仅本地软件完成，30增量未提交/未上传。
