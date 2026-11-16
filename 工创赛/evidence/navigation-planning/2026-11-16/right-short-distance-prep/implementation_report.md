# 右移150cm主方向计程对照：软件准备完成

状态：SOFTWARE_READY_NOT_DEPLOYED。用户授权软件准备，且在指定6.1-sol服务余额不足后明确允许当前模型接替；本轮没有下载、复位、串口、SWD或发车，也没有设置IMU七项。当前模型独立子agent审阅已完成，不能冒称已完成6.1-sol high。模型服务失败不是安全审批拒绝。

## 新入口和基线

新增PLAN12：右移mode2/kind6、目标1500mm、RPM预算102、速度参考上限400mm/s。CLI预设名right150_40。旧PLAN8/9/10/11仍分别前/左/右/后2100mm，默认仍PLAN8；不改变比例、PID、左2%、STOP/capture/保护，不开启旧PLAN0旋转。PLAN12只能精确匹配该tuple，不能变成任意距离/方向入口。

## MATLAB先行与局限

真实E:/Program Files/Polyspace/R2020a/bin/matlab.exe，20:59:20—31进程exit0，函数20:59:27执行，早于首次产品写入21:02:41。采用原已审同一分段S曲线与参数，非另造模型。

|参考|2100mm旧右移|1500mm新右移|
|---|---:|---:|
|参考峰速mm/s|400|400|
|有效A mm/s²|133.873157712|133.873157712|
|有效J mm/s³|251.619473135|251.619473135|
|参考总时长s|8.769949022|7.269949022|
|参考巡航s|1.730050978|0.230050978|
|加2s后预算s|10.769949022|9.269949022|

最低可达400参考峰速距离约1407.980mm。1500可以同参考峰值、同A/J与加减速段，但巡航很短，不能保证实车达到同峰速/稳定速度。后续实际速度和加减速覆盖必须保留；两个长度只能初步区分趋势，不能单独唯一判比例或据未同步末点强拟合。模型未包含真实地面滑移/载荷/机械延迟，未签准确性。

## 修改与验证

仅5个产品文件：app_entry.c/h、motion_trial40.h、app_motion_runtime.c、app_serial_runtime.c。1931项既有输入仅这5项变化，其余1926原字节；5文件GBK/CRLF保。before快照包含原有dirty，不把既有未提交成果当本次新写。协议精确加入PLAN12、BF1目标1500、idle动作回显、成功PoweredDone与VM保持均一致。独立新客户端采用明确plan/RPM/kind/mm映射，避免plan−4对12错误。

真实ARM-C/Unicorn局部测试trimON4741/OFF4741/inactive2539全部PASS；host84项PASS（含纯内存PLAN12→ARM→一次START→DONE），独立审阅者另33项内存检查通过。所有模拟命令未发到COM口。最终Vehicle_Integrated重建0错误0警告、离线ELF仅文件检查通过；不重跑旧全套capture/整场验收。

首host生成脚本两次命中数断言在内存退出、候选文件未写，失败记录保留。首ARM-C测试抓到共享方向ternary将12当后退，原白名单拒绝；已保存firstfailure和首版b7f2构建，修正唯一方向分支后原期望测试通过。首版不可交付。最终product_diff与final_changes才描述当前源码。

## 正确工程与最终身份

正常可重建工程：工创赛/tmp/right-short40-build-20261116/right150_40.uvprojx，目标Vehicle_Integrated，相对输出right150_40\，不是旧失败download-only工程。
最终AXF SHA256：38a7a344c0763abbf76570270d2fa3cf9c363d653c2fc4f59cd72bb01d4f3fd3。
加载116232B/SHA256：6e374139636161e7c70d180ba8c736bdd31f654396c12b119ef5dc71fc8aff3d。
冻结副本：本目录frozen/Vehicle_Integrated.axf。完整当前身份见frozen_verification_manifest.json。

原已知安装的旧821fae6a...9288现场AXF文件未覆盖；本轮没有更新板端。未来测150cm需在明确硬件授权/现场条件下先安装此新候选一次，再reset1/PLAN12 START1单趟，不能拿旧版当已支持12，不能自动连跑。当前execution_permitted=false，未创建可自动运行的硬件执行binding。原生RAW/末态供电尾仍沿既有机制，尺量点/初始对齐/固定地面基准沿用户已确认方法。

## 规则与未闭环

已核R RUN-01/PWR-01/MECH-01/02；E LOC-02/03/05/07、MOT-02/04/RUN-06；Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。未闭环：新候选实际下载与板身份、1500实车地面主距/旁偏/实际轮速、短长程对比解释和后续方向比例。本次只本地软件完成，不fetch/commit/push。
