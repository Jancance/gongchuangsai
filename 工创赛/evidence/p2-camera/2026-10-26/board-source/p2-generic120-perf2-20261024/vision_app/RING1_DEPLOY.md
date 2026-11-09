# 120°印刷圆环误报黑色：ring1工程候选

## 原因与边界

用户确认场景只有一件绿色物料，圆环有时误报black。已核验20261024-104324-197600.zip的曝光读回为Manual1/exposure83/gain80/AWB0；CRC、模块哈希、配置和逐帧统计交叉核对见evidence/p2-camera/2026-10-24/ring-black-104324/review.json。1061帧含首条STALE_RESULT和1060条FOLLOWING_PIXEL，后者均有green/T001；104条另有black。首个黑色误报在左侧印着3的环，候选bbox约[259,389,311,307]，来自combined_silhouette/DOMINANT_COLOR_EVIDENCE，不是黑色边缘恢复分支。前后黑色有效票29.98%→30.09%，跨通用30%颜色门。白色亮区被原质量筛选排除后，印刷线票的分母变化足以越界；不把此通用颜色占比直接当黑色实体证据。

这是当前场景已定位的失败，不把曝光83宣布成六色正式标定。配套green83-ring-console.log本地未找到，包内原图/逐帧/读回足够定位本轮原因，但不能据error:null证明没有解码警告。

## 修订内容（E）

从perf1构建独立ring1，只修改project_detector.py并在material_color_thresholds.json追加printed_ring_rejection_candidate配置。原六色HSV范围、35%质量门、150ms时效门、相机初始化与串口禁用保持。

仅对已被当作黑色物料的候选，检查当前黑色蒙版的归一化椭圆半径分布。中心有充分实心黑色证据时保留候选；否则外侧出现至少3组以浅色间隔分开的深色环带时，输出UNKNOWN/PRINTED_CONCENTRIC_RING_PATTERN，保留候选及证据字段。当前误报有5个深色带、中心黑色占比约22%。不依赖环编号、固定坐标或整幅ROI，不直接删除未知候选，也不关闭黑色类别或其反光恢复。

80径向采样、深带0.45/浅隙0.15、中心0.65和最少3带等均是待物理验证的工程参数，不是竞赛规则或已有准确率。真实黑料自身复杂同心纹理、强反光、遮挡和黑料放在印刷环上的困难场景仍须实测；证据不足可以UNKNOWN，不能强猜。

## 软件证据

8项定向回归：实际误报变UNKNOWN且green结果保留；所有已存且内容去重的原图保留一个green无black；关闭开关精确恢复基线；缩放/旋转/椭圆变形的开发增强；真实黑色首次反光/接触恢复；历史六色结果保持；合成黑色实心顶面覆盖印刷圆心仍能输出black；蓝色高光/空场旧黑框继续拒绝。开发增强和合成测试不是独立实物验收。

另检查六模式、四工具帮助、配置/编译/解压包；所有其他配置、跟踪器、最终时效门与相机源码逐字节保持。证据和包SHA256见evidence/p2-camera/2026-10-24/ring1/validation.json。

## 唯一下一项：曝光83同场景复测

Windows PowerShell上传：

```powershell
scp "D:\Codex\工创赛\output\p2_vision_generic120_ring1_20261024.zip" orangepi@10.252.117.211:/home/orangepi/
```

IP为用户本轮确认值，换网后先hostname -I。香橙派终端：

```bash
python3 -m zipfile -e ~/p2_vision_generic120_ring1_20261024.zip ~/p2-generic120-ring1-20261024
python3 ~/p2-generic120-ring1-20261024/vision_app/run.py --check
python3 -u ~/p2_green83_ring_capture.py --app ~/p2-generic120-ring1-20261024/vision_app 2>&1 | tee ~/ring1-green83-console.log
```

新目录若已有现场修改，另用未使用的目录名并同步参数。关闭其他预览，保持当前物料/光照/机位/FOLLOW，正常采集约90秒自动收尾；入口会在原初始化后设置曝光83并留读回，避免被166覆盖。回传Evidence ZIP路径后取回原图/逐帧/参数审查，不能只按没有black框签署通过。此轮只查当前绿料/印刷环场景；随后须另做真实黑料正样本门，不能跳过。没有操作机械臂或放行动作。

保留perf1目录不覆盖；回退时退出ring1，再用原83取证入口默认perf1。也可在候选配置关闭printed_ring_rejection_candidate.enabled恢复原判定，关闭开关的完整结果已做离线一致检查。

## 规则、证据与未闭环

R：VIS-01六色保留、FIELD-07补光方向，当前按用户补光OFF。E：VIS-02/03/09/11/12/13/15、CAL-02/03、RUN-06。Q：QR-04等仍待技术答疑确认，本文不新增官方规则。

ring1板端误报/漏检/耗时、实际黑料在圆环上的表现、长期/六色困难集、内参毫米/随臂与动态抓放未闭环，串口/动作禁用。构建scripts/build_p2_generic120_ring1.py，验证scripts/validate_p2_generic120_ring1.py。
