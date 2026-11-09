# 残缺环v1板端复核及v2性能候选

## 已核对的板端结果（E）

收到并读取 `offline-results-20261027-212121-167076.zip`（11547508字节，SHA256 `8d5e176beb107cd91efffd547b2f4392a1655c8d77b3d2b923c9e873e46d6654`）。CRC通过，报告中的source_manifest_sha256与发布包对应；OpenCV 4.5.4，19/19通过。与本地开发报告逐项核对接受/拒绝原因及输出坐标完全一致，最大坐标差0px，仍仅证明跨平台同图结果一致。

3张真实遮挡图板端离线耗时分别279.74、262.94、253.47ms；不是曝光至结果时间，也没有真实相机/连续稳定验收。所有输出stable=false，毫米和动作禁用。板端首张候选标记图已视检；报告和复核摘要见 `evidence/p2-camera/2026-10-27/partial-offline-board/review.json`、`verified-review.json`，原ZIP仍留项目根目录。

## v2实现及验证（E）

从冻结v1独立建立 `scripts/p2_partial_ring_candidate_v2.py`：

1. 复用全图轮廓提取、模糊灰度、Canny边缘和正式聚类，不在原质量门失败后再次提取。
2. 距离场和梯度只计算候选环局部区域。所有可能采样点外扩16px；原2px距离门及3×3梯度查询保持在有效邻域。Canny仍在全图计算，避免裁剪改变滞后连接。
3. 原几何计算机械拆分到 `scripts/p2_ring_raw_shared.py`；源SHA256标注在文件头。此文件由原提取函数保留同样计算后多返回两张中间图，原raw函数仅改为接受已提取候选，原算法文件没有修改。19张输入对照原raw字典逐值相同。
4. 原全部筛选、搜索范围、候选间隔、0.9质量门及150ms时效门均不变；没有用丢弃样本、减少搜索点或降低拒绝门提速。

`python scripts/validate_p2_partial_ring_v2.py` 通过19项：排除方法名/耗时字段后，v1/v2的接受状态、原因、坐标、分数、逐带支持及诊断字段完全相同。真实图、空场及合成难例继续沿用原开发回归集，不称独立场景验收。

同进程同图交替5次计时，本地三张遮挡图v1中位162.51/173.59/173.50ms，v2中位83.79/91.69/88.60ms；这些是本机预热后小样本，不是板端性能保证。发布入口独立复跑19/19通过，交替计时v2中位92.4/88.5/81.8ms，最大101.5/102.3/96.3ms。该次入口首图冷启动178.6ms仍超150ms，保留这一失败边界，不以预热后耗时掩盖冷启动。

证据：`output/partial-ring-v2-validation-20261027/review.json`；发布入口结果在 `output/p2_partial_ring_offline_v2_20261027/offline-results-20261027-212723-745204.zip`。未接普通R主程序、未创建连续稳定窗口；板端v2尚未执行。

## 下一单项：v2板端同图结果及交替计时

完整独立包 `output/p2_partial_ring_offline_v2_20261027.zip`，11474050字节，SHA256 `c643ac820644b0db6addf8feac4f7d3d7a077a7fc88ba4cf63e7aec2c058f16f`。保留冻结v1基线及原19张输入，新增v2和共享提取模块；manifest逐文件验证。建立新目录，不覆盖pixel1或v1工具。只读已存图片，不打开相机/串口。

Windows PowerShell：

```powershell
scp "C:\Users\Tenderness\Documents\ChatGPT\工创赛\output\p2_partial_ring_offline_v2_20261027.zip" orangepi@robot-opi5pro.local:/home/orangepi/
```

上传成功后，在香橙派VNC终端依次运行：

```bash
python3 -m zipfile -e /home/orangepi/p2_partial_ring_offline_v2_20261027.zip /home/orangepi/
python3 /home/orangepi/p2_partial_ring_offline_v2_20261027/p2_partial_ring_offline_replay.py --check
python3 /home/orangepi/p2_partial_ring_offline_v2_20261027/p2_partial_ring_offline_replay.py
```

回传 `passed=`、三条 `BENCH` 及 `Evidence ZIP:` 实际完整路径。工具先做19项逐字段v1/v2对照，随后三图各做5次交替计时；启动单次结果与预热重复计时分别报告。新程序变更触发本轮回归，不要求新的相机采样/物料操作。若DIFF或异常，先复核原结果；若超150ms，继续处理性能/启动路径，不放宽时效门。

## 已核需求与未闭环

R：SCORE-01不改，不以离线像素结果证明落环得分。

E：VIS-07（定位输出不变）、VIS-12/15（只有同图候选，无真实连续新鲜稳定验收）、VIS-13（可见边缘与歧义拒绝未放宽）、CAL-02/03（像素/毫米分离，几何继续暂缓）、RUN-06（拒绝不保留有效坐标）；VIS-04/05的编号/毫米映射仍未验。v2为工程优化，不是新增竞赛规则。

Q：官方未明确项保持待技术答疑确认。

未闭环：v2板端结果/小样本性能、冷启动超时与持续时效、实时整合/连续稳定/遮挡恢复、独立新场景及真实同步带、任意编号、几何/手眼/毫米和物理落环。动作继续禁用；不因离线低于150ms自动解除实时门。
