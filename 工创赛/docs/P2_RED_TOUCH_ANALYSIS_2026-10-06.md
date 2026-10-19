# 红色接触失检证据分析与候选修订

来源：用户板端生成并下载的`evidence/20261006-051539-184251.zip`，SHA256为`3df47d20698cd1073250be4cd7468d9a3fecc1d9c9acd63cb1f4ee60bb21458d`。原ZIP保留，CRC检查通过。可复现分析入口为`scripts/analyze_follow_touch_evidence.py`，结果在`output/follow-touch-analysis-20261006/report.json`。

## 直接证据

- 430个F检测结果：327个FOLLOWING_PIXEL、103个SEARCHING；3段失检后均重新出现，ID由T001逐步增加至T004。记录的三段间隔约4.862、0.763、1.429秒，均超过500ms暂存上限。
- 全部结果没有STALE_FRAME或STALE_RESULT，主机read完成至检测结果年龄最大123.678ms，小于150ms拒绝门。本次不能归因为过期拒绝，仍不证明传感器/执行端总延迟。
- 首次失检在第261个结果：原红色物料轮廓此前bbox=[500,324,261,304]；接触后六色合并掩膜的轮廓bbox=[0,187,761,720]、面积207381px²，延伸到左边界，理由TOUCHES_BORDER。原图的指尖碰侧面、红色顶面仍可见；对照保存配置重建掩膜，红色独立分量仍存在，但与黑色侧面/手部合并后的大轮廓被拒绝。首个触碰失检可明确归因于联合轮廓碰边，不写成“顶面遮挡”或单纯反光。
- 后续还出现54条SHAPE_UNCERTAIN候选记录，不全属于首次失检。保存的恢复前画面可复现红色物料整体形状拒绝，而单色圆面掩膜的细缺口经过局部闭合可得到强颜色/圆形支持。未保存每个失检帧，因此不能对103个SEARCHING结果逐帧作视觉根因判定。
- 诊断同步保存I/O最大410.202ms，会影响采样间隔及内部重捕获标记。原检测拒绝记录与图像相符，但此次处理/显示速度不能签署正式实时性能验收。

## 候选修改

只改`vision_app/code/project_detector.py`与`project_follow.py`。正常已接受的联合轮廓保持原路径。在联合轮廓因碰边或形状被拒绝时，检查其中已有HSV配置支持的独立彩色分量；该彩色候选自己必须不碰边，且位于拒绝的父轮廓中、未落入已接受目标。只在候选包围框内做7px同色细缺口闭合，不扩展到框外手指。追加严格圆形、实心度、长宽比、真实非饱和像素及颜色比例/间隔门，黑色不走此恢复分支。拒绝父轮廓的UNKNOWN记录保留，不将大手部直接当物料。

新候选记`proposal_source=chromatic_component`；普通候选为combined_silhouette。二者像素中心参考不同，因此切换参考的那一帧不计算速度/外推，防止产生虚假的速度跳变；之后仍按原时间/身份门计算。此处仍是像素圆面/轮廓，不是本机毫米或夹爪抓取点。没有延长150ms/500ms门，没有保留过期位置或改写未知身份，没有打开真实串口；ready与motion_allowed保持False。

## 验证结果与边界

- 保存的7个事件PNG（含重复画面）离线均输出恰好一个red，无重复black。原首次碰边帧和恢复前形状拒绝帧能得到ISOLATED_CHROMATIC_COMPONENT候选。这里是开发回放，不能称为430帧全量修复，因为只有事件PNG存档。
- 旧六色/单双物料静态图14张，修改前后已知标签完全一致；背景样本仍无已知物料。合成碰边手部与全白信息丢失圆不产生恢复目标。
- 新增4项回归覆盖实拍事件、手部/饱和拒绝、既有空背景和参考切换不产生速度跳变。全项目81项软件测试通过。
- 临时目录复制原10月3日完整包，应用补丁后--check正常；配置字节保持一致、原代码备份存在、重复安装安全退出、用户代码变更时拒绝覆盖均通过。没有香橙派新修订运行/连续ID/性能验收。

## 部署与复测

补丁`output/follow_touch_patch_2026-10-06.zip`，SHA256为`8d50af7eb904242030ea886c9ed5917d24127a0f28a0864d79a5f643ed8c1f9d`；安装入口`output/apply_follow_touch_patch.py`。只携带两模块和前后SHA；安装前验证版本，备份到`patch-backups/follow-touch-20261006`。更新后--check失败则恢复本次更新前代码，不替换颜色/相机配置。已验证的用户本轮IPv4为10.31.154.211，后续需变化时再hostname -I查询。

Windows PowerShell：

```powershell
scp "D:\Codex\工创赛\output\follow_touch_patch_2026-10-06.zip" "D:\Codex\工创赛\output\apply_follow_touch_patch.py" "orangepi@10.31.154.211:/home/orangepi/p2-integrated-20261003/"
```

关闭旧预览，在RealVNC终端运行：

```bash
python3 /home/orangepi/p2-integrated-20261003/apply_follow_touch_patch.py
```

仅在显示Patch applied或Patch already applied且无Traceback后继续：

```bash
python3 -u /home/orangepi/p2-integrated-20261003/p2_integrated_follow_audit.py
```

红物料静止5秒→手不遮顶面，接触侧面慢推2～3cm后松手→静止5秒→Q退出。回传新Evidence ZIP及框/ID现象；新候选的性能和失检需要板端年龄、图像和日志重新核对，不能用首次修复即宣布连续跟随抓取通过。

已检查VIS-01（R），VIS-03/09/11/12/13/14/15、CAL-02、RUN-06（E）。原触碰失检已取得并复现证据，候选修订软件验证完成；新板端实时、反光困难集、连续身份、毫米/时间/高度/P1联调及真实场地仍未闭环。QR-04（Q）仍待技术答疑。当前仍是桌面，安装、机械臂和场地切换前必须提醒。知识审计无数值冲突，API索引缺口不作为硬件通过证据；本轮不更新全局技能/记忆。
