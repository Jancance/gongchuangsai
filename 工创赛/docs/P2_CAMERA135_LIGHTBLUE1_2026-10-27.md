# 摄像头1.1：浅蓝漏检复核与lightblue1候选（E）

## 现场结果

输入20261027-181512-402452.zip（SHA256 a71d1fc8f24a421d9114c9a81cb6c22b021a19f049fdbf2d3163d70352be0783），CRC、4运行模块/2诊断模块及3配置源哈希/实际配置匹配blue2发布。2042帧，首末主机read完成跨度235.039秒：2035正常、7过期；正常帧1574次蓝+浅蓝、461次仅蓝，无正常帧漏蓝或蓝/浅蓝重复候选。461/2035=22.65%为本轮缺浅蓝输出比例，不是任意场景准确率。保存原图与上一轮相比有整体位置变化，不把跨轮比例变化归因于仅切换watch-color。

检测调用中位/95分位/最大95.769/102.615/150.707ms；主机结果年龄116.726/138.482/196.094ms，诊断开销0.330/0.391/18.136ms。7次过期均清空目标（检测器5次仅蓝、2次双颜色）。所有帧ready=false/motion_allowed=false，串口禁用；这些时间不是曝光到结果总延迟。

已视觉查看first-nonstale-lightblue-missing-raw.png：frame3物料完整可见。原程序回放无浅蓝；在当前S189浅蓝分量的搜索框[1302,511,306,215]，只提出圆心[1405,640]、半径72.2px的Hough圆，周向内外颜色支持率51/72=0.708333，小于原0.75门，被拒绝。圆候选位于物料侧面，不能把这个圆当顶面中心。未声称461帧全部是这一精确原因；现场历史提示诊断另含NO_CIRCLE_PROPOSAL/INSUFFICIENT_ARC，原图不足以逐一确认。

正常浅蓝输出来自current_pale_circle1082次、chromatic_circle491次、combined_silhouette1次，中心极差54.849/25px；正常蓝2027次combined_silhouette、8次projected_blue_component，中心极差36.626/8.135px。来源切换及中心语义未统一，不能签稳定跟踪/真实运动或抓点精度。312次浅蓝丢失/重现、13次换号、14个不同浅蓝ID，最长已闭合缺失0.830秒；ID未每次变更不等于每帧有新鲜有效观测。

现场复核与单帧圆弧拒绝的可重复脚本：scripts/review_camera135_lightblue_20261027.py；输出evidence/p2-camera/2026-10-27/lightblue/review.json，保留原ZIP与run-20261027-181512-402452解压目录。

## 独立候选与限制

从完整blue2复制至output/p2_vision_generic135_lightblue1_20261027/vision_app，保留全部原文件/功能。修改project_detector.py和material_color_thresholds.json，新增project_pale_component.py。LIGHTBLUE1_MANIFEST.json记录来源与候选哈希；继承的README/旧manifest仅为历史，当前以此清单和本文为准。新旧包并存，未修改板端。

原有全部浅蓝检测分支均无结果时，启用pale_projected_component_candidate检查本帧独立浅蓝整体投影。复用已有局部S189范围（H90..115/S25..189/V100..255），不改六色主HSV；使用已有蓝投影的7×7局部闭合与工程形状门：长宽比(1.4,1.8]、圆度≥0.60、凸实度≥0.94。非削顶有效像素≥500、有效比例≥0.35、浅蓝票≥0.90、竞争票差≥0.65；深蓝竞争蒙版只扣除同一S189浅蓝像素，其他竞争色取实际主配置。要求位于本帧拒绝父区域、避开已接受父区域与画面边界；多个合法分量输出歧义而非随意选择。以上均为工程候选，非官方规则或通用保证。

不改75%圆弧门，不复用旧位置填补目标，不替换旧有效中心。新增输出是整个物料投影的轮廓中心，明确标记非顶面/非标定抓点。它改善已定位的颜色漏检，不解决原有中心来源切换；新分支与旧圆分支之间的中心变化仍须单独验证。150ms主机年龄/500ms无新read退出、像素单位、禁用串口及动作保持；关闭新开关恢复blue2。

frame3新增框[1333,559,275,167]，凸实度0.959218、圆度0.682889、浅蓝票0.998777，蓝结果逐字段不变。243张去重实拍各有/无历史提示回放，禁用新开关完整结果同blue2；启用时仅12张图的20个输入组合增加一个浅蓝投影候选，其他原结果/诊断完全相同。新增12张图已查看汇总图，框位于真实浅蓝物料；这是有限实拍检查，不是现场任意物体误报率。15项分量正/负样本、六色合成与无配置叠加图一致、150ms过期拒绝、--check、六模式、四工具及ZIP逐成员字节/CRC检查通过。证据lightblue1-validation.json、lightblue1-added-contact-sheet.png、lightblue1-recovered-overlay.png与各功能日志；脚本scripts/validate_p2_generic135_lightblue1.py。板端计算开销未测，不能宣称零新增开销或实时验收完成。

## 下一项：候选板端复测

Windows PowerShell上传：

```powershell
scp "C:\Users\Tenderness\Documents\ChatGPT\工创赛\output\p2_vision_generic135_lightblue1_20261027.zip" orangepi@10.252.117.211:/home/orangepi/
```

香橙派VNC终端防覆盖解压（仅第一次执行；已有目录则停止，不覆盖）：

```bash
test ! -e /home/orangepi/p2-generic135-lightblue1-20261027 && python3 -m zipfile -e /home/orangepi/p2_vision_generic135_lightblue1_20261027.zip /home/orangepi/p2-generic135-lightblue1-20261027
python3 /home/orangepi/p2-generic135-lightblue1-20261027/vision_app/run.py --check
```

检查输出应为135°profile/1920×1080、serial=DISABLED、metric_geometry_verified=false、motion_allowed=false。若检查失败，保留输出，不运行下一条。

```bash
python3 /home/orangepi/p2-generic135-rejection-audit-20261026/p2_color_follow_audit.py --app /home/orangepi/p2-generic135-lightblue1-20261027/vision_app --watch-color lightblue --sample-every 5 --max-samples 4 --output-dir /home/orangepi/camera135-lightblue1-diagnostics
```

保持本次物料/圆环纸/机位/灯光不变，无其他程序占相机，运行约90秒，在预览窗口按Q退出；取回终端显示的精确ZIP路径。若新误报，保存原图，回退原blue2目录，不继续放宽门槛。旧程序/ZIP无需删除。

## 已检查与未闭环

VIS-01为R，保留六色要求；VIS-02/03/09/12/13/15、CAL-03、RUN-06为E，本轮补检与阈值均为工程候选。官方QR-04等Q仍待技术答疑确认。未闭环：lightblue1板端漏检/误报/时效、461帧其余原因、稳定跟踪与中心语义、历史浅蓝重复、六色困难集、内参/残余畸变/毫米与随臂标定、固定动作组前置及真实抓放。没有进行执行器动作或修改机构验收状态。
