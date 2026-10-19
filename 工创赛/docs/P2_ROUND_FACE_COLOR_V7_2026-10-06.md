# 直接蓝错色图与V7当前圆面判色

输入20261006-183128-046231.zip，CRC通过，SHA256 `9c77457e18f09c85b8ec2b5cf0c94a3fa3c994eae2548f3c769de963d480d69f`。入口SHA与分颜色取证版本匹配，实际保存first-lightblue-with-blue前后原图，不再被红手部事件抢占。831结果中523次浅蓝、201次蓝、1次红候选，最长浅蓝缺失约13.233秒；未签署颜色连续性通过。

## 已定位的机制

直接图中浅蓝圆面仍可见，侧面/阴影为深蓝。前一结果整体蓝44.83%/浅蓝33.72%，差不足15%，进入严格圆面复核，可得浅蓝圆支持（小圆面色票88.28%）。下一结果整体蓝47.35%/浅蓝32.00%，差刚超过15%，整体先被接受为blue；该轮廓未进入拒绝列表，浅蓝圆提议的父区域支持变成0，失去复核机会。这是接受顺序与侧面颜色干扰，不能继续只放宽饱和度或锁住旧浅蓝标签。

## V7候选

颜色配置保留V6的蓝S≥150/浅蓝S≤149，镜头/相机/灯/曝光不改。仅对被整体接受为blue、蓝票不足65%且浅蓝票至少15%的混色物料，提取当前轮廓内较亮区域（非削顶V的60分位阈值），经局部闭合后寻找明亮圆面。必须相对父面积≥20%、圆形度≥0.60、实心度≥0.90、长宽比≤1.4、可用非饱和比例≥35%和数量≥500、原始蓝族色票≥65%且领先≥25%；唯一圆面才确认，证据不够UNKNOWN，不继续用侧面确定颜色。

直接图明亮圆面bbox=[909,345,219,221]，浅蓝票69.45%、蓝30.55%，正确输出lightblue。选取与颜色票都来自当前图像，不依赖watch参数或历史颜色。V7仅是明亮圆面的图像启发式，不证明三维顶面、毫米抓取点或实际姿态；近边透视/高光/遮挡仍需独立验收。

整体轮廓/圆面参考变化时，面积不能直接同比。内部同色近邻关联继续保持100px距离门，参考变化使用真实bbox最小直径比0.5～2检查尺度；同参考仍使用原面积比，500ms内部暂存和150ms新鲜度均保持。记录geometry_reference_changed，跟随器已有源变化即取消该帧速度外推逻辑；当前结果仍ready/motion_allowed=False。不声称内部同ID就是物理身份已闭环。

## 软件验证

全项目102项软件测试通过。新增直接错色图（不借助历史提示）浅蓝圆面验证、参考切换同对象内部ID保持、小无关区域不误沿用ID。旧14张静态六色/单双图，红色及两轮深蓝保存事件图，V6/V7的已知颜色标签一致。静止浅蓝既有接受保持，不因该项修订丢失。输入图中未保存的201个蓝结果不称全量修复。

临时完整V6安装V7、--check、配置逐字节保持、原两文件备份、重复安装和自定义代码拒绝覆盖通过；尚无V7香橙派运行/耗时/连续性数据。原各版包与配置候选保留，Root默认配置仍是历史基准，正式同步须以板端回传认可为证据。

报告脚本`scripts/analyze_round_face_evidence.py`，输出`output/round-face-v7-analysis-20261006/report.json`与对应原图推理叠加，不改变原ZIP。

## 当前交付

`output/round_face_patch_v7_2026-10-06.zip`，SHA256 `e4f489c31f08c3b9bdbf13baa65ccbef981514f8588c3759aec8ddf659f27916`，只改检测器/内部轨迹元数据与参考切换匹配；安装器`output/apply_round_face_patch_v7.py`，原文件备份到patch-backups/round-face-v7-20261006，--check失败回滚。颜色/设备参数文件不替换。

Windows PowerShell使用刚查询的香橙派当前IPv4：

```powershell
$opiAddress = Read-Host "请输入香橙派当前IPv4地址"
scp "D:\Codex\工创赛\output\round_face_patch_v7_2026-10-06.zip" "D:\Codex\工创赛\output\apply_round_face_patch_v7.py" "orangepi@${opiAddress}:/home/orangepi/p2-integrated-20261003/"
```

关闭旧预览，RealVNC终端：

```bash
python3 /home/orangepi/p2-integrated-20261003/apply_round_face_patch_v7.py
```

显示Round face v7 applied/already applied且无错误后：

```bash
python3 -u /home/orangepi/p2-integrated-20261003/p2_color_follow_audit.py --watch-color lightblue
```

只放浅蓝，底面贴桌面，先静止5秒再复现此前会变blue的轻推方向，松手后静止5秒，Q退出，回传标签/ID与Evidence ZIP。此步仍桌面，安装、机构和场地切换需提前提醒。

已核对VIS-01、FIELD-07（R）、VIS-02/03/09/11/12/13/14/15、CAL-02、RUN-06（E）；QR-04（Q）待技术答疑。板端颜色连续性/性能、强反光/手部负样本、其他色、毫米/时间/高度及真实动作/场地均未闭环。
