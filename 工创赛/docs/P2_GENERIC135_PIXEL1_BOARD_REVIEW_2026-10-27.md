# pixel1普通主程序：板端静置预览与终端确认

## 已确认的结果（E）

用户提供普通主程序完整预览及终端粘贴日志，启动命令为/home/orangepi/p2-generic135-pixel1-20261027/vision_app/run.py --start-ring --ring-paper-labels 3,2,1。此前--check仅模块/配置门，现确认真实相机已打开、原图预览与普通R展示运行。

画面为RING_STATION，三个绿色十字位于完整纸面环心附近；输出slot1/2/3旁分别显示paper3/2/1(manual)，面板标注MANUAL - NOT OCR、stationary=UNVERIFIED、metric=UNVERIFIED、serial=DISABLED。当前纸面左/中/右实际是3/2/1，仅验证显式人工标签显示正确，不宣称实现数字识别。

终端共25条约每秒一次的状态采样：首条CANDIDATE、Centers=[]、PixelStable=False；后续24条全部STABLE且有三个稳定像素坐标。整个所贴片段无Traceback、模块缺失或Camera unavailable，均注明Serial=DISABLED、Metric=UNVERIFIED。候选到稳定的普通主程序显示路径已确认。

24条稳定状态采样中位为(481.4476,613.2088)/(947.2237,607.0395)/(1405.3805,600.2607)px，最大轴极差0.035637px。这里只是低频日志采样且含原三帧中值，不是全部处理帧、独立曝光重复性或真实定位误差。报告循环FPS范围22.8～28.2，片段中有下降，原因未查；截图显示23.3FPS/408帧。首条0.0为启动时尚未积累FPS。不能据此签持续吞吐或曝光到结果时延验收。

原文、截图、结构化复核分别保存：

- evidence/p2-camera/2026-10-27/pixel1-startup/ordinary-preview-console.txt
- evidence/p2-camera/2026-10-27/pixel1-startup/ordinary-preview-board.png
- evidence/p2-camera/2026-10-27/pixel1-startup/ordinary-preview-review.json

本轮没有每帧JSONL、主机时刻或新的模块/配置哈希，来源依据用户命令、日志和画面，不能将当前本地新ZIP哈希当板端本次实采哈希。板端仍可为中文名修复前下载的同代码旧封装，后续ZIP元数据修订没有用于自动覆盖当前程序。

## 当前门禁和后续

本次普通R入口的完整三环静置显示门通过：实时画面、候选/稳定区分、稳定Centers、显式人工编号来源均有证据。软件整合与当前场景预览项可结束，不重新进行完整三环、两位置、转纸、单一光照或空场基础采样。

未将整机、运动、失检/断连及复杂场景签为通过；普通入口新的LOST/STALE/断连UI路径目前有本地测试支持，历史v2有物理样本，但本轮静置片段未直接覆盖。后续优先按具体原图检查无关圆形/物料遮挡等复杂干扰与残缺环质量；严格强光/阴影、任意数字OCR、独立停稳/自身运动补偿、持续性能仍分别待验。

常用普通R启动入口保持：

```bash
python3 /home/orangepi/p2-generic135-pixel1-20261027/vision_app/run.py --start-ring --ring-paper-labels 3,2,1
```

启动时人工标签仅适用于该摆法；未知时省略--ring-paper-labels。旧lightblue1和独立记录器保留，不与当前入口同时占相机。本轮不改代码、配置或相机参数，不因预览通过开启串口、运动或毫米映射。

R：核SCORE-01，评分事实未修改，像素显示不证明真实落环得分。

E：核VIS-04/05/06/07/12/13/15、CAL-02/03/04、RUN-06。当前门和结论均为工程验证，几何标定按用户决定继续暂缓。

Q：官方未明确项继续待技术答疑确认。复杂干扰、残缺环、任意编号、独立停稳/时效、毫米/手眼、实际落环均未闭环。
