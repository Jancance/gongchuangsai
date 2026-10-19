# 浅蓝静态错分与V6颜色候选

输入`evidence/20261006-092130-937051.zip`，SHA256 `e778e3c9f5ec3b7f440a7330b7925ac7d36a505a4a52fac226b26adf5ace40f7`，CRC通过。watch_color=lightblue，224结果中223个可见目标均判为blue，另1帧启动超时拒绝；是持续静态错分，不是跟踪断帧或用户把watch参数写错。原始范围仍是项目候选而非正式相机颜色标定。

## 原因证据

首个异色原图中浅蓝顶面无遮挡。完整物料联合轮廓bbox=[515,145,264,359]包括暗侧面/阴影，蓝票约57.87%、浅蓝19.26%、黑23.79%，故旧算法输出blue。人工选取顶面中心开发ROI=[570,290,140,140]，HSV饱和度中位约140、90分位145；旧蓝色S≥140、浅蓝S≤139恰将大量真实浅蓝顶面归蓝。对照深蓝顶面样本饱和度主要255，两者并非无法区分。暗侧面另外增加蓝/黑票，不能把原因仅归于侧面。

可复現脚本`scripts/analyze_lightblue_evidence.py`，报告在`output/lightblue-v6-analysis-20261006/report.json`。固定ROI仅作这一张开发图的诊断，不加入运行识别位置。

## 候选与为什么选150

仅把蓝范围下限饱和度140→150、浅蓝上限139→149，Hue/Value、红黄绿黑、相机设备/分辨率/曝光/白平衡均保留。记录_candidate_update，原calibrated_for_this_camera=false标记保留，不宣称正式六色标定。

软件同时允许蓝/浅蓝/黑混票的AMBIGUOUS_COLOR联合轮廓交给既有严格独立圆面分支再检查，仍需真实色票、形状/圆周和非饱和质量，不直接强制浅蓝。150、160、170、180等分界开发比较：较高分界会损失某些已有深蓝高光图的蓝票；150在本次保存图上修正浅蓝并保留深蓝。不能因深蓝多数255就忽略其高光局部低饱和。

## 软件验证及边界

浅蓝保存的3个事件PNG均输出单个lightblue（同一静态场景，非3个独立场景）。对照已保存红/深蓝事件图，以该帧之前、年龄≤500ms的实际提示回放，未新增漏检或错色；部分已过期/原就未知的图仍UNKNOWN，不补未来提示。旧14张静态六色/单双图已知标签一致。

新增3项候选回归，包括两项范围/元数据以外配置不变、实际浅蓝图、深蓝强光严格蓝支持；全项目98项软件测试通过。完整临时V5部署安装候选后--check通过，原颜色配置逐字节备份，其余配置不变、重复安装及自定义配置拒绝覆盖通过。现有Root默认颜色配置保留旧基准，候选JSON在output单独存放；待用户板端认可后再同步当前默认值，避免提前把待验证参数冒充已完成标定。没有V6板端分类/运动数据。

## 当前交付与复测

`output/lightblue_patch_v6_2026-10-06.zip`，SHA256 `2de8576f2344a876c43b4eafbd1833b58b7d710d014c5b1db22704dfe950b2ab`；安装器`output/apply_lightblue_patch_v6.py`；可审阅颜色文件`output/lightblue_color_candidate_2026-10-06.json`。更新检测器及实测版本对应颜色配置，备份到patch-backups/lightblue-v6-20261006，版本不符不覆盖，--check失败回滚。

Windows PowerShell：

```powershell
scp "D:\Codex\工创赛\output\lightblue_patch_v6_2026-10-06.zip" "D:\Codex\工创赛\output\apply_lightblue_patch_v6.py" "orangepi@10.31.154.211:/home/orangepi/p2-integrated-20261003/"
```

关闭旧预览，RealVNC香橙派终端：

```bash
python3 /home/orangepi/p2-integrated-20261003/apply_lightblue_patch_v6.py
```

显示Lightblue v6 applied或already applied且无错误后：

```bash
python3 -u /home/orangepi/p2-integrated-20261003/p2_color_follow_audit.py --watch-color lightblue
```

仍只放浅蓝，底面贴桌面。先静止5秒检查label为lightblue，再保持顶面可见轻推2～3cm，静止5秒，Q退出。回传标签/框/ID与Evidence ZIP。若静止仍错分，保留本轮原图再诊断，不强行改watch颜色。深蓝回归及其他颜色实测后续继续，当前不要求装机/机械臂。

已检查VIS-01（R）、VIS-02/03/09/11/12/13/15、RUN-06（E）；QR-04（Q）继续待技术答疑。候选板端浅蓝/深蓝分离、强反光与手部负样本、其余四色、真实身份、毫米/高度/时间及机构/场地仍未闭环。
