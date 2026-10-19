# V8失框证据与V9等价计算优化

## 当前证据

20261006-195642-102168.zip，SHA256 67cc18b6a3cc6ca0c833c9c4f7f7218ad196b1ecd9121478d1c7a26a683e1b89，CRC通过；检测器7b15c2a817348e976378d342355a5e5fdebe8ee6a9fad8e55f1e17ad6d28713c匹配已交付V8，跟随器/轨迹器/取证入口匹配当前版本。用户反馈两种摆放颜色正确但容易失框，不能签署连续性通过。

350结果包含49 STALE_RESULT、295 FOLLOWING_PIXEL、6 SEARCHING。49个超时结果全部已检测出blue和lightblue，却因主机读取完成至检测完成年龄超过150ms而拒绝输出并重置内部轨迹。首次失框第6结果：年龄155.515ms，检测126.972ms，原图两件完整可见且两色均正确。主机年龄中位130.458ms/95分位158.309ms/最大208.561ms；检测中位112.862ms/95分位131.917ms/最大193.519ms。较频繁失框与32次浅蓝编号变化主要受处理耗时影响，不继续调整颜色阈值。

最长闭合浅蓝缺失1.172秒的结束前图另含手部与阴影进入右侧圆面，存在真实几何可见性问题；49超时与该手部段分开分析。开始/末尾未闭合缺失不包含在longest字段。多目标blue286/lightblue283/red5是观测次数，不是350帧准确率。手部小红误检仍未修复，记录器开销95分位1.618ms/最大16.876ms不能单独证明整体掉帧来源。

## V9改变（E）

只优化检测器和F跟随调用，保持V8颜色配置与圆面门、100px关联、150ms新鲜度、500ms内部暂存、速度外推取消和动作禁用。

- 已配置路径复用当前帧HSV和原颜色工具的配置验证/蒙版函数，去掉第二次整图HSV、未使用的灰度及会被覆盖的旧预设色票计算。
- 将轮廓腐蚀/色票限定到当前bbox，显式零边界保持原整图有效像素集合。
- 浅蓝圆面三个切分共用父蒙版与一次分位数统计。非削顶浅蓝像素少于325时跳过必然失败的复核（原接受门至少500有效像素、65%浅蓝）。三个切分、多面/相反色拒绝门保持。
- 原离线analyze默认仍返回叠加图；F调用关闭随后不会使用的叠加绘制，实际预览仍由draw_follow画本轮新鲜目标。输入原图不修改，不画旧框。

以上是计算等价工程改动，不把150ms阈值写成官方规则，不延长旧目标可用时间。原开源文件、四工具及其他模式不替换。

## 验证

111项项目测试通过。新增这轮40张保存原图在相同过去新鲜提示下，V8/V9除版本号外整个结果字典完全相同，包括颜色、bbox/中心、分数、拒绝理由和圆诊断；未配置模式与默认叠加兼容；优化后超过150ms仍清空输出。保存图不代表350帧全量图像回放或实机性能通过。

本PC同一首次失框图，预热后交替30次测量：V8检测中位70.859ms/95分位81.836ms，V9关闭未使用叠加中位44.410ms/95分位52.414ms。环境不同，不能作为香橙派耗时或帧率承诺。脚本scripts/analyze_v8_latency_evidence.py，报告output/v8-latency-v9-analysis-20261006/report.json。

scripts/check_follow_speed_patch_v9.py对临时完整V8安装包执行--check、重复安装、原代码字节备份、所有配置逐字节保持、检查失败回滚及自定义源码拒绝覆盖，均通过。--check不打开相机/串口。安装器同时核对本轮V8配置/轨迹器哈希，发现用户另改过内容会停下而不覆盖。

## 本次交付与下一项

output/follow_speed_patch_v9_2026-10-06.zip，SHA256 536aa0f75fc0bcf9a89cc9ad92a0513b4ef52c626d20885a0a97ba26c4e1ea56；output/apply_follow_speed_patch_v9.py。只写project_detector.py、project_follow.py，配置不替换，原代码备份patch-backups/follow-speed-v9-20261006。旧各版包保留。

Windows PowerShell：

```powershell
scp "D:\Codex\工创赛\output\follow_speed_patch_v9_2026-10-06.zip" "D:\Codex\工创赛\output\apply_follow_speed_patch_v9.py" "orangepi@10.31.154.211:/home/orangepi/p2-integrated-20261003/"
```

香橙派RealVNC终端，关闭旧预览：

```bash
python3 /home/orangepi/p2-integrated-20261003/apply_follow_speed_patch_v9.py
```

显示Follow speed v9 applied/already applied且无错误后：

```bash
python3 -u /home/orangepi/p2-integrated-20261003/p2_color_follow_pair_audit.py --watch-color lightblue --sample-every 2 --max-samples 12
```

先只验证两件同时完整入镜、分开、手离开静止20秒，保持光照/相机/曝光/增益/WB，Q退出并回传Evidence ZIP。这项针对静止时的超时闪框，不将移动手部混入静止验收。目标为新鲜结果始终正确标色，核对静止段超时/失检是否消除；后续再做互换与移动，不把同ID或PC优化当真实跟随抓取通过。IP变化须hostname -I重查。

已检查R：VIS-01、FIELD-07；E：VIS-02/03/09/11/12/13/14/15、CAL-02、RUN-06。Q：QR-04保持待技术答疑确认。未闭环：V9板端实际耗时/连续性，遮挡/强反光、手部误检、六色困难集、真实身份/曝光时间/毫米高度、机构和场地。仍在桌面，无需安装机械臂；安装与现场测试切换前提醒。
