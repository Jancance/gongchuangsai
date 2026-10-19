# 单蓝色强反光证据与V5候选

输入：`evidence/20261006-085047-455357.zip`，SHA256 `21807753f1866506a4f02a40098c878fbc81d8bec66ea87f9012d2c2652b840f`，CRC通过。watch_color为blue，模块SHA对应已安装V4，新颜色入口SHA对应交付版本，确认测试归属。统计/事件回放脚本为`scripts/analyze_blue_highlight_evidence.py`，结果在`output/blue-highlight-v5-analysis-20261006/report.json`。

## 直接证据与限制

539结果中491个FOLLOWING_PIXEL、48个SEARCHING；491次blue观测及另1次额外red候选，15段蓝色失检、8次蓝色可见编号变化，最长1.409386秒。没有STALE_FRAME/STALE_RESULT；主机结果年龄95分位117.397ms、最大149.001ms，所以本次不能归因为过期拒绝。部分编号变化也发生在可见结果之间，单靠高光修订不能声称修复全部身份匹配。

首次失检第203结果：手部与物料联合轮廓碰边，现有圆提议周向支持约68.1%/70.8%，低于75%门。图像中蓝色圆面主要仍可见，白色/低饱和高光把纯蓝蒙版周向切断，现有逻辑把颜色蒙版同时用于形状验真，造成不足。最长段恢复前内部轨迹已过500ms，搜索提示已失效；不能用恢复后的未来位置补回缺失帧。

另1次额外red与blue同时出现，只有JSON而无该帧原图，不能直接判定实际红物料或手部错检；新入口增加此类首个异色候选原图保存。用户“其他情况下没问题”为观察反馈，不替代这项记录。

## 修改与软件证据

仅修改检测器和按颜色取证入口。蓝色的几何提议/圆周检验使用原蓝蒙版、浅蓝蒙版及亮且低饱和/削顶高光的候选合并，原始蓝色色票完全保留；高光只作形状假设，不伪造蓝色像素。圆周75%、原色比例65%/领先25%、非饱和可用比例35%及500像素、父区域支持、歧义和时效门不变。无法证明颜色/形状仍UNKNOWN。红/其他色的几何蒙版保持原路径，颜色配置未调整，V4红色包与基准档保留。

首次强反光失检图离线由无目标变为当前blue圆，周向约88.9%，非饱和有效像素中的原blue票约73.6%；不是539帧全量回放。未恢复已经失去提示的最长段前帧，不延长500ms或画旧框。

全项目95项软件测试通过（含前次颜色工具2项与新3项）：实拍蓝高光、纯白/浅蓝不当蓝、已保存红色图对照V4标签不变。旧14张静态六色/单双物料图标签一致。临时V4部署应用V5后--check和新颜色入口--check通过，配置字节保留、备份、重复安装和自定义代码拒绝覆盖通过；尚无V5板端耗时或效果数据。

## 当前交付与下一步

`output/blue_highlight_patch_v5_2026-10-06.zip`，SHA256 `e929f70c927d46cde3fc9a7b4fc70ec509fb1d49ea66831bf9b2ed1f6eef968b`；安装器`output/apply_blue_highlight_patch_v5.py`。两文件版本验证，备份到patch-backups/blue-highlight-v5-20261006，配置不替换，启动检查失败回滚。需要用户保持相机/灯/曝光不变，只复测单蓝色先前强反光场景。

Windows PowerShell：

```powershell
scp "D:\Codex\工创赛\output\blue_highlight_patch_v5_2026-10-06.zip" "D:\Codex\工创赛\output\apply_blue_highlight_patch_v5.py" "orangepi@10.31.154.211:/home/orangepi/p2-integrated-20261003/"
```

关闭旧预览，RealVNC终端：

```bash
python3 /home/orangepi/p2-integrated-20261003/apply_blue_highlight_patch_v5.py
```

显示Blue highlight v5 applied或already applied、无报错后继续：

```bash
python3 -u /home/orangepi/p2-integrated-20261003/p2_color_follow_audit.py --watch-color blue
```

静止5秒，从先前强反光角度保持顶面可见慢推2～3cm后松手，静止5秒，Q退出。回传框/标签/编号与Evidence ZIP。光照和参数保持本轮一致，不同时调整曝光以混淆比较。本轮仍是桌面，非机构或场地验收。

已核对VIS-01、FIELD-07（R）、VIS-02/03/09/11/12/13/15、RUN-06（E）；QR-04（Q）待技术答疑。蓝色困难场景、额外红候选、其他色、连续身份、毫米/时间/高度及真实抓取/场地未闭环。
