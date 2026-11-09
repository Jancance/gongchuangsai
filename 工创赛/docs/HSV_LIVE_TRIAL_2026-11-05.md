# 整合5接续：HSV板端收益与实时复测

## 已有证据（E）

用户2026-11-05完整终端输出：8线程、4张保存原图，新旧各20次，exact_results=true；before median127.628843/p95158.963177/max159.339729ms，after109.726001/122.149157/122.286311ms，中位改善14.02727%，绝对减少17.902842ms。P95减少36.814021ms。来源明确为用户粘贴，尚未取回板端board-benchmark.json，归档于evidence/p2-camera/2026-11-05/hsv-mask-candidate/board-benchmark-user-reported.json。

仅此四张图的模块计算比较通过，不代表在线采集、持续负载、完整帧年龄、ID稳定或六色泛化通过。旧实时包266帧中39帧过期清空，仍作为待修复参照。

本机使用scripts/prepare_hsv_live_trial_20261105.py生成output/hsv-live-trial-20261105/vision_app。相对已验证蓝轮廓源仅替换code/project_detector.py、新增code/project_hsv_masks.py；原源文件SHA保持，准备报告见同证据目录live-preparation.json。候选即本次benchmark两个文件，未混入无收益缓存方案。--check退出0：1920×1080、serial DISABLED、metric_geometry_verified=false、motion_allowed=false；额外配置9检查0失败。已有完整HSV立方体/151实拍结果一致性证据沿用，不重复计算。

## 香橙派下一步（E，尚未执行）

关闭其他占用相机的预览。沿用本轮相机、灯光、物料摆放与曝光50/增益32。直接利用板端已有文件建独立副本，不下载新包，不覆盖旧程序。以下整段为同一条以&&连接的命令，任何一步失败都停止后续执行。

```bash
cd /home/orangepi &&
test -f p2-generic135-rejection-audit-20261026/p2_color_follow_audit.py &&
test ! -e p2-hsv-live-20261105 &&
printf '%s\n' \
  'e5014e86b7869d22cacaf01b9cfb30d57e8e6b4ed8670bbf043962f6df448c95  p2-blue-perimeter-20261105/vision_app/code/project_detector.py' \
  'cfbe8b1135059f8d3194fddf59c4a2fa090163e204152041d7c206ca9970e270  p2-hsv-benchmark-20261105/project_detector.py' \
  'ddb301a6bbae5148580e93d1687e5493d721183cdf3add104664a99bdaccb87b  p2-hsv-benchmark-20261105/project_hsv_masks.py' | sha256sum -c - &&
mkdir p2-hsv-live-20261105 &&
cp -a p2-blue-perimeter-20261105/vision_app p2-hsv-live-20261105/ &&
cp p2-hsv-benchmark-20261105/project_detector.py p2-hsv-benchmark-20261105/project_hsv_masks.py p2-hsv-live-20261105/vision_app/code/ &&
python3 p2-hsv-live-20261105/vision_app/run.py --check &&
python3 p2-generic135-rejection-audit-20261026/p2_color_follow_audit.py --app /home/orangepi/p2-hsv-live-20261105/vision_app --watch-color blue --sample-every 5 --max-samples 4
```

预览按F进入物料跟踪，保持红黄蓝完整在视野中，观察约90秒后按Q退出。保留终端输出的精确ZIP路径；新包默认位于/home/orangepi/p2-hsv-live-20261105/color-follow-audits/。本轮还没有取得该实时包，不能填写通过。若目录已存在，停止而不删除，先检查是否已有本轮结果。回退直接使用原p2-blue-perimeter-20261105目录。

取回后检查模块SHA（包括新增project_hsv_masks.py）、三色有效帧完整率、STALE_RESULT及其空目标出口、帧年龄p50/p95/max、各色ID变化及最长缺失、相机/输出配置。现有审计仅自动记录原四模块SHA，新增helper需要单独读回SHA，不能将本机准备报告当板端证明。沿用150ms工程门限，不放宽、不用旧位置补帧。90秒静态观察仍不等于动态转盘、热稳态或全场验收。

## 需求检查与未闭环

R已核：VIS-01（仍需六色）、QR-01/05/06（任务/颜色/实例语义保持）、DSP-01（观察不计抓放）、RUN-01/02/04/07、PWR-01、MECH-01/02（本轮不触发机器人动作）。E已核：VIS-02/03/09/12/15、CAL-02/03、RUN-06；保持阈值、分辨率、时效、像素单位、几何和执行权限。R无修改；QR-04及基线官方Q继续**待技术答疑确认**。

未闭环：优化后的在线帧新鲜度/ID连续性、六色困难集和动态转盘、顶面抓点、本机内参与工作面毫米/手眼关系、视觉通信及纠偏/真实抓放。整合5尚未整体完成。
