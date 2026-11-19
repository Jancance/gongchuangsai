# 第二/第三段统一桌面离线交付（2026-11-16）

当前用户已明确断电。本轮只操作Windows本地文件，没有SSH、相机、串口、probe、下载、安装或修改现场自启。此前现场安装的是55项归档（SHA简写15acd24…df492）；当时PID11517属于历史记录，断电后无有效当前PID。59项为其后的本地可观测性候选；本次在此基础上增加安装/回滚/模板/离线配置工具，最终项数及SHA以交付manifest与外部package-report.json为准。

E：MCU决定执行区域。有效SC2/SC1走原第二段左纠偏、右抓取；有效S31走第三段左圆环纠偏、右放置。一个应用、一个相机worker、一个串口owner。Pi不从画面猜场景，不扩固件协议。S31只有槽颜色和原始item，没有positions/first/count，右图只写“按主控任务映射”，颜色不等于目标圈，item0不证明实际库存。state6仅软件完成，不是假放置确认。

默认启动`bash start.sh`或`bash start-preview.sh`：状态RX开启，观测TX关闭。`bash start-trial.sh`为显式选择的观测桥；只发送观测、不发送电机命令。第三段默认配置`calibration_verified=false`且矩阵/参考为空，不能发送有效纠偏；未标定时trial可能发送valid=0观测，不能把“无有效观测”写成“串口完全无TX”。本轮没有放开标定。capture摘要serial DISABLED指旧底层dummy；实际状态看CONFIG、MCU_STATUS/MCU_STAGE3_STATUS、observations.jsonl。

## 离线重现（Windows，无硬件）

从仓库根目录执行：

```powershell
python scripts/stage23_pi_install_20261116.py --prepare-only
$env:STAGE23_REPLAY_OUTPUT='D:/Codex/工创赛/evidence/integration/2026-11-16/stage23-offline-ready/default'
python output/stage23_desktop_20261116/evidence/test_entry_preview.py
python output/stage23_desktop_20261116/evidence/test_region.py
python -m unittest discover -s output/stage23_desktop_20261116/stage3_candidate/tests -v
```

真实入口entry.main→capture.main→callback使用替身相机/串口，输入SC2→S31→S31→SC2；保存三张PNG与JSON，证明区域往返、单串口open1/read4/write0，不代表当前真实相机、实时曝光或实车通过。适配器回归另核新pose epoch、preview TX0、未标定valid0。前置MATLAB区域拓扑与preview RX/TX模型脚本/通过日志保留在evidence目录，本轮未改变控制拓扑。

包内保留第三段上游README原件，里面“尚未统一”等说明是其独立包历史，不作为本统一入口的现状。source主分支检测、HSV、rotation、OC1/OC2和渲染区段保持上游文本保真。

## 可选导入历史搜索ROI，仅像素预览

不覆盖默认配置、原source/tools或现场HSV。已有历史采图`evidence/integration/2026-11-16/stage3/field/initial.png`与`search-only-config.json`仅可用于离线回放。使用独立输出路径：

```powershell
python output/stage23_desktop_20261116/import-pixel-profile.py --search-profile evidence/integration/2026-11-16/stage3/field/search-only-config.json --output evidence/integration/2026-11-16/stage23-offline-ready/pixel-profile.json
$env:STAGE23_PIXEL_PROFILE='D:/Codex/工创赛/evidence/integration/2026-11-16/stage23-offline-ready/pixel-profile.json'
$env:STAGE23_REPLAY_IMAGE='D:/Codex/工创赛/evidence/integration/2026-11-16/stage3/field/initial.png'
$env:STAGE23_REPLAY_OUTPUT='D:/Codex/工创赛/evidence/integration/2026-11-16/stage23-offline-ready/historical-pixel'
python output/stage23_desktop_20261116/evidence/test_entry_preview.py
```

导入工具只复制search_calibration与尺寸，保留原件SHA来源，reference/A2/B3为空、verified=false。历史搜索框不是当前现场标定，不导入离线矩阵候选，不以画面中心自动确认TCP。若只需原始画面，清除这三个环境变量后回归默认。现场将来若选用独立像素profile，用`bash start.sh --stage3-config /绝对路径/pixel-profile.json`，仍是preview。

## 上电后操作路径（本轮未执行）

1. 先确认上电后实际运行程序/当前PID和自启入口，不能复用历史PID。保留原55项安装目录、原source目录及原自启文件可恢复副本；不要同时开两个应用。
2. 安装前核外部依赖：`/home/orangepi/p2-source-observer-20261106/vision_app/code`，以及`/home/orangepi/vision_tools/material_pickup_top65_20261114.json`和该配置引用的HSV文件路径/内容/SHA。当前断电无法核，不声称已齐或hash已验证。出现识别问题先调对应阈值，再核实际配置与新采图；不拿旧图验新阈值。
3. Windows生成ZIP，人工复制到Pi独立临时目录并解压。Pi执行`python3 /解压目录/install-local.py --destination /home/orangepi/vision_tools/stage23_desktop_20261116`。只复制核hash，不启动/停止、不改自启、不改包外配置；已存在且非旧manifest的本地修改会保留并报错。若原程序仍在运行，应先人工正常退出后再更新其目录。
4. 原程序已退出后，在RealVNC终端运行`bash /home/orangepi/vision_tools/stage23_desktop_20261116/start.sh`。这是预览，观察状态RX、区域切换、两视图、外部配置hash与新session记录。旧lock被占用时入口退出，先核当前owner，不删除锁来并开。
5. 自启模板只在人工确认新preview验屏通过后使用：先备份并替换原入口，确保只保留一个入口，再把`stage23-preview.desktop.template`复制为`~/.config/autostart/stage23-preview.desktop`。本轮只交模板，未修改现场自启。它默认preview，正式trial必须另有明确选择。
6. 回滚：先确认当前统一PID与原source的绝对`start-preview.sh`，执行`bash rollback-preview.sh 当前PID /原source绝对目录/start-preview.sh`。工具核/proc的cwd与命令后TERM，等待退出才启动原preview；不强杀，不替用户猜PID，不恢复trial或自启文件。自启回滚另将人工备份原文件恢复。

另一安装方式是仓库脚本显式`--install-only`，才会调用原SSH helper；它也只copy/check。脚本无参数或`--prepare-only`均仅本机打包。本轮只运行prepare路径。

R核对：FIELD-06、RUN-01/02、PWR-01、MECH-01/02；E核对：VIS-09/12/14/15、CAL-02～05、RUN-06。R不变，官方Q继续待技术答疑确认。未闭环：新包现场安装/自启验收、真实圆/上下边界、A2/B3/TCP/轴向、相机曝光时效、实物夹取/放置和三件全流程。host read时间仅主机时间围栏，不是曝光时间。
