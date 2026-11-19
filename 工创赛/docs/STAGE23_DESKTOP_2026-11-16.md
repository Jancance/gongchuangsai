# 第二/第三段统一桌面实际接续（2026-11-16）

> 2026-11-16离线收尾：用户已明确断电。以下现场PID/session均为断电前历史，不是当前运行事实。本轮未连接Pi/相机/串口/probe、未安装或改自启。59项候选已补安装/回滚/模板与历史像素配置导入工具，最终交付64项，ZIP SHA256 `e1a41fc7596f9fba98a11bcbbb144cbe23b57068cc3733ed9f0ef6b68bd2fe3f`。当前操作说明见[统一离线指南](../output/stage23_desktop_20261116/OFFLINE_READY.md)；上电后先核当前owner与包外top65/HSV配置hash，不能复用旧PID或旧曝光时效。

## 当前现场版本

主代理明确报告：统一桌面55项包已安装，归档SHA简写15acd24…df492；旧source PID2256已确认停止，统一程序PID11517，session为`captures/run-20261116-063217-583095824`。最新真实observations显示`region=stage3`、S31 `run2/state7/pose5/item0`、槽色215、align_done0/config_ready0；`preview_only=true/calibration_verified=false/observation_wire=null/motor_commands_sent=false`。因此已实际切入第三段，预览期间MCU状态RX有效、观测TX关闭。

capture原摘要`serial: DISABLED`仅为app.serial_link dummy的硬编码说明，`metric:null`也不代表区域callback未运行。曾据此推断RX关闭的判断已由主代理向用户纠正。实际判断看region/mcu_status与状态日志，不凭摘要猜测。子任务未自行连接Pi或操纵硬件。

第三段仍无真实标定，左图可显示原图和未标定提示，不能声称真实环线检测或毫米纠偏通过。右侧显示真实槽色与主控item；S31不含positions/first/count，目标圈标“按主控任务映射”，不把item当当前已抓槽、不把软件完成当实际放料成功。源区沿用原完整纠偏/抓取视图和检测/HSV/发包门控实现。

## 本地59项可观测性候选

目录`output/stage23_desktop_20261116`，安装准备脚本`scripts/stage23_pi_install_20261116.py`。新候选只明确CONFIG的状态RX/观测TX角色、记录S31状态变化，并在第三段每个新上下文第二渲染帧写session/views-current.jpg；没有为误读日志重复开串口，没有新增C键、算法、HSV、运动命令或保护。59项版本尚未替换现场55项程序，自启未由子任务改变。原source/stage3两包保留。

MATLAB区域分派前置与preview RX/TX分离模型通过；Python区域4项、第三段原8项、真实入口1项通过。真实入口测试执行entry.main→capture.main→callback，硬件和源区检测器替身输入S31 state2两帧再SC2 state2一帧：单owner串口open1/read3/write0，第三段views-current.jpg实际生成。该证据是本机模型/回归，不冒充现场相机或实车验收。55/59包manifest不覆盖板端外部top65/HSV文件，本轮未重新核其现场hash。

## 相关实物接续与边界

主代理已完成预收160候选a54eb0ab下载及完整读回匹配；用户确认160预收正常和180观察无擦碰。槽2红料对准，Z3200→4800→8000分步完成且fault0、未夹紧。用户仅要求下到8000，尚未明确接受最终取料高度。当前目标仍为单件示教，不把UI切区扩大成三件放料验收。

R核对FIELD-06、RUN-01/02、PWR-01、MECH-01/02；E核对VIS-09/12/14/15、CAL-02～05、RUN-06。R不变，官方Q继续待技术答疑确认。未闭环：真实取高、夹紧与中心单件，真实圆/上下边界、A2/B3/TCP与轴向、相机曝光时效、完整三件放料，以及主代理验屏后最终自启入口切换。当前本地记录未提交推送。

## 断电后64项离线交付

原59项功能代码未改；新增只安装不启动的`install-local.py`、需当前PID的preview回滚脚本、自启desktop模板、像素搜索配置导入及完整操作说明。打包脚本无参数默认prepare；只有显式`--install-only`才使用原SSH helper，未运行该路径。ZIP固定排序/时间戳/权限元数据，两次生成逐字节一致；临时目录安装全hash通过、已有本地编辑拒绝覆盖，包外source/tools/HSV不写。证据：[package-report.json](../evidence/integration/2026-11-16/stage23-offline-ready/package-report.json)。ZIP不是包含包外vision_app依赖的独立Python环境，依赖要上电后核对。

真实entry.main→capture.main→callback替身回放扩展为SC2→S31→S31→SC2，单相机、串口open1/read4/write0。默认黑帧模型与历史field/initial.png+独立search-only像素profile两组均通过，三张PNG分别保存source/stage3/source-return；后者显示圆环/边线像素，不确认任何毫米、TCP或轴向。默认和导入profile均`verified=false`、O31未发送。适配器4项、第三段8项通过；源区主分支7341字节逐字节与原source相同，SHA `f8eea63067a57649571755de241a453e85ed069491d9d5047ea93095f8b18164`。证据目录：[stage23-offline-ready](../evidence/integration/2026-11-16/stage23-offline-ready)。原MATLAB拓扑/preview模型通过日志保留，本轮无控制拓扑修改。上述全属本机软件证据，新64项没有现场安装、验屏或实车验收。
