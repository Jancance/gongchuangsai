# 每次到位后新帧识别、静止换位与回缩30mm并行转槽盘

## 最新上车：073dfa1e三件实物通过，用户要求同版再跑一轮

用户明确全部就绪授权部署：02:33:27 Keil Verify OK，完整185764字节读回073dfa1e一致，主控待机/故障0。新Pi包26文件SHA核对、ZIP10d0ec98、实际预览/持续帧/SC2待机核实并更新自启备份；未改HSV或几何。仅用户一次PC13，主机全程只读。

本轮完整`loaded-retract30-gate/pc13-watch-20261116-023518.json`：3/3、cycle8/fault0；一次纠偏5.986→1.552mm，首见state2到双鸣结束约3450ms，仍超3秒且采样存在间隔。双鸣后45.617秒、整轮55.989秒，单件7417/6700/6507ms，机械合计20.624秒；末松爪302ms，最终DM0°/X−1.8°/Z599.733p。用户明确“三件放稳归位无碰擦，提前旋转正常”，本轮按此范围实物通过，不扩展为重复性或碰撞包络保证。

Pi原始选中观测取回`observe-ready-each120/pi-live-observations.jsonl`，来源/哈希见`pi-live-summary.json`：三次观察均initial_stopped，各次确认5帧，首件净角约−123.80°、第2件−120.45°、第3件−120.35°首次提交。新帧围栏在三次分别接纳55/179/54帧，旧帧不参与；仍是主机读时间证据，不证明曝光真值。完整运动前窗停盘例外仍未启用。

用户随后明确要再来一遍，并确认三件复归源盘、车载槽清空及停稳空爪承托。02:40:08执行一次真实SYSRESETREQ，完整镜像匹配、cycle8→0、RUNNING/fault0，无主机动作请求；证据`retry-reset-20261116-024008.json`。Pi进程退出后已恢复同版包，未更换逻辑或再次烧录；当前待撤承托/预览确认后接监测再通知一次PC13，尚未记录第二轮结果。未上传GitHub。

## 本轮已澄清的目标

基线为本轮实车通过f6460ea3（原料区观察210°、槽盘36.5°、松爪后300ms、170预收/140夹紧、底盘30mm/45mm/s）。用户提出以下新要求，并经grill-me确认：

1. 每次伸缩到观察位并有真实到位反馈后，清空旧稳定计数，只采到位后的新画面累计5帧；首件及槽1/2返观察均如此。
2. 每次到位后物料若静止，不立即抓；须后续转过下一个位置（本轮确认仍120°），再采新停稳5帧抓取。到位后首次本来在转，停稳5帧可抓。
3. 用户另外确认例外：伸缩到位前转盘在转、恰好到位时停下，可用到位后的新5帧直接抓。运动期间可以只观察运动，不累计抓取5帧或发抓取。但当前运动相机补偿没有验证，不能用相机移动引起的像素位移伪造转盘运动证据，详见下段限制。
4. 抓后升降达到当前已验安全回升高度（原Z1800目标，非机械真0），伸缩实际距原位≤30mm即可让4310转槽盘36.5°，X继续回0；同组各轴全完成后才松爪。速度、目标、停止和槽盘300ms保持。

本轮仅本地实现及验证，未授权新烧录/试跑。不得以仿真通过冒充真实扫掠或识别通过。

## 观察准入的证据与限制

查实际MCU，初次观察前已等Integration10准备DONE；抓后返观察在完整group DONE后才SC state6。普通X到位沿用既有5mm位置窗、真实速度0/驱动到位状态；不是软件直接在X正在运行时无条件放行。Pi捕获循环先取图，process才收SC2状态，故当前处理帧可能在收到ready前已采集，frame_count>=2不能证明5帧都在到位后。

最小方案复用SC2 run/state6/pose作为本次完成epoch，不扩UART协议。每个新epoch丢弃read_completed<=ready时刻的帧，第一张read_completed>ready的帧也仅作为围栏，之后同一顺序capture worker更大sequence才可检测/累计5帧。依据已只读检查的LatestProjectCamera/单worker read→publish→next read机制，后续主机read开始晚于ready；缺元数据不伪造now，worker代际/状态/pose改变重建围栏，重复心跳不清计数，旧pose不能回退。

围栏只证明主机读边界，尚不证明传感器曝光时间或USB/V4L2底层缓存完全清空，sensor_exposure_epoch_verified保持false。当前state6也不是持续同步的yaw/X/Z遥测，仅为原动作完成语义。本轮不擅自收紧5mm到位容差。

每次观察按当前目标色及剩余颜色重建初态，删除后两件直接绕过120°的旧行为。首次静止分类后重新锚定后续120°；跨120°时旧停稳历史不能沿用，应在过120后重新采5帧。真实帧间隔、像素速度门限沿用原配置，不新加停车故障。

本机MATLAB`validate_observe_ready_each120_20261116.m`前置928检查通过，结果`observe-ready-each120/matlab-result.json`明确staged_validation=true、moving_before_ready_exception_validated=false。相机运动反例表明当前圆心平移/固定whitening不能消除镜头旋转及透视变化；运动中绝对像素速度不能当转盘运动真值。首件state5机构已停稳可作为有限前窗候选，但返观察state7相机仍动，无同等证据。当前新视觉实现只记录运动前窗UNKNOWN，不开放这一例外；这项用户目标仍未完整实现，不宣称全部完成。

## 机械并行组的前置与作用范围

原owner抓后phase5建立EXT/LIFT组，等两轴全完成才新建DM-only phase12。本轮改为同一个DM|LIFT|(必要EXT)组，目标DM36.5/X0/Z1800；最初只允许X/Z准备和启动，DM既不prepare也不启动，避免常使能DM准备即改目标。

放行DM必须满足本组Z真实done、Z新鲜位置在原安全回升窗口、X本次新鲜反馈距home绝对值≤30mm，以及原净升>65mm条件。已经done的Z在等待X期间需由sole CAN owner按需只读刷新；SPF默认不再poll done轴，若只看旧缓存会永远等到超时。近侧无X运动也需真实读取X，不假设在0。X未done可继续回零，不重建动作组；全部轴done才切原phase12释放路径。STOP/故障仍覆盖整组。

前置本机MATLAB`validate_loaded_retract30_gate_20261116.m`5533检查、5184门控组合、54合成调度通过，包含DM提前prepare和doneZ不刷新两反例。模型不是物理动力学/扫掠、碰撞或节时验收，相关标记保持false。

## 验证/交付记录

视觉与机械分开隔离产物，旧上车包和f6460ea3保持。实现模型分工按用户偏好：6.1 Sol high规划/审查，medium写实现；主代理整合交接，未声称主聊天自行切模型。

新视觉包路径`output/source_observe_ready_each120_pi_20261116/`，源模块`observation_frame_epoch.py`与`each_observation_rotation.py`，证据`evidence/integration/2026-11-16/observe-ready-each120/`。高档已指出并要求修复120°之前5帧被跨门复用，以及MCU快复位后新run被旧排序永久拒收两项，最终通过证据以后续python-result为准。

新机械候选`tmp/source_loaded_retract30_20261116/project`，只改owner `CHASSIS/USER/pzq_commit_overlap.inc`，其余固件、backend与generic SPF保持基线字节。证据`evidence/integration/2026-11-16/loaded-retract30-gate/`，刷新沿用原150ms新鲜度，不另增100ms门限。最终实际C/双构建及高档审查结果以该目录candidate/checks为准，未烧录。

机械最终结果已核：实际owner门控+真实SPF组163检查（含age150不刷新/151读、Z/X读失败不放行），原三位置三槽owner653、故障607、末件950（含MCU658）、锁轮65/组10通过，双Keil完整0错误0警告，高档独立审查无阻断发现。最终在线AXF SHA256 `073dfa1ed9fe97a26c8eef70dd899ed8b1d0219435a9d97f5ff022bd1aa12758`，Offline `b996e2cd71d50e4a6a5f4b30306879e13b10a483e1d0376b95fff41b128c8f68`；此前100ms提前刷新中间镜像7d5881d6不是最终交付。

视觉两项审查问题已先补本机MATLAB `validate_ready_boundary_rebind_20261116.m`39检查后修正：initial_stopped门跨120°时清direct历史，119.9°停5帧后缓动越120.1°不立即发包，必须重新5帧；见到完整idle全零签名先悬置旧ready，再等run1/pose1/state1/prep0/item0的准备签名重绑，无新增延时门限，快速复位不再被旧run永久拒收。修复版17项Python测试通过（含真实process AST），打包26文件/19个Python语法及ZIP清单检查通过。最终ZIP SHA256 `10d0ec98820a543a09e1e67196e8e5785da0dab2aba8245a31dcc0328db75902`；旧16项中间报告已标superseded，不作为最终证据。

高档独立复核最终修复版未发现新阻断问题：复跑17测试，并额外执行真实process的item0/1/2各自静止→119.9°停5→120.1°后重新1..5帧用例，三件均在新第5帧才OC2。MCU快速复位两阶段重绑、旧pose不能改epoch、worker代际重建围栏亦通过。审查只覆盖已实现主范围，完整运动前窗例外仍为未实现，曝光时效未验证。

当前固件073dfa1e与新Pi包分别为本地候选，车上继续保持f6460ea3与旧五帧包；载台上一实车验收三件已占用。未烧录、未部署、自启未替换、未复位或发新动作。运动相机预到位停盘例外仍未实现，不能把可实现主范围测试通过描述为完整需求全通过。

已核R RUN-01/02/04、SRC-01/02、FIELD-06、PWR-01、MECH-01/02；E SRC-04、CAL-05、VIS-12/14/15、RUN-06及本轮观察epoch/每次120/回缩30mm选择。现有安全高度和30mm为工程参数，不新增官方规则，Q保持待技术答疑确认。3秒纠偏/25秒目标、预到位停盘例外、真实曝光/角度/扫掠均待闭环，无GitHub提交/上传。
