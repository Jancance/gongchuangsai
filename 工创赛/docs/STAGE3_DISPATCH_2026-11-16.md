# 整合24后续：第三段独立分派与圆环放料（2026-11-16）

## 当前结论

用户已批准第三段方案实施并确认现场换版条件；固件本机MATLAB、实际C检查、真实heading及双Keil构建通过，最终高档复审未见阻断安装的问题。05:00:46主代理已烧录在线`815bed44`，Keil Verify OK / Application running；05:01:07只读完整192956字节匹配该候选，state0/fault0，未写动作请求。随后用户首次PC13完成215黄红黑人工装料，明确确认放稳、槽盘转动正常；装料后只读state11/IT2/fault0。

用户摆到第三段、移除承托并确认路径后，第二次PC13进入180°观察姿态。05:05:57保存的25秒watch证据`stage3-live-20261116-050557.json`最终state2/phase6/fault0、IT2/sequence2，各动作请求和arm/lift stop_pending均0；主机writes为空，动作仅由用户PC13触发。用户确认180°观察正常、无碰擦。当前保持观察，没有启动有效观测桥；此结果只验收人工装料及到观察姿态，不代表中心定位、纠偏或放料完成。

候选`tmp/stage3_dispatch_20261116/project`现与上述板端版本匹配；模块源`src/stage3_20261116/region_dispatch.h/.inc`、`stage3_cycle.h/.inc`。Pi圆环工具14文件已安装并校验，安装包SHA256为`5b7d5f2590d48fe14cf2c2b138f945e31bd7e4aec7adf2ce8989a06afca9ff2c`；默认标定false，未启bridge、未改自启。新采集1920×1080原图SHA前后缀`8bd6fc08…abd37`，原件本机取回工作由采图代理处理。真实中心单件、左右标定及完整三件均开放。三小时是工作预算，不是实车通过承诺。本地保留，不上传GitHub。

## 用户已确认的工程范围（E）

1. 原料区与第三段各自独立状态机，由上层根据到达区域分派，模块独占，不能同时发机构动作。测试采用人工搬车模拟上层“已到第三段”，不接自动导航，也不将调试人工步骤视为正式自主比赛。
2. 首次PC13执行车载三槽人工装料：槽盘0°等待既有700ms后停留2s，120°同样等待700ms并停留2s，240°等待700ms后结束；装料顺序215，即黄、红、黑。共6100ms名义等待，不伪造实际舵机到位反馈。
3. 用户搬车到模拟第三段后，第二次PC13进入第三段。观察姿态为DM180°、X0、Z顶部；使用中环XY与真实上下边线参考角完成一次初始纠偏闭环，允许闭环内多次小步。平面判据是**hypot(X,Y)≤2mm**，角度判据是**绝对误差≤1°**，两者须同时满足；这两个数值是用户批准的E，不是官方R。
4. 原入口5°最小转角需在第三段适用链路内支持小角度；仅残差>1°时继续旋转纠偏，不改变其他模式的原转角入口。纠偏完成后四轮位置保持，按槽1/2/3取放三件，期间不再次做视觉纠偏。
5. 测试任务对象是颜色215、位置123；表示槽1黄→环1、槽2红→环2、槽3黑→环3。它不是完整真实四段扫码结果，不得伪称本轮已完成正式扫码任务链路。
6. 放料次序按串行单轴动作组织：选槽→顶部/Xhome/DM36.5°取料位→下降→夹140°→升到顶部→目标环DM→目标X→放料Z→开210°→升顶部→Xhome→下一件；最终DM0°、槽盘240°，不安排回收。具体实际实现与已有机构接口仍需最终代码审查和构建确认。
7. 中心名义DM180°/X0已明确；两侧旋转角/伸缩量和放料高度尚待真实采集，不使用虚假有效默认值。IT3原有3200..29700界限保留，未知高度默认-1。先完成槽2红料到中心环单件，再标定两侧，最后验收完整三件。STOP沿用既有停止能力，不自行新增故障判据或保护阈值。

## MATLAB前置证据与限制

执行命令：`D:/MATLAB1/bin/matlab.exe -batch "run('D:/Codex/工创赛/scripts/validate_stage3_20261116.m')"`。

本机MATLAB R2020a最终退出码0，`evidence/integration/2026-11-16/stage3/matlab-result.json`读回12组检查全部true，覆盖92次状态/原因STOP与既有故障注入、36种任务排列。脚本为`scripts/validate_stage3_20261116.m`。

模型覆盖装料等待边界、任务映射、基准线正负角及180°等价、欧氏2mm和1°边界、先转向后XY重观测及最大误差轴逐次纠偏、单轴取放顺序/四轮保持/最终归位、缺失标定、IT3已有界限、区域独占、STOP/故障出口与禁止自动循环。观察协议模型检查run/pose/seq/valid/age≤200ms及CRC16/CCITT-FALSE（初值FFFF、多项式1021、MSB-first，`123456789`基准29B1，载荷包含末逗号）。

这是离散拓扑及理想代数模型，通过不代表真实底盘响应、噪声/延迟、轮胎滑移、物理锁轮、标定精度、侧位可达性、碰撞扫掠、固件实现或实车通过。实际代码的跨层小转门槛、协议和STOP时效仍须代码审查及针对性验证。

### 新增A2/B3标定拟合的单独时序记录

原固件流程/协议是在原MATLAB通过后实现。新增视觉fit-calibration部分曾在收到补验证要求前先写草稿，尚未运行、打包或部署；主代理已告知用户并冻结草稿，不能把全部代码概括成严格MATLAB先行。随后本轮补充纯MATLAB验证，结果通过后才通知视觉代理可修订公式与执行测试。

补脚本`scripts/validate_stage3_calibration_20261116.m`在本机R2020a运行退出0，结果`evidence/integration/2026-11-16/stage3/calibration-matlab-result.json`读回6组全部true，60个A2合成案例。覆盖A2比例/旋转/各向异性/剪切、椭圆轴与真实外轮廓定义、退化不可逆、B3正右/正前/正逆时针单位及符号/耦合/秩不足、拟合残差与独立验证残差区分。无向上下边线角差必须采用模180的[-90°,90°)表示，不能把端点翻转误成整车旋转180°。

A2由名义95mm圆环及图示150mm间距估计局部仿射，不代表完整透视标定；B3按观测变化与已知单次底盘指令拟合局部响应。命令值不是独立实测位移证据。三条独立样本可恰好零拟合残差，因此必须另留不参与拟合的真实验证样本，分开报告XY范数mm与yaw度。结果字段`physical_calibration_verified=false`，不能凭合成通过自动确认本车毫米标定/TCP或2mm/1°性能。

## 官方名义尺寸：R、E与Q

已从用户本机`竞赛命题与运行.pdf`p4～6提取并渲染视检；原文件SHA、逐页文字、PNG和完整解释在`evidence/integration/2026-11-16/stage3/manual-dimensions/`。

- R：粗加工/暂存区580×150mm；图3两端1/3号中心跨度300mm；图5初赛物料最大直径50mm；圆环线宽1.5mm；表3按φ递推六环外径。
- E：代入φ50得到1至6环外径53、58、65、75、85、95mm。95是最外轮廓直径；若检测黑线中心，最外线中心直径是93.5mm，不能混用。按图3等距居中布局推导相邻中心150mm、上下边界距中环各75mm；150/75未独立标尺寸箭头，保存为图示推导，不新增官方条文。
- Q：独立视觉基准线定义、对称布局精确公差未明确，保留待技术答疑确认。只有真实上下基准线确为该矩形两条长边，才能应用名义±75mm。PDF名义几何不代替真实毫米标定、畸变/透视校正或TCP标定。

`评分与规则.pdf`p2～3确认按任务顺序/位置垂直放置，底面接地后即确定评分位置，不得推行移动；未提供替代上述尺寸的新数值。本轮没有发现需覆盖现有基线的正式冲突。

## 当前设备与仓库快照

05:00:46在线`815bed44`烧录日志为`evidence/integration/2026-11-16/stage3/flash.log`，显示Verify OK / Application running。05:01:07只读证据`stage3-live-20261116-050107.json`完整192956字节匹配、state0、fault0；本轮主机未写动作请求。04:46:29的cd9fa966/186976字节快照保留为此前历史，不再代表当前板端。人工装料后证据`stage3-live-20261116-050350.json`及`050504.json`均state11/IT2/fault0；第二按后的`050557.json`最终state2/phase6/ITsequence2/fault0，用户确认观察正常、无碰擦。

主代理已确认旧USB探针超时经重插恢复，SSH连到`robot-opi5pro`。旧设备快照曾见原料视觉PID2242；当前原料进程不在，退出原因及操作者未知，主代理没有执行kill，不能将其记录为主动停原料进程。Generic相机by-id index0对应`/dev/video0`，1a86串口by-id对应`ttyUSB0`。Pi新工具已安装并完成一次原图capture，bridge未启动；保持相机/串口实际归属核查，不并行占用。

主代理已通过bare fetch取得远程进度`21522db`；根工作目录仍是无提交/无remote的本地目录，全部原有文件保留，不能称根目录已pull或已上传。没有GitHub上传授权，本轮不提交或推送。

## 已检查需求ID与现场下一步

R：QR-01/05（正式编码与颜色号）、RUN-01/02/03（自主/载台运输/放置后禁止推移）、FIELD-06（车道包络）、MECH-01/02（初始尺寸/自主展开）。E：QR-02/03、VIS-04/05/07/12/15、CAL-02/03/05、RUN-06。Q：QR-04、FIELD-05及上述局部基准线/公差未明确项继续待技术答疑确认。人工装料搬车是测试E流程，不改变R。

固件已换版，人工装215及第二按到180°观察姿态均有本轮软件记录和用户反馈。当前保持观察，下一项是检查本轮新原图、真实上下边线/中环及毫米/角度标定；中心定位/纠偏/放料/三件均未通过，不自动启动有效bridge或后续取放动作。

### 当前实际阻断：第三段地面放料高度窗未适配

用户本轮实量DM180°/X0/顶部姿态夹爪最低端到地面205mm。当前`SOURCE_LIFT_PULSES_PER_MM=400`、原料`SOURCE_PICK_PULSES=29700`仅覆盖74.25mm；第三段复用这个原料窗，不能据此完成地面放料。205mm换算82000p只是指尖到地面的几何距离，80000p只是200mm换算，均不是已经验证的机械全行程或放料目标；不得直接以205减物料名义60mm推放料深度，真实夹持后物料底面相对指尖偏移尚待现场示教。

只读代码证据：`lift_ruler_calibration.c`的DOWN_TARGET_PULSES来自SOURCE_PICK，SetDownTarget/Start只允许25000..29700，运动delta亦受该值限制；不存在另一个独立已验全行程值。SPF限制同时存在于`source_parallel_flow.h/.c`姿态校验、`integration10_complete.inc`的IT3准入、`vehicle_source_parallel.inc`步进delta以及`stage3_cycle.inc`的配置校验，工具当前IT轴入口也沿用29700。`HARDWARE/LIFT/lift_config.h`另有旧手动down105000/up233333，来源是旧导程/粗测阶段，不能在本轮400p/mm与新顶部参考下当作已验机械范围。

下一步仅分析第三段专用scope、真实示教深度与返回顶部路径：适用MATLAB验证先于对应实现，原料区29700业务窗保持不变；机械可用行程未闭环，不另造上限、放宽保护或发下降请求。当前仅机械准备到观察姿态通过，仍state2/无有效自动观测。本条为E工程阻断，不修改R或官方Q。

## 现场工具完成判据修复（E）

仅修改`scripts/stage3_live_20261116.py`及对应测试，未改固件、未接探针。所有动作的首错误先锁存并覆盖completed，失败以非零退出，不自动重试；IT3必须看到本次接纳sequence，IT4必须观察本次busy或elapsed变化，拒绝/旧DONE不算完成。IT sequence先读、state/request在采样尾部重读，避免主要的旧DONE与新sequence混帧窗口。body须请求消费且新command_sequence；teach须本轮三路反馈采样更新。continue/new_round的完成范围明确为`accepted_and_started`，不宣称全流程完成。

STOP等待既有IT/SBS/arm/lift pending消费、所需轮STOP ACK及被取消IT的DM停止握手，区分`software_stop_acknowledged`与`physical_stop_verified=false`。既有startup取消状态5/6不会自行变成新终态，现有软件证据不补造伸缩独立ACK或实物停稳。stdout仅打印摘要与证据路径，完整原始samples仍保存JSON。

验证：`python -m unittest discover -s tests -p test_stage3_live_20261116.py -v`实际9项PASS，覆盖拒绝首错、旧DONE、本次完成、采样混帧、body接纳、stop_pending/轮ACK/DM握手；py_compile及无参manifest通过。815bed44 ELF全部附加证据符号存在，脚本独立补读`g_it_action/sequence/elapsed`，不假定原CommittedPickBoard已经返回这些字段。此项是工具证据判断回归，不是控制模型仿真或实车动作验证。

现场依次执行真实上下边线/中环采集及毫米/角度标定、槽2红料中心单件、两侧可达/高度/伸缩采集与扫掠检查，最后一次完整三件。每次明确运行次数并保存首故障，不自动循环或重试；用户确认物料稳定、归位与无碰擦后才能记录实物通过。当前这些步骤均开放。

## Pi工具最终本机交付

A2/B3最终实现于追加MATLAB通过后重审修订，冻结草稿时序记录保持。8项unittest通过，含60个A2比例/旋转/剪切案例、180°无向角、B3符号/秩/耦合、真实函数拟合报告以及协议CRC/epoch/200ms边界；`pi-tests.log`为实际日志。实际合成PNG完成preview、双真实图像边线提取/record-calibration及fit三训练对+一独立验证对，结果在`pi-preview`、`pi-synthetic-calibration`、`pi-image-fit`。sample图环径/中心跨度不满足同一名义几何（ellipse_rank_residual约0.528），仅证明图像路径与工具执行，不能当作毫米精度或本车标定。发布config恢复reference/search/A2/B3为null、calibration_verified=false，合成记录留在证据目录。

输出`output/stage3_ring_pi_20261116.zip`由`scripts/stage3_pi_install_20261116.py --prepare-only`生成，逐文件SHA在包内manifest.json；安装命令`python -X utf8 scripts/stage3_pi_install_20261116.py --install-only`只上传校验及编译，不停止旧原料程序、不启桥、不改自启。现场目录`~/vision_tools/stage3_ring_pi_20261116`，后续最简命令`python3 stage3.py capture --output records/initial`和`python3 stage3.py preview --output records/preview`；待明确释放相机后才运行。

read开始到发送≤200ms只证明主机处理围栏；CAP_PROP_BUFFERSIZE=1是未核实后端执行的请求，不证明曝光/旧帧清空。当前不启动有效bridge。B3数学验证包含耦合，但主控先yaw后XY策略不自动适用于任意可逆强耦合矩阵，须本车验证。实际A2/B3、轮轴向、TCP、曝光时效、2mm/1°及三件落点均未闭环；不以直接改true替代证据。规则R/E/Q及待答疑项如上保持。

### 首轮现场采图与像素诊断更新

用户确认首按三槽215装好/槽盘正常，二按已到DM180°观察姿态且无碰擦；证据after-loading-console.log与stage3-live最新快照。主代理随后独立capture成功，1920×1080原图1794776769343850042-raw.png，SHA256 `8bd6fc08d1655e9391dc5ab38e7ac8dc2569751e7bc31a21b9de7ec56fcabd37`；原料PID2242不在、pgrep source/stage3为空，但退出原因未查，不能断言由助手kill或现场重启。仅读下载至`evidence/integration/2026-11-16/stage3/field/initial.png`，SHA一致。

原检测器三圆心像素(502.47,491.60)/(915.84,500.18)/(1331.41,509.84)，单帧1/3稳定候选；中环外椭圆271.16×271.34px、外圈support0.55。仅在本地证据配置副本`field/search-only-config.json`运行record-calibration点击918/502，输出双线候选：上(182,279)→(1648,305)、下(182,691)→(1648,728)，夹角均值1.233°。diag图`field/pixel-diagnostic.png`。上方黑线与白纸上边贴近、下白纸边低对比，候选不自动等于已确认的真实矩形长边；需视检边线定义及现场确认，不改变算法/滤波。当前参考/TCP未知、不把当前中心自动当零点，未导入A2/B3、未确认毫米有效，Pi与发布包配置保持false。

### 现场8帧像素观察（无串口）

经主代理明确授权现场视觉采集，仅上传独立`config/field-search-draft.json`（SHA05092ba8，calibration_verified=false），未覆盖Pi原stage3.json；运行`preview --config config/field-search-draft.json --headless --frames 8 --output records/field-pixel-eight`。真实相机8新帧完成，进程退出0；出现GStreamer打不开pipeline警告后OpenCV回退采集成功，保留警告不修改驱动。

8帧均检出中环及上下候选线，帧3/4/5的result_stable=true，其余false，不能写全程稳定；中环x915.8865～915.9934、y499.8480～499.9825，线角1.1862～1.2461°。1→3圆心连线1.2609°，同幅图边线均值1.2330°，方向相符仅为像素证据。日志实际detail为前两帧等待1/3、2/3，中间三帧“中环与基准线偏差超出验收精度”，末三帧“等待稳定观测（3/3）”；未设置TCP参考，不能把默认参考偏差当真实纠偏残差。全部metric_verified=false、wire=null，无串口观测或运动。原日志`field/field-pixel-eight.jsonl`已只读下载并校SHA4a407e3a584f2aa2719ffbd037f706a915ba6b31bed2ec9d07e23fe1dcb59d19，摘要同目录。8帧结束释放相机；真实长边/TCP/尺度、方向响应及Z落地行程仍待确认，不继续放料。

### 当前地面高度与深行程接续（用户现场数据）

用户现场测得当前顶部观察姿态下夹爪低端距地面205mm，并明确确认升降机构能够触到地面。这是用户测量及可达性确认，尚非本轮软件控制下降/放料实测。按既有约400p/mm换算约82000p只作为第三段独立深行程适配的候选范围，不是立即下降指令或最终落地目标；TCP、物料高度、末端间隙、实际反馈与落点仍需独立标定。

主代理已将第三段独立深行程MATLAB前置/实现交给finish_live_tools，方案高档审查交给final_stage3_review；原料区29700界限保持。当前板仍815bed44，state2像素观察等待，Pi无有效bridge、标定false；不得把用户“可以碰地”扩写为本轮已执行82000p或放料成功。8新帧中仅3/4/5稳定的范围保持。下一步先完成该范围适配的MATLAB、实现、复审与相称验证，再按单件中心方案推进，不能直接下降。

### 三个独立命令观测响应候选（未认证标定）

主代理分别完成right5、forward5、turn+5°的单次调试命令，板端完成/state2/fault0证据为stage3-live-20261116-051513、051858、052004。right5用户已确认无碰擦/稳定，后两次现场确认截至此记录待答。三命令量均不是独立尺量的真实位移/角度。每次动作后独立capture原图已只读下载并核原SHA，依次field/right5.png、forward5.png、turn5.png，比较基准严格为initial→right5→forward5→turn5。

原检测器中环/线角变化分别为右命令5mm：(+10.76050,+5.63116)px/+0.78905°；前命令5mm：(+13.70813,+0.19238)px/+0.00601°；逆时针命令5°：(+53.27826,+10.77411)px/+4.09917°。上/下线及三环像素诊断图JSON在field/对应名-diagnostic.*，未改阈值/算法。

initial名义A2为[[0.3617383,0.0077372],[-0.0077083,0.3502039]]，椭圆秩残差0.06338；这是95/150名义几何局部初估，非尺量毫米认证。得到每command响应R=[[0.787211,0.992049,3.871230],[0.377822,-0.007659,0.672490],[0.152782,0.001164,0.793894]]，rank3，混合mm/degree原单位cond29.006、各列单位范数归一cond17.653。两平移列夹角26.081°，不写成完全同向，也不归因PID或滑移。B=-invR候选在field/candidate-A2-B3-command-fit.json；正式fit-calibration工具对同3对现场图运行成功，field/fit-command-candidate/measured.json的R与上述重算逐元素一致。

只有3个训练对，可恰好零训练残差，没有独立验证；每轴单样本不能识别噪声/重复性不确定度，条件数也不能认证精度。后续需独立验证样本，当前calibration_verified=false、无valid bridge、无本代理运动。图像中心960/540仅待真实TCP落点验证的候选，不作为已确认参考零点。

### 独立right3 holdout结果（冻结模型）

主代理一次right3命令后state2/fault0，原图1794777778675706960-raw.png已只读下载为field/right3-validation.png，SHA7235fbd6d55f06d9c7ee229e82c4f3b6ac102189cb65627b50eef13cbab49513。以turn5为before，原算法检测成功，中环变化(+5.06519,+6.38470)px、线角+0.33619°；冻结R预测(+6.45630,+3.37870)px，像素残差(-1.39111,+3.00601)。

同一名义A2下预测残差平移norm1.16673mm/yaw−0.13267°；用冻结逆矩阵反解的command为(7.73236,−0.03320,−1.07779)，相比输入command(3,0,0)，command反解平面误差norm4.73248mm/yaw−1.07779°。这些分别是名义平面预测误差和命令反解差异，都不是尺量得到的物理落点误差；不能用前者小于2mm就认证毫米/TCP，也不能把后者直接当用户物理2mm/1°阈值。结果表明该一次holdout与既有局部响应拟合存在不一致，当前不足以确认B3可靠性，不新增自动停车门槛、不归因PID、不修改算法/阈值或为过关重拟合。

field/pairs-command-with-holdout.json将此对标记validation_only；正式工具输出field/fit-with-independent-right3/measured.json的R/B与原3训练对结果逐元素完全一致，独立验证1对、标定false。更多方向/重复性及物理尺量、TCP仍开放，无有效bridge或本代理运动。
