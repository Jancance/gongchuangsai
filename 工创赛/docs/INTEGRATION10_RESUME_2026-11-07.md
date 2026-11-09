# 整合10接续：新4310补丁版与单件一次运行准备（2026-11-07）


## 整合11：同固件重复验证在回零后退离失败，已托料原地松爪（2026-11-07，当前）
此前0b2ddaec版本一件抓取/回顶退离/槽1松爪已获用户实际放稳确认，该证据保留；本次不将单轮成功扩大为重复性通过。用户要求继续并告知下一步，明确确认原料已摆好、1号槽已清空、开始一轮。执行前只读核Flash0b2ddaec匹配、原top0/safe36006、参考有效、VM1/PE2高、无故障；本轮未复位、未重刷、未重建伸缩或原点。
单轮串行执行：空爪转回180.0356°成功，实际下降完成，输出140°1537us闭爪；快速回到顶前约2mm与60RPM末段回零均执行。回零后自动退离phase9出现cause8目标不符；真实原始数据backoff_active1、goal36000、driver_target_raw−1、位置约0，FD02/transport1（驱动接收正确应答）。原0.4s有界目标等待已运行，未更新到预期目标，不能再简单归因查询过早。没有转1号槽或自动松爪，父脚本因失败退出；没有循环试验。
本次不是0x0F堵转：故障时缓存flags3，随后STOP确认；已有只读入口新鲜查询全部result1，3A03/3B03/速度0/实时位置−3、O_SL_RP0(BaseOnMa)。用户现场明确“已经回顶了”，与轨迹一致。首次简报“返回准备失败”不够准确，已更正为回零完成后自动退离失败，未把“顶部到位”当机械顶压力测量或已退离。
因可能持料，保留VM和140闭爪、不直接断电/复位/烧录。用户明确确认“物料已托稳，可以原地松爪”后，在STOP无pending、故障快照保留和Flash核对下，只将TIM2_CCR1从1537缓动至2056，0步进/旋转命令，无供电或状态/原点/故障写入。当前夹爪名义210°张开、VM1/PE2高、旋转约180°保持、升降在原顶部附近、cause8锁存、STOP已确认、参考/自动退离有效位失效，1号槽本次未放入新物料。已通知取下托稳物料，实际取下尚未获确认；不能据PWM输出直接认定已空爪。不要复位/烧录或擅自再次发车。
下一步：先让用户取下已托稳物料，再针对同会话第二次回零到退离的衔接做定位，重点核FD接收与目标应用的差异，保留既有三项速度、108400原坐标目标与3200退离量；不得通过伪造目标/放宽到任意目标/无条件清错绕过。无需重做高度标定或下一次全流程试撞。重复性未通过、驱动内部未更新目标的根因尚未知。
证据集中evidence/integration/2026-11-07/single-cycle/integration11-auto-repeat-{pickup-01-result,pickup-01-first-fault,after-stop-driver-read,supported-open,outcome}.json及pickup01-samples.jsonl；回180日志在evidence/integration/2026-11-06/integration10-complete/integration11-auto-repeat-source180-01-*。本轮仅上位工具执行与本地记录，固件未改，未提交上传。已核R RUN-02/PWR-01，E RUN-06/CAL-05/SRC-04；官方Q保持待技术答疑确认。前三件/视觉/整场和正常整车移植仍未验收。


## 整合11：显式退出回零后低速退离，单件抓放实物完成（2026-11-07，最新）
本条覆盖此前“当前VM关闭/仍到顶/重跑失败”的现场状态，不删除失败证据。用户先确认独立3200p@30RPM向下退离后有小间隙无异常；最终又确认本次抓牢、已退离且新低2mm位置能越盘沿，松爪后1号槽独立放稳、夹指分离、顶部间隙正常且无异常声振。
当前板端：Vehicle_LiftRuler，工程tmp/integration11_auto_release_20261107/project/gongxun-f407.uvprojx；SHA256=0b2ddaec1216fe0572158e805240761fbc171426530b64cf132bb482a8065d1e。独占Keil下载Verify OK/Application running、实际执行前逐段Flash核对匹配；最终VM1/PE2高、夹爪210°/2056us、载盘0°/500us、DM模式2在35.7579956°保持、lift4/cause0/operation3、STOP无挂起、顶部参考与退离有效，原回零raw0、实停safe raw36006。不擅自切VM/复位/再运行。
最终逻辑：原快升到顶前3200p→60RPM末段真实回零保持；随后明确发送9C48并检查9C02应答，隔服务周期复读3B确认退出，再30RPM/60RPMs向下退离3200p。真实位置、零速、到位、保护、编码器一致后发布参考与单独safe位置；原home/top坐标不平移。下次原108400目标保持，因此从safe位置实际少下3200、名义剩105200p，不重新把108400作为从safe处相对下降量。上位it_at_top改按已实读safe位置且backoff_valid判断，只用于高位准入，不篡改原回零原点。
保留独立请求4，仅允许空闲初始化状态显式向下3200p，不生成有效回零参考；普通START/RETURN不串入该operation4。原驱动保护/STOP/位置/速度/编码器/错误锁存保留，无清保护命令。保存原始FD应答、底层首次失败码，目标匹配为backoff专用≤400ms有界等待，超时故障。
证据区分：独立退离1a8c…成功（0→36007、1.5秒后flags3/速度0，用户确认退离）；初始自动退离f4f…与加等待f5dc…均cause8目标不符，原始FD02表明收到命令但目标未更新，VM关闭，不能将其断言为通信/锁在回零。加入明确9C退出后最终0b2d版首次空爪回零+自动退离成功，随后一次下降抓取→原两段式返回+自动退离→35.758槽1→210松爪成功。只有这条退出衔接变更后的本次闭环通过，不保证全部驱动内部根因/长期重复性。f5dc下载包装曾日志参数下标错，未有完整Keil日志可作证；该轮实际Flash逐段匹配f5dc、动作/失败另有记录，不补造Verify。最终0b2d独占flash日志完整。
验证：MATLAB独立退离34、坐标不变33、有界目标等待72检查通过；实际C自动流程及退出拒绝/失联/持续运行、取消、保护、目标未更新/延迟、编码器等14338断言0失败（包含等待循环，非14338种独立场景），独立退离回归166检查通过；在线完整Build0错0警告Code113804，Offline全构建未做，不作为整车正式验收。保存最终AXF和4个C/H/inc变更清单/哈希，未改外部正式Keil或团队STM32副本，保留混合编码和CRLF。
现场主线已完成本次单件：用户已摆好单件/清槽授权下108400原目标抓取、回顶退离、转1号槽、直接松爪，不下放、不自动循环，不计真实库存事务。已核R RUN-02/PWR-01，E RUN-06/CAL-05；官方Q仍待技术答疑确认。未闭环：授权下后续重复性、普通整车移植/回归、三件/动态视觉/全场验收，新持料净空仅本轮用户目视接受。
证据：evidence/integration/2026-11-07/single-cycle/integration11-auto-release-physical-acceptance.json、integration11-auto-release-home-{01,02}-first-fault.json、integration11-auto-release-pickup-01-result.json/-samples.jsonl、integration11-auto-release-{exit-flash,exit-build,host-checks,source-audit}、integration11-auto-release-final.axf；原生home03/extension/carrier0/slot1/release日志在evidence/integration/2026-11-06/integration10-complete/integration11-auto-release-*。只读/动作/用户观察分别保存。
当前仅本地开发并已烧录实测，未提交或上传GitHub。后续仍先检查状态和团队进度，不把当前运行参考跨断电/复位继承，不再让用户重复确认已有108400/两段返回要求。


## 整合11：第一件实际放稳，重跑在下降前真实保护；用户确认仍到顶（2026-11-07，最新）
第一轮已完成：1651a68a固件下108400下降、140°闭爪、参考速度回到顶前3200p再60RPM回零；用户明确确认已夹住且可越盘沿。实际转1号槽35.7579956°、210°PWM2056松爪，用户明确确认物料独立放稳、夹指分离。该单件范围实物闭环，不扩大为视觉/三件/全场/库存事务验收。证据integration11-ready-pickup-01-result.json及原生action3/action6的integration11-ready-slot1/release-01日志、integration11-cycle1-physical-acceptance.json。
此前c3版实车phase28误将flags0x02（到位、失能、无保护）当fault，已按用户全程自锁/制动确认改为允许flags2/3，真实保护4/8位仍拒绝，正常移动前既有使能步骤保留。MATLAB16种flags、实际C控制器4368检查0失败、在线Build0错0警告。当前板端SHA1651a68a84782aeab63c325114aac8817db3cf8e97b3d7ab5c2eda237b3f9343；工程tmp/integration11_two_stage_ready_20261107/project/gongxun-f407.uvprojx，目标Vehicle_LiftRuler。下载曾因未等Keil结束就attach发生RDDI错误，之后仅终止自建卡住下载进程，未关闭其他实车Keil。最终独占、完整USB权限下载日志Erase/Programming/Verify OK/Application running，后续逐段Flash匹配成功。不得把早期Load/Erase或failed日志当成功。
用户问本次高度来源：沿用2026-11-06新标定与高速修正，空爪底面相对原料盘面约50mm，非下降距离50mm；原110800高速落点实量48.5mm，按1600p/mm少下2400改108400，再由用户功能接受约50mm，非新精确尺量。11-07的53.5mm是顶部夹爪底面到1号槽承料面，非本次原料夹取深度。
用户随后明确选择重跑一次，确认原料摆好、1号槽已清空，沿用108400。先空爪回180.0138°成功。repeat脚本误把初次operation1当唯一准入，没接受已回顶operation3；在未下降前预检查失败，异常出口又撤销空爪VM，导致DM保持会话因供电退出fault3。此为助手脚本问题，不解释为机械故障。已修正op准入1或3、DM空闲或成功保持均可（匹配固件ruler_ready），并将空爪失败断供限定为确已请求下降后；没有真实动作的预检查失败不再切VM。保留该错误证据，不删除原记录。
因前述会话退出，已在实际空爪/VM0条件下正式SYSRESETREQ一次、重建真实回零/伸缩/载盘参考，未伪写故障或有效原点。接续同一重跑的integration11-repeat-pickup-02：新鲜DM读取成功，170°准备后请求108400，phase3读取flags0x0F、speed0、moves_sent0，cause9，进入真实驱动保护。未发送位置命令、未闭爪/抓取/回升/转槽/松爪；父脚本失败退出，未执行后续slot/release。该次为实际堵转/保护，区别此前flags2误停，不能称保护根因已彻底解决或重复性通过。
用户现场问为何堵转，并明确“现在已经是到顶的”。接受当前位置到顶，不扩大为用户确认紧抵止挡/具体顶压力或退离毫米数。驱动错误已保存，本轮STOP pending1/confirmed0/unconfirmed1；最终及独立只读VM0/PE2低，未继承旧正常VM1，不把请求STOP当STOP成功；未收到本轮灯灭/声振观察，不补造。没有新的回零或下降。
目前证据表明真实保护发生于本次下降准备前、上次回零后的顶部阶段；不能归因本次108400下探过深或高速回升直接撞顶。用户要求的快速预升到顶前2mm→60RPM末段回零一直保留，末段仍会到顶；O_SL_RP0为BaseOnMa按电流回退，不证明已获得固定机械间隙。顶压保持或驱动使能/回退衔接仅为待验证原因，不写成已确认根因。下一步先针对该末段卸力/退出/真实退离机制验证，不循环清错回零、不改高度掩盖问题。用户新报到顶时未再发动作。
本轮最新原始证据integration11-repeat-pickup-02-result.json/-samples.jsonl、integration11-repeat-pickup-02-first-fault.json、integration11-repeat-current-user-top-confirmed.json；重复准备动作在evidence/integration/2026-11-06/integration10-complete/integration11-repeat-*；受控恢复integration11-repeat-recovery-01-*。软件和外部副本保持，未上传GitHub。R RUN-02/PWR-01，E RUN-06/CAL-05已核，官方Q待技术答疑确认。第一次单件成功与第二次重跑失败分别登记。


## 整合11：恢复原两段式返回、补齐退出确认与首故障证据（2026-11-07，最新）
用户纠正方向并要求继续：保留快升到顶前3200虚拟脉冲（本轮1600p/mm约2mm）、再60RPM末段真实回零，不再改为PE8开关回零，不把它误解成高速撞顶。现场全程机械自锁/制动确认仍有效。
同步：根工作区无提交/remote且全部未跟踪成果保留；隔离团队镜像fetch成功，FETCH_HEAD9f4ed20，未强拉覆盖或提交上传。开始只读板端逐段匹配另一AI新5734ec103f5d35c23903204b839e3eae795ddb91284bc6405b31b023e989417a：it9/cause3，lift6/cause4/phase27、moves_sent1，STOP已确认无pending、VM0/PE2低。此前502/7968部署描述已被外部后续换版取代；不能继续称板上502。moves_sent1仅表示提交尝试，不证明驱动已收到或机械已移动。
核厂商本地ZDT_X42S V1.0.5手册：第62页确认9C48为强制中断退出回零；第21/91页O_SL_RP为驱动碰撞完成后自身回退参数；0为BaseOnMa按电流检测回退。因此“退出命令可能错/驱动必然锁在回零态”的说法未经证明。查较新记录35不装原点开关，PE8历史规划不能当实装。
新增只读诊断中间版82ace452…，运动源码原样保留不运行；六条查询3A/3B/35/36/1A/3F均由原唯一CAN任务执行。短时VM窗口真实回复全部result1：电机flags3、home_flags3（回零进行位清）、速度0/位置0、options0x84、O_SL_RP0=BaseOnMa。没有使能/回零/位置/清保护/写参命令，结束VM0/PE2低。只读参数不能证明持料、自锁或保护根因修复；本次无需为纯查询另造运动仿真。
另一AI退离版还有应用错误：abort返回丢弃、case28等待break会phase++至未定义29、退离后平移top再沿用108400使抓取目标额外下移3200p。已在隔离tmp/integration11_two_stage_exit_20261107/project修正：保留原快升/60RPM回零；删除额外退离位置帧及相关准入；回零后读新鲜3B，若仍运行才发9C且检查ACK并复读；读正常3A、三次零速和稳定位置后才发布top_valid/home_reported，不平移回零基准。保留真保护/STOP/超时，不改108400/140闭爪/35.758槽角，不写驱动O_SL_RP。适配层保存首个底层result、命令、phase和原始reply，避免把白名单拒绝/发送失败/超时/E2统称驱动不退出。保留六项只读入口。源按ASCII片段字节修改，非目标编码/换行保留。
验证：本机MATLAB91项顺序/坐标不变检查通过，实际C退出链7类故障及10/100两缩放预升目标检查4357断言0失败（包括速度持续非零直到超时，非4357种独立场景）；Vehicle_LiftRuler在线构建0错0警告Code113184。只改三份C/H owning文件，审查见source-audit。本机外部实车工程/团队STM32副本未改。
当前最终板端固件C3D24F1B6B0E7796923667255F6E2AA171108288EDC91B7A3BFEED6972A857AE，目标Vehicle_LiftRuler，工程tmp/integration11_two_stage_exit_20261107/project/gongxun-f407.uvprojx。Keil下载Verify OK/Application running后再次逐段Flash只读核对成功；最终lift0/request0、参考无效待真实建立、VM0/PE2低。没有在此修正版上再回零/抓取/旋转，未把离线验证当保护根因或整件实测通过。
下一步仅安排必要的单次现场闭环验证原两段式返回及退出/下一下降衔接，遇故障先保存新首个result/raw，不反复清故障撞顶；还需新鲜参考、实际抓牢/带料跨沿净空与1号槽松爪独立落稳，本轮主线仍未完成。用户已授权继续原单件范围，不重复询问108400深度。
R核RUN-02/PWR-01，E核RUN-06/CAL-05；官方Q保持待技术答疑确认。证据evidence/integration/2026-11-07/single-cycle/integration11-two-stage-*、integration11-home-readonly-result.json、integration11-home-manual-pages.txt。仅本地代码与已下载板端版本，未提交上传GitHub。


## 整合11：补齐实际失能链路后单次下降仍保护（2026-11-07，最新）
用户带来另一AI已下载7968fbf5b5adb5131aeb581e572e542518a895b0363b621e4596bfe39bd7b8be版并要求继续单件108400抓取→回顶→1号槽35.758→直接松爪。逐段核板端匹配7968，当前参考有效、VM1/PE2高，夹爪1537、载盘500；不继承旧7c符号。
审查发现原case26新增disable帧被LiftRuler_CommandAllowed及VehicleArm_Exchange白名单拒绝，又忽略exchange返回；缓存flags3不是失能证明。用户本次明确确认升降有全程机械自锁或制动，不能将此前临时支承自动推广；当前新确认才作为载物失能方案依据。已核R RUN-02/PWR-01，E RUN-06/CAL-05/SRC-04，Q仍待技术答疑确认。
为服务原主线，先空爪撤销升降/startup实际VM、读回VM0/PE2低，再在tmp/integration11_disable_ack_20261107/project隔离修正两处C文件：失能帧仅限真实回零phase26通过；检查ACK，phase27读取新鲜3A确认无保护且使能位清除，phase28读零速，phase29核位置漂移≤0.3°后才发布top_valid/home_reported/DONE。保留真实故障/STOP出口，不改108400、速度、夹持角或槽角。原源存在混合编码，UTF-8/GBK全文件解码均失败；最终仅对ASCII代码段做字节级修改，保留其余字节/CRLF。未改旧候选、团队副本或外部实车Keil入口。
本机MATLAB350项模型检查通过；实际C控制器7类ACK/通信/使能未退/保护/速度/漂移等故障检查74项0失败；在线Vehicle_LiftRuler构建0错误0警告，Code112856。新固件SHA256=502ef3d0ec9230c24a098a50b077f6c6dc05fa652fe5213757ebd8973b468873，Keil下载Verify OK、Application running；后续每次请求前按新AXF逐段核Flash。当前板上是本502版，覆盖前文7968/7c部署说明。
新回零请求1约1.67s完成、phase29路径真实经过失能状态/零速/稳定位置确认后top_valid1；伸缩请求2约1.36s完成8/1、有效原点/保持；载盘请求8约1.58s完成CCR4=500。一次integration11-fixed-pickup-01先读真实DM、170°PWM1759后提交108400下降，仍在phase3新读保护并cause9，moves_sent0；未发位移、未140闭爪、未抓料/回顶/转槽/松爪。STOP仍未确认；最终VM0/PE2低，未循环清故障重试。
因此必须撤回“失能后保护根因已解决”的推断。能证明完成回零时失能检查已通过，不能证明之后驱动状态为何改变；外部使能、驱动内部恢复、时间/负载等尚无足够证据，不把它们写成根因。下一步查回零完成至下降准备之间的真实3A状态变化及驱动使能来源，不再先重跑整套回零抓取；实物是否仍抵止挡/屏幕保护/实际热状态尚无本轮新确认。
证据集中evidence/integration/2026-11-07/single-cycle/integration11-disable-ack-*、integration11-fixed-pickup-01-*及integration11-fixed-first-fault.json；三个原生动作日志在evidence/integration/2026-11-06/integration10-complete/integration11-fixed-{home,extension,carrier0}-01-*。隔离团队fetch仍9f4ed20，本地根库无remote/提交，未强拉覆盖、未提交上传；仅本地，抓放主线未完成。


## 整合11单件一次执行：回零后再次保护，尚未抓取（2026-11-07，最新）
用户明确要求直接执行单件抓取→回顶→1号槽→直接松爪，仅一次。执行前核主线：故障恢复和真实参考为必要前置，不新增放料下放或自动循环。当前7c210e82…完整Flash已核；MATLAB本机integration11-resume-matlab.json命令边界/恢复顺序12项通过，不替代实物夹持/净空。原pyOCD复位枚举SW_SYSRESETREQ不存在，首次在复位前退出、写入0；查本机枚举后改SYSRESETREQ。旧schedule-ready SHA与实际新固件不符，原通用脚本在写入前拒绝；新integration11_native_request明确限定当前已核7c210e82…且逐段核Flash，未篡改旧manifest。
随后受控软件复位一次（未下载/换固件），原故障快照已保存；已有请求1真实低速回零state4/cause0/operation1/top_valid1、top raw4/home_seen1/reported1；已有请求2伸缩回零8/1/reported1、reference_valid1/hold1。复位初始化夹爪PWM2056，载盘PWM0（不得称已恢复载盘0度输出）；未旋转载盘。
单次抓取integration11-pickup-01：新鲜DM 179.13949585°/ctrl1/motor0/valid7/range7，断供后模式已恢复1，不能沿用旧ctrl2；速度−0.007326rad/s。输出170°/1759us，提交108400下降；phase3新读flags0x0F、speed0、moves_sent0，cause9再次故障。没有位移命令、140闭爪、抓取、回顶、转1号槽或松爪；原目标尚未完成。提交STOP后仍pending1/confirmed0/unconfirmed1。取消升降和startup实际VM请求及permit；退出早期empty_failure_VM_off字段false为暂态，此后final及独立新只读均确认VM0/PE2低，不写成全程已关或软件STOP成功。
真实回零完成并不代表顶部解除接触；本次再次复现回零后的驱动堵转/保护，受控复位未解决根因。不再循环复位/回零/清保护重试、不伪造cause/state/top_valid，也不能简单归因4310通信。本次未实现回零后退离止挡；下一项针对35回零结束/止挡退离及保护语义修正，先MATLAB适用验证，严重故障出口保留。实际是否再次紧抵止挡、灯灭/声振和热情况无用户新观察，不能编造。恢复物理路径后才能再取料；持料转槽仍须净空证据。
证据：evidence/integration/2026-11-07/single-cycle/integration11-rehome[-02]-result.json、integration11-rehome-02-samples.jsonl、integration11-pickup-01-result.json/-samples.jsonl、integration11-pickup-first-fault.json；伸缩为evidence/integration/2026-11-06/integration10-complete/integration11-extension-01-result.json。新增脚本为本地上位编排，板端及外部Keil工程未修改。
已核R RUN-02/PWR-01、E RUN-06/CAL-05/SRC-04，官方Q仍待技术答疑确认。仅本地，未提交/上传。

用户提供另一AI完整操作记录，并明确4310问题已解决、已转到原料区180°且升降在顶部；要求把先前单件抓取/转运/入1号槽接续，并同意直接一次启动完成。不得继续沿用昨天9133cd38版的已失败状态解释当前运行版，也不重复旋转小步试验或物料下降标定。

## 当前依据

用户附件：`D:/Codex/home/attachments/d7294c6f-03bb-4a43-a9be-ee9024a20a24/已粘贴的文本.txt`。按记录，另一AI修改了CAN1一次性硬件初始化、ABOM、CAN1/CAN2事务门控和DM回复/总/发送等待时间，已做5°试转与1号槽往返，再纠正取料目标到用户规定180°。最新1号槽实物示教值为35.758°，旧35.1079407/35.146不再作为本轮最终交接角。CTRL_MODE2仅RAM写入，断电/复位可能恢复1，不为接续擅自重刷/复位/关VM。

当前工程：`tmp/integration10_can2_schedule_20261106/project/gongxun-f407.uvprojx`，目标`Vehicle_LiftRuler`；本机AXF SHA256及本轮完整板端Flash对照均为`7c210e8222d047fcf3809d0f307e752fec11483be887672536779f78b10edb0e`。这是另一AI已修改的版本，不覆盖其补丁，不用旧9133cd38/60daa57f符号下发。

本轮只读RAM：it_state2/cause0/request0；lift_state4/cause0/operation1/top_valid1/top_raw3、目标配置108400；startup_axes4/cause0、extension_reference_valid1/hold_confirmed1、reference/current0°；受检RTOS/Safety/CAN故障0。PE2高、VM两个请求均1，载盘CCR4=500µs/0°、夹爪CCR1=1537µs/140°。本轮未写内存、未发CAN查询或动作、未下载/复位。

重要：另一AI直接操纵CAN邮箱后的RAM显示角仍35.758°、ctrl_mode仍1，是缓存，不能据此否定用户报告实际已180°/模式2；也不能用该缓存驱动下一次动作。执行时需用真实新鲜状态重读同步，先读再接受移动目标，不写角度/模式/原点健康缓存伪装就绪。

## 单次完整流程和缺失参数

顺序：210°张爪→当前180°实际取料点下降抓取→140°闭爪→抬升到已验证转运高度→伸缩回原点（已在原点则无冗余位移）→4310到实物示教35.758°→下放→松爪→空爪撤离。仅执行一件一次，不自动循环，不计已抓稳/落稳库存，实际结果由现场观察验收。原料盘保持静止条件沿此前本次示教输入；不因此签动态3秒抓离或三件/第三段。

已验源下降配置108400脉冲可以在本轮顶部/机械标定相符时沿用，不按另一AI“深度完全未知”让用户重新从零示教；不能拿该值当所有工位的下降量。当前伸缩实物原点和180°若已经对准物料，就不凭旧默认48mm强制伸出。

当前尚缺**实际放料下降量**，本轮板端`g_it_place_depth_limit_pulses=0`，专题/slot1-teach.json只有角度，无放料高度。旧28000不得代填，用户“直接一步到位”是完整单次执行授权，不是提供未知放料量或允许越限。

已向用户只询问一个必要数据：当前顶部夹爪最低底面与1号槽承料面的竖直高度差（mm），也接受已示教的放料下降脉冲。沿用物料抓后下伸50mm的既有输入，后续结合用户既定松爪余量计算目标；机械调整后的高度差未量，不能套历史119/54.23/69.23生成新动作。收到数值后本机MATLAB做适用全路径/高度/时序检查，再准备现有接口单次编排、验证并执行。当前没有开始抓取，避免持料等待缺失高度。

## 需求与边界

R核RUN-01/02、PWR-01（本次人工调试不当正式自主验收）；E核SRC-04、CAL-05、RUN-06及当前参数/单次流程选择。官方Q保持待技术答疑确认。未闭环：真实放料高度、当前物料对中、新鲜运动状态同步、完整带料入1号槽与撤离、三件及第三段。

证据：`evidence/integration/2026-11-07/single-cycle/preflight-*.json`，附件及`evidence/integration/2026-11-06/can2-teach-slot1/slot1-teach.json`。根工作区无提交/remote，团队隔离fetch仍9f4ed20，未pull覆盖/提交/上传。当前仅本地准备与只读核对；本轮硬件动作0。


## 最新用户输入：53.5mm；识别210°→成功后立即170°抓取准备

用户实测“当前顶部夹爪最低底面，比1号槽承料面高53.5mm”，替代此前未知高度差，不套旧64.77mm。另明确要求记住：以后物料识别用210°开爪，识别成功后立刻减小到170°再进入抓取；本次仅记录，晚点视觉抓取时落实。210°不得继续作为抓取准备开度。170°不是本次新指定的最终闭合角，既有闭合140°暂保留，不能擅自把170°解释为已夹紧。新profile已分开识别开度210、抓取准备170与闭合140；尚未写入执行固件/改变PWM/发动作。

新高度的必要核对：若保留旧物料下伸50mm和载盘壁高15mm，则顶部持料底面仅比承料面高53.5-50=3.5mm，低于盘沿11.5mm；按此前越盘沿5mm目标还差16.5mm。新实测53.5成立，但这项推算是否适用取决于本次夹取后实际下伸和转运路径，不能覆盖用户此前“已调整”的现场事实，也不能把旧5.23差额直接套到现在。当前尚不能生成放料下降量或启动跨沿转运，应先确认本轮抓后物料下伸是否仍50mm，以及实际槽口路径/盘沿是否仍对应旧15mm。没有通过增大顶部行程或伪造标定绕开。

R RUN-02未改；E CAL-05/SRC-04/VIS-15及本次开度/几何输入已核，Q不变。本次跨对话偏好按用户“先记一下”另保存记忆补充；项目记录与原始输入见single-cycle-profile-pending.json、user-gripper-height-update.json。当前仅本地记录，无硬件动作、未上传。


## 最新用户方案及实际单次抓取尝试：35驱动保护阻止位移，VM关闭

用户不确定下伸量，明确改为“抓完回顶部→转到三槽盘→直接松爪”，取消载盘上方下放。这是新工程选择，不再把缺放料下降量作为唯一阻塞。但料底越盘沿仍未知，不能据“直接松”推定跨沿已通过；先抓回顶后仅需一次目视确认实际持料净空，再转松，不反复量毫米。

本机MATLAB对现有命令边界及170/140PWM映射作6项核对通过，未模拟或验收物理夹持/载盘净空。随后新增一次编排脚本single_pickup_to_top_20261107.py，使用板上既有升降START/RETURN/STOP接口、实际单次准入许可，不改固件/健康/原点/完成值；无横向命令。

一次执行pickup-to-top-01：板端Flash7c210e82…匹配。先请求新鲜DM身份/量程/状态，真反馈180.0356°、ctrl2/motor0、有效7/7，消除了原RAM35.758/ctrl1缓存与实际不符的问题；本轮4310未再报通信失败。夹爪按用户新要求170°/1759µs已输出，随后仅请求原108400下降。控制器在phase3读取驱动状态后报cause9，进入故障，尚未发送位移（moves_sent0）。未执行140闭爪、抓取、回升、横移或松爪，不得声称物料已抓起。

首故障控制器flags0x0f、options0x84、scale100、speed_raw0（按照源结构首段纯uint32布局flags偏移64读取），表明不仅堵转位0x04，还有保护位0x08；不能按此前“仅正常顶部保持flags7”解释或无条件清除。已提交STOP，随后状态5/cause9、stop_pending1/stop_unconfirmed1，软件STOP未确认；不把请求STOP等于已停。考虑空爪/未发送位移，已撤销升降controller实际VM请求和startup_axes实际VM请求、取消本次lift permit；未写驱动健康/原点/故障字段。VM0/PE2低读回确认。驱动断供后STOP确认仍不能补造，原错误记录保留。

当前夹爪170°、空爪未取料、机械升降没有本次位移命令，顶部软件参考因失败撤销。当前VM关闭，模式2断供后不保证保留；后续恢复须实际读取，不能复用本轮旧反馈。已请用户只观察VM灯/声振是否停止以及是否紧抵顶部止挡，不上电或手扳；正等现场反馈。真正阻塞已转为35实际驱动保护，不是缺放料量或本轮4310通信。

需先依据实际保护来源/机械止挡解除条件恢复35；不能强制top_valid、改flags、无条件清锁存或绕过保护来下探。执行恢复若涉及运动方案或对应代码，仍先本机MATLAB。原“正常VM保持”不妨碍本次实际异常退出断供。

证据：pickup-to-top-matlab.json/.log、pickup-to-top-01-samples.jsonl/-result.json、pickup-first-driver-fault.json、pickup-driver-flags-readonly.json。R RUN-02/PWR-01，E RUN-06/CAL-05/SRC-04与本次170抓取准备要求已核，Q不变。外部正式源及当前板固件未修改，未上传GitHub。


用户对VM灯灭/声振停止/升降紧抵顶部止挡回复“对的”，作为本轮现场确认保存。查现有项目历史P2_SOURCE_X_DESCENT_2026-11-01.md，有同类flags0x0F/FE E2堵转保护拒绝STOP的记录；历史按驱动Enter两次解除保护的步骤要求先可靠支承、解除机械顶压，不能在当前仍抵止挡时直接清错。

已请求保持断电空爪，仅在有安全手轮/联轴器操作方式时向下降方向轻退离顶部止挡约1～2mm，不强扳电机轴；若无安全手动方式则不动，等待进一步方案。该手动退离尚未获完成反馈，不能写成已解除顶压/已清保护。未发新的上电或运动命令、未改固件。后续先读真实保护状态并按明确故障恢复流程处理，不复用已失效原点或旧目标直接下降。当前VM0/PE2低、夹爪170°、未抓料，任务主线仍为单件入1号槽。


## 整合11接续：用户已轻退离顶部止挡（2026-11-07）
用户明确确认已轻退离顶部止挡；仅接受该现场事实，不推定退离毫米数或驱动保护已清除。新鲜只读attach：主控RUNNING，完整Flash匹配7c210e82…；VM请求0/PE2低，升降state5/cause9、stop_pending1/stop_unconfirmed1、top_valid0，permit0，安全/RTOS/CAN锁存0。未上电、写RAM、复位、下载或发运动；旧ctrl2且feedback_valid0不能当新驱动反馈。
当前目录无提交及remote，全部现有成果保留。隔离团队镜像fetch成功，FETCH_HEAD为9f4ed20；未pull覆盖本地，未提交上传。
下一步需可靠支承空爪升降滑台，避免恢复驱动供电/解除保护时重力滑落；随后受控恢复VM，仅核节点5实际保护状态，必要时按原厂已核Enter两次流程解除，不关Clog_Pro、不伪造原点，不直接重发108400。新鲜保护清除、停止确认和真实顶部参考仍未闭环，单件入1号槽未完成。
已核R RUN-02/PWR-01，E RUN-06/CAL-05/SRC-04；Q保留待技术答疑确认。只读步骤不产生新运动模型；后续新增运动方案仍须本机MATLAB先验。证据evidence/integration/2026-11-07/single-cycle/integration11-retreated-readonly.json。仅本地。


## 整合11：可靠支承确认后恢复STOP应答（2026-11-07）
用户确认空爪升降滑台已可靠支承。本轮完整Flash核对匹配后，仅恢复controller实际VM请求；没有写升降/旋转请求、故障、参考或位置。控制器已有STOP重试获得真实正应答（源码限定FE/02/6B）：stop_confirmed0→1、stop_pending1→0，moves_sent保持0；stop_unconfirmed1为此前超时历史保留，不覆盖为0。不能把STOP应答等同新鲜3A保护位清零或根因验收。
退出时只撤销升降VM后，0.4s读回VM仍1/PE2高；随后再次撤销升降与startup实际VM请求，读回VM0/PE2低，最终断供已确认。整个过程未重启、下载、位移或闭爪，现场灯灭/声振无新口述，不替代引脚证据。
当前软件state5/cause9/top_valid0仍保留。核现有入口：g_lift_ruler_request仅START1/STOP2/RETURN3；START明确拒绝cause或非IDLE/DONE，Integration10请求1也要求state0。该版本没有故障恢复入口，不能通过写cause/state/top_valid绕过；下一步需要正式恢复/重新初始化方案与新鲜35状态核验，并处理回零后脱离止挡，之后才重新建参考、单件取料。新增运动或代码前先本机MATLAB；无新运动模型或实现通过声明。
R RUN-02/PWR-01、E RUN-06/CAL-05/SRC-04已核，Q不变。证据single-cycle/integration11-power-stop-window.json及integration11-power-window-off.json（均位于evidence/integration/2026-11-07/）。仅本地记录，未上传。
