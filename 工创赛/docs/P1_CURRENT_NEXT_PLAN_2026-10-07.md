# 当前后续计划（2026-10-07）

## 2026-10-24最新：无线串口双向专用测试代码完成，VM强制关闭，未烧录

用户当前明确要求先写串口收发测试，让Codex通过COM接收/下发测试数据。本轮不是恢复先前运动遥测或远程发车。新增APP_UART_LINK_TEST=1，保持既有运动模式参数但CAN路径跳过外部交易、按键不发运动请求；Safety撤销VM且Power_Enable编译期拒绝开PE2，任务真实进展和原看门狗/故障监督保留。固件USART1 PA9/TX、PA10/RX，115200/8N1；1秒LINK心跳，PING/PONG、ECHO、INFO以及仅独立测试数字的SET/GET，数字限0..1000；无运动或任意变量写接口。64字节发送缓存保留，有界输入/待回包和忙时重试，不阻塞任务。

电脑端新增scripts/test_wireless_uart.ps1：先确认专用VM0心跳，再以随机串和测试数字执行7个匹配事务，超时停止、不重发，DTR/RTS关闭，保存JSONL及汇总；ListenOnly只接收，SelfTest不打开COM。19项主机协议/命令构造检查通过。模式1完整隔离构建31608/RO608/RW1724/ZI27516；模式0恢复前进39296/608/1680/27512，均0错误0警告。开始前fetch确认main/origin main0/0，既有成果保留；三源码GBK/CRLF及构建SHA匹配，差异格式检查通过。未烧录、未实测硬件，原工程AXF未重建。

当前本机仅COM3/6/12，COM17不存在；脚本预检opened=false、sent0/matched0，无字节下发。下一步外部VM关闭，在实际工程Rebuild核31608并下载，连续Run，按DAP RX→PA9、TX→PA10及共地接线，插入电脑端DAP后确认实际COM号，再由Codex执行收发脚本。不凭编译或端口发现宣布无线链路通过。结束测试须宏改0并重建下载才能恢复运动；串口测试不签横移/PD/速度问题解决。

指导docs/P1_WIRELESS_UART_LINK_TEST_2026-10-24.md；证据evidence/p1-foundation/2026-10-24/uart_link_test_review.json及两个构建日志，实际端口预检日志在tmp/uart_link_captures。核R RUN-01/PWR-01，E LOC-03/RUN-06，官方Q不变。仅本地完成，未提交推送。


## 2026-10-24最新：15RPM左右横移航向保持代码完成，默认仍前进，未上板

用户确认最近一轮15RPM前进无明显摆动、自动停稳、VM灯自行熄灭。两张汇总图非零提交样本54/56、最大1RPM、最后+1，结束误差约0.643/0.626度，峰值0.736/0.780度，均phase3/trip0/运行8008ms；保留作当前前进开发基线，不宣称精确路径或PD因果改善。用户随后要求先写横移代码并明确每轮基础15RPM。

新增APP_TRANSLATION_HOLD_MODE（0前进/1左移/2右移），当前实际源码默认0以保留原入口；左右横移独立15RPM/8秒、纠偏±2、命令幅值13至17、反馈限30。按既有逆运动学生成横移向量，执行器起始锁存模式、每轮核归属，动态纠偏不能切换方向；方向检查按实际命令符号，纠偏不会重置运行计时或反向保护。原前进函数保留包装，原地转向模式保持可编译。复用原VM准备、源/提交100ms、10度偏角、9秒独立兜底、500ms停帧、稳定核验/故障锁存/看门狗，不接串口、不增加远程发车。

新增纯数学启动自检与g_translation_hold_selftest（0未运行/1通过/2失败），三方向五级修正对照既有逆解并核非法输入；只计算，不发CAN，自检未通过禁止平移。该函数目前仅编译，未运行仿真或上板执行。Watch沿用g_forward_hold并新增motion_mode，纠偏汇总保留。

开始前fetch确认main/origin main0/0，既有未提交成果保留。最终四个完整隔离构建均0错0警告：前进39296/608/1680/27512；左/右各39296/608/1768/27424；原转角39244/608/1680/27512。六个源码与最终构建SHA/配置差异一致，GBK/CRLF保留、源码格式检查通过。证据evidence/p1-foundation/2026-10-24/lateral_hold_build_review.json及四个日志；指导docs/P1_LATERAL_HEADING_HOLD_2026-10-24.md。原工程AXF及板上程序未改，未烧录或实测横移；只有本地代码完成，未提交推送。

下一步由后续实测任务明确左移/右移模式再重建下载，先核健康闲置、motion_mode和自检1，之后才按原完整装机/空间/VM条件安排一次测试；本次先写代码不安排动作。横移实际方向/PD/负载和异常停车、速度高反馈原因及X/Y仍未闭环。核R RUN-01/PWR-01；E LOC-03/05/07、RUN-06、MOT-01/02/04，官方Q不变。


## 2026-10-23最新：新增纠偏汇总，免逐条翻73条过程记录

用户表示过程记录过多难查看；新增g_forward_hold三个只观察字段：correction_nonzero_samples为已接受的非零纠偏提交样本数，correction_peak_abs_rpm为最大修正绝对值，correction_last_nonzero_rpm为最后一次非零修正（带方向）。只在新HWT帧且ChassisAuto_UpdateForwardHold接受后更新，停车/中止不清除，每次开机零初始化且现入口只运行一次。样本数不是CAN实际应用次数；非零只能证明提交过命令，不代表纠偏效果。现有旧运行无法追溯填充新字段，需要下一次使用新固件运行后才有值。

保留15RPM/8秒/反馈限30RPM及PD、VM、故障和停止保护；不接串口、不加远程发车。本轮仅改main.c的诊断字段/统计。开发前fetch确认main/origin main0/0；GBK/CRLF保留、源SHA与隔离构建匹配、源码差异格式检查通过。完整隔离Rebuild Code38572/RO608/RW1672/ZI27512，0错误0警告。证据同日correction_summary_rebuild.log及correction_summary_build_review.json。未烧录/实测、未改原工程AXF；仅本地完成，未提交推送。

下一步仅需原工程Rebuild核日志并下载新诊断版，再按既有健康门禁和一次手动request1运行，结束观察上述三个汇总即可，不逐条翻数组。若nonzero_samples=0则没提交非零修正；>0需结合误差和实车表现判断，不能仅据计数宣称闭环通过。此前用户截图新一轮phase3/trip0、result1/cause0、elapsed8008ms、结束误差约0.494度/峰值0.736度，但全部修正值未知，不将用户单独“1”当纠偏已触发。PD效果、稳定轮速/X/Y仍未闭环。核R RUN-01，E LOC-03、MOT-02/04；官方Q不变。


## 2026-10-23最新：按用户决定撤回串口半成品，恢复已实测手动版本

用户明确暂不接入串口调试、先手动调，随后同意撤回未完成串口代码。本轮仅撤回main.c的FH1遥测块/任务调用、app_config遥测宏及USART1发送缓存扩容；恢复64字节缓存。串口远程发车没有实现，也没有通过串口发车或操作硬件。主工程887个C/H/汇编文件已逐字节核对，与串口开发前forward15_limit30_build_20261023基线完全一致；保留此前所有供电/CAN/PD/诊断/证据修改，不使用git重置。三个新建串口采集/解析/测试工具移至tmp/serial_telemetry_shelved_20261023保留，不放在当前scripts入口。串口草稿、旧失败构建和临时采集保留作历史，不作为当前可用交付。

当前仍基础15RPM、运行8秒、PD修正±2RPM、命令13至17RPM、前进反馈限30RPM；原源/停止/VM/故障/看门狗保护均保留，默认不开VM，手动g_forward_hold.request1一次触发。用户最新确认本轮15RPM无明显摆动、自动停稳且VM灯自行熄灭；图phase3/trip0、elapsed8008ms、final_valid1、结束误差0.532836914度、峰值0.741577148度、新帧观察最大间隔11ms、73条过程记录。仅基本流程及用户观察通过；全程PD是否参与、稳定轮速及此前高反馈原因未闭环，不宣称精确路径或高速恢复性能。证据forward15_limit30_user_motion_result.json及可用原图。

开始前fetch确认main/origin main0/0，原有未提交成果保留。恢复版隔离完整Rebuild Code38512/RO608/RW1672/ZI27496，0错误0警告，源码差异格式检查通过；详见同日manual_restore_rebuild.log、manual_restore_review.json。原工程AXF和板上程序均未改，无需为本次恢复再次烧录。下一步继续Keil Watch手动调试，串口开发暂停；X/Y、PD恢复能力和稳定轮速/速度解析仍待后续。核R RUN-01/PWR-01，E LOC-03、MOT-02/04；官方Q不变。仅本地完成，未提交推送。


## 2026-10-23最新：用户要求将前进反馈超速阈值独立提高到30RPM

15RPM首次测试未正常完成：phase4/trip5、后端result2/cause8、前端elapsed418ms、stop_mask[2]=0、VM请求0；四轮保留反馈21/-24/18/-28RPM，均超过原17RPM阈值且方向符号符合原映射。此次是异常中止，不能记录为正常停车或8秒完成；实际速度高于15RPM命令的原因（瞬态/驱动/单位解析等）尚未定位。用户明确要求提高超速阈值，本轮新增APP_FORWARD_HOLD_FEEDBACK_LIMIT_RPM=30，只在前进保持模式使用；目标15RPM、纠偏±2、命令最大17RPM、8秒、原转向限值及源/停止/VM/故障/看门狗保护均不变。阈值与命令上限分开并增加编译期一致性检查，不将调高阈值描述为解决速度异常。

开发前fetch确认main/origin main0/0，既有修改保留。实际两源码GBK/CRLF保留、与隔离构建SHA一致，差异格式检查通过。隔离完整Rebuild Code38512/RO608/RW1672/ZI27496，0错误0警告；本次未新增运行时或硬件验证、未烧录、未重建原工程AXF。证据evidence/p1-foundation/2026-10-23/forward15_limit30_build_review.json及对应日志。

下一步外部VM关闭退出Debug，在实际工程Rebuild核对新配置后再下载/健康闲置观察；保留本次失败证据，不清故障伪造通过。新阈值下运动停车和反馈速度问题未闭环，X/Y及PD恢复能力仍未闭环。核R RUN-01/PWR-01，E MOT-02/04；官方Q不变。仅本地完成，未提交推送。


## 2026-10-23最新：按用户要求准备15RPM/8秒直接加速测试

用户确认此前5RPM运行约8008ms，向前、无明显摆动、自动停稳且VM灯自行熄灭；Watch phase3/trip0、结束误差约0.417度、峰值0.461度、最大新帧间隔11ms、73条过程记录。用户后续表示过程无±1/±2纠偏，不将这次当PD恢复能力验收。结果存forward5_user_motion_result.json。用户明确不做小角度目标阶跃，改为直接加速，最终指定15RPM；8RPM仅是被替代的中间候选，未上板测试。

当前APP_FORWARD_HOLD_BASE_RPM=15，运行8秒，PD Kp/Kd、±2RPM限幅、滤波/斜率/迟滞不变；各轮逻辑13至17RPM，统一BASE+2命令上限。前进执行器反馈超速检测同步为绝对速度>17RPM，只在forward_hold_mode生效，其他模式原7RPM/静态转向10RPM门限不改。其余VM门禁、100ms源/提交、10度偏角、9秒运行兜底、500ms停帧、停止稳定/故障锁存/看门狗均保留。提高速度并不自动证明PD足以应对更大偏航。

开发前fetch确认main/origin main0/0，保留既有修改。最终隔离完整构建前进38512/RO608/RW1672/ZI27496，原转角回归39080/608/1672/27496，均0错误0警告。8个源与最终前进构建SHA一致、编码/CRLF保留，源码差异检查通过。证据evidence/p1-foundation/2026-10-23/forward15_build_review.json和两个forward15日志。未烧录、未实测15RPM、未重建原AXF，未提交推送。

下一步外部VM关闭退出Debug，实际工程Rebuild核日志后下载观察健康闲置，再在完整装机载物/机构固定及足够行程空间条件下仅一次request1，检查前进、摆动、自动停稳和VM灯及响应记录。15RPM/8秒名义行程约为前轮5RPM的3倍，不以HWT推断精准位移。PD动态恢复、提高速度后的制动/失效、X/Y未闭环。详见当前前进说明；核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-01/02/04，官方Q不变。仅本地完成。


## 2026-10-23最新：用户指定5RPM，前进纠偏改为受限PD，待实测

本轮开始fetch确认main/origin main0/0，既有修改全部保留。用户指出短时慢速固定修正不足以判断动态响应并授权改动，随后明确基础速度5RPM。当前APP_FORWARD_HOLD_RUN_MS=8000；PD Kp0.8/Kd0.25、角速度低通80ms、修正限±2RPM、连续参考限10RPM/s、整数命令迟滞，四轮逻辑3至7RPM，不放宽原>7RPM反馈保护。源码沿用HWT yaw/wz同向，参数仅工程候选，未证明正常或更高速度效果。基础启动沿用原执行器，停车与故障中止不受纠偏斜率拖延。

保留100ms源/新帧续约和提交保护、10度偏角中止、VM门禁/看门狗/故障锁存及500ms停帧兜底；独立运行时限随8秒观察期改为9秒。新增新帧间隔/误差与角速度峰值和最多96条约100ms响应记录，记录为提交值不是实际CAN落地或物理轮速。旧正反30度控制参数不变，不追加旧转角重复试验。

最终隔离完整构建：前进38496/RO608/RW1672/ZI27496，旧转角回归39064/608/1672/27496，均0错0警告。8个本次源SHA与前进构建一致、CRLF及逐文件编码保留，源码差异检查通过；无运行时仿真、烧录或实车操作。实际工程AXF未更新。下一步外部VM关闭退出Debug，在实际工程Rebuild核对日志，再下载验证健康闲置；之后才进行5RPM/8秒受控前进响应观察。动态效果、参数调校、异常停机实测及X/Y未闭环，不承诺更快速度可用或精准距离。

详细配置和Watch见工创赛/docs/P1_FORWARD_HEADING_HOLD_2026-10-23.md；本轮证据evidence/p1-foundation/2026-10-23/forward_pd_build_review.json及两个PD日志。先前同日forward_hold_*是旧版记录。核R RUN-01/PWR-01；E LOC-03/05/07、RUN-06、MOT-01/02/04，官方Q不变。本轮只在本地完成，未提交推送。


## 2026-10-23最新：低速前进与HWT航向保持已本地实现，待板上验证

用户同意转入下一项后，新增APP_FORWARD_HOLD_TEST=1（SESSION仍1，其余测试0）：3RPM前进、锁存HWT目标、±1度死区及±1RPM纠偏，四轮逻辑速度2至4RPM且不反向。四轮首次整组启动完成后5秒请求停车，复用原四轮准备/状态速度反馈/停止核验与VM关断。只用新HWT帧续约，保留100ms源和提交保护、偏航超过10度中止、独立6秒执行器运行兜底及500ms停止帧保护，PC13/request2取消，默认不开VM、不自动运行。旧正反30度功能及用户认可结果保留，FORWARD_HOLD设0恢复原入口；不追加旧转角重复试验。

本轮开始已fetch确认main与origin/main均dc5207f、ahead/behind0/0；保留全部既有未提交源码、证据、Keil设置及构建产物。ARMCC5隔离完整Rebuild：前进37532/RO588/RW1664/ZI25920，原转向回归39064/588/1664/25920，均0错误0警告。8个本次源码SHA256与前进构建一致且CRLF保留；源码差异检查通过。全仓diff检查还包含既有构建产物空白，不将其当本次源码问题或擅自清理。没有运行时仿真、烧录或实车操作，实际工程AXF未重建，禁止把旧AXF当本功能下载。

下一步外部VM关闭、退出Debug，在实际工程Rebuild并核日志，再下载验证健康闲置；板上使用g_forward_hold作为新入口，旧heading请求不再触发当前模式。后续受控前进仅request1一次，核向前/纠偏/自动停稳/VM灯及终态。前进方向和纠偏、异常失效实车验证、X/Y仍未闭环；不承诺精准距离，不把PE2软件命令当实测VM电压。详见工创赛/docs/P1_FORWARD_HEADING_HOLD_2026-10-23.md及evidence/p1-foundation/2026-10-23/forward_hold_build_review.json和两个日志。核R RUN-01/PWR-01；E LOC-03/05/07、RUN-06、MOT-01/02/04，官方Q不变。仅本地完成，未提交或推送GitHub。


## 2026-10-22用户决定：保留当前转角方案，结束本轮重复性测试

用户已确认本轮-30同样反向转动后自动停稳、VM灯自行熄灭；该实物确认补齐先前heading_tolerance1_negative30_result.json的待确认项。随后用户明确“不用测了很可以了这个下一步”，因此不再安排原地转角重复性测试，保持当前1度会话候选及5/3/1RPM/700ms稳定、30度入口和全部保护。接受本阶段正反向单次软件及用户观察证据用于继续开发，不追加宣称统计重复性、独立物理精度或正式整车安全验收。

当前建议的下一开发项为低速平移时HWT航向保持：接现有vx/vy/wz逆运动学，让车前进或横移时保持锁存朝向，利用航向误差生成受限旋转修正，保留反馈和异常停机。现有数学/编码接口已具备，当前实时航向执行器仅有原地转向接口，须设计平移执行和安全状态机，不能直接把现有HeadingCommands当完整平移控制。建议尚未实施，本轮未改固件。

HWT只能提供航向，不能单独给出平移距离或X/Y；本阶段不把定时平移当精确距离，后续实际导航仍需冻结并接入X/Y来源。用户本轮不用编码器估算航向的决定保留，不能偷换为所有定位传感器已不需要。下一步不直接宣称可到指定坐标或跑整场路线。

本次只更新本地交接/当前计划，未编译下载或操作硬件、未提交推送。R RUN-01/PWR-01；E LOC-03/05/07、RUN-06、MOT-01/02/04已核，官方Q不变。


> 2026-10-22电控组1.3复测未通过：用户报VM短亮后关闭、不能判断转动。图phase4/trip6、backend cause1/io8/address1，trace result8/phase3/length5/rx1/约1ms/ESR0；实际代码表明准备控制收到非02应答后失败，尚未到速度阶段。原始request和last_rx折叠，不直接沿用上次E2结论。当前唯一下一项松开外部VM开关、保留失败不复位不重发，展开g_can_ctrl_trace_request与g_can_ctrl_trace_last_rx（含Data）截图；暂不放大角度。证据evidence/p1-foundation/2026-10-22/heading_positive5_failed.json及两PNG。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04；根因、真实转角停车/X/Y及官方Q仍未闭环，未改实际固件。

> 2026-10-21用户表示可满足完整装机载物地面/断电机构收拢固定条件。当前唯一下一项按这些条件摆好，当前37760会话固件无需改码/下载，VM外部开关按下保持，连续Run预热≥3秒且源/闲置门禁满足后仅relative+5/exec_request1一次，不手改VM；观察约18秒，异常外部松开VM，不Stop替代停车、不重复请求，实际停稳/失败后保留Watch1/2与首因。证据heading_after_static_ready_for_positive5.json；试验结果待回传，原E2/转角停车/X/Y及官方Q仍未闭环。

> 2026-10-21恢复会话闲置截图通过：源1/帧龄17与8ms，VM/两fault/phase/trip/request/首次锁存均0，CAN guard0/valid0为VM0正常等待；证据heading_restore_after_F3_idle_pass.json/png。当前唯一下一项复测前确认完整装机载物地面条件及断电机构运输姿态可靠固定、不下滑/伸出/拖地；机构VM前序仍断开，未确认前不开VM/不发heading请求、不擅自恢复机构供电。之后才按一次+5度受控复测取证；原E2/运动停车/X/Y及官方Q未闭环。

> 2026-10-21航向模式恢复Rebuild37760/588/1604/25492、0错0警告/3秒，actual main/can/config SHA与此前已核诊断会话一致。当前唯一下一项VM外部松开，下载复位、F5连续Run约5秒，不按PC13或写请求，截图源valid/两帧龄及VM/fault与phase/trip/request/首次锁存，预期默认健康闲置。机构VM继续按静态隔离状态，本步不动，运动前须核完整装机载物/运输姿态。证据heading_restore_after_static_F3_actual_rebuild_pass.json；原E2/会话转角停车/X/Y及官方Q未闭环。

> 2026-10-21稳定VM下1号失能通过：result1/F3 02 6B，poll flags02/03/03/03，首次锁存0、ESR0、约1ms。证据can_disable_ID1_stable_VM_pass.json/png。当前唯一下一项结束供电VM0/松开开关，退出Debug仅恢复APP_HEADING_SESSION_TEST=1，其余KEY/STATIC/TURN0，完整Rebuild回传，本轮不下载/发请求。实际main/can未变，恢复配置哈希与此前已核会话一致，不重复新增测试。原E2在该静态条件未复现，不能唯一认定原根因；会话+5度/转角停车/X/Y及官方Q未闭环。

> 2026-10-21接轮VM22V/灯持续亮用户确认，结合valid0F/四flags03/guard ready1/两fault0，本次静态供电局部通过。当前唯一下一项按P1_DISABLE_ID1_AFTER_STABLE_VM_2026-10-21.md，仅通用模式稳定VM下发1号失能一次，读取result/原F3应答及flags[0]使能位；仍不发速度/不恢复SESSION，完成后VM0/松开开关。证据vm_four_wheel_22V_steady_user_pass.json；原E2与真实失能/运动/X/Y和官方Q未闭环，源码不改。

> 2026-10-21四轮静态截图：VM请求1、两fault0、Safety/CAN guard ready1、guard cause0、valid0F、四flags03。软件门禁/四轮状态在该时刻通过，证据vm_four_wheel_static_guard_ready.png/json；03不是底盘位姿到位。当前唯一下一项结束本次供电（Run中VM请求0并松开开关），补报刚才VM实测电压及灯是否持续亮，不重发请求/不试转动。持续供电实物结果、原F3 E2、运动/X/Y及官方Q未闭环。

> 2026-10-21用户明确22V测点为VM输出，短时空载开启通过。当前唯一下一项四宏全0固件不改，全断电接回四轮原供电/CAN插头，机构继续隔离，健康默认状态后一次VM请求1做约5秒静态供电/3A轮询观察，记录VM电压/持续灯和CAN guard/poll及故障；不发动作请求/不按PC13，结束请求0/松开VM，异常松开且不重试。证据vm_output22V_point_confirmed_user.json；空载通路局部通过，带轮持续供电/F3 E2/运动/X/Y及官方Q仍待闭环。

> 2026-10-21用户更正：之前静态0V时忘按VM开关，按下重做后灯短亮、测得22V。此前已要求换测VM_EN，当前唯一下一项先确认这22V的红表测点究竟是VM输出还是VM_EN，不猜；开关松开、负载继续断开，不再请求/接回。VM输出22V可支持短时空载开启通过，VM_EN22V则与3.3V允许预期不同须先核点/线路。更正证据vm_static_request1_corrected22V_user.json，先前静态0V错误条件保留历史，不将其当功率级故障证据；R/Q不变，原E2/带载/运动/X/Y待验。

> 2026-10-21静态请求1仍实测VM0V、灯未亮，证据vm_static_request1_measured0_user.json。当前唯一下一项将表移到电源板VM_EN（H2第6脚/R13控制线一端），在全部VM负载断开/复位恢复五项健康默认状态后，一次受监督request1读最初1～2秒，再request0/松开开关；不Stop维持供电、不改3秒CAN保护/代码，不猜脚序。先区分允许信号未到达与下游功率级，尚不能认定Q2坏或E2唯一低压原因；实际VM_EN/VM根因及运动/X/Y/Q未闭环。

> 2026-10-21用户实测request0时VM0V、确认全部VM负载断开。当前唯一下一项在通用模式健康默认状态下，一次受监督短时request1，先摆好GND-VM电压表并按下VM开关，读最初1～2秒后请求0/松开开关；不Stop延长供电/不发动作请求。缺CAN节点约3秒保护及后续IWDG复位可能发生，不放宽或绕过保护、不凭晚读0V判断从未开启，不反复请求。证据vm_static_request0_measured0_user.json和vm_all_loads_disconnected_user.json；开启VM/E2/运动/X/Y与官方Q未闭环。

> 2026-10-21板上通用模式闲置通过：VM0/RTOS0/Safety fault0/ready1/CAN test1，旧heading变量移除符合配置。当前唯一下一项全断电含USB、隔离VM输出所接电机/机构供电插头，先确认接口不重新上电/发请求。后续静态测量需保留缺CAN节点VM请求后3秒保护，不放宽/强写GPIO。证据vm_static_mode_idle_pass.png/json；实际VM/E2/运动/X/Y及官方Q未闭环。

> 2026-10-21通用模式Rebuild38976/588/1496/25368、0错0警告/3秒通过，actual四宏全0、config与候选字节一致、main/can SHA未变。当前唯一下一项VM外部关闭下载复位，连续Run约5秒不写请求/不按PC13，截图VM请求/RTOS故障/Safety故障/Safety ready/CAN test，预期0/0/0/1/1；本轮不开VM，后续再安排负载隔离及静态电压检查。证据vm_static_mode_actual_rebuild_pass.json；VM/E2/运动/X/Y与官方Q未闭环。

> 2026-10-21用户确认试验开关一直按下，不能归因松开档。当前唯一下一项按P1_VM_STATIC_MODE_PREPARATION_2026-10-21.md仅手改实际app_config.h的SESSION1→0（KEY/STATIC/TURN保持0），完整Rebuild后发日志，暂不下载/发请求；原SESSION闲置/失败主动清VM，先临时返回原Safety监督下通用模式，再安排静态VM测量。实际main/can候选ARMCC无诊断、真实源未改，证据vm_static_mode_preparation.json。通用模式仍有手动动作接口，不把它当纯电源固件；不强写PE2、不删除保护。VM_EN/VM、E2根因与运动/X/Y及官方Q未闭环。

> 2026-10-21VM排查更新：用户Q2输入侧22V；断电实测开关按下1-2通、松开2-3通，按现有图确认为按下允许PE2控制、松开强制关闭VM。当前唯一下一项确认此前+5度试验时开关是否持续处于按下档，不重发请求；不能未确认便认定开关为唯一E2根因。证据can_F3_E2_upstream_22V_user.json及vm_switch_pressed12_released23_user.json。实际VM/保护与转角停车/X/Y未闭环，R/官方Q不变。

> 2026-10-21首次诊断实测：1号F3 AB 00 00 6B在约1ms内收到匹配F3 E2 6B，result8/ESR0、PE2前后1，exec phase4/trip6、backend cause1/io8，用户VM灯未亮且车未转。本次非超时；厂家手册43页E2为参数/范围或条件不满足，低压等只是候选。当前停止重复运动，保留失败并关闭外部VM开关，先拍左后ID1电机当前显示/错误、不改菜单清故障，再据证据安排无运动VM诊断；PE2不代替电压实测。不修改等待/保护、不接受E2为成功。证据can_trace_F3_E2_failure.json/png，实际原因/转角停车/X/Y与官方Q未闭环。

> 2026-10-21电控组1.2接续：实际can.c诊断修改已完成，与已验证候选代码一致；本机最新完整构建37760/588/1604/25492、0错0警告/3秒。无需重复手改/编译。当前唯一下一项按P1_CAN_CONTROL_TRACE_CAPTURE_2026-10-21.md下载诊断固件并一次+5度取证，记录首次控制失败原帧/PE2/时长；不得强开VM、清锁存或扩大保护。证据can_control_trace_actual_build_verified.json；仅只读核对已有构建，未操作硬件，CAN/VM根因、转角停车和X/Y及官方Q仍未闭环。

> 2026-10-21+5度再次未通过：用户VM没亮，源有效/15与6ms，两fault0；exec phase4/trip3/state4/cause2，backend result2/cause1/io5/last_address1，旋转mask0。定位为1号失能准备命令在当前窗口未取得匹配回复，不再归因HWT或仅凭灯判电源坏。当前唯一下一项按P1_CAN_FIRST_CONTROL_FAILURE_TRACE_2026-10-21.md仅手改实际can.c：加诊断变量、整段替换ExchangeSingle和ControlSingle，记录首次控制失败的命令/原始RX/阶段时长/PE2 ODR/ESR，失败后冻结不被轮询覆盖；不改帧/等待/供电/100ms保护。GCC模拟和ARMCC完整can候选通过，实际源未改。用户完整Rebuild后发日志，本轮不下载或再试运动。证据heading_after50Hz_can1_prepare_timeout.json、can_control_trace_preparation.json；实际VM/CAN根因与运动/X/Y未验，R/Q不变。

> 2026-10-21恢复会话模式后30秒闲置观察通过：用户明确未看到valid跳0，图valid1、角度/角速度帧年龄17/18ms，VM/RTOS/Safety fault与三种接收错误均0，phase/trip/request0、yaw2.65869141/rate0；CAN余栈325字/1300字节、最小堆11320。仅有限Live Watch观察，不签全部最大间隔/动态资源。下一项原完整装机载物地面条件下一次相对+5度复测，无需改码/下载/重设50Hz；打开VM外部开关，先F5连续Run≥3秒恢复新鲜源，再Run中request1一次，relative5.0，观察约18秒，实际停止/失败后Stop记录Watch1/2及VM/动作/停稳现象。异常外部关VM，不用Stop替代停车/不反复请求。Watch2保留phase/trip/request并加入目标/误差/状态/后端结果字段。证据heading_after50Hz_idle30s_user_pass.json，R/Q与运动/X/Y边界不变。

> 2026-10-21恢复会话模式实际Rebuild通过：37208/588/1540/25476、0错0警告/3秒，实际四宏和main/hwt与已核候选一致，默认heading/rate50请求0。下一项保持VM外部关闭下载复位，连续F5先运行约5秒完成初始化/自动版本读取，再观察30秒，不写任何请求；Watch1保持13项，Watch2仅加exec_phase/exec_trip/exec_request。期望预热后source_valid保持1、两帧龄期<100ms，VM/RTOS/Safety故障及三项exec均0，接收错误不新增。初始化前几秒source_valid0不作为本观察失败。观察后Stop截图并报告是否看到跳0；一次截图不证明全部间隔上界，尚未放行转动。证据heading_mode_after50Hz_actual_rebuild_pass.json；R/Q与实物运动/X/Y边界不变。

> 2026-10-21下一项恢复目标航向会话模式：用户仅在实际USER/app_config.h设置KEY0/STATIC0/TURN0/SESSION1，其余代码及115200/100ms/CAN20/30保持；退出Debug、VM外部关闭后手改并完整Rebuild，本轮不下载/不设置请求。当前main与新增HWT服务按新模式隔离ARMCC编译无诊断，实际文件SHA未变，证据heading_mode_restore_after50Hz_preparation.json。原Watch13项先保留。HWT50Hz已设置/测频并按用户断电步骤读回8，无需重复设置；构建核对后先做VM关闭的新模式源状态观察，再决定+5度复测，不能把平均50Hz当最大间隔/物理运动通过。R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，官方Q/X/Y等边界不变。

> 2026-10-21用户按断电保持步骤后读回RRATE8：result1/raw8/count1、帧55 5F 08 00 06 00 00 00 00 00 C2校验正确，设置服务active/result/state/command/raw/request均0，VM/两fault0、源有效/年龄13/14ms。按用户执行该断电流程接受本次参数保持，不把截图当独立断电测量。证据hwt_rate50_powercycle_readback8.json。用户要求清理Watch，当前先按P1_WATCH_CURRENT_MINIMAL_2026-10-21.md只留13项健康/源/航向/错误/资源，删除所有重复及已完成RRATE/50Hz/版本/帧计数和暂不用会话项；只删Watch行，不删源码/不改值。固件不改/不重设速率。后续恢复SESSION模式先核100ms源连续性再受控+5度，实际运动/停车/X/Y与Q仍待闭环。

> 2026-10-21实际平均接收约50Hz通过：两图同组上方计数1166→2022、tick58950→76080ms，856/17.13≈49.97Hz；同组下方约49.97～50.03Hz，重复Live Watch行不同步不混配。checksum0/overflow0、UART错误1均无增量。仅验证区间平均频率，不签最大间隔/100ms源连续性。下一项保持VM外部关闭，对主控与HWT真实断电（含可能USB回供，不只RST），等约5秒再按原方式上电，沿用当前固件，不发rate50设置；Run≥5秒版本1/state0时仅rrate_request1一次，等约2秒Stop，截图RRATE result/raw/reply_count/原帧和VM/fault，预期raw8，另确认rate50_request/result未自行设置。证据hwt_rate50_measured_frequency.json；断电保持/运动停车/X/Y与官方Q仍待闭环。

> 2026-10-21固定50Hz流程已板上读回8：用户图active0/result1/state0/command3/raw8，帧55 5F 08 00 06 00 00 00 00 00 C2校验正确，VM及两fault0；接受本次设置流程与即时读回，原rrate_raw6为旧快照。两张图angle/rate_count均0x404=1028、update_tick均0xDB56=56150ms，不能据此计算频率。下一项保持VM外部关闭、不再request设置/运动、无需改码复位，F5连续Run中两次相隔约10秒截图angle_count/rate_count/checksum_errors/uart_errors/overflows/update_tick；用实际tick差估频，预期约50Hz但尚未验证。断电保持、最大间隔/100ms源连续性及目标转角停车/X/Y随后另验，官方Q不变。证据hwt_rate50_readback8_user.png/json。

> 2026-10-21固定50Hz服务实际Rebuild通过：40296/600/1524/25452、0错0警告/3秒，实际hwt.c/h与候选token完全相同、RRATE声明已补齐、main及STATIC1/SESSION0/TURN0/KEY1保持。下一项VM外部开关保持关闭，下载复位后连续Run≥5秒，确认版本1/state0/VM与两fault0；Run中先rrate_request1读取，看到result1/raw6后再等2秒，仅此条件下rate50_request1一次，连续Run约5秒后Stop截图rate50 active/result/state/command/raw及展开reply，VM/fault。若原读值非6或读取失败则不触发设置、先保留截图。成功候选active0/result1/raw8，原rrate_raw6仍是旧快照；频率/最大间隔/断电保持另验，不开启运动。证据hwt_rate50_actual_rebuild_pass.json，R/Q与物理闭环/X/Y边界不变。

> 2026-10-21RRATE板上读取通过：result1/raw6/reply_count1，帧55 5F 06 00 06 00 00 00 00 00 C0校验正确、VM及两fault0，和约10Hz实测相符。已取得厂家HWT101专用协议及厂商编写的同名协议PDF解锁/保存说明；只采用69/03/00/27四帧固定序列，不使用76（归零）。下一项按P1_HWT_RATE50_MANUAL_2026-10-21.md，用户仅改hwt101.c/h，补只读/50Hz声明、新静态一次性50Hz服务、两处互斥/IRQ owner3/更新调用，完整Rebuild；STATIC1等配置/main不变，本轮不下载或发请求。新服务默认不自动写，只有已读6/静态VM0/事务互斥等满足后受理，保存并读回8仍须真实频率和断电验证。GCC模拟及ARMCC完整hwt候选无诊断，实际源未改。证据hwt_rrate6_read_user_pass.json、hwt_rate50_preparation.json，R/Q及物理闭环/X/Y边界不变。

> 2026-10-21RRATE只读实际Rebuild通过：39292/580/1476/25444、0错0警告/3秒，实际hwt101.c与已测候选token一致，STATIC1/SESSION0/TURN0/KEY1、main未变。hwt101.h尚缺3行RRATE extern，对本文件内定义及当前Watch读取无影响，记为后续跨模块接入前补齐，不为此重复构建。下一项VM外部开关保持关闭，下载静态固件并复位，F5连续Run约5秒，version_result1/state0及VM/两fault0时，在Run中rrate_request1一次，再连续Run约2秒后Stop，截图RRATE result/raw/reply_count及展开11字节reply，连同version及VM/fault。禁止请求运动/校准/A7或手改结果。raw6与10Hz相符但5F无地址回显，只算候选读取证据，不提前写50Hz。证据hwt_rrate_read_actual_rebuild_pass.json；R/Q及物理闭环/X/Y边界不变。

> 2026-10-21板内RRATE只读下一步已准备：按P1_HWT_RRATE_READ_MANUAL_2026-10-21.md，用户将配置切STATIC1/SESSION0/TURN0/KEY1，只在实际hwt101.h加声明、hwt101.c加RRATE变量并整段替换Hwt_VersionUpdate，再完整Rebuild；main/硬件/CAN不动，本轮不下载。只发27读取03，不解锁/写入/保存；RRATE独立结果、静态VM0、事务互斥/1500ms5F隔离及有限截止，不污染版本/A7。GCC模拟及ARMCC hwt候选/main静态候选无诊断，实际源未改。5F无地址回显，result1仍为候选回复，后续需原帧+实测核对；未写50Hz。证据hwt_rrate_read_preparation.json，R/Q与物理闭环/X/Y边界不变。

> 2026-10-21用户补充HWT101焊在主控板且未引出IO：取消前轮USB-TTL接线/接口照片前置，不拆焊、不飞线、不改成品PCB。当前改走已有USART2 PD5 TX/PD6 RX的STM32配置路径；此前版本5F回复成功证明过该双向通道，不要求用户接第二路TX。下一项准备用户手动加入受限速率读取/设置流程，保持VM外部关闭与静态互斥，先核对RRATE再按匹配协议设置50Hz工程候选并读回/测频，不能把发送成功当参数生效或自动写未知解锁/保存。此轮仅确认路径，尚未新增配置代码、编译或发串口；100ms保护保持，实物配置/运动/X/Y及官方Q未闭环。

> 2026-10-21用户已确认有USB转TTL但未连接：采用上位机配置路径，暂不写STM32设置代码。当前唯一下一项先断整车电源/关VM，取得USB-TTL正反面针序/电平跳帽和HWT101当前端子照片，核实接口和实际TTL电平后再逐线接线，不猜针序、不并联STM32与适配器两个TX、不混供电。50Hz仍是待设置/读回/帧率验证的工程候选，100ms门槛不变，实物配置/运动和官方Q未闭环。

> 2026-10-21计数取证完成：角度/角速度各从593到744增加151，HWT更新59360→74460ms，约10Hz；校验0/溢出0、UART错误1均未增加。结合首次角速度100ms拒绝，确认当前平均周期与100ms源边界无余量。厂家HWT101页面默认10Hz可调参数仅作旁证，不代替当前RRATE读回；50Hz为工程候选。下一项保持VM关闭/不运动，确认用户现有维特上位机连接或仅STM32路径，再选择型号匹配的速率配置方式；不猜解锁/保存、不改100ms安全期限或波特率。详见P1_HWT_RATE10HZ_AND_NEXT_2026-10-21.md；原图/计算hwt_measured_rate10Hz.json。实际配置/时效与物理闭环/X/Y仍待验，R/Q不变。

> 2026-10-21首次源触发证据已取得：用户观察valid在0/1变化；首次fault_phase1、flags0x10、sample_age10ms、angle_age99ms、rate_age100ms、HWT_update_age10ms、tick0x9524=38180ms，backend0，确认等待阶段角速度帧到达100ms拒绝边界，非该时刻HWT任务长时间未更新。其下层原因（设备输出频率/无线延迟/缺帧/串口错误）尚未确认，不直接把它写成10Hz或硬件故障。当前唯一下一项保持VM外部开关关闭、不再发运动请求，连续Run中两次相隔约10秒记录HWT angle_count/rate_count/checksum_errors/uart_errors/overflows/update_tick；期间不Stop/复位/改参数，先估算实际接收频率并看错误增量。不改100ms门槛/固件，不重复运动。证据heading_source_first_rate100_user.png/json；R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，官方Q不变。

> 2026-10-21首次失败快照实际Rebuild通过：36000/564/1472/25456、0错0警告/3秒，实际main与已测诊断候选token一致，CAN20/30候选及模式未变。下一项下载诊断固件并复位，在既定完整装机载物地面条件下只复现一次+5度请求：外部VM关闭时下载，打开开关后连续Run≥3秒，确认phase0/trip0/源有效，在Run中request1一次、relative5.0；观察约18秒、实际停止或失败后Stop，不重复请求/不复位抹掉首次快照，不手改VM。回传exec_trip及fault_phase/source_flags/sample_age/angle_age/rate_age/hwt_update_age/tick，连同实际VM/动作现象。若发生动作异常立即外部关VM，不用Debug暂停替代停车。只为获取首次触发证据，未宣称修复source trip2；R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04与官方Q边界不变。证据heading_first_fault_actual_rebuild_pass.json。

> 2026-10-21期限修正后用户仍报告VM未亮/车未转；最新图改为phase4/trip2、session state0/cause0/backend_session0及后端全0，说明本次在执行器会话建立前源判定失效，不能沿用旧trip3归因。后续source_valid1、年龄38/49ms不代表触发时状态。当前下一项按[首次源失败快照手动指南](P1_HEADING_SOURCE_FIRST_FAULT_MANUAL_2026-10-21.md)，只替换实际main的App_HeadingRuntimeTrip完整函数为含7个诊断变量的新样本，首次记录源条件位/各龄期/阶段/时刻；不改判定或100ms门槛/CAN20/30/VM动作。实际controller/session联动mock及首次原因锁存/恢复不覆盖/WAIT与RUN/配置位用例通过，候选main ARMCC无诊断，真实源未改。用户完整Rebuild，先不下载或重复运动。启动前VM0V为默认预期，触发后是否曾实际供电未测。证据heading_after_canfix_source_trip2.json、heading_first_fault_preparation.json；R/Q不变，根因/物理闭环/X/Y待验。

> 2026-10-21CAN期限位置已修正，实际Rebuild通过：35744/564/1444/25452、0错0警告/3秒，实际can.c与已测候选token完全一致，回环TX100、新SESSION交易TX20/RX30、旧交易100/200、提交监督100ms、main与模式开关均已核对。下一项一次+5度受控复测：VM外部开关关闭时下载并复位新固件，完整装机载物地面条件；打开VM外部开关后连续Run≥3秒，在Run状态只写exec_request1一次，relative5.0，不手改VM，持续约18秒，停稳或已失败后Stop保留现有Watch与实际动作/VM现象。仍有异常立即外部关VM，不重写请求。若失败优先读auto_cause/io_result/last_address与exec_trip，不默认修复成功。证据heading_can_deadline_actual_rebuild_pass.json；R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，实物VM/转角/停车/X/Y及Q仍待闭环。

> 2026-10-21CAN期限手改Rebuild虽0错0警告/3秒，但实际TX宏改在回环函数：can.c第152行CAN1_LoopbackTest需恢复pdMS_TO_TICKS(100U)，第269行CAN1_ExchangeSingle才应使用pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS)。RX宏及其他改动正确。当前唯一下一项用户修正这两处并完整Rebuild，先不下载；修正隔离副本与已测CAN候选token一致、实际配置ARMCC无诊断，真实源未改。证据heading_can_deadline_placement_review.json；实车故障仍未复测，R/Q不变。

> 2026-10-21后端失败截图：会话编号1，state10/result2/cause3/io_result0/start_mask2为0，exec_cause2，确认接受过执行器会话但未记录成功旋转启动。源码发现确定时限冲突：唯一CAN任务产生提交，却可等待TX100ms/RX200ms，超过独立提交100ms监督；原IO可能尚未写回便被监督先作废。该机制与截图一致，尚未唯一锁定实机首次原因。下一项按[CAN期限协调手动指南](P1_HEADING_CAN_DEADLINE_FIX_MANUAL_2026-10-21.md)仅改实际SYSTEM/CAN/can.c：包含app_config、SESSION专用TX20/RX30宏，替换ExchangeSingle两处参数；旧模式100/200、帧/速率/所有100ms安全门禁保持。实际Exchange函数主机新旧模式/回绕/边界及ARMCC完整can模块候选编译通过，实际源未改。用户手改后完整Rebuild，本轮不下载/不复测运动。原实车测试未通过；源/调度、物理VM与目标角/停车/X/Y仍待闭环，R/Q不变。

> 2026-10-21首次+5度未通过：用户报告VM没亮，截图phase4/trip3/state4/request0/rpm0/VM0、两fault0、source_valid1、两帧龄期0x61/62=97/98ms、CAN最小余栈245字=980字节/最小堆11328。实际main仍与已测候选一致。trip3复用于多个平台/互斥检查，截图不能锁定首次失败位置，也不能把最终源龄期当触发时龄期或认定电源损坏。当前唯一下一项保留失败状态，不复位/重写request/强开VM；读取exec_cause/backend_session及chassis_auto_state/result/cause/io_result/start_mask[2]截图，先区分后端执行失败和入口条件，再决定针对修复。证据heading_positive5_vm_dark_trip3.png/json。实车+5度未验收，R/Q不变。

> 2026-10-21新模式默认Watch通过：用户截图phase/request/trip/state/rpm/VM请求/两fault均0、source_valid1、角度/角速度年龄6/7ms、CAN最小余栈0x145=325字=1300字节、最小堆0x2C40=11328字节。只覆盖闲置路径和该时刻，不称物理VM/动态资源或闭环通过。下一项按[首次+5度受控试验](P1_HEADING_FIRST_POSITIVE5_TRIAL_2026-10-21.md)：保持既定完整装机载物地面条件，VM开关可关闭；打开外部VM开关后连续Run≥3秒，在Run状态只写exec_request1一次，relative保持5.0，持续约18秒，记录实际逆时针小幅转动/自动停稳及终态。不要暂停排队运动/转动中Stop/同次重复请求或扩大角度。助手未操作硬件，实车结果仍待回传。证据heading_runtime_idle_user_pass.png/json；R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，官方Q不变。

> 2026-10-21真实会话接入整工程Rebuild通过：35744/564/1444/25452、0错0警告/3秒，实际main与已测候选token一致，SESSION1/STATIC0/TURN0/KEY0及ReadStatus=s_mode已核对。代码较旧静态预览缩小符合模式切换移除未引用代码的范围，不要求重复构建。下一项VM外部开关关闭、车不动，下载本次新模式，连续Run约3秒再Stop；不按PC13、不设置任何请求，记录exec_phase/request/trip/state/rpm、VM请求、两fault、source_valid、两帧龄期及CAN余栈/最小堆。预期exec闲置/无请求/无trip/VM0/fault0，源有效且两帧年龄<100ms。只是新模式默认不启动检查，不放行转动；未测物理VM、连续时效/失效响应、装机载物±5度/停车与X/Y。证据heading_runtime_actual_rebuild_pass.json；R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，官方Q仍待技术答疑确认。

> 2026-10-21会话构建报HEADING_SOURCE_FRESH_MS未定义（2错0警告）：新附件与实际main一致，两处外层条件已修正，但源快照外层if之后直接typedef，遗漏1kHz检查和100ms宏。下一项仅在该typedef之前恢复`#if configTICK_RATE_HZ != 1000U`/`#error Heading_source_requires_1000Hz_tick`/`#endif`/`#define HEADING_SOURCE_FRESH_MS 100U`四行；保持外层if不动，其余源码和配置不动，再完整Rebuild，不下载。恢复后与此前已测main的token完全一致，实际配置ARMCC隔离编译无诊断，实际固件未改；证据heading_runtime_missing_define_review.json。真实构建/实机仍待闭环，R/Q不变。

> 2026-10-21会话接入粘贴审阅：两附件SHA相同，与实际保存main的代码token一致；仅相邻两处外层编译条件与已测候选不同。用户将g_heading_bus变量块扩展到新模式，却把真实源快照块保留为旧条件。当前唯一下一项：main第718行旧邮箱块恢复STATIC条件、第726行源快照块扩展SESSION或STATIC条件，详见接入指南顶部；其余代码/config保持，完整Rebuild后发日志，不下载。隔离修正后与已测候选token完全一致，实际配置ARMCC编译无诊断，真实main哈希未变。证据heading_runtime_paste_review.json。整工程链接/默认VM0/实物闭环/X/Y尚待验，R/Q不变。

> 2026-10-21模式字段修正实际Rebuild通过：38908/576/1456/25432、0错0警告/2秒，实际ReadStatus已取s_mode，不重复该项。下一项按[目标航向真实任务接入指南](P1_HEADING_RUNTIME_BINDING_MANUAL_2026-10-21.md)由用户修改实际main/app_config：独立SESSION模式、源快照跨模式发布、CAN唯一会话/动作及归属检查、旧手动交易隔离、Key只取消、独立Safety监督；默认request0/VM0，首轮目标只允许±5度。隔离新模式和旧静态main均ARMCC无诊断编译，实际controller/session链接mock执行器的联调用例通过，实际源码SHA前后未变。用户手改后完整Rebuild，本轮不下载/不设置请求。软件mock不签RTOS/CAN/物理闭环，实机默认VM0、失效响应、装机载物±5度/停车与X/Y仍待验；R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，官方Q不变。

> 2026-10-21执行器快照接口用户完整Rebuild通过：38908/576/1456/25432、0错0警告/3秒。实际接口与上一版样本一致，但复核发现助手样本将dry_run误读g_chassis_auto_dry_run请求参数；Begin实际锁存并供执行使用的是s_mode。下一项仅由用户在ChassisAuto_ReadStatus中改为`status->dry_run = s_mode;`，保存并完整Rebuild，其他源码/main/静态配置不动、本轮不下载。更正样本已通过请求与锁存值相反的主机用例，实际模块隔离候选ARMCC编译无诊断；旧mock未覆盖该不一致，不作正确性证据。详见执行器状态快照指南和heading_backend_snapshot_mode_correction.json。真实会话/动作/Safety及物理闭环/X/Y未完成，R/Q不变。

> 2026-10-21真实源静态Watch通过：用户截图valid1、sequence0x2B8=696、发布/角度/角速度年龄1/4/5ms、yaw0.0384521484度、rate0、VM请求及两fault0。仅接受本观察时刻的数据交接，不签连续时效/断流或物理VM。下一项按[执行器状态快照手动指南](P1_HEADING_BACKEND_SNAPSHOT_MANUAL_2026-10-21.md)只在实际chassis_auto_test.h/c增加ChassisAutoStatus和ReadStatus，在同一临界区复制会话号/航向模式/演练标志/active/result/旋转stop_mask等，用户完整Rebuild；main与静态配置保持，本轮不下载。GCC字段/空指针/旋转段索引/mock临界区检查通过，不代替板上并发或物理停止。源截图/JSON和backend准备检查已归档；会话/动作/Safety绑定、真实运动停车及X/Y未闭环。R RUN-01/PWR-01，E LOC-03/05、RUN-06、MOT-02/04，官方Q仍待技术答疑确认。

> 2026-10-21真实航向源快照实际Rebuild通过：38908/576/1456/25432、0错0警告/3秒。只读实际外部工程确认快照代码与已测样本去注释/空白后一致，TaskA在HWT更新后唯一发布、CAN任务唯一观察，静态模式1/旧限时转向0。下一项关闭VM外部开关后下载静态固件，保持车不动，连续Run约3秒再Stop，记录source_valid/sequence/三个age/yaw/rate及VM请求、RTOS/Safety fault；不设置启动/转动/校准请求。一次暂停快照只证明该时刻，不证明连续100ms时效或真实会话/运动/停车/X/Y通过。证据heading_source_snapshot_rebuild_pass.json；R RUN-01/PWR-01，E LOC-03/05、RUN-06、MOT-02/04，官方Q保持待技术答疑确认。

> 2026-10-21新聊天接续：只读实际外部工程和Objects构建日志，确认heading_session.c已编译、38380/576/1424/25408、0错0警告，main尚未调用会话。下一项先按[真实航向源快照手动指南](P1_HEADING_SOURCE_SNAPSHOT_MANUAL_2026-10-21.md)在实际main增加完整源快照发布及CAN任务静态观察；用户自行修改、完整Rebuild后回传日志，本轮不下载。新快照分开保存发布/角度帧/角速度帧时间与发布序号，以100ms严格边界拒绝过期，不沿用原静态预览500ms作为行车许可。GCC软件用例及实际插入位置检查通过，实际源码SHA未变；未执行ARMCC/整工程构建或板上验证。源快照尚未接会话/执行器/Safety，真实运动/停车/X/Y未闭环。R RUN-01/PWR-01，E LOC-03/05、RUN-06、MOT-02/04；官方Q保持待技术答疑确认。

> 2026-10-21会话模块实际Rebuild通过：38380/576/1424/25408、0错0警告/3秒，日志含heading_session.c。实际USER/heading_session.h/c去除注释空白后与已测样本一致，uvprojx引用实际USER文件，main尚无HeadingSession调用；程序大小未变符合未引用函数被移除的范围，不据此要求重做构建。本项完成。下一项为会话与真实任务/源快照及执行动作的绑定，需保证输入真实健康/新鲜且序号明确、运行入口互斥、停止动作反馈和独立Safety监督。当前静态配置不改、本轮不下载；真实会话整合/运动/物理停车/X/Y仍待验，R/Q不变。

> 2026-10-21目标航向会话模块已准备：[手动加入会话模块指南](P1_HEADING_SESSION_MODULE_MANUAL_2026-10-21.md)。用户在实际外部工程USER新建heading_session.h/c，复制docs/code_samples对应样本，c加入USER组后完整Rebuild；main/静态配置不改、本轮不下载。模块区分临时SETTLING零速与最终停车，只有停止请求接受且执行器成功/非活动/mask0F才DONE；重复/过期/回退源、9秒未到位、停车2.5秒未确认、请求拒绝/取消及反向等均安全失败锁存。实际controller链接的GCC用例及ARMCC样本编译通过。此为纯会话判定层，尚未绑定真实任务/硬件，首轮候选±5度且源健康/互斥/Safety整合与物理闭环仍未放行，R/Q不变。

> 2026-10-21独立提交超时接口实际Rebuild通过：38380/576/1424/25408、0错0警告/3秒。实际源只读核对提交时间的Init/Begin清零、BeginHeading首次设定、Update合法提交更新及Auto_CheckLocked独立监督；去除注释/空白后与此前通过mock/ARMCC的候选一致。不再重复此接口构建。下一项为真实目标航向受控会话整合：源sequence/HWT健康绑定、互斥调试入口、到位/SETTLING/取消/故障区别、超时协调及停车反馈/VM兜底。当前仍静态模式，不下载未接入的接口、不运动；提交超时的实机效果、真实闭环/物理停车/X/Y待验，R/Q不变。

> 2026-10-21会话整合第一步：[独立指令提交超时手动指南](P1_HEADING_COMMAND_WATCHDOG_MANUAL_2026-10-21.md)已准备。用户只改实际外部工程chassis_auto_test.c：新增合法提交时间，在Init/Begin清零、BeginHeading/Update合法提交时更新，由现有Auto_CheckLocked的Safety路径检查新会话100ms未提交则故障关VM；正常停车改由原截止/反馈接管。mock状态机回归及真实宏头文件ARMCC候选编译通过；用户手改后完整Rebuild，main/h/静态配置保持、本轮不下载。此步不是完整会话；源sequence/HWT绑定、故障日志/整合、超时协调及实物闭环仍待完成，官方Q不变。

> 2026-10-21动态轮速接口用户修正后已完整Rebuild：38316/576/1420/25404、0错0警告/3秒。实际外部工程只读确认direction_age声明、Init单组清零、Begin新会话清零均正确，s_run_tick=now仅在首次START分支内，不再有无条件重复行；h中UpdateHeading声明已存在。此前两份相同粘贴代码中的4处问题已修正，不重复要求该步。下一项为目标航向会话与唯一CAN任务的受控整合，必须包含HWT/源指令新鲜度与独立Safety失效监督、正常到位与临时SETTLING停转的区别、故障/取消锁存与停车反馈/VM兜底；旧20秒控制器超时与执行器10秒上界尚须明确协调，同会话反向仍拒绝。不应仅把新接口串到静态预览便放行运动；当前保持静态模式，无需下载未接入的接口。本次未改实际固件或操纵硬件，物理闭环/停车/X/Y待验，R/Q不变。

> 2026-10-21单次航向入口实际构建通过：38044/576/1400/25408、0错0警告/4秒，实际源Init清零和BeginHeading已核。下一项按[整组动态更新手动指南](P1_HEADING_DYNAMIC_UPDATE_MANUAL_2026-10-21.md)，只在实际外部工程h/c加会话隔离、pending整组更新、方向反馈检查起点及UpdateHeading后完整Rebuild；main不接新接口，不下载、静态模式保持。mock状态机回归及实际宏/头文件ARMCC候选编译通过；同向更新/零速允许，停车优先，同会话反向/越界暂故障拒绝。10秒上界未放宽。源时效/会话整合、反向纠偏和真实动态运动/停车/X/Y仍待验；MOT-02加速度轨迹未完成，R/Q不变。

> 2026-10-21正常停车接口实际Rebuild已通过：38028/576/1400/25408、0错0警告/5秒；实际源已核。唯一下一项按[单次旋转入口手动指南](P1_HEADING_SINGLE_SESSION_MANUAL_2026-10-21.md)，在实际外部工程补Init缺少的两行finish状态清零，添加BeginHeading声明/实现后完整Rebuild；main/静态配置保持、本轮不下载。新入口复用原四轮准备、只选旋转段并使用传入正负轮速，原10秒上界保留。隔离mock及ARMCC候选编译通过，不签动态更新/真实闭环或物理停车通过；动态更新及会话整合仍是后续未完成工作，R/Q不变。

> 2026-10-21最新跨任务交接及正常停车准备：用户实际Rebuild37952/576/1392/25408、0错0警告，截图确认发布1/bus_valid1/rpm5/stop0，取消后state5/cause4及bus_rpm0/stop1。下一项按[正常停车接口手动指南](P1_HEADING_ORDERLY_STOP_MANUAL_2026-10-21.md)在实际外部工程chassis_auto_test.h/c加入RequestStop及首次截止、结束序列标志，用户完整Rebuild；当前不改main/不下载，静态模式保留。候选实际状态机mock-CAN检查及真实头文件/宏ARMCC隔离编译通过；实际动态调速、目标角闭环、物理停车/X/Y仍待验。接口完成只表示停车流程结束，不代表目标角成功，R/Q不变。

> 2026-10-21实际工程路径纠正：用户当前Keil为`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`，本机只读确认实际Publish/Read已保存且构建37688/564/1372/25404、0错0警告/5秒。此前项目内副本缺少接口是副本差异，不回退用户进度、不自动同步覆盖。唯一下一项按[任务间静态预览接入](P1_HEADING_MAILBOX_TASK_PREVIEW_MANUAL_2026-10-21.md)，用户只改实际main：TaskA发布、CAN任务读快照到观察变量，再完整Rebuild；保持静态模式、暂不下载。数据交接不代表许可或物理停车，真实动态执行/X/Y仍待验，R/Q不变。

> 2026-10-21最新：用户实际Rebuild37688/564/1352/25408、0错0警告；真实输入到四轮预览已截图确认正向方向全1/速度5、取消后全0速、反向方向全0/速度5，encode_result1，不重复该组检查。下一项按[CAN指令交接接口手动指南](P1_HEADING_CAN_MAILBOX_MANUAL_2026-10-21.md)，用户在chassis_auto_test.h/c加Publish/Read数据接口后完整Rebuild；不改main、不下载、不接CAN。隔离1ms/10ms tick数据及过期检查、ARMCC候选编译通过，真实调度和执行层尚未整合。R RUN-01/PWR-01、E LOC-03/05、RUN-06、MOT-02/04已检查，官方Q保持待技术答疑确认，实际闭环/停车/X/Y未验。

> 2026-10-20执行层第一小步已准备：[四轮指令转换手动指南](P1_HEADING_MOTOR_ADAPTER_MANUAL_2026-10-20.md)。用户在chassis_kinematics.h/c加入Chassis_HeadingCommands后完整Rebuild，当前不改main、不接CAN、保持静态模式且无需下载。复用实际0/1/0/1方向表，±5RPM内构造纯旋转，越界拒绝并清零。隔离GCC实际C运行和ARMCC候选编译通过；真实工程由用户手改，实际运动仍未放行。

> 2026-10-20最新板上进度：虚拟角度自检及真实HWT静态预览的0度到位、+45/-45度计算方向、取消、连续运行超时均已由用户截图确认。真实预览构建37488/564/1348/25388、0错0警告。最新超时state4/cause3/command0/stop1，VM请求及两fault0；此前cause2保留为独立失效记录，不冒充超时通过。下一项准备目标航向执行层手动接入指南及隔离检查，不重复静态用例，不直接放行电机；实际固件仍由用户手改。详见[P1静态预览结果](P1_HEADING_LIVE_PREVIEW_RESULTS_2026-10-20.md)。真实闭环、停止和实时X/Y仍待验，R/Q不变。

> 2026-10-20用户明确更正：此后代码和Keil工程由用户自己修改，助手给文件/位置/代码/解释/验证，不再直接改实际工程。用户已修ErrorDeg分别归一化并Rebuild通过：35604/452/1332/25284，0错0警告/4秒；只读源核对正确。最新唯一下一项用户按P1_HEADING_DRY_SELFTEST_MANUAL_2026-10-20.md第1～4节手动切现有静态模式并在main加一次性虚拟角度自检，然后Rebuild日志。指南同段代码隔离main/控制器ARMCC通过，虚拟result1/case0/正5,3,1,0,0/反-5,-3,-1,0,0/跨界2度，当前控制器58项边界检查通过；实际源/工程未改。板上自检及真实目标角闭环仍待验，不把旧53项或旧构建当成当前物理通过。

> 2026-10-20最新：远程origin/main仍为158e3df，无队友新提交。当前真实进度按10月19日交接：四轮受限运动、双向HWT航向随动与自动停VM已过，不回退到10月15日控制接口准备。已新增纯计算`heading_controller.c/h`并加入Keil工程，未接main/CAN/VM；5/3/1RPM分段、±180°跨界、连续稳定、HWT失效、超时/取消共53项主机检查通过，完整Rebuild 0错0警告，尺寸仍35604/452/1332/25284。当前唯一下一项是建立与旧限时转向互斥的目标航向干运行模式，复用现有准备/停止/取消/VM兜底，先离线验证+45°/-45°及跨界，再完整装机载物低速实测；实时X/Y仍未闭环。

> 2026-10-19后续开发增加[决赛现场调试交付](FINALS_FIELD_DEBUG_PLAN_2026-10-19.md)：集中底盘/定位/路线/雷达参数与中文说明，读回有效值和版本，分模块自检/有界测试、停止/故障诊断与稳定版恢复。沿现有USART1与唯一CAN服务逐步接入，实际调试协议尚待设计，不以当前Watch变量当正式现场入口。保留下面控制接口手动修改/Rebuild这一唯一下一项，不自动改源或放行运动。

> **2026-10-18最新：按用户明确要求假定兼容，先做受限A6试验**：不再把版本确认当本轮前置。hwt101增加显式request1静止确认后1s+前60s记录+A6 01采10s+A6 04退出+静置1s+后60s记录；VM持续0，PC13不运动。只试A6，不改A7/归零/保存/死区/出厂参数。失败时尽力有界退出，发送完成与模块接受/漂移改善分开。2109项模拟检查通过，Keil Code33896/RO448/RW1208/ZI25272、0错0警告。下一项烧写后放稳，version_result1/valid_mask3时cal_request1，F5持续约2分15秒，不碰车不暂停，再报告cal_result/before_deg/after_deg。兼容仍是用户试验假设，不写成厂商确认；实物试验/效果待验。详见P1_HWT101_A6_STATIC_TRIAL_2026-10-18.md。R RUN-01/PWR-01、E LOC-03/05/RUN-06/MOT-04，Q不变。

> **2026-10-18版本回复实测成功，兼容性仍待核对**：用户新版截图result1/tx5/valid_mask3，展开帧55 5F E2 27 0A 00 00 00 00 00 C7，校验正确，接受连续版本请求及双向串口本次交易通过。公开HWT101协议将2E定义16位版本，按此字段为0x27E2=10210；程序raw0x000A27E2只是前四payload字节拼接，不能据此解读为新文档1003X.1.8或宣布A6/A7支持。用户新截图为另一32位版本布局，需维特上位机的明确设备版本或厂商对应说明确认；不凭数值大小判新旧，不发送尚未确认的校准/配置命令。证据hwt101_version_reply_user_pass.json/png，保留静态VM0。R RUN-01/PWR-01、E LOC-03/05/RUN-06，官方Q不变，实际版本格式/校准兼容属工程待核。

> **2026-10-18最新：版本超时，改连续发送后复测**：用户截图tx_bytes5/version_result3/raw0/reply全0、valid_mask3，证明字节已写USART数据寄存器而角度接收正常，尚不能证明PD5物理发送线、模块接受或固件不支持。审查发现原TXE任务轮询可能使帧内字节间隔达10ms；已改为USART2 TXE中断连续发五字节、与RX同IRQ分别服务，保留200ms发送/TC和1000ms回复截止，UART错误立即停剩余发送；不重置早到回复基线。2072项模拟检查通过，Keil Code32388/RO448/RW1112/ZI25272、0错0警告。此发送改进排除一个软件变量，不宣称确定是原实物超时根因。下一项下载后F5运行约3秒，再看version_result/raw/reply；仍超时再结合PD5→HWT RX物理路径排查，保持VM0，不写校准配置。

> **2026-10-18最新：HWT101先读实际固件版本**：用户补齐HWT101协议/A6/A7与AGV版本门槛截图，明确开始实施。已启用USART2 PD5 TX并增加一次上电只读版本FF AA 27 2E 00及校验5F回复，2062项模拟检查通过，Keil Code32220/RO448/RW1108/ZI25276、0错0警告。保持APP_HWT_STATIC_TEST1、VM关闭、PC13不运动；未发送A6/A7/归零/保存。下一项烧写后F5运行约3秒，看version_result/raw及展开reply，确认模块RX实际接PD5；拿到实物版本后才选择兼容校准流程，不把示例版本当实物。详见P1_HWT101_VERSION_READ_2026-10-18.md。HWT已由用户验证角度有效与正反符号/转回，不重复验。R RUN-01/PWR-01、E LOC-01/02/03/05/07、RUN-06/MOT-04核对，Q不变。

> **2026-10-18最新：HWT101只读静态验收固件已构建**：用户确认HWT101型号及已装接线，新增USART2 PD6 Rx/115200只读接收和0x52/53校验解析，不写模块参数；2036项模拟检查通过，Keil Code31616/RO440/RW1060/ZI25244、0错0警告。当前APP_HWT_STATIC_TEST=1强制VM请求0、忽略PC13启动，覆盖此前按键即可运动步骤。下一项有线Watch看rx_bytes/angle_count/valid_mask/yaw_deg，再VM0手转车核角度变化；实物接线/波特率/实际数据和方向均待证据。此前用户已确认5RPM三方向及VM0V，不重做该旧项。细节P1_HWT101_STATIC_READ_2026-10-18.md。R RUN-01/PWR-01、E LOC-01/02/03/05/07、RUN-06/MOT-04核对，Q不变。

> **2026-10-18实车接续**：用户确认本次PC13脱机5RPM三段运行正常，前进/左移/逆时针三个方向均正确，结束VM实测回到0V；接受本项有限方向及结束断电测试通过，不推断精确速度/距离/角度、闭环定位或整车功能通过。用户另确认WT姿态传感器已安装接线。既有接口计划为HWT101/USART2 PD5 TX、PD6 RX，当前尚未实现该串口姿态读取，实物型号/实际接线/波特率仍须核对；下一步先静态采集和航向方向/新鲜度检查，不触发PC13运动。证据chassis_5rpm_three_directions_user_pass.json。R RUN-01/PWR-01及E LOC-03/05、RUN-06/MOT-04，Q不变。

> **2026-10-18最新用户调整：按下直接启动、5RPM**：取消前版5秒倒计时和人工等2秒/首次释放要求，开发测试模式检测PC13稳定按下20ms即请求启动，上电已按住也接受一次。仍先经Safety/CAN就绪与四轮失能/使能/零速检查才发运动帧，不承诺按下同一时刻转动；前进、左移、逆时针各约10秒，每轮目标5RPM（实测整数速度绝对值>7RPM中止的开发候选阈值）。一次上电仅一次，运行中再次按下取消，结束关VM。140项模拟检查通过，Keil Code30792/RO440/RW1012/ZI23132、0错0警告，原扫码模式main/key均ARMCC编译通过。无线烧写后可直接按键，无需Debug/Watch。先前1RPM、5秒延时和人工等待步骤均被本条替代；新脱机实转仍待现场验证。R RUN-01/PWR-01，E RUN-06/MOT-04已核对，Q不变。

> **2026-10-18最新：PC13脱机三段测试**：用户已提交演练state9/result1/cause0截图并确认VM实测0V，默认演练通过；实际三段车体运动尚待验证。用户确认PC13可按，已新增独立开发模式APP_CHASSIS_KEY_TEST_ENABLE=1：每次上电只接受一次PC13启动，5秒延时后请求VM，最多等5秒健康，自动三段各1RPM/10秒、停车并关VM；倒计时/等待/运行中再按取消，结果后再按不重启。设该编译开关0恢复原扫码按键路径。无线烧写后脱机按键运行，不依赖Debug；仅需Watch排查时再接有线。124项模拟检查（原103+新21）通过，Keil Code30808/RO440/RW1012/ZI23132，0错0警告；原扫码模式ARMCC编译通过。尚未下载或操纵硬件，脱机实转待用户确认。详见[P1_PC13_OFFLINE_CHASSIS_TEST_2026-10-18.md](P1_PC13_OFFLINE_CHASSIS_TEST_2026-10-18.md)。

> **2026-10-18当前唯一现场下一项（三段固件已构建，默认演练待现场）**：源码/引用/AXF及10月17日Keil日志已核对，自动三段执行已接入，103项整合检查过，Code30268/0错0警告，无需重复编译。重新载入工程下载后供电健康，默认g_chassis_auto_dry_run1，只设一次g_chassis_auto_request1，运行约40秒；预期result1/state9/cause0/cut1并表测VM0。尚无新三段演练/实转证据。上次文档写入因额度未执行，现已补齐；下方“尚未接入”均历史。详见P1_CHASSIS_AUTO_SEQUENCE_2026-10-18.md。

> **2026-10-17当前下一项（轮速适配已编译，三段执行尚未接入）**：用户运动学Rebuild含新c且0错0警告；按减少Watch手动核算的要求，已增加整数RPM/方向/反馈适配及三基本指令准备。实际模块4451+228检查过，本机已自动Keil Build，Code26016/RO440/RW904/ZI22992、0错0警告，不要求用户重复编译。下一项接有限三段执行/真实反馈/每段和总截止/故障后不自动续跑，当前jog仍四轮正向10秒；未下载/发新动作，VM保持0。详见P1_CHASSIS_COMMAND_ADAPTER_2026-10-17.md。下方待登记/待编译等均为历史。

> **2026-10-17最新下一项（重新加载工程登记）**：用户Rebuild0错0警告但新模块未编译；实际uvprojx无c/h登记、lnp/map无该对象。已重新只加入USER两项并验证XML/路径。现在重新打开磁盘工程，避免保存旧Keil列表覆盖，再Rebuild确认chassis_kinematics.c编译行。VM保持0，新数学模块源码和已通过检查未改。

> **2026-10-17最新下一项（X形计算模块待完整构建）**：用户已确认轮径77mm、轮距/轴距各约210mm及X形，尺寸确认阶段完成输入收集，精标仍待。新增USER/chassis_kinematics.c/h及Keil组引用，只做纯逻辑轮速正逆解/同比限幅、异常清零，无驱动动作；隔离ARMCC及实际模块4451项检查过。现在VM0完整Rebuild，确认新模块被编译；之后统一逻辑符号/反馈/量化和运动安全再接车体运动。下方要求再提供X形/尺寸是历史，不重复。

## 当前下一项（2026-10-17，四轮修正正向已实测通过）

用户确认四轮方向映射0/1/0/1正确，已完成前一项复验，不重做Rebuild/相同轮向测试。现在确认实际轮胎外径、左右轮中心距、前后轮中心距（mm）和麦轮滚子排布/车头方向，再建立统一轮速与反馈符号、麦轮正逆解及限幅/正式反馈安全。仍在阶段3底盘与定位，正式运动许可、几何与实际RPM精度、HWT/实时X/Y未完成。下方要求再次复验方向的段落均为历史。


## 现在唯一下一项（2026-10-17，方向映射已修正）

先保持VM请求0、外部VM关闭，完整Rebuild当前main，然后以原1RPM/10秒四轮流程复验逻辑正向。方向表为1左后0、2右后1、3左前0、4右前1。该版隔离编译及实际C主机109检查过，未完整链接/下载/实车复验。刚才截图四启动/四停止mask0F、state4/cut1/cause0仅为旧方向候选的本次运行结果；2/4方向按用户实测报告修正，不能把旧截图当新映射已过。

### 复验后沿交接继续

1. 冻结四轮逻辑正向，测量实际轮径、左右轮中心距、前后轮中心距并确认滚子排布。
2. 建立逻辑有符号轮速/反馈符号、麦轮正逆解及统一限幅，补正式运行反馈时效/失效停车/重新允许；现有调试动作不等于正式许可。
3. 受控验证前后、横移、原地旋转及组合动作，再接HWT101航向与实时X/Y定位；无有效X/Y不放行位姿跟踪。

仍处于交接阶段3“底盘与定位”，不是退回基础CAN，也不提前跳雷达或整场路线。当前已过基础、应答与调试动作不重复；正式运动和安全未闭环项继续保留。


> **2026-10-17当前下一项（四轮版本地完整构建已核对）**：实际build_log为Code26012/RO424/RW904/ZI22992、0错0警告，当前map/AXF含新增四轮符号；用户粘贴25492大小为旧日志。现在下载当前四轮产物并确认Watch数组/掩码后再触发，四轮实测尚未完成。

> **2026-10-17当前唯一下一项（四轮共同运行版待构建）**：用户确认1号反向正确，并明确选择四轮同时1RPM约10秒验证正向。已自动四失能/确认→四使能/确认→四F6方向0→四FE，准备5秒/运行10秒/截止10.5秒，默认dry1；主机109项实际C检查及main/Safety隔离编译通过，未完整链接/下载/四轮实测。现在VM0完整Rebuild给日志。方向0仅1号已确认正向，后三轮需观察；分笔CAN有启动差，不宣称精密同步。详见P1_FOUR_WHEEL_FORWARD_TEST_2026-10-17.md，下方旧单轮说明仅历史。

> **2026-10-17当前唯一下一项（1号反向版待构建）**：用户报告1号方向0正向转约10秒/停止/关VM，2～4无主动转动。按要求仅将main方向改1，速度raw1、10秒/10.5秒截止、自动四失能及仅1使能流程不变；ARMCC隔离main编译通过，未完整构建/下载/实测反向。现在保持VM0完整Rebuild给日志，随后同入口测反向。默认dry1保留，正式运动许可0。下方旧下一项仅历史。

> **2026-10-17当前唯一下一项（10秒自动准备版待构建）**：旧1秒默认演练state4/cut1/cause0与VM实测0V已过，用户随后报告见1号转动，要求延长10秒并自动先全部失能。已实现自动四失能→读回→仅1使能→读回→1RPM约10秒→停止，10.5秒关VM，准备5秒独立截止；默认dry1不发速度/停止帧，但自动F3准备真实执行。main/Safety隔离编译及实际C主机94检查通过，未完整链接/下载/实测新版。现在只保持VM0完整Rebuild给日志，后续新版演练不需手动四轮准备。下方1秒/手动准备说明仅为历史。正式运动许可仍0。

> **2026-10-17当前唯一下一项（1号1RPM短时测试版待构建）**：用户实测四轮失能再使能均通过。已加固定ID1/方向0/raw1约1秒及Safety独立1.5秒关VM；默认dry_run1不发速度/停止帧，先验证新截止实际关电再另行实转。main/Safety隔离编译及实际C主机80项检查通过，尚未链接/下载/实物演练。现在仅保持VM0完整Rebuild给日志。正式运动许可仍0，实际轮向/转速/制动及新截止硬件验证未完成。详见P1_WHEEL1_SHORT_JOG_2026-10-17.md。下方旧下一项保留历史。

> **2026-10-17当前唯一下一项（直接使能修复待构建）**：实测失能后F6返回E2导致旧使能前置阻塞。按用户要求main改为直接单次F3使能，删除enable_zero_result/reply；所有原条件/优先级保留。隔离ARMCC及实际分支主机69项检查通过，未链接/下载/实测。先撤VM关电，然后完整Rebuild给日志，再测失能后选址使能。历史零速前置版说明由此覆盖；非零运动仍未开放。

> **2026-10-17当前唯一下一项（锁轴入口待完整构建）**：失能版完整Rebuild0错0警告，用户报告四轮实测松轴均通过并完成关电步骤。main已新增g_can_enable_*，一次请求先发固定零速，成功后复查条件才使能；隔离编译及实际函数/分支主机106检查通过，尚未链接/下载/实测。现在只做完整Rebuild给日志，VM请求保持0；随后单轮锁轴/再失能。正式运动许可固定0，不开放非零速度。下方旧下一项保留历史。

> **2026-10-17当前唯一下一项（失能入口待完整构建）**：main已新增g_can_disable_*单次静态失能入口，固定enabled=0；ARMCC隔离编译与真实分支主机48项检查通过，未链接/下载/实测。现在只做Keil完整Rebuild给日志，VM请求保持0；之后按地址1验证F3应答及3A状态bit0=0。下方旧下一项保留历史；不开放使能或非零速度。

> **2026-10-17当前下一项（四台缩放现值已报告）**：用户现场报告四台S_Vel_IS全Disable，按Emm raw1=1RPM记录，不改菜单/不发4F；尚无菜单原图或非零精度验证。先结束查看菜单的VM供电，撤请求运行并实测0V，再准备单节点使能/失能静态入口及3A状态核对。非零速度、正式运动许可/运动时效限值和真实制动尚未验收，不用协议上限当安全速度。

> **2026-10-17当前唯一下一项（两种关断静态分支已过）**：停止发送失败软件注入cause1/cut1及用户实测VM0V、保持锁存、明确确认复位均通过；fail计数1→0为用户明确手动清零，非源码自动变化。超时cause2和正常四轮应答也已过，不重复。下一项只读核对四台驱动“缩小10倍输入”等速度缩放现值，不改配置、不发4F；其后设计使能/失能与受限非零测试。当前运动许可固定0、实际运动/制动和完整正式恢复尚未验收。

> **2026-10-17当前唯一下一项（发送失败注入版待构建）**：main已加入默认0的一次性自动停止发送失败模拟及计数，注入时不发帧、返回4并走原cause1关VM分支。隔离ARMCC和实际C主机82断言通过，未链接/下载/实物验收此版。现在用户完整Rebuild给日志，随后按健康启动后设关VM断点的步骤静态注入并实测。超时关断/恢复已过不重复，运动许可仍0。

> **2026-10-17当前下一项（超时关断已过）**：静态自动停止超时state3/cause2/cut1、用户实测VM0V，以及撤请求后保持关断、明确确认后state0/cut0/锁存0/result1已通过；最终VM请求0、故障0、运动许可0。下一项准备停止失败cause1的软件注入，先隔离检查再用户构建/静态实物关断验证，不重复超时case；真实运动制动/正式运动许可和速度限值仍未验。

> **2026-10-17当前唯一下一项（超时注入版待构建）**：静态自动停止四result1/ack0F/state2、恢复不重复触发、VM0明确确认复位均过，用户实测0V。main已加默认0的g_motion_diag_hold_stop_service，暂停自动停止领取而保留采集/状态保护，用于后续验证500ms候选超时实际关VM；隔离编译过。现在只做完整Rebuild给日志；其后按先健康启动、后设Power_Disable关断后断点的顺序静态注入并实测VM，未测前不签失败/超时兜底或运动停车通过。

> **2026-10-17当前唯一下一项（自动停止版待构建）**：诊断锁存/恢复不自动清除/VM1确认拒绝/VM0明确确认静态检查已过，用户实测VM0V。现已接入Safety发布四轮停止、CAN任务执行、失败或500ms静态候选超时追加关VM条件；运动许可仍0。ARMCC隔离两单元及真实C函数主机63断言通过，未整工程链接/下载/实物验收此版。现在只要求用户完整Rebuild给日志，后续静态验证自动FE接收；不能声称物理停车或正式运动恢复完成。

> **2026-10-17当前下一项（诊断锁存版待构建）**：Safety已加入反馈诊断锁存10/20、禁止原因40及VM请求关闭后的一次性确认接口；旧反馈恢复不自动清诊断，确认不清原Safety故障/不允许运动，复位会清RAM记录。Safety/main隔离编译过，尚未链接/下载/实测此版。唯一下一项用户完整Rebuild日志，再逐项静态检查默认、hold过期锁存、反馈恢复仍锁存、VM1确认拒绝和VM0确认；正式运动许可/停止联动/重新供电仍未完成。

> **2026-10-17最新验收后下一项（过期/恢复已过）**：静态旧快照自然过期与恢复通过：report178冻结、Safety两fresh0/reason38；恢复发布report1CF/两fresh0F/reason08，故障0/两个ready1/allowed0，用户最后撤VM实测0V。下一唯一开发项是默认禁止的运动故障锁存及明确确认诊断层，先隔离验证，再用户构建/静态检查；正式CAN停止联动、故障重新供电/重新允许、运动时效和非零速度仍未实现。不要重复旧快照检查或把新鲜恢复说成正式恢复已过。

> **2026-10-17当前唯一下一项（冻结发布诊断版）**：时效诊断完整构建及3D→38→08→38静态转换已过，用户撤VM实测0V。main已新增默认0的快照发布暂停开关及发布计数，隔离编译过；现在请用户完整Rebuild给日志，再验证旧采样在Safety自然过期。此时不重做旧四轮控制应答，不放行运动；正式许可、运动阈值、失效停车/重新允许仍未完成。

> **2026-10-17最新下一项**：首版运动禁止诊断Rebuild及VM0的0D/VM1就绪的08显示已过。现已加入速度/位置采样时间的Safety独立时效诊断（仍为静态1000ms，allowed固定0），隔离编译通过；唯一下一项为此版本完整Rebuild给日志。新预期VM0健康reason3D，VM1反馈关闭reason38，两类四轮反馈新鲜reason08。正式运动时效、失效停车、许可/恢复仍未闭环。

> **2026-10-17接手更新（优先于下方10月15日旧快照）**：CAN控制接口已真实构建，四轮静态停止/零速接收应答及随后RPM0读取已完成；撤VM用户实测0V，关闭请求下两个测试入口result9。当前开始运动许可第一阶段诊断，Safety新增allowed固定0及禁止原因位，尚未实现放行/恢复。**唯一下一项：诊断版完整Rebuild给日志**，其后只观察VM0默认allowed0/reason0x0D。助手隔离编译Safety/main已过，未链接/下载该诊断版；不能把静态应答或诊断当完整运动安全。细节以执行记录2026-10-17条及当天证据为准。下方旧的“接口尚未改/尚未构建”只作历史。

> 2026-10-15用户要求进入下一步，已确认整车机构装齐、可载物测试（仅装配状态报告）。当前唯一下一项按P1_CAN_MOTOR_CONTROL_INTERFACES_MANUAL_2026-10-15.md第2～5节手改can.c/can.h并Rebuild：新增使能/立即停止/Emm原始速度接口，保留四种读取与现有main。厂家42/43/50/53/60/81页已核对；实际速度缩放设置未核实，非02应答返回8保留原始帧。隔离ARMCC can/main编译及101项主机模拟断言通过，真实源未改、未链接/烧录/发CAN。正式构建后再接停止/零速度验证及运动门禁，不直接发转动指令。

基线docs/competition_requirements_2027.md；沿用用户旧编号12374568，搬运机构由视觉组并行，电控主线按新阶段1→3→4→5→6→7→8。用户手动改F407，SPL+FreeRTOS原生API、F103模块风格、中文注释；成品PCB不改，PA1软件选档跳过。以下为工程安排（E），不改正式规则。

## 当前已验证与仍待完成

截至2026-10-15，用户实物/日志已覆盖：8MHz外部时钟与168MHz基础运行、SPL＋原生FreeRTOS、LED/蜂鸣、USART1收发/独立RX、PC13自恢复按键消抖/释放重接、任务监督/RTOS故障关VM、IWDG正常与停喂复位/调试冻结行为、PE2默认VM0及独立硬件关断。UART4原屏完整15字符任务码、UART5实际扫描/多次换码、PC13启动请求锁码均已有反馈；正式结构化业务与完整READY/重新允许尚未完成，赛后抓放统计按用户决定延后。

四台X42S_V2.0/ZDT_CAN_V1.2/FW_Emm/CAN1_MAP/500k/固定6B，地址1左后/2右后/3左前/4右前，单独及共总线版本/状态、四台速度/位置单次读取均已过。持续采集以50ms交易后间隔通过，现已移入独立CAN任务，扫码/屏幕TaskA10ms，调试串口短临界区包装递归错误已修并正式构建/启动。分离后两图count18/18/17/17→39/39/39/39，两mask0F/error0、两fault0/两ready1，用户确认换码屏幕正常更新；CAN查询负载余栈297字、APP最低169字、监督230字、heap11320B。缺失ID1静态fault20/原因2/坏节点1及VM实测0、接回复位平台恢复且VM仍0均已通过。详见2026-10-15证据与执行日志，不把旧10ms超时认定为唯一频率原因。

上述是已接模块的基础/静态门禁，不等于整车安全或运动完成。用户最新报告机构完整装好、可载物测试，后续实际运动仍遵守完整装机载物低速；实际载荷、轮向/几何与安全速度限值待量测。连续数据失效到运动停止、正式就绪/重新允许、运动时效、HWT及实时X/Y等尚未完成。

## 阶段与交付

| 执行序号／旧编号 | 工作 | 验收重点 |
|---|---|---|
| 1／1 F407基础收尾 | 现有RTOS/通信框架保留，补公共就绪状态、默认禁止输出、急停/故障监督及屏幕/扫码；PA1软件不做 | 每个必需模块自检/超时明确；当前核心测试已过，不签整车安全完成 |
| 2／2 搬运机构（视觉组并行） | 夹爪、270°三槽盘、搬运轴/回零、机械臂动作组、库存与臂盘互锁；场地原料转盘上表面80～100mm集中配置 | 电控提供公共驱动/底盘状态，真实高度与视觉标定版本一致；取放实物证据，载物姿态就绪后才联调实际运动 |
| 3／3 底盘与定位 | 电机CAN硬件/固件/地址/状态→四轮反馈；轮径/轮距/滚子排布与方向；正逆解、同步/限幅、HWT101航向、实时X/Y与视觉纠偏 | 先静态/软件和安全前置，实际从完整装机载物低速；四向/旋转/组合误差，反馈丢失停车；X/Y未闭环不放行位姿跟踪 |
| 4／7 路线与曲线基础 | 已知受控路线，位置/朝向联合到位、连续途经点、S曲线及速度/加速度/加加速度约束 | 轮速和车体扫掠包络、载物稳定、终点停稳；做受约束耗时优化，不用未验收雷达图驱动 |
| 5／4＋6前置 雷达机构及验收 | 先180°舵机升降、ULN2003转向、零点/高度/安装固定/停稳，再实际标定及完整14圈扫描 | 两台各自配置/误差/黑障完整性；不用仅改ID硬并TTL TX，机构改变重标定 |
| 6／5 雷达协议 | F407中断/DMA收字节，任务处理COBS/CRC、握手、选区、心跳、scan_sequence与结果、超时/过期 | ACK不等完成；坏帧/缺包/旧结果拒绝，接收与安全任务不被等待阻塞 |
| 7／6剩余 自动扫描联动 | 启动条件满足后按合规时机升降→转向→停稳→扫描→完整结果→准备状态/提示；次数配置沿用 | 开局尺寸与出发后展开约束；失败不报全系统成功，不仅凭几个Init完或一次ACK触发 |
| 8／8 视觉整合及整场 | 真实障碍地图接入路线规划；电控与视觉统一坐标/时间/高度/标定版本，结构化目标和动作回执，载台库存/运输互锁 | 单件抓放后全任务，六色/四段码/显示，双抽签起点返回、车道包络、故障恢复与续航；必须有实车/视频/同步日志 |

顺序未重新改为雷达优先；新阶段5内部仍机构先于实际扫描。视觉接口提前约定，完整整合集中阶段8。开局图可靠且障碍未变可用本轮图，行驶中定位和必要感知仍持续。PA1跳过不删除FIELD-02抽签起点身份，start_id输入配置后续实现，不能假定雷达已自动确认它。

## 历史下一项（2026-10-15，已由当前进度覆盖）

用户手动按[使能/停止/速度接口指南](P1_CAN_MOTOR_CONTROL_INTERFACES_MANUAL_2026-10-15.md)第2～5节，只改SYSTEM/CAN/can.c和can.h，Rebuild并提交日志。main保持当前已验独立CAN/扫码/监督版本，不增加动作请求入口。通用单帧收发承接旧初始化/发送100ms/接收200ms及回帧校验，旧四种读取函数体不变；新增控制地址限1～4、同步位固定0，控制应答02仅表示接收正确，非02原始回帧保留返回8。速度接口明确使用原始命令值，四台缩放10倍设置未核实前不当作RPM换算。协议上限3000不是整车允许速度。

准备证据：厂家手册V1.0.5第42/43/50/53/60/81页和渲染图；隔离副本ARMCC5.06编译can.c及现有main，0错误警告输出；实际拟议函数体在主机模拟CAN/RTOS环境执行101项断言，含帧内容、无效参数不发帧、非02应答、过滤、超时、tick回绕及旧查询解码。真实can.c/h/main哈希前后不变、无链接/烧录/外部CAN动作；私有目录清理状态在证据中记录。此证据不替代用户构建和实际控制应答。

后续先接立即停止/零速度的可审查入口与实际回帧，再补正式运动允许、连续反馈失效到停止、较快运动时效/超时及有限动作窗口、载物低速限值、轮向/几何/正逆解/同步和HWT/运行X/Y。当前1秒数据候选只用于静态，不能据此宣称可以安全行车。原八阶段顺序、视觉并行工作包、抓放统计延后及用户手改中文SPL/FreeRTOS不变；不重复已过读版本/状态及原基础采集测试，后续仅针对新通用收发的真实读取回归和新控制路径验收。

## 规则检查与未闭环

R：FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04、PWR-01、MECH-01/02、QR-01、DSP-01/02。
E：LOC-01～07、MOT-01～04、RUN-05/06、CAL-05、VIS-09/12/14/15。
Q：FIELD-05、OBS-03、QR-04继续待技术答疑确认，不因工程选择而自动闭环。
工程未闭环：用户本轮控制接口正式构建/修改后实际读取回归/控制应答、四台速度输入缩放设置、运动反馈与失效策略/正式停止重新允许、实际载荷/轮向/几何/限值、HWT/实时X/Y、雷达机构/完整扫描、搬运视觉高度/动作/回执、赛后抓放真实统计/触发/字体实测及整场。基础IWDG、真实扫描显示/锁码、独立持续采集/扫码并行、缺失ID1静态关断和接回复位VM0不回退成待验；不能将它们扩大为运行中断线/实际运动或完整阶段通过。官方Q条目继续待技术答疑确认。助手未改真实固件或执行硬件操作。
