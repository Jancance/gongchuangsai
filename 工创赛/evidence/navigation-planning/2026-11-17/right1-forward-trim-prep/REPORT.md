# 2026-11-17 右移1%前向分量候选与左移坐标复核

## 本轮目标和当前状态

用户明确“先补偿1%试试，同时再看看惯导坐标、左移”。本轮按GPT-6.1-sol xhigh规划、medium编码与本机MATLAB前置、high独立实际C复核完成软件候选。只改5个活动CHASSIS/USER文件、38行纯新增，GBK/CRLF与原左分支字节保留；默认新宏关闭，隔离候选开启。新候选已Vehicle_Integrated完整Rebuild，0错误0警告，尚未下载/复位/发车；板最后安装仍79e74...fa2f0d。后退基线尚未实测，不能说新四向已齐。

## 1%实际含义与模式隔离

在原右移基础轮速b上加入车体前向t=0.01f*b RPM，heading h仍由原限幅提供；LR/RR/LF/RF命令为[-b+t-h,b+t+h,b+t-h,-b+t+h]。它不是整体轮速加1%、不是终点坐标硬加21mm，也不把命令当真实位移写进惯导。

仅APP_BASELINE40活动调试、APP_RIGHT40_FORWARD_TRIM_ENABLE=1、PLAN10/12/13、mode2、非route、AppDebugPlan40Allowed匹配完整tuple时选择新函数。10=右2100、12=右1500、13=右300mm，预算102RPM。原左2%后向trim、前/后/转向、serial guard、HWT和STOP/故障处理均未修改。

t保持float，最终只经现Chassis_EncodeEmm一次0.1RPM量化。低b1–4且heading0时实际前分量为0，b5半格可为.05RPM，b6约.1RPM，不能说每档精确1%。最高理想轮幅105.02RPM、协议实际最高105.0，右专用编码cap106；原130反馈边界保持。base0时h/t均0四轮0，越界/NaN失败清零。

右向精确tuple A/J预算150/1.01、300/1.01，原follower预留仍扣减；有效参考A132.3880mm/s2、J248.6492mm/s3，300/1500/2100mm参考时长3.58984/7.30385/8.80385s，加2秒仍小于原12.779s运行预算。参考峰300段约167.14mm/s，其余400；参考预算不等于轮胎真实加速度/jerk证明。主距离和其余方向参数保持。

## MATLAB先行与实际C验证

本机MATLAB R2020a最终attempt6于06:06:20–28 exit0，脚本SHA B67617FD19AA1ACFF24BE88DCCC539FDD8FC9C84A854D2FE5E3DA4173A3CB349，早于root批准产品写。17201轮速组合、10非法、2880scope，仅3个精确组合开启；三正向和错距反配/route/mode/inactive明确断言。逐轮最大量化差约.0500001RPM，F/L/H各约.05/.05/.049。前试夹具半格期望、MATLAB结构赋值/变量及12/13配对模型错误如实保留；high发现配对错后修正重跑，不以早期PASS放行。

medium使用真实kinematics/实际计划门控及真实Prepare校验：on/off各208124断言、inactive8295通过，默认0/合法1正编译及非法2、非baseline开启负编译通过。Begin仅临界区前实际校验段、非整个RTOS动作执行链，边界如IMPLEMENTATION_REPORT所述。

high独立ARM C开启/关闭各1018205断言，42127混合输入、6440实际门控矩阵，on仅3/off0；原generic方向校验和两配置均2467659381，原Left保持。逐轮量化差最高.0500107RPM、F/L/H约.040002/.050003/.049003，均在.0501界内；cap105、低b不编码反向、非法输入/零基速、编译保护通过。runtime条件与AppDebugPlan40Allowed从当前源原文抽取，非自写白名单；不把这些证明称为全RTOS/CAN或实车结果。

high首完整app_entry夹具因既有空route initializer失败，保留首build后改为原文提取真实门控函数；首独立仿真达到4亿指令预算而非断言失败，保留记录后只增加仿真器指令预算，无产品/阈值放宽。数值/调用级验证齐，实车效果未验证。

## 左移定位再次复核

HWT按用户可靠实际读数基准。当前OD已用同source时间、跨180展开、二维中点+sinc，未发现确定左右符号错误；ICM输出路径F/右而回放为F/左，比较需L=-R。ICM共轮锚且NO_LEVER/有效率限制尚存，不把输出接近轮坐标当独立地面精度。

冻结左210前两趟γ=.9935233、β=-.01850783保持未改，三次新左150不参加训练；high从原decoded再次独立复算与MATLAB差3.64e-12mm。左150主向表观MAE11.359→2.005mm，旁向20.060→7.961mm，但第3旁向绝对误差12.654→15.369mm变坏。合并左210第3留出与左150三趟，四留出主向12.302→1.861mm、旁向23.876→6.927mm，只有平均改善并不代表每趟都更好。均非同步地面真值，原尺量/摆位不确定性继续保留。因此本次不改左控制或部署左坐标残差。

用户补充旁偏波动有摆放和尺量因素，建议后续给轮误差一个可信权重。已接受作为未来工程残差增益讨论，不把全部散布归车；root提出50%仅为可试的离线候选，用户未确定具体值，也未测出50%统计置信度。权重作用于小残差修正而非总轮位移，不降低HWT信任；本阶段未实现该新权重。

## 构建、保存与下一单次实车

正式入口3.stm32f407工程/gongxun-f407.uvprojx；隔离工程工创赛/tmp/right1-forward-trim-build-20261117/right1_forward_trim.uvprojx，build_candidate.py明确只有-r无-f。新AXF SHA3043c4a98aed1b04eb0c80fd4b2ea819b2c70b52a2c0dbe60613a969bca48ad3，工程SHA71e86cf80538ebca88333bcb87377fb736d73b218c84d0759589bf0b0339899e，DAPLink选项继承已验证UID6afaae24，SHA397f0cae2a6cf65c6788b94a2f6d3dfa4134d42277c7d90193e4a57063d766da。frozen保存镜像/工程/选项；老79e74镜像不变。

新right1-forward-trim-field-run1已备PLAN10一次右210对照，execution_permitted/root_release/site_confirmation均false，未download/reset/START。完成本地软件提交后，确认回起点/朝向、右侧约2.6m/停车余量、收拢与断电，再一次换版/显式reset1/PLAN10 ARM START各1；不自动重复或回转。保20秒供电尾和首次RAW双读，量同测点主距/前后旁偏。若失败先留首证据，不用旧ready或盲复位掩盖。

现有4右RAW已在前序3cda6059作未补偿0/1/2%理想端点敏感性，本次未新增全4RAW的新闭环动力学回放；实车前馈与定位改进分开评价。候选成功不能自动证明惯导坐标已准。

## 规则与证据

R核RUN-01/PWR-01/MECH-01/02；E核LOC-02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04保持待技术答疑。官方规则未新增/改变。本阶段代码、数值、构建通过与下载/实测分开，后两项均未做。

证据：plan_xhigh.md、medium/{matlab_receipt.json,matlab_summary.json,source_diff.txt,actual_c_checks.json,negative_compile_checks.json,IMPLEMENTATION_REPORT.md}、independent-review/actual_c_report.json、high_review.md、left_coordinate_reassessment.json、root_preservation_and_build_checks.json、build_receipt.json。5源38行纯新增、原6dirty字节一致、GBK/CRLF保留；不fetch/push，每完整阶段本地commit。

原样保留的生成测试夹具有抽取产生的额外CR/return空格行，实际C编译与验证使用的就是这些字节；不为通过格式检查重写已测证据。5产品文件及脚本/文档另经whitespace核查通过，见generated_fixture_whitespace_note.json。
