# 六项短距预设独立审查

日期：2026-11-16。结论：**PASS_WITH_LIMITATIONS**。由当前模型独立审阅，不是6.1-sol high；依用户既有当前模型接替授权执行。未修改产品、根交接或Git，未打开串口/连接SWD/下载/复位/START。

## 范围与方向语义

检查活动工程`3.stm32f407工程/gongxun-f407.uvprojx`及`CHASSIS/USER`，以本目录`source_before`和`inputs_before.json`为本阶段前快照。新增14前1500、15前300、16后1500、17后300、18左1500、19左300；方向0/3预算100RPM，方向1/2预算102RPM，动作kind分别4/7/5/6。旧8..13及旧right30=3语义保持；默认8及比赛入口隔离保持。

2574个记录输入仅六个固件文件变化，其余2568原字节不变。六源GBK可解码，原/现均CRLF，CRCRLF及裸LF均0，无批量转码。chassis_auto_test.c实际增量仅左trim ID门禁、前横偏ID门禁及其精确Allowed调用；算法和其它正文未变，已按原字节替换式重建比较相等。app_config、倍率、heading、motion_profile、kinematics、STOP、capture与保护源均未变。

- 前/后保留纵向倍率1.0218254；后退仍继承，不能称独立地面标定。前专用横偏环仅8/14/15、mode0、精确ID/RPM/距离、非route、有效profile、12779ms及单动作启用；左/右/后不进入。
- 左2%后向trim仅9/18/19、mode1、非route及精确tuple，宏关拒绝；加速度/jerk先除1.02再扣跟随预算。其实际轮命令与前横偏不混用。右10/12/13及后11/16/17不套左trim或前横偏。
- 所有预设的Allowed、load、严格串口strcmp、ARM、KEEP_VM、powered-DONE、BF1 RPM/距离、feedback limit和idle echo已逐项核对。14..19对应双位ID解析正确，旧PLAN0旋转门禁仍拒绝，未开放任意距离。
- 新独立client的mode3是串口运行模式，与方向0/1/2/3不同；支持8..19精确tuple与旧right30=3。默认PLAN8观测允许选择新候选，收到选择后只接受该tuple。最终修复已扩为`self.plan in BASELINE40_SPECS`，独立交叉检验通过；不会因选择阶段只看到默认8而误拒绝新ID。

## 独立执行证据

1. `independent_gate_checks.py/.json`：从**实际chassis_auto_test.c**提取前横偏及左trim谓词，保留C表达式，仅把动作数取值改成夹具输入；与实际app_entry/motion headers、kinematics编译并在Unicorn执行。trim_on **10596**、trim_off **10596**、inactive **8421**断言通过。覆盖0..20 ID、0..4方向、99..103RPM、边界距离、错ID/方向/长度；另外覆盖route、profile.valid、run_ms、动作数门禁。实际`Chassis_Left40DriftCommands`验证负F投影、主左向投影、0速和轮预算。初次独审提取碰到初始化赋值，在编译前断言失败；改为最后一次目标赋值后执行，未改变产品或预期。
2. `independent_review_checks.py/.json`：**352**项通过；独立检查六源字节范围、旧/新tuple、12×12选择前后host健康矩阵、畸形命令、MATLAB先后、构建和ELF身份。没有真实串口TX。
3. 核root真实ARM C回执：trim_on **33811**、trim_off **33771**、inactive **5321**通过。核root host **392**项通过，8..19各一个独立内存单START路径。最终host scope已改为8..19/twelve，client SHA与回执一致。root曾有夹具/选择漏新ID失败，原client/失败夹具保留；当前通过不抹先前失败。
4. 实际MATLAB R2020a进程22:42:00—10 exit0，函数22:42:07—08，早于产品write22:44:14。逐向模型使用各自factor/A/J并重现实际短程S曲线分支；前后300峰约167.7025mm/s，左按1.02分配，均不是400mm/s恒速测距。此模型验证参考轨迹和离散预算，不等于含滑移/驱动延迟/地面扰动/横偏闭环的整车仿真，更不是实际速度或测距验收。
5. 实际Keil候选22:45:53—22:46:01返回0，0错误0警告。新正常及frozen AXF SHA `6b4c42de810137e39ddfe999800b4b96581790efcf4024daeec29f1262de7095`；PT_LOAD 117072B/SHA `4d2abce83bcd55461bd1d10d070d415e9dbdcd382cf28d3d0751d2171dece584`，独立ELF计算与冻结清单一致。旧已测右向AXF SHA仍`3f8fe3bf732637ebe1e9765b5ebf686c7042d536bcfd1898dc163033d260b3cd`，未覆盖。

## 规则与未闭环

已检查R RUN-01/PWR-01/MECH-01/02（正式自主入口隔离、供电和机械边界）；E LOC-02/03/05/07、MOT-02/04/RUN-06（定位、时效、四向验证、联合运动约束和出口）。Q FIELD-05/OBS-03/QR-04继续**待技术答疑确认**。本候选属于E，不新增或修改正式R。

六个新增短距尚未下载/实测；板载仍已测右向组合3f8fe3bf版本。前短程横偏环和左短程2%虽保持原方向策略，适用性仍需分别实车验证。目标、冻结progress、STOP首观察、完整里程模型与最终尺量必须继续区分；当前未重标比例、未证明滑移/旁偏原因、未签正式比赛参数接口或任意距离/速度。四向统一软件可准备一次下载，但每趟需现场授权及单START，不能把12条内存路径当自动实车连跑。
