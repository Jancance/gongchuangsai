# 两趟成功诊断离线数据（2026-11-15）

结论：两趟尺量右偏20/30mm、超距7.5/10mm，明显大于half历史四趟的右偏。OD中段峰均4mm不能否定尺量恶化；批次非严格配对，不能把原因唯一归于FF、插桩、供电或复位。用户已取消继续测试，本分析不发车、不连接硬件、不改控制。

## 来源与计时口径

两趟RAM均accepted稳定双读：前后RUNNING、同session、DONE9/result1/active0、VM/PE2高、fault0/PLAN8，四bin哈希与receipt一致，绑定5e803d…a185 ELF。离线parser的chip_cycles_measured=false/hardware_provenance=false是固定来源声明；外层真实硬件receipt和trusted_acquisition=true已核，不能因此误说未测。

DWT elapsed含IRQ和调用边界；以下百分比仅所测两个范围累计周期/各自first至last时窗，不是CPU总负载。168MHz为记录的SystemCoreClock，未另测RCC。

|趟次/范围|count|window ms|min/mean/max us|sum us|本范围时窗占比%|
|---|---:|---:|---|---:|---:|
|diagnostic-run1/target_plus_feedforward|113|7964|7.5179/7.8977/10.0060|892.440|0.011206|
|diagnostic-run1/step_budget_quantize|113|7964|3.7738/3.7934/3.8214|428.649|0.005382|
|diagnostic-run2-retry1/target_plus_feedforward|113|8030|7.5179/7.8885/9.9405|891.399|0.011101|
|diagnostic-run2-retry1/step_budget_quantize|113|8030|3.7262/3.7911/3.8155|428.399|0.005335|

两段同窗合计比例分别为0.016588%/0.016436%。这些已测范围不是毫秒级热点，当前不支持为提速改定点；未测路径不据此宣称没有热点。

## 末端三个真实软件状态

记录中的now/run_tick/common_ms来自同一板时基；remaining采用原profile duration8.7786626816s减actual elapsed。follow_rpm为跟随修正状态，pending/applied为目标命令，并非实际轮/地面速度。latest freeze不是完整四轮ACK；arrival槽在reached赋值前。

### diagnostic-run1
- first_tail_entry: now/run_tick=35914/28654ms，elapsed=7260ms，common_age=109ms；common mean/ref=1993.099/1993.736mm，差-0.637mm；ref F/follow=40.585/+0.810RPM，pending/applied=42/44RPM；profile_done=False，原profile还余1.518663s。
- latest_tail_batch_freeze_NOT_complete_ACK: now/run_tick=36618/28654ms，elapsed=7964ms，common_age=120ms；common mean/ref=2095.361/2078.355mm，差+17.005mm；ref F/follow=17.768/-0.132RPM，pending/applied=18/18RPM；profile_done=False，原profile还余0.814663s。
- arrival_BEFORE_reached_assignment: now/run_tick=36680/28654ms，elapsed=8026ms，common_age=105ms；common mean/ref=2102.968/2082.637mm，差+20.330mm；ref F/follow=15.759/-0.196RPM，pending/applied=16/18RPM；profile_done=False，原profile还余0.752663s。
- 冻结到位2102.967mm；最终PE原始编码器校准2113.098mm，比冻结多10.131mm；PE结束采样在首STOP ACK后[1034, 1067]ms。尺量2107.5mm、右20.0mm。上述量不同点/时刻，PE差不能全称物理滑行。
- 末OD along/right=2113/2mm，sample在首STOP后341ms；中段仅4点、abs峰4mm。最后完整ACK批F/L/C=20.0/0.3/0.1RPM，STOP ACK跨度33ms。
- HWT运动期采样右峰1.126°；STOP前最近样本相对起点左-0.093°（距首ACK 55ms），最终左+0.455°，追加左转0.548°。这不是横向地面越线。
- XD 925/925、drop0；telemetry queue drop6另列；VM/PE2=1、backend inactive、stop mask15、故障0，powered tail 20.047s、完成后TX0。

### diagnostic-run2-retry1
- first_tail_entry: now/run_tick=16280/9034ms，elapsed=7246ms，common_age=106ms；common mean/ref=1989.859/1991.382mm，差-1.522mm；ref F/follow=41.039/+0.891RPM，pending/applied=42/44RPM；profile_done=False，原profile还余1.532663s。
- latest_tail_batch_freeze_NOT_complete_ACK: now/run_tick=17064/9034ms，elapsed=8030ms，common_age=142ms；common mean/ref=2098.707/2082.896mm，差+15.812mm；ref F/follow=15.629/-0.149RPM，pending/applied=15/15RPM；profile_done=False，原profile还余0.748663s。
- arrival_BEFORE_reached_assignment: now/run_tick=17080/9034ms，elapsed=8046ms，common_age=114ms；common mean/ref=2102.757/2083.909mm，差+18.848mm；ref F/follow=15.110/-0.162RPM，pending/applied=15/15RPM；profile_done=False，原profile还余0.732663s。
- 冻结到位2102.756mm；最终PE原始编码器校准2113.434mm，比冻结多10.678mm；PE结束采样在首STOP ACK后[1034, 1067]ms。尺量2110.0mm、右30.0mm。上述量不同点/时刻，PE差不能全称物理滑行。
- 末OD along/right=2113/2mm，sample在首STOP后946ms；中段仅3点、abs峰4mm。最后完整ACK批F/L/C=18.0/0.2/-0.6RPM，STOP ACK跨度33ms。
- HWT运动期采样右峰1.405°；STOP前最近样本相对起点左+0.302°（距首ACK 49ms），最终左+0.692°，追加左转0.390°。这不是横向地面越线。
- XD 934/934、drop0；telemetry queue drop7另列；VM/PE2=1、backend inactive、stop mask15、故障0，powered tail 20.000s、完成后TX0。

到位时common mean已领先当时ref约20mm，而原profile尚未结束、ref F仍约15RPM；本次真实状态支持“参考尚未归零就触发到位”的机制。不能据此直接改距离比例，也不保证降低floor会改变到位命令或地面超距。

## 历史组描述性对比

|组|尺量右偏mm|均值/绝对均值mm|超距均值mm|
|---|---|---|---:|
|half|3.5/-0.5/10.0/7.5|5.125/5.375|2.125|
|S3|5.0/7.0/13.5/12.5|9.500/9.500|5.625|
|S2.5|8.0/10.0/7.5/10.0|8.875/8.875|7.500|
|本次diag|20/30|25.000/25.000|8.750|

这些是不同批次的实测端点，不是配对因果增益；half当时较好可作为用户选择恢复的依据，不证明half在所有工况最优。真实中段地面、测点/参考关系、轮地滑移及响应仍未知。

## 失败边界（不计入精度）

- diagnostic-run2：telemetry_session_changed；保存只读RCC观察USB超时，没有寄存器证据，不能确定重启原因。
- diagnostic-run3：USBTimeoutError，reset_attempted=false/reset=false，未成功执行复位，不是一趟精度样本。
- diagnostic-run3-retry1、diagnostic-run3-retry2：分别ack_timeout；保留首失败，不能与两趟正常完成混算精度。用户没有说明重启原因，当前已暂停重试。

已核R RUN-01/PWR-01；E LOC-01/02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04仍待技术答疑。恢复half和新增停止测距由父任务另行实现与验证，本目录仅数据分析，不修改固件或其他证据。
