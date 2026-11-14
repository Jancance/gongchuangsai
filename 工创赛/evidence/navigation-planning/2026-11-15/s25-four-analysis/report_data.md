# S2.5 四趟数据汇总（仅数据口径）

与已实测 S3 四趟比较；不是随机配对试验，不作方案结论。单位 mm；尺量右偏为正，HWT 左转为正。

|趟|尺量前进|尺量超距|尺量右偏|冻结到位进度|进度−尺量|最终PE现系数换算|PE−尺量|PE−冻结|中段OD峰/点数|
|---|---|---|---|---|---|---|---|---|---|
|1|2105.0|5.0|8.0|2104.655|-0.345|2114.703|9.703|10.048|4/4|
|2|2107.5|7.5|10.0|2101.789|-5.711|2114.395|6.895|12.606|4/3|
|3|2105.0|5.0|7.5|2102.141|-2.859|2113.073|8.073|10.932|4/3|
|4|2112.5|12.5|10.0|2102.677|-9.823|2113.541|1.041|10.864|5/3|

|口径|S3 已实测四趟|S2.5 本轮四趟|
|---|---|---|
|ruler_right_mm|[5.0, 7.0, 13.5, 12.5]；均值 9.5000；MAE 9.5000|[8.0, 10.0, 7.5, 10.0]；均值 8.8750；MAE 8.8750|
|ruler_forward_error_mm|[5.0, 0.0, 10.0, 7.5]；均值 5.6250；MAE 5.6250|[5.0, 7.5, 5.0, 12.5]；均值 7.5000；MAE 7.5000|
|middle_OD_peak_mm|[5, 4, 4, 5]；均值 4.5000；MAE 4.5000|[4.0, 4.0, 4.0, 5.0]；均值 4.2500；MAE 4.2500|
|software_minus_ruler_mm|[-2.206000000000131, 0.04300000000012005, -7.282999999999902, -6.5909999999998945]；均值 -4.0092；MAE 4.0308|[-0.3449999999997999, -5.710999999999785, -2.8589999999999236, -9.822999999999865]；均值 -4.6845；MAE 4.6845|
|final_encoder_minus_ruler_mm|[8.972913273296854, 14.080406777605276, 2.8206583411533757, 5.1754478177904275]；均值 7.7624；MAE 7.7624|[9.702737592019275, 6.89534388671791, 8.073362368825201, 1.0410534051125069]；均值 6.4281；MAE 6.4281|
|final_encoder_minus_frozen_progress_mm|[11.178913273296985, 14.037406777605156, 10.103658341153277, 11.766447817790322]；均值 11.7716；MAE 11.7716|[10.047737592019075, 12.606343886717696, 10.932362368825125, 10.864053405112372]；均值 11.1126；MAE 11.1126|
|HWT_early_1p3_2s_left_deg|[1.5809999999999889, 1.2309999999999945, 1.0879999999999939, 1.3830000000000098]；均值 1.3207；MAE 1.3207|[0.9989999999999952, 1.054000000000002, 0.9499999999999886, 1.0]；均值 1.0007；MAE 1.0007|
|HWT_change_pre_to_early_left_deg|[1.3629999999999995, 0.929000000000002, 0.7690000000000055, 1.1040000000000134]；均值 1.0413；MAE 1.0413|[0.6260000000000048, 0.6750000000000114, 0.6970000000000027, 0.46700000000001296]；均值 0.6163；MAE 0.6163|

## 时基、采样与动作核验

- 第1趟：首STOP ACK=30685 ms，四轮ACK跨度33 ms；PE末样本相对首ACK=[1034, 1067] ms。末个有效preSTOP OD：along=2016、right=2，样本早于ACK 781 ms。最终有效OD 2114/1的样本在ACK后 319 ms。
  - 最后完整批F/L/C=19.0/0.1/0.1 RPM，末ACK距STOP 55 ms；L/C峰=2.1/1.8 RPM，L30/C20 cap批=0/0（按中段OD回溯关联的cap=0/0）。F是解混合后的纯前进分量；尾段L=C=0的实际批数=0，不能把解混合F等同实际四轮纯前进批。最后8批与峰值批逐时刻在summary.json。
  - HWT起始绝对yaw=0.214°；停前相对左转=0.373°；停后1.3..2s中位左转=0.999°，相对停前又变化0.626°。最后1s左转均值=0.999°、跨度=0.000°；不替代早期窗口。+.1/.3/1/2s最近后继新鲜样本见summary.json。
  - XD=932/932，XD drop=0；TM队列累计drop=7（不同通道）；尾窗200帧、最后有效elapsed=20.015s；完成后TX=0。排除的末尾不完整片段=[]。
  - 中段OD右偏范围[3,4]，保存点两侧越零=False；这只描述已采样点，不排除采样间真实越零。preSTOP连续保存窗(along,right)=[(618, 4), (1016, 4), (1414, 4), (1785, 3), (2016, 2)]；窗尾以前的记录完整保留，缺失停车前亚秒段不插值。
- 第2趟：首STOP ACK=31757 ms，四轮ACK跨度33 ms；PE末样本相对首ACK=[1034, 1067] ms。末个有效preSTOP OD：along=2021、right=2，样本早于ACK 748 ms。最终有效OD 2114/1的样本在ACK后 264 ms。
  - 最后完整批F/L/C=20.0/0.2/-0.2 RPM，末ACK距STOP 77 ms；L/C峰=2.3/1.7 RPM，L30/C20 cap批=0/0（按中段OD回溯关联的cap=0/0）。F是解混合后的纯前进分量；尾段L=C=0的实际批数=0，不能把解混合F等同实际四轮纯前进批。最后8批与峰值批逐时刻在summary.json。
  - HWT起始绝对yaw=0.670°；停前相对左转=0.379°；停后1.3..2s中位左转=1.054°，相对停前又变化0.675°。最后1s左转均值=0.992°、跨度=0.006°；不替代早期窗口。+.1/.3/1/2s最近后继新鲜样本见summary.json。
  - XD=930/930，XD drop=0；TM队列累计drop=7（不同通道）；尾窗200帧、最后有效elapsed=20.063s；完成后TX=0。排除的末尾不完整片段=[]。
  - 中段OD右偏范围[4,4]，保存点两侧越零=False；这只描述已采样点，不排除采样间真实越零。preSTOP连续保存窗(along,right)=[(966, 4), (1379, 4), (1765, 4), (2021, 2)]；窗尾以前的记录完整保留，缺失停车前亚秒段不插值。
- 第3趟：首STOP ACK=31885 ms，四轮ACK跨度33 ms；PE末样本相对首ACK=[1034, 1067] ms。末个有效preSTOP OD：along=1998、right=2，样本早于ACK 858 ms。最终有效OD 2114/1的样本在ACK后 154 ms。
  - 最后完整批F/L/C=20.0/0.1/0.0 RPM，末ACK距STOP 55 ms；L/C峰=2.2/1.6 RPM，L30/C20 cap批=0/0（按中段OD回溯关联的cap=0/0）。F是解混合后的纯前进分量；尾段L=C=0的实际批数=0，不能把解混合F等同实际四轮纯前进批。最后8批与峰值批逐时刻在summary.json。
  - HWT起始绝对yaw=-0.335°；停前相对左转=0.253°；停后1.3..2s中位左转=0.950°，相对停前又变化0.697°。最后1s左转均值=0.950°、跨度=0.000°；不替代早期窗口。+.1/.3/1/2s最近后继新鲜样本见summary.json。
  - XD=928/928，XD drop=0；TM队列累计drop=7（不同通道）；尾窗200帧、最后有效elapsed=20.047s；完成后TX=0。排除的末尾不完整片段=[]。
  - 中段OD右偏范围[4,4]，保存点两侧越零=False；这只描述已采样点，不排除采样间真实越零。preSTOP连续保存窗(along,right)=[(917, 4), (1343, 4), (1728, 4), (1998, 2)]；窗尾以前的记录完整保留，缺失停车前亚秒段不插值。
- 第4趟：首STOP ACK=35167 ms，四轮ACK跨度33 ms；PE末样本相对首ACK=[1034, 1067] ms。末个有效preSTOP OD：along=2096、right=1，样本早于ACK 198 ms。最终有效OD 2113/1的样本在ACK后 858 ms。
  - 最后完整批F/L/C=18.0/0.1/-0.4 RPM，末ACK距STOP 33 ms；L/C峰=3.0/2.0 RPM，L30/C20 cap批=1/2（按中段OD回溯关联的cap=1/0）。F是解混合后的纯前进分量；尾段L=C=0的实际批数=0，不能把解混合F等同实际四轮纯前进批。最后8批与峰值批逐时刻在summary.json。
  - HWT起始绝对yaw=0.060°；停前相对左转=0.533°；停后1.3..2s中位左转=1.000°，相对停前又变化0.467°。最后1s左转均值=0.955°、跨度=0.006°；不替代早期窗口。+.1/.3/1/2s最近后继新鲜样本见summary.json。
  - XD=934/934，XD drop=0；TM队列累计drop=6（不同通道）；尾窗199帧、最后有效elapsed=19.953s；完成后TX=0。排除的末尾不完整片段=[{'tag': 'TMQ', 'bytes': 138}]。
  - 中段OD右偏范围[3,5]，保存点两侧越零=False；这只描述已采样点，不排除采样间真实越零。preSTOP连续保存窗(along,right)=[(827, 5), (1244, 4), (1635, 3), (1929, 3), (2096, 1)]；窗尾以前的记录完整保留，缺失停车前亚秒段不插值。

## 边界与未闭环

- Raw decoder/CRC evidence is reused from analyze_once/capture_check only after raw and journal SHA256 verification. PE CRC and every endpoint field are additionally checked directly against the raw frame. This script does not re-decode every raw frame.
- Ruler endpoint is operator ground measurement after stopping; unknown ruler reference offset and reset heading alignment prevent rigid causal/scale conclusions.
- Frozen software arrival progress is wheel distance at arrival, not final stopped wheel distance. First-seen telemetry time is not exact internal latch time.
- PE is signed raw count difference averaged with logical wheel directions, multiplied by pi*77/65536 and the existing 1.02182540 forward scale; double-precision reconstruction is not bitwise firmware float replay.
- PE four-wheel start/end samples are asynchronous. PE end is after STOP ACK and includes sample lag and braking evolution; PE-minus-progress is not pure ground coasting.
- OD is sparse integer wheel odometry, not independent ground motion. No negative saved sample does not exclude real intersample crossing or overshoot.
- preSTOP OD window ends with sample and packet before first STOP ACK; missing final subsecond is not silently interpolated.
- Complete ACK batches are commanded wheel targets, not simultaneous wheel/body measured speed. Middle association uses most recent common sample retrospectively and is descriptive, not online freshness proof.
- STOP markers are first/last motor acknowledgement timestamps, not physical stationary instants. HWT fresh telemetry uses live yaw, not frozen controller error.
- Early HWT 1.3..2s is primary endpoint comparison; later tail and final 1s are separate, so late handling cannot improve early stop yaw.
- For a front ruler point d mm forward from the pose reference, right-coordinate rotational contribution is -d*sin(delta_left) if initial axes coincide and lateral lever arm is zero. Unknown d/axes/reference mean no numeric correction is applied; general rigid transform requires both lever-arm components.
- Four runs per group are descriptive, not randomized paired trials; small differences cannot identify a unique cause or justify fitting a new distance coefficient.
- No new firmware, MATLAB model, download, reset, hardware access, network or further run is performed/authorized by this analysis.

R已核 FIELD-06/RUN-01/PWR-01；E已核 LOC-02/LOC-07/MOT-02/MOT-04/RUN-06。R/Q不修改；正式答疑未明项继续待技术答疑确认。
复算：`python 工创赛/evidence/navigation-planning/2026-11-15/s25-four-analysis/analyze_four.py`。输入SHA256/大小见input_manifest.json；脚本写入仅限本目录。
