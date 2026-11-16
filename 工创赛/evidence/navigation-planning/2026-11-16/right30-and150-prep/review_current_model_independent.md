# 右30及150候选当前模型独立审阅

结论：**PASS_WITH_LIMITATIONS（源码增量、MATLAB、ARM局部检查、host内存协议、完整构建与离线ELF范围）**。本报告由当前模型独立子agent完成，指定6.1-sol服务403后用户允许当前模型接替；不能称为6.1-sol high审阅。已补核主任务完成后的实际回执与新旧镜像SHA；不是新固件下载或实车验收。

## 已核对

1. 相对本地checkpoint `607368de0ce5eeb087f675b9183fbee4cb761f35`逐段查看5个产品文件diff，仅追加PLAN13；`app_entry.h`将最大ID提至13，旧8前100/2100、9左102/2100、10右102/2100、11后100/2100、12右102/1500保持。13仅右mode2/kind6、base102、300mm；`MotionTrial40Allowed`对short距离1500或300均要求direction2与RPM102，PLAN层再锁每个ID自身长度，不能跨PLAN冒用。
2. `app_serial_runtime.c`精确`strcmp(PLAN13)`解析，与10/11/12并列；ARM白名单、BF1的RPM与距离回显、成功PoweredDone、反馈限值130、idle动作kind及reference ceiling40000均含13。`app_motion_runtime.c`正常完成VM保持枚举包含13，未削弱session/VM/完成/故障条件。BF1应为`[13,102,300,400,150,300,130,12779]`；400仍明确为参考上限。
3. 新client是独立文件，显式`BASELINE40_SPECS[13]=(102,6,300)`，保留12=(102,6,1500)，未回到`plan−4`误判。`_speed_profile_healthy`含13并要求匹配新鲜BF1、RPM、kind、单动作、130反馈和40000参考；已选13时不能以8或12的BF1授权ARM。启动前允许先看到默认8仅用于选PLAN逻辑，不能略过所选13新鲜回显。
4. 独立脚本`independent_review_checks.py`实际exit0、91项PASS：6个PLAN各与6个BF1配置交叉验证、6类kind/base/fb/count/ref/age拒绝、6个合法PLAN命令、7个畸形PLAN拒绝以及5个控制/参数源保持。全程仅内存构造，无COM/SWD/Serial对象、无真实TX/START。回执`independent_review_checks.json`由工具保存实际stdout；脚本初次比较被Git LF/工作树CRLF差异触发，修正为仅归一化行尾后比较；之后Python直接写JSON被PermissionError，改stdout后通过，未影响产品。没有把失败当产品缺陷或放松实际行为期望。
5. `motion_profile.c`、`chassis_auto_test.c`、`chassis_kinematics.c`、`app_config.h`、`heading_controller.c`相对checkpoint内容完全保持（仅比较时归一化Git行尾）。raw工作树SHA256见独立回执；curve和chassis_auto的raw SHA也与本次写前`interface_source_identity.json`一致。故没有新增右30控制器，没有改方向比例/航向PD/左2%/前专用横偏/原STOP保护。
6. 方向参数逐源审查另见`direction_parameter_matrix.md`：前横偏仅PLAN8+mode0，左2%仅PLAN9+mode1；right13和12均复用右方向既有factor、A/J和通用S。后退沿用纵向1.0218254，不能称后退独立标定。普通/BASELINE40/旧route左trim不同条件已分清，通用数学curve存在不代表比赛motion/runtime任意方向/速度/距离已泛化。
7. 核读真实MATLAB回执：R2020a启动21:38:50、21:38:59 exit0，早于产品写21:42:56。其300结果peak167.93089480786773mm/s、duration3.5728982489283405s、tv0，与本审查独立标量解析一致；float32分支peak167.9309082、duration3.572898388。1500仍peak400、duration7.269949022s、tv.230050978s，2100仍8.769949022s。没有用Python替代MATLAB，也不把参考峰值当实测。
8. 已读`check_pair_c.py`与`actual_c_checks.json`：真实ARM编译/Unicorn局部断言trimON10998、OFF10998、inactive3863全部PASS。测试含PLAN8..13方向/RPM/长度正反矩阵、畸形PLAN、实际`MotionProfile_Prepare/Sample`对300/1500/2100的峰速/单调/约束/末态，以及右向四轮投影与限幅。沿用测试stub/仿真外设，不是RTOS或电机闭环实物通过，不把它称为真实硬件C运行。
9. 已读`check_pair_host.py`与`host_checks.json`：120项PASS，12/13各一条独立内存PLAN→ARM→唯一模拟START→DONE链，包含跨PLAN错BF1、wrong-kind/wrong-distance拒绝；serial_opened=false、TX_bytes=0。没有批量发车或双动作联跑入口。独审脚本在client帮助文案补齐12/13后重跑91项PASS，最终以`independent_review_checks_post_help.json`为准。当前client SHA256=`055159d85decc8bef8f9ef139396b1f9d4caba6cfbcef68eda0ddfffe6b37c0f`；先前`independent_review_checks.json`保留首次e272版本历史，不冒充最终身份。帮助文字问题已修，无当前残留误报。
10. `source_scope_check.json`检查1931输入，仅上述5个产品文件新增13，其余1926保持，5文件GBK回环/CRLF均保持；`product_diff.txt`与本审查直接`git diff 607368de`一致。未重跑无关整套旧suite，也未改正常保护、采样/时戳或旧方向标定。
11. 已读最终`build_receipt.json`及`build.log`：派生工程`工创赛/tmp/right30-and150-build-20261116/right30_150_40.uvprojx`、目标Vehicle_Integrated，于21:53:52启动/21:54:00返回0，实际日志0 Error(s)/0 Warning(s)。定义保留BASELINE40=1/LEFT40_TRIM=1/capture=1/static=0，显式VM_ONLY/VM_CALIBRATION_ONLY/WHEEL_ENABLE_HOLD/INTEGRATED/STAGE1为0。该派生工程仍绑定日常工程活动源，是本次测试构建入口，不另立日常产品工程。
12. 独立`Get-FileHash`核得normal AXF与`right30-and150-prep/frozen/Vehicle_Integrated.axf`均为`3f8fe3bf732637ebe1e9765b5ebf686c7042d536bcfd1898dc163033d260b3cd`；`offline_elf_check.json`为offline_elf_only，无硬件，Flash PT_LOAD0大小116424，capture A34020/B33600及tail176B等符号在文件中存在。ELF文件检查不能替代烧录/读回。
13. 独立精确路径核得旧150冻结`right-short-distance-prep/frozen/Vehicle_Integrated.axf`仍`38a7a344c0763abbf76570270d2fa3cf9c363d653c2fc4f59cd72bb01d4f3fd3`；旧210现场镜像`工创赛/tmp/right-reverse40-build-20261116/right_reverse210_40/Vehicle_Integrated.axf`仍`821fae6a877486e6b388620142dec6b9a9c8756366439ac8218e507b68ea9288`，与旧现场execution_binding一致。新pair使用独立目录未覆盖两者。注意更早right-reverse210-40-prep冻结81fc镜像是另一身份，不能与现场821混记。

## 局限与未闭环

- 本报告签实际看到的diff/源码/MATLAB回执、ARM局部断言、120项host、91项独审client、完整构建与ELF及SHA结果；不冒称独审者重新执行了root的全部命令，不把它写成整套旧suite回归或实车接受。
- 右1500及300尚无新板身份/地面主距/旁向/轮速/载荷/重复性实车验收；300峰速低于1500，不能作为严格同速倍率唯一辨识。新的软件白名单不构成硬件发车许可，不下载/复位/START，不自动重试。
- 已核R RUN-01/PWR-01；E LOC-02/03/05/07、MOT-02/04/RUN-06；Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。当前仅本地证据审阅，无fetch/pull/commit/push，无正式入口/整场/斜向验收。
