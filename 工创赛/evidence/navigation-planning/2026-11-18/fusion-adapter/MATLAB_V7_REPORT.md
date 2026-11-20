# 适配器 MATLAB 前置 v7 回执

日期：2026-11-18。只完成 pure copied-value 模型，尚未写产品 C/H。

- 实际本机 `E:/Program Files/Polyspace/R2020a/bin/matlab.exe -batch`；MATLAB `9.8.0.1323502 (R2020a)`。
- 开始 `2026-11-18T01:30:54.384328+08:00`，结束 `2026-11-18T01:31:01.369345+08:00`，exit 0。
- `verify_fusion_adapter_v7.m` SHA256 `2890ad36920818ee3bb222eac51dbb0d28f7460f097d629d12c9e6650fd1170d`。
- 739 个计数检查、120 行可复放估速向量。分组计数保存于 `matlab_summary_v7.json`：M1=37、M2=12、M3=71、M4=26、M5=35、M6=22、M7=536，总和739。
- 向量 `adapter_vectors_v7.csv` 列为 dt_ms,direction_deg,speed_mm_s,correction_mm,previous_map_heading,current_map_heading,vx,vy,omega,corrected_x,corrected_y。SHA256 `31e85fe56dd8e6c2501bfe6241a977c0613a84ac619155b00cc969976a82ee8b`。
- `matlab_receipt_v7.json` 记录命令、时刻、脚本/输出hash、前后产品C/H都不存在；没有硬件访问。

| 组 | 前置模型检查 |
|---|---|
| M1 | caller六身份、map三实际身份逐错；计划外壳/身份域/max_age/stop阈值错误；合法反向plan与signature新绑定prime；固定20..100ms不缩窗 |
| M2 | 正常回绕、sample0、begin/source/stop跨零、未来/旧样本/半周期、旧sample+低updates不倒退 |
| M3 | based/TRACKING/valid/reasons/continuity/local_origin、common age100/101、四轮各399/400与未来/早于begin、angle99/100；当前now重验，失效清窗后两个新样本恢复 |
| M4 | WAIT/ready重复、五核心数值冲突、每类metadata自然变化、同sample updates回退、短窗后冲突和旧样本、begin变化；缓存Observation除now保持 |
| M5 | 1/19/20/100/101ms、短窗累积且保anchor、长gap重prime、7浮点×NaN/±Inf、分量±10000和角速±3600边界/越界、巨大raw差分；不增加合速度10000门禁 |
| M6 | stop缺失/错run/未来/早于begin/过期、age100/101、cached0新stop pending；cached1撤销或当前证据无效整体拒绝、原stop过期不能新timestamp续命；valid stop且速度400并不estimated_stationary |
| M7 | 120组四向/斜向、map heading offset与正反跨180；raw匀速且correction跳变时仅位置包含overlay；原四向覆盖与BOUNDARY/UNSUPPORTED/TAIL审计；量化扫描；实际非零减速度raw积分轨迹经observe得到均速滞后 |

模型 `FusionObservation` 有效输出明确ESTIMATED。estimated_stationary只作估计报告，不自动生成stopped。raw含既有基础标定、不是地面真值；位置直接复制corrected map XY，不二次补偿；地图heading直接使用，不重复场地offset。layout/path/direction由caller绑定，不冒充map/雷达回显。map覆盖只按源报告，不签mixed校准。

历史保留：初版521及其脚本/回执/向量未改。v2/v3/v4失败脚本、日志、receipt均保留；失败分别是短窗夹具额外等计数限制、源时间早于begin的清窗优先级、回绕夹具错误地从now取偏移。v5(635)与v6(714)通过证据也保留。首次sandbox打开stage日志PermissionError在 `v2_launch_sandbox_error.txt`；之后仅将已授权stage MATLAB/证据命令以明确权限执行，无硬件命令。

此报告不等于真实C或Reference联合验收。C1必须在产品编写后另做，NULL/逐字状态不变/输入不变/实际Reference推进与DONE由ARM验收；本机MATLAB不模拟非法指针或RTOS。root/high审查后方可放行产品C。

核对R FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04/07、PWR-01、MECH-01/02；相关E LOC-02/03/05/06/07、MOT-01/02/03/04、CAL-02。Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。无runtime caller、真实速度发布、雷达bundle、A/J执行、下载或物理停稳证明。
