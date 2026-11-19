# 当前检测器两轴命令响应J候选

仅本地候选 `output/stage2_response_j_candidate_20261117/body-profile.json`，SHA256 `3aee1fdd6d365e725d7558e7405ba5c58c28c1c58616db2eed1b7b353d0db700`。只改response_px_per_command_mm，原profile_id、reference_center/reference_radius、HSV、300/5/2门控、5mm停止范数、20mm步长及physical_auto_correction_passed=false保持。当前Pi仍原J的preview，候选未部署；未替换实际STM32工程或下载固件。

新J为 `[[-0.0306588202780901,4.40558678007313],[3.78918614505116,0.148763313234528]]`，列依次right/forward，单位为像素/命令毫米。输入为本轮reset后54条有效TX构成53个左右转移，以及用户批准的两次前移10mm，不把命令距离或编码器当独立地面尺量。三点均用Pi本次SSH接收时刻作外部围栏，300ms之后同worker严格新seq的5帧，原始盘心最大两两距离≤2px，不复用同pose旧ready。

A=[957.779240,422.036789]、B=[1005.528820,419.442016]、C=[1045.890976,425.012055]px，散布分别0.64899/1.65019/0.19942px。两次真实前移均由主代理唯一probe调用既有SBS request2/dir0/10mm；实际seq1/session1、seq2/session2，SBS3/MotionDONE/backend1/STOP15后才采B/C，用户确认正常无碰擦。此前工具缺静态ELF符号的失败发生在写请求前，writes=[]，不计作运动。

MATLAB R2020a实际07:38:20及补充输出07:38:59退出0通过，之后才生成配置。55转移输入rank2，cond(J)=1.16584；前移互留出命令等效残差2.56015/3.02403mm，左右逐转移留一最大1.49200/RMSE0.75586mm。左右相邻转移共享中心噪声，非独立场景泛化；前移仅两样本，不能宣称2mm实物精度。既有5mm像素标度停止范数与留出命令等效毫米是不同量，报告中的≤5只为本轮离线工程筛选，不新增产品保护。补充向量数组未赋值的零值已从结果移除，不用于结论。

以拟合J作理想plant，从A/B/C起点回放：旧J30步仍约10.62～12.36像素标度mm，新J模型1～2步进入原5mm范数。该模型使用拟合plant，不能证明实车收敛、地面精度、reference物理对准或三件抓放。固定黄十字的来源仍是原reference_center，未随本次检测更新。

证据位于 `evidence/integration/2026-11-17/stage2-settle-ring/`：`pi-newrun-all-validtx.json`、`forward2-point-A/B/C.json`、`forward10-20261117-073311.json`、`forward10-20261117-073529.json`、`response-twoaxis-input.json`、`response-twoaxis-matlab-result.json`及log。验证脚本 `tmp/fit_stage2_response_20261117.m`。默认本地检查的单次工具 `scripts/stage2_forward10_once_18cc_20261117.py`只供主代理独占probe，不自动连续执行。

R核SRC-01/02、RUN-01/02、PWR-01、MECH-01/02；E核VIS-15、CAL-02/05、RUN-06。J为工程标定候选，规则R不改，官方Q继续待技术答疑确认。待高档审查后由主代理按已授权范围安排只J部署和保留原reference的受控新run；现场收敛、reference物理对准、三件/100ms余量与冷启持久性未闭环。仅本地，无GitHub提交推送。
