"""Cross-check saved MATLAB replay and write bounded conclusions."""
from pathlib import Path
import json,hashlib
import numpy as np
O=Path(__file__).resolve().parent;R=O.parents[4]
def read(p):return json.loads(p.read_text('utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
C=read(O/'contexts.json');S=read(O/'summary.json');P=read(O/'parameters.json');assert S['status']=='COMPLETED_WITH_LIMITATIONS' and len(S['runs'])==22
for s,h in read(O/'input_sha256.json').items():assert sha(R/s)==h,s
for s,h in read(O/'original_dirty_sha256.json').items():assert sha(R/s)==h,s
errmax=0;frozenmax=0;causal_samples=0
for c,r in zip(C,S['runs']):
 d=read(Path(c['decoded']));tr=d['header']['trigger_ms'];ep=c['endpoint'];trace=np.genfromtxt(O/(c['name']+'_replay.csv'),delimiter=',',names=True)
 proj={'forward':[1,1,1,1],'backward':[-1,-1,-1,-1],'left':[1,-1,-1,1],'right':[-1,1,1,-1]}[c['direction']];scale=P['forward_scale'] if c['direction'] in ['forward','backward'] else 1
 w=[]
 for j in range(4):
  vals={}
  for a in d['wheel']:
   if a['source']=='position' and a['valid'] and a['wheel']==j:vals.setdefault(a['source_ms'],(a['observed_ms'],a['value']*(1-2*a['sign'])))
  base=ep[f'start_raw{j+1}']*(1-2*ep[f'start_sign{j+1}']);ts=np.array(sorted(vals));w.append(((ts-tr)/1000,np.array([vals[t][0]-tr for t in ts])/1000,(np.array([vals[t][1] for t in ts])-base)*P['motor_forward_sign'][j]*proj[j]*np.pi*77/65536*scale))
 profile=np.genfromtxt(c['profile_trace'],delimiter=',');pv={t:x for t,x in profile[:,:2]};pt=np.array(sorted(pv));px=np.array([pv[t] for t in pt])
 # Independent NumPy check of all estimator events, using only received prefixes.
 for row in trace:
  now=float(row['receipt_event_s']);prefix=[(t[a<=now+1e-10],x[a<=now+1e-10]) for t,a,x in w];ct=min(t[-1] for t,x in prefix)
  assert abs(ct-row['common_source_s'])<1e-10
  stale=sum(np.interp(ct,t,x) for t,x in prefix)/4
  proxy=sum(np.interp(now,t,x) for t,a,x in w)/4
  errors=[abs(stale-row['causal_stale_progress_mm']),abs(proxy-row['retrospective_body_proxy_mm'])]
  for offset,key in zip([-.055,0,.055],['candidate_offset_minus55ms_mm','candidate_offset_zero_mm','candidate_offset_plus55ms_mm']):
   pred=stale+np.interp(now-offset,pt,px)-np.interp(ct-offset,pt,px);errors.append(abs(pred-row[key]))
  errmax=max(errmax,*errors);causal_samples+=1
 frozenmax=max(frozenmax,abs(trace['causal_stale_progress_mm'][-1]-c['frozen_progress_mm']))
assert errmax<1e-7 and frozenmax<.01
V=dict(status='PASS',matlab_primary_version=S['matlab_version'],trials=22,causal_prefix_events_verified=causal_samples,matlab_numpy_max_error_mm=errmax,frozen_progress_max_error_mm=frozenmax,input_hashes_verified=len(read(O/'input_sha256.json')),original_dirty_files_unchanged=6,hardware_access=False,closed_loop_claim=False,ground_accuracy_claim=False)
(O/'verification.json').write_text(json.dumps(V,indent=2),encoding='utf-8')
lines=['# 到达时序离线可行性与转向测试接续','', '日期2026-11-17；本阶段源基线97a0a704。MATLAB R2020a实际运行；只分析，不修改固件/参数、不连接硬件、不fetch/push。','',
'## 用户更正与结论边界','',
'用户在上一阶段之后明确补充：部分早期测试起始摆放不够精准。尚未逐趟标明哪些数据受影响，因此原尺量原话、数值及更正链保持，只作带未知起始误差的观测；上一报告的尺量均值不能用来认定车辆固定超距、滑移量、精度或拟合比例。该说明优先于此前“摆位条件足够一致”的推断，不追溯篡改原始测量。', '',
'109–164ms的共同源龄期及冻结progress重现不依赖摆位，可以保留。此阶段仅评价补偿估计对“事后轮位置同窗模型”的接近程度，未使用尺量拟合，也不把模型当地面真值。', '',
'## MATLAB方法与结果','',
'在每个已记录轮位置接收事件，仅使用此前已收到的四轮位置，按最老共同源时间重建progress。候选沿用当前速度跟随分支已存在的形式：p_est = p_common + r(t_now) - r(t_common)，将此形式用于到达判断的可行性分析。r是已验证S曲线参考；未把新形式接入产品代码。', '',
'参考曲线起点与真实s_run_tick没有直接日志绑定：capture在观察到START时触发，s_run_tick在START发送分支首次赋值，两者不能默认相等。本步采用-55/0/+55ms作为工程敏感性假设，不是已测误差上界。参考CSV边界重复时间取末值后再插值。', '',
'事后参照为同一四轮坐标在当前接收事件时间的源内插值；它可能需要后续收到的位置，所以只供离线评分，绝不进入因果候选计算。范围限制在实际STOP之前，尾段为事后主距>=目标-120mm；这不是地面绝对位置，也未建立改变STOP后的闭环轨迹。', '',
'22趟的尾段平均绝对误差：原旧共同源progress约12.62–18.34mm，零偏移候选约1.68–2.19mm；逐趟最大绝对误差约2.53–3.94mm。候选在保存轨迹上的首次目标交叉一般比实际STOP观察早87–164ms，此时事后轮模型仍超过目标约1.73–6.54mm。不能据此说能精确停在目标，也不能把这些毫秒/毫米直接作为固定提前量。', '',
'现有右150地面尺量原本接近1500mm，且摆位精度尚不明确；若直接按轮模型补偿提前停车，仍可能实际欠距。因此本阶段结论为“离线估计可改善，尚不可部署为实距补偿”。平移参数、原停止能力及130RPM边界保持。', '',
'|趟次|原尾段MAE mm|候选尾段MAE mm|候选最大绝对误差 mm|候选交叉早于STOP ms|交叉时轮模型−目标 mm|','|---|---:|---:|---:|---:|---:|']
for r in S['runs']:
 c=r['candidate_crossings'][1];lines.append(f"|{r['name']}|{r['stale_tail_bias_mae_maxabs_mm'][1]:.3f}|{r['candidate_tail_mae_mm'][1]:.3f}|{r['candidate_tail_maxabs_mm'][1]:.3f}|{1000*(r['actual_STOP_s']-c['event_s']):.0f}|{c['proxy_minus_target_mm']:.3f}|")
lines += ['', 'NumPy独立按接收前缀重算全部事件，仅作MATLAB数值/因果交叉检查，不替代MATLAB前置验证。verification.json保存事件数、差值、58项输入哈希与原6个dirty文件保持结果。该检查不是独立agent审查或实车验收。', '',
'## 转向测试什么时候做','',
'用户明确允许继续离线工作，并询问转向测试安排。本次时序分析完成后，下一软件阶段直接准备转向，不要求先把四向距离全部调准。当前仍无硬件执行授权。', '',
'- 当前组合版6b4c42de的AppDebugPlan_Load（app_entry.c:34–36）以及串口ARM白名单（app_serial_runtime.c:264–269）仅允许PLAN8..19平移，旧PLAN0被拒绝；不能直接发送旧PLAN0来试转向。',
'- 已有app_heading_runtime/HeadingSession及正反30度历史software_complete记录可复用为软件设计参考；历史2026-10-30回执不证明当前版本转向通过，不照搬旧整套AXF或恢复旧PLAN0多动作。',
'- 下一阶段先准备一个明确角度、仅一次动作的30度转向候选，复用既有航向执行器，保留HWT新鲜性、STOP、故障与互锁；先MATLAB适用验证、实际C门禁/构建并本地commit，再通知用户准备现场。不要附加平移补偿到同一候选。',
'- 实测先一个方向单次30度；记录起始、目标、首次到位、停止后角度及是否有平移。此趟完成并保存后，另获摆位授权再反向；30度重复性明确后再90度，不自动往返或循环。具体方向/速度由当前实现与准备结果确定，尚未填定或执行。',
'- 现场前核实际无线链路和版本，明确旋转扫掠空间、机构收拢/载物与可断电；安装与发车分别取得授权。当前用户无需摆车，也不沿用历史供电/场地状态。', '',
'## 规则与未闭环','',
'R已核RUN-01/PWR-01/MECH-01/02；E已核LOC-02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04继续待技术答疑确认，无新官方资料或规则变更。所有候选及测试顺序均为E。', '',
'未闭环：哪些历史尺量受摆位影响、真实s_run_tick偏差、同步地面位置/测量不确定度、补偿后的真实闭环及轮地面失配、短距重复性、使能非零反馈、转向当前入口与当前版本实车角度/中心偏移、IMU融合与比赛通用入口。', '',
'复现：仓库根运行本目录prepare_inputs.py；本机MATLAB添加本目录后调用replay_arrival_timing；最后运行verify_and_report.py。本次存档original_dirty_sha256.json为开始时6项保护基线。', '']
(O/'REPORT.md').write_text('\n'.join(lines),encoding='utf-8')
print(json.dumps(V,ensure_ascii=False))
