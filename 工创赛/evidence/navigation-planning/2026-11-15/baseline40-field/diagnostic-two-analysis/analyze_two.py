"""Saved artifacts only. Write these three analysis outputs; never access hardware."""
from pathlib import Path
import csv,hashlib,importlib.util,json,math,re,statistics,sys
sys.dont_write_bytecode=True
OUT=Path(__file__).resolve().parent;FIELD=OUT.parent;ROOT=OUT.parents[5]
EXPECTED='5e803d0def9e33155752d4ceec71b9ca93ab903311407185d5d7af3c4880a185'
DURATION_S=8.7786626816
manifest={}

def use(path):
    path=Path(path);manifest[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest();return path

def read(path):return json.loads(use(path).read_text('utf8'))
def rows(path):
    with use(path).open(encoding='utf-8-sig',newline='') as f:return [{k:float(v) if v else None for k,v in r.items()} for r in csv.DictReader(f)]
def parser(path,name):
    spec=importlib.util.spec_from_file_location(name,use(path));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m.parse
def wrap(a):return (a+180)%360-180

def analyze(name):
    p=FIELD/name;c=read(p/'capture_check.json');s=read(p/'result_summary.json');m=read(p/'operator_measurement.json');r=read(p/'diagnostic_read_receipt.json')
    assert s['status']=='software_complete' and s['powered_tail_valid']
    assert c['candidate_sha256']==r['candidate_sha256']==EXPECTED
    assert r['accepted'] and r['hardware_accessed'] and r['chip_cycles_measured']
    assert r['acquisition_status']=='accepted_stable_double_read' and r['before']==r['after']
    assert not any(r[x] for x in ['reset','halted','resumed','actuator_requests_written','vm_writes','irq_mask_writes','getter_calls','automatic_retry'])
    v=r['before']['values'];assert r['before']['core']=='RUNNING' and r['before']['pe2']==1
    assert v['g_chassis_auto_state']==9 and v['g_chassis_auto_result']==1 and v['g_chassis_auto_active']==0 and v['g_vm_test_request']==1 and v['g_debug_plan_id']==8
    assert all(v[n]==0 for n in ['g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched'])
    parsed={}
    for kind,size,addr in [('timing',104,0x20012768),('tail',176,0x2001306c)]:
        blobs=[]
        for i in (1,2):
            fn=f'{kind}_read{i}.bin';blob=use(p/fn).read_bytes();assert len(blob)==size and hashlib.sha256(blob).hexdigest()==r['raw'][fn]['sha256'];blobs.append(blob)
        assert blobs[0]==blobs[1]
        symbol='g_chassis_auto_lateral_timing' if kind=='timing' else 'g_chassis_auto_tail_snapshots'
        assert r['symbols'][symbol]==[addr,size]
        parsed[kind]=PARSERS[kind](blobs[0]);saved=read(p/(kind+'_parsed.json'))
        assert parsed[kind]==saved['raw_parser_output'] and saved['trusted_acquisition']['accepted']
        assert saved['trusted_acquisition']['stable_double_read'] and saved['trusted_acquisition']['same_session']==v['g_chassis_auto_session']
    timing=parsed['timing'];tail=parsed['tail'];assert tail['session']==v['g_chassis_auto_session']
    assert timing['chip_cycles_measured'] is False and tail['hardware_provenance_established'] is False
    assert timing['enabled'] and timing['SystemCoreClock']==168000000
    assert tail['format_version']==1 and tail['valid_mask']==7 and all(x['record_valid'] for x in tail['slots'])
    for scope in timing['scopes']:
        assert scope['valid'] and scope['count']>0 and scope['window_ms']>0
        scope['sum_elapsed_us']=scope['sum_cycles']*1e6/timing['SystemCoreClock']
        scope['measured_scope_cycles_per_own_window_percent']=100*scope['sum_cycles']/(timing['SystemCoreClock']*scope['window_ms']/1000)
    for slot in tail['slots']:
        slot['profile_duration_s']=DURATION_S
        slot['profile_time_remaining_s']=DURATION_S-slot['elapsed_ms']/1000
        slot['common_mean_minus_current_reference_mm']=slot['common_mean_mm']-slot['reference_position_mm']
        slot['reference_plus_follow_rpm']=slot['reference_rpm']+slot['follow_rpm']
        slot['pending_minus_current_reference_rpm']=slot['pending_base_rpm']-slot['reference_rpm']
    raw,=(p/'serial').glob('*.raw');journal,=(p/'serial').glob('*.jsonl');fullp,=(p/'serial').glob('*.summary.json');full=read(fullp)
    assert hashlib.sha256(use(raw).read_bytes()).hexdigest()==c['raw_sha256']
    assert hashlib.sha256(use(journal).read_bytes()).hexdigest()==c['journal_sha256']
    ep,=full['endpoints'];logical=[]
    for i in range(1,5):
        start=(-1 if ep[f'start_sign{i}'] else 1)*ep[f'start_raw{i}'];end=(-1 if ep[f'end_sign{i}'] else 1)*ep[f'end_raw{i}']
        logical.append((end-start)*(1 if i in (1,3) else -1)*math.pi*77/65536)
    unscaled=statistics.mean(logical);assert abs(unscaled-ep['translation_travel_mm'])<1e-8
    pe_mm=unscaled*DISTANCE_SCALE;frozen=s['last']['distance_progress_milli_mm']/1000
    od=rows(p/'od_samples.csv');batches=rows(p/'command_batches.csv');hwt=rows(p/'hwt_stop_window.csv')
    arrival=tail['slots'][2];assert abs(arrival['common_mean_mm']-frozen)<.002
    stop=c['first_stop_ack_ms'];pre=[x for x in od if x['sample_ms']<stop and x['tick_ms']<stop][-1]
    use(p/'authorization.json');use(p/'analyze_once.py')
    return dict(folder=name,candidate_sha256=EXPECTED,hardware_snapshot_verified=True,
      provenance=dict(raw_parser_chip_measured=False,receipt_chip_measured=True,trusted_acquisition=True,reason='offline parser literal is not a denial of verified hardware read'),
      ruler=dict(right_mm=m.get('right_mm',m.get('signed_right_mm')),forward_mm=m['forward_mm'],extra_mm=m['forward_error_mm']),
      timing=timing,tail=tail,
      combined_measured_scopes_percent_same_window=sum(q['measured_scope_cycles_per_own_window_percent'] for q in timing['scopes']) if len({(q['first_ms'],q['last_ms']) for q in timing['scopes']})==1 else None,
      stop_distance=dict(frozen_arrival_mm=frozen,PE_final_encoder_calibrated_mm=pe_mm,PE_minus_frozen_mm=pe_mm-frozen,
        PE_minus_ruler_mm=pe_mm-m['forward_mm'],ruler_minus_frozen_mm=m['forward_mm']-frozen,
        PE_end_after_firstSTOP_ms=[min(ep[f'end_ms{i}'] for i in range(1,5))-stop,max(ep[f'end_ms{i}'] for i in range(1,5))-stop],
        latest_preSTOP_OD=pre,last_OD=od[-1],last_OD_after_STOP_ms=od[-1]['sample_ms']-stop,
        last_complete_batch=batches[-1],first_stop_ack_ms=stop,stop_ack_span_ms=c['stop_ack_span_ms']),
      observations={k:c[k] for k in ['middle_samples','middle_sample_peak_abs_mm','lateral_peak_rpm','lateral_cap30_complete_batches','heading_peak_rpm','heading_cap20_complete_batches','hwt_start_deg','hwt_delta_deg','moving_sampled_peak_right_yaw_deg','pre_stop_left_from_start_deg','post_pre_stop_yaw_change_deg','pre_stop_gap_to_ack_ms','xd1','post_complete_tx','complete_batches']},
      software_final={k:s['last'][k] for k in ['vm_request','pe2_active','backend_active','backend_state','backend_result','can_fault','rtos_fault','safety_fault','stop_mask','telemetry_dropped','heap_min_bytes','can_stack_words','max_sample_gap_ms']},
      powered_tail_s=s['powered_tail_last_elapsed_s'])

def reference_group(base,names):
    out=[]
    for name in names:
        p=base/name;m=read(p/'operator_measurement.json');c=read(p/'capture_check.json')
        out.append(dict(folder=name,right_mm=m.get('signed_right_mm',m.get('right_mm')),extra_mm=m.get('forward_error_mm',m['forward_mm']-2100),OD_middle_peak_mm=c['middle_sample_peak_abs_mm']))
    return dict(runs=out,signed_right_mean_mm=statistics.mean(x['right_mm'] for x in out),right_abs_mean_mm=statistics.mean(abs(x['right_mm']) for x in out),extra_mean_mm=statistics.mean(x['extra_mm'] for x in out))

def main():
    global PARSERS,DISTANCE_SCALE
    PARSERS={'timing':parser(FIELD.parent/'float-timing-20261115/parse_snapshot.py','two_timing_parser'),
      'tail':parser(FIELD.parent/'stop-distance-study/snapshot-candidate/parse_snapshot.py','two_tail_parser')}
    elf=use(ROOT/'工创赛/tmp/tail-snapshot-build-20261115/baseline40_integrated/Vehicle_Integrated.axf');assert hashlib.sha256(elf.read_bytes()).hexdigest()==EXPECTED
    header=use(ROOT/'3.stm32f407工程/CHASSIS/USER/chassis_kinematics.h').read_text('gbk')
    DISTANCE_SCALE=float(re.search(r'#define CHASSIS_FORWARD_DISTANCE_SCALE\s+([0-9.]+)f',header)[1])
    runs=[analyze(n) for n in ['diagnostic-run1','diagnostic-run2-retry1']]
    refs={
      'half':reference_group(FIELD.parents[1]/'2026-11-14/baseline40-field',[f'conservative-ff-run{i}' for i in range(1,5)]),
      'S3':reference_group(FIELD,['bounded-run1','bounded-run2-retry1','bounded-run3','bounded-run4-retry1']),
      'S2.5':reference_group(FIELD,[f's25-run{i}' for i in range(1,5)])}
    failures=[]
    for name in ['diagnostic-run2','diagnostic-run3','diagnostic-run3-retry1','diagnostic-run3-retry2']:
        p=FIELD/name;item=dict(folder=name,counts_as_accuracy_trial=False)
        for filename in ['result_summary.json','reset_once_receipt.json','reset_observation.json','first_failure_summary.json','operator_failure_observation.json']:
            if (p/filename).exists():
                d=read(p/filename);item[filename]={k:d[k] for k in ['status','error','reset','reset_attempted','limits','user_reply','observation'] if k in d}
        failures.append(item)
    summary=dict(scope='Offline saved-data analysis only; user cancelled further testing',candidate_sha256=EXPECTED,runs=runs,references=refs,failures=failures,
      comparison=dict(diagnostic_right_mean_mm=statistics.mean(x['ruler']['right_mm'] for x in runs),diagnostic_right_abs_mean_mm=statistics.mean(abs(x['ruler']['right_mm']) for x in runs),diagnostic_extra_mean_mm=statistics.mean(x['ruler']['extra_mm'] for x in runs)),
      rules_checked=dict(R=['RUN-01','PWR-01'],E=['LOC-01','LOC-02','LOC-03','LOC-05','LOC-07','MOT-02','MOT-04','RUN-06'],Q=['FIELD-05','OBS-03','QR-04']),
      limitations=['Groups not paired/randomized; endpoint deterioration is real observation, unique cause unknown.','Sparse wheel OD cannot refute endpoint ruler data or establish ground trajectory.','DWT includes IRQ/call boundary costs. Ratios cover only two measured scopes, not total CPU occupancy.','168MHz is recorded SystemCoreClock, not independent RCC measurement.','Reference/follower/pending/applied RPM are command-state quantities, not measured physical speed.','PE minus frozen progress mixes time alignment/sample lag/braking; not pure ground after-STOP coasting.','No successful RCC reset-register read; reset reason unknown. No new hardware, simulation or firmware change.'])
    lines=['# 两趟成功诊断离线数据（2026-11-15）','',
      '结论：两趟尺量右偏20/30mm、超距7.5/10mm，明显大于half历史四趟的右偏。OD中段峰均4mm不能否定尺量恶化；批次非严格配对，不能把原因唯一归于FF、插桩、供电或复位。用户已取消继续测试，本分析不发车、不连接硬件、不改控制。','',
      '## 来源与计时口径','',
      '两趟RAM均accepted稳定双读：前后RUNNING、同session、DONE9/result1/active0、VM/PE2高、fault0/PLAN8，四bin哈希与receipt一致，绑定5e803d…a185 ELF。离线parser的chip_cycles_measured=false/hardware_provenance=false是固定来源声明；外层真实硬件receipt和trusted_acquisition=true已核，不能因此误说未测。','',
      'DWT elapsed含IRQ和调用边界；以下百分比仅所测两个范围累计周期/各自first至last时窗，不是CPU总负载。168MHz为记录的SystemCoreClock，未另测RCC。','',
      '|趟次/范围|count|window ms|min/mean/max us|sum us|本范围时窗占比%|','|---|---:|---:|---|---:|---:|']
    for x in runs:
        for q in x['timing']['scopes']:lines.append(f"|{x['folder']}/{q['name']}|{q['count']}|{q['window_ms']}|{q['min_us']:.4f}/{q['mean_us']:.4f}/{q['max_us']:.4f}|{q['sum_elapsed_us']:.3f}|{q['measured_scope_cycles_per_own_window_percent']:.6f}|")
    lines+=['','两段同窗合计比例分别为'+ '/'.join(f"{x['combined_measured_scopes_percent_same_window']:.6f}%" for x in runs)+'。这些已测范围不是毫秒级热点，当前不支持为提速改定点；未测路径不据此宣称没有热点。','','## 末端三个真实软件状态','',
      '记录中的now/run_tick/common_ms来自同一板时基；remaining采用原profile duration8.7786626816s减actual elapsed。follow_rpm为跟随修正状态，pending/applied为目标命令，并非实际轮/地面速度。latest freeze不是完整四轮ACK；arrival槽在reached赋值前。']
    for x in runs:
        lines+=['',f"### {x['folder']}"]
        for q in x['tail']['slots']:
            lines.append(f"- {q['name']}: now/run_tick={q['now_ms']}/{q['run_tick_ms']}ms，elapsed={q['elapsed_ms']}ms，common_age={q['common_age_ms']}ms；common mean/ref={q['common_mean_mm']:.3f}/{q['reference_position_mm']:.3f}mm，差{q['common_mean_minus_current_reference_mm']:+.3f}mm；ref F/follow={q['reference_rpm']:.3f}/{q['follow_rpm']:+.3f}RPM，pending/applied={q['pending_base_rpm']}/{q['applied_base_rpm']}RPM；profile_done={q['flags_decoded']['profile_done']}，原profile还余{q['profile_time_remaining_s']:.6f}s。")
        d=x['stop_distance'];o=x['observations']
        lines.append(f"- 冻结到位{d['frozen_arrival_mm']:.3f}mm；最终PE原始编码器校准{d['PE_final_encoder_calibrated_mm']:.3f}mm，比冻结多{d['PE_minus_frozen_mm']:.3f}mm；PE结束采样在首STOP ACK后{d['PE_end_after_firstSTOP_ms']}ms。尺量{ x['ruler']['forward_mm']:.1f}mm、右{x['ruler']['right_mm']:.1f}mm。上述量不同点/时刻，PE差不能全称物理滑行。")
        lines.append(f"- 末OD along/right={d['last_OD']['along_mm']:.0f}/{d['last_OD']['right_mm']:.0f}mm，sample在首STOP后{d['last_OD_after_STOP_ms']:.0f}ms；中段仅{o['middle_samples']}点、abs峰{o['middle_sample_peak_abs_mm']}mm。最后完整ACK批F/L/C={d['last_complete_batch']['F_rpm']:.1f}/{d['last_complete_batch']['L_rpm']:.1f}/{d['last_complete_batch']['C_rpm']:.1f}RPM，STOP ACK跨度{d['stop_ack_span_ms']}ms。")
        lines.append(f"- HWT运动期采样右峰{o['moving_sampled_peak_right_yaw_deg']:.3f}°；STOP前最近样本相对起点左{o['pre_stop_left_from_start_deg']:+.3f}°（距首ACK {o['pre_stop_gap_to_ack_ms']}ms），最终左{o['hwt_delta_deg']:+.3f}°，追加左转{o['post_pre_stop_yaw_change_deg']:.3f}°。这不是横向地面越线。")
        lines.append(f"- XD {o['xd1']['received_records']}/{o['xd1']['expected_records']}、drop{o['xd1']['dropped']}；telemetry queue drop{x['software_final']['telemetry_dropped']}另列；VM/PE2=1、backend inactive、stop mask15、故障0，powered tail {x['powered_tail_s']:.3f}s、完成后TX{o['post_complete_tx']}。")
    lines+=['','到位时common mean已领先当时ref约20mm，而原profile尚未结束、ref F仍约15RPM；本次真实状态支持“参考尚未归零就触发到位”的机制。不能据此直接改距离比例，也不保证降低floor会改变到位命令或地面超距。','','## 历史组描述性对比','','|组|尺量右偏mm|均值/绝对均值mm|超距均值mm|','|---|---|---|---:|']
    for name,g in refs.items():lines.append(f"|{name}|{'/'.join(str(q['right_mm']) for q in g['runs'])}|{g['signed_right_mean_mm']:.3f}/{g['right_abs_mean_mm']:.3f}|{g['extra_mean_mm']:.3f}|")
    lines+=['|本次diag|20/30|25.000/25.000|8.750|','','这些是不同批次的实测端点，不是配对因果增益；half当时较好可作为用户选择恢复的依据，不证明half在所有工况最优。真实中段地面、测点/参考关系、轮地滑移及响应仍未知。','','## 失败边界（不计入精度）','','- diagnostic-run2：telemetry_session_changed；保存只读RCC观察USB超时，没有寄存器证据，不能确定重启原因。','- diagnostic-run3：USBTimeoutError，reset_attempted=false/reset=false，未成功执行复位，不是一趟精度样本。','- diagnostic-run3-retry1、diagnostic-run3-retry2：分别ack_timeout；保留首失败，不能与两趟正常完成混算精度。用户没有说明重启原因，当前已暂停重试。','','已核R RUN-01/PWR-01；E LOC-01/02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04仍待技术答疑。恢复half和新增停止测距由父任务另行实现与验证，本目录仅数据分析，不修改固件或其他证据。']
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),'utf8')
    (OUT/'report_data.md').write_text('\n'.join(lines)+'\n','utf8')
    (OUT/'input_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),'utf8')
    for rel,h in manifest.items():assert hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()==h
    print(json.dumps({'runs':len(runs),'verified_inputs':len(manifest),'comparison':summary['comparison'],'scope_window_percent':[x['combined_measured_scopes_percent_same_window'] for x in runs]},indent=2))

if __name__=='__main__':main()
