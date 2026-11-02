"""Summarize offline FF results and check compile-time restrictions; no hardware."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
from p1_turn_fault_audit import function

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    report = json.loads((out / 'comparison.json').read_text(encoding='utf8'))
    gate = out / 'compile_gates'
    gate.mkdir(exist_ok=False)
    (gate / 'unit.c').write_text('#include <stdint.h>\n#include "longitudinal_feedforward.h"\nfloat check(void){return LongitudinalFeedforward(0,50);}\n')
    cases = [('defaults', [], True), ('vehicle_off', ['APP_X_OFFLINE_ONLY=0'], True),
             ('forward_vehicle_rejected', ['APP_X_OFFLINE_ONLY=0','APP_FF_FORWARD_ENABLE=1'], False),
             ('backward_vehicle_rejected', ['APP_X_OFFLINE_ONLY=0','APP_FF_BACKWARD_ENABLE=1'], False),
             ('invalid_switch', ['APP_FF_FORWARD_ENABLE=2'], False),
             ('negative_ratio', ['APP_FF_BACKWARD_PERMILLE=-1'], False),
             ('excessive_ratio', ['APP_FF_FORWARD_PERMILLE=21'], False),
             ('independent_ratios', ['APP_FF_FORWARD_ENABLE=1','APP_FF_BACKWARD_ENABLE=1',
                                     'APP_FF_FORWARD_PERMILLE=5','APP_FF_BACKWARD_PERMILLE=20'], True)]
    builds = []
    for name, defines, expected in cases:
        command = ['D:/Keil_v5/ARM/arm5_compiler/bin/armcc.exe', '--cpu', 'Cortex-M3', '--c99',
                   '-I', str(out / 'candidate/USER'), *['-D'+d for d in defines], '-c', 'unit.c', '-o', name+'.o']
        p = subprocess.run(command, cwd=gate, capture_output=True, text=True, errors='replace', timeout=60)
        builds.append(dict(name=name, command=command, returncode=p.returncode,
                           output=p.stdout+p.stderr, passed=(p.returncode == 0) == expected))
    (gate / 'report.json').write_text(json.dumps(builds, indent=2), encoding='utf8')
    assert all(row['passed'] for row in builds)
    before = json.loads((out / 'live_before.json').read_text(encoding='utf8'))
    live = ROOT / '3.stm32f407工程'
    after = {str(p.relative_to(live)):hashlib.sha256(p.read_bytes()).hexdigest()
             for p in live.rglob('*') if p.is_file()}
    candidate = out / 'candidate'
    differences = [name for name, value in before.items()
                   if name.startswith(('USER\\', 'SYSTEM\\', 'HARDWARE\\'))
                   and (candidate/name).is_file() and hashlib.sha256((candidate/name).read_bytes()).hexdigest() != value]
    assert differences == ['USER\\chassis_kinematics.c'], differences
    assert before == after
    original=(out/'baseline/USER/chassis_kinematics.c').read_bytes().decode('gbk')
    modified=(candidate/'USER/chassis_kinematics.c').read_bytes().decode('gbk')
    function_names=['Chassis_Inverse','Chassis_Forward','Chassis_HeadingCommands',
                    'Chassis_TurnBrakeCommands','Chassis_TranslationCommandsAtSpeed',
                    'Chassis_TranslationHoldCommandsAtSpeed','Chassis_ProfileCorrectionRpm',
                    'Chassis_CalibrateDistanceMm']
    unchanged_functions={n:function(original,n)==function(modified,n) for n in function_names}
    assert all(unchanged_functions.values())
    build_logs = {}
    for log in ('build_candidate.log', 'build_default_off.log'):
        text=(out/log).read_bytes().decode('gbk',errors='replace')
        build_logs[log]='0 Error(s), 0 Warning(s).' in text
    assert all(build_logs.values())
    diagnostic_path=out/'fault_reproduction/model/report.json'
    diagnostic=json.loads(diagnostic_path.read_text(encoding='utf8')) if diagnostic_path.exists() else {}
    route_ok=report['reports']['regression_baseline']['passed'] and report['reports']['regression_010']['passed']
    route_notes=[
        '无前馈与1%候选的完整路线回归均符合预期；各注入场景的ACK目标/反馈与注入次数见regression_*/report.json。',
        '夹具按真实ACK目标和完整末端条件注入；有效微反号检查恢复，大反号/连续反号/窗口外反号仍检查保护停车。',
    ] if route_ok else [
        '路线回归存在失败，不能宣称整体通过；具体场景、完成段数及首次失败见failures与regression_*/report.json。',
        '历史夹具只按基础速度判断微反号时机，实际ACK目标可能超出2RPM；保留失败证据，不能放宽固件保护来满足测试。',
    ]
    trace_stats={}
    for key in ('baseline','005','010','020'):
        stats={}
        with (out/('models_'+key)/'traces.csv').open(encoding='utf8',newline='') as f:
            for row in csv.DictReader(f):
                i=int(float(row['scenario']))
                error=abs((float(row['ground_heading_deg'])+90+180)%360-180)
                stats[i]=max(stats.get(i,0),error)
        trace_stats[key]=stats
    summary = dict(hardware_accessed=False, github_pushed=False, live_unchanged=True,
                   candidate_changed_existing_sources=differences,
                   unchanged_functions=unchanged_functions,
                   compile_gates_passed=len(builds), default_off_matches_baseline=report['default_off_matches_baseline'],
                   zero_matches_baseline=report['zero_matches_baseline'], independent_switches=report['independent_switches'],
                   model_executions=sum(v['models'] for v in report['reports'].values()),
                   assertions_in_passed_groups=sum(v['assertions'] or 0 for v in report['reports'].values()),
                   all_expected_outcomes_passed=all(v['passed'] for v in report['reports'].values()),
                   failed_groups=report['failures'], adoption='not_adopted_model_dependent_benefit' if route_ok else 'rejected_pending_regression_resolution',
                   targeted_reproduction=diagnostic.get('fault_diagnostic'),
                   targeted_reproduction_progress_mm=diagnostic.get('progress_mm'),
                   full_build_logs=build_logs, sampled_heading_peak_deg=trace_stats,
                   default_off_image_sha256=hashlib.sha256((candidate/'ObjectsDefault/gongxun-f407.axf').read_bytes()).hexdigest(),
                   candidate_image_sha256=hashlib.sha256((candidate/'ObjectsOffline/gongxun-f407.axf').read_bytes()).hexdigest())
    lines = ['# 前进/后退速度前馈离线对照', '',
             '日期：2026-11-02。仅电脑仿真，未连接串口/下载器，未下载、复位、供电或发车。主工程全部文件SHA256未变。', '',
             '独立候选：candidate/gongxun-f407-offline.uvprojx；显式离线、前后各1%，头文件默认两个开关关闭、两个系数0。',
             '只在S曲线距离目标入口叠加，普通定时Hold/横移/转向原样。单侧最大0.5RPM，与PD共用min(base,2)RPM；X加速度仍1000RPM/s。',
             '实际C中基础速度按原整RPM更新，最终目标为0.1RPM量化；前馈与基础速度成比例，但不宣称连续无台阶或已证实实车无抖动。', '',
             '模型：无偏载；右轮稳态增益低1%；右轮时间常数加倍；地面额外1%右滑但编码器正常；右轮增益高1%（反假设）。',
             '统一0.08秒响应，原逐轮命令时序保留，因此无偏载不等于理想同步直线。另测0.20/0.40秒响应+40ms延迟及每方向8种故障/取消。',
             '右滑模型只是假设性地面横向速度，不能识别实际侧滑原因；这些模型没有厂家内部闭环、真实RTOS调度或无线时延。', '',
             '左右速度差是右轮幅值减左轮幅值再除以基础速度；50RPM、1%对应量化前左49.75/右50.25RPM。',
             '量化前平均逻辑轮速保持；X各轮独立舍入到0.1RPM后，均值最多有0.05RPM舍入偏差，实际差值也可能变化，不能声称编码后平均值精确不变。',
             '本项纯偏航预补偿，不能保证任意滑移条件下横向精度改善。前后方向未扩展运行时路线/无线协议。', '',
             '| 方向 | 假设模型 | 无前馈横偏mm | 0.5% | 1% | 2% |',
             '|---|---|---:|---:|---:|---:|']
    names = ['无偏载','右轮稳态增益低1%','右轮响应慢一倍','地面额外右滑1%','右轮稳态增益高1%']
    custom = {}
    for key in ('baseline','005','010','020'):
        custom[key] = json.loads((out / ('models_'+key) / 'report.json').read_text(encoding='utf8'))
    completed_rows = []
    for d, offset in [('前进',0),('后退',15)]:
        for plant in range(5):
            values=[]
            for key in custom:
                row=custom[key]['models'][offset+plant]
                values.append(f"{row['ground_x_mm']-2100:+.3f}" if row['completed_actions'] else '保护停车')
            lines.append('| '+' | '.join([d,names[plant],*values])+' |')
            completed_rows.append(dict(direction=d,plant=names[plant],values_mm=values))
    lines += ['', '上表是场地X终点相对起点的有符号横向偏移：前进时负为车体右侧，后退仍沿相同场地X定义，不能混同车体左右。',
              '纵向目标前进Y=0mm、后退Y=4200mm，起点(2100,2100)、初始航向-90°；纵向误差另列comparison.json，未把纵向误差当成横偏。', '',
              f"共{summary['model_executions']}次模型执行；通过测试组累计{summary['assertions_in_passed_groups']}条断言。全部符合预期：{summary['all_expected_outcomes_passed']}。",
              '默认关闭、启用但零系数均逐项复现无前馈模型；单独开前进/后退时另一方向逐项复现基线。',
              '单段对照每个版本的16个注入故障/取消均按原保护预期处理；慢响应压力模型可能保护停车，不把保护停车算运动性能通过。',
              *route_notes,
              '横移目标函数未改不等于整条路线状态不变；保留原反向保护，不放宽阈值。',
              '穷举四方向0..52RPM与正负PD，检查合并限幅、零速、无目标反转、0.1RPM编码、非法/NaN输入；定时Hold自检沿用原实现。',
              '编译门禁8项通过：开启任一前馈时APP_X_OFFLINE_ONLY必须为1；非法开关/负系数/超过2%拒绝编译。', '',
              '结论：固定速度差在特定右轮欠响应假设下可能有效，但无偏载、反向负载或纯侧滑下存在变差。软件回归状态见上；保持主工程原PD，不自动采用前馈系数。',
              '下一步：若继续研究，应先用可重复、完整四轮目标/跟速与尺量证据辨识负载，再分别选前进/后退参数；本轮不提供实车下载/发车助手。', '',
              '已核对需求：R FIELD-01/06、RUN-01、PWR-01；E LOC-03/05、MOT-02/04、RUN-06。R/Q不变，真实XY、实车停止距离/横偏与供电可靠性仍未闭环。', '',
              '证据：comparison.json、final_review.json、models_*/report.json与traces.csv、regression_*/report.json、fault_reproduction/model/report.json、compile_gates/report.json、两份build日志。',
              '失败记录保留在同级longitudinal_feedforward、longitudinal_feedforward_v2及v3：分别为沙箱依赖读取失败、NaN夹具输入不正确、边界夹具未读取另一编译单元的前馈配置。修正夹具后v4全部重跑；未修改保护代码来通过。', '',
              '仅本地完成，未提交或推送GitHub。']
    (out / 'RESULT.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    summary['completed_model_table']=completed_rows
    (out / 'final_review.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
