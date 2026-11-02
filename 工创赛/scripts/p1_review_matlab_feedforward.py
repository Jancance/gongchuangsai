"""Review actual MATLAB outputs; preserve model-validation failures."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from p1_longitudinal_feedforward_sim import ROOT, hashes


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();out=a.out.resolve()
    r=json.loads((out/'matlab_results.json').read_text(encoding='utf8'))
    data=json.loads((out/'data_review.json').read_text(encoding='utf8'))
    assert all(hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==sha for path,sha in data['source_sha256'].items())
    assert hashes(ROOT/'3.stm32f407工程')==json.loads((out/'live_before.json').read_text(encoding='utf8'))
    assert len(r['fits'])==8 and len(r['simulations'])==144 and not r['hardware_accessed']
    assert 'MATLAB_FF_FINISHED fits8 simulations144' in (out/'matlab_run.log').read_text(errors='replace')
    rmse=[f['heldout_rmse_rpm'] for f in r['fits']]
    assert all(math.isfinite(x) and x>=0 for x in rmse)
    # This diagnostic threshold is an engineering screen, not a competition rule.
    yaw_checks=[dict(train_run=v['train_run'],test_run=v['test_run'],
                     heldout_yaw_rmse_deg=v['yaw_rmse_deg'],screen_limit_deg=1.0,
                     passed=v['yaw_rmse_deg']<=1,
                     predicted_peak_deg=v['predicted_peak_abs_yaw_deg'],
                     measured_peak_deg=v['measured_peak_abs_yaw_deg']) for v in r['open_loop_yaw_validation']]
    nominal=[s for s in r['simulations'] if s['assumed_extra_sensor_delay_s']==0 and s['assumed_slip_ratio']==0]
    assert len(nominal)==16
    for s in nominal:
        path=out/f"trajectory_fit{s['train_run']}_dir{s['direction']}_ff{round(s['ratio']*1000):03d}.csv"
        rows=list(csv.DictReader(path.open(encoding='utf8')))
        assert abs(float(rows[-1]['x_mm'])-s['final_x_mm'])<1e-5
        assert abs(float(rows[-1]['y_left_mm'])-s['final_y_left_mm'])<1e-5
    review=dict(execution_completed=True,hardware_accessed=False,github_pushed=False,live_unchanged=True,
                original_vehicle_logs_unchanged=True,crc_complete_runs=len(data['runs']),
                fits=8,simulations=144,aborted_simulations=sum(s['aborted'] for s in r['simulations']),
                heldout_wheel_rmse_rpm_range=[min(rmse),max(rmse)],yaw_validation=yaw_checks,
                ground_prediction_validated=False,backward_measured=False,
                all_selected_tau_at_search_lower_bound=all(f['tau_s']==.02 for f in r['fits']),
                model_adoption='not_qualified_for_vehicle_lateral_prediction',
                coefficient_adoption=False,
                nominal_sensitivity=nominal,
                important_limit='Model execution success is separate from failed physical yaw validation. No calibrated vehicle lateral prediction or optimal coefficient claim.')
    (out/'final_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf8')
    lines=['# MATLAB实车日志校准及前馈试验','',
           '2026-11-02，实际在本机MATLAB R2016b运行完成。主工程和原始实车日志哈希未变，无硬件访问、下载或发车，未提交推送。','',
           '**结论：这次确认了模型的局限，尚未得到可靠的实车前馈系数。轮速粗略模型可拟合，但“轮速模型→车身航向/横偏”的验证失败。**','',
           '输入是11月1日原PD诊断版两趟前进210cm：2023/2023和2056/2056条XD1记录，CRC重解码与JSON逐项一致、dropped=0；所有采样目标均与实际成功ACK历史配对，无配错。',
           '每轮约0.506秒采一条速度，每趟每轮46~48条含停车样本。原同趟尺量右偏21/27.5mm、欠15/17.5mm仅保留作独立比较，不用于强行拟合横偏。停稳灯灭/抖动现场确认原状态仍未补齐。','',
           '方法：分轮拟合一阶滞后+延迟+增益+带符号偏置，按真实ACK事件做精确零阶保持积分。仅用目标>=3RPM且处于运动状态的样本拟合；第一趟拟合、第二趟验证，再互换。测试日志未参与选择训练参数。',
           '网格：tau=0.02/0.04/0.06/0.08/0.10/0.15/0.20/0.30/0.40/0.60秒，延迟0..0.30秒每0.02秒；增益和偏置对每个网格点最小二乘拟合。',
           '8个最优tau都落在搜索下界0.02秒。近优参数范围广，尤其延迟与时间常数可互相替代；这些不是20ms内环已被精确测出的证据，也不是置信区间。不能用稀疏数据排除更快响应、短时振荡或短时限流。','',
           '| 拟合→验证 | 轮号 | K | 偏置RPM | tau秒 | 延迟秒 | 留出误差RMSE RPM |',
           '|---|---:|---:|---:|---:|---:|---:|']
    for f in r['fits']:
        lines.append(f"| {f['train_run']}→{f['test_run']} | {f['wheel']} | {f['gain']:.4f} | {f['bias_rpm']:+.3f} | {f['tau_s']:.2f} | {f['delay_s']:.2f} | {f['heldout_rmse_rpm']:.3f} |")
    lines+=['',f"留出轮速RMSE范围{min(rmse):.2f}~{max(rmse):.2f}RPM；前馈单侧最多0.5RPM，因此模型误差量级本身已大于待比较的小幅修正，不能据此精确排名系数。",'',
            '随后按留出日志的ACK命令重放，使用原名义77mm轮径及k=209mm麦轮运动学积分航向，结果：','',
            '| 拟合→验证 | 航向RMSE度 | 实测相对航向峰值度 | 模型相对航向峰值度 |',
            '|---|---:|---:|---:|']
    for v in yaw_checks:
        lines.append(f"| {v['train_run']}→{v['test_run']} | {v['heldout_yaw_rmse_deg']:.3f} | {v['measured_peak_deg']:.3f} | {v['predicted_peak_deg']:.3f} |")
    lines+=['','工程筛查阈值取留出航向RMSE<=1度，两组均未通过。1度仅为本次模型可信度筛查，不是竞赛规则或已批准实车精度标准。',
            '原因尚不能确定：稀疏轮速拟合偏差、轮胎接触/有效轮径、滑移、机械耦合及时间关系都可能影响；不从差异直接断言是某项原因。当前不具备可靠地面XY模型。','',
            '在保留上述限制的前提下，仍完成144组探索性前馈对照：2组拟合参数×前后2方向×0/0.5/1/2%×额外航向延迟0/40/80ms×假设横滑比例-0.5/0/+0.5%。',
            '额外航向延迟和横滑比例是人为敏感性假设，不是从日志辨识的值。后退全部沿用前进模型作带符号对称外推，没有后退实测校准。',
            'MATLAB数学实现包含PD、0.3度死区、80ms角速度滤波、10RPM/s输出限斜率、0.1RPM迟滞量化，前馈共用min(base,2)修正限幅，保留实际逐轮ACK更新时刻。',
            '基础速度采用留出日志的已记录时序，不重新闭环求距离；使用名义轮径，未用尺量反推地面比例，终点纵向约2043~2051mm。各方案纵向距离也列出，不能把这些当成严格相同实车2100mm的结果。',
            '本轮没有MEX编译器，故不是把整个实际C后端接入MATLAB；它是数学控制模型，不含真实RTOS、CAN时延调度器、完整停止/故障状态机。仅仿真角差超过10度会标记中止，不能代替上一轮实际C保护回归。','',
            '**以下仅为模型敏感性示例，不能预测实车会偏这些毫米，更不能把某个接近0的结果当作最佳实车系数。**','',
            '| 拟合组 | 方向 | 无前馈横偏mm | 0.5% | 1% | 2% |',
            '|---|---|---:|---:|---:|---:|']
    for train in (1,2):
        for direction in (1,-1):
            values=[next(s for s in nominal if s['train_run']==train and s['direction']==direction and s['ratio']==ratio)['final_y_left_mm'] for ratio in (0,.005,.01,.02)]
            lines.append('| '+str(train)+' | '+('前进' if direction==1 else '后退外推')+' | '+' | '.join(f'{x:+.3f}' for x in values)+' |')
    lines+=['','上表是场地Y有符号横向偏移，车头名义朝+X，+Y为起始车头左侧；无额外传感延迟/横滑的名义模型。比较不同拟合组，所谓最佳系数会变化，不能据此采纳。','',
            '下一步：保留主工程原PD和独立前馈候选；本轮不采用系数。若后续要提高预测能力，应另行安排前/后方向重复数据、稳定平台段/可辨识激励、保留现有保护的更高时间分辨率目标与反馈，以及同步地面轨迹。当前不自动安排或执行硬件试验。','',
            '图表：[轮速留出验证](wheel_validation.png)、[航向验证失败对照](yaw_validation_failure.png)、[仅供探索的前馈轨迹](ff_trajectories.png)。PNG与FIG均保留。',
            '复现：先用p1_prepare_matlab_feedforward.py --out 新目录导出CRC校验数据，再在相应目录运行matlab_ff_trial.m；报告p1_review_matlab_feedforward.py --out 结果目录。',
            '证据：data_review.json、commands/samples/telemetry.csv、fit_profile.csv、wheel_validation.csv、yaw_replay_*.csv、matlab_results.json/mat、matlab_run.log、final_review.json。',
            '首次几次运行的R2016b JSON路径/长字段名和空结构数组兼容错误保留在同一日志中，最终以MATLAB_FF_FINISHED及结果JSON为准，不把脚本退出码0单独当完成证据。','',
            '已核对R FIELD-01/06、RUN-01、PWR-01；E LOC-03/05、MOT-02/04、RUN-06。R/Q不变；实车航向/侧滑/地面精度、后退模型和现场停稳确认仍未闭环。']
    (out/'RESULT.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    print(json.dumps({k:review[k] for k in ['execution_completed','fits','simulations','heldout_wheel_rmse_rpm_range','yaw_validation','model_adoption','live_unchanged']},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
