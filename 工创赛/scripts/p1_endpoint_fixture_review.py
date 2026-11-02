"""Verify and summarize corrected endpoint regression evidence."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path

from p1_longitudinal_feedforward_sim import ROOT, hashes
from p1_endpoint_fixture import correct_endpoint_fixture


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True)
    a=parser.parse_args();out=a.out.resolve()
    old=ROOT/'工创赛/evidence/p1-foundation/2026-11-02/longitudinal_feedforward_v4'
    reports={v:json.loads((out/('full_'+v)/'models/report.json').read_text(encoding='utf8'))
             for v in ('baseline','zero','010')}
    checks={}
    original_fixture=(ROOT/'工创赛/evidence/p1-foundation/2026-11-02/runtime_route/fixture.c').read_text(encoding='utf8')
    corrected_fixture=correct_endpoint_fixture(original_fixture)
    (out/'fixture.diff').write_text(''.join(difflib.unified_diff(original_fixture.splitlines(True),corrected_fixture.splitlines(True),
        fromfile='historical_fixture.c',tofile='corrected_fixture.c')),encoding='utf8')
    checks['all_groups_passed']=all(r['passed'] for r in reports.values())
    checks['zero_equals_baseline']=reports['zero']['models']==reports['baseline']['models']
    checks['live_unchanged']=hashes(ROOT/'3.stm32f407工程')==json.loads((old/'live_before.json').read_text(encoding='utf8'))
    for variant in ('baseline','010'):
        previous=json.loads((old/('regression_'+variant)/'report.json').read_text(encoding='utf8'))
        checks[variant+'_non_endpoint_models_unchanged']=all(
            row==previous['models'][i] for i,row in enumerate(reports[variant]['models']) if i not in (28,29,30,31))
    details=[]
    for variant,r in reports.items():
        evidence=json.loads((out/('full_'+variant)/'verification.json').read_text(encoding='utf8'))
        checks[variant+'_sources_unchanged']=evidence['live_unchanged'] and evidence['candidate_user_unchanged']
        injections=r['endpoint_injections']
        checks[variant+'_all_four_cases_injected']={int(row['micro_case']) for row in injections}=={1,2,3,4}
        for event in injections:
            case=int(event['micro_case']);i=event['scenario'];row=r['models'][i]
            prefix=f'{variant}_case{case}_'
            checks[prefix+'opposite_sign']=event['ack_target_rpm']*event['injected_rpm']<0
            if case<4:
                checks[prefix+'ack_target_within_2rpm']=0<abs(event['ack_target_rpm'])<=2.00001
                checks[prefix+'decelerating_endpoint']=0<=event['remaining_mm']<=10 and event['reference_acceleration']<=0
            else:
                checks[prefix+'outside_endpoint']=event['remaining_mm']>10
            if case==1:
                checks[prefix+'exactly_once']=event['injection_count']==1
                checks[prefix+'completed_route']=row['completed_actions']==8 and row['backend_result']==1 and row['backend_cause']==0
            else:
                checks[prefix+'protected_stop']=row['completed_actions']==1 and row['backend_result']==2 and row['backend_cause']!=0
                if case==2:checks[prefix+'persistent_feedback_injected']=event['injection_count']>=2
            details.append(dict(variant=variant,**event,completed_actions=row['completed_actions'],
                                backend_cause=row['backend_cause'],backend_result=row['backend_result']))
    result=dict(passed=all(checks.values()),checks=checks,hardware_accessed=False,github_pushed=False,
                models=sum(len(r['models']) for r in reports.values()),
                assertions=sum(r['assertions'] or 0 for r in reports.values()),endpoint_cases=details,
                group_assertions={v:r['assertions'] for v,r in reports.items()},
                source_snapshot='longitudinal_feedforward_v4 baseline/candidate; firmware unchanged',
                old_failure_preserved=str(old/'regression_010/report.json'),
                scripts_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [
                    ROOT/'工创赛/scripts/p1_endpoint_fixture.py',ROOT/'工创赛/scripts/p1_endpoint_regression.py',
                    ROOT/'工创赛/scripts/p1_longitudinal_feedforward_sim.py']})
    (out/'final_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    assert result['passed'],[name for name,value in checks.items() if not value]
    lines=['# 端点微反号仿真夹具修正与回归','',
           '2026-11-02。仅修改电脑仿真，主工程1314文件及独立候选USER源码保持原样。未访问硬件、未烧录、未上传GitHub。','',
           '旧夹具按基础速度<=2RPM注入，未考虑PD叠加后的已确认目标；1%候选曾在ID1目标-2.3RPM时注入+0.3RPM，正确触发保护。旧失败及定点重现完整保留。','',
           '新夹具以该轮真实成功ACK目标为依据：有效非零目标、目标幅值<=2RPM、方向已稳定500ms；同时匹配原固件末端10mm、基础速度<=2RPM、参考位置/减速条件及非停止状态。',
           '注入反馈按ACK目标取反号；保留原CAN样本时序，不改控制输出、故障恢复或VM。每次记录目标/反馈/基础速度/位置/剩余距离/ACK年龄/参考加速度/注入次数。',
           '为增加符合原保护契约的采样机会，合法注入窗口从旧3mm改为固件原有10mm；这是测试选样范围，固件窗口未变。','',
           '| 版本 | 合法单次反弹ACK目标RPM | 注入RPM | 剩余距离mm | 注入次数 | 完成段数 |',
           '|---|---:|---:|---:|---:|---:|']
    for d in details:
        if d['micro_case']==1:
            lines.append(f"| {d['variant']} | {d['ack_target_rpm']:.1f} | {d['injected_rpm']:+.1f} | {d['remaining_mm']:.3f} | {int(d['injection_count'])} | {int(d['completed_actions'])}/8 |")
    lines+=['',f"无前馈、启用零系数、前后各1%三组完整路线各40场景，共{result['models']}模型、{result['assertions']}条断言符合预期。", 
            '合法单次微反号三版均注入成功并完成8段；持续异常、大反号(+/-0.6RPM)、10mm窗口外反号均在第二段按原保护停车，不能将预期停车当成路线完成。',
            '持续异常首次匹配合法末端后会持续提供同向异常反馈，覆盖STOP/SETTLE，不能因ACK变零或末端窗口关闭就偷偷停止注入。若已进入停车阶段，预期为未归零导致停稳超时；运行阶段则按连续反号保护。实际cause逐项记录在final_review.json。',
            '另直接运行实际C方向保护，覆盖正负目标0.1..2RPM下单次0.5RPM反号/同号恢复、旧样本不重复计数、目标2.1RPM拒绝、反号0.6RPM拒绝、窗口外拒绝、第二次新反号拒绝以及599/600ms超时边界。',
            '零系数与无前馈所有模型结果逐项一致；无前馈和1%候选除四个修正的端点注入场景外，其余36个模型结果分别与各自原记录逐项相同。','',
            '本次关闭的是旧夹具条件错误造成的回归未通过项。前馈在侧滑等模型中收益不稳定的结论不变；未自动将前馈合入主工程。','',
            '文件：p1_endpoint_fixture.py生成修正，p1_longitudinal_feedforward_sim.py的新仿真显式使用修正；历史重现仍可用corrected_endpoint=False。',
            '复现：python -X utf8 -B 工创赛/scripts/p1_endpoint_regression.py --out 新目录 --variant baseline/zero/010（每次选一项）；汇总p1_endpoint_fixture_review.py。',
            '报告：full_*/models/report.json、full_*/verification.json、final_review.json、fixture.diff。先行quick_010在上级目录，不计入上述120模型统计。','',
            '已核对R FIELD-01/06、RUN-01、PWR-01；E LOC-03/05、MOT-02/04、RUN-06。R/Q不变，模型未替代实车XY精度、停止距离或侧滑辨识。']
    (out/'RESULT.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    print(json.dumps({k:result[k] for k in ['passed','models','assertions','group_assertions']},indent=2))


if __name__=='__main__':
    main()
