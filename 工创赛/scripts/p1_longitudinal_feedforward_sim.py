"""Isolated longitudinal feedforward trial. ARM C emulator only; no hardware API.

Reuses the reviewed runtime-route model adapter, with explicit extra plant
hypotheses. Never changes the live firmware. Run with --out NEW_DIRECTORY.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
PREVIOUS = ROOT / '工创赛/evidence/p1-foundation/2026-11-02/runtime_route'


def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding='utf8')


def hashes(directory):
    return {str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in directory.rglob('*') if p.is_file()}


CONFIG = '''#ifndef LONGITUDINAL_FEEDFORWARD_H
#define LONGITUDINAL_FEEDFORWARD_H
#include "zdt_x_config.h"
/* 独立离线试验：前后方向分别开关；permille表示右减左的幅值差/基础速度。 */
#ifndef APP_FF_FORWARD_ENABLE
#define APP_FF_FORWARD_ENABLE 0
#endif
#ifndef APP_FF_BACKWARD_ENABLE
#define APP_FF_BACKWARD_ENABLE 0
#endif
#ifndef APP_FF_FORWARD_PERMILLE
#define APP_FF_FORWARD_PERMILLE 0
#endif
#ifndef APP_FF_BACKWARD_PERMILLE
#define APP_FF_BACKWARD_PERMILLE 0
#endif
#if (APP_FF_FORWARD_ENABLE != 0 && APP_FF_FORWARD_ENABLE != 1) || (APP_FF_BACKWARD_ENABLE != 0 && APP_FF_BACKWARD_ENABLE != 1)
#error Invalid_feedforward_switch
#endif
#if APP_FF_FORWARD_PERMILLE < 0 || APP_FF_FORWARD_PERMILLE > 20 || APP_FF_BACKWARD_PERMILLE < 0 || APP_FF_BACKWARD_PERMILLE > 20
#error Invalid_feedforward_ratio
#endif
#if (APP_FF_FORWARD_ENABLE || APP_FF_BACKWARD_ENABLE) && !APP_X_OFFLINE_ONLY
#error Feedforward_trial_requires_offline_build
#endif
/* 只用于S曲线距离入口。前进左减右加；后退左增右减。零速严格归零。
 * 不改变平均轮速；本项产生偏航，不直接产生横移。和PD共用原总修正限幅。
 * 单侧最多0.5RPM；目标量化仍由原X编码器执行。 */
static __inline float LongitudinalFeedforward(uint32_t mode, uint32_t base)
{
    float value=0.0f;
    if(mode==0U && APP_FF_FORWARD_ENABLE)
        value=(float)base*(float)APP_FF_FORWARD_PERMILLE*0.0005f;
    if(mode==3U && APP_FF_BACKWARD_ENABLE)
        value=-(float)base*(float)APP_FF_BACKWARD_PERMILLE*0.0005f;
    if(value>0.5f)value=0.5f;
    if(value< -0.5f)value= -0.5f;
    return value;
}
#endif
'''


BOUNDARIES = r'''
static void feedforward_boundaries(void){
 uint32_t m,b,w,k,nan_bits=0x7fc00000U;float invalid;ChassisMotorCommand got[4],want[4];
 const float cs[7]={-2,-1.9f,-.1f,0,.1f,1.9f,2};
 CHECK(Chassis_TranslationHoldSelfTest());CHECK(Chassis_DistanceSelfTest());
 for(m=0;m<4;m++)for(b=0;b<=52;b++)for(k=0;k<7;k++){
  float effective=0,ff=0,base=(float)b,logical;ChassisWheelSpeeds wheels;
  CHECK(Chassis_TranslationProfileCommands(m,b,cs[k],got));
  #ifdef APP_FF_FORWARD_ENABLE
  if(m==0 && APP_FF_FORWARD_ENABLE)ff=base*APP_FF_FORWARD_PERMILLE*.0005f;
  if(m==3 && APP_FF_BACKWARD_ENABLE)ff=-base*APP_FF_BACKWARD_PERMILLE*.0005f;
  if(ff>.5f)ff=.5f;if(ff<-.5f)ff=-.5f;
  #endif
  CHECK(Chassis_ProfileCorrectionRpm(b,cs[k],&effective));
  effective+=ff;
  if(effective>(b<2?base:2))effective=b<2?base:2;
  if(effective<-(b<2?base:2))effective=-(b<2?base:2);
  for(w=0;w<4;w++){
   float s=m==3?-1.0f:1.0f;
   if(m==1)s=(w==0||w==3)?1.0f:-1.0f;
   if(m==2)s=(w==0||w==3)?-1.0f:1.0f;
   logical=s*base+((w&1)?effective:-effective);wheels.rpm[w]=logical;
   CHECK(logical*s>=-.0001f);CHECK(fabsf(logical)<=base+2.0001f);
   if(b==0)CHECK(got[w].speed_raw==0);
  }
  CHECK(Chassis_EncodeX(&wheels,b+2,want));
  for(w=0;w<4;w++){CHECK(got[w].address==want[w].address);CHECK(got[w].direction==want[w].direction);CHECK(got[w].speed_raw==want[w].speed_raw);}
 }
 CHECK(!Chassis_TranslationProfileCommands(4,20,0,got));
 for(w=0;w<4;w++)CHECK(got[w].speed_raw==0);
 CHECK(!Chassis_TranslationProfileCommands(0,53,0,got));
 CHECK(!Chassis_TranslationProfileCommands(0,20,2.1f,got));
 memcpy(&invalid,&nan_bits,sizeof(invalid));
 CHECK(!Chassis_TranslationProfileCommands(0,20,invalid,got));
 CHECK(!Chassis_TranslationProfileCommands(0,20,0,0));
}
'''


def prepare(out):
    out.mkdir(parents=True, exist_ok=False)
    live = ROOT / '3.stm32f407工程'
    before = hashes(live)
    save(out / 'live_before.json', before)
    baseline = out / 'baseline'
    baseline.mkdir()
    shutil.copytree(live / 'USER', baseline / 'USER')
    candidate = out / 'candidate'
    shutil.copytree(live, candidate, ignore=shutil.ignore_patterns(
        'Objects*', 'Listings*', '*.uvguix.*', '.vscode'))
    user = candidate / 'USER'
    (user / 'longitudinal_feedforward.h').write_bytes(CONFIG.replace('\n', '\r\n').encode('gbk'))
    p = user / 'chassis_kinematics.c'
    data = p.read_bytes()
    assert b'\r\n' in data and b'\n' not in data.replace(b'\r\n', b'')
    source = data.decode('gbk')
    source = source.replace('#include "motion_trial20.h"', '#include "motion_trial20.h"\r\n#include "longitudinal_feedforward.h"', 1)
    old = '    return Chassis_TranslationCommandsAtSpeed(mode,base_rpm,effective,0U,MOTION_PROFILE_BASE_CEILING,commands);'
    new = '''    /* 先按原入口验证PD，再将前馈和PD合并限幅；无前馈路径逐位保持。 */
    if(LongitudinalFeedforward(mode,base_rpm)!=0.0f)
    {
        float limit=base_rpm<2U?(float)base_rpm:2.0f;
        effective+=LongitudinalFeedforward(mode,base_rpm);
        if(effective>limit)effective=limit;
        if(effective< -limit)effective= -limit;
    }
''' .replace('\n', '\r\n') + old
    assert source.count(old) == 1
    p.write_bytes(source.replace(old, new).encode('gbk'))
    # Explicit offline build with 1% in each direction; defaults in the header stay off.
    p = candidate / 'gongxun-f407-offline.uvprojx'
    data = p.read_bytes().replace(b'APP_X_OFFLINE_ONLY=1', b'APP_X_OFFLINE_ONLY=1,APP_FF_FORWARD_ENABLE=1,APP_FF_BACKWARD_ENABLE=1,APP_FF_FORWARD_PERMILLE=10,APP_FF_BACKWARD_PERMILLE=10')
    p.write_bytes(data)
    default = data.replace(b'APP_FF_FORWARD_ENABLE=1', b'APP_FF_FORWARD_ENABLE=0').replace(
        b'APP_FF_BACKWARD_ENABLE=1', b'APP_FF_BACKWARD_ENABLE=0').replace(
        b'APP_FF_FORWARD_PERMILLE=10', b'APP_FF_FORWARD_PERMILLE=0').replace(
        b'APP_FF_BACKWARD_PERMILLE=10', b'APP_FF_BACKWARD_PERMILLE=0').replace(
        b'ObjectsOffline', b'ObjectsDefault').replace(b'ListingsOffline', b'ListingsDefault')
    (candidate / 'gongxun-f407-default-off.uvprojx').write_bytes(default)
    return before, baseline, candidate


def adapter(out, forward, backward, custom, corrected_endpoint=False):
    folder = out / ('adapter_custom' if custom else 'adapter_regression')
    folder.mkdir(exist_ok=True)
    runner = (PREVIOUS / 'run_models.py').read_text(encoding='utf8')
    opts = ['-DAPP_X_OFFLINE_ONLY=1', f'-DAPP_FF_FORWARD_ENABLE={int(forward >= 0)}',
            f'-DAPP_FF_BACKWARD_ENABLE={int(backward >= 0)}',
            f'-DAPP_FF_FORWARD_PERMILLE={max(0, forward)}', f'-DAPP_FF_BACKWARD_PERMILLE={max(0, backward)}']
    runner = runner.replace("'-O1','-I'", "'-O1'," + ','.join(repr(x) for x in opts) + ",'-I'")
    # More useful report fields; no changes to completion/fault expectations.
    runner = runner.replace("fields=['full_perimeter'", "fields=['direction'" if custom else "fields=['full_perimeter'")
    if custom:
        runner = runner.replace("'wheel_bias','fault_kind'", "'plant_kind','fault_kind'")
    fixture = (PREVIOUS / 'fixture.c').read_text(encoding='utf8')
    if corrected_endpoint:
        from p1_endpoint_fixture import correct_endpoint_fixture
        fixture=correct_endpoint_fixture(fixture)
        marker="    (out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf8')"
        extra="""    data=struct.unpack('<560f',m.mem_read(address('endpoint_injections'),2240))
    keys=['ack_target_rpm','injected_rpm','base_rpm','progress_mm','remaining_mm','ack_age_ms','reference_mm','reference_acceleration','micro_case','injection_count']
    report['endpoint_injections']=[dict(scenario=i,**dict(zip(keys,data[i*10:(i+1)*10]))) for i in range(count) if data[i*10+9]]
"""
        assert marker in runner
        runner=runner.replace(marker,extra+marker)
    fixture = fixture.replace('uint32_t test_entry(void){', BOUNDARIES + '\nuint32_t test_entry(void){\n feedforward_boundaries();', 1)
    if custom:
        fixture = fixture.replace('static uint32_t dynamic_count;', 'static uint32_t dynamic_count,trial_direction,plant_kind;')
        fixture = fixture.replace('float gain=gain_value*(1.0f+((w&1U)?bias:-bias));', '''float gain=gain_value*(1.0f+((w&1U)?bias:-bias));
  float wheel_tau=tau_value;
  if(plant_kind==1U && (w&1U))gain*=.99f;
  if(plant_kind==2U && (w&1U))wheel_tau*=2.0f;
  if(plant_kind==4U && (w&1U))gain*=1.01f;''')
        fixture = fixture.replace('0.01f/(tau_value+0.01f)', '0.01f/(wheel_tau+0.01f)')
        # Encoder follows the motor; extra ground-only sideways slip remains invisible.
        fixture = fixture.replace('ground_rate=body.wz_rad_s', 'if(plant_kind==3U)body.vy_mm_s-=.01f*body.vx_mm_s;\n ground_rate=body.wz_rad_s')
        fixture = fixture.replace('MOTION_FORWARD_DISTANCE,0,0,0,distance,speed', 'MOTION_FORWARD_DISTANCE+trial_direction,0,0,0,distance,speed')
        fixture = fixture.replace('CHECK(MotionRoute_Build(0,distance,speed', 'CHECK(MotionRoute_Build(trial_direction,distance,speed')
        fixture = fixture.replace('results[scenario][0]=(float)full;', 'results[scenario][0]=(float)trial_direction;')
        fixture = fixture.replace('results[scenario][3]=wheel_bias;', 'results[scenario][3]=(float)plant_kind;')
        # Record every trial, not only historical scenarios 0 and 24.
        fixture = fixture.replace('if(scenario==0U || (fault_kind==0U&&scenario==24U))', 'if(1)')
        start = fixture.index('uint32_t test_entry(void){')
        fixture = fixture[:start] + '''uint32_t test_entry(void){
 uint32_t d,p,f;single_turn_case=-1;feedforward_boundaries();
 for(d=0;d<2;d++){
  trial_direction=d?3U:0U;
  for(p=0;p<5;p++){plant_kind=p;scenario_run(0,2100,200,.08f,0,0,0);}
  plant_kind=0;
  scenario_run(0,2100,200,.20f,4,0,0);
  scenario_run(0,2100,200,.40f,4,0,0);
  for(f=1;f<=8;f++)scenario_run(0,2100,200,.20f,4,0,f);
 }
 return first_failure?(0x80000000U|first_failure):checks;
}
'''
    (folder / 'fixture.c').write_text(fixture, encoding='utf8')
    (folder / 'run_models.py').write_text(runner, encoding='utf8')
    spec = importlib.util.spec_from_file_location('ff_adapter', folder / 'run_models.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--summarize-existing', action='store_true', help='Collect saved reports, including failed regressions; do not rerun')
    args = parser.parse_args()
    out = args.out.resolve()
    if not out.is_relative_to(ROOT):
        raise ValueError('Output must remain within this repository')
    if args.summarize_existing:
        before=json.loads((out/'live_before.json').read_text(encoding='utf8'))
        baseline,candidate=out/'baseline',out/'candidate'
    else:
        before, baseline, candidate = prepare(out)
    reports = {}
    trials = [('baseline', baseline, -1, -1), ('default_off', candidate, -1, -1),
              ('zero', candidate, 0, 0), ('005', candidate, 5, 5),
              ('010', candidate, 10, 10), ('020', candidate, 20, 20),
              ('forward_only', candidate, 10, -1), ('backward_only', candidate, -1, 10)]
    for name, project, f, b in trials:
        print('Running', name, flush=True)
        if args.summarize_existing:
            reports[name]=json.loads((out/('models_'+name)/'report.json').read_text(encoding='utf8'))
        else:
            reports[name] = adapter(out, f, b, True, corrected_endpoint=True).run(project, out / ('models_' + name))
    for name, project, f, b in [('baseline', baseline, -1, -1), ('010', candidate, 10, 10)]:
        print('Running route regression', name, flush=True)
        if args.summarize_existing:
            reports['regression_'+name]=json.loads((out/('regression_'+name)/'report.json').read_text(encoding='utf8'))
        else:
            try:
                reports['regression_' + name] = adapter(out, f, b, False, corrected_endpoint=True).run(project, out / ('regression_' + name))
            except RuntimeError:
                saved=out/('regression_'+name)/'report.json'
                if not saved.exists():raise
                reports['regression_'+name]=json.loads(saved.read_text(encoding='utf8'))
                print('Regression failed; preserving the failed result for comparison', flush=True)
    rows = []
    for i, base in enumerate(reports['baseline']['models']):
        direction = int(base['direction'])
        for name in ('default_off', 'zero', '005', '010', '020', 'forward_only', 'backward_only'):
            row = reports[name]['models'][i]
            # Ground origin is (2100,2100), initial heading -90 degrees.
            longitudinal_target = 2100 + (2100 if direction == 3 else -2100)
            rows.append(dict(trial=name, scenario=i, direction=direction,
                plant_kind=int(base['plant_kind']), fault_kind=int(base['fault_kind']),
                completed=bool(row['completed_actions']), backend_cause=row['backend_cause'],
                lateral_baseline_mm=base['ground_x_mm']-2100,
                lateral_candidate_mm=row['ground_x_mm']-2100,
                longitudinal_baseline_mm=base['ground_y_mm']-longitudinal_target,
                longitudinal_candidate_mm=row['ground_y_mm']-longitudinal_target))
    same = lambda a,b: reports[a]['models'] == reports[b]['models']
    independent = all(reports['forward_only']['models'][i] == reports['baseline']['models'][i]
                      for i in range(15,30)) and all(
                      reports['backward_only']['models'][i] == reports['baseline']['models'][i] for i in range(15))
    result = dict(hardware_accessed=False, live_unchanged=before == hashes(ROOT / '3.stm32f407工程'),
        default_off_matches_baseline=same('baseline','default_off'),
        zero_matches_baseline=same('baseline','zero'), independent_switches=independent,
        reports={k:dict(passed=v['passed'], assertions=v['assertions'], models=len(v['models'])) for k,v in reports.items()},
        failures={k:dict(failure_line=v['failure_line'],failure_context=v['failure_context']) for k,v in reports.items() if not v['passed']},
        comparison=rows,
        limits='Hypothetical gain/tau/slip plants, CAN/RTOS stubs. No identified vehicle plant, no vendor internal loop. Precision claims only for completed models; protection stops are not performance passes.')
    save(out / 'comparison.json', result)
    assert result['live_unchanged'] and result['default_off_matches_baseline'] and result['zero_matches_baseline'] and independent
    print(json.dumps({k:v for k,v in result.items() if k not in ('comparison','limits')}, indent=2), flush=True)


if __name__ == '__main__':
    main()
