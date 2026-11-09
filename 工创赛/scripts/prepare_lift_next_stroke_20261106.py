"""Create the 15000p explicit-key candidate after the local MATLAB gate."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import check_lift_ruler_candidate_20261105 as prior

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tmp/lift_ruler_20261105/project'
STAGE = ROOT / 'tmp/lift_next_stroke_20261106/project'
OUT = ROOT / 'evidence/integration/2026-11-06/lift-next-stroke'


def adapt_unit(text):
    for old, new in [('total=3.625', 'total=9.875'), ('t<3.125', 't<9.375'),
                     ('562.5', '1687.5'), ('565.5', '1690.5'), ('5000U*360U', '15000U*360U'),
                     ('i<220', 'i<340'), ('>=3625U', '>=9875U')]:
        assert old in text, old
        text = text.replace(old, new)
    text = text.replace(' CK(!LiftRuler_Start(&c,now,1));', ''' if(c.state==LRC_DONE){
  CK(LiftRuler_Start(&c,now,1));CK(c.samples==1 && c.vm_request==1);
  CK(!LiftRuler_Start(&c,now,1));
 }else CK(!LiftRuler_Start(&c,now,1));''')
    return text


def main():
    proof = json.loads((OUT / 'matlab-result.json').read_text(encoding='utf-8'))
    assert proof['model_checks_passed'] and proof['failure_count'] == 0
    assert not STAGE.exists(), 'Preserve any existing candidate'
    shutil.copytree(BASE, STAGE, ignore=shutil.ignore_patterns('Objects', 'ObjectsOffline', 'Listings', '*.uvguix.*'))
    changed = {}

    def edit(name, transform):
        path = STAGE / name
        raw = path.read_bytes()
        try:
            text, encoding = raw.decode('utf-8'), 'utf-8'
        except UnicodeDecodeError:
            text, encoding = raw.decode('gbk'), 'gbk'
        result = transform(text.replace('\r\n', '\n'))
        path.write_bytes(result.replace('\n', '\r\n').encode(encoding))
        changed[name] = {'before': hashlib.sha256(raw).hexdigest(), 'after': hashlib.sha256(path.read_bytes()).hexdigest()}

    def controller(t):
        t = t.replace('#define LRC_STEP_PULSES 5000U', '#define LRC_STEP_PULSES 15000U')
        old = ''' if(!c || ready!=1U || c->state!=LRC_IDLE || c->moves_sent || c->samples || c->stop_pending)return 0U;
 c->state=LRC_POWER;c->vm_request=1;c->begin_ms=c->last_service_ms=now;return 1U;'''
        new = ''' if(!c || ready!=1U || c->cause || c->stop_pending || c->samples>=LRC_MAX_SAMPLES)return 0U;
 if(c->state==LRC_IDLE){if(c->moves_sent || c->samples)return 0U;}
 else if(c->state!=LRC_DONE || !c->samples || c->moves_sent!=1U)return 0U;
 /* Explicit new request after ordinary DONE only; never drop VM or clear a fault. */
 c->phase=c->stable=c->moves_sent=c->stop_confirmed=c->stop_unconfirmed=0U;
 c->state=LRC_POWER;c->vm_request=1;c->begin_ms=c->last_service_ms=now;return 1U;'''
        assert t.count(old) == 1
        t = t.replace(old, new).replace('>=8000U', '>=14000U')
        t = t.replace('value-36864U', 'value-(uint32_t)(((uint64_t)LRC_STEP_PULSES*65536U/3200U)&65535U)')
        assert t.count('c->samples=1;c->state=LRC_DONE;') == 1
        return t.replace('c->samples=1;c->state=LRC_DONE;', 'c->samples++;c->state=LRC_DONE;')

    edit('CHASSIS/USER/lift_ruler_calibration.c', controller)
    edit('CHASSIS/USER/lift_ruler_calibration.h', lambda t: t.replace('#include <stdint.h>', '#include <stdint.h>\n#define LRC_MAX_SAMPLES 7U /* 7x15000p maximum from user-confirmed113mm */'))

    def adapter(t):
        old = ' g_lift_ruler_request=(g_lift_ruler_controller.state==LRC_IDLE && !g_lift_ruler_request)?LRC_REQUEST_START:LRC_REQUEST_STOP;'
        new = ''' if(g_lift_ruler_controller.state==LRC_DONE && g_lift_ruler_controller.samples>=LRC_MAX_SAMPLES){
  g_lift_ruler_rejected=3U;
 }else g_lift_ruler_request=((g_lift_ruler_controller.state==LRC_IDLE ||
  g_lift_ruler_controller.state==LRC_DONE) && !g_lift_ruler_request)?LRC_REQUEST_START:LRC_REQUEST_STOP;'''
        assert t.count(old) == 1
        return t.replace(old, new)

    edit('CHASSIS/USER/lift_ruler_adapter.c', adapter)
    edit('CHASSIS/USER/vehicle_mission.inc', lambda t: t.replace(',5000,300,%lu,', ',15000,300,%lu,'))
    unit = adapt_unit(prior.UNIT).replace('#include <stdio.h>', '#include <stdio.h>\n#include <stdlib.h>')
    repeat = r'''
static void repeat_limit(void){
 LiftRulerController c;unsigned k,i;test="explicit_repeat_limit";initial=-123.4;scale=100;enable=0;
 LiftRuler_Init(&c);
 for(k=0;k<7;k++){
  moves=stops=forbidden=0;delta=speed=0;now=k*20000U;
  CK(LiftRuler_Start(&c,now,1));CK(c.vm_request==1 && c.samples==k);
  CK(!LiftRuler_Start(&c,now,1));
  for(i=0;i<340;i++){
   now=k*20000U+i*50U;
   LiftRuler_Service(&c,now,1,1,1,bus,&c);
   if(c.state>=LRC_DONE)break;
  }
  CK(c.state==LRC_DONE && c.samples==k+1 && c.vm_request==1 && moves==1 && !forbidden);
  i=moves;LiftRuler_Service(&c,now+50,1,1,1,bus,&c);CK(moves==i);
  initial+=delta;
 }
 CK(!LiftRuler_Start(&c,now+100,1));CK(c.samples==7 && c.vm_request==1);
}
'''
    unit = unit.replace('int main(void){', repeat + '\nint main(void){')
    unit = unit.replace(' printf("LIFT_RULER cases=', ' repeat_limit();\n printf("LIFT_RULER cases=')
    source = OUT / 'next-stroke-host.c'
    source.write_text(unit, encoding='utf-8')
    exe = OUT / 'next-stroke-host.exe'
    cmd = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
           '-I' + str(STAGE / 'CHASSIS/USER'), str(source), str(STAGE / 'CHASSIS/USER/lift_ruler_calibration.c'),
           '-lm', '-o', str(exe)]
    built = subprocess.run(cmd, capture_output=True, text=True)
    (OUT / 'host-build.log').write_text(built.stdout + built.stderr, encoding='utf-8')
    assert built.returncode == 0, built.stderr
    ran = subprocess.run([str(exe)], capture_output=True, text=True)
    (OUT / 'host-check.log').write_text(ran.stdout + ran.stderr, encoding='utf-8')
    print(ran.stdout)
    assert ran.returncode == 0, ran.stdout + ran.stderr
    report = {'passed': True, 'changed_files': changed, 'host_result': ran.stdout.strip(),
              'stage': str(STAGE), 'next_step_virtual_pulses': 15000, 'rpm': 30, 'timeout_ms': 14000,
              'maximum_samples': 7, 'maximum_batch_pulses': 105000,
              'initial_height_user_mm': 113, 'previous_top_cumulative_pulses': 10000,
              'mechanical_remaining_at_least63mm_user_confirmed': True,
              'only_next_step_released_for_field_measurement': True,
              'ordinary_vehicle_limit_changed': False, 'installed_in_external_actual_source': False,
              'firmware_downloaded': False, 'hardware_operated': False}
    (OUT / 'candidate-checks.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    (OUT / 'dispatch-matlab-result.json').write_text(json.dumps({'passed': proof['model_checks_passed'], 'check_count': proof['check_count'], 'source': 'matlab-result.json'}), encoding='utf-8')


if __name__ == '__main__':
    main()
