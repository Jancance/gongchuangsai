"""准备/安装可调取放点位的手动调试入口；不烧录、不发硬件动作。"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT / 'tmp/dm_can2_points_20261029/project'
EV = ROOT / 'evidence/p2-arm/2026-10-29/dm-can2-point-motion'
FILES = ['HARDWARE/DM_CAN2/dm_can2.h', 'HARDWARE/DM_CAN2/dm_can2_motion.c']
PROTECTED = ['HARDWARE/DM_CAN2/dm_can2.c', 'HARDWARE/DM_CAN2/dm_can2_protocol.c',
             'USER/main.c', 'gongxun-f407.uvprojx', 'SYSTEM/CAN/can.c', 'USER/app_config.h',
             'HARDWARE/LIFT/lift.c', 'HARDWARE/LIFT/lift_home.c',
             'HARDWARE/LIFT/lift_config.h', 'HARDWARE/EXTENSION/extension.c',
             'HARDWARE/EXTENSION/extension.h']


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def save(name, value):
    (EV / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def build(folder, log):
    proc = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-b', str(folder / 'gongxun-f407.uvprojx'),
                           '-j0', '-o', str(log)], capture_output=True, timeout=120)
    message = log.read_text(encoding='utf-8', errors='replace')
    assert proc.returncode == 0 and re.search(r'0 Error\(s\), 0 Warning\(s\)', message), message[-5000:]
    print(message[-600:], flush=True)


if '--install' in sys.argv:
    manifest = json.loads((EV / 'manifest.json').read_text(encoding='utf-8'))
    assert manifest['host_pass'] and manifest['stage_build_pass']
    for entry in manifest['files']:
        assert sha(ACTUAL / entry['file']) == entry['before'], entry['file']
        assert sha(STAGE / entry['file']) == entry['after'], entry['file']
    for name, expected in manifest['protected'].items():
        assert sha(ACTUAL / name) == expected, name
    for entry in manifest['files']:
        shutil.copy2(STAGE / entry['file'], ACTUAL / entry['file'])
        assert sha(ACTUAL / entry['file']) == entry['after']
    manifest.update(installed=True, actual_build_pass=False)
    save('installed.json', manifest)
    build(ACTUAL, EV / 'actual-build.log')
    for name, expected in manifest['protected'].items():
        assert sha(ACTUAL / name) == expected, name
    manifest['actual_build_pass'] = True
    save('installed.json', manifest)
    print('Manual point candidate installed; no flash or hardware commands.', flush=True)
    raise SystemExit(0)

resuming = '--resume' in sys.argv
assert not STAGE.exists() or resuming
EV.mkdir(parents=True, exist_ok=True)
if not resuming:
    shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns('Objects', 'Listings', '.vscode', '*.uvguix.*'))
manifest = {'files': [], 'protected': {n: sha(ACTUAL / n) for n in PROTECTED},
            'host_pass': False, 'stage_build_pass': False, 'installed': False,
            'pick_candidate_deg': 177.833679, 'place_coarse_candidate_deg': 33.8401489,
            'place_final_calibration_passed': False, 'point_enable_default': 0,
            'flash_performed': False, 'motion_performed': False}
for name in FILES:
    backup = EV / 'original' / name
    backup.parent.mkdir(parents=True, exist_ok=True)
    if backup.exists():
        assert resuming and sha(backup) == sha(ACTUAL / name), name
    else:
        shutil.copy2(ACTUAL / name, backup)
    shutil.copy2(ACTUAL / name, STAGE / name)
for name, expected in manifest['protected'].items():
    assert sha(STAGE / name) == expected, name


def replace_once(text, old, new):
    assert text.count(old) == 1, old[:120]
    return text.replace(old, new, 1)


p = STAGE / FILES[0]
data = p.read_bytes()
text = data.decode('utf-8').replace('\r\n', '\n')
text = replace_once(text,
    ' * 这是单圈姿态显示，不恢复多圈数，不作为运动目标、软限位或安全放行。',
    ' * 单圈显示不恢复多圈数；点位调试另需人工准入、新鲜状态及下述调试范围检查。')
text = replace_once(text,
    '/* 默认0；1反馈坐标正向45度，2负向45度，3中止并请求失能。 */',
    '''/* 默认0；1正向45度，2负向45度，3中止/失能；4去取料候选，5去放料候选。
 * 4/5仅用于人工看护的空载调试，须先明确当前姿态及全路径避让后置point_enable=1。
 * 点位默认禁用，不自动往返，不用于比赛自主动作组；不得按模一圈显示猜线束圈数。 */
#define DM_CAN2_POINT_TIME_MS 15000U
extern volatile uint32_t g_dm_point_enable;
/* 单位为度；仅在空闲时调整，接单锁存，重启恢复下列源码初值。 */
extern volatile float g_dm_pick_target_deg;
extern volatile float g_dm_place_target_deg;
extern volatile float g_dm_point_latched_deg;''')
text = replace_once(text, ' * 10动作超时，11配置不匹配，12服务间隔过大。 */',
                    ' * 10动作超时，11配置不匹配，12服务间隔过大，13点位参数非法，14点位范围/方向不符。 */')
p.write_bytes(text.replace('\n', '\r\n').encode('utf-8'))

p = STAGE / FILES[1]
text = p.read_text(encoding='utf-8')
text = replace_once(text, 'volatile uint32_t g_dm_jog_request = 0U;', '''volatile uint32_t g_dm_jog_request = 0U;
/* 只供人工看护空载调试；上电默认不允许点位运动。 */
volatile uint32_t g_dm_point_enable = 0U;
volatile float g_dm_pick_target_deg = 177.833679F;
/* 用户明确存在偏差：仅作粗调初值，后续可独立调整，不当最终放料标定。 */
volatile float g_dm_place_target_deg = 33.8401489F;
volatile float g_dm_point_latched_deg = 0.0F;''')
text = replace_once(text, 'static int32_t s_sign = 1;', '''static int32_t s_sign = 1;
static uint32_t s_point_mode = 0U, s_duration_ms = DM_CAN2_JOG_TIME_MS;
static float s_point_goal;
static float DM_CAN2_DisplayRelativeDeg(float raw_deg);''')
text = replace_once(text, '    float ratio;', '    float ratio, current_work_deg, delta_deg;')
text = replace_once(text, '        if (request != 1U && request != 2U) return;',
                    '        if (request != 1U && request != 2U && request != 4U && request != 5U) return;')
text = replace_once(text, '        s_sign = request == 1U ? 1 : -1;', '''        s_point_mode = (request == 4U || request == 5U) ? request : 0U;
        s_duration_ms = s_point_mode ? DM_CAN2_POINT_TIME_MS : DM_CAN2_JOG_TIME_MS;
        if (s_point_mode)
        {
            if (g_dm_point_enable != 1U)
            { g_dm_jog_state = 8U; g_dm_jog_result = 3U; return; }
            s_point_goal = request == 4U ? g_dm_pick_target_deg : g_dm_place_target_deg;
            /* 0到180度仅为本轮调试候选区，不是实测安全机械限位。含NaN/Inf拒绝。 */
            if (!(s_point_goal >= 0.0F && s_point_goal <= 180.0F))
            { g_dm_jog_state = 8U; g_dm_jog_result = 13U; return; }
            g_dm_point_latched_deg = s_point_goal;
        }
        s_sign = request == 1U ? 1 : -1;''')
text = replace_once(text, '    /* 忙时正负向请求直接消费，不排队、不延长动作时间。 */',
                    '    /* 忙时新启动请求直接消费，不排队、不延长动作时间；点位参数已锁存。 */')
text = replace_once(text, '        if (!ready || busy) { JogStop(3U, now); return; }',
                    '        if (!ready || busy || (s_point_mode && g_dm_point_enable != 1U))\n        { JogStop(3U, now); return; }')
text = replace_once(text, 'pdMS_TO_TICKS(12000U)', 'pdMS_TO_TICKS(s_point_mode ? 23000U : 12000U)')
text = replace_once(text,
    '        s_target = s_start + (float)s_sign * DM_CAN2_JOG_DEG * JOG_RAD_PER_DEG;',
    '''        if (s_point_mode)
        {
            /* 从本次新鲜原始反馈选同一工作分支，沿本轮人工确认的半圈区域计算。
             * 不能用保存的raw角直接下发，也不能据此推断掉电转轴后的线束圈数。
             * 起点留2度调试余量，4仅逆时针/近同点，5仅顺时针/近同点。 */
            current_work_deg = DM_CAN2_DisplayRelativeDeg(s_start * JOG_DEG_PER_RAD);
            delta_deg = s_point_goal - current_work_deg;
            if (!(current_work_deg >= -2.0F && current_work_deg <= 182.0F) ||
                (s_point_mode == 4U && delta_deg < -0.1F) ||
                (s_point_mode == 5U && delta_deg > 0.1F))
            { JogStop(14U, now); return; }
            s_target = s_start + delta_deg * JOG_RAD_PER_DEG;
        }
        else s_target = s_start + (float)s_sign * DM_CAN2_JOG_DEG * JOG_RAD_PER_DEG;''')
text = replace_once(text, '(float)DM_CAN2_JOG_TIME_MS;', '(float)s_duration_ms;')
text = replace_once(text, '/* 仅用于显示：取覆盖0到180度工作区的同圈分支；不修改原始反馈或动作参考。 */',
                    '/* 同圈工作分支：显示使用；人工准入的点位调试再结合新鲜原始角及范围/方向检查使用。 */')
p.write_text(text, encoding='utf-8')

for name in FILES:
    manifest['files'].append({'file': name, 'before': sha(ACTUAL / name), 'after': sha(STAGE / name)})

host = ROOT / 'tmp/dm_can2_points_20261029/host-check'
shutil.copytree(ROOT / 'tmp/dm_can2_reference_wrap_20261029/host-check', host,
                ignore=shutil.ignore_patterns('*.exe'), dirs_exist_ok=resuming)
model = (host / 'motion.c').read_text(encoding='utf-8')
model = replace_once(model, '#include <assert.h>', '#include <assert.h>\n#include <math.h>')
model = replace_once(model, 'static const float original=-2.0739f;', 'static float original=-2.0739f;')
model = replace_once(model, 'static const char *scenario;', 'static const char *scenario;\nstatic uint32_t point_case;')
model = replace_once(model, 'return strcmp(scenario,s)==0;',
                    'return strcmp(scenario,s)==0 || (point_case && strcmp(scenario+6,s)==0);')
model = replace_once(model, '45.1f*0.01745329252f', '(point_case?183.0f:45.1f)*0.01745329252f')
model = replace_once(model, ' assert(argc==2);scenario=argv[1];', ''' assert(argc==2);scenario=argv[1];
 point_case=strncmp(scenario,"point_",6)==0;
 if(point_case){
  position=(is("place")?241.289658f:97.2961273f)*0.01745329252f;
  if(is("minus_turn"))position-=360.0f*0.01745329252f;
  if(is("outside"))position=(-296.544037f+190.0f)*0.01745329252f;
  original=position;g_dm_can2_position_rad=position;
 }''')
model = replace_once(model, ' g_dm_jog_request=is("negative")?2:1;', ''' g_dm_jog_request=is("negative")?2:1;
 if(point_case){
  assert(g_dm_point_enable==0);
  g_dm_point_enable=is("gated")?0:1;
  if(is("badtarget"))g_dm_pick_target_deg=181.0f;
  if(is("nantarget"))g_dm_pick_target_deg=NAN;
  if(is("wrong_direction"))g_dm_pick_target_deg=20.0f;
  g_dm_jog_request=is("place")?5:4;
 }''')
model = model.replace('i<1400', 'i<2600')
model = replace_once(model, '  now+=10;\n  if(is("repeat")', '''  now+=10;
  if(point_case && is("edit") && g_dm_jog_state==4)g_dm_pick_target_deg=25.0f;
  if(point_case && is("gate_drop") && g_dm_jog_state==4 && i>80)g_dm_point_enable=0;
  if(is("repeat")''')
model = replace_once(model, ' expected=(is("positive")', ''' expected=(point_case && (is("pick") || is("place") || is("minus_turn") || is("edit")))?1:
          (point_case && (is("gated") || is("gate_drop")))?3:
          (point_case && (is("badtarget") || is("nantarget")))?13:
          (point_case && (is("outside") || is("wrong_direction")))?14:
          (is("positive")''')
old = '  assert(ab((g_dm_jog_target_deg-g_dm_jog_start_deg)-(is("negative")?-45.0f:45.0f))<0.01f);'
new = '''  if(point_case){
   float expected_raw=(is("place")?97.2961273f:241.289658f);
   if(is("minus_turn"))expected_raw-=360.0f;
   assert(ab(g_dm_jog_target_deg-expected_raw)<0.03f);
   /* 模型中命令编码及回包编码各截断一次，容许两位置LSB加浮点余量。 */
   assert(ab(g_dm_can2_logical_position_deg-(is("place")?33.8401489f:177.833679f))<
          2.0f*25.0f/65535.0f*57.2957795f+0.001f);
   assert(ab(g_dm_point_latched_deg-(is("place")?33.8401489f:177.833679f))<0.001f);
  }else assert(ab((g_dm_jog_target_deg-g_dm_jog_start_deg)-(is("negative")?-45.0f:45.0f))<0.01f);'''
model = replace_once(model, old, new)
model = replace_once(model,
    ' }else if(is("stale")||is("busy")||is("badprofile"))assert(txcount==0);',
    ' }else if(is("stale")||is("busy")||is("badprofile")||is("gated")||is("badtarget")||is("nantarget")||is("outside")||is("wrong_direction"))assert(txcount==0);')
(host / 'motion.c').write_text(model, encoding='utf-8')
module = STAGE / 'HARDWARE/DM_CAN2'
cmd = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
       '-I' + str(host), '-I' + str(module), str(module / 'dm_can2_protocol.c'),
       str(module / 'dm_can2_motion.c'), str(host / 'motion.c'), '-lm', '-o', str(host / 'motion.exe')]
proc = subprocess.run(cmd, capture_output=True, text=True)
assert proc.returncode == 0, proc.stdout + proc.stderr
cases = ['positive', 'negative', 'stale', 'txfail', 'noreply', 'fault', 'disabled',
         'tracking', 'limit', 'busy', 'abort', 'repeat', 'gap', 'badprofile', 'stoplost']
cases += ['point_' + n for n in ['pick', 'place', 'minus_turn', 'edit', 'gated',
          'gate_drop', 'badtarget', 'nantarget', 'outside', 'wrong_direction',
          'abort', 'repeat', 'stale', 'noreply', 'fault', 'gap', 'stoplost']]
logs = ['Old 15 jog cases + 17 point cases; 12 display vectors; synthetic responses only.']
for case in cases:
    proc = subprocess.run([str(host / 'motion.exe'), case], capture_output=True, text=True)
    logs.append(proc.stdout + proc.stderr)
    assert proc.returncode == 0, (case, proc.stdout, proc.stderr)
(EV / 'host-check.log').write_text('\n'.join(logs), encoding='utf-8')
manifest['host_pass'] = True
manifest['synthetic_motion_case_count'] = len(cases)
save('manifest.json', manifest)
build(STAGE, EV / 'stage-build.log')
manifest['stage_build_pass'] = True
save('manifest.json', manifest)
print('Adjustable manual point candidate staged; actual project untouched.', flush=True)
