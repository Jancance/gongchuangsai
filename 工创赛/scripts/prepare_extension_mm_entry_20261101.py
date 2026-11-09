"""采用32脉冲/mm粗标接入毫米调试入口；保留原脉冲入口，不烧录。"""
from pathlib import Path
import argparse
import json
import shutil
import prepare_extension_stall_confirm_20261101 as common

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = common.ACTUAL
STAGE = ROOT / 'tmp/extension_mm_entry_20261101/project'
EV = ROOT / 'evidence/p2-arm/2026-11-01/extension-mm-entry'
FILES = common.FILES
one, sha, verify, build = common.one, common.sha, common.verify, common.build


def config(text):
    return one(text, '#define EXTENSION_STALL_CONFIRM_MS  2000U',
               '#define EXTENSION_STALL_CONFIRM_MS  2000U\n'
               '/* 1600脉冲约50mm粗标：仅看护调试，不代表精密标定或绝对行程。 */\n'
               '#define EXTENSION_PULSES_PER_MM     32.0f\n'
               '#define EXTENSION_DEFAULT_MM        10.0f\n'
               '#define EXTENSION_DEFAULT_USE_MM    1U')


def header(text):
    text = one(text,
               '/* 1完成，2人工停止，3超时，4通信/选项，5驱动状态，6互锁，7服务间隔超限。 */',
               '/* 1完成，2人工停止，3超时，4通信/选项，5驱动状态，6互锁，7服务超间隔，8距离/单位非法。 */')
    return one(text, 'extern volatile uint32_t g_extension_motion_request;',
               'extern volatile uint32_t g_extension_motion_request;\n'
               '/* use_mm=1按distance_mm换算；0保留原1600脉冲方式；请求1缩回/2伸出/3停止。 */\n'
               'extern volatile uint32_t g_extension_use_mm;\n'
               'extern volatile float g_extension_distance_mm;\n'
               '/* 本次接单的命令回显，不是实测位置；忙时编辑输入不改变已锁存目标。 */\n'
               'extern volatile uint32_t g_extension_target_pulses;\n'
               'extern volatile float g_extension_target_mm;')


def module(text):
    text = one(text, 'volatile uint32_t g_extension_motion_request = 0U;',
               'volatile uint32_t g_extension_motion_request = 0U;\n'
               'volatile uint32_t g_extension_use_mm = EXTENSION_DEFAULT_USE_MM;\n'
               'volatile float g_extension_distance_mm = EXTENSION_DEFAULT_MM;\n'
               'volatile uint32_t g_extension_target_pulses = 0U;\n'
               'volatile float g_extension_target_mm = 0.0f;')
    text = one(text, 'static uint32_t s_extension_stall_active = 0U;',
               'static uint32_t s_extension_stall_active = 0U;\n'
               'static uint32_t s_extension_target_pulses = EXTENSION_JOG_PULSES;')
    helper = '''/* 接单时换算并锁存；先检查浮点范围再转整数，拒绝NaN/无穷/越界。 */
static uint32_t Extension_LatchDistance(void)
{
    uint32_t units = g_extension_use_mm, pulses = EXTENSION_JOG_PULSES;
    float distance = g_extension_distance_mm, requested;
    if (!(EXTENSION_PULSES_PER_MM > 0.0f)) return 0U;
    if (units == 1U)
    {
        if (!(distance > 0.0f &&
              distance <= (float)EXTENSION_JOG_PULSES / EXTENSION_PULSES_PER_MM)) return 0U;
        requested = distance * EXTENSION_PULSES_PER_MM;
        if (!(requested >= 0.5f && requested <= (float)EXTENSION_JOG_PULSES)) return 0U;
        pulses = (uint32_t)(requested + 0.5f);
    }
    else if (units != 0U) return 0U;
    if (pulses == 0U || pulses > EXTENSION_JOG_PULSES) return 0U;
    s_extension_target_pulses = pulses;
    g_extension_target_pulses = pulses;
    g_extension_target_mm = (float)pulses / EXTENSION_PULSES_PER_MM;
    return 1U;
}

'''
    text = one(text, 'static void Extension_BuildMove(uint8_t out[12])',
               helper + 'static void Extension_BuildMove(uint8_t out[12])')
    text = one(text, '    const uint32_t pulses = EXTENSION_JOG_PULSES;',
               '    const uint32_t pulses = s_extension_target_pulses;')
    return one(text,
               '    if (!Extension_MotionIsBusy() && (request == 1U || request == 2U))\n    {\n',
               '    if (!Extension_MotionIsBusy() && (request == 1U || request == 2U))\n    {\n'
               '        if (Extension_LatchDistance() != 1U)\n'
               '        {\n'
               '            g_extension_target_pulses = 0U; g_extension_target_mm = 0.0f;\n'
               '            g_extension_motion_result = 8U; g_extension_motion_state = 9U;\n'
               '            return; /* 未发运动命令，不自动修正非法距离。 */\n'
               '        }\n')


def save(name, data):
    (EV / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'build-stage', 'install'])
    args = parser.parse_args()
    EV.mkdir(parents=True, exist_ok=True)
    if args.action == 'prepare':
        if STAGE.exists() or (EV / 'manifest.json').exists():
            raise RuntimeError('Refusing to overwrite prepared candidate')
        shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns(
            'Objects', 'Listings', '.git', '.vscode', '*.uvguix.*'))
        for rel in FILES:
            backup = EV / 'original' / rel
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ACTUAL / rel, backup)
        common.STAGE = STAGE
        for rel, transform in zip(FILES, (module, header, config)):
            common.edit(rel, transform)
        protected = {p.relative_to(ACTUAL).as_posix(): sha(p) for p in ACTUAL.rglob('*')
                     if p.is_file() and p.suffix.lower() in ('.c', '.h', '.uvprojx')
                     and p.relative_to(ACTUAL).as_posix() not in FILES}
        m = {'date': '2026-11-01', 'files': [{'file': rel, 'before': sha(ACTUAL / rel), 'after': sha(STAGE / rel)} for rel in FILES],
             'protected': protected, 'user_decision': '快速开发，不做反复重复，采用已过单轮粗标继续实现。',
             'candidate': {'pulses_per_mm': 32, 'default_mm': 10, 'default_use_mm': 1,
                           'max_pulses': 1600, 'rpm': 5, 'timeout_ms': 10000,
                           'coarse_scale_not_precision_acceptance': True},
             'host_pass': False, 'stage_build_pass': False, 'installed': False,
             'flash_performed': False, 'hardware_commands_sent': False}
        verify(STAGE, m)
        save('manifest.json', m)
        print(json.dumps({'prepared': True, 'changed_files': 3, 'protected_files': len(protected)}))
        return
    m = json.loads((EV / 'manifest.json').read_text(encoding='utf-8'))
    verify(STAGE, m)
    review = json.loads((EV / 'host-review.json').read_text(encoding='utf-8'))
    assert review['passed']
    for e in m['files']:
        assert sha(STAGE / e['file']) == e['after'] == review['changed_source_sha256'][e['file']]
    if args.action == 'build-stage':
        build(STAGE, EV / 'stage-build.log')
        verify(STAGE, m)
        m.update(host_pass=True, stage_build_pass=True)
        save('manifest.json', m)
        return
    assert m['host_pass'] and m['stage_build_pass']
    verify(ACTUAL, m)
    for e in m['files']:
        assert sha(ACTUAL / e['file']) == e['before']
    for e in m['files']:
        shutil.copy2(STAGE / e['file'], ACTUAL / e['file'])
    m.update(installed=True, actual_build_pass=False)
    save('installed.json', m)
    build(ACTUAL, EV / 'actual-build.log')
    verify(ACTUAL, m)
    for e in m['files']:
        assert sha(ACTUAL / e['file']) == e['after']
    m['actual_build_pass'] = True
    save('installed.json', m)
    print(json.dumps({'installed': True, 'actual_build_pass': True, 'protected_files': len(m['protected']), 'flashed': False}))


if __name__ == '__main__':
    main()
