"""普通伸缩区分瞬时堵转与保护，持续判定有界；不烧录或发送硬件命令。"""
from pathlib import Path
import argparse
import difflib
import json
import shutil
from prepare_extension_jog1600_20261101 import ACTUAL, build, sha, verify

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/extension_stall_confirm_20261101/project'
EV = ROOT / 'evidence/p2-arm/2026-11-01/extension-stall-confirm'
FILES = ['HARDWARE/EXTENSION/extension.c', 'HARDWARE/EXTENSION/extension.h',
         'HARDWARE/EXTENSION/extension_config.h']


def one(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError('Source anchor mismatch: ' + old[:120])
    return text.replace(old, new, 1)


def edit(relative, transform):
    path = STAGE / relative
    raw = path.read_bytes()
    original = raw.decode('utf-8').splitlines(keepends=True)
    before = [line.rstrip('\r\n') for line in original]
    changed = transform(raw.decode('utf-8').replace('\r\n', '\n'))
    after = changed.splitlines()
    pieces = []
    # 原文件含混合换行：未改行逐字节保留，只为新增行沿用临近行换行。
    for tag, a0, a1, b0, b1 in difflib.SequenceMatcher(None, before, after, autojunk=False).get_opcodes():
        if tag == 'equal':
            pieces.extend(original[a0:a1])
        elif tag in ('insert', 'replace'):
            adjacent = original[min(a0, len(original) - 1)]
            newline = '\r\n' if adjacent.endswith('\r\n') else '\n'
            pieces.extend(line + newline for line in after[b0:b1])
    path.write_bytes(''.join(pieces).encode('utf-8'))


def save(name, data):
    (EV / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def config(text):
    assert '#define EXTENSION_JOG_PULSES        1600U' in text
    assert '#define EXTENSION_MOVE_TIMEOUT_MS   10000U' in text
    return one(text, '#define EXTENSION_SERVICE_GAP_MS    500U',
               '#define EXTENSION_SERVICE_GAP_MS    500U\n'
               '/* 本机Clog_Ms口述2000ms；只约束普通运行的瞬时堵转，不写驱动参数。 */\n'
               '#define EXTENSION_STALL_CONFIRM_MS  2000U')


def header(text):
    return one(text, 'extern volatile uint32_t g_extension_motion_elapsed_ms;',
               'extern volatile uint32_t g_extension_motion_elapsed_ms;\n'
               '/* 普通运行瞬时堵转观察；保护位或失能仍立即停，以下值仅用于诊断。 */\n'
               'extern volatile uint32_t g_extension_stall_pending;\n'
               'extern volatile uint32_t g_extension_stall_elapsed_ms;\n'
               'extern volatile uint32_t g_extension_stall_max_ms;\n'
               'extern volatile uint32_t g_extension_stall_events;')


def module(text):
    text = one(text, 'volatile uint32_t g_extension_motion_elapsed_ms = 0U;',
               'volatile uint32_t g_extension_motion_elapsed_ms = 0U;\n'
               'volatile uint32_t g_extension_stall_pending = 0U;\n'
               'volatile uint32_t g_extension_stall_elapsed_ms = 0U;\n'
               'volatile uint32_t g_extension_stall_max_ms = 0U;\n'
               'volatile uint32_t g_extension_stall_events = 0U;')
    text = one(text, 'static int64_t s_extension_before_position = 0;',
               'static int64_t s_extension_before_position = 0;\n'
               'static TickType_t s_extension_stall_since;\n'
               'static uint32_t s_extension_stall_active = 0U;')
    text = one(text, '    return g_extension_stop_result;\n}',
               '    s_extension_stall_active = 0U; g_extension_stall_pending = 0U;\n'
               '    return g_extension_stop_result;\n}')
    text = one(text, '        g_extension_motion_elapsed_ms = 0U;\n',
               '        g_extension_motion_elapsed_ms = 0U;\n'
               '        s_extension_stall_active = 0U; g_extension_stall_pending = 0U;\n'
               '        g_extension_stall_elapsed_ms = 0U; g_extension_stall_max_ms = 0U;\n'
               '        g_extension_stall_events = 0U;\n')
    text = one(text, '    if (!Extension_MotionIsBusy()) return;\n',
               '    if (!Extension_MotionIsBusy()) return;\n'
               '    if (s_extension_stall_active)\n'
               '    {\n'
               '        g_extension_stall_elapsed_ms = (uint32_t)((now - s_extension_stall_since) * portTICK_PERIOD_MS);\n'
               '        if (g_extension_stall_elapsed_ms > g_extension_stall_max_ms)\n'
               '            g_extension_stall_max_ms = g_extension_stall_elapsed_ms;\n'
               '    }\n')
    text = one(text,
               '            if ((g_extension_flags & 0x0CU) != 0U || !(g_extension_flags & 1U))\n'
               '            { Extension_MotionFault(5U); break; }\n',
               '            /* 保护位/失能立即停；0x04单次低速大电流标志不冒充已持续保护。 */\n'
               '            if ((g_extension_flags & 0x08U) != 0U || !(g_extension_flags & 1U))\n'
               '            { Extension_MotionFault(5U); break; }\n'
               '            if (g_extension_flags & 0x04U)\n'
               '            {\n'
               '                if (!s_extension_stall_active)\n'
               '                {\n'
               '                    s_extension_stall_active = 1U; s_extension_stall_since = now;\n'
               '                    g_extension_stall_pending = 1U; g_extension_stall_elapsed_ms = 0U;\n'
               '                    g_extension_stall_events++;\n'
               '                }\n'
               '                /* 每次新鲜状态复核；持续到界仍停，整体10秒和服务超时不延长。 */\n'
               '                if ((TickType_t)(now - s_extension_stall_since) >= pdMS_TO_TICKS(EXTENSION_STALL_CONFIRM_MS))\n'
               '                { Extension_MotionFault(5U); break; }\n'
               '            }\n'
               '            else\n'
               '            {\n'
               '                s_extension_stall_active = 0U; g_extension_stall_pending = 0U;\n'
               '                g_extension_stall_elapsed_ms = 0U;\n'
               '            }\n')
    return one(text, '            if ((g_extension_flags & 2U) && speed == 0 && (delta >= 2 || delta <= -2))',
               '            if ((g_extension_flags & 2U) && !(g_extension_flags & 0x0CU) &&\n'
               '                speed == 0 && (delta >= 2 || delta <= -2))')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'build-stage', 'install'])
    args = parser.parse_args()
    EV.mkdir(parents=True, exist_ok=True)
    if args.action == 'prepare':
        if (EV / 'manifest.json').exists():
            raise RuntimeError('Candidate already exists; refusing evidence overwrite')
        if STAGE.exists():
            # 仅恢复尚无manifest的本脚本失败副本，不覆盖实际源或已有完成候选。
            for relative in FILES:
                assert sha(EV / 'original' / relative) == sha(ACTUAL / relative)
                shutil.copy2(ACTUAL / relative, STAGE / relative)
        else:
            shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns(
                'Objects', 'Listings', '.git', '.vscode', '*.uvguix.*'))
        for relative in FILES:
            backup = EV / 'original' / relative
            backup.parent.mkdir(parents=True, exist_ok=True)
            if backup.exists():
                assert sha(backup) == sha(ACTUAL / relative)
            else:
                shutil.copy2(ACTUAL / relative, backup)
        edit(FILES[0], module)
        edit(FILES[1], header)
        edit(FILES[2], config)
        protected = {p.relative_to(ACTUAL).as_posix(): sha(p) for p in ACTUAL.rglob('*')
                     if p.is_file() and p.suffix.lower() in ('.c', '.h', '.uvprojx')
                     and p.relative_to(ACTUAL).as_posix() not in FILES}
        manifest = {'date': '2026-11-01',
                    'files': [{'file': rel, 'before': sha(ACTUAL / rel), 'after': sha(STAGE / rel)} for rel in FILES],
                    'protected': protected,
                    'user_context': 'User confirmed wiring/rack and Clog_Ms2000; explicitly requested checking code versus threshold.',
                    'candidate': {'pulses': 1600, 'rpm': 5, 'timeout_ms': 10000,
                                  'instantaneous_stall_confirm_ms': 2000,
                                  'hard_protection_and_disabled_stop': 'immediate',
                                  'preflight_stall_rejection': 'unchanged'},
                    'host_pass': False, 'stage_build_pass': False, 'installed': False,
                    'flash_performed': False, 'hardware_commands_sent': False}
        verify(STAGE, manifest)
        save('manifest.json', manifest)
        print(json.dumps({'prepared': True, 'changed_files': FILES, 'protected_files': len(protected)}))
        return
    manifest = json.loads((EV / 'manifest.json').read_text(encoding='utf-8'))
    verify(STAGE, manifest)
    for entry in manifest['files']:
        assert sha(STAGE / entry['file']) == entry['after']
    review = json.loads((EV / 'host-review.json').read_text(encoding='utf-8'))
    assert review['passed']
    for entry in manifest['files']:
        assert review['changed_source_sha256'][entry['file']] == entry['after']
    if args.action == 'build-stage':
        build(STAGE, EV / 'stage-build.log')
        verify(STAGE, manifest)
        manifest.update(host_pass=True, stage_build_pass=True)
        save('manifest.json', manifest)
        return
    assert manifest['host_pass'] and manifest['stage_build_pass']
    verify(ACTUAL, manifest)
    for entry in manifest['files']:
        assert sha(ACTUAL / entry['file']) == entry['before']
    for entry in manifest['files']:
        shutil.copy2(STAGE / entry['file'], ACTUAL / entry['file'])
    manifest.update(installed=True, actual_build_pass=False)
    save('installed.json', manifest)
    build(ACTUAL, EV / 'actual-build.log')
    verify(ACTUAL, manifest)
    for entry in manifest['files']:
        assert sha(ACTUAL / entry['file']) == entry['after']
    manifest['actual_build_pass'] = True
    save('installed.json', manifest)
    print(json.dumps({'installed': True, 'actual_build_pass': True,
                      'protected_files': len(manifest['protected']), 'flashed': False}))


if __name__ == '__main__':
    main()
