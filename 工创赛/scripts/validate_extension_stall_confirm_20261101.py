"""派生既有真实C夹具，覆盖瞬态/持续/保护/超时；旧夹具原文件保持。"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EV = ROOT / 'evidence/p2-arm/2026-11-01/extension-stall-confirm'
source = (ROOT / 'scripts/validate_extension_jog1600_20261101.py').read_text(encoding='utf-8')
assert source.count('extension_jog1600_20261101') == 2
source = source.replace('extension_jog1600_20261101', 'extension_stall_confirm_20261101')
assert source.count("extension-jog1600'") == 1
source = source.replace("extension-jog1600'", "extension-stall-confirm'")
extra_model = r'''
model = replace_once(model, 'if(is("fault")&&moved)r->Data[1]=5;',
    'if(is("fault")&&moved)r->Data[1]=5;\n'
    ' if(is("transient_stall")&&moved&&now<700)r->Data[1]=5;\n'
    ' if(is("persistent_stall")&&moved)r->Data[1]=5;\n'
    ' if(is("wrap_stall")&&moved)r->Data[1]=5;\n'
    ' if(is("preexisting_stall"))r->Data[1]=5;\n'
    ' if(is("stall_then_protect")&&moved)r->Data[1]=now<700?5:9;\n'
    ' if(is("stall_then_disable")&&moved)r->Data[1]=now<700?5:0;\n'
    ' if(is("stall_then_read_fail")&&moved){if(now>700)return 5;r->Data[1]=5;}\n'
    ' if((is("stall_then_abort")||is("stall_then_power_loss")||is("stall_then_gap"))&&moved)r->Data[1]=5;\n'
    ' if(is("transient_no_arrival")&&moved)r->Data[1]=now<700?5:1;\n'
    ' if(is("intermittent_stall")&&moved)r->Data[1]=now<4000?(((now/600)%2)?5:1):3;\n'
    ' if(is("arrival_with_stall")&&moved)r->Data[1]=7;')
model = replace_once(model, 'if(is("wrap"))now=0xFFFFFFE0U;',
                     'if(is("wrap")||is("wrap_stall"))now=0xFFFFFFE0U;')
model = replace_once(model, 'ready=!(is("not_ready")||(is("power_loss")&&moved));',
    'ready=!(is("not_ready")||(is("power_loss")&&moved)||(is("stall_then_power_loss")&&g_extension_stall_pending));')
model = replace_once(model, 'if(is("gap")&&moved)now+=600;',
    'if(is("gap")&&moved)now+=600;\n'
    '  if(is("stall_then_gap")&&g_extension_stall_pending)now+=600;\n'
    '  if(is("stall_then_abort")&&g_extension_stall_pending)g_extension_motion_request=3;')
model = replace_once(model, 'assert(g_extension_motion_result==expected);',
    'if(is("persistent_stall")||is("wrap_stall")||is("preexisting_stall")||is("stall_then_protect")||is("stall_then_disable")||is("arrival_with_stall"))expected=5;\n'
    ' if(is("stall_then_read_fail"))expected=4;\n'
    ' if(is("stall_then_power_loss"))expected=6;\n'
    ' if(is("stall_then_gap"))expected=7;\n'
    ' if(is("stall_then_abort"))expected=2;\n'
    ' if(is("transient_no_arrival"))expected=3;\n'
    ' assert(g_extension_motion_result==expected);\n'
    ' if(is("transient_stall")){assert(g_extension_stall_events==1&&g_extension_stall_max_ms>0&&g_extension_stall_max_ms<2000&&g_extension_stall_pending==0);}\n'
    ' if(is("intermittent_stall")){assert(g_extension_stall_events>=2&&g_extension_stall_max_ms<2000);}\n'
    ' if(is("fault")||is("persistent_stall")||is("wrap_stall")||is("arrival_with_stall")){assert(g_extension_stall_max_ms>=2000&&g_extension_stall_max_ms<=2400&&moves==1&&stops==1);}\n'
    ' if(is("protect")){assert(g_extension_stall_events==0&&g_extension_motion_elapsed_ms<700&&stops==1);}\n'
    ' if(is("stall_then_protect")||is("stall_then_disable")){assert(g_extension_stall_max_ms<2000&&g_extension_motion_elapsed_ms<1500&&stops==1);}\n'
    ' if(is("preexisting_stall")){assert(moves==0&&stops==1);}\n'
    ' if(expected!=1)assert(g_extension_stall_pending==0);')
'''
anchor = "(HOST / 'motion-model.c').write_text(model, encoding='utf-8')"
assert source.count(anchor) == 1
source = source.replace(anchor, extra_model + '\n' + anchor, 1)
anchor = "'gap', 'abort', 'stop_fail', 'readonly', 'readonly_during', 'late_success']"
assert source.count(anchor) == 1
source = source.replace(anchor,
    "'gap', 'abort', 'stop_fail', 'readonly', 'readonly_during', 'late_success',\n"
    "                'transient_stall', 'persistent_stall', 'wrap_stall', 'preexisting_stall',\n"
    "                'stall_then_protect', 'stall_then_disable', 'stall_then_read_fail',\n"
    "                'stall_then_abort', 'stall_then_power_loss', 'stall_then_gap',\n"
    "                'transient_no_arrival', 'intermittent_stall', 'arrival_with_stall']", 1)
anchor = "'expected_pulses': 1600, 'expected_timeout_ms': 10000,"
assert source.count(anchor) == 1
source = source.replace(anchor, anchor + "\n    'expected_stall_confirm_ms': 2000,\n"
    "    'changed_source_sha256': {entry['file']: hashlib.sha256((STAGE / entry['file']).read_bytes()).hexdigest() for entry in manifest['files']},", 1)
EV.mkdir(parents=True, exist_ok=True)
(EV / 'derived-validator-source.py').write_text(source, encoding='utf-8')
exec(compile(source, str(EV / 'derived-validator-source.py'), 'exec'),
     {'__name__': '__main__', '__file__': str(Path(__file__).resolve())})
