"""Compare the extracted live heading module with main at da9f912; no hardware."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
USER = ROOT / '3.stm32f407工程/CHASSIS/USER'
OUT = ROOT / '工创赛/tmp/heading_source_main_a'
EVIDENCE = ROOT / '工创赛/evidence/navigation-planning/2026-11-10/main-a'
BIN = Path(r'D:\Keil_v5\ARM\arm5_compiler\Bin')
BASE = 'da9f912'


def functions(text):
    found = {}
    pattern = r'^(?:static\s+)?(?:__inline\s+)?(?:void|uint32_t|float|int)\s+(\w+)\([^;]*?\)\s*\{'
    for match in re.finditer(pattern, text, re.M):
        end = text.index('{', match.start()) + 1
        depth = 1
        while depth:
            depth += (text[end] == '{') - (text[end] == '}')
            end += 1
        found[match[1]] = text[match.start():end]
    return found


def main():
    old = subprocess.check_output(['git', 'show', f'{BASE}:3.stm32f407工程/CHASSIS/USER/main.c'], cwd=ROOT).decode('gbk')
    current = (USER / 'main.c').read_bytes().decode('gbk')
    module = (USER / 'heading_source.c').read_bytes().decode('gbk')
    before, after, moved = functions(old), functions(current), functions(module)
    checked = 0
    for name, body in before.items():
        if name in ('App_HeadingSourcePublish', 'App_HeadingSourceObserve'):
            assert body.removeprefix('static ') == moved[name], name
            continue
        if name == 'App_HeadingFrameAge':
            assert body in (USER / 'heading_source.h').read_bytes().decode('gbk')
            continue
        actual = after[name]
        actual = re.sub(r'^ +(?:AppHeadingSource heading;|uint32_t heading_ready;|(?:heading_ready = |\(void\))App_HeadingSourceRead\(&heading\);)\r?\n', '', actual, flags=re.M)
        actual = actual.replace('(void)App_HeadingSourceRead(&source);', 'taskENTER_CRITICAL(); source=s_heading_source; taskEXIT_CRITICAL();')
        if name != 'App_OdometryStep':
            actual = re.sub(r'\bheading_ready\b', 's_heading_source_ready', actual)
            actual = re.sub(r'\bheading\b', 's_heading_source', actual)
        actual = actual.replace('taskENTER_CRITICAL();\r\n            critical_cycle=', 'taskENTER_CRITICAL();critical_cycle=')
        assert actual == body, name
        checked += 1
    assert 's_heading_source' not in current
    assert current.count('App_HeadingSourcePublish();') == old.count('App_HeadingSourcePublish();') == 1
    assert re.search(r'HWT101_Update\(\);\s*#if[^\n]+\s*App_HeadingSourcePublish\(\);', current)
    for name in ('App_HeadingRuntimeStep', 'App_ForwardHoldStep', 'App_CombinedTestStep'):
        # Each originally unlocked preparation/service path must refresh on re-entry.
        assert after[name].count('App_HeadingSourceRead(&heading);') == 2, name
    OUT.mkdir(parents=True, exist_ok=True)
    for name in ('heading_source.c', 'heading_source.h', 'hwt101.h'):
        shutil.copyfile(USER / name, OUT / name)
    (OUT / 'app_config.h').write_text('#define APP_HEADING_SESSION_TEST 1\n#define APP_HWT_STATIC_TEST 0\n#define APP_HWT_TURN_TEST 0\n')
    (OUT / 'FreeRTOS.h').write_text('#include <stdint.h>\n#define configTICK_RATE_HZ 1000U\nextern uint32_t clock_ms,lock_depth;\n#define taskENTER_CRITICAL() (++lock_depth)\n#define taskEXIT_CRITICAL() (--lock_depth)\n')
    (OUT / 'task.h').write_text('#include "FreeRTOS.h"\n#define xTaskGetTickCount() clock_ms\n')
    start = old.index('#if APP_HEADING_SESSION_TEST || (APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST)\r\n#if configTICK_RATE_HZ')
    end = old.index('\r\n#endif\r\n#if APP_HEADING_SESSION_TEST', start) + len('\r\n#endif')
    block = old[start:end]
    names = ['AppHeadingSource', 's_heading_source', 's_heading_source_ready', 'App_HeadingFrameAge', 'App_HeadingSourcePublish', 'App_HeadingSourceObserve']
    names += re.findall(r'volatile (?:uint32_t|float) (g_heading_source_\w+)', block)
    rename = ''.join(f'#define {name} old_{name}\n' for name in names)
    undo = ''.join(f'#undef {name}\n' for name in names)
    harness = r'''
#include <stdint.h>
#include <string.h>
uint32_t clock_ms,lock_depth;
volatile uint32_t g_hwt_valid_mask,g_hwt_angle_age_ms,g_hwt_rate_age_ms,g_hwt_update_tick,g_hwt_version_result;
volatile uint32_t g_hwt_cal_active,g_hwt_cal_request,g_hwt_cal_end_tx_result,g_hwt_version_state,g_hwt_version_request,g_hwt_a7_request;
volatile float g_hwt_yaw_deg,g_hwt_wz_dps;
#include "heading_source.c"
''' + rename + block + '\n' + undo + r'''
volatile uint32_t assertions,cases;
#define CHECK(x) do {assertions++;if(!(x))return __LINE__;}while(0)
static uint32_t rng=0x12345678U;
static uint32_t next(void){rng=rng*1664525U+1013904223U;return rng;}
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 AppHeadingSource got, saved;uint32_t i,delay,ready;float bad_float;
 uint32_t nan_bits=0x7FC00000U;memcpy(&bad_float,&nan_bits,4);
 CHECK(App_HeadingSourceRead(&got)==0U);CHECK(lock_depth==0U);
 CHECK(got.sequence==0U && got.valid_mask==0U);
 for(i=0;i<3U;i++){
  clock_ms=0xFFFFFFF0U;g_hwt_update_tick=clock_ms;
  g_hwt_version_result=1U;g_hwt_valid_mask=3U;
  g_hwt_angle_age_ms=g_hwt_rate_age_ms=0U;
  App_HeadingSourcePublish();old_App_HeadingSourcePublish();
  clock_ms+=99U+i;App_HeadingSourceObserve();old_App_HeadingSourceObserve();
  CHECK(g_heading_source_valid==(i==0U));
  CHECK(g_heading_source_valid==old_g_heading_source_valid);
  CHECK(g_heading_source_angle_age_ms==99U+i);cases++;
 }
 for(i=0;i<4096U;i++){
  clock_ms=i<4U ? 0xFFFFFFFDU+i : next();
  g_hwt_update_tick=clock_ms-(i%7U);
  g_hwt_valid_mask=(i%8U<4U)?3U:i%4U;
  g_hwt_angle_age_ms=i%6U==0U?99U:i%6U==1U?100U:i%6U==2U?101U:next()%40U;
  g_hwt_rate_age_ms=next()%110U;
  if(i%13U==0U)g_hwt_angle_age_ms=0xFFFFFFFFU;
  if(i%17U==0U)g_hwt_rate_age_ms=0xFFFFFFFFU;
  g_hwt_version_result=i%19U==0U?0U:1U;
  g_hwt_yaw_deg=(float)((int32_t)(next()%361U)-180);
  g_hwt_wz_dps=(float)((int32_t)(next()%4001U)-2000);
  if(i%31U==0U)g_hwt_yaw_deg=180.1f;
  if(i%37U==0U)g_hwt_wz_dps=2000.1f;
  if(i%41U==0U)g_hwt_yaw_deg=bad_float;
  g_hwt_cal_active=i%43U==0U;g_hwt_cal_request=i%47U==0U;
  g_hwt_cal_end_tx_result=i%53U==0U?2U:0U;
  g_hwt_version_state=i%59U==0U;g_hwt_version_request=i%61U==0U;g_hwt_a7_request=i%67U==0U;
  if(i==4095U)s_heading_source.sequence=old_s_heading_source.sequence=0xFFFFFFFFU;
  App_HeadingSourcePublish();old_App_HeadingSourcePublish();
  ready=App_HeadingSourceRead(&got);CHECK(ready==old_s_heading_source_ready);
  CHECK(memcmp(&got,&old_s_heading_source,sizeof(got))==0);CHECK(lock_depth==0U);
  delay=i%4U==0U?99U:i%4U==1U?100U:i%4U==2U?101U:0U;clock_ms+=delay;
  App_HeadingSourceObserve();old_App_HeadingSourceObserve();
  CHECK(g_heading_source_valid==old_g_heading_source_valid);
  CHECK(g_heading_source_sequence==old_g_heading_source_sequence);
  CHECK(g_heading_source_age_ms==old_g_heading_source_age_ms);
  CHECK(g_heading_source_angle_age_ms==old_g_heading_source_angle_age_ms);
  CHECK(g_heading_source_rate_age_ms==old_g_heading_source_rate_age_ms);
  CHECK(memcmp((void*)&g_heading_source_yaw_deg,(void*)&old_g_heading_source_yaw_deg,4)==0);
  CHECK(memcmp((void*)&g_heading_source_rate_dps,(void*)&old_g_heading_source_rate_dps,4)==0);
  CHECK(App_HeadingFrameAge(&got,clock_ms,1U)==old_App_HeadingFrameAge(&old_s_heading_source,clock_ms,1U));
  CHECK(App_HeadingFrameAge(&got,clock_ms,2U)==old_App_HeadingFrameAge(&old_s_heading_source,clock_ms,2U));
  taskENTER_CRITICAL();(void)App_HeadingSourceRead(&got);CHECK(lock_depth==1U);taskEXIT_CRITICAL();CHECK(lock_depth==0U);
  cases++;
 }
 CHECK(got.sequence==0U);
 saved=got;g_hwt_yaw_deg=17.0f;App_HeadingSourcePublish();
 CHECK(memcmp(&got,&saved,sizeof(got))==0);
 CHECK(App_HeadingSourceRead(&got)==1U);CHECK(got.sequence==1U && got.yaw_deg==17.0f);
 (void)App_HeadingSourceRead(&saved);CHECK(memcmp(&got,&saved,sizeof(got))==0);
 return 0;
}
'''
    (OUT / 'unit.c').write_bytes(harness.encode('gbk'))
    (OUT / 'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n')
    commands = [[str(BIN / 'armcc.exe'), '--cpu', 'Cortex-M3', '--c99', '--split_sections', '-O1', '-I', '.', '-c', 'unit.c', '-o', 'unit.o'], [str(BIN / 'armlink.exe'), '--entry', 'test_entry', '--scatter', 'unit.sct', 'unit.o', '--library_type=microlib', '-o', 'unit.axf']]
    for command in commands:
        run = subprocess.run(command, cwd=OUT, capture_output=True, timeout=60)
        assert run.returncode == 0, (run.stdout + run.stderr).decode(errors='replace')
    sys.path.insert(0, str(ROOT / '工创赛/tmp/position_alignment_deps'))
    import unicorn as uc
    from unicorn import arm_const as reg
    blob = (OUT / 'unit.axf').read_bytes()
    elf = struct.unpack_from('<16sHHIIIIIHHHHHH', blob)
    machine = uc.Uc(uc.UC_ARCH_ARM, uc.UC_MODE_THUMB | uc.UC_MODE_MCLASS)
    machine.mem_map(0x08000000, 0x100000); machine.mem_map(0x20000000, 0x20000)
    for i in range(elf[10]):
        kind, offset, address, _, size, _, _, _ = struct.unpack_from('<IIIIIIII', blob, elf[5] + i * elf[9])
        if kind == 1 and size:
            machine.mem_write(address, blob[offset:offset + size])
    stop = 0x080FF000
    machine.reg_write(reg.UC_ARM_REG_SP, 0x2001FFF0); machine.reg_write(reg.UC_ARM_REG_LR, stop | 1)
    machine.emu_start(elf[4] | 1, stop, timeout=30000000, count=20000000)
    assert machine.reg_read(reg.UC_ARM_REG_PC) == stop, 'Emulator did not complete'
    result = machine.reg_read(reg.UC_ARM_REG_R0)
    assert result == 0, f'C assertion failed at generated unit.c line {result}'
    symbols = subprocess.check_output([str(BIN / 'fromelf.exe'), '--text', '-s', 'unit.axf'], cwd=OUT).decode(errors='replace')
    def value(name):
        address = int(re.search(r'\b' + name + r'\s+(0x[0-9a-fA-F]+)', symbols)[1], 16)
        return struct.unpack('<I', machine.mem_read(address, 4))[0]
    report = dict(ok=True, baseline=BASE, equivalent_main_functions=checked, differential_cases=value('cases'), assertions=value('assertions'), module_sha256=hashlib.sha256((USER / 'heading_source.c').read_bytes()).hexdigest(), hardware_accessed=False, limits='Actual heading C with stub clock/critical sections in ARM emulation; no RTOS scheduling, hardware or motion validation.')
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    (EVIDENCE / 'heading_source_check.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
