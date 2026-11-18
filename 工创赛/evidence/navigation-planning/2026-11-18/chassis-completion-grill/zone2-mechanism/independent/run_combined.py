from pathlib import Path
import ast
import hashlib
import json
import sys

root = Path.cwd()
here = Path(__file__).resolve().parent
out = here / sys.argv[1]
out.mkdir(exist_ok=False)
scripts = root / "工创赛/scripts"
sys.dont_write_bytecode = True
sys.path.insert(0, str(scripts))
from radar_zone2_arm_runner_20261111 import run

active = root / "3.stm32f407工程/CHASSIS/HARDWARE"
servo = active / "RADAR_SERVO"
radar = active / "RADAR_V6"
arm = Path("D:/Keil_v5/ARM/arm5_compiler/bin")
sources = (
    servo / "radar_lift.c", servo / "radar_servo.c", servo / "radar_servo_logic.c",
    radar / "radar_observer.c", radar / "radar_client.c", radar / "link_protocol.c",
)
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
before = {path: digest(path) for path in sources}
assert before[sources[3]].startswith("066dde3d"), before
for path in sources[1:]:
    (out / path.name).write_bytes(path.read_bytes())
for path in [*servo.glob("radar_*.h"), *radar.glob("*.h")]:
    (out / path.name).write_bytes(path.read_bytes())

# Reuse the register model from the current repo's TIM7 fixture.
historical = scripts / "check_radar_us733_lift_arm_20261112.py"
for node in ast.walk(ast.parse(historical.read_text(encoding="utf-8"))):
    if isinstance(node, ast.Constant) and isinstance(node.value, str) and "static TIM_TypeDef tim7" in node.value:
        tim7 = node.value
        break
else:
    raise ValueError("missing TIM7 stub")
(out / "stm32f4xx.h").write_text("#pragma once\n" + tim7 + (here / "combined_stm32_stub.h").read_text(), encoding="ascii")
(out / "stm32f4xx_tim.h").write_text('#include "stm32f4xx.h"\n', encoding="ascii")
(out / "app_entry_config.h").write_text("#define APP_IS_DEBUG 1\n", encoding="ascii")
(out / "zdt_emm_config.h").write_text("#define APP_MOTOR_OFFLINE_ONLY 0\n", encoding="ascii")
(out / "FreeRTOS.h").write_text("#define portTICK_PERIOD_MS 1U\n", encoding="ascii")
(out / "task.h").write_text("#include <stdint.h>\nuint32_t xTaskGetTickCount(void);\n", encoding="ascii")

host = root / "模快资料/雷达资料/radar_stm32例程/F407VET6/HostTests/stub_port.c"
uart = host.read_text(encoding="utf-8")
uart = uart.replace("static uint16_t receive_position;", "static uint16_t receive_position, read_pos;")
uart = uart.replace("    receive_position = 0U;", "    receive_position = 0U; read_pos = 0U;")
begin = uart.index("uint16_t RadarUartPort_GetReceivePosition")
end = uart.index("bool RadarUartPort_StartTransmit", begin)
uart = uart[:begin] + """bool RadarUartPort_ReadByte(uint8_t *byte)
{
    if (read_pos == receive_position) return false;
    *byte = receive_buffer[read_pos];
    read_pos = (uint16_t)((read_pos + 1U) % receive_size);
    return true;
}

extern void Fixture_Transmit(const uint8_t *data, uint16_t size);
""" + uart[end:]
uart = uart.replace("    ++transmit_count;", "    ++transmit_count;\n    Fixture_Transmit(data, length);")
(out / "stub_port.c").write_text(uart, encoding="utf-8")

fixture = (here / "combined_fixture.c").read_text(encoding="ascii")
unit = sources[0].read_text(encoding="utf-8") + "\n" + fixture
(out / "unit.c").write_text(unit, encoding="utf-8")
(out / "unit.sct").write_text(
    "LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { "
    "unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }",
    encoding="ascii",
)
files = ["unit.c", "stub_port.c", *(path.name for path in sources[1:])]
objects = [Path(name).with_suffix(".o").name for name in files]
commands = [[str(arm / "armcc.exe"), "--cpu", "Cortex-M4", "--fpu=SoftVFP", "--c99", "--gnu",
             "--split_sections", "-O1", "-I.", "-c", source, "-o", obj]
            for source, obj in zip(files, objects)]
commands.append([str(arm / "armlink.exe"), "--entry", "test_entry", "--scatter", "unit.sct",
                 *objects, "--library_type=microlib", "-o", "unit.axf"])
result = run(root, out, arm, unit, commands)
assert all(digest(path) == sha for path, sha in before.items())
result.update(
    combined=True,
    actual_modules_sha256={str(path.relative_to(root)): sha for path, sha in before.items()},
    harness_sha256={name: digest(here / name) for name in
                    ("run_combined.py", "combined_fixture.c", "combined_stm32_stub.h")},
    register_and_uart_loopback_stubs=True,
    hardware=False,
)
(out / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
