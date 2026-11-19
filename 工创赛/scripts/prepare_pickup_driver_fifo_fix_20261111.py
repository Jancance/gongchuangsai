"""Isolate a read-only diagnostic receive fix. Never modify the flashed project."""
from pathlib import Path
import hashlib
import json
import shutil

R = Path(__file__).resolve().parents[1]
B = R / 'tmp/pickup_half_descent_20261110/project'
P = R / 'tmp/pickup_driver_fifo_fix_20261111/project'
E = R / 'evidence/integration/2026-11-11/driver-fifo-fix'


def main():
    assert not P.exists(), 'Review existing candidate instead of overwriting it'
    E.mkdir(parents=True, exist_ok=True)
    shutil.copytree(B, P, ignore=shutil.ignore_patterns('Objects', 'Listings'))
    rel = 'CHASSIS/SYSTEM/CAN/can.c'
    path = P / rel
    data = path.read_bytes()
    begin = data.index(b'uint32_t CAN1_ExtensionReadOnlyBlock')
    end = data.index(b'uint32_t StartupAxis_Exchange', begin)
    block = data[begin:end]
    needle = b'   vTaskDelay(pdMS_TO_TICKS(1U));'
    # Leave the bounded TX wait unchanged. A five-frame response must be drained
    # without a millisecond sleep between frames: bxCAN FIFO holds only three.
    assert block.count(needle) == 2
    cut = block.rindex(needle)
    block = block[:cut] + b'   /* Drain RX continuously; the existing50ms deadline remains in force. */' + block[cut + len(needle):]
    data = data[:begin] + block + data[end:]
    path.write_bytes(data)
    (E / 'scope.json').write_text(json.dumps({
        'base': B.relative_to(R).as_posix(), 'project': P.relative_to(R).as_posix(),
        'changed_file': rel, 'base_sha256': hashlib.sha256((B / rel).read_bytes()).hexdigest(),
        'new_sha256': hashlib.sha256(data).hexdigest(),
        'read_only_receive_change': True, 'rx_deadline_ms': 50, 'tx_deadline_ms': 10,
        'no_motor_parameter_or_motion_change': True, 'hardware_access': False,
        'deployed': False, 'physical_stall_resolved': False,
        'observed_failure': '0x42 and0x43 both stopped after22 assembled bytes; actual packet-loss cause remains to be confirmed on hardware.',
    }, indent=2) + '\n', encoding='utf-8')
    source = (R / 'scripts/check_extension_driver_config_20261110.py').read_text(encoding='utf-8')
    source = source.replace('tmp/pickup_half_descent_20261110/project', 'tmp/pickup_driver_fifo_fix_20261111/project')
    source = source.replace('2026-11-10/half-descent-driver', '2026-11-11/driver-fifo-fix')
    source = source.replace('static CanRxMsg queue[12];', 'static CanRxMsg queue[12];\nstatic uint32_t fifo_model,fifo_overruns;')
    source = source.replace('static uint32_t xTaskGetTickCount(void){return tick;}', 'static uint32_t xTaskGetTickCount(void){if(empty_reply)tick++;return tick;}')
    source = source.replace('static void vTaskDelay(uint32_t x){tick+=x;}', 'static void vTaskDelay(uint32_t x){if(fifo_model && rx_tail-rx_head>3){queue[rx_tail-2]=queue[rx_tail-1];rx_tail--;fifo_overruns++;}tick+=x;}')
    source = source.replace('tick-start==50 && n==0', 'tick-start>=50 && tick-start<=54 && n==0')
    original = (B / rel).read_bytes().decode('utf-8', errors='replace')
    original = original[original.index('uint32_t CAN1_ExtensionReadOnlyBlock'):original.index('uint32_t StartupAxis_Exchange', original.index('uint32_t CAN1_ExtensionReadOnlyBlock'))]
    original = original.replace('CAN1_ExtensionReadOnlyBlock(', 'Old_CAN1_ExtensionReadOnlyBlock(', 1)
    extra = r'''
 for(uint32_t cmd=0x42;cmd<=0x43;cmd++){
  fifo_model=1;fifo_overruns=0;
  CK(Old_CAN1_ExtensionReadOnlyBlock((uint8_t)cmd,raw,&n)==6 && n==22 && fifo_overruns==1);
  rx_head=rx_tail;fifo_overruns=0;
  CK(CAN1_ExtensionReadOnlyBlock((uint8_t)cmd,raw,&n)==1 && n==36 && fifo_overruns==0);
  CK(ExtDriver_FormatKnown((uint8_t)cmd,raw,n));
  fifo_model=0;
 }
'''
    marker = ' uint8_t commands[7]={0x1f,0x21,0x24,0x26,0x27,0x42,0x43};'
    assert source.count(marker) == 1
    source = source.replace(marker, extra + marker, 1)
    source = source.replace('c.write_text(prefix+reader+test,', 'c.write_text(prefix+' + repr(original) + '+reader+test,', 1)
    compile(source, 'driver-fifo-fix-test', 'exec')
    (R / 'scripts/check_pickup_driver_fifo_fix_20261111.py').write_text(source, encoding='utf-8')
    print('Prepared isolated diagnostic-only fix and bounded-FIFO regression; no device access.')


if __name__ == '__main__':
    main()
