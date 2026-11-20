"""Exercise the linked CAN implementation with timed bxCAN and RTOS stubs."""
from pathlib import Path

HERE = Path(__file__).resolve()
BASE = HERE.parent.parent / 'f6/f6_independent_replay.py'
VECTORS = HERE.with_name('can_reads_vectors.c')
runner = BASE.read_text(encoding='utf-8')


def change(old, new):
    global runner
    assert runner.count(old) == 1, old
    runner = runner.replace(old, new)


change("output = Path(tempfile.mkdtemp(prefix='f6_independent_', dir=TMP))",
       "output = Path(tempfile.mkdtemp(prefix='can_reads_arm_', dir=TMP))")
change("namespace['UNIT'] = unit + EXTRA.read_text(encoding='ascii')", """
unit = replace(unit, 'static uint32_t pending_calls,', '''static uint32_t read_case, read_visible[8];
static void setup_read_frames(uint8_t command);
static uint32_t pending_calls,''')
unit = replace(unit, '}return 0;\\n}', '}if(read_case)setup_read_frames(f);return 0;\\n}')
unit = replace(unit, 'if(inject && (int32_t)(test_tick-tx_done_ms)<0) return CAN_TxStatus_Pending;',
    '''if(inject && (int32_t)(test_tick-tx_done_ms)<0){
        if(read_case && last_tx.Data[0]==0x43U)test_tick++;
        return CAN_TxStatus_Pending;
    }''')
unit = replace(unit, 'if(inject && (int32_t)(test_tick-first_visible_ms)<0)return 0U;',
    '''if(read_case && tx_count){
        if(rx_head<rx_count && (int32_t)(test_tick-read_visible[rx_head])>=0)return 1U;
        if(last_tx.Data[0]==0x43U)test_tick++;
        return 0U;
    }
    if(inject && (int32_t)(test_tick-first_visible_ms)<0)return 0U;''')
unit = replace(unit, 'if(force_rx_late)test_tick++;',
    'if(force_rx_late && (!read_case || rx_head==rx_count))test_tick++;')
f6_vectors = EXTRA.read_text(encoding='ascii').replace('void test_entry(void)', 'static void speed_cases(void)')
namespace['UNIT'] = unit + f6_vectors + VECTORS.read_text(encoding='ascii')
""")
change("inputs = [PRIOR, EXTRA, Path(__file__),", "inputs = [PRIOR, EXTRA, BASE, VECTORS, READS_RUNNER,")
exec(compile(runner, str(BASE), 'exec'), {
    '__file__': str(BASE), 'BASE': BASE, 'VECTORS': VECTORS, 'READS_RUNNER': HERE,
})
