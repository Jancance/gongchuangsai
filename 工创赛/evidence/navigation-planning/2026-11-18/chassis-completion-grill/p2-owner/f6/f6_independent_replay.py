"""Inject launch and reply boundary conditions into actual ARMCC CAN source."""
import hashlib
from pathlib import Path
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = next(path for path in HERE.parents if (path / '.git').exists())
TMP = ROOT / '工创赛/tmp'
TMP.mkdir(exist_ok=True)
PRIOR = ROOT / '工创赛/scripts/check_emm_migration.py'
EXTRA = HERE / 'f6_independent_vectors.c'
FIRMWARE = ROOT / '3.stm32f407工程/CHASSIS'


def replace(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)


runner = PRIOR.read_text(encoding='utf-8')
runner = replace(runner, "'Cortex-M3','--c99'", "'Cortex-M4','--c99'")
runner = replace(runner, 'return test_tick++;', 'return test_tick;')
namespace = {'__name__': '__zone2_independent__', '__file__': str(PRIOR)}
exec(compile(runner, str(PRIOR), 'exec'), namespace)
unit = namespace['UNIT']
unit = replace(unit, '#include "can.c"', '#include "freertos_hooks.h"\n#include "can.c"')
unit = replace(unit, 'void test_entry(void){',
    'static void legacy_cases(void){')
unit = replace(unit, 'static uint32_t tx_count,',
    '''volatile uint32_t g_startup_extension_home_active,g_startup_extension_home_notice;
volatile uint32_t g_rtos_fault,g_safety_fault,g_startup_axes_state,g_startup_axes_vm_request;
static uint32_t inject, launch_time, launch_depth, first_visible_ms, tx_done_ms;
static uint32_t pending_calls, last_rx_tick, cancels, preload, force_rx_late;
static uint32_t tx_count,''')
unit = replace(unit, '(void)c;last_tx=*m;tx_count++;', '''(void)c;last_tx=*m;tx_count++;
 if(inject){launch_time=test_tick;launch_depth=test_depth;if(inject==4U)return CAN_TxStatus_NoMailBox;}''')
unit = replace(unit, 'return CAN_TxStatus_Ok;}','''if(inject && (int32_t)(test_tick-tx_done_ms)<0) return CAN_TxStatus_Pending;
 return CAN_TxStatus_Ok;}''')
unit = replace(unit, 'return rx_head<rx_count;}','''if(inject && !tx_count){
  if(inject==6U && !pending_calls++)test_tick+=3U;
  return preload?1U:0U;
 }
 if(inject && (int32_t)(test_tick-first_visible_ms)<0)return 0U;
 if(!inject && no_reply) test_tick++;
 return rx_head<rx_count;}''')
unit = replace(unit, '*q=rx_queue[rx_head++];}', '''if(inject && !tx_count && preload){
  *q=(CanRxMsg){0};
  q->ExtId=0x100U;q->IDE=CAN_Id_Extended;q->DLC=3U;
  q->Data[0]=0xf6U;q->Data[1]=0x02U;q->Data[2]=0x6bU;
  if(inject!=5U)preload--;
  return;
 }
 *q=rx_queue[rx_head++];last_rx_tick=test_tick;
 if(force_rx_late)test_tick++;
}''')
unit = replace(unit, 'void Test_CAN_CancelTransmit(CAN_TypeDef *c,uint8_t m){(void)c;(void)m;}',
    'void Test_CAN_CancelTransmit(CAN_TypeDef *c,uint8_t m){(void)c;(void)m;cancels++;}')
namespace['UNIT'] = unit + EXTRA.read_text(encoding='ascii')
namespace['FW'] = FIRMWARE
output = Path(tempfile.mkdtemp(prefix='f6_independent_', dir=TMP))
namespace['OUT'] = output
inputs = [PRIOR, EXTRA, Path(__file__), FIRMWARE / 'SYSTEM/CAN/can.c', FIRMWARE / 'SYSTEM/CAN/can.h']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


print(f'replay={output.name}', flush=True)
before = {path.name: sha(path) for path in inputs}
print(before, flush=True)
sys.argv = sys.argv[:1]
namespace['main']()
assert before == {path.name: sha(path) for path in inputs}
