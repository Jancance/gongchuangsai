from pathlib import Path
import subprocess


WORK = Path(__file__).parent
ROOT = WORK / 'stage3_directhandoff_20261120/project'
USER = ROOT / 'CHASSIS/USER'
DM = ROOT / 'HARDWARE/DM_CAN2'
BUILD = WORK / (ROOT.parent.name+'_lower_fixture_build')
BUILD.mkdir(exist_ok=True)
(BUILD / 'stm32f4xx_can.h').write_text('''#pragma once
#include <stdint.h>
typedef struct {uint8_t Data[8];} CanRxMsg;
typedef struct {uint8_t Data[8];} CanTxMsg;
typedef struct {uint32_t ESR;} FixtureCAN;
extern FixtureCAN fixture_can2;
#define CAN2 (&fixture_can2)
#define CAN_ESR_BOFF 1U
#define CAN_FIFO0 0U
#define CAN_FLAG_FOV0 0U
#define CAN_TxStatus_Failed 0U
#define CAN_TxStatus_Ok 1U
#define CAN_TxStatus_NoMailBox 4U
#define RESET 0U
uint8_t CAN_Transmit(FixtureCAN *,CanTxMsg *);
uint8_t CAN_TransmitStatus(FixtureCAN *,uint8_t);
void CAN_CancelTransmit(FixtureCAN *,uint8_t);
void CAN_Receive(FixtureCAN *,uint8_t,CanRxMsg *);
uint8_t CAN_MessagePending(FixtureCAN *,uint8_t);
uint8_t CAN_GetFlagStatus(FixtureCAN *,uint32_t);
void CAN_ClearFlag(FixtureCAN *,uint32_t);
''', encoding='ascii')
motion = (DM / 'dm_can2_motion.c').read_text(encoding='gbk')
(BUILD / 'fixture_dm_motion.inc').write_text('''uint32_t DM_CAN2_StartupCwDelta(float,float,float *);
float SourcePrep_Duration(float,float,float);
float SourcePrep_Smooth(float);
''' + '\n'.join(line for line in motion.splitlines() if not line.startswith('#include')), encoding='utf-8')
exe = BUILD / 'stage3_directhandoff_fixture.exe'
handoff = 'SPF_CMD_STAGE3_EMPTY_HANDOFF' in (USER/'source_parallel_flow.h').read_text(encoding='gbk')
carry = 'SPF_CMD_STAGE3_CARRY_RISE' in (USER/'source_parallel_flow.h').read_text(encoding='gbk')
originfix = 'VSP_CarryOriginWithin' in (USER/'vehicle_source_parallel.inc').read_text(encoding='gbk')
risefix = b'SPF_STAGE3_CARRY_RISE_LIFT_RPM' in (USER/'source_parallel_flow.h').read_bytes()
settlefix = b'g_vsp_carry_wait_speed_count' in (USER/'vehicle_source_parallel.inc').read_bytes()
subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Wno-misleading-indentation',
                '-Wno-unused-function', '-Wno-unused-variable', f'-DTEST_EMPTY_HANDOFF={int(handoff)}', '-I', str(BUILD),
                f'-DTEST_CARRY_PLACE={int(carry)}', f'-DTEST_ORIGIN_FIX={int(originfix)}', f'-DTEST_RISE_FIX={int(risefix)}',
                f'-DTEST_SETTLE_FIX={int(settlefix)}',
                '-I', str(USER), '-I', str(DM), str(WORK/'stage3_directhandoff_fixture.c'),
                str(USER/'source_parallel_flow.c'), '-lm', '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
