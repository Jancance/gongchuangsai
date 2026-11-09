from pathlib import Path
import shutil,json,hashlib,subprocess,re,sys
R=Path(__file__).resolve().parents[1]; A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程'); S=R/'tmp/arm_x_20261101/project'; E=R/'evidence/p2-arm/2026-11-01/arm-x'; E.mkdir(parents=True,exist_ok=True)
F=['SYSTEM/CAN/can.c','SYSTEM/CAN/can.h','SYSTEM/CAN/arm_x.h','HARDWARE/LIFT/lift.c','HARDWARE/LIFT/lift.h','HARDWARE/LIFT/lift_config.h','HARDWARE/LIFT/lift_protocol.c','HARDWARE/LIFT/lift_protocol.h','HARDWARE/LIFT/lift_home.c','HARDWARE/EXTENSION/extension.c','HARDWARE/EXTENSION/extension.h','HARDWARE/EXTENSION/extension_config.h','HARDWARE/EXTENSION/extension_home.c']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(m): (E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
def build(d,n):
 log=E/n
 z=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(d/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],capture_output=True,timeout=180)
 t=log.read_text(encoding='utf-8',errors='replace');assert z.returncode==0 and '0 Error(s), 0 Warning(s)' in t,t[-3000:]
 print(t[-450:],flush=True)
if '--install' in sys.argv:
 m=json.loads((E/'manifest.json').read_text(encoding='utf-8'));assert m['host_pass'] and m['stage_build_pass']
 assert not m.get('legacy_mstep_values_pending', True), 'Actual old MStep values are required before installing the position-unit conversion.'
 for f in F:assert ((not (A/f).exists()) if m['before'][f] is None else sha(A/f)==m['before'][f]) and sha(S/f)==m['after'][f]
 for f,h in m['protected'].items():assert sha(A/f)==h,f
 for f in F:
  (A/f).parent.mkdir(parents=True,exist_ok=True);shutil.copy2(S/f,A/f)
 m['installed']=True;save(m);build(A,'actual-build.log')
 for f,h in m['protected'].items():assert sha(A/f)==h,f
 m['actual_build_pass']=True;save(m);print('Installed and built; no flash or hardware commands.');sys.exit()




assert not S.exists()
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','Listings','*.uvguix.*','.git'))
m={'before':{f:sha(A/f) if (A/f).exists() else None for f in F},'protected':{str(p.relative_to(A)).replace(chr(92),'/'):sha(p) for p in A.rglob('*') if p.is_file() and p.suffix.lower() in ['.c','.h','.uvprojx'] and str(p.relative_to(A)).replace(chr(92),'/') not in F},'host_pass':False,'stage_build_pass':False,'installed':False,'actual_build_pass':False,'flash_performed':False,'physical_acceptance':False}
for f in F:
 if not (A/f).exists():continue
 b=E/'original'/f;b.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(A/f,b)
def once(t,a,b):assert t.count(a)==1,a[:100];return t.replace(a,b,1)
def edit(f,fn):
 p=S/f;t=p.read_bytes().decode('utf-8').replace('\r\n','\n');p.write_bytes(fn(t).replace('\n','\r\n').encode('utf-8'))

header=r"""#ifndef ARM_X_H
#define ARM_X_H
#include <stdint.h>
#include <string.h>
typedef struct { uint32_t pulses_per_turn, units_per_turn, valid; } ArmXEncoding;
/* 保留旧Emm脉冲输入的机械行程；细分取切换前实读值，不假定16。 */
static __inline uint32_t ArmX_Configure(uint8_t options, uint32_t legacy_mstep, ArmXEncoding *out)
{
    if (!out) return 0U;
    memset(out, 0, sizeof(*out));
    if ((options & 2U) || legacy_mstep == 0U || legacy_mstep > 256U) return 0U;
    out->pulses_per_turn = legacy_mstep * ((options & 1U) ? 400U : 200U);
    out->units_per_turn = (options & 0x80U) ? 36000U : 3600U;
    out->valid = 1U;
    return 1U;
}
static __inline uint32_t ArmX_AngleUnits(const ArmXEncoding *e, uint32_t pulses, uint32_t *units)
{
    uint64_t v;
    if (!e || !units || !e->valid || e->pulses_per_turn == 0U || pulses == 0U) return 0U;
    v = ((uint64_t)pulses * e->units_per_turn + e->pulses_per_turn / 2U) / e->pulses_per_turn;
    if (v == 0U || v > 0xFFFFFFFFULL) return 0U;
    *units = (uint32_t)v; return 1U;
}
static __inline uint32_t ArmX_BuildMove(const ArmXEncoding *e, uint8_t direction,
    uint32_t pulses, uint32_t max_pulses, uint16_t speed_deci_rpm,
    uint16_t acceleration, uint16_t deceleration, uint8_t out[15])
{
    uint32_t angle;
    if (!out || direction > 1U || pulses > max_pulses ||
        !speed_deci_rpm || speed_deci_rpm > 30000U || !acceleration || !deceleration ||
        !ArmX_AngleUnits(e, pulses, &angle)) return 0U;
    out[0]=0xFDU;out[1]=direction;
    out[2]=(uint8_t)(acceleration>>8);out[3]=(uint8_t)acceleration;
    out[4]=(uint8_t)(deceleration>>8);out[5]=(uint8_t)deceleration;
    out[6]=(uint8_t)(speed_deci_rpm>>8);out[7]=(uint8_t)speed_deci_rpm;
    out[8]=(uint8_t)(angle>>24);out[9]=(uint8_t)(angle>>16);out[10]=(uint8_t)(angle>>8);out[11]=(uint8_t)angle;
    out[12]=2U;out[13]=0U;out[14]=0x6BU;
    return 1U;
}
#endif
"""
(S/'SYSTEM/CAN/arm_x.h').write_text(header,encoding='utf-8')
# New adapter declarations. Existing chassis API bodies are preserved byte-for-byte.
p=S/'SYSTEM/CAN/can.h';data=p.read_bytes();marker=b'#endif';addition=b'\r\n#include "arm_x.h"\r\nuint32_t CAN1_ArmXConfigure(uint8_t address, uint8_t options, uint32_t legacy_mstep);\r\nuint32_t CAN1_ArmXBuildMove(uint8_t address, uint8_t direction, uint32_t pulses, uint16_t speed_deci_rpm, uint16_t acceleration, uint16_t deceleration, uint8_t out[15]);\r\n'
assert data.count(marker)==1;p.write_bytes(data.replace(marker,addition+marker,1))
p=S/'SYSTEM/CAN/can.c';t=p.read_bytes().decode('latin1');newline='\r\n' if '\r\n' in t else '\n';t=t.replace('\r\n','\n')
a=t.index('uint32_t CAN1_LiftExchange(');b=t.index('uint32_t CAN1_ReadMotorOptions(',a);old=t[a:b]
fn=old[:old.rindex('}\n')+2]
fn=fn.replace('uint32_t CAN1_LiftExchange(const uint8_t *request, uint8_t length, CanRxMsg *reply)','static uint32_t CAN1_ArmXExchange(uint8_t address, const uint8_t *request, uint8_t length, CanRxMsg *reply)')
assert 'static uint32_t CAN1_ArmXExchange' in fn
fn=fn.replace('uint32_t result, pulses, speed_rpm;','uint32_t result, angle, max_angle, speed_raw, maximum_pulses;\n    const ArmXEncoding *encoding;')
fn=once(fn,'    if (APP_ARM_LIFT_TEST != 1U) return 9U;','    if (APP_ARM_LIFT_TEST != 1U) return 9U;\n    if (address != 5U && address != 6U) return 7U;')
fn=fn.replace('CAN1_ExchangeSingle(5U,','CAN1_ExchangeSingle(address,')
start=fn.index('    /* RPM is a two-byte field;');end=fn.index('    memset(reply, 0, sizeof(*reply));',start)
check=r"""    encoding = &s_arm_x_encoding[address - 5U];
    if (length != 15U || request[0] != 0xFDU || request[1] > 1U ||
        request[12] != 2U || request[13] != 0U || request[14] != 0x6BU || !encoding->valid) return 7U;
    speed_raw = ((uint32_t)request[6] << 8) | request[7];
    if (speed_raw == 0U || speed_raw > (address == 5U ? LIFT_X_SPEED_DECI_RPM : EXTENSION_X_SPEED_DECI_RPM) ||
        (request[2] == 0U && request[3] == 0U) || (request[4] == 0U && request[5] == 0U)) return 7U;
    angle = ((uint32_t)request[8] << 24) | ((uint32_t)request[9] << 16) |
            ((uint32_t)request[10] << 8) | request[11];
    maximum_pulses = address == 5U ? (request[1] ? LIFT_UP_MAX_PULSES : LIFT_DOWN_MAX_PULSES) : EXTENSION_JOG_PULSES;
    if (!ArmX_AngleUnits(encoding, maximum_pulses, &max_angle) || !angle || angle > max_angle) return 7U;
"""
fn=fn[:start]+check+fn[end:];fn=fn.replace('0x0500U | packet','((uint32_t)address << 8) | packet').replace('rx.ExtId == 0x0500U','rx.ExtId == ((uint32_t)address << 8)')
prefix=r"""static ArmXEncoding s_arm_x_encoding[2];
uint32_t CAN1_ArmXConfigure(uint8_t address, uint8_t options, uint32_t legacy_mstep)
{
    if (address != 5U && address != 6U) return 0U;
    return ArmX_Configure(options, legacy_mstep, &s_arm_x_encoding[address - 5U]);
}
uint32_t CAN1_ArmXBuildMove(uint8_t address, uint8_t direction, uint32_t pulses,
                         uint16_t speed_deci_rpm, uint16_t acceleration,
                         uint16_t deceleration, uint8_t out[15])
{
    uint32_t maximum;
    if (address != 5U && address != 6U) return 0U;
    maximum = address == 5U ? (direction ? LIFT_UP_MAX_PULSES : LIFT_DOWN_MAX_PULSES) : EXTENSION_JOG_PULSES;
    return ArmX_BuildMove(&s_arm_x_encoding[address - 5U], direction, pulses, maximum,
                         speed_deci_rpm, acceleration, deceleration, out);
}
"""
wrappers='\nuint32_t CAN1_LiftExchange(const uint8_t *request, uint8_t length, CanRxMsg *reply)\n{ return CAN1_ArmXExchange(5U, request, length, reply); }\n\n'
t=t[:a]+prefix+fn+wrappers+t[b:]
a=t.index('uint32_t CAN1_ExtensionExchange(');b=t.index('uint32_t CAN1_ReadExtensionHomeParameters(',a)
t=t[:a]+'uint32_t CAN1_ExtensionExchange(const uint8_t *request, uint8_t length, CanRxMsg *reply)\n{ return CAN1_ArmXExchange(6U, request, length, reply); }\n\n'+t[b:]
p.write_bytes(t.replace('\n',newline).encode('latin1'))
# Explicit X speeds match the reference's encoded values; legacy MStep is supplied from actual motors.
for file,prefix,speed in [(F[5],'LIFT',18464),(F[11],'EXTENSION',1060)]:
 edit(file,lambda t,prefix=prefix,speed=speed:t.replace('#endif',f'/* X模式单位：速度0.1RPM，加/减速度RPM/s。旧细分须实读后填入。 */\n#define {prefix}_X_SPEED_DECI_RPM {speed}U\n#define {prefix}_X_ACCEL_RPM_S 45055U\n#define {prefix}_X_DECEL_RPM_S 45055U\n#define {prefix}_X_LEGACY_MSTEP 0U\n\n#endif'))
# Lift builder uses the same address5 software encoding cache configured before motion.
p=S/F[6];p.write_text("""#include "lift_protocol.h"
#include "lift_config.h"
#include "can.h"
uint8_t LiftProtocol_Move(uint8_t direction, uint32_t pulses, uint8_t out[15])
{
    return (uint8_t)CAN1_ArmXBuildMove(5U, direction, pulses, LIFT_X_SPEED_DECI_RPM,
                                     LIFT_X_ACCEL_RPM_S, LIFT_X_DECEL_RPM_S, out);
}
""",encoding='utf-8')
edit(F[7],lambda t:t.replace('out[12]','out[15]'))
def lift(t):
 t=once(t,'volatile int32_t g_lift_speed_rpm = 0;','volatile int32_t g_lift_speed_rpm = 0;\nvolatile int32_t g_lift_speed_deci_rpm = 0;\nvolatile uint32_t g_lift_x_legacy_mstep = LIFT_X_LEGACY_MSTEP;')
 t=t.replace('uint8_t command[12];','uint8_t command[15];')
 t=once(t,'if (g_lift_options_result != 1U || (g_lift_options_reply.Data[2] & 0x02U) == 0U)', 'if (g_lift_options_result != 1U || (g_lift_options_reply.Data[2] & 0x02U) != 0U ||\n            !CAN1_ArmXConfigure(5U, g_lift_options_reply.Data[2], g_lift_x_legacy_mstep))')
 t=t.replace('command,12U,','command,15U,').replace('g_lift_speed_rpm=speed;','g_lift_speed_deci_rpm=speed;g_lift_speed_rpm=speed/10;').replace('(delta >= 2 || delta <= -2)','(delta >= 1 || delta <= -1)')
 t=t.replace('/* 当前选项必须确为Emm，不依赖固件版本号猜格式。 */','/* 当前选项必须为X，按切换前实读细分保持旧脉冲对应行程。 */')
 return t
edit(F[3],lift)
edit(F[4],lambda t:t.replace('#endif','/* X模式速度原始0.1RPM计数；旧rpm字段为整数显示，delta_raw为X角度反馈单位。 */\nextern volatile int32_t g_lift_speed_deci_rpm;\nextern volatile uint32_t g_lift_x_legacy_mstep;\n#endif'))
def extension(t):
 t=once(t,'volatile int32_t g_extension_speed_rpm = 0;','volatile int32_t g_extension_speed_rpm = 0;\nvolatile int32_t g_extension_speed_deci_rpm = 0;\nvolatile uint32_t g_extension_x_legacy_mstep = EXTENSION_X_LEGACY_MSTEP;')
 a=t.index('static void Extension_BuildMove(');b=t.index('void Extension_MotionService(',a)
 t=t[:a]+"""static uint32_t Extension_BuildMove(uint8_t out[15])
{
    return CAN1_ArmXBuildMove(6U, (uint8_t)g_extension_motion_direction, s_extension_target_pulses,
                              EXTENSION_X_SPEED_DECI_RPM, EXTENSION_X_ACCEL_RPM_S,
                              EXTENSION_X_DECEL_RPM_S, out);
}

"""+t[b:]
 t=t.replace('uint8_t command[12],','uint8_t command[15],')
 t=once(t,'(g_extension_motion_reply.Data[2] & 0x02U) == 0U)', '(g_extension_motion_reply.Data[2] & 0x02U) != 0U ||\n            !CAN1_ArmXConfigure(6U, g_extension_motion_reply.Data[2], g_extension_x_legacy_mstep))')
 t=t.replace('        Extension_BuildMove(command);','        if (!Extension_BuildMove(command)) { Extension_MotionFault(4U); break; }')
 t=t.replace('command, 12U,','command, 15U,').replace('g_extension_speed_rpm = speed;','g_extension_speed_deci_rpm = speed; g_extension_speed_rpm = speed / 10;').replace('(delta >= 2 || delta <= -2)','(delta >= 1 || delta <= -1)')
 return t
edit(F[9],extension)
edit(F[10],lambda t:t.replace('#endif','/* X模式速度单位0.1RPM；legacy_mstep用于保持旧位移输入行程。 */\nextern volatile int32_t g_extension_speed_deci_rpm;\nextern volatile uint32_t g_extension_x_legacy_mstep;\n#endif'))
# Both firmware families share homing frames; retain actual speed/current/direction settings, change only firmware identity expectation.
edit(F[8],lambda t:once(t,'(s_reply.Data[2] & 2U) == 0U','(s_reply.Data[2] & 2U) != 0U'))
edit(F[12],lambda t:once(t,'(s_reply.Data[2] & 2U) == 0U','(s_reply.Data[2] & 2U) != 0U'))
m['after']={f:sha(S/f) for f in F};m['legacy_mstep_values_pending']=True;save(m)
print('Arm-only X implementation staged; legacy MStep inputs pending, no actual source install yet.',flush=True)
