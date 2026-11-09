"""Prepare bounded node5 Emm relative-position control in isolated arm mode."""
from pathlib import Path
import hashlib,json,re,shutil,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE=ROOT/'tmp/lift_emm_jog_20261025/project'
EV=ROOT/'evidence/p2-arm/2026-10-25/lift-emm-jog'

PROTO_H='''#ifndef LIFT_PROTOCOL_H
#define LIFT_PROTOCOL_H
#include <stdint.h>
uint8_t LiftProtocol_Move(uint8_t direction, uint32_t pulses, uint8_t out[12]);
#endif
'''
PROTO_C='''#include "lift_protocol.h"
#include "lift_config.h"

/* Emm位置帧不含节点地址；mode2表示相对当前真实位置，不沿用旧目标。 */
uint8_t LiftProtocol_Move(uint8_t direction, uint32_t pulses, uint8_t out[12])
{
    if (out == 0 || direction > 1U || pulses == 0U ||
        pulses > LIFT_JOG_PULSES || LIFT_JOG_RPM == 0U || LIFT_JOG_RPM > 5U)
    {
        return 0U;
    }
    out[0] = 0xFDU; out[1] = direction;
    out[2] = (uint8_t)(LIFT_JOG_RPM >> 8);
    out[3] = (uint8_t)LIFT_JOG_RPM;
    out[4] = LIFT_JOG_ACCELERATION;
    out[5] = (uint8_t)(pulses >> 24);
    out[6] = (uint8_t)(pulses >> 16);
    out[7] = (uint8_t)(pulses >> 8);
    out[8] = (uint8_t)pulses;
    out[9] = 2U; out[10] = 0U; out[11] = 0x6BU;
    return 1U;
}
'''

TRANSPORT='''
/* 专供隔离升降模式，固定节点5；不放开原底盘控制接口的地址保护。 */
uint32_t CAN1_LiftExchange(const uint8_t *request, uint8_t length, CanRxMsg *reply)
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t started;
    uint32_t result, pulses;
    uint8_t offset, packet, chunk, mailbox, status, flush;

    if (APP_ARM_LIFT_TEST != 1U) return 9U;
    if (request == NULL || reply == NULL) return 7U;
    /* 只允许锁轴、停止、有限相对位置；禁止松轴、回零和参数修改。 */
    if (length == 5U && request[0] == 0xF3U && request[1] == 0xABU &&
        request[2] == 1U && request[3] == 0U && request[4] == 0x6BU)
    {
        result = CAN1_ExchangeSingle(5U, request, length, 3U, reply);
        return (result == 1U && reply->Data[1] != 0x02U) ? 8U : result;
    }
    if (length == 4U && request[0] == 0xFEU && request[1] == 0x98U &&
        request[2] == 0U && request[3] == 0x6BU)
    {
        result = CAN1_ExchangeSingle(5U, request, length, 3U, reply);
        return (result == 1U && reply->Data[1] != 0x02U) ? 8U : result;
    }
    if (length != 12U || request[0] != 0xFDU || request[1] > 1U ||
        request[2] != 0U || request[3] == 0U || request[3] > 5U ||
        request[4] == 0U || request[9] != 2U || request[10] != 0U ||
        request[11] != 0x6BU) return 7U;
    pulses = ((uint32_t)request[5] << 24) | ((uint32_t)request[6] << 16) |
             ((uint32_t)request[7] << 8) | request[8];
    if (pulses == 0U || pulses > 16U) return 7U;
    memset(reply, 0, sizeof(*reply));
    if (s_can1_normal_ready == 0U)
    {
        result = CAN1_InitMode(CAN_Mode_Normal);
        if (result != 1U) return result;
        s_can1_normal_ready = 1U;
    }
    for (flush = 0U; flush < 3U && CAN_MessagePending(CAN1, CAN_FIFO0); flush++)
        CAN_Receive(CAN1, CAN_FIFO0, &rx);
    offset = 1U; packet = 0U;
    while (offset < length)
    {
        memset(&tx, 0, sizeof(tx));
        chunk = (uint8_t)(length - offset);
        if (chunk > 7U) chunk = 7U;
        tx.ExtId = 0x0500U | packet;
        tx.IDE = CAN_Id_Extended; tx.RTR = CAN_RTR_Data;
        tx.DLC = (uint8_t)(chunk + 1U); tx.Data[0] = request[0];
        memcpy(&tx.Data[1], &request[offset], chunk);
        mailbox = CAN_Transmit(CAN1, &tx);
        if (mailbox == CAN_TxStatus_NoMailBox) return 4U;
        started = xTaskGetTickCount();
        for (;;)
        {
            status = CAN_TransmitStatus(CAN1, mailbox);
            if (status == CAN_TxStatus_Ok) break;
            if (status == CAN_TxStatus_Failed ||
                (TickType_t)(xTaskGetTickCount() - started) >=
                pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS))
            {
                CAN_CancelTransmit(CAN1, mailbox);
                return 4U;
            }
            vTaskDelay(pdMS_TO_TICKS(1U));
        }
        offset = (uint8_t)(offset + chunk); packet++;
    }
    started = xTaskGetTickCount();
    while ((TickType_t)(xTaskGetTickCount() - started) <
           pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS))
    {
        if (CAN_MessagePending(CAN1, CAN_FIFO0))
        {
            CAN_Receive(CAN1, CAN_FIFO0, &rx);
            if (rx.IDE == CAN_Id_Extended && rx.RTR == CAN_RTR_Data &&
                rx.ExtId == 0x0500U && rx.DLC != 0U && rx.Data[0] == 0xFDU)
            {
                *reply = rx;
                if (rx.DLC != 3U || rx.Data[2] != 0x6BU) return 6U;
                /* 接收确认或到位通知都不能替代后续真实位置/静止反馈。 */
                return (rx.Data[1] == 0x02U || rx.Data[1] == 0x9FU) ? 1U : 8U;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return 5U;
}

uint32_t CAN1_ReadMotorOptions(uint8_t address, CanRxMsg *reply)
{
    return CAN1_ReadParameter(address, 0x1AU, 3U, reply);
}
'''

MOTION='''
/* 所有下发仍由现有唯一CAN任务串行执行。 */
volatile uint32_t g_lift_motion_request = 0U;
volatile uint32_t g_lift_motion_state = 0U;
volatile uint32_t g_lift_motion_result = 0U;
volatile uint32_t g_lift_enable_result = 0U;
volatile uint32_t g_lift_move_result = 0U;
volatile uint32_t g_lift_stop_result = 0U;
volatile uint32_t g_lift_flags = 0U;
volatile int32_t g_lift_speed_rpm = 0;
volatile int64_t g_lift_delta_raw = 0;
volatile uint32_t g_lift_can_esr = 0U;
volatile uint32_t g_lift_options_result = 0U;
CanRxMsg g_lift_control_reply;
static CanRxMsg s_motion_reply;
static uint8_t s_direction = 0U, s_position_sign = 0U;
static uint32_t s_position_raw = 0U;
static int64_t s_before_position = 0;
static TickType_t s_motion_started = 0U, s_poll_tick = 0U;
static uint8_t s_poll_phase = 0U;

static uint32_t Lift_StopCommand(void)
{
    const uint8_t stop[4] = {0xFEU, 0x98U, 0U, 0x6BU};
    g_lift_stop_result = CAN1_LiftExchange(stop, 4U, &g_lift_control_reply);
    g_lift_can_esr = CAN1->ESR;
    return g_lift_stop_result;
}

static void Lift_MotionFault(uint32_t reason)
{
    (void)Lift_StopCommand();
    g_lift_motion_result = reason;
    g_lift_motion_state = 9U;
}

static int64_t Lift_SignedPosition(uint8_t sign, uint32_t raw)
{
    return sign ? -(int64_t)raw : (int64_t)raw;
}

static void Lift_MotionService(TickType_t now)
{
    uint32_t request, result;
    uint8_t command[12];
    int64_t delta;
    request = g_lift_motion_request;
    g_lift_motion_request = 0U;
    if (request == 3U)
    {
        result = Lift_StopCommand();
        g_lift_motion_result = (result == 1U) ? 2U : 4U;
        g_lift_motion_state = (result == 1U) ? 8U : 9U;
        return;
    }
    /* 失败后须先明确停止或重新建立供电/身份会话。 */
    if (g_lift_motion_state == 9U) return;
    if ((request == 1U || request == 2U) &&
        !(g_lift_motion_state >= 1U && g_lift_motion_state <= 6U))
    {
        s_direction = (request == 1U) ? 0U : 1U;
        g_lift_motion_result = 0U;
        g_lift_enable_result = 0U; g_lift_move_result = 0U;
        g_lift_stop_result = 0U; g_lift_delta_raw = 0;
        g_lift_motion_state = 1U;
        s_motion_started = now;
    }
    /* 忙时请求不排队、不重启；继续监测原动作的反馈与超时。 */
    if (g_lift_motion_state < 1U || g_lift_motion_state > 6U) return;
    if ((TickType_t)(now - s_motion_started) >= pdMS_TO_TICKS(LIFT_MOVE_TIMEOUT_MS))
    {
        Lift_MotionFault(3U);
        return;
    }
    switch (g_lift_motion_state)
    {
    case 1U:
        g_lift_options_result = CAN1_ReadMotorOptions(LIFT_CAN_ADDRESS, &s_motion_reply);
        if (g_lift_options_result != 1U || (s_motion_reply.Data[1] & 0x02U) == 0U)
        {
            /* 当前选项必须确为Emm，不依赖固件版本号猜格式。 */
            g_lift_motion_result = 4U; g_lift_motion_state = 9U;
        }
        else g_lift_motion_state = 2U;
        break;
    case 2U:
        result = CAN1_ReadStatus(LIFT_CAN_ADDRESS, &s_motion_reply);
        if (result != 1U) { Lift_MotionFault(4U); break; }
        g_lift_flags = s_motion_reply.Data[1];
        if ((g_lift_flags & 0x0CU) != 0U) { Lift_MotionFault(5U); break; }
        g_lift_motion_state = (g_lift_flags & 1U) ? 4U : 3U;
        break;
    case 3U:
        command[0]=0xF3U; command[1]=0xABU; command[2]=1U;
        command[3]=0U; command[4]=0x6BU;
        g_lift_enable_result=CAN1_LiftExchange(command,5U,&g_lift_control_reply);
        g_lift_can_esr=CAN1->ESR;
        if (g_lift_enable_result != 1U) { Lift_MotionFault(4U); break; }
        g_lift_motion_state=4U;
        break;
    case 4U:
        result=CAN1_ReadPosition(LIFT_CAN_ADDRESS,&s_motion_reply,
                                &s_position_sign,&s_position_raw);
        if (result != 1U) { Lift_MotionFault(4U); break; }
        s_before_position=Lift_SignedPosition(s_position_sign,s_position_raw);
        g_lift_motion_state=5U;
        break;
    case 5U:
        if (!LiftProtocol_Move(s_direction,LIFT_JOG_PULSES,command))
        { Lift_MotionFault(4U); break; }
        g_lift_move_result=CAN1_LiftExchange(command,12U,&g_lift_control_reply);
        g_lift_can_esr=CAN1->ESR;
        if (g_lift_move_result != 1U) { Lift_MotionFault(4U); break; }
        g_lift_motion_state=6U; s_poll_phase=0U;
        s_poll_tick=xTaskGetTickCount();
        break;
    case 6U:
        if ((TickType_t)(now-s_poll_tick) < pdMS_TO_TICKS(LIFT_POLL_MS)) break;
        if (s_poll_phase == 0U)
        {
            result=CAN1_ReadStatus(LIFT_CAN_ADDRESS,&s_motion_reply);
            if (result != 1U) { Lift_MotionFault(4U); break; }
            g_lift_flags=s_motion_reply.Data[1];
            if ((g_lift_flags & 0x0CU) || !(g_lift_flags & 1U))
            { Lift_MotionFault(5U); break; }
            s_poll_phase=1U;
        }
        else if (s_poll_phase == 1U)
        {
            result=CAN1_ReadPosition(LIFT_CAN_ADDRESS,&s_motion_reply,
                                    &s_position_sign,&s_position_raw);
            if (result != 1U) { Lift_MotionFault(4U); break; }
            g_lift_delta_raw=Lift_SignedPosition(s_position_sign,s_position_raw)-s_before_position;
            s_poll_phase=2U;
        }
        else
        {
            int32_t speed=0;
            result=CAN1_ReadSpeed(LIFT_CAN_ADDRESS,&s_motion_reply,&speed);
            if (result != 1U) { Lift_MotionFault(4U); break; }
            g_lift_speed_rpm=speed;
            delta=g_lift_delta_raw;
            if ((g_lift_flags & 2U) && speed == 0 && (delta >= 2 || delta <= -2))
            {
                /* 新位置变化+到位+静止反馈；不以02确认包当完成。 */
                g_lift_motion_result=1U; g_lift_motion_state=7U;
            }
            s_poll_phase=0U; s_poll_tick=now;
        }
        break;
    default: break;
    }
}
'''

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def one(data,old,new):
    assert data.count(old)==1,old[:70]
    return data.replace(old,new)

def prepare():
    assert not STAGE.exists()
    shutil.copytree(ACTUAL,STAGE,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
    EV.mkdir(parents=True,exist_ok=True)
    p=STAGE/'USER/app_config.h';d=p.read_bytes();d=one(d,b'#define APP_ARM_MENU_POWER_TEST 1U',b'#define APP_ARM_MENU_POWER_TEST 0U\n#define APP_ARM_LIFT_TEST 1U');d=one(d,b'#if APP_ARM_MENU_POWER_TEST && (',b'#if (APP_ARM_MENU_POWER_TEST || APP_ARM_LIFT_TEST) && (');d+=b'\n#if APP_ARM_MENU_POWER_TEST && APP_ARM_LIFT_TEST\n#error Arm_power_modes_must_be_exclusive\n#endif\n';p.write_bytes(d)
    p=STAGE/'USER/arm_power_mode.h';d=p.read_bytes();d=one(d,b'uint32_t ArmPower_IsMenuMode(void);',b'uint32_t ArmPower_IsMenuMode(void);\nuint32_t ArmPower_IsIsolatedMode(void);');p.write_bytes(d)
    p=STAGE/'USER/arm_power_mode.c';s=p.read_text(encoding='utf-8');marker='/* 仅监督任务调用：单次最多五分钟，关请求后才能重新开始。 */';assert marker in s
    s=s.replace(marker,'''uint32_t ArmPower_IsIsolatedMode(void)
{
    return (APP_ARM_MENU_POWER_TEST || APP_ARM_LIFT_TEST) ? 1U : 0U;
}

/* 菜单模式五分钟上限；升降实操由操作员关电，不在动作中定时松电。 */''')
    s=s.replace('    if (ArmPower_IsMenuMode() == 0U)\n', '''    if (APP_ARM_LIFT_TEST != 0U)
    {
        g_arm_menu_power_state = (request == 1U) ? 1U : 0U;
        return request;
    }
    if (ArmPower_IsMenuMode() == 0U)
''')
    s=s.replace('static TickType_t s_menu_started = 0U;',
                '#if !APP_ARM_LIFT_TEST\nstatic TickType_t s_menu_started = 0U;\n#endif')
    s=s.replace('''    if (APP_ARM_LIFT_TEST != 0U)
    {
        g_arm_menu_power_state = (request == 1U) ? 1U : 0U;
        return request;
    }
    if (ArmPower_IsMenuMode() == 0U)''','''#if APP_ARM_LIFT_TEST
    g_arm_menu_power_state = (request == 1U) ? 1U : 0U;
    return request;
#else
    if (ArmPower_IsMenuMode() == 0U)''')
    s=s.replace('    return 1U;\n}', '    return 1U;\n#endif\n}')
    p.write_text(s,encoding='utf-8')
    for rel in ['USER/main.c','SYSTEM/SAFETY/safety.c']:
        p=STAGE/rel;d=p.read_bytes();assert b'ArmPower_IsMenuMode()' in d;d=d.replace(b'ArmPower_IsMenuMode()',b'ArmPower_IsIsolatedMode()');p.write_bytes(d)
    p=STAGE/'HARDWARE/LIFT/lift_config.h';s=p.read_text(encoding='utf-8');s=s.replace('#endif','''/* 首次方向试验只走16个脉冲；不是已经标定的毫米行程。 */
#define LIFT_JOG_PULSES       16U
#define LIFT_JOG_RPM          5U
#define LIFT_JOG_ACCELERATION 20U
#define LIFT_MOVE_TIMEOUT_MS  3000U
#define LIFT_POLL_MS          100U

#endif''');p.write_text(s,encoding='utf-8')
    p=STAGE/'HARDWARE/LIFT/lift.h';s=p.read_text(encoding='utf-8');s=s.replace('/* 只由现有CAN任务调用；本阶段不包含使能/失能/运动。 */','''/* 运动请求：1方向0小步，2方向1小步，3立即停止，不松轴。 */
extern volatile uint32_t g_lift_motion_request;
/* 0闲置，1~6准备/运行，7反馈完成，8用户停止，9失败。 */
extern volatile uint32_t g_lift_motion_state;
/* 1反馈完成，2用户停，3超时，4通讯/格式，5驱动保护，6未就绪。 */
extern volatile uint32_t g_lift_motion_result;
extern volatile uint32_t g_lift_enable_result;
extern volatile uint32_t g_lift_move_result;
extern volatile uint32_t g_lift_stop_result;
extern volatile uint32_t g_lift_flags;
extern volatile int32_t g_lift_speed_rpm;
extern volatile int64_t g_lift_delta_raw;
extern volatile uint32_t g_lift_can_esr;
extern volatile uint32_t g_lift_options_result;
extern CanRxMsg g_lift_control_reply;
/* 只由现有CAN任务串行调用。 */''');p.write_text(s,encoding='utf-8')
    p=STAGE/'HARDWARE/LIFT/lift.c';s=p.read_text(encoding='utf-8');s=s.replace('#include "lift_config.h"','#include "lift_config.h"\n#include "lift_protocol.h"');s=s.replace('void Lift_Service(uint32_t power_ready)',MOTION+'\nvoid Lift_Service(uint32_t power_ready)')
    marker='        g_lift_state = 0U;\n        return;';assert s.count(marker)==1;s=s.replace(marker,'''        if (g_lift_motion_state >= 1U && g_lift_motion_state <= 6U)
        { g_lift_motion_state=9U; g_lift_motion_result=6U; }
        g_lift_motion_request=0U;
        g_lift_state = 0U;
        return;''')
    s=s.replace('    if (g_lift_read_request != 0U)','    if (g_lift_state == 4U)\n    {\n        Lift_MotionService(now);\n        return;\n    }\n    if (g_lift_motion_request != 0U)\n    {\n        g_lift_motion_request=0U; g_lift_motion_result=6U;\n    }\n    if (g_lift_read_request != 0U)')
    s=s.replace('''    if (g_lift_state == 0U)
    {
        memset''','''    if (g_lift_state == 0U)
    {
        g_lift_motion_state = 0U;
        g_lift_motion_result = 0U;
        memset''')
    # 恢复完整状态后的显式重读；动作期间不重建会话。
    s=s.replace('''    if (g_lift_state == 4U)
    {
        Lift_MotionService(now);
        return;
    }
''','''    if (g_lift_state == 4U)
    {
        if (g_lift_read_request != 0U &&
            !(g_lift_motion_state >= 1U && g_lift_motion_state <= 6U))
        {
            g_lift_read_request = 0U;
            g_lift_state = 0U;
        }
        else
        {
            g_lift_read_request = 0U;
            Lift_MotionService(now);
            return;
        }
    }
''')
    p.write_text(s,encoding='utf-8')
    for rel,source in [('HARDWARE/LIFT/lift_protocol.h',PROTO_H),('HARDWARE/LIFT/lift_protocol.c',PROTO_C)]:
        assert not (ACTUAL/rel).exists();(STAGE/rel).write_text(source,encoding='utf-8')
    p=STAGE/'SYSTEM/CAN/can.c';d=p.read_bytes();d=one(d,b'#include "can.h"',b'#include "can.h"\n#include "app_config.h"');d+=TRANSPORT.encode('utf-8');p.write_bytes(d)
    p=STAGE/'SYSTEM/CAN/can.h';d=p.read_bytes();assert d.count(b'#endif')==1;d=d.replace(b'#endif',b'uint32_t CAN1_LiftExchange(const uint8_t *request, uint8_t length, CanRxMsg *reply);\nuint32_t CAN1_ReadMotorOptions(uint8_t address, CanRxMsg *reply);\n\n#endif');p.write_bytes(d)
    p=STAGE/'gongxun-f407.uvprojx';s=p.read_text(encoding='utf-8');pattern=r'(<Group>\s*<GroupName>LIFT</GroupName>.*?<Files>)(.*?)(</Files>)'
    def ins(m):
        body=m.group(2)
        for name,typ in [('lift_protocol.c',1),('lift_protocol.h',5)]:body+=f'<File><FileName>{name}</FileName><FileType>{typ}</FileType><FilePath>.\\HARDWARE\\LIFT\\{name}</FilePath></File>\n'
        return m.group(1)+body+m.group(3)
    s,n=re.subn(pattern,ins,s,flags=re.S);assert n==1;p.write_text(s,encoding='utf-8');ET.parse(p)
    files=['USER/app_config.h','USER/arm_power_mode.h','USER/arm_power_mode.c','USER/main.c','SYSTEM/SAFETY/safety.c','HARDWARE/LIFT/lift_config.h','HARDWARE/LIFT/lift.h','HARDWARE/LIFT/lift.c','HARDWARE/LIFT/lift_protocol.c','HARDWARE/LIFT/lift_protocol.h','SYSTEM/CAN/can.c','SYSTEM/CAN/can.h','gongxun-f407.uvprojx']
    rows=[]
    for rel in files:
        original=ACTUAL/rel
        if original.exists():
            dest=EV/'original'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(original,dest)
        rows.append({'file':rel,'before_sha256':sha(original) if original.exists() else None,'after_sha256':sha(STAGE/rel)})
    (EV/'manifest.json').write_text(json.dumps({'actual':str(ACTUAL),'stage':str(STAGE),'files':rows,'mode':'Isolated node5 Emm16-pulse relative-to-real-position jog,5RPM','flash_performed':False},ensure_ascii=False,indent=2),encoding='utf-8');print('Prepared',STAGE)
if __name__=='__main__':prepare()
