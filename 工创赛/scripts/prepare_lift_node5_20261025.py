"""Stage isolated node5 lift identity module, reusing the sole CAN task."""
from pathlib import Path
import hashlib,json,re,shutil,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE=ROOT/'tmp/lift_node5_20261025/project'
EV=ROOT/'evidence/p2-arm/2026-10-25/lift-node5'
CONFIG='''#ifndef LIFT_CONFIG_H
#define LIFT_CONFIG_H

/* 用户当前升降实参：张大头35、地址5、CAN1_MAP、500kbit/s。 */
#define LIFT_CAN_ADDRESS       5U
#define LIFT_CAN_BAUD          500000U
#define LIFT_POWER_SETTLE_MS   1000U
#define LIFT_READ_RETRY_MS     300U
#define LIFT_READ_MAX_ATTEMPTS 3U

#endif
'''
HEADER='''#ifndef LIFT_H
#define LIFT_H
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"

extern const uint32_t g_lift_address;
/* 0等供电，1等驱动启动，2读版本，3读状态，4读取完成，5读取失败。 */
extern volatile uint32_t g_lift_state;
extern volatile uint32_t g_lift_read_request;
extern volatile uint32_t g_lift_read_attempts;
extern volatile uint32_t g_lift_version_result;
extern volatile uint32_t g_lift_status_result;
extern CanRxMsg g_lift_version_reply;
extern CanRxMsg g_lift_status_reply;

/* 只由现有CAN任务调用；本阶段不包含使能/失能/运动。 */
void Lift_Service(uint32_t power_ready);
#endif
'''
SOURCE='''#include "lift.h"
#include "lift_config.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

const uint32_t g_lift_address = LIFT_CAN_ADDRESS;
volatile uint32_t g_lift_state = 0U;
volatile uint32_t g_lift_read_request = 0U;
volatile uint32_t g_lift_read_attempts = 0U;
volatile uint32_t g_lift_version_result = 0U;
volatile uint32_t g_lift_status_result = 0U;
CanRxMsg g_lift_version_reply;
CanRxMsg g_lift_status_reply;
static TickType_t s_wait_started = 0U;
static uint32_t s_wait_ms = 0U;

void Lift_Service(uint32_t power_ready)
{
    TickType_t now = xTaskGetTickCount();
    if (power_ready != 1U)
    {
        /* 保留本次原始回复方便观察，供电恢复再新建读取会话。 */
        g_lift_state = 0U;
        return;
    }
    if (g_lift_read_request != 0U)
    {
        g_lift_read_request = 0U;
        g_lift_state = 0U;
    }
    if (g_lift_state == 0U)
    {
        memset(&g_lift_version_reply, 0, sizeof(g_lift_version_reply));
        memset(&g_lift_status_reply, 0, sizeof(g_lift_status_reply));
        g_lift_version_result = 0U;
        g_lift_status_result = 0U;
        g_lift_read_attempts = 0U;
        s_wait_started = now;
        s_wait_ms = LIFT_POWER_SETTLE_MS;
        g_lift_state = 1U;
        return;
    }
    if (g_lift_state == 1U)
    {
        if ((TickType_t)(now - s_wait_started) < pdMS_TO_TICKS(s_wait_ms))
        {
            return;
        }
        g_lift_state = 2U;
    }
    if (g_lift_state == 2U)
    {
        /* 通用单帧查询支持非零地址，现有底盘运动地址保护不变。 */
        g_lift_read_attempts++;
        g_lift_version_result = CAN1_ReadVersion(LIFT_CAN_ADDRESS,
                                                 &g_lift_version_reply);
        if (g_lift_version_result == 1U)
        {
            g_lift_state = 3U;
        }
        else if (g_lift_read_attempts < LIFT_READ_MAX_ATTEMPTS)
        {
            s_wait_started = xTaskGetTickCount();
            s_wait_ms = LIFT_READ_RETRY_MS;
            g_lift_state = 1U;
        }
        else
        {
            g_lift_state = 5U;
        }
        return;
    }
    if (g_lift_state == 3U)
    {
        g_lift_status_result = CAN1_ReadStatus(LIFT_CAN_ADDRESS,
                                               &g_lift_status_reply);
        g_lift_state = (g_lift_status_result == 1U) ? 4U : 5U;
    }
}
'''
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def prepare():
    assert not STAGE.exists()
    shutil.copytree(ACTUAL,STAGE,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
    EV.mkdir(parents=True,exist_ok=True)
    new={'HARDWARE/LIFT/lift_config.h':CONFIG,'HARDWARE/LIFT/lift.h':HEADER,'HARDWARE/LIFT/lift.c':SOURCE}
    for rel,text in new.items():
        assert not (ACTUAL/rel).exists()
        p=STAGE/rel;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(text.encode('utf-8'))
    p=STAGE/'USER/main.c';data=p.read_bytes();old=b'#include "arm_power_mode.h"';assert data.count(old)==1;data=data.replace(old,old+b'\n#include "lift.h"')
    old=b'''        if (ArmPower_IsMenuMode() != 0U)
        {
            g_can_task_stack_words'''
    replacement=b'''        if (ArmPower_IsMenuMode() != 0U)
        {
            Lift_Service(((g_can_test_result == 1U) &&
                          (g_safety_ready == 1U) &&
                          (g_vm_test_request == 1U) &&
                          (g_arm_menu_power_state == 1U)) ? 1U : 0U);
            g_can_task_stack_words'''
    assert data.count(old)==1;data=data.replace(old,replacement)
    data=data.replace(b'/* Menu power mode never executes CAN polling or motor commands. */',b'/* Isolated menu mode permits node5 identity reads, never motion. */')
    p.write_bytes(data)
    p=STAGE/'gongxun-f407.uvprojx';text=p.read_text(encoding='utf-8')
    pattern=r'(<Cads>.*?<VariousControls>.*?<IncludePath>)(.*?)(</IncludePath>)'
    text,n=re.subn(pattern,lambda m:m.group(1)+m.group(2)+r';.\HARDWARE\LIFT'+m.group(3),text,flags=re.S);assert n==1
    files=''
    for name,kind in [('lift.c',1),('lift.h',5),('lift_config.h',5)]:
        files+=f'<File><FileName>{name}</FileName><FileType>{kind}</FileType><FilePath>.\\HARDWARE\\LIFT\\{name}</FilePath></File>\n'
    assert text.count('</Groups>')==1;text=text.replace('</Groups>','<Group><GroupName>LIFT</GroupName><Files>'+files+'</Files></Group>\n</Groups>');p.write_text(text,encoding='utf-8');ET.parse(p)
    records=[]
    for rel in ['USER/main.c','gongxun-f407.uvprojx',*new]:
        p=ACTUAL/rel
        if p.exists():
            dest=EV/'original'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
        records.append({'file':rel,'before_sha256':sha(p) if p.exists() else None,'after_sha256':sha(STAGE/rel)})
    (EV/'manifest.json').write_text(json.dumps({'actual':str(ACTUAL),'stage':str(STAGE),'files':records,'node':5,'source':'User changed ID_Addr from1 to5; CAN1_MAP500000 confirmed','motion_commands_added':False,'flash_performed':False},ensure_ascii=False,indent=2),encoding='utf-8');print('Prepared',STAGE)
if __name__=='__main__':prepare()
