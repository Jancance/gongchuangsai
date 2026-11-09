"""Stage menu-only VM power mode; no CAN motor commands in this mode."""
from pathlib import Path
import hashlib,json,re,shutil,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE=ROOT/'tmp/arm_menu_power_20261025/project'
EV=ROOT/'evidence/p2-arm/2026-10-25/menu-power-mode'

HEADER='''#ifndef ARM_POWER_MODE_H
#define ARM_POWER_MODE_H
#include <stdint.h>

/* 0未请求，1菜单供电请求有效，2本次菜单供电时间已到。不是VM电压反馈。 */
extern volatile uint32_t g_arm_menu_power_state;
uint32_t ArmPower_IsMenuMode(void);
uint32_t ArmPower_FilterRequest(uint32_t request);
#endif
'''
SOURCE='''#include "arm_power_mode.h"
#include "app_config.h"
#include "FreeRTOS.h"
#include "task.h"

volatile uint32_t g_arm_menu_power_state = 0U;
static TickType_t s_menu_started = 0U;

uint32_t ArmPower_IsMenuMode(void)
{
    return APP_ARM_MENU_POWER_TEST;
}

/* 仅监督任务调用：单次最多五分钟，关请求后才能重新开始。 */
uint32_t ArmPower_FilterRequest(uint32_t request)
{
    if (ArmPower_IsMenuMode() == 0U)
    {
        return request;
    }
    if (request != 1U)
    {
        g_arm_menu_power_state = 0U;
        return 0U;
    }
    if (g_arm_menu_power_state == 0U)
    {
        s_menu_started = xTaskGetTickCount();
        g_arm_menu_power_state = 1U;
    }
    if ((g_arm_menu_power_state == 2U) ||
        ((TickType_t)(xTaskGetTickCount() - s_menu_started) >=
         pdMS_TO_TICKS(ARM_MENU_POWER_TIMEOUT_MS)))
    {
        g_arm_menu_power_state = 2U;
        return 0U;
    }
    return 1U;
}
'''

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def once(data,old,new):
    assert data.count(old)==1,old[:100]
    return data.replace(old,new)
def prepare():
    assert not STAGE.exists()
    shutil.copytree(ACTUAL,STAGE,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
    EV.mkdir(parents=True,exist_ok=True)
    p=STAGE/'USER/app_config.h';data=p.read_bytes()
    marker=b'#define APP_HEADING_SESSION_TEST 0U'
    data=once(data,marker,marker+b'\n\n/* Menu-only VM supply: no chassis CAN commands or start key. */\n#define APP_ARM_MENU_POWER_TEST 1U\n#define ARM_MENU_POWER_TIMEOUT_MS 300000U\n\n#if APP_ARM_MENU_POWER_TEST && (APP_HEADING_SESSION_TEST || APP_HWT_STATIC_TEST || APP_HWT_TURN_TEST || APP_CHASSIS_KEY_TEST_ENABLE)\n#error Arm_menu_power_and_motion_test_modes_must_be_exclusive\n#endif')
    p.write_bytes(data)
    p=STAGE/'USER/main.c';data=p.read_bytes();data=once(data,b'#include "gripper_task.h"',b'#include "gripper_task.h"\n#include "arm_power_mode.h"')
    data=once(data,b'        g_can_task_count++;',b'''        g_can_task_count++;
        /* Menu power mode never executes CAN polling or motor commands. */
        if (ArmPower_IsMenuMode() != 0U)
        {
            g_can_task_stack_words = uxTaskGetStackHighWaterMark(NULL);
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }''')
    marker=b'''    while (1)
    {
#if APP_HEADING_SESSION_TEST
        if (Key_GetEvent() != 0U)'''
    replacement=b'''    while (1)
    {
        /* Consume the start key without creating a motion request. */
        if (ArmPower_IsMenuMode() != 0U)
        {
            (void)Key_GetEvent();
            Safety_Report(SAFETY_TASK_KEY);
            vTaskDelay(pdMS_TO_TICKS(1U));
            continue;
        }
#if APP_HEADING_SESSION_TEST
        if (Key_GetEvent() != 0U)'''
    data=once(data,marker,replacement);p.write_bytes(data)
    p=STAGE/'SYSTEM/SAFETY/safety.c';data=p.read_bytes();data=once(data,b'#include "safety.h"',b'#include "safety.h"\n#include "arm_power_mode.h"')
    data=once(data,b'    g_safety_count++;',b'    vm_request = ArmPower_FilterRequest(vm_request);\n    g_safety_count++;')
    line=b'    Safety_CanGuardUpdate(((g_wheel_test_cut_vm == 1U) || (g_chassis_auto_cut_vm == 1U)) ? 0U : vm_request);'
    data=once(data,line,b'''    if (ArmPower_IsMenuMode() != 0U)
    {
        /* Four chassis nodes are intentionally absent; motion stays forbidden. */
        Safety_CanGuardUpdate(0U);
    }
    else
    {
    '''+line+b'\n    }')
    first=data.index(b'    Safety_WheelTestUpdate(vm_request);');last=data.index(b'\n\n    if ((g_safety_ready == 1U)',first)
    original=data[first:last]
    assert b'Safety_MotionStopUpdate' in original
    data=data[:first]+b'''    if (ArmPower_IsMenuMode() != 0U)
    {
        g_motion_allowed = 0U;
        g_motion_block_reason = MOTION_BLOCK_POLICY_PENDING | MOTION_BLOCK_CAN;
    }
    else
    {
'''+original+b'\n    }'+data[last:];p.write_bytes(data)
    for rel,source in [('USER/arm_power_mode.h',HEADER),('USER/arm_power_mode.c',SOURCE)]:
        assert not (ACTUAL/rel).exists()
        (STAGE/rel).write_bytes(source.encode('utf-8'))
    p=STAGE/'gongxun-f407.uvprojx';text=p.read_text(encoding='utf-8')
    pat=r'(<Group>\s*<GroupName>USER</GroupName>.*?<Files>)(.*?)(</Files>)'
    def insert(m):
        body=m.group(2)
        for name,typ in [('arm_power_mode.c',1),('arm_power_mode.h',5)]:
            body+=f'\n            <File><FileName>{name}</FileName><FileType>{typ}</FileType><FilePath>.\\USER\\{name}</FilePath></File>\n'
        return m.group(1)+body+m.group(3)
    text,n=re.subn(pat,insert,text,flags=re.S);assert n==1;p.write_text(text,encoding='utf-8');ET.parse(p)
    files=['USER/app_config.h','USER/main.c','SYSTEM/SAFETY/safety.c','USER/arm_power_mode.c','USER/arm_power_mode.h','gongxun-f407.uvprojx'];records=[]
    for rel in files:
        original=ACTUAL/rel
        if original.exists():
            dest=EV/'original'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(original,dest)
        records.append({'file':rel,'before_sha256':sha(original) if original.exists() else None,'after_sha256':sha(STAGE/rel)})
    (EV/'manifest.json').write_text(json.dumps({'actual':str(ACTUAL),'stage':str(STAGE),'files':records,'flash_performed':False},ensure_ascii=False,indent=2),encoding='utf-8')
    print('Prepared',STAGE)
if __name__=='__main__':prepare()
