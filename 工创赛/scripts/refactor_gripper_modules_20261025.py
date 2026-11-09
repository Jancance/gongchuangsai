"""Stage, build and install the explicitly authorized gripper-only refactor."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT/'tmp/gripper_modular_20261025/project'
EVIDENCE = ROOT/'evidence/p2-arm/2026-10-25/gripper-modular'

CONFIG = '''#ifndef GRIPPER_CONFIG_H
#define GRIPPER_CONFIG_H

/* 标称角度映射起点，实际机械开合范围后续按实物调整。 */
#define JIAZHUA_ANGLE_MAX_DEG  270.0f
#define JIAZHUA_PULSE_MIN_US   500U
#define JIAZHUA_PULSE_MAX_US   2500U
#define JIAZHUA_PERIOD_US      20000U

/* 调试任务：原生FreeRTOS，默认不发送位置命令。 */
#define GRIPPER_TASK_STACK_WORDS  256U
#define GRIPPER_TASK_PRIORITY     2U
#define GRIPPER_TASK_PERIOD_MS    10U
#define GRIPPER_DEBUG_ANGLE_DEG   90.0f

#endif
'''

TASK_H = '''#ifndef GRIPPER_TASK_H
#define GRIPPER_TASK_H

#include <stdint.h>
#include "FreeRTOS.h"

/* Watch：先填angle，再写request=1；request=2撤销PWM。 */
extern volatile float g_jiazhua_debug_angle;
extern volatile uint32_t g_jiazhua_debug_request;
/* 0未接受/角度无效，1指令已写入，2已撤销信号，3未知请求。 */
extern volatile uint32_t g_jiazhua_debug_result;
extern volatile BaseType_t g_gripper_task_create;

void Gripper_TaskInit(void);

#endif
'''

TASK_C = '''#include "gripper_task.h"
#include "gripper.h"
#include "gripper_config.h"
#include "task.h"

volatile float g_jiazhua_debug_angle = GRIPPER_DEBUG_ANGLE_DEG;
volatile uint32_t g_jiazhua_debug_request = 0U;
volatile uint32_t g_jiazhua_debug_result = 0U;
volatile BaseType_t g_gripper_task_create = pdFAIL;

static TaskHandle_t s_gripper_task = NULL;

/* 调试请求集中在本任务执行，不占用主循环或串口接收任务。 */
static void Gripper_Task(void *argument)
{
    uint32_t request;
    float angle;
    (void)argument;

    for (;;)
    {
        /* 一起取得请求与参数，避免处理过程参数被其他任务改动。 */
        taskENTER_CRITICAL();
        request = g_jiazhua_debug_request;
        angle = g_jiazhua_debug_angle;
        g_jiazhua_debug_request = 0U;
        taskEXIT_CRITICAL();

        if (request == 1U)
        {
            g_jiazhua_debug_result = jiazhua(angle);
        }
        else if (request == 2U)
        {
            Gripper_Stop();
            g_jiazhua_debug_result = 2U;
        }
        else if (request != 0U)
        {
            g_jiazhua_debug_result = 3U;
        }
        vTaskDelay(pdMS_TO_TICKS(GRIPPER_TASK_PERIOD_MS));
    }
}

void Gripper_TaskInit(void)
{
    /* 只创建一次；初始化不会自动发出90度或其他角度命令。 */
    if (s_gripper_task == NULL)
    {
        g_gripper_task_create = xTaskCreate(Gripper_Task, "Gripper",
            GRIPPER_TASK_STACK_WORDS, NULL, GRIPPER_TASK_PRIORITY,
            &s_gripper_task);
        configASSERT(g_gripper_task_create == pdPASS);
    }
}
'''


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def add_project_files(text, group, entries):
    pattern = r'(<Group>\s*<GroupName>' + re.escape(group) + r'</GroupName>.*?<Files>)(.*?)(</Files>)'
    count = 0
    def replace(match):
        nonlocal count
        count += 1
        body = match.group(2)
        for name, path, kind in entries:
            if '<FileName>'+name+'</FileName>' not in body:
                body += ('\n            <File>\n              <FileName>'+name+'</FileName>\n'
                         '              <FileType>'+str(kind)+'</FileType>\n'
                         '              <FilePath>'+path+'</FilePath>\n            </File>\n          ')
        return match.group(1) + body + match.group(3)
    text = re.sub(pattern, replace, text, flags=re.S)
    if count != 1:
        raise RuntimeError('Expected one project group: '+group)
    return text


def prepare():
    if STAGE.exists() or (EVIDENCE/'manifest.json').exists():
        raise FileExistsError('Staging/manifest exists; preserve previous work')
    shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns(
        'Objects', 'Listings', '.git', '.vscode', '*.uvguix.*'))
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    main = STAGE/'USER/main.c'
    data = main.read_bytes()
    old = b'volatile BaseType_t g_gripper_task_create = pdFAIL;\n'
    assert data.count(old) == 1
    data = data.replace(old, b'')
    old = b'#include "gripper.h"'
    assert data.count(old) == 1
    data = data.replace(old, old+b'\n#include "gripper_task.h"')
    start = data.index(b'        if (g_jiazhua_debug_request == 2U)')
    end = data.index(b'        g_test_count_a++;', start)
    removed = data[start:end]
    assert removed.count(b'jiazhua(g_jiazhua_debug_angle)') == 1
    data = data[:start] + data[end:]
    pattern = (rb'    g_gripper_task_create = xTaskCreate\(Gripper_TestTask, "Gripper",\s*'
               rb'256U, NULL, 2U, NULL\);\s*configASSERT\(g_gripper_task_create == pdPASS\);')
    data, count = re.subn(pattern, b'    Gripper_TaskInit();', data)
    assert count == 1
    main.write_bytes(data)
    gripper = STAGE/'HARDWARE/GRIPPER/gripper.c'
    raw = gripper.read_bytes()
    try:
        source = raw.decode('utf-8')
    except UnicodeDecodeError:
        source = raw.decode('gb18030')
    source = source.replace('\r\n','\n').replace('\r','\n')
    source = source.replace('#include "gripper.h"', '#include "gripper.h"\n#include "gripper_config.h"', 1)
    for name in ['JIAZHUA_ANGLE_MAX_DEG','JIAZHUA_PULSE_MIN_US',
                 'JIAZHUA_PULSE_MAX_US','JIAZHUA_PERIOD_US']:
        source, count = re.subn(r'^#define '+name+r'[^\n]*\n', '', source, flags=re.M)
        assert count == 1
    gripper.write_bytes(source.encode('utf-8'))
    new_files = {'HARDWARE/GRIPPER/gripper_config.h': CONFIG,
                 'USER/gripper_task.h': TASK_H, 'USER/gripper_task.c': TASK_C}
    for rel, text in new_files.items():
        path = STAGE/rel
        assert not (ACTUAL/rel).exists(), 'Do not overwrite unexpected user module: '+rel
        path.write_bytes(text.encode('utf-8'))
    project = STAGE/'gongxun-f407.uvprojx'
    text = project.read_text(encoding='utf-8')
    text = add_project_files(text, 'USER', [
        ('gripper_task.c', r'.\USER\gripper_task.c', 1),
        ('gripper_task.h', r'.\USER\gripper_task.h', 5)])
    text = add_project_files(text, 'HARDWARE', [
        ('gripper_config.h', r'.\HARDWARE\GRIPPER\gripper_config.h', 5)])
    text = add_project_files(text, 'LIB', [
        ('stm32f4xx_tim.c', r'.\StdPeriph_Driver\src\stm32f4xx_tim.c', 1)])
    project.write_text(text, encoding='utf-8')
    ET.parse(project)
    changed = ['USER/main.c', 'HARDWARE/GRIPPER/gripper.c',
               'gongxun-f407.uvprojx', *new_files]
    records = []
    for rel in changed:
        original = ACTUAL/rel
        if original.exists():
            backup = EVIDENCE/'original'/rel
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(original, backup)
        records.append({'file':rel, 'before_sha256':sha(original) if original.exists() else None,
                        'after_sha256':sha(STAGE/rel)})
    (EVIDENCE/'manifest.json').write_text(json.dumps({
        'actual_project':str(ACTUAL), 'stage':str(STAGE), 'files':records,
        'authorization':'User explicitly requested automatic edits and modularization',
        'flash_performed':False}, ensure_ascii=False, indent=2), encoding='utf-8')
    print('Prepared:', STAGE)


def apply():
    manifest = json.loads((EVIDENCE/'manifest.json').read_text(encoding='utf-8'))
    for item in manifest['files']:
        path = ACTUAL/item['file']
        current = sha(path) if path.exists() else None
        if current != item['before_sha256']:
            raise RuntimeError('User source changed since staging: '+str(path))
        if sha(STAGE/item['file']) != item['after_sha256']:
            raise RuntimeError('Staged source changed: '+item['file'])
    for item in manifest['files']:
        dest = ACTUAL/item['file']
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(STAGE/item['file'], dest)
    (EVIDENCE/'installed.json').write_text(json.dumps({
        'installed_files':manifest['files'], 'flash_performed':False},
        ensure_ascii=False, indent=2), encoding='utf-8')
    print('Installed:', ACTUAL)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare','apply'))
    args = parser.parse_args()
    {'prepare':prepare,'apply':apply}[args.action]()
