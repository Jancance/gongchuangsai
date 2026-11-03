"""Prepare a reviewed patch for the actual project; installation is separate."""
from pathlib import Path
import copy
import hashlib
import json
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT / 'tmp/stage1_early_scan_startup_20261103'
EV = ROOT / 'evidence/integration/2026-11-03/early-scan-startup'
PROJECT = 'gongxun-f407.uvprojx'


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def read(p):
    b = p.read_bytes()
    try:
        return b.decode('utf-8').replace('\r\n', '\n').replace('\r', ''), 'utf-8'
    except UnicodeDecodeError:
        return b.decode('gbk').replace('\r\n', '\n').replace('\r', ''), 'gbk'


def edit(rel, fn):
    p = STAGE / rel
    s, encoding = read(p)
    p.write_bytes(fn(s).replace('\n', '\r\n').encode(encoding))


def replace(s, old, new):
    assert s.count(old) == 1, old[:100]
    return s.replace(old, new)


def sources(folder):
    return {p.relative_to(folder).as_posix(): sha(p) for p in folder.rglob('*')
            if p.is_file() and p.suffix.lower() in {'.c', '.h', '.s', '.inc', '.uvprojx'}
            and not {'Objects', 'Listings'}.intersection(p.relative_to(folder).parts)}


def main_patch(s):
    s = replace(s, '#include "app_config.h"', '#include "app_config.h"\n#include "startup_pose.h"')
    # These flags are read by the CAN-owned motion state machine; TaskA only
    # publishes the stop request. No physical completion is inferred here.
    mark = 'volatile uint32_t g_scan_display_result = 0U;'
    s = replace(s, mark, 'volatile uint32_t g_stage1_stop_requested=0U;\n'
                'volatile uint32_t g_stage1_motion_skipped=0U;\n' + mark)
    mark = '            g_forward_hold.error_deg = error;\n'
    s = replace(s, mark, mark + '''            /* 有效任务码提前结束本段；沿用唯一CAN任务有序停车及零速核验。 */
            if (VEHICLE_STAGE1_ENABLE && g_stage1_stop_requested &&
                !g_forward_hold.stopping && status.active && status.state>=CHASSIS_AUTO_START &&
                status.state<=CHASSIS_AUTO_SETTLE)
            {
                if (ChassisAuto_RequestStop()!=1U) App_ForwardHoldTrip(8U);
                else {g_forward_hold.stopping=1U;g_forward_hold.correction_rpm=0;s_hold_stop_ms=now;}
            }
''')
    mark = '    if (g_combined_test.state == 0U && s_motion_session_open)'
    s = replace(s, mark, '''    /* 启动请求尚未发给执行器时已取得码：撤销排队，不制造成功路程。 */
    if (VEHICLE_STAGE1_ENABLE && g_stage1_stop_requested &&
        g_combined_test.state==0U && request==1U && !s_motion_session_open)
    {
        s_serial_start_permit=0U;g_stage1_motion_skipped=1U;
        g_combined_test.state=3U;s_combined_end_ms=now;
        taskEXIT_CRITICAL();return;
    }
''' + mark)
    mark = '''    if (g_combined_test.state == 2U)
    {
        /* 保留上一段诊断'''
    s = replace(s, mark, '''    if (g_combined_test.state == 2U)
    {
        /* 段间取得码后直接释放已停稳会话，不再装载下一段。 */
        if (VEHICLE_STAGE1_ENABLE && g_stage1_stop_requested)
        {
            s_motion_list_submit=1U;(void)MotionFinish();s_motion_list_submit=0U;
            Power_ReadCommandStatus(&power);
            if(!g_vm_test_request && !power.active)
            {g_combined_test.state=3U;s_combined_end_ms=now;}
            taskEXIT_CRITICAL();return;
        }
        /* 保留上一段诊断''')
    # Only within the DONE branch of the combined route.
    lo = s.index('        if (phase == HR_DONE)', s.index('static void App_CombinedTestStep'))
    hi = s.index('\n    taskEXIT_CRITICAL();\n}', lo)
    part = s[lo:hi]
    part = part.replace('if ((g_combined_test.index + 1U) == MOTION_ACTION_COUNT)',
                        'if (g_stage1_stop_requested || (g_combined_test.index + 1U) == MOTION_ACTION_COUNT)')
    part = part.replace('((g_combined_test.index + 1U) < MOTION_ACTION_COUNT) ?',
                        '(!g_stage1_stop_requested && (g_combined_test.index + 1U) < MOTION_ACTION_COUNT) ?')
    part = replace(part, '                g_combined_test.completed++;',
                   '                if(!g_stage1_stop_requested || status.distance_reached) g_combined_test.completed++;')
    part = replace(part, '((index + 1U) == MOTION_ACTION_COUNT) ? 3U : 2U;',
                   '(g_stage1_stop_requested || (index + 1U) == MOTION_ACTION_COUNT) ? 3U : 2U;')
    part = replace(part, 'if ((index + 1U) == MOTION_ACTION_COUNT) s_combined_end_ms = now;',
                   'if (g_stage1_stop_requested || (index + 1U) == MOTION_ACTION_COUNT) s_combined_end_ms = now;')
    s = s[:lo] + part + s[hi:]
    # Remove the old path that both discarded en-route scans and lost failed
    # screen writes. The new service owns an immutable code and bounded retries.
    lo = s.index('        /* 确认启动请求后，保留本轮码和屏幕，不替换任务对象 */', s.index('SAOMA_Update();'))
    hi = s.index('        App_StartGateUpdate();', lo)
    old = s[lo:hi]
    generic = old[:old.index('        App_Stage1Service();')]
    generic = generic.replace('(!VEHICLE_STAGE1_ENABLE || App_Stage1FreshCode((uint32_t)xTaskGetTickCount())) &&', '1U &&')
    s = s[:lo] + '''        if(VEHICLE_STAGE1_ENABLE)
        {
            App_Stage1Service();
            App_Stage1Display((TickType_t)(xTaskGetTickCount()-screen_start_tick)>=pdMS_TO_TICKS(1000U));
        }
        else
        {
''' + generic + '''        }

''' + s[hi:]
    s = replace(s, 'if(g_stage1_state==1U || g_stage1_state==2U)',
                'if(g_stage1_state==1U || g_stage1_state==2U || g_stage1_state==6U)')
    s = replace(s, 'if(g_stage1_state==1U) g_combined_test.request=2U;',
                'if(g_stage1_state==1U || g_stage1_state==6U) g_combined_test.request=2U;')
    s = replace(s, '    Power_Init();', '    Power_Init();\n    StartupPose_Init();')
    return s


RUNTIME = r'''/* 第一阶段：本轮有效码可在途中锁存；0待机1移动2等码3完成4失败5取消6扫码停车。
 * 请求停车不等于停稳，显示串口发送成功也不等于屏幕可视验收。
 * 第二阶段当前只有带run_id的交接状态，不发未实现的路线/机械臂动作。 */
volatile uint32_t g_stage1_state=0U,g_stage1_cause=0U,g_stage1_run_id=0U;
volatile uint32_t g_stage1_key_request=0U,g_stage1_denied=0U,g_stage1_scan_valid=0U;
volatile uint32_t g_stage2_ready=0U,g_stage2_run_id=0U;
volatile uint32_t g_stage1_display_attempts=0U;
static uint32_t s_stage1_begin_ms,s_stage1_end_ms,s_stage1_scan_ms;
static uint32_t s_stage1_scan_baseline,s_stage1_stop_ms,s_stage1_display_ms;
static uint32_t s_stage1_display_done,s_stage1_code_ms;
static FieldStage1Plan s_stage1_plan;
char g_stage1_task_code[16]={0};

static uint32_t App_Stage1Rearm(void)
{
    ChassisAutoStatus status;ForwardHoldDebug forward={0};CombinedTestDebug combined={0};
    if(g_stage1_stop_requested || g_stage2_ready || g_stage1_state!=3U ||
       g_combined_test.state!=3U || g_combined_test.cause ||
       g_combined_test.completed!=MOTION_ACTION_COUNT || s_motion_session_open ||
       g_forward_hold.trip || g_heading_exec_trip || g_serial_motion_link_lost) return 0U;
    (void)ChassisAuto_ReadStatus(&status);
    if(!ChassisAuto_RearmCompleted(status.session)) return 0U;
    g_combined_test=combined;g_forward_hold=forward;
    g_heading_exec_phase=HR_IDLE;g_heading_exec_state=HS_IDLE;g_heading_exec_request=0U;
    HeadingSession_Init(&s_heading_exec);
    s_motion_has_action=s_combined_turn=s_combined_powered_segment=s_combined_keep_vm=0U;
    s_serial_start_permit=0U;g_serial_motion_state=g_serial_motion_result=0U;
    s_route_heading_reference=(RouteHeadingReference){0};s_route_heading_turn_active=0U;
    s_perimeter_odom_started=s_perimeter_completed_seen=0U;
    memset((void*)g_perimeter_records,0,sizeof(g_perimeter_records));
    g_stage1_state=0U;g_stage1_scan_valid=0U;
    return 1U;
}
static uint32_t App_Stage1FreshCode(uint32_t now)
{
    return (g_stage1_state==1U || g_stage1_state==2U) && !g_stage1_scan_valid &&
        g_saoma_accept_count!=s_stage1_scan_baseline && g_saoma_last_result==1U &&
        (uint32_t)(g_saoma_frame_start_ms-s_stage1_begin_ms)>0U &&
        (uint32_t)(g_saoma_frame_start_ms-s_stage1_begin_ms)<0x80000000U &&
        (uint32_t)(now-g_saoma_frame_end_ms)<VEHICLE_STAGE1_SCAN_FRESH_MS &&
        FieldMap_TaskCodeValid(SAOMA_Data);
}
static uint32_t App_Stage1Stopped(void)
{
    ChassisAutoStatus status;PowerCommandStatus power;
    (void)ChassisAuto_ReadStatus(&status);Power_ReadCommandStatus(&power);
    if(g_combined_test.state!=3U || g_combined_test.cause || s_motion_session_open ||
       g_forward_hold.trip || g_heading_exec_trip || status.active ||
       power.active || g_vm_test_request || g_safety_fault || g_rtos_fault ||
       g_can_ctrl_fault_latched) return 0U;
    if(g_stage1_motion_skipped) return status.state==CHASSIS_AUTO_IDLE && !status.result && !status.cause;
    return status.state==CHASSIS_AUTO_DONE && status.result==1U && !status.cause &&
        status.rotation_stop_mask==15U && g_combined_test.index<MOTION_ACTION_COUNT &&
        status.session==g_combined_test.session[g_combined_test.index];
}
static void App_Stage1Fail(uint32_t cause,uint32_t now)
{
    g_stage2_ready=0U;g_stage1_cause=cause;s_stage1_end_ms=now;g_stage1_state=4U;
}
static void App_Stage1Service(void)
{
#if VEHICLE_STAGE1_ENABLE
    uint32_t key,now=(uint32_t)xTaskGetTickCount();
    taskENTER_CRITICAL();key=g_stage1_key_request;g_stage1_key_request=0U;taskEXIT_CRITICAL();
    if(key==2U && (g_stage1_state==1U || g_stage1_state==2U || g_stage1_state==6U)) {
        if(g_stage1_state!=2U) MotionStop();
        g_stage2_ready=0U;g_stage1_cause=4U;s_stage1_end_ms=now;g_stage1_state=5U;return;
    }
    if(key==1U) {
        if(g_stage1_state!=0U && g_stage1_state!=3U) {g_stage1_denied++;return;}
        if(!FieldMap_Stage1(&s_stage1_plan)) {App_Stage1Fail(10U,now);return;}
        taskENTER_CRITICAL();
        if(g_radar_zone_ready!=1U || g_radar_zone_selected!=VEHICLE_STAGE1_ZONE ||
           g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched ||
           (g_stage1_state==3U && !App_Stage1Rearm()) || !App_SerialIdleReady(now))
            {g_stage1_denied++;taskEXIT_CRITICAL();return;}
        memset(s_perimeter_targets,0,sizeof(s_perimeter_targets));
        s_perimeter_targets[1][0]=s_stage1_plan.forward_cm*10.0f;
        s_perimeter_targets[2][0]=s_stage1_plan.forward_cm*10.0f;
        s_perimeter_targets[2][1]=s_stage1_plan.left_cm*10.0f;
        g_stage1_stop_requested=g_stage1_motion_skipped=0U;
        s_serial_start_permit=1U;g_combined_test.request=1U;
        g_serial_motion_state=2U;g_serial_motion_result=2U;g_serial_motion_accepted++;
        g_stage1_run_id++;if(!g_stage1_run_id)g_stage1_run_id=1U;
        g_stage1_cause=g_stage1_scan_valid=g_stage2_ready=g_stage2_run_id=0U;
        g_stage1_display_attempts=s_stage1_display_done=0U;
        memset(g_stage1_task_code,0,sizeof(g_stage1_task_code));
        s_stage1_scan_baseline=g_saoma_accept_count;
        s_stage1_begin_ms=now;s_stage1_end_ms=0U;g_stage1_state=1U;
        taskEXIT_CRITICAL();
    }
    if(g_stage1_state==1U || g_stage1_state==2U || g_stage1_state==6U) {
        if(g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched || g_serial_motion_link_lost) {
            if(g_stage1_state!=2U)MotionStop();App_Stage1Fail(5U,now);return;
        }
        if(g_combined_test.state>=4U) {App_Stage1Fail(2U,now);return;}
        if(App_Stage1FreshCode(now)) {
            memcpy(g_stage1_task_code,SAOMA_Data,sizeof(g_stage1_task_code));
            SAOMA_DataReady=0U;g_stage1_scan_valid=1U;s_stage1_code_ms=now;
            if(g_stage1_state==1U && !App_Stage1Stopped()) {
                taskENTER_CRITICAL();g_stage1_stop_requested=1U;taskEXIT_CRITICAL();
                s_stage1_stop_ms=now;g_stage1_state=6U;
            }
        }
    }
    if(g_stage1_state==1U) {
        if((uint32_t)(now-s_stage1_begin_ms)>=VEHICLE_STAGE1_MOVE_TIMEOUT_MS) {
            MotionStop();App_Stage1Fail(6U,now);return;
        }
        if(App_Stage1Stopped()) {s_stage1_scan_ms=now;g_stage1_state=2U;}
    }
    if(g_stage1_state==6U) {
        if(App_Stage1Stopped()) {s_stage1_scan_ms=now;g_stage1_state=2U;}
        else if((uint32_t)(now-s_stage1_stop_ms)>=VEHICLE_STAGE1_QR_STOP_TIMEOUT_MS) {
            MotionStop();App_Stage1Fail(8U,now);return;
        }
    }
    if(g_stage1_state==2U) {
        if(g_stage1_scan_valid && s_stage1_display_done && App_Stage1Stopped()) {
            g_stage1_cause=0U;s_stage1_end_ms=now;g_stage1_state=3U;
            g_stage2_run_id=g_stage1_run_id;g_stage2_ready=1U;
        } else if(!g_stage1_scan_valid &&
                  (uint32_t)(now-s_stage1_scan_ms)>=VEHICLE_STAGE1_SCAN_TIMEOUT_MS) {
            App_Stage1Fail(3U,now);
        }
    }
    /* 故障或停车证明失效后撤销交接，保留原任务码供取证。 */
    if(g_stage2_ready && !App_Stage1Stopped()) {App_Stage1Fail(5U,now);}
#endif
}
static void App_Stage1Display(uint32_t screen_ready)
{
#if VEHICLE_STAGE1_ENABLE
    uint32_t now=(uint32_t)xTaskGetTickCount();
    if(!screen_ready || !g_stage1_scan_valid || s_stage1_display_done ||
       (g_stage1_state!=2U && g_stage1_state!=6U)) return;
    /* 先完成有序停车，屏幕发送不抢占第一阶段运动决策。 */
    if(!App_Stage1Stopped())return;
    if(g_stage1_display_attempts && (uint32_t)(now-s_stage1_display_ms)<250U)return;
    if(g_stage1_display_attempts>=3U || (uint32_t)(now-s_stage1_code_ms)>=VEHICLE_STAGE1_SCAN_TIMEOUT_MS)
    {App_Stage1Fail(7U,now);return;}
    s_stage1_display_ms=now;g_stage1_display_attempts++;
    g_scan_display_result=Screen_ShowTaskCode(g_stage1_task_code);
    if(g_scan_display_result==SCREEN_OK){g_scan_display_count++;s_stage1_display_done=1U;}
#else
    (void)screen_ready;
#endif
}
'''

POSE_CONFIG = '''#ifndef STARTUP_POSE_CONFIG_H
#define STARTUP_POSE_CONFIG_H
/* 用户2026-11-03指定：只改上电停放角，不改动作组135闭合/180张开。 */
#ifndef STARTUP_GRIPPER_ANGLE_DEG
#define STARTUP_GRIPPER_ANGLE_DEG 140.0f
#endif
/* 在这里填写车载盘上电角，范围0~270度；-1表示尚未确定，不输出位置PWM。
 * 示例：改为90.0f后重新编译下载；本项不改槽1/2/3的分度角。 */
#ifndef STARTUP_CARRIER_ANGLE_DEG
#define STARTUP_CARRIER_ANGLE_DEG (-1.0f)
#endif
#endif
'''

POSE_SOURCE = '''#include "startup_pose.h"
#include "app_config.h"
#include "../../USER/startup_pose_config.h"
#include "gripper.h"
#include "carrier_pwm.h"
#include "carrier_config.h"
volatile uint32_t g_startup_gripper_result=0U,g_startup_carrier_result=0U;
void StartupPose_Init(void)
{
#if !APP_MOTOR_OFFLINE_ONLY
    float angle=STARTUP_CARRIER_ANGLE_DEG;uint32_t pulse;
    Gripper_Init();g_startup_gripper_result=jiazhua(STARTUP_GRIPPER_ANGLE_DEG);
    /* 未配置时不强制转0度；初始化引脚为低，不发舵机位置脉冲。 */
    if(CarrierPwm_Init()!=1U){g_startup_carrier_result=2U;return;}
    if(angle==-1.0f)return;
    if(!(angle>=0.0f && angle<=CARRIER_ANGLE_MAX_DEG)){g_startup_carrier_result=3U;return;}
    pulse=CARRIER_PULSE_MIN_US+(uint32_t)(angle*(CARRIER_PULSE_MAX_US-CARRIER_PULSE_MIN_US)/CARRIER_ANGLE_MAX_DEG+0.5f);
    g_startup_carrier_result=CarrierPwm_Write(pulse)?1U:4U;
#endif
}
'''


def stage():
    assert not STAGE.exists()
    EV.mkdir(parents=True, exist_ok=True)
    original = sources(ACTUAL)
    for rel in original:
        dst=STAGE/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ACTUAL/rel,dst)
    edit('CHASSIS/USER/main.c', main_patch)
    old,_=read(STAGE/'CHASSIS/USER/stage1_runtime.inc')
    telemetry=old[old.index('static void App_Stage1Telemetry'):].replace('g_stage1_state>=3U?', '(g_stage1_state>=3U && g_stage1_state<=5U)?')
    (STAGE/'CHASSIS/USER/stage1_runtime.inc').write_text(RUNTIME+telemetry,encoding='utf-8')
    edit('CHASSIS/USER/vehicle_tuning.h',lambda s:replace(s,'#define VEHICLE_STAGE1_SCAN_AFTER_STOP 1U',
        '#define VEHICLE_STAGE1_SCAN_AFTER_STOP 0U\n#define VEHICLE_STAGE1_QR_STOP_TIMEOUT_MS 5000U'))
    edit('CHASSIS/USER/field_map.c',lambda s:replace(s,'hash=hash_word(hash,VEHICLE_STAGE1_SCAN_AFTER_STOP);',
        'hash=hash_word(hash,VEHICLE_STAGE1_SCAN_AFTER_STOP);\n    hash=hash_word(hash,VEHICLE_STAGE1_QR_STOP_TIMEOUT_MS);'))
    (STAGE/'USER/startup_pose_config.h').write_text(POSE_CONFIG,encoding='utf-8')
    (STAGE/'CHASSIS/USER/startup_pose.h').write_text('#ifndef STARTUP_POSE_H\n#define STARTUP_POSE_H\nvoid StartupPose_Init(void);\n#endif\n',encoding='utf-8')
    (STAGE/'CHASSIS/USER/startup_pose.c').write_text(POSE_SOURCE,encoding='utf-8')
    edit('HARDWARE/GRIPPER/gripper_config.h',lambda s:replace(s,'#define GRIPPER_CONFIG_H',
        '#define GRIPPER_CONFIG_H\n#include "../../USER/startup_pose_config.h"'))
    edit('USER/gripper_task.c',lambda s:replace(s,'g_jiazhua_debug_result=jiazhua(JIAZHUA_OPEN_ANGLE_DEG);',
        'g_jiazhua_debug_result=jiazhua(STARTUP_GRIPPER_ANGLE_DEG);').replace(
        '/* 沿用用户指定的调试启动180度张开，不由此宣称带料复位安全。 */',
        '/* 上电停放140度；普通抓放仍保留135度闭合和180度张开。 */'))
    edit('HARDWARE/CARRIER/carrier_config.h',lambda s:replace(s,'#define CARRIER_CONFIG_H',
        '#define CARRIER_CONFIG_H\n#include "../../USER/startup_pose_config.h"'))
    def carrier(s):
        s=replace(s,'    g_carrier_init_result = CarrierPwm_Init();',
            '    float angle=STARTUP_CARRIER_ANGLE_DEG;uint32_t pulse;\n    g_carrier_init_result = CarrierPwm_Init();')
        s=replace(s,'    if (CarrierPwm_Write(CARRIER_PULSE_MIN_US) != 1U)',
            '''    /* 未填写上电角时不转向默认0度；分度槽位配置保持原值。 */
    if(angle==-1.0f)return;
    if(!(angle>=0.0f && angle<=CARRIER_ANGLE_MAX_DEG))
    {g_carrier_state=9U;g_carrier_result=3U;return;}
    pulse=CARRIER_PULSE_MIN_US+(uint32_t)(angle*(CARRIER_PULSE_MAX_US-CARRIER_PULSE_MIN_US)/CARRIER_ANGLE_MAX_DEG+0.5f);
    if (CarrierPwm_Write(pulse) != 1U)''')
        s=s.replace('g_carrier_debug_angle = CARRIER_SLOT_1_DEG;', 'g_carrier_debug_angle = angle;')
        s=s.replace('g_carrier_latched_angle = CARRIER_SLOT_1_DEG;', 'g_carrier_latched_angle = angle;')
        s=s.replace('g_carrier_pulse_us = CARRIER_PULSE_MIN_US;', 'g_carrier_pulse_us = pulse;')
        s=s.replace('初始化后立即发0度','初始化后按集中配置输出停放角').replace('启动0度独立于','启动停放角独立于')
        return s
    edit('HARDWARE/CARRIER/carrier.c',carrier)
    tree=ET.parse(STAGE/PROJECT)
    for t in tree.findall('Targets/Target'):
        chassis=t.findtext('TargetName').startswith('Chassis_')
        for f in t.findall('.//Groups/Group/Files/File'):
            if chassis and f.findtext('FilePath') in ('.\\HARDWARE\\GRIPPER\\gripper.c','.\\HARDWARE\\CARRIER\\carrier_pwm.c'):
                f.find('FileOption/CommonProperty/IncludeInBuild').text='1'
        if chassis:
            inc=t.find('.//Cads/VariousControls/IncludePath')
            inc.text+=';.\\HARDWARE\\GRIPPER;.\\HARDWARE\\CARRIER'
        group=next(g for g in t.findall('Groups/Group') if g.findtext('GroupName')=='Chassis_USER')
        f=ET.SubElement(group.find('Files'),'File')
        ET.SubElement(f,'FileName').text='startup_pose.c';ET.SubElement(f,'FileType').text='1'
        ET.SubElement(f,'FilePath').text='.\\CHASSIS\\USER\\startup_pose.c'
        opt=ET.SubElement(f,'FileOption');com=ET.SubElement(opt,'CommonProperty')
        ET.SubElement(com,'IncludeInBuild').text='1' if chassis else '0'
    tree.write(STAGE/PROJECT,encoding='utf-8',xml_declaration=True)
    after=sources(STAGE)
    changed={rel:h for rel,h in after.items() if original.get(rel)!=h}
    (EV/'manifest.json').write_text(json.dumps(dict(original=original,changed=changed,stage=str(STAGE),
        actual=str(ACTUAL),installed=False,flashed=False,hardware_commands_sent=False,
        user_observation='Download succeeded; forward/left direction correct; scanner read code before endpoint; screen task code absent.',
        user_carrier_angle='Unknown; user will edit angle; -1 disables boot position output.'),ensure_ascii=False,indent=2),encoding='utf-8')
    print('staged changed files',list(changed),flush=True)


def build(folder,prefix):
    for name in ['Vehicle_Integrated','Vehicle_Offline']:
        log=EV/(prefix+'-'+name.replace(' ','_')+'.log')
        p=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(folder/PROJECT),'-t',name,'-j0','-o',str(log)],capture_output=True,timeout=180)
        s=log.read_text(encoding='utf-8',errors='replace');print(name,p.returncode,s[-400:],flush=True)
        assert p.returncode==0 and '0 Error(s), 0 Warning(s)' in s,s[-6000:]


def install():
    m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    assert (EV/'host-check.json').is_file()
    assert sources(ACTUAL)==m['original'],'Actual source changed; re-review needed'
    for name in ['Vehicle_Integrated','Vehicle_Offline']:
        assert '0 Error(s), 0 Warning(s)' in (EV/('stage-'+name+'.log')).read_text(errors='replace')
    for rel,h in m['changed'].items():
        assert sha(STAGE/rel)==h
        dest=ACTUAL/rel
        if dest.exists():
            backup=EV/'before-install'/rel;backup.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(dest,backup)
        dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(STAGE/rel,dest)
    for rel,h in m['original'].items():
        assert sha(ACTUAL/rel)==m['changed'].get(rel,h),rel
    m['installed']=True;(EV/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
    build(ACTUAL,'actual')


if __name__=='__main__':
    if '--install' in sys.argv:install()
    elif '--build' in sys.argv:build(STAGE,'stage')
    else:stage()
