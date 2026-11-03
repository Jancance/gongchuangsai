"""Stage the explicitly requested concurrent DM/extension preparation and VM hold."""
from pathlib import Path
import hashlib,json,xml.etree.ElementTree as E

R=Path(__file__).resolve().parents[1]
S=R/'tmp/stage1_early_scan_startup_20261103'
EV=R/'evidence/integration/2026-11-03/early-scan-startup'

HEADER='''#ifndef STARTUP_AXES_H
#define STARTUP_AXES_H
#include <stdint.h>
extern volatile uint32_t g_startup_axes_state,g_startup_axes_cause;
extern volatile uint32_t g_startup_axes_vm_request;
extern volatile float g_startup_extension_current_deg,g_startup_extension_target_deg;
uint32_t StartupAxes_Begin(void);
void StartupAxes_Cancel(void);
void StartupAxes_Service(uint32_t platform_ready,uint32_t can_guard_ready);
void StartupAxes_Safety(uint32_t healthy);
uint32_t StartupAxes_Holding(void);
uint32_t StartupAxes_NormalStopHold(void);
uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t length,uint8_t reply_length,void *reply);
#endif
'''

SOURCE=r'''#include "startup_axes.h"
#include "../../USER/startup_pose_config.h"
#include "app_config.h"
#include "dm_can2.h"
#include "can.h"
#include "chassis_auto_test.h"
#include "power.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>
/* 0未启动1供电2准备3并行运行4到位保持5失败6取消7未填写参考。
 * 按键前不使能、不设零、不自动回零。旧驱动坐标不能替代机械参考。 */
volatile uint32_t g_startup_axes_state=0U,g_startup_axes_cause=0U,g_startup_axes_vm_request=0U;
volatile float g_startup_extension_current_deg=0.0f,g_startup_extension_target_deg=0.0f;
static uint32_t s_begin_ms,s_last_service,s_phase,s_dm_ready,s_ext_ready,s_started;
static uint32_t s_poll_ms,s_settled,s_monitor;
static uint8_t s_options,s_move[15];
static CanRxMsg s_reply;
static float s_target,s_start;
static uint32_t s_ext_done;
extern volatile uint32_t g_dm_startup_go;

static uint32_t exchange(const uint8_t *q,uint8_t length,uint8_t n)
{return StartupAxis_Exchange(q,length,n,&s_reply);}
static uint32_t query(uint8_t command,uint8_t n)
{uint8_t q[2]={command,0x6b};return exchange(q,2U,n);}
static uint32_t control(const uint8_t *q,uint8_t n)
{return exchange(q,n,3U)==1U && (s_reply.Data[1]==2U || (q[0]==0xfd && s_reply.Data[1]==0x9f));}
static void fail(uint32_t cause)
{g_startup_axes_cause=cause;g_startup_axes_state=5U;g_startup_axes_vm_request=0U;g_dm_jog_request=3U;}
uint32_t StartupAxes_Holding(void)
{return g_startup_axes_state==4U && g_startup_axes_vm_request==1U;}
uint32_t StartupAxes_NormalStopHold(void)
{
    ChassisAutoStatus s;
    if(!StartupAxes_Holding())return 0U;
    (void)ChassisAuto_ReadStatus(&s);
    return !s.active && s.state==CHASSIS_AUTO_DONE && s.result==1U && !s.cause && s.rotation_stop_mask==15U;
}
uint32_t StartupAxes_Begin(void)
{
#if APP_MOTOR_OFFLINE_ONLY
    return 0U;
#else
    if(g_startup_axes_state!=0U)return 0U;
    /* 机械零参考及目标由操作者填写；不以默认0和粗标冒充已建立坐标。 */
    if(STARTUP_EXTENSION_REFERENCE_VALID!=1U ||
       !(STARTUP_EXTENSION_TARGET_MM>=0.0f && STARTUP_EXTENSION_TARGET_MM<=50.0f) ||
       !(STARTUP_EXTENSION_ZERO_DEG>=-10000.0f && STARTUP_EXTENSION_ZERO_DEG<=10000.0f) ||
       !(STARTUP_EXTENSION_EXTEND_SIGN==1 || STARTUP_EXTENSION_EXTEND_SIGN==-1))
    {g_startup_axes_state=7U;g_startup_axes_cause=1U;return 0U;}
    s_target=STARTUP_EXTENSION_ZERO_DEG+STARTUP_EXTENSION_EXTEND_SIGN*
        STARTUP_EXTENSION_TARGET_MM*32.0f*360.0f/3200.0f;
    g_startup_extension_target_deg=s_target;
    g_startup_axes_state=1U;g_startup_axes_cause=0U;g_startup_axes_vm_request=1U;
    s_begin_ms=s_last_service=(uint32_t)xTaskGetTickCount();
    s_phase=s_dm_ready=s_ext_ready=s_started=s_ext_done=s_settled=s_monitor=0U;
    g_dm_startup_go=0U;return 1U;
#endif
}
void StartupAxes_Cancel(void)
{g_startup_axes_state=6U;g_startup_axes_vm_request=0U;g_dm_jog_request=3U;}
void StartupAxes_Safety(uint32_t healthy)
{
    uint32_t now=(uint32_t)xTaskGetTickCount();
    if(g_startup_axes_state>=1U && g_startup_axes_state<=4U) {
        if(!healthy || (uint32_t)(now-s_last_service)>150U)fail(2U);
        else if(g_startup_axes_state!=4U && (uint32_t)(now-s_begin_ms)>=15000U)fail(3U);
    }
}
static uint32_t read_position(void)
{
    uint32_t raw;
    if(query(0x36U,7U)!=1U || s_reply.Data[1]>1U)return 0U;
    raw=((uint32_t)s_reply.Data[2]<<24)|((uint32_t)s_reply.Data[3]<<16)|((uint32_t)s_reply.Data[4]<<8)|s_reply.Data[5];
    g_startup_extension_current_deg=(s_reply.Data[1]?-1.0f:1.0f)*(float)raw*0.1f;return 1U;
}
void StartupAxes_Service(uint32_t platform_ready,uint32_t can_guard_ready)
{
#if !APP_MOTOR_OFFLINE_ONLY
    uint32_t now=(uint32_t)xTaskGetTickCount(),angle,scale;float delta;
    PowerCommandStatus power;const uint8_t enable[5]={0xf3,0xab,1,0,0x6b};
    const uint8_t stop[4]={0xfe,0x98,0,0x6b};
    if(!g_startup_axes_state || g_startup_axes_state==7U)return;
    s_last_service=now;
    if(g_startup_axes_state==5U || g_startup_axes_state==6U) {
        if(g_dm_jog_request || DM_CAN2_JogIsBusy() || g_dm_posvel_holding)DM_CAN2_ModuleService(0U,0U);
        if(s_started){(void)control(stop,4U);s_started=0U;}return;
    }
    if(!platform_ready){fail(2U);return;}
    Power_ReadCommandStatus(&power);
    if(!power.active || !can_guard_ready)return;
    if(g_startup_axes_state==1U){g_startup_axes_state=2U;}
    if(g_startup_axes_state==2U) {
        /* 一轮最多一个伸缩交易，DM与CAN1共用同一个任务，无并发FIFO访问。 */
        if(s_phase==0U) {
            if(query(0x6cU,4U)!=1U){fail(4U);return;}
            s_options=s_reply.Data[2];
            if(s_options&2U){fail(5U);return;}s_phase++;return;
        }
        if(s_phase==1U) {
            if(!read_position()){fail(4U);return;}s_start=g_startup_extension_current_deg;
            delta=s_target-s_start;
            if(!(delta>=-180.0f && delta<=180.0f)){fail(6U);return;}
            scale=(s_options&0x80U)?100U:10U;
            angle=(uint32_t)(fabsf(delta)*(float)scale+0.5f);
            memset(s_move,0,sizeof(s_move));s_move[0]=0xfd;s_move[1]=delta<0.0f;
            s_move[2]=s_move[4]=0xaf;s_move[3]=s_move[5]=0xff;
            s_move[6]=0x04;s_move[7]=0x24; /* 保留106.0RPM、45055RPM/s。 */
            s_move[8]=(uint8_t)(angle>>24);s_move[9]=(uint8_t)(angle>>16);s_move[10]=(uint8_t)(angle>>8);s_move[11]=(uint8_t)angle;
            s_move[12]=2U;s_move[14]=0x6b;
            if(fabsf(delta)<=0.2f)s_ext_done=1U;
            g_dm_point_enable=1U;g_dm_posvel_speed_deg_s=DM_CAN2_POSVEL_SPEED_DEG_S*0.5f;
            g_dm_jog_request=6U;s_phase++;s_started=1U;
        }
        DM_CAN2_ModuleService(1U,0U);
        if(g_dm_jog_state==8U){fail(7U);return;}
        if(!s_ext_ready) {if(!control(enable,5U)){fail(4U);return;}s_ext_ready=1U;}
        if(g_dm_jog_state==3U && g_dm_can2_motor_state==1U)s_dm_ready=1U;
        if(s_dm_ready && s_ext_ready) {
            /* 两轴准备屏障通过后同一服务轮释放；CAN1/CAN2并行运动，非硬件同步。 */
            g_dm_startup_go=1U;
            if(!s_ext_done && !control(s_move,15U)){fail(4U);return;}
            g_startup_axes_state=3U;s_poll_ms=now;
        }
        return;
    }
    DM_CAN2_ModuleService(1U,0U);
    if(g_dm_jog_state==8U){fail(7U);return;}
    if((uint32_t)(now-s_poll_ms)>=50U) {
        s_poll_ms=now;
        if(s_monitor==0U) {
            if(query(0x3aU,3U)!=1U || !(s_reply.Data[1]&1U) || (s_reply.Data[1]&0x0cU)) {fail(8U);return;}
        } else if(s_monitor==1U) {
            if(!read_position()){fail(4U);return;}
            if(fabsf(g_startup_extension_current_deg-s_target)>2.0f && g_startup_axes_state==4U){fail(9U);return;}
        } else {
            if(query(0x35U,5U)!=1U){fail(4U);return;}
            if((s_reply.Data[2] || s_reply.Data[3]) || fabsf(g_startup_extension_current_deg-s_target)>0.3f)s_settled=0U;
            else if(++s_settled>=3U)s_ext_done=1U;
        }
        s_monitor=(s_monitor+1U)%3U;
    }
    if(g_startup_axes_state==3U && s_ext_done && g_dm_jog_state==7U &&
       g_dm_jog_result==1U && g_dm_posvel_holding) {
        g_startup_axes_state=4U;g_dm_posvel_speed_deg_s=DM_CAN2_POSVEL_SPEED_DEG_S;
    }
#else
    (void)platform_ready;(void)can_guard_ready;
#endif
}
'''

TRANSPORT=r'''
/* 启动并行伸缩专用节点6，不扩展四轮控制接口；唯一CAN任务调用。 */
uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t length,uint8_t reply_length,void *out)
{
    CanRxMsg *reply=(CanRxMsg*)out,rx;CanTxMsg tx;uint8_t packet=0U,offset=1U,chunk,mailbox,status,flush;
    TickType_t start;uint32_t r;
    if(!q || !out)return 7U;
    if(length==2U && q[1]==0x6b &&
       ((q[0]==0x6c && reply_length==4U) || (q[0]==0x36 && reply_length==7U) ||
        (q[0]==0x3a && reply_length==3U) || (q[0]==0x35 && reply_length==5U)))
        return CAN1_ExchangeSingle(6U,q,length,reply_length,reply);
    if(length==4U && q[0]==0xfe && q[1]==0x98 && q[2]==0 && q[3]==0x6b)
        return CAN1_ExchangeSingle(6U,q,length,3U,reply);
#if APP_MOTOR_OFFLINE_ONLY
    return 9U;
#else
    if(!g_startup_axes_vm_request || g_startup_axes_state<2U || g_startup_axes_state>4U ||
       g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;
    if(length==5U && q[0]==0xf3 && q[1]==0xab && q[2]==1 && q[3]==0 && q[4]==0x6b)
        return CAN1_ExchangeSingle(6U,q,length,3U,reply);
    if(length!=15U || q[0]!=0xfd || q[1]>1 || q[12]!=2 || q[13]!=0 || q[14]!=0x6b || reply_length!=3U)return 7U;
    if(!s_can1_normal_ready){r=CAN1_InitMode(CAN_Mode_Normal);if(r!=1U)return r;s_can1_normal_ready=1U;}
    for(flush=0;flush<3 && CAN_MessagePending(CAN1,CAN_FIFO0);flush++)CAN_Receive(CAN1,CAN_FIFO0,&rx);
    while(offset<length) {
        memset(&tx,0,sizeof(tx));chunk=length-offset;if(chunk>7)chunk=7;
        tx.ExtId=0x0600U|packet;tx.IDE=CAN_Id_Extended;tx.RTR=CAN_RTR_Data;tx.DLC=chunk+1;tx.Data[0]=q[0];
        memcpy(&tx.Data[1],&q[offset],chunk);mailbox=CAN_Transmit(CAN1,&tx);if(mailbox>2)return 4U;
        start=xTaskGetTickCount();
        for(;;){status=CAN_TransmitStatus(CAN1,mailbox);if(status==CAN_TxStatus_Ok)break;
            if(status==CAN_TxStatus_Failed || xTaskGetTickCount()-start>=pdMS_TO_TICKS(20U)){CAN_CancelTransmit(CAN1,mailbox);return 4U;}
            vTaskDelay(pdMS_TO_TICKS(1U));}
        offset+=chunk;packet++;
    }
    start=xTaskGetTickCount();
    while(xTaskGetTickCount()-start<pdMS_TO_TICKS(30U)) {
        if(CAN_MessagePending(CAN1,CAN_FIFO0)) {
            CAN_Receive(CAN1,CAN_FIFO0,&rx);
            if(rx.IDE==CAN_Id_Extended && rx.RTR==CAN_RTR_Data && rx.ExtId==0x600 && rx.Data[0]==0xfd)
            { *reply=rx;return rx.DLC==3 && rx.Data[2]==0x6b ? 1U:6U; }
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }return 5U;
#endif
}
'''

def edit(rel,fn):
    p=S/rel;b=p.read_bytes()
    try:s=b.decode('utf-8');enc='utf-8'
    except UnicodeDecodeError:s=b.decode('gbk');enc='gbk'
    s=s.replace('\r\n','\n');p.write_bytes(fn(s).replace('\n','\r\n').encode(enc))

def scoped(s,name,fn):
    lo=s.index('static ',s.index(name)-20);start=s.index('{',s.index(name));end=start+1;depth=1
    while depth:depth+=(s[end]=='{')-(s[end]=='}');end+=1
    return s[:lo]+fn(s[lo:end])+s[end:]

def main(s):
    s=s.replace('#include "startup_pose.h"','#include "startup_pose.h"\n#include "startup_axes.h"',1)
    s=s.replace('s_motion_has_action || g_vm_test_request || power.active ||',
        's_motion_has_action || g_vm_test_request || (power.active && !StartupAxes_Holding()) ||')
    s=s.replace('if (status.active || status.result || status.cut_vm || power.active ||',
        'if (status.active || status.result || status.cut_vm || (power.active && !StartupAxes_Holding()) ||')
    s=s.replace('(g_vm_test_request == 0U) && (power.active == 0U) &&',
        '(g_vm_test_request == 0U) && ((power.active == 0U) || StartupAxes_Holding()) &&')
    for name in ['App_CanPollStep','App_CanPollRefresh']:
        s=scoped(s,name,lambda t:t.replace('(g_vm_test_request != 1U)','(g_vm_test_request != 1U && !g_startup_axes_vm_request)').replace(
            '(g_vm_test_request == 1U)','(g_vm_test_request == 1U || g_startup_axes_vm_request)'))
    s=s.replace('                App_CombinedTestStep();',
        '                StartupAxes_Service(g_safety_ready && g_can_test_result==1U,g_can_guard_ready);\n'
        '                if(g_startup_axes_state!=1U && g_startup_axes_state!=2U && g_startup_axes_state!=3U) App_CombinedTestStep();')
    s=s.replace('        App_CombinedTestSafety();',
        '        StartupAxes_Safety(!g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched);\n'
        '        App_CombinedTestSafety();')
    s=s.replace('g_stage1_state==2U || g_stage1_state==6U)', 'g_stage1_state==2U || g_stage1_state==6U || g_stage1_state==7U)')
    s=s.replace('if(g_stage1_state==1U || g_stage1_state==6U) g_combined_test.request=2U;',
        'if(g_stage1_state==7U) StartupAxes_Cancel();\n'
        '                    else if(g_stage1_state==1U || g_stage1_state==6U) g_combined_test.request=2U;')
    return s

def runtime(s):
    s=s.replace('power.active || g_vm_test_request', '(power.active && !StartupAxes_Holding()) || g_vm_test_request')
    s=s.replace('volatile uint32_t g_stage2_ready=0U,g_stage2_run_id=0U;',
        'volatile uint32_t g_stage2_ready=0U,g_stage2_run_id=0U;\nvolatile uint32_t g_stage1_scan_bypassed=0U;')
    s=s.replace('g_stage1_state==2U || g_stage1_state==6U)', 'g_stage1_state==2U || g_stage1_state==6U || g_stage1_state==7U)')
    s=s.replace('if(g_stage1_state!=2U) MotionStop();', 'if(g_stage1_state==7U)StartupAxes_Cancel();\n        else if(g_stage1_state!=2U) MotionStop();',1)
    s=s.replace('        s_serial_start_permit=1U;g_combined_test.request=1U;',
        '        s_serial_start_permit=0U;g_combined_test.request=0U;')
    s=s.replace('s_stage1_begin_ms=now;s_stage1_end_ms=0U;g_stage1_state=1U;',
        's_stage1_begin_ms=now;s_stage1_end_ms=0U;g_stage1_state=7U;\n'
        '        g_stage1_scan_bypassed=VEHICLE_SCAN_BYPASS;\n'
        '        g_serial_motion_state=0U;g_serial_motion_result=0U;\n'
        '        if(!StartupAxes_Begin()){g_stage1_state=4U;g_stage1_cause=9U;}')
    key='    if(g_stage1_state==1U || g_stage1_state==2U || g_stage1_state==6U || g_stage1_state==7U) {'
    s=s.replace(key,'''    if(g_stage1_state==7U) {
        if(g_startup_axes_state==5U || g_startup_axes_state==6U || g_startup_axes_state==7U)
        {App_Stage1Fail(9U,now);return;}
        if(StartupAxes_Holding()) {
            if(!App_SerialIdleReady(now)){App_Stage1Fail(9U,now);return;}
            s_serial_start_permit=1U;g_combined_test.request=1U;
            g_serial_motion_state=2U;g_serial_motion_result=2U;
            s_stage1_begin_ms=now;s_stage1_scan_baseline=g_saoma_accept_count;g_stage1_state=1U;
        } else return;
    }
'''+key)
    s=s.replace('        if(App_Stage1FreshCode(now)) {','        if(!g_stage1_scan_bypassed && App_Stage1FreshCode(now)) {')
    s=s.replace('if(g_stage1_scan_valid && s_stage1_display_done && App_Stage1Stopped())',
        'if((g_stage1_scan_bypassed || (g_stage1_scan_valid && s_stage1_display_done)) && App_Stage1Stopped())')
    s=s.replace('!g_stage1_scan_valid &&\n', '!g_stage1_scan_bypassed && !g_stage1_scan_valid &&\n')
    s=s.replace('g_stage1_state<=5U)?', 'g_stage1_state<=5U)?')
    return s

if __name__=='__main__':
    (S/'CHASSIS/USER/startup_axes.h').write_text(HEADER,encoding='utf-8')
    (S/'CHASSIS/USER/startup_axes.c').write_text(SOURCE,encoding='utf-8')
    edit('CHASSIS/USER/main.c',main)
    edit('CHASSIS/USER/stage1_runtime.inc',runtime)
    edit('CHASSIS/USER/vehicle_tuning.h',lambda s:s.replace('#define VEHICLE_STAGE1_ENABLE 1U',
        '#define VEHICLE_STAGE1_ENABLE 1U\n#ifndef VEHICLE_SCAN_BYPASS\n#define VEHICLE_SCAN_BYPASS 1U\n#endif'))
    edit('CHASSIS/USER/field_map.c',lambda s:s.replace('hash=hash_word(hash,VEHICLE_STAGE1_QR_STOP_TIMEOUT_MS);',
        'hash=hash_word(hash,VEHICLE_STAGE1_QR_STOP_TIMEOUT_MS);\n    hash=hash_word(hash,VEHICLE_SCAN_BYPASS);'))
    edit('USER/startup_pose_config.h',lambda s:s.replace('#endif\n', '#endif\n',1).rsplit('#endif',1)[0]+'''
/* 伸缩固定目标（毫米）。32脉冲/mm只是既有粗标，不是精度保证。
 * 先建立本次有效机械缩回端对应的驱动角参考，再填VALID=1；手推/失电后重核。
 * 默认0禁止并行启动，不会把未确定的目标作为0mm下发。 */
#define STARTUP_EXTENSION_REFERENCE_VALID 0U
#define STARTUP_EXTENSION_ZERO_DEG 0.0f
#define STARTUP_EXTENSION_EXTEND_SIGN (-1)
#define STARTUP_EXTENSION_TARGET_MM 0.0f
#endif
''')
    edit('HARDWARE/DM_CAN2/dm_can2_motion.c',lambda s:s.replace('volatile uint32_t g_dm_jog_request = 0U;',
        'volatile uint32_t g_dm_jog_request = 0U;\nvolatile uint32_t g_dm_startup_go=0U;').replace(
        'pdMS_TO_TICKS(50U)', 'pdMS_TO_TICKS(s_point_mode==6U?100U:50U)').replace(
        '        s_move_started = now;\n        g_dm_jog_state = 4U;',
        '        if(s_point_mode==6U && !g_dm_startup_go){JogReference(s_start,now);return;}\n'
        '        s_move_started = now;\n        g_dm_jog_state = 4U;'))
    edit('CHASSIS/SYSTEM/CAN/can.c',lambda s:s.replace('#include "app_config.h"',
        '#include "app_config.h"\n#include "startup_axes.h"',1)+TRANSPORT)
    def safety(s):
        s=s.replace('#include "app_config.h"','#include "app_config.h"\n#include "startup_axes.h"',1)
        if '#include "startup_axes.h"' not in s:s='#include "startup_axes.h"\n'+s
        s=s.replace(' ? 0U : vm_request);\n    g_safety_ready',
            ' ? (StartupAxes_NormalStopHold()?g_startup_axes_vm_request:0U) : (vm_request || g_startup_axes_vm_request));\n    g_safety_ready')
        s=s.replace('(g_safety_ready == 1U) && (vm_request == 1U)',
            '(g_safety_ready == 1U) && (vm_request == 1U || g_startup_axes_vm_request==1U)')
        s=s.replace('(g_chassis_auto_cut_vm == 0U))\n    {\n        Power_Enable();',
            '((g_chassis_auto_cut_vm == 0U) || StartupAxes_NormalStopHold()))\n    {\n        Power_Enable();')
        return s
    edit('CHASSIS/SYSTEM/SAFETY/safety.c',safety)
    tree=E.parse(S/'gongxun-f407.uvprojx')
    for t in tree.findall('Targets/Target'):
        chassis=t.findtext('TargetName').startswith('Chassis_')
        if chassis:
            inc=t.find('.//Cads/VariousControls/IncludePath');inc.text+=';.\\HARDWARE\\DM_CAN2'
            for f in t.findall('.//Groups/Group/Files/File'):
                if (f.findtext('FilePath') or '').startswith('.\\HARDWARE\\DM_CAN2\\'):
                    f.find('FileOption/CommonProperty/IncludeInBuild').text='1'
        g=next(g for g in t.findall('Groups/Group') if g.findtext('GroupName')=='Chassis_USER')
        f=E.SubElement(g.find('Files'),'File')
        for tag,text in [('FileName','startup_axes.c'),('FileType','1'),('FilePath','.\\CHASSIS\\USER\\startup_axes.c')]:E.SubElement(f,tag).text=text
        option=E.SubElement(f,'FileOption');common=E.SubElement(option,'CommonProperty');E.SubElement(common,'IncludeInBuild').text='1' if chassis else '0'
    tree.write(S/'gongxun-f407.uvprojx',encoding='utf-8',xml_declaration=True)
    m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    for p in S.rglob('*'):
        if p.is_file() and p.suffix.lower() in {'.c','.h','.s','.inc','.uvprojx'} and not {'Objects','Listings'}.intersection(p.relative_to(S).parts):
            rel=p.relative_to(S).as_posix();h=hashlib.sha256(p.read_bytes()).hexdigest()
            if m['original'].get(rel)!=h:m['changed'][rel]=h
    (EV/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
