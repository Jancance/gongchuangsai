"""一次性迁移当前主工程；历史冻结资料不改，不操作硬件。"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
FW = ROOT / '3.stm32f407工程'
RENAMES = {'USER/zdt_x_config.h':'USER/zdt_emm_config.h',
           'USER/zdt_x_protocol.h':'USER/zdt_emm_protocol.h',
           'USER/x_load_diag.h':'USER/chassis_load_diag.h',
           'USER/x_load_diag.c':'USER/chassis_load_diag.c'}
REPLACE = {'zdt_x_config.h':'zdt_emm_config.h','zdt_x_protocol.h':'zdt_emm_protocol.h',
 'ZDT_X_CONFIG_H':'ZDT_EMM_CONFIG_H','ZDT_X_PROTOCOL_H':'ZDT_EMM_PROTOCOL_H',
 'APP_X_OFFLINE_ONLY':'APP_MOTOR_OFFLINE_ONLY',
 'ZDT_X_APP_MAX_SPEED_RAW':'ZDT_EMM_APP_MAX_SPEED_RAW',
 'ZDT_X_POSITION_UNITS_PER_REV':'ZDT_EMM_POSITION_UNITS_PER_REV',
 'ZdtX_':'ZdtEmm_','Chassis_EncodeX':'Chassis_EncodeEmm',
 'CAN1_SetXSpeedRaw':'CAN1_SetEmmSpeedRaw','CAN1_StageXSpeedRaw':'CAN1_StageEmmSpeedRaw',
 'CAN1_XSpeed':'CAN1_EmmSpeed','CAN1_ReadXSystemSnapshot':'CAN1_ReadEmmSystemSnapshot',
 'x_load_diag':'chassis_load_diag','X_LOAD_DIAG_H':'CHASSIS_LOAD_DIAG_H',
 'XLoad':'ChassisLoad','XLOAD_':'CHASSIS_LOAD_','XSystemSnapshot':'ChassisSystemSnapshot'}

def read(p):
    b=p.read_bytes()
    try:return b.decode('gbk'),'gbk'
    except UnicodeDecodeError:return b.decode('utf8'),'utf8'

for p in FW.rglob('*'):
    if not p.is_file() or p.suffix not in {'.c','.h','.inc','.uvprojx'}:continue
    if any(x in p.parts for x in ['FreeRTOS','StdPeriph_Driver','Objects','ObjectsOffline','Listings','ListingsOffline']):continue
    if p.suffix=='.uvprojx':t,enc=p.read_bytes().decode('utf8'),'utf8'
    else:t,enc=read(p)
    u=t
    for a,b in REPLACE.items():u=u.replace(a,b)
    if u!=t:p.write_bytes(u.encode(enc))

for a,b in RENAMES.items():
    p,q=FW/a,FW/b
    assert p.exists() and not q.exists(), '迁移脚本仅执行一次'
    q.write_bytes(p.read_bytes());p.unlink()

def put(rel,t):
    (FW/rel).write_bytes(t.replace('\r\n','\n').replace('\n','\r\n').encode('gbk'))

put('USER/zdt_emm_config.h','''#ifndef ZDT_EMM_CONFIG_H
#define ZDT_EMM_CONFIG_H
/* 离线工程禁止VM、使能及非零速度，不触碰驱动持久参数。 */
#ifndef APP_MOTOR_OFFLINE_ONLY
#define APP_MOTOR_OFFLINE_ONLY 1U
#endif
/* 四台驱动人工确认FW_Emm及S_Vel_IS=Enable后，再改1并重编译。
 * 未确认可停止/失能及只读查询，禁止使能和非零速度。 */
#ifndef ZDT_EMM_SETTINGS_CONFIRMED
#define ZDT_EMM_SETTINGS_CONFIRMED 0U
#endif
#if ZDT_EMM_SETTINGS_CONFIRMED > 1U
#error Invalid_Emm_settings_confirmation
#endif
#define ZDT_EMM_SPEED_INPUT_DIV10 1U
/* 保留原底盘300RPM软件上限，不因协议切换自动提速。 */
#define ZDT_EMM_APP_MAX_SPEED_RAW 3000U
#define ZDT_EMM_POSITION_UNITS_PER_REV 65536U
/* Emm加速度为档位，236档按手册公式名义约1000RPM/s。
 * 不可沿用原1000数值或截断成8位；实际带载斜率待验证。 */
#define ZDT_EMM_MOTION_ACCELERATION 236U
#endif
''')

put('USER/zdt_emm_protocol.h','''#ifndef ZDT_EMM_PROTOCOL_H
#define ZDT_EMM_PROTOCOL_H
#include <stdint.h>
#include <stddef.h>
#include "zdt_emm_config.h"
/* CAN负载不含地址：Emm速度帧7字节，输入须缩小10倍。
 * 加速度为档位而非RPM/s；超过255直接拒绝，避免静默截断。 */
static __inline uint32_t ZdtEmm_EncodeSpeed(uint8_t dir,uint16_t speed,uint16_t accel,
    uint8_t sync,uint8_t out[7])
{
 if(!out || dir>1U || speed>ZDT_EMM_APP_MAX_SPEED_RAW || accel>255U || sync>1U)return 0U;
 out[0]=0xF6U;out[1]=dir;out[2]=(uint8_t)(speed>>8);out[3]=(uint8_t)speed;
 out[4]=(uint8_t)accel;out[5]=sync;out[6]=0x6BU;return 1U;
}
static __inline uint32_t ZdtEmm_EncodeSyncTrigger(uint8_t out[3])
{ if(!out)return 0U;out[0]=0xFFU;out[1]=0x66U;out[2]=0x6BU;return 1U; }
/* 速度缩放仅影响输入；Emm实时反馈为整数RPM，不能再除10。 */
static __inline uint32_t ZdtEmm_DecodeSpeed(const uint8_t *p,uint32_t n,float *rpm)
{
 uint32_t raw;
 if(!rpm)return 0U;*rpm=0.0f;
 if(!p || n!=5U || p[0]!=0x35U || p[1]>1U || p[4]!=0x6BU)return 0U;
 raw=((uint32_t)p[2]<<8)|p[3];*rpm=(float)raw;if(p[1])*rpm= -*rpm;return 1U;
}
/* Emm位置原生65536计数/圈，直接传递，不做0.1度位置换算。 */
static __inline uint32_t ZdtEmm_DecodePosition(const uint8_t *p,uint32_t n,
    uint8_t *sign,uint32_t *counts)
{
 if(!sign || !counts)return 0U;*sign=0U;*counts=0U;
 if(!p || n!=7U || p[0]!=0x36U || p[1]>1U || p[6]!=0x6BU)return 0U;
 *sign=p[1];*counts=((uint32_t)p[2]<<24)|((uint32_t)p[3]<<16)|((uint32_t)p[4]<<8)|p[5];return 1U;
}
static __inline uint32_t ZdtEmm_DecodeAck(uint8_t function,const uint8_t *p,uint32_t n)
{ return p && n==3U && p[0]==function && p[1]==0x02U && p[2]==0x6BU; }
/* 日志统一0.1RPM编码，不表示反馈也有0.1RPM分辨率。 */
static __inline int32_t ZdtEmm_RpmDeci(float rpm)
{
 if(!(rpm>=-65535.0f && rpm<=65535.0f))return (-2147483647L-1L);
 return (int32_t)(rpm<0?rpm*10.0f-0.5f:rpm*10.0f+0.5f);
}
typedef struct {
 int32_t speed_decirpm;
 uint32_t position_counts,position_deci_deg;
 uint16_t phase_ma,bus_mv;
 uint8_t position_sign,home_flags,motor_flags;
} ZdtEmmSystemData;
/* Emm系统包共31字节，去地址后30字节；拒绝其他固件格式。
 * 仅日志位置换成0.1度，里程读取始终用原生计数。 */
static __inline uint32_t ZdtEmm_DecodeSystem(const uint8_t *p,uint32_t n,ZdtEmmSystemData *s)
{
 uint32_t raw;
 if(!s)return 0U;*s=(ZdtEmmSystemData){0};
 if(!p || n!=30U || p[0]!=0x43U || p[1]!=31U || p[2]!=9U || p[29]!=0x6BU ||
    p[9]>1U || p[14]>1U || p[17]>1U || p[22]>1U)return 0U;
 raw=((uint32_t)p[15]<<8)|p[16];s->speed_decirpm=(int32_t)(raw*10U);
 if(p[14])s->speed_decirpm= -s->speed_decirpm;
 s->position_counts=((uint32_t)p[18]<<24)|((uint32_t)p[19]<<16)|((uint32_t)p[20]<<8)|p[21];
 s->position_deci_deg=(uint32_t)(((uint64_t)s->position_counts*3600U+32768U)/65536U);
 s->position_sign=p[17];s->phase_ma=(uint16_t)(((uint16_t)p[5]<<8)|p[6]);
 s->bus_mv=(uint16_t)(((uint16_t)p[3]<<8)|p[4]);s->home_flags=p[27];s->motor_flags=p[28];return 1U;
}
#endif
''')

p=FW/'SYSTEM/CAN/can.c';t,enc=read(p)
t=t.replace('/* X实时转速只读：CAN回帧35、符号、速度高低字节、6B */','/* Emm实时转速为整数RPM，输入缩放不影响反馈单位。 */')
t=t.replace('/* X当前位置只读；符号与32位幅值分开，避免int32溢出 */','/* Emm实时位置为65536计数/圈，符号与幅值分开。 */')
t=t.replace('/* X原始速度控制，方向0为CW、1为CCW */','/* Emm速度控制：0为CW、1为CCW，输入缩小10倍。 */')
t=t.replace('uint8_t request[8];','uint8_t request[7];')
t=t.replace(' if(speed_raw>ZDT_EMM_APP_MAX_SPEED_RAW',' if(speed_raw && !ZDT_EMM_SETTINGS_CONFIRMED)return 9U;\r\n if(speed_raw>ZDT_EMM_APP_MAX_SPEED_RAW')
t=t.replace('CAN1_ControlSingle(address,request,8U,reply)','CAN1_ControlSingle(address,request,7U,reply)')
needle='    request[2] = enabled;';assert needle in t
t=t.replace(needle,'''    if(enabled) {
        CanRxMsg check_reply;float checked_rpm;uint32_t checked;
        if(!ZDT_EMM_SETTINGS_CONFIRMED)return 9U;
        checked=CAN1_ReadEmmSystemSnapshot(address,&check_reply,&checked_rpm);
        if(checked!=1U)return checked;
    }
'''.replace('\n','\r\n')+'\r\n'+needle)
t=t.replace('uint8_t data[36]={0};uint32_t raw;','uint8_t data[30]={0};ZdtEmmSystemData decoded;')
t=t.replace('rx.DLC-offset>36U','rx.DLC-offset>30U')
a=t.index('        if(length==36U){');b=t.index('\n    }\r\nfinished:',a)
t=t[:a]+'''        if(length==30U){
            if(!ZdtEmm_DecodeSystem(data,length,&decoded)){result=6U;break;}
            sample.speed_decirpm=decoded.speed_decirpm;
            sample.position_deci_deg=decoded.position_deci_deg;sample.position_sign=decoded.position_sign;
            sample.phase_ma=decoded.phase_ma;sample.bus_mv=decoded.bus_mv;
            sample.home_flags=decoded.home_flags;sample.motor_flags=decoded.motor_flags;
            *rpm=(float)sample.speed_decirpm*0.1f;result=1U;break;
        }'''.replace('\n','\r\n')+t[b:]
p.write_bytes(t.encode(enc))

p=FW/'USER/chassis_auto_test.c';t,enc=read(p)
old='1000U, &reply); /* X RPM/s; outer profile unchanged. */';assert old in t
t=t.replace(old,'ZDT_EMM_MOTION_ACCELERATION, &reply); /* Emm加速度档位，外层轨迹保持。 */');p.write_bytes(t.encode(enc))

p=FW/'USER/chassis_kinematics.h';t,enc=read(p)
t=t.replace('X固件：速度原始单位为0.1RPM，不使用Emm的S_Vel_IS选项。','Emm须启用S_Vel_IS，命令0.1RPM；反馈仍为整数RPM。');p.write_bytes(t.encode(enc))
p=FW/'SYSTEM/CAN/can.h';t,enc=read(p)
t=t.replace('返回带符号RPM，float；X原始幅值已经除10。','返回带符号整数RPM的float，Emm反馈不除10。')
t=t.replace('返回符号及归一化65536计数/圈，原始X角度不直接进入里程。','返回符号及原生65536计数/圈，直接进入内部里程。')
t=t.replace('X速度控制；speed_raw是原始命令值，暂不承诺为RPM','Emm速度控制：输入须缩小10倍，加速度是0..255档');p.write_bytes(t.encode(enc))

p=ROOT/'工创赛/scripts/check_map_stage1.py';t=p.read_text(encoding='utf8')
t=t.replace('APP_X_OFFLINE_ONLY','APP_MOTOR_OFFLINE_ONLY')
t=t.replace("runner=base[base.index('reports=[]'):]", "runner=base[base.index('reports=[]'):].replace('APP_X_OFFLINE_ONLY','APP_MOTOR_OFFLINE_ONLY')")
p.write_text(t,encoding='utf8',newline='\r\n')
print('迁移源文件完成；未操作硬件。')
