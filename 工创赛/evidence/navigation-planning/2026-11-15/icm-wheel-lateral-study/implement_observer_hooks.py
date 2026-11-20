from pathlib import Path

R=Path.cwd(); U=R/'3.stm32f407工程/CHASSIS/USER'
def mutate(path, old, new, count=1):
    b=path.read_bytes(); old=old.encode('ascii'); new=new.encode('ascii')
    assert b.count(old)==count,(path,b.count(old))
    path.write_bytes(b.replace(old,new))

# Existing files are byte-edited; no decode/re-encode of neighboring GBK text.
mutate(U/'main.c', b'#include "app_odometry.h"'.decode(), '#include "app_odometry.h"\r\n#include "app_icm_observer.h"')
mutate(U/'main.c', '        IcmRaw_GetStatus(&snapshot);',
       '        IcmRaw_GetStatus(&snapshot);\r\n#if APP_BASELINE40_ACTIVE\r\n        AppIcmObserver_Step(&snapshot);\r\n#endif')
mutate(U/'app_serial_runtime.c', '#include "icm45686_raw.h"',
       '#include "icm45686_raw.h"\r\n#include "app_icm_observer.h"')
telemetry=r'''/* IO1 is diagnostic only; no retries or commands. Millimetres scale by 1000. */
static int32_t App_ObserverMilli(float value)
{
    if(!(value>=-2000000.0f && value<=2000000.0f))return (-2147483647L-1L);
    return (int32_t)(value*1000.0f);
}
static void App_IcmObserverTelemetry(void)
{
    static char frame[384];static uint32_t last,sent,sequence,dropped;
    AppIcmObserverStatus o; int32_t values[27];
    uint32_t now=(uint32_t)xTaskGetTickCount(),phase=now-s_serial_tx_tick,i,used=4U;
    int n;uint16_t crc;
    if((sent && (uint32_t)(now-last)<500U) || !App_SerialDiagnosticWindow(now,40U) || phase>55U)return;
    AppIcmObserver_Read(&o);
    values[0]=(int32_t)sequence++;values[1]=(int32_t)now;values[2]=1;
    values[3]=(int32_t)o.sequence;values[4]=(int32_t)o.session;
    values[5]=(int32_t)o.valid;values[6]=(int32_t)o.reasons;values[7]=(int32_t)o.flags;
    values[8]=(int32_t)o.estimate_ms;values[9]=(int32_t)o.sample_sequence;
    values[10]=(int32_t)o.reference_ms;values[11]=(int32_t)o.t0_ms;values[12]=(int32_t)o.t1_ms;
    values[13]=(int32_t)o.wheel_age_ms;values[14]=(int32_t)o.hwt_age_ms;
    values[15]=(int32_t)o.cal_samples;values[16]=(int32_t)o.history_samples;
    values[17]=(int32_t)o.gaps;
    for(i=18U;i<26U;i++)values[i]=(-2147483647L-1L);
    if(o.flags&ICM_OBS_WHEEL){values[18]=App_ObserverMilli(o.wheel_f_mm);values[19]=App_ObserverMilli(o.wheel_r_mm);}
    if(o.valid){values[20]=App_ObserverMilli(o.position_f_mm);values[21]=App_ObserverMilli(o.position_r_mm);
        values[22]=App_ObserverMilli(o.velocity_f_mm_s);values[23]=App_ObserverMilli(o.velocity_r_mm_s);}
    if(o.flags&ICM_OBS_ACCEL){values[24]=App_ObserverMilli(o.accel_f_mm_s2);values[25]=App_ObserverMilli(o.accel_r_mm_s2);}
    values[26]=(int32_t)dropped;memcpy(frame,"IO1,",4U);
    for(i=0U;i<27U;i++) {
        n=snprintf(frame+used,sizeof(frame)-used,"%ld,",(long)values[i]);
        if(n<0 || (uint32_t)n>=sizeof(frame)-used){dropped++;return;}used+=(uint32_t)n;
    }
    crc=App_SerialCrc(frame,used);n=snprintf(frame+used,sizeof(frame)-used,"%04X\r\n",crc);
    if(n<0 || (uint32_t)n>=sizeof(frame)-used || !App_DebugSendString(frame)){dropped++;return;}
    last=now;sent=1U;
}
'''.replace('\n','\r\n')
mutate(U/'app_serial_runtime.c','/* IR1: software association times; DWT cycles are not sensor ADC timestamps. */',telemetry+'/* IR1: software association times; DWT cycles are not sensor ADC timestamps. */')
mutate(U/'app_serial_runtime.c','        if(!VEHICLE_INTEGRATED_ENABLE)App_PerimeterTelemetry();\r\n#endif\r\n#endif\r\n}',
       '        if(!VEHICLE_INTEGRATED_ENABLE)App_PerimeterTelemetry();\r\n#endif\r\n#endif\r\n#if APP_BASELINE40_ACTIVE\r\n        App_IcmObserverTelemetry();\r\n#endif\r\n}')
project=R/'3.stm32f407工程/gongxun-f407.uvprojx'
entry='''            <File>\r\n              <FileName>app_icm_observer.c</FileName>\r\n              <FileType>1</FileType>\r\n              <FilePath>.\\CHASSIS\\USER\\app_icm_observer.c</FilePath>\r\n            </File>\r\n'''
needle='''            <File>\r\n              <FileName>app_odometry.c</FileName>'''
mutate(project,needle,entry+needle,count=2)
for name in ('app_icm_observer.c','app_icm_observer.h'):
    path=U/name;path.write_bytes(path.read_bytes().replace(b'\r\n',b'\n').replace(b'\n',b'\r\n'))
print('main/serial hooks, two project entries, new ASCII/CRLF observer completed')
