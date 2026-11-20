from pathlib import Path
R=Path(__file__).resolve().parents[2]; U=R/'3.stm32f407工程/CHASSIS/USER'
def edit(p,pairs,encoding='cp936'):
    b=p.read_bytes(); crlf=b.count(b'\n')==b.count(b'\r\n');s=b.decode(encoding).replace('\r\n','\n')
    for old,new in pairs:
        assert s.count(old)==1,(p.name,old,s.count(old));s=s.replace(old,new)
    p.write_bytes((s.replace('\n','\r\n') if crlf else s).encode(encoding))
edit(U/'main.c', [('#if !ICM45686_SAMPLE_ONLY\n#if APP_BASELINE40_ACTIVE && ICM45686_SAMPLE_ONLY\n#error Baseline40_cannot_use_sample_only\n#endif','#if APP_BASELINE40_ACTIVE && ICM45686_SAMPLE_ONLY\n#error Baseline40_cannot_use_sample_only\n#endif\n#if !ICM45686_SAMPLE_ONLY')])
edit(R/'3.stm32f407工程/CHASSIS/HARDWARE/ICM45686/icm45686_raw.h', [('int IcmRaw_Init(void);','/* Published by main RAW task; equal even publication guards the entire snapshot. */\nuint32_t AppIcm_ReadPublished(IcmRawStatus *status, uint32_t *publication);\nint IcmRaw_Init(void);')])
raw='''#if APP_BASELINE40_ACTIVE
/* IR1: software association times; DWT cycles are not sensor ADC timestamps. */
static void App_IcmRawTelemetry(void)
{
    static char frame[512];static IcmRawStatus sample;
    static uint32_t sequence,last,last_sample,dropped,seen;
    uint32_t now=(uint32_t)xTaskGetTickCount(),publication,used,i;
    uint32_t words[24];int n;uint16_t crc;
    if ((uint32_t)(now-last)<200U || !App_SerialDiagnosticWindow(now,40U)) return;
    last=now;
    if (!AppIcm_ReadPublished(&sample,&publication) || !sample.sample_sequence ||
        (seen && sample.sample_sequence==last_sample)) return;
    sample.age_ms=now-sample.sample_tick_ms;
    sample.fresh=sample.fresh && sample.age_ms<=20U;
    words[0]=sequence++;words[1]=now;words[2]=publication;words[3]=sample.sample_sequence;
    words[4]=sample.state;words[5]=sample.last_error;words[6]=sample.failed_reg;words[7]=sample.whoami;
    words[8]=sample.accel_config;words[9]=sample.gyro_config;words[10]=sample.big_endian;
    words[11]=sample.edge_sequence;words[12]=sample.edge_cycle;
    words[13]=sample.read_start_cycle;words[14]=sample.read_end_cycle;
    words[15]=sample.sample_tick_ms;words[16]=sample.age_ms;words[17]=sample.fresh;
    words[18]=sample.transfer_errors;words[19]=sample.skipped_edges;words[20]=sample.read_races;
    words[21]=dropped;words[22]=seen ? sample.sample_sequence-last_sample-1U : 0U;
    words[23]=SystemCoreClock;
    memcpy(frame,"IR1,",4U);used=4U;
    for (i=0U;i<31U;i++) {
        if (i<24U) n=snprintf(frame+used,sizeof(frame)-used,"%lu,",(unsigned long)words[i]);
        else n=snprintf(frame+used,sizeof(frame)-used,"%ld,",(long)(i<27U ? sample.raw_accel[i-24U] :
            (i<30U ? sample.raw_gyro[i-27U] : sample.raw_temperature)));
        if (n<0 || (uint32_t)n>=sizeof(frame)-used) {dropped++;return;}used+=(uint32_t)n;
    }
    crc=App_SerialCrc(frame,used);n=snprintf(frame+used,sizeof(frame)-used,"%04X\\r\\n",crc);
    if (n<0 || (uint32_t)n>=sizeof(frame)-used || !App_DebugSendString(frame)) {dropped++;return;}
    last_sample=sample.sample_sequence;seen=1U;
}
static void App_Baseline40Telemetry(void)
{
    static char frame[160];static uint32_t last,sent,sequence;
    uint32_t now=(uint32_t)xTaskGetTickCount();int n;uint16_t crc;
    if ((sent && now-last<1000U) || !App_SerialDiagnosticWindow(now,40U)) return;
    n=snprintf(frame,sizeof(frame),"BF1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
        (unsigned long)sequence++,(unsigned long)now,(unsigned long)g_debug_plan_id,
        (unsigned long)APP_BASELINE40_RPM,(unsigned long)APP_BASELINE40_MM,(unsigned long)APP_BASELINE40_MM_S,
        (unsigned long)APP_BASELINE40_ACCEL_MM_S2,(unsigned long)APP_BASELINE40_JERK_MM_S3,
        (unsigned long)APP_BASELINE40_FEEDBACK_RPM,(unsigned long)APP_BASELINE40_RUN_MS);
    if (n<0 || (uint32_t)n>=sizeof(frame)) return;
    crc=App_SerialCrc(frame,(uint32_t)n);
    if (snprintf(frame+n,sizeof(frame)-(uint32_t)n,"%04X\\r\\n",crc)<0) return;
    if (App_DebugSendString(frame)) {last=now;sent=1U;}
}
#endif

'''
edit(U/'app_serial_runtime.c',[
('#include "field_map.h"','#include "field_map.h"\n#include "icm45686_raw.h"'),
('    if(strncmp(line,"RQ1,",4U)==0) { App_RuntimeRouteCommand(line);return; }','    if(strncmp(line,"RQ1,",4U)==0) {\n        if(APP_BASELINE40_ACTIVE) {App_SerialReject(6U);return;}\n        App_RuntimeRouteCommand(line);return;\n    }'),
('!AppRuntimeRoute_Live()->ready || AppRuntimeRoute_Live()->staging || AppRuntimeRoute_Live()->consumed','(!APP_BASELINE40_ACTIVE && (!AppRuntimeRoute_Live()->ready || AppRuntimeRoute_Live()->staging || AppRuntimeRoute_Live()->consumed)) ||\n                 (APP_BASELINE40_ACTIVE && (g_debug_plan_id!=DEBUG_PLAN_FORWARD210_40 ||\n                  *AppMotion_Live()->action_count!=1U))'),
('#if APP_TEST_MODE == 3U\n        App_SerialReject(6U);\n#else','#if APP_TEST_MODE == 3U && !APP_BASELINE40_ACTIVE\n        App_SerialReject(6U);\n#else'),
('static void App_WheelLogTelemetry(void)',raw+'static void App_WheelLogTelemetry(void)'),
('    if (APP_SPEED20_TRIAL_ACTIVE && g_debug_plan_id==DEBUG_PLAN_FORWARD210_20)','    if (APP_BASELINE40_ACTIVE && g_debug_plan_id==DEBUG_PLAN_FORWARD210_40)\n        values[39]=(int32_t)APP_BASELINE40_FEEDBACK_RPM;\n    if (APP_SPEED20_TRIAL_ACTIVE && g_debug_plan_id==DEBUG_PLAN_FORWARD210_20)'),
('    if (g_motion_plan_valid != 1U) values[37] = 2;','    if (APP_BASELINE40_ACTIVE && g_debug_plan_id==DEBUG_PLAN_FORWARD210_40)\n        values[57]=40000; /* 40cm/s reference ceiling, not wheel/ground measurement. */\n    if (g_motion_plan_valid != 1U) values[37] = 2;'),
('        App_WheelLogTelemetry();','        App_WheelLogTelemetry();\n#if APP_BASELINE40_ACTIVE\n        App_IcmRawTelemetry();\n        App_Baseline40Telemetry();\n#endif')])
fields=['seq','tick_ms','publication_sequence','sample_sequence','state','last_error','failed_reg','whoami','accel_config','gyro_config','big_endian','edge_sequence','edge_cycle','read_start_cycle','read_end_cycle','sample_tick_ms','age_ms','fresh','transfer_errors','skipped_edges','read_races','telemetry_dropped','sample_gap','dwt_hz','accel_x_raw','accel_y_raw','accel_z_raw','gyro_x_raw','gyro_y_raw','gyro_z_raw','temperature_raw']
parser='''ICM_FIELDS = %r
BASELINE40_FIELDS = ["seq", "tick_ms", "plan_id", "base_rpm", "distance_mm", "reference_mm_s",
                     "acceleration_mm_s2", "jerk_mm_s3", "feedback_limit_rpm", "run_ms"]

def parse_baseline40(line: bytes):
    if not line.startswith((b"IR1,", b"BF1,")):
        return None
    body, sep, checksum = line.rstrip(b"\\r\\n").rpartition(b",")
    if not sep or len(checksum) != 4 or binascii.crc_hqx(body+b",", 0xffff) != int(checksum, 16):
        raise ValueError("baseline40 CRC mismatch")
    raw = line.startswith(b"IR1,")
    fields = ICM_FIELDS if raw else BASELINE40_FIELDS
    values = list(map(int, body.split(b",")[1:]))
    if len(values) != len(fields):
        raise ValueError("baseline40 field count")
    result = dict(zip(fields, values))
    for key, value in result.items():
        if raw and key.endswith("_raw"):
            if not -32768 <= value <= 32767:
                raise ValueError("IR1 signed raw range")
        elif not 0 <= value <= 0xffffffff:
            raise ValueError("baseline40 uint32 range")
    if raw:
        if result["publication_sequence"] & 1 or not result["sample_sequence"] or result["state"] > 3 or result["fresh"] > 1 or result["big_endian"] > 1:
            raise ValueError("IR1 state/publication")
        if ((result["tick_ms"]-result["sample_tick_ms"]) & 0xffffffff) != result["age_ms"]:
            raise ValueError("IR1 sample time/age mismatch")
        if result["fresh"] and result["age_ms"] > 20:
            raise ValueError("IR1 stale marked fresh")
        if not result["dwt_hz"]:
            raise ValueError("IR1 missing cycle timebase")
    elif values[2:] != [8, 100, 2100, 400, 150, 300, 130, 12779]:
        raise ValueError("BF1 configuration mismatch")
    return result


''' % fields
edit(R/'工创赛/scripts/vehicle_uart_capture.py',[
('def parse_straight_test(line: bytes):',parser+'def parse_straight_test(line: bytes):'),
('for parser in [parse_straight_test,','for parser in [parse_baseline40, parse_straight_test,'),
('        if tag != "TEXT" and tag != "ERROR" and any(k.startswith("field_") for k in data):','        if tag == "IR1":\n            previous = getattr(self, "last_icm_sample", None)\n            if previous is not None:\n                delta = (data["sample_sequence"] - previous) & 0xffffffff\n                row["sample_duplicate"] = delta == 0\n                row["sample_out_of_order"] = delta >= 0x80000000\n                row["sample_sequence_gap"] = delta-1 if 0 < delta < 0x80000000 else 0\n            if previous is None or 0 < ((data["sample_sequence"]-previous) & 0xffffffff) < 0x80000000:\n                self.last_icm_sample = data["sample_sequence"]\n        if tag != "TEXT" and tag != "ERROR" and any(k.startswith("field_") for k in data):')], 'utf8')
print('Baseline40 protocol installed.')
