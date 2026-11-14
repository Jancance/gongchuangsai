from pathlib import Path
import hashlib,json

R=Path.cwd();U=R/'3.stm32f407工程/CHASSIS/USER';E=Path(__file__).resolve().parent
sha=lambda b:hashlib.sha256(b).hexdigest()
before={n:(U/n).read_bytes() for n in ['chassis_auto_test.c','chassis_auto_test.h']}
assert sha(before['chassis_auto_test.c'])=='49a1c2bfcfb5f5b8fd7716444f395d6a88653a1309d985aab6c3b1334abacd94'
for n,data in before.items():(E/(n+'.before')).write_bytes(data)

header='''/* Three baseline40 tail events, debug RAM only; F is reference/request/applied,
 * never measured speed or a complete ACK batch. Slots: entry/latest freeze/arrival.
 * Retained after STOP/fault; reset only by an accepted matching session. */
typedef struct
{
    uint32_t now_ms, run_tick_ms, common_ms, follow_tick_ms;
    float common_mean_mm, reference_position_mm, reference_rpm;
    float follow_rpm, follow_rate_rpm_s;
    uint32_t pending_base_rpm, applied_base_rpm, flags;
} ChassisAutoTailSnapshot;
typedef struct
{
    uint32_t format_version, session, valid_mask;
    uint32_t entry_count, freeze_count, arrival_count, reserved[2];
    ChassisAutoTailSnapshot slot[3];
} ChassisAutoTailSnapshots;
#define CHASSIS_TAIL_PROFILE_ACTIVE 1U
#define CHASSIS_TAIL_PROFILE_DONE 2U
#define CHASSIS_TAIL_RUN_STARTED 4U
#define CHASSIS_TAIL_REACHED_BEFORE_EVENT 8U
#define CHASSIS_TAIL_STOP_DUE_BEFORE_EVENT 16U
#define CHASSIS_TAIL_FINISH_BEFORE_EVENT 32U
#define CHASSIS_TAIL_ALIGNMENT_SEEN 64U
#define CHASSIS_TAIL_COMMON_FRESH 128U
typedef char ChassisAutoTailSizeCheck[(sizeof(ChassisAutoTailSnapshot)==48U &&
    sizeof(ChassisAutoTailSnapshots)==176U) ? 1 : -1];
extern volatile ChassisAutoTailSnapshots g_chassis_auto_tail_snapshots;
void ChassisAuto_ReadTailSnapshots(ChassisAutoTailSnapshots *snapshot);
'''
helpers='''volatile ChassisAutoTailSnapshots g_chassis_auto_tail_snapshots;
static void Auto_TailSnapshotsResetLocked(void)
{
    volatile uint8_t *bytes=(volatile uint8_t *)&g_chassis_auto_tail_snapshots;
    uint32_t i;
    for(i=0U;i<sizeof(g_chassis_auto_tail_snapshots);i++) bytes[i]=0U;
    g_chassis_auto_tail_snapshots.format_version=1U;
    g_chassis_auto_tail_snapshots.session=g_chassis_auto_session;
}
static void Auto_TailSnapshotRecordLocked(uint32_t slot, TickType_t now)
{
    volatile ChassisAutoTailSnapshot *p=&g_chassis_auto_tail_snapshots.slot[slot];
    uint32_t ms=(uint32_t)now*portTICK_PERIOD_MS;
    if(!s_lateral_enabled || !s_profile_active || !g_chassis_auto_active ||
       g_chassis_auto_tail_snapshots.session!=g_chassis_auto_session ||
       (g_chassis_auto_tail_snapshots.valid_mask&4U) ||
       (slot==0U && (g_chassis_auto_tail_snapshots.valid_mask&1U))) return;
    p->now_ms=ms; p->run_tick_ms=(uint32_t)s_run_tick*portTICK_PERIOD_MS;
    p->common_ms=s_alignment_ms;
    p->follow_tick_ms=(uint32_t)s_profile_follow_tick*portTICK_PERIOD_MS;
    p->common_mean_mm=s_distance_progress_mm;
    p->reference_position_mm=s_distance_reference.position_mm;
    p->reference_rpm=s_distance_reference.speed_mm_s/s_profile_mm_per_rpm_s;
    p->follow_rpm=s_profile_follow_rpm; p->follow_rate_rpm_s=s_profile_follow_rate;
    p->pending_base_rpm=s_translation_base_rpm; p->applied_base_rpm=s_translation_applied_base;
    p->flags=(s_profile_active?CHASSIS_TAIL_PROFILE_ACTIVE:0U) |
        (s_profile_done?CHASSIS_TAIL_PROFILE_DONE:0U) |
        (s_run_started?CHASSIS_TAIL_RUN_STARTED:0U) |
        (s_distance_reached?CHASSIS_TAIL_REACHED_BEFORE_EVENT:0U) |
        (s_stop_due?CHASSIS_TAIL_STOP_DUE_BEFORE_EVENT:0U) |
        (s_finish_requested?CHASSIS_TAIL_FINISH_BEFORE_EVENT:0U) |
        (s_alignment_seen?CHASSIS_TAIL_ALIGNMENT_SEEN:0U) |
        (s_alignment_seen && ms-s_alignment_ms<400U?CHASSIS_TAIL_COMMON_FRESH:0U);
    g_chassis_auto_tail_snapshots.valid_mask|=1UL<<slot;
    if(slot==0U) g_chassis_auto_tail_snapshots.entry_count++;
    else if(slot==1U) g_chassis_auto_tail_snapshots.freeze_count++;
    else g_chassis_auto_tail_snapshots.arrival_count++;
}
void ChassisAuto_ReadTailSnapshots(ChassisAutoTailSnapshots *snapshot)
{
    if(!snapshot) return;
    taskENTER_CRITICAL();
    *snapshot=g_chassis_auto_tail_snapshots;
    taskEXIT_CRITICAL();
}
'''
changes=[]
def replace(name,old,new):
    global before
    data=after[name];assert data.count(old)==1,(name,old[:90]);after[name]=data.replace(old,new);changes.append(dict(file=name,old_hex=old.hex(),new_hex=new.hex()))
after=before.copy()
replace('chassis_auto_test.h',b'void ChassisAuto_ReadLateralTiming(ChassisAutoLateralTiming *snapshot);\n',b'void ChassisAuto_ReadLateralTiming(ChassisAutoLateralTiming *snapshot);\n'+header.encode('gbk'))
cr=lambda t:t.replace('\n','\r\n').encode('gbk')
replace('chassis_auto_test.c',cr('static int32_t s_lateral_applied_deci;\n'),cr('static int32_t s_lateral_applied_deci;\n'+helpers))
replace('chassis_auto_test.c',cr('        s_profile_done = (result == MOTION_PROFILE_DONE);\n'),cr('        s_profile_done = (result == MOTION_PROFILE_DONE);\n#if APP_BASELINE40_ACTIVE\n        if(elapsed>=s_distance_profile.boundary_s[4] && mean>=s_distance_target_mm-120.0f)\n            Auto_TailSnapshotRecordLocked(0U,now);\n#endif\n'))
replace('chassis_auto_test.c',cr('            s_distance_reached = 1U; s_finish_requested = s_stop_due = 1U;\n'),cr('''#if APP_BASELINE40_ACTIVE
            Auto_TailSnapshotRecordLocked(2U,now);
#endif
            s_distance_reached = 1U; s_finish_requested = s_stop_due = 1U;
'''))
replace('chassis_auto_test.c',cr('        state = CHASSIS_AUTO_START;\n    }\n\n    if ((state == CHASSIS_AUTO_START || state == CHASSIS_AUTO_RUN) && s_stop_due)'),cr('''        state = CHASSIS_AUTO_START;
#if APP_BASELINE40_ACTIVE
        if(g_chassis_auto_tail_snapshots.valid_mask&1U) Auto_TailSnapshotRecordLocked(1U,now);
#endif
    }

    if ((state == CHASSIS_AUTO_START || state == CHASSIS_AUTO_RUN) && s_stop_due)'''))
replace('chassis_auto_test.c',cr('            s_lateral_begin_ms=(*AppMotion_Live()->begin_ms)*portTICK_PERIOD_MS;\n'),cr('            s_lateral_begin_ms=(*AppMotion_Live()->begin_ms)*portTICK_PERIOD_MS;\n            if(s_lateral_enabled) Auto_TailSnapshotsResetLocked();\n'))
for n,data in after.items():
    assert data.decode('gbk').encode('gbk')==data
    if n.endswith('.c'):assert data.count(b'\n')==data.count(b'\r\n')
    else:assert data.count(b'\r\n')==before[n].count(b'\r\n')==1
    (U/n).write_bytes(data)
(E/'source_changes.json').write_text(json.dumps(changes,indent=2),'utf8')
(E/'source_manifest.json').write_text(json.dumps({n:dict(before_sha256=sha(before[n]),after_sha256=sha(after[n]),before_bytes=len(before[n]),after_bytes=len(after[n])) for n in before},indent=2),'utf8')
print((E/'source_manifest.json').read_text())
