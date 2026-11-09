from pathlib import Path
import shutil,json,hashlib,subprocess,re,sys
R=Path(__file__).resolve().parents[1]; A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程'); S=R/'tmp/dm_posvel_20261101/project'; E=R/'evidence/p2-arm/2026-11-01/dm-posvel'; E.mkdir(parents=True,exist_ok=True)
F=['HARDWARE/DM_CAN2/dm_can2.h','HARDWARE/DM_CAN2/dm_can2.c','HARDWARE/DM_CAN2/dm_can2_protocol.c','HARDWARE/DM_CAN2/dm_can2_motion.c','USER/main.c','USER/arm_sequence.c','USER/arm_sequence.h']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(m): (E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
def build(d,n):
 log=E/n
 z=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(d/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],capture_output=True,timeout=180)
 t=log.read_text(encoding='utf-8',errors='replace');assert z.returncode==0 and '0 Error(s), 0 Warning(s)' in t,t[-3000:]
 print(t[-450:],flush=True)
if '--install' in sys.argv:
 m=json.loads((E/'manifest.json').read_text(encoding='utf-8'));assert m['host_pass'] and m['stage_build_pass']
 for f in F:assert sha(A/f)==m['before'][f] and sha(S/f)==m['after'][f]
 for f,h in m['protected'].items():assert sha(A/f)==h,f
 for f in F:shutil.copy2(S/f,A/f)
 m['installed']=True;save(m);build(A,'actual-build.log')
 for f,h in m['protected'].items():assert sha(A/f)==h,f
 m['actual_build_pass']=True;save(m);print('Installed and built; no flash or hardware commands.');sys.exit()


assert S.parent.name == "dm_posvel_20261101"
shutil.copytree(A,S,dirs_exist_ok=True,ignore=shutil.ignore_patterns('Objects','Listings','*.uvguix.*','.git'))
m={'before':{f:sha(A/f) for f in F},'protected':{str(p.relative_to(A)).replace(chr(92),'/'):sha(p) for p in A.rglob('*') if p.is_file() and p.suffix.lower() in ['.c','.h','.uvprojx'] and str(p.relative_to(A)).replace(chr(92),'/') not in F},'host_pass':False,'stage_build_pass':False,'installed':False,'actual_build_pass':False,'flash_performed':False,'physical_acceptance':False}
for f in F:
 b=E/'original'/f;b.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(A/f,b)
def once(t,a,b):assert t.count(a)==1,a[:120];return t.replace(a,b,1)
def edit(f,fn):
 p=S/f;t=p.read_bytes().decode('utf-8').replace('\r\n','\n');p.write_bytes(fn(t).replace('\n','\r\n').encode('utf-8'))
def header(t):
 t=once(t,'#define DM_CAN2_MODE          1U','#define DM_CAN2_MODE          2U')
 t=t.replace('/* 本机已实读：接收ID1、反馈ID0、MIT模式1、经典CAN 1Mbps。 */','/* 已实读ID1/反馈0、CAN1Mbps；本版目标为位置速度模式2，待板端读回。 */')
 t=once(t,'extern volatile uint32_t g_dm_point_enable;',"""extern volatile uint32_t g_dm_point_enable;
/* 参考Tower_task的G6220_Position(...,300)，单位度/秒；接单锁存。 */
#define DM_CAN2_POSVEL_SPEED_DEG_S 300.0F
extern volatile float g_dm_posvel_speed_deg_s;
extern volatile float g_dm_posvel_latched_speed_deg_s;
extern volatile uint32_t g_dm_mode_change_state;
uint32_t DM_CAN2_BuildPosVel(float position, float speed, CanTxMsg *tx);
uint32_t DM_CAN2_BuildMode2(CanTxMsg *tx);""")
 t=t.replace('4平滑运动','4驱动器位置速度运动').replace('5末端停稳','5持续等待末端停稳').replace('8位置/跟随越界，9速度/力矩超限','8位置越界，9保留')
 t=t.replace('14点位范围/方向不符。','14点位范围/方向不符，15模式切换未确认。')
 return t
edit(F[0],header)
def reader(t):
 return once(t,'if (value != s_expected[s_index]) { DM_CAN2_Fail(10U); return; }',"""/* 只读允许识别原MIT或目标模式；仅动作入口在失能下切换到2。 */
                if ((s_index < 2U && value != s_expected[s_index]) ||
                    (s_index == 2U && value != 1U && value != DM_CAN2_MODE))
                { DM_CAN2_Fail(10U); return; }""")
edit(F[1],reader)
def proto(t):
 t=once(t,'tx->StdId = DM_CAN2_NODE_ID;\n    tx->IDE = CAN_Id_Standard;', 'tx->StdId = 0x100U + DM_CAN2_NODE_ID;\n    tx->IDE = CAN_Id_Standard;') if False else t
 # 特殊命令仅FC/FD函数使用模式偏移，旧MIT编码函数保留备份兼容但不再被动作调用。
 a=t.index('uint32_t DM_CAN2_BuildSwitch(');b=t.index('uint32_t DM_CAN2_BuildMitTrajectory(',a)
 part=t[a:b];part=once(part,'tx->StdId = DM_CAN2_NODE_ID;','tx->StdId = 0x100U + DM_CAN2_NODE_ID;');t=t[:a]+part+t[b:]
 t+='\n'+"""/* 只支持本次指定的CTRL_MODE=2，不提供任意参数写或保存零点入口。 */
uint32_t DM_CAN2_BuildMode2(CanTxMsg *tx)
{
    if (!DM_CAN2_BuildRead(0x0AU, tx)) return 0U;
    tx->Data[2] = 0x55U;
    tx->Data[4] = 2U;
    return 1U;
}

uint32_t DM_CAN2_BuildPosVel(float position, float speed, CanTxMsg *tx)
{
    uint32_t p, v;
    if (!tx || !(position >= -12.5F && position <= 12.5F) ||
        !(speed >= 0.0F && speed <= DM_CAN2_POSVEL_SPEED_DEG_S * 0.01745329252F)) return 0U;
    memcpy(&p, &position, sizeof(p));
    memcpy(&v, &speed, sizeof(v));
    memset(tx, 0, sizeof(*tx));
    tx->StdId = 0x100U + DM_CAN2_NODE_ID;
    tx->IDE = CAN_Id_Standard; tx->RTR = CAN_RTR_Data; tx->DLC = 8U;
    tx->Data[0] = (uint8_t)p; tx->Data[1] = (uint8_t)(p >> 8);
    tx->Data[2] = (uint8_t)(p >> 16); tx->Data[3] = (uint8_t)(p >> 24);
    tx->Data[4] = (uint8_t)v; tx->Data[5] = (uint8_t)(v >> 8);
    tx->Data[6] = (uint8_t)(v >> 16); tx->Data[7] = (uint8_t)(v >> 24);
    return 1U;
}
"""
 return t
edit(F[2],proto)
def motion(t):
 t=once(t,'volatile uint32_t g_dm_point_enable = 0U;',"""volatile uint32_t g_dm_point_enable = 0U;
volatile float g_dm_posvel_speed_deg_s = DM_CAN2_POSVEL_SPEED_DEG_S;
volatile float g_dm_posvel_latched_speed_deg_s = 0.0F;
volatile uint32_t g_dm_mode_change_state = 0U;
static uint32_t s_mode_pending = 0U, s_mode_attempted = 0U;
static TickType_t s_mode_tick;
static float s_speed;""")
 t=t.replace('s_started, s_step, s_move_started, s_settle_started, s_last_service','s_started, s_step, s_move_started, s_last_service')
 t=once(t,'    if (JogAbs(s_reference - s_feedback.position_rad) > 8.0F * JOG_RAD_PER_DEG)', '    if (0)') if False else t
 t=once(t,"""    if (s_feedback.position_rad < low || s_feedback.position_rad > high ||
        JogAbs(s_reference - s_feedback.position_rad) > 8.0F * JOG_RAD_PER_DEG) return 8U;
    if (JogAbs(s_feedback.velocity_rad_s) > 0.5F || JogAbs(s_feedback.torque_nm) > 1.0F)
        return 9U;""","""    /* 位置速度模式直接发最终目标，不能再套MIT参考跟随8度判停。
     * 用户指定参考速度300度/秒，不沿用旧低速0.5rad/s阈值。 */
    if (s_feedback.position_rad < low || s_feedback.position_rad > high) return 8U;""")
 a=t.index('static void JogReference(');b=t.index('static void JogService(',a)
 t=t[:a]+"""static void JogReference(float reference, TickType_t now)
{
    CanTxMsg tx;
    s_reference = reference;
    g_dm_jog_reference_deg = reference * JOG_DEG_PER_RAD;
    if (!DM_CAN2_BuildPosVel(reference, s_speed, &tx) || !JogSend(&tx, now)) JogStop(5U, now);
}

"""+t[b:]
 t=t.replace('float ratio, current_work_deg, delta_deg;','float current_work_deg, delta_deg;')
 t=once(t,'        s_point_mode = (request == 4U || request == 5U) ? request : 0U;',"""        s_mode_pending = 0U; s_mode_attempted = 0U; g_dm_mode_change_state = 0U;
        if (!(g_dm_posvel_speed_deg_s > 0.0F && g_dm_posvel_speed_deg_s <= DM_CAN2_POSVEL_SPEED_DEG_S))
        { g_dm_jog_state = 8U; g_dm_jog_result = 13U; return; }
        g_dm_posvel_latched_speed_deg_s = g_dm_posvel_speed_deg_s;
        s_speed = g_dm_posvel_latched_speed_deg_s * JOG_RAD_PER_DEG;
        s_point_mode = (request == 4U || request == 5U) ? request : 0U;""")
 # 用户明确要求动作无期限等待；原通信/服务异常不是动作等待超时。
 t=once(t,"""        if ((TickType_t)(now - s_started) > pdMS_TO_TICKS(s_point_mode ? 23000U : 12000U))
        { JogStop(10U, now); return; }
""",'        /* 用户指定：动作/到位等待不设总期限，显式停止仍可中止。 */\n')
 marker='    if (g_dm_jog_state == 1U)\n'
 t=once(t,marker,"""    if (s_mode_pending)
    {
        if (s_mode_pending == 1U)
        {
            reply = CAN_TransmitStatus(CAN2, s_mailbox);
            if (reply == CAN_TxStatus_Failed ||
                (reply != CAN_TxStatus_Ok && (TickType_t)(now - s_step) >= pdMS_TO_TICKS(30U)))
            { s_mode_pending = 0U; JogStop(5U, now); return; }
            if (reply != CAN_TxStatus_Ok) return;
            s_pending = 0U; s_mailbox = CAN_TxStatus_NoMailBox;
            s_mode_pending = 2U; s_mode_tick = now;
        }
        if ((TickType_t)(now - s_mode_tick) < pdMS_TO_TICKS(50U)) return;
        s_mode_pending = 0U; s_cycle = g_dm_can2_read_cycle;
        g_dm_can2_read_request = 2U;
        g_dm_mode_change_state = 2U;
        return;
    }
    if (g_dm_jog_state == 1U)
""")
 t=once(t,'g_dm_can2_ctrl_mode != 1U ||','(g_dm_can2_ctrl_mode != 1U && g_dm_can2_ctrl_mode != DM_CAN2_MODE) ||')
 marker='        s_pmax = g_dm_can2_pmax;'
 t=once(t,marker,"""        if (g_dm_can2_ctrl_mode != DM_CAN2_MODE)
        {
            if (s_mode_attempted) { g_dm_mode_change_state = 4U; JogStop(15U, now); return; }
            /* 当前新鲜状态已确认失能且零速，只切RAM模式，不保存Flash/零点。 */
            s_mode_attempted = 1U; g_dm_mode_change_state = 1U;
            DM_CAN2_CancelRead();
            if (!DM_CAN2_BuildMode2(&tx) || !JogSend(&tx, now)) { JogStop(5U, now); return; }
            s_mode_pending = 1U; return;
        }
        g_dm_mode_change_state = 3U;
"""+marker)
 t=once(t,'DM_CAN2_BuildMitPreload(s_start, s_pmax, s_vmax, s_tmax, &tx)','DM_CAN2_BuildPosVel(s_start, 0.0F, &tx)')
 a=t.index('    if (g_dm_jog_state == 3U)\n');b=t.index('\n}\n\n/* 同圈',a)
 t=t[:a]+"""    if (g_dm_jog_state == 3U)
    {
        if (s_feedback.motor_state != 1U) { JogStop(7U, now); return; }
        s_move_started = now;
        g_dm_jog_state = 4U;
    }
    if (g_dm_jog_state == 4U || g_dm_jog_state == 5U)
    {
        g_dm_jog_elapsed_ms = (uint32_t)((now - s_move_started) * portTICK_PERIOD_MS);
        if (JogAbs(s_target - s_feedback.position_rad) <= 3.0F * JOG_RAD_PER_DEG &&
            JogAbs(s_feedback.velocity_rad_s) <= 0.035F) s_settled++;
        else s_settled = 0U;
        if (s_settled >= 15U) { s_clean_done = 1U; JogStop(1U, now); return; }
        /* 与参考代码一致持续发最终位置和速度上限，未到位持续等待。 */
        JogReference(s_target, now);
    }"""+t[b:]
 # 老动作时长变量不再用于规划。
 t=t.replace('static uint32_t s_point_mode = 0U, s_duration_ms = DM_CAN2_JOG_TIME_MS;','static uint32_t s_point_mode = 0U;').replace('        s_duration_ms = s_point_mode ? DM_CAN2_POINT_TIME_MS : DM_CAN2_JOG_TIME_MS;\n','')
 # 停止优先时清除模式等待，避免中止后再触发读/写。
 t=once(t,'    s_pending = 0U;\n    DM_CAN2_CancelRead();','    s_pending = 0U;\n    s_mode_pending = 0U;\n    DM_CAN2_CancelRead();')
 return t
edit(F[3],motion)
p=S/F[4];t=p.read_bytes();t=once(t,b'DM_CAN2_JogIsBusy() ? 2U : 10U',b'DM_CAN2_JogIsBusy() ? 12U : 10U');t=t.replace('/* 旋转时提高服务频率，其他机构保持原节拍。 */'.encode('gb18030'),'/* 位置速度模式按参考约12毫秒重发目标，其他机构保持原节拍。 */'.encode('gb18030'),1);p.write_bytes(t)
def arm(t):
 a=t.index('static uint32_t ConfirmTimeout(void)');b=t.index('static void Fail(',a);t=t[:a]+t[b:]
 import re
 t,n=re.subn(r'else if\(now-s_phase>(?:ConfirmTimeout\(\)|15000U|13000U|1500U)\)Fail\(7U,9U\);','/* 用户指定：等待确认/子动作不设动作层超时。 */',t)
 t=t.replace('if(!s_ticket){if(now-s_phase>1500U)Fail(5U,9U);break;}','if(!s_ticket){break;}')
 t=t.replace('else if(!s_settling && now-s_phase>1500U)Fail(7U,9U);','/* 等待夹爪结果，不设动作层超时。 */')
 assert 'ConfirmTimeout' not in t and 'now-s_phase>' not in t.replace('now-s_phase>=',''), 'remaining deadline'
 return t
edit(F[5],arm)
m['after']={f:sha(S/f) for f in F};save(m)

edit(F[6],lambda t:t.replace('/* 仅模式2看护调试：state2/state12人工确认窗口120秒。正式运行不使用此入口。 */','/* 用户指定：state2/state12及动作层等待无期限；显式停止仍有效。 */'))
m['after']={f:sha(S/f) for f in F};save(m)
