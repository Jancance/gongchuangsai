"""Latch the first valid code without interrupting the planned nominal/search leg."""
from pathlib import Path
import json,hashlib,shutil
R=Path(__file__).resolve().parents[1];A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/scan_continue_20261104';E=R/'evidence/integration/2026-11-04/scan-continue-protection-audit'
E.mkdir(parents=True,exist_ok=True);assert not S.exists()
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','.git'))
rel='CHASSIS/USER/vehicle_mission.inc';p=S/rel;b=(A/rel).read_bytes()
try:u=b.decode('utf-8');enc='utf-8'
except UnicodeDecodeError:u=b.decode('gbk');enc='gbk'
u=u.replace('\r\n','\n')
for old in ['            if(g_vehicle_state==2U)g_stage1_stop_requested=1U;','        if(g_vehicle_stage==1U && s_vehicle_code_valid && !g_vehicle_scan_bypassed)g_stage1_stop_requested=1U;']:
 assert u.count(old)==1;u=u.replace(old,'')
u=u.replace('            memcpy(s_vehicle_task_code,SAOMA_Data,16);s_vehicle_code_valid=1U;',
'''            /* 本轮只锁首个合法码；当前名义/搜索动作走完，再停车交接。 */
            memcpy(s_vehicle_task_code,SAOMA_Data,16);s_vehicle_code_valid=1U;''')
p.write_bytes(u.replace('\n','\r\n').encode(enc));(E/'before-vehicle_mission.inc').write_bytes(b)
(E/'manifest.json').write_text(json.dumps(dict(actual=str(A),stage=str(S),original={rel:hashlib.sha256(b).hexdigest()},changed={rel:hashlib.sha256(p.read_bytes()).hexdigest()},installed=False,flashed=False,code_latch_without_stop=True,protective_stops_unchanged=True),ensure_ascii=False,indent=2),encoding='utf-8');print(S)
