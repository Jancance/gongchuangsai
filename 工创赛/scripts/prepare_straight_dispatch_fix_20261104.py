"""Stage the isolated straight-test dispatch gate fix, preserving all motor protections."""
from pathlib import Path
import json,hashlib,shutil
R=Path(__file__).resolve().parents[1];A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/straight_dispatch_fix_20261104';E=R/'evidence/integration/2026-11-04/straight-dispatch-fix'
E.mkdir(parents=True,exist_ok=True);assert not S.exists()
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','.git'))
rel='CHASSIS/USER/main.c';p=S/rel;b=(A/rel).read_bytes()
try:u=b.decode('utf-8');enc='utf-8'
except UnicodeDecodeError:u=b.decode('gbk');enc='gbk'
u=u.replace('\r\n','\n')
old='if(!VEHICLE_EXTENSION_HOME_ONLY && !VEHICLE_DM_STARTUP_ONLY && !VEHICLE_STARTUP_GROUP_ONLY && !VehicleArm_Busy() && (!VEHICLE_INTEGRATED_ENABLE || StartupAxes_Holding()) && g_startup_axes_state!=1U && g_startup_axes_state!=2U && g_startup_axes_state!=3U) App_CombinedTestStep();'
assert u.count(old)==1
u=u.replace(old,'if(App_VehicleCombinedAllowed()) App_CombinedTestStep();')
marker='static void App_CanTask(void *argument)';assert u.count(marker)==1
u=u.replace(marker,'''/* 独立直线测试不启动机构，仅轴状态0可使用底盘；原整合模式仍须持位。 */
static uint32_t App_VehicleCombinedAllowed(void)
{
    return !VEHICLE_EXTENSION_HOME_ONLY && !VEHICLE_DM_STARTUP_ONLY && !VEHICLE_STARTUP_GROUP_ONLY &&
        !VehicleArm_Busy() &&
        ((!VEHICLE_INTEGRATED_ENABLE || StartupAxes_Holding()) ||
         (VEHICLE_STRAIGHT_TEST_ENABLE && g_startup_axes_state==0U)) &&
        g_startup_axes_state!=1U && g_startup_axes_state!=2U && g_startup_axes_state!=3U;
}
'''+marker)
p.write_bytes(u.replace('\n','\r\n').encode(enc));(E/'before-main.c').write_bytes(b)
(E/'manifest.json').write_text(json.dumps(dict(actual=str(A),stage=str(S),original={rel:hashlib.sha256(b).hexdigest()},changed={rel:hashlib.sha256(p.read_bytes()).hexdigest()},installed=False,flashed=False),ensure_ascii=False,indent=2),encoding='utf-8');print(S)
