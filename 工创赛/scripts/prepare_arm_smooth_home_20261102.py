from pathlib import Path
import hashlib,json,subprocess,sys,shutil
R=Path(__file__).resolve().parents[1]
A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/arm_parameters_20261102/project'
E=R/'evidence/p2-arm/2026-11-02/parameters-145-104000-27000'
n='HARDWARE/LIFT/lift_home.c'
m=json.loads((E/'manifest.json').read_text(encoding='utf-8'))
assert not m['installed']
backup=E/'original'/n;backup.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(A/n,backup)
m['before'][n]=m['protected'].pop(n)
p=S/n;s=p.read_text(encoding='utf-8')
assert s.count('    /* 原停止请求')==1
s=s.replace('    /* 原停止请求','service_step:\n    /* 原停止请求',1)
old='    default: break;\n    }\n}'
assert s.count(old)==1
s=s.replace(old,'''    default: break;
    }
    /* 准备步骤只向前推进：每次CAN核验成功即接下一步，省去任务周期间隔。
     * 每步重读停止请求及时间；保留参数核验、通信失败和2500ms准备期限。
     * 进入运动/轮询阶段后返回任务，不在本函数等待回零完成。 */
    if (g_lift_home_state >= 2U && g_lift_home_state <= 5U)
    {
        request = g_lift_home_request;
        g_lift_home_request = 0U;
        now = xTaskGetTickCount();
        goto service_step;
    }
}''')
p.write_text(s,encoding='utf-8');m['after'][n]=hashlib.sha256(p.read_bytes()).hexdigest();m['stage_build_pass']=False
(E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
t=(R/'scripts/validate_lift_home_wait_speed_20261101.py').read_text(encoding='utf-8')
t=t.replace('static unsigned options, start_fail, seen_running=1;', 'static unsigned options, start_fail, seen_running=1, slow_params, stop_options, bad_options;')
t=t.replace('Lift_ReadHomeParametersNow(void) {return 1;}', 'Lift_ReadHomeParametersNow(void) {if(slow_params)now+=2600;return 1;}')
t=t.replace('r->Data[2]=(uint8_t)options;return 1;', 'r->Data[2]=(uint8_t)options;if(stop_options)g_lift_home_request=2;return bad_options?5:1;')
t=t.replace(' g_lift_home_request=1;step(1);', ''' if(!strcmp(s,"prepare_timeout"))slow_params=1;
 if(!strcmp(s,"stop_prepare"))stop_options=1;
 if(!strcmp(s,"options_communication"))bad_options=1;
 g_lift_home_request=1;step(1);
 if(!strcmp(s,"stop_prepare")){assert(!starts&&aborts==1&&stops==1&&g_lift_home_state==10);return 0;}
 if(!strcmp(s,"options_communication")){assert(!starts&&g_lift_home_state==9&&g_lift_home_result==4);return 0;}
 if(!strcmp(s,"fast_prepare")){assert(starts==1&&g_lift_home_state==6&&now==10);return 0;}''')
t=t.replace("cases = ['idle'", "cases = ['fast_prepare','stop_prepare','options_communication','idle'")
check=R/'scripts/check_arm_smooth_home_20261102.py';check.write_text(t,encoding='utf-8')
r=subprocess.run([sys.executable,str(check),'--project',str(S),'--host',str(R/'tmp/arm_parameters_20261102/home-host')],capture_output=True,text=True)
(E/'home-check.log').write_text(r.stdout+r.stderr,encoding='utf-8');print(r.stdout+r.stderr);assert r.returncode==0
subprocess.run([sys.executable,str(R/'scripts/check_arm_parameters_20261102.py')],check=True)
log=E/'stage-build.log'
r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(S/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],timeout=120)
t=log.read_text(encoding='utf-8',errors='replace');print(t[-700:]);assert r.returncode==0 and '0 Error(s), 0 Warning(s)' in t
m['stage_build_pass']=True;m['home_checks_pass']=True
(E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
