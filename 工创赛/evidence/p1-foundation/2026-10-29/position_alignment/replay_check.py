from pathlib import Path
import json,subprocess,struct,sys,re
r=Path.cwd();o=r/'工创赛/evidence/p1-foundation/2026-10-29/position_alignment';d=o/'route_checks_extended/candidate'
s=(d/'unit.c').read_bytes().decode('gbk');a=s.index('uint32_t test_entry(void)');b=s.index('uint32_t MoveStart(',a)
s=s[:a]+'''uint32_t test_entry(void)
{
    uint32_t i; float mm[4],sum=0,lo=0,hi=0;
    const uint32_t t0[4]={53392,53425,53458,53359};
    const uint32_t t1[4]={53557,53623,53458,53491};
    const uint32_t r0[4]={265950,270531,267257,264536};
    const uint32_t r1[4]={274884,281153,267257,271728};
    backend_reset();Auto_PositionReset(1U);
    g_chassis_auto_state=CHASSIS_AUTO_RUN;s_distance_mode=1U;s_run_started=1U;
    s_run_tick=44000U;test_tick=53633U;s_profile_active=0U;
    s_distance_target_mm=2100.0f;s_distance_progress_mm=1000.0f;s_distance_max_rpm=15U;
    s_position_valid=15U;s_endpoints[s_endpoint_index].start_mask=15U;
    for(i=0;i<4;i++) {
        Auto_PositionRemember(i,t0[i],(float)r0[i]*3.14159265358979323846f*77.0f/65536.0f);
        Auto_PositionRemember(i,t1[i],(float)r1[i]*3.14159265358979323846f*77.0f/65536.0f);
        s_position_tick[i]=s_distance_seen_ms[i]=t1[i];
        s_position.raw[i]=s_distance_seen_raw[i]=r1[i];
        s_position.sign[i]=s_distance_seen_sign[i]=g_chassis_forward_direction[i];
        CHECK(Auto_PositionAt(i,53458U,&mm[i]));
        if(!i||mm[i]<lo)lo=mm[i];if(!i||mm[i]>hi)hi=mm[i];sum+=mm[i];
    }
    CHECK(hi-lo>18.60f&&hi-lo<18.64f);
    Auto_DistanceCheckLocked(test_tick);
    CHECK(!s_distance_fault_reason);CHECK(!s_distance_reached);
    CHECK(fabsf(s_distance_progress_mm-sum*0.25f*CHASSIS_FORWARD_DISTANCE_SCALE)<0.1f);
    CHECK(!Auto_PositionAt(0,54000U,&mm[0]));
    CHECK(!Auto_PositionAt(0,53000U,&mm[0]));
    /* Genuine synchronized wheel mismatch must still trip the original 50mm line. */
    for(i=0;i<4;i++) {
        s_position_history_count[i]=s_position_history_head[i]=0U;
        Auto_PositionRemember(i,53623U,i==2U?1060.0f:1000.0f);
        s_position_tick[i]=s_distance_seen_ms[i]=53623U;
    }
    Auto_DistanceCheckLocked(test_tick);CHECK(s_distance_fault_reason==5U);CHECK(g_chassis_auto_cut_vm);
    /* uint32 tick wrap: interpolation stays within a measured bracket. */
    s_position_history_count[0]=s_position_history_head[0]=0U;
    Auto_PositionRemember(0,0xfffffff0U,10.0f);Auto_PositionRemember(0,0x10U,42.0f);
    CHECK(Auto_PositionAt(0,0U,&mm[0]));CHECK(fabsf(mm[0]-26.0f)<0.001f);
    return first_failure ? 0x80000000U|first_failure : checks;
}
''' +s[b:]
(d/'replay.c').write_bytes(s.encode('gbk'))
commands=json.loads((d/'build.json').read_text())
for row in commands:
 cmd=row['command']
 if 'unit.c' in cmd:cmd=[v.replace('unit.c','replay.c') for v in cmd]
 z=subprocess.run(cmd,cwd=d,capture_output=True,text=True);assert z.returncode==0,(z.stdout,z.stderr)
sys.path.insert(0,str(r/'工创赛/tmp/position_alignment_deps'))
import unicorn as uc
from unicorn import arm_const as reg
blob=(d/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',blob)
m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
for i in range(h[10]):
 k,offset,address,_,size,_,_,_=struct.unpack_from('<IIIIIIII',blob,h[5]+i*h[9])
 if k==1 and size:m.mem_write(address,blob[offset:offset+size])
stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(h[4]|1,stop,count=10000000)
v=m.reg_read(reg.UC_ARM_REG_R0);result={'ok':v<0x80000000,'assertions':v if v<0x80000000 else None,'failure_line':v&0x7fffffff if v>=0x80000000 else None,'hardware_accessed':False}
(o/'real_log_replay.json').write_text(json.dumps(result,indent=2));print(result);assert result['ok']
