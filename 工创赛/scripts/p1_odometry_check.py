"""Run actual odometry/backend C in ARM emulator. No hardware access."""
import argparse, hashlib, json, shutil, struct, subprocess, sys
from pathlib import Path
from p1_four_wheel_sim import SOURCE_FILES
ROOT=Path(__file__).resolve().parents[2]
USER=ROOT/'3.stm32f407工程/USER'
C=r'''
#include "src/chassis_odometry.c"
static ChassisOdometry odom;
static uint32_t time_ms;
static void wheel(uint32_t i,float mm,uint32_t ms) {
 float driver=mm*(g_chassis_forward_direction[i] ? -1.0f : 1.0f);
 ChassisOdometry_Wheel(&odom,i,driver<0.0f,(uint32_t)(fabsf(driver)*65536.0f/(3.14159265358979323846f*77.0f)+0.5f),ms);
}
static void sample(float x,float y,float turn,float yaw,uint32_t ms) {
 ChassisOdometry_Yaw(&odom,yaw,ms);
 wheel(0,x+y-turn,ms);wheel(1,x-y+turn,ms);
 wheel(2,x-y-turn,ms);wheel(3,x+y+turn,ms);
 ChassisOdometry_Update(&odom,ms);
}
static void reset_odom(float yaw) {
 ChassisOdometry_Init(&odom);time_ms=1000;
 sample(0,0,0,yaw,time_ms);CHECK(odom.status.valid);
}
volatile float odom_failure[4];
static uint32_t near_index;
static void near(float got,float expected) {
 near_index++;
 if(fabsf(got-expected)>=0.06f && !first_failure) {
  odom_failure[0]=got;odom_failure[1]=expected;odom_failure[2]=(float)near_index;
 }
 CHECK(fabsf(got-expected)<0.06f);
}
/* MAIN_ADAPTER */
uint32_t test_entry(void) {
 uint32_t i,j,updates;
 ChassisOdometrySamples capture;
 /* Real backend capture survives per-action reset; no extra CAN calls. */
 ChassisAuto_Init();
 Auto_OdometrySample(0,1,0,123,300);ChassisAuto_ReadOdometrySamples(&capture);
 CHECK(capture.mask==1 && capture.raw[0]==123 && capture.ms[0]==300);
 Auto_PositionReset(1);ChassisAuto_ReadOdometrySamples(&capture);CHECK(capture.mask==1);
 Auto_OdometrySample(0,0,0,0,400);ChassisAuto_ReadOdometrySamples(&capture);CHECK(!capture.mask);
 /* Four-way translations including reverse and driver sign crossing. */
 for(j=0;j<4;j++) {
  reset_odom(177.0f);
  for(i=1;i<=100;i++) sample(j<2 ? (j==0?1.0f:-1.0f)*i*2.0f:0,
    j>=2 ? (j==2?1.0f:-1.0f)*i*2.0f:0,0,177,1000+i*20);
  CHECK(odom.status.valid);near(odom.status.x_mm,j<2?(j==0?1:-1)*200*CHASSIS_FORWARD_DISTANCE_SCALE:0);
  near(odom.status.y_mm,j>=2?(j==2?200:-200):0);
 }
 /* Pure rotation both ways crossing +/-180; centre must remain stationary. */
 for(j=0;j<2;j++) {
  reset_odom(170);
  for(i=1;i<=90;i++) {float deg=(j? -1.0f:1.0f)*i;
   sample(0,0,deg*3.14159265358979323846f/180.0f*209.0f,wrap(170+deg),1000+i*40);
  }
  CHECK(odom.status.valid);near(odom.status.x_mm,0);near(odom.status.y_mm,0);
  near(odom.status.heading_deg,j?-90:90);
 }
 /* Rotate then forward in fixed axes; no segment-reset API is involved. */
 reset_odom(0);
 for(i=1;i<=90;i++) sample(0,0,i*3.14159265358979323846f/180.0f*209.0f,(float)i,1000+i*40);
 for(i=1;i<=100;i++) sample(i*2.0f,0,3.14159265358979323846f*104.5f,90,4600+i*20);
 CHECK(odom.status.valid);near(odom.status.x_mm,0);near(odom.status.y_mm,200*CHASSIS_FORWARD_DISTANCE_SCALE);
 /* Constant simultaneous twist: exact SE2 endpoint for 90 degree arc. */
 reset_odom(0);
 for(i=1;i<=90;i++) sample((float)i,0,i*3.14159265358979323846f/180.0f*209,(float)i,1000+i*40);
 CHECK(odom.status.valid);near(odom.status.x_mm,90*CHASSIS_FORWARD_DISTANCE_SCALE*2/3.14159265358979323846f);
 near(odom.status.y_mm,90*CHASSIS_FORWARD_DISTANCE_SCALE*2/3.14159265358979323846f);
 /* Async 0/20/40/60ms wheels and 10ms yaw; never extrapolate. */
 ChassisOdometry_Init(&odom);
 for(i=0;i<=100;i++) {
  uint32_t now=1000+i*10;
  ChassisOdometry_Yaw(&odom,0,now);
  for(j=0;j<4;j++) if(i>=2*j && (i-2*j)%8==0) wheel(j,(now-1000)*0.2f,now);
  ChassisOdometry_Update(&odom,now);
 }
 CHECK(odom.status.valid);near(odom.status.x_mm,(odom.status.sample_ms-1060)*0.2f*CHASSIS_FORWARD_DISTANCE_SCALE);
 near(odom.status.y_mm,0);
 updates=odom.status.updates;ChassisOdometry_Update(&odom,2000);CHECK(updates==odom.status.updates);
 /* Staleness is sticky: recovery cannot silently repair missing displacement. */
 ChassisOdometry_Update(&odom,2500);CHECK(!odom.status.valid&&odom.status.continuity_lost);
 sample(200,0,0,0,2510);CHECK(!odom.status.valid);
 CHECK(ChassisOdometry_SetPose(&odom,100,200,90,0));
 sample(200,0,0,0,2600);CHECK(odom.status.valid&&!odom.status.local_origin);
 sample(202,0,0,0,2620);near(odom.status.x_mm,100);near(odom.status.y_mm,200+2*CHASSIS_FORWARD_DISTANCE_SCALE);
 /* Gap, encoder reset, repeated conflicting timestamp and bad sign. */
 reset_odom(0);sample(100,0,0,0,1020);CHECK(!odom.status.valid);
 reset_odom(0);wheel(0,1,1000);ChassisOdometry_Update(&odom,1000);CHECK(!odom.status.valid);
 reset_odom(0);ChassisOdometry_Wheel(&odom,0,2,0,1020);ChassisOdometry_Update(&odom,1020);CHECK(!odom.status.valid);
 reset_odom(0);ChassisOdometry_Yaw(&odom,100,1020);ChassisOdometry_Update(&odom,1020);CHECK(!odom.status.valid);
 reset_odom(0);ChassisOdometry_Yaw(&odom,0,999);ChassisOdometry_Update(&odom,1020);CHECK(!odom.status.valid);
 reset_odom(0);sample(2,0,0,0,1020);sample(0,0,0,0,1040);CHECK(odom.status.valid);near(odom.status.x_mm,0);
 /* True 32-bit clock wrap, large absolute counts, and no premature float cast. */
 ChassisOdometry_Init(&odom);sample(0,0,0,0,0xFFFFFFF0U);sample(2,0,0,0,4U);
 CHECK(odom.status.valid);near(odom.status.x_mm,2*CHASSIS_FORWARD_DISTANCE_SCALE);
 ChassisOdometry_Init(&odom);ChassisOdometry_Yaw(&odom,0,1000);
 for(i=0;i<4;i++)ChassisOdometry_Wheel(&odom,i,g_chassis_forward_direction[i],4000000000U,1000);
 ChassisOdometry_Update(&odom,1000);ChassisOdometry_Yaw(&odom,0,1020);
 for(i=0;i<4;i++)ChassisOdometry_Wheel(&odom,i,g_chassis_forward_direction[i],4000000100U,1020);
 ChassisOdometry_Update(&odom,1020);CHECK(odom.status.valid);
 near(odom.status.x_mm,100*3.14159265358979323846f*77/65536*CHASSIS_FORWARD_DISTANCE_SCALE);
 CHECK(!ChassisOdometry_SetPose(&odom,0,0,999,0));
 /* Invalid startup samples do not poison a newly established origin. */
 ChassisOdometry_Init(&odom);ChassisOdometry_Yaw(&odom,999,1000);
 sample(0,0,0,0,1020);CHECK(odom.status.valid);
 /* Actual CAN-task adapter: short snapshot locks, no CAN operations or reset on action. */
 ChassisAuto_Init();ChassisOdometry_Init(&s_odometry);test_tick=1000;
 s_heading_source.healthy=1;s_heading_source.angle_ms=1000;s_heading_source.valid_mask=1;
 s_heading_source.yaw_deg=20;
 for(i=0;i<4;i++)Auto_OdometrySample(i,1,0,0,1000);
 App_OdometryStep();CHECK(g_chassis_odometry.valid);CHECK(test_depth==0);
 test_tick=1020;s_heading_source.angle_ms=1020;
 for(i=0;i<4;i++)Auto_OdometrySample(i,1,g_chassis_forward_direction[i],500,1020);
 App_OdometryStep();CHECK(g_chassis_odometry.valid);CHECK(g_chassis_odometry.x_mm>1);
 Auto_PositionReset(2);App_OdometryStep();CHECK(g_chassis_odometry.valid);
 s_heading_source.healthy=0;App_OdometryStep();CHECK(!g_chassis_odometry.valid);
 s_heading_source.healthy=1;App_OdometryStep();CHECK(!g_chassis_odometry.valid);
 g_odometry_pose_request=1;test_power=1;App_OdometryStep();CHECK(g_odometry_pose_result==2);
 g_odometry_pose_request=1;test_power=0;g_vm_test_request=1;App_OdometryStep();CHECK(g_odometry_pose_result==2);
 g_odometry_pose_request=1;g_vm_test_request=0;g_odometry_pose_x_mm=100;App_OdometryStep();CHECK(g_odometry_pose_result==1);
 CHECK(!g_odometry_pose_request);CHECK(test_depth==0);
 return first_failure ? (0x80000000U|first_failure) : checks;
}
'''
def run(out):
    out.mkdir(parents=True,exist_ok=False);(out/'src').mkdir()
    for name in set(SOURCE_FILES+['motion_control.h','chassis_odometry.c','chassis_odometry.h']):shutil.copyfile(USER/name,out/'src'/name)
    shutil.copytree(Path(__file__).parent/'p1_offline_tests/stubs',out/'stubs')
    prefix=(Path(__file__).parent/'p1_offline_tests/route_speed.c').read_text(encoding='utf8').split('/* FRONT_TYPE */')[0]
    prefix=prefix.replace('void Power_ReadCommandStatus(PowerCommandStatus *p){p->active=1U;}',
                          'static uint32_t test_power; void Power_ReadCommandStatus(PowerCommandStatus *p){p->active=test_power;}')
    from p1_turn_fault_audit import function as extract
    main=(USER/'main.c').read_bytes().decode('gbk')
    adapter='''
typedef struct {uint32_t sequence,sample_ms,angle_ms,rate_ms,healthy,valid_mask;float yaw_deg,rate_dps;} AppHeadingSource;
static AppHeadingSource s_heading_source;
static ChassisOdometry s_odometry;
volatile ChassisOdometryStatus g_chassis_odometry;
volatile uint32_t g_odometry_pose_request,g_odometry_pose_result,g_vm_test_request;
volatile float g_odometry_pose_x_mm,g_odometry_pose_y_mm,g_odometry_pose_heading_deg;
static uint32_t s_can_jog_preparing;
static uint32_t App_HeadingFrameAge(const AppHeadingSource *s,uint32_t now,uint32_t bit){return s->valid_mask&bit ? now-s->angle_ms:0xFFFFFFFFU;}
'''+extract(main,'App_OdometryStep')
    (out/'unit.c').write_bytes((prefix+C.replace('/* MAIN_ADAPTER */',adapter)).encode('gbk'))
    (out/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }\n')
    arm=Path('D:/Keil_v5/ARM/arm5_compiler/bin');logs=[]
    for source,obj in [('unit.c','unit.o'),('src/chassis_kinematics.c','kinematics.o'),('src/motion_profile.c','profile.o')]:
        cmd=[str(arm/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-I','src','-I','stubs','-c',source,'-o',obj]
        p=subprocess.run(cmd,cwd=out,capture_output=True,text=True,errors='replace');logs.append(dict(command=cmd,returncode=p.returncode,output=p.stdout+p.stderr))
        (out/'build.json').write_text(json.dumps(logs,indent=2))
        if p.returncode:raise RuntimeError(p.stdout+p.stderr)
    cmd=[str(arm/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','kinematics.o','profile.o','--library_type=microlib','-o','unit.axf']
    p=subprocess.run(cmd,cwd=out,capture_output=True,text=True);logs.append(dict(command=cmd,returncode=p.returncode,output=p.stdout+p.stderr));(out/'build.json').write_text(json.dumps(logs,indent=2))
    if p.returncode:raise RuntimeError(p.stdout+p.stderr)
    sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
    import unicorn as uc
    from unicorn import arm_const as reg
    m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
    b=(out/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
    for i in range(h[10]):
        kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
        if kind==1 and size:m.mem_write(addr,b[off:off+size])
    stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(h[4]|1,stop,count=50000000)
    value=m.reg_read(reg.UC_ARM_REG_R0)
    import re
    symbols=subprocess.check_output([str(arm/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out,text=True)
    address=int(re.search(r'\bodom_failure\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
    report=dict(hardware_accessed=False,assertions=value,failure_values=list(struct.unpack('<4f',m.mem_read(address,16))),passed=not(value&0x80000000) and m.reg_read(reg.UC_ARM_REG_PC)==stop,source_sha256={n:hashlib.sha256((USER/n).read_bytes()).hexdigest() for n in ['chassis_odometry.c','chassis_odometry.h','chassis_auto_test.c','main.c']})
    (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    if not report['passed']:raise RuntimeError(f'actual C failed at unit.c:{value&0x7fffffff}')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);run(p.parse_args().out.resolve())
