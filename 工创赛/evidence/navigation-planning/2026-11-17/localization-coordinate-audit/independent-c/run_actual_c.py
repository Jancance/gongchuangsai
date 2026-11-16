"""Compile current pure odometry with ARMCC and run five cases in Unicorn; no I/O."""
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import sys

OUT = Path(__file__).resolve().parent
ROOT = next(p for p in OUT.parents if (p / '3.stm32f407工程/gongxun-f407.uvprojx').is_file())
SOURCE = ROOT / '3.stm32f407工程/CHASSIS/USER'
ARM = Path('D:/Keil_v5/ARM/arm5_compiler/bin')
PARAMS = ROOT / '3.stm32f407工程/USER/vehicle_parameters.h'
inputs = [SOURCE / n for n in ('chassis_odometry.c', 'chassis_odometry.h',
          'chassis_kinematics.c', 'chassis_kinematics.h')] + [PARAMS]
hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
# Only ASCII macro declarations are parsed; preserve all source bytes unchanged.
params = PARAMS.read_bytes().decode('latin1')
kin_h = (SOURCE / 'chassis_kinematics.h').read_bytes().decode('latin1')
kin_c = (SOURCE / 'chassis_kinematics.c').read_bytes().decode('latin1')
def number(name, text):
    return re.search(r'^#define\s+' + name + r'\s+([0-9.]+f)\s*$', text, re.M).group(1)
diameter = number('VEHICLE_WHEEL_DIAMETER_MM', params)
track = number('VEHICLE_TRACK_WIDTH_MM', params)
base = number('VEHICLE_WHEELBASE_MM', params)
scale = number('CHASSIS_FORWARD_DISTANCE_SCALE', kin_h)
directions = re.search(r'g_chassis_forward_direction\[.*?\]\s*=\s*\{([^}]+)\}', kin_c).group(1)
assert [int(n) for n in re.findall(r'(\d+)U', directions)] == [0, 1, 0, 1]
for name in ('chassis_odometry.c', 'chassis_odometry.h'):
    (OUT / name).write_bytes((SOURCE / name).read_bytes())
(OUT / 'chassis_kinematics.h').write_text('''#include <stdint.h>
typedef struct {float wheel_diameter_mm,track_width_mm,wheelbase_mm;} ChassisGeometry;
extern const ChassisGeometry g_chassis_geometry;
extern const uint8_t g_chassis_forward_direction[4];
#define CHASSIS_FORWARD_DISTANCE_SCALE ''' + scale + '\n', encoding='ascii')
prefix = f'''#include <stdint.h>
#include <math.h>
#include "chassis_odometry.h"
#include "chassis_kinematics.h"
const ChassisGeometry g_chassis_geometry={{{diameter},{track},{base}}};
const uint8_t g_chassis_forward_direction[4]={{{directions}}};
'''
c = r'''
static ChassisOdometry odom;
static uint32_t checks;
volatile float case_values[15];
#define CHECK(x) do {checks++;if(!(x))return 0x80000000U|__LINE__;}while(0)
#define NEAR(a,b,t) CHECK(fabsf((a)-(b))<(t))
#define PI 3.14159265358979323846f
static float wrap_test(float a){while(a>180)a-=360;while(a<=-180)a+=360;return a;}
static void wheel(uint32_t i,float mm,uint32_t ms){
 float driver=mm*(g_chassis_forward_direction[i]?-1.0f:1.0f);
 uint32_t raw=(uint32_t)(fabsf(driver)*65536.0f/(PI*g_chassis_geometry.wheel_diameter_mm)+0.5f);
 ChassisOdometry_Wheel(&odom,i,driver<0.0f,raw,ms);
}
static void sample(float x,float y,float turn,float yaw,uint32_t ms){
 ChassisOdometry_Yaw(&odom,wrap_test(yaw),ms);
 wheel(0,x+y-turn,ms);wheel(1,x-y+turn,ms);
 wheel(2,x-y-turn,ms);wheel(3,x+y+turn,ms);
 ChassisOdometry_Update(&odom,ms);
}
static void save(uint32_t c){case_values[c*3]=odom.status.x_mm;
 case_values[c*3+1]=odom.status.y_mm;case_values[c*3+2]=odom.status.heading_deg;}
uint32_t test_entry(void){
 uint32_t i,j;float turn,angle,x,y,k=(g_chassis_geometry.track_width_mm+g_chassis_geometry.wheelbase_mm)*0.5f;
 /* Case 1: HWT 89.8 degrees, then corrected 1000mm body-forward. */
 ChassisOdometry_Init(&odom);sample(0,0,0,0,1000);CHECK(odom.status.valid);
 for(i=1;i<=90;i++){angle=89.8f*(float)i/90.0f;turn=angle*PI/180.0f*k;
  sample(0,0,turn,angle,1000+i*40);CHECK(odom.status.valid);}
 turn=89.8f*PI/180.0f*k;
 for(i=1;i<=100;i++){sample((float)i*10.0f/CHASSIS_FORWARD_DISTANCE_SCALE,0,turn,89.8f,4600+i*20);CHECK(odom.status.valid);}
 NEAR(odom.status.x_mm,1000.0f*cosf(89.8f*PI/180.0f),0.08f);
 NEAR(odom.status.y_mm,1000.0f*sinf(89.8f*PI/180.0f),0.08f);
 NEAR(odom.status.heading_deg,89.8f,0.001f);save(0);
 /* Case 2: explicit field heading crossing +180, then forward. */
 CHECK(ChassisOdometry_SetPose(&odom,0,0,170,170));sample(0,0,0,170,10000);
 for(i=1;i<=20;i++){sample(0,0,(float)i*PI/180.0f*k,170+(float)i,10000+i*40);CHECK(odom.status.valid);}
 turn=20.0f*PI/180.0f*k;
 for(i=1;i<=10;i++)sample((float)i*10/CHASSIS_FORWARD_DISTANCE_SCALE,0,turn,-170,10800+i*20);
 CHECK(odom.status.valid);NEAR(odom.status.heading_deg,-170,0.001f);
 NEAR(odom.status.x_mm,100*cosf(-170*PI/180),0.08f);
 NEAR(odom.status.y_mm,100*sinf(-170*PI/180),0.08f);save(1);
 /* Case 3: both directions of pure turn leave centre at origin. */
 for(j=0;j<2;j++){
  ChassisOdometry_Init(&odom);sample(0,0,0,0,20000);
  for(i=1;i<=90;i++){angle=(j?-1.0f:1.0f)*(float)i;
   sample(0,0,angle*PI/180*k,angle,20000+i*40);CHECK(odom.status.valid);}
  NEAR(odom.status.x_mm,0,0.01f);NEAR(odom.status.y_mm,0,0.01f);
  NEAR(odom.status.heading_deg,j?-90.0f:90.0f,0.001f);
 }save(2);
 /* Case 4: gap freezes pose; later fresh samples cannot silently recover. */
 ChassisOdometry_Init(&odom);sample(0,0,0,0,30000);sample(2,0,0,0,30020);
 CHECK(odom.status.valid);x=odom.status.x_mm;y=odom.status.y_mm;
 ChassisOdometry_Update(&odom,30500);CHECK(!odom.status.valid&&odom.status.continuity_lost);
 sample(12,0,0,0,30520);sample(14,0,0,0,30540);
 CHECK(!odom.status.valid&&odom.status.continuity_lost);NEAR(odom.status.x_mm,x,0.001f);NEAR(odom.status.y_mm,y,0.001f);save(3);
 /* Case 5: explicit SetPose discards old history and restores stated frame. */
 CHECK(ChassisOdometry_SetPose(&odom,100,200,90,17));sample(14,0,0,17,30600);
 CHECK(odom.status.valid&&!odom.status.continuity_lost&&!odom.status.local_origin);
 sample(24,0,0,17,30620);CHECK(odom.status.valid);
 NEAR(odom.status.x_mm,100,0.02f);NEAR(odom.status.y_mm,200+10*CHASSIS_FORWARD_DISTANCE_SCALE,0.02f);
 NEAR(odom.status.heading_deg,90,0.001f);save(4);
 return checks;
}
'''
(OUT / 'unit.c').write_text(prefix + c, encoding='ascii')
(OUT / 'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }\n', encoding='ascii')
logs = []
commands = [[str(ARM / 'armcc.exe'), '--cpu', 'Cortex-M4', '--c99', '--split_sections', '-O1', '-I', '.', '-c', n, '-o', o]
            for n, o in [('unit.c', 'unit.o'), ('chassis_odometry.c', 'odometry.o')]]
commands += [[str(ARM / 'armlink.exe'), '--entry', 'test_entry', '--scatter', 'unit.sct',
              'unit.o', 'odometry.o', '--library_type=microlib', '-o', 'unit.axf']]
for command in commands:
    p = subprocess.run(command, cwd=OUT, capture_output=True, text=True, errors='replace')
    logs.append(dict(command=command, returncode=p.returncode, output=p.stdout + p.stderr))
    (OUT / 'build.json').write_text(json.dumps(logs, indent=2), encoding='utf-8')
    if p.returncode: raise RuntimeError(p.stdout + p.stderr)
sys.path.insert(0, str(ROOT / '工创赛/tmp/position_alignment_deps'))
import unicorn as uc
from unicorn import arm_const as reg
m = uc.Uc(uc.UC_ARCH_ARM, uc.UC_MODE_THUMB | uc.UC_MODE_MCLASS)
m.mem_map(0x08000000, 0x100000)
m.mem_map(0x20000000, 0x20000)
b = (OUT / 'unit.axf').read_bytes()
h = struct.unpack_from('<16sHHIIIIIHHHHHH', b)
for i in range(h[10]):
    kind, off, addr, _, size, _, _, _ = struct.unpack_from('<IIIIIIII', b, h[5] + i * h[9])
    if kind == 1 and size: m.mem_write(addr, b[off:off + size])
stop = 0x080ff000
m.reg_write(reg.UC_ARM_REG_SP, 0x2001fff0)
m.reg_write(reg.UC_ARM_REG_LR, stop | 1)
m.emu_start(h[4] | 1, stop, count=20000000)
value = m.reg_read(reg.UC_ARM_REG_R0)
symbols = subprocess.check_output([str(ARM / 'fromelf.exe'), '--text', '-s', 'unit.axf'], cwd=OUT, text=True)
address = int(re.search(r'\bcase_values\s+(0x[0-9a-fA-F]+)', symbols)[1], 16)
values = struct.unpack('<15f', m.mem_read(address, 60))
report = dict(passed=not (value & 0x80000000) and m.reg_read(reg.UC_ARM_REG_PC) == stop,
              assertions=value if not value & 0x80000000 else None,
              failure_line=value & 0x7fffffff if value & 0x80000000 else None,
              cases=[dict(name=name, x_mm=values[i*3], y_mm=values[i*3+1], heading_deg=values[i*3+2])
                     for i, name in enumerate(['actual_89_8_then_forward', 'cross_180_then_forward',
                                               'pure_rotation_both_directions', 'gap_freezes_pose', 'explicit_setpose_restores'])],
              source_sha256=hashes, constants=dict(diameter=diameter, track=track, base=base, scale=scale,
                                                 forward_direction=directions),
              source_copy_byte_identical=all((OUT / n).read_bytes() == (SOURCE / n).read_bytes()
                                            for n in ('chassis_odometry.c', 'chassis_odometry.h')),
              hardware_accessed=False, product_modified=False,
              limits='Actual current pure odometry C on ARM emulator; geometry/sign externs stubbed from current source. Not app lifecycle, scheduling, sensor delay, wheel-ground truth or vehicle acceptance.')
assert all(hashlib.sha256(p.read_bytes()).hexdigest() == hashes[str(p.relative_to(ROOT))] for p in inputs)
(OUT / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps(report, indent=2))
if not report['passed']: raise RuntimeError('Actual C assertion failed')
