"""Compile/test the actual Stage2 parser and transaction code, without hardware."""
from pathlib import Path
import importlib.util
import itertools
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/integration12_stage2_vision_20261107/project'
OUT = ROOT / 'evidence/integration/2026-11-07/stage2-mainline'

PREFIX = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "source_stage2_vision_bridge.h"
static uint32_t now_ms;
static uint8_t incoming[2048],sent[32];
static uint32_t rx_head,rx_tail,rx_enable_count,tx_count,tx_busy;
volatile uint32_t g_uart6_rx_errors;
static uint32_t xTaskGetTickCount(void){return now_ms;}
static void USART6_VisionEnableReceive(void){rx_enable_count++;}
static uint8_t USART6_VisionReadByte(uint8_t*p){if(rx_tail==rx_head)return 0;*p=incoming[rx_tail++];return 1;}
static uint8_t USART6_VisionSend(const uint8_t*p,uint16_t n){if(tx_busy)return 0;memcpy(sent,p,n);tx_count++;return 1;}
/* MA1 receive/CRC/request path is tested separately using its actual module. */
static uint32_t SourceObserver_SharedByte(uint8_t b,uint32_t now,uint32_t active){(void)b;(void)now;(void)active;return 0;}
static uint32_t SourceObserver_TrySend(uint8_t(*send)(const uint8_t*,uint16_t)){(void)send;return 0;}
'''

TESTS = r'''
static unsigned checks,failures;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL line %d\n",__LINE__);}}while(0)
static void feed(const uint8_t*p,unsigned n){memcpy(incoming+rx_head,p,n);rx_head+=n;Vision_Poll();if(rx_tail==rx_head)rx_head=rx_tail=0;}
static void material(unsigned a,unsigned b,unsigned c){uint8_t p[6]={0xcc,2,(uint8_t)a,(uint8_t)b,(uint8_t)c,0xdd};feed(p,6);}
int main(void){
 uint32_t old,bad,rev,loaded[3];S2_Event event;uint8_t center[7]={0xcc,5,0x40,1,0x58,2,0xdd};
 uint8_t embedded[7]={0xcc,5,0xcc,0xdd,0xcc,0xdd,0xdd};
 uint8_t broken[6]={0xcc,2,1,2,3,0};uint8_t unknown[2]={0xcc,0x55};
 uint8_t partial[3]={0xcc,2,1};uint8_t combined[13]={0xcc,2,1,2,3,0xdd,0xcc,5,10,0,20,0,0xdd};
 uint8_t burst[132];unsigned i;
 Vision_Poll();CK(rx_enable_count==1 && tx_count==0);
 now_ms=100;material(1,2,3);feed(center,7);
 CK(g_vis_mat_cycle==1 && g_vis_material_valid && g_vis_mat_left==1 && g_vis_mat_right==2 && g_vis_mat_down==3);
 CK(g_vis_center_cycle==1 && g_vis_center_x==320 && g_vis_center_y==600);
 g_vis_pick_region=0;g_vis_taught_mask=0;CK(!Vision_TakeTarget(now_ms) && g_vis_txn_result==6);
 g_vis_taught_mask=1;g_vis_region_deg[0]=193;g_vis_region_ext_mm[0]=48;
 g_vis_mm_per_px=0.4f;g_vis_ref_x=310;g_vis_ref_y=590;
 CK(Vision_TakeTarget(now_ms) && g_src_txn_result==1);
 CK(g_src_pick_deg==193 && g_src_extension_mm==48 && fabsf(g_src_chassis_dx_mm-4)<.001f && fabsf(g_src_chassis_dy_mm-4)<.001f && g_vis_metric_valid);
 rev=g_src_txn_revision;CK(!Vision_TakeTarget(now_ms) && g_vis_txn_result==8 && g_src_txn_revision==rev);
 material(0,2,3);CK(!Vision_TakeTarget(now_ms) && g_vis_txn_result==9);
 material(1,2,3);now_ms=3101;CK(!Vision_TakeTarget(now_ms) && g_vis_txn_result==7);
 now_ms=3100;CK(Vision_TakeTarget(now_ms));
 old=g_vis_mat_cycle;bad=g_vis_rx_bad;feed(broken,6);CK(g_vis_mat_cycle==old && g_vis_rx_bad==bad+1);
 bad=g_vis_rx_bad;material(255,2,3);CK(g_vis_mat_cycle==old && g_vis_rx_bad==bad+1);
 bad=g_vis_rx_bad;feed(unknown,2);CK(g_vis_rx_bad==bad+1);
 feed(embedded,7);CK(g_vis_center_x==0xddcc && g_vis_center_y==0xddcc);
 now_ms=4000;feed(partial,3);bad=g_vis_rx_bad;now_ms=4200;material(1,2,3);
 CK(g_vis_rx_bad==bad+1 && g_vis_mat_cycle==old+1 && vis_len==0);
 feed(combined,13);CK(g_vis_center_x==10 && g_vis_center_y==20 && g_vis_mat_right==2);
 g_uart6_rx_errors++;Vision_Poll();CK(!g_vis_material_valid && !g_vis_center_valid);
 CK(!Vision_TakeTarget(now_ms) && g_vis_txn_result==7);
 material(1,2,3);g_vis_mm_per_px=0;CK(Vision_TakeTarget(now_ms) && !g_vis_metric_valid && g_src_chassis_dx_mm==0 && g_src_chassis_dy_mm==0);
 material(1,2,3);g_vis_region_ext_mm[0]=171;CK(!Vision_TakeTarget(now_ms) && g_src_txn_result==4);
 g_vis_region_ext_mm[0]=48;g_vis_region_deg[0]=NAN;CK(!Vision_TakeTarget(now_ms) && g_src_txn_result==4);
 g_vis_region_deg[0]=193;CK(Vision_TakeTarget(now_ms));
 material(1,2,3);tx_busy=1;g_vis_command_request=2;old=tx_count;Vision_Poll();
 CK(g_vis_command_request==2 && tx_count==old);
 tx_busy=0;Vision_Poll();CK(g_vis_command_request==0 && tx_count==old+1 && sent[0]==0xaa && sent[1]==2 && sent[2]==0xbb && !g_vis_material_valid);
 g_vis_command_request=99;old=tx_count;Vision_Poll();CK(g_vis_command_request==0 && g_vis_command_rejected==1 && tx_count==old);
 now_ms=0xfffffff0U;material(1,2,3);now_ms=0x20U;CK(Vision_TakeTarget(now_ms));
 for(i=0;i<sizeof(burst);i++)burst[i]=0x77;
 old=g_vis_rx_bytes;feed(burst,sizeof(burst));CK(g_vis_rx_bytes-old==128);Vision_Poll();CK(g_vis_rx_bytes-old==132);
 CK(rx_enable_count==1);
 printf("stage2 parser/txn checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    source = (STAGE / 'CHASSIS/USER/integration10_complete.inc').read_bytes()
    begin = source.index(b'volatile uint32_t g_src_txn_revision')
    end = source.index(b'static uint32_t it_platform', begin)
    actual = source[begin:end]
    fixture, exe = OUT / 'vision-parser-host.c', OUT / 'vision-parser-host.exe'
    # Generate test bytes with the actual Orange Pi encoder, not a second encoder.
    spec = importlib.util.spec_from_file_location('stage2_wire_codec', ROOT / 'vision_app/code/turntable_serial.py')
    codec = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = codec
    spec.loader.exec_module(codec)
    vectors = []
    for colors in itertools.product(range(7), repeat=3):
        frame = codec.TurntableSerialLink.encode_material_slots(*colors)
        vectors.append('{' + ','.join(map(str, frame)) + '}')
    centers = []
    values = [0, 1, 0xcc, 0xdd, 255, 256, 0xccdd, 0xddcc, 65535]
    for x, y in itertools.product(values, repeat=2):
        frame = codec.TurntableSerialLink.encode_center_frame(x, y)
        centers.append('{' + ','.join(map(str, frame)) + '}')
    vector_c = ('static const uint8_t material_wire[][6]={' + ','.join(vectors) + '};\n'
                'static const uint8_t center_wire[][7]={' + ','.join(centers) + '};\n')
    tests = TESTS.replace(' CK(rx_enable_count==1);', '''
 for(i=0;i<sizeof(material_wire)/sizeof(material_wire[0]);i++){
  feed(material_wire[i],6);
  CK(g_vis_mat_left==material_wire[i][2] && g_vis_mat_right==material_wire[i][3] && g_vis_mat_down==material_wire[i][4]);
 }
 for(i=0;i<sizeof(center_wire)/sizeof(center_wire[0]);i++){
  feed(center_wire[i],7);
  CK(g_vis_center_x==((uint32_t)center_wire[i][2]|((uint32_t)center_wire[i][3]<<8)) &&
     g_vis_center_y==((uint32_t)center_wire[i][4]|((uint32_t)center_wire[i][5]<<8)));
 }
 CK(rx_enable_count==1);''')
    tests = tests.replace(' printf("stage2 parser/txn', r'''
 /* Same actual C path as request13: QR order -> region -> target -> evidence. */
 CK(!Vision_SetTask("111+123+111+123",1,10));
 CK(!Vision_SetTask("146+124+614+231",1,10));
 CK(!Vision_SetTask("146+123+123+231",1,10));
 CK(!Vision_SetTask("146+123+614+231",1,0));
 CK(Vision_SetTask("146+123+614+231",1,10));
 CK(g_vis_task_colors[0]==1 && g_vis_task_colors[1]==4 && g_vis_task_colors[2]==6);
 CK(!Vision_TakeTaskTarget(now_ms) && g_vis_txn_result==7);
 g_vis_taught_mask=7;g_vis_region_deg[0]=190;g_vis_region_deg[1]=160;g_vis_region_deg[2]=200;
 g_vis_region_ext_mm[0]=48;g_vis_region_ext_mm[1]=60;g_vis_region_ext_mm[2]=80;
 material(1,1,6);CK(!Vision_TakeTaskTarget(now_ms) && g_vis_txn_result==12);
 material(4,6,1);CK(Vision_TakeTaskTarget(now_ms));
 CK(g_vis_pick_region==2 && g_src_slot==1 && g_src_pick_deg==200 && g_src_extension_mm==80);
 CK(g_vis_task_item==0 && g_vis_loaded_mask==0 && !g_vis_batch_complete);
 CK(!Vision_ReadLoadedBatch(10,1,loaded));
 rev=g_vis_pending_seq;
 CK(!Vision_TakeTaskTarget(now_ms) && g_vis_txn_result==11);
 CK(!Vision_SetTask("146+123+614+231",2,10));
 CK(!Vision_ReportGrasp(11,rev,1,1,now_ms,now_ms));
 CK(!Vision_ReportGrasp(10,rev,0,1,now_ms,now_ms));
 CK(!Vision_ReportGrasp(10,rev,1,1,now_ms-1001U,now_ms));
 CK(!Vision_ReportPlaced(10,rev,1,1,1,1,now_ms,now_ms));
 CK(Vision_ReportGrasp(10,rev,1,1,now_ms,now_ms));
 CK(!Vision_ReportPlaced(10,rev,2,1,1,1,now_ms,now_ms));
 CK(!Vision_ReportPlaced(10,rev,1,1,1,0,now_ms,now_ms));
 CK(Vision_ReportPlaced(10,rev,1,1,1,1,now_ms,now_ms));
 CK(g_vis_task_item==1 && g_vis_loaded_mask==1 && !g_vis_pending_seq);
 CK(!Vision_ReportPlaced(10,rev,1,1,1,1,now_ms,now_ms));
 CK(Vision_SetTask("146+123+614+231",1,10) && g_vis_task_item==1);
 material(4,6,0);CK(Vision_TakeTaskTarget(now_ms) && g_vis_pick_region==0 && g_src_slot==2);
 rev=g_vis_pending_seq;CK(Vision_ReportGrasp(10,rev,1,1,now_ms,now_ms));
 CK(Vision_ReportPlaced(10,rev,2,1,1,1,now_ms,now_ms));
 material(0,6,0);CK(Vision_TakeTaskTarget(now_ms) && g_vis_pick_region==1 && g_src_slot==3);
 rev=g_vis_pending_seq;CK(Vision_ReportGrasp(10,rev,1,1,now_ms,now_ms));
 CK(Vision_ReportPlaced(10,rev,3,1,1,1,now_ms,now_ms));
 CK(g_vis_task_item==3 && g_vis_loaded_mask==7 && g_vis_batch_complete);
 CK(Vision_ReadLoadedBatch(10,1,loaded) && loaded[0]==1 && loaded[1]==4 && loaded[2]==6);
 CK(!Vision_ReadLoadedBatch(11,1,loaded) && !Vision_ReadLoadedBatch(10,2,loaded));
 CK(!Vision_TakeTaskTarget(now_ms) && g_vis_txn_result==10);
 CK(Vision_SetTask("146+123+614+231",2,10));
 CK(g_vis_task_colors[0]==6 && g_vis_task_item==0 && !g_vis_batch_complete);
 CK(Vision_SetTask("146+123+614+231",1,11) && g_vis_task_run_id==11 && !g_vis_material_valid);
 material(4,6,1);CK(Vision_TakeTaskTarget(now_ms));rev=g_vis_pending_seq;
 memset(&event,0,sizeof(event));event.run_id=11;event.slot=1;event.observed_ms=now_ms;
 event.evidence_flags=S2_HELD;CK(!S2_VisionGrasp(0,11,rev,&event,now_ms));
 event.evidence_flags=S2_SOURCE_CLEAR|S2_HELD;CK(S2_VisionGrasp(0,11,rev,&event,now_ms));
 event.evidence_flags=S2_RELEASED|S2_PRESENT;CK(!S2_VisionPlaced(0,11,rev,&event,now_ms));
 event.evidence_flags|=S2_STABLE;CK(S2_VisionPlaced(0,11,rev,&event,now_ms));
 CK(g_vis_task_item==1 && g_vis_loaded_mask==1 && !g_vis_pending_seq);
 material(4,6,0);CK(Vision_TakeTaskTarget(now_ms));rev=g_vis_pending_seq;
 CK(!Vision_RejectUnstarted(12,rev) && g_vis_pending_seq==rev);
 CK(Vision_RejectUnstarted(11,rev) && !g_vis_pending_seq && g_vis_mat_consumed==g_vis_mat_cycle);
 CK(!Vision_TakeTaskTarget(now_ms) && g_vis_txn_result==8);
 printf("stage2 parser/txn''')
    fixture.write_bytes(PREFIX.encode('ascii') + actual + vector_c.encode('ascii') + tests.encode('ascii'))
    built = subprocess.run(['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
                            '-Wno-misleading-indentation', '-I' + str(STAGE / 'CHASSIS/USER'),
                            str(fixture), str(STAGE / 'CHASSIS/USER/source_stage2_vision_bridge.c'),
                            '-lm', '-o', str(exe)],
                           capture_output=True, text=True)
    (OUT / 'vision-parser-build.log').write_text(built.stdout + built.stderr, encoding='utf-8')
    assert built.returncode == 0, built.stderr
    ran = subprocess.run([str(exe.resolve())], capture_output=True, text=True)
    (OUT / 'vision-parser-check.log').write_text(ran.stdout + ran.stderr, encoding='utf-8')
    print(ran.stdout)
    assert ran.returncode == 0, ran.stdout + ran.stderr
    main_c = (STAGE / 'CHASSIS/USER/main.c').read_bytes()
    driver = (STAGE / 'CHASSIS/SYSTEM/USART/usart.c').read_bytes()
    assert b'USART1_ReadByte' not in actual and b'USART6_VisionReadByte' in actual
    assert b'GPIO_Pin_6 | GPIO_Pin_7' in driver and b'GPIO_AF_USART6' in driver
    assert b'void USART6_IRQHandler' in driver and b'void USART1_IRQHandler' in driver
    assert b'USART1_WaitByte(&data, 100U)' in main_c
    assert b'if(VEHICLE_INTEGRATED_ENABLE || VEHICLE_LIFT_RULER_ONLY)USART6_VisionConfig();' in main_c
    assert b'if(VEHICLE_INTEGRATED_ENABLE && !VEHICLE_LIFT_RULER_ONLY)Vision_Poll();' in main_c
    # Test the exact read-only bridge to the existing route/QR owner as well.
    mission = (STAGE / 'CHASSIS/USER/vehicle_mission.inc').read_bytes()
    a = mission.index(b'uint32_t VehicleMission_ReadSourceTask(')
    b = mission.index(b'uint32_t VehicleMission_ReadAlignmentRequest(', a)
    provider = mission[a:b]
    provider_prefix = b'''#include <stdint.h>
#include <string.h>
#include <stdio.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static uint32_t s_vehicle_code_valid,g_vehicle_scan_bypassed,g_vehicle_run_id,g_vehicle_stage,g_vehicle_cause;
static char s_vehicle_task_code[16];
'''
    provider_tests = b'''
int main(void){char code[16];uint32_t run=0,batch=0;unsigned checks=0;
#define CK(v) do{checks++;if(!(v))return 1;}while(0)
 CK(!VehicleMission_ReadSourceTask(code,&run,&batch));
 memcpy(s_vehicle_task_code,"146+123+614+231",16);s_vehicle_code_valid=1;g_vehicle_run_id=10;g_vehicle_stage=2;
 CK(VehicleMission_ReadSourceTask(code,&run,&batch) && run==10 && batch==1 && !memcmp(code,s_vehicle_task_code,16));
 g_vehicle_stage=5;CK(VehicleMission_ReadSourceTask(code,&run,&batch) && batch==2);
 g_vehicle_stage=3;CK(!VehicleMission_ReadSourceTask(code,&run,&batch));g_vehicle_stage=2;
 g_vehicle_scan_bypassed=1;CK(!VehicleMission_ReadSourceTask(code,&run,&batch));g_vehicle_scan_bypassed=0;
 g_vehicle_cause=1;CK(!VehicleMission_ReadSourceTask(code,&run,&batch));g_vehicle_cause=0;
 g_vehicle_run_id=0;CK(!VehicleMission_ReadSourceTask(code,&run,&batch));
 CK(!VehicleMission_ReadSourceTask(0,&run,&batch));
 printf("source QR provider checks=%u failures=0\\n",checks);return 0;}
'''
    psrc, pexe = OUT / 'source-task-provider.c', OUT / 'source-task-provider.exe'
    psrc.write_bytes(provider_prefix + provider + provider_tests)
    built_provider = subprocess.run(['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
                                     str(psrc), '-o', str(pexe)], capture_output=True, text=True)
    assert built_provider.returncode == 0, built_provider.stderr
    tested_provider = subprocess.run([str(pexe.resolve())], capture_output=True, text=True)
    (OUT / 'source-task-provider-check.log').write_text(tested_provider.stdout + tested_provider.stderr, encoding='utf-8')
    assert tested_provider.returncode == 0, tested_provider.stdout + tested_provider.stderr
    print(tested_provider.stdout)
    a = mission.index(b'static uint32_t VehicleMission_SourceBatchComplete(')
    b = mission.index(b'static void VehicleMission_Advance(', a)
    completion_gate = mission[a:b]
    a = mission.index(b'    if(g_vehicle_state==8U){')
    b = mission.index(b'    if(g_vehicle_state==3U && g_vehicle_stage==2U', a)
    actual_wait_branch = mission[a:b]
    handoff_prefix = b'''#include <stdint.h>
#include <stdio.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static uint32_t g_vehicle_stage=2,g_vehicle_run_id=42,g_vehicle_state=8,g_stage2_ready,loaded,advances;
static uint32_t Vision_ReadLoadedBatch(uint32_t run,uint32_t batch,uint32_t colors[3]){
 (void)colors;return loaded && run==42 && batch==1;}
static uint32_t SourceStage2_IsActive(void){return 0;}
static void VehicleMission_Advance(void){advances++;g_vehicle_state=4;}
'''
    handoff_tests = b'''
int main(void){unsigned n=0;
#define CK(v) do{n++;if(!(v))return 1;}while(0)
 service();CK(g_vehicle_state==8 && !g_stage2_ready && !advances);
 loaded=1;g_vehicle_run_id=43;service();CK(g_vehicle_state==8 && !advances);
 g_vehicle_run_id=42;g_vehicle_stage=3;service();CK(g_vehicle_state==8 && !advances);
 g_vehicle_stage=2;service();CK(g_vehicle_state==4 && g_stage2_ready && advances==1);
 service();CK(advances==1);
 printf("source loaded handoff checks=%u failures=0\\n",n);return 0;}
'''
    hsrc, hexe = OUT / 'source-loaded-handoff.c', OUT / 'source-loaded-handoff.exe'
    hsrc.write_bytes(handoff_prefix + completion_gate + b'\nstatic void service(void){\n'
                     + actual_wait_branch + b'\n}\n' + handoff_tests)
    compiled = subprocess.run(['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
                               str(hsrc), '-o', str(hexe)], capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stderr
    handoff = subprocess.run([str(hexe.resolve())], capture_output=True, text=True)
    (OUT / 'source-loaded-handoff-check.log').write_text(handoff.stdout + handoff.stderr, encoding='utf-8')
    assert handoff.returncode == 0, handoff.stdout + handoff.stderr
    print(handoff.stdout)
    assert b'g_vehicle_state=8U;g_stage2_ready=0U;' in mission
    assert b'|| g_vehicle_state==8U)' in mission
    report = {'passed': True, 'actual_C_result': ran.stdout.strip(),
              'physical_port': 'USART6 PC6/PC7', 'baud': 115200,
              'USART1_debug_preserved': True, 'one_vision_queue_consumer': True,
              'source_is_actual_candidate_C': True, 'UART_clock_queue_hardware': 'mocked',
              'actual_OrangePi_encoder_vectors': len(vectors) + len(centers),
              'source_task_provider_result': tested_provider.stdout.strip(),
              'source_loaded_handoff_result': handoff.stdout.strip(),
              'real_wire_or_OrangePi_verified': False, 'motor_commands': 0}
    (OUT / 'vision-parser-checks.json').write_text(json.dumps(report, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
