"""Actual MA1 stream/CRC/session path over the shared USART6 transport."""
from pathlib import Path
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/integration12_stage2_vision_20261107/project/CHASSIS/USER'
OUT = ROOT / 'evidence/integration/2026-11-07/stage2-mainline'
FIXTURE = r'''
#include "source_observer.h"
#include "vehicle_mission.h"
#include <stdio.h>
#include <string.h>
static uint32_t ready=1,busy=1,window=1;
uint32_t Stage2Body_ObservationWindow(void){return window;}
static char sent[256];static unsigned checks,failures;
static VehicleAlignmentRequest current={2,2,1,2,14,2060,1200,0};
uint32_t VehicleMission_ReadAlignmentRequest(VehicleAlignmentRequest*out){if(!ready)return 0;*out=current;return 1;}
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL %d reason%u\n",__LINE__,g_source_observer_reason);}}while(0)
static uint8_t send_frame(const uint8_t*b,uint16_t n){if(busy)return 0;memcpy(sent,b,n);sent[n]=0;return 1;}
static uint8_t publish_while_sending(const uint8_t*b,uint16_t n){
 memcpy(sent,b,n);sent[n]=0;SourceObserver_Invalidate();SourceObserver_Service(310);return 1;}
static void feed(const char*p,uint32_t now){while(*p){CK(SourceObserver_SharedByte((uint8_t)*p++,now,0)==1);}}
int main(void){
 MetricAlignmentProfile p={1,10,20,30,40,50,200,900,100000,10000};MetricAlignmentObservation o;char request[256];
 SourceObserver_Init(0);SourceObserver_Service(100);CK(g_source_observer_reason==MA_UNCALIBRATED);
 SourceObserver_Init(&p);SourceObserver_Service(100);
 CK(!SourceObserver_TrySend(send_frame));busy=0;CK(SourceObserver_TrySend(send_frame));
 CK(strncmp(sent,"MR1,1,2,2,1,2,14,",strlen("MR1,1,2,2,1,2,14,"))==0);CK(!SourceObserver_Request(request,sizeof(request)));
 CK(!SourceObserver_SharedByte('M',120,1));
 feed(WIRE1,125);SourceObserver_Service(140);
 CK(g_source_observer_accepted && SourceObserver_Read(140,&o)==MA_OK && o.dx_milli_mm==-12500 && o.dy_milli_mm==2500);
 SourceObserver_Invalidate();SourceObserver_Service(200);
 feed(WIRE1,210);SourceObserver_Service(215);CK(!g_source_observer_accepted && g_source_observer_reason==MA_SESSION);
 feed(WIRE2,230);SourceObserver_Service(240);CK(g_source_observer_accepted && SourceObserver_Read(240,&o)==MA_OK);
 SourceObserver_Invalidate();SourceObserver_Service(300);
 CK(SourceObserver_TrySend(publish_while_sending));
 CK(SourceObserver_TrySend(send_frame));CK(strncmp(sent,"MR1,4,2,2,1,2,14,",strlen("MR1,4,2,2,1,2,14,"))==0);
 feed(WIRE4,400);SourceObserver_Service(420);CK(g_source_observer_accepted && SourceObserver_Read(420,&o)==MA_OK);
 feed(BADCRC,430);SourceObserver_Service(440);CK(g_source_observer_reason==MA_BAD_ARGUMENT);
 SourceObserver_Service(560);CK(!g_source_observer_accepted);
 ready=0;CK(SourceObserver_Read(570,&o)!=MA_OK);
 ready=1;current.position_executor_enabled=1;window=0;SourceObserver_Invalidate();SourceObserver_Service(600);
 CK(g_source_observer_reason==MA_NOT_STOPPED && !g_source_observer_accepted);
 window=1;SourceObserver_Service(610);CK(g_source_observer_reason==MA_OK);
 printf("metric shared-UART checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''


def main():
    sys.path.insert(0, str(ROOT / 'vision_app/code'))
    from metric_alignment_protocol import MetricObservation, encode_observation

    def wire(request_id):
        return encode_observation(MetricObservation(
            request_id, 2, 2, 1, 2, 14, 10, 20, 30, 40, 50, 1, 25, 5, 950, 1, -12500, 2500, 0))

    def cstr(data):
        return '"' + ''.join('\\%03o' % b for b in data) + '"'

    definitions = ''.join('#define WIRE%d %s\n' % (i, cstr(wire(i))) for i in [1, 2, 4])
    bad = bytearray(wire(4))
    bad[-4] = ord('0') if bad[-4] != ord('0') else ord('1')
    definitions += '#define BADCRC ' + cstr(bad) + '\n'
    src, exe = OUT / 'metric-shared-host.c', OUT / 'metric-shared-host.exe'
    src.write_text(definitions + FIXTURE, encoding='ascii')
    command = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
               '-Wno-unused-function', '-Wno-misleading-indentation', '-DSOURCE_OBSERVER_HOST',
               '-I' + str(STAGE), str(src), str(STAGE / 'source_observer.c'), '-o', str(exe)]
    built = subprocess.run(command, capture_output=True, text=True)
    (OUT / 'metric-shared-build.log').write_text(built.stdout + built.stderr, encoding='utf-8')
    assert built.returncode == 0, built.stderr
    ran = subprocess.run([str(exe.resolve())], capture_output=True, text=True)
    (OUT / 'metric-shared-check.log').write_text(ran.stdout + ran.stderr, encoding='utf-8')
    print(ran.stdout)
    assert ran.returncode == 0, ran.stdout + ran.stderr
    (OUT / 'metric-shared-checks.json').write_text(json.dumps({
        'passed': True, 'result': ran.stdout.strip(), 'actual_C_MA1_stream': True,
        'frames_generated_by_actual_Python_encoder': True,
        'hardware_access': False, 'actual_camera_or_calibration': False,
    }, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
