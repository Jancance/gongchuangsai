"""Prepare full source candidate with observation-only UART inbox. No flash."""
from pathlib import Path
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT / 'tmp/source_observer_20261106/project'
OUT = ROOT / 'evidence/integration/2026-11-06/source-observer'


def main():
    assert json.loads((OUT / 'matlab-result.json').read_text(encoding='utf-8'))['passed']
    assert not STAGE.exists(), 'Preserve existing full candidate'
    shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns('Objects', 'ObjectsOffline', 'Listings', '*.uvguix.*'))
    before, after = {}, {}

    def edit(name, transform):
        p = STAGE / name
        raw = p.read_bytes()
        try:
            text, encoding = raw.decode('utf-8'), 'utf-8'
        except UnicodeDecodeError:
            text, encoding = raw.decode('gbk'), 'gbk'
        p.write_bytes(transform(text.replace('\r\n', '\n')).replace('\n', '\r\n').encode(encoding))
        before[name] = hashlib.sha256(raw).hexdigest()
        after[name] = hashlib.sha256(p.read_bytes()).hexdigest()

    def arm(t):
        old = '((s_address==6U && (s_options&0x80U))?100.0f:10.0f)'
        assert t.count(old) == 1
        return t.replace(old, '((s_options&0x80U)?100.0f:10.0f)')
    edit('CHASSIS/USER/vehicle_arm.c', arm)

    def params(t):
        replacements = [
            ('#define VEHICLE_SOURCE_Z_PULSES 105000.0f', '#define VEHICLE_SOURCE_Z_PULSES 108400.0f /* current empty-gripper function accepted */'),
            ('#define VEHICLE_LIFT_MAX_TARGET_PULSES 105000.0f', '#define VEHICLE_LIFT_MAX_TARGET_PULSES 108400.0f'),
            ('#define VEHICLE_CARRIER_Z_PULSES 28000.0f', '#define VEHICLE_CARRIER_Z_PULSES (-1.0f) /* new carrier height not measured */'),
            ('#define VEHICLE_SOURCE_PREP_DM_SPEED_DEG_S 150.0f', '#define VEHICLE_SOURCE_PREP_DM_SPEED_DEG_S 300.0f'),
            ('#define VEHICLE_SOURCE_PREP_EXTENSION_RPM 30.0f', '#define VEHICLE_SOURCE_PREP_EXTENSION_RPM 106.0f'),
            ('#define VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S 60U', '#define VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S 45055U'),
            ('#define STARTUP_DM_SPEED_SCALE 0.5f', '#define STARTUP_DM_SPEED_SCALE 1.0f'),
        ]
        for old, new in replacements:
            assert t.count(old) == 1, old
            t = t.replace(old, new)
        return t.replace('/* 300×0.5=150deg/s，按本次角度逆时针到0。 */', '/* Reference300deg/s, unchanged direction. */')
    edit('USER/vehicle_parameters.h', params)
    edit('CHASSIS/USER/source_prepare.inc', lambda t: t.replace('VEHICLE_SOURCE_PREP_DM_SPEED_DEG_S<=150', 'VEHICLE_SOURCE_PREP_DM_SPEED_DEG_S<=300').replace('VEHICLE_SOURCE_PREP_EXTENSION_RPM<=30', 'VEHICLE_SOURCE_PREP_EXTENSION_RPM<=106').replace('VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S<=300', 'VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S<=45055'))

    files = [ROOT / 'software_interfaces/source_observer/source_observer.c',
             ROOT / 'software_interfaces/source_observer/source_observer.h',
             ROOT / 'software_interfaces/source_observer/source_observer_config.h',
             ROOT / 'software_interfaces/stage2_metric_alignment/metric_alignment.h',
             ROOT / 'software_interfaces/stage2_metric_alignment/metric_alignment_bridge.h']
    for p in files:
        name = 'CHASSIS/USER/' + p.name
        assert not (ACTUAL / name).exists()
        shutil.copy2(p, STAGE / name)
        before[name] = None
        after[name] = hashlib.sha256(p.read_bytes()).hexdigest()

    def main_c(t):
        marker = '#include "vehicle_arm.h"'
        assert t.count(marker) == 1
        t = t.replace(marker, marker + '\n#include "source_observer.h"\n#include "source_observer_config.h"')
        marker = '            g_serial_rx_last = data;'
        assert t.count(marker) == 1
        t = t.replace(marker, marker + '\n            SourceObserver_Byte(data,(uint32_t)xTaskGetTickCount());')
        marker = '        App_StartGateUpdate();'
        assert t.count(marker) == 1
        t = t.replace(marker, marker + '''
        {
            static uint32_t observer_initialized;
            static char observer_request[256];
            if(!observer_initialized){
                MetricAlignmentProfile p=SOURCE_OBSERVER_PROFILE_INITIALIZER;
                /* No field calibration is synthesized from old images. */
                SourceObserver_Init(&p);observer_initialized=1U;
            }
            SourceObserver_Service((uint32_t)xTaskGetTickCount());
            if(SourceObserver_Request(observer_request,sizeof(observer_request)))
                (void)App_DebugSendString(observer_request);
        }''')
        return t
    edit('CHASSIS/USER/main.c', main_c)
    project = STAGE / 'gongxun-f407.uvprojx'
    before['gongxun-f407.uvprojx'] = hashlib.sha256(project.read_bytes()).hexdigest()
    tree = ET.parse(project)
    for target in tree.getroot().findall('./Targets/Target'):
        old = target.findtext('TargetName')
        name = 'Vehicle_SourceObserver_Offline' if 'Offline' in old else 'Vehicle_SourceObserver'
        target.find('TargetName').text = name
        common = target.find('./TargetOption/TargetCommonOption')
        common.find('OutputName').text = name
        common.find('OutputDirectory').text = '.\\Objects\\' + name + '\\'
        common.find('ListingPath').text = '.\\Listings\\' + name + '\\'
        group = ET.SubElement(target.find('Groups'), 'Group')
        ET.SubElement(group, 'GroupName').text = 'Source_Observation_Only'
        item = ET.SubElement(ET.SubElement(group, 'Files'), 'File')
        ET.SubElement(item, 'FileName').text = 'source_observer.c'
        ET.SubElement(item, 'FileType').text = '1'
        ET.SubElement(item, 'FilePath').text = '.\\CHASSIS\\USER\\source_observer.c'
    tree.write(project, encoding='utf-8', xml_declaration=True)
    after['gongxun-f407.uvprojx'] = hashlib.sha256(project.read_bytes()).hexdigest()
    report = {'actual': str(ACTUAL), 'stage': str(STAGE), 'before_sha256': before, 'candidate_sha256': after,
              'new_source_target_pulses': 108400, 'carrier_height_intentionally_unconfigured': True,
              'node5_scale_decode_corrected': True, 'observation_profile_calibrated': False,
              'position_executor_enabled': False, 'stage3_released': False,
              'UART_candidate_port': 'existing USART1 debug chain; actual SBC port still needs field confirmation',
              'actual_source_updated': False, 'firmware_downloaded': False,
              'current_multi_axis_trajectory_field_validation_complete': False,
              'direct_field_flash_ready': False}
    (OUT / 'firmware-staged-manifest.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'staged': True, 'files': len(after), 'profile_calibrated': False, 'flash_ready': False}))


if __name__ == '__main__':
    main()
