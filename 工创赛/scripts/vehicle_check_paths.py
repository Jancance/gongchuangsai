"""整车软件桩检查的源码入口；不访问车辆或串口。"""
import os
from pathlib import Path


def firmware_root():
    """可显式指定待验源码，否则使用仓库工程或本地验证副本。"""
    root = Path(__file__).resolve().parents[1]
    configured = os.environ.get('VEHICLE_SOURCE_ROOT')
    candidates = [Path(configured)] if configured else [
        root.parent / '3.stm32f407工程',
        root / 'tmp/stage1_early_scan_startup_20261103',
    ]
    for candidate in candidates:
        if (candidate / 'USER/vehicle_parameters.h').is_file():
            return candidate.resolve()
    raise FileNotFoundError('请将 VEHICLE_SOURCE_ROOT 指向含 USER/vehicle_parameters.h 的整合工程')
