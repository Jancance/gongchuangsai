"""All user tuning in one document; suggested values never equality-lock IO."""
from __future__ import annotations
from copy import deepcopy
from dataclasses import dataclass
import json, math
from pathlib import Path


class TeachingRequired(ValueError):
    pass


@dataclass
class Parameters:
    values: dict

    def __post_init__(self):
        self.values = deepcopy(self.values)

    @property
    def tuning(self):
        return self.values['tuning']

    def zone(self, zone):
        return self.values['zones'][str(getattr(zone, 'value', zone))]

    def required(self, mapping, key, label):
        value = mapping.get(key)
        if value is None:
            raise TeachingRequired(label)
        if not isinstance(value, (float, int)) or not math.isfinite(value):
            raise TeachingRequired(label + '：需要有限数值')
        return float(value)

    def pose(self, raw, label, *, depth_key=None, stack=False):
        if raw is None:
            raise TeachingRequired(label + '：请示教4310角度和伸缩量')
        p = {'yaw_deg': self.required(raw, 'yaw_deg', label+'旋转角度'),
             'extension_mm': self.required(raw, 'extension_mm', label+'伸缩距离'),
             'descent_mm': 0.0}
        if depth_key:
            p['descent_mm'] = self.required(raw, depth_key, label+'从顶部下降多少mm')
            if stack:
                p['descent_mm'] -= float(self.tuning['stack_height_mm'])
        limits = self.values['physical_limits']
        if not limits['yaw_min_deg'] <= p['yaw_deg'] <= limits['yaw_max_deg']:
            raise TeachingRequired(label+'超过当前实际旋转范围；更正目标或更新实测范围')
        if not 0 <= p['extension_mm'] <= limits['extension_max_mm']:
            raise TeachingRequired(label+'超过当前实际伸缩行程；更正目标或更新实测行程')
        if p['descent_mm'] < 0:
            raise TeachingRequired(label+'第二层高于当前顶部放料能力')
        maximum = limits.get('lift_descent_max_mm')
        if depth_key and maximum is None:
            raise TeachingRequired('由升降标定/执行器接口提供实际可用下降行程mm')
        if maximum is not None and p['descent_mm'] > maximum:
            raise TeachingRequired(label+'超过当前实际升降行程')
        return p

    def speeds(self):
        keys = ('dm_deg_s', 'extension_rpm', 'lift_rpm', 'stepper_accel_rpm_s')
        result = {k: self.required(self.tuning, k, k) for k in keys}
        if any(x <= 0 for x in result.values()):
            raise TeachingRequired('速度/加速度应为正数')
        return result


def load_parameters(path: str | Path) -> Parameters:
    return Parameters(json.loads(Path(path).read_text(encoding='utf-8')))
