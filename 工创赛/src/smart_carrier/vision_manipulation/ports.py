"""Small module interfaces for the chassis owner and existing F407 executor.

These are required callbacks, not default success stubs. No port implementation
may convert command acceptance into physical arrival, held or placed evidence.
"""
from dataclasses import dataclass
from typing import Callable
from .types import Action, Arrival, Feedback, Zone


@dataclass
class ChassisPort:
    travel_begin: Callable[[str, Zone, dict], bool]
    travel_poll: Callable[[str], Arrival | None]
    align_begin: Callable[[Action], bool]
    align_poll: Callable[[Action], Feedback | None]
    hold: Callable[[str], bool]
    stop: Callable[[str], None]
    stop_poll: Callable[[str], bool]


@dataclass
class ActuatorPort:
    begin: Callable[[Action], bool]
    poll: Callable[[Action], Feedback | None]
    stop: Callable[[str], None]
    stop_poll: Callable[[str], bool]


def split_axis_correction(amount_mm: float, adapter_step_cap_mm: float) -> tuple[float, ...]:
    """Respect a backend capability by splitting, not by declaring a fault.

Caller reobserves between steps. This is planning output, not an auto-loop or
permission to execute the sum using stale frames.
"""
    import math
    if not math.isfinite(amount_mm) or not math.isfinite(adapter_step_cap_mm) or adapter_step_cap_mm <= 0:
        raise ValueError('纠偏量需有限，执行器步长能力需为正')
    if amount_mm == 0:return ()
    n = math.ceil(abs(amount_mm)/adapter_step_cap_mm)
    return (amount_mm/n,)*n
