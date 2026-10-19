from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class RobotMode(str, Enum):
    BOOT = "BOOT"
    SELF_TEST = "SELF_TEST"
    READY = "READY"
    RUNNING = "RUNNING"
    SAFE_STOP = "SAFE_STOP"
    FAULT = "FAULT"
    DONE = "DONE"


class PlanAction(str, Enum):
    FOLLOW = "FOLLOW"
    GLOBAL_REPLAN = "GLOBAL_REPLAN"
    STOP = "STOP"


class MotionState(str, Enum):
    MOVING = "MOVING"
    STATIONARY = "STATIONARY"
    UNKNOWN = "UNKNOWN"


class ActionStatus(str, Enum):
    ACCEPTED = "ACCEPTED"
    DONE = "DONE"
    ERROR = "ERROR"


@dataclass(frozen=True)
class Pose2D:
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class ObservationBase:
    """Frozen P2 observation contract shared by every detector."""

    timestamp_ns: int
    frame_id: int
    source_id: str
    calibration_id: str
    track_id: str | None
    confidence: float
    age_ms: float
    pixel_xy: tuple[float, float] | None = None
    ground_xy_mm: tuple[float, float] | None = None
    robot_xy_mm: tuple[float, float] | None = None
    pose: Pose2D | None = None
    uncertainty: tuple[float, float] | None = None
    visible_fraction: float = 1.0
    rejection_reason: str | None = None

    @property
    def accepted(self) -> bool:
        return self.rejection_reason is None and self.confidence > 0.0


@dataclass(frozen=True)
class MaterialObservation(ObservationBase):
    color: str = "UNKNOWN"
    grasp_xy_mm: tuple[float, float] | None = None
    motion_state: MotionState = MotionState.UNKNOWN


@dataclass(frozen=True)
class RingObservation(ObservationBase):
    ring_id: int | None = None
    radius_mm: float | None = None


@dataclass(frozen=True)
class LaneObservation(ObservationBase):
    lateral_error_mm: float = 0.0
    heading_error_rad: float = 0.0
    visible_distance_mm: float = 0.0
    lane_left_mm: float = -200.0
    lane_right_mm: float = 200.0


@dataclass(frozen=True)
class IntersectionObservation(ObservationBase):
    branches: tuple[str, ...] = ()
    event_id: int = 0


@dataclass(frozen=True)
class ObstacleObservation(ObservationBase):
    obstacles: tuple["Obstacle", ...] = ()


@dataclass(frozen=True)
class HomeZoneObservation(ObservationBase):
    start_id: int = 0
    all_corners_inside: bool = False


@dataclass(frozen=True)
class PlacementVerification(ObservationBase):
    action_seq: int = 0
    target_id: str = ""
    visual_present: bool = False
    centered: bool = False
    stable: bool = False


@dataclass(frozen=True)
class VisionHealth:
    timestamp_ns: int
    healthy: bool
    camera_age_ms: dict[str, float] = field(default_factory=dict)
    calibration_ids: dict[str, str] = field(default_factory=dict)
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True)
class Frame:
    image: Any
    timestamp_ns: int
    sequence: int
    source_id: str
    calibration_id: str


@dataclass(frozen=True)
class Obstacle:
    x_mm: float
    y_mm: float
    radius_mm: float
    confidence: float


@dataclass(frozen=True)
class Observations:
    frame_id: int
    timestamp_ns: int
    calibration_id: str
    confidence: float
    obstacles: tuple[Obstacle, ...] = ()
    lane_left_mm: float = -200.0
    lane_right_mm: float = 200.0
    rejection_reason: str | None = None


@dataclass(frozen=True)
class RobotState:
    timestamp_ns: int
    mode: RobotMode
    stm32_heartbeat_age_ms: float
    tof_min_mm: float | None = None
    lateral_mm: float = 0.0
    speed_mm_s: float = 0.0
    pose: Pose2D = field(default_factory=Pose2D)
    angular_speed_rad_s: float = 0.0
    yaw_rad: float = 0.0
    yaw_rate_rad_s: float = 0.0
    tof_left_mm: float | None = None
    tof_center_mm: float | None = None
    tof_right_mm: float | None = None
    tray_occupied: tuple[bool, bool, bool] = (False, False, False)
    gripper_state: str = "UNKNOWN"
    motor_faults: tuple[str, ...] = ()


@dataclass(frozen=True)
class ActionResult:
    seq: int
    status: ActionStatus
    final_pose: Pose2D | None = None
    error_code: str | None = None


@dataclass(frozen=True)
class SafePlan:
    action: PlanAction
    target_lateral_mm: float
    speed_mm_s: float
    reason: str
    waypoints_mm: tuple[tuple[float, float], ...] = field(default_factory=tuple)
