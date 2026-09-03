#!/usr/bin/env python3
"""Local RobStride dashboard for the RobStride USB-CAN adapter."""

from __future__ import annotations

import argparse
import json
import math
import mimetypes
import os
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, urlparse

from robstride_usb import (
    DEFAULT_ACCEL_RAD_S2,
    DEFAULT_CURRENT_LIMIT_A,
    DEFAULT_HOST_ID,
    DEFAULT_MODEL,
    DEFAULT_MOTOR_ID,
    DEFAULT_POSITION_ACCEL_RAD_S2,
    DEFAULT_POSITION_KP,
    DEFAULT_POSITION_RAD,
    DEFAULT_POSITION_VEL_RAD_S,
    DEFAULT_SERIAL_BAUD,
    DEFAULT_SERIAL_PORT,
    DEFAULT_SPEED_RAD_S,
    PARAM_ACC_RAD,
    PARAM_LOC_REF,
    PARAM_LIMIT_CUR,
    PARAM_LOC_KP,
    PARAM_PP_ACC_SET,
    PARAM_PP_VEL_MAX,
    PARAM_RUN_MODE,
    PARAM_SPD_REF,
    PRIVATE_HOST_CANDIDATES,
    PRIVATE_MODEL_LIMITS,
    PROTOCOL_PRIVATE,
    RUN_MODE_OPERATION,
    RUN_MODE_POSITION,
    RUN_MODE_VELOCITY,
    SCAN_FIRST_PRIVATE_ID,
    SCAN_LAST_ID,
    SCAN_PER_ID_TIMEOUT_S,
    TRANSPORT_ROBSTRIDE_USB,
    build_private_ext_id,
    bytes_hex,
    create_bus,
    decode_private_fault_payload,
    f32_le,
    float_to_uint,
    fmt_id,
    private_fault_summary,
    read_f32_le,
    split_private_ext_id,
    uint_to_float,
)


COMM_GET_ID = 0x00
COMM_OPERATION_CONTROL = 0x01
COMM_OPERATION_STATUS = 0x02
COMM_ENABLE = 0x03
COMM_DISABLE = 0x04
COMM_SET_DEVICE_ID = 0x07
COMM_READ_PARAM = 0x11
COMM_WRITE_PARAM = 0x12
COMM_FAULT = 0x15
COMM_SAVE_PARAMETERS = 0x16
COMM_PROACTIVE_REPORT = 0x18

MOTOR_STUDIO_JOG_SPEED_RAD_S = 1.0
MOTOR_STUDIO_JOG_S = 0.75
OSCILLATION_PERIOD_S = 2.5
VELOCITY_REFRESH_S = 0.10
POSITION_REFRESH_S = 0.10
ARM_OPERATION_REFRESH_S = 0.03
USB_OPEN_SETTLE_S = 0.20
PRIVATE_SCAN_RECOVERY_ATTEMPTS = 3
PRIVATE_SCAN_RECOVERY_SETTLE_S = 0.35
STARTUP_SCAN_DELAY_S = 1.25
STARTUP_SCAN_RETRY_S = 2.0
STARTUP_SCAN_ROUNDS = 1
MAX_LOG_LINES = 240
MAX_FRAME_HISTORY = 600
COMMAND_TIMEOUT_S = 0.6
TELEMETRY_ACTIVE_WINDOW_S = 5.0
SHUTDOWN_COMMAND_TIMEOUT_S = 2.0
ARM_AXES = ("base", "shoulder", "elbow")
ARM_JOINT_COUNTS = (2, 3)
ARM_ROLE_DEFAULT_IDS = {"base": 0x01, "shoulder": 0x02, "elbow": 0x03}
ARM_TWIST_DEFAULT_LIMIT_RAD = math.pi
ARM_TWIST_MIN_LIMIT_RAD = math.radians(1.0)
ARM_TWIST_MAX_LIMIT_RAD = math.pi
ARM_ROUTE_MAX_STEP_RAD = math.radians(45.0)
ARM_PRESET_MAX_STEP_RAD = math.radians(12.0)
ARM_ROUTE_SETTLE_S = 0.08
ARM_ROUTE_MIN_INTERVAL_S = 0.12
ARM_MIN_TARGET_REACH = 0.001
ARM_BASE_PLANE_MIN_Z = 0.0
ARM_CURRENT_LIMIT_MAX_A = 10.0
ARM_DEFAULT_POSITION_VEL_RAD_S = 0.35
ARM_DEFAULT_POSITION_ACCEL_RAD_S2 = 2.5
ARM_DEFAULT_POSITION_KP = 4.0
ARM_DEFAULT_DAMPING_KD = 2.0
ARM_DEFAULT_CURRENT_LIMIT_A = 4.0
ARM_POSITION_VEL_MAX_RAD_S = 1.5
ARM_POSITION_ACCEL_MAX_RAD_S2 = 8.0
ARM_POSITION_KP_MAX = 10.0
ARM_DAMPING_KD_MAX = 5.0
ARM_TORQUE_BIAS_MAX_NM = 5.0
ARM_ASSIST_FADE_BAND_RAD = math.radians(7.0)
ARM_ASSIST_TARGET_SCALE = 0.65
ARM_ASSIST_OVERSHOOT_FADE_BAND_RAD = math.radians(3.0)
ARM_ASSIST_OVERSHOOT_SCALE = 0.35
ARM_ASSIST_FEEDBACK_MISSING_SCALE = 0.65
ARM_FEEDBACK_START_MAX_AGE_S = 1.0
ARM_MOTION_PRESET_LABELS = {
    "showcase": "Showcase",
    "sweep": "Sweep",
    "lift": "Lift",
    "orbit": "Orbit",
    "flex": "Flex",
}
SHUTDOWN_COMMANDS = (
    ("/usr/bin/sudo", "-n", "/usr/local/sbin/helion-poweroff"),
    ("/bin/sudo", "-n", "/usr/local/sbin/helion-poweroff"),
    ("/usr/bin/sudo", "-n", "/usr/bin/systemctl", "poweroff"),
    ("/bin/sudo", "-n", "/usr/bin/systemctl", "poweroff"),
    ("/usr/bin/sudo", "-n", "/bin/systemctl", "poweroff"),
    ("/bin/sudo", "-n", "/bin/systemctl", "poweroff"),
    ("/usr/bin/sudo", "-n", "/usr/sbin/shutdown", "-h", "now"),
    ("/bin/sudo", "-n", "/usr/sbin/shutdown", "-h", "now"),
    ("/usr/bin/sudo", "-n", "/sbin/shutdown", "-h", "now"),
    ("/bin/sudo", "-n", "/sbin/shutdown", "-h", "now"),
    ("/usr/local/sbin/helion-poweroff",),
    ("/usr/bin/systemctl", "poweroff"),
    ("/bin/systemctl", "poweroff"),
    ("/usr/sbin/shutdown", "-h", "now"),
    ("/sbin/shutdown", "-h", "now"),
)

ROOT_DIR = Path(__file__).resolve().parent
WEB_DIR = ROOT_DIR / "web"
VALUES_PATH = Path(
    os.environ.get(
        "HELION_VALUES_PATH",
        Path.home() / ".config" / "helion" / "dashboard-values.json",
    )
)
APP_VERSION = "2026.09.03.4"


def parse_int(value: Any, default: int) -> int:
    if value is None or value == "":
        return default
    if isinstance(value, int):
        return value
    return int(str(value), 0)


def private_device_id(value: Any, name: str, default: Optional[int] = None, allow_zero: bool = False) -> int:
    if default is None and (value is None or value == ""):
        raise ValueError(f"{name} is required")
    parsed = parse_int(value, default if default is not None else -1)
    minimum = 0 if allow_zero else 1
    if parsed < minimum or parsed > SCAN_LAST_ID:
        first = "0x00" if allow_zero else "0x01"
        raise ValueError(f"{name} must be {first}..{fmt_id(SCAN_LAST_ID)}, got {fmt_id(parsed & 0xFF)}")
    return parsed & 0xFF


def parse_float(value: Any, default: float) -> float:
    if value is None or value == "":
        return default
    return float(value)


def positive_float(value: Any, default: float, maximum: float) -> float:
    try:
        parsed = parse_float(value, default)
    except (TypeError, ValueError):
        parsed = default
    if not math.isfinite(parsed) or parsed <= 0.0:
        parsed = default
    return min(abs(parsed), maximum)


def nonnegative_float(value: Any, default: float, maximum: float) -> float:
    try:
        parsed = parse_float(value, default)
    except (TypeError, ValueError):
        parsed = default
    if not math.isfinite(parsed) or parsed < 0.0:
        parsed = default
    return min(parsed, maximum)


def limited_float(value: Any, default: float, maximum_abs: float) -> float:
    try:
        parsed = parse_float(value, default)
    except (TypeError, ValueError):
        parsed = default
    if not math.isfinite(parsed):
        parsed = default
    limit = abs(maximum_abs)
    return min(max(parsed, -limit), limit)


def twist_limit_rad(value: Any, default: float = ARM_TWIST_DEFAULT_LIMIT_RAD) -> float:
    try:
        parsed = parse_float(value, default)
    except (TypeError, ValueError):
        parsed = default
    if not math.isfinite(parsed) or parsed <= 0.0:
        parsed = default
    return min(max(abs(parsed), ARM_TWIST_MIN_LIMIT_RAD), ARM_TWIST_MAX_LIMIT_RAD)


def signed_direction(value: Any, default: int = 1) -> int:
    try:
        parsed = int(str(value), 0)
    except (TypeError, ValueError):
        parsed = default
    return -1 if parsed < 0 else 1


def model_name(value: Any, default: str = DEFAULT_MODEL) -> str:
    parsed = str(value or default).lower()
    return parsed if parsed in PRIVATE_MODEL_LIMITS else default


def arm_joint_count(value: Any, default: int = 3) -> int:
    try:
        parsed = int(str(value), 0)
    except (TypeError, ValueError):
        parsed = default
    return parsed if parsed in ARM_JOINT_COUNTS else default if default in ARM_JOINT_COUNTS else 3


def arm_axes_for_count(joint_count: int) -> Tuple[str, ...]:
    return ("base", "shoulder") if joint_count == 2 else ARM_AXES


def point_sub(a: Dict[str, float], b: Dict[str, float]) -> Dict[str, float]:
    return {"x": a["x"] - b["x"], "y": a["y"] - b["y"], "z": a["z"] - b["z"]}


def point_add(a: Dict[str, float], b: Dict[str, float]) -> Dict[str, float]:
    return {"x": a["x"] + b["x"], "y": a["y"] + b["y"], "z": a["z"] + b["z"]}


def point_scale(a: Dict[str, float], scale: float) -> Dict[str, float]:
    return {"x": a["x"] * scale, "y": a["y"] * scale, "z": a["z"] * scale}


def point_dot(a: Dict[str, float], b: Dict[str, float]) -> float:
    return a["x"] * b["x"] + a["y"] * b["y"] + a["z"] * b["z"]


def point_length(a: Dict[str, float]) -> float:
    return math.sqrt(point_dot(a, a))


def trim_segment(
    start: Dict[str, float],
    end: Dict[str, float],
    trim_start: float,
    trim_end: float,
) -> Optional[Tuple[Dict[str, float], Dict[str, float]]]:
    vector = point_sub(end, start)
    length = point_length(vector)
    if length <= 0.000001 or trim_start + trim_end >= length:
        return None
    direction = point_scale(vector, 1.0 / length)
    return (
        point_add(start, point_scale(direction, trim_start)),
        point_add(end, point_scale(direction, -trim_end)),
    )


def closest_point_on_segment_distance(
    point: Dict[str, float],
    start: Dict[str, float],
    end: Dict[str, float],
) -> float:
    segment = point_sub(end, start)
    length_sq = point_dot(segment, segment)
    if length_sq <= 0.000001:
        return point_length(point_sub(point, start))
    t = max(0.0, min(1.0, point_dot(point_sub(point, start), segment) / length_sq))
    closest = point_add(start, point_scale(segment, t))
    return point_length(point_sub(point, closest))


def segment_distance(
    a0: Dict[str, float],
    a1: Dict[str, float],
    b0: Dict[str, float],
    b1: Dict[str, float],
) -> float:
    # Sampling both trimmed capsules is stable enough for short robot-arm links
    # and avoids false positives at the shared elbow joint.
    distances = []
    for i in range(17):
        t = i / 16.0
        point = point_add(a0, point_scale(point_sub(a1, a0), t))
        distances.append(closest_point_on_segment_distance(point, b0, b1))
    for i in range(17):
        t = i / 16.0
        point = point_add(b0, point_scale(point_sub(b1, b0), t))
        distances.append(closest_point_on_segment_distance(point, a0, a1))
    return min(distances) if distances else 0.0


def routed_angle_within_twist(angle: float, reference: float, twist_limit: float) -> float:
    limit = twist_limit_rad(twist_limit)
    if not math.isfinite(angle):
        return 0.0
    reference_angle = reference if math.isfinite(reference) else 0.0
    center_turn = round((reference_angle - angle) / math.tau)
    candidates = []
    for turn in range(center_turn - 2, center_turn + 3):
        candidate = angle + (turn * math.tau)
        if abs(candidate) <= limit + 0.000001:
            candidates.append(candidate)
    if not candidates:
        return angle + (center_turn * math.tau)
    return min(candidates, key=lambda item: (abs(item - reference_angle), abs(item)))


def route_arm_joint_angles(
    joint_count: int,
    joint_angles: Dict[str, float],
    twist_limits: Dict[str, float],
    previous_joint_angles: Optional[Dict[str, float]] = None,
) -> Dict[str, float]:
    routed = dict(joint_angles)
    previous = previous_joint_angles or {}
    for axis in arm_axes_for_count(joint_count):
        try:
            angle = float(joint_angles.get(axis, 0.0))
        except (TypeError, ValueError):
            angle = 0.0
        try:
            reference = float(previous.get(axis, 0.0))
        except (TypeError, ValueError):
            reference = 0.0
        routed[axis] = routed_angle_within_twist(
            angle,
            reference,
            twist_limits.get(axis, ARM_TWIST_DEFAULT_LIMIT_RAD),
        )
    if joint_count == 2:
        routed["elbow"] = 0.0
    return routed


def clamp_arm_target_to_reach(
    joint_count: int,
    target: Dict[str, float],
    link_1: float,
    link_2: float,
) -> Dict[str, float]:
    x = float(target.get("x", 0.0))
    y = float(target.get("y", 0.0))
    z = float(target.get("z", 0.0))
    if not math.isfinite(x):
        x = 0.0
    if not math.isfinite(y):
        y = 0.0
    if not math.isfinite(z):
        z = 0.0
    z = max(z, ARM_BASE_PLANE_MIN_Z)
    max_reach = max(abs(link_1) + abs(link_2), 0.001)
    min_reach = ARM_MIN_TARGET_REACH if joint_count == 2 else abs(abs(link_1) - abs(link_2))
    min_reach = min(min_reach, max_reach)
    reach = math.sqrt(x * x + y * y + z * z)

    if reach > max_reach:
        scale = max_reach / reach
        return {"x": x * scale, "y": y * scale, "z": z * scale}
    if min_reach > 0.0 and reach < min_reach:
        if reach <= 0.0000001:
            return {"x": min_reach, "y": 0.0, "z": 0.0}
        scale = min_reach / reach
        return {"x": x * scale, "y": y * scale, "z": z * scale}
    return {"x": x, "y": y, "z": z}


def plan_arm_joint_route(
    joint_count: int,
    start_angles: Dict[str, float],
    target_angles: Dict[str, float],
    max_step_rad: float = ARM_ROUTE_MAX_STEP_RAD,
) -> List[Dict[str, float]]:
    axes = arm_axes_for_count(joint_count)
    deltas = {
        axis: float(target_angles.get(axis, 0.0)) - float(start_angles.get(axis, 0.0))
        for axis in axes
    }
    largest_delta = max((abs(delta) for delta in deltas.values()), default=0.0)
    steps = max(1, int(math.ceil(largest_delta / max(max_step_rad, 0.001))))
    waypoints: List[Dict[str, float]] = []
    for step in range(1, steps + 1):
        blend = step / steps
        waypoint = dict(start_angles)
        for axis in axes:
            waypoint[axis] = float(start_angles.get(axis, 0.0)) + (deltas[axis] * blend)
        if joint_count == 2:
            waypoint["elbow"] = 0.0
        waypoints.append(waypoint)
    return waypoints


def arm_twist_safety_check(
    joint_count: int,
    joint_angles: Dict[str, float],
    twist_limits: Dict[str, float],
) -> List[str]:
    warnings: List[str] = []
    labels = {"base": "Base", "shoulder": "Shoulder", "elbow": "Elbow"}
    for axis in arm_axes_for_count(joint_count):
        angle = float(joint_angles.get(axis, 0.0))
        limit = twist_limit_rad(twist_limits.get(axis, ARM_TWIST_DEFAULT_LIMIT_RAD))
        if abs(angle) > limit + 0.000001:
            warnings.append(
                f"{labels[axis]} twist {math.degrees(angle):.1f} deg exceeds "
                f"+/-{math.degrees(limit):.1f} deg from home"
            )
    return warnings


@dataclass
class ArmIkSolution:
    base: float
    shoulder: float
    elbow: float


def solve_three_axis_arm_ik(
    x: float,
    y: float,
    z: float,
    link_1: float,
    link_2: float,
    elbow_up: bool,
) -> ArmIkSolution:
    link_1 = max(abs(link_1), 0.001)
    link_2 = max(abs(link_2), 0.001)
    radial = math.hypot(x, y)
    reach = math.hypot(radial, z)
    max_reach = link_1 + link_2
    min_reach = abs(link_1 - link_2)
    if reach > max_reach or reach < min_reach:
        raise ValueError(
            f"IK target unreachable: reach={reach:.3f}, allowed={min_reach:.3f}..{max_reach:.3f}"
        )

    base = math.atan2(y, x)
    cos_elbow = (radial * radial + z * z - link_1 * link_1 - link_2 * link_2) / (2.0 * link_1 * link_2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow = math.acos(cos_elbow)
    if elbow_up:
        elbow = -elbow
    shoulder = math.atan2(z, radial) - math.atan2(link_2 * math.sin(elbow), link_1 + link_2 * math.cos(elbow))
    return ArmIkSolution(base=base, shoulder=shoulder, elbow=elbow)


def solve_arm_ik(
    joint_count: int,
    x: float,
    y: float,
    z: float,
    link_1: float,
    link_2: float,
    elbow_up: bool,
) -> ArmIkSolution:
    if joint_count != 2:
        return solve_three_axis_arm_ik(x, y, z, link_1, link_2, elbow_up)

    link = max(abs(link_1) + abs(link_2), 0.001)
    radial = math.hypot(x, y)
    reach = math.hypot(radial, z)
    if reach <= 0.0001:
        raise ValueError("IK target unreachable: 2-joint target cannot be at the base origin")
    if reach > link:
        raise ValueError(f"IK target unreachable: reach={reach:.3f}, allowed=0.000..{link:.3f}")
    return ArmIkSolution(
        base=math.atan2(y, x),
        shoulder=math.atan2(z, radial),
        elbow=0.0,
    )


def arm_solution_points(
    joint_count: int,
    solution: ArmIkSolution,
    target: Dict[str, float],
    link_1: float,
    link_2: float,
) -> List[Dict[str, float]]:
    p0 = {"x": 0.0, "y": 0.0, "z": 0.0}
    if joint_count == 2:
        return [p0, {"x": target["x"], "y": target["y"], "z": target["z"]}]

    base_dir = {"x": math.cos(solution.base), "y": math.sin(solution.base)}
    p1 = {
        "x": link_1 * math.cos(solution.shoulder) * base_dir["x"],
        "y": link_1 * math.cos(solution.shoulder) * base_dir["y"],
        "z": link_1 * math.sin(solution.shoulder),
    }
    p2 = {
        "x": p1["x"] + link_2 * math.cos(solution.shoulder + solution.elbow) * base_dir["x"],
        "y": p1["y"] + link_2 * math.cos(solution.shoulder + solution.elbow) * base_dir["y"],
        "z": p1["z"] + link_2 * math.sin(solution.shoulder + solution.elbow),
    }
    return [p0, p1, p2]


def arm_safety_check(
    joint_count: int,
    points: List[Dict[str, float]],
    link_radii: Dict[str, float],
    joint_angles: Optional[Dict[str, float]] = None,
    twist_limits: Optional[Dict[str, float]] = None,
) -> Dict[str, Any]:
    warnings: List[str] = []
    min_z = min((float(point.get("z", 0.0)) for point in points), default=0.0)
    if min_z < ARM_BASE_PLANE_MIN_Z - 0.000001:
        warnings.append(
            f"arm dips below base plane: min Z={min_z:.3f} m; "
            "raise the target or use Elbow Up"
        )
    if joint_count == 3 and len(points) >= 3:
        radius_1 = max(0.0, link_radii.get("link1", 0.0))
        radius_2 = max(0.0, link_radii.get("link2", 0.0))
        required_clearance = radius_1 + radius_2
        if required_clearance > 0.0:
            link_1_length = point_length(point_sub(points[1], points[0]))
            link_2_length = point_length(point_sub(points[2], points[1]))
            if required_clearance >= min(link_1_length, link_2_length):
                warnings.append(
                    "link radii are too large for the configured lengths: "
                    f"{required_clearance:.3f} m clearance needed"
                )
            else:
                upper = trim_segment(points[0], points[1], 0.0, required_clearance)
                forearm = trim_segment(points[1], points[2], required_clearance, 0.0)
                if upper and forearm:
                    clearance = segment_distance(upper[0], upper[1], forearm[0], forearm[1])
                    if clearance < required_clearance:
                        warnings.append(
                            "link radii overlap: "
                            f"clearance {clearance:.3f} m is below {required_clearance:.3f} m"
                        )
    if joint_angles is not None and twist_limits is not None:
        warnings.extend(
            arm_twist_safety_check(
                joint_count,
                joint_angles,
                twist_limits,
            )
        )
    return {"ok": not warnings, "warnings": warnings}


class DashboardController:
    def __init__(
        self,
        serial_port: str,
        serial_baud: int,
        motor_id: int,
        host_id: int,
        model: str,
        open_can: bool,
    ):
        self.lock = threading.RLock()
        self.frame_cv = threading.Condition(self.lock)
        self.command_lock = threading.Lock()
        self.bus = create_bus(serial_port=serial_port, serial_baud=serial_baud)
        self.connected = False
        self.open_error = ""
        self.running = True
        self.bus_lock = threading.Lock()
        self.frame_seq = 0
        self.frames: Deque[Tuple[int, Any]] = deque(maxlen=MAX_FRAME_HISTORY)
        self.logs: Deque[str] = deque(maxlen=MAX_LOG_LINES)

        self.motor_id = motor_id & 0xFF
        self.host_id = host_id & 0xFF
        self.model = model.lower()
        self.test_speed = DEFAULT_SPEED_RAD_S
        self.commanded_speed = 0.0
        self.position_target = DEFAULT_POSITION_RAD
        self.position_velocity_limit = DEFAULT_POSITION_VEL_RAD_S
        self.position_acceleration = DEFAULT_POSITION_ACCEL_RAD_S2
        self.position_kp = DEFAULT_POSITION_KP
        self.velocity_configured = False
        self.position_configured = False
        self.arm_joint_count = 3
        self.arm_motor_ids = {
            "base": self.motor_id,
            "shoulder": 0x01,
            "elbow": 0x02,
        }
        self.arm_motor_models = {axis: self.model for axis in ARM_AXES}
        self.arm_offsets = {axis: 0.0 for axis in ARM_AXES}
        self.arm_directions = {axis: 1 for axis in ARM_AXES}
        self.arm_link_1 = 0.25
        self.arm_link_2 = 0.25
        self.arm_link_radii = {"link1": 0.015, "link2": 0.015}
        self.arm_twist_limits = {axis: ARM_TWIST_DEFAULT_LIMIT_RAD for axis in ARM_AXES}
        self.arm_target = {"x": 0.25, "y": 0.0, "z": 0.10}
        self.arm_elbow_up = False
        self.arm_velocity_limit = ARM_DEFAULT_POSITION_VEL_RAD_S
        self.arm_acceleration = ARM_DEFAULT_POSITION_ACCEL_RAD_S2
        self.arm_position_kp = ARM_DEFAULT_POSITION_KP
        self.arm_damping_kd = ARM_DEFAULT_DAMPING_KD
        self.arm_current_limit = ARM_DEFAULT_CURRENT_LIMIT_A
        self.arm_torque_biases = {axis: 0.0 for axis in ARM_AXES}
        self.arm_position_configured = False
        self.arm_position_signature: Optional[Tuple[Any, ...]] = None
        self.arm_current_limit_signature: Optional[Tuple[Any, ...]] = None
        self.arm_motor_targets = {axis: 0.0 for axis in ARM_AXES}
        self.arm_joint_angles = {axis: 0.0 for axis in ARM_AXES}
        self.arm_route_waypoints: Deque[Dict[str, Dict[str, float]]] = deque()
        self.arm_route_next_at = 0.0
        self.active_reports = False
        self.oscillating = False
        self.jog_active = False
        self.jog_stop_at = 0.0
        self.last_oscillation_at = time.monotonic()
        self.last_velocity_refresh_at = 0.0
        self.last_position_refresh_at = 0.0
        self.last_arm_position_refresh_at = 0.0
        self.last_feedback_at = 0.0
        self.last_private_fault_at = 0.0
        self.last_raw_frame: Optional[Dict[str, Any]] = None
        self.last_feedback: Optional[Dict[str, Any]] = None
        self.feedback_by_motor: Dict[int, Dict[str, Any]] = {}
        self.feedback_at_by_motor: Dict[int, float] = {}
        self.last_private_fault: Optional[Dict[str, Any]] = None
        self.private_faults_by_motor: Dict[int, Dict[str, Any]] = {}
        self.private_faults_at_by_motor: Dict[int, float] = {}
        self.discovered_private: List[int] = []
        self.busy = False

        self.load_saved_values()

        if open_can:
            self.open_bus()

        self.rx_thread = threading.Thread(target=self.rx_loop, name="can-rx", daemon=True)
        self.rx_thread.start()

        self.update_thread = threading.Thread(target=self.update_loop, name="motion-update", daemon=True)
        self.update_thread.start()

        self.startup_scan_thread: Optional[threading.Thread] = None
        if open_can:
            self.startup_scan_thread = threading.Thread(
                target=self.startup_scan_loop,
                name="startup-scan",
                daemon=True,
            )
            self.startup_scan_thread.start()

    def log(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        with self.lock:
            self.logs.append(f"{stamp} {message}")

    def clear_logs(self) -> None:
        with self.lock:
            self.logs.clear()

    def clear_cached_private_faults_locked(self, motor_ids: List[int]) -> None:
        target_ids = {motor_id & 0xFF for motor_id in motor_ids}
        for motor_id in target_ids:
            self.private_faults_by_motor.pop(motor_id, None)
            self.private_faults_at_by_motor.pop(motor_id, None)
        if self.last_private_fault and (int(self.last_private_fault.get("motorId", -1)) & 0xFF) in target_ids:
            self.last_private_fault = None
            self.last_private_fault_at = 0.0

    def bus_label(self) -> str:
        return self.bus.label()

    def bus_transport(self) -> str:
        return getattr(self.bus, "transport", TRANSPORT_ROBSTRIDE_USB)

    def open_bus(self) -> bool:
        with self.lock:
            label = self.bus_label()
        try:
            with self.bus_lock:
                self.bus.open()
        except (RuntimeError, OSError) as exc:
            with self.lock:
                self.connected = False
                self.open_error = str(exc)
            self.log(f"USB adapter open failed on {label}: {exc}")
            return False
        with self.lock:
            self.connected = True
            self.open_error = ""
        time.sleep(USB_OPEN_SETTLE_S)
        self.log(f"Opened {label}")
        return True

    def reopen_bus(
        self,
        serial_port: Optional[str] = None,
        serial_baud: Optional[int] = None,
    ) -> bool:
        with self.lock:
            self.oscillating = False
            self.jog_active = False
            self.velocity_configured = False
            self.position_configured = False
            self.arm_position_configured = False
            self.clear_arm_route()
            next_serial_port = serial_port or getattr(self.bus, "serial_port", DEFAULT_SERIAL_PORT)
            next_serial_baud = serial_baud or getattr(self.bus, "serial_baud", DEFAULT_SERIAL_BAUD)
            self.connected = False
        with self.bus_lock:
            self.bus.close()
            self.bus = create_bus(serial_port=next_serial_port, serial_baud=next_serial_baud)
            time.sleep(0.05)
        return self.open_bus()

    def close(self) -> None:
        self.running = False
        with self.bus_lock:
            self.bus.close()

    def shutdown_host(self) -> Dict[str, Any]:
        self.log("Safe shutdown requested")
        cleanup_errors: List[str] = []

        try:
            if self.connected:
                self.stop_arm()
        except Exception as exc:
            cleanup_errors.append(f"arm stop failed: {exc}")
            self.log(cleanup_errors[-1])

        try:
            if self.connected:
                self.stop_and_disable()
        except Exception as exc:
            cleanup_errors.append(f"selected motor stop failed: {exc}")
            self.log(cleanup_errors[-1])

        try:
            self.save_values()
        except Exception as exc:
            cleanup_errors.append(f"values save failed: {exc}")
            self.log(cleanup_errors[-1])

        with self.bus_lock:
            self.bus.close()
        with self.lock:
            self.connected = False

        if os.name != "posix":
            message = "Safe shutdown is only available on the Raspberry Pi/Linux host."
            self.log(message)
            return {"ok": False, "message": message, "cleanupErrors": cleanup_errors}

        try:
            subprocess.run(
                ("sync",),
                check=False,
                timeout=SHUTDOWN_COMMAND_TIMEOUT_S,
            )
        except (FileNotFoundError, subprocess.SubprocessError) as exc:
            self.log(f"sync before shutdown skipped: {exc}")

        last_error = ""
        attempt_errors: List[str] = []
        identity = []
        if hasattr(os, "getuid"):
            identity.append(f"uid={os.getuid()}")
        if hasattr(os, "geteuid"):
            identity.append(f"euid={os.geteuid()}")
        user = os.environ.get("USER") or os.environ.get("LOGNAME")
        if user:
            identity.append(f"user={user}")
        if identity:
            self.log(f"Safe shutdown process identity: {' '.join(identity)}")
        for command in SHUTDOWN_COMMANDS:
            try:
                completed = subprocess.run(
                    command,
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=SHUTDOWN_COMMAND_TIMEOUT_S,
                )
            except FileNotFoundError:
                last_error = f"{command[0]} not found"
                attempt_errors.append(f"{' '.join(command)}: {last_error}")
                self.log(f"Shutdown attempt failed: {attempt_errors[-1]}")
                continue
            except subprocess.CalledProcessError as exc:
                detail = (exc.stderr or exc.stdout or "").strip()
                last_error = detail or f"exit {exc.returncode}"
                attempt_errors.append(f"{' '.join(command)}: {last_error}")
                self.log(f"Shutdown attempt failed: {attempt_errors[-1]}")
                continue
            except subprocess.SubprocessError as exc:
                last_error = str(exc)
                attempt_errors.append(f"{' '.join(command)}: {last_error}")
                self.log(f"Shutdown attempt failed: {attempt_errors[-1]}")
                continue

            message = "Safe shutdown requested; wait for the Pi activity LED to stop before cutting power."
            if completed.stdout.strip():
                self.log(completed.stdout.strip())
            self.log(message)
            return {"ok": True, "message": message, "cleanupErrors": cleanup_errors}

        message = (
            "Shutdown command failed. Re-run raspi/install_dashboard.sh to grant "
            "passwordless /usr/local/sbin/helion-poweroff, or shut down with sudo poweroff."
        )
        if last_error:
            message += f" Last error: {last_error}"
        if attempt_errors:
            message += f" Attempts: {' | '.join(attempt_errors)}"
        self.log(message)
        return {
            "ok": False,
            "message": message,
            "cleanupErrors": cleanup_errors,
            "attemptErrors": attempt_errors,
        }

    def send(self, arbitration_id: int, data: bytes, extended: bool) -> None:
        error: Optional[Exception] = None
        with self.bus_lock:
            if not self.connected:
                raise RuntimeError("RobStride USB adapter is not open")
            try:
                self.bus.send(arbitration_id, data, extended=extended)
            except (OSError, RuntimeError) as exc:
                error = exc
        if error is not None:
            with self.lock:
                self.connected = False
                self.open_error = str(error)
            self.log(f"USB adapter send failed: {error}")
            raise error

    def send_private(self, comm_type: int, extra_data: int, target_id: int, data: bytes) -> None:
        self.send(build_private_ext_id(comm_type, extra_data, target_id), data, extended=True)

    def model_for_motor(self, motor_id: int) -> str:
        motor_id &= 0xFF
        for axis in ARM_AXES:
            if self.arm_motor_ids.get(axis) == motor_id:
                return self.arm_motor_models.get(axis, self.model)
        return self.model

    def active_arm_axes(self) -> Tuple[str, ...]:
        return arm_axes_for_count(self.arm_joint_count)

    def send_private_get_id(self, target_id: int) -> None:
        self.send_private(COMM_GET_ID, self.host_id, target_id, bytes(8))

    def send_private_get_id_with_host(self, target_id: int, host_id: int) -> None:
        self.send_private(COMM_GET_ID, host_id & 0xFF, target_id & 0xFF, bytes(8))

    def send_private_set_device_id(self, old_id: int, new_id: int, host_id: int, token: bytes) -> None:
        extra = ((new_id & 0xFF) << 8) | (host_id & 0xFF)
        self.send_private(COMM_SET_DEVICE_ID, extra, old_id & 0xFF, bytes(token[:8]).ljust(8, b"\x00"))

    def send_private_save_parameters_to(self, target_id: int, host_id: int) -> None:
        self.send_private(COMM_SAVE_PARAMETERS, host_id & 0xFF, target_id & 0xFF, bytes([1, 2, 3, 4, 5, 6, 7, 8]))

    def send_private_disable_to(self, target_id: int, clear_error: bool = False) -> None:
        payload = bytearray(8)
        payload[0] = 1 if clear_error else 0
        self.send_private(COMM_DISABLE, self.host_id, target_id & 0xFF, bytes(payload))

    def send_private_disable(self, clear_error: bool = False) -> None:
        self.send_private_disable_to(self.motor_id, clear_error)

    def send_private_enable_to(self, target_id: int) -> None:
        self.send_private(COMM_ENABLE, self.host_id, target_id & 0xFF, bytes(8))

    def send_private_enable(self) -> None:
        self.send_private_enable_to(self.motor_id)

    def write_private_param_u8_to(self, target_id: int, index: int, value: int) -> None:
        payload = bytearray(8)
        payload[0:2] = (index & 0xFFFF).to_bytes(2, "little")
        payload[4] = value & 0xFF
        self.send_private(COMM_WRITE_PARAM, self.host_id, target_id & 0xFF, bytes(payload))

    def write_private_param_u8(self, index: int, value: int) -> None:
        self.write_private_param_u8_to(self.motor_id, index, value)

    def write_private_param_f32_to(self, target_id: int, index: int, value: float) -> None:
        payload = bytearray(8)
        payload[0:2] = (index & 0xFFFF).to_bytes(2, "little")
        payload[4:8] = f32_le(value)
        self.send_private(COMM_WRITE_PARAM, self.host_id, target_id & 0xFF, bytes(payload))

    def write_private_param_f32(self, index: int, value: float) -> None:
        self.write_private_param_f32_to(self.motor_id, index, value)

    def read_private_param_from(self, target_id: int, index: int) -> None:
        payload = bytearray(8)
        payload[0:2] = (index & 0xFFFF).to_bytes(2, "little")
        self.send_private(COMM_READ_PARAM, self.host_id, target_id & 0xFF, bytes(payload))

    def read_private_param(self, index: int) -> None:
        self.read_private_param_from(self.motor_id, index)

    def rx_loop(self) -> None:
        while self.running:
            with self.bus_lock:
                connected = self.connected
            if not connected:
                time.sleep(0.25)
                continue
            try:
                frame = self.bus.recv(0.05)
            except OSError as exc:
                with self.lock:
                    self.connected = False
                    self.open_error = str(exc)
                self.log(f"USB adapter receive failed: {exc}")
                continue
            except RuntimeError:
                time.sleep(0.10)
                continue
            if frame is None:
                continue
            self.record_frame(frame)

    def record_frame(self, frame: Any) -> None:
        with self.frame_cv:
            self.frame_seq += 1
            self.frames.append((self.frame_seq, frame))
            self.last_raw_frame = self.frame_json(frame)
            self.parse_feedback_locked(frame)
            self.frame_cv.notify_all()

    def parse_feedback_locked(self, frame: Any) -> None:
        if frame.extended and len(frame.data) >= 8:
            comm_type, extra, host = split_private_ext_id(frame.arbitration_id)
            source_motor = extra & 0xFF
            if comm_type == COMM_GET_ID and source_motor not in self.discovered_private:
                self.discovered_private.append(source_motor)
                self.log(
                    f"Private Get-ID reply motor={fmt_id(source_motor)} "
                    f"host/check={fmt_id(host)} uuid={bytes_hex(frame.data).replace(' ', '')}"
                )
                return
            if comm_type == COMM_OPERATION_STATUS:
                model = self.model_for_motor(source_motor)
                p_max, v_max, t_max = PRIVATE_MODEL_LIMITS.get(
                    model,
                    PRIVATE_MODEL_LIMITS[DEFAULT_MODEL],
                )
                pos_raw = (frame.data[0] << 8) | frame.data[1]
                vel_raw = (frame.data[2] << 8) | frame.data[3]
                torque_raw = (frame.data[4] << 8) | frame.data[5]
                temp_raw = (frame.data[6] << 8) | frame.data[7]
                now = time.monotonic()
                feedback = {
                    "protocol": PROTOCOL_PRIVATE,
                    "targetHost": host,
                    "motorId": source_motor,
                    "motorIdHex": fmt_id(source_motor),
                    "positionRad": uint_to_float(pos_raw, -p_max, p_max, 16),
                    "velocityRadS": uint_to_float(vel_raw, -v_max, v_max, 16),
                    "torqueNm": uint_to_float(torque_raw, -t_max, t_max, 16),
                    "temperatureC": temp_raw * 0.1,
                    "fault": False,
                    "warning": False,
                    "modeState": None,
                    "model": model,
                    "ageMs": 0,
                }
                self.last_feedback_at = now
                self.last_feedback = dict(feedback)
                self.feedback_at_by_motor[source_motor] = now
                self.feedback_by_motor[source_motor] = feedback
                return
            if comm_type in (COMM_READ_PARAM, COMM_WRITE_PARAM):
                index = int.from_bytes(frame.data[0:2], "little")
                raw = frame.data[4:8]
                if index == PARAM_RUN_MODE:
                    self.log(f"param run_mode={raw[0]}")
                return
            if comm_type == COMM_FAULT:
                report = decode_private_fault_payload(frame.data)
                now = time.monotonic()
                fault = {
                    "motorId": source_motor,
                    "motorIdHex": fmt_id(source_motor),
                    "faultRaw": report.fault_raw,
                    "faultRawHex": f"0x{report.fault_raw:08X}",
                    "warningRaw": report.warning_raw,
                    "warningRawHex": f"0x{report.warning_raw:08X}",
                    "faults": report.fault_names,
                    "warnings": report.warning_names,
                    "ageMs": 0,
                }
                self.last_private_fault_at = now
                self.last_private_fault = dict(fault)
                self.private_faults_at_by_motor[source_motor] = now
                self.private_faults_by_motor[source_motor] = fault
                self.log(
                    f"Private fault frame motor={fmt_id(source_motor)} "
                    f"{private_fault_summary(report)}"
                )
            return

    def frame_json(self, frame: Any) -> Dict[str, Any]:
        return {
            "kind": "ext" if frame.extended else "std",
            "id": frame.arbitration_id,
            "idHex": fmt_id(frame.arbitration_id, 8 if frame.extended else 3),
            "dlc": len(frame.data),
            "dataHex": bytes_hex(frame.data),
        }

    def current_seq(self) -> int:
        with self.lock:
            return self.frame_seq

    def wait_for_frame(
        self,
        predicate: Callable[[Any], bool],
        timeout_s: float,
        after_seq: Optional[int] = None,
    ) -> Optional[Any]:
        deadline = time.monotonic() + timeout_s
        with self.frame_cv:
            start_seq = self.frame_seq if after_seq is None else after_seq
            while True:
                for seq, frame in self.frames:
                    if seq > start_seq and predicate(frame):
                        return frame
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.frame_cv.wait(remaining)

    def wait_private_status_for(self, target_id: int, timeout_s: float, after_seq: Optional[int] = None) -> Optional[Any]:
        def matches(frame: Any) -> bool:
            if not frame.extended:
                return False
            comm_type, extra, _host = split_private_ext_id(frame.arbitration_id)
            return comm_type == COMM_OPERATION_STATUS and (extra & 0xFF) == (target_id & 0xFF)

        return self.wait_for_frame(matches, timeout_s, after_seq)

    def wait_private_status(self, timeout_s: float, after_seq: Optional[int] = None) -> Optional[Any]:
        return self.wait_private_status_for(self.motor_id, timeout_s, after_seq)

    def wait_private_get_id(
        self,
        target_id: int,
        host_id: int,
        timeout_s: float,
        after_seq: Optional[int] = None,
    ) -> Optional[Any]:
        start_seq = self.current_seq() if after_seq is None else after_seq
        self.send_private_get_id_with_host(target_id, host_id)

        def matches(frame: Any) -> bool:
            if not frame.extended or len(frame.data) < 8:
                return False
            comm_type, extra, _responder = split_private_ext_id(frame.arbitration_id)
            return comm_type == COMM_GET_ID and (extra & 0xFF) == (target_id & 0xFF)

        return self.wait_for_frame(matches, timeout_s, start_seq)

    def ping_private_candidates(self, target_id: int, timeout_s: float) -> Optional[Tuple[bytes, int, int]]:
        per_host_timeout = max(0.08, timeout_s / max(1, len(self.private_host_candidates())))
        for host in self.private_host_candidates():
            frame = self.wait_private_get_id(target_id, host, per_host_timeout)
            if frame is None:
                continue
            _comm_type, _extra, responder = split_private_ext_id(frame.arbitration_id)
            return bytes(frame.data[:8]).ljust(8, b"\x00"), host, responder
        return None

    def wait_private_save_ack(self, target_id: int, timeout_s: float, after_seq: int) -> bool:
        def matches(frame: Any) -> bool:
            if not frame.extended or len(frame.data) < 8:
                return False
            comm_type, extra, _responder = split_private_ext_id(frame.arbitration_id)
            source_motor = extra & 0xFF
            return source_motor == (target_id & 0xFF) and comm_type in (
                COMM_GET_ID,
                COMM_OPERATION_STATUS,
                COMM_READ_PARAM,
                COMM_WRITE_PARAM,
            )

        return self.wait_for_frame(matches, timeout_s, after_seq) is not None

    def private_status_position_rad(self, frame: Any, target_id: Optional[int] = None) -> float:
        if frame is None or not frame.extended or len(frame.data) < 2:
            raise ValueError("status frame did not include a motor position")
        if target_id is None:
            _comm_type, extra, _host = split_private_ext_id(frame.arbitration_id)
            target_id = extra & 0xFF
        model = self.model_for_motor(target_id)
        p_max, _v_max, _t_max = PRIVATE_MODEL_LIMITS.get(
            model,
            PRIVATE_MODEL_LIMITS[DEFAULT_MODEL],
        )
        pos_raw = (frame.data[0] << 8) | frame.data[1]
        return uint_to_float(pos_raw, -p_max, p_max, 16)

    def read_disabled_position_for(self, target_id: int, timeout_s: float) -> Optional[Tuple[float, int]]:
        original_host = self.host_id
        for host in self.private_host_candidates():
            with self.lock:
                self.host_id = host
            start_seq = self.current_seq()
            self.send_private_disable_to(target_id, False)
            frame = self.wait_private_status_for(target_id, timeout_s, start_seq)
            if frame is not None:
                return self.private_status_position_rad(frame, target_id), host
        with self.lock:
            self.host_id = original_host
        return None

    def wait_private_param_from(self, target_id: int, index: int, timeout_s: float) -> Optional[bytes]:
        start_seq = self.current_seq()
        self.read_private_param_from(target_id, index)

        def matches(frame: Any) -> bool:
            if not frame.extended or len(frame.data) < 8:
                return False
            comm_type, extra, _host = split_private_ext_id(frame.arbitration_id)
            return (
                comm_type == COMM_READ_PARAM
                and (extra & 0xFF) == (target_id & 0xFF)
                and int.from_bytes(frame.data[0:2], "little") == index
            )

        frame = self.wait_for_frame(matches, timeout_s, start_seq)
        return None if frame is None else frame.data[4:8]

    def wait_private_param(self, index: int, timeout_s: float) -> Optional[bytes]:
        return self.wait_private_param_from(self.motor_id, index, timeout_s)

    def run_command(self, command: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        if not self.command_lock.acquire(blocking=False):
            if command in ("stop", "clear-fault", "arm-stop", "arm-clear-fault", "shutdown-host"):
                try:
                    if command == "stop":
                        self.stop_and_disable()
                        message = "Stop sent while another command was running."
                    elif command == "clear-fault":
                        self.clear_fault()
                        message = "Clear fault sent while another command was running."
                    elif command == "arm-stop":
                        self.stop_arm()
                        message = "Arm stop sent while another command was running."
                    elif command == "arm-clear-fault":
                        self.clear_arm_faults()
                        message = "Arm clear fault sent while another command was running."
                    else:
                        return self.shutdown_host()
                    return {"ok": True, "message": message}
                except Exception as exc:
                    return {"ok": False, "message": str(exc)}
            return {"ok": False, "message": "Another command is already running."}
        payload = payload or {}
        with self.lock:
            self.busy = True
        try:
            return self._run_command(command, payload)
        except Exception as exc:
            self.log(f"{command} failed: {exc}")
            return {"ok": False, "message": str(exc)}
        finally:
            with self.lock:
                self.busy = False
            self.command_lock.release()

    def _run_command(self, command: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        if command != "arm-live-move":
            self.log(f"Command {command}")
        if command == "reopen":
            ok = self.reopen_bus()
            return {"ok": ok}
        if command == "shutdown-host":
            return self.shutdown_host()
        if not self.connected:
            opened = self.open_bus()
            if not opened:
                return {"ok": False, "message": self.open_error or "CAN is not open"}

        if command == "scan":
            motors = self.scan_private()
            return {"ok": True, "motors": [fmt_id(item) for item in motors]}
        if command == "scan-private":
            motors = self.scan_private()
            return {"ok": True, "motors": [fmt_id(item) for item in motors]}
        if command == "id-scan":
            motors = self.scan_private()
            return {"ok": True, "motors": [fmt_id(item) for item in motors]}
        if command == "assign-motor-id":
            return self.assign_motor_id(payload)
        if command == "configure":
            return {"ok": self.configure_velocity()}
        if command == "forward":
            self.oscillating = False
            self.jog_active = False
            return {"ok": self.set_speed(abs(self.test_speed))}
        if command == "backward":
            self.oscillating = False
            self.jog_active = False
            return {"ok": self.set_speed(-abs(self.test_speed))}
        if command == "jog-left":
            return {"ok": self.start_jog(-1)}
        if command == "jog-right":
            return {"ok": self.start_jog(1)}
        if command == "move-position":
            position = parse_float(payload.get("positionRad"), self.position_target)
            velocity_limit = positive_float(payload.get("velocityLimit"), self.position_velocity_limit, 20.0)
            acceleration = positive_float(payload.get("acceleration"), self.position_acceleration, 200.0)
            position_kp = nonnegative_float(payload.get("positionKp"), self.position_kp, 200.0)
            return {"ok": self.move_position(position, velocity_limit, acceleration, position_kp)}
        if command == "arm-move":
            return {"ok": self.move_arm_ik(payload)}
        if command == "arm-live-move":
            return {"ok": self.move_arm_ik(payload, live=True)}
        if command == "arm-preset":
            return self.move_arm_preset(payload)
        if command == "arm-home-zero":
            return self.home_arm_zero(payload)
        if command == "arm-stop":
            self.stop_arm()
            return {"ok": True}
        if command == "arm-clear-fault":
            self.clear_arm_faults()
            return {"ok": True}
        if command == "zero-speed":
            self.oscillating = False
            self.jog_active = False
            return {"ok": self.set_speed(0.0)}
        if command == "stop":
            self.stop_and_disable()
            return {"ok": True}
        if command == "clear-fault":
            self.clear_fault()
            return {"ok": True}
        if command == "toggle-oscillation":
            return {"ok": self.toggle_oscillation()}
        if command == "active-report":
            enabled = bool(payload.get("enabled", not self.active_reports))
            self.set_active_report(enabled)
            return {"ok": True}
        if command == "status":
            self.request_status()
            return {"ok": True}
        if command == "set-speed":
            speed = parse_float(payload.get("speed"), self.test_speed)
            with self.lock:
                self.test_speed = max(0.0, min(3.0, abs(speed)))
            self.log(f"test speed={self.test_speed:.2f} rad/s")
            return {"ok": True}
        if command == "raw":
            self.log_last_frames()
            return {"ok": True}
        return {"ok": False, "message": f"unknown command {command}"}

    def configure(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self.command_lock:
            with self.lock:
                old_serial_port = getattr(self.bus, "serial_port", DEFAULT_SERIAL_PORT)
                old_serial_baud = getattr(self.bus, "serial_baud", DEFAULT_SERIAL_BAUD)
                old_motor_id = self.motor_id
                old_host_id = self.host_id
                old_model = self.model
                new_serial_port = str(payload.get("serialPort") or old_serial_port)
                new_serial_baud = parse_int(payload.get("serialBaud"), old_serial_baud)
                new_motor_id = parse_int(payload.get("motorId"), old_motor_id) & 0xFF
                new_host_id = parse_int(payload.get("hostId"), old_host_id) & 0xFF
                new_model = str(payload.get("model") or old_model).lower()
                bus_changed = (
                    new_serial_port != old_serial_port
                    or new_serial_baud != old_serial_baud
                )
                control_changed = (
                    new_motor_id != old_motor_id
                    or new_host_id != old_host_id
                    or new_model != old_model
                )
                if bus_changed or control_changed:
                    self.motor_id = new_motor_id
                    self.host_id = new_host_id
                    self.model = new_model
                    self.velocity_configured = False
                    self.position_configured = False
                    self.arm_position_configured = False
                    self.clear_arm_route()
                    self.oscillating = False
                    self.jog_active = False
            if bus_changed:
                self.reopen_bus(
                    serial_port=new_serial_port,
                    serial_baud=new_serial_baud,
                )
            if bus_changed or control_changed:
                self.log(
                    f"Config transport={self.bus_transport()} adapter={self.bus_label()} "
                    f"motor={fmt_id(self.motor_id)} host={fmt_id(self.host_id)}"
                )
        return {"ok": True}

    def values_snapshot(self) -> Dict[str, Any]:
        with self.lock:
            serial_port = getattr(self.bus, "serial_port", DEFAULT_SERIAL_PORT)
            serial_baud = getattr(self.bus, "serial_baud", DEFAULT_SERIAL_BAUD)
            return {
                "schemaVersion": 1,
                "appVersion": APP_VERSION,
                "serialPort": serial_port,
                "serialBaud": serial_baud,
                "motorId": fmt_id(self.motor_id),
                "hostId": fmt_id(self.host_id),
                "model": self.model,
                "testSpeed": self.test_speed,
                "position": {
                    "target": self.position_target,
                    "velocityLimit": self.position_velocity_limit,
                    "acceleration": self.position_acceleration,
                    "positionKp": self.position_kp,
                },
                "arm": {
                    "jointCount": self.arm_joint_count,
                    "motorIds": {
                        axis: fmt_id(self.arm_motor_ids[axis])
                        for axis in ARM_AXES
                    },
                    "models": dict(self.arm_motor_models),
                    "link1": self.arm_link_1,
                    "link2": self.arm_link_2,
                    "radii": dict(self.arm_link_radii),
                    "twistLimits": dict(self.arm_twist_limits),
                    "elbowUp": self.arm_elbow_up,
                    "target": dict(self.arm_target),
                    "velocityLimit": self.arm_velocity_limit,
                    "acceleration": self.arm_acceleration,
                    "positionKp": self.arm_position_kp,
                    "dampingKd": self.arm_damping_kd,
                    "currentLimit": self.arm_current_limit,
                    "torqueBiases": dict(self.arm_torque_biases),
                    "offsets": dict(self.arm_offsets),
                    "directions": dict(self.arm_directions),
                },
            }

    def save_values(self) -> None:
        payload = self.values_snapshot()
        VALUES_PATH.parent.mkdir(parents=True, exist_ok=True)
        temp_path = VALUES_PATH.with_name(f"{VALUES_PATH.name}.tmp")
        temp_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temp_path.replace(VALUES_PATH)

    def arm_payload_from_values(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        arm = payload.get("arm")
        if not isinstance(arm, dict):
            arm = {}
        motor_ids = arm.get("motorIds")
        if not isinstance(motor_ids, dict):
            motor_ids = arm.get("motorIdHex")
        if not isinstance(motor_ids, dict):
            motor_ids = {}
        models = arm.get("models")
        if not isinstance(models, dict):
            models = {}
        offsets = arm.get("offsets")
        if not isinstance(offsets, dict):
            offsets = {}
        radii = arm.get("radii")
        if not isinstance(radii, dict):
            radii = {}
        twist_limits = arm.get("twistLimits")
        if not isinstance(twist_limits, dict):
            twist_limits = {}
        directions = arm.get("directions")
        if not isinstance(directions, dict):
            directions = {}
        torque_biases = arm.get("torqueBiases")
        if not isinstance(torque_biases, dict):
            torque_biases = {}
        target = arm.get("target")
        if not isinstance(target, dict):
            target = {}
        return {
            "armJointCount": payload.get("armJointCount", arm.get("jointCount", self.arm_joint_count)),
            "armBaseMotorId": payload.get(
                "armBaseMotorId",
                motor_ids.get("base", self.arm_motor_ids["base"]),
            ),
            "armShoulderMotorId": payload.get(
                "armShoulderMotorId",
                motor_ids.get("shoulder", self.arm_motor_ids["shoulder"]),
            ),
            "armElbowMotorId": payload.get(
                "armElbowMotorId",
                motor_ids.get("elbow", self.arm_motor_ids["elbow"]),
            ),
            "armBaseModel": payload.get(
                "armBaseModel",
                models.get("base", self.arm_motor_models["base"]),
            ),
            "armShoulderModel": payload.get(
                "armShoulderModel",
                models.get("shoulder", self.arm_motor_models["shoulder"]),
            ),
            "armElbowModel": payload.get(
                "armElbowModel",
                models.get("elbow", self.arm_motor_models["elbow"]),
            ),
            "armLink1": payload.get("armLink1", arm.get("link1", self.arm_link_1)),
            "armLink2": payload.get("armLink2", arm.get("link2", self.arm_link_2)),
            "armLink1Radius": payload.get(
                "armLink1Radius",
                radii.get("link1", self.arm_link_radii["link1"]),
            ),
            "armLink2Radius": payload.get(
                "armLink2Radius",
                radii.get("link2", self.arm_link_radii["link2"]),
            ),
            "armBaseTwistLimit": payload.get(
                "armBaseTwistLimit",
                twist_limits.get("base", self.arm_twist_limits["base"]),
            ),
            "armShoulderTwistLimit": payload.get(
                "armShoulderTwistLimit",
                twist_limits.get("shoulder", self.arm_twist_limits["shoulder"]),
            ),
            "armElbowTwistLimit": payload.get(
                "armElbowTwistLimit",
                twist_limits.get("elbow", self.arm_twist_limits["elbow"]),
            ),
            "armElbowUp": payload.get("armElbowUp", arm.get("elbowUp", self.arm_elbow_up)),
            "armTargetX": payload.get("armTargetX", target.get("x", self.arm_target["x"])),
            "armTargetY": payload.get("armTargetY", target.get("y", self.arm_target["y"])),
            "armTargetZ": payload.get("armTargetZ", target.get("z", self.arm_target["z"])),
            "armVelocityLimit": payload.get(
                "armVelocityLimit",
                arm.get("velocityLimit", self.arm_velocity_limit),
            ),
            "armAcceleration": payload.get(
                "armAcceleration",
                arm.get("acceleration", self.arm_acceleration),
            ),
            "armPositionKp": payload.get(
                "armPositionKp",
                arm.get("positionKp", self.arm_position_kp),
            ),
            "armDampingKd": payload.get(
                "armDampingKd",
                arm.get("dampingKd", self.arm_damping_kd),
            ),
            "armCurrentLimit": payload.get(
                "armCurrentLimit",
                arm.get("currentLimit", self.arm_current_limit),
            ),
            "armBaseTorqueBias": payload.get(
                "armBaseTorqueBias",
                torque_biases.get("base", self.arm_torque_biases["base"]),
            ),
            "armShoulderTorqueBias": payload.get(
                "armShoulderTorqueBias",
                torque_biases.get("shoulder", self.arm_torque_biases["shoulder"]),
            ),
            "armElbowTorqueBias": payload.get(
                "armElbowTorqueBias",
                torque_biases.get("elbow", self.arm_torque_biases["elbow"]),
            ),
            "armBaseOffset": payload.get(
                "armBaseOffset",
                offsets.get("base", self.arm_offsets["base"]),
            ),
            "armBaseDirection": payload.get(
                "armBaseDirection",
                directions.get("base", self.arm_directions["base"]),
            ),
            "armShoulderOffset": payload.get(
                "armShoulderOffset",
                offsets.get("shoulder", self.arm_offsets["shoulder"]),
            ),
            "armShoulderDirection": payload.get(
                "armShoulderDirection",
                directions.get("shoulder", self.arm_directions["shoulder"]),
            ),
            "armElbowOffset": payload.get(
                "armElbowOffset",
                offsets.get("elbow", self.arm_offsets["elbow"]),
            ),
            "armElbowDirection": payload.get(
                "armElbowDirection",
                directions.get("elbow", self.arm_directions["elbow"]),
            ),
        }

    def apply_values_payload(self, payload: Dict[str, Any]) -> Tuple[bool, str, int, bool]:
        if not isinstance(payload, dict):
            raise ValueError("values payload must be a JSON object")
        position = payload.get("position")
        if not isinstance(position, dict):
            position = {}
        with self.lock:
            old_serial_port = getattr(self.bus, "serial_port", DEFAULT_SERIAL_PORT)
            old_serial_baud = getattr(self.bus, "serial_baud", DEFAULT_SERIAL_BAUD)
            old_motor_id = self.motor_id
            old_host_id = self.host_id
            old_model = self.model
            old_arm_joint_count = self.arm_joint_count
            old_arm_motor_models = dict(self.arm_motor_models)
            new_serial_port = str(payload.get("serialPort") or old_serial_port)
            new_serial_baud = parse_int(payload.get("serialBaud"), old_serial_baud)
            new_motor_id = parse_int(payload.get("motorId"), old_motor_id) & 0xFF
            new_host_id = parse_int(payload.get("hostId"), old_host_id) & 0xFF
            new_model = str(payload.get("model") or old_model).lower()
            self.motor_id = new_motor_id
            self.host_id = new_host_id
            self.model = new_model
            self.test_speed = max(
                0.0,
                min(3.0, abs(parse_float(payload.get("testSpeed"), self.test_speed))),
            )
            self.position_target = parse_float(
                position.get("target", payload.get("positionTarget")),
                self.position_target,
            )
            self.position_velocity_limit = positive_float(
                position.get("velocityLimit", payload.get("positionVelocityLimit")),
                self.position_velocity_limit,
                20.0,
            )
            self.position_acceleration = positive_float(
                position.get("acceleration", payload.get("positionAcceleration")),
                self.position_acceleration,
                200.0,
            )
            self.position_kp = nonnegative_float(
                position.get("positionKp", payload.get("positionKp")),
                self.position_kp,
                200.0,
            )
            self.apply_arm_payload(self.arm_payload_from_values(payload))
            bus_changed = (
                new_serial_port != old_serial_port
                or new_serial_baud != old_serial_baud
            )
            control_changed = (
                new_motor_id != old_motor_id
                or new_host_id != old_host_id
                or new_model != old_model
                or self.arm_joint_count != old_arm_joint_count
                or self.arm_motor_models != old_arm_motor_models
            )
            if bus_changed or control_changed:
                self.velocity_configured = False
                self.position_configured = False
                self.arm_position_configured = False
                self.clear_arm_route()
                self.oscillating = False
                self.jog_active = False
                self.commanded_speed = 0.0
        return bus_changed, new_serial_port, new_serial_baud, control_changed

    def apply_values(self, payload: Dict[str, Any], persist: bool = True) -> Dict[str, Any]:
        try:
            with self.command_lock:
                bus_changed, serial_port, serial_baud, control_changed = self.apply_values_payload(payload)
                reopened: Optional[bool] = None
                if bus_changed:
                    reopened = self.reopen_bus(
                        serial_port=serial_port,
                        serial_baud=serial_baud,
                    )
                if persist:
                    self.save_values()
        except (OSError, TypeError, ValueError) as exc:
            self.log(f"Values save failed: {exc}")
            return {"ok": False, "message": str(exc)}

        if persist:
            self.log(f"Values saved to {VALUES_PATH}")
        elif bus_changed or control_changed:
            self.log("Dashboard values applied")
        result: Dict[str, Any] = {
            "ok": True,
            "path": str(VALUES_PATH),
        }
        if reopened is not None:
            result["reopened"] = reopened
            if not reopened:
                result["message"] = self.open_error
        return result

    def load_saved_values(self) -> None:
        try:
            payload = json.loads(VALUES_PATH.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return
        except (OSError, json.JSONDecodeError) as exc:
            self.log(f"Saved values skipped: {exc}")
            return
        try:
            bus_changed, serial_port, serial_baud, _control_changed = self.apply_values_payload(payload)
            if bus_changed:
                with self.bus_lock:
                    self.bus.close()
                    self.bus = create_bus(serial_port=serial_port, serial_baud=serial_baud)
            self.log(f"Loaded saved values from {VALUES_PATH}")
        except (TypeError, ValueError) as exc:
            self.log(f"Saved values skipped: {exc}")

    def configure_velocity(self) -> bool:
        return self.configure_private_velocity()

    def clear_private_fault(self, label: str = "Private clear-error sent") -> None:
        self.send_private_disable(True)
        self.wait_private_status(0.30)
        with self.lock:
            self.clear_cached_private_faults_locked([self.motor_id])
        self.log(label)

    def prepare_private_mode_switch_for(self, target_id: int, label: str) -> None:
        # Some RobStride firmware ignores run_mode writes unless torque is
        # explicitly disabled first. Clear-error alone can still leave the
        # controller reporting the old run_mode.
        self.send_private_disable_to(target_id, False)
        self.wait_private_status_for(target_id, 0.30)
        time.sleep(0.08)
        self.send_private_disable_to(target_id, True)
        self.wait_private_status_for(target_id, 0.30)
        with self.lock:
            self.clear_cached_private_faults_locked([target_id])
        self.log(label)
        time.sleep(0.08)

    def prepare_private_mode_switch(self, label: str) -> None:
        self.prepare_private_mode_switch_for(self.motor_id, label)

    def clear_fault(self) -> None:
        self.oscillating = False
        self.jog_active = False
        self.velocity_configured = False
        self.position_configured = False
        self.arm_position_configured = False
        self.clear_arm_route()
        self.clear_private_fault()

    def configure_private_velocity(self) -> bool:
        self.velocity_configured = False
        self.position_configured = False
        self.arm_position_configured = False
        self.clear_arm_route()
        self.oscillating = False
        self.jog_active = False
        self.log(
            f"Configuring private velocity motor={fmt_id(self.motor_id)} "
            f"host={fmt_id(self.host_id)}"
        )
        self.prepare_private_mode_switch("Private disabled/clear-error before velocity configure")

        if not self.write_private_run_mode_verified(RUN_MODE_VELOCITY):
            self.log("Private velocity setup failed: run_mode did not verify")
            return False

        self.send_private_enable()
        self.wait_private_status(0.30)
        self.commanded_speed = 0.0
        self.write_private_param_f32(PARAM_SPD_REF, 0.0)
        self.write_private_param_f32(PARAM_LIMIT_CUR, DEFAULT_CURRENT_LIMIT_A)
        self.write_private_param_f32(PARAM_ACC_RAD, DEFAULT_ACCEL_RAD_S2)
        self.velocity_configured = True
        self.position_configured = False
        self.log("Private velocity mode configured")
        return True

    def write_private_run_mode_verified_for(self, target_id: int, desired_mode: int) -> bool:
        original_host = self.host_id
        for host in self.private_host_candidates():
            self.host_id = host
            for attempt in range(1, 4):
                self.write_private_param_u8_to(target_id, PARAM_RUN_MODE, desired_mode)
                time.sleep(0.03)
                raw = self.wait_private_param_from(target_id, PARAM_RUN_MODE, COMMAND_TIMEOUT_S)
                if raw is None:
                    self.log(f"motor={fmt_id(target_id)} host={fmt_id(host)} attempt={attempt} run_mode timeout")
                    continue
                self.log(f"motor={fmt_id(target_id)} host={fmt_id(host)} attempt={attempt} run_mode readback={raw[0]}")
                if raw[0] == desired_mode:
                    return True
        self.host_id = original_host
        return False

    def write_private_run_mode_verified(self, desired_mode: int) -> bool:
        return self.write_private_run_mode_verified_for(self.motor_id, desired_mode)

    def move_position(self, position: float, velocity_limit: float, acceleration: float, position_kp: float) -> bool:
        self.position_target = position
        self.position_velocity_limit = velocity_limit
        self.position_acceleration = acceleration
        self.position_kp = position_kp
        return self.move_private_position(position, velocity_limit, acceleration, position_kp)

    def move_private_position(self, position: float, velocity_limit: float, acceleration: float, position_kp: float) -> bool:
        self.velocity_configured = False
        self.position_configured = False
        self.arm_position_configured = False
        self.clear_arm_route()
        self.oscillating = False
        self.jog_active = False
        self.commanded_speed = 0.0
        kp = max(0.0, position_kp)
        self.log(
            f"Configuring private position motor={fmt_id(self.motor_id)} "
            f"host={fmt_id(self.host_id)} pos={position:+.3f} rad "
            f"vlim={velocity_limit:.2f} rad/s loc_kp={kp:.2f}"
        )

        self.prepare_private_mode_switch("Private disabled/clear-error before position configure")

        if not self.write_private_run_mode_verified(RUN_MODE_POSITION):
            self.log("Private position setup failed: run_mode did not verify")
            return False

        self.send_private_enable()
        self.wait_private_status(0.30)
        self.write_private_param_f32(PARAM_LIMIT_CUR, DEFAULT_CURRENT_LIMIT_A)
        self.write_private_param_f32(PARAM_PP_VEL_MAX, velocity_limit)
        self.write_private_param_f32(PARAM_PP_ACC_SET, acceleration)
        self.write_private_param_f32(PARAM_LOC_KP, kp)
        self.write_private_param_f32(PARAM_LOC_REF, position)
        self.position_kp = kp
        self.position_configured = True
        self.last_position_refresh_at = time.monotonic()
        self.log(
            f"private position target={position:+.3f} rad "
            f"vlim={velocity_limit:.2f} rad/s acc={acceleration:.1f} rad/s^2 loc_kp={kp:.2f} sent"
        )
        return True

    def send_private_operation_control_to(
        self,
        target_id: int,
        position: float,
        velocity: float,
        kp: float,
        kd: float,
        torque_ff: float = 0.0,
    ) -> None:
        model = self.model_for_motor(target_id)
        p_max, v_max, t_max = PRIVATE_MODEL_LIMITS.get(
            model,
            PRIVATE_MODEL_LIMITS[DEFAULT_MODEL],
        )
        payload = bytearray(8)
        payload[0:2] = float_to_uint(position, -p_max, p_max, 16).to_bytes(2, "big")
        payload[2:4] = float_to_uint(velocity, -v_max, v_max, 16).to_bytes(2, "big")
        payload[4:6] = float_to_uint(kp, 0.0, 500.0, 16).to_bytes(2, "big")
        payload[6:8] = float_to_uint(kd, 0.0, 5.0, 16).to_bytes(2, "big")
        torque = float_to_uint(torque_ff, -t_max, t_max, 16)
        self.send_private(COMM_OPERATION_CONTROL, torque, target_id & 0xFF, bytes(payload))

    def configure_private_operation_motor(
        self,
        axis: str,
        motor_id: int,
        position: float,
        current_limit: float,
    ) -> bool:
        motor_id &= 0xFF
        self.prepare_private_mode_switch_for(
            motor_id,
            f"{axis} {fmt_id(motor_id)} disabled/clear-error before damped arm configure",
        )
        if not self.write_private_run_mode_verified_for(motor_id, RUN_MODE_OPERATION):
            self.log(f"{axis} {fmt_id(motor_id)} damped arm setup failed: run_mode did not verify")
            return False
        torque_ff = self.arm_effective_torque_bias(axis, position)
        self.send_private_enable_to(motor_id)
        self.send_private_operation_control_to(
            motor_id,
            position,
            0.0,
            self.arm_position_kp,
            self.arm_damping_kd,
            torque_ff,
        )
        self.wait_private_status_for(motor_id, 0.30)
        self.write_private_param_f32_to(motor_id, PARAM_LIMIT_CUR, current_limit)
        self.send_private_operation_control_to(
            motor_id,
            position,
            0.0,
            self.arm_position_kp,
            self.arm_damping_kd,
            torque_ff,
        )
        self.log(
            f"{axis} {fmt_id(motor_id)} operation hold pos={position:+.3f} "
            f"kp={self.arm_position_kp:.2f} kd={self.arm_damping_kd:.2f} "
            f"assist={torque_ff:+.2f}Nm current={current_limit:.2f}A"
        )
        return True

    def apply_arm_payload(self, payload: Dict[str, Any]) -> None:
        self.arm_joint_count = arm_joint_count(payload.get("armJointCount"), self.arm_joint_count)
        self.arm_motor_ids = {
            "base": parse_int(payload.get("armBaseMotorId"), self.arm_motor_ids["base"]) & 0xFF,
            "shoulder": parse_int(payload.get("armShoulderMotorId"), self.arm_motor_ids["shoulder"]) & 0xFF,
            "elbow": parse_int(payload.get("armElbowMotorId"), self.arm_motor_ids["elbow"]) & 0xFF,
        }
        self.arm_motor_models = {
            "base": model_name(payload.get("armBaseModel"), self.arm_motor_models["base"]),
            "shoulder": model_name(payload.get("armShoulderModel"), self.arm_motor_models["shoulder"]),
            "elbow": model_name(payload.get("armElbowModel"), self.arm_motor_models["elbow"]),
        }
        self.arm_offsets = {
            "base": parse_float(payload.get("armBaseOffset"), self.arm_offsets["base"]),
            "shoulder": parse_float(payload.get("armShoulderOffset"), self.arm_offsets["shoulder"]),
            "elbow": parse_float(payload.get("armElbowOffset"), self.arm_offsets["elbow"]),
        }
        self.arm_directions = {
            "base": signed_direction(payload.get("armBaseDirection"), self.arm_directions["base"]),
            "shoulder": signed_direction(payload.get("armShoulderDirection"), self.arm_directions["shoulder"]),
            "elbow": signed_direction(payload.get("armElbowDirection"), self.arm_directions["elbow"]),
        }
        self.arm_link_1 = positive_float(payload.get("armLink1"), self.arm_link_1, 10.0)
        self.arm_link_2 = positive_float(payload.get("armLink2"), self.arm_link_2, 10.0)
        self.arm_link_radii = {
            "link1": nonnegative_float(payload.get("armLink1Radius"), self.arm_link_radii["link1"], 2.0),
            "link2": nonnegative_float(payload.get("armLink2Radius"), self.arm_link_radii["link2"], 2.0),
        }
        self.arm_twist_limits = {
            "base": twist_limit_rad(payload.get("armBaseTwistLimit"), self.arm_twist_limits["base"]),
            "shoulder": twist_limit_rad(payload.get("armShoulderTwistLimit"), self.arm_twist_limits["shoulder"]),
            "elbow": twist_limit_rad(payload.get("armElbowTwistLimit"), self.arm_twist_limits["elbow"]),
        }
        self.arm_target = clamp_arm_target_to_reach(
            self.arm_joint_count,
            {
                "x": parse_float(payload.get("armTargetX"), self.arm_target["x"]),
                "y": parse_float(payload.get("armTargetY"), self.arm_target["y"]),
                "z": parse_float(payload.get("armTargetZ"), self.arm_target["z"]),
            },
            self.arm_link_1,
            self.arm_link_2,
        )
        self.arm_elbow_up = bool(payload.get("armElbowUp", self.arm_elbow_up))
        self.arm_velocity_limit = positive_float(
            payload.get("armVelocityLimit"),
            self.arm_velocity_limit,
            ARM_POSITION_VEL_MAX_RAD_S,
        )
        self.arm_acceleration = positive_float(
            payload.get("armAcceleration"),
            self.arm_acceleration,
            ARM_POSITION_ACCEL_MAX_RAD_S2,
        )
        self.arm_position_kp = nonnegative_float(
            payload.get("armPositionKp"),
            self.arm_position_kp,
            ARM_POSITION_KP_MAX,
        )
        self.arm_damping_kd = nonnegative_float(
            payload.get("armDampingKd"),
            self.arm_damping_kd,
            ARM_DAMPING_KD_MAX,
        )
        self.arm_current_limit = positive_float(
            payload.get("armCurrentLimit"),
            self.arm_current_limit,
            ARM_CURRENT_LIMIT_MAX_A,
        )
        self.arm_torque_biases = {
            "base": limited_float(
                payload.get("armBaseTorqueBias"),
                self.arm_torque_biases["base"],
                ARM_TORQUE_BIAS_MAX_NM,
            ),
            "shoulder": limited_float(
                payload.get("armShoulderTorqueBias"),
                self.arm_torque_biases["shoulder"],
                ARM_TORQUE_BIAS_MAX_NM,
            ),
            "elbow": limited_float(
                payload.get("armElbowTorqueBias"),
                self.arm_torque_biases["elbow"],
                ARM_TORQUE_BIAS_MAX_NM,
            ),
        }

    def arm_motor_target(self, axis: str, joint_angle: float) -> float:
        return self.arm_offsets[axis] + (self.arm_directions[axis] * joint_angle)

    def arm_motor_targets_for_joints(self, joint_angles: Dict[str, float]) -> Dict[str, float]:
        return {
            axis: self.arm_motor_target(axis, joint_angles[axis])
            for axis in self.active_arm_axes()
        }

    def arm_feedback_joint_angles(self) -> Optional[Dict[str, float]]:
        now = time.monotonic()
        with self.lock:
            angles = {axis: self.arm_joint_angles.get(axis, 0.0) for axis in ARM_AXES}
            for axis in self.active_arm_axes():
                motor_id = self.arm_motor_ids[axis] & 0xFF
                feedback = self.feedback_by_motor.get(motor_id)
                seen_at = self.feedback_at_by_motor.get(motor_id)
                if feedback is None or seen_at is None or now - seen_at > ARM_FEEDBACK_START_MAX_AGE_S:
                    return None
                position = feedback.get("positionRad")
                if not isinstance(position, (int, float)) or not math.isfinite(float(position)):
                    return None
                direction = self.arm_directions[axis]
                angles[axis] = (float(position) - self.arm_offsets[axis]) / direction
        return angles

    def arm_effective_torque_bias(self, axis: str, target_position: float) -> float:
        bias = self.arm_torque_biases.get(axis, 0.0)
        if abs(bias) <= 0.000001:
            return 0.0

        motor_id = self.arm_motor_ids[axis] & 0xFF
        now = time.monotonic()
        with self.lock:
            feedback = self.feedback_by_motor.get(motor_id)
            seen_at = self.feedback_at_by_motor.get(motor_id)
            if feedback is None or seen_at is None or now - seen_at > ARM_FEEDBACK_START_MAX_AGE_S:
                return bias * ARM_ASSIST_FEEDBACK_MISSING_SCALE
            position = feedback.get("positionRad")
            if not isinstance(position, (int, float)) or not math.isfinite(float(position)):
                return bias * ARM_ASSIST_FEEDBACK_MISSING_SCALE

        assist_direction = 1.0 if bias > 0.0 else -1.0
        lag_toward_target = (target_position - float(position)) * assist_direction
        if lag_toward_target >= ARM_ASSIST_FADE_BAND_RAD:
            scale = 1.0
        elif lag_toward_target >= 0.0:
            fade = lag_toward_target / ARM_ASSIST_FADE_BAND_RAD
            scale = ARM_ASSIST_TARGET_SCALE + ((1.0 - ARM_ASSIST_TARGET_SCALE) * fade)
        else:
            overshoot = -lag_toward_target
            if overshoot >= ARM_ASSIST_OVERSHOOT_FADE_BAND_RAD:
                scale = ARM_ASSIST_OVERSHOOT_SCALE
            else:
                fade = overshoot / ARM_ASSIST_OVERSHOOT_FADE_BAND_RAD
                scale = ARM_ASSIST_TARGET_SCALE - (
                    (ARM_ASSIST_TARGET_SCALE - ARM_ASSIST_OVERSHOOT_SCALE) * fade
                )
        return bias * scale

    def clear_arm_route(self) -> None:
        self.arm_route_waypoints.clear()
        self.arm_route_next_at = 0.0

    def arm_route_interval(self, previous: Dict[str, float], next_angles: Dict[str, float]) -> float:
        max_delta = max(
            (
                abs(float(next_angles.get(axis, 0.0)) - float(previous.get(axis, 0.0)))
                for axis in self.active_arm_axes()
            ),
            default=0.0,
        )
        velocity = max(abs(self.arm_velocity_limit), 0.05)
        return max(ARM_ROUTE_MIN_INTERVAL_S, (max_delta / velocity) + ARM_ROUTE_SETTLE_S)

    def apply_arm_route_waypoint(self, waypoint: Dict[str, Dict[str, float]]) -> None:
        self.arm_joint_angles = dict(waypoint["jointAngles"])
        self.arm_motor_targets = {
            axis: waypoint["motorTargets"].get(axis, self.arm_motor_targets.get(axis, 0.0))
            for axis in ARM_AXES
        }

    def arm_route_waypoints_to_target(
        self,
        target: Dict[str, float],
        previous_angles: Dict[str, float],
        max_step_rad: float,
    ) -> Tuple[List[Dict[str, Dict[str, float]]], Dict[str, float], Dict[str, float]]:
        clamped_target = clamp_arm_target_to_reach(
            self.arm_joint_count,
            target,
            self.arm_link_1,
            self.arm_link_2,
        )
        solution = solve_arm_ik(
            self.arm_joint_count,
            clamped_target["x"],
            clamped_target["y"],
            clamped_target["z"],
            self.arm_link_1,
            self.arm_link_2,
            self.arm_elbow_up,
        )
        raw_joint_angles = {
            "base": solution.base,
            "shoulder": solution.shoulder,
            "elbow": solution.elbow,
        }
        joint_angles = route_arm_joint_angles(
            self.arm_joint_count,
            raw_joint_angles,
            self.arm_twist_limits,
            previous_angles,
        )
        route_angles = plan_arm_joint_route(
            self.arm_joint_count,
            previous_angles,
            joint_angles,
            max_step_rad,
        )
        route_waypoints: List[Dict[str, Dict[str, float]]] = []
        for waypoint_angles in route_angles:
            waypoint_solution = ArmIkSolution(
                base=waypoint_angles["base"],
                shoulder=waypoint_angles["shoulder"],
                elbow=waypoint_angles["elbow"],
            )
            waypoint_points = arm_solution_points(
                self.arm_joint_count,
                waypoint_solution,
                clamped_target,
                self.arm_link_1,
                self.arm_link_2,
            )
            waypoint_safety = arm_safety_check(
                self.arm_joint_count,
                waypoint_points,
                self.arm_link_radii,
                waypoint_angles,
                self.arm_twist_limits,
            )
            if not waypoint_safety["ok"]:
                raise ValueError(f"Unsafe IK route: {'; '.join(waypoint_safety['warnings'])}")
            route_waypoints.append(
                {
                    "jointAngles": dict(waypoint_angles),
                    "motorTargets": self.arm_motor_targets_for_joints(waypoint_angles),
                }
            )
        return route_waypoints, clamped_target, joint_angles

    def build_arm_route_for_targets(
        self,
        targets: List[Dict[str, float]],
        max_step_rad: float,
    ) -> Tuple[List[Dict[str, Dict[str, float]]], Dict[str, float], Dict[str, float]]:
        previous_angles = self.arm_feedback_joint_angles() or dict(self.arm_joint_angles)
        final_target = dict(self.arm_target)
        final_joint_angles = dict(previous_angles)
        route_waypoints: List[Dict[str, Dict[str, float]]] = []
        for target in targets:
            segment, final_target, final_joint_angles = self.arm_route_waypoints_to_target(
                target,
                previous_angles,
                max_step_rad,
            )
            route_waypoints.extend(segment)
            previous_angles = final_joint_angles
        if not route_waypoints:
            raise ValueError("Arm route has no waypoints")
        return route_waypoints, final_target, final_joint_angles

    def start_arm_route(
        self,
        route_waypoints: List[Dict[str, Dict[str, float]]],
        config_signature: Tuple[Any, ...],
        label: str,
        live: bool = False,
    ) -> bool:
        first_waypoint = route_waypoints[0]
        remaining_waypoints = route_waypoints[1:]
        needs_config = (
            not self.arm_position_configured
            or self.arm_position_signature != config_signature
        )
        self.oscillating = False
        self.jog_active = False
        self.velocity_configured = False
        self.position_configured = False
        self.clear_arm_route()
        self.commanded_speed = 0.0
        if needs_config:
            self.arm_position_configured = False
        should_log = not live or needs_config
        if should_log:
            joint_angles = route_waypoints[-1]["jointAngles"]
            self.log(
                f"{label} "
                f"joints base={joint_angles['base']:+.3f} shoulder={joint_angles['shoulder']:+.3f} elbow={joint_angles['elbow']:+.3f} "
                f"route_waypoints={len(route_waypoints)} kp={self.arm_position_kp:.2f} kd={self.arm_damping_kd:.2f} "
                f"assist shoulder={self.arm_torque_biases['shoulder']:+.2f}Nm elbow={self.arm_torque_biases['elbow']:+.2f}Nm"
            )
            if 0.0 < self.arm_position_kp < 2.0:
                self.log(
                    "Arm operation Kp is very soft for a loaded arm; "
                    "try Position Kp around 4.0 with Damping Kd around 2.0."
                )
        if needs_config:
            for axis in self.active_arm_axes():
                if not self.configure_private_operation_motor(
                    axis,
                    self.arm_motor_ids[axis],
                    first_waypoint["motorTargets"][axis],
                    self.arm_current_limit,
                ):
                    self.stop_arm()
                    return False
            self.arm_current_limit_signature = self.arm_current_limit_config_signature()
        else:
            self.refresh_arm_current_limits_if_needed()
        self.apply_arm_route_waypoint(first_waypoint)
        if not needs_config:
            for axis in self.active_arm_axes():
                self.send_private_operation_control_to(
                    self.arm_motor_ids[axis],
                    self.arm_motor_targets[axis],
                    0.0,
                    self.arm_position_kp,
                    self.arm_damping_kd,
                    self.arm_effective_torque_bias(axis, self.arm_motor_targets[axis]),
                )
        self.arm_route_waypoints = deque(remaining_waypoints)
        self.arm_route_next_at = (
            time.monotonic() + self.arm_route_interval(self.arm_joint_angles, remaining_waypoints[0]["jointAngles"])
            if remaining_waypoints
            else 0.0
        )
        self.arm_position_configured = True
        self.arm_position_signature = config_signature
        self.last_arm_position_refresh_at = time.monotonic()
        if should_log:
            active_targets = " ".join(
                f"{axis}={self.arm_motor_targets[axis]:+.3f}"
                for axis in self.active_arm_axes()
            )
            queued = len(self.arm_route_waypoints)
            self.log(f"Arm route started {active_targets}; queued={queued}")
        return True

    def validate_arm_command_motors(self, payload: Dict[str, Any]) -> Tuple[bool, str]:
        joint_count = arm_joint_count(payload.get("armJointCount"), self.arm_joint_count)
        axes = arm_axes_for_count(joint_count)
        keys = {
            "base": "armBaseMotorId",
            "shoulder": "armShoulderMotorId",
            "elbow": "armElbowMotorId",
        }
        parsed: Dict[str, int] = {}
        missing = []
        for axis in axes:
            value = payload.get(keys[axis])
            if value is None or value == "":
                missing.append(axis)
                continue
            try:
                parsed[axis] = parse_int(value, -1) & 0xFF
            except (TypeError, ValueError):
                return False, f"{axis} motor ID is invalid"
        if missing:
            return False, f"Select detected motor IDs for: {', '.join(missing)}"

        seen: Dict[int, str] = {}
        for axis, motor_id in parsed.items():
            if motor_id in seen:
                return False, f"{axis} and {seen[motor_id]} both use {fmt_id(motor_id)}"
            seen[motor_id] = axis
        return True, ""

    def arm_position_config_signature(self) -> Tuple[Any, ...]:
        axes = self.active_arm_axes()
        return (
            self.host_id & 0xFF,
            self.arm_joint_count,
            tuple(
                (
                    axis,
                    self.arm_motor_ids[axis] & 0xFF,
                )
                for axis in axes
            ),
        )

    def arm_current_limit_config_signature(self) -> Tuple[Any, ...]:
        axes = self.active_arm_axes()
        return (
            self.host_id & 0xFF,
            tuple((axis, self.arm_motor_ids[axis] & 0xFF) for axis in axes),
            round(self.arm_current_limit, 6),
        )

    def refresh_arm_current_limits_if_needed(self) -> None:
        signature = self.arm_current_limit_config_signature()
        if self.arm_current_limit_signature == signature:
            return
        for axis in self.active_arm_axes():
            self.write_private_param_f32_to(
                self.arm_motor_ids[axis],
                PARAM_LIMIT_CUR,
                self.arm_current_limit,
            )
        self.arm_current_limit_signature = signature
        self.log(f"Arm current limit updated to {self.arm_current_limit:.2f} A without mode reset")

    def home_arm_zero(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        ok, message = self.validate_arm_command_motors(payload)
        if not ok:
            self.log(message)
            return {"ok": False, "message": message}
        self.apply_arm_payload(payload)
        with self.lock:
            self.oscillating = False
            self.jog_active = False
            self.velocity_configured = False
            self.position_configured = False
            self.arm_position_configured = False
            self.clear_arm_route()
            self.commanded_speed = 0.0

        offsets: Dict[str, float] = dict(self.arm_offsets)
        hosts: Dict[str, int] = {}
        for axis in self.active_arm_axes():
            motor_id = self.arm_motor_ids[axis]
            result = self.read_disabled_position_for(motor_id, COMMAND_TIMEOUT_S)
            if result is None:
                message = f"{axis} {fmt_id(motor_id)} position timeout during home zero"
                self.log(message)
                return {"ok": False, "message": message}
            position, host = result
            offsets[axis] = position
            hosts[axis] = host
            self.log(f"{axis} {fmt_id(motor_id)} home position={position:+.4f} rad host={fmt_id(host)}")

        with self.lock:
            self.arm_offsets = offsets
            self.arm_target = {
                "x": self.arm_link_1 + self.arm_link_2,
                "y": 0.0,
                "z": 0.0,
            }
            self.arm_joint_angles = {axis: 0.0 for axis in ARM_AXES}
            self.arm_motor_targets = dict(offsets)

        self.save_values()
        self.log(f"Arm home zero saved to {VALUES_PATH}")
        return {
            "ok": True,
            "message": "Arm home zero saved",
            "offsets": dict(offsets),
            "hosts": {axis: fmt_id(hosts[axis]) for axis in self.active_arm_axes()},
            "path": str(VALUES_PATH),
        }

    def arm_motion_preset_targets(self, preset: str) -> List[Dict[str, float]]:
        key = str(preset or "showcase").strip().lower()
        if key not in ARM_MOTION_PRESET_LABELS:
            choices = ", ".join(ARM_MOTION_PRESET_LABELS)
            raise ValueError(f"Unknown arm movement preset '{preset}'. Choose one of: {choices}")

        reach = max(abs(self.arm_link_1) + abs(self.arm_link_2), 0.001)

        def target(radial_scale: float, yaw_deg: float, z_scale: float) -> Dict[str, float]:
            radial = reach * radial_scale
            yaw = math.radians(yaw_deg)
            return clamp_arm_target_to_reach(
                self.arm_joint_count,
                {
                    "x": radial * math.cos(yaw),
                    "y": radial * math.sin(yaw),
                    "z": reach * z_scale,
                },
                self.arm_link_1,
                self.arm_link_2,
            )

        if self.arm_joint_count == 2:
            presets = {
                "showcase": [
                    (0.68, -36, 0.12),
                    (0.60, -12, 0.38),
                    (0.74, 30, 0.18),
                    (0.56, 36, 0.44),
                    (0.80, 0, 0.12),
                    (0.58, -28, 0.30),
                    (0.72, 0, 0.18),
                ],
                "sweep": [
                    (0.70, -42, 0.14),
                    (0.76, -18, 0.24),
                    (0.72, 18, 0.20),
                    (0.66, 42, 0.32),
                    (0.74, 0, 0.18),
                ],
                "lift": [
                    (0.70, 0, 0.08),
                    (0.58, 0, 0.42),
                    (0.74, 0, 0.24),
                    (0.62, 0, 0.14),
                ],
                "orbit": [
                    (0.68, -30, 0.14),
                    (0.58, -8, 0.46),
                    (0.68, 30, 0.14),
                    (0.78, 8, 0.10),
                    (0.68, -30, 0.14),
                ],
                "flex": [
                    (0.82, 0, 0.10),
                    (0.48, 0, 0.38),
                    (0.72, -24, 0.22),
                    (0.50, 24, 0.36),
                    (0.78, 0, 0.16),
                ],
            }
        else:
            presets = {
                "showcase": [
                    (0.74, -34, 0.14),
                    (0.50, -12, 0.44),
                    (0.82, 22, 0.16),
                    (0.46, 34, 0.36),
                    (0.70, -28, 0.28),
                    (0.54, 0, 0.50),
                    (0.84, 0, 0.12),
                    (0.66, 0, 0.24),
                ],
                "sweep": [
                    (0.72, -42, 0.16),
                    (0.62, -16, 0.34),
                    (0.72, 20, 0.18),
                    (0.58, 42, 0.40),
                    (0.76, 0, 0.20),
                ],
                "lift": [
                    (0.76, 0, 0.10),
                    (0.54, 0, 0.48),
                    (0.84, 0, 0.18),
                    (0.56, 0, 0.32),
                    (0.72, 0, 0.16),
                ],
                "orbit": [
                    (0.70, -34, 0.16),
                    (0.54, -8, 0.46),
                    (0.70, 34, 0.16),
                    (0.84, 8, 0.12),
                    (0.58, -24, 0.34),
                    (0.70, -34, 0.16),
                ],
                "flex": [
                    (0.86, 0, 0.10),
                    (0.46, 0, 0.38),
                    (0.76, -24, 0.18),
                    (0.44, 22, 0.42),
                    (0.82, 0, 0.14),
                ],
            }
        return [target(*item) for item in presets[key]]

    def move_arm_ik(self, payload: Dict[str, Any], live: bool = False) -> bool:
        ok, message = self.validate_arm_command_motors(payload)
        if not ok:
            raise ValueError(message)
        self.apply_arm_payload(payload)
        config_signature = self.arm_position_config_signature()
        route_waypoints, final_target, _final_joint_angles = self.build_arm_route_for_targets(
            [dict(self.arm_target)],
            ARM_ROUTE_MAX_STEP_RAD,
        )
        self.arm_target = final_target
        label = (
            ("Arm live IK target " if live else "Arm IK target ")
            + f"x={self.arm_target['x']:+.3f} y={self.arm_target['y']:+.3f} z={self.arm_target['z']:+.3f}"
        )
        return self.start_arm_route(route_waypoints, config_signature, label, live=live)

    def move_arm_preset(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        ok, message = self.validate_arm_command_motors(payload)
        if not ok:
            return {"ok": False, "message": message}
        self.apply_arm_payload(payload)
        preset = str(payload.get("armMotionPreset", "showcase")).strip().lower()
        label = ARM_MOTION_PRESET_LABELS.get(preset, preset.title())
        targets = self.arm_motion_preset_targets(preset)
        route_waypoints, final_target, _final_joint_angles = self.build_arm_route_for_targets(
            targets,
            ARM_PRESET_MAX_STEP_RAD,
        )
        self.arm_target = final_target
        ok = self.start_arm_route(
            route_waypoints,
            self.arm_position_config_signature(),
            f"Arm preset {label} ({self.arm_joint_count}-joint)",
        )
        return {
            "ok": ok,
            "message": (
                f"Arm preset {label} started with {len(targets)} poses "
                f"and {len(route_waypoints)} route waypoints"
            ) if ok else f"Arm preset {label} failed to start",
        }

    def send_arm_position_targets(self) -> None:
        now = time.monotonic()
        if self.arm_route_waypoints and now >= self.arm_route_next_at:
            waypoint = self.arm_route_waypoints.popleft()
            self.apply_arm_route_waypoint(waypoint)
            if self.arm_route_waypoints:
                self.arm_route_next_at = now + self.arm_route_interval(
                    self.arm_joint_angles,
                    self.arm_route_waypoints[0]["jointAngles"],
                )
            else:
                self.arm_route_next_at = 0.0
                self.log("Arm route complete")
        for axis in self.active_arm_axes():
            self.send_private_operation_control_to(
                self.arm_motor_ids[axis],
                self.arm_motor_targets[axis],
                0.0,
                self.arm_position_kp,
                self.arm_damping_kd,
                self.arm_effective_torque_bias(axis, self.arm_motor_targets[axis]),
            )
        self.last_arm_position_refresh_at = time.monotonic()

    def stop_arm(self) -> None:
        self.velocity_configured = False
        self.position_configured = False
        self.arm_position_configured = False
        self.clear_arm_route()
        self.oscillating = False
        self.jog_active = False
        self.commanded_speed = 0.0
        for motor_id in sorted(set(self.arm_motor_ids[axis] for axis in self.active_arm_axes())):
            self.send_private_disable_to(motor_id, False)
            self.wait_private_status_for(motor_id, 0.20)
        self.log("Arm stop/disable sent")

    def clear_arm_faults(self) -> None:
        self.arm_position_configured = False
        self.velocity_configured = False
        self.position_configured = False
        self.clear_arm_route()
        self.oscillating = False
        self.jog_active = False
        self.commanded_speed = 0.0
        for motor_id in sorted(set(self.arm_motor_ids[axis] for axis in self.active_arm_axes())):
            self.send_private_disable_to(motor_id, True)
            self.wait_private_status_for(motor_id, 0.20)
        with self.lock:
            self.clear_cached_private_faults_locked([
                self.arm_motor_ids[axis]
                for axis in self.active_arm_axes()
            ])
        self.log("Arm clear-fault sent")

    def private_host_candidates(self) -> List[int]:
        out: List[int] = []
        for host in (self.host_id, *PRIVATE_HOST_CANDIDATES):
            host &= 0xFF
            if host not in out:
                out.append(host)
        return out

    def send_velocity_target(self, speed: float) -> None:
        self.write_private_param_f32(PARAM_SPD_REF, speed)
        self.last_velocity_refresh_at = time.monotonic()

    def send_position_target(self) -> None:
        self.write_private_param_f32(PARAM_LOC_REF, self.position_target)
        self.last_position_refresh_at = time.monotonic()

    def set_speed(self, speed: float) -> bool:
        if not self.velocity_configured and not self.configure_velocity():
            return False
        self.commanded_speed = speed
        self.send_velocity_target(speed)
        self.position_configured = False
        self.arm_position_configured = False
        self.clear_arm_route()
        self.log(f"private speed={speed:+.2f} rad/s sent")
        return True

    def start_jog(self, direction: int) -> bool:
        self.oscillating = False
        speed = MOTOR_STUDIO_JOG_SPEED_RAD_S if direction > 0 else -MOTOR_STUDIO_JOG_SPEED_RAD_S
        if not self.set_speed(speed):
            return False
        self.jog_active = True
        self.jog_stop_at = time.monotonic() + MOTOR_STUDIO_JOG_S
        self.log(f"Jog {'right' if direction > 0 else 'left'} for {int(MOTOR_STUDIO_JOG_S * 1000)} ms")
        return True

    def toggle_oscillation(self) -> bool:
        self.jog_active = False
        if self.oscillating:
            self.oscillating = False
            self.log("Oscillation off")
            return True
        if not self.set_speed(self.test_speed):
            return False
        self.oscillating = True
        self.last_oscillation_at = time.monotonic()
        self.log("Oscillation on")
        return True

    def stop_and_disable(self) -> None:
        self.oscillating = False
        self.jog_active = False
        if self.velocity_configured:
            try:
                self.write_private_param_f32(PARAM_SPD_REF, 0.0)
                time.sleep(0.08)
            except Exception as exc:
                self.log(f"zero-speed before stop failed: {exc}")
        try:
            self.send_private_disable(False)
        finally:
            self.velocity_configured = False
            self.position_configured = False
            self.arm_position_configured = False
            self.clear_arm_route()
            self.commanded_speed = 0.0
            self.log("Stop/disable sent")

    def set_active_report(self, enabled: bool) -> None:
        payload = bytes([1, 2, 3, 4, 5, 6, 1 if enabled else 0, 0])
        self.send_private(COMM_PROACTIVE_REPORT, self.host_id, self.motor_id, payload)
        self.active_reports = enabled
        self.log(f"Active reports {'on' if enabled else 'off'}")

    def request_status(self) -> None:
        for index, label in (
            (PARAM_RUN_MODE, "run_mode"),
            (PARAM_SPD_REF, "spd_ref"),
            (PARAM_LOC_REF, "loc_ref"),
            (PARAM_LIMIT_CUR, "limit_cur"),
            (PARAM_LOC_KP, "loc_kp"),
            (PARAM_PP_VEL_MAX, "vel_max"),
            (PARAM_PP_ACC_SET, "acc_set"),
        ):
            raw = self.wait_private_param(index, COMMAND_TIMEOUT_S)
            if raw is None:
                self.log(f"param {label} timeout")
            elif index == PARAM_RUN_MODE:
                self.log(f"param {label}={raw[0]}")
            else:
                self.log(f"param {label}={read_f32_le(raw):.4f}")

    def assign_motor_id(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        old_id = private_device_id(payload.get("oldMotorId"), "current motor ID", self.motor_id, allow_zero=True)
        new_id = private_device_id(payload.get("newMotorId"), "new motor ID")
        if old_id == new_id:
            raise ValueError("current motor ID and new motor ID are the same")

        role = str(payload.get("role") or "").strip().lower()
        if role and role not in ARM_AXES:
            raise ValueError("IK role must be base, shoulder, elbow, or blank")

        store = bool(payload.get("store", True))
        with self.lock:
            self.oscillating = False
            self.jog_active = False
            self.velocity_configured = False
            self.position_configured = False
            self.arm_position_configured = False
            self.clear_arm_route()
            self.commanded_speed = 0.0

        ping = self.ping_private_candidates(old_id, COMMAND_TIMEOUT_S)
        if ping is None:
            raise ValueError(f"current motor ID {fmt_id(old_id)} did not respond")
        token, host_id, responder = ping
        self.log(
            f"Assigning motor ID {fmt_id(old_id)} -> {fmt_id(new_id)} "
            f"host={fmt_id(host_id)} responder={fmt_id(responder)} uuid={bytes_hex(token).replace(' ', '')}"
        )

        with self.lock:
            self.host_id = host_id & 0xFF
        self.send_private_disable_to(old_id, False)
        time.sleep(0.08)
        self.send_private_set_device_id(old_id, new_id, host_id, token)
        time.sleep(0.25)

        verified = self.ping_private_candidates(new_id, COMMAND_TIMEOUT_S)
        if verified is None:
            old_still_present = self.ping_private_candidates(old_id, max(0.18, COMMAND_TIMEOUT_S / 2)) is not None
            if old_still_present:
                raise ValueError(f"motor stayed at {fmt_id(old_id)}; ID change did not verify")
            raise ValueError(f"new motor ID {fmt_id(new_id)} did not verify; power-cycle and scan before retrying")

        save_ack = False
        if store:
            start = self.current_seq()
            self.send_private_save_parameters_to(new_id, host_id)
            save_ack = self.wait_private_save_ack(new_id, 0.50, start)
            if not save_ack:
                self.log(f"Save-parameters ack not seen for {fmt_id(new_id)}; ID works but power-cycle persistence is unconfirmed")

        with self.lock:
            self.motor_id = new_id
            if role:
                self.arm_motor_ids[role] = new_id
                used_ids = {new_id}
                for axis in ARM_AXES:
                    if axis == role:
                        continue
                    if self.arm_motor_ids[axis] in used_ids:
                        replacement = ARM_ROLE_DEFAULT_IDS[axis]
                        if replacement in used_ids:
                            replacement = next(
                                candidate
                                for candidate in range(1, SCAN_LAST_ID + 1)
                                if candidate not in used_ids
                            )
                        self.arm_motor_ids[axis] = replacement
                    used_ids.add(self.arm_motor_ids[axis])
            discovered = set(self.discovered_private)
            discovered.discard(old_id)
            discovered.add(new_id)
            self.discovered_private = sorted(discovered)

        if role and store:
            self.save_values()

        message = f"Motor ID assigned {fmt_id(old_id)} -> {fmt_id(new_id)}"
        if role:
            message += f" for {role}"
        if store:
            message += " and save sent"
        self.log(message)
        return {
            "ok": True,
            "message": message,
            "oldMotorId": old_id,
            "oldMotorIdHex": fmt_id(old_id),
            "newMotorId": new_id,
            "newMotorIdHex": fmt_id(new_id),
            "role": role,
            "stored": store,
            "saveAck": save_ack,
        }

    def scan_private_once(self) -> List[int]:
        with self.lock:
            self.discovered_private = []
        self.log(f"Scanning private IDs {fmt_id(SCAN_FIRST_PRIVATE_ID)}..{fmt_id(SCAN_LAST_ID)}")
        for target_id in range(SCAN_FIRST_PRIVATE_ID, SCAN_LAST_ID + 1):
            start = self.current_seq()
            self.send_private_get_id(target_id)

            def matches(frame: Any, wanted: int = target_id) -> bool:
                if not frame.extended or len(frame.data) < 8:
                    return False
                comm_type, extra, _host = split_private_ext_id(frame.arbitration_id)
                return comm_type == COMM_GET_ID and (extra & 0xFF) == wanted

            self.wait_for_frame(matches, SCAN_PER_ID_TIMEOUT_S, start)
        with self.lock:
            found = sorted(set(self.discovered_private))
            self.discovered_private = found
        return found

    def scan_private(self, recover_on_empty: bool = True) -> List[int]:
        attempts = PRIVATE_SCAN_RECOVERY_ATTEMPTS if recover_on_empty else 1
        found: List[int] = []
        for attempt in range(1, attempts + 1):
            if attempt > 1:
                self.log(f"Private scan recovery {attempt}/{attempts}: reopening USB adapter")
                if not self.reopen_bus():
                    self.log("Private scan recovery stopped: USB adapter did not reopen")
                    break
                time.sleep(PRIVATE_SCAN_RECOVERY_SETTLE_S)

            try:
                found = self.scan_private_once()
            except (OSError, RuntimeError) as exc:
                self.log(f"Private scan attempt {attempt}/{attempts} failed: {exc}")
                found = []
                if not recover_on_empty:
                    raise
            if found or not recover_on_empty:
                break

        if found:
            with self.lock:
                self.motor_id = found[0]
                self.discovered_private = found
            self.velocity_configured = False
            self.position_configured = False
            self.log(f"Private scan found {len(found)} motor(s); selected {fmt_id(self.motor_id)}")
        else:
            self.log("Private scan found no motors")
        return found

    def startup_scan_loop(self) -> None:
        time.sleep(STARTUP_SCAN_DELAY_S)
        for round_index in range(1, STARTUP_SCAN_ROUNDS + 1):
            if not self.running:
                return
            with self.lock:
                if self.discovered_private:
                    return

            if not self.command_lock.acquire(blocking=False):
                time.sleep(STARTUP_SCAN_RETRY_S)
                continue

            with self.lock:
                self.busy = True
            try:
                with self.lock:
                    connected = self.connected
                if not connected:
                    self.log(f"Startup motor scan {round_index}/{STARTUP_SCAN_ROUNDS}: opening USB adapter")
                    opened = self.open_bus()
                else:
                    self.log(f"Startup motor scan {round_index}/{STARTUP_SCAN_ROUNDS}")
                    opened = True

                if opened and self.scan_private(recover_on_empty=True):
                    return
            except Exception as exc:
                self.log(f"Startup motor scan failed: {exc}")
            finally:
                with self.lock:
                    self.busy = False
                self.command_lock.release()

            time.sleep(STARTUP_SCAN_RETRY_S)

    def update_loop(self) -> None:
        while self.running:
            now = time.monotonic()
            try:
                if self.jog_active and now >= self.jog_stop_at:
                    self.jog_active = False
                    if self.velocity_configured:
                        self.set_speed(0.0)
                        self.log("Jog released: speed=0")
                if self.oscillating and now - self.last_oscillation_at >= OSCILLATION_PERIOD_S:
                    self.last_oscillation_at = now
                    self.set_speed(-self.commanded_speed)
                if (
                    self.velocity_configured
                    and not self.position_configured
                    and abs(self.commanded_speed) > 0.0001
                    and now - self.last_velocity_refresh_at >= VELOCITY_REFRESH_S
                    and not self.command_lock.locked()
                ):
                    self.send_velocity_target(self.commanded_speed)
                if (
                    self.position_configured
                    and now - self.last_position_refresh_at >= POSITION_REFRESH_S
                    and not self.command_lock.locked()
                ):
                    self.send_position_target()
                if self.arm_position_configured and not self.command_lock.locked():
                    arm_route_due = bool(self.arm_route_waypoints) and now >= self.arm_route_next_at
                    arm_refresh_due = now - self.last_arm_position_refresh_at >= ARM_OPERATION_REFRESH_S
                    if arm_route_due or arm_refresh_due:
                        self.send_arm_position_targets()
            except Exception as exc:
                self.log(f"motion update failed: {exc}")
            time.sleep(0.03)

    def log_last_frames(self) -> None:
        with self.lock:
            frames = [frame for _seq, frame in list(self.frames)[-32:]]
        for frame in frames:
            self.log(
                f"raw {'ext' if frame.extended else 'std'} "
                f"id={fmt_id(frame.arbitration_id, 8 if frame.extended else 3)} "
                f"dlc={len(frame.data)} data={bytes_hex(frame.data)}"
            )

    def arm_solution_snapshot(self) -> Dict[str, Any]:
        try:
            solution = solve_arm_ik(
                self.arm_joint_count,
                self.arm_target["x"],
                self.arm_target["y"],
                self.arm_target["z"],
                self.arm_link_1,
                self.arm_link_2,
                self.arm_elbow_up,
            )
        except ValueError as exc:
            return {
                "ok": False,
                "message": str(exc),
                "safety": {"ok": False, "warnings": [str(exc)]},
                "jointAngles": dict(self.arm_joint_angles),
                "motorTargets": dict(self.arm_motor_targets),
            }
        raw_joint_angles = {
            "base": solution.base,
            "shoulder": solution.shoulder,
            "elbow": solution.elbow,
        }
        joint_angles = route_arm_joint_angles(
            self.arm_joint_count,
            raw_joint_angles,
            self.arm_twist_limits,
            self.arm_joint_angles,
        )
        routed_solution = ArmIkSolution(
            base=joint_angles["base"],
            shoulder=joint_angles["shoulder"],
            elbow=joint_angles["elbow"],
        )
        points = arm_solution_points(
            self.arm_joint_count,
            routed_solution,
            self.arm_target,
            self.arm_link_1,
            self.arm_link_2,
        )
        safety = arm_safety_check(
            self.arm_joint_count,
            points,
            self.arm_link_radii,
            joint_angles,
            self.arm_twist_limits,
        )
        return {
            "ok": True,
            "message": "",
            "safety": safety,
            "jointAngles": joint_angles,
            "motorTargets": {
                axis: self.arm_motor_target(axis, joint_angles[axis])
                for axis in self.active_arm_axes()
            },
        }

    def telemetry_roles_for_motor(self, motor_id: int) -> List[str]:
        motor_id &= 0xFF
        roles: List[str] = []
        if motor_id == (self.motor_id & 0xFF):
            roles.append("Selected")
        labels = {"base": "Base", "shoulder": "Shoulder", "elbow": "Elbow"}
        for axis in self.active_arm_axes():
            if (self.arm_motor_ids[axis] & 0xFF) == motor_id:
                roles.append(labels[axis])
        return roles

    def active_command_motor_ids(self) -> List[int]:
        ids: List[int] = []
        if self.velocity_configured or self.position_configured or self.jog_active or self.oscillating:
            ids.append(self.motor_id & 0xFF)
        if self.arm_position_configured or self.arm_route_waypoints:
            ids.extend(self.arm_motor_ids[axis] & 0xFF for axis in self.active_arm_axes())
        out: List[int] = []
        for motor_id in ids:
            if motor_id not in out:
                out.append(motor_id)
        return out

    def telemetry_target_for_motor(self, motor_id: int) -> Tuple[Optional[float], Optional[float]]:
        motor_id &= 0xFF
        for axis in self.active_arm_axes():
            if (self.arm_motor_ids[axis] & 0xFF) == motor_id:
                return self.arm_motor_targets.get(axis), None
        if motor_id == (self.motor_id & 0xFF):
            if self.position_configured:
                return self.position_target, None
            if self.velocity_configured or self.jog_active or self.oscillating:
                return None, self.commanded_speed
        return None, None

    def telemetry_motors_snapshot(self, now: float) -> List[Dict[str, Any]]:
        commanded_ids = self.active_command_motor_ids()
        commanded_set = set(commanded_ids)
        ids = set(commanded_ids)
        for motor_id, seen_at in self.feedback_at_by_motor.items():
            if motor_id in commanded_set or motor_id == (self.motor_id & 0xFF) or now - seen_at <= TELEMETRY_ACTIVE_WINDOW_S:
                ids.add(motor_id)

        arm_order = {
            self.arm_motor_ids[axis] & 0xFF: index
            for index, axis in enumerate(self.active_arm_axes())
        }

        def sort_key(motor_id: int) -> Tuple[int, int, int]:
            return (
                0 if motor_id in commanded_set else 1,
                arm_order.get(motor_id, 9),
                motor_id,
            )

        rows: List[Dict[str, Any]] = []
        for motor_id in sorted(ids, key=sort_key):
            feedback = dict(self.feedback_by_motor.get(motor_id, {}))
            feedback_seen_at = self.feedback_at_by_motor.get(motor_id)
            fault = dict(self.private_faults_by_motor.get(motor_id, {}))
            fault_seen_at = self.private_faults_at_by_motor.get(motor_id)
            target_rad, commanded_velocity = self.telemetry_target_for_motor(motor_id)
            age_ms: Optional[int] = None
            if feedback_seen_at is not None:
                age_ms = int((now - feedback_seen_at) * 1000)
            fault_age_ms: Optional[int] = None
            if fault and fault_seen_at is not None:
                fault_age_ms = int((now - fault_seen_at) * 1000)
                fault["ageMs"] = fault_age_ms

            roles = self.telemetry_roles_for_motor(motor_id)
            active = motor_id in commanded_set or (
                feedback_seen_at is not None and now - feedback_seen_at <= TELEMETRY_ACTIVE_WINDOW_S
            )
            row: Dict[str, Any] = {
                "protocol": feedback.get("protocol", PROTOCOL_PRIVATE),
                "motorId": motor_id,
                "motorIdHex": fmt_id(motor_id),
                "roles": roles,
                "role": ", ".join(roles),
                "model": feedback.get("model", self.model_for_motor(motor_id)),
                "ageMs": age_ms,
                "active": active,
                "targetRad": target_rad,
                "commandedVelocityRadS": commanded_velocity,
                "positionRad": feedback.get("positionRad"),
                "velocityRadS": feedback.get("velocityRadS"),
                "torqueNm": feedback.get("torqueNm"),
                "temperatureC": feedback.get("temperatureC"),
                "modeState": feedback.get("modeState"),
                "fault": bool(feedback.get("fault")) or bool(fault.get("faultRaw")),
                "warning": bool(feedback.get("warning")) or bool(fault.get("warningRaw")),
                "faultFrame": fault or None,
            }
            rows.append(row)
        return rows

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            now = time.monotonic()
            feedback = dict(self.last_feedback) if self.last_feedback else None
            if feedback and self.last_feedback_at:
                feedback["ageMs"] = int((now - self.last_feedback_at) * 1000)
            private_fault = dict(self.last_private_fault) if self.last_private_fault else None
            if private_fault and self.last_private_fault_at:
                private_fault["ageMs"] = int((now - self.last_private_fault_at) * 1000)
            transport = self.bus_transport()
            serial_port = getattr(self.bus, "serial_port", DEFAULT_SERIAL_PORT)
            serial_baud = getattr(self.bus, "serial_baud", DEFAULT_SERIAL_BAUD)
            bus_label = self.bus_label()
            bus_stats = self.bus.stats()
            snapshot = {
                "appVersion": APP_VERSION,
                "valuesPath": str(VALUES_PATH),
                "connected": self.connected,
                "openError": self.open_error,
                "transport": transport,
                "transportLabel": bus_label,
                "serialPort": serial_port,
                "serialBaud": serial_baud,
                "motorId": self.motor_id,
                "motorIdHex": fmt_id(self.motor_id),
                "hostId": self.host_id,
                "hostIdHex": fmt_id(self.host_id),
                "model": self.model,
                "testSpeed": self.test_speed,
                "commandedSpeed": self.commanded_speed,
                "positionTarget": self.position_target,
                "positionVelocityLimit": self.position_velocity_limit,
                "positionAcceleration": self.position_acceleration,
                "positionKp": self.position_kp,
                "velocityConfigured": self.velocity_configured,
                "positionConfigured": self.position_configured,
                "arm": {
                    "jointCount": self.arm_joint_count,
                    "motorIds": dict(self.arm_motor_ids),
                    "motorIdHex": {axis: fmt_id(self.arm_motor_ids[axis]) for axis in ARM_AXES},
                    "models": dict(self.arm_motor_models),
                    "offsets": dict(self.arm_offsets),
                    "directions": dict(self.arm_directions),
                    "link1": self.arm_link_1,
                    "link2": self.arm_link_2,
                    "radii": dict(self.arm_link_radii),
                    "twistLimits": dict(self.arm_twist_limits),
                    "target": dict(self.arm_target),
                    "elbowUp": self.arm_elbow_up,
                    "velocityLimit": self.arm_velocity_limit,
                    "acceleration": self.arm_acceleration,
                    "positionKp": self.arm_position_kp,
                    "dampingKd": self.arm_damping_kd,
                    "currentLimit": self.arm_current_limit,
                    "torqueBiases": dict(self.arm_torque_biases),
                    "configured": self.arm_position_configured,
                    "routeRemaining": len(self.arm_route_waypoints),
                    "jointAngles": dict(self.arm_joint_angles),
                    "motorTargets": dict(self.arm_motor_targets),
                    "solution": self.arm_solution_snapshot(),
                },
                "activeReports": self.active_reports,
                "oscillating": self.oscillating,
                "jogActive": self.jog_active,
                "busy": self.busy,
                "lastFeedback": feedback,
                "telemetryMotors": self.telemetry_motors_snapshot(now),
                "telemetryActiveWindowMs": int(TELEMETRY_ACTIVE_WINDOW_S * 1000),
                "lastPrivateFault": private_fault,
                "lastRawFrame": self.last_raw_frame,
                "discoveredPrivate": [fmt_id(item) for item in self.discovered_private],
                "logs": list(self.logs),
                "canStats": bus_stats,
            }
        return snapshot


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "RobStrideDashboard/1.0"

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/state":
            self.send_json(self.controller.snapshot())
            return
        if parsed.path == "/api/logs":
            query = parse_qs(parsed.query)
            after = parse_int((query.get("after") or [0])[0], 0)
            logs = self.controller.snapshot()["logs"]
            self.send_json({"after": after, "logs": logs})
            return
        if parsed.path == "/api/values":
            self.send_json(self.controller.values_snapshot())
            return
        self.serve_static(parsed.path)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        payload = self.read_json()
        if parsed.path == "/api/config":
            self.send_json(self.controller.configure(payload))
            return
        if parsed.path == "/api/values":
            self.send_json(self.controller.apply_values(payload))
            return
        if parsed.path == "/api/logs/clear":
            self.controller.clear_logs()
            self.send_json({"ok": True})
            return
        if parsed.path == "/api/command":
            command = str(payload.get("command") or "")
            background = command.startswith("scan") or command == "raw"
            if background:
                thread = threading.Thread(
                    target=self.controller.run_command,
                    args=(command, payload),
                    daemon=True,
                )
                thread.start()
                self.send_json({"ok": True, "queued": True})
            else:
                self.send_json(self.controller.run_command(command, payload))
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    @property
    def controller(self) -> DashboardController:
        return self.server.controller  # type: ignore[attr-defined]

    def read_json(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        return json.loads(raw.decode("utf-8"))

    def send_json(self, payload: Dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def serve_static(self, path: str) -> None:
        if path in ("", "/"):
            rel = "index.html"
        else:
            rel = path.lstrip("/")
        target = (WEB_DIR / rel).resolve()
        try:
            target.relative_to(WEB_DIR.resolve())
        except ValueError:
            self.send_error(HTTPStatus.FORBIDDEN)
            return
        if not target.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        body = target.read_bytes()
        mime = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: object) -> None:
        return


class DashboardServer(ThreadingHTTPServer):
    def __init__(self, server_address: Tuple[str, int], handler: Any, controller: DashboardController):
        super().__init__(server_address, handler)
        self.controller = controller


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--transport", default=TRANSPORT_ROBSTRIDE_USB, help=argparse.SUPPRESS)
    parser.add_argument("--interface", default="", help=argparse.SUPPRESS)
    parser.add_argument("--feedback-id", default="", help=argparse.SUPPRESS)
    parser.add_argument("--protocol", default=PROTOCOL_PRIVATE, help=argparse.SUPPRESS)
    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--serial-baud", type=int, default=DEFAULT_SERIAL_BAUD)
    parser.add_argument("--motor-id", default=hex(DEFAULT_MOTOR_ID))
    parser.add_argument("--host-id", default=hex(DEFAULT_HOST_ID))
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--no-open", action="store_true", help="start dashboard without opening the USB adapter")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    controller = DashboardController(
        serial_port=args.serial_port,
        serial_baud=args.serial_baud,
        motor_id=parse_int(args.motor_id, DEFAULT_MOTOR_ID),
        host_id=parse_int(args.host_id, DEFAULT_HOST_ID),
        model=args.model,
        open_can=not args.no_open,
    )
    server = DashboardServer((args.host, args.port), DashboardHandler, controller)
    print(f"RobStride dashboard listening on http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        controller.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
