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
    fmt_id,
    private_fault_summary,
    read_f32_le,
    split_private_ext_id,
    uint_to_float,
)


COMM_GET_ID = 0x00
COMM_OPERATION_STATUS = 0x02
COMM_ENABLE = 0x03
COMM_DISABLE = 0x04
COMM_READ_PARAM = 0x11
COMM_WRITE_PARAM = 0x12
COMM_FAULT = 0x15
COMM_PROACTIVE_REPORT = 0x18

MOTOR_STUDIO_JOG_SPEED_RAD_S = 1.0
MOTOR_STUDIO_JOG_S = 0.75
OSCILLATION_PERIOD_S = 2.5
VELOCITY_REFRESH_S = 0.10
POSITION_REFRESH_S = 0.10
ARM_POSITION_REFRESH_S = 0.10
MAX_LOG_LINES = 240
MAX_FRAME_HISTORY = 600
COMMAND_TIMEOUT_S = 0.6
ARM_AXES = ("base", "shoulder", "elbow")

ROOT_DIR = Path(__file__).resolve().parent
REPO_DIR = ROOT_DIR.parent
WEB_DIR = ROOT_DIR / "web"
UPDATE_LOG_PATH = ROOT_DIR / "update.log"
APP_VERSION = "2026.08.27.7"


def parse_int(value: Any, default: int) -> int:
    if value is None or value == "":
        return default
    if isinstance(value, int):
        return value
    return int(str(value), 0)


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


def signed_direction(value: Any, default: int = 1) -> int:
    try:
        parsed = int(str(value), 0)
    except (TypeError, ValueError):
        parsed = default
    return -1 if parsed < 0 else 1


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
        self.arm_motor_ids = {
            "base": self.motor_id,
            "shoulder": 0x01,
            "elbow": 0x02,
        }
        self.arm_offsets = {axis: 0.0 for axis in ARM_AXES}
        self.arm_directions = {axis: 1 for axis in ARM_AXES}
        self.arm_link_1 = 0.25
        self.arm_link_2 = 0.25
        self.arm_target = {"x": 0.25, "y": 0.0, "z": 0.10}
        self.arm_elbow_up = False
        self.arm_velocity_limit = DEFAULT_POSITION_VEL_RAD_S
        self.arm_acceleration = DEFAULT_POSITION_ACCEL_RAD_S2
        self.arm_position_kp = DEFAULT_POSITION_KP
        self.arm_position_configured = False
        self.arm_motor_targets = {axis: 0.0 for axis in ARM_AXES}
        self.arm_joint_angles = {axis: 0.0 for axis in ARM_AXES}
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
        self.last_private_fault: Optional[Dict[str, Any]] = None
        self.discovered_private: List[int] = []
        self.busy = False
        self.update_lock = threading.Lock()
        self.update_process: Optional[subprocess.Popen[bytes]] = None
        self.update_started_at = 0.0
        self.update_last_exit: Optional[int] = None
        self.repo_cache: Dict[str, Any] = {}
        self.repo_cache_at = 0.0

        if open_can:
            self.open_bus()

        self.rx_thread = threading.Thread(target=self.rx_loop, name="can-rx", daemon=True)
        self.rx_thread.start()

        self.update_thread = threading.Thread(target=self.update_loop, name="motion-update", daemon=True)
        self.update_thread.start()

    def log(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        with self.lock:
            self.logs.append(f"{stamp} {message}")

    def clear_logs(self) -> None:
        with self.lock:
            self.logs.clear()

    def repo_info(self) -> Dict[str, Any]:
        now = time.monotonic()
        with self.update_lock:
            process = self.update_process
            running = process is not None and process.poll() is None
            last_exit = self.update_last_exit
            started_at = self.update_started_at
            cached = dict(self.repo_cache) if now - self.repo_cache_at < 2.0 else None

        if cached is None:
            def git(*args: str) -> str:
                try:
                    result = subprocess.run(
                        ["git", "-C", str(REPO_DIR), *args],
                        check=False,
                        capture_output=True,
                        text=True,
                        timeout=2.0,
                    )
                except (OSError, subprocess.TimeoutExpired):
                    return ""
                return result.stdout.strip() if result.returncode == 0 else ""

            status_lines = [line for line in git("status", "--short").splitlines() if line.strip()]
            cached = {
                "remote": git("remote", "get-url", "origin"),
                "branch": git("symbolic-ref", "--quiet", "--short", "HEAD"),
                "commit": git("rev-parse", "--short", "HEAD"),
                "dirtyCount": len(status_lines),
                "dirty": len(status_lines) > 0,
            }
            with self.update_lock:
                self.repo_cache = dict(cached)
                self.repo_cache_at = now

        return {
            **cached,
            "updateRunning": running,
            "updateStartedAt": started_at,
            "updateLastExit": last_exit,
            "updateLog": self.update_log_tail(),
        }

    def update_log_tail(self, max_bytes: int = 12000) -> str:
        try:
            with UPDATE_LOG_PATH.open("rb") as handle:
                handle.seek(0, os.SEEK_END)
                size = handle.tell()
                handle.seek(max(0, size - max_bytes), os.SEEK_SET)
                return handle.read().decode("utf-8", errors="replace")
        except OSError:
            return ""

    def start_repo_update(self, remote: str = "origin", branch: str = "") -> Dict[str, Any]:
        with self.update_lock:
            if self.update_process is not None and self.update_process.poll() is None:
                return {"ok": False, "message": "An update is already running."}

            command = ["bash", str(ROOT_DIR / "update_from_github.sh"), remote]
            if branch:
                command.append(branch)

            UPDATE_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            with UPDATE_LOG_PATH.open("ab", buffering=0) as log_file:
                log_file.write(
                    f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] Dashboard update requested\n".encode(
                        "utf-8"
                    )
                )
                try:
                    process = subprocess.Popen(
                        command,
                        cwd=str(REPO_DIR),
                        stdin=subprocess.DEVNULL,
                        stdout=log_file,
                        stderr=subprocess.STDOUT,
                    )
                except OSError as exc:
                    log_file.write(f"failed to start update: {exc}\n".encode("utf-8"))
                    self.update_process = None
                    self.update_last_exit = -1
                    return {"ok": False, "message": str(exc)}

            self.update_process = process
            self.update_started_at = time.time()
            self.update_last_exit = None

        self.log(f"GitHub update started pid={process.pid}")
        watcher = threading.Thread(target=self.watch_update_process, args=(process,), daemon=True)
        watcher.start()
        return {"ok": True, "pid": process.pid}

    def watch_update_process(self, process: subprocess.Popen[bytes]) -> None:
        exit_code = process.wait()
        with self.update_lock:
            if self.update_process is process:
                self.update_last_exit = exit_code
        self.log(f"GitHub update exited with code {exit_code}")

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

    def send(self, arbitration_id: int, data: bytes, extended: bool) -> None:
        with self.bus_lock:
            if not self.connected:
                raise RuntimeError("RobStride USB adapter is not open")
            self.bus.send(arbitration_id, data, extended=extended)

    def send_private(self, comm_type: int, extra_data: int, target_id: int, data: bytes) -> None:
        self.send(build_private_ext_id(comm_type, extra_data, target_id), data, extended=True)

    def send_private_get_id(self, target_id: int) -> None:
        self.send_private(COMM_GET_ID, self.host_id, target_id, bytes(8))

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
                p_max, v_max, t_max = PRIVATE_MODEL_LIMITS.get(
                    self.model,
                    PRIVATE_MODEL_LIMITS[DEFAULT_MODEL],
                )
                pos_raw = (frame.data[0] << 8) | frame.data[1]
                vel_raw = (frame.data[2] << 8) | frame.data[3]
                torque_raw = (frame.data[4] << 8) | frame.data[5]
                temp_raw = (frame.data[6] << 8) | frame.data[7]
                self.last_feedback_at = time.monotonic()
                self.last_feedback = {
                    "protocol": PROTOCOL_PRIVATE,
                    "targetHost": host,
                    "motorId": source_motor,
                    "positionRad": uint_to_float(pos_raw, -p_max, p_max, 16),
                    "velocityRadS": uint_to_float(vel_raw, -v_max, v_max, 16),
                    "torqueNm": uint_to_float(torque_raw, -t_max, t_max, 16),
                    "temperatureC": temp_raw * 0.1,
                    "fault": False,
                    "warning": False,
                    "modeState": None,
                    "ageMs": 0,
                }
                return
            if comm_type in (COMM_READ_PARAM, COMM_WRITE_PARAM):
                index = int.from_bytes(frame.data[0:2], "little")
                raw = frame.data[4:8]
                if index == PARAM_RUN_MODE:
                    self.log(f"param run_mode={raw[0]}")
                return
            if comm_type == COMM_FAULT:
                report = decode_private_fault_payload(frame.data)
                self.last_private_fault_at = time.monotonic()
                self.last_private_fault = {
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
            if command in ("stop", "clear-fault", "arm-stop", "arm-clear-fault"):
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
                    else:
                        self.clear_arm_faults()
                        message = "Arm clear fault sent while another command was running."
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
        self.log(f"Command {command}")
        if command == "reopen":
            ok = self.reopen_bus()
            return {"ok": ok}
        if not self.connected:
            opened = self.open_bus()
            if not opened:
                return {"ok": False, "message": self.open_error or "CAN is not open"}

        if command == "scan":
            self.scan_private()
            return {"ok": True}
        if command == "scan-private":
            self.scan_private()
            return {"ok": True}
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

    def configure_velocity(self) -> bool:
        return self.configure_private_velocity()

    def clear_private_fault(self, label: str = "Private clear-error sent") -> None:
        self.send_private_disable(True)
        self.wait_private_status(0.30)
        with self.lock:
            self.last_private_fault = None
            self.last_private_fault_at = 0.0
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
            self.last_private_fault = None
            self.last_private_fault_at = 0.0
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
        self.clear_private_fault()

    def configure_private_velocity(self) -> bool:
        self.velocity_configured = False
        self.position_configured = False
        self.arm_position_configured = False
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

    def configure_private_position_motor(
        self,
        axis: str,
        motor_id: int,
        position: float,
        velocity_limit: float,
        acceleration: float,
        position_kp: float,
    ) -> bool:
        motor_id &= 0xFF
        self.prepare_private_mode_switch_for(
            motor_id,
            f"{axis} {fmt_id(motor_id)} disabled/clear-error before position configure",
        )
        if not self.write_private_run_mode_verified_for(motor_id, RUN_MODE_POSITION):
            self.log(f"{axis} {fmt_id(motor_id)} position setup failed: run_mode did not verify")
            return False
        self.send_private_enable_to(motor_id)
        self.wait_private_status_for(motor_id, 0.30)
        self.write_private_param_f32_to(motor_id, PARAM_LIMIT_CUR, DEFAULT_CURRENT_LIMIT_A)
        self.write_private_param_f32_to(motor_id, PARAM_PP_VEL_MAX, velocity_limit)
        self.write_private_param_f32_to(motor_id, PARAM_PP_ACC_SET, acceleration)
        self.write_private_param_f32_to(motor_id, PARAM_LOC_KP, position_kp)
        self.write_private_param_f32_to(motor_id, PARAM_LOC_REF, position)
        return True

    def apply_arm_payload(self, payload: Dict[str, Any]) -> None:
        self.arm_motor_ids = {
            "base": parse_int(payload.get("armBaseMotorId"), self.arm_motor_ids["base"]) & 0xFF,
            "shoulder": parse_int(payload.get("armShoulderMotorId"), self.arm_motor_ids["shoulder"]) & 0xFF,
            "elbow": parse_int(payload.get("armElbowMotorId"), self.arm_motor_ids["elbow"]) & 0xFF,
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
        self.arm_target = {
            "x": parse_float(payload.get("armTargetX"), self.arm_target["x"]),
            "y": parse_float(payload.get("armTargetY"), self.arm_target["y"]),
            "z": parse_float(payload.get("armTargetZ"), self.arm_target["z"]),
        }
        self.arm_elbow_up = bool(payload.get("armElbowUp", self.arm_elbow_up))
        self.arm_velocity_limit = positive_float(payload.get("armVelocityLimit"), self.arm_velocity_limit, 20.0)
        self.arm_acceleration = positive_float(payload.get("armAcceleration"), self.arm_acceleration, 200.0)
        self.arm_position_kp = nonnegative_float(payload.get("armPositionKp"), self.arm_position_kp, 200.0)

    def arm_motor_target(self, axis: str, joint_angle: float) -> float:
        return self.arm_offsets[axis] + (self.arm_directions[axis] * joint_angle)

    def move_arm_ik(self, payload: Dict[str, Any]) -> bool:
        self.apply_arm_payload(payload)
        solution = solve_three_axis_arm_ik(
            self.arm_target["x"],
            self.arm_target["y"],
            self.arm_target["z"],
            self.arm_link_1,
            self.arm_link_2,
            self.arm_elbow_up,
        )
        joint_angles = {
            "base": solution.base,
            "shoulder": solution.shoulder,
            "elbow": solution.elbow,
        }
        motor_targets = {
            axis: self.arm_motor_target(axis, joint_angles[axis])
            for axis in ARM_AXES
        }
        self.oscillating = False
        self.jog_active = False
        self.velocity_configured = False
        self.position_configured = False
        self.arm_position_configured = False
        self.commanded_speed = 0.0
        self.log(
            "Arm IK target "
            f"x={self.arm_target['x']:+.3f} y={self.arm_target['y']:+.3f} z={self.arm_target['z']:+.3f} "
            f"joints base={joint_angles['base']:+.3f} shoulder={joint_angles['shoulder']:+.3f} elbow={joint_angles['elbow']:+.3f}"
        )
        for axis in ARM_AXES:
            if not self.configure_private_position_motor(
                axis,
                self.arm_motor_ids[axis],
                motor_targets[axis],
                self.arm_velocity_limit,
                self.arm_acceleration,
                self.arm_position_kp,
            ):
                self.stop_arm()
                return False
        self.arm_joint_angles = joint_angles
        self.arm_motor_targets = motor_targets
        self.arm_position_configured = True
        self.last_arm_position_refresh_at = time.monotonic()
        self.log(
            "Arm targets sent "
            f"base={motor_targets['base']:+.3f} shoulder={motor_targets['shoulder']:+.3f} elbow={motor_targets['elbow']:+.3f}"
        )
        return True

    def send_arm_position_targets(self) -> None:
        for axis in ARM_AXES:
            self.write_private_param_f32_to(
                self.arm_motor_ids[axis],
                PARAM_LOC_REF,
                self.arm_motor_targets[axis],
            )
        self.last_arm_position_refresh_at = time.monotonic()

    def stop_arm(self) -> None:
        self.arm_position_configured = False
        self.velocity_configured = False
        self.position_configured = False
        self.oscillating = False
        self.jog_active = False
        self.commanded_speed = 0.0
        for motor_id in sorted(set(self.arm_motor_ids.values())):
            self.send_private_disable_to(motor_id, False)
            self.wait_private_status_for(motor_id, 0.20)
        self.log("Arm stop/disable sent")

    def clear_arm_faults(self) -> None:
        self.arm_position_configured = False
        self.velocity_configured = False
        self.position_configured = False
        self.oscillating = False
        self.jog_active = False
        self.commanded_speed = 0.0
        for motor_id in sorted(set(self.arm_motor_ids.values())):
            self.send_private_disable_to(motor_id, True)
            self.wait_private_status_for(motor_id, 0.20)
        with self.lock:
            self.last_private_fault = None
            self.last_private_fault_at = 0.0
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

    def scan_private(self) -> None:
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
        if self.discovered_private:
            self.motor_id = self.discovered_private[0]
            self.velocity_configured = False
            self.position_configured = False
            self.log(f"Private scan found {len(self.discovered_private)} motor(s); selected {fmt_id(self.motor_id)}")
        else:
            self.log("Private scan found no motors")

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
                if (
                    self.arm_position_configured
                    and now - self.last_arm_position_refresh_at >= ARM_POSITION_REFRESH_S
                    and not self.command_lock.locked()
                ):
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
            solution = solve_three_axis_arm_ik(
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
                "jointAngles": dict(self.arm_joint_angles),
                "motorTargets": dict(self.arm_motor_targets),
            }
        joint_angles = {
            "base": solution.base,
            "shoulder": solution.shoulder,
            "elbow": solution.elbow,
        }
        return {
            "ok": True,
            "message": "",
            "jointAngles": joint_angles,
            "motorTargets": {
                axis: self.arm_motor_target(axis, joint_angles[axis])
                for axis in ARM_AXES
            },
        }

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            feedback = dict(self.last_feedback) if self.last_feedback else None
            if feedback and self.last_feedback_at:
                feedback["ageMs"] = int((time.monotonic() - self.last_feedback_at) * 1000)
            private_fault = dict(self.last_private_fault) if self.last_private_fault else None
            if private_fault and self.last_private_fault_at:
                private_fault["ageMs"] = int((time.monotonic() - self.last_private_fault_at) * 1000)
            transport = self.bus_transport()
            serial_port = getattr(self.bus, "serial_port", DEFAULT_SERIAL_PORT)
            serial_baud = getattr(self.bus, "serial_baud", DEFAULT_SERIAL_BAUD)
            bus_label = self.bus_label()
            bus_stats = self.bus.stats()
            snapshot = {
                "appVersion": APP_VERSION,
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
                    "motorIds": dict(self.arm_motor_ids),
                    "motorIdHex": {axis: fmt_id(self.arm_motor_ids[axis]) for axis in ARM_AXES},
                    "offsets": dict(self.arm_offsets),
                    "directions": dict(self.arm_directions),
                    "link1": self.arm_link_1,
                    "link2": self.arm_link_2,
                    "target": dict(self.arm_target),
                    "elbowUp": self.arm_elbow_up,
                    "velocityLimit": self.arm_velocity_limit,
                    "acceleration": self.arm_acceleration,
                    "positionKp": self.arm_position_kp,
                    "configured": self.arm_position_configured,
                    "jointAngles": dict(self.arm_joint_angles),
                    "motorTargets": dict(self.arm_motor_targets),
                    "solution": self.arm_solution_snapshot(),
                },
                "activeReports": self.active_reports,
                "oscillating": self.oscillating,
                "jogActive": self.jog_active,
                "busy": self.busy,
                "lastFeedback": feedback,
                "lastPrivateFault": private_fault,
                "lastRawFrame": self.last_raw_frame,
                "discoveredPrivate": [fmt_id(item) for item in self.discovered_private],
                "logs": list(self.logs),
                "canStats": bus_stats,
            }
        snapshot["repo"] = self.repo_info()
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
        if parsed.path == "/api/update/log":
            self.send_json({"log": self.controller.update_log_tail()})
            return
        self.serve_static(parsed.path)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        payload = self.read_json()
        if parsed.path == "/api/config":
            self.send_json(self.controller.configure(payload))
            return
        if parsed.path == "/api/logs/clear":
            self.controller.clear_logs()
            self.send_json({"ok": True})
            return
        if parsed.path == "/api/update":
            remote = str(payload.get("remote") or "origin")
            branch = str(payload.get("branch") or "")
            self.send_json(self.controller.start_repo_update(remote, branch))
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
