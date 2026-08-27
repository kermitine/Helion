#!/usr/bin/env python3
"""Local RobStride dashboard for Raspberry Pi CAN adapters."""

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
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, urlparse

from robstride_socketcan import (
    CAN_SFF_MASK,
    DEFAULT_ACCEL_RAD_S2,
    DEFAULT_CURRENT_LIMIT_A,
    DEFAULT_FEEDBACK_ID,
    DEFAULT_HOST_ID,
    DEFAULT_INTERFACE,
    DEFAULT_MODEL,
    DEFAULT_MOTOR_ID,
    DEFAULT_POSITION_ACCEL_RAD_S2,
    DEFAULT_POSITION_RAD,
    DEFAULT_POSITION_VEL_RAD_S,
    DEFAULT_SERIAL_BAUD,
    DEFAULT_SERIAL_PORT,
    DEFAULT_SPEED_RAD_S,
    MIT_TYPED_ID_POSITION,
    MIT_TYPED_ID_VELOCITY,
    PARAM_ACC_RAD,
    PARAM_LOC_REF,
    PARAM_LIMIT_CUR,
    PARAM_PP_ACC_SET,
    PARAM_PP_VEL_MAX,
    PARAM_RUN_MODE,
    PARAM_SPD_REF,
    PRIVATE_HOST_CANDIDATES,
    PRIVATE_MODEL_LIMITS,
    PROTOCOL_MIT,
    PROTOCOL_PRIVATE,
    RUN_MODE_POSITION,
    RUN_MODE_VELOCITY,
    SCAN_FIRST_MIT_ID,
    SCAN_FIRST_PRIVATE_ID,
    SCAN_LAST_ID,
    SCAN_PER_ID_TIMEOUT_S,
    TRANSPORT_CHOICES,
    TRANSPORT_ROBSTRIDE_SERIAL,
    TRANSPORT_SOCKETCAN,
    build_private_ext_id,
    bytes_hex,
    create_bus,
    decode_mit_feedback,
    decode_private_fault_payload,
    f32_le,
    fmt_id,
    mit_active_report_payload,
    mit_fault_query_payload,
    mit_position_payload,
    mit_set_mode_payload,
    mit_special,
    mit_typed_id,
    mit_velocity_payload,
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
MAX_LOG_LINES = 240
MAX_FRAME_HISTORY = 600
COMMAND_TIMEOUT_S = 0.6

ROOT_DIR = Path(__file__).resolve().parent
REPO_DIR = ROOT_DIR.parent
WEB_DIR = ROOT_DIR / "web"
UPDATE_LOG_PATH = ROOT_DIR / "update.log"
APP_VERSION = "2026.08.27.3"


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


def can_stats(interface: str) -> Dict[str, str]:
    base = Path("/sys/class/net") / interface
    out: Dict[str, str] = {}
    for name in ("operstate", "carrier"):
        with contextlib_read(base / name) as value:
            if value is not None:
                out[name] = value
    stats = base / "statistics"
    for name in ("rx_packets", "tx_packets", "rx_errors", "tx_errors", "rx_dropped", "tx_dropped"):
        with contextlib_read(stats / name) as value:
            if value is not None:
                out[name] = value
    return out


class contextlib_read:
    def __init__(self, path: Path):
        self.path = path
        self.value: Optional[str] = None

    def __enter__(self) -> Optional[str]:
        try:
            self.value = self.path.read_text(encoding="utf-8").strip()
        except OSError:
            self.value = None
        return self.value

    def __exit__(self, *_args: object) -> None:
        return None


class DashboardController:
    def __init__(
        self,
        transport: str,
        interface: str,
        serial_port: str,
        serial_baud: int,
        motor_id: int,
        host_id: int,
        feedback_id: int,
        protocol: str,
        model: str,
        open_can: bool,
    ):
        self.lock = threading.RLock()
        self.frame_cv = threading.Condition(self.lock)
        self.command_lock = threading.Lock()
        self.bus = create_bus(
            transport=transport,
            interface=interface,
            serial_port=serial_port,
            serial_baud=serial_baud,
        )
        self.connected = False
        self.open_error = ""
        self.running = True
        self.bus_lock = threading.Lock()
        self.frame_seq = 0
        self.frames: Deque[Tuple[int, Any]] = deque(maxlen=MAX_FRAME_HISTORY)
        self.logs: Deque[str] = deque(maxlen=MAX_LOG_LINES)

        self.motor_id = motor_id & 0xFF
        self.host_id = host_id & 0xFF
        self.feedback_id = feedback_id & CAN_SFF_MASK
        self.protocol = protocol
        self.model = model.lower()
        self.accept_any_feedback_id = True
        self.test_speed = DEFAULT_SPEED_RAD_S
        self.commanded_speed = 0.0
        self.position_target = DEFAULT_POSITION_RAD
        self.position_velocity_limit = DEFAULT_POSITION_VEL_RAD_S
        self.position_acceleration = DEFAULT_POSITION_ACCEL_RAD_S2
        self.velocity_configured = False
        self.position_configured = False
        self.active_reports = False
        self.oscillating = False
        self.jog_active = False
        self.jog_stop_at = 0.0
        self.last_oscillation_at = time.monotonic()
        self.last_velocity_refresh_at = 0.0
        self.last_feedback_at = 0.0
        self.last_private_fault_at = 0.0
        self.last_raw_frame: Optional[Dict[str, Any]] = None
        self.last_feedback: Optional[Dict[str, Any]] = None
        self.last_private_fault: Optional[Dict[str, Any]] = None
        self.discovered_private: List[int] = []
        self.discovered_mit: List[int] = []
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
        if hasattr(self.bus, "label"):
            return self.bus.label()
        return self.bus.interface

    def bus_transport(self) -> str:
        return getattr(self.bus, "transport", TRANSPORT_SOCKETCAN)

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
            self.log(f"CAN open failed on {label}: {exc}")
            return False
        with self.lock:
            self.connected = True
            self.open_error = ""
        self.log(f"Opened {label}")
        return True

    def reopen_bus(
        self,
        interface: Optional[str] = None,
        transport: Optional[str] = None,
        serial_port: Optional[str] = None,
        serial_baud: Optional[int] = None,
    ) -> bool:
        with self.lock:
            self.oscillating = False
            self.jog_active = False
            self.velocity_configured = False
            self.position_configured = False
            next_transport = transport or self.bus_transport()
            next_interface = interface or self.bus.interface
            next_serial_port = serial_port or getattr(self.bus, "serial_port", DEFAULT_SERIAL_PORT)
            next_serial_baud = serial_baud or getattr(self.bus, "serial_baud", DEFAULT_SERIAL_BAUD)
            self.connected = False
        with self.bus_lock:
            self.bus.close()
            self.bus = create_bus(
                transport=next_transport,
                interface=next_interface,
                serial_port=next_serial_port,
                serial_baud=next_serial_baud,
            )
            time.sleep(0.05)
        return self.open_bus()

    def close(self) -> None:
        self.running = False
        with self.bus_lock:
            self.bus.close()

    def send(self, arbitration_id: int, data: bytes, extended: bool) -> None:
        with self.bus_lock:
            if not self.connected:
                raise RuntimeError("CAN transport is not open")
            self.bus.send(arbitration_id, data, extended=extended)

    def send_private(self, comm_type: int, extra_data: int, target_id: int, data: bytes) -> None:
        self.send(build_private_ext_id(comm_type, extra_data, target_id), data, extended=True)

    def send_private_get_id(self, target_id: int) -> None:
        self.send_private(COMM_GET_ID, self.host_id, target_id, bytes(8))

    def send_private_disable(self, clear_error: bool = False) -> None:
        payload = bytearray(8)
        payload[0] = 1 if clear_error else 0
        self.send_private(COMM_DISABLE, self.host_id, self.motor_id, bytes(payload))

    def send_private_enable(self) -> None:
        self.send_private(COMM_ENABLE, self.host_id, self.motor_id, bytes(8))

    def write_private_param_u8(self, index: int, value: int) -> None:
        payload = bytearray(8)
        payload[0:2] = (index & 0xFFFF).to_bytes(2, "little")
        payload[4] = value & 0xFF
        self.send_private(COMM_WRITE_PARAM, self.host_id, self.motor_id, bytes(payload))

    def write_private_param_f32(self, index: int, value: float) -> None:
        payload = bytearray(8)
        payload[0:2] = (index & 0xFFFF).to_bytes(2, "little")
        payload[4:8] = f32_le(value)
        self.send_private(COMM_WRITE_PARAM, self.host_id, self.motor_id, bytes(payload))

    def read_private_param(self, index: int) -> None:
        payload = bytearray(8)
        payload[0:2] = (index & 0xFFFF).to_bytes(2, "little")
        self.send_private(COMM_READ_PARAM, self.host_id, self.motor_id, bytes(payload))

    def send_mit_to_motor(self, payload: bytes) -> None:
        self.send(self.motor_id, payload, extended=False)

    def send_mit_velocity(self, speed: float) -> None:
        self.send(
            mit_typed_id(MIT_TYPED_ID_VELOCITY, self.motor_id),
            mit_velocity_payload(speed, DEFAULT_CURRENT_LIMIT_A),
            extended=False,
        )

    def send_mit_position(self, position: float, velocity_limit: float) -> None:
        self.send(
            mit_typed_id(MIT_TYPED_ID_POSITION, self.motor_id),
            mit_position_payload(position, velocity_limit),
            extended=False,
        )

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
                self.log(f"CAN receive failed: {exc}")
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
                    "feedbackId": host,
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

        feedback = decode_mit_feedback(frame)
        if feedback is None:
            return
        if feedback.motor_id != self.motor_id:
            return
        if self.accept_any_feedback_id and feedback.feedback_id != self.feedback_id:
            self.feedback_id = feedback.feedback_id
            self.log(f"Learned MIT feedback ID {fmt_id(feedback.feedback_id, 3)}")
        elif feedback.feedback_id != self.feedback_id:
            return
        self.last_feedback_at = time.monotonic()
        self.last_feedback = {
            "protocol": PROTOCOL_MIT,
            "feedbackId": feedback.feedback_id,
            "motorId": feedback.motor_id,
            "positionRad": feedback.position_rad,
            "velocityRadS": feedback.velocity_rad_s,
            "torqueNm": feedback.torque_nm,
            "temperatureC": feedback.winding_temp_c,
            "fault": feedback.has_fault,
            "warning": feedback.has_warning,
            "modeState": feedback.mode_state,
            "ageMs": 0,
        }

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

    def wait_private_status(self, timeout_s: float, after_seq: Optional[int] = None) -> Optional[Any]:
        def matches(frame: Any) -> bool:
            if not frame.extended:
                return False
            comm_type, extra, _host = split_private_ext_id(frame.arbitration_id)
            return comm_type == COMM_OPERATION_STATUS and (extra & 0xFF) == self.motor_id

        return self.wait_for_frame(matches, timeout_s, after_seq)

    def wait_private_param(self, index: int, timeout_s: float) -> Optional[bytes]:
        start_seq = self.current_seq()
        self.read_private_param(index)

        def matches(frame: Any) -> bool:
            if not frame.extended or len(frame.data) < 8:
                return False
            comm_type, extra, _host = split_private_ext_id(frame.arbitration_id)
            return (
                comm_type == COMM_READ_PARAM
                and (extra & 0xFF) == self.motor_id
                and int.from_bytes(frame.data[0:2], "little") == index
            )

        frame = self.wait_for_frame(matches, timeout_s, start_seq)
        return None if frame is None else frame.data[4:8]

    def wait_mit_feedback(self, timeout_s: float, motor_id: Optional[int] = None, after_seq: Optional[int] = None) -> Optional[Any]:
        expected_motor_id = self.motor_id if motor_id is None else motor_id

        def matches(frame: Any) -> bool:
            return not frame.extended and len(frame.data) >= 8 and frame.data[0] == expected_motor_id

        return self.wait_for_frame(matches, timeout_s, after_seq)

    def run_command(self, command: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        if not self.command_lock.acquire(blocking=False):
            if command in ("stop", "clear-fault"):
                try:
                    if command == "stop":
                        self.stop_and_disable()
                        message = "Stop sent while another command was running."
                    else:
                        self.clear_fault()
                        message = "Clear fault sent while another command was running."
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
            ok = self.reopen_bus(str(payload.get("interface") or self.bus.interface))
            return {"ok": ok}
        if not self.connected:
            opened = self.open_bus()
            if not opened:
                return {"ok": False, "message": self.open_error or "CAN is not open"}

        if command == "scan":
            self.scan_private()
            self.scan_mit()
            return {"ok": True}
        if command == "scan-private":
            self.scan_private()
            return {"ok": True}
        if command == "scan-mit":
            self.scan_mit()
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
            return {"ok": self.move_position(position, velocity_limit, acceleration)}
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
                old_transport = self.bus_transport()
                old_interface = self.bus.interface
                old_serial_port = getattr(self.bus, "serial_port", DEFAULT_SERIAL_PORT)
                old_serial_baud = getattr(self.bus, "serial_baud", DEFAULT_SERIAL_BAUD)
                old_protocol = self.protocol
                old_motor_id = self.motor_id
                old_host_id = self.host_id
                old_feedback_id = self.feedback_id
                old_model = self.model
                new_transport = str(payload.get("transport") or old_transport)
                if new_transport not in TRANSPORT_CHOICES:
                    new_transport = old_transport
                new_interface = str(payload.get("interface") or old_interface)
                new_serial_port = str(payload.get("serialPort") or old_serial_port)
                new_serial_baud = parse_int(payload.get("serialBaud"), old_serial_baud)
                new_protocol = str(payload.get("protocol") or old_protocol)
                if new_protocol not in (PROTOCOL_PRIVATE, PROTOCOL_MIT):
                    new_protocol = old_protocol
                new_motor_id = parse_int(payload.get("motorId"), old_motor_id) & 0xFF
                new_host_id = parse_int(payload.get("hostId"), old_host_id) & 0xFF
                new_feedback_id = parse_int(payload.get("feedbackId"), old_feedback_id) & CAN_SFF_MASK
                new_model = str(payload.get("model") or old_model).lower()
                bus_changed = (
                    new_transport != old_transport
                    or new_interface != old_interface
                    or new_serial_port != old_serial_port
                    or new_serial_baud != old_serial_baud
                )
                control_changed = (
                    new_protocol != old_protocol
                    or new_motor_id != old_motor_id
                    or new_host_id != old_host_id
                    or new_feedback_id != old_feedback_id
                    or new_model != old_model
                )
                if bus_changed or control_changed:
                    self.protocol = new_protocol
                    self.motor_id = new_motor_id
                    self.host_id = new_host_id
                    self.feedback_id = new_feedback_id
                    self.model = new_model
                    self.velocity_configured = False
                    self.position_configured = False
                    self.oscillating = False
                    self.jog_active = False
            if bus_changed:
                self.reopen_bus(
                    interface=new_interface,
                    transport=new_transport,
                    serial_port=new_serial_port,
                    serial_baud=new_serial_baud,
                )
            if bus_changed or control_changed:
                self.log(
                    f"Config transport={self.bus_transport()} bus={self.bus_label()} "
                    f"interface={self.bus.interface} protocol={self.protocol} "
                    f"motor={fmt_id(self.motor_id)} host={fmt_id(self.host_id)} feedback={fmt_id(self.feedback_id, 3)}"
                )
        return {"ok": True}

    def configure_velocity(self) -> bool:
        if self.protocol == PROTOCOL_MIT:
            return self.configure_mit_velocity()
        return self.configure_private_velocity()

    def clear_private_fault(self, label: str = "Private clear-error sent") -> None:
        self.send_private_disable(True)
        self.wait_private_status(0.30)
        with self.lock:
            self.last_private_fault = None
            self.last_private_fault_at = 0.0
        self.log(label)

    def prepare_private_mode_switch(self, label: str) -> None:
        # Some RobStride firmware ignores run_mode writes unless torque is
        # explicitly disabled first. Clear-error alone can still leave the
        # controller reporting the old run_mode.
        self.send_private_disable(False)
        self.wait_private_status(0.30)
        time.sleep(0.08)
        self.send_private_disable(True)
        self.wait_private_status(0.30)
        with self.lock:
            self.last_private_fault = None
            self.last_private_fault_at = 0.0
        self.log(label)
        time.sleep(0.08)

    def clear_fault(self) -> None:
        self.oscillating = False
        self.jog_active = False
        self.velocity_configured = False
        self.position_configured = False
        if self.protocol == PROTOCOL_MIT:
            self.send_mit_to_motor(mit_fault_query_payload(True))
            self.wait_mit_feedback(0.35)
            self.log("MIT clear-fault sent")
            return
        self.clear_private_fault()

    def configure_private_velocity(self) -> bool:
        self.velocity_configured = False
        self.position_configured = False
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

    def configure_mit_velocity(self) -> bool:
        self.velocity_configured = False
        self.position_configured = False
        self.oscillating = False
        self.jog_active = False
        self.log(
            f"Configuring MIT velocity motor={fmt_id(self.motor_id)} "
            f"feedback={fmt_id(self.feedback_id, 3)}"
        )
        start = self.current_seq()
        self.send_mit_to_motor(mit_set_mode_payload(RUN_MODE_VELOCITY))
        mode_fb = self.wait_mit_feedback(0.35, after_seq=start)
        self.log("MIT mode=2 " + ("ok" if mode_fb else "sent, no feedback"))
        time.sleep(0.06)

        start = self.current_seq()
        self.send_mit_to_motor(mit_special(0xFC))
        enable_fb = self.wait_mit_feedback(0.35, after_seq=start)
        self.log("MIT enable " + ("ok" if enable_fb else "sent, no feedback"))
        time.sleep(0.06)

        start = self.current_seq()
        self.commanded_speed = 0.0
        self.send_mit_velocity(0.0)
        self.wait_mit_feedback(0.25, after_seq=start)
        self.velocity_configured = True
        self.position_configured = False
        self.log("MIT velocity mode configured")
        return True

    def write_private_run_mode_verified(self, desired_mode: int) -> bool:
        original_host = self.host_id
        for host in self.private_host_candidates():
            self.host_id = host
            for attempt in range(1, 4):
                self.write_private_param_u8(PARAM_RUN_MODE, desired_mode)
                time.sleep(0.03)
                raw = self.wait_private_param(PARAM_RUN_MODE, COMMAND_TIMEOUT_S)
                if raw is None:
                    self.log(f"host={fmt_id(host)} attempt={attempt} run_mode timeout")
                    continue
                self.log(f"host={fmt_id(host)} attempt={attempt} run_mode readback={raw[0]}")
                if raw[0] == desired_mode:
                    return True
        self.host_id = original_host
        return False

    def move_position(self, position: float, velocity_limit: float, acceleration: float) -> bool:
        self.position_target = position
        self.position_velocity_limit = velocity_limit
        self.position_acceleration = acceleration
        if self.protocol == PROTOCOL_MIT:
            return self.move_mit_position(position, velocity_limit)
        return self.move_private_position(position, velocity_limit, acceleration)

    def move_private_position(self, position: float, velocity_limit: float, acceleration: float) -> bool:
        self.velocity_configured = False
        self.position_configured = False
        self.oscillating = False
        self.jog_active = False
        self.commanded_speed = 0.0
        self.log(
            f"Configuring private position motor={fmt_id(self.motor_id)} "
            f"host={fmt_id(self.host_id)} pos={position:+.3f} rad vlim={velocity_limit:.2f} rad/s"
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
        self.write_private_param_f32(PARAM_LOC_REF, position)
        self.position_configured = True
        self.log(
            f"private position target={position:+.3f} rad "
            f"vlim={velocity_limit:.2f} rad/s acc={acceleration:.1f} rad/s^2 sent"
        )
        return True

    def move_mit_position(self, position: float, velocity_limit: float) -> bool:
        self.velocity_configured = False
        self.position_configured = False
        self.oscillating = False
        self.jog_active = False
        self.commanded_speed = 0.0
        self.log(
            f"Configuring MIT position motor={fmt_id(self.motor_id)} "
            f"feedback={fmt_id(self.feedback_id, 3)} pos={position:+.3f} rad vlim={velocity_limit:.2f} rad/s"
        )

        start = self.current_seq()
        self.send_mit_to_motor(mit_set_mode_payload(RUN_MODE_POSITION))
        mode_fb = self.wait_mit_feedback(0.35, after_seq=start)
        self.log("MIT mode=1 " + ("ok" if mode_fb else "sent, no feedback"))
        time.sleep(0.06)

        start = self.current_seq()
        self.send_mit_to_motor(mit_special(0xFC))
        enable_fb = self.wait_mit_feedback(0.35, after_seq=start)
        self.log("MIT enable " + ("ok" if enable_fb else "sent, no feedback"))
        time.sleep(0.06)

        start = self.current_seq()
        self.send_mit_position(position, velocity_limit)
        self.wait_mit_feedback(0.35, after_seq=start)
        self.position_configured = True
        self.log(f"mit position target={position:+.3f} rad vlim={velocity_limit:.2f} rad/s sent")
        return True

    def private_host_candidates(self) -> List[int]:
        out: List[int] = []
        for host in (self.host_id, *PRIVATE_HOST_CANDIDATES):
            host &= 0xFF
            if host not in out:
                out.append(host)
        return out

    def send_velocity_target(self, speed: float) -> None:
        if self.protocol == PROTOCOL_MIT:
            self.send_mit_velocity(speed)
        else:
            self.write_private_param_f32(PARAM_SPD_REF, speed)
        self.last_velocity_refresh_at = time.monotonic()

    def set_speed(self, speed: float) -> bool:
        if not self.velocity_configured and not self.configure_velocity():
            return False
        self.commanded_speed = speed
        self.send_velocity_target(speed)
        self.position_configured = False
        self.log(f"{self.protocol} speed={speed:+.2f} rad/s sent")
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
                if self.protocol == PROTOCOL_MIT:
                    self.send_mit_velocity(0.0)
                else:
                    self.write_private_param_f32(PARAM_SPD_REF, 0.0)
                time.sleep(0.08)
            except Exception as exc:
                self.log(f"zero-speed before stop failed: {exc}")
        try:
            if self.protocol == PROTOCOL_MIT:
                self.send_mit_to_motor(mit_special(0xFD))
            else:
                self.send_private_disable(False)
        finally:
            self.velocity_configured = False
            self.position_configured = False
            self.commanded_speed = 0.0
            self.log("Stop/disable sent")

    def set_active_report(self, enabled: bool) -> None:
        if self.protocol == PROTOCOL_MIT:
            self.send_mit_to_motor(mit_active_report_payload(enabled))
        else:
            payload = bytes([1, 2, 3, 4, 5, 6, 1 if enabled else 0, 0])
            self.send_private(COMM_PROACTIVE_REPORT, self.host_id, self.motor_id, payload)
        self.active_reports = enabled
        self.log(f"Active reports {'on' if enabled else 'off'}")

    def request_status(self) -> None:
        if self.protocol == PROTOCOL_MIT:
            self.send_mit_to_motor(mit_fault_query_payload(False))
            self.wait_mit_feedback(0.4)
            self.log("MIT status queried")
            return
        for index, label in (
            (PARAM_RUN_MODE, "run_mode"),
            (PARAM_SPD_REF, "spd_ref"),
            (PARAM_LOC_REF, "loc_ref"),
            (PARAM_LIMIT_CUR, "limit_cur"),
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
            self.protocol = PROTOCOL_PRIVATE
            self.velocity_configured = False
            self.position_configured = False
            self.log(f"Private scan found {len(self.discovered_private)} motor(s); selected {fmt_id(self.motor_id)}")
        else:
            self.log("Private scan found no motors")

    def scan_mit(self) -> None:
        with self.lock:
            self.discovered_mit = []
        self.log(f"Scanning MIT IDs {fmt_id(SCAN_FIRST_MIT_ID)}..{fmt_id(SCAN_LAST_ID)}")
        for target_id in range(SCAN_FIRST_MIT_ID, SCAN_LAST_ID + 1):
            start = self.current_seq()
            self.send(target_id, mit_fault_query_payload(False), extended=False)

            def matches(frame: Any, wanted: int = target_id) -> bool:
                return not frame.extended and len(frame.data) >= 8 and frame.data[0] == wanted

            frame = self.wait_for_frame(matches, SCAN_PER_ID_TIMEOUT_S, start)
            if frame is not None and target_id not in self.discovered_mit:
                self.discovered_mit.append(target_id)
                self.feedback_id = frame.arbitration_id
                self.log(f"MIT scan hit motor={fmt_id(target_id)} feedback={fmt_id(frame.arbitration_id, 3)}")
        if self.discovered_mit:
            self.motor_id = self.discovered_mit[0]
            self.protocol = PROTOCOL_MIT
            self.velocity_configured = False
            self.position_configured = False
            self.log(f"MIT scan found {len(self.discovered_mit)} motor(s); selected {fmt_id(self.motor_id)}")
        else:
            self.log("MIT scan found no motors")

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

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            feedback = dict(self.last_feedback) if self.last_feedback else None
            if feedback and self.last_feedback_at:
                feedback["ageMs"] = int((time.monotonic() - self.last_feedback_at) * 1000)
            private_fault = dict(self.last_private_fault) if self.last_private_fault else None
            if private_fault and self.last_private_fault_at:
                private_fault["ageMs"] = int((time.monotonic() - self.last_private_fault_at) * 1000)
            interface = self.bus.interface
            transport = self.bus_transport()
            serial_port = getattr(self.bus, "serial_port", DEFAULT_SERIAL_PORT)
            serial_baud = getattr(self.bus, "serial_baud", DEFAULT_SERIAL_BAUD)
            bus_label = self.bus_label()
            bus_stats = self.bus.stats() if hasattr(self.bus, "stats") else can_stats(interface)
            snapshot = {
                "appVersion": APP_VERSION,
                "connected": self.connected,
                "openError": self.open_error,
                "transport": transport,
                "transportLabel": bus_label,
                "interface": interface,
                "serialPort": serial_port,
                "serialBaud": serial_baud,
                "protocol": self.protocol,
                "motorId": self.motor_id,
                "motorIdHex": fmt_id(self.motor_id),
                "hostId": self.host_id,
                "hostIdHex": fmt_id(self.host_id),
                "feedbackId": self.feedback_id,
                "feedbackIdHex": fmt_id(self.feedback_id, 3),
                "model": self.model,
                "testSpeed": self.test_speed,
                "commandedSpeed": self.commanded_speed,
                "positionTarget": self.position_target,
                "positionVelocityLimit": self.position_velocity_limit,
                "positionAcceleration": self.position_acceleration,
                "velocityConfigured": self.velocity_configured,
                "positionConfigured": self.position_configured,
                "activeReports": self.active_reports,
                "oscillating": self.oscillating,
                "jogActive": self.jog_active,
                "busy": self.busy,
                "lastFeedback": feedback,
                "lastPrivateFault": private_fault,
                "lastRawFrame": self.last_raw_frame,
                "discoveredPrivate": [fmt_id(item) for item in self.discovered_private],
                "discoveredMit": [fmt_id(item) for item in self.discovered_mit],
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
    parser.add_argument("--transport", choices=TRANSPORT_CHOICES, default=TRANSPORT_ROBSTRIDE_SERIAL)
    parser.add_argument("--interface", default=DEFAULT_INTERFACE)
    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--serial-baud", type=int, default=DEFAULT_SERIAL_BAUD)
    parser.add_argument("--motor-id", default=hex(DEFAULT_MOTOR_ID))
    parser.add_argument("--host-id", default=hex(DEFAULT_HOST_ID))
    parser.add_argument("--feedback-id", default=hex(DEFAULT_FEEDBACK_ID))
    parser.add_argument("--protocol", choices=(PROTOCOL_PRIVATE, PROTOCOL_MIT), default=PROTOCOL_PRIVATE)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--no-open", action="store_true", help="start dashboard without opening CAN")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    controller = DashboardController(
        transport=args.transport,
        interface=args.interface,
        serial_port=args.serial_port,
        serial_baud=args.serial_baud,
        motor_id=parse_int(args.motor_id, DEFAULT_MOTOR_ID),
        host_id=parse_int(args.host_id, DEFAULT_HOST_ID),
        feedback_id=parse_int(args.feedback_id, DEFAULT_FEEDBACK_ID),
        protocol=args.protocol,
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
