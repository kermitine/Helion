#!/usr/bin/env python3
"""RobStride bench-control tool for Raspberry Pi CAN adapters.

This is the Raspberry Pi rewrite of the ESP32 serial test sketch. It supports a
Linux SocketCAN interface such as can0 and the official RobStride USB-CAN serial
adapter that appears as a CH340 tty device.
"""

from __future__ import annotations

import argparse
import contextlib
import glob
import os
import select
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Any, Callable, Iterable, List, Optional, Tuple


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF
CAN_FRAME = struct.Struct("=IB3x8s")


DEFAULT_INTERFACE = "can0"
TRANSPORT_SOCKETCAN = "socketcan"
TRANSPORT_ROBSTRIDE_SERIAL = "robstride-serial"
TRANSPORT_CHOICES = (TRANSPORT_SOCKETCAN, TRANSPORT_ROBSTRIDE_SERIAL)
DEFAULT_SERIAL_PORT = "auto"
DEFAULT_SERIAL_BAUD = 921600
ROBSTRIDE_SERIAL_HEADER = b"AT"
ROBSTRIDE_SERIAL_TRAILER = b"\r\n"
ROBSTRIDE_SERIAL_EXTENDED_FLAG = 0x04
DEFAULT_MOTOR_ID = 0x7F
DEFAULT_HOST_ID = 0xFD
DEFAULT_FEEDBACK_ID = 0xFD
DEFAULT_MODEL = "rs-05"

SCAN_FIRST_PRIVATE_ID = 0x00
SCAN_FIRST_MIT_ID = 0x01
SCAN_LAST_ID = 0x7F
SCAN_PER_ID_TIMEOUT_S = 0.040

COMM_GET_ID = 0x00
COMM_OPERATION_CONTROL = 0x01
COMM_OPERATION_STATUS = 0x02
COMM_ENABLE = 0x03
COMM_DISABLE = 0x04
COMM_READ_PARAM = 0x11
COMM_WRITE_PARAM = 0x12
COMM_FAULT = 0x15
COMM_SAVE_PARAMS = 0x16
COMM_PROACTIVE_REPORT = 0x18
COMM_SET_PROTOCOL = 0x19

PARAM_RUN_MODE = 0x7005
PARAM_SPD_REF = 0x700A
PARAM_LOC_REF = 0x7016
PARAM_LIMIT_SPD = 0x7017
PARAM_LIMIT_CUR = 0x7018
PARAM_MECH_POS = 0x7019
PARAM_MECH_VEL = 0x701B
PARAM_VBUS = 0x701C
PARAM_ACC_RAD = 0x7022
PARAM_PP_VEL_MAX = 0x7024
PARAM_PP_ACC_SET = 0x7025

RUN_MODE_MIT = 0
RUN_MODE_POSITION = 1
RUN_MODE_VELOCITY = 2

MIT_TYPED_ID_POSITION = 1
MIT_TYPED_ID_VELOCITY = 2
MIT_TYPED_ID_PARAM_READ = 3
MIT_TYPED_ID_PARAM_WRITE = 4

PROTOCOL_PRIVATE = "private"
PROTOCOL_MIT = "mit"

DEFAULT_SPEED_RAD_S = 0.30
MOTOR_STUDIO_JOG_SPEED_RAD_S = 1.0
DEFAULT_POSITION_RAD = 0.0
DEFAULT_POSITION_VEL_RAD_S = 1.0
DEFAULT_POSITION_ACCEL_RAD_S2 = 10.0
DEFAULT_CURRENT_LIMIT_A = 1.00
DEFAULT_ACCEL_RAD_S2 = 5.0
OSCILLATION_PERIOD_S = 2.5
MOTOR_STUDIO_JOG_S = 0.75
RAW_TRACE_DURATION_S = 3.0
RAW_TRACE_FRAME_LIMIT = 32
STATUS_PRINT_PERIOD_S = 0.5

PRIVATE_HOST_CANDIDATES = (0xFD, 0xFF, 0xFE, 0x00, 0xAA)

PRIVATE_MODEL_LIMITS = {
    "rs-00": (4.0 * 3.141592653589793, 33.0, 14.0),
    "rs-01": (4.0 * 3.141592653589793, 44.0, 17.0),
    "rs-02": (4.0 * 3.141592653589793, 44.0, 17.0),
    "rs-03": (4.0 * 3.141592653589793, 20.0, 60.0),
    "rs-04": (4.0 * 3.141592653589793, 15.0, 120.0),
    "rs-05": (4.0 * 3.141592653589793, 50.0, 5.5),
    "rs-06": (4.0 * 3.141592653589793, 50.0, 36.0),
    "el-05": (4.0 * 3.141592653589793, 50.0, 6.0),
}

MIT_P_MIN = -12.57
MIT_P_MAX = 12.57
MIT_V_MIN = -33.0
MIT_V_MAX = 33.0
MIT_T_MIN = -14.0
MIT_T_MAX = 14.0


@dataclass
class CanFrame:
    arbitration_id: int
    data: bytes
    extended: bool


@dataclass
class MitFeedback:
    motor_id: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    mode_state: int
    has_fault: bool
    has_warning: bool
    winding_temp_c: float
    feedback_id: int


@dataclass
class PrivateFaultReport:
    fault_raw: int
    warning_raw: int
    fault_names: List[str]
    warning_names: List[str]


PRIVATE_FAULT_BITS = (
    (0, "overtemperature"),
    (1, "driver_fault"),
    (2, "undervoltage"),
    (3, "overvoltage"),
    (4, "phase_b_overcurrent"),
    (5, "phase_c_overcurrent"),
    (7, "encoder_uncalibrated"),
    (8, "hardware_id_fault"),
    (9, "position_init_fault"),
    (14, "stall_overload"),
    (16, "phase_a_overcurrent"),
)

PRIVATE_WARNING_BITS = (
    (0, "overtemperature_warning"),
)


def parse_id(raw: str) -> int:
    return int(raw, 0)


def fmt_id(value: int, width: int = 2) -> str:
    return f"0x{value:0{width}X}"


def bytes_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def detect_serial_port() -> str:
    patterns = (
        "/dev/serial/by-id/*",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
    )
    candidates: List[str] = []
    for pattern in patterns:
        candidates.extend(sorted(glob.glob(pattern)))

    if not candidates:
        raise RuntimeError(
            "could not find a serial adapter. Plug in the RobStride USB-CAN adapter "
            "or set Serial Port to its device path."
        )

    preferred_tokens = ("robstride", "usb-can", "ch340", "ch341", "qinheng", "1a86")
    for candidate in candidates:
        name = candidate.lower()
        if any(token in name for token in preferred_tokens):
            return candidate
    return candidates[0]


def f32_le(value: float) -> bytes:
    return struct.pack("<f", float(value))


def read_f32_le(data: bytes) -> float:
    return struct.unpack("<f", data[:4])[0]


def uint_to_float(value: int, min_value: float, max_value: float, bits: int) -> float:
    return float(value) * (max_value - min_value) / float((1 << bits) - 1) + min_value


def build_private_ext_id(comm_type: int, extra_data: int, node_id: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((extra_data & 0xFFFF) << 8) | (node_id & 0xFF)


def split_private_ext_id(arbitration_id: int) -> Tuple[int, int, int]:
    return (
        (arbitration_id >> 24) & 0x1F,
        (arbitration_id >> 8) & 0xFFFF,
        arbitration_id & 0xFF,
    )


def robstride_serial_pack_id(arbitration_id: int, extended: bool) -> int:
    mask = CAN_EFF_MASK if extended else CAN_SFF_MASK
    packed = (arbitration_id & mask) << 3
    if extended:
        packed |= ROBSTRIDE_SERIAL_EXTENDED_FLAG
    return packed & 0xFFFFFFFF


def robstride_serial_encode_frame(arbitration_id: int, data: bytes, extended: bool) -> bytes:
    payload = bytes(data[:8]).ljust(8, b"\x00")
    packed_id = robstride_serial_pack_id(arbitration_id, extended)
    return (
        ROBSTRIDE_SERIAL_HEADER
        + packed_id.to_bytes(4, "big")
        + bytes([len(payload)])
        + payload
        + ROBSTRIDE_SERIAL_TRAILER
    )


def robstride_serial_try_parse(buffer: bytearray) -> Optional[CanFrame]:
    while buffer:
        start = buffer.find(ROBSTRIDE_SERIAL_HEADER)
        if start < 0:
            keep_last_a = len(buffer) > 0 and buffer[-1:] == ROBSTRIDE_SERIAL_HEADER[:1]
            if keep_last_a:
                del buffer[:-1]
            else:
                del buffer[:]
            return None
        if start:
            del buffer[:start]
        if len(buffer) < 7:
            return None

        dlc = buffer[6]
        if dlc > 8:
            del buffer[0]
            continue

        frame_len = 2 + 4 + 1 + dlc + 2
        if len(buffer) < frame_len:
            return None
        if buffer[frame_len - 2 : frame_len] != ROBSTRIDE_SERIAL_TRAILER:
            del buffer[0]
            continue

        packed_id = int.from_bytes(buffer[2:6], "big")
        extended = (packed_id & ROBSTRIDE_SERIAL_EXTENDED_FLAG) != 0
        mask = CAN_EFF_MASK if extended else CAN_SFF_MASK
        arbitration_id = (packed_id >> 3) & mask
        data = bytes(buffer[7 : 7 + dlc])
        del buffer[:frame_len]
        return CanFrame(arbitration_id, data, extended)

    return None


def mit_typed_id(mode_type: int, motor_id: int) -> int:
    return ((mode_type & 0x07) << 8) | (motor_id & 0xFF)


def mit_special(command_byte: int) -> bytes:
    return bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command_byte & 0xFF])


def mit_set_mode_payload(mode: int) -> bytes:
    return bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, mode & 0xFF, 0xFC])


def mit_active_report_payload(enabled: bool) -> bytes:
    return bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 1 if enabled else 0, 0xF9])


def mit_fault_query_payload(clear_fault: bool = False) -> bytes:
    return bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF if clear_fault else 0x00, 0xFB])


def mit_velocity_payload(velocity_rad_s: float, current_limit_a: float) -> bytes:
    return f32_le(velocity_rad_s) + f32_le(abs(current_limit_a))


def mit_position_payload(position_rad: float, velocity_limit_rad_s: float) -> bytes:
    return f32_le(position_rad) + f32_le(abs(velocity_limit_rad_s))


def names_from_bits(raw: int, bits: Iterable[Tuple[int, str]]) -> List[str]:
    return [name for bit, name in bits if raw & (1 << bit)]


def decode_private_fault_payload(data: bytes) -> PrivateFaultReport:
    payload = bytes(data[:8]).ljust(8, b"\x00")
    fault_raw = int.from_bytes(payload[0:4], "little", signed=False)
    warning_raw = int.from_bytes(payload[4:8], "little", signed=False)
    return PrivateFaultReport(
        fault_raw=fault_raw,
        warning_raw=warning_raw,
        fault_names=names_from_bits(fault_raw, PRIVATE_FAULT_BITS),
        warning_names=names_from_bits(warning_raw, PRIVATE_WARNING_BITS),
    )


def private_fault_summary(report: PrivateFaultReport) -> str:
    faults = ",".join(report.fault_names) if report.fault_names else "none"
    warnings = ",".join(report.warning_names) if report.warning_names else "none"
    return (
        f"fault_raw=0x{report.fault_raw:08X} [{faults}] "
        f"warning_raw=0x{report.warning_raw:08X} [{warnings}]"
    )


def decode_mit_feedback(frame: CanFrame) -> Optional[MitFeedback]:
    if frame.extended or len(frame.data) < 8:
        return None

    data = frame.data
    pos_u = (data[1] << 8) | data[2]
    vel_u = (data[3] << 4) | (data[4] >> 4)
    torque_u = ((data[4] & 0x0F) << 8) | data[5]
    temp_u = ((data[6] & 0x0F) << 8) | data[7]
    return MitFeedback(
        motor_id=data[0],
        position_rad=uint_to_float(pos_u, MIT_P_MIN, MIT_P_MAX, 16),
        velocity_rad_s=uint_to_float(vel_u, MIT_V_MIN, MIT_V_MAX, 12),
        torque_nm=uint_to_float(torque_u, MIT_T_MIN, MIT_T_MAX, 12),
        mode_state=data[6] >> 6,
        has_fault=(data[6] & 0x20) != 0,
        has_warning=(data[6] & 0x10) != 0,
        winding_temp_c=float(temp_u) / 10.0,
        feedback_id=frame.arbitration_id,
    )


class SocketCanBus:
    def __init__(self, interface: str):
        self.interface = interface
        self.transport = TRANSPORT_SOCKETCAN
        self.serial_port = DEFAULT_SERIAL_PORT
        self.serial_baud = DEFAULT_SERIAL_BAUD
        self.sock: Optional[socket.socket] = None

    def open(self) -> None:
        try:
            sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.bind((self.interface,))
        except AttributeError as exc:
            raise RuntimeError("SocketCAN is only available on Linux") from exc
        except OSError as exc:
            raise RuntimeError(
                f"could not open {self.interface}: {exc}. "
                "Bring the interface up at 1 Mbps first."
            ) from exc
        self.sock = sock

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def reopen(self) -> None:
        self.close()
        time.sleep(0.05)
        self.open()

    def label(self) -> str:
        return f"{self.interface} at 1 Mbps"

    def stats(self) -> dict[str, str]:
        base = f"/sys/class/net/{self.interface}"
        stats_dir = os.path.join(base, "statistics")
        out: dict[str, str] = {}
        for name in ("operstate", "carrier"):
            raw = read_text_file(os.path.join(base, name))
            if raw:
                out[name] = raw
        for name in ("rx_packets", "tx_packets", "rx_errors", "tx_errors", "rx_dropped", "tx_dropped"):
            raw = read_text_file(os.path.join(stats_dir, name))
            if raw:
                out[name] = raw
        return out

    def send(self, arbitration_id: int, data: bytes, extended: bool = False) -> None:
        if self.sock is None:
            raise RuntimeError("CAN socket is not open")
        payload = bytes(data[:8]).ljust(8, b"\x00")
        can_id = arbitration_id & (CAN_EFF_MASK if extended else CAN_SFF_MASK)
        if extended:
            can_id |= CAN_EFF_FLAG
        packet = CAN_FRAME.pack(can_id, len(payload), payload)
        self.sock.send(packet)

    def recv(self, timeout_s: float) -> Optional[CanFrame]:
        if self.sock is None:
            raise RuntimeError("CAN socket is not open")
        ready, _, _ = select.select([self.sock], [], [], max(0.0, timeout_s))
        if not ready:
            return None
        packet = self.sock.recv(CAN_FRAME.size)
        can_id, dlc, data = CAN_FRAME.unpack(packet)
        if can_id & CAN_ERR_FLAG:
            return None
        extended = (can_id & CAN_EFF_FLAG) != 0
        mask = CAN_EFF_MASK if extended else CAN_SFF_MASK
        return CanFrame(can_id & mask, data[: min(dlc, 8)], extended)


class RobStrideSerialBus:
    """Official RobStride USB-CAN adapter transport over its CH340 serial port."""

    def __init__(self, serial_port: str, serial_baud: int, interface: str = DEFAULT_INTERFACE):
        self.interface = interface
        self.transport = TRANSPORT_ROBSTRIDE_SERIAL
        self.serial_port = serial_port
        self.active_serial_port = ""
        self.serial_baud = int(serial_baud)
        self.fd: Optional[int] = None
        self.rx_buffer = bytearray()
        self.rx_packets = 0
        self.tx_packets = 0
        self.rx_errors = 0
        self.tx_errors = 0
        self.rx_dropped = 0
        self.tx_dropped = 0
        self._recent_tx: List[Tuple[float, int, bool, bytes]] = []

    def open(self) -> None:
        if os.name != "posix":
            raise RuntimeError("direct RobStride serial transport is only available on Linux")
        try:
            port = detect_serial_port() if self.serial_port in ("", "auto") else self.serial_port
        except RuntimeError as exc:
            raise RuntimeError(f"{exc} Looked for /dev/serial/by-id/*, /dev/ttyUSB*, and /dev/ttyACM*.") from exc

        try:
            fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            raise RuntimeError(
                f"could not open {port}: {exc}. "
                "Stop slcand if it is running, check the adapter path, and make sure the dashboard user is in the dialout group."
            ) from exc

        try:
            self._configure_serial(fd)
        except Exception as exc:
            os.close(fd)
            raise RuntimeError(f"could not configure {port} at {self.serial_baud} baud: {exc}") from exc

        self.fd = fd
        self.active_serial_port = port
        self.rx_buffer.clear()
        self._recent_tx.clear()

    def _configure_serial(self, fd: int) -> None:
        import termios

        baud_const = getattr(termios, f"B{self.serial_baud}", None)
        if baud_const is None:
            supported = "115200, 230400, 460800, 500000, 576000, 921600, 1000000, 1500000, 2000000"
            raise RuntimeError(f"unsupported baud {self.serial_baud}; try one of: {supported}")

        attrs = termios.tcgetattr(fd)
        attrs[0] = getattr(termios, "IGNPAR", 0)
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[2] &= ~getattr(termios, "CRTSCTS", 0)
        attrs[3] = 0
        attrs[4] = baud_const
        attrs[5] = baud_const
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 1
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def reopen(self) -> None:
        self.close()
        time.sleep(0.05)
        self.open()

    def label(self) -> str:
        if self.serial_port in ("", "auto"):
            port = self.active_serial_port or "auto"
            return f"{port} at {self.serial_baud} baud"
        return f"{self.serial_port} at {self.serial_baud} baud"

    def stats(self) -> dict[str, str]:
        return {
            "operstate": "up" if self.fd is not None else "down",
            "carrier": "1" if self.fd is not None else "0",
            "device": self.active_serial_port or self.serial_port,
            "rx_packets": str(self.rx_packets),
            "tx_packets": str(self.tx_packets),
            "rx_errors": str(self.rx_errors),
            "tx_errors": str(self.tx_errors),
            "rx_dropped": str(self.rx_dropped),
            "tx_dropped": str(self.tx_dropped),
        }

    def _write_all(self, data: bytes) -> None:
        if self.fd is None:
            raise RuntimeError("RobStride serial port is not open")

        remaining = memoryview(data)
        deadline = time.monotonic() + 0.5
        while remaining:
            try:
                written = os.write(self.fd, remaining)
            except BlockingIOError:
                written = 0
            except OSError:
                self.tx_errors += 1
                raise

            if written:
                remaining = remaining[written:]
                continue

            wait_s = deadline - time.monotonic()
            if wait_s <= 0:
                self.tx_errors += 1
                raise OSError("serial write timeout")
            select.select([], [self.fd], [], min(0.05, wait_s))

    def _remember_tx(self, arbitration_id: int, data: bytes, extended: bool) -> None:
        now = time.monotonic()
        self._recent_tx = [item for item in self._recent_tx if now - item[0] < 0.5]
        self._recent_tx.append((now, arbitration_id, extended, data))
        del self._recent_tx[:-32]

    def _is_echo(self, frame: CanFrame) -> bool:
        now = time.monotonic()
        kept: List[Tuple[float, int, bool, bytes]] = []
        matched = False
        for item in self._recent_tx:
            stamp, arbitration_id, extended, data = item
            if now - stamp >= 0.5:
                continue
            if (
                not matched
                and arbitration_id == frame.arbitration_id
                and extended == frame.extended
                and data[: len(frame.data)] == frame.data
            ):
                matched = True
                continue
            kept.append(item)
        self._recent_tx = kept
        return matched

    def send(self, arbitration_id: int, data: bytes, extended: bool = False) -> None:
        payload = bytes(data[:8]).ljust(8, b"\x00")
        self._write_all(robstride_serial_encode_frame(arbitration_id, payload, extended))
        self.tx_packets += 1
        self._remember_tx(arbitration_id & (CAN_EFF_MASK if extended else CAN_SFF_MASK), payload, extended)

    def recv(self, timeout_s: float) -> Optional[CanFrame]:
        if self.fd is None:
            raise RuntimeError("RobStride serial port is not open")

        deadline = time.monotonic() + max(0.0, timeout_s)
        while True:
            frame = robstride_serial_try_parse(self.rx_buffer)
            if frame is not None:
                if self._is_echo(frame):
                    continue
                self.rx_packets += 1
                return frame

            remaining = max(0.0, deadline - time.monotonic())
            ready, _, _ = select.select([self.fd], [], [], remaining)
            if not ready:
                return None
            try:
                chunk = os.read(self.fd, 512)
            except BlockingIOError:
                continue
            except OSError:
                self.rx_errors += 1
                raise
            if not chunk:
                continue

            self.rx_buffer.extend(chunk)
            if len(self.rx_buffer) > 4096:
                del self.rx_buffer[:-512]
                self.rx_dropped += 1


def create_bus(
    transport: str,
    interface: str = DEFAULT_INTERFACE,
    serial_port: str = DEFAULT_SERIAL_PORT,
    serial_baud: int = DEFAULT_SERIAL_BAUD,
) -> Any:
    if transport == TRANSPORT_ROBSTRIDE_SERIAL:
        return RobStrideSerialBus(serial_port=serial_port, serial_baud=serial_baud, interface=interface)
    if transport == TRANSPORT_SOCKETCAN:
        return SocketCanBus(interface)
    raise ValueError(f"unknown CAN transport: {transport}")


class RobStrideSocketCanTool:
    def __init__(
        self,
        bus: Any,
        protocol: str,
        motor_id: int,
        host_id: int,
        feedback_id: int,
        model: str,
        accept_any_feedback_id: bool,
        scan_per_id_timeout_s: float,
    ):
        self.bus = bus
        self.protocol = protocol
        self.motor_id = motor_id & 0xFF
        self.host_id = host_id & 0xFF
        self.feedback_id = feedback_id & CAN_SFF_MASK
        self.model = model.lower()
        self.accept_any_feedback_id = accept_any_feedback_id
        self.scan_per_id_timeout_s = scan_per_id_timeout_s

        self.velocity_mode_configured = False
        self.oscillating = False
        self.jog_active = False
        self.active_reports = False
        self.test_speed = DEFAULT_SPEED_RAD_S
        self.commanded_speed = 0.0
        self.jog_stop_at = 0.0
        self.last_oscillation_at = time.monotonic()
        self.last_feedback_at = 0.0
        self.last_status_print_at = 0.0
        self.raw_trace_until = 0.0
        self.raw_trace_frames = 0

    def print_help(self) -> None:
        print()
        print("RobStride Raspberry Pi bench test")
        print("Motor will not move until you command f/b/g/< />.")
        print(
            "transport={transport} bus={bus} motor={motor} host={host} feedback={feedback} "
            "protocol={protocol} speed={speed:.2f} rad/s".format(
                transport=getattr(self.bus, "transport", TRANSPORT_SOCKETCAN),
                bus=self.bus.label() if hasattr(self.bus, "label") else self.bus.interface,
                motor=fmt_id(self.motor_id),
                host=fmt_id(self.host_id),
                feedback=fmt_id(self.feedback_id),
                protocol=self.protocol,
                speed=self.test_speed,
            )
        )
        print()
        print("Commands:")
        print("  p  scan current protocol for motors")
        print("  P  scan both private and MIT-standard protocols")
        print("  v  configure velocity mode only")
        print("  f  forward at test speed")
        print("  b  backward at test speed")
        print("  <  Motor Studio JOG- left at 1.0 rad/s")
        print("  >  Motor Studio JOG+ right at 1.0 rad/s")
        print("  g  toggle forward/back oscillation")
        print("  0  set speed to zero, keep enabled")
        print("  s  set speed zero and disable motor")
        print("  e  clear latched motor fault")
        print("  +  increase test speed by 0.10 rad/s")
        print("  -  decrease test speed by 0.10 rad/s")
        print("  r  read private params or query MIT status")
        print("  a  toggle active motor status reports")
        print("  m  toggle protocol: private / MotorBridge MIT-standard")
        print("  x  print raw CAN frames for 3 seconds")
        print("  d  print adapter counters")
        print("  c  close and reopen CAN transport")
        print("  h  cycle private host ID: FD, FF, FE, 00, AA")
        print("  t  run local frame-encoding self-test")
        print("  q  quit")
        print("  ?  show this help")
        print()

    def private_limits(self) -> Tuple[float, float, float]:
        return PRIVATE_MODEL_LIMITS.get(self.model, PRIVATE_MODEL_LIMITS[DEFAULT_MODEL])

    def private_host_candidates(self) -> List[int]:
        candidates = [self.host_id]
        candidates.extend(PRIVATE_HOST_CANDIDATES)
        out: List[int] = []
        for candidate in candidates:
            candidate &= 0xFF
            if candidate not in out:
                out.append(candidate)
        return out

    def drain_rx(self) -> None:
        while self.bus.recv(0.0) is not None:
            pass

    def send_private(self, comm_type: int, extra_data: int, target_id: int, data: bytes) -> None:
        can_id = build_private_ext_id(comm_type, extra_data, target_id)
        self.bus.send(can_id, data, extended=True)

    def send_private_get_id(self, target_id: int) -> None:
        self.send_private(COMM_GET_ID, self.host_id, target_id, bytes(8))

    def send_private_disable(self, clear_error: bool = False) -> None:
        payload = bytearray(8)
        payload[0] = 1 if clear_error else 0
        self.send_private(COMM_DISABLE, self.host_id, self.motor_id, bytes(payload))

    def clear_private_fault(self) -> None:
        self.send_private_disable(True)
        self.wait_private_status(0.30)
        print("Private clear-error sent.")

    def clear_fault(self) -> None:
        self.oscillating = False
        self.jog_active = False
        self.velocity_mode_configured = False
        if self.protocol == PROTOCOL_MIT:
            self.send_mit_fault_query(self.motor_id, clear_fault=True)
            feedback = self.wait_mit_feedback(0.35)
            print(f"MIT clear-fault {'ok' if feedback else 'sent, no feedback'}.")
        else:
            self.clear_private_fault()

    def send_private_enable(self) -> None:
        self.send_private(COMM_ENABLE, self.host_id, self.motor_id, bytes(8))

    def write_private_param_u8(self, index: int, value: int) -> None:
        payload = bytearray(8)
        payload[0:2] = struct.pack("<H", index & 0xFFFF)
        payload[4] = value & 0xFF
        self.send_private(COMM_WRITE_PARAM, self.host_id, self.motor_id, bytes(payload))

    def write_private_param_f32(self, index: int, value: float) -> None:
        payload = bytearray(8)
        payload[0:2] = struct.pack("<H", index & 0xFFFF)
        payload[4:8] = f32_le(value)
        self.send_private(COMM_WRITE_PARAM, self.host_id, self.motor_id, bytes(payload))

    def read_private_param(self, index: int) -> None:
        payload = bytearray(8)
        payload[0:2] = struct.pack("<H", index & 0xFFFF)
        self.send_private(COMM_READ_PARAM, self.host_id, self.motor_id, bytes(payload))

    def wait_for_frame(
        self,
        predicate: Callable[[CanFrame], bool],
        timeout_s: float,
    ) -> Optional[CanFrame]:
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            frame = self.bus.recv(remaining)
            if frame is None:
                return None
            if predicate(frame):
                return frame
            self.handle_frame(frame)

    def wait_private_status(self, timeout_s: float) -> Optional[CanFrame]:
        def matches(frame: CanFrame) -> bool:
            if not frame.extended:
                return False
            comm_type, extra, _host = split_private_ext_id(frame.arbitration_id)
            return comm_type == COMM_OPERATION_STATUS and (extra & 0xFF) == self.motor_id

        return self.wait_for_frame(matches, timeout_s)

    def wait_private_param(self, index: int, timeout_s: float) -> Optional[bytes]:
        self.read_private_param(index)

        def matches(frame: CanFrame) -> bool:
            if not frame.extended or len(frame.data) < 8:
                return False
            comm_type, extra, _host = split_private_ext_id(frame.arbitration_id)
            if comm_type != COMM_READ_PARAM or (extra & 0xFF) != self.motor_id:
                return False
            return struct.unpack("<H", frame.data[0:2])[0] == index

        frame = self.wait_for_frame(matches, timeout_s)
        return None if frame is None else frame.data[4:8]

    def send_mit_to_motor(self, payload: bytes) -> None:
        self.bus.send(self.motor_id, payload, extended=False)

    def send_mit_set_mode(self, mode: int) -> None:
        self.send_mit_to_motor(mit_set_mode_payload(mode))

    def send_mit_enable(self) -> None:
        self.send_mit_to_motor(mit_special(0xFC))

    def send_mit_stop(self) -> None:
        self.send_mit_to_motor(mit_special(0xFD))

    def send_mit_active_report(self, enabled: bool) -> None:
        self.send_mit_to_motor(mit_active_report_payload(enabled))

    def send_mit_fault_query(self, target_id: int, clear_fault: bool = False) -> None:
        self.bus.send(target_id, mit_fault_query_payload(clear_fault), extended=False)

    def send_mit_velocity(self, velocity_rad_s: float, current_limit_a: float) -> None:
        self.bus.send(
            mit_typed_id(MIT_TYPED_ID_VELOCITY, self.motor_id),
            mit_velocity_payload(velocity_rad_s, current_limit_a),
            extended=False,
        )

    def send_mit_position(self, position_rad: float, velocity_limit_rad_s: float) -> None:
        self.bus.send(
            mit_typed_id(MIT_TYPED_ID_POSITION, self.motor_id),
            mit_position_payload(position_rad, velocity_limit_rad_s),
            extended=False,
        )

    def wait_mit_feedback_frame(self, timeout_s: float, motor_id: Optional[int] = None) -> Optional[CanFrame]:
        expected_motor_id = self.motor_id if motor_id is None else motor_id

        def matches(frame: CanFrame) -> bool:
            if frame.extended or len(frame.data) < 8 or frame.data[0] != expected_motor_id:
                return False
            return self.accept_any_feedback_id or frame.arbitration_id == self.feedback_id

        frame = self.wait_for_frame(matches, timeout_s)
        if frame is not None and frame.arbitration_id != self.feedback_id:
            print(f"Learned MIT feedback ID {fmt_id(frame.arbitration_id)}")
            self.feedback_id = frame.arbitration_id
        return frame

    def wait_mit_feedback(self, timeout_s: float, motor_id: Optional[int] = None) -> Optional[MitFeedback]:
        frame = self.wait_mit_feedback_frame(timeout_s, motor_id)
        if frame is None:
            return None
        feedback = decode_mit_feedback(frame)
        return feedback

    def configure_private_velocity(self) -> bool:
        print("Configuring private extended-ID velocity mode:")
        print(
            f"  motor={fmt_id(self.motor_id)} host={fmt_id(self.host_id)} "
            f"limit_cur={DEFAULT_CURRENT_LIMIT_A:.2f}A acc={DEFAULT_ACCEL_RAD_S2:.1f} rad/s^2"
        )
        self.oscillating = False
        self.jog_active = False
        self.velocity_mode_configured = False
        self.protocol = PROTOCOL_PRIVATE

        try:
            self.send_private_disable(True)
            status = self.wait_private_status(0.30)
            print(f"  clear-error    {'ok' if status else 'sent, no status'}")
            time.sleep(0.06)

            verified = False
            original_host = self.host_id
            for host in self.private_host_candidates():
                self.host_id = host
                for attempt in range(1, 4):
                    self.write_private_param_u8(PARAM_RUN_MODE, RUN_MODE_VELOCITY)
                    time.sleep(0.03)
                    raw = self.wait_private_param(PARAM_RUN_MODE, 0.50)
                    if raw is None:
                        print(f"    host={fmt_id(host)} attempt={attempt} run_mode readback timeout")
                        continue
                    actual = raw[0]
                    print(f"    host={fmt_id(host)} attempt={attempt} run_mode readback={actual}")
                    if actual == RUN_MODE_VELOCITY:
                        verified = True
                        break
                if verified:
                    break
            if not verified:
                self.host_id = original_host
                print("  run_mode=2     FAILED")
                return False

            print("  run_mode=2     ok")
            self.send_private_enable()
            print(f"  enable         {'ok' if self.wait_private_status(0.30) else 'sent, no status'}")
            time.sleep(0.08)
            self.commanded_speed = 0.0
            self.write_private_param_f32(PARAM_SPD_REF, 0.0)
            print("  spd_ref=0      sent")
            time.sleep(0.04)
            self.write_private_param_f32(PARAM_LIMIT_CUR, DEFAULT_CURRENT_LIMIT_A)
            print("  limit_cur      sent")
            time.sleep(0.04)
            self.write_private_param_f32(PARAM_ACC_RAD, DEFAULT_ACCEL_RAD_S2)
            print("  acc_rad        sent")
        except OSError as exc:
            print(f"  CAN TX failed: {exc}")
            return False

        self.velocity_mode_configured = True
        print("Private velocity mode configured.")
        return True

    def configure_mit_velocity(self) -> bool:
        print("Configuring MotorBridge MIT-standard velocity mode:")
        print(
            f"  motor={fmt_id(self.motor_id)} feedback={fmt_id(self.feedback_id)} "
            f"limit_cur={DEFAULT_CURRENT_LIMIT_A:.2f}A"
        )
        self.oscillating = False
        self.jog_active = False
        self.velocity_mode_configured = False
        self.protocol = PROTOCOL_MIT

        try:
            self.send_mit_set_mode(RUN_MODE_VELOCITY)
            mode_feedback = self.wait_mit_feedback(0.35)
            print(f"  mit mode=2     {'ok' if mode_feedback else 'sent, no feedback'}")

            time.sleep(0.06)
            self.send_mit_enable()
            enable_feedback = self.wait_mit_feedback(0.35)
            print(f"  mit enable     {'ok' if enable_feedback else 'sent, no feedback'}")

            time.sleep(0.06)
            self.commanded_speed = 0.0
            self.send_mit_velocity(0.0, DEFAULT_CURRENT_LIMIT_A)
            zero_feedback = self.wait_mit_feedback(0.25)
            print(f"  mit vel=0      {'ok' if zero_feedback else 'sent'}")
        except OSError as exc:
            print(f"  CAN TX failed: {exc}")
            return False

        self.velocity_mode_configured = True
        print("MotorBridge MIT velocity mode configured.")
        return True

    def configure_velocity_mode(self) -> bool:
        if self.protocol == PROTOCOL_MIT:
            return self.configure_mit_velocity()
        return self.configure_private_velocity()

    def move_position(
        self,
        position_rad: float,
        velocity_limit_rad_s: float = DEFAULT_POSITION_VEL_RAD_S,
        acceleration_rad_s2: float = DEFAULT_POSITION_ACCEL_RAD_S2,
    ) -> bool:
        if self.protocol == PROTOCOL_MIT:
            return self.move_mit_position(position_rad, velocity_limit_rad_s)
        return self.move_private_position(position_rad, velocity_limit_rad_s, acceleration_rad_s2)

    def move_private_position(
        self,
        position_rad: float,
        velocity_limit_rad_s: float,
        acceleration_rad_s2: float,
    ) -> bool:
        velocity_limit = abs(velocity_limit_rad_s) or DEFAULT_POSITION_VEL_RAD_S
        acceleration = abs(acceleration_rad_s2) or DEFAULT_POSITION_ACCEL_RAD_S2

        print("Configuring private extended-ID position mode:")
        print(
            f"  motor={fmt_id(self.motor_id)} host={fmt_id(self.host_id)} "
            f"pos={position_rad:+.3f} rad vlim={velocity_limit:.2f} rad/s acc={acceleration:.1f} rad/s^2"
        )
        self.oscillating = False
        self.jog_active = False
        self.velocity_mode_configured = False
        self.protocol = PROTOCOL_PRIVATE

        try:
            self.send_private_disable(True)
            status = self.wait_private_status(0.30)
            print(f"  clear-error    {'ok' if status else 'sent, no status'}")
            time.sleep(0.06)

            verified = False
            original_host = self.host_id
            for host in self.private_host_candidates():
                self.host_id = host
                for attempt in range(1, 4):
                    self.write_private_param_u8(PARAM_RUN_MODE, RUN_MODE_POSITION)
                    time.sleep(0.03)
                    raw = self.wait_private_param(PARAM_RUN_MODE, 0.50)
                    if raw is None:
                        print(f"    host={fmt_id(host)} attempt={attempt} run_mode readback timeout")
                        continue
                    actual = raw[0]
                    print(f"    host={fmt_id(host)} attempt={attempt} run_mode readback={actual}")
                    if actual == RUN_MODE_POSITION:
                        verified = True
                        break
                if verified:
                    break
            if not verified:
                self.host_id = original_host
                print("  run_mode=1     FAILED")
                return False

            print("  run_mode=1     ok")
            self.send_private_enable()
            print(f"  enable         {'ok' if self.wait_private_status(0.30) else 'sent, no status'}")
            time.sleep(0.08)
            self.write_private_param_f32(PARAM_LIMIT_CUR, DEFAULT_CURRENT_LIMIT_A)
            print("  limit_cur      sent")
            time.sleep(0.04)
            self.write_private_param_f32(PARAM_PP_VEL_MAX, velocity_limit)
            print("  vel_max        sent")
            time.sleep(0.04)
            self.write_private_param_f32(PARAM_PP_ACC_SET, acceleration)
            print("  acc_set        sent")
            time.sleep(0.04)
            self.write_private_param_f32(PARAM_LOC_REF, position_rad)
            print("  loc_ref        sent")
        except OSError as exc:
            print(f"  CAN TX failed: {exc}")
            return False

        self.commanded_speed = 0.0
        print("Private position target sent.")
        return True

    def move_mit_position(self, position_rad: float, velocity_limit_rad_s: float) -> bool:
        velocity_limit = abs(velocity_limit_rad_s) or DEFAULT_POSITION_VEL_RAD_S
        print("Configuring MotorBridge MIT-standard position mode:")
        print(
            f"  motor={fmt_id(self.motor_id)} feedback={fmt_id(self.feedback_id)} "
            f"pos={position_rad:+.3f} rad vlim={velocity_limit:.2f} rad/s"
        )
        self.oscillating = False
        self.jog_active = False
        self.velocity_mode_configured = False
        self.protocol = PROTOCOL_MIT

        try:
            self.send_mit_set_mode(RUN_MODE_POSITION)
            mode_feedback = self.wait_mit_feedback(0.35)
            print(f"  mit mode=1     {'ok' if mode_feedback else 'sent, no feedback'}")

            time.sleep(0.06)
            self.send_mit_enable()
            enable_feedback = self.wait_mit_feedback(0.35)
            print(f"  mit enable     {'ok' if enable_feedback else 'sent, no feedback'}")

            time.sleep(0.06)
            self.send_mit_position(position_rad, velocity_limit)
            pos_feedback = self.wait_mit_feedback(0.35)
            print(f"  mit pos        {'ok' if pos_feedback else 'sent'}")
        except OSError as exc:
            print(f"  CAN TX failed: {exc}")
            return False

        self.commanded_speed = 0.0
        print("MotorBridge MIT position target sent.")
        return True

    def send_speed_target(self, speed_rad_s: float) -> bool:
        try:
            if self.protocol == PROTOCOL_MIT:
                self.send_mit_velocity(speed_rad_s, DEFAULT_CURRENT_LIMIT_A)
                feedback = self.wait_mit_feedback(0.10)
                if feedback:
                    self.print_mit_feedback(feedback, prefix="  ")
            else:
                self.write_private_param_f32(PARAM_SPD_REF, speed_rad_s)
        except OSError as exc:
            print(f"CAN TX failed: {exc}")
            return False
        return True

    def set_speed(self, speed_rad_s: float) -> bool:
        if not self.velocity_mode_configured and not self.configure_velocity_mode():
            print("Velocity mode setup failed.")
            return False
        self.commanded_speed = speed_rad_s
        ok = self.send_speed_target(self.commanded_speed)
        print(f"{self.protocol} speed={self.commanded_speed:+.2f} rad/s {'sent' if ok else 'FAILED'}")
        return ok

    def start_jog(self, direction: int) -> None:
        self.oscillating = False
        self.jog_active = False
        speed = -MOTOR_STUDIO_JOG_SPEED_RAD_S if direction < 0 else MOTOR_STUDIO_JOG_SPEED_RAD_S
        if self.set_speed(speed):
            self.jog_active = True
            self.jog_stop_at = time.monotonic() + MOTOR_STUDIO_JOG_S
            print(
                f"Motor Studio JOG{'-' if direction < 0 else '+'} "
                f"{speed:.2f} rad/s for {int(MOTOR_STUDIO_JOG_S * 1000)} ms."
            )

    def stop_speed_only(self) -> None:
        self.oscillating = False
        self.jog_active = False
        self.set_speed(0.0)

    def stop_and_disable(self) -> None:
        self.oscillating = False
        self.jog_active = False
        if self.velocity_mode_configured:
            self.send_speed_target(0.0)
            time.sleep(0.08)
        try:
            if self.protocol == PROTOCOL_MIT:
                self.send_mit_stop()
                self.wait_mit_feedback(0.25)
            else:
                self.set_active_report(False)
                time.sleep(0.03)
                self.send_private_disable(False)
                self.wait_private_status(0.25)
        except OSError as exc:
            print(f"CAN TX failed while stopping: {exc}")
        self.velocity_mode_configured = False
        self.commanded_speed = 0.0
        print("Stop/disable sent.")

    def set_active_report(self, enabled: bool) -> None:
        try:
            if self.protocol == PROTOCOL_MIT:
                self.send_mit_active_report(enabled)
                feedback = self.wait_mit_feedback(0.30)
                print(f"MIT active reports {'on' if enabled else 'off'} {'ok' if feedback else 'sent'}")
            else:
                payload = bytes([1, 2, 3, 4, 5, 6, 1 if enabled else 0, 0])
                self.send_private(COMM_PROACTIVE_REPORT, self.host_id, self.motor_id, payload)
                print(f"Private active reports {'on' if enabled else 'off'} sent")
        except OSError as exc:
            print(f"active-report TX failed: {exc}")
            return
        self.active_reports = enabled

    def request_readback(self) -> None:
        if self.protocol == PROTOCOL_MIT:
            print("Querying MIT-standard fault/status...")
            self.send_mit_fault_query(self.motor_id, clear_fault=False)
            frame = self.wait_mit_feedback_frame(0.50)
            if frame is None:
                print("  no MIT feedback")
            else:
                fault_code = int.from_bytes(frame.data[1:5], "little", signed=False)
                feedback = decode_mit_feedback(frame)
                if feedback is not None:
                    self.print_mit_feedback(feedback, prefix="  ")
                print(f"  fault_raw=0x{fault_code:08X}")
            return

        reads = [
            (PARAM_RUN_MODE, "run_mode", "u8"),
            (PARAM_MECH_VEL, "mechVel", "f32"),
            (PARAM_MECH_POS, "mechPos", "f32"),
            (PARAM_VBUS, "VBUS", "f32"),
        ]
        for index, name, kind in reads:
            raw = self.wait_private_param(index, 0.50)
            if raw is None:
                print(f"param {name} timeout")
            elif kind == "u8":
                print(f"param {name}={raw[0]}")
            else:
                value = read_f32_le(raw)
                unit = "V" if name == "VBUS" else ("rad/s" if name == "mechVel" else "rad")
                print(f"param {name}={value:+.4f} {unit}")
            time.sleep(0.02)

    def scan_private(self, start_id: int, end_id: int) -> List[int]:
        hits: List[int] = []
        self.drain_rx()
        print(
            f"Scanning private extended IDs {fmt_id(start_id)}..{fmt_id(end_id)} "
            f"with host ID={fmt_id(self.host_id)}"
        )
        for target_id in range(start_id, end_id + 1):
            self.send_private_get_id(target_id)
            deadline = time.monotonic() + self.scan_per_id_timeout_s
            while time.monotonic() < deadline:
                frame = self.bus.recv(deadline - time.monotonic())
                if frame is None:
                    break
                if frame.extended and len(frame.data) == 8:
                    comm_type, extra, host_check = split_private_ext_id(frame.arbitration_id)
                    source_motor = extra & 0xFF
                    if comm_type == COMM_GET_ID and source_motor not in hits:
                        hits.append(source_motor)
                        print(
                            f"Found private motor ID={fmt_id(source_motor)} "
                            f"host/check={fmt_id(host_check)} uuid={bytes_hex(frame.data).replace(' ', '')}"
                        )
                        continue
                self.handle_frame(frame)
        if hits:
            self.motor_id = hits[0]
            self.protocol = PROTOCOL_PRIVATE
            self.velocity_mode_configured = False
            print(f"Private scan complete: {len(hits)} motor(s), selected {fmt_id(self.motor_id)}.")
        else:
            print("Private scan complete: no replies.")
        return hits

    def scan_mit(self, start_id: int, end_id: int) -> List[int]:
        hits: List[int] = []
        self.drain_rx()
        start_id = max(start_id, SCAN_FIRST_MIT_ID)
        print(
            f"Scanning MotorBridge MIT-standard IDs {fmt_id(start_id)}..{fmt_id(end_id)} "
            f"for feedback frames"
        )
        for target_id in range(start_id, end_id + 1):
            self.send_mit_fault_query(target_id, clear_fault=False)
            deadline = time.monotonic() + self.scan_per_id_timeout_s
            while time.monotonic() < deadline:
                frame = self.bus.recv(deadline - time.monotonic())
                if frame is None:
                    break
                if not frame.extended and len(frame.data) >= 8 and frame.data[0] == target_id:
                    if target_id not in hits:
                        hits.append(target_id)
                        fault_raw = int.from_bytes(frame.data[1:5], "little", signed=False)
                        print(
                            f"Found MIT motor ID={fmt_id(target_id)} "
                            f"feedback={fmt_id(frame.arbitration_id, 3)} fault_raw=0x{fault_raw:08X}"
                        )
                    continue
                self.handle_frame(frame)
        if hits:
            self.motor_id = hits[0]
            self.protocol = PROTOCOL_MIT
            self.velocity_mode_configured = False
            print(f"MIT scan complete: {len(hits)} motor(s), selected {fmt_id(self.motor_id)}.")
        else:
            print("MIT scan complete: no replies.")
        return hits

    def scan_current_protocol(self) -> None:
        if self.protocol == PROTOCOL_MIT:
            self.scan_mit(SCAN_FIRST_MIT_ID, SCAN_LAST_ID)
        else:
            self.scan_private(SCAN_FIRST_PRIVATE_ID, SCAN_LAST_ID)

    def scan_both_protocols(self) -> None:
        original_motor_id = self.motor_id
        original_protocol = self.protocol
        private_hits = self.scan_private(SCAN_FIRST_PRIVATE_ID, SCAN_LAST_ID)
        mit_hits = self.scan_mit(SCAN_FIRST_MIT_ID, SCAN_LAST_ID)

        if private_hits:
            self.motor_id = private_hits[0]
            self.protocol = PROTOCOL_PRIVATE
        elif mit_hits:
            self.motor_id = mit_hits[0]
            self.protocol = PROTOCOL_MIT
        else:
            self.motor_id = original_motor_id
            self.protocol = original_protocol

        self.velocity_mode_configured = False
        print(f"Selected motor={fmt_id(self.motor_id)} protocol={self.protocol}")

    def print_raw_frame(self, frame: CanFrame) -> None:
        frame_kind = "ext" if frame.extended else "std"
        width = 8 if frame.extended else 3
        print(
            f"raw {frame_kind} id={fmt_id(frame.arbitration_id, width)} "
            f"dlc={len(frame.data)} data={bytes_hex(frame.data)}"
        )

    def start_raw_trace(self) -> None:
        self.raw_trace_until = time.monotonic() + RAW_TRACE_DURATION_S
        self.raw_trace_frames = 0
        print(
            f"Raw CAN trace on for {int(RAW_TRACE_DURATION_S * 1000)} ms "
            f"or {RAW_TRACE_FRAME_LIMIT} frames."
        )

    def update_raw_trace(self) -> None:
        if self.raw_trace_until and time.monotonic() > self.raw_trace_until:
            self.raw_trace_until = 0.0
            print("Raw CAN trace off: time limit reached.")
            self.print_interface_status()

    def handle_frame(self, frame: CanFrame) -> None:
        if self.raw_trace_until:
            self.print_raw_frame(frame)
            self.raw_trace_frames += 1
            if self.raw_trace_frames >= RAW_TRACE_FRAME_LIMIT:
                self.raw_trace_until = 0.0
                print("Raw CAN trace off: frame limit reached.")
                self.print_interface_status()

        if frame.extended:
            self.handle_private_frame(frame)
        else:
            self.handle_standard_frame(frame)

    def handle_private_frame(self, frame: CanFrame) -> None:
        if len(frame.data) < 8:
            return
        comm_type, extra, host = split_private_ext_id(frame.arbitration_id)
        source_motor = extra & 0xFF

        if comm_type == COMM_GET_ID:
            print(
                f"Get-ID reply: motor={fmt_id(source_motor)} "
                f"host/check={fmt_id(host)} uuid={bytes_hex(frame.data).replace(' ', '')}"
            )
            return

        if comm_type == COMM_OPERATION_STATUS:
            p_max, v_max, t_max = self.private_limits()
            pos_raw = (frame.data[0] << 8) | frame.data[1]
            vel_raw = (frame.data[2] << 8) | frame.data[3]
            torque_raw = (frame.data[4] << 8) | frame.data[5]
            temp_raw = (frame.data[6] << 8) | frame.data[7]
            pos = uint_to_float(pos_raw, -p_max, p_max, 16)
            vel = uint_to_float(vel_raw, -v_max, v_max, 16)
            torque = uint_to_float(torque_raw, -t_max, t_max, 16)
            temp_c = temp_raw * 0.1
            self.last_feedback_at = time.monotonic()
            if self.last_feedback_at - self.last_status_print_at >= STATUS_PRINT_PERIOD_S:
                self.last_status_print_at = self.last_feedback_at
                print(
                    f"fb motor={fmt_id(source_motor)} target={fmt_id(host)} "
                    f"pos={pos:+.3f} rad vel={vel:+.3f} rad/s "
                    f"tq={torque:+.3f} Nm temp={temp_c:.1f} C"
                )
            return

        if comm_type in (COMM_READ_PARAM, COMM_WRITE_PARAM):
            index = struct.unpack("<H", frame.data[0:2])[0]
            raw = frame.data[4:8]
            if index == PARAM_RUN_MODE:
                print(f"param run_mode={raw[0]}")
            elif index == PARAM_MECH_VEL:
                print(f"param mechVel={read_f32_le(raw):+.4f} rad/s")
            elif index == PARAM_MECH_POS:
                print(f"param mechPos={read_f32_le(raw):+.4f} rad")
            elif index == PARAM_VBUS:
                print(f"param VBUS={read_f32_le(raw):.2f} V")
            return

        if comm_type == COMM_FAULT:
            report = decode_private_fault_payload(frame.data)
            print(f"FAULT private motor={fmt_id(source_motor)} {private_fault_summary(report)}")

    def handle_standard_frame(self, frame: CanFrame) -> None:
        feedback = decode_mit_feedback(frame)
        if feedback is None or feedback.motor_id != self.motor_id:
            return
        if not self.accept_any_feedback_id and feedback.feedback_id != self.feedback_id:
            return
        self.last_feedback_at = time.monotonic()
        if feedback.feedback_id != self.feedback_id:
            self.feedback_id = feedback.feedback_id
        if self.last_feedback_at - self.last_status_print_at >= STATUS_PRINT_PERIOD_S:
            self.last_status_print_at = self.last_feedback_at
            self.print_mit_feedback(feedback)

    def print_mit_feedback(self, feedback: MitFeedback, prefix: str = "") -> None:
        print(
            f"{prefix}mit fb id={fmt_id(feedback.feedback_id, 3)} "
            f"motor={fmt_id(feedback.motor_id)} mode={feedback.mode_state} "
            f"fault={1 if feedback.has_fault else 0} warn={1 if feedback.has_warning else 0} "
            f"pos={feedback.position_rad:+.3f} rad vel={feedback.velocity_rad_s:+.3f} rad/s "
            f"tq={feedback.torque_nm:+.3f} Nm temp={feedback.winding_temp_c:.1f} C"
        )

    def poll_can(self) -> None:
        while True:
            frame = self.bus.recv(0.0)
            if frame is None:
                break
            self.handle_frame(frame)

    def update_jog(self) -> None:
        if self.jog_active and time.monotonic() >= self.jog_stop_at:
            self.jog_active = False
            if self.velocity_mode_configured:
                self.commanded_speed = 0.0
                self.send_speed_target(0.0)
                print("Motor Studio JOG release: speed=0 sent.")

    def update_oscillation(self) -> None:
        if not self.oscillating:
            return
        now = time.monotonic()
        if now - self.last_oscillation_at >= OSCILLATION_PERIOD_S:
            self.last_oscillation_at = now
            self.commanded_speed = -self.commanded_speed
            self.send_speed_target(self.commanded_speed)
            print(f"osc speed={self.commanded_speed:+.2f} rad/s")

    def update(self) -> None:
        self.update_raw_trace()
        self.update_jog()
        self.update_oscillation()

    def print_interface_status(self) -> None:
        stats = self.bus.stats() if hasattr(self.bus, "stats") else {}
        values = []
        for name in ("operstate", "rx_packets", "tx_packets", "rx_errors", "tx_errors", "rx_dropped", "tx_dropped"):
            raw = stats.get(name)
            if raw:
                values.append(f"{name}={raw}")
        label = self.bus.label() if hasattr(self.bus, "label") else self.bus.interface
        print(f"{getattr(self.bus, 'transport', 'can')} {label}: " + (" ".join(values) if values else "status unavailable"))

    def reopen_socket(self) -> None:
        self.oscillating = False
        self.jog_active = False
        self.velocity_mode_configured = False
        self.bus.reopen()
        label = self.bus.label() if hasattr(self.bus, "label") else self.bus.interface
        print(f"Reopened {label}.")

    def cycle_host_id(self) -> None:
        hosts = list(PRIVATE_HOST_CANDIDATES)
        try:
            index = hosts.index(self.host_id)
            self.host_id = hosts[(index + 1) % len(hosts)]
        except ValueError:
            self.host_id = hosts[0]
        self.velocity_mode_configured = False
        print(f"Private host ID now {fmt_id(self.host_id)}")

    def toggle_protocol(self) -> None:
        self.protocol = PROTOCOL_MIT if self.protocol == PROTOCOL_PRIVATE else PROTOCOL_PRIVATE
        self.velocity_mode_configured = False
        self.oscillating = False
        self.jog_active = False
        print(f"Control protocol now {self.protocol}. Velocity mode will be reconfigured.")

    def handle_command(self, raw: str) -> bool:
        if raw in ("\r", "\n", " ", "\t"):
            return True
        cmd = raw.lower()
        print(f"Command '{raw}'")

        if cmd == "q":
            self.stop_and_disable()
            return False
        if cmd == "?":
            self.print_help()
        elif raw == "P":
            self.scan_both_protocols()
        elif cmd == "p":
            self.scan_current_protocol()
        elif cmd == "v":
            self.configure_velocity_mode()
        elif cmd == "f":
            self.oscillating = False
            self.jog_active = False
            self.set_speed(self.test_speed)
        elif cmd == "b":
            self.oscillating = False
            self.jog_active = False
            self.set_speed(-self.test_speed)
        elif raw in ("<", ","):
            self.start_jog(-1)
        elif raw in (">", "."):
            self.start_jog(1)
        elif cmd == "g":
            self.jog_active = False
            if self.oscillating:
                self.oscillating = False
                print("Oscillation off.")
            elif self.set_speed(self.test_speed):
                self.oscillating = True
                self.last_oscillation_at = time.monotonic()
                print("Oscillation on.")
        elif cmd == "0":
            self.stop_speed_only()
        elif cmd == "s":
            self.stop_and_disable()
        elif cmd == "e":
            self.clear_fault()
        elif cmd == "+":
            self.test_speed = min(3.0, self.test_speed + 0.10)
            print(f"test speed={self.test_speed:.2f} rad/s")
        elif cmd == "-":
            self.test_speed = max(0.10, self.test_speed - 0.10)
            print(f"test speed={self.test_speed:.2f} rad/s")
        elif cmd == "r":
            self.request_readback()
        elif cmd == "a":
            self.set_active_report(not self.active_reports)
        elif cmd == "m":
            self.toggle_protocol()
        elif cmd == "x":
            self.start_raw_trace()
        elif cmd == "d":
            self.print_interface_status()
        elif cmd == "c":
            self.reopen_socket()
        elif cmd == "h":
            self.cycle_host_id()
        elif cmd == "t":
            run_encoding_self_test(verbose=True)
        else:
            print("Unknown command. Type '?' for help.")
        return True


def read_text_file(path: str) -> str:
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return handle.read().strip()
    except OSError:
        return ""


@contextlib.contextmanager
def cbreak_stdin() -> Iterable[None]:
    if not sys.stdin.isatty():
        yield
        return

    import termios
    import tty

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def interactive_loop(tool: RobStrideSocketCanTool) -> None:
    tool.print_help()
    print("Press a command key. Use q to stop and quit.")
    with cbreak_stdin():
        running = True
        while running:
            tool.poll_can()
            tool.update()
            ready, _, _ = select.select([sys.stdin], [], [], 0.02)
            if ready:
                char = sys.stdin.read(1)
                running = tool.handle_command(char)


def run_for_duration(tool: RobStrideSocketCanTool, duration_s: float) -> None:
    deadline = time.monotonic() + max(0.0, duration_s)
    while time.monotonic() < deadline:
        tool.poll_can()
        tool.update()
        time.sleep(0.01)


def run_encoding_self_test(verbose: bool = False) -> bool:
    private_ping = build_private_ext_id(COMM_GET_ID, DEFAULT_HOST_ID, DEFAULT_MOTOR_ID)
    private_reply_parts = split_private_ext_id(0x00007FFE)
    mit_pos_id = mit_typed_id(MIT_TYPED_ID_POSITION, DEFAULT_MOTOR_ID)
    mit_pos = mit_position_payload(1.0, 0.5)
    mit_vel_id = mit_typed_id(MIT_TYPED_ID_VELOCITY, DEFAULT_MOTOR_ID)
    mit_vel = mit_velocity_payload(0.30, 1.0)
    private_fault = decode_private_fault_payload(bytes([0x04, 0, 0, 0, 0, 0, 0, 0]))
    official_private_write_id = build_private_ext_id(COMM_WRITE_PARAM, DEFAULT_HOST_ID, 0x01)
    official_private_write_data = bytes([0x05, 0x70, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00])
    official_serial = robstride_serial_encode_frame(
        official_private_write_id,
        official_private_write_data,
        extended=True,
    )
    expected_official_serial = bytes.fromhex("41 54 90 07 e8 0c 08 05 70 00 00 01 00 00 00 0d 0a")
    parsed_official = robstride_serial_try_parse(bytearray(expected_official_serial))
    passed = (
        private_ping == 0x0000FD7F
        and private_reply_parts == (0, 0x007F, 0xFE)
        and mit_pos_id == 0x17F
        and len(mit_pos) == 8
        and mit_vel_id == 0x27F
        and len(mit_vel) == 8
        and private_fault.fault_names == ["undervoltage"]
        and mit_special(0xFC) == bytes([0xFF] * 7 + [0xFC])
        and official_serial == expected_official_serial
        and parsed_official == CanFrame(official_private_write_id, official_private_write_data, True)
    )
    if verbose:
        print(f"encoding self-test {'PASSED' if passed else 'FAILED'}")
        print(f"  private Get-ID id={fmt_id(private_ping, 8)}")
        print(f"  private reply 0x00007FFE parts={private_reply_parts}")
        print(f"  MIT position id={fmt_id(mit_pos_id, 3)} data={bytes_hex(mit_pos)}")
        print(f"  MIT velocity id={fmt_id(mit_vel_id, 3)} data={bytes_hex(mit_vel)}")
        print(f"  private fault sample {private_fault_summary(private_fault)}")
        print(f"  RobStride serial example={bytes_hex(official_serial)}")
    return passed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", choices=TRANSPORT_CHOICES, default=TRANSPORT_ROBSTRIDE_SERIAL)
    parser.add_argument("--interface", default=DEFAULT_INTERFACE, help="SocketCAN interface, usually can0")
    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT, help="official RobStride adapter tty")
    parser.add_argument("--serial-baud", type=int, default=DEFAULT_SERIAL_BAUD)
    parser.add_argument("--motor-id", default=hex(DEFAULT_MOTOR_ID), help="RobStride motor/node ID")
    parser.add_argument("--host-id", default=hex(DEFAULT_HOST_ID), help="private-protocol host ID")
    parser.add_argument("--feedback-id", default=hex(DEFAULT_FEEDBACK_ID), help="MIT feedback/host CAN ID")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="model for private telemetry scaling")
    parser.add_argument("--protocol", choices=(PROTOCOL_PRIVATE, PROTOCOL_MIT), default=PROTOCOL_PRIVATE)
    parser.add_argument(
        "--strict-feedback-id",
        action="store_true",
        help="MIT mode: only accept feedback on --feedback-id",
    )
    parser.add_argument(
        "--command",
        choices=(
            "interactive",
            "scan",
            "scan-both",
            "scan-private",
            "scan-mit",
            "configure",
            "forward",
            "backward",
            "vel",
            "jog-left",
            "jog-right",
            "position",
            "clear-fault",
            "stop",
            "status",
            "raw",
        ),
        default="interactive",
    )
    parser.add_argument("--vel", type=float, default=DEFAULT_SPEED_RAD_S, help="velocity for --command vel")
    parser.add_argument("--pos", type=float, default=DEFAULT_POSITION_RAD, help="target radian position for --command position")
    parser.add_argument("--vlim", type=float, default=DEFAULT_POSITION_VEL_RAD_S, help="position velocity limit")
    parser.add_argument("--acc", type=float, default=DEFAULT_POSITION_ACCEL_RAD_S2, help="private PP position acceleration")
    parser.add_argument("--duration", type=float, default=0.75, help="run duration for one-shot motion commands")
    parser.add_argument(
        "--scan-timeout-ms",
        type=float,
        default=SCAN_PER_ID_TIMEOUT_S * 1000.0,
        help="per-ID scan wait time",
    )
    parser.add_argument("--self-test", action="store_true", help="test frame encoders without opening CAN")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        return 0 if run_encoding_self_test(verbose=True) else 1

    bus = create_bus(
        transport=args.transport,
        interface=args.interface,
        serial_port=args.serial_port,
        serial_baud=args.serial_baud,
    )
    try:
        bus.open()
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        print(
            "SocketCAN setup: sudo bash raspi/can_up.sh can0\n"
            "Official RobStride adapter: stop slcand, then use "
            "--transport robstride-serial --serial-port auto",
            file=sys.stderr,
        )
        return 2

    tool = RobStrideSocketCanTool(
        bus=bus,
        protocol=args.protocol,
        motor_id=parse_id(args.motor_id),
        host_id=parse_id(args.host_id),
        feedback_id=parse_id(args.feedback_id),
        model=args.model,
        accept_any_feedback_id=not args.strict_feedback_id,
        scan_per_id_timeout_s=max(0.001, args.scan_timeout_ms / 1000.0),
    )

    try:
        if args.command == "interactive":
            interactive_loop(tool)
        elif args.command == "scan":
            tool.scan_both_protocols()
        elif args.command == "scan-both":
            tool.scan_both_protocols()
        elif args.command == "scan-private":
            tool.scan_private(SCAN_FIRST_PRIVATE_ID, SCAN_LAST_ID)
        elif args.command == "scan-mit":
            tool.scan_mit(SCAN_FIRST_MIT_ID, SCAN_LAST_ID)
        elif args.command == "configure":
            tool.configure_velocity_mode()
        elif args.command == "forward":
            tool.set_speed(tool.test_speed)
            run_for_duration(tool, args.duration)
            tool.stop_and_disable()
        elif args.command == "backward":
            tool.set_speed(-tool.test_speed)
            run_for_duration(tool, args.duration)
            tool.stop_and_disable()
        elif args.command == "vel":
            tool.set_speed(args.vel)
            run_for_duration(tool, args.duration)
            tool.stop_and_disable()
        elif args.command == "jog-left":
            tool.start_jog(-1)
            run_for_duration(tool, MOTOR_STUDIO_JOG_S + 0.25)
            tool.stop_and_disable()
        elif args.command == "jog-right":
            tool.start_jog(1)
            run_for_duration(tool, MOTOR_STUDIO_JOG_S + 0.25)
            tool.stop_and_disable()
        elif args.command == "position":
            tool.move_position(args.pos, args.vlim, args.acc)
            run_for_duration(tool, args.duration)
            tool.stop_and_disable()
        elif args.command == "clear-fault":
            tool.clear_fault()
        elif args.command == "stop":
            tool.stop_and_disable()
        elif args.command == "status":
            tool.request_readback()
        elif args.command == "raw":
            tool.start_raw_trace()
            run_for_duration(tool, RAW_TRACE_DURATION_S + 0.25)
    except KeyboardInterrupt:
        print()
        print("Interrupted; stopping motor.")
        tool.stop_and_disable()
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
