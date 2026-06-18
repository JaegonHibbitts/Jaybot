# This will handle:
# opening the serial port
# closing the serial port
# forming command strings
# sending commands
# reading Arduino responses
# parsing telemetry
# checking checksums or field counts
# tracking connection status
# detecting stale telemetry

"""JayBot Raspberry Pi serial interface.

Command frame sent to Arduino:
    @sequence,leftRPMx10,rightRPMx10,leftSteeringx10,rightSteeringx10*CRC\n

Telemetry frame received from Arduino:
    !sequence,leftMeasuredRPMx10,rightMeasuredRPMx10,leftSignedPWM,
      rightSignedPWM,leftTicks,rightTicks,status*CRC\n

The sequence number is only an acknowledgment/correlation value. It is not a
motion mode and it does not cause a maneuver on the Arduino.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Optional

import serial


STATUS_COMMS_TIMEOUT = 1 << 0
STATUS_BAD_PACKET = 1 << 1
STATUS_RX_OVERFLOW = 1 << 2


@dataclass(frozen=True)
class Telemetry:
    sequence: int
    left_measured_rpm: float
    right_measured_rpm: float
    left_signed_pwm: int
    right_signed_pwm: int
    left_ticks: int
    right_ticks: int
    status: int

    @property
    def communication_timed_out(self) -> bool:
        return bool(self.status & STATUS_COMMS_TIMEOUT)


def crc8_atm(payload: bytes) -> int:
    """CRC-8/ATM, polynomial 0x07, initial value 0x00."""
    crc = 0x00

    for byte in payload:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF

    return crc


class JaybotSerialInterface:
    """Nonblocking setpoint sender and telemetry receiver."""

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 115200,
    ) -> None:
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0,
            write_timeout=0.1,
        )

        self._sequence = 0
        self._receive_buffer = bytearray()
        self._receiving_telemetry = False

    def close(self) -> None:
        self._serial.close()

    def __enter__(self) -> "JaybotSerialInterface":
        return self

    def __exit__(self, *_: object) -> None:
        # Send several neutral commands before closing so one has a strong
        # chance of being accepted before the USB port disappears.
        for _ in range(3):
            try:
                self.send_drive_command(0.0, 0.0, 90.0, 90.0)
                time.sleep(0.02)
            except (serial.SerialException, serial.SerialTimeoutException):
                break
        self.close()

    def _next_sequence(self) -> int:
        self._sequence = (self._sequence + 1) & 0xFFFF
        return self._sequence

    @staticmethod
    def _to_x10(value: float) -> int:
        if not math.isfinite(value):
            raise ValueError("Command values must be finite.")
        return int(round(value * 10.0))

    def send_drive_command(
        self,
        left_rpm: float,
        right_rpm: float,
        left_steering_deg: float,
        right_steering_deg: float,
    ) -> int:
        """Send one complete drive-and-steering setpoint.

        Call this at approximately 20 Hz. Sending zero RPM and neutral steering
        is the normal stop command; there is no separate mode field.
        """
        sequence = self._next_sequence()

        fields = (
            sequence,
            self._to_x10(left_rpm),
            self._to_x10(right_rpm),
            self._to_x10(left_steering_deg),
            self._to_x10(right_steering_deg),
        )

        if not (-3500 <= fields[1] <= 3500):
            raise ValueError("Left RPM is outside -350.0 to 350.0 RPM.")
        if not (-3500 <= fields[2] <= 3500):
            raise ValueError("Right RPM is outside -350.0 to 350.0 RPM.")
        if not (0 <= fields[3] <= 1800):
            raise ValueError("Left steering is outside 0.0 to 180.0 degrees.")
        if not (0 <= fields[4] <= 1800):
            raise ValueError("Right steering is outside 0.0 to 180.0 degrees.")

        payload_text = ",".join(str(field) for field in fields)
        payload = payload_text.encode("ascii")
        checksum = crc8_atm(payload)

        frame = f"@{payload_text}*{checksum:02X}\n".encode("ascii")
        self._serial.write(frame)

        return sequence

    def read_latest_telemetry(self) -> Optional[Telemetry]:
        """Drain available bytes and return the newest valid telemetry frame."""
        newest: Optional[Telemetry] = None

        waiting = self._serial.in_waiting
        if waiting <= 0:
            return None

        for byte in self._serial.read(waiting):
            character = chr(byte)

            if character == "!":
                self._receiving_telemetry = True
                self._receive_buffer.clear()
                continue

            if not self._receiving_telemetry:
                continue

            if character == "\r":
                continue

            if character == "\n":
                parsed = self._parse_telemetry_frame(
                    bytes(self._receive_buffer)
                )
                if parsed is not None:
                    newest = parsed

                self._receive_buffer.clear()
                self._receiving_telemetry = False
                continue

            if len(self._receive_buffer) < 95:
                self._receive_buffer.append(byte)
            else:
                self._receive_buffer.clear()
                self._receiving_telemetry = False

        return newest

    @staticmethod
    def _parse_telemetry_frame(frame: bytes) -> Optional[Telemetry]:
        try:
            payload, checksum_text = frame.rsplit(b"*", maxsplit=1)
        except ValueError:
            return None

        if len(checksum_text) != 2:
            return None

        try:
            received_checksum = int(checksum_text, 16)
        except ValueError:
            return None

        if crc8_atm(payload) != received_checksum:
            return None

        try:
            fields = [int(part) for part in payload.split(b",")]
        except ValueError:
            return None

        if len(fields) != 8:
            return None

        return Telemetry(
            sequence=fields[0],
            left_measured_rpm=fields[1] / 10.0,
            right_measured_rpm=fields[2] / 10.0,
            left_signed_pwm=fields[3],
            right_signed_pwm=fields[4],
            left_ticks=fields[5],
            right_ticks=fields[6],
            status=fields[7],
        )


@dataclass
class EncoderDistanceTracker:
    """Pi-owned helper for a later fixed-distance corrective maneuver.

    This helper does not create an Arduino motion mode. The higher-order Pi
    controller repeatedly sends the desired signed RPM, checks telemetry, and
    sends zero RPM once the target encoder distance is reached.
    """

    counts_per_revolution: float = 660.0
    wheel_diameter_mm: float = 65.0
    start_left_ticks: Optional[int] = None
    start_right_ticks: Optional[int] = None
    target_counts: float = 0.0

    def start(self, distance_mm: float, telemetry: Telemetry) -> None:
        if distance_mm <= 0:
            raise ValueError("Distance must be greater than zero.")

        circumference_mm = math.pi * self.wheel_diameter_mm
        self.target_counts = (
            distance_mm / circumference_mm
        ) * self.counts_per_revolution

        self.start_left_ticks = telemetry.left_ticks
        self.start_right_ticks = telemetry.right_ticks

    def progress_counts(self, telemetry: Telemetry) -> float:
        if self.start_left_ticks is None or self.start_right_ticks is None:
            raise RuntimeError("Distance tracking has not been started.")

        left_delta = abs(telemetry.left_ticks - self.start_left_ticks)
        right_delta = abs(telemetry.right_ticks - self.start_right_ticks)

        return (left_delta + right_delta) / 2.0

    def complete(self, telemetry: Telemetry) -> bool:
        return self.progress_counts(telemetry) >= self.target_counts
