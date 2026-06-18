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

Command sent to the Arduino:
    @sequence,leftRPMx10,rightRPMx10,leftSteeringx10,rightSteeringx10*CRC\n

Telemetry received from the Arduino:
    !sequence,leftMeasuredRPMx10,rightMeasuredRPMx10,leftSignedPWM,
      rightSignedPWM,leftTicks,rightTicks,status*CRC\n

The Arduino receives only two RPM targets and two steering targets. The
sequence number is an acknowledgement/correlation value, not a motion mode.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Optional

import serial


LEFT_STRAIGHT_ANGLE_DEG = 136.0
RIGHT_STRAIGHT_ANGLE_DEG = 39.0

STATUS_COMMS_TIMEOUT = 1 << 0
STATUS_BAD_PACKET = 1 << 1
STATUS_RX_OVERFLOW = 1 << 2


@dataclass(frozen=True)
class DriveSetpoint:
    left_rpm: float
    right_rpm: float
    left_steering_deg: float
    right_steering_deg: float


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

    @property
    def bad_packet_seen(self) -> bool:
        return bool(self.status & STATUS_BAD_PACKET)

    @property
    def receive_overflow_seen(self) -> bool:
        return bool(self.status & STATUS_RX_OVERFLOW)


def crc8_atm(payload: bytes) -> int:
    """Return CRC-8/ATM: polynomial 0x07, initial value 0x00."""
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
    """Send drive/steering setpoints and optionally read debug telemetry."""

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

        self._last_setpoint = DriveSetpoint(
            left_rpm=0.0,
            right_rpm=0.0,
            left_steering_deg=LEFT_STRAIGHT_ANGLE_DEG,
            right_steering_deg=RIGHT_STRAIGHT_ANGLE_DEG,
        )

    @property
    def last_setpoint(self) -> DriveSetpoint:
        return self._last_setpoint

    def close(self) -> None:
        self._serial.close()

    def __enter__(self) -> "JaybotSerialInterface":
        return self

    def __exit__(self, *_: object) -> None:
        # Send several neutral commands before closing. The Arduino watchdog
        # remains the final protection if the port disappears unexpectedly.
        for _ in range(3):
            try:
                self.send_neutral()
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

    def send_neutral(self) -> int:
        """Stop both motors and center both steering servos."""
        return self.send_drive_command(
            left_rpm=0.0,
            right_rpm=0.0,
            left_steering_deg=LEFT_STRAIGHT_ANGLE_DEG,
            right_steering_deg=RIGHT_STRAIGHT_ANGLE_DEG,
        )

    def send_drive_command(
        self,
        left_rpm: float,
        right_rpm: float,
        left_steering_deg: float,
        right_steering_deg: float,
    ) -> int:
        """Send one complete drive-and-steering setpoint.

        Normal operation should call this approximately 20 times per second.
        Zero RPM with straight steering is the neutral/stop command.
        """
        setpoint = DriveSetpoint(
            left_rpm=left_rpm,
            right_rpm=right_rpm,
            left_steering_deg=left_steering_deg,
            right_steering_deg=right_steering_deg,
        )

        sequence = self._next_sequence()
        fields = (
            sequence,
            self._to_x10(setpoint.left_rpm),
            self._to_x10(setpoint.right_rpm),
            self._to_x10(setpoint.left_steering_deg),
            self._to_x10(setpoint.right_steering_deg),
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

        # Update only after the complete frame has been handed to pySerial.
        self._last_setpoint = setpoint
        return sequence

    def corrective_reverse(
        self,
        duration_seconds: float = 1.0,
        speed_fraction: float = 0.5,
        reference_rpm: Optional[float] = None,
        command_rate_hz: float = 20.0,
        restore_previous: bool = True,
    ) -> None:
        """Reverse straight for a fixed time using ordinary serial commands.

        The previous drive/steering setpoint is saved. During the correction,
        equal negative RPM targets and the calibrated straight steering angles
        are transmitted at `command_rate_hz`. The previous setpoint is then
        restored.

        Telemetry is not used to start, stop, or complete this action.

        `reference_rpm` may be supplied explicitly. Otherwise the function uses
        the greater magnitude of the currently commanded left/right RPM.
        """
        if duration_seconds <= 0.0:
            raise ValueError("duration_seconds must be greater than zero.")
        if not (0.0 < speed_fraction <= 1.0):
            raise ValueError("speed_fraction must be in the range (0, 1].")
        if command_rate_hz <= 0.0:
            raise ValueError("command_rate_hz must be greater than zero.")

        previous = self._last_setpoint

        if reference_rpm is None:
            reference_speed = max(
                abs(previous.left_rpm),
                abs(previous.right_rpm),
            )
        else:
            reference_speed = abs(reference_rpm)

        if reference_speed < 0.5:
            raise ValueError(
                "No usable reference RPM is available. Supply reference_rpm "
                "or call corrective_reverse while a nonzero command is active."
            )

        reverse_rpm = -reference_speed * speed_fraction
        period_seconds = 1.0 / command_rate_hz
        finish_time = time.monotonic() + duration_seconds
        next_send_time = time.monotonic()

        try:
            while True:
                now = time.monotonic()
                if now >= finish_time:
                    break

                self.send_drive_command(
                    left_rpm=reverse_rpm,
                    right_rpm=reverse_rpm,
                    left_steering_deg=LEFT_STRAIGHT_ANGLE_DEG,
                    right_steering_deg=RIGHT_STRAIGHT_ANGLE_DEG,
                )

                next_send_time += period_seconds
                sleep_duration = min(
                    next_send_time,
                    finish_time,
                ) - time.monotonic()

                if sleep_duration > 0.0:
                    time.sleep(sleep_duration)
        finally:
            if restore_previous:
                self.send_drive_command(
                    left_rpm=previous.left_rpm,
                    right_rpm=previous.right_rpm,
                    left_steering_deg=previous.left_steering_deg,
                    right_steering_deg=previous.right_steering_deg,
                )
            else:
                self.send_neutral()

    def read_latest_telemetry(self) -> Optional[Telemetry]:
        """Return the newest valid telemetry frame currently available.

        Telemetry is optional and intended for debugging/logging. It does not
        control the timed corrective reverse.
        """
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
