#!/usr/bin/env python3
"""Simple end-to-end JayBot drive and steering test.

Run from the JayBot project root:

    python3 -m tests.test_serial_link --port /dev/ttyACM0

The script:
1. waits for the Uno to reset,
2. continuously sends one wheel-speed and steering command,
3. returns to zero RPM and calibrated straight steering,
4. exits cleanly.

Keep the drive wheels lifted for the first test.
"""

from __future__ import annotations

import argparse
import time

from comms.serial_interface import (
    JaybotSerialInterface,
    LEFT_STRAIGHT_ANGLE_DEG,
    RIGHT_STRAIGHT_ANGLE_DEG,
)


TEST_DURATION_SECONDS = 4.0
NEUTRAL_DURATION_SECONDS = 1.0
COMMAND_RATE_HZ = 20.0

# Desired wheel RPM values sent to the Arduino PI controllers.
TEST_LEFT_RPM = 55.0
TEST_RIGHT_RPM = 55.0

# Absolute servo angles sent to the Arduino.
# Straight calibration:
#   left  = 136 degrees
#   right = 39 degrees
#
# This pair requests a small steering movement. Verify the physical direction
# with the wheels lifted, then adjust the signs if your linkage moves opposite.
TEST_LEFT_STEERING_DEG = 126.0
TEST_RIGHT_STEERING_DEG = 49.0


def send_for_duration(
    interface: JaybotSerialInterface,
    *,
    left_rpm: float,
    right_rpm: float,
    left_steering_deg: float,
    right_steering_deg: float,
    duration_seconds: float,
    label: str,
) -> None:
    """Continuously refresh one setpoint so the watchdog stays active."""
    period_seconds = 1.0 / COMMAND_RATE_HZ
    finish_time = time.monotonic() + duration_seconds
    next_send_time = time.monotonic()

    print(
        f"{label}: RPM=({left_rpm:.1f}, {right_rpm:.1f}), "
        f"steering=({left_steering_deg:.1f}°, "
        f"{right_steering_deg:.1f}°)"
    )

    while time.monotonic() < finish_time:
        sequence = interface.send_drive_command(
            left_rpm=left_rpm,
            right_rpm=right_rpm,
            left_steering_deg=left_steering_deg,
            right_steering_deg=right_steering_deg,
        )

        telemetry = interface.read_latest_telemetry()
        if telemetry is not None:
            print(
                f"seq={sequence:5d} "
                f"measured_rpm=("
                f"{telemetry.left_measured_rpm:6.1f}, "
                f"{telemetry.right_measured_rpm:6.1f}) "
                f"pwm=("
                f"{telemetry.left_signed_pwm:4d}, "
                f"{telemetry.right_signed_pwm:4d}) "
                f"status={telemetry.status}"
            )

        next_send_time += period_seconds
        sleep_time = next_send_time - time.monotonic()
        if sleep_time > 0.0:
            time.sleep(sleep_time)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Arduino serial port, for example /dev/ttyACM0 or COM4",
    )
    args = parser.parse_args()

    with JaybotSerialInterface(port=args.port) as interface:
        print("Waiting for Arduino Uno reset...")
        time.sleep(2.0)

        try:
            send_for_duration(
                interface,
                left_rpm=TEST_LEFT_RPM,
                right_rpm=TEST_RIGHT_RPM,
                left_steering_deg=TEST_LEFT_STEERING_DEG,
                right_steering_deg=TEST_RIGHT_STEERING_DEG,
                duration_seconds=TEST_DURATION_SECONDS,
                label="RUN",
            )
        finally:
            send_for_duration(
                interface,
                left_rpm=0.0,
                right_rpm=0.0,
                left_steering_deg=LEFT_STRAIGHT_ANGLE_DEG,
                right_steering_deg=RIGHT_STRAIGHT_ANGLE_DEG,
                duration_seconds=NEUTRAL_DURATION_SECONDS,
                label="NEUTRAL",
            )

    print("Test complete. Serial port closed.")


if __name__ == "__main__":
    main()
