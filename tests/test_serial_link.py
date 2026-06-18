#!/usr/bin/env python3
"""Bench-test the JayBot Arduino serial protocol.

Run a link-only test:
    python3 test_serial_link.py --port /dev/ttyACM0

Run the motor sequence with the wheels safely lifted:
    python3 test_serial_link.py --port /dev/ttyACM0 --run-motion
"""

from __future__ import annotations

import argparse
import time

from comms.serial_interface import (
    JaybotSerialInterface,
    LEFT_STRAIGHT_ANGLE_DEG,
    RIGHT_STRAIGHT_ANGLE_DEG,
)


COMMAND_RATE_HZ = 20.0


def print_telemetry(interface: JaybotSerialInterface) -> None:
    telemetry = interface.read_latest_telemetry()
    if telemetry is None:
        return

    print(
        f"seq={telemetry.sequence:5d}  "
        f"rpm=({telemetry.left_measured_rpm:7.1f},"
        f"{telemetry.right_measured_rpm:7.1f})  "
        f"pwm=({telemetry.left_signed_pwm:4d},"
        f"{telemetry.right_signed_pwm:4d})  "
        f"ticks=({telemetry.left_ticks:8d},"
        f"{telemetry.right_ticks:8d})  "
        f"status={telemetry.status}"
    )


def hold_setpoint(
    interface: JaybotSerialInterface,
    left_rpm: float,
    right_rpm: float,
    duration_seconds: float,
    label: str,
) -> None:
    print(f"\n{label}: left={left_rpm:.1f}, right={right_rpm:.1f} RPM")

    period = 1.0 / COMMAND_RATE_HZ
    finish = time.monotonic() + duration_seconds
    next_send = time.monotonic()

    while time.monotonic() < finish:
        interface.send_drive_command(
            left_rpm=left_rpm,
            right_rpm=right_rpm,
            left_steering_deg=LEFT_STRAIGHT_ANGLE_DEG,
            right_steering_deg=RIGHT_STRAIGHT_ANGLE_DEG,
        )
        print_telemetry(interface)

        next_send += period
        sleep_duration = next_send - time.monotonic()
        if sleep_duration > 0.0:
            time.sleep(sleep_duration)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--run-motion", action="store_true")
    args = parser.parse_args()

    with JaybotSerialInterface(port=args.port) as interface:
        # Opening a USB serial port commonly resets an Uno.
        print("Waiting for the Uno to restart...")
        time.sleep(2.0)

        print("\nLink test: repeatedly sending neutral commands.")
        hold_setpoint(
            interface,
            left_rpm=0.0,
            right_rpm=0.0,
            duration_seconds=2.0,
            label="NEUTRAL",
        )

        if not args.run_motion:
            print(
                "\nLink-only test complete. Re-run with --run-motion after "
                "lifting the drive wheels safely."
            )
            return

        input(
            "\nConfirm the drive wheels are lifted and clear, then press Enter."
        )

        hold_setpoint(interface, 55.0, 55.0, 3.0, "STRAIGHT CAUTION")
        hold_setpoint(interface, 60.0, 90.0, 3.0, "UNEQUAL 60/90")
        hold_setpoint(interface, 90.0, 60.0, 3.0, "UNEQUAL 90/60")
        hold_setpoint(interface, 90.0, 90.0, 1.0, "PRE-CORRECTION")

        print("\nTIMED CORRECTIVE REVERSE: half speed for 1.0 second")
        interface.corrective_reverse(
            duration_seconds=1.0,
            speed_fraction=0.5,
            command_rate_hz=COMMAND_RATE_HZ,
            restore_previous=True,
        )

        # Keep the restored 90/90 command alive briefly so it does not expire.
        hold_setpoint(interface, 90.0, 90.0, 1.0, "RESTORED COMMAND")
        hold_setpoint(interface, 0.0, 0.0, 1.0, "FINAL NEUTRAL")

        print("\nMotion test complete.")


if __name__ == "__main__":
    main()
