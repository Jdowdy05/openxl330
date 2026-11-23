"""Discover servos and report the baud rate and firmware for each one."""
from __future__ import annotations

import argparse
import sys

from openxl330 import Bus, Calibration, Error


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    connection = parser.add_mutually_exclusive_group(required=True)
    connection.add_argument("--port", help="adapter port")
    connection.add_argument("--sim", type=int, metavar="N", help="use N simulated servos")
    parser.add_argument("--all-bauds", action="store_true", help="search the supported baud rates")
    args = parser.parse_args()

    try:
        bus = Bus.simulated(args.sim) if args.sim is not None else Bus(args.port, calibration=Calibration())
        with bus:
            for servo in bus.scan(all_bauds=args.all_bauds):
                print(f"ID {servo.id:3}  {servo.model}  {servo.release or servo.firmware}  {servo.baud} baud")
    except Error as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
