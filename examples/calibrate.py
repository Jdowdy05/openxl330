"""Use the current position as a joint's zero in the host calibration."""
from __future__ import annotations

import argparse
import sys

from openxl330 import Bus, Error


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, epilog="Place the joint at its desired zero before running this example."
    )
    connection = parser.add_mutually_exclusive_group(required=True)
    connection.add_argument("--port", help="adapter port")
    connection.add_argument("--sim", type=int, metavar="N", help="use N simulated servos")
    parser.add_argument("--id", type=int, default=1, help="servo ID (default 1)")
    parser.add_argument("--direction", type=int, choices=(-1, 1), help="set the sign of positive motion")
    args = parser.parse_args()

    try:
        bus = Bus.simulated(args.sim) if args.sim is not None else Bus(args.port)
        with bus:
            servo = bus.servo(args.id)
            joint = servo.calibrate(zero="here", direction=args.direction)
            print(f"ID {servo.id}  zero {joint.zero_counts} counts  direction {joint.direction:+d}  "
                  f"position {servo.position:+.4f} rad")
            if bus.calibration.path is not None:
                print(f"Saved host calibration to {bus.calibration.path}")
    except (Error, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
