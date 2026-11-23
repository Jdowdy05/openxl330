"""Read model-based torque estimates in one Fast Sync Read."""
from __future__ import annotations

import argparse
import sys

from openxl330 import Bus, Error


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    connection = parser.add_mutually_exclusive_group(required=True)
    connection.add_argument("--port", help="adapter port")
    connection.add_argument("--sim", type=int, metavar="N", help="use N simulated servos")
    parser.add_argument("--ids", nargs="+", type=int, help="IDs to read (default all simulated IDs or hardware ID 1)")
    args = parser.parse_args()

    try:
        bus = Bus.simulated(args.sim) if args.sim is not None else Bus(args.port)
        with bus:
            ids = args.ids or (range(1, args.sim + 1) if args.sim is not None else [1])
            for i, state in bus.group(ids).read().items():
                print(f"ID {i}  motor estimate {state.torque_motor:+.4f} N·m  "
                      f"horn estimate {state.torque_out:+.4f} N·m")
    except Error as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
