"""Move servos 1–6 by five degrees together, then release torque."""
from __future__ import annotations

import argparse
import math
import sys

from openxl330 import Bus, Error


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog="Clear space for the move and support loaded joints before running. "
               "A joint can fall when torque turns OFF.",
    )
    connection = parser.add_mutually_exclusive_group(required=True)
    connection.add_argument("--port", help="adapter port for the six installed servos")
    connection.add_argument("--sim", type=int, metavar="N", help="use at least six simulated servos")
    args = parser.parse_args()
    if args.sim is not None and args.sim < 6:
        parser.error("--sim needs at least six servos for IDs 1–6")

    try:
        bus = Bus.simulated(args.sim) if args.sim is not None else Bus(args.port)
        with bus:
            arm = bus.group(range(1, 7), off_on_exit=True)
            arm.configure()
            with arm:
                arm.enable()
                targets = {i: state.position + math.radians(5) for i, state in arm.read().items()}
                arm.move(targets, duration=0.3)
                bus.wire.sleep(0.5)
                for i, state in arm.read().items():
                    print(f"ID {i}  position {math.degrees(state.position):.2f} deg")
            still_enabled = [i for i, status in arm.status().items() if status.enabled]
            if still_enabled:
                raise Error(f"torque remains enabled on IDs {still_enabled}")
    except (Error, KeyboardInterrupt) as exc:
        interrupted = isinstance(exc, KeyboardInterrupt)
        print(f"error: {'interrupted' if interrupted else exc}", file=sys.stderr)
        off_error = getattr(exc, "off_on_exit_error", None)
        if off_error is not None:
            print(f"error: torque OFF could not be confirmed ({off_error})", file=sys.stderr)
        return 130 if interrupted else 1

    print("Confirmed torque OFF on IDs 1–6")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
