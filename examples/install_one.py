"""Install openxl330 on one XL330-M288 set to 57,600 baud, keeping its configured ID."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

from openxl330 import Bus, Calibration, Error, images, install
from openxl330.sim import SimPort, SimServo


def simulate_install(count: int, stock_id: int) -> dict:
    # Only one unit starts on stock firmware. The others are already running openxl330.
    program = images.bundled(images.DEFAULT_RELEASE).packaged().raw
    other_ids = [i for i in range(1, 253) if i != stock_id][:count - 1]
    devices = [SimServo(stock_id)]
    devices.extend(SimServo(i, mode="app", program=program, seed=i) for i in other_ids)
    with TemporaryDirectory(prefix="openxl330-example-") as records:
        with Bus(SimPort(devices), calibration=Calibration(), check_adapter=False) as bus:
            return install.from_stock(bus.wire, stock_id=stock_id, records=Path(records))


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog="For hardware, give the servo a unique ID in Dynamixel Wizard and connect it by itself. "
               "Installing replaces the stock firmware permanently.",
    )
    connection = parser.add_mutually_exclusive_group(required=True)
    connection.add_argument("--port", help="adapter port for the single stock servo")
    connection.add_argument("--sim", type=int, metavar="N", help="simulate a bus of N servos")
    parser.add_argument("--stock-id", type=int, required=True,
                        help="the ID set in Dynamixel Wizard (required; the installed firmware keeps it)")
    args = parser.parse_args()
    if not 0 <= args.stock_id <= 252:
        parser.error("--stock-id must be in 0..252")
    if args.sim is not None and not 1 <= args.sim <= 252:
        parser.error("--sim must be in 1..252")

    try:
        if args.sim is not None:
            receipt = simulate_install(args.sim, args.stock_id)
        else:
            with Bus(args.port, calibration=Calibration()) as bus:
                receipt = install.from_stock(bus.wire, stock_id=args.stock_id)
    except KeyboardInterrupt as exc:
        print("error: interrupted", file=sys.stderr)
        if getattr(exc, "recovery_note", None):
            print(exc.recovery_note.strip(), file=sys.stderr)
        if getattr(exc, "recovery", None):
            print(f"To recover the servo, run: {exc.recovery}", file=sys.stderr)
        return 130
    except Error as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    for warning in receipt["warnings"]:
        print(f"warning: {warning}", file=sys.stderr)
    print(f"Installed {receipt['release']} on ID {receipt['servo_id']}")
    if args.sim is None:
        print(install.records_root() / receipt["run_dir"] / "receipt.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
