"""Command-line setup and diagnostics for openxl330."""
from __future__ import annotations

import argparse
import json
import re
import sys
import tempfile
import textwrap
import traceback
import warnings
from dataclasses import asdict, fields
from pathlib import Path

from . import (AdapterWarning, Bus, BusUncertain, Calibration, CalibrationError, Error, NoReply, Rejected, Status,
               __version__, images, install)
from .bus import BUS_BAUDS
from .registers import TORQUE_ENABLE
from .wire import APP_BAUD, LOADER_BAUD

PERMANENT_ID = ("Installing keeps the ID the servo has on its stock firmware, so set a unique ID in Dynamixel Wizard "
                "first; afterwards change it over the bus with `openxl330 set-id` (Dynamixel Wizard cannot change the "
                "ID of a servo running openxl330).")
CONFIRM = "This replaces the stock firmware; there is no supported way back. Continue? [y/N] "
EXPERIMENTAL_NOTE = ("EXPERIMENTAL: tested on one bench with two XL330-M288 servos; see docs/experimental.md before "
                     "relying on it.")
FLASH_ALL_CONFIRM = "This replaces the stock firmware of every servo marked 'install' above. Continue? [y/N] "
FACTORY_ID_WARNING = ("warning: factory default ID 1; set a unique ID in Dynamixel Wizard first if this servo will share "
                      "a bus")


def _positive_int(text):
    value = int(text)
    if value < 1:
        raise argparse.ArgumentTypeError("expected a positive integer")
    return value


def _servo_id(text):
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError("expected a servo ID from 0 to 252") from None
    if not 0 <= value <= 252:
        raise argparse.ArgumentTypeError("expected a servo ID from 0 to 252")
    return value


def _sim_count(text):
    value = _servo_id(text)
    if value == 0:
        raise argparse.ArgumentTypeError("--sim requires 1 to 252 servos")
    return value


def _ids(text):
    spec = text.strip()
    message = "SPEC must be an ascending range like 1-6 or a list like 1,3,5 of unique IDs from 0 to 252"
    try:
        if re.fullmatch(r"[0-9]+\s*-\s*[0-9]+", spec):
            first, last = map(_servo_id, spec.split("-"))
            if first <= last:
                return list(range(first, last + 1))
        elif re.fullmatch(r"[0-9]+(?:\s*,\s*[0-9]+)*", spec):
            ids = list(map(_servo_id, spec.split(",")))
            if len(set(ids)) == len(ids):
                return ids
    except argparse.ArgumentTypeError:
        pass
    raise argparse.ArgumentTypeError(message)


class _HelpFormatter(argparse.HelpFormatter):
    """argparse's formatter, but never breaking an option name such as --allow-id-change across lines."""

    def _split_lines(self, text, width):
        return textwrap.wrap(" ".join(text.split()), width, break_on_hyphens=False, break_long_words=False)

    def _fill_text(self, text, width, indent):
        return textwrap.fill(" ".join(text.split()), width, initial_indent=indent, subsequent_indent=indent,
                             break_on_hyphens=False, break_long_words=False)


def _global_options(parser, *, after_command=False):
    # Suppressed defaults on subparsers keep options supplied before the command.
    default = argparse.SUPPRESS if after_command else None
    flag_default = argparse.SUPPRESS if after_command else False
    parser.add_argument("--port", metavar="P", default=default, help="adapter port")
    parser.add_argument("--baud", type=_positive_int, default=default, help=f"host baud rate (default {APP_BAUD}, {LOADER_BAUD} for recover)")
    parser.add_argument("--sim", type=_sim_count, metavar="N", default=default, help="use N simulated servos")
    parser.add_argument("--json", action="store_true", default=flag_default, help="output JSON")
    parser.add_argument("--debug", action="store_true", default=flag_default, help="show a traceback on errors")


def _parser():
    parser = argparse.ArgumentParser(prog="openxl330", allow_abbrev=False, formatter_class=_HelpFormatter,
                                     description="Set up and inspect openxl330 servos.")
    _global_options(parser)
    commands = parser.add_subparsers(dest="command", required=True)

    def command(name, help_text, description=None, epilog=None):
        sub = commands.add_parser(name, help=help_text, description=description or help_text, epilog=epilog,
                                  allow_abbrev=False, formatter_class=_HelpFormatter)
        _global_options(sub, after_command=True)
        return sub

    command("ports", "List adapter ports.")
    command("version", "Show the package version and bundled releases.")
    command("scan", "Find servos on the bus.").add_argument(
        "--all-bauds", action="store_true",
        help="also try 2 and 3 Mbaud, 57600 and 115200 baud (about 0.8 s each), then return to --baud")

    first = command("install", "Install the bundled firmware on a factory servo and keep its ID.",
                    "Install the bundled firmware on a factory servo (stock ROBOTIS firmware) and keep its ID. "
                    "Installing replaces the stock firmware; there is no supported way back. Connect one servo at a "
                    "time.",
                    epilog=PERMANENT_ID + " Installing takes about 30 s: do not unplug or interrupt it.")
    first.add_argument("--stock-id", type=_servo_id, required=True, metavar="ID",
                       help="the ID the servo has on its stock firmware, set in Dynamixel Wizard (required); the "
                            "installed firmware keeps it")
    first.add_argument("--stock-baud", type=_positive_int, default=57_600, metavar="BPS",
                       help="the servo's stock baud rate (default 57600, the factory setting)")
    first.add_argument("--allow-untested", action="store_true",
                       help="install over a stock firmware version other than the proven V50, V52 and V53 (at your own "
                            "risk; please share the receipt)")
    first.add_argument("--yes", action="store_true",
                       help="do not ask for confirmation (required when not run in a terminal)")
    update = command("update", "Install the bundled release on a servo running openxl330 (it keeps its ID).")
    update.add_argument("--id", type=_servo_id, required=True, help="the servo's ID")
    update.add_argument("--allow-id-change", action="store_true",
                        help="allow a servo whose ID came from an earlier tool's device block to move back to its "
                             "stock NV ID")
    for sub in (first, update):
        sub.add_argument("--release", choices=images.release_names(), default=None,
                         help="the bundled release to install (default: the one for the servo's model and --variant)")
        sub.add_argument("--variant", choices=images.VARIANTS, default=None,
                         help="performance (the default for install) or logging (the data-acquisition drain and extra "
                              "counters, no ID log); update keeps the variant the servo runs unless this is given")
        sub.add_argument("--beta", action="store_true",
                         help="accept a beta servo model (the XL330-M077, constants not yet measured); it needs a "
                              "release built for that model")

    set_id = command("set-id", "Change a servo's ID over the bus (openxl330 release 17 and later).",
                     "Change a servo's ID over the bus (openxl330 release 17 and later). The servo must be OFF and the "
                     "new ID free on the bus. The new ID stays after power-off, and the host calibration entry moves "
                     "with it.")
    set_id.add_argument("--id", type=_servo_id, required=True, help="the servo's current ID")
    set_id.add_argument("--new-id", type=_servo_id, required=True, metavar="ID",
                        help="the ID to give it (0..252, not used by another servo)")
    set_id.add_argument("--yes", action="store_true",
                        help="do not ask for confirmation (required when not run in a terminal)")

    recover = command("recover", "Finish installing on a servo waiting in the loader.")
    recover.add_argument("--id", type=_servo_id, required=True,
                         help="the ID the servo answers at once it runs the firmware: its stock NV ID, as the failed "
                              "run named it")
    recover.add_argument("--receipt", type=Path, metavar="PATH",
                         help="the failed run's receipt.json or its folder (default: the newest run for this ID that "
                              "left the servo in the loader)")
    recover.add_argument("--loader-fresh", action="store_true",
                         help="the servo was power-cycled since the failed install, so the loader waits for its first "
                              "prompt")

    baud = command("set-baud", "Change the baud rate of the bus.")
    baud.add_argument("--to", type=int, choices=BUS_BAUDS, required=True, metavar="BPS",
                      help=f"the new baud rate: {', '.join(map(str, BUS_BAUDS))}")
    group_commands = (
        ("configure", "Configure PD gains while torque is off."),
        ("status", "Show servo state, faults and firmware release."),
        ("diag", "Show servo diagnostics."),
        ("identify", "Turn on the selected servo's LED."),
        ("off", "Turn off servo torque (on any firmware: no identity check first)."),
        ("clear", "Clear latched faults while torque is off."),
    )
    for name, help_text in group_commands:
        sub = command(name, help_text)
        ids_help = "ID range 1-6 or list 1,3,5"
        if name == "off":
            which = sub.add_mutually_exclusive_group(required=True)
            which.add_argument("--ids", type=_ids, metavar="SPEC", help=ids_help + " (each is sent OFF and confirmed)")
            which.add_argument("--all", action="store_true",
                               help="broadcast OFF to every ID at this baud, then confirm each servo that answers")
            continue
        sub.add_argument("--ids", type=_ids, metavar="SPEC", required=True, help=ids_help)
        if name == "configure":
            sub.add_argument("--kp", type=float, help="proportional gain in N·m/rad")
            sub.add_argument("--kd", type=float, help="derivative gain in N·m·s/rad")
        elif name == "identify":
            sub.add_argument("--id", type=_servo_id, required=True, help="the servo whose LED to turn on")

    _experimental_parser(command("experimental", "EXPERIMENTAL: flash-all and uninstall (see docs/experimental.md).",
                                 "EXPERIMENTAL commands, tested on one bench with two XL330-M288 servos (see "
                                 "docs/experimental.md). flash-all installs on every stock servo of the bus at once; "
                                 "uninstall and uninstall-restore go back to stock firmware through ROBOTIS Dynamixel "
                                 "Wizard's Firmware Recovery."))

    calibrate = command("calibrate", "Set or reset a joint's host calibration.")
    calibrate.add_argument("--id", type=_servo_id, required=True, help="the servo's ID")
    zero = calibrate.add_mutually_exclusive_group(required=True)
    zero.add_argument("--zero-here", action="store_true", help="make the current position 0 rad")
    zero.add_argument("--reset", action="store_true",
                      help="reset this joint to the default calibration (host file only; no servo needed)")
    calibrate.add_argument("--direction", type=int, choices=(-1, 1), help="1, or -1 to reverse the joint's direction")
    return parser


def _experimental_parser(parent):
    from .experimental import RESTORE_BAUDS
    commands = parent.add_subparsers(dest="experimental_command", required=True, metavar="COMMAND")

    def command(name, help_text, description=None):
        sub = commands.add_parser(name, help=help_text, description=description or help_text, allow_abbrev=False,
                                  formatter_class=_HelpFormatter)
        _global_options(sub, after_command=True)
        return sub

    flash = command("flash-all", "Install on every stock XL330-M288 on the bus; each keeps its ID.",
                    "Install the bundled firmware on every stock XL330-M288 on the bus, one at a time in ID order; "
                    "each keeps its ID and starts at 1 Mbaud. Servos already on openxl330 firmware are skipped. The "
                    "bus is scanned at 57600, 1M, 2M, 3M and 115200 baud first, and the run is refused before anything is "
                    "installed if two servos share an ID, an ID answers at several bauds, a device answers with an "
                    "error, or a servo is not an XL330-M288. Other stock servos stay on the bus and are checked after "
                    "each install.")
    flash.add_argument("--allow-untested", action="store_true",
                       help="also install over stock firmware versions other than the proven V50, V52 and V53")
    flash.add_argument("--yes", action="store_true",
                       help="do not ask for confirmation (required when not run in a terminal)")
    flash.add_argument("--release", choices=images.release_names(), default=None,
                       help=f"the bundled release to install (default {images.DEFAULT_RELEASE}, or its logging image "
                            f"with --variant logging)")
    flash.add_argument("--variant", choices=images.VARIANTS, default=None,
                       help="performance (the default) or logging")
    uninstall = command("uninstall", "Go back to stock, step 1: record a servo's settings and leave it waiting for "
                                     "Dynamixel Wizard's Firmware Recovery.")
    uninstall.add_argument("--id", type=_servo_id, required=True, help="the servo's ID (it must run openxl330 at "
                                                                         "1 Mbaud)")
    uninstall.add_argument("--yes", action="store_true",
                           help="do not ask for confirmation (required when not run in a terminal)")
    uninstall.add_argument("--allow-untested", action="store_true",
                           help="allow stock servos other than V53 to stay on the bus at 57600 during the hand-off and "
                                "Wizard's recovery")
    restore = command("uninstall-restore", "Go back to stock, step 2 (after Wizard's recovery): write the recorded "
                                           "settings, ID and baud back.")
    restore.add_argument("--record", type=Path, required=True, metavar="PATH",
                         help="the uninstall-record.json that `experimental uninstall` wrote, or its folder")
    restore.add_argument("--stock-baud", type=int, choices=RESTORE_BAUDS, metavar="BPS",
                         help="the baud to leave the servo at: 57600, 115200, 1000000, 2000000 or 3000000 (default: its "
                              "first stock baud from the record when it is one of these, else 57600)")
    restore.add_argument("--yes", action="store_true",
                         help="do not ask for confirmation (required when not run in a terminal)")


def _table(headers, rows):
    cells = [headers, *(tuple(str(value) for value in row) for row in rows)]
    widths = [max(map(len, column)) for column in zip(*cells)]
    for row in cells:
        print("  ".join(value.ljust(width) for value, width in zip(row, widths)).rstrip())


def _show(args, data, headers, rows):
    if args.json:
        print(json.dumps(data, indent=2))
    else:
        _table(headers, rows)


def _scan(args, bus):
    infos = bus.scan(all_bauds=args.all_bauds)
    data = [dict(asdict(info), features=sorted(info.features)) for info in infos]
    rows = ((i.id, i.model, i.firmware, i.release or "-", i.baud, "yes" if i.supported else "no", i.label)
            for i in infos)
    _show(args, data, ("ID", "Model", "Firmware", "Release", "Baud", "Supported", "Label"), rows)


def _update_available(args, bus, info):
    if info.release == images.DEFAULT_RELEASE:
        return None
    try:
        current = images.bundled(images.DEFAULT_RELEASE).release_id["number"]
        if info.release in images.RELEASES:
            found = images.bundled(info.release).release_id["number"]
        else:
            match = re.fullmatch(r"FW231 release ([0-9]+)", info.release or "")
            found = int(match[1]) if match else current
    except Error:
        return None
    if "release_id" in info.features and found >= current:
        return None
    return install.update_command(bus.wire, info.id)        # simulated: `--port SIM`, like every printed command


def _status(args, bus):
    infos, missing = [], []
    for servo_id in args.ids:
        try:
            infos.append(bus.servo_info(servo_id))
        except NoReply:
            missing.append(servo_id)
    supported_ids = [info.id for info in infos if info.supported]
    statuses = bus.group(supported_ids).status() if supported_ids else {}
    unavailable = {field.name: None for field in fields(Status) if field.name != "id"}
    data = []
    rows = []
    for info in infos:
        status = statuses.get(info.id)
        measured = asdict(status) if status is not None else unavailable
        update = _update_available(args, bus, info) if status is not None else None
        data.append(dict(measured, id=info.id, model=info.model, firmware=info.firmware, abi=info.abi,
                         supported=info.supported, release=info.release, baud=info.baud, update_available=update))
        if status is None:
            identity = f"{info.firmware} (ABI {info.abi})" if info.abi is not None else info.firmware
            rows.append((info.id, identity, "unverified", "-", "-", "-", "-", "-"))
        else:
            rows.append((info.id, info.release or "-", "ON" if status.enabled else "OFF",
                         ", ".join(status.faults) or "-", f"{status.position:.4f}", f"{status.voltage:.2f}",
                         f"{status.temperature:g}", ", ".join(WARNING_TEXT.get(w, w) for w in status.warnings) or "-"))
    if data or args.json:
        _show(args, data, ("ID", "Release", "Torque", "Faults", "Position (rad)", "Supply (V)", "Temp (°C)",
                           "Warnings"), rows)
    if not args.json:
        for row in data:
            if row["update_available"]:
                print(f'ID {row["id"]} update available: {row["update_available"]}')
                if row["baud"] != APP_BAUD:
                    print(f"Switch the bus back to {APP_BAUD} baud before updating.")
    if missing:
        print(f"missing: {missing}", file=sys.stderr)
        return 1
    return 0


def _port_option(args):
    """The --port option as suggested commands print it (`--port SIM` in a simulation, like the installer's)."""
    if getattr(args, "port", None) is not None:
        return f"--port {args.port}"
    return "--port SIM" if args.sim is not None else "--port P"


class _Progress:
    """The upload progress on stderr: one line rewritten in place on a terminal, plain lines (every 10 %) otherwise."""

    def __init__(self, stream=None):
        self.stream = stream if stream is not None else sys.stderr
        self.tty = _isatty(self.stream)
        self.shown, self.open = -1, False

    def __call__(self, done, total):
        percent = 100 if total <= 0 else min(100, done * 100 // total)
        if self.tty:
            self.stream.write(f"\rUploading firmware: {percent:3d}% ({done}/{total} B)")
            self.open = percent < 100
            if not self.open:
                self.stream.write("\n")
            self.stream.flush()
        elif percent // 10 > self.shown:
            self.shown = percent // 10
            print(f"Uploading firmware: {percent}% ({done}/{total} B)", file=self.stream, flush=True)

    def close(self):
        """End a line left open by an interrupted upload, so the next message starts on its own line."""
        if self.open:
            self.stream.write("\n")
            self.stream.flush()
            self.open = False


def _isatty(stream):
    try:
        return bool(stream.isatty())
    except (AttributeError, ValueError, OSError):
        return False


def _confirm_install(args):
    """Before anything is sent: the factory-ID warning, then the confirmation (a real install only; --yes skips both)."""
    if args.yes:
        return
    if args.stock_id == 1:
        print(FACTORY_ID_WARNING, file=sys.stderr)
    if args.sim is not None:
        return                                                  # a simulation replaces nothing
    if not _isatty(sys.stdin):
        raise Error("not installed: installing replaces the stock firmware and there is no supported way back, so it "
                    "needs a confirmation. Run it in a terminal, or pass --yes. Nothing was sent")
    print(CONFIRM, end="", file=sys.stderr, flush=True)
    answer = sys.stdin.readline().strip().lower()
    if answer not in ("y", "yes"):
        raise Error("not installed: not confirmed. Nothing was sent")


WARNING_TEXT = {"supply_high": "supply above 7.0 V", "voltage_protection_off": "upper voltage protection off"}
SET_ID_CONFIRM = "This changes ID {id} to ID {new}; the servo keeps the new ID after power-off. Continue? [y/N] "


def _confirm_set_id(args):
    """Before the port opens (a real bus only; --yes skips it)."""
    if args.yes or args.sim is not None:
        return
    if not _isatty(sys.stdin):
        raise Error("ID not changed: changing an ID needs a confirmation. Run it in a terminal, or pass --yes. "
                    "Nothing was sent")
    print(SET_ID_CONFIRM.format(id=args.id, new=args.new_id), end="", file=sys.stderr, flush=True)
    if sys.stdin.readline().strip().lower() not in ("y", "yes"):
        raise Error("ID not changed: not confirmed. Nothing was sent")


def _set_id(args, bus):
    from .servo import change_id
    try:
        servo = bus.servo(args.id)
    except NoReply:
        if _finish_id_change(args, bus):
            return 0
        raise
    records = _sim_records(args)
    new, receipt = change_id(servo, args.new_id, records)
    root = records if records is not None else install.records_root()
    _show(args, receipt, ("ID", "Previous ID", "Receipt"),
          [(new.id, args.id, root / receipt["run_dir"] / "receipt.json")])
    return 0


def _finish_id_change(args, bus):
    """A re-run after a set-id whose reply was lost: install.finished_set_id proves, from that run's receipt and the
    servo's calibration fingerprint, that the servo now at the new ID is the one it changed. Then finish the host side
    (the calibration entry). Returns True when it did. A --sim run keeps no records between runs, so it never does."""
    if args.sim is not None:
        return False
    try:
        bus.servo(args.new_id)
        receipt = install.finished_set_id(bus.wire, servo_id=args.id, new_id=args.new_id)
    except Error:
        return False
    if receipt is None:
        return False
    moved = bus.calibration.has(args.id)
    if moved:
        bus.calibration.move(args.id, args.new_id)
    print(f"ID {args.id} already changed to ID {args.new_id} (the set-id run {receipt['run_dir']}); "
          + ("moved its host calibration." if moved else "no host calibration to move."))
    return True


def _sim_records(args):
    """None (the real records), or for --sim a temporary folder: simulated runs never reach the user's records."""
    if args.sim is None:
        return None
    records = Path(tempfile.mkdtemp(prefix="openxl330-sim-records-"))
    print(f"simulation: records go to {records} (a temporary folder, not your install records)", file=sys.stderr)
    return records


def _installation(args, bus):
    wire = bus.wire
    records = _sim_records(args)
    servo_id = args.stock_id if args.command == "install" else args.id
    print(f"Installing firmware on ID {servo_id}: about 30 s; do not unplug or interrupt.", file=sys.stderr,
          flush=True)
    progress = _Progress()
    try:
        if args.command == "install":
            receipt = install.from_stock(wire, stock_id=args.stock_id, stock_baud=args.stock_baud,
                                         release=args.release, allow_untested=args.allow_untested, records=records,
                                         on_progress=progress, allow_beta=args.beta, variant=args.variant)
        elif args.command == "update":
            receipt = install.update(wire, servo_id=args.id, release=args.release,
                                     allow_id_change=args.allow_id_change, records=records, on_progress=progress,
                                     allow_beta=args.beta, variant=args.variant)
        else:
            receipt = install.recover(wire, servo_id=args.id, receipt=args.receipt, loader_fresh=args.loader_fresh,
                                      records=records, on_progress=progress)
    finally:
        progress.close()

    for warning in receipt.get("warnings", ()):
        print(f"warning: {warning}", file=sys.stderr)
    if args.command == "recover" and not receipt["completed"]:
        print("warning: installed, calibration preservation not verified (no earlier records)", file=sys.stderr)
    root = records if records is not None else install.records_root()
    receipt_path = root / receipt["run_dir"] / "receipt.json"
    _show(args, receipt, ("ID", "Installed release", "Receipt"),
          [(receipt["servo_id"], receipt["release"], receipt_path)])
    if receipt.get("share_receipt") and not args.json and args.sim is None:
        print(f"Please share the receipt with the openxl330 maintainers: open an issue at {install.ISSUES_URL} and attach it.")
    return 0


UNINSTALL_CONFIRM = ("This sends ID {id} to its retained ROBOTIS loader, where it waits for Dynamixel Wizard's Firmware "
                     "Recovery (EXPERIMENTAL). Continue? [y/N] ")
RESTORE_CONFIRM = ("This writes the recorded stock settings, ID and baud to the stock servo at ID 1 / 57600 baud "
                   "(EXPERIMENTAL). Continue? [y/N] ")


def _confirm_experimental(args):
    """uninstall / uninstall-restore: one confirmation before the port opens (a real run only; --yes skips it)."""
    if args.yes or args.sim is not None:
        return
    what = "uninstall" if args.experimental_command == "uninstall" else "uninstall-restore"
    if not _isatty(sys.stdin):
        raise Error(f"not done: {what} needs a confirmation. Run it in a terminal, or pass --yes. Nothing was sent")
    prompt = UNINSTALL_CONFIRM.format(id=args.id) if what == "uninstall" else RESTORE_CONFIRM
    print(prompt, end="", file=sys.stderr, flush=True)
    if sys.stdin.readline().strip().lower() not in ("y", "yes"):
        raise Error("not done: not confirmed. Nothing was sent")


def _confirm_plan(args, plan):
    """flash-all: show the plan (stderr), then ask once (a real run in a terminal; --yes and --sim skip it)."""
    print("Plan:", file=sys.stderr)
    for s in plan["stock"]:
        note = " (untested stock version)" if s["untested"] else ""
        print(f"  ID {s['id']}: stock V{s['firmware']} at {s['baud']} baud -> install {args.release}, keeps ID "
              f"{s['id']}, starts at {APP_BAUD} baud{note}", file=sys.stderr)
    for s in plan["fw231"]:
        print(f"  ID {s['id']}: openxl330 firmware at {s['baud']} baud -> skip", file=sys.stderr)
    if args.yes or args.sim is not None:
        return True
    if not _isatty(sys.stdin):
        raise Error("not installed: flash-all replaces the stock firmware of every servo in the plan, so it needs a "
                    "confirmation. Run it in a terminal, or pass --yes. Nothing was sent beyond pings")
    print(FLASH_ALL_CONFIRM, end="", file=sys.stderr, flush=True)
    if sys.stdin.readline().strip().lower() not in ("y", "yes"):
        raise Error("not installed: not confirmed. Nothing was sent beyond pings")
    return True


def _experimental(args, bus):
    from . import experimental
    print(EXPERIMENTAL_NOTE, file=sys.stderr)
    records = _sim_records(args)
    root = records if records is not None else install.records_root()
    if args.experimental_command == "flash-all":
        bars = {}

        def progress(servo_id, done, total):
            if servo_id not in bars:
                for bar in bars.values():
                    bar.close()
                print(f"Installing firmware on ID {servo_id}: about 30 s; do not unplug or interrupt.",
                      file=sys.stderr, flush=True)
                bars[servo_id] = _Progress()
            bars[servo_id](done, total)
        try:
            result = experimental.flash_all(bus.wire, confirm=lambda plan: _confirm_plan(args, plan),
                                            release=args.release, allow_untested=args.allow_untested,
                                            records=records, on_progress=progress, variant=args.variant)
        finally:
            for bar in bars.values():
                bar.close()
        if not result["plan"]["stock"] and not args.json:
            print("nothing to install: no stock XL330-M288 answered", file=sys.stderr)
        rows = [(s["id"], f"stock V{s['from_firmware']} at {s['baud']}", "installed",
                 ", ".join(f"ID {i} {what}" for i, what in sorted(s["bystanders"].items())) or "-",
                 root / s["run_dir"] / "receipt.json") for s in result["installed"]]
        rows += [(s["id"], f"openxl330 at {s['baud']}", "skipped", "-", "-") for s in result["skipped"]]
        _show(args, result, ("ID", "Before", "Result", "Other stock servos", "Receipt"), sorted(rows))
        for s in result["installed"]:
            for warning in s["warnings"]:
                print(f"warning: ID {s['id']}: {warning}", file=sys.stderr)
    elif args.experimental_command == "uninstall":
        out = experimental.uninstall_prepare(bus.wire, servo_id=args.id, allow_untested=args.allow_untested,
                                             records=records)
        if args.json:
            print(json.dumps(dict(id=args.id, record=str(out["record"]), next=out["next"]), indent=2))
        else:
            _table(("ID", "Record"), [(args.id, out["record"])])
            for line in out["next"].splitlines():          # commands (indented) stay whole, for copying
                print(line if line.startswith("  ") else textwrap.fill(line, 100, break_on_hyphens=False,
                                                                       break_long_words=False))
    else:
        result = experimental.uninstall_restore(bus.wire, record=args.record, baud=args.stock_baud, records=records)
        _show(args, result, ("ID", "Baud", "Firmware", "Settings written"),
              [(result["id"], result["baud"], f"stock V{result['firmware']}",
                ", ".join(map(str, result["written"])) or "none (already equal)")])
    return 0


def _show_joint(args, joint):
    _show(args, dict(id=args.id, **asdict(joint)), ("ID", "Zero (counts)", "Direction"),
          [(args.id, joint.zero_counts, joint.direction)])


def _reset_calibration(args, calibration):
    try:
        calibration.reset(args.id)
        if args.direction is not None:
            calibration.set(args.id, direction=args.direction)
        if calibration.path is not None:
            calibration.save()
    except OSError as exc:
        raise CalibrationError(f"cannot save calibration at {calibration.path} ({exc}); "
                               "check the path and permissions") from exc
    _show_joint(args, calibration.joint(args.id))


def _group_command(args, bus):
    group = bus.group(args.ids)
    if args.command == "diag":
        data = group.diagnostics()
        _show(args, data, ("ID", "Diagnostic", "Value"),
              ((i, key, json.dumps(value) if isinstance(value, (dict, list)) else value)
               for i, values in data.items() for key, value in values.items()))
    elif args.command == "configure":
        gains = group.configure(kp=args.kp, kd=args.kd)
        _show(args, gains, ("ID", "Kp (N·m/rad)", "Kd (N·m·s/rad)", "Compensation"),
              ((i, g["kp"], g["kd"], "on" if g["comp"] else "off") for i, g in gains.items()))
    elif args.command == "identify":
        group.identify(args.id)
        result = {i: i == args.id for i in group.ids}
        _show(args, result, ("ID", "LED"), ((i, "ON" if on else "OFF") for i, on in result.items()))
    else:
        for i in group.ids:
            bus.servo(i).clear_faults()
        _show_result(args, "Faults cleared")
    return 0


def _show_result(args, state):
    _show(args, {"ids": args.ids, "result": state}, ("ID", "Result"), ((i, state) for i in args.ids))


OFF, NOT_ANSWERING = "OFF", "not answering"


def _off_one(wire, servo_id):
    """Write 64 <- 0 (torque OFF on stock firmware and on FW231 alike) to one ID, without identifying it first, then
    read the byte back where the firmware allows it. Returns the result for the table."""
    try:
        wire.write(servo_id, TORQUE_ENABLE, b"\0")
    except NoReply:
        return NOT_ANSWERING
    except Rejected as exc:
        return f"refused OFF (error {exc.error})"
    try:
        enabled = wire.read(servo_id, TORQUE_ENABLE, 1)[0]
    except (NoReply, Rejected):
        return OFF                                       # the acknowledged OFF write is the confirmation
    return "still reports torque enabled" if enabled else OFF


def _off(args, bus):
    """OFF that is always possible: no identity probe (a newer or unknown firmware still gets the write)."""
    wire = bus.wire
    if args.all:
        try:
            wire.drain_stale()                           # bytes already on the line must not stop the OFF
            wire.write_broadcast(TORQUE_ENABLE, b"\0")
        except BusUncertain as exc:                      # still broadcast OFF once, without judging the line
            sent = wire.emergency_off(broadcast=True)
            raise Error(f"the bus state became unknown before OFF could be broadcast at {wire.port.baudrate} baud "
                        f"({str(exc).split(' The bus state is unknown', 1)[0].rstrip('.')}); "
                        + ("OFF was then broadcast once, unconfirmed" if sent else "nothing could be sent")
                        + ". Support the joints and check them on a new connection") from exc
        try:
            ids = sorted(wire.scan(252))                 # who to confirm: every ID that answers a ping here
        except BusUncertain as exc:
            raise Error(f"OFF was broadcast at {wire.port.baudrate} baud, but the scan that confirms it failed, so "
                        f"no servo is confirmed OFF: {exc}") from exc
    else:
        ids = list(args.ids)
    results, uncertain = {}, None
    for k, servo_id in enumerate(ids):
        try:
            results[servo_id] = _off_one(wire, servo_id)
        except BusUncertain as exc:                      # the rest still get OFF once, unconfirmed
            uncertain = exc
            for i in ids[k:]:
                results[i] = "unconfirmed (bus uncertain)"
            for i in wire.emergency_off(ids[k:]):
                results[i] = "OFF sent once, unconfirmed (bus uncertain)"
            break
    data = {"ids": ids, "result": {str(i): r for i, r in results.items()}, "broadcast": bool(args.all)}
    if results or args.json:
        _show(args, data, ("ID", "Result"), results.items())
    if args.all and not ids:
        raise Error(f"OFF was broadcast at {wire.port.baudrate} baud, but no servo answered a ping there to confirm "
                    f"it: check power and wiring, or find the servos with `openxl330 scan {_port_option(args)} "
                    f"--all-bauds`")
    failed = [i for i, r in results.items() if r != OFF]
    if uncertain is not None:
        reason = str(uncertain).split(" The bus state is unknown", 1)[0].rstrip(".")
        sent = [i for i, r in results.items() if r.startswith("OFF sent once")]
        raise Error(f"Could not confirm torque OFF for {_id_list(failed)}: the bus state became unknown ({reason}). "
                    + (f"OFF was then sent once, unconfirmed, to {_id_list(sent)}. " if sent else "")
                    + "Support the joints and check them on a new connection")
    if failed:
        raise Error("Could not confirm torque OFF for " + ", ".join(f"ID {i} ({results[i]})" for i in failed))
    return 0


def _set_baud(args, bus):
    result = bus.set_baud(args.to)
    if result["missing"]:
        raise Error(f'Baud {args.to} unconfirmed for IDs {_id_list(result["missing"])} '
                    f'(confirmed IDs: {_id_list(result["confirmed"])}). '
                    f'Check for those IDs at {result["previous"]} and {args.to} baud with scan --all-bauds.')
    if not result["confirmed"]:
        raise Error(f'no servo confirmed {args.to} baud. Check which servos answer with scan --all-bauds.')
    _show(args, result, ("Baud", "Confirmed", "Missing"),
          [(result["baud"], _id_list(result["confirmed"]), _id_list(result["missing"]))])
    if result["baud"] != APP_BAUD:
        print(f"note: the bus now runs at {result['baud']} baud: pass --baud {result['baud']} on later commands. "
              f"Servos start at {APP_BAUD} baud again after a power cycle.", file=sys.stderr)
    return 0


def _dispatch(args, bus):
    if args.command in ("install", "update", "recover"):
        return _installation(args, bus)
    if args.command == "experimental":
        return _experimental(args, bus)
    if args.command == "scan":
        _scan(args, bus)
    elif args.command == "status":
        return _status(args, bus)
    elif args.command == "set-baud":
        return _set_baud(args, bus)
    elif args.command == "set-id":
        return _set_id(args, bus)
    elif args.command == "calibrate":
        if args.reset:
            _reset_calibration(args, bus.calibration)
        else:
            _show_joint(args, bus.servo(args.id).calibrate(zero="here", direction=args.direction))
    elif args.command == "off":
        return _off(args, bus)
    else:
        return _group_command(args, bus)
    return 0


def _id_list(ids):
    return ", ".join(map(str, ids)) or "-"


def _cli_hints(message, args):
    """Advice written for Python callers ("call disable() first", "call reset() or reopen the port") in CLI terms."""
    port = _port_option(args)
    ids = getattr(args, "ids", None)
    spec = ",".join(map(str, ids)) if ids else "<IDs>"
    message = message.replace("call disable() first", f"turn it off first with `openxl330 off {port} --ids {spec}`")
    message = re.sub(r"(?i)\bcall configure\(\) first", f"configure it first with `openxl330 configure {port} --ids "
                     f"{spec}`", message)
    # Each CLI command opens the port afresh: "reset()" / "reopen the port" mean running the command again.
    for library in ("call reset() or reopen the port", "reset() again", "reopen the port", "open the port again"):
        message = message.replace(library, "run the command again")
    return message


def _plain_adapter_warnings(shown):
    """A warnings.showwarning that prints AdapterWarning as one plain `warning: ...` line on stderr (no file and line
    number) and passes anything else to `shown`."""
    def show(message, category, filename, lineno, file=None, line=None):
        if issubclass(category, AdapterWarning):
            print(f"warning: {message}", file=sys.stderr)
        else:
            shown(message, category, filename, lineno, file, line)
    return show


def main(argv: list[str] | None = None) -> int:
    """Run one command and return its process exit status."""
    parser = _parser()
    try:
        args = parser.parse_args(argv)
        if args.port is not None and args.sim is not None:
            parser.error("choose either --port or --sim")
        reset_calibration = args.command == "calibrate" and args.reset
        if not reset_calibration and args.command not in ("ports", "version") and args.port is None and args.sim is None:
            parser.error("--port is required unless --sim is used")
    except SystemExit as exc:
        return exc.code

    with warnings.catch_warnings():
        warnings.simplefilter("always", AdapterWarning)
        warnings.showwarning = _plain_adapter_warnings(warnings.showwarning)
        return _run(args, reset_calibration)


def _run(args, reset_calibration):
    try:
        if args.command == "version":
            releases = images.release_names()
            data = dict(version=__version__, releases=releases, default_release=images.DEFAULT_RELEASE)
            if not args.json:
                print(f"openxl330 {__version__}")
            _show(args, data, ("Bundled release", "Default"),
                  ((name, "yes" if name == images.DEFAULT_RELEASE else "") for name in releases))
        elif args.command == "ports":
            if args.sim is None:
                ports = Bus.ports()
            else:
                from .sim import SimPort
                ports = [SimPort.port]
            _show(args, ports, ("Port",), ((port,) for port in ports))
        else:
            if args.command == "install":
                _confirm_install(args)                  # before the port opens: nothing is sent until confirmed
            elif args.command == "set-id":
                _confirm_set_id(args)
            elif args.command == "experimental" and args.experimental_command != "flash-all":
                _confirm_experimental(args)             # flash-all asks after its census, with the plan
            if args.sim is not None:
                with Bus.simulated(args.sim) as bus:
                    if args.baud is not None:
                        bus.wire.set_baud(args.baud)
                    return _dispatch(args, bus)
            if reset_calibration:
                _reset_calibration(args, Calibration.load())
                return 0
            recovering = args.command == "recover"
            default_baud = LOADER_BAUD if recovering else APP_BAUD
            baud = args.baud if args.baud is not None else default_baud
            calibration = None if args.command in ("calibrate", "status", "set-id") else Calibration()
            # The loader may be waiting for its prompt, so skip the adapter's ping; OFF must not depend on it either.
            with Bus(args.port, baud=baud, calibration=calibration,
                     check_adapter=not (recovering or args.command == "off")) as bus:
                return _dispatch(args, bus)
    except KeyboardInterrupt as exc:
        print("error: interrupted", file=sys.stderr)
        recovery = getattr(exc, "recovery", None)
        if recovery:
            note = getattr(exc, "recovery_note", None)
            if note:
                print(note.strip(), file=sys.stderr)
            print(f"To recover the servo, run: {recovery}", file=sys.stderr)
        return 130
    except Error as exc:
        if args.debug:
            print("Traceback (most recent call last):", file=sys.stderr)
            traceback.print_tb(exc.__traceback__, file=sys.stderr)
        # InstallError already includes the recovery command in its message.
        print(f"error: {_cli_hints(str(exc), args)}", file=sys.stderr)
        return 1
    return 0
