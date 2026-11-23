"""Maintainer qualification for FW231 on a bench bus or the in-memory simulator."""
from __future__ import annotations

import argparse
import json
import math
import statistics
import struct
import sys
from dataclasses import asdict
from pathlib import Path
from tempfile import TemporaryDirectory

from . import Bus, Calibration, Error, InvalidArgument, NoReply, Refused, Rejected, images, install
from .bus import _check_ids
from .cli import _ids, _servo_id, _sim_count
from .registers import RELEASE_ID, decode_release_id
from .sim import FakeClock, SimPort, SimServo
from .wire import APP_BAUD

_DEADLINE = (852, 16)
_TICK_LIMIT_US = 500
_SOAK_S = 30
_MOVE_S = 0.3
_SETTLE_S = 0.2
_READS = 1000
_CHECKS = ("identity", "soak", "motion", "group_read", "group_move", "lease", "roundtrip")


class _SelfTest:
    def __init__(self, bus, ids, roundtrip_id, out):
        self.bus = bus
        self.wire = bus.wire
        self.ids = ids
        self.roundtrip_id = roundtrip_id
        self.out = out
        self.simulated = isinstance(bus.port, SimPort)
        self.group = None
        self.deadline_start = {}
        self.off_unconfirmed = set()
        self.lease_restore = None

    def health(self):
        if self.wire.uncertain:
            return {"ok": False, "problems": ["bus is uncertain"]}
        statuses = self.group.status()
        deadlines = self.bus.sync_read(self.ids, *_DEADLINE)
        servos, problems = {}, []
        for i, status in statuses.items():
            total, window, window_max, _ = struct.unpack("<4I", deadlines[i])
            delta = (total - self.deadline_start.setdefault(i, total)) & 0xFFFFFFFF
            hardware_faults = status.raw["hardware_faults"]
            servos[i] = dict(enabled=status.enabled, faults=status.faults, hardware_faults=hardware_faults,
                             worst_tick_us=status.worst_tick_us, deadline_total=total, deadline_misses=delta,
                             deadline_window=window, deadline_window_max=window_max)
            if status.enabled:
                problems.append(f"ID {i} is still enabled")
            if status.fault_bits or hardware_faults:
                problems.append(f"ID {i} faults {status.fault_bits:#x}, hardware faults {hardware_faults:#x}")
            if delta or window:
                problems.append(f"ID {i} deadline misses {delta} since start, {window} in the current window")
            if status.worst_tick_us >= _TICK_LIMIT_US:
                problems.append(f"ID {i} worst tick {status.worst_tick_us} us reached the {_TICK_LIMIT_US} us limit")
        return {"ok": not problems, "servos": servos, "problems": problems}

    def identity(self):
        self.bus.scan()
        self.group = self.bus.group(self.ids)
        self.servos = {i: self.bus.servo(i) for i in self.ids}
        devices = {}
        for i, servo in self.servos.items():
            info = asdict(servo.info)
            info.pop("label", None)
            info["features"] = sorted(servo.info.features)
            devices[i] = info
        missing_fast_read = [i for i, device in devices.items() if "fast_sync_read" not in device["features"]]
        if missing_fast_read:
            return {"ok": False, "devices": devices, "error": f"Fast Sync Read is unavailable on IDs {missing_fast_read}"}
        return {"ok": True, "devices": devices}

    def soak(self):
        start = self.wire.clock()
        samples = 0
        while True:
            health = self.health()
            samples += 1
            elapsed = self.wire.clock() - start
            if not health["ok"] or elapsed >= _SOAK_S:
                return {"ok": health["ok"], "elapsed_s": elapsed, "samples": samples, "health": health}
            self.wire.sleep(min(1.0, _SOAK_S - elapsed))

    def _disable(self, servo):
        servo.disable()
        if servo.status().enabled:
            raise Refused(f"ID {servo.id} still shows torque enabled after OFF")
        self.off_unconfirmed.discard(servo.id)

    def _sample_positions(self, statuses, targets):
        errors = {i: math.degrees(abs(statuses[i].position - target)) for i, target in targets.items()}
        off = [i for i, status in statuses.items() if not status.enabled]
        faults = {i: status.faults for i, status in statuses.items() if status.fault_bits}
        hardware_faults = {i: status.raw["hardware_faults"] for i, status in statuses.items()
                           if status.raw["hardware_faults"]}
        worst_tick_us = {i: status.worst_tick_us for i, status in statuses.items()}
        ok = (not off and not faults and not hardware_faults
              and max(worst_tick_us.values(), default=0) < _TICK_LIMIT_US
              and all(error < 1 for error in errors.values()))
        return {"error_deg": errors, "off_ids": off, "faults": faults, "hardware_faults": hardware_faults,
                "worst_tick_us": worst_tick_us, "ok": ok}

    def _sample_move(self, move, read, home, angle):
        self.wire.sleep(_SETTLE_S)
        held = self._sample_positions(read(), home)
        samples = []
        if held["ok"]:
            for offset in (angle, 0, -angle, 0):
                targets = {i: pos + math.radians(offset) for i, pos in home.items()}
                move(targets)
                self.wire.sleep(_MOVE_S + _SETTLE_S)
                sample = self._sample_positions(read(), targets)
                samples.append({"offset_deg": offset, **sample})
                if not sample["ok"]:
                    break
        return {"ok": held["ok"] and all(sample["ok"] for sample in samples), "arm_hold": held, "samples": samples}

    def motion(self):
        motions = {}
        for i, servo in self.servos.items():
            servo.configure()
            home = {i: servo.position}
            self.off_unconfirmed.add(i)
            servo.enable()
            motions[i] = self._sample_move(
                lambda targets: servo.move(targets[i], duration=_MOVE_S, profile="quintic"),
                lambda: {i: servo.status()}, home, 10,
            )
            self._disable(servo)
            if not motions[i]["ok"]:
                break
        return {"ok": all(move["ok"] for move in motions.values()), "servos": motions}

    def group_read(self):
        elapsed = []
        misses, failure = 0, {}
        for _ in range(_READS):
            start = self.wire.clock()
            try:
                self.group.read(fast=True)
            except NoReply as exc:
                misses = 1
                failure = {"error": str(exc), "missing_ids": exc.missing, "partial_ids": list(exc.partial)}
                break
            elapsed.append(self.wire.clock() - start)
        return {"ok": len(elapsed) == _READS, "attempted": len(elapsed) + misses, "completed": len(elapsed),
                "misses": misses, "median_ms": 1e3 * statistics.median(elapsed) if elapsed else None, **failure}

    def group_move(self):
        self.group.configure()
        home = {i: state.position for i, state in self.group.read().items()}
        self.off_unconfirmed.update(self.ids)
        self.group.enable()
        result = self._sample_move(
            lambda targets: self.group.move(targets, duration=_MOVE_S, profile="quintic"), self.group.status, home, 5,
        )
        self.group.disable()
        statuses = self.group.status()
        self.off_unconfirmed.difference_update(i for i, status in statuses.items() if not status.enabled)
        return result

    def lease(self):
        servo = self.servos[self.ids[0]]
        previous = servo.lease_status()
        servo.configure()
        self.lease_restore = (servo, previous["ms"] / 1000, previous["action"])
        servo.lease(0.3, action="off")
        self.off_unconfirmed.add(servo.id)
        servo.enable()
        for _ in range(3):
            self.wire.sleep(0.14)
            servo.heartbeat()
        renewed = servo.lease_status()
        held = servo.status().enabled
        self.wire.sleep(0.4)
        expired = servo.lease_status()
        still_enabled = servo.status().enabled
        self._disable(servo)
        self._restore_lease()
        return {"ok": held and renewed["state"] == "running" and not still_enabled
                and expired["state"] == "expired" and expired["expiries"] == renewed["expiries"] + 1,
                "id": servo.id, "enabled_with_heartbeats": held, "enabled_after_expiry": still_enabled,
                "renewed": renewed, "expired": expired}

    def _restore_lease(self):
        if self.lease_restore is not None:
            servo, seconds, action = self.lease_restore
            if servo.id not in self.off_unconfirmed:
                servo.lease(seconds, action=action)
                self.lease_restore = None

    def roundtrip(self):
        i = self.roundtrip_id
        if self.bus.port.baudrate != APP_BAUD:
            raise Refused("The round trip needs the bus at 1 Mbaud")
        try:
            identity = decode_release_id(self.bus.read(i, *RELEASE_ID))
        except Rejected as exc:
            raise Refused(f"ID {i} did not provide a release ID for a same-release round trip") from exc
        release = next((name for name in images.release_names() if images.bundled(name).release_id == identity), None)
        if release is None:
            raise Refused(f"ID {i} release {identity['number']} does not match a bundled release for a same-release round trip")
        records = self.out / "sim-records" if self.simulated else None
        receipt = install.update(self.wire, servo_id=i, release=release, records=records)
        steps = {step["step"]: step for step in receipt["steps"]}
        backup, verified = steps.get("backed_up", {}), steps.get("verified", {})
        loader = bool(backup.get("loader_crc32")) and backup["loader_crc32"] == verified.get("loader_crc32")
        nvm = bool(backup.get("nvm_sha256")) and backup["nvm_sha256"] == verified.get("nvm_sha256")
        self.deadline_start[i] = 0  # update boots the same image and resets its counters
        return {"ok": receipt["completed"] and receipt["servo_id"] == i and receipt["release"] == release
                and loader and nvm, "id": i, "release": release, "loader_unchanged": loader, "nvm_unchanged": nvm,
                "receipt": receipt["run_dir"], "loader_known": receipt["loader_known"],
                "warning_count": len(receipt["warnings"])}

    def cleanup(self):
        issues = []
        interrupted = False
        try:
            for i in sorted(self.off_unconfirmed):
                if self.wire.uncertain or self.wire.loader_possible:
                    break
                try:
                    self._disable(self.servos[i])
                except Exception as exc:
                    issues.append(f"ID {i}: {exc}")
            if not self.wire.uncertain and not self.wire.loader_possible:
                try:
                    self._restore_lease()
                except Exception as exc:
                    issues.append(f"Could not restore the lease: {exc}")
        except KeyboardInterrupt:
            interrupted = True
            issues.append("Cleanup interrupted")
        if self.off_unconfirmed and self.wire.uncertain:
            issues.append("Bus uncertain, so torque OFF could not be confirmed")
        elif self.off_unconfirmed:
            issues.append(f"Torque OFF could not be confirmed for IDs {sorted(self.off_unconfirmed)}")
        return {"ok": not interrupted and not self.off_unconfirmed and self.lease_restore is None,
                "off_unconfirmed_ids": sorted(self.off_unconfirmed), "interrupted": interrupted, "issues": issues}

    def health_after_error(self):
        if self.wire.uncertain:
            return {"ok": False, "problems": ["bus is uncertain"]}
        if self.wire.loader_possible:
            return {"ok": False, "problems": ["a servo may be in the loader"]}
        if self.group is None:
            return {"ok": False, "problems": ["group identity was not established"]}
        try:
            return self.health()
        except Exception as exc:
            return {"ok": False, "problems": [f"Could not read group health: {exc}"]}


def _run_checks(test, report):
    for check in _CHECKS:
        if not report["ok"]:
            break
        if check == "roundtrip" and test.roundtrip_id is None:
            report[check] = {"ok": True, "skipped": True, "reason": "no round-trip ID selected"}
            continue
        result = {}
        try:
            result = getattr(test, check)()
            if "health" not in result:
                result["health"] = test.health()
            result["ok"] = result["ok"] and result["health"]["ok"]
        except BaseException as exc:
            report["interrupted"] = isinstance(exc, KeyboardInterrupt)
            result.update(ok=False, error=str(exc) or ("interrupted" if report["interrupted"] else type(exc).__name__))
            for field in ("recovery", "recovery_note"):
                instruction = getattr(exc, field, None)
                if instruction:
                    result[field] = instruction
            if not isinstance(exc, (Error, KeyboardInterrupt)):
                raise
        finally:
            report[check] = result
            report["ok"] = result["ok"]


def _existing_report(path):
    return Refused(f"A self-test report already exists at {path}. Choose a new --out directory.")


def run(bus: Bus, ids: list[int], roundtrip_id: int | None, out: Path) -> dict:
    """Run the qualification, stop after the first failure, and write a new report.json in out."""
    ids = _check_ids(ids)
    if roundtrip_id is not None and (type(roundtrip_id) is not int or roundtrip_id not in ids):
        raise InvalidArgument("round-trip ID must be one of the IDs under test")
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    report_path = out / "report.json"
    try:
        with report_path.open("x", encoding="utf-8") as report_file:
            report_file.write('{"ok": false, "incomplete": true}\n')
    except FileExistsError as exc:
        raise _existing_report(report_path) from exc
    test = _SelfTest(bus, ids, roundtrip_id, out)
    report = {"ok": True, "simulated": test.simulated, "ids": ids, "roundtrip_id": roundtrip_id,
              "interrupted": False}
    report.update({check: {"ok": False, "skipped": True, "reason": "an earlier check failed"} for check in _CHECKS})
    original_error = None
    try:
        _run_checks(test, report)
    except BaseException as exc:
        original_error = exc
        raise
    finally:
        try:
            report["cleanup"] = test.cleanup()
            report["interrupted"] |= report["cleanup"]["interrupted"]
            for check in _CHECKS:
                result = report[check]
                if not result.get("skipped") and "health" not in result:
                    if report["cleanup"]["interrupted"]:
                        result["health"] = {"ok": False, "problems": ["cleanup interrupted"]}
                    else:
                        result["health"] = test.health_after_error()
                    break
            report["wire_uncertain"] = bus.wire.uncertain
            report["ok"] = report["ok"] and report["cleanup"]["ok"] and not report["wire_uncertain"]
            with TemporaryDirectory(prefix=".selftest-", dir=out) as staging:
                final = Path(staging) / "report.json"
                final.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
                final.replace(report_path)
        except (Exception, KeyboardInterrupt) as exc:
            if original_error is None:
                raise
            if hasattr(original_error, "add_note"):
                original_error.add_note(f"Self-test diagnostics could not be finished: {exc}")
    return report


def _simulated_bus(n):
    program = images.bundled(images.DEFAULT_RELEASE).packaged().raw
    devices = [SimServo(i, mode="app", program=program, seed=i) for i in range(1, n + 1)]
    return Bus(SimPort(devices, FakeClock()), calibration=Calibration(), check_adapter=False)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Qualify FW231 on a bench bus. The motion tests move each selected joint by up to 10 degrees.",
        allow_abbrev=False,
    )
    connection = parser.add_mutually_exclusive_group(required=True)
    connection.add_argument("--port", help="adapter port")
    connection.add_argument("--sim", type=_sim_count, metavar="N", help="use N simulated servos")
    parser.add_argument("--ids", type=_ids, required=True, help="test servo IDs, e.g. 1,2,3")
    parser.add_argument("--roundtrip-id", type=_servo_id, help="optionally reinstall the current bundled release on this ID")
    parser.add_argument("--out", type=Path, default=Path("selftest-report"), help="directory for a new report.json")
    try:
        args = parser.parse_args(argv)
        if args.roundtrip_id is not None and args.roundtrip_id not in args.ids:
            parser.error("--roundtrip-id must be one of --ids")
    except SystemExit as exc:
        return exc.code
    try:
        report_path = args.out / "report.json"
        if report_path.exists() or report_path.is_symlink():
            raise _existing_report(report_path)
        bus = _simulated_bus(args.sim) if args.sim is not None else Bus(args.port)
        with bus:
            report = run(bus, args.ids, args.roundtrip_id, args.out)
    except KeyboardInterrupt:
        print("error: interrupted", file=sys.stderr)
        return 130
    except (Error, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    if not report["ok"]:
        for check in _CHECKS:
            result = report[check]
            if result["ok"] or result.get("skipped"):
                continue
            reason = result.get("error") or " / ".join(result["health"]["problems"]) or "see report for measurements"
            print(f"{check}: {reason}", file=sys.stderr)
            if result.get("recovery_note") and result["recovery_note"] not in reason:
                print(result["recovery_note"], file=sys.stderr)
            if result.get("recovery") and result["recovery"] not in reason:
                print(f"To recover the servo, run: {result['recovery']}", file=sys.stderr)
            break
        for issue in report["cleanup"]["issues"]:
            print(issue, file=sys.stderr)
    print(f"{'PASS' if report['ok'] else 'FAIL'} {args.out / 'report.json'}")
    return 130 if report["interrupted"] else int(not report["ok"])


if __name__ == "__main__":
    raise SystemExit(main())
