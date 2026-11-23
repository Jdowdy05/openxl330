"""Maintainer self-test integration and failure cases on an in-memory bus."""
from __future__ import annotations

import json
import struct

import pytest

from openxl330 import Bus, Calibration, InvalidArgument, Refused, Servo, images, registers
from openxl330 import selftest
from openxl330.selftest import main, run
from openxl330.sim import FakeClock, SimPort, SimServo


@pytest.fixture
def simulated_bus():
    program = images.bundled(images.DEFAULT_RELEASE).packaged().raw
    devices = [SimServo(i, mode="app", program=program, seed=i) for i in (1, 2, 3)]
    with Bus(SimPort(devices, FakeClock()), calibration=Calibration(), check_adapter=False) as bus:
        yield bus, devices


def test_selftest_passes_on_sim(tmp_path, capsys):
    out = tmp_path / "selftest"
    assert main(["--sim", "3", "--ids", "1,2,3", "--roundtrip-id", "2", "--out", str(out)]) == 0
    assert "PASS" in capsys.readouterr().out
    report = json.loads((out / "report.json").read_text(encoding="utf-8"))
    assert report["ok"] and report["simulated"]
    for stage in ("identity", "soak", "motion", "group_read", "group_move", "lease", "roundtrip"):
        assert report[stage]["ok"] and report[stage]["health"]["ok"]
    assert report["soak"]["elapsed_s"] >= 30
    assert report["soak"]["samples"] > 1
    assert report["group_read"]["completed"] == 1000
    assert report["group_read"]["misses"] == 0
    assert report["group_read"]["median_ms"] > 0
    assert report["roundtrip"]["id"] == 2
    assert report["roundtrip"]["release"] == images.DEFAULT_RELEASE
    assert report["roundtrip"]["loader_unchanged"] and report["roundtrip"]["nvm_unchanged"]
    assert {"loader_crc32", "nvm_sha256", "fingerprint"}.isdisjoint(report["roundtrip"])
    assert not report["cleanup"]["off_unconfirmed_ids"]


@pytest.mark.parametrize("ids", [[], [1, 1], [True], [253]])
def test_invalid_ids_fail_before_probing_or_creating_report(simulated_bus, tmp_path, ids):
    bus, _ = simulated_bus
    out = tmp_path / "selftest"
    with pytest.raises(InvalidArgument):
        run(bus, ids, None, out)
    assert not bus.port.tx_log and not out.exists()


def test_repeat_selftest_preserves_previous_report(simulated_bus, tmp_path, monkeypatch, capsys):
    bus, _ = simulated_bus
    report = tmp_path / "report.json"
    previous = '{"ok": false, "first_run": true}\n'
    report.write_text(previous, encoding="utf-8")
    with pytest.raises(Refused, match="report already exists"):
        run(bus, [1, 2, 3], None, tmp_path)
    assert report.read_text(encoding="utf-8") == previous
    assert not bus.port.tx_log

    def unexpected_open(_):
        pytest.fail("self-test opened the bus before checking the existing report")

    monkeypatch.setattr(selftest, "_simulated_bus", unexpected_open)
    assert main(["--sim", "3", "--ids", "1,2,3", "--out", str(tmp_path)]) == 1
    assert "report already exists" in capsys.readouterr().err


@pytest.mark.parametrize("failure", ["fault", "deadline", "500-us tick"])
def test_selftest_stops_during_soak_on_health_failure(simulated_bus, tmp_path, monkeypatch, failure):
    bus, devices = simulated_bus
    device = devices[0]
    start = bus.wire.clock()
    original = device.read

    def unhealthy(address, size):
        if bus.port.clock.t - start > 2:
            if failure == "deadline" and (address, size) == (852, 16):
                return 0, struct.pack("<4I", 1, 0, 1, 100 | (2000 << 16))
            if failure == "fault":
                device.faults = 16
            if failure == "500-us tick" and (address, size) == registers.STATUS:
                data = bytearray(original(address, size)[1])
                struct.pack_into("<I", data, 40, 500)
                return 0, data
        return original(address, size)

    monkeypatch.setattr(device, "read", unhealthy)
    report = run(bus, [1, 2, 3], None, tmp_path)
    assert report["identity"]["ok"] and not report["soak"]["ok"]
    assert report["soak"]["elapsed_s"] < 30
    assert report["motion"]["skipped"] and report["group_read"]["skipped"]
    detail = report["soak"]["health"]["servos"][1]
    if failure == "deadline":
        assert detail["deadline_misses"] == 1 and detail["deadline_window"] == 0
    elif failure == "fault":
        assert "temperature" in detail["faults"]
    else:
        assert detail["worst_tick_us"] == 500
    assert not any(device.enabled for device in devices)


def test_missed_fast_read_fails_instead_of_averaging_remaining_samples(simulated_bus, tmp_path, monkeypatch):
    bus, devices = simulated_bus
    original = devices[1].on_frame

    def missing(frame, clock):
        if frame["instruction"] == 0x8A:
            return None
        return original(frame, clock)

    monkeypatch.setattr(devices[1], "on_frame", missing)
    report = run(bus, [1, 2, 3], None, tmp_path)
    assert report["motion"]["ok"]
    assert not report["group_read"]["ok"] and report["group_read"]["misses"] == 1
    assert report["group_read"]["completed"] == 0
    assert report["group_move"]["skipped"] and report["lease"]["skipped"]
    assert not any(device.enabled for device in devices)


@pytest.mark.parametrize(("offset", "field", "value"),
                         [(20, "hardware_faults", 4), (40, "worst_tick_us", 500)])
def test_selftest_stops_for_a_fault_in_a_live_motion_sample(
        simulated_bus, tmp_path, monkeypatch, offset, field, value):
    bus, devices = simulated_bus
    device = devices[0]
    original = device.read
    injected = False

    def read(address, size):
        nonlocal injected
        code, raw = original(address, size)
        if (address, size) == registers.STATUS and device.enabled and not injected:
            raw = bytearray(raw)
            struct.pack_into("<I", raw, offset, value)
            injected = True
        return code, raw

    monkeypatch.setattr(device, "read", read)
    report = run(bus, [1, 2, 3], None, tmp_path)
    assert injected and report["soak"]["ok"]
    motion = report["motion"]["servos"][1]
    assert not motion["ok"] and not motion["arm_hold"]["ok"]
    assert motion["arm_hold"][field][1] == value
    assert motion["samples"] == []
    assert report["group_read"]["skipped"] and report["cleanup"]["ok"]
    assert not any(device.enabled for device in devices)


def test_uncertain_bus_preserves_unconfirmed_off_in_the_report(simulated_bus, tmp_path, monkeypatch, capsys):
    bus, devices = simulated_bus
    device = devices[0]
    original = device.write
    sent_at_failure = []

    def disrupt(address, data, broadcast=False):
        status = original(address, data, broadcast)
        if device.enabled and address == registers.TARGET[0]:
            sent_at_failure.append(len(bus.port.tx_log))
            bus.wire.mark_uncertain("injected loss of framing while moving")
        return status

    monkeypatch.setattr(device, "write", disrupt)
    monkeypatch.setattr(selftest, "_simulated_bus", lambda _: bus)
    assert main(["--sim", "3", "--ids", "1,2,3", "--out", str(tmp_path)]) == 1
    assert "torque OFF could not be confirmed" in capsys.readouterr().err
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert not report["motion"]["ok"] and report["group_read"]["skipped"]
    assert report["motion"]["health"]["problems"] == ["bus is uncertain"]
    assert report["wire_uncertain"] and not report["cleanup"]["ok"]
    assert report["cleanup"]["off_unconfirmed_ids"] == [1]
    assert device.enabled and len(bus.port.tx_log) == sent_at_failure[0]


def test_selftest_fails_if_arm_does_not_hold_the_original_position(simulated_bus, tmp_path, monkeypatch):
    bus, devices = simulated_bus
    device = devices[0]
    original = device.write

    def arm_with_stale_target(address, data, broadcast=False):
        if address == registers.TORQUE_ENABLE and data == b"\x01":
            device.target_counts = round(device.position) + 300
            device.duration_ticks = 1
        return original(address, data, broadcast)

    monkeypatch.setattr(device, "write", arm_with_stale_target)
    report = run(bus, [1, 2, 3], None, tmp_path)
    move = report["motion"]["servos"][1]
    assert not move["ok"] and move["samples"] == []
    assert move["arm_hold"]["error_deg"][1] > 1
    assert not device.enabled and report["group_read"]["skipped"]


def test_unexpected_motion_exception_disables_armed_servo_and_propagates(simulated_bus, tmp_path, monkeypatch):
    bus, devices = simulated_bus

    def fail_while_armed(self, *args, **kwargs):
        assert devices[0].enabled
        raise RuntimeError("unexpected application error")

    monkeypatch.setattr(Servo, "move", fail_while_armed)
    with pytest.raises(RuntimeError, match="unexpected application error"):
        run(bus, [1, 2, 3], None, tmp_path)
    assert not any(device.enabled for device in devices)
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert not report["motion"]["ok"] and report["cleanup"]["ok"]


def test_interrupted_cleanup_reports_unconfirmed_off_and_returns_130(simulated_bus, tmp_path, monkeypatch, capsys):
    bus, devices = simulated_bus
    sent_at_interrupt = []

    def fail_while_armed(self, *args, **kwargs):
        assert devices[0].enabled
        raise Refused("injected error during motion")

    def interrupt_disable(self):
        assert devices[0].enabled
        sent_at_interrupt.append(len(bus.port.tx_log))
        raise KeyboardInterrupt

    monkeypatch.setattr(selftest, "_simulated_bus", lambda _: bus)
    monkeypatch.setattr(Servo, "move", fail_while_armed)
    monkeypatch.setattr(Servo, "disable", interrupt_disable)
    assert main(["--sim", "3", "--ids", "1,2,3", "--out", str(tmp_path)]) == 130
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert report["interrupted"] and not report["cleanup"]["ok"]
    assert report["cleanup"]["off_unconfirmed_ids"] == [1]
    assert "cleanup interrupted" in capsys.readouterr().err.lower()
    assert len(bus.port.tx_log) == sent_at_interrupt[0]


def test_roundtrip_refuses_an_unbundled_running_release(simulated_bus, tmp_path):
    bus, devices = simulated_bus
    device = devices[1]
    device.release_id = {**device.release_id, "number": 99}
    report = run(bus, [1, 2, 3], 2, tmp_path)
    assert not report["roundtrip"]["ok"]
    assert "bundled release" in report["roundtrip"]["error"]
    assert bus.wire.loader_phase is None and device.mode == "app"


def test_roundtrip_checks_fresh_release_id_without_extra_full_scan(simulated_bus, tmp_path, monkeypatch):
    bus, devices = simulated_bus
    scan = bus.scan
    scans = []

    def track_scan(*args, **kwargs):
        scans.append(True)
        return scan(*args, **kwargs)

    def wrong_variant():
        devices[1].release_id = {**devices[1].release_id, "variant": "pd"}
        return {"ok": True}

    monkeypatch.setattr(bus, "scan", track_scan)
    monkeypatch.setattr(selftest._SelfTest, "lease", lambda _: wrong_variant())
    report = run(bus, [1, 2, 3], 2, tmp_path)
    assert scans == [True]
    assert not report["roundtrip"]["ok"]
    assert "bundled release" in report["roundtrip"]["error"]
    assert bus.wire.loader_phase is None and devices[1].mode == "app"


def test_lease_cleanup_does_not_rewrite_existing_config_if_pd_configuration_fails(simulated_bus, monkeypatch, tmp_path):
    bus, _ = simulated_bus
    test = selftest._SelfTest(bus, [1, 2, 3], None, tmp_path)
    test.identity()
    servo = test.servos[1]
    servo.lease(2.0, action="hold")

    def configure_failure():
        raise Refused("injected configuration failure")

    monkeypatch.setattr(servo, "configure", configure_failure)
    with pytest.raises(Refused, match="configuration failure"):
        test.lease()
    tx_count = len(bus.port.tx_log)
    assert test.cleanup()["ok"]
    assert len(bus.port.tx_log) == tx_count
