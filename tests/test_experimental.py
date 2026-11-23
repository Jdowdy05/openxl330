"""Experimental commands against the simulated bus: flash-all (every stock servo on the bus, stock bystanders allowed)
and uninstall (hand a servo to Dynamixel Wizard's Firmware Recovery, then restore its recorded stock settings).
Evidence for the bystander rule: a 2-servo bench (2026-11-23), where a stock V53 servo at 57600 ignored the loader
session and kept its control table byte for byte. No port is opened; records go to tmp dirs."""
from __future__ import annotations

import io
import json
import struct
import sys

import pytest
from conftest import make_bus

from openxl330 import Bus, Calibration
from openxl330 import experimental as X
from openxl330 import install
from openxl330.cli import main
from openxl330.errors import InstallError, InvalidArgument, Rejected
from openxl330.sim import SimPort, SimServo, default_eeprom
from openxl330.wire import APP_BAUD, LOADER_BAUD


@pytest.fixture
def records(tmp_path):
    return tmp_path / "records"


def stock(nv_id, *, seed, version=53, baud_index=1, model=1200, **fields):
    e = default_eeprom(nv_id, baud_index=baud_index)
    e[0:2] = struct.pack("<H", model)
    e[6] = version
    for addr, value in fields.items():
        e[int(addr[1:])] = value
    return SimServo(nv_id, mode="stock", seed=seed, eeprom=e)


def f9_frames(port):
    return [d for _, d in port.tx_log if len(d) >= 8 and d[:4] == b"\xff\xff\xfd\x00" and d[7] == 0xF9]


def wizard_recovery(servo):
    """What Dynamixel Wizard's Firmware Recovery leaves (bench 2026-11-23): stock firmware, factory EEPROM defaults
    (ID 1, 57600), calibration untouched."""
    assert servo.mode == "loader"
    e = default_eeprom(1)
    e[6] = 53
    servo.eeprom = e
    servo.reset("stock")


def YES(plan):
    return True


# ---------------------------------------------------------------------------------------- simulator: stock EEPROM
def test_sim_stock_eeprom_writes_need_torque_off_and_apply_like_hardware(sim_bus):
    s = stock(1, seed=1)
    wire, _ = sim_bus([s], LOADER_BAUD)
    wire.write(1, 9, b"\x00")                                  # return delay
    wire.write(1, 7, b"\x05")                                  # the reply still comes from ID 1, then the ID is 5
    assert s.effective_id() == 5 and s.eeprom[9] == 0
    wire.write(5, 8, b"\x03")                                  # 1 Mbaud, after the reply
    wire.set_baud(APP_BAUD)
    assert wire.scan(252) == {5: dict(model=1200, firmware=53, alert=False)}
    wire.write(5, 64, b"\x01")
    with pytest.raises(Rejected) as e:
        wire.write(5, 11, b"\x00")                             # EEPROM is locked while torque is on
    assert e.value.error == 7


# ---------------------------------------------------------------------------------------- flash-all
def test_flash_all_converts_every_stock_servo_and_keeps_ids(sim_bus, records):
    a, b = stock(2, seed=2), stock(3, seed=3)
    wire, port = sim_bus([a, b], LOADER_BAUD)
    seen = []
    result = X.flash_all(wire, confirm=lambda plan: seen.append(plan) or True, records=records)
    assert (a.mode, b.mode, a.effective_id(), b.effective_id()) == ("app", "app", 2, 3)
    assert [p["id"] for p in seen[0]["stock"]] == [2, 3] and seen[0]["fw231"] == []
    assert [r["id"] for r in result["installed"]] == [2, 3] and all(r["completed"] for r in result["installed"])
    first = result["installed"][0]
    assert first["bystanders"] == {"3": "unchanged"} and result["installed"][1]["bystanders"] == {}
    assert len(f9_frames(port)) == 2 and not result["cancelled"]


def test_flash_all_skips_servos_already_on_fw231(sim_bus, records):
    done = stock(4, seed=4)
    w0, _ = sim_bus([done], LOADER_BAUD)
    assert install.from_stock(w0, stock_id=4, records=records)["completed"]
    fresh = stock(5, seed=5)
    wire, _ = sim_bus([done, fresh], LOADER_BAUD)
    result = X.flash_all(wire, confirm=YES, records=records)
    assert [r["id"] for r in result["installed"]] == [5] and [s["id"] for s in result["skipped"]] == [4]
    assert fresh.mode == "app"


@pytest.mark.parametrize("servos, why", [
    (lambda: [stock(1, seed=1), stock(1, seed=2)], "census"),                         # duplicate IDs
    (lambda: [stock(2, seed=1), stock(2, seed=2, baud_index=3)], "several bauds"),     # one ID at two bauds
    (lambda: [stock(2, seed=1), stock(3, seed=2, model=1060)], "XL330-M288"),          # another model
])
def test_flash_all_refuses_before_any_f9(sim_bus, records, servos, why):
    group = servos()
    wire, port = sim_bus(group, LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        X.flash_all(wire, confirm=YES, records=records)
    assert why in str(e.value) and e.value.recovery is None
    assert f9_frames(port) == [] and all(s.mode == "stock" for s in group)


def test_flash_all_refuses_error_replies(sim_bus, records, monkeypatch):
    a, b = stock(2, seed=1), stock(3, seed=2)
    original = b.on_frame

    def ping_error(f, clock):
        r = original(f, clock)
        if r is not None and f["instruction"] == 1:
            return (r[0], b.status(2, struct.pack("<HB", 1200, 53)))
        return r
    b.on_frame = ping_error
    wire, port = sim_bus([a, b], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        X.flash_all(wire, confirm=YES, records=records)
    assert "error" in str(e.value) and f9_frames(port) == []


def test_flash_all_cancel_sends_nothing(sim_bus, records):
    a, b = stock(2, seed=1), stock(3, seed=2)
    wire, port = sim_bus([a, b], LOADER_BAUD)
    result = X.flash_all(wire, confirm=lambda plan: False, records=records)
    assert result["cancelled"] and result["installed"] == [] and f9_frames(port) == []
    assert (a.mode, b.mode) == ("stock", "stock")


def test_flash_all_reports_a_bystander_that_changed(sim_bus, records, monkeypatch):
    a, b = stock(2, seed=1), stock(3, seed=2)
    wire, _ = sim_bus([a, b], LOADER_BAUD)
    real = install._verify_installed

    def tamper(*args, **kw):
        b.eeprom[9] ^= 1                                      # the bystander's return delay changes mid-install
        return real(*args, **kw)
    monkeypatch.setattr(install, "_verify_installed", tamper)
    with pytest.raises(InstallError) as e:
        X.flash_all(wire, confirm=YES, records=records)
    assert "bystander" in str(e.value) and "ID 3" in str(e.value) and a.mode == "app"


def test_flash_all_stops_at_the_first_failure(sim_bus, records, monkeypatch):
    a, b = stock(2, seed=1), stock(3, seed=2)
    wire, _ = sim_bus([a, b], LOADER_BAUD)
    calls = []
    real = install._verify_installed

    def fail_first(*args, **kw):
        calls.append(1)
        raise InstallError("simulated failure after the launch", step="verify", recovery="openxl330 update --id 2")
    monkeypatch.setattr(install, "_verify_installed", fail_first)
    with pytest.raises(InstallError) as e:
        X.flash_all(wire, confirm=YES, records=records)
    assert e.value.recovery == "openxl330 update --id 2" and len(calls) == 1 and b.mode == "stock"
    assert "ID 2" in str(e.value) and "not started" in str(e.value)


def test_public_from_stock_still_refuses_stock_bystanders(sim_bus, records):
    a, b = stock(2, seed=1), stock(3, seed=2)
    wire, port = sim_bus([a, b], LOADER_BAUD)
    with pytest.raises(InstallError):
        install.from_stock(wire, stock_id=2, records=records)
    assert f9_frames(port) == []


def test_v53_is_proven():
    assert (1200, 53) in install.PROVEN_STOCK


# ---------------------------------------------------------------------------------------- uninstall
def _ours(sim_bus, records, sid, seed, **fields):
    s = stock(sid, seed=seed, **fields)
    w, _ = sim_bus([s], LOADER_BAUD)
    assert install.from_stock(w, stock_id=sid, records=records)["completed"]
    return s


def test_uninstall_prepare_records_settings_and_leaves_the_loader_waiting(sim_bus, records):
    s = _ours(sim_bus, records, 2, 7, a9=0, a11=0)            # user settings: return delay 0, current mode
    wire, port = sim_bus([s], APP_BAUD)
    out = X.uninstall_prepare(wire, servo_id=2, records=records)
    rec = json.loads(out["record"].read_text())
    low = bytes.fromhex(rec["stock_settings_hex"])
    assert rec["id"] == 2 and low[9] == 0 and low[11] == 0 and rec["original_stock_baud"] == 57600
    assert s.mode == "loader" and s.loader_text == "banner" and wire.loader_possible
    assert "Firmware Recovery" in out["next"] and len(f9_frames(port)) == 1


def test_uninstall_prepare_refuses_an_id1_clash_at_57600(sim_bus, records):
    ours = _ours(sim_bus, records, 2, 7)
    other = stock(1, seed=9)
    wire, port = sim_bus([ours, other], APP_BAUD)
    with pytest.raises(InstallError) as e:
        X.uninstall_prepare(wire, servo_id=2, records=records)
    assert "ID 1" in str(e.value) and f9_frames(port) == [] and ours.mode == "app"


def test_uninstall_prepare_refuses_a_stock_target(sim_bus, records):
    s = stock(2, seed=1, baud_index=3)
    wire, port = sim_bus([s], APP_BAUD)
    with pytest.raises(InstallError):
        X.uninstall_prepare(wire, servo_id=2, records=records)
    assert f9_frames(port) == []


def test_uninstall_restore_writes_the_recorded_settings_id_and_baud(sim_bus, records):
    s = _ours(sim_bus, records, 2, 7, a9=0, a11=0)
    wire, _ = sim_bus([s], APP_BAUD)
    rec = X.uninstall_prepare(wire, servo_id=2, records=records)["record"]
    wizard_recovery(s)
    assert (s.effective_id(), s.eeprom[9], s.eeprom[11]) == (1, 250, 3)
    wire, _ = sim_bus([s], LOADER_BAUD)
    out = X.uninstall_restore(wire, record=rec, baud=1_000_000)
    assert (s.mode, s.effective_id(), s.baud, s.eeprom[9], s.eeprom[11]) == ("stock", 2, 1_000_000, 0, 0)
    assert out["settings_match"] and out["id"] == 2 and set(out["written"]) == {9, 11}


def test_uninstall_restore_defaults_to_the_original_stock_baud(sim_bus, records):
    s = _ours(sim_bus, records, 2, 7)
    wire, _ = sim_bus([s], APP_BAUD)
    rec = X.uninstall_prepare(wire, servo_id=2, records=records)["record"]
    wizard_recovery(s)
    wire, _ = sim_bus([s], LOADER_BAUD)
    assert X.uninstall_restore(wire, record=rec)["baud"] == 57600 and s.baud == 57600


def test_uninstall_restore_needs_a_freshly_recovered_servo(sim_bus, records):
    s = _ours(sim_bus, records, 2, 7)
    wire, _ = sim_bus([s], APP_BAUD)
    rec = X.uninstall_prepare(wire, servo_id=2, records=records)["record"]
    s.reset("app")                                            # Wizard was never run
    wire, _ = sim_bus([s], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        X.uninstall_restore(wire, record=rec)
    assert "ID 1" in str(e.value)


# ---------------------------------------------------------------------------------------- recover after a stock trip
def test_recover_accepts_a_changed_nv_journal_with_the_same_calibration(sim_bus, records):
    """After a trip back to stock (Wizard's recovery resets the stock settings, kept in the NV journal half of the
    servo's own 8 KB), recover compares the calibration page, not the whole 8 KB (the 2026-11-23 bench found the
    deployment tool refusing here)."""
    s = _ours(sim_bus, records, 2, 7)
    s.flash[0xF000:0xF010] = bytes(16)                        # the NV journal half changed; calibration did not
    s.reset("loader")
    s.pending_banner = False
    wire, _ = sim_bus([s], LOADER_BAUD)
    r = install.recover(wire, servo_id=2, loader_fresh=True, records=records)
    assert r["completed"] and r["known_servo"] and r["kept"]["nvm.bin"].startswith("changed:")


def test_after_prepare_recover_with_a_fresh_prompt_keeps_openxl330(sim_bus, records):
    """Changing one's mind after uninstall_prepare: the loader waits at its first prompt, so recover --loader-fresh
    (named in the prepare's advice) puts FW231 back."""
    s = _ours(sim_bus, records, 2, 7)
    wire, _ = sim_bus([s], APP_BAUD)
    out = X.uninstall_prepare(wire, servo_id=2, records=records)
    assert "openxl330 recover --port SIM --id 2 --loader-fresh" in out["next"]
    wire, _ = sim_bus([s], LOADER_BAUD)
    r = install.recover(wire, servo_id=2, loader_fresh=True, records=records)
    assert r["completed"] and (s.mode, s.effective_id()) == ("app", 2)


# ---------------------------------------------------------------------------------------- CLI
def use_sim(monkeypatch, *servos, baud=APP_BAUD):
    def simulated(cls, n):
        assert n == len(servos)
        return cls(SimPort(servos), baud=baud, calibration=Calibration(), check_adapter=False)
    monkeypatch.setattr(Bus, "simulated", classmethod(simulated))


def test_cli_flash_all_shows_the_plan_and_installs(monkeypatch, capsys):
    a, b = stock(2, seed=2), stock(3, seed=3)
    use_sim(monkeypatch, a, b)
    assert main(["--sim", "2", "experimental", "flash-all"]) == 0
    out, err = capsys.readouterr()
    assert "EXPERIMENTAL" in err and "ID 2: stock V53 at 57600 baud -> install" in err and "ID 3: stock V53" in err
    assert (a.mode, b.mode) == ("app", "app")
    rows = [line.split()[0] for line in out.splitlines()[1:]]
    assert rows == ["2", "3"] and "ID 3 unchanged" in out


def test_cli_flash_all_needs_a_confirmation_outside_a_terminal(monkeypatch, capsys):
    a, b = stock(2, seed=2), stock(3, seed=3)
    port = SimPort([a, b])
    monkeypatch.setattr("openxl330.bus._open_serial", lambda name, baud: port)
    monkeypatch.setattr("openxl330.adapters.ftdi_latency_warning", lambda _: None)
    monkeypatch.setattr(sys, "stdin", io.StringIO(""))
    assert main(["--port", "/dev/ttyUSB9", "experimental", "flash-all"]) == 1
    out, err = capsys.readouterr()
    assert "needs a confirmation" in err and "pass --yes" in err and f9_frames(port) == []
    assert (a.mode, b.mode) == ("stock", "stock")


def test_cli_uninstall_then_restore(monkeypatch, capsys, records):
    s = _ours(make_bus, records, 2, 7, a9=0)
    use_sim(monkeypatch, s)
    assert main(["--sim", "1", "--json", "experimental", "uninstall", "--id", "2"]) == 0
    out, err = capsys.readouterr()
    done = json.loads(out)
    assert "Firmware Recovery" in done["next"] and s.mode == "loader"
    wizard_recovery(s)
    use_sim(monkeypatch, s, baud=LOADER_BAUD)
    assert main(["--sim", "1", "experimental", "uninstall-restore", "--record", done["record"]]) == 0
    out, err = capsys.readouterr()
    assert (s.mode, s.effective_id(), s.baud, s.eeprom[9]) == ("stock", 2, 57600, 0)
    assert out.splitlines()[1].split()[:3] == ["2", "57600", "stock"]


# ---------------------------------------------------------------------------------------- review fixes
def test_flash_all_refuses_an_untested_bystander_version_at_57600_unless_allowed(sim_bus, records):
    a, b = stock(2, seed=1, version=52), stock(3, seed=2, version=52)
    wire, port = sim_bus([a, b], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        X.flash_all(wire, confirm=YES, records=records)
    assert "only stock V53 was tested" in str(e.value) and "Firmware Update" in str(e.value) and f9_frames(port) == []
    wire, _ = sim_bus([a, b], LOADER_BAUD)
    result = X.flash_all(wire, confirm=YES, records=records, allow_untested=True)
    assert [r["id"] for r in result["installed"]] == [2, 3] and (a.mode, b.mode) == ("app", "app")


def test_flash_all_refuses_an_unproven_stock_version(sim_bus, records):
    wire, port = sim_bus([stock(2, seed=1, version=51)], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        X.flash_all(wire, confirm=YES, records=records)
    assert "stock V51, which is not proven" in str(e.value) and f9_frames(port) == []


def test_flash_all_census_names_the_duplicate_id_problem(sim_bus, records):
    wire, port = sim_bus([stock(1, seed=1), stock(1, seed=2)], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        X.flash_all(wire, confirm=YES, records=records)
    assert "census at 57600 baud failed" in str(e.value) and "Two servos at one ID collide" in str(e.value)
    assert f9_frames(port) == []


def test_a_stock_bystander_at_115200_is_checked_too(sim_bus, records):
    a, far = stock(2, seed=1), stock(5, seed=2, baud_index=2)
    wire, _ = sim_bus([a, far], LOADER_BAUD)
    result = X.flash_all(wire, confirm=YES, records=records)
    assert result["installed"][0]["bystanders"] == {"5": "unchanged"}


def test_a_bystander_ram_difference_is_a_warning_but_torque_on_stops(sim_bus, records, monkeypatch):
    a, b = stock(2, seed=1), stock(3, seed=2)
    wire, _ = sim_bus([a, b], LOADER_BAUD)
    real = install._verify_installed

    def led_on(*args, **kw):
        b.led = 1                                            # RAM 65: reported, not a stop
        return real(*args, **kw)
    monkeypatch.setattr(install, "_verify_installed", led_on)
    result = X.flash_all(wire, confirm=YES, records=records)
    assert result["installed"][0]["bystanders"] == {"3": "unchanged"}
    assert any("RAM byte(s) [65]" in w for w in result["installed"][0]["warnings"])
    c, d = stock(4, seed=3), stock(5, seed=4)
    wire, _ = sim_bus([c, d], LOADER_BAUD)

    def torque_on(*args, **kw):
        d.enabled = 1                                        # torque enable 64: strict
        return real(*args, **kw)
    monkeypatch.setattr(install, "_verify_installed", torque_on)
    with pytest.raises(InstallError) as e:
        X.flash_all(wire, confirm=YES, records=records)
    assert "bystander ID 5: control table byte(s) [64] changed" in str(e.value) and e.value.installed == [4]


def test_a_garbled_bystander_check_points_to_a_scan(sim_bus, records, monkeypatch):
    a, b = stock(2, seed=1), stock(3, seed=2)
    wire, _ = sim_bus([a, b], LOADER_BAUD)
    real = install._verify_installed

    def garble(*args, **kw):
        out = real(*args, **kw)
        original = b.on_frame

        def bad(f, clock):
            r = original(f, clock)
            return (r[0], r[1][:5] + b"\x00\x00") if r and f["instruction"] == 1 else r
        b.on_frame = bad
        return out
    monkeypatch.setattr(install, "_verify_installed", garble)
    with pytest.raises(InstallError) as e:
        X.flash_all(wire, confirm=YES, records=records)
    assert e.value.step == "bystanders" and e.value.recovery == "openxl330 scan --port SIM --all-bauds"
    assert a.mode == "app" and json.loads(next((records / "_runs").glob("*id2/receipt.json")).read_text())["completed"]


@pytest.mark.parametrize("layout", ["stock_at_57600_target_at_1m", "stock_at_the_targets_baud"])
def test_public_from_stock_refuses_stock_bystanders_in_every_layout(sim_bus, records, layout):
    if layout == "stock_at_57600_target_at_1m":
        target, other = stock(2, seed=1, baud_index=3), stock(3, seed=2)
    else:
        target, other = stock(2, seed=1, baud_index=3), stock(3, seed=2, baud_index=3)
    wire, port = sim_bus([target, other], APP_BAUD)
    with pytest.raises(InstallError):
        install.from_stock(wire, stock_id=2, stock_baud=APP_BAUD, records=records)
    assert f9_frames(port) == [] and (target.mode, other.mode) == ("stock", "stock")


def test_restore_refuses_while_the_recorded_id_still_runs_openxl330(sim_bus, records):
    """Review finding 1: prepare, change one's mind (recover), then a new factory servo appears at ID 1 / 57600: the
    record must not be written into it while the original ID 2 still answers."""
    s = _ours(sim_bus, records, 2, 7)
    wire, _ = sim_bus([s], APP_BAUD)
    rec = X.uninstall_prepare(wire, servo_id=2, records=records)["record"]
    wire, _ = sim_bus([s], LOADER_BAUD)
    assert install.recover(wire, servo_id=2, loader_fresh=True, records=records)["completed"]
    newcomer = stock(1, seed=30)
    wire, port = sim_bus([s, newcomer], LOADER_BAUD)
    with pytest.raises(InstallError) as e:
        X.uninstall_restore(wire, record=rec)
    assert "still runs openxl330" in str(e.value) and newcomer.effective_id() == 1
    assert not [d for _, d in port.tx_log if len(d) > 7 and d[7] == 3]          # no write at all


def test_restore_bauds_are_the_ones_the_adapters_reach(sim_bus, records):
    with pytest.raises(InvalidArgument):
        X.uninstall_restore(sim_bus([])[0], record=records / "x.json", baud=4_000_000)


def test_prepare_advice_names_the_nv_id_for_the_way_back():
    lines = X._wizard_steps("P", 7, "rec.json", 3).splitlines()
    assert "  openxl330 recover --port P --id 3 --loader-fresh" in lines and any("stock ID 3, not 7" in l for l in lines)


def test_cli_uninstall_needs_a_confirmation_outside_a_terminal(monkeypatch, capsys, records):
    s = _ours(make_bus, records, 2, 7)
    port = SimPort([s])
    monkeypatch.setattr("openxl330.bus._open_serial", lambda name, baud: port)
    monkeypatch.setattr("openxl330.adapters.ftdi_latency_warning", lambda _: None)
    monkeypatch.setattr(sys, "stdin", io.StringIO(""))
    assert main(["--port", "/dev/ttyUSB9", "experimental", "uninstall", "--id", "2"]) == 1
    out, err = capsys.readouterr()
    assert "needs a confirmation" in err and port.tx_log == [] and s.mode == "app"


def test_plan_does_not_flag_the_first_servo_it_installs(sim_bus, records):
    """Review 2 nit: the lowest-ID stock servo is installed first and never waits through another's loader session."""
    wire, _ = sim_bus([stock(2, seed=1, version=52), stock(3, seed=2, version=53)], LOADER_BAUD)
    result = X.flash_all(wire, confirm=YES, records=records)
    assert [r["id"] for r in result["installed"]] == [2, 3]


def test_uninstall_prepare_refuses_an_untested_stock_bystander_at_57600(sim_bus, records):
    ours = _ours(sim_bus, records, 2, 7)
    old = stock(5, seed=31, version=50)
    wire, port = sim_bus([ours, old], APP_BAUD)
    with pytest.raises(InstallError) as e:
        X.uninstall_prepare(wire, servo_id=2, records=records)
    assert "ID 5" in str(e.value) and "only stock V53 was tested" in str(e.value) and f9_frames(port) == []
    wire, port = sim_bus([ours, old], APP_BAUD)
    X.uninstall_prepare(wire, servo_id=2, allow_untested=True, records=records)
    assert ours.mode == "loader" and old.mode == "stock"
