"""CLI tests. Every invocation uses an in-memory bus."""
from __future__ import annotations

import argparse
import io
import json
import runpy
import sys
import warnings
from pathlib import Path

import pytest

from conftest import UNKNOWN_LOADER
from openxl330 import AdapterWarning, Bus, Calibration, InstallError, __version__, images, install, paths, registers
from openxl330 import cli
from openxl330.cli import main
from openxl330.sim import SimPort, SimServo, default_eeprom


@pytest.fixture(autouse=True)
def isolated_home(tmp_path, monkeypatch):
    monkeypatch.setenv("OPENXL330_HOME", str(tmp_path))


@pytest.fixture
def run_cli(capsys):
    def run(*args, n=1):
        code = main(["--sim", str(n), *args])
        out, err = capsys.readouterr()
        return code, out, err

    return run


def app_servo(servo_id=1):
    return SimServo(servo_id, mode="app", program=images.bundled(images.DEFAULT_RELEASE).packaged().raw)


def use_sim(monkeypatch, *servos, calibration=None, baud=1_000_000):
    def simulated(cls, n):
        assert n == len(servos)
        return cls(SimPort(servos), baud=baud, calibration=calibration or Calibration(), check_adapter=False)

    monkeypatch.setattr(Bus, "simulated", classmethod(simulated))


def test_version_lists_bundled(run_cli):
    code, out, err = run_cli("version")
    assert code == 0 and not err
    assert __version__ in out and images.DEFAULT_RELEASE in out and "default" in out.lower()
    assert all(name in out for name in images.release_names())


def test_scan_sim_table(run_cli):
    code, out, err = run_cli("scan", "--all-bauds", n=3)
    assert code == 0 and not err
    lines = out.splitlines()
    assert "ID" in lines[0] and "Model" in lines[0]
    assert [line.split()[0] for line in lines[1:]] == ["1", "2", "3"]
    assert all("XL330-M288" in line for line in lines[1:])


@pytest.mark.parametrize("args", [
    ("--port", "SIM", "scan", "--sim", "1"),
    ("--sim", "1", "scan", "--port", "SIM"),
])
def test_simulation_and_port_are_mutually_exclusive(args, capsys):
    assert main(list(args)) == 2
    captured = capsys.readouterr()
    assert not captured.out
    assert "choose either --port or --sim" in captured.err


@pytest.mark.parametrize("args", [
    ("install", "--new-id", "4"),
    ("install", "--label", "base"),
    ("install", "--boot-baud", "2000000"),
    ("update", "--id", "1", "--new-id", "4"),
    ("recover", "--new-id", "4"),
    ("set-id", "--id", "1", "--new-id", "4"),
])
def test_rejects_obsolete_identity_options(run_cli, args):
    code, out, err = run_cli(*args)
    assert code == 2 and not out and "usage:" in err


def test_install_keeps_id_and_warns_about_unknown_loader(run_cli, monkeypatch):
    factory = SimServo(4, loader=UNKNOWN_LOADER)
    use_sim(monkeypatch, factory)
    code, out, err = run_cli("install", "--stock-id", "4", "--stock-baud", "57600", "--json")
    assert code == 0, err
    receipt = json.loads(out)
    assert receipt["completed"] and receipt["servo_id"] == 4
    assert factory.effective_id() == 4
    assert any("unknown loader version" in warning for warning in receipt["warnings"])
    assert "warning: unknown loader version" in err


def test_install_tells_what_it_does_and_shows_progress(run_cli, monkeypatch):
    """A notice before anything is installed, then upload progress, all on stderr: --json stdout stays pure JSON. Not
    a terminal: plain lines (no carriage returns), one per 10 %."""
    use_sim(monkeypatch, SimServo(4))
    code, out, err = run_cli("install", "--stock-id", "4", "--json")
    assert code == 0, err
    assert json.loads(out)["completed"]
    lines = err.splitlines()
    notice = "Installing firmware on ID 4: about 30 s; do not unplug or interrupt."
    progress = [line for line in lines if line.startswith("Uploading firmware:")]
    assert notice in lines and lines.index(notice) < lines.index(progress[0])
    assert "\r" not in err and progress[0].startswith("Uploading firmware: 0% (0/")
    assert progress[-1].startswith("Uploading firmware: 100% (") and len(progress) == 11


def test_progress_line_updates_in_place_on_a_terminal():
    class Terminal(io.StringIO):
        def isatty(self):
            return True

    stream = Terminal()
    progress = cli._Progress(stream)
    for done in (0, 64, 128, 200):
        progress(done, 200)
    text = stream.getvalue()
    assert text.count("\r") == 4 and text.endswith("100% (200/200 B)\n") and text.count("\n") == 1
    progress = cli._Progress(stream)
    progress(10, 200)
    progress.close()                                       # an interrupted upload ends its line
    assert stream.getvalue().endswith("(10/200 B)\n")


def test_sim_install_keeps_records_out_of_the_users_folder(run_cli):
    """--sim installs write their records to a temporary folder (named on stderr), report the known loader (no
    warning, nothing to share), and print commands in the same `--port SIM` form as everything else."""
    code, out, err = run_cli("update", "--id", "1")
    assert code == 0, err
    assert not (paths.home() / "records").exists()
    line = next(line for line in err.splitlines() if line.startswith("simulation: records go to "))
    folder = Path(line.split("records go to ", 1)[1].split(" (", 1)[0])
    receipt = next(folder.glob("_runs/*/receipt.json"))
    assert str(receipt) in out and "warning:" not in err and "share" not in out
    assert json.loads(receipt.read_text())["loader_known"] is True


def test_install_needs_the_stock_id(run_cli):
    code, out, err = run_cli("install", "--yes")
    assert code == 2 and not out and "--stock-id" in err


def test_install_help_says_the_id_is_permanent(run_cli):
    code, out, err = run_cli("install", "--help")
    assert code == 0 and not err
    text = " ".join(out.split())
    assert ("The ID you set in Dynamixel Wizard before installing is permanent in v1; changing it after flashing is "
            "not supported (except `openxl330 update --allow-id-change`, which moves a servo back to its stock NV "
            "ID).") in text
    assert "no supported way back" in text and "about 30 s" in text


def test_every_option_has_help():
    parser = cli._parser()
    commands = next(a for a in parser._actions if isinstance(a, argparse._SubParsersAction))
    missing = []
    for name, sub in [("openxl330", parser), *commands.choices.items()]:
        for action in sub._actions:
            if action.option_strings and action.help in (None, "") and not isinstance(action, argparse._HelpAction):
                missing.append(f"{name} {action.option_strings[0]}")
    assert not missing


def _stock_on_port(monkeypatch, servo):
    """A named port that is really an in-memory bus with this servo; records whether it was opened."""
    opened = []

    def open_serial(name, baud):
        opened.append(name)
        return SimPort([servo])

    monkeypatch.setattr("openxl330.bus._open_serial", open_serial)
    monkeypatch.setattr("openxl330.adapters.ftdi_latency_warning", lambda _: None)
    return opened


class _Stdin(io.StringIO):
    def isatty(self):
        return True


@pytest.mark.parametrize("answer", ["", "n\n", "no\n"])
def test_install_asks_for_confirmation_on_a_terminal(monkeypatch, capsys, answer):
    servo = SimServo(4)
    opened = _stock_on_port(monkeypatch, servo)
    monkeypatch.setattr(sys, "stdin", _Stdin(answer))
    assert main(["install", "--port", "/dev/ttyUSB9", "--stock-id", "4"]) == 1
    out, err = capsys.readouterr()
    assert "This replaces the stock firmware; there is no supported way back. Continue? [y/N]" in err
    assert "not confirmed" in err and not out and opened == [] and servo.mode == "stock"


def test_install_runs_after_yes_on_a_terminal(monkeypatch, capsys):
    servo = SimServo(4)
    opened = _stock_on_port(monkeypatch, servo)
    monkeypatch.setattr(sys, "stdin", _Stdin("y\n"))
    assert main(["install", "--port", "/dev/ttyUSB9", "--stock-id", "4", "--json"]) == 0
    out, err = capsys.readouterr()
    assert json.loads(out)["completed"] and opened == ["/dev/ttyUSB9"] and servo.mode == "app"
    assert "Continue? [y/N]" in err and "factory default ID 1" not in err
    assert (paths.home() / "records").is_dir(), "a real install keeps its records in the user's records folder"


def test_install_without_a_terminal_needs_yes(monkeypatch, capsys):
    servo = SimServo(4)
    opened = _stock_on_port(monkeypatch, servo)
    monkeypatch.setattr(sys, "stdin", io.StringIO("y\n"))                  # not a terminal: never read
    assert main(["install", "--port", "/dev/ttyUSB9", "--stock-id", "4"]) == 1
    out, err = capsys.readouterr()
    assert "--yes" in err and "Nothing was sent" in err and opened == [] and servo.mode == "stock"
    assert main(["install", "--port", "/dev/ttyUSB9", "--stock-id", "4", "--yes"]) == 0
    out, err = capsys.readouterr()
    assert "Continue?" not in err and servo.mode == "app" and opened == ["/dev/ttyUSB9"]


def test_install_at_factory_id_1_warns_unless_yes(monkeypatch, capsys):
    warning = ("warning: factory default ID 1; set a unique ID in Dynamixel Wizard first if this servo will share "
               "a bus")
    use_sim(monkeypatch, SimServo(1))
    assert main(["--sim", "1", "install", "--stock-id", "1"]) == 0
    assert warning in capsys.readouterr().err
    use_sim(monkeypatch, SimServo(1))
    assert main(["--sim", "1", "install", "--stock-id", "1", "--yes"]) == 0
    assert "factory default ID 1" not in capsys.readouterr().err


def test_install_duplicate_id_shows_wizard_advice(run_cli, monkeypatch):
    factory = SimServo(4)
    use_sim(monkeypatch, factory, app_servo(4))
    code, out, err = run_cli("install", "--stock-id", "4", n=2)
    assert code == 1 and not out
    assert "unique ID" in err and "Dynamixel Wizard" in err
    assert factory.mode == "stock"


def test_update_uses_the_requested_release(run_cli):
    code, out, err = run_cli("update", "--id", "1", "--release", images.DEFAULT_RELEASE,
                             "--allow-id-change", "--json")
    assert code == 0, err
    receipt = json.loads(out)
    assert receipt["completed"] and receipt["release"] == images.DEFAULT_RELEASE
    assert receipt["servo_id"] == 1 and receipt["allow_id_change"] is True


def test_error_is_one_line_with_recovery(run_cli, monkeypatch):
    def interrupted(wire, **kwargs):
        recovery = install.recovery_command(wire, kwargs["servo_id"])
        raise InstallError("upload failed", step="upload", recovery=recovery)

    monkeypatch.setattr(install, "update", interrupted)
    code, out, err = run_cli("update", "--id", "1", "--allow-id-change")
    assert code == 1 and not out
    assert err.splitlines()[-2:] == [
        "error: upload failed",
        "To recover the servo, run: openxl330 recover --port SIM --id 1",
    ]
    assert err.splitlines()[-3] == "Installing firmware on ID 1: about 30 s; do not unplug or interrupt."


def test_interrupt_has_exit_130_and_recovery(run_cli, monkeypatch):
    def interrupted(wire, **kwargs):
        error = KeyboardInterrupt()
        error.recovery = install.recovery_command(wire, kwargs["stock_id"])
        raise error

    monkeypatch.setattr(install, "from_stock", interrupted)
    code, out, err = run_cli("install", "--stock-id", "1", "--yes")
    assert code == 130 and not out
    assert err.count("openxl330 recover --port SIM --id 1") == 1


def test_recover_without_previous_records_warns(run_cli, monkeypatch, tmp_path):
    use_sim(monkeypatch, SimServo(1, mode="loader"), baud=57_600)
    failed = tmp_path / "previous-receipt.json"
    failed.write_text(json.dumps({"servo_id": 1, "in_loader": True}), encoding="utf-8")
    code, out, err = run_cli("recover", "--id", "1", "--receipt", str(failed), "--loader-fresh", "--json")
    assert code == 0, err
    receipt = json.loads(out)
    assert not receipt["completed"] and "unverified" in receipt["preservation"]
    assert receipt["failed_run"].endswith(failed.name)
    assert "warning: installed, calibration preservation not verified" in err


def test_json_output_parses(run_cli):
    code, out, err = run_cli("scan", "--json", n=2)
    assert code == 0 and not err
    result = json.loads(out)
    assert [servo["id"] for servo in result] == [1, 2]
    assert all(servo["release"] == images.DEFAULT_RELEASE and servo["supported"] for servo in result)


def test_status_shows_faults_by_name(run_cli, monkeypatch):
    servo = app_servo()
    servo.faults = 1 | 16
    use_sim(monkeypatch, servo)
    code, out, err = run_cli("status", "--ids", "1")
    assert code == 0 and not err
    assert "sensor" in out and "temperature" in out
    assert images.DEFAULT_RELEASE in out


def test_no_stock_command_exists(run_cli):
    code, out, err = run_cli("--help")
    assert code == 0 and not err
    assert "install" in out and "recover" in out
    assert all(command not in out for command in ("stock", "restore", "set-id"))


@pytest.mark.parametrize("baud", [1_000_000, 3_000_000])
def test_status_flags_outdated(run_cli, monkeypatch, baud):
    servo = app_servo()
    read = servo.read
    monkeypatch.setattr(servo, "read", lambda a, n: (4, b"") if (a, n) in
                        (registers.RELEASE_ID, registers.LAST_RESET) else read(a, n))
    servo.baud = baud
    use_sim(monkeypatch, servo, baud=baud)
    code, out, err = run_cli("status", "--ids", "1", "--baud", str(baud))
    assert code == 0 and not err
    assert "release 10 or older" in out
    assert ("Switch the bus back to 1000000 baud" in out) == (baud != 1_000_000)
    assert "update available: openxl330 update --port SIM --id 1" in out


def test_status_offers_fast16_update_for_fast15(run_cli, monkeypatch):
    previous, current = app_servo(1), app_servo(2)
    previous.release_id = dict(previous.release_id, number=15, abi=3, source_id=0xB3D14882)
    use_sim(monkeypatch, previous, current)
    code, out, err = run_cli("status", "--ids", "1,2", n=2)
    assert code == 0 and not err
    assert "FW231 release 15" in out
    assert "fw231-comp2-fast16" in out
    assert "update available: openxl330 update --port SIM --id 1" in out
    assert out.count("update available:") == 1


def test_status_reports_discovery_only_for_unknown_abi(run_cli, monkeypatch):
    compatible, unknown = app_servo(1), app_servo(2)
    unknown.release_id = dict(unknown.release_id, abi=5)
    read = unknown.read
    attempted = []

    def observe(address, size):
        attempted.append((address, size))
        return read(address, size)

    monkeypatch.setattr(unknown, "read", observe)
    use_sim(monkeypatch, compatible, unknown)
    code, out, err = run_cli("status", "--ids", "1,2", "--json", n=2)
    assert code == 0 and not err
    first, second = json.loads(out)
    assert first["id"] == 1 and first["supported"] and first["enabled"] is False
    assert second["id"] == 2 and second["abi"] == 5 and not second["supported"]
    assert second["enabled"] is None and second["faults"] is None
    assert attempted == [registers.MARKER, registers.BUSDIAG]


def test_corrupt_calibration_does_not_block_off_and_scan(monkeypatch, tmp_path, capsys):
    device = app_servo()
    device.enabled = 1
    calibration = tmp_path / "calibration.json"
    calibration.write_text("{")
    # Exercise the named-port initialization path using only an in-memory port.
    monkeypatch.setattr("openxl330.bus._open_serial", lambda *_: SimPort([device]))
    monkeypatch.setattr("openxl330.adapters.ftdi_latency_warning", lambda _: None)
    assert main(["off", "--port", "SIM", "--ids", "1"]) == 0
    assert not device.enabled
    assert main(["scan", "--port", "SIM"]) == 0
    capsys.readouterr()
    assert main(["calibrate", "--port", "SIM", "--id", "1", "--reset"]) == 1
    assert "move the file aside" in capsys.readouterr().err
    assert calibration.read_text() == "{"


@pytest.mark.parametrize("connection", [[], ["--port", "SIM"]])
def test_calibration_reset_is_local(monkeypatch, tmp_path, capsys, connection):
    calibration = Calibration(tmp_path / "calibration.json")
    calibration.set(1, zero_counts=1200, direction=-1)
    calibration.set(2, zero_counts=1300, direction=-1)
    calibration.save()

    def unexpected_open(*args, **kwargs):
        pytest.fail("host calibration reset must not open a port")

    monkeypatch.setattr("openxl330.bus._open_serial", unexpected_open)
    assert main(["calibrate", "--id", "1", "--reset", *connection, "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["zero_counts"] == registers.CENTER_COUNTS
    restored = Calibration.load(calibration.path)
    assert restored.joint(1).zero_counts == registers.CENTER_COUNTS and restored.joint(1).direction == 1
    assert restored.joint(2).zero_counts == 1300 and restored.joint(2).direction == -1


def test_recover_ignores_corrupt_host_calibration(monkeypatch, tmp_path, capsys):
    device = SimServo(1, mode="loader")
    (tmp_path / "calibration.json").write_text("{")
    monkeypatch.setattr("openxl330.bus._open_serial", lambda *_: SimPort([device]))
    assert main(["recover", "--port", "SIM", "--id", "1", "--loader-fresh"]) == 0
    out, err = capsys.readouterr()
    assert "preservation not verified" in err and images.DEFAULT_RELEASE in out


def test_diag_json_includes_reply_timing(run_cli):
    code, out, err = run_cli("diag", "--ids", "1", "--json")
    assert code == 0 and not err
    assert json.loads(out)["1"]["reply_timing"]["direct_max_us"] == 180


@pytest.mark.parametrize("spec", ["3-1", "1,1", "1,,2", "1-2,3", "253"])
def test_invalid_id_spec_is_usage_error(run_cli, spec):
    code, out, err = run_cli("status", "--ids", spec)
    assert code == 2 and not out and "SPEC" in err


def test_group_options_and_simulator_controls(run_cli, monkeypatch, tmp_path):
    servos = [app_servo(1), app_servo(2)]
    calibration = Calibration(tmp_path / "calibration.json")
    use_sim(monkeypatch, *servos, calibration=calibration)
    assert run_cli("configure", "--ids", "1-2", "--kp", "2", "--kd", "0.1", n=2)[0] == 0
    assert all(servo.config and not servo.enabled for servo in servos)
    assert run_cli("identify", "--ids", "2,1", "--id", "2", n=2)[0] == 0
    assert [servo.led for servo in servos] == [0, 1]
    servos[0].enabled = 1
    assert run_cli("off", "--ids", "1", n=2)[0] == 0
    assert not servos[0].enabled
    servos[0].faults = 1
    assert run_cli("clear", "--ids", "1", n=2)[0] == 0 and not servos[0].faults
    assert run_cli("diag", "--ids", "1,2", n=2)[0] == 0
    assert run_cli("calibrate", "--id", "1", "--zero-here", "--direction", "-1", n=2)[0] == 0
    assert calibration.joint(1).direction == -1
    assert run_cli("calibrate", "--id", "1", "--reset", n=2)[0] == 0
    assert Calibration.load(calibration.path).joint(1).zero_counts == registers.CENTER_COUNTS
    assert run_cli("set-baud", "--to", "3000000", n=2)[0] == 0
    assert all(servo.baud == 3_000_000 for servo in servos)


def test_off_still_disables_answering_servos(run_cli, monkeypatch):
    servo = app_servo()
    servo.enabled = 1
    use_sim(monkeypatch, servo)
    code, out, err = run_cli("off", "--ids", "2,1")
    assert code == 1 and "ID 2 (not answering)" in err
    assert [line.split(None, 1) for line in out.splitlines()[1:]] == [["2", "not answering"], ["1", "OFF"]]
    assert not servo.enabled


def test_off_checks_reported_torque_after_ack(run_cli, monkeypatch):
    stuck, working = app_servo(1), app_servo(2)
    stuck.enabled = working.enabled = 1
    write = stuck.write

    def ignore_disable(address, data, broadcast=False):
        if address == registers.TORQUE_ENABLE and data == b"\x00":
            return 0
        return write(address, data, broadcast)

    monkeypatch.setattr(stuck, "write", ignore_disable)
    use_sim(monkeypatch, stuck, working)
    code, out, err = run_cli("off", "--ids", "1,2", "--json", n=2)
    assert code == 1
    assert json.loads(out)["result"] == {"1": "still reports torque enabled", "2": "OFF"}
    assert "ID 1 (still reports torque enabled)" in err
    assert stuck.enabled and not working.enabled


@pytest.mark.parametrize("kind", ["newer_abi", "no_marker", "stock"])
def test_off_needs_no_identity(run_cli, monkeypatch, kind):
    """OFF is a raw 64 <- 0 write per ID, with no identity probe first: it reaches a servo with a newer ABI, one
    without the openxl330 marker, and stock firmware alike."""
    if kind == "stock":
        servo = SimServo(1, eeprom=default_eeprom(1, baud_index=3))          # stock firmware at 1 Mbaud
    else:
        servo = app_servo()
        if kind == "newer_abi":
            servo.release_id = dict(servo.release_id, abi=9)
        else:
            read = servo.read
            monkeypatch.setattr(servo, "read", lambda a, n: (4, b"") if (a, n) == registers.MARKER else read(a, n))
    servo.enabled = 1
    use_sim(monkeypatch, servo)
    code, out, err = run_cli("off", "--ids", "1")
    assert code == 0, err
    assert out.splitlines()[1].split() == ["1", "OFF"] and not servo.enabled


def test_off_all_broadcasts_then_confirms_each_servo(run_cli, monkeypatch):
    servos = [app_servo(1), app_servo(3)]
    for servo in servos:
        servo.enabled = 1
    ports = []

    def simulated(cls, n):
        port = SimPort(servos)
        ports.append(port)
        return cls(port, calibration=Calibration(), check_adapter=False)

    monkeypatch.setattr(Bus, "simulated", classmethod(simulated))
    code, out, err = run_cli("off", "--all", "--json", n=2)
    assert code == 0, err
    assert json.loads(out) == {"ids": [1, 3], "result": {"1": "OFF", "3": "OFF"}, "broadcast": True}
    assert not any(servo.enabled for servo in servos)
    broadcast = [d for _, d in ports[0].tx_log if d[:5] == b"\xff\xff\xfd\x00\xfe" and d[7] == 3]
    assert broadcast and broadcast[0][8:11] == b"\x40\x00\x00"


def test_off_all_with_nothing_answering_is_an_error(run_cli, monkeypatch):
    monkeypatch.setattr(Bus, "simulated", classmethod(lambda cls, n: cls(SimPort([]), calibration=Calibration(),
                                                                          check_adapter=False)))
    code, out, err = run_cli("off", "--all")
    assert code == 1 and "broadcast" in err and "scan --port SIM --all-bauds" in err


def test_off_all_says_the_broadcast_went_out_when_the_confirming_scan_fails(run_cli, monkeypatch):
    twins = [app_servo(2), app_servo(2)]                    # one ID twice: their ping replies collide
    for servo in twins:
        servo.enabled = 1
    monkeypatch.setattr(Bus, "simulated", classmethod(lambda cls, n: cls(SimPort(twins), calibration=Calibration(),
                                                                          check_adapter=False)))
    code, out, err = run_cli("off", "--all")
    assert code == 1 and "OFF was broadcast" in err and "no servo is confirmed OFF" in err
    assert not any(servo.enabled for servo in twins)


def test_off_needs_ids_or_all(run_cli):
    assert run_cli("off")[0] == 2
    assert run_cli("off", "--all", "--ids", "1")[0] == 2


def test_status_shows_the_servos_that_answered_and_the_missing_ones(run_cli):
    code, out, err = run_cli("status", "--ids", "1-4", n=2)
    assert code == 1
    assert [line.split()[0] for line in out.splitlines()[1:]] == ["1", "2"]
    assert err.strip() == "missing: [3, 4]"
    code, out, err = run_cli("status", "--ids", "2,7", "--json", n=2)
    assert code == 1 and [row["id"] for row in json.loads(out)] == [2] and "missing: [7]" in err


def test_set_baud_reminds_how_to_reach_the_servos(run_cli):
    code, out, err = run_cli("set-baud", "--to", "3000000", n=2)
    assert code == 0
    assert "pass --baud 3000000 on later commands" in err and "after a power cycle" in err
    code, out, err = run_cli("set-baud", "--to", "1000000", n=2)
    assert code == 0 and "--baud" not in err


def test_cli_names_commands_instead_of_python_calls(run_cli, monkeypatch):
    servo = app_servo()
    servo.enabled = 1
    use_sim(monkeypatch, servo)
    code, out, err = run_cli("configure", "--ids", "1", "--kp", "2")
    assert code == 1 and "turn it off first with `openxl330 off --port SIM --ids 1`" in err and "disable()" not in err
    code, out, err = run_cli("clear", "--ids", "1")
    assert code == 1 and "`openxl330 off --port SIM --ids 1`" in err and "disable()" not in err
    assert cli._cli_hints("ID 2 is not configured since power-up: call configure() first (x)",
                          argparse.Namespace(port="/dev/ttyUSB0", sim=None, ids=[2])) == (
        "ID 2 is not configured since power-up: configure it first with `openxl330 configure --port /dev/ttyUSB0 "
        "--ids 2` (x)")


def test_cli_names_no_python_calls_after_a_bus_latch(run_cli, monkeypatch):
    """A garbled reply latches the bus; the library's "call reset() or reopen the port" becomes CLI advice."""
    servo = app_servo()
    real = servo.on_frame

    def garbled(frame, clock):
        reply = real(frame, clock)
        return None if reply is None else (reply[0], reply[1][:-1] + bytes([reply[1][-1] ^ 1]))
    monkeypatch.setattr(servo, "on_frame", garbled)
    use_sim(monkeypatch, servo)
    code, out, err = run_cli("diag", "--ids", "1")
    assert code == 1 and "run the command again" in err
    assert "reset()" not in err and "reopen the port" not in err
    for library, cli_text in (
            ("x: check the adapter and wiring, then call reset() or reopen the port", "x: check the adapter and wiring, "
             "then run the command again"),
            ("then reset() again", "then run the command again"),
            ("give each its own ID, then reopen the port", "give each its own ID, then run the command again"),
            ("reconnect the adapter, then open the port again", "reconnect the adapter, then run the command again")):
        assert cli._cli_hints(library, argparse.Namespace(port="P", sim=None)) == cli_text


def test_adapter_warning_is_one_plain_line(monkeypatch, capsys):
    device = app_servo()
    monkeypatch.setattr("openxl330.bus._open_serial", lambda *_: SimPort([device]))
    monkeypatch.setattr("openxl330.adapters.ftdi_latency_warning", lambda _: "the FTDI adapter is slow")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")                    # the CLI shows AdapterWarning itself, whatever the filters
        assert main(["scan", "--port", "/dev/ttyUSB9"]) == 0
    err = capsys.readouterr().err
    assert err.splitlines() == ["warning: the FTDI adapter is slow"]
    assert issubclass(AdapterWarning, UserWarning)


def test_recover_attach_failure_advice_on_the_cli(run_cli):
    """The simulated servos run the firmware, so nothing answers the loader: the advice offers --loader-fresh (if
    power-cycled) or a scan at every baud, never the same command again."""
    code, out, err = run_cli("recover", "--id", "1")
    assert code == 1 and not out
    assert "run `openxl330 recover --port SIM --id 1 --loader-fresh`" in err
    assert "Otherwise run `openxl330 scan --port SIM --all-bauds` to find it." in err
    assert "openxl330 recover --port SIM --id 1`" not in err and "To recover the servo, run:" not in err


def test_set_baud_without_any_confirmation_is_an_error(run_cli):
    code, out, err = run_cli("set-baud", "--baud", "2000000", "--to", "2000000", "--json")
    assert code == 1 and not out and "no servo confirmed" in err


def test_ports_with_sim_lists_a_virtual_port(run_cli):
    code, out, err = run_cli("ports", "--json")
    assert code == 0 and not err
    assert json.loads(out) == [SimPort.port]


def test_module_entry_point(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["openxl330", "version", "--sim", "1", "--json"])
    with pytest.raises(SystemExit) as stopped:
        runpy.run_module("openxl330", run_name="__main__")
    assert stopped.value.code == 0
    assert json.loads(capsys.readouterr().out)["default_release"] == images.DEFAULT_RELEASE
