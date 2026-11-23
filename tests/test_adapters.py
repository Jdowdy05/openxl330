"""USB adapter checks: an adapter that echoes our own request bytes, and FTDI latency-timer advice. No port is opened."""
from __future__ import annotations

import re
import warnings
from types import SimpleNamespace

import pytest
import serial

from openxl330 import Bus, adapters
from openxl330._protocol import framing
from openxl330.errors import AdapterEchoes, AdapterWarning, BusUncertain
from openxl330.sim import SimPort
from openxl330.wire import Wire


@pytest.fixture(autouse=True)
def _home(tmp_path, monkeypatch):
    monkeypatch.setenv("OPENXL330_HOME", str(tmp_path / "home"))


def echo_port(extra=b""):
    """A port that hands every request back (and then `extra`), as a full-duplex or echoing RS-485 adapter does."""
    port = SimPort([])

    def write(data):
        port.tx_log.append((port.baudrate, bytes(data)))
        port.rx += bytes(data) + extra
        return len(data)
    port.write = write
    return port


def wire_on(port):
    return Wire(port, clock=port.clock, sleep=port.clock.sleep)


def test_echo_detected():
    port = echo_port()
    wire = wire_on(port)
    with pytest.raises(AdapterEchoes) as e:
        adapters.check_echo(wire)
    assert "echoes" in str(e.value) and "half-duplex" in str(e.value)
    assert port.tx_log == [(1_000_000, framing.frame(0, 253, 1))], "one ping to ID 253"
    assert wire.uncertain, "nothing more is sent on an echoing adapter"
    with pytest.raises(AdapterEchoes):
        Bus(echo_port())


def test_no_echo_passes_and_a_stray_reply_latches():
    port = SimPort([])
    wire = wire_on(port)
    assert adapters.check_echo(wire) is None and not wire.uncertain
    stray = SimPort([])
    stray.write = lambda data: stray.rx.extend(b"\xff\xff\xfd\x00\x05") or len(data)
    wire = wire_on(stray)
    with pytest.raises(BusUncertain):
        adapters.check_echo(wire)


def test_linux_latency_warning(tmp_path, monkeypatch):
    monkeypatch.setattr(adapters, "_platform", lambda: "linux")
    monkeypatch.setattr(adapters, "SYSFS_USB_SERIAL", tmp_path)
    timer = tmp_path / "ttyUSB0" / "latency_timer"
    timer.parent.mkdir()
    timer.write_text("16\n")
    text = adapters.ftdi_latency_warning("/dev/ttyUSB0")
    assert f"echo 1 | sudo tee {timer}" in text and "16 ms" in text
    timer.write_text("1\n")
    assert adapters.ftdi_latency_warning("/dev/ttyUSB0") is None
    assert adapters.ftdi_latency_warning("/dev/ttyACM0") is None             # not a usb-serial (FTDI) device
    timer.write_text("garbage")
    assert adapters.ftdi_latency_warning("/dev/ttyUSB0") is None


def test_macos_ftdi_note(monkeypatch):
    monkeypatch.setattr(adapters, "_platform", lambda: "darwin")
    ports = [SimpleNamespace(device="/dev/cu.usbserial-FT1", vid=0x0403, pid=0x6014),
             SimpleNamespace(device="/dev/cu.usbserial-FT2", vid=0x0403, pid=0x6001),
             SimpleNamespace(device="/dev/cu.usbmodem1", vid=0x2E8A, pid=0x000A)]
    monkeypatch.setattr(adapters, "_comports", lambda: ports)
    for name in ("/dev/cu.usbserial-FT1", "/dev/cu.usbserial-FT2"):
        assert "15 ms per exchange" in adapters.ftdi_latency_warning(name)
    assert adapters.ftdi_latency_warning("/dev/cu.usbmodem1") is None
    monkeypatch.setattr(adapters, "_platform", lambda: "win32")
    assert adapters.ftdi_latency_warning("/dev/cu.usbserial-FT1") is None


def test_bus_warns_about_a_slow_ftdi_adapter(tmp_path, monkeypatch):
    monkeypatch.setattr(adapters, "_platform", lambda: "linux")
    monkeypatch.setattr(adapters, "SYSFS_USB_SERIAL", tmp_path)
    (tmp_path / "ttyUSB0").mkdir()
    (tmp_path / "ttyUSB0" / "latency_timer").write_text("16\n")
    monkeypatch.setattr(serial, "Serial", lambda name, baudrate, **kw: SimPort([]))
    with pytest.warns(AdapterWarning, match=re.escape("echo 1 | sudo tee")):
        Bus("/dev/ttyUSB0")
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        Bus("/dev/ttyUSB0", check_adapter=False)                            # no checks, no warning
