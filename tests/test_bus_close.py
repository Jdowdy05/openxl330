"""Adapter close failures must preserve install and connection diagnostics."""
from __future__ import annotations

import pytest

from openxl330 import AdapterEchoes, AdapterError, Bus, Calibration, InstallError
from openxl330 import bus as bus_module
from openxl330.sim import SimPort


class CloseFails(SimPort):
    def close(self):
        raise OSError("simulated close failure")


def test_context_exit_preserves_install_recovery():
    install_error = InstallError("upload failed", step="upload", recovery="openxl330 recover --port SIM --id 1")
    bus = Bus(CloseFails([]), calibration=Calibration(), check_adapter=False)

    with pytest.raises(InstallError) as caught, bus:
        raise install_error

    assert caught.value is install_error
    assert caught.value.recovery == "openxl330 recover --port SIM --id 1"
    assert isinstance(caught.value.close_error, AdapterError)


def test_open_probe_failure_survives_failed_adapter_close(monkeypatch):
    adapter = CloseFails([])
    echo_error = AdapterEchoes("adapter echoed ping")

    def check_echo(wire):
        assert wire.port is adapter
        raise echo_error

    monkeypatch.setattr(bus_module, "_open_serial", lambda *args: adapter)
    monkeypatch.setattr(bus_module.adapters, "check_echo", check_echo)
    with pytest.raises(AdapterEchoes) as caught:
        Bus("SIM", calibration=Calibration())

    assert caught.value is echo_error
    assert isinstance(caught.value.close_error, AdapterError)


def test_standalone_close_failure_has_adapter_error():
    bus = Bus(CloseFails([]), calibration=Calibration(), check_adapter=False)
    with pytest.raises(AdapterError, match="simulated close failure"):
        bus.close()
