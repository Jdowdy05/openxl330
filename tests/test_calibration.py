"""Calibration: per-servo zero and direction, counts <-> radians."""
from __future__ import annotations

import json
import math

import pytest

from openxl330 import paths
from openxl330.calibration import Calibration, Joint
from openxl330.errors import CalibrationError, InvalidArgument


def test_default_zero_is_center():
    cal = Calibration()
    assert cal.to_rad(1, 2048) == 0.0
    assert cal.to_rad(1, 3072) == pytest.approx(math.pi / 2)


def test_direction_flips():
    cal = Calibration()
    cal.set(3, zero_counts=2100, direction=-1)
    assert cal.sign(3) == -1
    assert cal.to_rad(3, 2100 - 1024) == pytest.approx(math.pi / 2)
    assert cal.to_counts(3, math.pi / 2) == 2100 - 1024
    assert cal.sign(4) == 1


def test_round_trip_multi_turn():
    cal = Calibration()
    cal.set(2, zero_counts=100, direction=-1)
    for sid in (1, 2):
        counts = cal.to_counts(sid, 10.0)
        assert abs(cal.to_rad(sid, counts) - 10.0) <= math.pi / 4096 + 1e-12
        assert cal.to_counts(sid, cal.to_rad(sid, 123456)) == 123456


@pytest.mark.parametrize("radians", [float("nan"), float("inf"), 1e308, "1.0", True])
def test_to_counts_requires_representable_radians(radians):
    with pytest.raises(InvalidArgument):
        Calibration().to_counts(1, radians)


def test_joint_snapshot_can_be_modified_without_affecting_calibration():
    cal = Calibration()
    cal.joint(1).direction = -1
    assert cal.sign(1) == 1
    cal.set(1, direction=-1)
    snapshot = cal.joint(1)
    snapshot.direction = 1
    assert cal.sign(1) == -1


def test_missing_file_gives_defaults(tmp_path):
    cal = Calibration.load(tmp_path / "nope.json")
    assert cal.joint(5) == Joint(2048, 1)


def test_default_path_uses_home(tmp_path, monkeypatch):
    monkeypatch.setenv("OPENXL330_HOME", str(tmp_path / "h"))
    cal = Calibration.load()
    cal.set(1, zero_counts=10)
    cal.save()
    assert (paths.home() / "calibration.json").is_file()
    assert Calibration.load().joint(1).zero_counts == 10


def test_save_load_roundtrip_and_format(tmp_path):
    p = tmp_path / "sub" / "c.json"
    cal = Calibration.load(str(p))
    cal.set(3, zero_counts=2100, direction=-1)
    cal.save()
    data = json.loads(p.read_text())
    assert data["version"] == 1
    assert data["joints"]["3"] == {"zero_counts": 2100, "direction": -1}
    assert Calibration.load(p).joint(3) == Joint(2100, -1)
    assert not list(p.parent.glob("*.tmp"))


def test_set_partial_keeps_other_field():
    cal = Calibration()
    cal.set(1, zero_counts=5)
    cal.set(1, direction=-1)
    assert cal.joint(1) == Joint(5, -1)


@pytest.mark.parametrize("text", ["{not json", "[]", '{"version": 2, "joints": {}}',
                                  '{"version": 1, "joints": []}',
                                  '{"version": 1, "joints": {"1": {"zero_counts": "x", "direction": 1}}}',
                                  '{"version": 1, "joints": {"1": {"zero_counts": 1, "direction": 2}}}',
                                  '{"version": 1, "joints": {"abc": {"zero_counts": 1, "direction": 1}}}'])
def test_corrupt_file_raises_calibration_error(tmp_path, text):
    p = tmp_path / "c.json"
    p.write_text(text)
    with pytest.raises(CalibrationError) as e:
        Calibration.load(p)
    assert str(p) in str(e.value)
    assert "move the file aside" in str(e.value)


def test_extra_ids_ignored(tmp_path):
    p = tmp_path / "c.json"
    p.write_text(json.dumps({"version": 1, "joints": {"9": {"zero_counts": 7, "direction": -1}}}))
    cal = Calibration.load(p)
    assert cal.joint(1) == Joint(2048, 1)
    assert cal.joint(9) == Joint(7, -1)


@pytest.mark.parametrize("d", [0, 2, -2, True, 1.0, "1"])
def test_direction_must_be_plus_minus_one(d):
    with pytest.raises(InvalidArgument):
        Calibration().set(1, direction=d)
    with pytest.raises(InvalidArgument):
        Joint(2048, d)


@pytest.mark.parametrize("z", [1.5, True, "5", None])
def test_zero_must_be_int(z):
    with pytest.raises(InvalidArgument):
        Joint(z, 1)


@pytest.mark.parametrize("sid", [-1, 253, 1.0, True, "1"])
def test_servo_id_validated(sid):
    with pytest.raises(InvalidArgument):
        Calibration().joint(sid)


def test_reset(tmp_path):
    cal = Calibration()
    cal.set(1, zero_counts=5)
    cal.set(2, direction=-1)
    cal.reset(1)
    assert cal.joint(1) == Joint()
    assert cal.joint(2).direction == -1
    cal.reset()
    assert cal.joint(2) == Joint()
