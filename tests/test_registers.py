"""Register layer: layouts, decoders, encoders, device block."""
from __future__ import annotations

import struct

import pytest

from openxl330 import registers as r
from openxl330.errors import InvalidArgument


def test_decode_status_signed_fields():
    words = [0] * 17
    words[8] = (-5) & 0xFFFFFFFF
    words[9] = (-1200) & 0xFFFFFFFF
    words[15] = 0x00010000 | 7400
    d = r.decode_status(struct.pack("<17I", *words))
    assert d["position_counts"] == -5
    assert d["velocity_mrad_s"] == -1200
    assert d["voltage_mV"] == 7400


def test_decode_motion_scales():
    raw = struct.pack("<iiiihh", 2085, 65536, 1 << 24, -(1 << 23), -120, 240)
    d = r.decode_motion(raw)
    assert d["position_counts"] == 2085
    assert d["velocity_rad_s"] == 1.0
    assert d["tau_motor_nm"] == 1.0
    assert d["tau_out_nm"] == -0.5
    assert d["current_ma"] == -120
    assert d["duty"] == 0.5


def test_decode_reset_names():
    d = r.decode_reset(struct.pack("<3I", 0x02000301, 4, 1))
    assert d["reason"] == "stall"
    assert d["hang"] is True
    assert d["detail"] == 3
    assert d["app_reboots_armed"] is True and d["iwdg_update_timeout"] is False
    assert d["hang_reboots"] == 4 and d["quick_hangs"] == 1
    assert r.decode_reset(struct.pack("<3I", 0x06000008, 0, 0))["iwdg_update_timeout"] is True   # fast15 bit 26


def test_decode_busdiag():
    v = [0] * 16
    v[0] = (2 << 24) | (5 << 18) | (1 << 16) | (3 << 8) | 7
    v[1] = 3000000 | 1
    d = r.decode_busdiag(struct.pack("<16I", *v))
    assert d["id"] == 7 and d["nv_id"] == 3 and d["abi"] == 2
    assert d["devcfg_state"] == "override" and d["boot_baud"] == 3000000
    assert d["baud"] == 3000000 - 0 and d["baud_unconfirmed"] is True


def test_pd_config_bounds():
    for kp in (-1, 51):
        with pytest.raises(ValueError):
            r.pd_config(kp, 0.1, 1)
    with pytest.raises(ValueError):
        r.pd_config(1, 0.1, 0)
    blob = r.pd_config(r.KP_COMP, r.KD_COMP, 5)
    assert len(blob) == 48
    assert struct.unpack_from("<I", blob, 0)[0] == 5
    assert struct.unpack_from("<I", blob, 44)[0] == r.CONFIG_MAGIC


def test_encode_target_layout():
    assert r.encode_target(-100, 2000, 1) == struct.pack("<iIII", -100, 2000, 1, 0)
    assert len(r.encode_target(2048, 1, 0)) == 16


def test_encode_gains_and_lease():
    assert r.encode_gains(3.5, 0.1) == struct.pack("<ii", 229376, 6554)
    assert r.encode_lease(500, "hold") == struct.pack("<III", 500, 1, r.LEASE_MAGIC)
    with pytest.raises(ValueError):
        r.encode_lease(60001, "off")
    with pytest.raises(ValueError):
        r.encode_lease(10, "bogus")


def test_decode_lease():
    d = r.decode_lease(struct.pack("<4I", 500, 1, (3 << 8) | 2, 42))
    assert d == dict(ms=500, action="hold", state="expired", expiries=3, since_renewal_ms=42)


@pytest.mark.parametrize("action,state", [(2, 1), (0, 3)])
def test_decode_lease_rejects_an_unknown_action_or_state(action, state):
    with pytest.raises(InvalidArgument, match="lease"):
        r.decode_lease(struct.pack("<4I", 500, action, state, 0))


def test_fault_names_cover_bits():
    assert sorted(r.FAULT_NAMES) == [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024]
    assert r.FAULT_NAMES[32] == "timing" and r.FAULT_NAMES[1] == "sensor"


def test_motion_model_off_and_sign():
    assert r.motion_model(100, 5.0, 1.0, enabled=False) == (0.0, 0.0)
    tm, to = r.motion_model(240, 5.0, 0.0)
    assert tm == pytest.approx(r.MOTION_K * 5.0 * 0.5)
    assert to == pytest.approx(tm)   # v = 0: no loss term, no planned acceleration


def test_constants():
    assert (r.COUNTS_PER_REV, r.CENTER_COUNTS, r.TICK_HZ, r.DUTY_FULL) == (4096, 2048, 2000, 480)
    assert r.STATUS == (64, 68) and r.LAST_RESET == (1112, 12) and r.MOTION == (1088, 20)


def test_release_id_and_loader_digest_layout():
    """Read 1120 (fast12+): word 0 = release number | variant << 16 | ABI << 24, word 1 = source ID, words 2, 3 = 0."""
    assert (r.RELEASE_ID, r.LOADER_DIGEST, r.RELEASE_VARIANTS) == ((1120, 16), (1136, 8), ("pd", "comp", "demo", "lab"))
    raw = struct.pack("<4I", 12 | (1 << 16) | (3 << 24), 0x2FAD3AEA, 0, 0)
    assert r.decode_release_id(raw) == dict(number=12, variant="comp", abi=3, source_id=0x2FAD3AEA)
    assert r.encode_release_id(12, "comp", 3, 0x2FAD3AEA) == raw
    assert r.decode_release_id(struct.pack("<4I", 7 | (9 << 16) | (2 << 24), 1, 0, 0))["variant"] == "code 9"
    with pytest.raises(r.InvalidArgument):
        r.encode_release_id(12, "stock", 3, 0)


def test_devcfg_round_trip(devcfg):
    """parse_devcfg reads the blocks earlier tools wrote (built here by the tests' own helper)."""
    block = devcfg(7, "wrist", 3_000_000)
    assert len(block) == r.DEVCFG_BYTES
    assert r.parse_devcfg(block) == dict(state="override", id=7, label="wrist", boot_baud=3000000)
    assert r.parse_devcfg(devcfg()) == dict(state="default", id=None, label="", boot_baud=1000000)
    assert r.parse_devcfg(b"\0" * 32)["state"] == "absent"
    bad = bytearray(block)
    bad[9] ^= 1
    assert r.parse_devcfg(bytes(bad))["state"] == "invalid"


def test_no_device_block_writer():
    """v1 installs one identical image and never writes a device block: no encoder for one is shipped."""
    for gone in ("build_devcfg", "BAUD_CODES", "LABEL_BYTES"):
        assert not hasattr(r, gone), gone


def test_bad_values_raise_invalid_argument():
    from openxl330.errors import Error, InvalidArgument
    for call in (lambda: r.encode_lease(70000, "off"),
                 lambda: r.encode_lease(100, "brake"), lambda: r.pd_config(-1, 0.1, 1), lambda: r.pd_config(1, 0.1, 0),
                 lambda: r.encode_gains(1e6, 0), lambda: r.encode_gains(float("nan"), 0),
                 lambda: r.encode_target(2 ** 31, 1, 0), lambda: r.encode_target(0, -1, 0)):
        with pytest.raises(InvalidArgument) as e:
            call()
        assert isinstance(e.value, Error) and isinstance(e.value, ValueError)
