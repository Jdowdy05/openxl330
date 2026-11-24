"""Upper-voltage protection on fast17 rev b (owner 2026-11-24): an 8.0 V cap and default for the 848 upper limit, a
non-latching warning above 7.0 V (status word 3 bit 0; clears at 6.8 V), and register 1152 that turns both the warning
and the upper trip off (status word 3 bit 1 while off), back on at every boot."""
from __future__ import annotations

import struct

import pytest

from openxl330 import Bus, Calibration, images, registers
from openxl330.errors import InvalidArgument, NotSupported, Rejected
from openxl330.sim import SimPort, SimServo

RELEASE = images.Release(images.DEFAULT_RELEASE)
FAST17 = dict(number=17, variant='comp', abi=5, source_id=0x80d4d494)


def servo_and_bus(release_id=FAST17, supply_mv=5000):
    s = SimServo(1, mode='app', program=RELEASE.packaged().raw, release_id=release_id)
    s.supply_mv = supply_mv
    bus = Bus(SimPort([s]), calibration=Calibration(), check_adapter=False)
    return s, bus


def test_status_word_3_decodes_the_warnings():
    words = [0] * 17
    words[3] = 1 | 2
    d = registers.decode_status(struct.pack('<17I', *words))
    assert d['warnings'] == 3 and registers.warning_names(3) == ['supply_high', 'voltage_protection_off']
    assert registers.warning_names(0) == [] and registers.warning_names(8) == ['bit 3']


def test_the_upper_limit_defaults_to_8_v_and_caps_there():
    s, bus = servo_and_bus()
    assert struct.unpack('<4I', bus.read(1, 848, 16))[2] == 80
    bus.write(1, 848, struct.pack('<3I', 1000, 65, 80))
    with pytest.raises(Rejected) as e:
        bus.write(1, 848, struct.pack('<3I', 1000, 65, 81))
    assert e.value.error == 4


def test_a_7_4_v_supply_warns_without_a_fault_and_the_warning_has_hysteresis():
    s, bus = servo_and_bus(supply_mv=7400)
    servo = bus.servo(1)
    st = servo.status()
    assert st.warnings == ['supply_high'] and st.faults == []
    servo.configure()
    servo.enable()
    assert servo.status().enabled
    s.supply_mv = 6900
    assert servo.status().warnings == ['supply_high']           # clears only at or below 6.8 V
    s.supply_mv = 6800
    assert servo.status().warnings == []


def test_turning_voltage_protection_off_removes_the_warning_and_the_upper_trip():
    s, bus = servo_and_bus(supply_mv=8500)
    servo = bus.servo(1)
    assert servo.voltage_protection is True and servo.status().faults == ['voltage']
    servo.voltage_protection = False
    assert servo.voltage_protection is False and s.vprot == 0
    s.faults = 0
    st = servo.status()
    assert st.faults == [] and st.warnings == ['voltage_protection_off']
    servo.configure()
    servo.enable()
    assert servo.status().enabled
    servo.voltage_protection = True                             # allowed while armed: the guard comes back
    assert 'voltage' in servo.status().faults


def test_the_lower_limit_stays_with_protection_off():
    s, bus = servo_and_bus(supply_mv=3400)
    servo = bus.servo(1)
    servo.voltage_protection = False
    assert 'voltage' in servo.status().faults


def test_voltage_protection_is_back_on_after_a_power_cycle():
    s, bus = servo_and_bus()
    bus.servo(1).voltage_protection = False
    s.power_cycle()
    assert bus.servo(1).voltage_protection is True


@pytest.mark.parametrize('bad', [1, 0, None, 'off'])
def test_voltage_protection_takes_a_bool(bad):
    _, bus = servo_and_bus()
    with pytest.raises(InvalidArgument):
        bus.servo(1).voltage_protection = bad


def test_group_sets_voltage_protection_for_every_member():
    sims = [SimServo(i, mode='app', program=RELEASE.packaged().raw, seed=i, release_id=FAST17) for i in (1, 2)]
    bus = Bus(SimPort(sims), calibration=Calibration(), check_adapter=False)
    bus.group([1, 2]).set_voltage_protection(False)
    assert [s.vprot for s in sims] == [0, 0]


def test_older_releases_have_no_voltage_protection_switch():
    from conftest import make_fw231_program
    s = SimServo(1, mode='app', program=make_fw231_program())    # an older image: 7.0 V cap, no 1152
    bus = Bus(SimPort([s]), calibration=Calibration(), check_adapter=False)
    with pytest.raises(NotSupported) as e:
        bus.servo(1).voltage_protection = False
    assert 'openxl330 update' in str(e.value)


def test_cli_status_shows_the_warning(capsys, monkeypatch):
    from openxl330.cli import main
    from test_cli import use_sim
    s = SimServo(1, mode='app', program=RELEASE.packaged().raw, release_id=FAST17)
    s.supply_mv = 7400
    use_sim(monkeypatch, s)
    assert main(['--sim', '1', 'status', '--ids', '1']) == 0
    assert 'supply above 7.0 V' in capsys.readouterr().out
