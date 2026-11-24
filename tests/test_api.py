"""The friendly API (Bus, Servo, Group) against the simulated bus. No port is opened: a string port only ever reaches
a faked pyserial.

The test_fleet_* tests cover multi-servo behaviour: baud switch and status, a servo that misses the switch,
targets/gains/identify/off, gains picked from the image (an armed servo is refused before anything is written: Refused,
"call disable() first"), one group read (sync and fast), group order and rejected targets, the command lease, last reset
after power-on and a host reboot, the configuration tick race, configure on an armed servo, the full ping window, and
the stale input drained at open.
"""
from __future__ import annotations

import math
import struct
import time

import pytest
import serial

import openxl330
from openxl330 import (Bus, Calibration, Group, Servo, ServoInfo, State, Status, images, registers)
from openxl330.errors import (AdapterError, BusUncertain, CalibrationError, Faulted, InvalidArgument, NoReply, NoServosFound,
                              NotEnabled, NotSupported, PortBusy, Refused, Rejected, UnsupportedServo)
from openxl330.sim import SimPort, SimServo, default_eeprom, parse_requests
from openxl330.bus import MAX_ABI
from openxl330.wire import Wire

RELEASE = images.Release(images.DEFAULT_RELEASE)
ALL_FEATURES = frozenset({"motion", "fast_sync_read", "last_reset", "release_id", "torque_command", "set_id"})


@pytest.fixture(autouse=True)
def _home(tmp_path, monkeypatch):
    """Keep the user's real $OPENXL330_HOME (calibration.json) out of every test."""
    monkeypatch.setenv("OPENXL330_HOME", str(tmp_path / "home"))


# ---------------------------------------------------------------------------------------------- helpers
def release_servos(n, first=1):
    """Servos running the bundled release (read 1120, 1112, comp loaded at power-up) at IDs first..first+n-1."""
    return [SimServo(i, mode="app", program=RELEASE.packaged().raw, seed=i) for i in range(first, first + n)]


def stock_servo(servo_id, version=52, baud_index=3):
    eeprom = default_eeprom(servo_id, baud_index=baud_index)
    eeprom[6] = version
    return SimServo(servo_id, mode="stock", eeprom=eeprom)


def bus_on(servos, **kw):
    """A Bus on a SimPort of these servos: the port's FakeClock drives the Wire (bus.wire.sleep advances it)."""
    return Bus(SimPort(servos), **kw)


def ready(bus, ids):
    """configure() (lost at every power-up) then enable(): the group holds where it is."""
    g = bus.group(ids)
    g.configure()
    g.enable()
    return g


def rad(counts):
    return (counts - registers.CENTER_COUNTS) * 2 * math.pi / registers.COUNTS_PER_REV


def target_of(servo):
    """(counts, duration ticks, profile) the simulated servo holds as its last target."""
    return struct.unpack("<iIII", servo.target)[:3]


def flaky(servo, address, times, error=7, data=None):
    """The servo answers `times` writes to `address` (with `data`, if given) with `error`; returns [left]."""
    real, left = servo.write, [times]

    def write(a, d, broadcast=False):
        if a == address and (data is None or bytes(d) == data) and left[0] > 0:
            left[0] -= 1
            return error
        return real(a, d, broadcast)
    servo.write = write
    return left


def without(servo, *reads):
    """The servo answers these reads ((address, size)) with error 4, like an older image."""
    real = servo.read
    servo.read = lambda a, n: (4, b"") if (a, n) in reads else real(a, n)
    return servo


def with_abi(servo, abi):
    """The servo reports this ABI in read 1000."""
    real = servo.read

    def read(a, n):
        err, data = real(a, n)
        if (a, n) == registers.BUSDIAG and not err:
            data = bytes(data[:3]) + bytes([abi]) + bytes(data[4:])
        return err, data
    servo.read = read
    return servo


def writes_to(port, address):
    """Host writes (0x03) and Sync Write frames (0x83) to `address` in the SimPort's TX log."""
    out = []
    for _, data in port.tx_log:
        for kind, f in parse_requests(data):
            if kind == "frame" and f["instruction"] in (3, 0x83) and struct.unpack_from("<H", f["params"])[0] == address:
                out.append(f)
    return out


# ---------------------------------------------------------------------------------------------- package surface
def test_package_exports():
    assert openxl330.__version__ == "0.2.0"
    for name in ("Bus", "Servo", "Group", "State", "Status", "ServoInfo", "Calibration", "Error", "InvalidArgument",
                 "Refused", "BusUncertain", "NoReply", "Rejected", "NotSupported", "NotEnabled", "Faulted",
                 "InstallError", "UntestedServo", "UnsupportedServo", "NoServosFound", "CalibrationError",
                 "AdapterError", "PortBusy", "AdapterEchoes", "AdapterWarning"):
        assert hasattr(openxl330, name), name
    assert set(openxl330.__all__) >= {"Bus", "Servo", "Group", "State", "Status", "ServoInfo", "Calibration"}


# ---------------------------------------------------------------------------------------------- scan and identity
def test_scan_lists_servo_info():
    bus = bus_on(release_servos(2) + [stock_servo(5)])
    infos = bus.scan()
    assert [i.id for i in infos] == [1, 2, 5]
    assert infos[0] == ServoInfo(id=1, model="XL330-M288", firmware="FW231", supported=True, features=ALL_FEATURES,
                                 release=images.DEFAULT_RELEASE, label="", baud=1_000_000,
                                 abi=RELEASE.release_id["abi"])
    stock = infos[2]
    assert (stock.firmware, stock.supported, stock.model, stock.features, stock.release) == \
        ("stock V52", False, "XL330-M288", frozenset(), None)
    assert not bus.wire.uncertain


def test_scan_empty_suggests_all_bauds():
    """Review Focus 1: a servo at another baud is never a silent empty list."""
    s = release_servos(1)[0]
    s.baud = 3_000_000
    bus = bus_on([s])
    with pytest.raises(NoServosFound) as e:
        bus.scan()
    msg = str(e.value)
    assert "no servos at 1 Mbaud" in msg and "scan(all_bauds=True)" in msg and "openxl330 scan --all-bauds" in msg
    infos = bus.scan(all_bauds=True)
    assert [(i.id, i.firmware, i.baud) for i in infos] == [(1, "FW231", 3_000_000)]
    assert bus.port.baudrate == 1_000_000, "the host is back at its baud"
    with pytest.raises(NoServosFound) as e:
        bus_on([]).scan(all_bauds=True)
    assert "no servos answered at any baud" in str(e.value)


def test_release_identification():
    """Read 1120 when it answers (a bundled release by name, else its number); else read 1112 (release 11); else
    release 10 or older. Each servo is probed once per Bus."""
    alternate = release_servos(1)[0]
    alternate.release_id = dict(alternate.release_id, source_id=0x12345678)
    eleven = without(SimServo(2, mode="app", program=RELEASE.packaged().raw, seed=2), registers.RELEASE_ID)
    ten = without(SimServo(3, mode="app", program=RELEASE.packaged().raw, seed=3), registers.RELEASE_ID,
                  registers.LAST_RESET)
    bus = bus_on(release_servos(1, first=4) + [alternate, eleven, ten])
    got = {i.id: (i.release, i.features) for i in bus.scan()}
    assert got[4] == (images.DEFAULT_RELEASE, ALL_FEATURES)
    assert got[1] == (f"FW231 release {RELEASE.release_id['number']}", ALL_FEATURES)
    assert got[2] == ("FW231 (release 11)", frozenset({"motion", "fast_sync_read", "last_reset"}))
    assert got[3] == ("FW231 (release 10 or older)", frozenset({"motion", "fast_sync_read"}))
    sent = len(bus.port.tx_log)
    assert bus.servo(2).info.release == "FW231 (release 11)"
    bus.group([1, 2, 3, 4])
    assert len(bus.port.tx_log) == sent, "probed once per Bus: servo()/group() reuse the scan"


def test_stock_servo_refused_with_advice():
    bus = bus_on(release_servos(1) + [stock_servo(5)])
    with pytest.raises(UnsupportedServo) as e:
        bus.servo(5)
    msg = str(e.value)
    assert msg.startswith("ID 5 runs stock firmware V52")
    assert "every servo runs openxl330 firmware" in msg and "separate adapter" in msg and "openxl330 install" in msg
    with pytest.raises(UnsupportedServo):
        bus.group([1, 5])
    assert bus.ping(5)["firmware"] == 52, "the low level stays generic Protocol 2.0"
    assert bus.servo(1).info.supported


def test_abi_outside_the_supported_range_is_refused_with_the_update_command():
    newer = MAX_ABI + 1
    bus = bus_on([with_abi(release_servos(1)[0], 1), with_abi(release_servos(1, first=2)[0], newer)])
    infos = {i.id: i for i in bus.scan()}
    assert not infos[1].supported and not infos[2].supported
    for servo_id, words in ((1, "ABI 1"), (2, f"ABI {newer}")):
        with pytest.raises(UnsupportedServo) as e:
            bus.servo(servo_id)
        assert words in str(e.value) and f"openxl330 update --port SIM --id {servo_id}" in str(e.value)
    with pytest.raises(UnsupportedServo) as e:
        bus.servo(2)
    assert "newer" in str(e.value)


def test_servo_and_group_ids_are_checked():
    bus = bus_on(release_servos(2))
    for bad in (253, -1, "1", 1.0, True):
        with pytest.raises(InvalidArgument):
            bus.servo(bad)
    with pytest.raises(InvalidArgument):
        bus.group([1, 1])
    with pytest.raises(InvalidArgument):
        bus.group([])
    with pytest.raises(NoReply) as e:
        bus.group([1, 2, 9])
    assert e.value.missing == [9] and "scan()" in str(e.value)


# ---------------------------------------------------------------------------------------------- reading
def test_group_read_si_units():
    servos = release_servos(3)
    bus = bus_on(servos)
    g = bus.group([1, 2, 3])
    assert isinstance(g, Group) and g.ids == [1, 2, 3]
    g.configure()
    g.enable()
    g.move({1: 0.5, 2: -0.25}, duration=0.2)
    bus.wire.sleep(0.4)
    st = g.read()
    assert list(st) == [1, 2, 3] and all(isinstance(v, State) for v in st.values())
    assert st[1].position == pytest.approx(0.5, abs=0.01) and st[2].position == pytest.approx(-0.25, abs=0.01)
    assert st[3].position == pytest.approx(rad(2085))
    g.move({1: 2.0})                                                          # a step: at most 6 rad/s
    bus.wire.sleep(0.05)
    one = g.read()[1]
    raw = one.raw
    assert one.velocity == pytest.approx(raw["velocity_rad_s"]) and 5.0 < one.velocity <= 6.0 + 1e-6
    assert one.position == pytest.approx(rad(raw["position_counts"]))
    assert one.current == pytest.approx(raw["current_ma"] / 1000) and one.current > 0
    assert one.duty == pytest.approx(raw["pwm"] / 480) and 0 < one.duty <= 1
    assert one.torque_motor == pytest.approx(raw["tau_motor_nm"]) and one.torque_out == pytest.approx(raw["tau_out_nm"])
    status = g.status()
    assert list(status) == [1, 2, 3] and all(isinstance(v, Status) for v in status.values())
    s1 = status[1]
    assert (s1.id, s1.enabled, s1.ready, s1.faults, s1.fault_bits) == (1, True, True, [], 0)
    assert (s1.voltage, s1.temperature, s1.worst_tick_us) == (5.0, 31.0, 210)
    assert s1.position == pytest.approx(rad(s1.raw["position_counts"]))
    assert len({one, status[1], status[2]}) == 3, "State and Status are hashable"
    bus.wire.sleep(0.5)                                                       # settled at 2.0 rad
    servo = bus.servo(1)
    assert isinstance(servo, Servo) and servo.position == pytest.approx(servo.state().position)
    assert servo.position == pytest.approx(2.0, abs=0.01)
    assert servo.status().enabled is True


def test_missing_servo_in_group_read_raises_no_reply_with_partial():
    servos = release_servos(3)
    bus = bus_on(servos)
    g = bus.group([1, 2, 3])
    servos[1].mode = "dead"                                                   # unplugged after the group was made
    with pytest.raises(NoReply) as e:
        g.read()                                                              # Fast Sync Read: a missing row ends it
    assert list(e.value.partial) == [1] and isinstance(e.value.partial[1], State)
    assert e.value.missing == [2, 3]
    with pytest.raises(NoReply) as e:
        g.read(fast=False)
    assert sorted(e.value.partial) == [1, 3] and e.value.missing == [2]
    assert all(isinstance(v, State) for v in e.value.partial.values())
    with pytest.raises(NoReply) as e:
        g.status()
    assert e.value.missing == [2] and isinstance(e.value.partial[3], Status)
    assert not bus.wire.uncertain


# ---------------------------------------------------------------------------------------------- moving
def test_move_step_and_quintic():
    servos = release_servos(2)
    bus = bus_on(servos)
    s = bus.servo(1)
    bus.group([1, 2]).configure()
    s.enable()
    s.move(math.pi / 2)                                                       # duration 0: a step
    assert target_of(servos[0]) == (3072, 1, 0)
    bus.wire.sleep(0.5)
    assert s.position == pytest.approx(math.pi / 2, abs=0.01)
    s.move(60, duration=0.4, units="deg")                                     # quintic over 0.4 s
    assert target_of(servos[0]) == (2048 + round(4096 / 6), 800, 0)
    bus.wire.sleep(0.2)
    assert math.degrees(s.position) == pytest.approx(75, abs=3)
    bus.wire.sleep(0.3)
    assert s.position == pytest.approx(math.radians(60), abs=0.01)
    s.move(0.0, duration=0.1, profile="linear")
    assert target_of(servos[0])[1:] == (200, 1)
    s.move(0.0, duration=0.0001)
    assert target_of(servos[0])[1] == 1, "ticks = max(1, round(duration * 2000))"
    g = bus.group([1, 2])
    g.enable()                                                                # ID 1 is already enabled: no change
    g.move({1: 0.3, 2: -0.3}, duration=0.3)
    bus.wire.sleep(0.5)
    assert [round(v.position, 2) for v in g.read().values()] == [0.3, -0.3]
    g.move({2: 10}, units="deg")                                              # a subset moves, the rest holds
    bus.wire.sleep(0.3)
    st = g.read()
    assert st[1].position == pytest.approx(0.3, abs=0.01) and st[2].position == pytest.approx(math.radians(10), abs=0.01)


def test_move_while_off_raises_not_enabled():
    """Review Focus 4: a move to a servo that is OFF raises NotEnabled with its faults instead of not moving."""
    servos = release_servos(2)
    bus = bus_on(servos)
    g = bus.group([1, 2])
    with pytest.raises(NotEnabled) as e:
        g.move({1: 0.1, 2: 0.2})
    msg = str(e.value)
    assert "ID 1" in msg and "ID 2" in msg and "enable()" in msg and e.value.servo_ids == [1, 2]
    g.configure()
    g.enable()
    assert [round(v.raw["position_counts"]) for v in g.read().values()] == [2085, 2085], \
        "enable() holds where they are: the targets the refused move stored are not executed"
    servos[1].faults = 4                                                      # a current trip turns ID 2 OFF
    bus.wire.sleep(0.01)
    with pytest.raises(NotEnabled) as e:
        g.move({1: 0.1, 2: 0.2})
    msg = str(e.value)
    assert "ID 2" in msg and "current" in msg and "clear_faults()" in msg and "ID 1 " not in msg
    assert (e.value.servo_ids, e.value.faults) == ([2], {2: ["current"]})
    with pytest.raises(NotEnabled) as e:
        bus.servo(2).move(0.2)
    assert "current" in str(e.value)
    g.move({1: 0.1, 2: 0.2}, verify=False)                                    # verify=False: no check, no raise


def test_move_verify_reports_untaken_target(fw231_program):
    """A Sync Write has no reply, so move() reads the targets back. An image older than release 12 refuses a new
    target while a move runs; release 12+ retargets it."""
    old = SimServo(1, mode="app", program=fw231_program(), seed=1)
    new = release_servos(1, first=2)[0]
    bus = bus_on([old, new])
    g = ready(bus, [1, 2])
    g.move({1: 1.0, 2: 1.0}, duration=1.0)
    bus.wire.sleep(0.1)
    with pytest.raises(Rejected) as e:
        g.move({1: 0.0, 2: 0.0})
    msg = str(e.value)
    assert e.value.servo_ids == [1] and "ID 1" in msg and "ID 2" not in msg
    assert "a move is still running (firmware older than release 12)" in msg
    assert target_of(new)[0] == 2048, "release 12+ took the new target"
    with pytest.raises(Rejected) as e:
        bus.servo(1).move(0.0)                                                # unicast: the servo's own answer
    assert "a move is still running (firmware older than release 12)" in str(e.value)
    flaky(new, 256, 1)                                                        # a target refused on a newer image
    bus.wire.sleep(1.0)
    with pytest.raises(Rejected) as e:
        g.move({1: -0.5, 2: -0.5})
    assert e.value.servo_ids == [2] and "lease" in str(e.value)


def test_set_gains_duty_led_and_clear_faults():
    servos = release_servos(2)
    cal = Calibration()
    cal.set(2, direction=-1)
    bus = bus_on(servos, calibration=cal)
    s1, s2 = bus.servo(1), bus.servo(2)
    with pytest.raises(Refused) as e:
        s1.set_gains(2.0, 0.05)                                               # live gains need the configuration
    assert "configure() first" in str(e.value)
    bus.group([1, 2]).configure()
    s1.set_gains(2.0, 0.05)
    assert servos[0].gains == (round(2.0 * 65536), round(0.05 * 65536))
    bus.group([1, 2]).set_gains({1: (3.5, 0.1), 2: (1.0, 0.0)})
    assert (servos[0].gains, servos[1].gains) == ((round(3.5 * 65536), round(0.1 * 65536)), (65536, 0))
    with pytest.raises(Refused) as e:
        s1.set_duty(0.5)                                                      # OFF: it would stay latched
    assert "enable() first" in str(e.value) and writes_to(bus.port, registers.DUTY) == []
    s1.enable(); s2.enable()
    s1.set_duty(0.5)
    s2.set_duty(0.5)                                                          # direction -1: the raw duty is negative
    bus.wire.sleep(0.1)
    st = bus.group([1, 2]).read()
    assert (st[1].raw["pwm"], st[1].duty) == (240, 0.5) and (st[2].raw["pwm"], st[2].duty) == (-240, 0.5)
    assert st[1].velocity > 0 and st[2].velocity > 0
    s1.release_duty()
    held = s1.position
    bus.wire.sleep(0.2)
    assert s1.position == pytest.approx(held, abs=0.003)
    s1.led = True
    assert servos[0].led == 1
    s1.led = False
    assert servos[0].led == 0
    with pytest.raises(AttributeError):
        s1.led
    servos[0].faults = 16                                                     # a temperature trip turns ID 1 OFF
    bus.wire.sleep(0.01)
    st = s1.status()
    assert (st.enabled, st.faults, st.fault_bits) == (False, ["temperature"], 16)
    s1.clear_faults()                                                         # OFF: allowed
    assert servos[0].faults == 0 and s1.status().faults == []
    with pytest.raises(Refused) as e:
        s2.clear_faults()                                                     # ID 2 is enabled
    assert "call disable() first" in str(e.value)
    assert len(writes_to(bus.port, registers.CLEAR_FAULTS)) == 1, "only ID 1's clear: ID 2 was refused before any write"


def test_invalid_arguments_raise_invalid_argument():
    servos = release_servos(2)
    bus = bus_on(servos)
    s, g = bus.servo(1), bus.group([1, 2])
    g.configure()
    s.enable()
    sent = len(bus.port.tx_log)
    nan, inf = float("nan"), float("inf")
    for call in (lambda: s.set_gains(nan, 0.1), lambda: s.set_gains(-1, 0.1), lambda: s.set_gains(51, 0.1),
                 lambda: s.set_gains(1.0, -0.1), lambda: s.set_gains(1.0, 2.5), lambda: s.set_gains("1", 0.1),
                 lambda: s.set_duty(1.5), lambda: s.set_duty(nan), lambda: s.set_duty(None),
                 lambda: s.move(nan), lambda: s.move(inf), lambda: s.move(1e30), lambda: s.move("0"),
                 lambda: s.move(1e308), lambda: s.move(-1e308, units="deg"), lambda: g.move({1: 1e308}),
                 lambda: s.move(0, duration=-1), lambda: s.move(0, duration=inf), lambda: s.move(0, duration=1e9),
                 lambda: s.move(0, duration=1e308), lambda: g.move({1: 0.0}, duration=1e308),
                 lambda: s.lease(1e308), lambda: bus.set_baud(3e6), lambda: bus.set_baud(True),
                 lambda: bus.set_baud("3000000"),
                 lambda: s.move(0, profile="cubic"), lambda: s.move(0, profile=[]), lambda: s.move(0, units="grad"),
                 lambda: g.move({9: 0.0}), lambda: g.move({True: 0.0}), lambda: g.move({1.0: 0.0}),
                 lambda: g.move([0.0, 0.0]), lambda: g.move({1: nan}),
                 lambda: g.set_gains({1: (nan, 0.0)}), lambda: g.set_gains({9: (1.0, 0.0)}),
                 lambda: g.set_gains({1: 1.0}), lambda: g.set_gains({1: {0.1: 9, 0.2: 10}}),
                 lambda: s.configure(kp=-1), lambda: g.configure(kd=3),
                 lambda: s.lease(-1), lambda: s.lease(61), lambda: s.lease(nan), lambda: s.lease(0.1, "stop"),
                 lambda: g.lease(0.1, "stop"), lambda: g.lease(0.1, []),
                 lambda: g.identify(9), lambda: g.identify(True), lambda: g.identify(1.0), lambda: setattr(s, "led", 2),
                 lambda: s.calibrate(zero=0.5), lambda: s.calibrate(direction=2), lambda: g.calibrate(zeros="there"),
                 lambda: g.calibrate(directions={1: 0})):
        with pytest.raises(InvalidArgument):
            call()
    assert len(bus.port.tx_log) == sent, "refused before anything was sent"


def test_linear_one_tick_move_uses_the_real_step_encoding():
    servos = release_servos(2)
    bus = bus_on(servos)
    group = ready(bus, [1, 2])
    bus.servo(1).move(0.5, profile="linear")
    group.move({2: 0.5}, duration=0.0006, profile="linear")
    assert target_of(servos[0])[1:] == target_of(servos[1])[1:] == (1, 0)


def test_moves_reject_durations_over_the_firmware_reference_limit():
    bus = bus_on(release_servos(1))
    joint = bus.servo(1)
    sent = len(bus.port.tx_log)
    with pytest.raises(InvalidArgument, match="duration"):
        joint.move(0.5, duration=3037000500 / registers.TICK_HZ, verify=False)
    assert len(bus.port.tx_log) == sent


# ---------------------------------------------------------------------------------------------- enable, configure
def test_configure_requires_off():
    servos = release_servos(2)
    bus = bus_on(servos)
    servos[0].enabled = 1
    with pytest.raises(Refused) as e:
        bus.servo(1).configure()
    assert "call disable() first" in str(e.value) and "ID 1" in str(e.value)
    with pytest.raises(Refused) as e:
        bus.group([1, 2]).configure(kp=2.0, kd=0.05)
    assert "call disable() first" in str(e.value)
    assert writes_to(bus.port, registers.PD_CONFIG) == [] and (servos[0].token, servos[1].token) == (0, 0)
    servos[0].enabled = 0
    assert bus.group([1, 2]).configure(kp=2.0, kd=0.05) == {1: dict(kp=2.0, kd=0.05, comp=True),
                                                           2: dict(kp=2.0, kd=0.05, comp=True)}


def test_enable_retries_access_error():
    servos = release_servos(2)
    bus = bus_on(servos)
    bus.group([1, 2]).configure()
    left = flaky(servos[0], 64, 3, data=b"\1")
    bus.servo(1).enable()
    assert left == [0] and servos[0].enabled == 1
    flaky(servos[1], 64, 25, data=b"\1")
    with pytest.raises(Rejected) as e:
        bus.servo(2).enable()
    assert e.value.error == 7 and len(writes_to(bus.port, 64)) == 4 + 20, "20 tries"
    del servos[1].write                                                       # back to the simulated firmware
    servos[1].faults = 4
    with pytest.raises(Faulted) as e:
        bus.group([1, 2]).enable()
    assert "ID 2" in str(e.value) and "current" in str(e.value) and "clear_faults()" in str(e.value)


# ---------------------------------------------------------------------------------------------- calibration
def test_calibration_applied_to_read_and_move(tmp_path):
    servos = release_servos(2)
    cal = Calibration()
    cal.set(1, zero_counts=3072, direction=-1)
    bus = bus_on(servos, calibration=cal)
    s = bus.servo(1)
    assert s.position == pytest.approx(-(2085 - 3072) * 2 * math.pi / 4096)
    s.configure()
    s.enable()
    s.move(0.0)
    assert target_of(servos[0])[0] == 3072
    bus.wire.sleep(0.5)
    assert s.position == pytest.approx(0.0, abs=0.01)
    s.move(0.5, duration=0.3)
    assert target_of(servos[0])[0] == 3072 - round(0.5 * 4096 / (2 * math.pi))
    bus.wire.sleep(0.15)
    st = s.state()
    assert st.velocity > 0 and st.raw["velocity_rad_s"] < 0, "direction-signed"
    assert st.raw["tau_motor_nm"] < 0 and st.torque_motor == -st.raw["tau_motor_nm"]
    assert st.raw["current_ma"] < 0 and st.current == -st.raw["current_ma"] / 1000
    assert st.duty == -st.raw["duty"] and st.torque_out == -st.raw["tau_out_nm"]
    bus.wire.sleep(0.4)
    assert s.position == pytest.approx(0.5, abs=0.01)
    joint = s.calibrate(zero="here")                                          # in memory: this Calibration has no file
    assert joint.zero_counts == round(servos[0].position) and cal.path is None
    assert s.position == pytest.approx(0.0, abs=0.001) and joint.direction == -1
    path = tmp_path / "cal.json"
    bus2 = bus_on(servos, calibration=path)
    s2 = bus2.servo(2)
    s2.calibrate(zero=2000, direction=-1)
    assert Calibration.load(path).joint(2).zero_counts == 2000 and Calibration.load(path).joint(2).direction == -1
    assert s2.position == pytest.approx(-(2085 - 2000) * 2 * math.pi / 4096)
    g = bus2.group([1, 2])
    g.calibrate(zeros="here", directions={1: 1})
    loaded = Calibration.load(path)
    assert (loaded.joint(1).zero_counts, loaded.joint(1).direction) == (round(servos[0].position), 1)
    assert all(abs(v.position) < 0.001 for v in g.read().values())
    g.calibrate(zeros={2: 100})
    assert Calibration.load(path).joint(2).zero_counts == 100


@pytest.mark.parametrize("grouped", [False, True])
def test_calibration_keeps_old_transforms_if_saving_fails(tmp_path, monkeypatch, grouped):
    path = tmp_path / "cal.json"
    cal = Calibration.load(path)
    cal.set(1, zero_counts=3000)
    cal.save()
    previous = path.read_bytes()
    bus = bus_on(release_servos(2), calibration=cal)

    def disk_failure(*args):
        raise OSError("write error")

    monkeypatch.setattr("openxl330.calibration.os.replace", disk_failure)
    with pytest.raises(CalibrationError, match="write error"):
        if grouped:
            bus.group([1, 2]).calibrate(zeros={1: 100, 2: 200}, directions={1: -1, 2: -1})
        else:
            bus.servo(1).calibrate(zero=100, direction=-1)
    assert cal.joint(1) == Calibration.load(path).joint(1)
    assert cal.joint(2) == Calibration.load(path).joint(2)
    assert path.read_bytes() == previous and not list(tmp_path.glob("*.tmp"))


# ---------------------------------------------------------------------------------------------- lease, exits
def test_lease_opt_in_default_off():
    servos = release_servos(2)
    bus = bus_on(servos)
    bus.scan()
    g = bus.group([1, 2])
    g.configure()
    g.enable()
    g.move({1: 0.1, 2: 0.1}, duration=0.1)
    g.read(); g.status(); g.disable()
    assert writes_to(bus.port, registers.LEASE) == [], "no lease unless asked"
    assert {v["state"] for v in g.lease_status().values()} == {"off"}
    assert bus.servo(1).lease_status()["ms"] == 0


def test_off_on_exit_false_by_default():
    servos = release_servos(2)
    bus = bus_on(servos)
    with bus.group([1, 2]) as g:
        g.configure()
        g.enable()
    assert [s.enabled for s in servos] == [1, 1], "no hidden auto-OFF"
    with bus.group([1, 2], off_on_exit=True):
        pass
    assert [s.enabled for s in servos] == [0, 0]
    bus.group([1, 2]).enable()
    with pytest.raises(KeyError):
        with bus.group([1, 2], off_on_exit=True):
            raise KeyError("boom")
    assert [s.enabled for s in servos] == [0, 0]
    bus.group([1, 2]).enable()
    with pytest.raises(KeyError) as e:                                        # a failing OFF does not hide the error
        with bus.group([1, 2], off_on_exit=True):
            servos[0].mode = servos[1].mode = "dead"
            raise KeyError("boom")
    assert "off_on_exit" in " ".join(getattr(e.value, "__notes__", ["off_on_exit"]))


def test_last_reset_not_supported_on_fast10(fw231_program):
    ten = without(SimServo(1, mode="app", program=fw231_program(), seed=1), registers.LAST_RESET)
    bus = bus_on([ten])
    s = bus.servo(1)
    assert (s.info.release, s.info.features) == ("FW231 (release 10 or older)",
                                                 frozenset({"motion", "fast_sync_read"}))
    with pytest.raises(NotSupported) as e:
        s.last_reset()
    assert "release 11" in str(e.value) and "openxl330 update --port SIM --id 1" in str(e.value)
    assert s.diagnostics()["last_reset"] is None


def test_reboot_on_a_faster_bus_comes_back_at_its_baud():
    servos = release_servos(2)
    bus = bus_on(servos)
    assert bus.set_baud(3_000_000)["confirmed"] == [1, 2]
    bus.servo(1).reboot()
    assert (servos[0].baud, servos[0].baud_unconfirmed, bus.port.baudrate) == (3_000_000, False, 3_000_000)
    assert bus.servo(1).last_reset()["reason"] == "host reboot" and bus.ping(2)["firmware"] == 231
    assert not bus.wire.uncertain


def test_faster_reboot_refuses_to_introduce_a_duplicate_id_at_app_baud():
    target = release_servos(1)[0]
    other = release_servos(1)[0]
    target.baud = 3_000_000
    bus = bus_on([target, other], baud=3_000_000)
    with pytest.raises(Refused, match=r"ID 1.*1 Mbaud"):
        bus.servo(1).reboot()
    assert not any(frame["instruction"] == 8 for _, raw in bus.port.tx_log
                   for kind, frame in parse_requests(raw) if kind == "frame")
    assert target.baud == bus.port.baudrate == 3_000_000
    assert other.baud == 1_000_000 and not bus.wire.uncertain


def test_reboot_checks_that_the_application_returned(monkeypatch):
    servo = release_servos(1)[0]
    bus = bus_on([servo])
    joint = bus.servo(1)
    monkeypatch.setattr(servo, "reset", lambda *args: setattr(servo, "mode", "dead"))
    with pytest.raises(NoReply):
        joint.reboot()


# ---------------------------------------------------------------------------------------------- the Bus itself
def test_low_level_passthrough():
    servos = release_servos(2)
    bus = bus_on(servos)
    assert isinstance(bus.wire, Wire) and bus.wire.port is bus.port
    assert bus.ping(1) == dict(model=1200, firmware=231, alert=False)
    assert bus.read(1, *registers.MARKER).startswith(registers.FW231_MARKER)
    assert bus.write(1, registers.LED, b"\1") is False and servos[0].led == 1
    assert bus.sync_read([1, 2], *registers.MOTION) == bus.fast_sync_read([1, 2], *registers.MOTION)
    bus.group([1, 2]).configure()
    bus.sync_write(registers.GAINS, 8, {1: registers.encode_gains(1.0, 0.0), 2: registers.encode_gains(2.0, 0.0)})
    assert (servos[0].gains, servos[1].gains) == ((65536, 0), (131072, 0))
    rows = bus.bulk_read([(1, 64, 68), (2, 1088, 20)])
    assert (len(rows[1]), len(rows[2])) == (68, 20)
    assert bus.reboot(1) == 128 and bus.ping(1)["firmware"] == 231
    bus.port.rx += b"\x55"
    with pytest.raises(BusUncertain):
        bus.ping(1)
    bus.reset()
    assert bus.ping(2)["firmware"] == 231


def test_simulated_bus_follows_the_callers_time():
    bus = Bus.simulated(3)
    assert [(i.id, i.firmware, i.release) for i in bus.scan()] == [(n, "FW231", images.DEFAULT_RELEASE)
                                                                    for n in (1, 2, 3)]
    g = bus.group([1, 2, 3])
    assert g.configure()[1] == dict(kp=registers.KP_COMP, kd=registers.KD_COMP, comp=True)
    g.enable()
    g.move({1: 0.4}, duration=0.1)
    time.sleep(0.3)                                                           # real time passes on the simulated bus
    assert g.read()[1].position == pytest.approx(0.4, abs=0.01)
    with pytest.raises(InvalidArgument):
        Bus.simulated(0)
    assert Bus.simulated(1).calibration.path is None, "a simulated bus never touches calibration.json by default"


def test_port_busy_message(monkeypatch):
    def busy(*args, **kwargs):
        raise serial.SerialException("[Errno 16] Resource busy")
    monkeypatch.setattr(serial, "Serial", busy)
    with pytest.raises(PortBusy) as e:
        Bus("/dev/ttyUSB7")
    assert "another program has the port open" in str(e.value) and "/dev/ttyUSB7" in str(e.value)

    def missing(*args, **kwargs):
        raise serial.SerialException(2, "could not open port /dev/ttyUSB8: [Errno 2] No such file or directory")
    monkeypatch.setattr(serial, "Serial", missing)
    with pytest.raises(AdapterError) as e:
        Bus("/dev/ttyUSB8")
    assert not isinstance(e.value, PortBusy) and "Bus.ports()" in str(e.value)


def test_a_port_name_opens_pyserial_exclusively(monkeypatch):
    seen = {}

    def fake_serial(name, baudrate, **kwargs):
        seen.update(name=name, baudrate=baudrate, **kwargs)
        port = SimPort(release_servos(1))
        port.port = name
        return port
    monkeypatch.setattr(serial, "Serial", fake_serial)
    with Bus("/dev/ttyFAKE0", 1_000_000) as bus:
        assert seen == dict(name="/dev/ttyFAKE0", baudrate=1_000_000, timeout=0.002, exclusive=True)
        assert bus.servo(1).info.firmware == "FW231"
    assert isinstance(Bus.ports(), list)


def test_set_baud_checks_each_servos_baud_word():
    """A servo found at the new baud must also report it in 844; one that does not is reported missing."""
    servos = release_servos(2)
    real = servos[1].read
    servos[1].read = lambda a, n: (0, struct.pack("<I", 1_000_000)) if (a, n) == (registers.BAUD, 4) else real(a, n)
    bus = bus_on(servos)
    r = bus.set_baud(2_000_000)
    assert (r["confirmed"], r["missing"]) == ([1], [2])


def test_unplug_mid_loop_raises_bus_uncertain():
    """Review Focus 2 at the API level: an adapter that fails mid-loop gives BusUncertain('adapter disconnected ...'),
    never a pyserial traceback, and nothing more is sent."""
    servos = release_servos(2)
    bus = bus_on(servos)
    g = bus.group([1, 2])
    g.read()

    def unplugged(data):
        raise serial.SerialException("write failed: [Errno 6] Device not configured")
    bus.port.write = unplugged
    with pytest.raises(BusUncertain) as e:
        g.read()
    assert "adapter disconnected" in str(e.value) and e.value.__cause__ is None
    with pytest.raises(BusUncertain):
        g.disable()


# ---------------------------------------------------------------------------------------------- enable holds where it is
def test_enable_holds_where_the_servo_is_after_a_move_and_a_hand_move():
    """The firmware's ARM plans from the shaft to the STORED target (the last 256 write) over its duration: enable()
    first makes the current position the target, so a joint moved by hand while OFF is held, not driven back."""
    servos = release_servos(1)
    bus = bus_on(servos)
    s = bus.servo(1)
    s.configure()
    s.enable()
    s.move(1.0)                                                               # a step to +1 rad
    bus.wire.sleep(0.5)
    s.disable()
    servos[0].position = 2048 + 300.0                                         # moved by hand while OFF
    s.enable()
    bus.wire.sleep(1.0)
    assert s.state().raw["position_counts"] == 2348 and target_of(servos[0])[:2] == (2348, 1)


def test_enable_after_power_up_and_a_hand_move_holds():
    """After power-up the stored target is the boot position (2000 ticks): enable() must not return the joint there."""
    servos = release_servos(1)
    bus = bus_on(servos)
    servos[0].position = 2085 + 700.0                                         # moved by hand after power-up
    s = bus.servo(1)
    s.configure()
    s.enable()
    bus.wire.sleep(1.5)
    assert s.state().raw["position_counts"] == 2785


def test_enable_clears_a_latched_duty():
    """A duty written while OFF stays latched through the firmware's ARM: enable() clears it (260 <- INT32_MIN)."""
    servos = release_servos(1)
    bus = bus_on(servos)
    s = bus.servo(1)
    s.configure()
    bus.write(1, registers.DUTY, struct.pack("<i", 300))                      # low level, while OFF: latched
    s.enable()
    bus.wire.sleep(0.5)
    st = s.state()
    assert (st.raw["position_counts"], st.raw["pwm"]) == (2085, 0)


def test_enable_needs_configure_and_is_idempotent():
    servos = release_servos(2)
    bus = bus_on(servos)
    with pytest.raises(Refused) as e:
        bus.group([1, 2]).enable()                                            # configured is 0 after every power-up
    assert "ID 1 is not configured since power-up: call configure() first" in str(e.value)
    assert len(writes_to(bus.port, registers.TORQUE_ENABLE)) == 0, "group preflight sends no ARM"
    g = ready(bus, [1, 2])
    sent = len(writes_to(bus.port, registers.TARGET)), len(writes_to(bus.port, registers.DUTY))
    g.enable()                                                                # already enabled: nothing changes
    assert [s.enabled for s in servos] == [1, 1]
    assert (len(writes_to(bus.port, registers.TARGET)), len(writes_to(bus.port, registers.DUTY))) == sent
    servos[0].reset("app")                                                    # a reboot loses the configuration
    with pytest.raises(Refused):
        bus.servo(1).enable()


def test_group_enable_preflight_leaves_earlier_servo_off():
    servos = release_servos(2)
    bus = bus_on(servos)
    bus.servo(1).configure()
    with pytest.raises(Refused, match="ID 2"):
        bus.group([1, 2]).enable()
    assert [servo.enabled for servo in servos] == [0, 0]
    assert not writes_to(bus.port, registers.TORQUE_ENABLE)


def test_group_enable_reports_progress_on_runtime_refusal():
    servos = release_servos(4)
    bus = bus_on(servos)
    group = bus.group([1, 2, 3, 4])
    group.configure()
    bus.servo(1).enable()
    flaky(servos[2], registers.TORQUE_ENABLE, 1, error=4, data=b"\1")
    with pytest.raises(Rejected) as caught:
        group.enable()
    assert [servo.enabled for servo in servos] == [1, 1, 0, 0]
    assert caught.value.enabled_ids == [2]
    assert caught.value.already_enabled_ids == [1]
    assert caught.value.unconfirmed_ids == [3, 4]


def test_arm_retry_refreshes_target_after_shaft_motion():
    simulated = release_servos(1)[0]
    bus = bus_on([simulated])
    servo = bus.servo(1)
    servo.configure()
    real = simulated.write

    def moved(address, data, broadcast=False):
        if address == registers.TORQUE_ENABLE and simulated.position == 2085:
            simulated.position = 2585
            return 7
        return real(address, data, broadcast)

    simulated.write = moved
    servo.enable()
    bus.wire.sleep(0.25)
    assert servo.state().raw["position_counts"] == 2585
    assert target_of(simulated)[:2] == (2585, 1)


def test_group_move_requires_readback_duration_to_match():
    simulated = release_servos(1)[0]
    bus = bus_on([simulated])
    group = ready(bus, [1])
    bus.servo(1).lease(0.02, action="hold")
    bus.wire.sleep(0.04)
    position = group.read()[1].position
    with pytest.raises(Rejected) as caught:
        group.move({1: position}, duration=0.3, profile="linear")
    assert caught.value.servo_ids == [1] and simulated.duration_ticks == 1


def test_group_off_continues_after_rejection_and_attaches_cleanup_error():
    servos = release_servos(2)
    bus = bus_on(servos)
    group = ready(bus, [1, 2])
    group.off_on_exit = True
    flaky(servos[0], registers.TORQUE_ENABLE, 1, data=b"\0")
    with pytest.raises(RuntimeError) as caught:
        with group:
            raise RuntimeError("test error")
    assert str(caught.value) == "test error"
    assert [servo.enabled for servo in servos] == [1, 0]
    assert caught.value.off_on_exit_error.confirmed_off_ids == [2]
    assert caught.value.off_on_exit_error.off_failures.keys() == {1}


def test_group_gains_reports_cleanly_refused_ids_with_unchecked_option():
    servos = release_servos(2)
    bus = bus_on(servos)
    group = bus.group([1, 2])
    bus.servo(1).configure()
    gains = {1: (2.0, 0.1), 2: (2.0, 0.1)}
    with pytest.raises(Rejected) as caught:
        group.set_gains(gains)
    assert caught.value.unconfirmed_ids == [2]
    assert caught.value.applied_ids == [1]
    group.set_gains(gains, verify=False)
    assert servos[0].gains == (131072, 6554) and servos[1].gains is None


def test_zero_proportional_gain_is_accepted():
    simulated = release_servos(1)[0]
    bus = bus_on([simulated])
    servo = bus.servo(1)
    assert servo.configure(kp=0, kd=0.03)["kp"] == 0
    servo.enable()
    servo.set_gains(0, 0.15)
    assert simulated.gains == (0, 9830)


def test_target_range_uses_firmware_q16_bounds_after_calibration():
    simulated = release_servos(1)[0]
    cal = Calibration()
    cal.set(1, zero_counts=30_000_000)
    bus = bus_on([simulated], calibration=cal)
    servo = bus.servo(1)
    with pytest.raises(InvalidArgument, match="reference range"):
        servo.move(0)
    assert not writes_to(bus.port, registers.TARGET[0])
    zero_raw_position = cal.to_rad(1, 0)
    servo.move(zero_raw_position, verify=False)
    assert simulated.target_counts == 0
    with pytest.raises(Rejected) as caught:
        bus.write(1, registers.TARGET[0], registers.encode_target(30_000_000, 1, 0))
    assert caught.value.error == 4


def test_group_calibration_checks_all_directions_before_any_changes():
    bus = bus_on(release_servos(2), calibration=Calibration())
    with pytest.raises(InvalidArgument):
        bus.group([1, 2]).calibrate(directions={1: -1, 2: 1.0})
    assert bus.calibration.sign(1) == 1


def test_reply_timing_uses_common_words_and_supports_short_legacy_read():
    servos = release_servos(2)
    without(servos[1], (1064, 24))
    bus = bus_on(servos)
    fast = bus.servo(1).reply_timing()
    assert fast["direct_max_us"] == 180 and fast["predecessor_done_max_us"] == 1900
    legacy = bus.group([1, 2]).diagnostics()[2]["reply_timing"]
    assert legacy["chained_max_us"] == 160 and "late_chains" not in legacy
    without(servos[1], (1064, 16))
    assert bus.servo(2).diagnostics()["reply_timing"] is None


def test_group_configure_batches_reads_before_acknowledged_writes():
    bus = bus_on(release_servos(3))
    group = bus.group([1, 2, 3])
    start = len(bus.port.tx_log)
    group.configure()
    assert len(bus.port.tx_log) - start == 6
    assert len(writes_to(bus.port, registers.PD_CONFIG)) == 3


def test_group_move_verification_uses_compact_status_and_target_reads():
    bus = bus_on(release_servos(6))
    group = ready(bus, range(1, 7))
    start = len(bus.port.tx_log)
    group.move({i: 0.4 for i in group.ids})
    requests = [f for _, raw in bus.port.tx_log[start:] for kind, f in parse_requests(raw) if kind == "frame"]
    assert [f["instruction"] for f in requests] == [0x83, 0x82, 0x82]
    assert [struct.unpack_from("<HH", f["params"]) for f in requests[1:]] == [(64, 20), registers.TARGET]


def test_servo_position_reads_only_the_current_count():
    bus = bus_on(release_servos(1))
    servo = bus.servo(1)
    assert servo.position == pytest.approx(rad(2085))
    kind, frame = parse_requests(bus.port.tx_log[-1][1])[0]
    assert kind == "frame" and frame["instruction"] == 2
    assert struct.unpack("<HH", frame["params"]) == (96, 4)


def test_group_unchecked_move_can_opt_in_to_less_settle_time():
    bus = bus_on(release_servos(1))
    group = ready(bus, [1])
    start = bus.wire.clock()
    group.move({1: 0.2}, verify=False)
    default_time = bus.wire.clock() - start
    start = bus.wire.clock()
    group.move({1: 0.3}, verify=False, settle_s=0)
    fast_time = bus.wire.clock() - start
    assert default_time - fast_time == pytest.approx(0.002)
    with pytest.raises(InvalidArgument):
        group.move({1: 0.4}, settle_s=0)


def test_group_read_reports_error_rows_apart_from_missing_ones():
    servos = release_servos(3)
    bus = bus_on(servos)
    g = bus.group([1, 2, 3])
    without(servos[1], registers.MOTION)                                      # ID 2 answers with error 4
    servos[2].mode = "dead"                                                   # ID 3 does not answer
    with pytest.raises(NoReply) as e:
        g.read(fast=False)
    assert (list(e.value.partial), e.value.missing, e.value.errors) == ([1], [3], {2: 4})
    assert "no reply from ID(s) [3]" in str(e.value) and "ID 2 answered with error 4" in str(e.value)


def test_scan_all_bauds_restores_the_host_baud_without_hiding_the_error():
    class Port(SimPort):
        refuse_1m = False

        @property
        def baudrate(self):
            return self._baud

        @baudrate.setter
        def baudrate(self, bps):
            if bps == 1_000_000 and self.refuse_1m:
                raise OSError("adapter gone")
            self._baud = bps

    junk = release_servos(1, first=7)[0]
    junk.baud = 2_000_000

    def garbled(frame, clock):
        junk.post = None
        port.refuse_1m = True                                                 # the restore will fail too
        return (0, b"\x12\x34")
    junk.on_frame = garbled
    port = Port(release_servos(1) + [junk])
    bus = Bus(port)
    with pytest.raises(BusUncertain) as e:
        bus.scan(all_bauds=True)
    assert "unexpected bytes" in str(e.value) and "adapter gone" not in str(e.value)
    port.refuse_1m = False
    junk.mode = "dead"
    bus2 = Bus(Port(release_servos(1)))
    bus2.port.servos[0].baud = 3_000_000
    assert [i.baud for i in bus2.scan(all_bauds=True)] == [3_000_000] and bus2.port.baudrate == 1_000_000


def test_close_closes_the_wire_log(tmp_path):
    bus = bus_on(release_servos(1), log=True)
    bus.ping(1)
    path = bus.wire.log.path
    assert path.is_file() and bus.wire.log.f is not None
    bus.close()
    assert bus.wire.log.f is None and path.read_text().count("\n") >= 2
    seen = []
    bus = bus_on(release_servos(1), log=seen.append)                          # a caller's log is the caller's
    bus.ping(1)
    bus.close()
    assert seen


# ---------------------------------------------------------------------------------------------- multi-servo (fleet) tests
def test_fleet_bus_wide_baud_switch_and_status(fw231_servos):
    servos = fw231_servos()
    bus = bus_on(servos)
    r = bus.set_baud(3_000_000)
    assert (r["confirmed"], r["missing"], r["previous"]) == (list(range(1, 7)), [], 1_000_000)
    assert all(s.baud == 3_000_000 and not s.baud_unconfirmed for s in servos)
    g = bus.group(range(1, 7))
    st = g.status()
    assert sorted(st) == list(range(1, 7))
    assert all(not v.enabled and v.faults == [] for v in st.values())
    diag = g.diagnostics()
    assert {d["id"] for d in diag.values()} == set(range(1, 7))
    assert {d["devcfg_state"] for d in diag.values()} == {"default"}          # v1: the image's default device block
    assert {i.baud for i in bus.scan()} == {3_000_000}


def test_fleet_servo_that_misses_the_switch_is_reported(fw231_servos):
    servos = fw231_servos(3)
    servos[2].write = lambda a, d, bc=False: 7                                # never accepts 844
    bus = bus_on(servos)
    r = bus.set_baud(2_000_000)
    assert (r["confirmed"], r["missing"]) == ([1, 2], [3])
    assert servos[2].baud == 1_000_000
    with pytest.raises(InvalidArgument):
        bus.set_baud(115_200)


def test_fleet_targets_gains_identify_off(fw231_servos):
    servos = fw231_servos(3)
    bus = bus_on(servos)
    g = ready(bus, [1, 2, 3])
    g.move({1: rad(100), 2: rad(-200), 3: rad(4096)}, duration=0.5)
    assert [target_of(s)[:2] for s in servos] == [(100, 1000), (-200, 1000), (4096, 1000)]
    bus.wire.sleep(1.0)
    assert [v.raw["position_counts"] for v in g.read().values()] == [100, -200, 4096]
    g.set_gains({1: (1.71, 0.043), 2: (0.5, 0.0), 3: (5.0, 0.06)})
    assert servos[0].gains == (round(1.71 * 65536), round(0.043 * 65536))
    g.identify(2)
    assert [s.led for s in servos] == [0, 1, 0]
    g.disable()
    assert {i: v.enabled for i, v in g.status().items()} == {1: False, 2: False, 3: False}


def test_fleet_configure_picks_gains_from_the_image(fw231_servos):
    comp = release_servos(1)[0]                                               # the bundled image: cancellation loaded
    bus = bus_on([comp])
    used = bus.servo(1).configure()
    assert used == dict(kp=3.5, kd=0.10, comp=True)
    assert comp.token == 1
    kp, kd = struct.unpack_from("<ii", comp.config, 4)
    assert (kp, kd) == (round(3.5 * 65536), round(0.10 * 65536))
    assert bus.servo(1).configure()["comp"] and comp.token == 2, "token = read 704 word 0 + 1"
    plain = fw231_servos(1)[0]                                                # an image without the cancellation
    assert bus_on([plain]).servo(1).configure() == dict(kp=1.71, kd=0.043, comp=False)
    comp.enabled = 1
    with pytest.raises(Refused) as e:
        bus_on([comp]).servo(1).configure()
    assert "call disable() first" in str(e.value)


def test_fleet_one_group_read_sync_and_fast(fw231_servos):
    servos = fw231_servos(3)
    bus = bus_on(servos)
    for s, (en, pwm, v, cur) in zip(servos, ((0, 0, 0.0, 0), (1, 240, 3.0, 300), (1, -120, 2.0, 150))):
        s.enabled, s.motion_pwm, s.motion_v, s.motion_current = en, pwm, v, cur
    g = bus.group([1, 2, 3])
    fast, slow = g.read(), g.read(fast=False)
    assert fast == slow
    assert (fast[1].torque_motor, fast[1].torque_out, fast[1].duty) == (0.0, 0.0, 0.0)            # OFF
    tm, to = registers.motion_model(240, 5.0, 3.0)
    assert fast[2].torque_motor == pytest.approx(tm, abs=1e-6) and fast[2].torque_out == pytest.approx(to, abs=1e-6)
    assert (fast[2].current, fast[3].current, fast[3].duty, fast[2].velocity) == (0.3, -0.15, -0.25, 3.0)


def test_fleet_keeps_group_order_and_reports_rejected_targets(fw231_servos):
    a, b = fw231_servos(2)
    bus = bus_on([a, b])
    assert bus.group([2, 1]).ids == [2, 1]
    assert list(bus.group([2, 1]).read()) == [2, 1]
    real = b.write
    g = ready(bus, [1, 2])
    b.write = lambda adr, data, bc=False: 7 if adr == 256 else real(adr, data, bc)        # busy: running profile
    with pytest.raises(Rejected) as e:
        g.move({1: rad(100), 2: rad(200)})
    assert e.value.servo_ids == [2]
    a.write = lambda adr, data, bc=False: 7 if adr == 844 else SimServo.write(a, adr, data, bc)
    r = bus.set_baud(3_000_000)
    assert (r["confirmed"], r["missing"]) == ([2], [1])


def test_fleet_command_lease(fw231_servos):
    a, b = fw231_servos(2)
    bus = bus_on([a, b])
    g = bus.group([1, 2])
    assert {v["state"] for v in g.lease_status().values()} == {"off"}                       # default after boot: off
    g.lease(0.1, "off")
    g.configure()
    g.enable()
    for _ in range(6):                                                         # heartbeats keep it alive
        bus.wire.sleep(0.05)
        g.heartbeat()
    assert (a.enabled, b.enabled) == (1, 1) and g.lease_status()[1]["state"] == "running"
    bus.wire.write_broadcast(registers.HEARTBEAT, b"\1")                      # a plain broadcast does not renew
    bus.wire.sleep(0.15)
    bus.read(1, *registers.STATUS)
    assert (a.enabled, b.enabled) == (0, 0)
    assert {v["state"] for v in g.lease_status().values()} == {"expired"}
    with pytest.raises(NotEnabled):                                          # disarmed: a new target is accepted
        bus.servo(1).move(0.3)
    assert target_of(a)[0] == 2048 + round(0.3 * 4096 / (2 * math.pi)), "target accepted when disarmed"
    bus.servo(1).lease(0.1, "hold")                                            # 'hold': refuses motion until re-armed
    bus.servo(1).enable()
    bus.wire.sleep(0.15)
    with pytest.raises(Rejected) as e:
        bus.servo(1).move(0.5)
    assert e.value.error == 7 and a.enabled == 1
    bus.servo(1).enable()                                                     # an explicit re-arm
    bus.servo(1).move(0.5)
    bus.servo(1).heartbeat()
    g.lease(0)
    assert {v["ms"] for v in g.lease_status().values()} == {0}


def test_fleet_last_reset_after_power_on_and_host_reboot(fw231_servos):
    s = fw231_servos(1)[0]
    bus = bus_on([s])
    servo = bus.servo(1)
    assert servo.info.release == "FW231 (release 11)" and "last_reset" in servo.info.features
    assert servo.last_reset()["reason"] == "power-on"
    assert servo.diagnostics()["last_reset"]["reason"] == "power-on"
    bus.wire.sleep(0.5)
    servo.reboot()
    assert servo.last_reset()["reason"] == "host reboot"
    assert (s.mode, s.enabled, bus.wire.uncertain) == ("app", 0, False)


def test_fleet_configure_retries_a_tick_race_and_lease_passes(fw231_servos):
    s = fw231_servos(1)[0]
    bus = bus_on([s])
    left = flaky(s, 320, 1)
    servo = bus.servo(1)
    assert servo.configure() == dict(kp=1.71, kd=0.043, comp=False)
    assert left == [0] and s.token == 1, "the 7 never applied the config: the same token stays valid"
    servo.lease(0.2, "off")
    servo.enable()
    assert servo.lease_status()["state"] == "running"
    servo.heartbeat()
    servo.disable()
    servo.lease(0)
    assert servo.lease_status()["state"] == "off"


def test_fleet_configure_on_an_armed_servo_still_raises(fw231_servos):
    s = fw231_servos(1)[0]
    bus = bus_on([s])
    flaky(s, 320, 99)
    with pytest.raises(Rejected) as e:
        bus.servo(1).configure()
    assert e.value.error == 7 and len(writes_to(bus.port, registers.PD_CONFIG)) == 20


def test_fleet_h8_fleet_waits_for_the_full_ping_window(fw231_servos, fw231_program):
    a, b = fw231_servos(2)
    far = SimServo(40, mode="app", program=fw231_program(), seed=70)          # stock NV ID 40
    bus = bus_on([a, b, far])
    bus.port.ping_slots = True                                                # ID 40 answers 120 ms after the ping
    assert [i.id for i in bus.scan()] == [1, 2, 40]
    assert bus.ping(1)["firmware"] == 231                                     # no late reply in front of it
    r = bus.set_baud(3_000_000, ids=[1, 2])
    assert (r["confirmed"], r["missing"]) == ([1, 2], [])
    assert bus.ping(40)["firmware"] == 231                                    # it followed too; the bus is clean
    assert bus.set_baud(1_000_000)["confirmed"] == [1, 2, 40]
    assert not bus.wire.uncertain


def test_fleet_wizard_connect_drains_stale_input(fw231_servos):
    """Opening a Bus drains stale input (an FTDI adapter hands over bytes buffered while the port was closed)."""
    s = fw231_servos(1)[0]
    port = SimPort([s])
    port.rx += b"\x00\x76\x67"
    bus = Bus(port)
    assert not bus.wire.uncertain
    assert bus.ping(1)["firmware"] == 231
    assert bus.servo(1).info.firmware == "FW231"
