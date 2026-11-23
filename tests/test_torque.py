"""The torque command (release 16+, ABI 4): Servo.set_torque / release_torque / control_mode and the Group versions,
against the simulated bus, which follows the firmware's rules for writes 264 / 268 (see openxl330.sim)."""
from __future__ import annotations

import math
import struct

import pytest

from openxl330 import Bus, Calibration, InvalidArgument, NotSupported, Refused, Rejected, images
from openxl330.registers import decode_drive, encode_torque
from openxl330.sim import SimPort, SimServo

from conftest import make_fw231_program

RELEASE = images.Release(images.DEFAULT_RELEASE)
Q24 = 1 << 24


def servos(n):
    return [SimServo(i, mode="app", program=RELEASE.packaged().raw, seed=i) for i in range(1, n + 1)]


def armed(sims, calibration=None):
    bus = Bus(SimPort(sims), calibration=calibration or Calibration(), check_adapter=False)
    group = bus.group([s.effective_id() for s in sims])
    group.configure()
    group.enable()
    return bus, group


def test_the_bundled_release_has_the_torque_command():
    assert RELEASE.release_id["number"] >= 16 and RELEASE.release_id["abi"] == 4
    bus, _ = armed(servos(1))
    assert "torque_command" in bus.servo_info(1).features


def test_set_torque_moves_the_shaft_and_release_holds_where_it_is():
    (sim,) = servos(1)
    bus, group = armed([sim])
    servo = bus.servo(1)
    start = sim.position
    servo.set_torque(0.02)
    assert (sim.tau_mode, sim.tau_q24) == (1, round(0.02 * Q24))
    mode = servo.control_mode()
    assert mode["mode"] == "torque" and mode["torque"] == pytest.approx(0.02, abs=1e-6)
    bus.wire.sleep(0.2)
    servo.position                                         # a read runs the model up to now
    assert sim.position > start + 50                       # turning in + (about 0.93 rad/s)
    servo.set_torque(-0.02)
    turned = sim.position
    bus.wire.sleep(0.2)
    servo.position
    assert sim.position < turned                           # reversed
    servo.release_torque()
    assert servo.control_mode()["mode"] == "position" and sim.tau_mode == 0
    held = sim.position
    bus.wire.sleep(0.2)
    servo.position
    assert abs(sim.position - held) < 1                    # holds where it was released


def test_raw_torque_uses_268_and_reports_raw_mode():
    (sim,) = servos(1)
    bus, _ = armed([sim])
    bus.servo(1).set_torque(0.02, compensate=False)
    assert sim.tau_mode == 2 and bus.servo(1).control_mode()["mode"] == "raw_torque"


def test_set_torque_follows_the_calibrated_direction():
    calibration = Calibration()
    calibration.set(1, direction=-1)
    (sim,) = servos(1)
    bus, _ = armed([sim], calibration)
    bus.servo(1).set_torque(0.03)
    assert sim.tau_q24 == -round(0.03 * Q24)
    assert bus.servo(1).control_mode()["torque"] == pytest.approx(0.03, abs=1e-6)


def test_set_torque_needs_an_enabled_servo_and_sends_nothing_while_off():
    (sim,) = servos(1)
    bus, group = armed([sim])
    group.disable()
    before = len(bus.port.tx_log)
    with pytest.raises(Refused) as e:
        bus.servo(1).set_torque(0.02)
    assert "call enable() first" in str(e.value) and sim.tau_mode == 0
    assert not [d for _, d in bus.port.tx_log[before:] if len(d) > 9 and d[7] == 3]   # no write was sent


@pytest.mark.parametrize("bad", [math.nan, math.inf, 128.0, -128.0, True, "0.1", None])
def test_set_torque_refuses_bad_values_before_sending(bad):
    (sim,) = servos(1)
    bus, _ = armed([sim])
    with pytest.raises(InvalidArgument):
        bus.servo(1).set_torque(bad)
    assert sim.tau_mode == 0


def test_a_move_in_torque_mode_is_refused_with_the_reason():
    (sim,) = servos(1)
    bus, _ = armed([sim])
    servo = bus.servo(1)
    servo.set_torque(0.01)
    with pytest.raises(Rejected) as e:
        servo.move(servo.position + 0.1, duration=0.5)
    assert e.value.error == 7 and "release_torque()" in str(e.value)
    servo.release_torque()
    servo.move(servo.position + 0.1, duration=0.5)         # accepted again


def test_disable_leaves_torque_mode_and_enable_starts_in_position():
    (sim,) = servos(1)
    bus, group = armed([sim])
    bus.servo(1).set_torque(0.02)
    group.disable()
    assert sim.tau_mode == 0
    group.enable()
    assert bus.servo(1).control_mode()["mode"] == "position"


def test_set_duty_and_set_torque_exclude_each_other():
    (sim,) = servos(1)
    bus, _ = armed([sim])
    servo = bus.servo(1)
    servo.set_duty(0.2)
    servo.set_torque(0.02)
    assert sim.duty is None and servo.control_mode()["mode"] == "torque"
    servo.set_duty(0.2)
    assert sim.tau_mode == 0 and servo.control_mode()["mode"] == "duty"
    servo.release_torque()                                 # INT32_MIN to 264 also leaves duty mode
    assert servo.control_mode()["mode"] == "position"


def test_an_expired_hold_lease_refuses_torque_until_enable():
    (sim,) = servos(1)
    bus, group = armed([sim])
    servo = bus.servo(1)
    servo.lease(0.1, "hold")
    servo.set_torque(0.02)                                 # renews the lease
    bus.wire.sleep(0.3)
    servo.position                                         # the lease expires: holds, torque mode left
    assert sim.tau_mode == 0
    with pytest.raises(Rejected) as e:
        servo.set_torque(0.02)
    assert e.value.error == 7 and "lease" in str(e.value)


def test_older_firmware_has_no_torque_command():
    old = SimServo(1, mode="app", program=make_fw231_program(), seed=1)
    bus = Bus(SimPort([old]), calibration=Calibration(), check_adapter=False)
    servo = bus.servo(1)
    assert "torque_command" not in servo.info.features
    with pytest.raises(NotSupported) as e:
        servo.set_torque(0.02)
    assert "release 16" in str(e.value) and "openxl330 update" in str(e.value)


def test_group_set_torque_in_one_sync_write_and_release():
    sims = servos(3)
    bus, group = armed(sims)
    group.set_torque({1: 0.02, 3: -0.01})
    assert [s.tau_mode for s in sims] == [1, 0, 1]
    assert [s.tau_q24 for s in sims] == [round(0.02 * Q24), 0, round(-0.01 * Q24)]
    group.set_torque(0.005, compensate=False)
    assert [s.tau_mode for s in sims] == [2, 2, 2]
    modes = group.control_modes()
    assert {i: m["mode"] for i, m in modes.items()} == {1: "raw_torque", 2: "raw_torque", 3: "raw_torque"}
    group.release_torque()
    assert [s.tau_mode for s in sims] == [0, 0, 0]


def test_group_set_torque_refuses_before_sending_when_a_member_is_off():
    sims = servos(2)
    bus, group = armed(sims)
    bus.servo(2).disable()
    with pytest.raises(Refused) as e:
        group.set_torque(0.02)
    assert "ID 2 is OFF" in str(e.value) and [s.tau_mode for s in sims] == [0, 0]


def test_group_set_torque_verify_names_a_member_that_did_not_take_it():
    sims = servos(2)
    bus, group = armed(sims)
    real = sims[1].write

    def refuse(a, data, broadcast=False):
        return 7 if a == 264 else real(a, data, broadcast)
    sims[1].write = refuse                                 # a silently rejected Sync Write row
    with pytest.raises(Rejected) as e:
        group.set_torque(0.02)
    assert "ID 2 did not switch to torque mode" in str(e.value) and e.value.servo_ids == [2]
    assert sims[0].tau_mode == 1


def test_torque_encoding_and_snapshot_decoding():
    assert encode_torque(0.02) == struct.pack("<i", round(0.02 * Q24))
    with pytest.raises(InvalidArgument):
        encode_torque(128)
    words = [0] * 16
    words[4], words[13] = round(-0.5 * Q24), 2 << 2
    words[5] = (-480 & 0xFFFF) << 16 | (-480 & 0xFFFF)     # applied duty at full scale (high half)
    words[5] -= 1 << 32
    words[9] = 2 | 8                                       # controller flags: saturated, limited
    assert decode_drive(struct.pack("<16i", *words)) == dict(mode="raw_torque", torque_nm=-0.5, duty=-1.0,
                                                             saturated=True, limited=True)
    words[9] = 0
    words[5] = 0
    words[4], words[13] = 0, 1 << 1
    assert decode_drive(struct.pack("<16i", *words))["mode"] == "duty"


def test_enable_clears_a_torque_command_stored_while_off():
    """The firmware keeps a 264 written while OFF and would apply it at the next ARM; enable() writes INT32_MIN to 260
    first, which leaves torque mode (release review, fast16)."""
    (sim,) = servos(1)
    bus, group = armed([sim])
    group.disable()
    bus.wire.write(1, 264, encode_torque(0.05))            # another client, while OFF
    assert sim.tau_mode == 1
    group.enable()
    assert sim.tau_mode == 0 and bus.servo(1).control_mode()["mode"] == "position"


def test_control_mode_reports_applied_duty_and_saturation():
    (sim,) = servos(1)
    bus, _ = armed([sim])
    servo = bus.servo(1)
    servo.set_torque(1.5)                                  # far above stall: the duty saturates
    bus.wire.sleep(0.05)
    mode = servo.control_mode()
    assert mode["mode"] == "torque" and mode["saturated"] and mode["duty"] == pytest.approx(1.0)
    assert not mode["limited"]                             # 1.5 N*m is inside the default 2 N*m limit
    servo.set_torque(3.0)
    assert servo.control_mode()["limited"] and servo.control_mode()["torque"] == pytest.approx(2.0)
    servo.set_torque(0.0)
    bus.wire.sleep(0.05)
    assert not servo.control_mode()["saturated"]


def test_position_mode_torque_is_never_negative_zero():
    calibration = Calibration()
    calibration.set(1, direction=-1)
    (sim,) = servos(1)
    bus, _ = armed([sim], calibration)
    mode = bus.servo(1).control_mode()
    assert mode["mode"] == "position" and math.copysign(1.0, mode["torque"]) == 1.0


def test_verify_false_sends_one_transaction_for_torque_loops():
    sims = servos(2)
    bus, group = armed(sims)
    servo = bus.servo(1)
    before = len(bus.port.tx_log)
    servo.set_torque(0.01, verify=False)
    assert len(bus.port.tx_log) - before == 1 and sims[0].tau_mode == 1
    before = len(bus.port.tx_log)
    group.set_torque(0.02, verify=False)
    assert len(bus.port.tx_log) - before == 1 and [s.tau_mode for s in sims] == [1, 1]
    group.disable()
    servo.set_torque(0.03, verify=False)                   # OFF: the firmware keeps it ...
    assert sims[0].tau_mode == 1
    group.enable()                                          # ... and enable() clears it before arming
    assert [s.tau_mode for s in sims] == [0, 0]


def test_position_mode_reports_the_pd_output_like_the_firmware():
    (sim,) = servos(1)
    bus, _ = armed([sim])
    servo = bus.servo(1)
    servo.move(servo.position + 0.5, duration=1.0, verify=False)
    bus.wire.sleep(0.3)
    mode = servo.control_mode()
    assert mode["mode"] == "position" and mode["torque"] > 0      # the PD pushes toward the moving reference
