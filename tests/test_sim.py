"""The simulated bus itself: servo behaviour the Wire tests and later layers rely on."""
from __future__ import annotations

import math
import struct
import zlib

import pytest

from conftest import UNKNOWN_LOADER
from openxl330 import images, install, registers, sim
from openxl330._protocol import framing
from openxl330.errors import NoReply, Rejected
from openxl330.sim import SimServo, default_eeprom
from openxl330.wire import Loader


@pytest.mark.parametrize("case", ["complete", "missing_head", "missing_row", "collision"])
def test_simulated_fast_packet_preserves_wire_bytes_with_one_crc_pass(monkeypatch, case):
    rows, size = 6, 20
    slots = {i: [bytes([0, i + 1]) + bytes([i]) * size] for i in range(rows)}
    if case == "missing_head":
        del slots[0]
    elif case == "missing_row":
        del slots[2]
    elif case == "collision":
        slots[2].append(bytes([0, 3]) + bytes([0xff]) * size)

    expected = bytearray()
    if 0 in slots:
        expected += b"\xff\xff\xfd\x00\xfe" + struct.pack("<H", 1 + rows * (size + 4)) + b"\x55"
        for index in range(rows):
            replies = slots.get(index, [])
            if not replies:
                break
            if len(replies) > 1:
                expected += sim.SimPort._collide(replies)
                break
            expected += replies[0]
            expected += struct.pack("<H", framing.crc16(expected))

    crc16 = framing.crc16
    hashed_bytes = []

    def count_crc(data, initial=0):
        hashed_bytes.append(len(data))
        return crc16(data, initial)

    monkeypatch.setattr(framing, "crc16", count_crc)
    assert sim.fast_packet(slots, rows, size) == expected
    assert sum(hashed_bytes) <= len(expected)


def test_unconfirmed_switch_reverts_after_2s(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, _ = sim_bus([s])
    wire.write(1, registers.BAUD, struct.pack("<I", 3_000_000))
    assert s.baud == 3_000_000 and s.baud_unconfirmed
    wire.sleep(2.1)
    assert struct.unpack("<I", wire.read(1, registers.BAUD, 4))[0] == 1_000_000


def test_sync_write_baud_switches_only_listed_servo(sim_bus, packaged):
    servos = [SimServo(i, mode="app", program=packaged().raw) for i in (1, 2)]
    wire, _ = sim_bus(servos)
    wire.sync_write(registers.BAUD, 4, {1: struct.pack("<I", 3_000_000)})
    assert (servos[0].baud, servos[1].baud) == (3_000_000, 1_000_000)
    assert servos[0].baud_unconfirmed
    wire.set_baud(3_000_000)
    assert wire.read(1, registers.BAUD, 4) == struct.pack("<I", 3_000_000)
    assert not servos[0].baud_unconfirmed


def test_bundled_release_identity_is_independent_per_simulator(packaged):
    one, two = [SimServo(i, mode="app", program=packaged().raw) for i in (1, 2)]
    original = two.release_id["number"]
    one.release_id["number"] -= 1
    assert two.release_id["number"] == original
    assert images.bundled(images.DEFAULT_RELEASE).release_id["number"] == original


def test_reads_can_span_status_and_other_low_register_regions(sim_bus, packaged):
    servo = SimServo(1, mode="app", program=packaged().raw)
    wire, _ = sim_bus([servo])
    table = wire.read(1, 0, 256)
    assert table[:64] == wire.read(1, 0, 64)
    status = registers.decode_status(table[64:132])
    assert status["position_counts"] == round(servo.position)
    assert wire.read(1, 64, 20) == table[64:84]
    assert struct.unpack_from("<i", wire.read(1, 64, 36), 32)[0] == status["position_counts"]
    assert table[208:224].rstrip(b"\0") == registers.FW231_MARKER


@pytest.mark.parametrize("attr, value, mask", [("temperature_c", 65, 16), ("supply_mv", 3400, 8)])
def test_protections_latch_while_off_until_the_condition_recovers(sim_bus, packaged, attr, value, mask):
    servo = SimServo(1, mode="app", program=packaged().raw)
    safe_value = getattr(servo, attr)
    setattr(servo, attr, value)
    wire, _ = sim_bus([servo])
    status = registers.decode_status(wire.read(1, *registers.STATUS))
    assert (status["enabled"], status["ready"], status["faults"]) == (0, 0, mask)
    with pytest.raises(Rejected) as exc:
        wire.write(1, registers.CLEAR_FAULTS, b"\x5a")
    assert exc.value.error == 7
    setattr(servo, attr, safe_value)
    assert registers.decode_status(wire.read(1, *registers.STATUS))["faults"] == mask
    wire.write(1, registers.CLEAR_FAULTS, b"\x5a")
    assert registers.decode_status(wire.read(1, *registers.STATUS))["faults"] == 0


def test_reboot_restores_default_protection_settings(sim_bus, packaged):
    servo = SimServo(1, mode="app", program=packaged().raw)
    wire, _ = sim_bus([servo])
    wire.write(1, registers.PROTECT, struct.pack("<3I", 800, 60, 65))
    servo.power_cycle()
    assert struct.unpack_from("<3I", wire.read(1, registers.PROTECT, 16)) == (1000, 65, 70)


def test_latency_tag_and_reset(sim_bus, fw231_servos):
    """The reply-timing maxima reset: only an all-zero 4-byte write is accepted."""
    s = fw231_servos(1)[0]
    wire, _ = sim_bus([s])
    wire.write(1, 1064, b"\0\0\0\0")
    with pytest.raises(Rejected):
        wire.write(1, 1064, b"\1\0\0\0")


def test_program_without_fw231_marker_is_dead(sim_bus, fw231_program):
    """Public servos are stock or FW231: a program without the FW231 marker does not boot."""
    other = bytearray(fw231_program())
    at = other.index(registers.FW231_MARKER)
    other[at:at + len(registers.FW231_MARKER)] = bytes(len(registers.FW231_MARKER))
    s = SimServo(1, mode="app", program=bytes(other))
    assert (s.mode, s.version) == ("dead", None)
    wire, _ = sim_bus([s])
    with pytest.raises(NoReply):
        wire.ping(1)
    s = SimServo(1, mode="app", program=fw231_program())
    assert (s.mode, s.version) == ("app", 231)


def test_stock_identity_comes_from_its_eeprom(sim_bus):
    eeprom = default_eeprom(4, baud_index=3)
    eeprom[6] = 52
    s = SimServo(4, mode="stock", eeprom=eeprom)
    assert (s.mode, s.version, s.baud) == ("stock", 52, 1_000_000)
    wire, _ = sim_bus([s])
    assert wire.ping(4) == dict(model=1200, firmware=52, alert=False)
    assert wire.scan(10) == {4: dict(model=1200, firmware=52, alert=False)}
    assert wire.read(4, 0, 64)[6] == 52
    assert SimServo(1, mode="stock").version == 50                 # the default factory servo: V50


def test_sim_boots_installed_release(sim_bus, packaged, flash_snapshot_of):
    """The simulated loader boots a bundled release from its shipped upload (images.plaintext_for): the program region
    then holds the release's plaintext, and the servo answers as that release (read 1120, read 1136, ABI 4, no A0 of
    the loader region), at its stock NV ID: the image carries the default device block."""
    s = SimServo(4, mode="stock", seed=4)
    before = flash_snapshot_of(s)
    wire, port = sim_bus([s], 57_600)
    loader = wire.enter_loader(4)
    pack = packaged()
    loader.upload(pack)
    loader.leave()
    assert (s.mode, s.version, s.effective_id(), port.baudrate) == ("app", 231, 4, 1_000_000)
    assert bytes(s.flash[0x4000:0x4000 + len(pack.raw)]) == pack.raw and s.opaque_upload is None
    assert flash_snapshot_of(s) == before and s.writes_outside_program == 0
    assert wire.flash_read(4, 0x08004000, len(pack.raw)) == pack.raw
    rid = images.Release(images.DEFAULT_RELEASE).release_id
    w0, w1, w2, w3 = struct.unpack("<4I", wire.read(4, 1120, 16))
    assert (w0 & 0xFFFF, (w0 >> 16) & 0xFF, w0 >> 24) == (rid["number"], 1, rid["abi"])
    assert (w1, w2, w3) == (rid["source_id"], 0, 0)
    assert wire.read(4, 1136, 8) == struct.pack("<II", zlib.crc32(before[0]), 0x4000)
    assert registers.decode_busdiag(wire.read(4, *registers.BUSDIAG))["abi"] == rid["abi"] == 4
    with pytest.raises(Rejected) as e:
        wire._flash_read(4, 0x08000000, 16)                             # the release never returns loader bytes
    assert e.value.error == 4
    assert wire.flash_read(4, 0x0800E000, 16) == before[1][:16]
    assert not wire.uncertain


def test_older_images_have_no_release_id(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, _ = sim_bus([s])
    for address, size in ((1120, 16), (1136, 8)):
        with pytest.raises(Rejected):
            wire.read(1, address, size)
    assert wire._flash_read(1, 0x08000000, 16) == bytes(s.flash[0:16])
    assert registers.decode_busdiag(wire.read(1, *registers.BUSDIAG))["abi"] == 2


def test_unknown_upload_boots_dead(sim_bus, fw231_program, upload_of, flash_snapshot_of):
    """No decoder ships: the simulated loader only knows the bundled releases' uploads. Any other bytes (here: a
    program sent as is, marker and all) are written as received and pass the loader's CRC check, since that is what
    was sent, but the servo then runs nothing."""
    s = SimServo(1, mode="app", program=fw231_program(), seed=4)
    before = flash_snapshot_of(s)
    wire, _ = sim_bus([s])
    loader = wire.enter_loader(1)
    unknown = upload_of(fw231_program(size=40000))
    assert images.plaintext_for(unknown.encoded) is None
    loader.upload(unknown)
    loader.leave()
    assert (s.mode, s.version) == ("dead", None)
    assert bytes(s.flash[0x4000:0x4000 + 40000]) == unknown.encoded and s.opaque_upload == unknown.encoded
    assert flash_snapshot_of(s) == before and s.writes_outside_program == 0
    with pytest.raises(NoReply):
        wire.ping(1)
    assert not wire.uncertain
    s.power_cycle()                                                     # still nothing to run
    assert (s.mode, s.version) == ("dead", None)


def test_sim_reports_the_known_loader_by_default(sim_bus):
    """A simulated servo's loader region has the proven loader's CRC-32 (b6934188; synthetic bytes, not the loader's),
    so a simulated install neither warns about the loader nor asks for the receipt. loader= gives another loader."""
    known = images.known_loaders()
    assert zlib.crc32(bytes(SimServo(1).flash[0:0x4000])) == 0xB6934188 and 0xB6934188 in known
    assert zlib.crc32(UNKNOWN_LOADER) not in known
    assert bytes(SimServo(1, loader=UNKNOWN_LOADER).flash[0:0x4000]) == UNKNOWN_LOADER
    s = SimServo(2, mode="stock", seed=9)
    wire, _ = sim_bus([s], 57_600)
    r = install.from_stock(wire, stock_id=2)
    assert r["completed"] and r["loader_known"] is True and r["loader_crc32"] == "b6934188"
    assert (r["warnings"], r["share_receipt"], r["share_why"]) == ([], False, [])


def test_fa_reports_a_mismatch_for_a_region_the_upload_did_not_write(sim_bus, packaged):
    """The simulated loader's FA answers only for what the last upload wrote: the CRC of that upload's bytes. A program
    region changed after the upload reads as a mismatch, and so does one longer than the upload. (Loader.upload runs FA
    itself, once, over exactly its upload; the model is checked here directly.)"""
    s = SimServo(1, seed=5)
    s.reset("loader")
    wire, _ = sim_bus([s], 57_600)
    loader = Loader(wire)
    loader.handshake()
    pack = packaged()
    assert s._fa_crc(len(pack.encoded), pack.crc16) != pack.crc16                     # nothing uploaded yet
    assert loader.upload(pack) == pack.crc16
    assert s._fa_crc(1000, 0) == framing.crc16(pack.encoded[:1000])
    assert s._fa_crc(len(pack.encoded) + 8, 0x1234) != 0x1234
    s.flash[0x4000 + 50] ^= 1
    assert s._fa_crc(len(pack.encoded), pack.crc16) != pack.crc16
    assert not wire.uncertain


# ---------------------------------------------------------------------------------------------- the motion model
# The firmware rules these tests pin (FW231 app.c / joint_control.c): the PD configuration (320) is lost at every boot and
# ARM is refused (7) until it is written, and while already enabled; ARM plans from the shaft to the STORED target over the
# stored duration (the last accepted 256 write, stored even while OFF; at boot the boot position over 2000 ticks); 260
# INT32_MIN is accepted in any state and makes the current position the target; any other duty needs the configuration,
# is accepted while OFF and stays latched through ARM until OFF or INT32_MIN.
def _rad(counts):
    return counts * 2 * math.pi / registers.COUNTS_PER_REV


def _motion(wire, servo_id=1):
    return registers.decode_motion(wire.read(servo_id, *registers.MOTION))


def _configure(wire, servo_id=1):
    token = struct.unpack_from("<I", wire.read(servo_id, *registers.PD_CONFIG_READ))[0] + 1
    wire.write(servo_id, registers.PD_CONFIG, registers.pd_config(1.71, 0.043, token))


def _target_word(wire, servo_id=1):
    return struct.unpack("<iIII", wire.read(servo_id, *registers.TARGET))


def _arm_here(wire, servo_id=1):
    """What the library's enable() does: target = here (1 tick), then ARM: the servo holds where it is."""
    here = _motion(wire, servo_id)["position_counts"]
    wire.write(servo_id, registers.TARGET[0], registers.encode_target(here, 1, 0))
    wire.write(servo_id, registers.TORQUE_ENABLE, b"\1")
    return here


@pytest.mark.parametrize("duration, profile", [(1, 1), (2, 0), (3_037_000_500, 0)])
def test_active_targets_require_representable_references(sim_bus, packaged, duration, profile):
    servo = SimServo(1, mode="app", program=packaged().raw)
    wire, _ = sim_bus([servo])
    _configure(wire)
    here = _arm_here(wire)
    with pytest.raises(Rejected) as exc:
        wire.write(1, registers.TARGET[0], registers.encode_target(here + 1000, duration, profile))
    assert exc.value.error == 4
    assert _target_word(wire) == (here, 1, 0, 0)


def test_impossible_stored_target_is_refused_at_arm(sim_bus, packaged):
    servo = SimServo(1, mode="app", program=packaged().raw)
    wire, _ = sim_bus([servo])
    _configure(wire)
    goal = round(servo.position) + 1000
    wire.write(1, registers.TARGET[0], registers.encode_target(goal, 1, 1))
    assert _target_word(wire) == (goal, 1, 0, 0)     # While OFF, write only stores the target.
    with pytest.raises(Rejected) as exc:
        wire.write(1, registers.TORQUE_ENABLE, b"\1")
    assert exc.value.error == 4
    assert servo.enabled == 0


@pytest.mark.parametrize("word, value", [(1, 51 << 16), (2, 3 << 16), (3, 0), (8, 1 << 24), (11, 0)])
def test_bad_pd_config_does_not_apply_or_consume_token(sim_bus, packaged, word, value):
    servo = SimServo(1, mode="app", program=packaged().raw)
    wire, _ = sim_bus([servo])
    config = bytearray(registers.pd_config(2, 0.25, 1))
    struct.pack_into("<I", config, 4 * word, value)
    with pytest.raises(Rejected) as exc:
        wire.write(1, registers.PD_CONFIG, config)
    assert exc.value.error == 4
    assert struct.unpack_from("<3I", wire.read(1, *registers.PD_CONFIG_READ)) == (0, 0, 0)
    wire.write(1, registers.PD_CONFIG, registers.pd_config(2, 0.25, 1))
    assert servo.token == 1


def test_gains_write_rejects_out_of_range_without_changing_pd(sim_bus, packaged):
    servo = SimServo(1, mode="app", program=packaged().raw)
    wire, _ = sim_bus([servo])
    _configure(wire)
    previous = wire.read(1, *registers.PD_CONFIG_READ)
    with pytest.raises(Rejected) as exc:
        wire.write(1, registers.GAINS, struct.pack("<ii", 2 << 16, -1))
    assert exc.value.error == 4
    assert wire.read(1, *registers.PD_CONFIG_READ) == previous


def test_released_image_refuses_lab_only_reset_trigger(sim_bus, packaged):
    servo = SimServo(1, mode="app", program=packaged().raw)
    wire, _ = sim_bus([servo])
    with pytest.raises(Rejected) as exc:
        wire.write(1, 1116, b"\1")
    assert exc.value.error == 7
    assert servo.hang_reboots == 0


def test_sim_motion_follows_target(sim_bus, fw231_servos):
    """SimServo's first-order position model: the position follows the reference at most 6 rad/s, step or quintic,
    only while ARMed; the motion block and the status position report that model. ARM executes the stored target."""
    s = fw231_servos(1)[0]
    wire, _ = sim_bus([s])
    start = _motion(wire)["position_counts"]
    goal = registers.CENTER_COUNTS + registers.COUNTS_PER_REV // 4                 # pi/2
    with pytest.raises(Rejected) as e:
        wire.write(1, registers.TORQUE_ENABLE, b"\1")                              # not configured since power-up
    assert e.value.error == 7
    assert struct.unpack_from("<3I", wire.read(1, *registers.PD_CONFIG_READ))[2] == 0
    _configure(wire)
    assert struct.unpack_from("<3I", wire.read(1, *registers.PD_CONFIG_READ))[2] == 1
    wire.write(1, registers.TARGET[0], registers.encode_target(goal, 1, 0))       # OFF: stored, nothing moves
    wire.sleep(0.3)
    assert _motion(wire)["position_counts"] == start and _target_word(wire) == (goal, 1, 0, 0)
    wire.write(1, registers.TORQUE_ENABLE, b"\1")                                  # ARM executes the stored step
    wire.sleep(0.1)
    m = _motion(wire)
    assert 0.5 < _rad(m["position_counts"] - start) <= 0.6 + 0.01                  # at most 6 rad/s
    assert 5.0 < m["velocity_rad_s"] <= 6.0 + 1e-6 and m["pwm"] > 0
    with pytest.raises(Rejected) as e:
        wire.write(1, registers.TORQUE_ENABLE, b"\1")                              # already enabled: refused
    assert e.value.error == 7
    wire.sleep(0.4)                                                                 # 0.5 s after the step
    m = _motion(wire)
    assert abs(_rad(m["position_counts"] - goal)) < 0.01 and abs(m["velocity_rad_s"]) < 0.05
    status = registers.decode_status(wire.read(1, *registers.STATUS))
    assert status["position_counts"] == m["position_counts"] and status["enabled"] == 1
    # quintic over 0.4 s, 0.5 rad back: halfway at 0.2 s (within the model's lag), there at the end
    back = goal - round(0.5 * registers.COUNTS_PER_REV / (2 * math.pi))
    wire.write(1, registers.TARGET[0], registers.encode_target(back, 800, 1))
    assert _target_word(wire) == (back, 800, 0, 0), "read 256 reports 0 in the profile word"
    wire.write(1, registers.TARGET[0], registers.encode_target(back, 800, 0))
    wire.sleep(0.2)
    mid = _motion(wire)["position_counts"]
    assert abs(_rad(mid - (goal + back) / 2)) < 0.05
    wire.sleep(0.3)
    assert abs(_rad(_motion(wire)["position_counts"] - back)) < 0.01
    for bad in (registers.encode_target(back, 0, 0), registers.encode_target(back, 1, 2),
                struct.pack("<iIII", back, 1, 0, 1)):
        with pytest.raises(Rejected) as e:
            wire.write(1, registers.TARGET[0], bad)                                 # no duration, profile 2, word 3
        assert e.value.error == 4


def test_sim_arm_plans_to_the_stored_target_like_the_firmware(sim_bus, fw231_servos):
    """At boot the stored target is the boot position over 2000 ticks: a joint moved by hand and then ARMed goes back
    there over 1 s. 260 INT32_MIN (accepted OFF and unconfigured) makes the current position the target: ARM holds."""
    s = fw231_servos(1)[0]
    wire, _ = sim_bus([s])
    boot = _motion(wire)["position_counts"]
    assert _target_word(wire) == (boot, 2000, 0, 0)
    s.position = boot + 600.0                                                       # moved by hand while OFF
    _configure(wire)
    wire.write(1, registers.TORQUE_ENABLE, b"\1")
    wire.sleep(0.5)
    assert abs(_motion(wire)["position_counts"] - (boot + 300)) < 30                 # halfway through the 1 s quintic
    wire.sleep(0.6)
    assert abs(_motion(wire)["position_counts"] - boot) <= 1
    wire.write(1, registers.TORQUE_ENABLE, b"\0")
    s.position = boot - 400.0                                                       # moved by hand again
    wire.write(1, registers.DUTY, struct.pack("<i", -2 ** 31))                       # target = here
    assert _target_word(wire)[0] == boot - 400
    wire.write(1, registers.TORQUE_ENABLE, b"\1")
    wire.sleep(1.0)
    assert _motion(wire)["position_counts"] == boot - 400
    fresh = fw231_servos(1)[0]
    wire, _ = sim_bus([fresh])
    wire.write(1, registers.DUTY, struct.pack("<i", -2 ** 31))                       # unconfigured and OFF: accepted


def test_sim_off_and_faults_stop_the_model(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, _ = sim_bus([s])
    _configure(wire)
    here = _arm_here(wire)
    wire.write(1, registers.TARGET[0], registers.encode_target(here + 4000, 1, 0))
    wire.sleep(0.1)
    wire.write(1, registers.TORQUE_ENABLE, b"\0")                                  # OFF mid-move: it stops
    stopped = _motion(wire)
    wire.sleep(0.3)
    m = _motion(wire)
    assert m["position_counts"] == stopped["position_counts"] and (m["velocity_rad_s"], m["pwm"]) == (0.0, 0)
    assert (m["tau_motor_nm"], m["tau_out_nm"]) == (0.0, 0.0)
    wire.write(1, registers.DUTY, struct.pack("<i", -2 ** 31))
    wire.write(1, registers.TORQUE_ENABLE, b"\1")
    s.faults = 4                                                                   # a latched fault turns it OFF
    wire.sleep(0.01)
    status = registers.decode_status(wire.read(1, *registers.STATUS))
    assert (status["enabled"], status["faults"]) == (0, 4)
    with pytest.raises(Rejected) as e:
        wire.write(1, registers.TORQUE_ENABLE, b"\1")                              # no ARM with a latched fault
    assert e.value.error == 7
    wire.write(1, registers.CLEAR_FAULTS, b"\x5a")
    wire.write(1, registers.TORQUE_ENABLE, b"\1")
    s.reset("app")                                                                 # a reboot: OFF, configuration lost
    assert (s.enabled, s.configured) == (0, False)


def test_sim_protections_comp_and_duty(sim_bus, fw231_servos):
    """848 protections {current RMS mA, temperature C, max volts x10 (50..70)} read back with the live RMS, are
    refused while enabled and trip the servo OFF; 324 compensation reads back what was written; 260 open-loop duty
    needs the configuration, is latched while OFF and applies at ARM, until OFF or INT32_MIN."""
    s = fw231_servos(1)[0]
    wire, _ = sim_bus([s])
    assert struct.unpack("<4I", wire.read(1, registers.PROTECT, 16))[:3] == (1000, 65, 70)       # defaults
    wire.write(1, registers.PROTECT, struct.pack("<3I", 800, 60, 65))
    assert struct.unpack("<4I", wire.read(1, registers.PROTECT, 16))[:3] == (800, 60, 65)
    for bad in (struct.pack("<3I", 800, 10, 65), struct.pack("<3I", 800, 60, 75)):
        with pytest.raises(Rejected) as e:
            wire.write(1, registers.PROTECT, bad)                                   # temperature 10, vmax 7.5 V
        assert e.value.error == 4
    comp = struct.pack("<10I", 1, 2, 3, 4, 5, 6, 7, 8, 0x102, 0x31504d43)
    wire.write(1, registers.COMP[0], comp)
    assert wire.read(1, *registers.COMP) == comp
    with pytest.raises(Rejected) as e:
        wire.write(1, registers.DUTY, struct.pack("<i", 240))                       # not configured: 4
    assert e.value.error == 4
    with pytest.raises(Rejected) as e:
        wire.write(1, registers.GAINS, registers.encode_gains(1.0, 0.0))            # live gains need it too
    assert e.value.error == 4
    _configure(wire)
    here = _motion(wire)["position_counts"]
    wire.write(1, registers.DUTY, struct.pack("<i", 240))                          # OFF: accepted and latched
    wire.sleep(0.1)
    assert _motion(wire)["position_counts"] == here
    wire.write(1, registers.TORQUE_ENABLE, b"\1")                                  # ARM keeps it: open loop at once
    with pytest.raises(Rejected) as e:
        wire.write(1, registers.PROTECT, struct.pack("<3I", 800, 60, 65))          # not while enabled
    assert e.value.error == 7
    wire.sleep(0.2)
    m = _motion(wire)
    assert m["pwm"] == 240 and m["position_counts"] > here
    wire.write(1, registers.DUTY, struct.pack("<i", -2 ** 31))                     # back to PD: holds, target = here
    held = _motion(wire)["position_counts"]
    assert abs(_target_word(wire)[0] - held) <= 1
    wire.sleep(0.2)
    assert abs(_motion(wire)["position_counts"] - held) <= 1
    wire.write(1, registers.TORQUE_ENABLE, b"\0")
    wire.write(1, registers.DUTY, struct.pack("<i", 240))
    wire.write(1, registers.TORQUE_ENABLE, b"\0")                                  # OFF clears a latched duty
    wire.write(1, registers.TORQUE_ENABLE, b"\1")
    wire.sleep(0.1)
    assert _motion(wire)["pwm"] != 240
    s.temperature_c = 61
    wire.sleep(0.01)
    status = registers.decode_status(wire.read(1, *registers.STATUS))
    assert (status["enabled"], status["faults"], status["temperature"]) == (0, 16, 61)


def test_sim_older_image_refuses_a_retarget_while_moving(sim_bus, fw231_servos, packaged):
    """Release 12+ retargets a running profile; an older image answers error 7 until the move ends."""
    old = fw231_servos(1)[0]
    new = SimServo(2, mode="app", program=packaged().raw, seed=2)
    wire, _ = sim_bus([old, new])
    for i in (1, 2):
        _configure(wire, i)
        _arm_here(wire, i)
        wire.write(i, registers.TARGET[0], registers.encode_target(2500, 2000, 0))   # 1 s quintic
    wire.sleep(0.1)
    with pytest.raises(Rejected) as e:
        wire.write(1, registers.TARGET[0], registers.encode_target(1500, 1, 0))
    assert e.value.error == 7
    wire.write(2, registers.TARGET[0], registers.encode_target(1500, 1, 0))
    wire.sleep(1.0)
    wire.write(1, registers.TARGET[0], registers.encode_target(1500, 1, 0))       # the move ended


def test_sim_lease_expiry_stops_the_model_at_its_instant(sim_bus, fw231_servos):
    """An 'off' lease that runs out in the middle of a long wait stops the servo where it was at the expiry, not at
    the end of the wait; a 'hold' expiry holds there and makes it the target."""
    s = fw231_servos(1)[0]
    wire, _ = sim_bus([s])
    _configure(wire)
    wire.write(1, registers.LEASE, registers.encode_lease(100, "off"))
    here = _arm_here(wire)
    wire.write(1, registers.TARGET[0], registers.encode_target(here + 8000, 1, 0))
    wire.sleep(0.5)
    m = _motion(wire)
    assert s.enabled == 0 and 0.5 < _rad(m["position_counts"] - here) <= 0.62     # ~0.1 s at 6 rad/s, not 0.5 s
    wire.write(1, registers.LEASE, registers.encode_lease(100, "hold"))
    here = _arm_here(wire)
    wire.write(1, registers.TARGET[0], registers.encode_target(here - 8000, 1, 0))
    wire.sleep(0.5)
    m = _motion(wire)
    assert s.enabled == 1 and 0.5 < _rad(here - m["position_counts"]) <= 0.62
    assert abs(_target_word(wire)[0] - m["position_counts"]) <= 1
