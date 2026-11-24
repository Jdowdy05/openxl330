"""Baud migration and identity policy, using the in-memory bus."""
from __future__ import annotations

import pytest

from openxl330 import Bus, BusUncertain, Calibration, InvalidArgument, NoReply, NoServosFound, Refused, UnsupportedServo
from openxl330 import images, registers
from openxl330.sim import SimPort, SimServo, default_eeprom


def app(servo_id=1, baud=1_000_000):
    device = SimServo(servo_id, mode="app", program=images.bundled(images.DEFAULT_RELEASE).packaged().raw)
    device.baud = baud
    return device


def bus_on(devices):
    return Bus(SimPort(devices), calibration=Calibration(), check_adapter=False)


def sent_write(bus):
    return any(frame[:3] == b"\xff\xff\xfd" and frame[7] == 3 for _, frame in bus.port.tx_log)


@pytest.mark.parametrize("ids", [None, [2]])
def test_set_baud_refuses_destination_collision_even_for_unlisted_source_id(ids):
    first, second, bystander = app(1), app(2), app(1, 3_000_000)
    with bus_on([first, second, bystander]) as bus:
        with pytest.raises(Refused, match="already.*ID 1|ID 1.*already") as e:
            bus.set_baud(3_000_000, ids=ids)
        assert not isinstance(e.value, InvalidArgument), "a state refusal, not a bad argument"
        assert bus.port.baudrate == first.baud == second.baud == 1_000_000
        assert bystander.baud == 3_000_000 and not sent_write(bus)
        assert not bus.wire.uncertain


def test_destination_responder_cannot_confirm_an_id_absent_at_source():
    first, bystander = app(1), app(2, 3_000_000)
    with bus_on([first, bystander]) as bus:
        report = bus.set_baud(3_000_000, ids=[1, 2])
        assert report["confirmed"] == [1]
        assert report["missing"] == [2]


def test_destination_collision_keeps_strict_latch_and_restores_host_baud():
    source = app(1)
    destination = [app(2, 3_000_000), app(2, 3_000_000)]
    with bus_on([source, *destination]) as bus:
        with pytest.raises(BusUncertain):
            bus.set_baud(3_000_000)
        assert bus.port.baudrate == source.baud == 1_000_000
        assert not sent_write(bus)
        assert bus.wire.uncertain


def test_empty_source_never_broadcasts_configuration():
    with bus_on([app(1, 3_000_000)]) as bus:
        result = bus.set_baud(3_000_000)
        assert result["confirmed"] == []
        assert bus.port.baudrate == 1_000_000
        assert not sent_write(bus)


@pytest.mark.parametrize("compatibility", ["model", "marker", "abi"])
def test_set_baud_refuses_unsupported_source(compatibility):
    supported, unsupported = app(1), app(2)
    if compatibility == "model":
        low_table = unsupported.low_table
        unsupported.low_table = lambda: (1190).to_bytes(2, "little") + low_table()[2:]
    elif compatibility == "marker":
        read = unsupported.read
        unsupported.read = lambda a, n: (4, b"") if (a, n) == registers.MARKER else read(a, n)
    else:
        unsupported.release_id = dict(unsupported.release_id, abi=6)          # newer than MAX_ABI (5)
    with bus_on([supported, unsupported]) as bus:
        assert not bus.scan()[1].supported
        with pytest.raises(UnsupportedServo, match="ID 2"):
            bus.set_baud(3_000_000, ids=[1])
        assert not sent_write(bus)
        assert supported.baud == unsupported.baud == 1_000_000


def test_confirming_later_baud_changes_continues_after_a_clean_read_error():
    a, b = app(1), app(2)
    read = a.read
    a.read = lambda address, n: (4, b"") if (address, n) == (registers.BAUD, 4) else read(address, n)
    with bus_on([a, b]) as bus:
        result = bus.set_baud(3_000_000)
        assert result["confirmed"] == [2]
        assert result["missing"] == [1]
        assert not bus.wire.uncertain


def test_matching_release_number_and_source_do_not_hide_a_different_variant():
    bundled = app(1)
    plain_pd = app(2)
    plain_pd.release_id = dict(plain_pd.release_id, variant="pd")
    with bus_on([bundled, plain_pd]) as bus:
        first, second = bus.scan()
        assert first.release == images.DEFAULT_RELEASE
        assert second.release == f"FW231 release {plain_pd.release_id['number']}"
        assert second.supported


def test_disagreeing_release_and_bus_abis_are_refused():
    device = app()
    read = device.read

    def status(address, size):
        if (address, size) == registers.RELEASE_ID:
            return 0, registers.encode_release_id(**dict(device.release_id, abi=2))
        return read(address, size)

    device.read = status
    with bus_on([device]) as bus:
        with pytest.raises(UnsupportedServo, match="ABI.*disagree"):
            bus.servo(1)
        assert not bus.servo_info(1).supported
        assert not bus.wire.uncertain


def test_reset_reprobes_device_replaced_at_the_same_id():
    device = app()
    with bus_on([device]) as bus:
        assert bus.servo_info(1).release == images.DEFAULT_RELEASE
        device.release_id = dict(device.release_id, number=device.release_id["number"] - 1)
        bus.reset()
        assert bus.servo_info(1).release == f"FW231 release {device.release_id['number']}"


def test_completed_scan_invalidates_absent_devices():
    device = app()
    with bus_on([device]) as bus:
        assert bus.servo_info(1).supported
        device.mode = "dead"
        with pytest.raises(NoServosFound):
            bus.scan()
        with pytest.raises(NoReply):
            bus.servo_info(1)


@pytest.mark.parametrize("rediscover", [Bus.reset, Bus.scan])
def test_live_handles_refuse_incompatible_replacement(rediscover):
    stock = SimServo(1, mode="stock", eeprom=default_eeprom(1, baud_index=3))
    with bus_on([app()]) as bus:
        servo = bus.servo(1)
        group = bus.group([1])
        stock.clock = bus.port.clock
        bus.port.servos[:] = [stock]
        rediscover(bus)
        first = len(bus.port.tx_log)

        with pytest.raises(UnsupportedServo):
            servo.led = True
        with pytest.raises(UnsupportedServo):
            servo.enable()
        with pytest.raises(UnsupportedServo):
            group.identify(1)
        with pytest.raises(UnsupportedServo):
            group.read()

        assert stock.led is None and not stock.enabled
        assert all(frame[7] in (1, 2) for _, frame in bus.port.tx_log[first:])
        assert not bus.wire.uncertain


def test_live_handle_refuses_a_different_supported_firmware_after_scan():
    device = app()
    with bus_on([device]) as bus:
        old_handle = bus.servo(1)
        device.release_id = dict(device.release_id, number=device.release_id["number"] - 1)
        assert bus.scan()[0].supported
        with pytest.raises(Refused, match="identity changed.*new"):
            old_handle.led = True
        assert device.led is None
        bus.servo(1).led = True
        assert device.led == 1


def test_group_disable_continues_after_a_replaced_member_is_refused():
    stock, valid = SimServo(1, mode="stock", eeprom=default_eeprom(1, baud_index=3)), app(2)
    with bus_on([app(1), valid]) as bus:
        group = bus.group([1, 2])
        valid.enabled = stock.enabled = 1
        stock.clock = bus.port.clock
        bus.port.servos[0] = stock
        bus.scan()
        with pytest.raises(UnsupportedServo) as failure:
            group.disable()
        assert failure.value.confirmed_off_ids == [2]
        assert failure.value.unconfirmed_ids == [1]
        assert stock.enabled and not valid.enabled


def test_live_handles_keep_working_without_reprobe_after_confirmed_baud_change():
    device = app()
    with bus_on([device]) as bus:
        servo, group = bus.servo(1), bus.group([1])
        assert bus.set_baud(2_000_000)["confirmed"] == [1]
        first = len(bus.port.tx_log)
        servo.led = True
        group.heartbeat()
        assert group.read()[1].id == 1
        assert servo.info.baud == 2_000_000
        assert [(baud, frame[7]) for baud, frame in bus.port.tx_log[first:]] == [
            (2_000_000, 3), (2_000_000, 0x83), (2_000_000, 0x8A),
        ]


# ---------------------------------------------------------------------------------------- OFF after a bus latch
def _armed_group(n=3):
    from conftest import make_bus
    from openxl330 import images
    from openxl330.sim import SimServo as _Sim
    program = images.bundled(images.DEFAULT_RELEASE).packaged().raw
    servos = [_Sim(i, mode="app", program=program, seed=i) for i in range(1, n + 1)]
    wire, port = make_bus(servos)
    bus = Bus(port, calibration=Calibration(), check_adapter=False)
    group = bus.group([s.effective_id() for s in servos])
    group.configure()
    group.enable()
    assert all(s.enabled for s in servos)
    return bus, group, servos, port


def _stray_after_first_write(port):
    real = port.write
    calls = []

    def write(raw):
        n = real(raw)
        calls.append(raw)
        if len(calls) == 1:
            port.rx += b"\x00\x55"                       # stray bytes after the first OFF request's reply
        return n
    port.write = write


def test_group_disable_still_turns_off_every_member_after_a_latch():
    """Release review P1-1: one stray byte after ID 1's OFF latched the Wire and left IDs 2 and 3 armed."""
    bus, group, servos, port = _armed_group()
    _stray_after_first_write(port)
    with pytest.raises(BusUncertain) as caught:
        group.disable()
    assert [s.enabled for s in servos] == [0, 0, 0]
    e = caught.value
    assert e.unconfirmed_ids == [1, 2, 3] and e.emergency_off_ids == [1, 2, 3] and "unconfirmed" in str(e)
    assert bus.wire.uncertain                            # still latched: nothing confirms the OFF
    with pytest.raises(BusUncertain):
        bus.wire.ping(1)


def test_off_on_exit_turns_off_every_member_when_the_block_latched_the_bus():
    bus, group, servos, port = _armed_group()
    group.off_on_exit = True
    with pytest.raises(BusUncertain) as caught:
        with group:
            port.rx += b"\x00"                           # the block's next request latches on this
            group.read()
    assert [s.enabled for s in servos] == [0, 0, 0]
    assert caught.value.off_on_exit_error.emergency_off_ids == [1, 2, 3]


def test_emergency_off_sends_nothing_while_a_servo_may_be_in_the_loader():
    bus, group, servos, port = _armed_group(1)
    wire = bus.wire
    wire.loader_phase = "f9_acked"
    before = len(port.tx_log)
    assert wire.emergency_off([1]) == [] and len(port.tx_log) == before


def test_cli_off_ids_still_turns_off_the_rest_after_a_latch(monkeypatch, capsys):
    from openxl330.cli import main
    bus, group, servos, port = _armed_group()
    _stray_after_first_write(port)
    monkeypatch.setattr(Bus, "simulated", classmethod(lambda cls, n: bus))
    assert main(["--sim", "3", "off", "--ids", "1-3"]) == 1
    out, err = capsys.readouterr()
    assert [s.enabled for s in servos] == [0, 0, 0]
    assert "OFF sent once, unconfirmed (bus uncertain)" in out and "Could not confirm torque OFF" in err


def test_off_after_a_latch_does_not_need_the_handle_identity_check():
    """Review 2, finding 1: a stale handle (after bus.reset) re-checks its identity first; a latch there must still
    end in an OFF to every member."""
    bus, group, servos, port = _armed_group()
    bus.reset()                                          # every handle now re-checks before its next request
    port.rx += b"\x00"
    with pytest.raises(BusUncertain) as caught:
        group.disable()
    assert [s.enabled for s in servos] == [0, 0, 0] and caught.value.emergency_off_ids == [1, 2, 3]


def test_off_after_a_latch_goes_out_at_the_members_baud_too():
    """Review 2, finding 1: a scan that latched at another baud leaves the host there; the OFF must reach the
    members at their own baud."""
    bus, group, servos, port = _armed_group()
    bus.wire.set_baud(57_600)
    port.rx += b"\x00"
    with pytest.raises(BusUncertain):
        bus.wire.scan(252)                               # latched, host still at 57600
    with pytest.raises(BusUncertain) as caught:
        group.disable()
    assert [s.enabled for s in servos] == [0, 0, 0] and caught.value.emergency_off_ids == [1, 2, 3]


def test_a_failing_wire_log_does_not_stop_the_emergency_off():
    bus, group, servos, port = _armed_group()

    def log(event):
        raise OSError(28, "No space left on device")
    bus.wire.log = log                                   # the first logged exchange latches the Wire
    with pytest.raises(BusUncertain) as caught:
        group.disable()
    assert [s.enabled for s in servos] == [0, 0, 0] and caught.value.emergency_off_ids == [1, 2, 3]


def test_ctrl_c_during_disable_still_sends_off_to_the_rest():
    bus, group, servos, port = _armed_group()
    real = port.write
    calls = []

    def write(raw):
        calls.append(raw)
        if len(calls) == 1:
            raise KeyboardInterrupt
        return real(raw)
    port.write = write
    with pytest.raises(KeyboardInterrupt) as caught:
        group.disable()
    assert [s.enabled for s in servos] == [0, 0, 0] and caught.value.emergency_off_ids == [1, 2, 3]


def test_cli_off_all_still_broadcasts_after_stray_bytes(monkeypatch, capsys):
    from openxl330.cli import main
    bus, group, servos, port = _armed_group()
    real = port.write

    def write(raw):
        n = real(raw)
        if raw[4] == 0xFE and raw[7] == 3:               # the broadcast OFF: then garble the confirming scan
            port.rx += b"\x00"
        return n
    port.rx += b"\x00\x55"                               # bytes already on the line before the broadcast
    port.write = write
    monkeypatch.setattr(Bus, "simulated", classmethod(lambda cls, n: bus))
    main(["--sim", "3", "off", "--all"])
    assert [s.enabled for s in servos] == [0, 0, 0]


def test_emergency_broadcast_off_reaches_every_servo_on_a_latched_wire():
    bus, group, servos, port = _armed_group()
    port.rx += b"\x00"
    with pytest.raises(BusUncertain):
        bus.wire.ping(1)
    assert bus.wire.emergency_off(broadcast=True) == [0xFE]
    assert [s.enabled for s in servos] == [0, 0, 0] and bus.wire.uncertain
