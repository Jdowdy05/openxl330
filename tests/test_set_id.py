"""Over-the-bus ID change (fast17+, ABI 5): register 1144, the simulated ID log, and (later tasks) the recorded run,
Servo.set_id and the CLI."""
from __future__ import annotations

import struct

import pytest

from openxl330 import Bus, Calibration, images, registers
from openxl330.errors import InvalidArgument, NoReply
from openxl330.sim import SimPort, SimServo

from conftest import make_bus, make_fw231_program

RELEASE = images.Release(images.DEFAULT_RELEASE)
FAST17 = dict(number=17, variant='comp', abi=5, source_id=0x17171717)


def fast17_servo(nv_id=1, **kw):
    """A servo standing in for fast17 (release 17, ABI 5: register 1144 and the ID log)."""
    return SimServo(nv_id, mode='app', program=RELEASE.packaged().raw, seed=nv_id, release_id=FAST17, **kw)


def write_frame(port, address):
    return [d for _, d in port.tx_log if len(d) > 9 and d[7] == 3 and struct.unpack_from('<H', d, 8)[0] == address]


# ------------------------------------------------------------------------------------------------ register encoding
def test_encode_set_id_layout():
    assert registers.SET_ID == (1144, 8)
    assert registers.encode_set_id(5) == b'SID1' + bytes([5, 0, 0, 0])
    assert registers.encode_set_id(0) == b'SID1' + bytes(4)
    assert registers.encode_set_id(0xFF) == b'SID1' + bytes([0xFF, 0, 0, 0])      # drop the override


@pytest.mark.parametrize('bad', [253, 254, 256, -1, True, 1.0, None, '5'])
def test_encode_set_id_refuses_bad_ids(bad):
    with pytest.raises(InvalidArgument):
        registers.encode_set_id(bad)


def test_decode_id_log():
    raw = bytes([7, 2]) + struct.pack('<HHH', 3, 252, 0)
    assert registers.decode_id_log(raw) == dict(log_id=7, source='log', used=3, free=252)
    raw = bytes([0xFF, 0]) + struct.pack('<HHH', 0, 255, 0)
    assert registers.decode_id_log(raw) == dict(log_id=None, source='nv', used=0, free=255)
    assert registers.decode_id_log(bytes([0xFF, 1]) + bytes(6))['source'] == 'devcfg'


# ------------------------------------------------------------------------------------------------ simulated firmware
def test_sim_set_id_replies_from_old_id_then_answers_new():
    s = fast17_servo(1)
    wire, port = make_bus([s])
    wire.write(1, 1144, registers.encode_set_id(5), timeout=0.1)      # the ACK comes from ID 1
    assert s.effective_id() == 5
    assert wire.ping(5)['firmware'] == 231
    with pytest.raises(NoReply):
        wire.ping(1)
    log = registers.decode_id_log(wire.read(5, *registers.SET_ID))
    assert log == dict(log_id=5, source='log', used=1, free=254)
    assert registers.decode_busdiag(wire.read(5, *registers.BUSDIAG))['id'] == 5


@pytest.mark.parametrize('data, error', [(b'SID1' + bytes([5, 0, 0]), 5), (b'SID2' + bytes([5, 0, 0, 0]), 4),
                                         (b'SID1' + bytes([5, 1, 0, 0]), 4), (b'SID1' + bytes([253, 0, 0, 0]), 4)])
def test_sim_set_id_errors(data, error):
    s = fast17_servo(1)
    wire, _ = make_bus([s])
    from openxl330.errors import Rejected
    with pytest.raises(Rejected) as e:
        wire.write(1, 1144, data)
    assert e.value.error == error and s.effective_id() == 1 and s.id_log == []


def test_sim_set_id_refused_while_armed_error_7():
    s = fast17_servo(1)
    bus = Bus(SimPort([s]), calibration=Calibration(), check_adapter=False)
    servo = bus.servo(1)
    servo.configure()
    servo.enable()
    from openxl330.errors import Rejected
    with pytest.raises(Rejected) as e:
        bus.wire.write(1, 1144, registers.encode_set_id(5))
    assert e.value.error == 7 and s.effective_id() == 1


def test_sim_id_log_survives_power_cycle_and_ff_drops_back_to_nv_id():
    s = fast17_servo(3)
    wire, _ = make_bus([s])
    wire.write(3, 1144, registers.encode_set_id(9))
    s.power_cycle()
    assert s.effective_id() == 9
    wire.write(9, 1144, registers.encode_set_id(0xFF))
    assert s.effective_id() == 3
    assert registers.decode_id_log(wire.read(3, *registers.SET_ID)) == dict(log_id=None, source='nv', used=2, free=253)


def test_sim_broadcast_and_sync_write_set_id_ignored():
    s = fast17_servo(1)
    wire, _ = make_bus([s])
    wire.write_broadcast(1144, registers.encode_set_id(5))
    wire.sync_write(1144, 8, {1: registers.encode_set_id(6)})
    assert s.effective_id() == 1 and s.id_log == []


def test_sim_older_release_has_no_id_log():
    s = SimServo(1, mode='app', program=make_fw231_program())            # an older image: no release 17
    wire, _ = make_bus([s])
    from openxl330.errors import Rejected
    with pytest.raises(Rejected):
        wire.read(1, *registers.SET_ID)
    with pytest.raises(Rejected):
        wire.write(1, 1144, registers.encode_set_id(5))
    assert s.effective_id() == 1


def test_sim_install_of_an_image_reaching_the_log_page_erases_it():
    """A logging image (39220 B) reaches into the log page 0x0800D800..0x0800DFFF; a performance image (38896 B) ends
    below it and keeps the log unless the loader erases the whole region (SimServo.loader_erases_region)."""
    s = fast17_servo(1)
    s.id_log = [4]
    s.flash_write(0x4000, images.Release('fw231-comp2-fast17-log').packaged().raw)
    assert s.id_log == []
    k = fast17_servo(4)
    k.id_log = [4]
    k.flash_write(0x4000, RELEASE.packaged().raw)                       # the performance image: below the page
    assert k.id_log == [4]
    assert s.id_log == []
    t = fast17_servo(2, loader_erases_region=True)
    t.id_log = [4]
    t.flash_write(0x4000, RELEASE.packaged().raw[:0x9000])
    assert t.id_log == []
    u = fast17_servo(3)
    u.id_log = [4]
    u.flash_write(0x4000, RELEASE.packaged().raw[:0x9000])
    assert u.id_log == [4]


# ------------------------------------------------------------------------------------------------ feature gate
def test_feature_set_id_needs_release_17():
    bus = Bus(SimPort([fast17_servo(1), SimServo(2, mode='app', program=make_fw231_program(), seed=2)]),
              calibration=Calibration(), check_adapter=False)
    assert 'set_id' in bus.servo(1).info.features
    assert 'set_id' not in bus.servo(2).info.features


# ------------------------------------------------------------------------------------------------ the recorded run
import json                                                    # noqa: E402

from openxl330 import install                                  # noqa: E402
from openxl330.errors import CalibrationError, InstallError, NotSupported, Refused  # noqa: E402,F401
from openxl330.wire import APP_BAUD                            # noqa: E402


def bus_of(*sims, calibration=None):
    return Bus(SimPort(list(sims)), calibration=calibration or Calibration(), check_adapter=False)


def history(records, receipt):
    return [json.loads(line) for line in (records / receipt['servo_dir'] / 'history.jsonl').read_text().splitlines()]


def test_set_id_happy_path_receipt_and_history(tmp_path):
    s = fast17_servo(1)
    wire, port = make_bus([s])
    r = install.set_id(wire, servo_id=1, new_id=5, records=tmp_path)
    assert r['completed'] and r['servo_id'] == 5 and r['previous_id'] == 1 and r['write_sent'] is True
    assert r['id_log_before']['source'] == 'nv' and r['id_log_after'] == dict(log_id=5, source='log', used=1, free=254)
    assert history(tmp_path, r)[-1]['id'] == 5 and history(tmp_path, r)[-1]['completed'] is True
    assert s.effective_id() == 5 and len(write_frame(port, 1144)) == 1


def test_set_id_refuses_taken_id_before_sending(tmp_path):
    a, b = fast17_servo(1), fast17_servo(5)
    wire, port = make_bus([a, b])
    with pytest.raises(Refused) as e:
        install.set_id(wire, servo_id=1, new_id=5, records=tmp_path)
    assert 'ID 5' in str(e.value) and 'already' in str(e.value)
    assert write_frame(port, 1144) == [] and a.effective_id() == 1


def test_set_id_refuses_enabled_servo_before_sending(tmp_path):
    s = fast17_servo(1)
    bus = bus_of(s)
    bus.servo(1).configure()
    bus.servo(1).enable()
    with pytest.raises(Refused) as e:
        install.set_id(bus.wire, servo_id=1, new_id=5, records=tmp_path)
    assert 'disable()' in str(e.value) and write_frame(bus.port, 1144) == [] and s.enabled


@pytest.mark.parametrize('new_id', [1, 253, -1, True])
def test_set_id_refuses_same_id_and_bad_ids(tmp_path, new_id):
    wire, port = make_bus([fast17_servo(1)])
    with pytest.raises(InvalidArgument):
        install.set_id(wire, servo_id=1, new_id=new_id, records=tmp_path)
    assert port.tx_log == []


def test_set_id_refuses_older_release(tmp_path):
    s = SimServo(1, mode='app', program=make_fw231_program())
    wire, port = make_bus([s])
    with pytest.raises(NotSupported) as e:
        install.set_id(wire, servo_id=1, new_id=5, records=tmp_path)
    assert 'release 17' in str(e.value) and 'openxl330 update' in str(e.value)
    assert write_frame(port, 1144) == []


def test_set_id_no_reply_after_write_records_write_sent(tmp_path):
    s = fast17_servo(1)
    real_status = s.status

    def lose_the_1144_reply(*a, **kw):
        frame = real_status(*a, **kw)
        return b'' if s.id_log else frame                     # the write took effect, its reply is lost
    s.status = lose_the_1144_reply
    wire, _ = make_bus([s])
    with pytest.raises(InstallError) as e:
        install.set_id(wire, servo_id=1, new_id=5, records=tmp_path)
    receipt = json.loads((tmp_path / '_runs').glob('*/receipt.json').__next__().read_text())
    assert receipt['write_sent'] is True and receipt['completed'] is False
    assert 'ID 1' in str(e.value) and 'ID 5' in str(e.value) and 'openxl330 scan' in str(e.value)


def test_servo_set_id_moves_calibration_and_invalidates_old_handle(tmp_path):
    cal = Calibration(tmp_path / 'calibration.json')
    cal.set(1, zero_counts=1500, direction=-1)
    cal.save()
    bus = bus_of(fast17_servo(1), calibration=cal)
    old = bus.servo(1)
    new = old.set_id(5, records=tmp_path / 'records')
    assert new.id == 5 and new.info.id == 5
    assert (cal.joint(5).zero_counts, cal.joint(5).direction) == (1500, -1) and cal.joint(1).zero_counts == 2048
    assert Calibration.load(tmp_path / 'calibration.json').joint(5).zero_counts == 1500
    with pytest.raises(Refused) as e:
        old.position
    assert 'ID 5' in str(e.value)
    new.position


def test_set_id_refuses_when_new_id_has_calibration(tmp_path):
    cal = Calibration()
    cal.set(5, zero_counts=100)
    bus = bus_of(fast17_servo(1), calibration=cal)
    with pytest.raises(Refused) as e:
        bus.servo(1).set_id(5, records=tmp_path)
    assert 'calibrate --id 5 --reset' in str(e.value) and write_frame(bus.port, 1144) == []


def test_group_holding_old_id_refuses_after_set_id(tmp_path):
    bus = bus_of(fast17_servo(1), fast17_servo(2))
    group = bus.group([1, 2])
    bus.servo(1).set_id(7, records=tmp_path)
    with pytest.raises(Refused) as e:
        group.read()
    assert 'ID 7' in str(e.value)
    bus.group([7, 2]).read()


def test_a_new_servo_at_the_old_id_is_usable(tmp_path):
    first, second = fast17_servo(1), fast17_servo(9)
    bus = bus_of(first, second)
    bus.servo(1).set_id(5, records=tmp_path)
    bus.servo(9).set_id(1, records=tmp_path)
    assert bus.servo(1).position is not None and second.effective_id() == 1


# ------------------------------------------------------------------------------------------------ CLI
def test_cli_set_id_prints_the_receipt_and_moves_the_id(capsys, monkeypatch, tmp_path):
    from openxl330.cli import main
    from test_cli import use_sim
    s = fast17_servo(2)
    use_sim(monkeypatch, s)
    assert main(['--sim', '1', 'set-id', '--id', '2', '--new-id', '6', '--json']) == 0
    out, err = capsys.readouterr()
    receipt = json.loads(out)
    assert receipt['completed'] and receipt['servo_id'] == 6 and s.effective_id() == 6


def test_cli_set_id_needs_yes_off_a_terminal(capsys, monkeypatch):
    import io
    import sys
    from openxl330.cli import main
    from test_cli import _stock_on_port
    s = fast17_servo(2)
    opened = _stock_on_port(monkeypatch, s)
    monkeypatch.setattr(sys, 'stdin', io.StringIO('y\n'))
    assert main(['set-id', '--port', '/dev/ttyUSB9', '--id', '2', '--new-id', '6']) == 1
    out, err = capsys.readouterr()
    assert '--yes' in err and 'Nothing was sent' in err and opened == [] and s.effective_id() == 2


# ------------------------------------------------------------------------------------------------ update keeps the ID
def fast17_release():
    """The bundled program standing in for a fast17 release (the sim servo keeps reporting FAST17 after the upload)."""
    release = images.Release(images.DEFAULT_RELEASE)
    release.release_id = dict(FAST17)
    release.set_id = True                                      # its manifest says it has the ID log
    return release


def logged_servo(nv_id=1, logged=5, **kw):
    s = fast17_servo(nv_id, **kw)
    s.id_log = [logged]
    return s


def test_update_refuses_to_lose_a_logged_id_to_an_older_release(tmp_path):
    s = logged_servo()
    wire, port = make_bus([s])
    with pytest.raises(InstallError) as e:                     # a logging image has no ID log
        install.update(wire, servo_id=5, release='fw231-comp2-fast17-log', records=tmp_path)
    assert 'set-id' in str(e.value) and 'ID 1' in str(e.value) and 'allow_id_change' in str(e.value)
    assert [d for _, d in port.tx_log if len(d) > 7 and d[7] == 0xF9] == [] and s.effective_id() == 5


def test_update_reapplies_an_erased_logged_id(tmp_path):
    s = logged_servo(loader_erases_region=True)
    wire, port = make_bus([s])
    r = install.update(wire, servo_id=5, release=fast17_release(), records=tmp_path)
    assert r['completed'] and r['servo_id'] == 5 and s.effective_id() == 5
    assert [st['step'] for st in r['steps']].count('id_reapplied') == 1 and len(write_frame(port, 1144)) == 1


def test_update_keeps_a_surviving_logged_id_without_a_write(tmp_path):
    s = logged_servo()
    real = s.flash_write

    def keep_the_log(address, data):                           # a loader that erases only the pages it programs,
        real(address, data)                                     # with an image ending below the log page
        s.id_log = [5]
    s.flash_write = keep_the_log
    wire, port = make_bus([s])
    r = install.update(wire, servo_id=5, release=fast17_release(), records=tmp_path)
    assert r['completed'] and r['servo_id'] == 5 and write_frame(port, 1144) == []
    assert 'id_reapplied' not in [st['step'] for st in r['steps']]


def test_update_of_a_logged_id_needs_its_fallback_id_free(tmp_path):
    s, other = logged_servo(), fast17_servo(1)
    other.id_log = []
    wire, port = make_bus([s, other])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=5, release=fast17_release(), records=tmp_path)
    assert 'ID 1' in str(e.value) and [d for _, d in port.tx_log if len(d) > 7 and d[7] == 0xF9] == []


# ------------------------------------------------------------------------------------------------ final review fixes
def read_frames(port, address):
    return [d for _, d in port.tx_log if len(d) > 9 and d[7] == 2 and struct.unpack_from('<H', d, 8)[0] == address]


def test_update_of_a_release_16_servo_never_reads_1144(tmp_path):
    """I2: update traffic to an older servo (ABI below 5) never reads 1144."""
    s = SimServo(1, mode='app', program=make_fw231_program())
    wire, port = make_bus([s])
    assert install.update(wire, servo_id=1, records=tmp_path)['completed']
    assert read_frames(port, 1144) == []


def test_set_id_refuses_an_id_taken_at_another_bus_baud(tmp_path):
    """I4: FW231 servos boot at 1 Mbaud, so an ID held at 3 Mbaud collides after the next power cycle."""
    a, b = fast17_servo(1), fast17_servo(5)
    b.baud = 3_000_000
    wire, port = make_bus([a, b])
    with pytest.raises(Refused) as e:
        install.set_id(wire, servo_id=1, new_id=5, records=tmp_path)
    assert 'ID 5' in str(e.value) and '3 Mbaud' in str(e.value)
    assert write_frame(port, 1144) == [] and a.effective_id() == 1 and wire.port.baudrate == 1_000_000


def garble_after_set_id(s):
    """The 1144 write takes effect, and its reply arrives with a corrupted CRC (line noise)."""
    real_status = s.status

    def garbled(*a, **kw):
        frame = real_status(*a, **kw)
        return frame[:-1] + bytes([frame[-1] ^ 0xFF]) if s.id_log else frame
    s.status = garbled


def lose_after_set_id(s):
    real_status = s.status
    s.status = lambda *a, **kw: b'' if s.id_log else real_status(*a, **kw)


def test_set_id_garbled_reply_names_both_ids_and_the_scan(tmp_path):
    """I1: a corrupted reply (BusUncertain) gets the same advice as a lost one."""
    s = fast17_servo(1)
    garble_after_set_id(s)
    wire, _ = make_bus([s])
    with pytest.raises(InstallError) as e:
        install.set_id(wire, servo_id=1, new_id=5, records=tmp_path)
    text = str(e.value)
    assert 'ID 1' in text and 'ID 5' in text and 'openxl330 scan' in text and 'reset()' in text
    receipt = json.loads(next((tmp_path / '_runs').glob('*/receipt.json')).read_text())
    assert receipt['write_sent'] is True and receipt['recovery'].startswith('openxl330 scan')


@pytest.mark.parametrize('fault', [lose_after_set_id, garble_after_set_id])
def test_servo_set_id_failure_after_the_write_retires_handles_and_names_the_calibration(tmp_path, fault):
    cal = Calibration()
    cal.set(1, zero_counts=1500)
    s = fast17_servo(1)
    bus = bus_of(s, calibration=cal)
    old = bus.servo(1)
    fault(s)
    with pytest.raises(InstallError) as e:
        old.set_id(5, records=tmp_path)
    assert 'calibration' in str(e.value) and 'set-id --id 1 --new-id 5' in str(e.value)
    assert cal.joint(1).zero_counts == 1500 and not cal.has(5)
    with pytest.raises(Refused):
        old.position


def test_cli_sim_rerun_without_proof_leaves_the_calibration(capsys, monkeypatch):
    """P2-4: without the failed run's receipt (a --sim run keeps no records) a servo answering at the new ID is not
    proven to be the one that moved: nothing is moved, and the old ID's silence is reported."""
    from openxl330.cli import main
    from test_cli import use_sim
    s = fast17_servo(1)
    s.id_log = [5]                                             # something answers at 5 with 5 logged
    cal = Calibration()
    cal.set(1, zero_counts=1500, direction=-1)
    use_sim(monkeypatch, s, calibration=cal)
    assert main(['--sim', '1', 'set-id', '--id', '1', '--new-id', '5']) == 1
    assert 'no servo answers at ID 1' in capsys.readouterr().err
    assert cal.has(1) and not cal.has(5)


def test_cli_set_id_rerun_refuses_when_neither_id_answers_as_changed(capsys, monkeypatch):
    from openxl330.cli import main
    from test_cli import use_sim
    use_sim(monkeypatch, fast17_servo(9))
    assert main(['--sim', '1', 'set-id', '--id', '1', '--new-id', '5']) == 1
    assert 'no servo answers at ID 1' in capsys.readouterr().err


def interrupted_update(s, tmp_path, release, keep_log, allow_id_change=False):
    """update() of logged servo `s` (ID 5, NV 1) fails its FA check and leaves it in the loader; the loader then
    keeps (keep_log) or erases the ID log page on the recover's upload."""
    from openxl330.wire import LOADER_BAUD
    real = s.flash_write

    def write(address, data, corrupt=True):
        data = bytearray(data)
        if corrupt:
            data[1000] ^= 0xFF
        logged = list(s.id_log)
        real(address, bytes(data))
        if keep_log:
            s.id_log = logged
    s.flash_write = write
    wire, _ = make_bus([s])
    with pytest.raises(InstallError):
        install.update(wire, servo_id=5, release=release, records=tmp_path, allow_id_change=allow_id_change)
    s.flash_write = lambda address, data: write(address, data, corrupt=False)
    wire, port = make_bus([s], LOADER_BAUD)
    return wire, port


@pytest.mark.parametrize('keep_log', [False, True])
def test_recover_after_an_interrupted_update_keeps_the_logged_id(tmp_path, keep_log):
    """I3: the recover command names the NV ID; the servo must end at its set-id ID either way the loader behaves."""
    s = logged_servo(loader_erases_region=not keep_log)
    wire, port = interrupted_update(s, tmp_path, fast17_release(), keep_log)
    r = install.recover(wire, servo_id=1, release=fast17_release(), records=tmp_path)
    assert r['servo_id'] == 5 and s.effective_id() == 5
    steps = [st['step'] for st in r['steps']]
    assert ('id_reapplied' in steps) is (not keep_log)
    assert len(write_frame(port, 1144)) == (0 if keep_log else 1)


def test_recover_to_a_release_without_an_id_log_says_the_logged_id_is_lost(tmp_path):
    s = logged_servo()
    wire, _ = interrupted_update(s, tmp_path, images.Release('fw231-comp2-fast17-log'), keep_log=False,
                                 allow_id_change=True)
    s.release_override = None                                 # what boots now is the logging image itself (no log)
    r = install.recover(wire, servo_id=1, records=tmp_path)
    assert r['servo_id'] == 1 and s.effective_id() == 1 and r['logged_id_lost'] == 5
    assert any('ID 5' in w and 'set-id' in w for w in r['warnings'])


def timed(port):
    """Record (clock time, frame) for every host write on this SimPort."""
    log, real = [], port.write

    def write(data):
        log.append((port.clock(), bytes(data)))
        return real(data)
    port.write = write
    return log


def quiet_after_1144(log):
    """Seconds between each 1144 write and the host's next frame."""
    gaps = []
    for k, (t, d) in enumerate(log):
        if len(d) > 9 and d[7] == 3 and struct.unpack_from('<H', d, 8)[0] == 1144 and k + 1 < len(log):
            gaps.append(log[k + 1][0] - t)
    return gaps


def test_the_bus_stays_quiet_for_100_ms_after_a_1144_write(tmp_path):
    """The first 1144 after an install erases the log page (20-40 ms CPU stall); traffic meanwhile can overrun the
    servo's RX ring. The firmware session asks for 100 ms of quiet after every 1144 write."""
    wire, port = make_bus([fast17_servo(1)])
    first = timed(port)
    install.set_id(wire, servo_id=1, new_id=5, records=tmp_path)
    s = logged_servo(loader_erases_region=True)
    wire2, port2 = make_bus([s])
    second = timed(port2)
    install.update(wire2, servo_id=5, release=fast17_release(), records=tmp_path)
    for gaps in (quiet_after_1144(first), quiet_after_1144(second)):
        assert gaps and min(gaps) >= 0.1


# ------------------------------------------------------------------------------------------------ gate P2s
def test_set_id_on_a_full_log_compacts_it(tmp_path):
    """P2-1: the firmware erases a full page and writes the new record; the write always succeeds."""
    s = fast17_servo(1)
    s.id_log = [7, 0xFF] * 127 + [0xFF]                        # 255 records, none active: answers at its NV ID 1
    wire, _ = make_bus([s])
    r = install.set_id(wire, servo_id=1, new_id=5, records=tmp_path)
    assert r['completed'] and r['id_log_before']['free'] == 0
    assert r['id_log_after'] == dict(log_id=5, source='log', used=1, free=254)


def test_set_id_error_7_rereads_the_log_and_warns_when_it_was_erased(tmp_path):
    """P2-2: a 7 on the erase/format path can leave the log empty: the servo then answers at its stock ID after the
    next reset."""
    s = logged_servo(nv_id=1, logged=5)
    real = s.write

    def write(a, data, broadcast=False):
        if a == 1144 and not broadcast:
            s.id_log = []                                         # erased, then the program step failed
            s.pending_effective = 5
            return 7
        return real(a, data, broadcast)
    s.write = write
    real_effective = s.effective_id
    s.effective_id = lambda: getattr(s, 'pending_effective', None) or real_effective()   # until the next reset
    wire, _ = make_bus([s])
    with pytest.raises(InstallError) as e:
        install.set_id(wire, servo_id=5, new_id=6, records=tmp_path)
    receipt = json.loads(next((tmp_path / '_runs').glob('*/receipt.json')).read_text())
    assert receipt['id_log_after_error']['source'] == 'nv'
    assert 'ID 1' in str(e.value) and 'power cycle' in str(e.value)
    assert any('ID 1' in w for w in receipt['warnings'])


def test_update_of_a_logged_id_on_a_shared_bus_says_to_update_it_alone(tmp_path):
    """P2-3: the servo answers at its stock ID during the update, so that ID must be free."""
    s, other = logged_servo(), fast17_servo(1)
    wire, _ = make_bus([s, other])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=5, release=fast17_release(), records=tmp_path)
    assert 'alone' in str(e.value) and 'disconnect ID 1' in str(e.value)


def test_finished_set_id_needs_the_failed_run_and_the_same_servo(tmp_path):
    """P2-4: a re-run moves the calibration only onto the servo the failed run changed (same calibration
    fingerprint), proven by that run's receipt."""
    s = fast17_servo(1)
    lose_after_set_id(s)
    wire, _ = make_bus([s])
    with pytest.raises(InstallError):
        install.set_id(wire, servo_id=1, new_id=5, records=tmp_path)
    s.status = SimServo.status.__get__(s)                     # replies work again
    found = install.finished_set_id(wire, servo_id=1, new_id=5, records=tmp_path)
    assert found is not None and found['write_sent'] and not found['completed']
    stranger = fast17_servo(9)                                 # another servo now answering at 5, its own calibration
    stranger.id_log = [5]
    wire2, _ = make_bus([stranger])
    assert install.finished_set_id(wire2, servo_id=1, new_id=5, records=tmp_path) is None
    assert install.finished_set_id(wire, servo_id=1, new_id=5, records=tmp_path / 'elsewhere') is None


def test_cli_rerun_after_a_lost_reply_finishes_the_calibration_move(capsys, monkeypatch):
    """P2-4 end to end on a named port (the real records and calibration file)."""
    from openxl330 import paths
    from openxl330.cli import main
    from test_cli import _stock_on_port
    cal = Calibration(paths.home() / 'calibration.json')
    cal.set(1, zero_counts=1500, direction=-1)
    cal.save()
    s = fast17_servo(1)
    lose_after_set_id(s)
    _stock_on_port(monkeypatch, s)
    assert main(['set-id', '--port', '/dev/ttyUSB9', '--id', '1', '--new-id', '5', '--yes']) == 1
    s.status = SimServo.status.__get__(s)
    assert main(['set-id', '--port', '/dev/ttyUSB9', '--id', '1', '--new-id', '5', '--yes']) == 0
    out, err = capsys.readouterr()
    assert 'already' in out
    moved = Calibration.load(paths.home() / 'calibration.json')
    assert (moved.joint(5).zero_counts, moved.joint(5).direction) == (1500, -1) and not moved.has(1)


def test_sim_release_17_with_abi_4_has_no_id_log():
    """The fast17-size build (release 17, ABI 4) has no 1144."""
    s = SimServo(1, mode='app', program=RELEASE.packaged().raw, release_id=dict(FAST17, abi=4))
    wire, _ = make_bus([s])
    from openxl330.errors import Rejected
    with pytest.raises(Rejected):
        wire.read(1, *registers.SET_ID)


# ------------------------------------------------------------------------------------------------ logging images
FAST17_LOG = dict(number=17, variant='comp-log', abi=5, source_id=0x80d4d494)


def test_sim_logging_image_has_no_id_log():
    """fast17 logging builds report ABI 5 but have no ID log: read 1144 answers 4, write 7."""
    from openxl330.errors import Rejected
    s = SimServo(1, mode='app', program=RELEASE.packaged().raw, release_id=FAST17_LOG)
    wire, _ = make_bus([s])
    with pytest.raises(Rejected) as e:
        wire.read(1, *registers.SET_ID)
    assert e.value.error == 4
    with pytest.raises(Rejected) as e:
        wire.write(1, 1144, registers.encode_set_id(5))
    assert e.value.error == 7 and s.effective_id() == 1


def test_logging_image_has_no_set_id_feature_and_set_id_is_not_supported(tmp_path):
    s = SimServo(1, mode='app', program=RELEASE.packaged().raw, release_id=FAST17_LOG)
    bus = bus_of(s)
    assert 'set_id' not in bus.servo(1).info.features
    with pytest.raises(NotSupported):
        install.set_id(bus.wire, servo_id=1, new_id=5, records=tmp_path)
    assert write_frame(bus.port, 1144) == []


def test_update_to_a_release_without_an_id_log_follows_its_manifest(tmp_path):
    """The target release's manifest says whether it has the ID log (set_id), not its ABI."""
    release = fast17_release()
    release.set_id = False                                     # a logging build: ABI 5, no ID log
    s = logged_servo()
    wire, port = make_bus([s])
    with pytest.raises(InstallError) as e:
        install.update(wire, servo_id=5, release=release, records=tmp_path)
    assert 'set-id' in str(e.value) and 'allow_id_change' in str(e.value)


def test_bundled_release_manifest_fields_default():
    assert (RELEASE.set_id, RELEASE.kind, RELEASE.beta) == (True, 'performance', False)
