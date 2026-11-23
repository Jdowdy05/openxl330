"""Wire client and errors against the simulated bus (no port is opened)."""
from __future__ import annotations

import json
import struct
from pathlib import Path
from types import SimpleNamespace

import pytest
import serial

from openxl330 import errors, paths, registers, sim
from openxl330._protocol import framing
from openxl330.errors import (PROTOCOL_ERRORS, AdapterEchoes, AdapterError, AdapterWarning, BusUncertain,
                              CalibrationError, Faulted, InstallError, InvalidArgument, NoReply, NoServosFound,
                              NotEnabled, NotSupported, PortBusy, Refused, Rejected, UnsupportedServo, UntestedServo)
from openxl330.sim import FakeClock, SimPort, SimServo, default_eeprom
from openxl330.wire import APP_BAUD, BROADCAST, LOADER_BAUD, Loader, Wire


# ---------------------------------------------------------------------------------------------- errors
def test_error_classes_and_protocol_errors():
    assert PROTOCOL_ERRORS == {1: "result fail", 2: "instruction error", 3: "CRC error", 4: "data range error",
                               5: "data length error", 6: "data limit error",
                               7: "access error (busy, OFF-only, or not allowed now)"}
    for cls in (InvalidArgument, Refused, BusUncertain, NoReply, Rejected, NotSupported, NotEnabled, Faulted,
                InstallError, UntestedServo, UnsupportedServo, NoServosFound, CalibrationError, AdapterError, PortBusy,
                AdapterEchoes):
        assert issubclass(cls, errors.Error), cls
    assert issubclass(errors.Error, Exception) and issubclass(InvalidArgument, ValueError)
    assert issubclass(NoReply, TimeoutError) and issubclass(UntestedServo, InstallError)
    assert issubclass(PortBusy, AdapterError) and issubclass(AdapterEchoes, AdapterError)
    assert issubclass(AdapterWarning, UserWarning) and not issubclass(AdapterWarning, errors.Error)
    e = NoReply("no reply from ID(s) [3]", {2: {"id": 2}}, [3])
    assert (e.partial, e.missing, str(e)) == ({2: {"id": 2}}, [3], "no reply from ID(s) [3]")
    assert (NoReply("x").partial, NoReply("x").missing) == ({}, [])
    e = InstallError("upload interrupted", step="uploading", recovery="openxl330 recover --port P --id 4")
    assert (e.step, e.recovery) == ("uploading", "openxl330 recover --port P --id 4")
    assert "upload interrupted" in str(e) and "openxl330 recover --port P --id 4" in str(e)
    assert InstallError("refused before F9", step="census").recovery is None
    assert "unknown" in Rejected(3, 9).meaning


def test_rejected_has_meaning(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, _ = sim_bus([s])
    s.enabled = 1
    with pytest.raises(Rejected) as e:
        wire.write(1, registers.PD_CONFIG, bytes(48))          # an OFF-only write while armed
    assert (e.value.servo_id, e.value.error) == (1, 7)
    assert "access" in e.value.meaning
    assert e.value.meaning in str(e.value) and "ID 1" in str(e.value)
    assert not wire.uncertain


def test_wire_constants():
    assert (APP_BAUD, LOADER_BAUD, BROADCAST) == (1_000_000, 57_600, 0xFE)


# ---------------------------------------------------------------------------------------------- log
def test_wire_log_only_when_asked(tmp_path, monkeypatch, fw231_servos):
    monkeypatch.setenv("OPENXL330_HOME", str(tmp_path))
    s = fw231_servos(1)[0]
    clock = FakeClock()
    port = SimPort([s], clock)
    wire = Wire(port, clock=clock, sleep=clock.sleep)          # normal use: no log, no files
    assert wire.log is None
    wire.ping(1)
    assert list(tmp_path.iterdir()) == []
    wire = Wire(port, log=None, clock=clock, sleep=clock.sleep)  # None: no log either
    assert wire.log is None
    wire.ping(1)
    assert list(tmp_path.iterdir()) == []
    with pytest.raises(InvalidArgument):
        Wire(port, log="wire.jsonl")
    wire = Wire(port, log=True, clock=clock, sleep=clock.sleep)  # True: the default wire log file (installer)
    wire.ping(1)
    path = wire.log.path
    assert path.is_file() and path.parent == tmp_path / "logs"
    events = [json.loads(line)["event"] for line in path.read_text().splitlines()]
    assert events[:2] == ["tx", "rx"]
    wire.log.close()


def test_every_test_runs_with_its_own_home():
    """The autouse isolate_home fixture (conftest): no test writes into the user's real ~/.openxl330."""
    home = paths.home()
    assert home != Path.home() / ".openxl330" and home.is_dir() and not any(home.iterdir())


def test_home_defaults_to_dot_openxl330(monkeypatch, tmp_path):
    monkeypatch.delenv("OPENXL330_HOME", raising=False)
    assert paths.home() == Path.home() / ".openxl330"
    monkeypatch.setenv("OPENXL330_HOME", str(tmp_path / "x"))
    assert paths.home() == tmp_path / "x"


# ---------------------------------------------------------------------------------------------- latching
def test_duplicate_ids_latch_uncertain(sim_bus, fw231_servos, fw231_program):
    a = fw231_servos(1)[0]
    b = SimServo(1, mode="app", program=fw231_program(), seed=8)    # default device block: its NV ID 1
    wire, _ = sim_bus([a, b])
    with pytest.raises(BusUncertain):
        wire.scan(10)
    with pytest.raises(BusUncertain):
        wire.ping(1)


def test_redact_latch_replaces_the_reason_while_latched(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    with pytest.raises(Refused) as e:
        wire.redact_latch("bytes not shown")                       # not latched: nothing to replace
    assert "not latched" in str(e.value) and not wire.uncertain
    real = s.on_frame
    s.on_frame = lambda f, clock: (real(f, clock)[0], b"\x12\x34\x56\x78" * 4)   # a reply that is no status packet
    with pytest.raises(BusUncertain) as e:
        wire.ping(1)                                               # latched, quoting the bytes
    assert "12345678" in str(e.value)
    s.on_frame = real
    for bad in ("", None, 5):
        with pytest.raises(InvalidArgument):
            wire.redact_latch(bad)
    wire.redact_latch("a reply was garbled (bytes not shown)")
    assert wire.uncertain
    sent = len(port.tx_log)
    with pytest.raises(BusUncertain) as e:
        wire.ping(1)
    assert "a reply was garbled (bytes not shown)" in str(e.value) and "12345678" not in str(e.value)
    assert len(port.tx_log) == sent, "still latched: nothing sent"
    wire.reset()
    assert wire.ping(1)["firmware"] == 231


def scripted_port(reply):
    """A port that answers every write with `reply` (and records what was written)."""
    clock = FakeClock()
    port = SimPort([], clock)
    port.sent = []

    def write(data):
        port.sent.append(bytes(data))
        port.rx += reply
        return len(data)
    port.write = write
    return Wire(port, clock=clock, sleep=clock.sleep), port


def test_scan_latches_on_two_clean_replies_from_one_id():
    one = framing.frame(0, 1, 0x55, b"\x00" + struct.pack("<HB", 1200, 231))
    wire, _ = scripted_port(one + one)
    with pytest.raises(BusUncertain) as e:
        wire.scan(10)
    assert "duplicate IDs" in str(e.value) and "own ID" in str(e.value)


def test_scan_refuses_reserved_reply_id():
    reply = framing.frame(0, 253, 0x55, b"\x00" + struct.pack("<HB", 1200, 231))
    wire, _ = scripted_port(reply)
    with pytest.raises(BusUncertain, match="invalid servo ID 253"):
        wire.scan()


def test_status_rejects_unstuffed_payload_even_with_a_valid_crc():
    payload = b"\x55\x00\xff\xff\xfd"
    status = b"\xff\xff\xfd\x00\x01" + struct.pack("<H", len(payload) + 2) + payload
    wire, _ = scripted_port(status + struct.pack("<H", framing.crc16(status)))
    with pytest.raises(BusUncertain, match="non-canonical byte stuffing"):
        wire.read(1, 0, 3)


def test_standard_sync_read_validates_each_reply_checksum_once(monkeypatch):
    ids, data = range(1, 7), b"\xff\xff\xfd\xfd" * 17
    replies = b"".join(framing.frame(0, i, 0x55, b"\0" + data) for i in ids)
    wire, port = scripted_port(replies)
    hashed_bytes = []
    crc16 = framing.crc16

    def count_crc(data, initial=0):
        hashed_bytes.append(len(data))
        return crc16(data, initial)

    monkeypatch.setattr(framing, "crc16", count_crc)
    assert wire.sync_read(ids, 64, len(data)) == dict.fromkeys(ids, data)
    assert sum(hashed_bytes) == len(replies) + len(port.sent[0]) - 2 * (len(ids) + 1)


def test_partial_write_latches(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    port.write = lambda data: len(data) - 1
    with pytest.raises(BusUncertain) as e:
        wire.ping(1)
    assert "partial serial write" in str(e.value)
    assert wire.uncertain


def failing_log(on, error):
    """A wire log callable that raises `error` on events named `on` (None: on every event)."""
    def log(event):
        if on is None or event["event"] == on:
            raise error
    return log


def test_failing_log_after_tx_latches(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    wire.log = failing_log("tx", OSError(28, "No space left on device"))
    with pytest.raises(BusUncertain) as e:
        wire.ping(1)                                               # the request is on the wire, its reply unread
    assert "wire log failed" in str(e.value) and e.value.__suppress_context__
    assert wire.uncertain
    sent = len(port.tx_log)
    with pytest.raises(BusUncertain) as again:
        wire.ping(1)
    assert len(port.tx_log) == sent, "nothing is written once latched"
    assert "wire log failed" in str(again.value)


def test_interrupted_log_after_tx_latches_and_propagates(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    wire.log = failing_log("tx", KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        wire.ping(1)
    assert wire.uncertain
    sent = len(port.tx_log)
    with pytest.raises(BusUncertain):
        wire.ping(1)
    assert len(port.tx_log) == sent


def test_failing_log_on_unsolicited_bytes_still_latches(sim_bus, fw231_servos):
    """_ready reads the junk and logs it: a log that raises on whatever it logs there cannot leave the junk
    consumed and the Wire unlatched."""
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    wire.log = failing_log(None, OSError(28, "No space left on device"))
    port.rx += b"\x55\x55"
    sent = len(port.tx_log)
    with pytest.raises(BusUncertain) as e:
        wire.ping(1)
    assert "unsolicited bytes" in str(e.value)
    assert wire.uncertain and len(port.tx_log) == sent and port.rx == b""
    with pytest.raises(BusUncertain):
        wire.ping(1)
    assert len(port.tx_log) == sent


def test_bad_arguments_raise_invalid_argument_before_any_tx(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    for call in (lambda: wire.ping(300), lambda: wire.ping(BROADCAST), lambda: wire.ping("1"),
                 lambda: wire.read(1, 70000, 4), lambda: wire.read(1, 0, -1),
                 lambda: wire.write(BROADCAST, 64, b"\0"), lambda: wire.write(1, 64, 1),
                 lambda: wire.write_broadcast(64, 1), lambda: wire.sync_read([1, 1], 0, 4),
                 lambda: wire.sync_read([1, 300], 0, 4), lambda: wire.sync_read([], 0, 4),
                 lambda: wire.bulk_read([]), lambda: wire.sync_write(256, 1, {}),
                 lambda: wire.fast_sync_read([], 0, 4),
                 lambda: wire.fast_sync_read([1, 1], 0, 4), lambda: wire.fast_sync_read([1], 0, 129),
                 lambda: wire.fast_sync_read([1], 0, 0), lambda: wire.sync_write(256, 2, {1: b"\0"}),
                 lambda: wire.sync_write(256, 1, {999: b"\0"}), lambda: wire.bulk_read([(1, 0, 4), (1, 64, 4)]),
                 lambda: wire.bulk_read([(1, 0)]), lambda: wire.flash_read(1, 0x0800FFF0, 32),
                 lambda: wire.flash_read(1, 0x07FFFFFF, 4), lambda: wire.reboot(-1), lambda: wire.enter_loader(256)):
        with pytest.raises(InvalidArgument):
            call()
    assert port.tx_log == [] and not wire.uncertain
    assert wire.ping(1)["firmware"] == 231


@pytest.mark.parametrize("timeout", [0, -0.01, float("nan"), float("inf"), "0.1"])
def test_invalid_timeouts_prevent_transmission(timeout, sim_bus, fw231_servos):
    wire, port = sim_bus(fw231_servos(1))
    for call in (lambda: wire.ping(1, timeout), lambda: wire.read(1, 0, 4, timeout),
                 lambda: wire.write(1, 65, b"\1", timeout), lambda: wire.sync_read([1], 0, 4, timeout),
                 lambda: wire.fast_sync_read([1], 0, 4, timeout), lambda: wire.bulk_read([(1, 0, 4)], timeout),
                 lambda: wire.reboot(1, timeout)):
        with pytest.raises(InvalidArgument, match="timeout"):
            call()
    assert not port.tx_log and not wire.uncertain


def test_scan_checks_window_arguments_before_transmission(sim_bus, fw231_servos):
    wire, port = sim_bus(fw231_servos(1))
    for call in (lambda: wire.scan(-1), lambda: wire.scan(253), lambda: wire.scan(settle=float("nan"))):
        with pytest.raises(InvalidArgument):
            call()
    assert not port.tx_log and not wire.uncertain


def test_upload_that_would_not_fit_is_refused_before_any_tx(sim_bus, upload_of):
    s = SimServo(1, seed=7)
    s.reset("loader")
    wire, port = sim_bus([s], 57_600)
    ld = Loader(wire)
    ld.handshake()
    sent = len(port.tx_log)
    phase = wire.loader_phase
    with pytest.raises(InvalidArgument) as e:
        ld.upload(upload_of(bytes(0xE000 - 0x4000 + 8)))           # would reach 0x0800E000 (the servo's own NVM)
    assert "0x0800DFFF" in str(e.value)
    assert (len(port.tx_log), ld.started, wire.loader_phase) == (sent, False, phase)


def test_upload_packets_must_match_the_upload_bytes(sim_bus, upload_of):
    """upload() sends only packets built from the upload's own bytes, so the loader's CRC check covers exactly what it
    acknowledged: an upload whose packets, progress or CRC disagree with its bytes is refused before any TX."""
    s = SimServo(1, seed=7)
    s.reset("loader")
    wire, port = sim_bus([s], 57_600)
    ld = Loader(wire)
    ld.handshake()
    good = upload_of(bytes(range(256)) * 4 + b"\1")
    sent, phase = len(port.tx_log), wire.loader_phase
    for bad in (dict(packets=good.packets[:1] + good.packets[2:]), dict(progress=good.progress[:-1] + ((9, 9),)),
                dict(crc16=good.crc16 ^ 1), dict(packets=upload_of(b"\2" * 1025).packets)):
        with pytest.raises(InvalidArgument) as e:
            ld.upload(SimpleNamespace(**dict(vars(good), **bad)))
        assert "bundled release" in str(e.value)
    with pytest.raises(InvalidArgument):                                 # 128-byte multiple: no F7 packets for it
        ld.upload(SimpleNamespace(encoded=b"\0" * 256, packets=(), progress=(), crc16=0))
    assert (len(port.tx_log), ld.started, wire.loader_phase) == (sent, False, phase)
    assert ld.upload(good) == good.crc16 and not wire.uncertain


def test_loader_crc_check_is_private_to_upload(sim_bus, upload_of):
    """The loader's FA CRC check is not public API: only upload() runs it, once, over exactly the upload it has just
    sent. Any other use (before an upload, another length, a second time) is Refused before anything is sent."""
    assert not hasattr(Loader, "fa")
    s = SimServo(1, seed=7)
    s.reset("loader")
    wire, port = sim_bus([s], 57_600)
    ld = Loader(wire)
    ld.handshake()
    pack = upload_of(bytes(range(256)) * 8 + b"\1")
    sent = len(port.tx_log)
    for length in (1, 2, 1000, len(pack.encoded)):
        with pytest.raises(Refused) as e:
            ld._fa(length, 0)
        assert "upload()" in str(e.value)
    assert len(port.tx_log) == sent
    assert ld.upload(pack) == pack.crc16
    sent = len(port.tx_log)
    for length in (len(pack.encoded) - 1, len(pack.encoded) + 1, len(pack.encoded)):
        with pytest.raises(Refused):
            ld._fa(length, pack.crc16)
    assert len(port.tx_log) == sent and not wire.uncertain
    assert tx_fa_lengths(port) == [len(pack.encoded)]


def tx_fa_lengths(port):
    """The lengths of every FA request sent on this port."""
    out = []
    for _, raw in port.tx_log:
        if raw[:5] == b"\xff\xff\xfd\x0a\xfe" and len(raw) > 7 and raw[7] == 0xFA:
            out.append(struct.unpack_from("<I", raw, 8)[0])
    return out


def test_unsolicited_bytes_latch(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    port.rx += b"\x55"
    with pytest.raises(BusUncertain):
        wire.ping(1)
    port.rx.clear()
    with pytest.raises(BusUncertain):
        wire.ping(1)


def test_stale_input_drained_at_open(sim_bus, fw231_servos):
    """FTDI adapters hand over bytes buffered while the port was closed, some a few ms after open: they are read
    and logged at open without a latch; bytes after that still latch before a request."""
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    events = []
    wire.log = events.append
    port.rx += b"\x00\x76"
    port.later.append((port.clock.t + 0.010, b"\x67"))           # within the FTDI latency timer
    assert wire.drain_stale() == b"\x00\x76\x67"
    assert not wire.uncertain
    assert [e for e in events if e["event"] == "rx_stale_at_open"][0]["hex"] == "007667"
    assert wire.ping(1)["firmware"] == 231
    assert wire.drain_stale() == b""                              # a quiet port: nothing, no event
    assert sum(e["event"] == "rx_stale_at_open" for e in events) == 1
    port.rx += b"\x55"                                             # after open: the strict latch as before
    with pytest.raises(BusUncertain):
        wire.ping(1)


def test_missing_early_member_returns_partial_without_latching(sim_bus, fw231_servos):
    s2 = fw231_servos(2)[1]                                       # only ID 2 is present
    wire, _ = sim_bus([s2])
    with pytest.raises(NoReply) as e:
        wire.sync_read([1, 2], 64, 68)
    assert list(e.value.partial) == [2]
    assert e.value.missing == [1]
    assert "[1]" in str(e.value)
    assert not wire.uncertain
    assert wire.ping(2)["firmware"] == 231                         # the bus keeps working


def test_write_or_read_exception_latches(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])

    def boom(data):
        raise OSError("USB unplugged")
    port.write = boom
    with pytest.raises(BusUncertain):
        wire.ping(1)
    with pytest.raises(BusUncertain):
        wire.ping(1)                                               # nothing more is sent


class Flaky(SimPort):
    broken_waiting = broken_baud = False

    @property
    def in_waiting(self):
        if self.broken_waiting:
            raise OSError("device disconnected")
        return SimPort.in_waiting.fget(self)

    @property
    def baudrate(self):
        return self._baud

    @baudrate.setter
    def baudrate(self, v):
        if self.broken_baud:
            raise OSError("cannot set baud")
        self._baud = v


def test_h6_in_waiting_and_baud_failures_latch(fw231_servos):
    s = fw231_servos(1)[0]
    for flag, act in (("broken_waiting", lambda wire: wire.ping(1)),
                      ("broken_baud", lambda wire: wire.set_baud(3_000_000))):
        clock = FakeClock()
        port = Flaky([s], clock)
        wire = Wire(port, clock=clock, sleep=clock.sleep)
        assert wire.ping(1)["firmware"] == 231
        setattr(port, flag, True)
        with pytest.raises(BusUncertain):
            act(wire)
        assert wire.uncertain, flag
        setattr(port, flag, False)
        with pytest.raises(BusUncertain):
            wire.ping(1)                                           # nothing more is sent


class Unpluggable(SimPort):
    """A port whose USB adapter is pulled: read, write and in_waiting raise as pyserial does then."""
    failing = ()
    after_write = ()
    error = None
    writes = 0

    def unplug(self, call, error):
        """call 'write' fails at once; 'read' / 'in_waiting' fail once the next request is out (mid-read)."""
        self.error = error
        if call == "write":
            self.failing = ("write",)
        else:
            self.after_write = (call,)

    @property
    def in_waiting(self):
        if "in_waiting" in self.failing:
            raise self.error
        return SimPort.in_waiting.fget(self)

    def read(self, n=1):
        if "read" in self.failing:
            raise self.error
        return SimPort.read(self, n)

    def write(self, data):
        self.writes += 1
        if "write" in self.failing:
            raise self.error
        count = SimPort.write(self, data)
        self.failing += self.after_write
        return count


UNPLUG_ERRORS = [OSError(6, "Device not configured"),
                 serial.SerialException("device reports readiness to read but returned no data "
                                        "(device disconnected or multiple access on port?)")]


@pytest.mark.parametrize("error", UNPLUG_ERRORS, ids=["oserror", "serialexception"])
@pytest.mark.parametrize("call", ["read", "in_waiting", "write"])
def test_unplug_mid_read_latches_with_clear_message(call, error, fw231_servos):
    s = fw231_servos(1)[0]
    clock = FakeClock()
    port = Unpluggable([s], clock)
    wire = Wire(port, clock=clock, sleep=clock.sleep)
    assert wire.ping(1)["firmware"] == 231
    port.unplug(call, error)
    with pytest.raises(BusUncertain) as e:
        wire.ping(1)
    assert "adapter disconnected" in str(e.value)
    assert e.value.__suppress_context__, "no pyserial traceback in front of the message"
    assert wire.uncertain
    writes = port.writes
    with pytest.raises(BusUncertain) as again:
        wire.ping(1)
    assert port.writes == writes, "nothing is written once latched"
    assert "adapter disconnected" in str(again.value) and "reconnect" in str(again.value)


def test_reset_clears_uncertain_only_outside_loader(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    port.rx += b"\x55"
    with pytest.raises(BusUncertain):
        wire.ping(1)
    port.rx += b"\x00\x00"                                         # more junk still on the line
    assert wire.reset() == b"\x00\x00"
    assert not wire.uncertain
    assert wire.ping(1)["firmware"] == 231
    wire.enter_loader(1)                                           # now a servo is in the retained loader
    assert wire.loader_possible
    sent = len(port.tx_log)
    with pytest.raises(Refused) as e:                              # not latched: refused, nothing changes
        wire.reset()
    assert "loader" in str(e.value) and "--id <ID>, with <ID> the ID of the servo you were installing" in str(e.value)
    assert not wire.uncertain
    wire.mark_uncertain("interrupted")
    with pytest.raises(BusUncertain) as e:                         # latched: stays latched
        wire.reset()
    assert "loader" in str(e.value) and "openxl330 recover" in str(e.value)
    assert wire.uncertain and len(port.tx_log) == sent


def test_reset_refuses_a_line_that_stays_busy(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    wire.mark_uncertain("interrupted")
    port.later += [(port.clock.t + 0.01 * k, b"\0") for k in range(1, 200)]
    with pytest.raises(BusUncertain) as e:
        wire.reset()
    assert "busy" in str(e.value)
    assert wire.uncertain


# ---------------------------------------------------------------------------------------------- reboot / drain
def test_reboot_returns_to_the_application(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, _ = sim_bus([s])
    drained = wire.reboot(1)
    assert drained == 128, "the loader output after the reset is drained, not latched"
    assert (s.mode, wire.ping(1)["firmware"]) == ("app", 231)
    assert not wire.uncertain


def test_reboot_noise_without_drain_latches(sim_bus, fw231_servos):
    """A plain request into the reset noise (what a first hardware test did) latches."""
    s = fw231_servos(1)[0]
    wire, _ = sim_bus([s])
    wire._send(framing.frame(0, 1, 8))
    wire._collect([1], 0.1)
    wire.sleep(0.06)
    with pytest.raises(BusUncertain):
        wire.ping(1)


def test_drain_latches_if_the_line_never_goes_quiet(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    port.later += [(port.clock.t + 0.01 * k, b"\0") for k in range(1, 200)]
    with pytest.raises(BusUncertain):
        wire.drain(0.3, quiet=0.05, cap=0.9)


# ---------------------------------------------------------------------------------------------- Fast Sync Read
def test_manual_example_bytes():
    """The ROBOTIS e-manual example: IDs 3, 7 and 4 at Present Position 166, 2079 and 1023, exact wire bytes."""
    reply = bytes.fromhex("FFFFFD00FE1900550003A60000008408" "00071F08000016CA" "0004FF030000D19E")
    wire, port = scripted_port(reply)
    got = wire.fast_sync_read([3, 7, 4], 132, 4)
    assert port.sent[0].hex().upper() == "FFFFFD00FE0A008A8400040003070420F2"
    assert {i: struct.unpack("<i", v)[0] for i, v in got.items()} == {3: 166, 7: 2079, 4: 1023}


@pytest.mark.parametrize("settle_s", [None, 0, 0.0005])
def test_sync_write_paces_the_complete_frame_and_optional_guard(sim_bus, fw231_servos, settle_s):
    servos = fw231_servos()
    for servo in servos:
        servo.baud = 3_000_000
    wire, port = sim_bus(servos, baud=3_000_000)
    started = port.clock.t
    options = {} if settle_s is None else {"settle_s": settle_s}
    wire.sync_write(256, 16, {i: registers.encode_target(2100, 1, 0) for i in range(1, 7)}, **options)
    airtime = 10 * len(port.tx_log[-1][1]) / 3_000_000
    assert port.clock.t - started == pytest.approx(airtime + (0.002 if settle_s is None else settle_s))


@pytest.mark.parametrize("settle_s", [-0.001, float("nan"), float("inf"), True, "0", 1e20, 10**400])
def test_sync_write_rejects_bad_guard_before_transmitting(sim_bus, fw231_servos, settle_s):
    wire, port = sim_bus(fw231_servos())
    with pytest.raises(InvalidArgument, match="settle_s"):
        wire.sync_write(256, 16, {1: registers.encode_target(2100, 1, 0)}, settle_s=settle_s)
    assert not port.tx_log


def test_sync_write_latches_unexpected_reply_during_the_guard(sim_bus, fw231_servos):
    wire, port = sim_bus(fw231_servos())
    port.timed.append((wire.clock() + 0.001, APP_BAUD, b"unexpected"))
    with pytest.raises(BusUncertain, match="unexpected bytes after Sync Write"):
        wire.sync_write(256, 16, {1: registers.encode_target(2100, 1, 0)})
    assert wire.uncertain
    assert len(port.tx_log) == 1


def test_matches_sync_read_on_six_servos(sim_bus, fw231_servos):
    servos = fw231_servos()
    wire, port = sim_bus(servos)
    fast = wire.fast_sync_read(range(1, 7), 0, 16)
    slow = wire.sync_read(range(1, 7), 0, 16)
    assert fast == slow
    assert [v[7] for v in fast.values()] == list(range(1, 7))      # address 7 = each servo's ID
    assert port.tx_log[-2][1][7] == 0x8A


def test_fast_sync_read_hashes_each_reply_byte_at_most_once(monkeypatch):
    ids, size = range(1, 7), 20
    reply = bytearray(b"\xff\xff\xfd\0\xfe" + struct.pack("<H", 1 + len(ids) * (size + 4)) + b"\x55")
    for servo_id in ids:
        reply.extend(bytes([0, servo_id]) + bytes([servo_id]) * size)
        reply.extend(struct.pack("<H", framing.crc16(reply)))
    wire, port = scripted_port(reply)
    hashed_bytes = []
    crc16 = framing.crc16

    def count_crc(data, initial=0):
        hashed_bytes.append(len(data))
        return crc16(data, initial)

    monkeypatch.setattr(framing, "crc16", count_crc)
    assert wire.fast_sync_read(ids, 1088, size) == {i: bytes([i]) * size for i in ids}
    assert sum(hashed_bytes) <= len(reply) + len(port.sent[0]) - 2


def test_missing_servo_ends_the_packet(sim_bus, fw231_servos):
    servos = fw231_servos(4)
    servos[2].mode = "dead"
    wire, _ = sim_bus(servos)
    with pytest.raises(NoReply) as e:
        wire.fast_sync_read([1, 2, 3, 4], 0, 16)
    assert sorted(e.value.partial) == [1, 2]
    assert e.value.missing == [3, 4]
    assert not wire.uncertain
    assert sorted(wire.sync_read([1, 2, 4], 0, 16)) == [1, 2, 4]   # bus still usable


def test_bad_row_crc_latches(sim_bus, fw231_servos, monkeypatch):
    servos = fw231_servos(3)
    wire, _ = sim_bus(servos)
    real = sim.fast_packet

    def corrupt(slots, rows, size):
        pkt = bytearray(real(slots, rows, size))
        pkt[8 + (size + 4) + 3] ^= 1
        return bytes(pkt)
    monkeypatch.setattr(sim, "fast_packet", corrupt)
    with pytest.raises(BusUncertain):
        wire.fast_sync_read([1, 2, 3], 0, 16)


@pytest.mark.parametrize("mutate", [lambda p: p[:8 + 8 + 3], lambda p: p[:9] + bytes([9]) + p[10:]],
                         ids=["cut_row", "wrong_id"])
def test_wrong_id_and_cut_row_latch(mutate, sim_bus, fw231_servos, monkeypatch):
    servos = fw231_servos(2)
    wire, _ = sim_bus(servos)
    real = sim.fast_packet
    monkeypatch.setattr(sim, "fast_packet", lambda slots, rows, size: mutate(real(slots, rows, size)))
    with pytest.raises(BusUncertain):
        wire.fast_sync_read([1, 2], 0, 4)


# ---------------------------------------------------------------------------------------------- scan
def error_on_ping(servo, error=2):
    """A device seen in the field: answers a ping with an error status (still carrying model/firmware)."""
    real = servo.on_frame

    def reply(f, clock):
        r = real(f, clock)
        if r and f["instruction"] == 1:
            return (r[0], servo.status(error, struct.pack("<HB", 1200, 52)))
        return r
    servo.on_frame = reply
    return servo


def test_h7_scan_keeps_collecting_after_an_error_reply(sim_bus, fw231_program):
    odd = error_on_ping(SimServo(3, mode="stock", seed=66, eeprom=default_eeprom(3, baud_index=3)))
    ours = SimServo(1, mode="app", program=fw231_program(servo_id=5), seed=67)
    wire, port = sim_bus([odd, ours])
    port.ping_slots = True                                         # replies in their 3 ms slots
    found = wire.scan(10)
    assert sorted(found) == [3, 5]
    assert (found[3]["error"], found[3]["firmware"], found[5]["firmware"]) == (2, 52, 231)
    assert "error" not in found[5]
    assert wire.ping(5)["firmware"] == 231                         # the later slot did not latch it
    assert not wire.uncertain


# ---------------------------------------------------------------------------------------------- loader
def test_two_servos_in_loader_detected(sim_bus):
    a, b = SimServo(1, seed=1), SimServo(2, seed=2)
    a.reset("loader")
    b.reset("loader")
    wire, _ = sim_bus([a, b], 57_600)
    with pytest.raises(BusUncertain):
        Loader(wire).handshake()
    assert wire.uncertain


def test_partial_upload_forbids_f8(sim_bus):
    s = SimServo(1, seed=6)
    s.reset("loader")
    wire, port = sim_bus([s], 57_600)
    ld = Loader(wire)
    ld.handshake()
    ld.started = True
    sent = len(port.tx_log)
    with pytest.raises(Refused) as e:
        ld.leave()
    assert "openxl330 recover --port" in str(e.value) and len(port.tx_log) == sent


def test_enter_loader_upload_and_launch(sim_bus, fw231_program, packaged, flash_snapshot_of):
    """A servo installed earlier with a device-block override (ID 6) gets the bundled release (default device block):
    it then answers at its stock NV ID (4)."""
    s = SimServo(4, mode="app", program=fw231_program(servo_id=6, label="wrist"), seed=3)
    before = flash_snapshot_of(s)
    wire, port = sim_bus([s])
    loader = wire.enter_loader(6)
    assert (wire.loader_possible, wire.loader_phase, wire.f9_acked) == (True, "loader", 6)
    assert (port.baudrate, s.mode) == (LOADER_BAUD, "loader")
    sent = len(port.tx_log)
    with pytest.raises(Refused) as e:
        wire.enter_loader(2)                                       # one servo in the loader at a time
    assert "openxl330 recover --port SIM --id <ID>" in str(e.value) and len(port.tx_log) == sent
    pack = packaged()
    seen = []
    loader.upload(pack, on_progress=lambda done, total: seen.append((done, total)))
    assert wire.loader_phase == "uploaded" and seen[-1] == (len(pack.encoded), len(pack.encoded))
    loader.leave()
    assert (wire.loader_possible, wire.loader_phase, wire.f9_acked, port.baudrate) == (False, "launched", None, APP_BAUD)
    assert bytes(s.flash[0x4000:0x4000 + len(pack.raw)]) == pack.raw
    assert flash_snapshot_of(s) == before, "loader and own NVM untouched"
    assert (s.writes_outside_program, s.mode, s.version, s.effective_id()) == (0, "app", 231, 4)
    assert wire.ping(4)["firmware"] == 231
    assert wire.flash_read(4, 0x08004000, 300) == pack.raw[:300]
    assert not wire.uncertain


# ---------------------------------------------------------------------------------------------- primitives for adapters
def test_echo_probe_sends_only_the_id_253_ping(sim_bus, fw231_servos):
    """There is no public raw send: the adapter echo check uses a fixed, private probe that can only send the ping to
    ID 253 (no servo uses it) and returns what arrives, unjudged."""
    assert not hasattr(Wire, "send_and_listen")
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    assert wire._echo_probe() == b""                                       # nobody at ID 253
    assert [d for _, d in port.tx_log] == [framing.frame(0, 253, 1)]
    port.rx += b"\x55"
    with pytest.raises(BusUncertain):                                       # unsolicited bytes before it still latch
        wire._echo_probe()
    sent = len(port.tx_log)
    with pytest.raises(BusUncertain):
        wire._echo_probe()                                                  # latched: nothing sent
    assert len(port.tx_log) == sent


def _fa_frames(port, since=0):
    """Every frame with instruction 0xFA in the simulated port's TX log after entry `since`."""
    out = []
    for _, data in port.tx_log[since:]:
        start = data.find(b"\xff\xff\xfd")
        while start >= 0:
            if len(data) > start + 7 and data[start + 7] == 0xFA:
                out.append(data[start:])
            start = data.find(b"\xff\xff\xfd", start + 1)
    return out


FA_FRAME = bytes.fromhex("fffffd0afe0900fa010000000000") + b"\0\0"
_PROBE_ARGS = [(), (1,), (254,), (1, 0xFA), (1, 0, 1), (1, 64, b"\0"), (254, 0xFA, b"\1\0\0\0\0\0"),
               (0x08000000, 1), (1, 0x08000000, 16), (FA_FRAME,), ([1], 0, 4), (0xFA, 6, {1: bytes(6)}),
               ([(1, 0, 4)],), (57_600,), (True,), (False,)]


def _in_loader(sim_bus, fw231_program):
    s = SimServo(1, mode="app", program=fw231_program(), seed=3)
    wire, port = sim_bus([s])
    loader = wire.enter_loader(1)
    return wire, port, loader


def _public_methods(obj):
    return sorted(name for name in dir(obj) if not name.startswith("_") and callable(getattr(obj, name)))


def test_no_public_method_but_upload_sends_the_loader_crc_check(sim_bus, fw231_program, packaged):
    """With a servo in the retained loader, no public Wire or Loader method other than Loader.upload() sends an FA
    (0xFA) request, whatever its arguments; upload() sends exactly one."""
    wire, port, loader = _in_loader(sim_bus, fw231_program)
    timing = {"clock", "sleep", "drain"}            # the Wire's clock and a read-only listening window: no TX at all
    since = len(port.tx_log)
    wire.clock(), wire.sleep(0.01), wire.drain(0.1)
    assert len(port.tx_log) == since and not wire.uncertain
    names = [("wire", n) for n in _public_methods(wire) if n not in timing]
    names += [("loader", n) for n in _public_methods(loader) if n != "upload"]
    assert ("wire", "flash_read") in names and ("loader", "leave") in names and ("wire", "write") in names
    for owner, name in names:
        for args in _PROBE_ARGS:
            wire, port, loader = _in_loader(sim_bus, fw231_program)
            since = len(port.tx_log)
            method = getattr(wire if owner == "wire" else loader, name)
            try:
                method(*args)
            except Exception:
                pass
            assert not _fa_frames(port, since), f"{owner}.{name}{args!r} sent FA"
    wire, port, loader = _in_loader(sim_bus, fw231_program)
    since = len(port.tx_log)
    pack = packaged()
    assert loader.upload(pack) == pack.crc16
    assert [struct.unpack_from("<I", f, 8)[0] for f in _fa_frames(port, since)] == [len(pack.encoded)]


def test_public_requests_are_refused_while_a_servo_may_be_in_the_loader(sim_bus, fw231_program):
    """Only the loader session talks to a servo in the retained loader: every public Wire request is refused before
    anything is sent, until F8 is acknowledged."""
    wire, port, loader = _in_loader(sim_bus, fw231_program)
    sent = len(port.tx_log)
    for call in (lambda: wire.ping(1), lambda: wire.read(1, 0, 4), lambda: wire.write(1, 64, b"\0"),
                 lambda: wire.write_broadcast(64, b"\0"), lambda: wire.scan(), lambda: wire.sync_read([1], 0, 4),
                 lambda: wire.fast_sync_read([1], 0, 4), lambda: wire.sync_write(256, 1, {1: b"\0"}),
                 lambda: wire.bulk_read([(1, 0, 4)]), lambda: wire.reboot(1), lambda: wire.flash_read(1, 0x08004000, 4),
                 lambda: wire._echo_probe()):
        with pytest.raises(Refused) as e:
            call()
        assert "loader" in str(e.value)
    assert len(port.tx_log) == sent and not wire.uncertain


def test_flash_read_of_the_loader_region_is_refused_before_tx(sim_bus, fw231_servos):
    """Public flash_read never reads the retained loader (0x08000000..0x08003FFF): refused before anything is sent,
    on any image. The program and the calibration region still read."""
    s = fw231_servos(1)[0]                                                  # an older image: A0 would return the loader
    wire, port = sim_bus([s])
    for address, size in ((0x08000000, 16), (0x08003FFF, 1), (0x08003FF0, 32), (0x08000000, 0x10000)):
        with pytest.raises(Refused) as e:
            wire.flash_read(1, address, size)
        assert "0x08000000..0x08003FFF" in str(e.value)
    assert port.tx_log == [] and not wire.uncertain
    assert wire.flash_read(1, 0x08004000, 16) == bytes(s.flash[0x4000:0x4010])
    assert wire.flash_read(1, 0x0800E000, 16) == bytes(s.flash[0xE000:0xE010])


def test_restore_baud_works_while_latched(sim_bus, fw231_servos):
    s = fw231_servos(1)[0]
    wire, port = sim_bus([s])
    wire.set_baud(3_000_000)
    wire.mark_uncertain("test")
    with pytest.raises(BusUncertain):
        wire.set_baud(1_000_000)
    sent = len(port.tx_log)
    wire.restore_baud(1_000_000)                                            # nothing is sent; the latch stays
    assert (port.baudrate, wire.uncertain, len(port.tx_log)) == (1_000_000, True, sent)
