"""Strict multi-ID Protocol 2.0 client plus the retained-loader session. Opens no port.

The caller owns the serial port (pyserial-like: write/read/in_waiting/flush/baudrate).
  * The host is the only initiator, so unsolicited or unparseable bytes mean the framing is unknown: the Wire
    latches `uncertain` and refuses every later transmission (BusUncertain) until reset().
  * Replies must be canonical, CRC-valid status packets from exactly the expected IDs.
  * A port call that fails (an unplugged adapter: OSError or pyserial's SerialException from read, write or
    in_waiting) leaves what reached the wire unknown: it latches the same way ("adapter disconnected ...").
    So does a wire log that fails mid-exchange.
  * Bad arguments raise InvalidArgument and state refusals raise Refused, both before anything is sent.
  * The one exception to the latch is emergency_off(): the OFF path (Servo.disable, Group.disable, CLI off) still
    sends torque OFF once to each ID it was turning off, draining replies without judging them, and stays latched.
"""
from __future__ import annotations

import itertools
import json
import math
import os
import struct
import time
from datetime import datetime, timezone
from pathlib import Path

from . import paths
from ._protocol import framing, packets
from .errors import BusUncertain, InvalidArgument, NoReply, Refused, Rejected

BROADCAST = 0xFE
LOADER_BAUD = 57_600
APP_BAUD = 1_000_000
SYNC_WRITE_SETTLE_S = 0.002
_PROGRAM_BYTES = 0xE000 - 0x4000    # the program region 0x08004000..0x0800DFFF: the most an upload may hold
_LOADER_END = 0x08004000             # the retained loader is 0x08000000..0x08003FFF

_ADVICE = 'check the adapter and wiring, then call reset() or reopen the port'
_RECONNECT = 'reconnect the adapter, then open the port again'
_LOG_ADVICE = 'fix the wire log (disk space, or the log callable), then call reset() or reopen the port'
_serial = itertools.count(1)


# ---------------------------------------------------------------- argument checks (before anything is sent)
def _unicast_id(servo_id):
    """One servo's ID. 253 passes (a ping to it is how an echoing adapter is detected); broadcast does not."""
    if type(servo_id) is not int or not 0 <= servo_id <= 253:
        raise InvalidArgument(f'servo ID {servo_id!r} is not valid: use 0..252 (scan() and write_broadcast() '
                              f'address every servo)')
    return servo_id


def _group_ids(ids, what):
    try:
        ids = list(ids)
    except TypeError:
        raise InvalidArgument(f'{what}: give an iterable of IDs') from None
    if not ids:
        raise InvalidArgument(f'{what}: give at least one ID')
    bad = [i for i in ids if type(i) is not int or not 0 <= i <= 252]
    if bad:
        raise InvalidArgument(f'{what}: ID(s) {bad} not valid: use 0..252')
    if len(set(ids)) != len(ids):
        raise InvalidArgument(f'{what}: duplicate IDs in {ids}: list each servo once')
    return ids


def _check_flash_range(servo_id, address, size):
    _unicast_id(servo_id)
    if (type(address) is not int or type(size) is not int or size < 0
            or not (0x08000000 <= address and address + size <= 0x08010000)):
        raise InvalidArgument(f'flash read of {size!r} B at {address!r}: stay within 0x08000000..0x0800FFFF')


def _pack(fmt, *values):
    """struct.pack for request fields: an out-of-range address, size or value is the caller's argument."""
    try:
        return struct.pack(fmt, *values)
    except struct.error as e:
        raise InvalidArgument(f'request field out of range ({e}): check the address and size') from None


def _payload(data):
    """bytes(data) for a write. An int is refused: bytes(1) is one zero byte, not the value 1."""
    if isinstance(data, (int, str)):
        raise InvalidArgument(f'data must be bytes (for example b"\\x01"), not {type(data).__name__}')
    try:
        return bytes(data)
    except (TypeError, ValueError) as e:
        raise InvalidArgument(f'data must be bytes: {e}') from None


def _seconds(value, what, *, allow_zero=False):
    valid = isinstance(value, (int, float)) and not isinstance(value, bool)
    if valid:
        try:
            if math.isfinite(value) and (value >= 0 if allow_zero else value > 0):
                return value
        except OverflowError:
            pass
    bound = 'nonnegative' if allow_zero else 'positive'
    raise InvalidArgument(f'{what} must be a finite {bound} number of seconds')


class WireLog:
    """JSONL wire log, opened on the first event, line-buffered (survives a crash mid-run)."""

    def __init__(self, path):
        self.path, self.f = Path(path), None

    def __call__(self, event):
        if self.f is None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.f = self.path.open('a', buffering=1, encoding='utf-8')
        self.f.write(json.dumps(event, separators=(',', ':')) + '\n')

    def close(self):
        if self.f:
            self.f.close()
            self.f = None


def default_log():
    """A new wire log file under $OPENXL330_HOME/logs (see paths.home): what Wire(port, log=True) uses; the
    installer keeps one per run."""
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    return WireLog(paths.home() / 'logs' / f'{stamp}-{os.getpid()}-{next(_serial)}.jsonl')


# Loader progress on one Wire (receipts record it as `phase`). 'f9_sent' / 'f8_sent': sent, not (yet) acknowledged;
# 'recovering': a recover is taking over a servo that is already in the loader; 'launched': F8 acknowledged.
LOADER_PHASES = (None, 'f9_sent', 'f9_acked', 'loader_prompt_started', 'loader_prompt_done', 'loader', 'recovering',
                 'recovering_fresh', 'uploading', 'uploaded', 'f8_sent', 'launched')


class Wire:
    def __init__(self, port, log=False, clock=time.monotonic, sleep=time.sleep):
        """log: False or None = no log and no files (normal use); True = a new wire log file from default_log()
        (what the installer uses); a callable(dict) receives every wire event. The log is part of each exchange:
        if it raises, the Wire latches."""
        if log is True:
            log = default_log()
        elif log is None or log is False:
            log = None
        elif not callable(log):
            raise InvalidArgument(f'log must be False or None (no log), True (a log file under $OPENXL330_HOME/logs) '
                                  f'or a callable, not {log!r}')
        self.port, self.clock, self.sleep = port, clock, sleep
        self.uncertain = False
        self._why, self._advice = None, _ADVICE
        self.f9_acked = None            # ID that acknowledged F9 and has not acknowledged F8 since (receipts: in_loader)
        self.loader_phase = None        # last loader step on this Wire (LOADER_PHASES); 'f9_sent'/'f8_sent' = no ACK yet
        self.log = log
        self.loader = None

    @property
    def loader_possible(self):
        """True while a servo reached through this Wire may be in the retained loader: F9 sent or acknowledged, a
        loader attached or being recovered, and no F8 acknowledged since."""
        return self.loader is not None or self.f9_acked is not None or self.loader_phase not in (None, 'launched')

    @property
    def _recover_command(self):
        port = getattr(self.port, 'port', None) or '<port>'
        return f'openxl330 recover --port {port} --id <ID>, with <ID> the ID of the servo you were installing'

    # ------------------------------------------------------------ log and latch
    def _log(self, kind, **data):
        self.log(dict(event=kind, t=round(self.clock(), 6), baud=self.port.baudrate, **data))

    def _event(self, kind, **data):
        """Log one wire event. The log is part of the exchange: if it fails, the exchange did not finish (a request
        may be on the wire with its reply unread), so the Wire latches; an interrupt latches and propagates."""
        if not self.log:
            return
        try:
            self._log(kind, **data)
        except Exception as e:
            raise self._latched(f'the wire log failed on a {kind!r} event: {e!r}', _LOG_ADVICE) from None
        except BaseException as e:
            self.mark_uncertain(f'the wire log was interrupted on a {kind!r} event: {e!r}')
            raise

    def _latched(self, why, advice=_ADVICE, **data):
        """Latch and return the BusUncertain to raise. The latch is set before the 'uncertain' event is logged, and
        a log failure here is not raised over the cause (the next logged event reports it)."""
        self.uncertain, self._why, self._advice = True, why, advice
        if self.log:
            try:
                self._log('uncertain', why=why, **data)
            except Exception:
                pass
        return BusUncertain(f'{why}. The bus state is unknown, so nothing more is sent: {advice}.')

    def _latch(self, why, advice=_ADVICE, **data):
        raise self._latched(why, advice, **data)

    def _still_latched(self):
        return BusUncertain(f'nothing sent: the bus state became unknown earlier ({self._why}); {self._advice}.')

    def mark_uncertain(self, why):
        """Latch without raising: the caller is already propagating another exception (an interrupted exchange,
        e.g. KeyboardInterrupt in the middle of a request, leaves the framing unknown)."""
        if not self.uncertain:
            self._latched(why)

    def redact_latch(self, reason):
        """Replace the reason of the current latch, which every later refusal quotes, while the Wire stays latched:
        for a latch whose reason quotes bytes that must not be shown (the installer's loader-digest read). Nothing is
        sent, logged or unlatched. Refused (Refused) when the Wire is not latched: there is no reason to replace."""
        if not isinstance(reason, str) or not reason.strip():
            raise InvalidArgument(f'latch reason {reason!r}: give a non-empty text')
        if not self.uncertain:
            raise Refused('latch reason not replaced: the Wire is not latched (redact_latch only rewords an existing '
                          'latch, it never latches)')
        self._why = reason

    def _port_failed(self, what, e):
        """The BusUncertain for a port call that raised. An OSError (pyserial's SerialException is one) means the
        adapter is gone or not responding; the pyserial traceback is not chained in front of the message."""
        if isinstance(e, OSError):
            return self._latched(f'adapter disconnected or not responding ({what} failed: {e!r})', _RECONNECT)
        return self._latched(f'{what} failed: {e!r}')

    def _io(self, what, call, *args):
        try:
            return call(*args)
        except Exception as e:                       # what reached the wire is unknown
            raise self._port_failed(what, e) from None
        except BaseException as e:                   # interrupted (KeyboardInterrupt): same, but keep it propagating
            self.mark_uncertain(f'{what} interrupted: {e!r}')
            raise

    # ------------------------------------------------------------ raw I/O
    def _waiting(self):
        """port.in_waiting through the latch: a port that cannot say what is buffered leaves the framing unknown."""
        return self._io('serial in_waiting', lambda: self.port.in_waiting)

    def _read(self, n):
        return self._io('serial read', self.port.read, n)

    def _port_baud(self, bps):
        """Every host baud change goes through here: a failed change leaves the line state unknown (latch)."""
        try:
            self.port.baudrate = bps
        except Exception as e:
            raise self._latched(f'setting the host baud to {bps} failed: {e!r}',
                                'check that the adapter supports this baud and is still connected, '
                                'then reopen the port') from None
        self._event('host_baud', bps=bps)

    def _ready(self):
        if self.uncertain:
            raise self._still_latched()
        n = self._waiting()
        if n:
            junk = self._read(n)          # logged with the latch (event 'uncertain', hex), which no log failure undoes
            self._latch('unsolicited bytes before a request', hex=junk.hex())

    def _send(self, raw, *, loader=False):
        """Write one request. Only the loader session (enter_loader's F9 and Loader) passes loader=True. Every other
        request is refused before anything is sent while a servo on this Wire may be in the retained loader, and can
        never be a loader frame (reserved byte 10) or an FA request (0xFA): only Loader.upload sends FA."""
        if not loader and not self.uncertain:
            if self.loader_possible:
                raise Refused(f'not sent: a servo on this bus may still be in the retained loader (phase '
                              f'{self.loader_phase!r}), and only the loader session talks to it. Finish it '
                              f'(Loader.upload, then Loader.leave) or recover the servo: {self._recover_command}')
            if raw[3:4] == b'\x0a' or raw[7:8] == b'\xfa':
                raise Refused('not sent: loader requests are sent only by the loader session (Loader)')
        self._ready()
        count = self._io('serial write', self.port.write, raw)
        if self.log:
            self._event('tx', hex=raw.hex())
        if count != len(raw):
            self._latch('partial serial write')
        self._io('serial flush', self.port.flush)

    def _quiet_read(self, quiet, limit):
        """Read (without latching on content) until nothing arrived for `quiet` s, at most `limit` s.
        Returns (bytes, went_quiet)."""
        got, start = bytearray(), self.clock()
        last = start
        while self.clock() - start < limit:
            n = self._waiting()
            if n:
                got += self._read(n)
                last = self.clock()
            elif self.clock() - last >= quiet:
                return bytes(got), True
            else:
                self.sleep(0.002)
        return bytes(got), False

    def drain_stale(self, quiet=0.03, limit=0.5):
        """Call once after opening the port, before the first request: read and log whatever is already waiting.
        An FTDI adapter (U2D2, FT232H) keeps buffering bus traffic while the port is closed and hands it over within
        its latency timer (16 ms with the macOS driver); a hardware test saw 0x00 0x76 0x67 there with a second
        adapter on the bus. Nothing has been asked yet, so this is not framing trouble and does not latch: input is
        read until it has been quiet for `quiet` s (at most `limit` s). Anything arriving later is judged by the strict
        rules again (unsolicited bytes before a request latch). Returns the bytes drained."""
        if self.uncertain:
            raise self._still_latched()
        got, _ = self._quiet_read(quiet, limit)
        if got:
            self._event('rx_stale_at_open', n=len(got), hex=got.hex())
        return got

    def reset(self, quiet=0.03, limit=0.5):
        """Clear the uncertain latch once the line is quiet: whatever is waiting is read and logged until nothing
        arrived for `quiet` s. Refused while a servo on this Wire may be in the retained loader (BusUncertain if
        latched, else Refused; nothing is read), and when the line is still busy after `limit` s (the latch stays).
        Returns the drained bytes."""
        if self.loader_possible:
            raise (BusUncertain if self.uncertain else Refused)(
                f'not reset: a servo on this bus may still be in the retained loader (phase {self.loader_phase!r}). '
                f'Recover it first: {self._recover_command}')
        got, went_quiet = self._quiet_read(quiet, limit)
        if got:
            self._event('rx_drained', hex=got.hex())
        if not went_quiet:
            self._latch(f'line still busy after {limit} s, so not reset',
                        'make sure nothing else talks on the bus (a second adapter, a servo resetting), then reset() '
                        'again')
        self.uncertain, self._why, self._advice = False, None, _ADVICE
        self._event('reset', drained=len(got))
        return got

    def set_baud(self, bps):
        if self.uncertain:
            raise self._still_latched()
        self._port_baud(bps)

    def restore_baud(self, bps):
        """Set the host baud even while latched, to put the host back where it was after a failed sequence. Nothing is
        sent and the latch stays; a failure latches (BusUncertain)."""
        self._port_baud(bps)

    def _echo_probe(self, quiet=0.04, limit=0.2):
        """The adapter echo check's only request: one ping to ID 253, which no servo uses (refused while latched, and
        unsolicited bytes waiting before it latch, as for every request). Returns the bytes that arrive until the line
        has been quiet for `quiet` s (at most `limit` s), without judging them: adapters.check_echo decides what they
        mean. Logged as tx/rx; nothing here latches on what came back. There is no public raw send."""
        self._send(framing.frame(0, 253, 1))
        got, _ = self._quiet_read(quiet, limit)
        if got:
            self._event('rx', hex=got.hex())
        return got

    def _take(self, buf, reserved=0):
        """Parse one complete status frame from the front of buf. None if incomplete."""
        if len(buf) >= 1 and buf[0] != 0xFF or len(buf) >= 2 and buf[1] != 0xFF or len(buf) >= 3 and buf[2] != 0xFD:
            self._latch('unexpected bytes where a status packet should start: ' + bytes(buf[:16]).hex())
        if len(buf) < 7:
            return None
        total = 7 + int.from_bytes(buf[5:7], 'little')
        if not 11 <= total <= 1100:
            self._latch('implausible status length')
        if len(buf) < total:
            return None
        raw = bytes(buf[:total])
        del buf[:total]
        try:
            st = framing.parse_status(raw)
        except ValueError as e:
            self._latch('invalid status packet (CRC/collision?): ' + str(e))
        if framing.stuff(bytes([0x55, st['error']]) + st['data']) != raw[7:-2]:
            self._latch('non-canonical byte stuffing in status')
        if st['reserved'] != reserved:
            self._latch('wrong protocol reserved byte')
        return st

    def _collect(self, ids, timeout, reserved=0, extra_ok=False):
        """Read status packets from `ids` in order until all arrived or timeout. A group member may be
        missing: later members still answer after their fallback, so a reply from a LATER expected ID
        marks the earlier ones missing. Unexpected, duplicate or decreasing IDs, bad CRC and partial
        bytes still latch."""
        buf, got, missing, deadline = bytearray(), {}, [], self.clock() + timeout
        want = list(ids)
        while want and self.clock() < deadline:
            part = self._read(max(1, min(self._waiting(), 4096)))
            if part:
                if self.log:
                    self._event('rx', hex=part.hex())
                buf.extend(part)
            while want:
                st = self._take(buf, reserved)
                if st is None:
                    break
                if st['id'] not in want:
                    self._latch(f"reply from unexpected, repeated or out-of-order ID {st['id']}")
                k = want.index(st['id'])
                missing += want[:k]
                want = want[k + 1:]
                got[st['id']] = st
        if buf:
            self._latch('trailing partial bytes after replies')
        if want or missing:
            lost = missing + want
            raise NoReply(f'no reply from ID(s) {lost}: check that they are powered and connected, and their ID and '
                          f'baud', got, lost)
        if not extra_ok and self._waiting():
            self.sleep(0.002)
            self._ready()
        return got

    def _checked(self, st, size=None):
        if st['error'] & 0x7F:
            raise Rejected(st['id'], st['error'] & 0x7F, st['data'])
        if size is not None and len(st['data']) != size:
            self._latch(f"ID {st['id']} returned {len(st['data'])} bytes, expected {size}")
        return st['data']

    def _byte_s(self):
        return 10.0 / self.port.baudrate

    # ------------------------------------------------------------ application
    def ping(self, servo_id, timeout=0.1):
        _seconds(timeout, 'ping timeout')
        self._send(framing.frame(0, _unicast_id(servo_id), 1))
        st = self._collect([servo_id], timeout)[servo_id]
        data = self._checked(st, 3)
        model, fw = struct.unpack('<HB', data)
        return dict(model=model, firmware=fw, alert=bool(st['error'] & 0x80))

    def scan(self, max_id=252, settle=0.05):
        """Broadcast ping. Replies come in ID order, ID * 3 ms apart (stock slot), so the window covers IDs
        0..max_id: a servo above max_id answers after it and latches the next request. Use the full 252
        unless every ID on the bus is known to be lower.
        {id: dict(model, firmware, alert)}. A reply with an error status does not end the scan (the later slots
        are still on their way): its entry gets `error` (the 7-bit status error), with model/firmware decoded
        when it carried the 3 ping bytes and None otherwise."""
        if type(max_id) is not int or not 0 <= max_id <= 252:
            raise InvalidArgument('scan max_id must be an integer from 0 to 252')
        _seconds(settle, 'scan settle', allow_zero=True)
        self._send(framing.frame(0, BROADCAST, 1))
        buf, found, deadline = bytearray(), {}, self.clock() + (max_id + 1) * 0.003 + settle
        while self.clock() < deadline:
            part = self._read(max(1, min(self._waiting(), 4096)))
            if part:
                self._event('rx', hex=part.hex())
                buf.extend(part)
            while True:
                st = self._take(buf)
                if st is None:
                    break
                if st['id'] > 252:
                    self._latch(f"scan reply from invalid servo ID {st['id']}")
                if st['id'] in found:
                    self._latch(f"two replies from ID {st['id']} (duplicate IDs on the bus)",
                                'connect the servos one at a time and give each its own ID, then reopen the port')
                error = st['error'] & 0x7F
                if not error:
                    model, fw = struct.unpack('<HB', self._checked(st, 3))
                    found[st['id']] = dict(model=model, firmware=fw, alert=bool(st['error'] & 0x80))
                    continue
                model, fw = struct.unpack('<HB', st['data']) if len(st['data']) == 3 else (None, None)
                found[st['id']] = dict(model=model, firmware=fw, alert=bool(st['error'] & 0x80), error=error,
                                       data=st['data'].hex())
                self._event('scan_error_reply', id=st['id'], error=error)
        if buf:
            self._latch('partial bytes after broadcast ping (collision or duplicate IDs?)',
                        'connect the servos one at a time and give each its own ID, then reopen the port')
        return found

    def read(self, servo_id, address, size, timeout=0.1):
        _seconds(timeout, 'read timeout')
        self._send(framing.frame(0, _unicast_id(servo_id), 2, _pack('<HH', address, size)))
        return self._checked(self._collect([servo_id], timeout)[servo_id], size)

    def write(self, servo_id, address, data, timeout=0.1):
        """Returns True when the reply carried the alert bit (a fault is latched)."""
        _seconds(timeout, 'write timeout')
        self._send(framing.frame(0, _unicast_id(servo_id), 3, _pack('<H', address) + _payload(data)))
        st = self._collect([servo_id], timeout)[servo_id]
        self._checked(st, 0)
        return bool(st['error'] & 0x80)

    def emergency_off(self, ids=(), *, bauds=None, broadcast=False, quiet=0.005, limit=0.05):
        """The OFF path's last resort once this Wire is latched (BusUncertain): torque OFF (write 64 <- 0, safe and
        idempotent on every firmware) once to each of `ids`, in order, or one broadcast OFF (broadcast=True), without
        judging the line or any reply. `bauds`: the host bauds to send at, in order (default: the current one; a
        latched scan may have left the host elsewhere): at a baud a servo does not listen at, the writes are line noise
        to it. Before and after each write the line is read until quiet (at most `limit` s), logged when a log works,
        never parsed. The Wire stays latched, so nothing confirms the OFF: support the joints and check them on a new
        connection. Nothing is sent while a servo on this Wire may be in the retained loader (its framing there is
        unproven). A failing log never stops it; a failing port does. Returns the IDs (or [254] for broadcast) an OFF
        write went to."""
        ids = [_unicast_id(servo_id) for servo_id in ids]
        if self.loader_possible:
            return []
        targets = [BROADCAST] if broadcast else ids
        frames = [(i, framing.frame(0, i, 3, _pack('<H', 64) + b'\x00')) for i in targets]
        sent = set()
        for baud in list(dict.fromkeys(bauds or [self.port.baudrate])):
            try:
                if self.port.baudrate != baud:
                    self.port.baudrate = baud
                    self._quiet_log('host_baud', bps=baud)
                self._drained(quiet, limit)
                for servo_id, raw in frames:
                    count = self.port.write(raw)
                    self.port.flush()
                    if count is not None and count != len(raw):
                        raise OSError(f'partial write ({count} of {len(raw)} bytes)')
                    sent.add(servo_id)
                    self._quiet_log('emergency_off_tx', hex=raw.hex())
                    self._drained(quiet, limit)
            except Exception as e:                   # a failing port: what reached the wire is unknown; stop
                self.mark_uncertain('emergency OFF interrupted')
                self._why = f'{self._why}; the emergency OFF stopped: {e!r}'
                break
        self.mark_uncertain('torque OFF was sent without judging the replies (emergency OFF)')   # stays latched
        return [i for i in targets if i in sent]

    def _drained(self, quiet, limit):
        """Read until the line is quiet; log what came (never parsed). A failing port raises; a failing log does not."""
        got, start = bytearray(), self.clock()
        last = start
        while self.clock() - start < limit:
            n = self.port.in_waiting
            if n:
                got += self.port.read(n)
                last = self.clock()
            elif self.clock() - last >= quiet:
                break
            else:
                self.sleep(0.002)
        if got:
            self._quiet_log('emergency_off_drained', hex=bytes(got).hex())

    def _quiet_log(self, kind, **data):
        if self.log:
            try:
                self._log(kind, **data)
            except Exception:
                pass

    def write_broadcast(self, address, data):
        data = _payload(data)
        self._send(framing.frame(0, BROADCAST, 3, _pack('<H', address) + data))
        self.sleep(len(data) * self._byte_s() + 0.002)

    def sync_read(self, ids, address, size, timeout=None):
        ids = _group_ids(ids, 'sync read')
        if timeout is not None:
            _seconds(timeout, 'sync read timeout')
        self._send(framing.frame(0, BROADCAST, 0x82, _pack('<HH', address, size) + bytes(ids)))
        timeout = timeout if timeout is not None else 0.05 + len(ids) * ((11 + size + size // 3) * self._byte_s() + 0.0006)
        got = self._collect(ids, timeout)
        return {i: self._checked(st, size) for i, st in got.items()}

    def fast_sync_read(self, ids, address, size, timeout=None):
        """Fast Sync Read 0x8A (FW231): ONE combined status packet, FF FF FD 00 FE LEN 55, then per row
        ERR ID data CRC (no byte stuffing; each row's CRC covers every packet byte before it, so the
        last one is the packet CRC). Row k is appended only after its servo checked rows 0..k-1, so a
        missing or corrupted row ends the packet there: NoReply carries the rows before it (partial).
        Anything else (bad header, wrong ID, bad CRC, a cut-off row, extra bytes) latches the Wire."""
        ids = _group_ids(ids, 'fast sync read')
        if type(size) is not int or not 1 <= size <= 128:
            raise InvalidArgument(f'fast sync read size {size!r}: use 1..128 bytes (sync_read() reads more)')
        if timeout is not None:
            _seconds(timeout, 'fast sync read timeout')
        seg, n = size + 4, len(ids)
        total, length = 8 + n * seg, 1 + n * seg
        self._send(framing.frame(0, BROADCAST, 0x8A, _pack('<HH', address, size) + bytes(ids)))
        timeout = timeout if timeout is not None else 0.05 + n * (seg * self._byte_s() + 0.0006) + 8 * self._byte_s()
        buf, deadline = bytearray(), self.clock() + timeout
        while len(buf) < total and self.clock() < deadline:
            part = self._read(max(1, min(self._waiting(), total - len(buf))))
            if part:
                if self.log:
                    self._event('rx', hex=part.hex())
                buf.extend(part)
        head = bytes([0xFF, 0xFF, 0xFD, 0x00, BROADCAST]) + struct.pack('<H', length) + b'\x55'
        if bytes(buf[:8]) != head[:min(8, len(buf))]:
            self._latch('fast sync read: unexpected header ' + bytes(buf[:8]).hex())
        got, k = {}, 0
        view = memoryview(buf)
        crc = framing.crc16(head)
        while len(buf) >= 8 + (k + 1) * seg:
            at = 8 + k * seg
            row = view[at:at + seg]
            crc = framing.crc16(row[:-2], crc)
            if int.from_bytes(row[-2:], 'little') != crc:
                self._latch(f'fast sync read: bad CRC in row {k} (collision or corruption)')
            if row[1] != ids[k]:
                self._latch(f'fast sync read: row {k} from ID {row[1]}, expected {ids[k]}')
            got[ids[k]] = dict(id=ids[k], error=row[0], data=bytes(row[2:2 + size]), reserved=0)
            # The next row's checksum includes this row's two checksum bytes.
            crc = framing.crc16(row[-2:], crc)
            k += 1
        if len(buf) not in (0, 8 + k * seg) or (k == 0 and len(buf)):
            self._latch('fast sync read: packet cut off inside a row')
        if k < n:
            self.sleep(0.004)            # the servos skip the rest of the packet until 3 ms of quiet
            self._ready()
            raise NoReply(f'no fast sync read row from ID(s) {ids[k:]} (a missing row ends the packet, so the IDs '
                          f'after it could not answer): check that ID {ids[k]} is powered and connected, and its ID '
                          f'and baud', got, ids[k:])
        if self._waiting():
            self.sleep(0.002)
            self._ready()
        return {i: self._checked(st, size) for i, st in got.items()}

    def sync_write(self, address, size, rows, *, settle_s=SYNC_WRITE_SETTLE_S):
        """Write `rows` {id: bytes of `size`}. No replies.

        Allow airtime for the complete frame after the adapter flush and `settle_s` additional seconds (0 to 1).
        The default retains a 2 ms guard. Qualify shorter guards with the adapter and fleet before streaming.
        """
        if isinstance(settle_s, bool) or not isinstance(settle_s, (int, float)) or not 0 <= settle_s <= 1:
            raise InvalidArgument('settle_s must be a number from 0 to 1 seconds')
        body = _pack('<HH', address, size)
        for servo_id, data in zip(_group_ids(rows, 'sync write'), rows.values()):
            data = _payload(data)
            if len(data) != size:
                raise InvalidArgument(f'sync write row for ID {servo_id} has {len(data)} bytes, expected {size}')
            body += bytes([servo_id]) + data
        request = framing.frame(0, BROADCAST, 0x83, body)
        self._send(request)
        self.sleep(len(request) * self._byte_s() + settle_s)
        waiting = self._waiting()
        if waiting:
            self._latch('unexpected bytes after Sync Write', hex=self._read(waiting).hex())

    def bulk_read(self, rows, timeout=None):
        """rows: [(id, address, size)] -> {id: bytes}"""
        if timeout is not None:
            _seconds(timeout, 'bulk read timeout')
        rows = [tuple(r) for r in rows]
        if any(len(r) != 3 for r in rows):
            raise InvalidArgument('bulk read rows are (id, address, size)')
        ids = _group_ids([r[0] for r in rows], 'bulk read')
        body = b''.join(_pack('<BHH', *r) for r in rows)
        self._send(framing.frame(0, BROADCAST, 0x92, body))
        timeout = timeout if timeout is not None else 0.05 + sum((11 + n + n // 3) * self._byte_s() + 0.0006 for _, _, n in rows)
        got = self._collect(ids, timeout)
        return {i: self._checked(got[i], n) for i, _, n in rows}

    def drain(self, seconds, quiet=0.05, cap=None):
        """A bounded window in which line activity is EXPECTED (a servo resetting: the retained
        loader prints its banner at 57600, which reads as 0x00 bytes at 1 Mbaud). Reads and logs
        everything without latching; returns the byte count. The window ends after `seconds` once
        the line has been quiet for `quiet`; still noisy at `cap` (default 3 x seconds) means the
        framing is unknown, so the Wire latches."""
        if self.uncertain:
            raise self._still_latched()
        start = self.clock()
        cap = start + (cap if cap is not None else 3 * seconds)
        last, n = start, 0
        while True:
            now = self.clock()
            if now - start >= seconds and now - last >= quiet:
                break
            if now >= cap:
                self._latch('line still noisy after the expected reset window')
            part = self._read(max(1, min(self._waiting(), 4096)))
            if part:
                n += len(part)
                last = self.clock()
                self._event('rx_drained', hex=part.hex())
        return n

    def reboot(self, servo_id, timeout=0.1, settle=0.3):
        """0x08 Reboot. FW231 resets back into the application. After the ACK the resetting servo may put
        loader output on the line, so the Wire drains for `settle` seconds instead of treating it as unknown
        framing. Returns the drained byte count. Send nothing else on the bus during that window. The servo
        restarts at 1 Mbaud (and moves to its start-up baud after 1.5 s unless it hears a 1-Mbaud frame); the
        host baud is NOT changed here, so on a bus running faster the caller must talk to it at 1 Mbaud inside
        that window."""
        _seconds(timeout, 'reboot timeout')
        _seconds(settle, 'reboot settle')
        self._send(framing.frame(0, _unicast_id(servo_id), 8))
        buf, deadline, st = bytearray(), self.clock() + timeout, None
        while st is None and self.clock() < deadline:
            part = self._read(max(1, min(self._waiting(), 4096)))
            if part:
                self._event('rx', hex=part.hex())
                buf.extend(part)
            st = self._take(buf) if len(buf) >= 7 else None
        if st is None:
            if buf:
                self._latch('partial reply to reboot')
            raise NoReply(f'no reply from ID {servo_id} to reboot: check that it is powered and connected, and its '
                          f'ID and baud', missing=[servo_id])
        if st['id'] != servo_id:
            self._latch(f"reboot reply from ID {st['id']}")
        self._checked(st, 0)
        if buf:
            self._event('rx_drained', hex=bytes(buf).hex())
        return len(buf) + self.drain(settle)

    def flash_read(self, servo_id, address, size):
        """A0: `size` bytes of flash at `address`, within the program (0x08004000..0x0800DFFF) and the servo's own
        calibration/NV journal (0x0800E000..0x0800FFFF). A range that touches the retained loader (0x08000000..
        0x08003FFF) is refused before anything is sent: this package never reads loader bytes back (read 1136 gives
        the loader's CRC-32)."""
        _check_flash_range(servo_id, address, size)
        if size and address < _LOADER_END:
            raise Refused(f'flash read of {size} B at 0x{address:08X} not sent: it touches the retained loader region '
                          f'0x08000000..0x08003FFF, which this package never reads (read 1136 gives its CRC-32)')
        return self._flash_read(servo_id, address, size)

    def _flash_read(self, servo_id, address, size):
        """flash_read without the loader-region refusal, for the installer only: the loader digest on images without
        read 1136 hashes this read in memory, with the wire log redacted (install.loader_crc32)."""
        _check_flash_range(servo_id, address, size)
        out = bytearray()
        while len(out) < size:
            n = min(96, size - len(out))
            self._send(framing.frame(0, servo_id, 0xA0, struct.pack('<IH', address + len(out), n)))
            out += self._checked(self._collect([servo_id], 0.1)[servo_id], n)
        return bytes(out)

    # ------------------------------------------------------------ loader
    def enter_loader(self, servo_id, app_baud=None):
        """F9 to one servo (stock or ours), then the retained loader handshake at 57600.
        Every other servo on the bus must stay in its application (its ID differs).
        Refused (before any TX) while this Wire may still have a servo in the loader: F9 acknowledged or a loader
        attached, and no F8 acknowledged since. app_baud is not used here: Loader.leave(app_baud) sets the host
        baud after the launch."""
        transition = self._f9(servo_id)
        self._port_baud(LOADER_BAUD)
        loader = Loader(self, transition)
        loader.handshake()
        self.loader = loader
        self.loader_phase = 'loader'
        return loader

    def _hand_to_loader(self, servo_id):
        """Experimental uninstall only: F9 to one servo, then wait at 57600 for the retained loader's banner WITHOUT
        answering it, so the loader stays at its first prompt for another program (ROBOTIS Dynamixel Wizard's Firmware
        Recovery). The Wire stays in phase 'f9_acked' (a servo is in the loader, waiting for '#'): nothing more is sent
        on it except a recovery with a fresh prompt."""
        transition = self._f9(servo_id)
        self._port_baud(LOADER_BAUD)
        Loader(self, transition)._wait_text(b'RDP : Yes\r\n', 256, prefix_ok=True)   # early bytes may be garbled

    def _f9(self, servo_id):
        """F9 to one servo and its acknowledgement (enter_loader's first half). Returns the bytes received after the
        acknowledgement (the start of the loader's banner, at the old baud)."""
        _unicast_id(servo_id)
        if self.loader_possible:
            raise Refused(f'not sent: a servo on this bus may still be in the retained loader (phase '
                          f'{self.loader_phase!r}: F9 sent or acknowledged, or a loader attached, and no F8 '
                          f'acknowledged since). Recover it first: {self._recover_command}')
        self._ready()
        self.loader_phase = 'f9_sent'                 # from here the servo may be in the loader, ACK or not
        self._send(framing.frame(0, servo_id, 0xF9), loader=True)
        buf, deadline = bytearray(), self.clock() + 0.3
        st = None
        while st is None and self.clock() < deadline:
            part = self._read(max(1, min(self._waiting(), 4096)))
            if part:
                self._event('rx', hex=part.hex())
                buf.extend(part)
            st = self._take(buf) if len(buf) >= 7 else None
        if st is None:
            self._latch('no F9 acknowledgement')
        if st['id'] != servo_id or st['error'] & 0x7F or st['data']:
            self._latch('wrong F9 acknowledgement')
        self.f9_acked = servo_id                      # from here the servo may be in the loader (receipts)
        self.loader_phase = 'f9_acked'
        return bytearray(buf)                         # banner bytes already received at the old baud


def _upload_plan(encoded):
    """(F7 requests, expected F7 progress, CRC-16) for these upload bytes, built here so that what is sent and checked
    is exactly the upload. InvalidArgument (nothing sent) when no F7 packets exist for them."""
    try:
        encoded = bytes(encoded)
        requests = tuple(packets.build_f7_upload_packets(encoded))
    except (TypeError, ValueError) as e:
        raise InvalidArgument(f'no F7 upload packets for these bytes ({e}): use a bundled release '
                              f'(images.Release(name).packaged()). Nothing was sent') from None
    count = (len(encoded) + 127) // 128
    progress = ((0, 0),) + tuple((i + 1, min((i + 1) * 128, len(encoded))) for i in range(count))
    return requests, progress, framing.crc16(encoded)


class Loader:
    """Retained ROBOTIS loader (version 6): frames use reserved 10, broadcast ID 254, replies ID 252.
    Only ONE servo may be in the loader on a shared bus (they all answer ID 254)."""

    def __init__(self, bus, transition=b''):
        self.bus, self.transition = bus, bytearray(transition)
        self.started = self.verified = False
        self._fa_length = None          # set by upload() for its own CRC check only (see _fa)

    def _wait_text(self, token, limit, timeout=0.7, prefix_ok=False):
        b = self.bus
        deadline = b.clock() + timeout
        while b.clock() < deadline:
            if token in self.transition:
                prefix, _, tail = self.transition.partition(token)
                if tail or b._waiting():
                    b._latch('bytes after loader token')
                if prefix and not prefix_ok:
                    b._latch('unexpected loader prefix ' + bytes(prefix).hex())
                if len(prefix) > limit:
                    b._latch('loader banner prefix too long')
                self.transition.clear()
                return
            if len(self.transition) > limit + len(token):
                b._latch('no loader token within the bounded banner')
            part = b._read(max(1, min(b._waiting(), 512)))
            if part:
                b._event('rx', hex=part.hex())
                self.transition.extend(part)
        b._latch('missing loader token ' + repr(token))

    def _prompt(self):
        b = self.bus
        b.loader_phase = 'loader_prompt_started'
        b._send(b'#', loader=True)
        self._wait_text(b'#', 0)
        b._send(b'\r', loader=True)
        self._wait_text(b'\r\n$', 0)
        b.loader_phase = 'loader_prompt_done'

    def handshake(self):
        b = self.bus
        self._wait_text(b'RDP : Yes\r\n', 256, prefix_ok=True)   # early banner bytes may be garbled
        self._prompt()
        if self._exchange(framing.frame(10, 254, 0xF1), 1)['data'] != b'\x06':
            b._latch('retained loader is not version 6')

    def attach(self, prompt_done=True):
        """Take over a servo that is ALREADY in the loader. prompt_done=True: the '#'/CR prompt
        already happened in this power cycle (failed FA / interrupted upload), so only F1 is
        sent. prompt_done=False: the loader is fresh (banner already gone) and waits for '#'.
        The two are never mixed: what the loader does with a frame before '#' is unproven."""
        b = self.bus
        if not prompt_done:
            self._prompt()
        if self._exchange(framing.frame(10, 254, 0xF1), 1)['data'] != b'\x06':
            b._latch('retained loader is not version 6')
        b.loader = self
        b.loader_phase = 'loader'

    def _exchange(self, request, size, timeout=0.7, allowed=(0,)):
        b = self.bus
        b._send(request, loader=True)
        st = b._collect([252], timeout, reserved=10)[252]
        if st['error'] not in allowed:
            b._latch(f"loader rejected with error {st['error']}")
        if len(st['data']) != size:
            b._latch('unexpected loader reply size')
        return st

    def upload(self, packaged, on_progress=None):
        """Send an upload: `packaged` has .encoded (the upload bytes), .packets (its F7 requests), .progress (the
        expected (index, bytes) per F7 reply) and .crc16 (CRC of .encoded), as images.Release.packaged() gives them for
        the bundled release. The packets, progress and CRC must be the ones .encoded gives; then the loader's CRC check
        over exactly the uploaded bytes must match before F8 is allowed. on_progress(done_bytes, total_bytes) after
        every F7 reply. An upload that would not fit the program region 0x08004000..0x0800DFFF, or whose packets do not
        match its bytes, is refused before any TX."""
        encoded = packaged.encoded
        size = len(encoded)
        if not 1 <= size <= _PROGRAM_BYTES:
            raise InvalidArgument(f'an upload of {size} B does not fit the program region 0x08004000..0x0800DFFF '
                                  f'({_PROGRAM_BYTES} B): this is not an upload for this servo; use a bundled release')
        requests, progress, crc = _upload_plan(encoded)
        if (tuple(packaged.packets) != requests or tuple(tuple(p) for p in packaged.progress) != progress
                or packaged.crc16 != crc):
            raise InvalidArgument('the upload\'s packets, progress or CRC do not match its bytes (nothing was sent): '
                                  'use a bundled release (images.Release(name).packaged())')
        self.started, self.verified, self._fa_length = True, False, None
        self.bus.loader_phase = 'uploading'
        for index, (request, expect) in enumerate(zip(requests, progress)):
            st = self._exchange(request, 6, 10 if index == 0 else 2)
            if struct.unpack('<HI', st['data']) != expect:
                self.bus._latch('F7 progress mismatch')
            if on_progress:
                on_progress(expect[1], size)
        self._fa_length = size                      # every F7 reply matched: the check may cover exactly these bytes
        value = self._fa(size, crc)
        if value != crc:
            self.bus._latch('FA CRC of the uploaded image differs')
        self.verified = True
        self.bus.loader_phase = 'uploaded'
        return value

    def _fa(self, length, expected):
        """FA: the loader's CRC over the `length` bytes it has just written (its status says whether it equals
        `expected`). Returns that CRC. Only upload() calls it, once, right after every F7 reply of its upload matched,
        over exactly that upload's length; any other use is Refused before anything is sent."""
        allowed, self._fa_length = self._fa_length, None
        if allowed is None or type(length) is not int or length != allowed:
            raise Refused('FA not sent: the loader\'s CRC check runs only inside upload(), once, over exactly the '
                          'upload it has just sent. Use Loader.upload()')
        if type(expected) is not int or not 0 <= expected <= 0xFFFF:
            raise InvalidArgument(f'FA check against CRC {expected!r}: use a CRC-16 (0..0xFFFF). Nothing was sent')
        request = framing.frame(10, 254, 0xFA, struct.pack('<IH', length, expected))
        st = self._exchange(request, 2, 3, allowed=(0, 245))
        value = int.from_bytes(st['data'], 'little')
        if st['error'] != (0 if value == expected else 245):
            self.bus._latch('FA status disagrees with its CRC')
        return value

    def leave(self, app_baud=APP_BAUD):
        """F8: launch the application. Refused (before any TX) after a partial/unverified upload."""
        if self.started and not self.verified:
            raise Refused(f'F8 (launch) not sent: the upload was partial or not verified, so the program in flash is '
                          f'incomplete. Upload the image again with this Loader, or recover the servo: '
                          f'{self.bus._recover_command}')
        b = self.bus
        b.loader_phase = 'f8_sent'
        self._exchange(framing.frame(10, 254, 0xF8), 0, 2)
        # F8 acknowledged: the loader launches the application, so nothing on this Wire is in the loader any more
        # (a stale f9_acked would make every later failure on the same Wire read in_loader=True).
        b.loader, b.f9_acked, b.loader_phase = None, None, 'launched'
        b.sleep(0.25)
        b._port_baud(app_baud)
