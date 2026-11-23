"""The bus: one serial adapter and the servos on it. Bus opens the port, finds and identifies servos, and hands out
Servo and Group objects; its low-level methods are plain Protocol 2.0 (no promises for servos that do not run
openxl330 firmware).

    bus = Bus("/dev/ttyUSB0")            # 1 Mbaud; drains stale bytes; checks the adapter
    bus.scan()                           # [ServoInfo(...)]; NoServosFound suggests scan(all_bauds=True)
    arm = bus.group([1, 2, 3])           # refuses servos without supported openxl330 firmware
    s = bus.servo(3)

v1 supports buses where every servo runs openxl330 firmware (FW231): scan() lists the others as not supported, and
servo()/group() refuse them. Each servo is identified once per Bus (ping, marker, ABI, release): read 1120 names the
release (a bundled one by name), else read 1112 tells release 11 from release 10 or older.
"""
from __future__ import annotations

import dataclasses
import errno
import struct
import warnings
from dataclasses import dataclass
from pathlib import Path

from . import adapters, images
from .calibration import Calibration
from .errors import (PROTOCOL_ERRORS, AdapterError, AdapterWarning, Error, InvalidArgument, NoReply, NoServosFound,
                     PortBusy, Refused, Rejected, UnsupportedServo)
from .group import Group
from .install import update_command
from .registers import (BAUD, BUSDIAG, DEVBLOCK, FW231_MARKER, LAST_RESET, MARKER, MIN_ABI, RELEASE_ID,
                        decode_release_id, parse_devcfg)
from .servo import Servo, ids_text
from .wire import APP_BAUD, SYNC_WRITE_SETTLE_S, Wire, WireLog

MAX_ABI = 4                                     # the newest register map this version knows
FW231 = 231
SUPPORTED_MODEL = 1200
MODEL_NAMES = {1200: 'XL330-M288', 1190: 'XL330-M077', 1060: 'XL430-W250'}
BUS_BAUDS = (1_000_000, 2_000_000, 3_000_000)   # what set_baud() can switch FW231 servos to
# scan(all_bauds=True), current baud first. Not 9600: a ping reply there outlasts the 3 ms ID slot, so replies collide.
SCAN_BAUDS = (1_000_000, 2_000_000, 3_000_000, 57_600, 115_200)
CONFIRM_WAIT = 2.2                              # s: a servo that missed the switch reverts after 2 s
PORT_TIMEOUT = 0.002


@dataclass(frozen=True)
class ServoInfo:
    """What a servo is. model: 'XL330-M288' (or 'model N'); firmware: 'FW231', 'stock V52' or 'unknown'; supported:
    usable through servo()/group(); features: any of 'motion', 'fast_sync_read', 'last_reset', 'release_id',
    'torque_command' (release 16+ / ABI 4: Servo.set_torque); release:
    a bundled release's name, 'FW231 release N', 'FW231 (release 11)', 'FW231 (release 10 or older)', or None
    (not FW231); label: the device-block label an earlier tool may have written (normally ''); baud: where it
    answered. abi: reported register-map version when the openxl330 marker and ABI header were readable."""
    id: int
    model: str
    firmware: str
    supported: bool
    features: frozenset = frozenset()
    release: str | None = None
    label: str = ''
    baud: int = APP_BAUD
    abi: int | None = None


def _baud_text(bps):
    return f'{bps // 1_000_000} Mbaud' if bps % 1_000_000 == 0 else f'{bps} baud'


def _model_name(model):
    if model is None:
        return 'unknown'
    return MODEL_NAMES.get(model, f'model {model}')


def _check_id(servo_id):
    if isinstance(servo_id, bool) or not isinstance(servo_id, int) or not 0 <= servo_id <= 252:
        raise InvalidArgument(f'servo ID {servo_id!r}: use an integer 0..252')
    return servo_id


def _check_ids(ids):
    if isinstance(ids, (str, bytes)) or not hasattr(ids, '__iter__'):
        raise InvalidArgument(f'IDs {ids!r}: give a list of servo IDs, for example [1, 2, 3]')
    ids = [_check_id(i) for i in ids]
    if not ids:
        raise InvalidArgument('IDs: give at least one servo ID')
    if len(set(ids)) != len(ids):
        raise InvalidArgument(f'IDs {ids}: list each servo once')
    return ids


def _release_name(rid):
    """A bundled release's name when all fields from read 1120 match its manifest."""
    for name in images.RELEASES:
        try:
            bundled_id = images.bundled(name).release_id
        except Error:
            continue
        if bundled_id == rid:
            return name
    return f'FW231 release {rid["number"]}'


def _calibration(calibration):
    if calibration is None:
        return Calibration.load()
    if isinstance(calibration, Calibration):
        return calibration
    if isinstance(calibration, (str, Path)):
        return Calibration.load(calibration)
    raise InvalidArgument(f'calibration {calibration!r}: give a Calibration, a path to a calibration file, or None '
                          f'(the default file)')


def _open_error(name, e):
    """The openxl330 error for a port that would not open (no pyserial traceback)."""
    text, code = str(e), getattr(e, 'errno', None)
    low = text.lower()
    if code in (errno.EBUSY, errno.EAGAIN) or 'busy' in low or 'exclusively lock' in low or 'access is denied' in low:
        return PortBusy(f'{name} is busy: another program has the port open (Dynamixel Wizard, a serial monitor, '
                        f'another script or Bus). Close it, then open the Bus again ({text})')
    if code == errno.ENOENT or 'no such file' in low or 'cannot find the file' in low:
        return AdapterError(f'{name} does not exist: check that the adapter is plugged in and the port name; '
                            f'Bus.ports() (CLI: openxl330 ports) lists the serial ports ({text})')
    if code == errno.EACCES or 'permission denied' in low:
        return AdapterError(f'no permission to open {name}: on Linux add your user to the dialout group (sudo usermod '
                            f'-aG dialout $USER, then log in again) ({text})')
    return AdapterError(f'cannot open {name} ({text}): check the adapter and the port name; Bus.ports() lists the '
                        f'serial ports')


def _open_serial(name, baud):
    import serial
    try:
        return serial.Serial(name, baudrate=baud, timeout=PORT_TIMEOUT, exclusive=True)
    except Exception as e:                      # pyserial raises SerialException (an OSError), or ValueError
        raise _open_error(name, e) from None


class Bus:
    """One adapter and the servos on it.

    port: a port name (opened with pyserial, exclusively, timeout 2 ms: PortBusy if another program has it) or a
    pyserial-like object (write/read/in_waiting/flush/baudrate), such as sim.SimPort; a port that brings its own clock
    (a callable `clock` with a `sleep` method, as SimPort does) also drives the Wire's time. baud: the host baud.
    calibration: a Calibration, a path to a calibration file, or None ($OPENXL330_HOME/calibration.json; missing =
    defaults). log: False (no files), True (a wire log under $OPENXL330_HOME/logs) or a callable(dict).
    check_adapter: at open, ping ID 253 to detect an echoing adapter (AdapterEchoes) and warn (AdapterWarning) about
    a slow FTDI latency timer. Stale input is always drained at open."""

    def __init__(self, port, baud: int = APP_BAUD, *, calibration=None, log=False, check_adapter: bool = True):
        if isinstance(baud, bool) or not isinstance(baud, int) or baud <= 0:
            raise InvalidArgument(f'baud {baud!r}: use a positive integer, for example 1000000')
        self.calibration = _calibration(calibration)
        opened = isinstance(port, str)
        self.port = _open_serial(port, baud) if opened else port
        self._opened, self._probed, self._wire, self._own_log = opened, {}, None, log is True
        self._identity_generation = 0
        try:
            clock = getattr(self.port, 'clock', None)
            if callable(clock) and callable(getattr(clock, 'sleep', None)):
                self._wire = Wire(self.port, log=log, clock=clock, sleep=clock.sleep)
            else:
                self._wire = Wire(self.port, log=log)
            if self.port.baudrate != baud:
                self._wire.set_baud(baud)
            self._wire.drain_stale()
            if check_adapter:
                adapters.check_echo(self._wire)
                note = adapters.ftdi_latency_warning(port) if opened else None
                if note:
                    warnings.warn(note, AdapterWarning, stacklevel=2)
        except BaseException as error:
            if opened:
                self._close_after_error(error)
            raise

    @classmethod
    def simulated(cls, n: int = 3, *, calibration=None) -> Bus:
        """A Bus on a simulated bus of `n` servos running the bundled default release at IDs 1..n (no hardware).
        Its clock follows real time, so time.sleep between commands lets the simulated servos move. calibration=None
        gives an in-memory Calibration (no file is read or written)."""
        if isinstance(n, bool) or not isinstance(n, int) or not 1 <= n <= 252:
            raise InvalidArgument(f'simulated bus of {n!r} servos: use 1..252')
        from . import sim
        program = images.bundled(images.DEFAULT_RELEASE).packaged().raw
        servos = [sim.SimServo(i, mode='app', program=program, seed=i) for i in range(1, n + 1)]
        return cls(sim.SimPort(servos, sim.SimClock()),
                   calibration=calibration if calibration is not None else Calibration())

    @staticmethod
    def ports() -> list:
        """Serial port names on this computer (pyserial's list)."""
        try:
            from serial.tools import list_ports
            return sorted(p.device for p in list_ports.comports())
        except Exception:
            return []

    def __repr__(self):
        name = getattr(self.port, 'port', None) or type(self.port).__name__
        return f'Bus({name!r}, baud={self.port.baudrate})'

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        if exc is None:
            self.close()
        else:
            self._close_after_error(exc)
        return False

    def _close_after_error(self, error):
        try:
            self.close()
        except Exception as close_error:
            error.close_error = close_error
            if hasattr(error, "add_note"):
                error.add_note(f"openxl330 also failed to close the bus: {close_error}")

    @property
    def wire(self) -> Wire:
        """The strict Protocol 2.0 client underneath (for install and low-level use)."""
        return self._wire

    def close(self):
        """Close the port (nothing is sent: servos keep doing what they do) and the wire log this Bus opened
        (log=True)."""
        try:
            try:
                close = getattr(self.port, 'close', None)
                if close is not None:
                    close()
            finally:
                log = self._wire.log if self._wire is not None else None
                if self._own_log and isinstance(log, WireLog):
                    log.close()
        except OSError as exc:
            raise AdapterError(f'could not close the bus: {exc}') from exc

    def reset(self):
        """Clear the uncertain latch and cached device identities once the line is quiet."""
        got = self._wire.reset()
        self._invalidate_identities()
        return got

    def _invalidate_identities(self, baud=None):
        if baud is None:
            self._probed.clear()
        else:
            self._probed = {key: info for key, info in self._probed.items() if key[0] != baud}
        self._identity_generation += 1

    # ------------------------------------------------------------ finding and identifying servos
    def scan(self, all_bauds: bool = False) -> list:
        """[ServoInfo] of every servo answering a broadcast ping at the current baud (all 253 ID slots), each
        identified. Nothing found: NoServosFound, which suggests all_bauds=True. all_bauds=True also tries
        1M, 2M, 3M, 57600 and 115200 baud (about 0.8 s each), then returns the host to its baud."""
        if not isinstance(all_bauds, bool):
            raise InvalidArgument(f'all_bauds must be True or False, not {all_bauds!r}')
        here = self.port.baudrate
        if not all_bauds:
            found = self._scan_here()
            if not found:
                raise NoServosFound(f'no servos at {_baud_text(here)}; try scan(all_bauds=True) / `openxl330 scan '
                                    f'--all-bauds` in case they run at another baud, and check power and wiring')
            return found
        bauds = [here] + [b for b in SCAN_BAUDS if b != here]
        found = []
        try:
            for bps in bauds:
                self._wire.set_baud(bps)
                found += self._scan_here()
        except BaseException:
            try:                                             # back to where it was; this never hides the error
                self._wire.restore_baud(here)
            except Exception:
                pass
            raise
        self._wire.set_baud(here)
        if not found:
            raise NoServosFound(f'no servos answered at any baud ({", ".join(_baud_text(b) for b in bauds)}): check '
                                f'that they are powered (the adapter alone does not power them) and wired, and that '
                                f'this is the right port')
        return found

    def _scan_here(self):
        baud, out = self.port.baudrate, []
        found = self._wire.scan(252)
        self._invalidate_identities(baud)
        for servo_id in sorted(found):
            self._probed[(baud, servo_id)] = self._identify(servo_id, found[servo_id])
            out.append(self._probed[(baud, servo_id)][0])
        return out

    def _optional(self, servo_id, register):
        """A read that an older image may refuse: its bytes, or None when the servo answers with an error."""
        try:
            return self._wire.read(servo_id, *register)
        except Rejected:
            return None

    def _identify(self, servo_id, ping):
        """(ServoInfo, refusal text or None) from a ping reply plus, for FW231, marker, ABI, release and label."""
        baud = self.port.baudrate
        model_no, fw = ping.get('model'), ping.get('firmware')
        model = _model_name(model_no)
        if ping.get('error'):
            meaning = PROTOCOL_ERRORS.get(ping['error'], f'unknown error {ping["error"]}')
            return (ServoInfo(servo_id, model, 'unknown', False, baud=baud),
                    f'ID {servo_id} answers a ping with error {ping["error"]} ({meaning}), so it is not a working '
                    f'openxl330 servo: check it on its own adapter')
        if fw != FW231:
            stock = fw is not None and fw < 200            # ROBOTIS firmware versions are small numbers (V50, V52)
            info = ServoInfo(servo_id, model, f'stock V{fw}' if stock else 'unknown', False, baud=baud)
            what = f'stock firmware V{fw}' if stock else f'firmware version {fw}, not openxl330 firmware'
            advice = ('put stock servos on a separate adapter, or install with `openxl330 install`'
                      if model_no == SUPPORTED_MODEL else
                      'put it on a separate adapter (openxl330 firmware runs on the XL330-M288 only)')
            return info, (f'ID {servo_id} runs {what} ({model}): v1 supports buses where every servo runs openxl330 '
                          f'firmware; {advice}')
        if model_no != SUPPORTED_MODEL:
            return (ServoInfo(servo_id, model, 'FW231', False, baud=baud),
                    f'ID {servo_id} reports FW231 on {model}, but openxl330 supports the XL330-M288 only')
        marker = self._optional(servo_id, MARKER)
        if marker is None or not marker.startswith(FW231_MARKER):
            return (ServoInfo(servo_id, model, 'unknown', False, baud=baud),
                    f'ID {servo_id} reports firmware 231 but not the openxl330 marker, so it is not openxl330 firmware: '
                    f'put it on a separate adapter')
        raw = self._optional(servo_id, BUSDIAG)
        if raw is None:
            return (ServoInfo(servo_id, model, 'FW231', False, baud=baud),
                    f'ID {servo_id} rejected the ABI header read, so its firmware layout cannot be verified')
        abi = raw[3]    # ABI selector in the first word; decode no other fields until the layout is known.
        command = update_command(self._wire, servo_id)
        if abi < MIN_ABI:
            info = ServoInfo(servo_id, model, 'FW231', False, baud=baud, abi=abi)
            return info, (f'ID {servo_id} runs FW231 with ABI {abi}, older than this openxl330 supports (ABI '
                          f'{MIN_ABI}..{MAX_ABI}): update it with `{command}`')
        if abi > MAX_ABI:
            info = ServoInfo(servo_id, model, 'FW231', False, baud=baud, abi=abi)
            return info, (f'ID {servo_id} runs FW231 with ABI {abi}, newer than this openxl330 knows (ABI '
                          f'{MIN_ABI}..{MAX_ABI}): upgrade openxl330 (pip install --upgrade openxl330), or install the '
                          f'release this version bundles with `{command}`')
        features = {'motion', 'fast_sync_read'}
        rid = self._optional(servo_id, RELEASE_ID)
        if rid is not None:
            rid = decode_release_id(rid)
            if rid['abi'] != abi:
                return (ServoInfo(servo_id, model, 'FW231', False, baud=baud, abi=abi),
                        f"ID {servo_id} ABI fields disagree (bus diagnostics {abi}, release ID {rid['abi']}). "
                        f'Check the running firmware before commanding this servo')
            features.add('release_id')
            if rid['number'] >= 11:
                features.add('last_reset')
            if rid['number'] >= 16 or abi >= 4:
                features.add('torque_command')
            release = _release_name(rid)
        elif self._optional(servo_id, LAST_RESET) is not None:
            features.add('last_reset')
            release = 'FW231 (release 11)'
        else:
            release = 'FW231 (release 10 or older)'
        label = parse_devcfg(self._optional(servo_id, DEVBLOCK) or b'')['label']
        return ServoInfo(servo_id, model, 'FW231', True, frozenset(features), release, label, baud, abi), None

    def _probe(self, servo_id):
        """(ServoInfo, refusal) for one ID at the current baud, probed once per Bus (scan() fills the same cache)."""
        key = (self.port.baudrate, servo_id)
        if key not in self._probed:
            try:
                ping = self._wire.ping(servo_id)
            except Rejected as e:
                ping = dict(model=None, firmware=None, alert=False, error=e.error)
            self._probed[key] = self._identify(servo_id, ping)
        return self._probed[key]

    def _checked_probe(self, servo_id):
        _check_id(servo_id)
        try:
            return self._probe(servo_id)
        except NoReply:
            raise NoReply(f'no servo answers at ID {servo_id} ({_baud_text(self.port.baudrate)}): check its power, '
                          f'wiring and ID; scan() lists what answers', missing=[servo_id]) from None

    def _revalidate_handle(self, previous):
        current, refusal = self._checked_probe(previous.id)
        if refusal:
            raise UnsupportedServo(refusal)
        if previous.baud != current.baud:
            previous = dataclasses.replace(previous, baud=current.baud)
        if current != previous:
            raise Refused(f'ID {previous.id} identity changed. Check the bus and obtain a new Servo or Group')
        return current

    def servo_info(self, servo_id) -> ServoInfo:
        """Read-only identity of one responder, including devices outside the supported model or ABI."""
        return self._checked_probe(servo_id)[0]

    def servo(self, servo_id) -> Servo:
        """The servo at `servo_id`. UnsupportedServo if it does not run supported openxl330 firmware; NoReply if
        nothing answers there."""
        info, refusal = self._checked_probe(servo_id)
        if refusal:
            raise UnsupportedServo(refusal)
        return Servo(self, info)

    def group(self, ids, off_on_exit: bool = False) -> Group:
        """A Group of these servos, in this order (the order of group reads). Every ID is probed (once per Bus): a
        servo without supported openxl330 firmware raises UnsupportedServo, a missing one NoReply. off_on_exit=True
        turns them OFF when a `with` block ends (default False: nothing is turned off by itself)."""
        ids = _check_ids(ids)
        if not isinstance(off_on_exit, bool):
            raise InvalidArgument(f'off_on_exit must be True or False, not {off_on_exit!r}')
        infos, missing, refusals = {}, [], []
        for servo_id in ids:
            try:
                info, refusal = self._probe(servo_id)
            except NoReply:
                missing.append(servo_id)
                continue
            if refusal:
                refusals.append(refusal)
            infos[servo_id] = info
        if missing:
            raise NoReply(f'no reply from {ids_text(missing)} ({_baud_text(self.port.baudrate)}): check their power, '
                          f'wiring and IDs; scan() lists what answers', infos, missing)
        if refusals:
            raise UnsupportedServo('; '.join(refusals))
        return Group(self, [infos[i] for i in ids], off_on_exit)

    def set_baud(self, bps: int, ids=None) -> dict:
        """Switch FW231 servos and the host to `bps` (1, 2 or 3 Mbaud).

        Every source responder must be supported (UnsupportedServo otherwise) and its ID unused at the destination
        (Refused otherwise) before the broadcast write.
        `ids` controls which source IDs are confirmed and reported (default: all source responders). Returns
        dict(baud, confirmed, missing, previous). If nothing answers at the source, no write or baud change occurs.
        After a write the host stays at the new baud. An unconfirmed servo may be at either baud."""
        if isinstance(bps, bool) or not isinstance(bps, int) or bps not in BUS_BAUDS:
            raise InvalidArgument(f'baud {bps!r}: FW231 runs at 1000000, 2000000 or 3000000 (an integer)')
        wire, old = self._wire, self.port.baudrate
        expected = _check_ids(ids) if ids is not None else None
        census = wire.scan(252)
        self._invalidate_identities(old)
        expected = sorted(census) if expected is None else expected
        if not census:
            return dict(baud=old, confirmed=[], missing=expected, previous=old)
        source = {}
        for servo_id in sorted(census):
            info, refusal = self._identify(servo_id, census[servo_id])
            self._probed[(old, servo_id)] = (info, refusal)
            if refusal:
                raise UnsupportedServo(f'baud change refused before writing: {refusal}')
            source[servo_id] = info
        if bps == old:
            return dict(baud=bps, confirmed=[i for i in expected if i in source],
                        missing=[i for i in expected if i not in source], previous=old)
        try:
            wire.set_baud(bps)
            destination = wire.scan(252)
        except BaseException:
            try:
                wire.restore_baud(old)
            except Exception:
                pass
            raise
        wire.set_baud(old)
        self._invalidate_identities(bps)
        overlap = sorted(source.keys() & destination.keys())
        if overlap:
            raise Refused(f'baud change refused: {_baud_text(bps)} already has {ids_text(overlap)} from the source '
                          f'bus. Nothing was written: isolate the devices and give them unique IDs before merging')

        # All source servos hear the broadcast, including those omitted from the confirmation list.
        for servo_id in source:
            self._probed.pop((old, servo_id), None)
        wire.write_broadcast(BAUD, struct.pack('<I', bps))
        wire.sleep(0.005)
        wire.set_baud(bps)
        found = wire.scan(252)
        confirmed, missing = [], []
        for servo_id in expected:
            if servo_id in source and servo_id in found and not found[servo_id].get('error'):
                try:
                    word = struct.unpack('<I', wire.read(servo_id, BAUD, 4))[0]
                except (NoReply, Rejected):
                    missing.append(servo_id)
                    continue
                (confirmed if word == bps else missing).append(servo_id)
            else:
                missing.append(servo_id)
        if missing:
            wire.sleep(CONFIRM_WAIT)
        for servo_id in confirmed:
            self._probed[(bps, servo_id)] = (dataclasses.replace(source[servo_id], baud=bps), None)
        return dict(baud=bps, confirmed=confirmed, missing=missing, previous=old)

    # ------------------------------------------------------------ low level (generic Protocol 2.0, raw bytes)
    def ping(self, servo_id, timeout=0.1) -> dict:
        return self._wire.ping(servo_id, timeout)

    def read(self, servo_id, address, size, timeout=0.1) -> bytes:
        return self._wire.read(servo_id, address, size, timeout)

    def write(self, servo_id, address, data, timeout=0.1) -> bool:
        """Returns True when the reply carried the alert bit (a fault is latched)."""
        return self._wire.write(servo_id, address, data, timeout)

    def sync_read(self, ids, address, size, timeout=None) -> dict:
        return self._wire.sync_read(ids, address, size, timeout)

    def fast_sync_read(self, ids, address, size, timeout=None) -> dict:
        return self._wire.fast_sync_read(ids, address, size, timeout)

    def sync_write(self, address, size, rows, *, settle_s=SYNC_WRITE_SETTLE_S):
        return self._wire.sync_write(address, size, rows, settle_s=settle_s)

    def bulk_read(self, rows, timeout=None) -> dict:
        return self._wire.bulk_read(rows, timeout)

    def reboot(self, servo_id, timeout=0.1, settle=0.3) -> int:
        """0x08 Reboot (see Wire.reboot): returns the bytes drained while the servo restarted."""
        return self._wire.reboot(servo_id, timeout, settle)
