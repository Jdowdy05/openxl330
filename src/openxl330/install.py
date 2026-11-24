"""Install the bundled FW231 firmware on XL330-M288 servos through the retained ROBOTIS loader, one servo at a time.
Opens no port: every function takes a Wire on a port the caller opened.

  from_stock(wire, stock_id=4)   a factory servo still running ROBOTIS firmware
  update(wire, servo_id=4)       a servo already on FW231
  recover(wire, servo_id=4)      the one servo left waiting in the retained loader

Every servo gets the same pre-encoded image (images.Release.packaged), so nothing is personalised: the firmware
answers at the ID the servo had on its stock firmware (its NV journal ID) and starts at 1 Mbaud. Give each servo a
unique ID in ROBOTIS Dynamixel Wizard, on its stock firmware, BEFORE its first install; an install
does not change IDs (from_stock refuses an ID that another servo on the bus already uses); set_id() does, on
release 17 and later (register 1144), as a recorded run of its own.

Invariants:
  * only the program region 0x08004000..0x0800DFFF is ever written (the bundled upload fits it, Loader.upload
    refuses anything larger, and the retained loader writes from 0x08004000); no chip erase, no option bytes;
  * the retained loader (0x08000000..0x08003FFF) and the servo's own calibration/NV journal (0x0800E000..0x0800FFFF)
    are never written: update() reads the loader's CRC-32 and the calibration bytes before F9 and again after the
    launch, and they must be unchanged (a change within one install is an unexpected error); recover() compares the
    calibration with the servo's record; a first install from stock (stock firmware cannot read flash) records them
    after the launch and compares the stock EEPROM view instead;
  * every install appends the loader's CRC-32 to the servo's loader history and compares it with the list of known
    loaders (PROVEN_LOADERS, from the bundled manifest) and with the servo's last recorded install. An unknown loader
    or one that changed since the last install is a WARNING only (receipt['warnings'], also returned in the receipt):
    the install goes on;
  * every installed program is read back over A0 and must equal the image byte for byte (not only the loader's CRC),
    and the servo must report the release ID (read 1120) its manifest names;
  * every refusal happens before F9 (the request that sends a servo into the loader);
  * unknown framing latches the Wire (no further transmission);
  * the loader's bytes are never stored or logged: only their CRC-32 is kept;
  * only FW231 is ever installed: this module never installs stock firmware (the way back to stock is the
    experimental route through ROBOTIS Dynamixel Wizard, openxl330.experimental).

Records go to records_root() ($OPENXL330_HOME/records) unless `records=` names another folder:
  _runs/<UTC>-<run>/receipt.json, wire.jsonl   one folder per run (schema xlbus-install-v1), written as it happens
  <fingerprint>/nvm.bin                        the servo's own calibration/NV journal (local backup, never overwritten;
                                               a different later copy is kept beside it)
  <fingerprint>/loader.crc32                   the loader's CRC-32 history: one line '<crc32> <utc>' per install
  <fingerprint>/history.jsonl                  one line per run
The fingerprint is the first 16 hex digits of the SHA-256 of the servo's factory calibration page (0x0800E000..
0x0800EFFF), so a servo keeps its history across runs. Receipts are meant to be shareable: the paths in them (run_dir,
servo_dir, failed_run) are relative to that records folder.

Failures: every failure is an openxl330.Error. Bad arguments raise InvalidArgument before anything is sent. A
failure during a run raises InstallError (`.step`, `.receipt` = the local path of its receipt). Its `.recovery` is
the exact command to run next, which is also in the message and the receipt:
  * while the servo may be in the retained loader (F9 may have reached it, F8 not acknowledged):
    `openxl330 recover --port <port> --id <id>`, with `--loader-fresh` when the prompt was incomplete;
  * when recover() itself gets no answer from the loader: `openxl330 scan --port <port> --all-bauds`; the message
    adds the recover command to use instead if the servo was (or, after --loader-fresh, was not) power-cycled since
    the failed install, so the command that just failed is never offered again;
  * once F8 was acknowledged (the servo runs the new firmware): `openxl330 update --port <port> --id <id>`, except
    for the unexpected failures (loader, calibration or release ID not as they must be), whose message says to share
    the receipt instead (`.recovery` None), and a servo that does not answer at that ID: `openxl330 scan --port
    <port>` (it answers at its stock NV ID);
  * before F9: None (nothing was installed).
UntestedServo / UnsupportedServo are the servo-policy refusals (before F9). A KeyboardInterrupt (Ctrl-C) is
re-raised as itself, not wrapped, so Ctrl-C still stops the program; it carries the same `.recovery`, `.step` and
`.receipt` attributes (and, on Python 3.11+, a note with the command), and the Wire is latched.

In v1 every servo gets one identical pre-encoded image: no new ID, label or start-up baud is written, and
update(allow_id_change=True) accepts the move of a servo whose device block (written by an earlier tool) gave it
another ID back to its NV ID. Only the loader's CRC-32 is kept, never its bytes; the release ID (read 1120) is checked;
stock firmware outside the proven versions needs allow_untested=True; once F8 was acknowledged the way on is update,
not recover.
"""
from __future__ import annotations

import hashlib
import json
import os
import struct
import weakref
import zlib
from datetime import datetime, timezone
from functools import lru_cache
from pathlib import Path

from . import images, models, paths
from .errors import (BusUncertain, Error, InstallError, InvalidArgument, NoReply, NotSupported, Refused, Rejected,
                     UnsupportedServo, UntestedServo)
from .images import DEFAULT_RELEASE, PROGRAM_BASE, Release
from .registers import (BAUD, BUSDIAG, DEVBLOCK, FW231_MARKER, LOADER_DIGEST, MARKER, RELEASE_ID, SET_ID,
                        SET_ID_QUIET_S, SET_ID_REPLY_S, STATUS, TORQUE_ENABLE, decode_busdiag, decode_id_log, decode_release_id,
                        decode_status, encode_set_id, parse_devcfg)
from .wire import APP_BAUD, LOADER_BAUD, Loader, WireLog

# ---------------------------------------------------------------- policy tables (one place; releases extend them)
SUPPORTED_MODEL = models.M288.number               # XL330-M288: the only model a stock bystander may be
PROVEN_STOCK = {(m.number, v) for m in models.MODELS.values() for v in m.proven_stock}   # proven by a first install

SCHEMA = 'xlbus-install-v1'
FW231 = 231
LOADER_ADDR, LOADER_BYTES = 0x08000000, 16384
NVM_ADDR, NVM_BYTES = 0x0800E000, 8192
CALIBRATION_BYTES = 4096                            # the factory calibration page; the rest of NVM is the NV journal
BYSTANDER_BYTES = 120                               # stock control table 0..119 (EEPROM + RAM settings); 120.. is live
BYSTANDER_STRICT = 65                               # EEPROM 0..63 and torque enable 64 must not change; 65..119 may
# (model, stock version) proven to ignore another servo's whole loader session at 57600 (bench 2026-11-23: one V53)
PROVEN_BYSTANDERS = {(1200, 53)}
LOADER_DIGEST_TIMEOUT = 0.25                        # read 1136: fast13+ caches the CRC at boot; fast12
                                                    # computes it on request (~12.7 ms measured)
CENSUS_BAUDS = (LOADER_BAUD, APP_BAUD, 2_000_000, 3_000_000)   # censused before F9
BYSTANDER_CENSUS_BAUDS = CENSUS_BAUDS + (115_200,)               # flash-all (experimental): also 115200
STOCK_BAUDS = (9600, 57_600, 115_200, 1_000_000, 2_000_000, 3_000_000, 4_000_000, 4_500_000)
# Stock EEPROM bytes FW231 reports differently on purpose: 6 firmware version, 8 live baud index (1 Mbaud = 3). Byte 7
# (the ID) must match: the image keeps the stock NV journal ID.
EEPROM_OURS = {6, 8}

_STEPS = {
    'precheck': 'checking the bus', 'census': 'the census of the bus', 'identify': 'identifying the servo',
    'census_other_bauds': 'the census at the other bauds', 'backup': 'backing up the calibration',
    'enter_loader': 'entering the retained loader (F9)', 'attach': 'attaching to the retained loader',
    'upload': 'uploading the firmware', 'launch': 'launching the firmware (F8)', 'verify': 'verifying the install',
    'census_after': 'the census after the install', 'nv_journal': 'comparing the NV journal',
    'records': 'writing the records', 'receipt': "reading the failed run's receipt",
    'loader_digest': 'reading the loader digest', 'bystanders': 'checking the other stock servos',
    'plan': 'planning the installs', 'handoff': 'handing the servo to the retained loader',
    'restore': 'restoring the stock settings', 'set_id': 'changing the ID', 'reapply_id': 're-applying the ID',
}
ISSUES_URL = 'https://github.com/Jdowdy05/openxl330/issues'   # where users share receipts (no personal contact)
_IN_LOADER = ' The servo may still be waiting in the retained loader.'
_LAUNCHED = (' F8 was acknowledged, so the servo has left the loader and runs the new program: re-install it with '
             'update (not recover).')
_UNEXPECTED = (' This is unexpected: the installer never writes the retained loader or the calibration/NV journal. '
               'F8 was acknowledged, so the servo runs the new program. Do not rely on this servo yet, and please '
               'share this run\'s receipt (receipt.json) with the openxl330 maintainers at ' + ISSUES_URL + '.')
_RELEASE_DIFFERS = (' The program bytes on the servo equal the bundled image, so the package\'s image and manifest '
                    'disagree: re-installing would not change this. Reinstall openxl330, do not rely on this servo '
                    'yet, and please share this run\'s receipt (receipt.json) with the openxl330 maintainers at ' + ISSUES_URL + '.')
_WIZARD = ('Give each servo a unique ID in ROBOTIS Dynamixel Wizard (on its stock firmware, connected alone) before '
           'its first install: the installed firmware keeps the ID the servo has on its stock firmware.')
_LOADER_READ_FAILED = ('the loader digest read over A0 failed ({kind}); the reply bytes are not shown because they '
                       'would be loader code')
_pending = weakref.WeakKeyDictionary()              # Wire -> the ID of the servo an unfinished loader session flashed


def records_root() -> Path:
    """Where install records go by default: $OPENXL330_HOME/records (see paths.home)."""
    return paths.home() / 'records'


def _port_name(wire):
    return getattr(wire.port, 'port', None) or '<port>'


def scan_command(wire) -> str:
    """The command that lists the servos on this Wire's port (for a servo that does not answer where expected)."""
    return f'openxl330 scan --port {_port_name(wire)}'


def recovery_command(wire, servo_id, *, loader_fresh=False) -> str:
    """The recovery command for a servo already at the loader, with a fresh prompt when requested."""
    command = f'openxl330 recover --port {_port_name(wire)} --id {servo_id}'
    return command + ' --loader-fresh' if loader_fresh else command


def _loader_recovery(wire, servo_id):
    phase = wire.loader_phase
    fresh = phase in ('f9_sent', 'f9_acked', 'recovering_fresh', 'loader_prompt_started')
    if phase == 'loader_prompt_started':
        note = (' The loader prompt was interrupted. Isolate this servo and power-cycle it. Use the command below '
                'only if the servo starts at the fresh loader prompt.')
    elif phase == 'f9_sent':
        note = (' Loader entry was not acknowledged. Use the command below only if the servo is waiting at the '
                'fresh loader prompt.')
    elif fresh:
        note = ' The servo is waiting for the initial loader prompt.'
    else:
        note = ''
    return recovery_command(wire, servo_id, loader_fresh=fresh), note


def _attach_failed(wire, servo_id, loader_fresh):
    """(recovery command, note) when recover() got no answer from the retained loader. The advice depends on whether
    the servo was power-cycled since the failed install; the command that just failed is offered again only after a
    new power cycle of the isolated servo (a failed --loader-fresh prompt)."""
    scan = scan_command(wire) + ' --all-bauds'
    fresh = recovery_command(wire, servo_id, loader_fresh=True)
    if loader_fresh:
        first = (f'If the servo was not power-cycled since the failed install, run '
                 f'`{recovery_command(wire, servo_id)}` (without --loader-fresh). If the servo was power-cycled again '
                 f'or its prompt failed, isolate it, power-cycle it, and run `{fresh}` again.')
        return scan, f' {first} If neither applies, run `{scan}` to find it.'
    first = f'If the servo was power-cycled since the failed install, run `{fresh}`.'
    return scan, f' {first} Otherwise run `{scan}` to find it.'


def update_command(wire, servo_id) -> str:
    """The exact command that re-installs the firmware on a servo that runs it (after F8 was acknowledged)."""
    return f'openxl330 update --port {_port_name(wire)} --id {servo_id}'


def _utc():
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')


def _sha256(data):
    return hashlib.sha256(data).hexdigest()


def _check_id(value, what):
    if type(value) is not int or not 0 <= value <= 252:
        raise InvalidArgument(f'{what} must be an integer 0..252, got {value!r} (nothing was sent)')


def _check_bool(value, what):
    if type(value) is not bool:
        raise InvalidArgument(f'{what} must be True or False, got {value!r} (nothing was sent)')


def _check_progress(callback):
    if callback is not None and not callable(callback):
        raise InvalidArgument('on_progress must be a callable or None (nothing was sent)')


def _path(value, what):
    try:
        return Path(value)
    except TypeError:
        raise InvalidArgument(f'{what} must be a path (nothing was sent)') from None


@lru_cache(maxsize=1)
def _proven_loaders():
    return images.known_loaders()


def __getattr__(name):
    if name == 'PROVEN_LOADERS':
        return _proven_loaders()
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')


def _release(release):
    selected = release if isinstance(release, Release) else Release(release)
    _proven_loaders()
    return selected


def _unexpected(message, step, note=_UNEXPECTED):
    """An InstallError for something that must not happen (loader, calibration or release ID not as they must be):
    its advice is to share the receipt, not to re-install."""
    e = InstallError(message, step=step)
    e.unexpected, e.note = True, note
    return e


# ---------------------------------------------------------------- records
class _Records:
    def __init__(self, root):
        self.root = Path(root)

    def servo(self, fingerprint):
        d = self.root / fingerprint
        (d / 'runs').mkdir(parents=True, exist_ok=True)
        return d

    def relative(self, path):
        """`path` relative to the records folder (as stored in receipts); outside it: its last two parts only."""
        path = Path(path)
        try:
            return path.resolve().relative_to(self.root.resolve()).as_posix()
        except (ValueError, OSError):
            return f'{path.parent.name}/{path.name}'

    @staticmethod
    def keep(path, data):
        """First-seen copy; a different later copy is kept beside it, never overwritten."""
        path = Path(path)
        if not path.exists():
            path.write_bytes(data)
            return 'new'
        if path.read_bytes() == data:
            return 'same'
        alt = path.with_name(f'{path.stem}-{_utc()}{path.suffix}')
        alt.write_bytes(data)
        return 'changed:' + alt.name


class _Run:
    """One run's evidence: receipt.json (rewritten after every step) and the wire log, written as it happens."""

    def __init__(self, records, name, servo_id, recovering=False):
        self.records = records
        base = records.root / '_runs'
        base.mkdir(parents=True, exist_ok=True)
        stamp = _utc()
        for k in range(1000):                     # a new folder, never an existing run's
            self.dir = base / (f'{stamp}-{name}' if k == 0 else f'{stamp}-{name}-{k}')
            try:
                self.dir.mkdir()
                break
            except FileExistsError:
                continue
        else:
            raise FileExistsError(f'no free run folder name under {base}')
        self.wire_log = WireLog(self.dir / 'wire.jsonl')
        self.receipt = dict(schema=SCHEMA, started_utc=stamp, run=name, run_dir=records.relative(self.dir), steps=[],
                            warnings=[])
        self.at, self.servo_id, self.servo_dir = 'precheck', servo_id, None
        self.recovering = recovering              # recover(): the servo is in the loader from the start
        self.loader_fresh = False                 # recover(loader_fresh=True): the attach starts with the prompt
        self.save()

    @property
    def path(self):
        return self.dir / 'receipt.json'

    def log(self, event):
        self.wire_log(event)

    def step(self, name, **data):
        self.receipt['steps'].append(dict(step=name, utc=_utc(), **data))
        self.save()

    def save(self):
        tmp = self.dir / 'receipt.json.tmp'
        tmp.write_text(json.dumps(self.receipt, indent=1, default=str) + '\n')
        os.replace(tmp, self.path)

    def finish(self):
        self.receipt['finished_utc'] = _utc()
        try:
            self.save()
        finally:
            self.wire_log.close()
        if self.servo_dir:
            with (self.servo_dir / 'history.jsonl').open('a', encoding='utf-8') as h:
                h.write(json.dumps(dict(run=self.receipt['run_dir'], completed=self.receipt.get('completed', False),
                                        id=self.receipt.get('servo_id'),
                                        program_sha256=self.receipt.get('program_sha256')),
                                   separators=(',', ':')) + '\n')


def _start(records, name, servo_id, recovering=False):
    root = _Records(_path(records, 'records') if records is not None else records_root())
    try:
        return _Run(root, name, servo_id, recovering)
    except OSError as e:
        raise InstallError(f'cannot write the install records under {root.root} ({e}). Nothing was sent: fix the '
                           f'folder or its permissions, or pass another records folder', step='records') from e


def _program_fields(release, pack):
    return dict(release=release.name, program_sha256=pack.raw_sha256, program_bytes=len(pack.raw),
                encoded_sha256=pack.encoded_sha256)


def _servo_folder(run, nvm):
    """The servo's records folder, from its calibration fingerprint (created; recorded in the receipt)."""
    fingerprint = _sha256(nvm[:4096])[:16]
    run.servo_dir = run.records.servo(fingerprint)
    run.receipt.update(fingerprint=fingerprint, servo_dir=run.records.relative(run.servo_dir))
    return run.servo_dir


def _keep(run, nvm, **more):
    """Keep the calibration (and `more` {file name: bytes}) in the servo's folder."""
    files = {'nvm.bin': nvm}
    files.update(more)
    return {name: _Records.keep(run.servo_dir / name, data) for name, data in files.items()}


_HEX = frozenset('0123456789abcdefABCDEF')


def _loader_history(run, crc):
    """The loader rule of every install: compare the loader's CRC-32 with the known loaders and with the servo's last
    recorded install, then append it to the servo's history (loader.crc32, one line '<crc32> <utc>' per install). An
    unknown loader, or one that changed since the last install, is a warning only: appended to receipt['warnings'] and
    returned; the install goes on."""
    path = run.servo_dir / 'loader.crc32'
    previous, unreadable = None, 0
    history_text = path.read_text(encoding='utf-8', errors='replace') if path.exists() else ''
    for line in history_text.splitlines():
        words = line.split(maxsplit=1)
        if not words:
            continue
        word = words[0]
        if len(word) == 8 and set(word) <= _HEX:
            previous = int(word, 16)
        else:
            unreadable += 1
    warnings = []
    if crc not in _proven_loaders():
        warnings.append(f'unknown loader version, untested (CRC-32 {crc:08x})')
    if previous is not None and previous != crc:
        warnings.append(f'loader CRC changed since the last install ({previous:08x} -> {crc:08x})')
    if unreadable:
        warnings.append(f'{unreadable} unreadable line(s) in this servo\'s loader history were ignored')
    with path.open('a', encoding='utf-8') as history:
        separator = '\n' if history_text and not history_text.endswith('\n') else ''
        history.write(f'{separator}{crc:08x} {_utc()}\n')
    run.receipt['warnings'].extend(warnings)
    run.receipt['loader_history'] = dict(previous=None if previous is None else f'{previous:08x}',
                                         changed=previous is not None and previous != crc)
    return warnings


def _loader_report(crc, how):
    name = _proven_loaders().get(crc)
    return dict(loader_crc32=f'{crc:08x}', loader_digest=how, loader_known=name is not None, loader_name=name)


def _conclude(run, loader, how, reasons=()):
    """Record the loader verdict and whether the user is asked to share the receipt (what is new to this installer)."""
    report = _loader_report(loader, how)
    reasons = [r for r in reasons if r]
    if not report['loader_known']:
        reasons.append(f'loader CRC-32 {loader:08x} is not a proven loader')
    run.receipt.update(report)
    run.receipt.update(share_receipt=bool(reasons), share_why=reasons)


def _tee(run_log, bus_log):
    """Wire events go to the run's own log and keep going to the Wire's log."""
    if not bus_log:
        return run_log

    def both(event):
        run_log(event)
        bus_log(event)
    return both


def _execute(wire, run, work):
    """Run `work()` with the run's wire log attached. Every failure is recorded in the receipt and raised as described
    in the module docstring; the receipt is always finished."""
    previous = wire.log
    wire.log = _tee(run.log, previous)
    try:
        result = work()
    except BaseException as e:
        out = _failed(run, wire, e)
        _close(wire, run, previous, quiet=True)
        if out is None:
            raise
        raise out from e
    _close(wire, run, previous, quiet=False)
    return result


def _close(wire, run, previous, quiet):
    wire.log = previous
    try:
        run.finish()
    except Exception as e:
        if not quiet:                     # an earlier failure is what the caller needs to see
            raise InstallError(f'the servo was handled, but its records in {run.dir} could not be finished: {e}',
                               step='records') from e


def _sentence(e, note):
    """str(e) followed by `note` (which starts with a space), with a full stop between them when str(e) has none."""
    text = str(e).rstrip()
    return text + ('.' if note and not text.endswith(('.', '!', '?', ':')) else '') + note


def _failed(run, wire, e):
    """Record `e` in the receipt; return the exception to raise instead of it, or None to re-raise `e` itself."""
    interrupted = not isinstance(e, Exception)
    if interrupted:                       # it may have cut an exchange in half: nothing more is sent on this Wire
        wire.mark_uncertain(f'install interrupted: {e!r}')
    step = e.step if isinstance(e, InstallError) else run.at
    unexpected = getattr(e, 'unexpected', False)
    # The servo may be in the loader once F9 may have gone out: enter_loader sets the phase just before sending it
    # (from_stock/update start with the phase cleared); recover() starts with the servo in the loader. Once F8 was
    # acknowledged it runs the new firmware: re-installing (update) is the way on, unless the failure is unexpected.
    launched = wire.loader_phase == 'launched'
    recovery_note = ''
    if not (run.recovering or wire.loader_phase is not None):
        recovery, note = None, ''
    elif run.recovering and step == 'attach' and not launched and not interrupted:
        recovery, recovery_note = _attach_failed(wire, run.servo_id, run.loader_fresh)
        note = recovery_note
    elif launched and unexpected:
        recovery, note = None, getattr(e, 'note', _UNEXPECTED)
    elif launched:
        recovery, note = update_command(wire, run.servo_id), _LAUNCHED
    else:
        recovery, recovery_note = _loader_recovery(wire, run.servo_id)
        note = recovery_note or _IN_LOADER
    if isinstance(e, InstallError) and e.recovery:     # the failure knows better (no reply at the expected ID: scan)
        recovery = e.recovery
    in_loader = not launched if run.recovering else wire.loader_possible
    run.receipt.update(completed=False, error=repr(e), failed_step=step, wire_uncertain=wire.uncertain,
                       in_loader=in_loader, phase=wire.loader_phase)
    if recovery:
        run.receipt['recovery'] = recovery
    if recovery_note:
        run.receipt['recovery_note'] = recovery_note.strip()
    if unexpected:
        run.receipt.update(share_receipt=True, share_why=[str(e)])
    receipt = str(run.path)
    if interrupted:                       # re-raised as itself (Ctrl-C stays Ctrl-C), with the same attributes
        try:
            e.recovery, e.step, e.receipt = recovery, step, receipt
            e.recovery_note = recovery_note.strip()
            if recovery and hasattr(e, 'add_note'):           # Python 3.11+: shown with the traceback
                e.add_note(f'{note.strip()}\nTo recover the servo, run: {recovery}')
        except Exception:
            pass
        return None
    if isinstance(e, InstallError) and (e.recovery or not (recovery or unexpected)):
        out = None
        e.receipt = receipt
    elif isinstance(e, UnsupportedServo) and not recovery:
        out = None
        e.receipt = receipt
    elif isinstance(e, InstallError):
        out = type(e)(_sentence(e, note), step=e.step, recovery=recovery)
    else:
        out = InstallError(f'{_STEPS.get(step, step)} failed: {_sentence(e, note)}', step=step, recovery=recovery)
    if out is not None:
        out.receipt = receipt
    return out


# ---------------------------------------------------------------- loader digest
_SAFE_KEYS = ('event', 't', 'baud', 'bps', 'n', 'drained')


class _Unlogged:
    """While the loader region is read over A0, nothing a servo sent is logged: every event other than the host's
    own requests keeps only its kind, time, baud and counts (rx bytes become `redacted_bytes`; free text such as a
    latch reason, which may quote the bytes, becomes `redacted`). The loader is ROBOTIS code; only its CRC-32 is
    ever kept."""

    def __init__(self, wire):
        self.wire, self.log = wire, wire.log

    def __enter__(self):
        if self.log:
            log = self.log

            def redacted(event):
                if event.get('event') != 'tx':                   # host requests: an A0 address and size
                    out = {k: v for k, v in event.items() if k in _SAFE_KEYS and not isinstance(v, (str, bytes))
                           or k == 'event'}
                    if 'hex' in event:
                        out['redacted_bytes'] = len(event['hex']) // 2
                    dropped = sorted(k for k in event if k not in out and k != 'hex')
                    if dropped:
                        out['redacted'] = dropped
                    event = out
                log(event)
            self.wire.log = redacted
        return self

    def __exit__(self, *exc):
        self.wire.log = self.log


def loader_crc32(wire, servo_id):
    """(CRC-32 (zlib) of the retained loader 0x08000000..0x08003FFF, how it was obtained) from a servo running FW231.
    fast12 and later answer read 1136 with this digest (fast12 computes it on request, so the read waits up to
    LOADER_DIGEST_TIMEOUT; fast13 and later cache it at boot) and refuse A0 reads of the loader. Older images (fast11
    and before) reject read 1136. The fallback hashes an A0 read in memory and discards the bytes. A failed A0 read
    raises InstallError(step='loader_digest') without bus bytes in its message, repr or chain. A latched Wire remains
    latched and its latch reason is replaced by a byte-free one (Wire.redact_latch)."""
    try:
        crc, size = struct.unpack('<II', wire.read(servo_id, *LOADER_DIGEST, timeout=LOADER_DIGEST_TIMEOUT))
    except Rejected:
        pass
    else:
        if size != LOADER_BYTES:
            raise InstallError(f'ID {servo_id} reports a loader digest over {size} bytes, expected {LOADER_BYTES}: '
                               f'this firmware is not one this installer knows. Nothing was changed by this read',
                               step='loader_digest')
        return crc, 'read 1136'
    was_uncertain, failure = wire.uncertain, None
    with _Unlogged(wire):
        try:
            data = wire._flash_read(servo_id, LOADER_ADDR, LOADER_BYTES)     # hashed here, never kept
        except Error as e:                # its message may quote reply bytes: keep only the kind
            failure = type(e).__name__
    if failure is None:
        return zlib.crc32(data), 'A0 read, hashed in memory (image without read 1136)'
    # Raised here, outside the handler, so no exception that quotes the bytes is chained (__context__) to it.
    if wire.uncertain and not was_uncertain:
        wire.redact_latch('a reply was garbled while the loader digest was read (bytes not shown)')   # stays latched
    raise InstallError(f'ID {servo_id}: could not read the loader digest: {_LOADER_READ_FAILED.format(kind=failure)}'
                       + ('. The bus state is unknown, so nothing more is sent on this connection'
                          if wire.uncertain else ''), step='loader_digest')


# ---------------------------------------------------------------- checks before F9
def _bus_clean(wire):
    """Before any TX of a new install: refuse a Wire whose last loader session did not end with an F8 ACK (a servo may
    still be in the loader; pinging it at 57600 or sending F9 elsewhere is unproven), else clear the phase."""
    if wire.loader_possible:
        servo_id = _pending.get(wire)
        recovery, note = _loader_recovery(wire, servo_id) if servo_id is not None else (None, '')
        how = '' if recovery else (f': run {recovery_command(wire, "<ID>")}, with <ID> the ID of the servo that '
                                   f'install was flashing')
        raise InstallError(f'not started: a servo on this bus may still be in the retained loader (phase '
                           f'{wire.loader_phase!r}, no F8 acknowledged since). Nothing was sent. Recover it first'
                           f'{how}.{note}', step='precheck', recovery=recovery)
    wire.loader_phase = None


def _census_bauds(wire, bauds, restore):
    """Broadcast-ping census at each baud: {baud: scan}. Only pings are sent, so no servo changes its baud; the host
    goes back to `restore` afterwards, also when a scan fails (unless the Wire latched)."""
    out = {}
    try:
        for baud in bauds:
            wire.set_baud(baud)
            out[baud] = wire.scan(252)
    finally:
        if not wire.uncertain:
            wire.set_baud(restore)
    return out


def _refuse_error_replies(found, baud, step):
    """A ping answered with an error status is an unidentified device: refused before F9."""
    odd = sorted((i, p['error']) for i, p in found.items() if p.get('error'))
    if odd:
        raise InstallError(f'at {baud} baud ' + ', '.join(f'ID {i} answered the ping with error {e}' for i, e in odd)
                           + ': identify or unplug it before installing. Nothing was installed', step=step)


def _stock_m288(p):
    return p.get('model') == SUPPORTED_MODEL and p.get('firmware') != FW231


def _bystander_ok(p, allow_untested):
    """A stock servo that may stay at 57600 through another servo's loader session (experimental flash-all)."""
    return _stock_m288(p) and (allow_untested or (p['model'], p['firmware']) in PROVEN_BYSTANDERS)


def _judge_other_bauds(censuses, stock_ok=False, allow_untested=False):
    """Rules for the census at the bauds other than the target's. Returns {ID: [bauds it answered at]}: the IDs taken.
      * 57600 is where the loader session runs (and where factory servos start): nothing may answer there, except,
        when stock_ok (the experimental flash-all), stock XL330-M288 servos of a version in PROVEN_BYSTANDERS (any
        stock version with allow_untested): a stock V53 servo at 57600 ignored a whole loader session on the 2026-11-23
        bench;
      * every ID found at another baud is taken (it would collide once both run at the same baud)."""
    taken = {}
    for baud, found in censuses.items():
        _refuse_error_replies(found, baud, 'census_other_bauds')
        if baud == LOADER_BAUD and found and stock_ok and all(_bystander_ok(p, allow_untested)
                                                              for p in found.values()):
            pass
        elif baud == LOADER_BAUD and found:
            raise InstallError(f'servo(s) {sorted(found)} answer at {LOADER_BAUD} baud, where the loader session runs: '
                               'unplug them (or install them first, one at a time) before this install. Nothing was '
                               'installed', step='census_other_bauds')
        for servo_id in found:
            taken.setdefault(servo_id, []).append(baud)
    return taken


def _bauds(bauds):
    return ' and '.join(map(str, bauds)) + ' baud'


def _proven_versions(model):
    return ', '.join(f'V{v}' for m, v in sorted(PROVEN_STOCK) if m == model)


def _image_for(servo_id, spec, release):
    """UnsupportedServo unless `release` is built for this model (before F9)."""
    built = models.by_number(release.model)
    if built is not spec:
        raise UnsupportedServo(f'ID {servo_id} is an {spec.name}, but {release.name} is built for the {built.name}: '
                               f'leave the release out to install the {spec.name} image '
                               f'({images.release_for(spec.number)}). Nothing was installed')


def _stock_policy(stock_id, ident, allow_untested, allow_beta=False, release=None):
    """The model and untested-servo policy (before F9). Returns None for a proven servo, else why it is untested. A
    beta model (allow_beta) counts as untested whatever its stock version: no stock version of it is proven."""
    model, version = ident['model'], ident['firmware']
    spec = models.require(model, allow_beta=allow_beta, what=f'ID {stock_id}')
    if version == FW231:
        raise InstallError(f'ID {stock_id} already runs FW231: use update() (CLI: openxl330 update --id {stock_id}) '
                           f'instead. Nothing was installed', step='identify')
    if release is not None:
        _image_for(stock_id, spec, release)
    if (model, version) in PROVEN_STOCK:
        return None
    if spec.beta:
        return f'{spec.name} is a beta model: no stock firmware of it is proven (this one runs V{version})'
    why = f'stock firmware V{version} is not proven (proven: {_proven_versions(model)})'
    if not allow_untested:
        raise UntestedServo(f'ID {stock_id} runs stock firmware V{version}, and installing over it has not been '
                            f'tested (proven: {_proven_versions(model)}). Nothing was sent beyond pings and reads. To '
                            f'install anyway, at your own risk, pass allow_untested=True (CLI: --allow-untested), '
                            f'and please share the receipt afterwards', step='identify')
    return why


# ---------------------------------------------------------------- after F8
def _wait_boot(wire, servo_id, attempts=20, gap=0.05):
    """After F8 the application needs a moment (clock, sensors) before it answers."""
    last = None
    for _ in range(attempts):
        try:
            return wire.ping(servo_id)
        except NoReply as e:
            last = e
            wire.sleep(gap)
    scan = scan_command(wire)
    raise InstallError(f'no reply from ID {servo_id} after the launch. The firmware answers at the ID the servo had on '
                       f'its stock firmware (its NV journal ID). Find the servo with "{scan}", then re-install it with '
                       f'"openxl330 update --port {_port_name(wire)} --id <that ID>".', step='verify',
                       recovery=scan) from last


def _check_release_id(wire, run, servo_id, release):
    """Read 1120: the servo must report the release ID the bundled manifest names (every bundled release has one).
    Recorded in the receipt as release_id (source_id as 8 hex digits). Returns it."""
    want = release.release_id
    try:
        got = decode_release_id(wire.read(servo_id, *RELEASE_ID))
    except Rejected as e:
        raise _unexpected(f'ID {servo_id} does not report a release ID (read 1120 rejected with error {e.error}), but '
                          f'{release.name} has one', 'verify', _RELEASE_DIFFERS) from None
    shown = dict(got, source_id=f'{got["source_id"]:08x}')
    run.receipt['release_id'] = shown
    if got != want:
        expected = dict(want, source_id=f'{want["source_id"]:08x}')
        raise _unexpected(f'ID {servo_id} reports release ID {shown}, but the {release.name} manifest names '
                          f'{expected}', 'verify', _RELEASE_DIFFERS)
    return shown


def _verify_installed(wire, run, pack, servo_id, release, loader_before=None, nvm_before=None):
    """Prove what is on the servo now, up to the loader and calibration comparisons (a change within this install is
    unexpected). The caller then writes the servo's records and calls _verified(). Returns the evidence:
    dict(loader, how, nvm, marker, devcfg, program_sha256)."""
    run.at = 'verify'
    ident = _wait_boot(wire, servo_id)
    run.step('launched', ping=ident)
    if ident['model'] != release.model or ident['firmware'] != release.version:
        raise InstallError(f'ID {servo_id} answers as model {ident["model"]} firmware {ident["firmware"]}, '
                           f'expected model {release.model} firmware {release.version}', step='verify')
    marker = wire.read(servo_id, *MARKER).rstrip(b'\0').decode('ascii', 'replace')
    if marker != release.marker:
        raise InstallError(f'wrong firmware marker {marker!r} (expected {release.marker!r})', step='verify')
    block = wire.read(servo_id, *DEVBLOCK)
    if block != pack.devcfg:
        raise InstallError('the device block on the servo differs from the one installed', step='verify')
    program = wire.flash_read(servo_id, PROGRAM_BASE, len(pack.raw))
    if program != pack.raw:
        raise InstallError(f'the program bytes on the servo differ from the image (A0 SHA-256 {_sha256(program)}, '
                           f'image {pack.raw_sha256})', step='verify')
    loader, how = loader_crc32(wire, servo_id)
    nvm = wire.flash_read(servo_id, NVM_ADDR, NVM_BYTES)
    if loader_before is not None and loader != loader_before:
        raise _unexpected(f'retained loader changed during this install (CRC-32 {loader_before:08x} -> '
                          f'{loader:08x})', 'verify')
    if nvm_before is not None and nvm != nvm_before:
        raise _unexpected('own calibration/NVM changed', 'verify')
    return dict(loader=loader, how=how, nvm=nvm, marker=marker, devcfg=parse_devcfg(block),
                program_sha256=_sha256(program))


def _verified(wire, run, servo_id, release, seen):
    """The rest of the verification, once the servo's records are written (so a failure here still leaves them): the
    release ID (read 1120), and the servo must be OFF. Records the 'verified' step."""
    run.at = 'verify'
    release_id = _check_release_id(wire, run, servo_id, release)
    status = decode_status(wire.read(servo_id, *STATUS))
    diag = decode_busdiag(wire.read(servo_id, *BUSDIAG))
    if status['enabled']:
        raise InstallError('the servo is not OFF after the launch', step='verify')
    run.step('verified', marker=seen['marker'], devcfg=seen['devcfg'], program_sha256=seen['program_sha256'],
             release_id=release_id, nvm_sha256=_sha256(seen['nvm']), status=status, busdiag=diag,
             **_loader_report(seen['loader'], seen['how']))


def _status_brief(s):
    return {k: s[k] for k in ('enabled', 'ready', 'faults', 'hardware_faults', 'voltage_mV', 'temperature')}


def _upload(wire, run, enter, pack, on_progress):
    """F9 (or attach) -> upload -> F8. `enter()` returns the Loader."""
    _pending[wire] = run.servo_id         # for the refusal of a later install on this Wire, if this one stops
    loader = enter()
    run.receipt['in_loader'] = True
    run.step('loader_ready')
    run.at = 'upload'
    loader.upload(pack, on_progress)
    run.step('uploaded', fa_crc16=pack.crc16)
    run.at = 'launch'
    loader.leave(APP_BAUD)
    _pending.pop(wire, None)
    # Record the acknowledged transition even if later verification or final receipt writing is interrupted.
    run.receipt['in_loader'] = False
    run.save()


def _require_off(servo_id, enabled, step):
    if enabled:
        raise InstallError(f'ID {servo_id} still reports torque enabled after the OFF command. Turn torque off '
                           f'and confirm the state before trying again. Nothing was installed', step=step)


# ---------------------------------------------------------------- public: first install from stock
def from_stock(wire, *, stock_id: int = 1, stock_baud: int = 57_600, release: str | Release | None = None,
               allow_untested: bool = False, records: Path | None = None, on_progress=None,
               allow_beta: bool = False, variant: str | None = None) -> dict:
    """First install on a factory servo still running ROBOTIS firmware, at `stock_id` / `stock_baud`. Returns the
    receipt.

    The servo keeps its stock ID: afterwards it answers at `stock_id`, at 1 Mbaud. Set that ID in ROBOTIS Dynamixel
    Wizard first (each servo its own); the bus is censused at 57600, 1M, 2M and 3M before F9, and an ID another servo
    already uses there is refused. At its baud the servo must be the only one that is not FW231, and nothing else may
    answer at 57600 (the loader session runs there); FW231 servos may stay on the bus. The servo must be an XL330-M288
    (model 1200) with a proven stock firmware (PROVEN_STOCK, else UntestedServo unless allow_untested=True, which the
    receipt records), or, with allow_beta=True, a beta model (the XL330-M077, openxl330.models; its stock firmware
    counts as untested; recorded); anything else is UnsupportedServo before F9. The image: `release` when given (it must
    be built for the servo's model), else the bundled one for the servo's model and `variant` ('performance', the
    default, or 'logging': images.release_for). Its stock EEPROM is read before and compared after (FW231 reports
    the same NV journal bytes). Stock firmware cannot read flash, so its loader digest and calibration become the
    first records after the launch. on_progress(done_bytes, total_bytes) after every upload packet."""
    _check_id(stock_id, 'stock_id')
    _check_bool(allow_untested, 'allow_untested')
    _check_bool(allow_beta, 'allow_beta')
    _check_variant(variant, release)
    _check_progress(on_progress)
    if type(stock_baud) is not int or stock_baud not in STOCK_BAUDS:
        raise InvalidArgument(f'stock_baud {stock_baud!r}: use one of {", ".join(map(str, STOCK_BAUDS))} '
                              f'(nothing was sent)')
    return _install_stock(wire, stock_id=stock_id, stock_baud=stock_baud, release=release,
                          allow_untested=allow_untested, records=records, on_progress=on_progress, allow_beta=allow_beta,
                          variant=variant)


def _bundle_or_given(release):
    """The given Release, or None after checking that every bundled release loads: before anything is sent, a broken
    local bundle blocks the install even when the image is chosen only after the census."""
    if release is not None:
        return _release(release)
    for name in images.RELEASES:
        _release(name).packaged()
    return None


def _check_variant(variant, release):
    if variant is not None and variant not in images.VARIANTS:
        raise InvalidArgument(f'variant {variant!r}: use one of {", ".join(images.VARIANTS)} (nothing was sent)')
    if variant is not None and release is not None and _release(release).kind != variant:
        raise InvalidArgument(f'release {_release(release).name} is a {_release(release).kind} image, not {variant}: '
                              f'give one or the other (nothing was sent)')


def _choose(release, model, variant, what, allow_beta):
    """(Release, packaged) for a servo of `model`: the given release, else the bundled one for its model and variant."""
    if release is None:
        spec = models.require(model, allow_beta=allow_beta, what=what)
        release = _release(images.release_for(spec.number, variant or 'performance'))
    return release, release.packaged()


def _install_stock(wire, *, stock_id, stock_baud, release, allow_untested, records, on_progress, bystanders=False,
                   allow_beta=False, variant=None):
    """from_stock(); bystanders=True is the experimental flash-all's rule (openxl330.experimental): other stock
    XL330-M288 servos with their own IDs may stay on the bus, at the target's baud and at 57600 too (at 57600 only a
    version in PROVEN_BYSTANDERS, unless allow_untested). The census also covers 115200. Each stock bystander's control
    table 0..119 is read before F9 and again after the launch: EEPROM 0..63 and torque enable 64 must be unchanged
    (receipt['bystanders']); other RAM differences are warnings."""
    release = _bundle_or_given(release)
    _bus_clean(wire)
    run = _start(records, f'first-install-id{stock_id}', stock_id)
    if bystanders:
        run.receipt['experimental_bystanders'] = True
    return _execute(wire, run, lambda: _from_stock(wire, run, release, None, stock_id, stock_baud, allow_untested,
                                                   on_progress, bystanders, allow_beta, variant))


def _census_rule(census, stock_id, stock_baud, bystanders, allow_untested=False):
    """None when the census at the target's baud allows the install, else why not."""
    others = {i: p for i, p in census.items() if i != stock_id}
    found = ', '.join(f"ID {i} firmware {p['firmware']}" for i, p in sorted(census.items())) or 'nothing'
    if not bystanders:
        if stock_id not in census or (stock_baud == LOADER_BAUD and others) or any(
                p['firmware'] != FW231 for p in others.values()):
            return (f'need servo ID {stock_id} at {stock_baud} baud with only FW231 servos beside it (and nothing else '
                    f'at {LOADER_BAUD}); found {found}. Connect the factory servos one at a time')
        return None
    if stock_id not in census or any(p['model'] != SUPPORTED_MODEL for p in others.values()) or (
            stock_baud == LOADER_BAUD and not all(_bystander_ok(p, allow_untested) for p in others.values())):
        return (f'need servo ID {stock_id} at {stock_baud} baud with only XL330-M288 servos beside it (stock or '
                f'FW231; at {LOADER_BAUD} only stock {_bystander_versions()}, or any stock version with '
                f'allow_untested); found {found}')
    return None


def _bystander_versions():
    return '/'.join(f'V{v}' for _, v in sorted(PROVEN_BYSTANDERS))


def _snapshot_bystanders(wire, watch, restore):
    """{ID: (baud, control table 0..119)} of each stock bystander, read at its baud; the host goes back to `restore`."""
    out = {}
    try:
        for servo_id, baud in sorted(watch.items()):
            wire.set_baud(baud)
            out[servo_id] = (baud, wire.read(servo_id, 0, BYSTANDER_BYTES, timeout=0.3))
    except BaseException:
        _back_to(wire, restore)
        raise
    wire.set_baud(restore)
    return out


def _back_to(wire, baud):
    """After a failure: put the host back at `baud` without ever replacing the error being raised."""
    try:
        if not wire.uncertain:
            wire.restore_baud(baud)
    except Exception:
        pass


def _check_bystanders(wire, snapshots):
    """({str(ID): 'unchanged' or what differs}, [warnings]) for each snapshot; the host ends at 1 Mbaud. EEPROM 0..63
    and torque enable 64 must be unchanged; other RAM differences (65..119, e.g. hardware error status 70) are
    warnings. A bus that becomes uncertain ends the checks: the rest read 'not checked'."""
    out, warnings = {}, []
    for servo_id, (baud, before) in sorted(snapshots.items()):
        key = str(servo_id)
        if wire.uncertain:
            out[key] = 'not checked: the bus state is unknown'
            continue
        try:
            wire.set_baud(baud)
            ident = wire.ping(servo_id)
            now = wire.read(servo_id, 0, BYSTANDER_BYTES, timeout=0.3)
        except (NoReply, Rejected) as e:
            out[key] = f'no good answer at {baud} baud any more ({type(e).__name__})'
            continue
        except BusUncertain as e:
            out[key] = f'the check at {baud} baud garbled the bus: {e}'
            continue
        strict = [k for k in range(BYSTANDER_STRICT) if now[k] != before[k]]
        ram = [k for k in range(BYSTANDER_STRICT, BYSTANDER_BYTES) if now[k] != before[k]]
        if ident['firmware'] != before[6]:
            out[key] = f'firmware V{before[6]} -> {ident["firmware"]}'
        elif strict:
            out[key] = f'control table byte(s) {strict} changed'
        else:
            out[key] = 'unchanged'
        if ram:
            warnings.append(f'stock servo ID {servo_id}: RAM byte(s) {ram} read differently after the install')
    _back_to(wire, APP_BAUD)
    return out, warnings


def _from_stock(wire, run, release, pack, stock_id, stock_baud, allow_untested, on_progress, bystanders=False,
                allow_beta=False, variant=None):
    census_bauds = BYSTANDER_CENSUS_BAUDS if bystanders else CENSUS_BAUDS
    run.at = 'census'
    wire.set_baud(stock_baud)
    try:
        census = wire.scan(252)
    except BusUncertain as e:
        raise InstallError(f'the census at {stock_baud} baud failed: {e} If two servos share an ID (factory servos all '
                           f'start at ID 1), connect them one at a time. {_WIZARD}', step='census') from e
    run.step('census', census=census, baud=stock_baud)
    others = {i: p for i, p in census.items() if i != stock_id}
    if bystanders:                                      # an error reply is named as such, not as a census mismatch
        _refuse_error_replies(census, stock_baud, 'census')
    why = _census_rule(census, stock_id, stock_baud, bystanders, allow_untested)
    if why:
        raise InstallError(why, step='census')
    if not bystanders:
        _refuse_error_replies(census, stock_baud, 'census')
    run.at = 'identify'
    ident = census[stock_id]
    run.receipt.update(model=ident['model'], stock_firmware=ident['firmware'], allow_untested=allow_untested,
                       allow_beta=allow_beta)
    release, pack = _choose(release, ident['model'], variant, f'ID {stock_id}', allow_beta)
    run.receipt['variant'] = release.kind
    untested = _stock_policy(stock_id, ident, allow_untested, allow_beta, release)
    run.receipt['untested'] = untested
    run.at = 'census_other_bauds'
    censuses = _census_bauds(wire, [b for b in census_bauds if b != stock_baud], stock_baud)
    run.step('census_other_bauds', censuses=censuses)
    taken = _judge_other_bauds(censuses, stock_ok=bystanders, allow_untested=allow_untested)
    if stock_id in taken:
        raise InstallError(f'ID {stock_id} is already used by another servo on this bus (at '
                           f'{_bauds(taken[stock_id])}), so the two would answer together once this one is installed. '
                           f'{_WIZARD} Nothing was installed', step='census_other_bauds')
    snapshots = {}
    if bystanders:
        run.at = 'bystanders'
        watch = {i: stock_baud for i, p in others.items() if p['firmware'] != FW231}
        watch.update({i: baud for baud, found in censuses.items() for i, p in found.items() if p['firmware'] != FW231})
        snapshots = _snapshot_bystanders(wire, watch, stock_baud)
        run.step('bystander_snapshots', bauds={str(i): b for i, (b, _) in snapshots.items()},
                 hex={str(i): t.hex() for i, (_, t) in snapshots.items()})
    run.at = 'identify'
    eeprom = wire.read(stock_id, 0, 64)
    if eeprom[7] != stock_id or struct.unpack_from('<H', eeprom)[0] != ident['model'] or eeprom[6] != ident['firmware']:
        raise InstallError(f'the stock EEPROM of ID {stock_id} disagrees with its ping (ID {eeprom[7]}, model '
                           f'{struct.unpack_from("<H", eeprom)[0]}, firmware {eeprom[6]}). Nothing was installed',
                           step='identify')
    wire.write(stock_id, TORQUE_ENABLE, b'\0')
    _require_off(stock_id, wire.read(stock_id, TORQUE_ENABLE, 1) != b'\0', run.at)
    run.step('stock_identified', model=ident['model'], firmware=ident['firmware'], eeprom_hex=eeprom.hex(),
             untested=untested)
    run.receipt.update(servo_id=stock_id, stock_eeprom_hex=eeprom.hex(), **_program_fields(release, pack))
    run.at = 'enter_loader'
    _upload(wire, run, lambda: wire.enter_loader(stock_id), pack, on_progress)
    seen = _verify_installed(wire, run, pack, stock_id, release)
    loader, how, nvm = seen['loader'], seen['how'], seen['nvm']
    run.at = 'records'                                  # the servo's first records
    _servo_folder(run, nvm)
    kept = _keep(run, nvm, **{'stock-eeprom.bin': eeprom})
    warnings = _loader_history(run, loader)
    run.at = 'nv_journal'
    low = wire.read(stock_id, 0, 64)
    diffs = [i for i in range(64) if i not in EEPROM_OURS and low[i] != eeprom[i]]
    run.receipt.update(_loader_report(loader, how))
    run.step('nv_journal_compared', differing_bytes=diffs, kept=dict(kept, **{'loader.crc32': 'appended'}),
             warnings=warnings)
    if diffs:
        raise _unexpected(f'NV journal bytes {diffs} read differently under FW231 than under the stock firmware',
                          'nv_journal')
    _verified(wire, run, stock_id, release, seen)
    _conclude(run, loader, how, [untested])
    if bystanders:
        run.at = 'bystanders'
        run.receipt['bystanders'], ram = _check_bystanders(wire, snapshots)
        run.receipt['warnings'].extend(ram)
        run.step('bystanders_checked', result=run.receipt['bystanders'], warnings=ram)
    run.receipt['completed'] = True
    return run.receipt


# ---------------------------------------------------------------- public: update a servo already on FW231
def update(wire, *, servo_id: int, release: str | Release | None = None, allow_id_change: bool = False,
           records: Path | None = None, on_progress=None, allow_beta: bool = False, variant: str | None = None) -> dict:
    """(Re)install the bundled image on a servo already running FW231, on a shared bus at 1 Mbaud. Returns the receipt.

    The servo keeps its ID. Its device block becomes the image's default one: a label or start-up baud an earlier
    tool wrote there is dropped (the receipt keeps what it said as device_block_before). A servo whose ID came from
    such a block, not from its stock NV journal, would answer at its NV ID afterwards: refused unless
    allow_id_change=True (recorded in the receipt, with previous_id; the NV ID must be free, and the verification and
    any recovery command then use it). Every other servo on the bus
    stays in its application; FW231 bystanders hear the 57600 loader traffic as line noise and count it. Before F9 the
    bus is also censused at 57600 (nothing may answer there) and at 2M and 3M (an ID found there is taken). The
    loader's CRC-32 and the calibration are read before F9 and must be unchanged after. A servo of a beta model (the
    XL330-M077) is updated only with allow_beta=True and only to an image built for its model. The image: `release`
    when given, else the bundled one for the servo's model and `variant`, which defaults to the variant it runs now
    (performance or logging)."""
    _check_id(servo_id, 'servo_id')
    _check_bool(allow_id_change, 'allow_id_change')
    _check_bool(allow_beta, 'allow_beta')
    _check_variant(variant, release)
    _check_progress(on_progress)
    release = _bundle_or_given(release)
    _bus_clean(wire)
    if wire.port.baudrate != APP_BAUD:
        raise InstallError(f'the host is at {wire.port.baudrate} baud: update needs the bus at 1 Mbaud (switch every '
                           f'servo back to 1 Mbaud, or power-cycle them and connect within 1.5 s). Nothing was sent',
                           step='precheck')
    run = _start(records, f'update-id{servo_id}', servo_id)
    run.receipt.update(allow_id_change=allow_id_change, allow_beta=allow_beta)
    return _execute(wire, run, lambda: _update(wire, run, release, None, servo_id, allow_id_change, on_progress,
                                               allow_beta, variant))


def _running_variant(wire, servo_id):
    """'logging' when the servo runs a logging image (its release-ID variant ends in -log), else 'performance'."""
    try:
        return 'logging' if decode_release_id(wire.read(servo_id, *RELEASE_ID))['variant'].endswith('-log') \
            else 'performance'
    except Rejected:
        return 'performance'


def _update(wire, run, release, pack, servo_id, allow_id_change, on_progress, allow_beta=False, variant=None):
    run.at = 'census'
    census = wire.scan(252)
    run.step('census', census=census)
    if servo_id not in census:
        raise InstallError(f'no servo answers at ID {servo_id} (1 Mbaud): check its ID, power and wiring',
                           step='census')
    others = {i: p for i, p in census.items() if i != servo_id}
    _refuse_error_replies(census, APP_BAUD, 'census')
    run.at = 'identify'
    ident = census[servo_id]
    spec = models.require(ident['model'], allow_beta=allow_beta, what=f'ID {servo_id}')
    if release is None and variant is None and ident['firmware'] == FW231:
        variant = _running_variant(wire, servo_id)
    release, pack = _choose(release, spec.number, variant, f'ID {servo_id}', allow_beta)
    run.receipt['variant'] = release.kind
    _image_for(servo_id, spec, release)
    if ident['firmware'] != FW231:
        raise InstallError(f'ID {servo_id} runs ROBOTIS firmware V{ident["firmware"]}, not FW231: install it with '
                           f'from_stock() (CLI: openxl330 install) instead. Nothing was installed', step='identify')
    if wire.read(servo_id, *MARKER).rstrip(b'\0') != FW231_MARKER:
        raise InstallError(f'ID {servo_id} reports version {FW231} without an openxl330 firmware marker. '
                           f'Check the servo firmware before updating. Nothing was installed', step='identify')
    word = struct.unpack('<I', wire.read(servo_id, BAUD, 4))[0]
    if word != APP_BAUD:
        raise InstallError(f'ID {servo_id} is not at a confirmed 1 Mbaud (baud word {word}). Nothing was installed',
                           step='identify')
    before = parse_devcfg(wire.read(servo_id, *DEVBLOCK))
    diag = decode_busdiag(wire.read(servo_id, *BUSDIAG))
    nv_id = diag['nv_id']
    if not 0 <= nv_id <= 252:
        raise InstallError(f'ID {servo_id} reports an invalid NV journal ID {nv_id}. Check its firmware and '
                           f'nonvolatile settings before updating. Nothing was installed', step='identify')
    run.receipt['device_block_before'] = before
    log = _id_log(wire, servo_id) if diag['abi'] >= 5 else None     # 1144 exists from ABI 5 (fast17) only
    logged = log is not None and log['source'] == 'log'
    keep_logged = logged and release.set_id          # its manifest: the image has the ID log
    run.step('device_block', on_servo=before, nv_id=nv_id, id_log=log)
    if logged and not keep_logged and not allow_id_change:
        raise InstallError(f'ID {servo_id} has that ID from set-id (its ID log), but {release.name} has no ID log '
                           f'(release 17 and later have one), so after this update the servo would answer at its '
                           f'stock ID {nv_id}. Either keep the firmware this servo runs now, or accept the move to ID '
                           f'{nv_id} with allow_id_change=True (CLI: --allow-id-change). Nothing was installed',
                           step='identify')
    if nv_id != servo_id and not allow_id_change and not logged:
        raise InstallError(f'ID {servo_id} has that ID from its device block (written by an earlier install), but its '
                           f'stock ID (NV journal) is {nv_id}. The bundled image has no device block of its own, so '
                           f'after this update the servo would answer at ID {nv_id}. openxl330 changes IDs only on '
                           f'release 17 and later (set-id), and Dynamixel Wizard cannot change the ID of a servo '
                           f'running this firmware. '
                           f'Either keep the firmware this servo runs now (it works), or accept the move to ID '
                           f'{nv_id} with allow_id_change=True (CLI: --allow-id-change). Nothing was installed',
                           step='identify')
    target = nv_id                                       # the ID it answers at after the install
    if target != servo_id:
        run.servo_id = target                            # the verification and any recovery command use it
        if not keep_logged:
            run.receipt.update(previous_id=servo_id)
    run.at = 'census_other_bauds'
    censuses = _census_bauds(wire, [b for b in CENSUS_BAUDS if b != APP_BAUD], APP_BAUD)
    run.step('census_other_bauds', censuses=censuses)
    taken = _judge_other_bauds(censuses)
    if target != servo_id and target in others:
        taken.setdefault(target, []).insert(0, APP_BAUD)
    if target in taken and logged:
        raise InstallError(f'ID {target} is also used by another servo on this bus (at {_bauds(taken[target])}). During '
                           f'the update this servo answers at its stock ID {target} (its ID {servo_id} comes from '
                           f'set-id), so the two would collide: update this servo alone, or disconnect ID {target} '
                           f'first. Nothing was installed', step='census_other_bauds')
    if target in taken:
        which = (f'ID {target} (the stock ID this servo would answer at after the update) is already used'
                 if target != servo_id else f'ID {servo_id} is also used')
        raise InstallError(f'{which} by another servo on this bus (at {_bauds(taken[target])}): once both run at the '
                           f'same baud they would answer together. Unplug that servo (or put it on its own adapter) '
                           f'before this update. Nothing was installed', step='census_other_bauds')
    run.at = 'backup'
    wire.write(servo_id, TORQUE_ENABLE, b'\0')
    status = decode_status(wire.read(servo_id, *STATUS))
    _require_off(servo_id, status['enabled'], run.at)
    loader_before, how = loader_crc32(wire, servo_id)
    nvm_before = wire.flash_read(servo_id, NVM_ADDR, NVM_BYTES)
    _servo_folder(run, nvm_before)
    kept = _keep(run, nvm_before)
    warnings = _loader_history(run, loader_before)
    run.step('backed_up', status=_status_brief(status), nvm_sha256=_sha256(nvm_before),
             kept=dict(kept, **{'loader.crc32': 'appended'}), warnings=warnings, **_loader_report(loader_before, how))
    run.receipt.update(servo_id=target, **_program_fields(release, pack))
    run.at = 'enter_loader'
    _upload(wire, run, lambda: wire.enter_loader(servo_id), pack, on_progress)
    if keep_logged:                                      # the log survives only if the loader kept its page
        target = _booted_at(wire, [nv_id, servo_id])
        run.servo_id = target
        run.receipt['servo_id'] = target
    seen = _verify_installed(wire, run, pack, target, release, loader_before, nvm_before)
    loader, how = seen['loader'], seen['how']
    if keep_logged and target != servo_id:
        target = _reapply_id(wire, run, target, servo_id)
    _verified(wire, run, target, release, seen)
    run.at = 'census_after'
    after = wire.scan(252)
    run.step('census_after', census=after)
    expected = {i: (p['model'], p['firmware']) for i, p in others.items()}
    expected[target] = (release.model, release.version)
    confirmed = {i: (p['model'], p['firmware']) for i, p in after.items() if not p.get('error')}
    changed = sorted(i for i in expected.keys() | after.keys() if i not in expected or confirmed.get(i) != expected[i])
    if changed:
        raise InstallError(f'the post-update census disagrees with the expected bus for ID(s) {changed}. '
                           'The updated servo passed verification before this census. Check the reported servos, '
                           'their IDs, and the bus connections', step='census_after', recovery=scan_command(wire))
    _conclude(run, loader, how)
    run.receipt['completed'] = True
    return run.receipt


# ---------------------------------------------------------------- public: recover a servo left in the loader
# ---------------------------------------------------------------- set ID (fast17+, register 1144)
def _id_log(wire, servo_id):
    """Read 1144 (decode_id_log), or None on an image without the ID log (before fast17)."""
    try:
        return decode_id_log(wire.read(servo_id, *SET_ID))
    except Rejected:
        return None


def _booted_at(wire, ids, attempts=20, gap=0.05):
    """After F8: the first of `ids` that answers (the firmware needs a moment before it does)."""
    for _ in range(attempts):
        for servo_id in ids:
            try:
                wire.ping(servo_id)
                return servo_id
            except NoReply:
                pass
        wire.sleep(gap)
    scan = scan_command(wire)
    raise InstallError(f'no reply at ID {" or ".join(map(str, ids))} after the launch. Find the servo with "{scan}", '
                       f'then re-install it with "openxl330 update --port {_port_name(wire)} --id <that ID>".',
                       step='verify', recovery=scan)


def _write_id(wire, run, servo_id, new_id, step, logged_before=False):
    """Write 1144 and prove the result: the servo answers at new_id with it logged, and no longer at servo_id."""
    run.at = step
    run.receipt['write_sent'] = True
    try:
        wire.write(servo_id, SET_ID[0], encode_set_id(new_id), timeout=SET_ID_REPLY_S)
    except Rejected as e:
        if e.error == 7:                                 # the erase/format path can fail after erasing the log
            _after_refused_write(wire, run, servo_id, step, logged_before, e)
        raise
    except (NoReply, BusUncertain) as e:
        scan = scan_command(wire)
        reset = (' The bus state is uncertain: reopen the port (or call bus.reset()) first.' if wire.uncertain else '')
        raise InstallError(f'no clean reply to the ID change sent to ID {servo_id} ({type(e).__name__}): the servo may '
                           f'now answer at ID {new_id}, or still at ID {servo_id} if it did not store the change. Find '
                           f'it with "{scan}" and check it before sending anything else.{reset}', step=step,
                           recovery=scan) from e
    run.servo_id = new_id
    wire.sleep(SET_ID_QUIET_S)
    log = _id_log(wire, new_id)
    if log is None or log['source'] != 'log' or log['log_id'] != new_id:
        raise _unexpected(f'ID {new_id} reports ID log {log} after the change to ID {new_id}', step)
    diag = decode_busdiag(wire.read(new_id, *BUSDIAG))
    if diag['id'] != new_id:
        raise _unexpected(f'ID {new_id} reports effective ID {diag["id"]} in its bus diagnostics', step)
    try:
        wire.ping(servo_id)
    except NoReply:
        return log
    raise InstallError(f'ID {servo_id} still answers after its ID changed to {new_id}: two servos share ID '
                       f'{servo_id}, or the change did not take. Check the bus with "{scan_command(wire)}"', step=step,
                       recovery=scan_command(wire))


def _after_refused_write(wire, run, servo_id, step, logged_before, error):
    """A 7 from 1144: read the log again (receipt id_log_after_error). If the servo's ID came from the log and the log
    is now empty, it answers at its device-block or stock ID after the next reset: say which (warning, InstallError)."""
    try:
        after = _id_log(wire, servo_id)
    except (NoReply, BusUncertain):
        after = None
    run.receipt['id_log_after_error'] = after
    if not (logged_before and after is not None and after['source'] != 'log'):
        return
    block = parse_devcfg(wire.read(servo_id, *DEVBLOCK))
    fallback = block['id'] if block['state'] == 'override' else decode_busdiag(wire.read(servo_id, *BUSDIAG))['nv_id']
    warning = (f'ID {servo_id} erased its ID log but did not store the new record (error 7): after the next power '
               f'cycle it answers at ID {fallback}')
    run.receipt['warnings'].append(warning)
    raise InstallError(f'{warning}. Run set-id again now, while it still answers at ID {servo_id}, or find it at ID '
                       f'{fallback} after a power cycle', step=step) from error


def _reapply_id(wire, run, at, servo_id):
    """update(): the install erased the ID log, so the servo answers at its device-block / NV ID `at`: log `servo_id`
    again. Returns servo_id."""
    log = _write_id(wire, run, at, servo_id, 'reapply_id')
    run.receipt.update(servo_id=servo_id)
    run.step('id_reapplied', from_id=at, id_log=log)
    return servo_id


def finished_set_id(wire, *, servo_id: int, new_id: int, records: Path | None = None) -> dict | None:
    """After a set-id run that sent its write but did not finish (a lost reply): that run's receipt when the change
    did happen on the same servo, else None. Proof: the newest set-id run from servo_id to new_id under the records
    sent its write and did not complete; servo_id is silent; the servo at new_id has new_id logged and the same
    calibration fingerprint as that run's. Reads only."""
    _check_id(servo_id, 'servo_id')
    _check_id(new_id, 'new_id')
    root = _path(records, 'records') if records is not None else records_root()
    name = f'set-id-id{servo_id}'
    for path in sorted((root / '_runs').glob('*/receipt.json'), key=lambda p: p.parent.name, reverse=True):
        try:
            receipt = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        if not isinstance(receipt, dict) or receipt.get('run') != name or receipt.get('new_id') != new_id:
            continue
        if not receipt.get('write_sent') or receipt.get('completed') or not receipt.get('fingerprint'):
            return None
        try:
            wire.ping(servo_id)
            return None                                   # the old ID still answers: the change did not happen
        except NoReply:
            pass
        log = _id_log(wire, new_id)
        if log is None or log['source'] != 'log' or log['log_id'] != new_id:
            return None
        nvm = wire.flash_read(new_id, NVM_ADDR, CALIBRATION_BYTES)
        return receipt if _sha256(nvm)[:16] == receipt['fingerprint'] else None
    return None


def set_id(wire, *, servo_id: int, new_id: int, records: Path | None = None) -> dict:
    """Change the ID of a servo running openxl330 release 17 or later, over the bus (register 1144), and return the
    receipt. The servo stores the new ID in its ID log (in the program flash, never the loader or its NV journal) and
    answers at it from then on, after power cycles too. A reinstall may erase the log: update() puts the ID back.

    Read-only checks first, nothing sent before they pass: new_id must differ and be 0..252 (InvalidArgument), the
    servo must run release 17+ (NotSupported, with the update command), be OFF (Refused) and new_id must be free at
    this baud (Refused, naming what answers there). Then a recorded run (run name set-id-id<old>, in records_root()
    unless `records`): the calibration page is read for the servo's records folder, 1144 is written (the receipt's
    write_sent is set first), and the result is proven: the servo answers at new_id with it logged and no longer at
    the old ID. A lost reply raises InstallError naming both IDs and the scan command. The receipt has servo_id (the
    new ID), previous_id, id_log_before and id_log_after; the servo's history.jsonl gets a line with the new ID."""
    _check_id(servo_id, 'servo_id')
    _check_id(new_id, 'new_id')
    if new_id == servo_id:
        raise InvalidArgument(f'new_id {new_id}: ID {servo_id} already has that ID (nothing was sent)')
    _bus_clean(wire)
    before = _set_id_preflight(wire, servo_id, new_id)
    run = _start(records, f'set-id-id{servo_id}', servo_id)
    run.receipt.update(servo_id=servo_id, new_id=new_id, baud=wire.port.baudrate, write_sent=False,
                       release_id=before['release_id'], id_log_before=before['log'])
    return _execute(wire, run, lambda: _set_id(wire, run, servo_id, new_id))


def _set_id_preflight(wire, servo_id, new_id):
    try:
        ident = wire.ping(servo_id)
    except NoReply:
        raise NoReply(f'no servo answers at ID {servo_id} ({wire.port.baudrate} baud): check its ID, power and wiring. '
                      f'Nothing was sent', missing=[servo_id]) from None
    if ident['firmware'] != FW231 or wire.read(servo_id, *MARKER).rstrip(b'\0') != FW231_MARKER:
        raise UnsupportedServo(f'ID {servo_id} does not run openxl330 firmware (firmware {ident["firmware"]}): set '
                               f'the ID of a stock servo in Dynamixel Wizard. Nothing was sent')
    try:
        rid = decode_release_id(wire.read(servo_id, *RELEASE_ID))
    except Rejected:
        rid = None
    if rid is not None and decode_busdiag(wire.read(servo_id, *BUSDIAG))['abi'] != rid['abi']:
        rid = None
    log = _id_log(wire, servo_id) if rid and rid['number'] >= 17 and rid['abi'] >= 5 else None
    if log is None:                                       # older, or a logging build (no room for the ID log)
        what = (f'release {rid["number"]} ({rid["variant"]})' if rid else 'an openxl330 release older than 12')
        raise NotSupported(f'ID {servo_id} runs {what}, which cannot change its ID over the bus (the release 17 '
                           f'performance images can): update it with `{update_command(wire, servo_id)}`. Nothing '
                           f'was sent')
    if decode_status(wire.read(servo_id, *STATUS))['enabled']:
        raise Refused(f'ID {servo_id} is enabled, and its ID changes only while it is OFF: call disable() first. '
                      f'Nothing was sent')
    try:
        other = wire.ping(new_id)
    except NoReply:
        other = None
    except Rejected as e:
        other = dict(model='?', firmware='?', error=e.error)
    if other is not None:
        raise Refused(f'ID {new_id} is already used by another servo on this bus (model {other["model"]}, firmware '
                      f'{other["firmware"]}): choose a free ID, or move that servo first. Nothing was sent')
    elsewhere = _answers_at_other_bauds(wire, new_id)
    if elsewhere:
        where = ' and '.join(f'{b // 1_000_000} Mbaud' for b in elsewhere)
        raise Refused(f'ID {new_id} is already used by another servo at {where}: openxl330 servos start at 1 Mbaud '
                      f'after a power cycle, so the two would then share ID {new_id}. Choose a free ID, or move that '
                      f'servo first. Nothing was written')
    return dict(release_id=dict(rid, source_id=f'{rid["source_id"]:08x}'), log=log)


def _answers_at_other_bauds(wire, servo_id):
    """The application bauds (1, 2, 3 Mbaud) other than the host's where `servo_id` answers a unicast ping. The host
    goes back to its baud afterwards (unless the Wire latched)."""
    here, found = wire.port.baudrate, []
    try:
        for baud in (b for b in (APP_BAUD, 2_000_000, 3_000_000) if b != here):
            wire.set_baud(baud)
            try:
                wire.ping(servo_id)
                found.append(baud)
            except NoReply:
                pass
            except Rejected:
                found.append(baud)
    finally:
        if not wire.uncertain:
            wire.set_baud(here)
    return found


def _set_id(wire, run, servo_id, new_id):
    run.at = 'backup'
    nvm = wire.flash_read(servo_id, NVM_ADDR, NVM_BYTES)
    _servo_folder(run, nvm)
    run.step('identified', nvm_sha256=_sha256(nvm), id_log=run.receipt['id_log_before'])
    log = _write_id(wire, run, servo_id, new_id, 'set_id',
                    logged_before=run.receipt['id_log_before']['source'] == 'log')
    rid = decode_release_id(wire.read(new_id, *RELEASE_ID))
    if dict(rid, source_id=f'{rid["source_id"]:08x}') != run.receipt['release_id']:
        raise _unexpected(f'ID {new_id} reports release ID {rid}, not the one ID {servo_id} reported before', 'set_id')
    run.receipt.update(completed=True, servo_id=new_id, previous_id=servo_id, id_log_after=log)
    run.step('id_changed', previous_id=servo_id, new_id=new_id, id_log=log)
    return run.receipt


def _failed_run(servo_id, receipt, root):
    """The receipt path and data for the failed run, or two None values. `receipt` (a receipt.json or its run folder;
    a relative path that does not exist is also tried inside the records folder, as receipts store it) must be from a
    run for ID `servo_id` that left the servo in the loader (in_loader true). Without it: the newest run for
    `servo_id` under the records, when that run left the servo in the loader. Reads files only."""
    if receipt is None:
        for path in sorted((root / '_runs').glob('*/receipt.json'), key=lambda p: p.parent.name, reverse=True):
            try:                                                # names start with UTC: newest first
                r = json.loads(path.read_text())
            except (OSError, ValueError):
                continue
            if isinstance(r, dict) and type(r.get('servo_id')) is int and r['servo_id'] == servo_id:
                return (path, r) if r.get('in_loader') is True else (None, None)
        return None, None
    path = _path(receipt, 'receipt')
    if not path.is_absolute() and not path.exists() and (root / path).exists():
        path = root / path
    path = path / 'receipt.json' if path.is_dir() else path
    advice = 'give the receipt.json of the run that left this servo in the loader, or its folder (or leave it out)'
    try:
        r = json.loads(path.read_text())
    except (OSError, ValueError) as e:
        raise InstallError(f'cannot read the receipt {path} ({e}): {advice}. Nothing was sent',
                           step='receipt') from None
    if not isinstance(r, dict) or 'servo_id' not in r:
        raise InstallError(f'the receipt {path} is from a run that stopped before F9 (it never sent a servo to the '
                           f'loader): {advice}. Nothing was sent', step='receipt')
    if type(r['servo_id']) is not int or r['servo_id'] != servo_id:
        raise InstallError(f'the receipt {path} is for ID {r["servo_id"]}, not ID {servo_id}: {advice}. Nothing was '
                           f'sent', step='receipt')
    if r.get('in_loader') is not True:
        raise InstallError(f'the receipt {path} is from a run that did not leave the servo in the loader (in_loader '
                           f'{r.get("in_loader")!r}): {advice}. Nothing was sent', step='receipt')
    return path, r


def recover(wire, *, servo_id: int, receipt: Path | None = None, release: str | Release | None = None,
            records: Path | None = None, loader_fresh: bool = False, on_progress=None) -> dict:
    """Finish an install on the one servo left in the retained loader (failed FA check, interrupted upload, a servo
    that came up in the loader). Exactly one servo may be in the loader. Returns the receipt.

    servo_id: the ID the servo answers at once it runs the firmware, which is its stock NV journal ID (the one its
    failed run named). loader_fresh=False: the loader's '#'/CR prompt already happened in this power cycle (an install
    failed mid-way on this power-up), so only F1 is sent. loader_fresh=True: the servo was power-cycled into the
    loader, which waits for '#'. receipt: the failed run's receipt.json (or its folder), recorded as failed_run;
    without it, the newest run for servo_id under the records is recorded when it left the servo in the loader. No
    census here: the servo is already in the loader, where a ping is unproven. The servo is identified only after the
    launch (its calibration fingerprint). A calibration page that differs from the servo's record is refused as
    unexpected (the NV journal half may differ: the stock settings change on a trip back to stock firmware, and the
    new copy is kept beside the record); a loader CRC-32 that differs from its last recorded install is a warning
    (receipt['warnings']), as on every install. `completed` is True only when the servo's records hold both its calibration (nvm.bin), which
    matches, and its loader history; otherwise the servo is installed but its receipt says the preservation is
    unverified. When nothing answers the loader, the advice says what to run instead (`--loader-fresh` if the servo was
    power-cycled since the failed install, else a scan at every baud). on_progress(done_bytes, total_bytes) after every
    upload packet. The image: `release` when given, else the failed run's release when it is bundled, else the
    default (fw231-comp2-fast17)."""
    _check_id(servo_id, 'servo_id')
    _check_bool(loader_fresh, 'loader_fresh')
    _check_progress(on_progress)
    root = _path(records, 'records') if records is not None else records_root()
    failed, failed_receipt = _failed_run(servo_id, receipt, root)
    if release is None:
        named = (failed_receipt or {}).get('release')
        release = named if named in images.RELEASES else DEFAULT_RELEASE
    release = _release(release)
    pack = release.packaged()
    run = _start(records, f'recover-id{servo_id}', servo_id, recovering=True)
    run.loader_fresh = loader_fresh
    fingerprint = (failed_receipt.get('expected_fingerprint', failed_receipt.get('fingerprint'))
                   if failed_receipt else None)
    logged = _logged_id_of(failed_receipt)
    return _execute(wire, run, lambda: _recover(wire, run, release, pack, servo_id, failed, fingerprint, loader_fresh,
                                                on_progress, logged))


def _logged_id_of(receipt):
    """The set-id ID the failed update's servo had (its device_block step's ID log), or None."""
    for step in (receipt or {}).get('steps', ()):
        log = step.get('id_log') if isinstance(step, dict) and step.get('step') == 'device_block' else None
        if isinstance(log, dict) and log.get('source') == 'log' and type(log.get('log_id')) is int:
            return log['log_id']
    return None


def _recover(wire, run, release, pack, servo_id, failed, fingerprint, loader_fresh, on_progress, logged=None):
    run.at = 'attach'
    run.receipt.update(servo_id=servo_id, **_program_fields(release, pack),
                       failed_run=run.records.relative(failed) if failed else None)
    if fingerprint is not None:
        run.receipt['expected_fingerprint'] = fingerprint

    def attach():
        wire.loader_phase = 'recovering_fresh' if loader_fresh else 'recovering'
        wire.set_baud(LOADER_BAUD)
        loader = Loader(wire)
        loader.attach(prompt_done=not loader_fresh)
        run.receipt['in_loader'] = True
        run.step('loader_attached', loader_fresh=loader_fresh)
        return loader
    _upload(wire, run, attach, pack, on_progress)
    target = servo_id
    if logged is not None and logged != servo_id:        # its ID came from set-id: the log may have survived
        target = _booted_at(wire, [servo_id, logged])
        run.servo_id = target
        run.receipt['servo_id'] = target
    seen = _verify_installed(wire, run, pack, target, release)
    loader, how, nvm = seen['loader'], seen['how'], seen['nvm']
    if logged is not None and target != logged:
        if release.set_id:
            target = _reapply_id(wire, run, target, logged)
        else:
            run.receipt['logged_id_lost'] = logged
            run.receipt['warnings'].append(
                f'ID {logged} (given by set-id) was lost: {release.name} has no ID log, so the servo now answers at '
                f'its stock ID {target}. Give it its ID back with set-id after installing a release that has an ID '
                f'log (release 17 and later)')
    run.at = 'records'
    actual_fingerprint = _sha256(nvm[:4096])[:16]
    if fingerprint is not None and fingerprint != actual_fingerprint:
        raise _unexpected("the recovered servo's calibration fingerprint differs from the failed run's "
                          "calibration fingerprint", 'records')
    nvm_record = run.records.root / actual_fingerprint / 'nvm.bin'
    known_nvm = nvm_record.exists()
    # The calibration page must match; the NV journal half holds the stock settings, which a trip back to stock
    # firmware (Dynamixel Wizard's recovery) resets, so a different journal is kept beside the record (_keep).
    if known_nvm and nvm_record.read_bytes()[:CALIBRATION_BYTES] != nvm[:CALIBRATION_BYTES]:
        raise _unexpected("the factory calibration page differs from this servo's record", 'records')
    _servo_folder(run, nvm)
    kept = _keep(run, nvm)
    _loader_history(run, loader)
    known = known_nvm and run.receipt['loader_history']['previous'] is not None
    run.receipt.update(known_servo=known, kept=dict(kept, **{'loader.crc32': 'appended'}),
                       preservation="verified against this servo's records" if known else
                       'unverified: no earlier records for this servo')
    _verified(wire, run, target, release, seen)
    run.receipt['completed'] = known
    _conclude(run, loader, how)
    return run.receipt


__all__ = ['PROVEN_LOADERS', 'PROVEN_STOCK', 'SCHEMA', 'SUPPORTED_MODEL', 'from_stock', 'loader_crc32', 'recover',
           'set_id', 'finished_set_id',
           'records_root', 'recovery_command', 'scan_command', 'update', 'update_command']
