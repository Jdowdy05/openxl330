"""EXPERIMENTAL: install on every stock servo of a bus at once, and go back to stock firmware through ROBOTIS Dynamixel
Wizard. Opens no port: every function takes a Wire on a port the caller opened.

  flash_all(wire, confirm=f)                every stock XL330-M288 on the bus gets the bundled firmware and keeps its ID
  uninstall_prepare(wire, servo_id=2)       record a servo's stock settings, then leave it waiting in the retained
                                            loader for Dynamixel Wizard's Firmware Recovery
  uninstall_restore(wire, record=path)      after that recovery: write the recorded stock settings, ID and baud back

The evidence is one bench with two XL330-M288 servos (2026-11-23):
  * a stock V53 servo at 57600 baud (the loader's own baud) sent nothing during another servo's whole loader session
    and kept its control table 0..119 byte for byte, so flash_all lets stock servos with their own IDs stay on the bus
    (from_stock refuses them);
  * Dynamixel Wizard 2.0's Firmware Recovery picked up a servo waiting in the retained loader without a power cycle
    (three times, with the other servo on the bus). It resets the stock settings to factory defaults (ID 1, 57600
    baud) and leaves the factory calibration page untouched.
Not covered there: more than two servos, stock servos at bauds other than 57600, Wizard on other operating systems,
and writing back stock settings a user had changed (on the bench they were factory defaults, so only the ID was
written).

No stock firmware is shipped, downloaded or read here: Dynamixel Wizard installs it. Each install runs from_stock's
own steps and checks (install.py); only the census rule differs, and each stock bystander is re-checked after the
launch. Failures raise InstallError with `.recovery` as in install.py.
"""
from __future__ import annotations

import json
from pathlib import Path

from . import install
from .errors import BusUncertain, InstallError, InvalidArgument, UnsupportedServo
from .images import DEFAULT_RELEASE, Release
from .registers import BUSDIAG, DEVBLOCK, FW231_MARKER, MARKER, STATUS, TORQUE_ENABLE, decode_busdiag, decode_status, \
    parse_devcfg
from .wire import APP_BAUD, LOADER_BAUD

FW231 = install.FW231
# Where flash_all and uninstall look for servos (each install censuses the same list): every baud a scan can separate
# (not 9600: a ping reply there outlasts its 3 ms ID slot). A stock servo at 9600 or 4 Mbaud is not seen.
CENSUS_BAUDS = install.BYSTANDER_CENSUS_BAUDS
STOCK_BAUD_INDEX = {9600: 0, 57_600: 1, 115_200: 2, 1_000_000: 3, 2_000_000: 4, 3_000_000: 5, 4_000_000: 6,
                    4_500_000: 7}
RESTORE_BAUDS = (57_600, 115_200, 1_000_000, 2_000_000, 3_000_000)   # what this package's scans and adapters reach
# Stock EEPROM settings written back after Wizard's recovery, as (address, size). ID (7) and baud (8) are written last;
# 0..6 are read-only or the version, and 13 (protocol type) is left alone: this package speaks Protocol 2.0 only.
USER_FIELDS = ((9, 1), (10, 1), (11, 1), (12, 1), (20, 4), (24, 4), (31, 1), (32, 2), (34, 2), (36, 2), (38, 2),
               (44, 4), (48, 4), (52, 4), (60, 1), (62, 1), (63, 1))
RECORD_SCHEMA = 'openxl330-uninstall-v1'
RECORD_NAME = 'uninstall-record.json'


def _wizard_steps(port, servo_id, record, nv_id):
    """What to do next, as lines: prose, then each command on its own line, indented (nothing glued to it). The way back
    to openxl330 names the NV ID: the bundled image answers there."""
    record = f'"{record}"' if ' ' in str(record) else record
    back = ('To keep openxl330 on this servo instead, run:' if nv_id == servo_id else
            f'To keep openxl330 on this servo instead, run (it will then answer at its stock ID {nv_id}, not {servo_id}):')
    return '\n'.join((
        f'ID {servo_id} is waiting in the retained ROBOTIS loader. Close this program (and anything else using the '
        f'port), then in ROBOTIS Dynamixel Wizard 2.0 open Tools > Firmware Recovery, choose XL330-M288, the latest '
        f'firmware and this adapter. No power cycle is needed: Wizard finds the waiting loader. Do not power-cycle the '
        f'servos while others are connected: at power-up every servo\'s loader listens, so the recovery could reach '
        f'all of them. The recovery resets the stock settings to factory defaults (ID 1, 57600 baud).',
        'After the recovery, run:',
        f'  openxl330 experimental uninstall-restore --port {port} --record {record}',
        back,
        f'  openxl330 recover --port {port} --id {nv_id} --loader-fresh'))


# ---------------------------------------------------------------- census and plan
def _census(wire, restore):
    """{baud: scan} at every CENSUS_BAUDS (pings only); the host goes back to `restore` unless the Wire latched."""
    out = {}
    try:
        for baud in CENSUS_BAUDS:
            wire.set_baud(baud)
            try:
                out[baud] = wire.scan(252)
            except BusUncertain as e:
                raise InstallError(f'the census at {baud} baud failed: {e} Two servos at one ID collide (factory '
                                   f'servos all start at ID 1): give each servo its own ID in Dynamixel Wizard, '
                                   f'connected alone, then try again. Nothing was installed', step='census') from e
    except BaseException:
        install._back_to(wire, restore)
        raise
    wire.set_baud(restore)
    return out


def _by_id(censuses):
    seen = {}
    for baud, found in censuses.items():
        for servo_id, ping in found.items():
            seen.setdefault(servo_id, []).append((baud, ping))
    return seen


def plan(censuses, *, allow_untested=False) -> dict:
    """What flash_all would do with this census ({baud: scan}): {'stock': [...], 'fw231': [...]}, each entry
    {id, baud, firmware} (stock entries also `untested`), in ID order. Raises InstallError(step='plan') listing every
    problem: an ID answering at several bauds, an error reply, a model other than the XL330-M288, a stock version that
    is not proven, and a stock servo at 57600 that would stay there through another servo's loader session while its
    version is not a proven bystander (install.PROVEN_BYSTANDERS) (the last two unless allow_untested)."""
    problems, stock, ours = [], [], []
    for servo_id, hits in sorted(_by_id(censuses).items()):
        if len(hits) > 1:
            problems.append(f'ID {servo_id} answers at several bauds ({install._bauds([b for b, _ in hits])}): '
                            f'every servo needs its own ID')
            continue
        baud, ping = hits[0]
        if ping.get('error'):
            problems.append(f'ID {servo_id} at {baud} baud answered the ping with error {ping["error"]}')
        elif ping['model'] != install.SUPPORTED_MODEL:
            problems.append(f'ID {servo_id} at {baud} baud is model {ping["model"]}, not an XL330-M288 (model '
                            f'{install.SUPPORTED_MODEL}): unplug it')
        elif ping['firmware'] == FW231:
            ours.append(dict(id=servo_id, baud=baud, firmware=FW231))
        else:
            untested = (ping['model'], ping['firmware']) not in install.PROVEN_STOCK
            if untested and not allow_untested:
                problems.append(f'ID {servo_id} at {baud} baud runs stock V{ping["firmware"]}, which is not proven '
                                f'(proven: {install._proven_versions(ping["model"])}); allow_untested=True (CLI: '
                                f'--allow-untested) installs over it at your own risk')
            stock.append(dict(id=servo_id, baud=baud, firmware=ping['firmware'], untested=untested))
    first = min((s['id'] for s in stock), default=None)   # installed first (ID order): never waits for another
    if not allow_untested:
        for s in stock:
            if s['baud'] == LOADER_BAUD and s['id'] != first and (
                    install.SUPPORTED_MODEL, s['firmware']) not in install.PROVEN_BYSTANDERS:
                problems.append(f'ID {s["id"]} (stock V{s["firmware"]} at {LOADER_BAUD} baud) would stay on the bus '
                                f'through the other servos\' loader sessions, which only stock '
                                f'{install._bystander_versions()} was tested for: update its stock firmware in '
                                f'Dynamixel Wizard (Firmware Update), or pass allow_untested=True (CLI: '
                                f'--allow-untested)')
    if problems:
        raise InstallError('flash-all not started (nothing was sent beyond pings): ' + '; '.join(problems), step='plan')
    return dict(stock=stock, fw231=ours)


def _stopped(e, servo_id, done, rest):
    """The flash-all error for a failed install: which servo, what was done, then the install's own message (which
    ends with its recovery command)."""
    head = f'flash-all stopped at ID {servo_id}'
    if done:
        head += f' (installed before it: IDs {", ".join(map(str, done))})'
    head += f'; not started: IDs {", ".join(map(str, rest))}' if rest else '; nothing else was left to install'
    if isinstance(e, InstallError):
        out = type(e)(f'{head}. {e}', step=e.step, recovery=e.recovery)
    else:
        out = type(e)(f'{head}. {e}')
    for name in ('receipt', 'unexpected', 'note'):
        if hasattr(e, name):
            setattr(out, name, getattr(e, name))
    out.installed = list(done)
    return out


# ---------------------------------------------------------------- flash-all
def flash_all(wire, *, confirm, release: str | Release = DEFAULT_RELEASE, allow_untested: bool = False,
              records: Path | None = None, on_progress=None) -> dict:
    """EXPERIMENTAL. Install the bundled firmware on every stock XL330-M288 on the bus, one at a time in ID order;
    each keeps its ID and starts at 1 Mbaud. Servos already on FW231 are skipped.

    The bus is censused at CENSUS_BAUDS first (pings only). Refused before anything else is sent, all problems listed
    at once: duplicate IDs (the census collides), an ID answering at several bauds, error replies, other models, stock
    versions that are not proven, and stock servos at 57600 that would stay there through another servo's loader
    session while their version is not a proven bystander (install.PROVEN_BYSTANDERS, V53) (the last two unless
    allow_untested). Then confirm(plan) is called once with plan() of that census;
    a false answer cancels with nothing sent beyond pings. Each install is from_stock's, except that other stock
    XL330-M288 servos may stay on the bus (also at 57600): each one's control table 0..119 is read before F9 and again
    after the launch; its EEPROM 0..63 and torque enable 64 must be unchanged (other RAM differences are warnings). The
    first failed install, or a bystander that changed, stops the run (InstallError with the install's recovery command;
    `.installed` lists the IDs done before it). A final census checks that every installed
    servo answers at 1 Mbaud on FW231. Returns {experimental, release, plan, installed, skipped, cancelled, final};
    `installed` holds one summary per servo (id, baud, from_firmware, completed, fingerprint, run_dir, bystanders,
    warnings). on_progress(servo_id, done_bytes, total_bytes) after every upload packet."""
    if not callable(confirm):
        raise InvalidArgument('confirm must be a callable(plan) -> bool (nothing was sent)')
    install._check_bool(allow_untested, 'allow_untested')
    install._check_progress(on_progress)
    release = install._release(release)
    install._bus_clean(wire)
    here = wire.port.baudrate
    the_plan = plan(_census(wire, here), allow_untested=allow_untested)
    result = dict(experimental=True, release=release.name, plan=the_plan, installed=[], skipped=the_plan['fw231'],
                  cancelled=False, final=None)
    if not the_plan['stock']:
        return result
    if not confirm(the_plan):
        result['cancelled'] = True
        return result
    ids = [entry['id'] for entry in the_plan['stock']]
    for k, entry in enumerate(the_plan['stock']):
        servo_id = entry['id']
        progress = None if on_progress is None else (lambda done, total, i=servo_id: on_progress(i, done, total))
        try:
            receipt = install._install_stock(wire, stock_id=servo_id, stock_baud=entry['baud'], release=release,
                                             allow_untested=allow_untested, records=records, on_progress=progress,
                                             bystanders=True)
        except (InstallError, UnsupportedServo) as e:
            raise _stopped(e, servo_id, ids[:k], ids[k + 1:]) from e
        summary = dict(id=servo_id, baud=entry['baud'], from_firmware=entry['firmware'],
                       completed=receipt.get('completed', False), fingerprint=receipt.get('fingerprint'),
                       run_dir=receipt.get('run_dir'), bystanders=receipt.get('bystanders', {}),
                       warnings=receipt.get('warnings', []))
        result['installed'].append(summary)
        changed = {i: what for i, what in summary['bystanders'].items() if what != 'unchanged'}
        if changed:
            garbled = wire.uncertain
            e = InstallError(f'flash-all stopped after ID {servo_id}: ID {servo_id} was installed and verified, but '
                             + '; '.join(f'bystander ID {i}: {what}' for i, what in sorted(changed.items()))
                             + (f'. Not started: IDs {", ".join(map(str, ids[k + 1:]))}' if ids[k + 1:] else '')
                             + '. Check that servo in Dynamixel Wizard, and please share the run\'s receipt with the '
                             f'openxl330 maintainers at {install.ISSUES_URL}', step='bystanders',
                             recovery=install.scan_command(wire) + ' --all-bauds' if garbled else None)
            e.installed = ids[:k + 1]
            raise e
    final = _census(wire, here)
    result['final'] = final
    where = _by_id(final)
    expect = {s['id']: APP_BAUD for s in result['installed']}            # installed: FW231 at 1 Mbaud
    expect.update({s['id']: s['baud'] for s in the_plan['fw231']})      # skipped: where they were
    wrong = sorted(i for i, baud in expect.items()
                   if [(b, p.get('firmware')) for b, p in where.get(i, [])] != [(baud, FW231)])
    if wrong:
        e = InstallError(f'every install passed its own checks, but in the final census ID(s) {wrong} do not answer '
                         f'as FW231 at the expected baud (alone). Check them with the command below',
                         step='census_after',
                         recovery=install.scan_command(wire) + ' --all-bauds')
        e.installed = ids
        raise e
    return result


# ---------------------------------------------------------------- uninstall
def uninstall_prepare(wire, *, servo_id: int, allow_untested: bool = False, records: Path | None = None) -> dict:
    """EXPERIMENTAL. Step 1 of going back to stock firmware: record the servo's stock settings and leave it waiting in
    the retained loader for ROBOTIS Dynamixel Wizard's Firmware Recovery.

    The servo must run FW231 at 1 Mbaud (the bus is censused at CENSUS_BAUDS: pings only). Refused before F9: error
    replies, the servo's ID at another baud too, any servo answering as ID 1 at 57600 (after the recovery this servo
    is ID 1 at 57600), and a servo at 57600 that is not stock of a version in install.PROVEN_BYSTANDERS (it would sit
    through the loader's banner and Wizard's recovery traffic; allow_untested=True accepts any stock XL330-M288). Records (a run folder under the records, as for installs): its stock settings (bytes
    0..63, the stock NV journal as FW231 reports it), NV ID, label, its first stock baud (from its first-install
    records, when there), and its calibration/NV journal backup. Then OFF, one ID-addressed F9, and the loader's
    banner is awaited without being answered. Returns {record: path of the record file, receipt, next: what to do in
    Wizard}. The Wire stays in the loader phase: send nothing more on it."""
    install._check_id(servo_id, 'servo_id')
    install._check_bool(allow_untested, 'allow_untested')
    install._bus_clean(wire)
    run = install._start(records, f'uninstall-id{servo_id}', servo_id)
    run.receipt.update(experimental='uninstall', allow_untested=allow_untested)
    return install._execute(wire, run, lambda: _prepare(wire, run, servo_id, allow_untested))


def _prepare(wire, run, servo_id, allow_untested=False):
    run.at = 'census'
    censuses = _census(wire, APP_BAUD)
    run.step('census', censuses=censuses)
    for baud, found in censuses.items():
        install._refuse_error_replies(found, baud, 'census')
    me = censuses[APP_BAUD].get(servo_id)
    if not me or me['model'] != install.SUPPORTED_MODEL or me['firmware'] != FW231:
        raise InstallError(f'ID {servo_id} does not answer at 1 Mbaud as an XL330-M288 on FW231 (found '
                           + (f'model {me["model"]} firmware {me["firmware"]}' if me else 'nothing')
                           + '): uninstall starts from a servo running openxl330 at 1 Mbaud. Nothing was sent beyond '
                             'pings', step='census')
    elsewhere = [b for b, found in censuses.items() if b != APP_BAUD and servo_id in found]
    if elsewhere:
        raise InstallError(f'ID {servo_id} also answers at {install._bauds(elsewhere)}: two servos share the ID. '
                           f'Unplug the other one first. Nothing was sent beyond pings', step='census')
    if 1 in censuses[LOADER_BAUD]:
        raise InstallError(f'a servo already answers as ID 1 at {LOADER_BAUD} baud: after Dynamixel Wizard\'s '
                           f'recovery ID {servo_id} starts as ID 1 at {LOADER_BAUD} too, and the two would answer '
                           f'together. Give that servo another ID (or unplug it) first. Nothing was sent beyond pings',
                           step='census')
    odd = sorted((i, p) for i, p in censuses[LOADER_BAUD].items() if not install._bystander_ok(p, allow_untested))
    if odd:
        raise InstallError(', '.join(f'ID {i} (model {p["model"]} firmware {p["firmware"]})' for i, p in odd)
                           + f' would stay at {LOADER_BAUD} baud through this servo\'s loader banner and Dynamixel '
                             f'Wizard\'s recovery traffic, which only stock {install._bystander_versions()} was tested '
                             f'for. Unplug it, update its stock firmware in Dynamixel Wizard, or pass '
                             f'allow_untested=True (CLI: --allow-untested). Nothing was sent beyond pings',
                           step='census')
    run.at = 'identify'
    if wire.read(servo_id, *MARKER).rstrip(b'\0') != FW231_MARKER:
        raise InstallError(f'ID {servo_id} reports version {FW231} without the openxl330 firmware marker. Nothing was '
                           f'sent beyond pings and reads', step='identify')
    settings = wire.read(servo_id, 0, 64)
    nv_id = decode_busdiag(wire.read(servo_id, *BUSDIAG))['nv_id']
    label = parse_devcfg(wire.read(servo_id, *DEVBLOCK)).get('label', '')
    nvm = wire.flash_read(servo_id, install.NVM_ADDR, install.NVM_BYTES)
    run.at = 'records'
    servo_dir = install._servo_folder(run, nvm)
    kept = install._keep(run, nvm)
    first = servo_dir / 'stock-eeprom.bin'
    first_baud = None
    if first.exists():
        data = first.read_bytes()
        first_baud = {v: k for k, v in STOCK_BAUD_INDEX.items()}.get(data[8]) if len(data) > 8 else None
    record = dict(schema=RECORD_SCHEMA, id=servo_id, nv_id=nv_id, label=label, fingerprint=run.receipt['fingerprint'],
                  stock_settings_hex=settings.hex(), original_stock_baud=first_baud, run_dir=run.receipt['run_dir'])
    path = run.dir / RECORD_NAME
    path.write_text(json.dumps(record, indent=1) + '\n')
    run.receipt.update(uninstall=record, servo_id=servo_id)
    run.step('recorded', record=RECORD_NAME, kept=kept)
    run.at = 'backup'
    wire.write(servo_id, TORQUE_ENABLE, b'\0')
    install._require_off(servo_id, decode_status(wire.read(servo_id, *STATUS))['enabled'], run.at)
    run.at = 'handoff'
    install._pending[wire] = servo_id
    run.receipt['in_loader'] = True                # from F9 on, until Wizard (or recover) takes it
    run.save()
    wire._hand_to_loader(servo_id)
    run.step('handed_off', loader='banner seen; waiting at its first prompt')
    run.receipt['completed'] = True
    return dict(record=path, receipt=run.receipt, next=_wizard_steps(install._port_name(wire), servo_id, path, nv_id))


def _read_record(record, root):
    path = install._path(record, 'record')
    if not path.is_absolute() and not path.exists() and (root / path).exists():
        path = root / path
    path = path / RECORD_NAME if path.is_dir() else path
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError) as e:
        raise InstallError(f'cannot read the uninstall record {path} ({e}): give the {RECORD_NAME} that `uninstall` '
                           f'wrote, or its run folder. Nothing was sent', step='receipt') from None
    ok = (isinstance(data, dict) and data.get('schema') == RECORD_SCHEMA and type(data.get('id')) is int
          and 0 <= data['id'] <= 252 and isinstance(data.get('stock_settings_hex'), str))
    try:
        settings = bytes.fromhex(data['stock_settings_hex']) if ok else b''
    except ValueError:
        settings = b''
    if len(settings) != 64:
        raise InstallError(f'{path} is not an uninstall record ({RECORD_SCHEMA}). Nothing was sent', step='receipt')
    return path, data, settings


def uninstall_restore(wire, *, record: Path, baud: int | None = None, records: Path | None = None) -> dict:
    """EXPERIMENTAL. Step 2 of going back to stock firmware, after Dynamixel Wizard's Firmware Recovery: find the
    freshly recovered servo (stock firmware, ID 1, 57600 baud, torque off) and write back the stock settings
    uninstall_prepare recorded (USER_FIELDS that differ), then its ID, then the baud: `baud` if given (one of
    RESTORE_BAUDS), else its first stock baud from the record when it is one of those, else 57600. The bus is censused
    at CENSUS_BAUDS first. Refused before any write: no stock XL330-M288 at ID 1 / 57600, an error reply, and the
    recorded ID answering anywhere (as openxl330 firmware: the recovery was not done on that servo). Returns {id, baud,
    firmware, written (addresses), settings_match, mismatching_bytes}; settings read back differently raise
    InstallError."""
    if baud is not None and (type(baud) is not int or baud not in RESTORE_BAUDS):
        raise InvalidArgument(f'baud {baud!r}: use one of {", ".join(map(str, RESTORE_BAUDS))} (nothing was sent)')
    root = install._path(records, 'records') if records is not None else install.records_root()
    path, rec, want = _read_record(record, root)
    first = rec.get('original_stock_baud')
    target = baud if baud is not None else first if first in RESTORE_BAUDS else LOADER_BAUD
    install._bus_clean(wire)
    new_id = rec['id']
    run = install._start(records, f'uninstall-restore-id{new_id}', new_id)
    run.receipt.update(experimental='uninstall-restore', record=str(path))
    return install._execute(wire, run, lambda: _restore(wire, run, want, new_id, target))


def _restore(wire, run, want, new_id, target):
    run.at = 'census'
    censuses = _census(wire, LOADER_BAUD)
    run.step('census', censuses=censuses)
    for baud, there in censuses.items():
        install._refuse_error_replies(there, baud, 'census')
    elsewhere = {b: there[new_id] for b, there in censuses.items() if new_id in there and (b, new_id) != (LOADER_BAUD, 1)}
    if elsewhere:
        b, p = sorted(elsewhere.items())[0]
        what = ('still runs openxl330: Dynamixel Wizard\'s Firmware Recovery has not been done on it, or another '
                'servo has its ID' if p['firmware'] == FW231 else 'is already used by another servo')
        raise InstallError(f'ID {new_id} answers at {b} baud and {what}. This record is for ID {new_id}; restoring it '
                           f'onto the servo at ID 1 / {LOADER_BAUD} would give two servos that ID. Nothing was written',
                           step='census')
    found = censuses[LOADER_BAUD]
    fresh = found.get(1)
    if not fresh or fresh['model'] != install.SUPPORTED_MODEL or fresh['firmware'] == FW231:
        raise InstallError(f'no freshly recovered stock XL330-M288 answers as ID 1 at {LOADER_BAUD} baud (found '
                           + (', '.join(f"ID {i} model {p['model']} firmware {p['firmware']}"
                                        for i, p in sorted(found.items())) or 'nothing')
                           + '). Run Dynamixel Wizard\'s Firmware Recovery first (see `openxl330 experimental '
                             'uninstall`). Nothing was written', step='census')
    run.at = 'restore'
    if wire.read(1, TORQUE_ENABLE, 1) != b'\0':
        raise InstallError('the recovered servo reports torque on: stock firmware accepts EEPROM writes only with '
                           'torque off. Nothing was written', step='restore')
    before = wire.read(1, 0, 64)
    run.step('stock_before', eeprom_hex=before.hex())
    written = []
    for address, size in USER_FIELDS:
        if before[address:address + size] != want[address:address + size]:
            wire.write(1, address, want[address:address + size], timeout=0.3)
            written.append(address)
    run.step('settings_written', addresses=written)
    if new_id != 1:
        wire.write(1, 7, bytes([new_id]), timeout=0.3)   # it answers from ID 1, then is ID new_id
        wire.sleep(0.1)
    if STOCK_BAUD_INDEX[target] != before[8]:
        wire.write(new_id, 8, bytes([STOCK_BAUD_INDEX[target]]), timeout=0.3)   # in effect after its reply
        wire.sleep(0.2)
        wire.set_baud(target)
    after = wire.read(new_id, 0, 64, timeout=0.3)
    firmware = wire.ping(new_id)['firmware']
    mismatch = [i for address, size in USER_FIELDS for i in range(address, address + size) if after[i] != want[i]]
    result = dict(id=new_id, baud=target, firmware=firmware, written=written, settings_match=not mismatch,
                  mismatching_bytes=mismatch)
    run.receipt.update(restored=result, servo_id=new_id)
    run.step('restored', **result)
    if mismatch:
        raise InstallError(f'ID {new_id} at {target} baud reads back stock settings bytes {mismatch} differently from '
                           f'the record. Check them in Dynamixel Wizard', step='restore')
    run.receipt['completed'] = True
    return result


__all__ = ['CENSUS_BAUDS', 'USER_FIELDS', 'flash_all', 'plan', 'uninstall_prepare', 'uninstall_restore']
