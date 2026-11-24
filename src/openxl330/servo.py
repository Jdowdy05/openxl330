"""One servo running openxl330 firmware (FW231), in SI units: Servo, and the State and Status it reports.

A Servo comes from Bus.servo(id). Units: radians, rad/s, N*m, N*m/rad, N*m*s/rad, seconds, volts, amps, degrees C;
degrees only through units="deg". Positions go through the Bus's calibration (zero and direction per servo; by default
0 rad = encoder count 2048, the horn's centre mark, continuous over many turns). Every call does exactly what it says:
nothing turns a servo off by itself, and the command lease is opt-in (lease()).

The helpers below are shared with Group (group.py).
"""
from __future__ import annotations

import json
import math
import numbers
import struct
from dataclasses import dataclass, field

from . import models
from .errors import BusUncertain, CalibrationError, Error, Faulted, InvalidArgument, NoReply, NotEnabled, NotSupported, Refused, Rejected
from .registers import (VOLTAGE_PROTECTION, warning_names, BAUD, BUSDIAG, CLEAR_FAULTS, COMP, DUTY, DUTY_FULL, FAULT_NAMES, GAINS, HEARTBEAT,
                        LAST_RESET, LEASE, LED, MAX_TARGET_COUNTS, MIN_TARGET_COUNTS,
                        FW231_MARKER, MARKER, MAX_REFERENCE_TICKS, MOTION, PD_CONFIG, PD_CONFIG_READ, POSITION,
                        REPLY_TIMING, SNAPSHOT, STATUS, STATUS_ARM, STATUS_CONTROL, TARGET, TICK_HZ, TORQUE,
                        TORQUE_ENABLE, TORQUE_RAW, TORQUE_RELEASE, decode_busdiag, decode_control_status, decode_drive,
                        decode_lease, decode_motion, decode_reply_timing, decode_reset, decode_status, encode_gains,
                        encode_lease, encode_target, encode_torque, pd_config)
from .wire import APP_BAUD

PROFILES = {'quintic': 0, 'linear': 1}
UNITS = ('rad', 'deg')
RETRIES, RETRY_GAP = 20, 0.005          # ARM and PD-config writes: error 7 = a control tick was due (retry)
DUTY_PD = -2 ** 31                      # write 260: back to the PD controller
LEASE_MAX_S = 60.0


@dataclass(frozen=True)
class State:
    """Read 1088 (the motion block), calibrated and direction-signed: position rad, velocity rad/s, torque_motor and
    torque_out N*m (estimates: see registers.decode_motion for when torque_out can be trusted), current A (|I| with
    the sign of the duty), duty -1..1. raw: the decoded block in firmware units (counts, mA, pwm)."""
    id: int
    position: float
    velocity: float
    torque_motor: float
    torque_out: float
    current: float
    duty: float
    raw: dict = field(default_factory=dict, repr=False, hash=False)


@dataclass(frozen=True)
class Status:
    """Read 64/68 (the status block): enabled (ARMed), ready, faults by name (registers.FAULT_NAMES) and as bits,
    supply voltage V, temperature degrees C, calibrated position rad, worst control-tick time us; warnings by name
    (registers.warning_names: 'supply_high' above 7.0 V, 'voltage_protection_off'; never latched). raw: the decoded
    block."""
    id: int
    enabled: bool
    ready: bool
    faults: list = field(hash=False)
    fault_bits: int
    voltage: float
    temperature: float
    position: float
    worst_tick_us: int
    warnings: list = field(default_factory=list, hash=False)
    raw: dict = field(default_factory=dict, repr=False, hash=False)


# ---------------------------------------------------------------------------------------------- conversions
def fault_names(bits: int) -> list:
    """Names of the fault bits (registers.FAULT_NAMES); an unknown bit is 'bit N'."""
    if bits == 0:
        return []
    names = [name for bit, name in sorted(FAULT_NAMES.items()) if bits & bit]
    unknown = bits & ~sum(FAULT_NAMES)
    return names + [f'bit {b}' for b in range(32) if unknown >> b & 1]


def to_state(calibration, servo_id, raw) -> State:
    m = decode_motion(raw)
    sign = calibration.sign(servo_id)
    return State(id=servo_id, position=calibration.to_rad(servo_id, m['position_counts']),
                 velocity=sign * m['velocity_rad_s'], torque_motor=sign * m['tau_motor_nm'],
                 torque_out=sign * m['tau_out_nm'], current=sign * m['current_ma'] / 1000.0, duty=sign * m['duty'],
                 raw=m)


def to_status(calibration, servo_id, raw) -> Status:
    d = decode_status(raw)
    return Status(id=servo_id, enabled=bool(d['enabled']), ready=bool(d['ready']), faults=fault_names(d['faults']),
                  fault_bits=d['faults'], voltage=d['voltage_mV'] / 1000.0, temperature=float(d['temperature']),
                  position=calibration.to_rad(servo_id, d['position_counts']), worst_tick_us=d['worst_tick_us'],
                  warnings=warning_names(d['warnings']), raw=d)


# ---------------------------------------------------------------------------------------------- argument checks
def real(value, what) -> float:
    """A finite real number (bool refused), as float."""
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise InvalidArgument(f'{what} must be a number, not {value!r}')
    try:
        value = float(value)
    except OverflowError:
        raise InvalidArgument(f'{what} is too large to represent') from None
    if not math.isfinite(value):
        raise InvalidArgument(f'{what} must be a finite number, not {value!r}')
    return value


def check_gains(kp, kd):
    """Gains within the firmware's PD bounds: 0 <= kp <= 50 N*m/rad, 0 <= kd <= 2 N*m*s/rad."""
    kp, kd = real(kp, 'kp'), real(kd, 'kd')
    if not 0 <= kp <= 50:
        raise InvalidArgument(f'kp {kp!r} N*m/rad is out of range: use 0 <= kp <= 50')
    if not 0 <= kd <= 2:
        raise InvalidArgument(f'kd {kd!r} N*m*s/rad is out of range: use 0 <= kd <= 2')
    return kp, kd


def check_config_gains(kp, kd):
    """configure(): each gain may be None (the image's default)."""
    if kp is not None:
        check_gains(kp, 1.0)
    if kd is not None:
        check_gains(1.0, kd)


def check_move(duration, profile, units):
    """(duration ticks at 2 kHz, profile code). duration 0 = a step (1 tick)."""
    duration = real(duration, 'duration')
    if duration < 0:
        raise InvalidArgument(f'duration {duration!r} s: use 0 (a step) or more seconds')
    if duration * TICK_HZ > MAX_REFERENCE_TICKS:
        raise InvalidArgument(f'duration {duration!r} s is too long: use at most {MAX_REFERENCE_TICKS / TICK_HZ} s')
    ticks = max(1, round(duration * TICK_HZ))
    if not isinstance(profile, str) or profile not in PROFILES:
        raise InvalidArgument(f"profile {profile!r}: use 'quintic' or 'linear'")
    check_units(units)
    # A step uses the one-tick quintic encoding. The firmware's linear planner requires at least two ticks.
    return ticks, PROFILES[profile] if ticks > 1 else PROFILES['quintic']


def check_units(units):
    if units not in UNITS:
        raise InvalidArgument(f"units {units!r}: use 'rad' or 'deg'")


def to_counts(calibration, servo_id, position, units) -> int:
    value = real(position, 'position')
    radians = math.radians(value) if units == 'deg' else value
    try:
        counts = calibration.to_counts(servo_id, radians)
    except OverflowError:
        counts = None
    if counts is None or not MIN_TARGET_COUNTS <= counts <= MAX_TARGET_COUNTS:
        raise InvalidArgument(f'position {position!r} {units} for ID {servo_id} is outside the firmware reference '
                              f'range ({MIN_TARGET_COUNTS} to {MAX_TARGET_COUNTS} raw encoder counts after calibration)')
    return counts


def lease_ms(timeout) -> int:
    timeout = real(timeout, 'lease timeout')
    if not 0 <= timeout <= LEASE_MAX_S:
        raise InvalidArgument(f'lease timeout {timeout!r} s: use 0 (no lease) up to {LEASE_MAX_S:.0f} s')
    return max(1, round(timeout * 1000)) if timeout > 0 else 0


def check_zero(zero):
    """calibrate(zero=...): None (keep), 'here' (the current position) or an encoder count (int)."""
    if zero is None or zero == 'here':
        return zero
    if isinstance(zero, bool) or not isinstance(zero, int) or not -2 ** 31 <= zero < 2 ** 31:
        raise InvalidArgument(f"zero {zero!r}: use 'here' (the current position), an encoder count (int) or None")
    return zero


def check_direction(direction):
    if direction is not None and (isinstance(direction, bool) or not isinstance(direction, int)
                                  or direction not in (1, -1)):
        raise InvalidArgument(f'direction {direction!r}: use 1 or -1 (or None to keep it)')
    return direction


def led_byte(on):
    if on not in (True, False) or not isinstance(on, (bool, int)):
        raise InvalidArgument(f'led {on!r}: use True (on) or False (off)')
    return b'\x01' if on else b'\x00'


def ids_text(ids):
    return ('ID ' if len(ids) == 1 else 'IDs ') + ', '.join(str(i) for i in ids)


def require_torque_command(bus, ids, infos):
    """NotSupported naming the members whose firmware has no torque command (before release 16 / ABI 4)."""
    from .install import update_command
    old = [i for i in ids if 'torque_command' not in infos[i].features]
    if old:
        raise NotSupported(f'{ids_text(old)} run{"s" if len(old) == 1 else ""} firmware without the torque command '
                           f'(release 16, ABI 4, adds it): `{update_command(bus.wire, old[0])}` installs the bundled '
                           f'release')


def check_torque(value, what='torque'):
    """A torque in N*m as a float, and its Q24 bytes (InvalidArgument outside -128..128)."""
    value = real(value, what)
    encode_torque(value)
    return value


def torque_refused(servo_id, error):
    """The Rejected for a torque command the servo answered with an error."""
    e = Rejected(servo_id, error)
    if error == 7:
        e.args = (f"ID {servo_id} refused the torque command (error 7): an expired 'hold' lease refuses motion until "
                  f'enable() re-arms the servo (lease_status() shows it)',)
    elif error == 4:
        e.args = (f'ID {servo_id} refused the torque command (error 4): it is not configured since boot, or a built-in '
                  f'trajectory runs; call configure() with torque OFF first',)
    return e


def drive_of(cal, servo_id, raw):
    """control_mode()'s dict from a read-640 snapshot, in the calibrated direction."""
    d, sign = decode_drive(raw), cal.sign(servo_id)
    return dict(mode=d['mode'], torque=sign * d['torque_nm'] + 0.0, duty=sign * d['duty'] + 0.0,   # no -0.0
                saturated=d['saturated'], limited=d['limited'])


def emergency_note(exc, sent):
    """Record an emergency OFF on a BusUncertain: emergency_off_ids, and a message that says what was sent."""
    exc.emergency_off_ids = list(sent)
    reason = str(exc).split(' The bus state is unknown', 1)[0].rstrip('.')
    exc.args = (f'{reason}. The bus state is unknown; torque OFF was then sent once, unconfirmed, to '
                f'{ids_text(sent) if sent else "no servo (nothing could be sent)"}. Support the joints and check '
                f'them on a new connection.',)


# ---------------------------------------------------------------------------------------------- shared operations
def voltage_switch(servo, on):
    """Read (on=None) or write register 1152; NotSupported (with the update command) on images without it."""
    if on is not None and type(on) is not bool:
        raise InvalidArgument(f'voltage_protection must be True or False, not {on!r}')
    wire = servo._wire
    try:
        if on is None:
            return wire.read(servo.id, VOLTAGE_PROTECTION, 1) == b'\x01'
        wire.write(servo.id, VOLTAGE_PROTECTION, b'\x01' if on else b'\x00')
    except Rejected:                                       # 1152 is allowed at any time: a refusal means no 1152
        from .install import update_command
        raise NotSupported(f'ID {servo.id} runs {servo.info.release}, which has no upper-voltage protection switch '
                           f'(register 1152, the release 17 images have it): update it with '
                           f'`{update_command(wire, servo.id)}`') from None
    return on


def _write_was_sent(error):
    """Whether a failed set-id run had sent its 1144 write (its receipt says write_sent)."""
    path = getattr(error, 'receipt', None)
    if not path:
        return False
    try:
        with open(path, encoding='utf-8') as f:
            return bool(json.load(f).get('write_sent'))
    except (OSError, ValueError):
        return True                                       # unknown: assume the worst


def _retire(bus, old, new, why_old):
    """Every Servo / Group handle for `old` or `new` made before now refuses further use (Bus._moved)."""
    bus._invalidate_identities()
    generation = bus._identity_generation
    bus._moved[old] = (generation, why_old)
    bus._moved[new] = (generation, f'ID {new} now belongs to the servo that was ID {old} (set_id): get a new '
                                   f'handle with bus.servo({new})')


def change_id(servo, new_id, records=None):
    """Servo.set_id, also returning the run's receipt: (Servo for the new ID, receipt)."""
    return servo._change_id(new_id, records)


def configured(wire, servo_id) -> bool:
    """Read 704 word 2: the PD configuration was written since this power-up (the firmware loses it at every boot)."""
    return bool(struct.unpack_from('<3I', wire.read(servo_id, *PD_CONFIG_READ))[2])


def not_configured(servo_id) -> Refused:
    return Refused(f'ID {servo_id} is not configured since power-up: call configure() first (the firmware loses the PD '
                   f'configuration at every power-up and reboot)')


def check_arm_status(servo_id, status, *, require_ready=False):
    if status['enabled']:
        return
    if status['faults']:
        names = fault_names(status['faults'])
        error = Faulted(f'ID {servo_id} cannot be enabled: latched faults ({", ".join(names)}). Fix the cause, '
                        f'then call clear_faults() (it is OFF) and enable() again')
        error.servo_id, error.faults = servo_id, names
        raise error
    if not status['allowed']:
        raise Refused(f'ID {servo_id} firmware does not allow torque. Check that it is a motion-capable release')
    if require_ready and not status['ready']:
        raise Refused(f'ID {servo_id} is not ready to enable. Check status() and wait for the servo to be ready')


def arm(bus, servo_id, *, preflighted=False):
    """Enable and return True if an OFF servo ACKed ARM. Refresh the hold position on every retry."""
    wire = bus.wire
    st = decode_control_status(wire.read(servo_id, *STATUS_ARM))
    cleared_duty = False
    for attempt in range(RETRIES):
        check_arm_status(servo_id, st)
        if not st['enabled'] and not cleared_duty:
            if not preflighted and not configured(wire, servo_id):
                raise not_configured(servo_id)
            wire.write(servo_id, DUTY, struct.pack('<i', DUTY_PD))
            cleared_duty = True
            st = decode_control_status(wire.read(servo_id, *STATUS_ARM))
            check_arm_status(servo_id, st)
        try:
            if not st['enabled']:
                wire.write(servo_id, TARGET[0], encode_target(st['position_counts'], 1, PROFILES['quintic']))
            wire.write(servo_id, TORQUE_ENABLE, b'\x01')
            return not st['enabled']
        except Rejected as e:
            if e.error != 7 or attempt + 1 == RETRIES:
                raise
        if not st['enabled']:
            wire.sleep(RETRY_GAP)
        st = decode_control_status(wire.read(servo_id, *STATUS_ARM))
        if st['enabled']:
            return False
        check_arm_status(servo_id, st)
        if not configured(wire, servo_id):
            raise not_configured(servo_id)
        preflighted = True


def model_of(info):
    """The openxl330.models.Model of a supported ServoInfo (the XL330-M288 for anything older that names none)."""
    return models.by_name(info.model) or models.M288


def configure(bus, servo_id, kp, kd, model=models.M288):
    """Read the current compensation and config before writing new acknowledged PD settings."""
    wire = bus.wire
    compensation = wire.read(servo_id, *COMP)
    previous_config = wire.read(servo_id, *PD_CONFIG_READ)
    return write_config(wire, servo_id, kp, kd, compensation, previous_config, model)


def write_config(wire, servo_id, kp, kd, compensation, previous_config, model=models.M288):
    """Write the PD config (320; the servo must be OFF, which the caller checked) with the model's torque constant.
    The default gains depend on the model and the image: the model's kp_comp/kd_comp when the image's gear-train
    cancellation is loaded (324 non-zero), else its kp_plain/kd_plain (openxl330.models). The token
    is read 704 word 0 + 1. Error 7 on an OFF servo means a control tick was due (firmware before release 12 did not
    retry that race itself): retried up to 20 times, 5 ms apart; the token stays valid because a 7 never applies the
    config. Returns dict(kp, kd, comp)."""
    comp = any(struct.unpack_from('<9i', compensation))
    kp = kp if kp is not None else (model.kp_comp if comp else model.kp_plain)
    kd = kd if kd is not None else (model.kd_comp if comp else model.kd_plain)
    token = struct.unpack_from('<I', previous_config)[0] % 0xFFFFFFFF + 1
    data = pd_config(kp, kd, token, gain=model.k_nm_per_v)
    for attempt in range(RETRIES):
        try:
            wire.write(servo_id, PD_CONFIG, data)
            break
        except Rejected as e:
            if e.error != 7 or attempt + 1 == RETRIES:
                raise
        wire.sleep(RETRY_GAP)
    return dict(kp=kp, kd=kd, comp=comp)


def refused_while_enabled(ids, what):
    verb = 'is' if len(ids) == 1 else 'are'
    return Refused(f'{what}: {ids_text(ids)} {verb} enabled, and this needs the servo OFF: call disable() first. '
                   f'Nothing was written')


def not_enabled(statuses) -> NotEnabled:
    """NotEnabled for {id: decoded status} of servos that are OFF, naming their faults."""
    parts, faults = [], {}
    for servo_id, st in statuses.items():
        faults[servo_id] = fault_names(st['faults'])
        what = ('faults: ' + ', '.join(faults[servo_id])) if faults[servo_id] else 'no faults'
        parts.append(f'ID {servo_id} is OFF ({what})')
    advice = ('fix the cause, clear the faults with clear_faults(), call enable() (it then holds where it is) and '
              'send the move again' if any(faults.values())
              else 'call enable() first (it then holds where it is) and send the move again')
    e = NotEnabled(f'{"; ".join(parts)}: it will not move. To move it, {advice}')
    e.servo_ids, e.faults = list(statuses), faults
    return e


def targets_rejected(bus, ids, infos) -> Rejected:
    """Rejected (error 7) naming the servos whose new target did not take, with the likely reason for each."""
    from .install import update_command
    reasons = []
    for servo_id in ids:
        if 'release_id' not in infos[servo_id].features:
            reasons.append(f'ID {servo_id}: a move is still running (firmware older than release 12): it takes a new '
                           f'target once the running move ends; `{update_command(bus.wire, servo_id)}` installs a '
                           f'release that retargets a running move')
        elif 'torque_command' in infos[servo_id].features:
            reasons.append(f"ID {servo_id}: refused; it is in torque mode (call release_torque() first), or an expired "
                           f"'hold' lease refuses motion until enable() re-arms the servo (lease_status() shows it)")
        else:
            reasons.append(f"ID {servo_id}: refused; an expired 'hold' lease refuses motion until enable() re-arms the "
                           f'servo (lease_status() shows it)')
    e = Rejected(ids[0], 7)
    e.args = (f'{ids_text(ids)} did not take the new target. ' + '; '.join(reasons),)
    e.servo_ids = list(ids)
    return e


# ---------------------------------------------------------------------------------------------- Servo
class Servo:
    """One FW231 servo on a Bus. Get it with Bus.servo(id), which refuses servos that do not run supported openxl330
    firmware. `info` is what the Bus probed (ServoInfo)."""

    def __init__(self, bus, info):
        self._bus, self._info = bus, info
        self.id = info.id
        self._identity_generation = self._born = bus._identity_generation

    def __repr__(self):
        return f'Servo(id={self.id}, release={self._info.release!r})'

    @property
    def info(self):
        return self._info

    @property
    def bus(self):
        return self._bus

    @property
    def _wire(self):
        self._require_current_identity()
        return self._bus.wire

    def _require_current_identity(self):
        bus = self._bus
        moved = bus._moved.get(self.id)
        if moved and self._born < moved[0]:
            raise Refused(moved[1])
        if self._identity_generation != bus._identity_generation or self._info.baud != bus.port.baudrate:
            self._info = bus._revalidate_handle(self._info)
            self._identity_generation = bus._identity_generation

    @property
    def _cal(self):
        return self._bus.calibration

    # ------------------------------------------------------------ reading
    def status(self) -> Status:
        """Read the status block: enabled, faults, voltage, temperature, position, worst tick."""
        return to_status(self._cal, self.id, self._wire.read(self.id, *STATUS))

    def state(self) -> State:
        """Read the motion block: position, velocity, torques, current, duty."""
        return to_state(self._cal, self.id, self._wire.read(self.id, *MOTION))

    @property
    def position(self) -> float:
        """The calibrated position in radians."""
        return self._cal.to_rad(self.id, self._position_counts())

    def _position_counts(self) -> int:
        return struct.unpack('<i', self._wire.read(self.id, *POSITION))[0]

    def lease_status(self) -> dict:
        """The command lease: dict(ms, action, state 'off'/'running'/'expired', expiries, since_renewal_ms)."""
        return decode_lease(self._wire.read(self.id, LEASE, 16))

    def last_reset(self) -> dict:
        """Why the servo last started (registers.decode_reset). NotSupported before release 11."""
        wire = self._wire
        if 'last_reset' not in self._info.features:
            from .install import update_command
            raise NotSupported(f'ID {self.id} runs {self._info.release}, which has no last-reset register (release 11 '
                               f'and later have it): update it with `{update_command(wire, self.id)}`')
        return decode_reset(wire.read(self.id, *LAST_RESET))

    def reply_timing(self) -> dict[str, int]:
        """Return firmware reply timing maxima in microseconds and late-chain count, when available."""
        address, size = REPLY_TIMING
        for length in (size, 16):
            try:
                return decode_reply_timing(self._wire.read(self.id, address, length))
            except Rejected as exc:
                if exc.error not in (2, 4, 5):
                    raise
        raise NotSupported(f'ID {self.id} does not expose reply timing at register {address}')

    def diagnostics(self) -> dict:
        """Bus counters, release, last reset and reply timing. Unavailable diagnostics are None."""
        d = decode_busdiag(self._wire.read(self.id, *BUSDIAG))
        d['last_reset'] = self.last_reset() if 'last_reset' in self._info.features else None
        d['release'] = self._info.release
        try:
            d['reply_timing'] = self.reply_timing()
        except NotSupported:
            d['reply_timing'] = None
        return d

    # ------------------------------------------------------------ control
    def configure(self, kp=None, kd=None) -> dict:
        """Write the PD configuration (N*m/rad, N*m*s/rad). The servo must be OFF (Refused otherwise: call disable()
        first). Defaults per model (openxl330.models): its kp_comp/kd_comp when the image's gear-train cancellation is
        loaded, else its kp_plain/kd_plain; the XL330-M288's are KP_COMP/KD_COMP and KP_PLAIN/KD_PLAIN.
        Returns dict(kp, kd, comp)."""
        check_config_gains(kp, kd)
        if decode_control_status(self._wire.read(self.id, *STATUS_CONTROL))['enabled']:
            raise refused_while_enabled([self.id], 'PD config not written')
        return configure(self._bus, self.id, kp, kd, model_of(self._info))

    def enable(self):
        """Turn the PD controller on, holding the servo where it is and discarding any target or duty written while
        OFF. Requires configure() once per power-up and raises Faulted for latched faults. If already enabled, it
        retains the target and rearms an expired hold lease. Retries a busy answer (error 7) up to 20 attempts,
        5 ms apart. Keep the shaft still across the hold target write and ARM."""
        self._require_current_identity()
        arm(self._bus, self.id)

    def disable(self):
        """OFF: no torque (a loaded joint falls). If the bus becomes (or already is) uncertain, also while this handle
        is being re-checked, the OFF write is still sent once through Wire.emergency_off (at the current host baud and
        at the baud this servo answered at; its reply is drained, not judged) and BusUncertain is raised with
        emergency_off_ids: the OFF is then unconfirmed."""
        try:
            self._wire.write(self.id, TORQUE_ENABLE, b'\x00')
        except BusUncertain as exc:
            bus = self._bus                                    # not self._wire: no identity re-check on this path
            sent = bus.wire.emergency_off([self.id], bauds=[bus.port.baudrate, self._info.baud])
            emergency_note(exc, sent)
            raise

    def move(self, position, duration=0.0, profile='quintic', units='rad', verify=True):
        """Move to `position` (calibrated; radians, or degrees with units='deg') over `duration` seconds with a
        'quintic' or 'linear' profile; duration 0 is a step. The servo answers the target write itself, so a refused
        target raises Rejected (error 7: a move still running on firmware before release 12, or an expired 'hold'
        lease). verify=True (default) then reads the status: a servo that is OFF raises NotEnabled with its faults.
        Call release_duty() before sending position targets when using open-loop duty. An accepted target does not
        switch the controller from open loop to position control."""
        ticks, kind = check_move(duration, profile, units)
        data = encode_target(to_counts(self._cal, self.id, position, units), ticks, kind)
        try:
            self._wire.write(self.id, TARGET[0], data)
        except Rejected as e:
            if e.error == 7:
                raise targets_rejected(self._bus, [self.id], {self.id: self._info}) from None
            raise
        if verify:
            st = decode_control_status(self._wire.read(self.id, *STATUS_CONTROL))
            if not st['enabled']:
                raise not_enabled({self.id: st})

    def set_gains(self, kp, kd):
        """Live PD gains (N*m/rad, N*m*s/rad), applied at once: 0 <= kp <= 50, 0 <= kd <= 2. Needs configure() once per
        power-up (Refused otherwise)."""
        kp, kd = check_gains(kp, kd)
        try:
            self._wire.write(self.id, GAINS, encode_gains(kp, kd))
        except Rejected as e:
            if e.error == 4 and not configured(self._wire, self.id):
                raise not_configured(self.id) from None
            raise

    def set_duty(self, duty):
        """Open-loop duty -1..1 (direction-signed), replacing the PD output; release_duty() returns to PD. Only while
        enabled (Refused otherwise: the firmware would keep a duty written while OFF and apply it at the next ARM)."""
        duty = real(duty, 'duty')
        if not -1 <= duty <= 1:
            raise InvalidArgument(f'duty {duty!r}: use -1..1')
        pwm = round(self._cal.sign(self.id) * duty * DUTY_FULL)
        if not decode_control_status(self._wire.read(self.id, *STATUS_CONTROL))['enabled']:
            raise Refused(f'duty not written: ID {self.id} is OFF, and a duty written while OFF would wait and apply '
                          f'at the next enable(); call enable() first')
        self._wire.write(self.id, DUTY, struct.pack('<i', pwm))

    def set_torque(self, nm, compensate=True, verify=True):
        """Command torque directly instead of a position (release 16+). nm: output torque at the horn in N*m, signed in
        the joint's calibrated direction. compensate=True (default) passes it through the gear-train compensation
        (friction, back-EMF, rotor inertia) like the PD's own output; compensate=False commands motor torque referred to
        the output with no compensation (duty = torque / (K x supply voltage)). It replaces the PD from the next control
        tick (<= 0.5 ms) until release_torque(), set_duty(), disable() or a fault. No limit is added: only the configured
        torque limit (configure()) and full duty bound it, and the voltage, temperature and current protections and an
        opt-in lease apply as always. A move() is refused while it is active. verify=True (default) first checks that the
        servo is enabled (Refused otherwise: a command written while OFF would wait and apply at the next ARM);
        verify=False sends the write alone, one bus transaction, for torque loops (a servo that is OFF keeps the command,
        and enable() clears it before arming). The torque is set, not measured: see the torque guide for what the model
        assumes."""
        nm = check_torque(nm)
        require_torque_command(self._bus, [self.id], {self.id: self._info})
        if verify and not decode_control_status(self._wire.read(self.id, *STATUS_CONTROL))['enabled']:
            raise Refused(f'torque not commanded: ID {self.id} is OFF, and a torque written while OFF would wait and '
                          f'apply at the next enable(); call enable() first')
        data = encode_torque(self._cal.sign(self.id) * nm)
        try:
            self._wire.write(self.id, TORQUE if compensate else TORQUE_RAW, data)
        except Rejected as e:
            raise torque_refused(self.id, e.error) from None

    def release_torque(self):
        """Back from torque (or open-loop duty) to the PD controller, holding where the servo is (which also becomes its
        target)."""
        require_torque_command(self._bus, [self.id], {self.id: self._info})
        self._wire.write(self.id, TORQUE, struct.pack('<i', TORQUE_RELEASE))

    def control_mode(self) -> dict:
        """What drives the motor now (read 640): {'mode': 'position' | 'duty' | 'torque' | 'raw_torque',
        'torque': the requested output torque in N*m after the configured limit (calibrated direction; the PD's own
        output in position mode), 'duty': the applied duty -1..1 (calibrated direction), 'saturated': the request needs
        more than full duty, so the motor delivers less than asked (at the default 2 N*m limit, saturation is what
        bounds a large request), 'limited': the configured torque limit clipped the request}."""
        return drive_of(self._cal, self.id, self._wire.read(self.id, *SNAPSHOT))

    def release_duty(self):
        """Back from open-loop duty to the PD controller, holding where the servo is (which also becomes its
        target)."""
        self._wire.write(self.id, DUTY, struct.pack('<i', DUTY_PD))

    def _set_led(self, on):
        self._wire.write(self.id, LED, led_byte(on))

    led = property(None, _set_led, doc='The LED: True on, False off. Set only (the firmware has no LED read-back).')

    def clear_faults(self):
        """Clear latched faults. The servo must be OFF (Refused otherwise: call disable() first)."""
        if decode_control_status(self._wire.read(self.id, *STATUS_CONTROL))['enabled']:
            raise refused_while_enabled([self.id], 'faults not cleared')
        self._wire.write(self.id, CLEAR_FAULTS, b'\x5a')

    def lease(self, timeout, action='off'):
        """Opt-in command lease: if no motion command or heartbeat() reaches the enabled servo for `timeout` seconds
        (up to 60), it goes OFF (action='off') or holds where it is ('hold', refusing motion until enable()). Once per
        expiry. timeout=0 switches the lease off; it is off after every boot."""
        data = encode_lease(lease_ms(timeout), action)
        self._wire.write(self.id, LEASE, data)

    def heartbeat(self):
        """Renew the lease without moving anything."""
        self._wire.write(self.id, HEARTBEAT, b'\x01')

    @property
    def voltage_protection(self) -> bool:
        """The upper-voltage protection (register 1152, fast17 rev b): True (every boot) = the upper-voltage trip at
        the configured limit (default 8.0 V) and the 'supply_high' warning above 7.0 V; False = neither. The 3.5 V
        lower limit always applies. Settable at any time, also while enabled; back to True at every boot."""
        return voltage_switch(self, None)

    @voltage_protection.setter
    def voltage_protection(self, on):
        if type(on) is not bool:
            raise InvalidArgument(f'voltage_protection must be True or False, not {on!r}')
        voltage_switch(self, on)

    def set_id(self, new_id, *, records=None) -> 'Servo':
        """Change this servo's ID over the bus (openxl330 release 17+, register 1144) and return a Servo for the new
        ID. The servo must be OFF and new_id free on the bus; nothing is sent until those checks pass (Refused,
        NotSupported, InvalidArgument). The change is a recorded run like an install (openxl330.install.set_id: receipt
        and history under records_root(), or `records`). The host calibration entry moves with it, and new_id must not
        have one of its own (Refused: reset it first). Afterwards this handle, and every Servo or Group handle made
        before the change for either ID, refuses further use (Refused)."""
        return change_id(self, new_id, records)[0]

    def _change_id(self, new_id, records):
        from .install import set_id as set_id_run, update_command
        wire = self._wire
        if 'set_id' not in self._info.features:
            raise NotSupported(f'ID {self.id} runs {self._info.release}, which cannot change its ID over the bus '
                               f'(release 17 and later can): update it with `{update_command(wire, self.id)}`')
        cal = self._cal
        if type(new_id) is int and new_id != self.id and 0 <= new_id <= 252 and cal.has(new_id):
            raise Refused(f'ID {new_id} already has a calibration entry on this host: reset it first (`openxl330 '
                          f'calibrate --id {new_id} --reset`) or choose another ID. Nothing was sent')
        bus, old = self._bus, self.id
        try:
            receipt = set_id_run(wire, servo_id=self.id, new_id=new_id, records=records)
        except Error as e:
            if _write_was_sent(e):                       # the servo may already answer at new_id
                _retire(bus, old, new_id, f'the ID change of ID {old} to ID {new_id} did not finish (set_id): find '
                                          f'the servo with bus.scan(), then get a new handle')
                note = (f' The host calibration of ID {old} was not moved: once the servo answers at ID {new_id}, run '
                        f'`openxl330 set-id --id {old} --new-id {new_id}` again (it finishes the move) or call '
                        f'bus.calibration.move({old}, {new_id}).') if cal.has(old) else ''
                e.args = (str(e).rstrip() + note,) + e.args[1:]
            raise
        _retire(bus, old, new_id, f'ID {old} changed to ID {new_id} (set_id): get a new handle with '
                                  f'bus.servo({new_id}), or a new Group with ID {new_id}')
        if cal.has(old):
            try:
                cal.move(old, new_id)
            except CalibrationError as e:
                raise CalibrationError(f'ID {old} is now ID {new_id}, but its calibration entry could not be moved: '
                                       f'{e}') from e
        return bus.servo(new_id), receipt

    def reboot(self):
        """Restart the firmware and verify its return. A faster bus needs a free ID at 1 Mbaud during reboot."""
        wire = self._wire
        bus_baud = wire.port.baudrate
        try:
            if bus_baud != APP_BAUD:
                wire.set_baud(APP_BAUD)
                if self._reboot_baud_in_use(wire):
                    raise Refused(f'ID {self.id} is already in use at 1 Mbaud. Isolate the servo before rebooting it')
                wire.set_baud(bus_baud)
            wire.reboot(self.id)
            if bus_baud != APP_BAUD:
                wire.set_baud(APP_BAUD)
            if not wire.read(self.id, *MARKER).startswith(FW231_MARKER):
                raise Refused(f'ID {self.id} did not return to the openxl330 application after reboot')
            if bus_baud != APP_BAUD:
                wire.write(self.id, BAUD, struct.pack('<I', bus_baud))
        except BaseException:
            if wire.port.baudrate != bus_baud:
                try:
                    wire.restore_baud(bus_baud)
                except Exception:
                    pass
            raise
        if bus_baud != APP_BAUD:
            wire.set_baud(bus_baud)
            wire.ping(self.id)                       # a frame at the new baud confirms the switch

    def _reboot_baud_in_use(self, wire) -> bool:
        try:
            wire.ping(self.id)
        except NoReply:
            return False
        except Rejected:
            return True                             # a valid error reply is still an occupied ID
        return True

    def calibrate(self, zero=None, direction=None):
        """Set this joint's zero (zero='here': the current position; or an encoder count) and/or direction (1 or -1),
        and save the calibration when it has a file. Returns the joint (calibration.Joint)."""
        zero, direction = check_zero(zero), check_direction(direction)
        if zero == 'here':
            zero = self._position_counts()
        self._cal._apply({self.id: (zero, direction)})
        return self._cal.joint(self.id)
