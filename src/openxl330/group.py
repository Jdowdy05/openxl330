"""Several servos as one: Group. Reads are ONE group transaction (Fast Sync Read or Sync Read) in the group's order;
targets and gains go out in ONE Sync Write. A servo missing from a group read raises NoReply with the others'
results in .partial (never zeros).

Get a Group with Bus.group(ids, off_on_exit=False). Nothing turns the servos off by itself: `with bus.group(...) as g`
turns them OFF on exit only when the group was made with off_on_exit=True.
"""
from __future__ import annotations

import struct
from collections.abc import Iterable, Mapping, Set

from .errors import PROTOCOL_ERRORS, BusUncertain, Error, InvalidArgument, NoReply, Refused, Rejected, UnsupportedServo
from .registers import (COMP, GAINS, HEARTBEAT, LEASE, LED, MOTION, PD_CONFIG_READ, POSITION, SNAPSHOT, STATUS,
                        STATUS_CONTROL, TARGET, TORQUE, TORQUE_RAW, TORQUE_RELEASE, decode_control_status, decode_drive,
                        encode_gains, encode_lease, encode_target, encode_torque)
from .servo import (Servo, arm, check_arm_status, check_config_gains, check_direction, check_gains, check_move,
                    check_torque, check_zero, drive_of, emergency_note, ids_text, lease_ms, not_configured, not_enabled,
                    refused_while_enabled, require_torque_command, targets_rejected, to_counts, to_state, to_status,
                    write_config, model_of)
from .wire import SYNC_WRITE_SETTLE_S


def group_rows(wire, ids, register, convert, fast=False):
    """{id: convert(id, bytes)} from ONE Sync Read (or Fast Sync Read) of `register` (address, size), in `ids` order.
    A missing servo raises NoReply: .partial holds the converted rows that did arrive, .missing the servos that did not
    answer, and .errors {id: protocol error} the ones that answered with an error instead of data."""
    address, size = register
    read = wire.fast_sync_read if fast else wire.sync_read
    try:
        rows = read(ids, address, size)
    except NoReply as e:
        partial, errors = {}, {}
        for servo_id in ids:
            st = e.partial.get(servo_id)
            if not isinstance(st, dict):
                continue
            if st.get('error', 0) & 0x7F:
                errors[servo_id] = st['error'] & 0x7F
            elif len(st.get('data', b'')) == size:
                partial[servo_id] = convert(servo_id, st['data'])
        text = str(e)
        if errors:
            text += '; ' + '; '.join(f'ID {i} answered with error {c} ({PROTOCOL_ERRORS.get(c, "unknown")})'
                                     for i, c in errors.items())
        err = NoReply(f'{text} ({len(partial)} of {len(ids)} servos returned data; it is in .partial)', partial,
                      e.missing)
        err.errors = errors
        raise err from None
    return {i: convert(i, rows[i]) for i in ids}


def _members(group, mapping, what):
    """The group's IDs (in group order) that `mapping` names; refuses a non-mapping or IDs outside the group."""
    if not isinstance(mapping, Mapping):
        raise InvalidArgument(f'{what} must be a dict {{servo ID: value}}, not {type(mapping).__name__}')
    unknown = [i for i in mapping if type(i) is not int or i not in group._infos]
    if unknown:
        raise InvalidArgument(f'{what}: {ids_text(unknown)} not in this group ({group._ids})')
    return [i for i in group._ids if i in mapping]


class Group:
    """Servos used together in a fixed read order. Make a group with Bus.group(ids).

    If opt-in OFF cleanup fails while handling an exception, the exception's off_on_exit_error contains that failure.
    """

    def __init__(self, bus, infos, off_on_exit=False):
        self._bus = bus
        self._infos = {info.id: info for info in infos}
        self._ids = [info.id for info in infos]
        self._servos = {info.id: Servo(bus, info) for info in infos}
        self._identity_generation = bus._identity_generation
        self._validated_baud = bus.port.baudrate
        self.off_on_exit = off_on_exit

    def __repr__(self):
        return f'Group({self._ids}, off_on_exit={self.off_on_exit})'

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        if not self.off_on_exit:
            return False
        if exc is None:
            self.disable()
            return False
        try:                                                 # the error that ended the block stays the one raised
            self.disable()
        except Exception as failed:
            exc.off_on_exit_error = failed
            if hasattr(exc, 'add_note'):
                exc.add_note(f'openxl330: off_on_exit could not turn every servo OFF: {failed}')
        return False

    @property
    def ids(self) -> list:
        return list(self._ids)

    def _ready_wire(self, ids=None):
        bus = self._bus
        generation, baud = bus._identity_generation, bus.port.baudrate
        if self._identity_generation != generation or self._validated_baud != baud:
            members = self._ids if ids is None else ids
            for servo_id in members:
                self._servos[servo_id]._require_current_identity()
            if members == self._ids:
                self._identity_generation, self._validated_baud = generation, baud
        return bus.wire

    @property
    def _cal(self):
        return self._bus.calibration

    # ------------------------------------------------------------ reading
    def read(self, fast=True) -> dict:
        """{id: State} from ONE Fast Sync Read of the motion block (fast=False: one Sync Read)."""
        return group_rows(self._ready_wire(), self._ids, MOTION, lambda i, raw: to_state(self._cal, i, raw), fast)

    def status(self) -> dict:
        """{id: Status} from ONE Sync Read of the status block."""
        return group_rows(self._ready_wire(), self._ids, STATUS, lambda i, raw: to_status(self._cal, i, raw))

    def lease_status(self) -> dict:
        return {i: s.lease_status() for i, s in self._servos.items()}

    def diagnostics(self) -> dict:
        return {i: s.diagnostics() for i, s in self._servos.items()}

    # ------------------------------------------------------------ control
    def enable(self):
        """Check group readiness and configuration, then ARM each servo in order (each holds where it is).
        Errors during ARM carry enabled_ids (newly armed), already_enabled_ids (observed at preflight), and
        unconfirmed_ids (the failing attempt and any IDs still pending). The caller decides how to handle partial
        enable. Torque remains on for servos already enabled."""
        wire = self._ready_wire()
        status = group_rows(wire, self._ids, STATUS_CONTROL, lambda i, raw: decode_control_status(raw))
        already = [i for i in self._ids if status[i]['enabled']]
        off = [i for i in self._ids if not status[i]['enabled']]
        for i in off:
            check_arm_status(i, status[i], require_ready=True)
        if off:
            config = group_rows(wire, off, PD_CONFIG_READ, lambda i, raw: struct.unpack_from('<I', raw, 8)[0])
            for i in off:
                if not config[i]:
                    raise not_configured(i)
        enabled = []
        for index, i in enumerate(self._ids):
            try:
                if arm(self._bus, i, preflighted=i in off):
                    enabled.append(i)
            except (Error, KeyboardInterrupt) as exc:
                exc.enabled_ids = enabled
                exc.already_enabled_ids = already
                exc.unconfirmed_ids = self._ids[index:]
                if isinstance(exc, Error):
                    exc.args = (f'{exc}. Group ARM newly enabled {enabled}, observed already enabled {already}, '
                                f'unconfirmed {exc.unconfirmed_ids}',)
                raise

    def disable(self):
        """OFF each servo in order. Missing replies and refusals are reported after attempting the rest. Once the bus is
        uncertain (BusUncertain), every member not yet confirmed still gets one OFF write through Wire.emergency_off
        (replies drained, not judged; the Wire stays latched), then BusUncertain is raised with emergency_off_ids.
        A failure carries confirmed_off_ids, unconfirmed_ids and a map of off_failures by ID."""
        confirmed, missing, failed = [], [], {}
        for index, servo_id in enumerate(self._ids):
            try:
                self._servos[servo_id].disable()
            except NoReply as exc:
                missing.append(servo_id)
                failed[servo_id] = exc
            except (Refused, Rejected, UnsupportedServo) as exc:
                failed[servo_id] = exc
            except BusUncertain as exc:
                failed[servo_id] = exc
                rest = missing + self._ids[index + 1:]          # not a refused or replaced member
                sent = list(getattr(exc, 'emergency_off_ids', [])) + self._emergency_off(rest)
                emergency_note(exc, sent)
                exc.off_failures = failed
                exc.confirmed_off_ids = confirmed
                exc.unconfirmed_ids = [i for i in self._ids if i not in confirmed]
                raise
            except KeyboardInterrupt as exc:                    # Ctrl-C mid-OFF: the rest still get OFF once
                exc.emergency_off_ids = self._emergency_off(missing + self._ids[index:])
                raise
            else:
                confirmed.append(servo_id)
        if failed:
            message = (f'disable(): OFF unconfirmed for {ids_text(failed)}. Check their power and wiring. '
                       + ' / '.join(str(exc) for exc in failed.values()))
            if missing:
                error = NoReply(message, missing=missing)
            else:
                first = next(iter(failed.values()))
                if isinstance(first, Rejected):
                    error = Rejected(first.servo_id, first.error, first.data)
                    error.args = (message,)
                else:
                    error = type(first)(message)
            error.confirmed_off_ids, error.off_failures = confirmed, failed
            error.unconfirmed_ids = list(failed)
            raise error

    def _emergency_off(self, ids):
        bus = self._bus
        if not ids:
            return []
        if not bus.wire.uncertain:
            bus.wire.mark_uncertain('OFF interrupted')
        return bus.wire.emergency_off(ids, bauds=[bus.port.baudrate, self._validated_baud])

    def set_voltage_protection(self, on: bool) -> None:
        """Servo.voltage_protection for every member, one acknowledged write each (register 1152)."""
        if type(on) is not bool:
            raise InvalidArgument(f'voltage_protection must be True or False, not {on!r}')
        for servo_id in self._ids:
            self._servos[servo_id].voltage_protection = on

    def configure(self, kp=None, kd=None) -> dict:
        """Write the PD configuration to every member; all must be OFF (Refused otherwise: call disable() first).
        Gains default per servo (see Servo.configure). Returns {id: dict(kp, kd, comp)}."""
        check_config_gains(kp, kd)
        wire = self._ready_wire()
        st = group_rows(wire, self._ids, STATUS_CONTROL, lambda i, raw: decode_control_status(raw))
        on = [i for i in self._ids if st[i]['enabled']]
        if on:
            raise refused_while_enabled(on, 'PD config not written')
        compensation = group_rows(wire, self._ids, COMP, lambda i, raw: raw)
        config = group_rows(wire, self._ids, PD_CONFIG_READ, lambda i, raw: raw)
        return {i: write_config(wire, i, kp, kd, compensation[i], config[i], model_of(self._infos[i])) for i in self._ids}

    def move(self, targets, duration=0.0, profile='quintic', units='rad', verify=True, *,
             settle_s=SYNC_WRITE_SETTLE_S):
        """Move members to `targets` {id: position} (calibrated; radians, or degrees with units='deg') together,
        over `duration` seconds with a 'quintic' or 'linear' profile; duration 0 is a step. Members not named hold.
        One Sync Write has no replies. verify=True (default) checks that each servo is enabled and reads back the
        stored position and duration. Readback does not expose profile or distinguish a repeated identical command
        from the previously stored one. Release open-loop duty before expecting position tracking.
        settle_s sets the additional Sync Write delay for verify=False. Reduced delays need adapter validation."""
        ticks, kind = check_move(duration, profile, units)
        ids = _members(self, targets, 'move targets')
        if not ids:
            return
        if verify and settle_s != SYNC_WRITE_SETTLE_S:
            raise InvalidArgument('override the Sync Write settle delay only for verify=False')
        rows = {i: encode_target(to_counts(self._cal, i, targets[i], units), ticks, kind) for i in ids}
        wire = self._ready_wire(ids)
        wire.sync_write(TARGET[0], TARGET[1], rows, settle_s=settle_s)
        if not verify:
            return
        st = group_rows(wire, ids, STATUS_CONTROL, lambda i, raw: decode_control_status(raw))
        off = {i: st[i] for i in ids if not st[i]['enabled']}
        if off:
            raise not_enabled(off)
        back = group_rows(wire, ids, TARGET, lambda i, raw: raw[:8])
        untaken = [i for i in ids if back[i] != rows[i][:8]]
        if untaken:
            raise targets_rejected(self._bus, untaken, self._infos)

    def set_torque(self, torques, compensate=True, verify=True, *, settle_s=SYNC_WRITE_SETTLE_S):
        """Command torque directly on members (release 16+; see Servo.set_torque), in one Sync Write: `torques` is
        {id: N*m} (members not named keep what they do) or one number for every member. compensate=True uses the
        gear-train compensation (write 264), False commands raw motor torque (268). A Sync Write has no replies.
        verify=True (default) first checks that every named member is enabled (Refused otherwise, before anything is
        sent), then reads each member's controller snapshot back and raises Rejected for any that is not in the requested
        torque mode (an expired 'hold' lease refuses torque commands). verify=False sends the Sync Write alone, one bus
        transaction, for torque loops (a member that is OFF keeps the command; enable() clears it before arming).
        settle_s sets the Sync Write delay for verify=False."""
        if isinstance(torques, Mapping):
            ids = _members(self, torques, 'torques')
            values = {i: check_torque(torques[i], f'torque for ID {i}') for i in ids}
        else:
            value = check_torque(torques)
            ids, values = list(self._ids), {i: value for i in self._ids}
        if not ids:
            return
        if verify and settle_s != SYNC_WRITE_SETTLE_S:
            raise InvalidArgument('override the Sync Write settle delay only for verify=False')
        require_torque_command(self._bus, ids, self._infos)
        wire = self._ready_wire(ids)
        st = group_rows(wire, ids, STATUS_CONTROL, lambda i, raw: decode_control_status(raw)) if verify else {}
        off = [i for i in ids if verify and not st[i]['enabled']]
        if off:
            raise Refused(f'torque not commanded: {ids_text(off)} {"is" if len(off) == 1 else "are"} OFF, and a torque '
                          f'written while OFF would wait and apply at the next enable(); call enable() first. Nothing '
                          f'was sent')
        rows = {i: encode_torque(self._cal.sign(i) * values[i]) for i in ids}
        wire.sync_write(TORQUE if compensate else TORQUE_RAW, 4, rows, settle_s=settle_s)
        if verify:
            self._check_drive(wire, ids, 'torque' if compensate else 'raw_torque')

    def release_torque(self, verify=True):
        """Every member back from torque (or open-loop duty) to the PD, holding where it is, in one Sync Write.
        verify=True reads each member's controller snapshot back."""
        require_torque_command(self._bus, self._ids, self._infos)
        wire = self._ready_wire()
        wire.sync_write(TORQUE, 4, {i: struct.pack('<i', TORQUE_RELEASE) for i in self._ids})
        if verify:
            self._check_drive(wire, self._ids, 'position')

    def control_modes(self) -> dict:
        """{id: {'mode', 'torque'}} for every member (Servo.control_mode), from one Sync Read."""
        return group_rows(self._ready_wire(), self._ids, SNAPSHOT, lambda i, raw: drive_of(self._cal, i, raw))

    def _check_drive(self, wire, ids, mode):
        got = group_rows(wire, ids, SNAPSHOT, lambda i, raw: decode_drive(raw)['mode'])
        wrong = [i for i in ids if got[i] != mode]
        if wrong:
            e = Rejected(wrong[0], 7)
            e.args = (f'{ids_text(wrong)} did not switch to {mode} mode (now: '
                      + ', '.join(f'ID {i} {got[i]}' for i in wrong)
                      + "). An expired 'hold' lease refuses motion until enable() re-arms the servo, and a servo that "
                        "went OFF leaves torque mode: check status() and lease_status()",)
            e.servo_ids = wrong
            raise e

    def set_gains(self, gains, *, verify=True):
        """Live gains {id: (kp, kd)} (N*m/rad, N*m*s/rad) in one Sync Write.
        verify=True checks that each servo is configured and reads back both gains. Use verify=False to send
        without confirmation."""
        ids = _members(self, gains, 'gains')
        rows = {}
        for i in ids:
            pair = gains[i]
            if (isinstance(pair, (str, bytes, bytearray, Mapping, Set))
                    or not isinstance(pair, Iterable) or not hasattr(pair, '__len__') or len(pair) != 2):
                raise InvalidArgument(f'gains for ID {i}: give a pair (kp, kd), not {pair!r}')
            rows[i] = encode_gains(*check_gains(*pair))
        if not rows:
            return
        wire = self._ready_wire(ids)
        wire.sync_write(GAINS, 8, rows)
        if not verify:
            return
        config = group_rows(wire, ids, PD_CONFIG_READ, lambda i, raw: struct.unpack_from('<I8s', raw, 8))
        unconfigured = [i for i in ids if not config[i][0]]
        applied = [i for i in ids if config[i][0] and config[i][1] == rows[i]]
        unconfirmed = [i for i in ids if i not in applied]
        if unconfirmed:
            error = Rejected(unconfirmed[0], 4)
            message = f'Gains unconfirmed for {ids_text(unconfirmed)}. Check status and readback'
            if unconfigured:
                message += f'. Call configure() first for {ids_text(unconfigured)}'
            error.args = (message,)
            error.applied_ids, error.unconfirmed_ids = applied, unconfirmed
            raise error

    def lease(self, timeout, action='off'):
        """The opt-in command lease on every member (see Servo.lease); timeout=0 switches it off."""
        data = encode_lease(lease_ms(timeout), action)
        wire = self._ready_wire()
        for servo_id in self._ids:
            wire.write(servo_id, LEASE, data)

    def heartbeat(self):
        """Renew every member's lease in ONE Sync Write (a plain broadcast write would not count)."""
        self._ready_wire().sync_write(HEARTBEAT, 1, {i: b'\x01' for i in self._ids})

    def identify(self, servo_id):
        """LED on for `servo_id`, off for the other members: which physical servo has which ID."""
        if type(servo_id) is not int or servo_id not in self._infos:
            raise InvalidArgument(f'ID {servo_id!r} is not in this group ({self._ids})')
        wire = self._ready_wire()
        for i in self._ids:
            wire.write(i, LED, b'\x01' if i == servo_id else b'\x00')

    def calibrate(self, zeros=None, directions=None) -> dict:
        """Set zeros ('here' for every member at its current position, or {id: 'here' | encoder count}) and/or
        directions ({id: 1 or -1}), and save the calibration when it has a file. Returns {id: Joint}."""
        if zeros is None or zeros == 'here':
            zero_of = {i: zeros for i in self._ids} if zeros else {}
        else:
            zero_of = {i: check_zero(zeros[i]) for i in _members(self, zeros, 'zeros')}
        dir_of = ({} if directions is None else
                  {i: check_direction(directions[i]) for i in _members(self, directions, 'directions')})
        here = [i for i in self._ids if zero_of.get(i) == 'here']
        if here:
            counts = group_rows(self._ready_wire(here), here, POSITION, lambda i, raw: struct.unpack('<i', raw)[0])
            zero_of.update(counts)
        if zero_of or dir_of:
            self._cal._apply({i: (zero_of.get(i), dir_of.get(i)) for i in self._ids if i in zero_of or i in dir_of})
        return {i: self._cal.joint(i) for i in self._ids}
