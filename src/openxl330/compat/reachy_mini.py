"""Reachy Mini motor-controller API on openxl330 servos.

Drop-in Python stand-ins for `reachy_mini_motor_controller.ReachyMiniMotorController` and
`ReachyMiniPyControlLoop` (pollen-robotics/reachy-mini-motor-controller), same method names and arguments:

    from openxl330.compat.reachy_mini import ReachyMiniMotorController
    c = ReachyMiniMotorController("/dev/ttyACM0")
    c.enable_torque()
    c.set_stewart_platform_position([0.0] * 6)

Reachy Mini has nine servos: body rotation ID 10 (an XC330, which openxl330 cannot run), Stewart platform IDs 11..16
and antennas 17, 18 (XL330-M288). The XL330s go through openxl330.compat.StockTable; the XC330 stays on stock
firmware and is driven with raw stock Protocol 2.0 reads and writes on the same bus (body_rotation='stock', default),
or left out (body_rotation=None: body_yaw reads NaN and its commands raise NotSupported).

Positions are radians around the stock centre count 2048, exactly as rustypot converts them.

SUPPLY VOLTAGE: Reachy Mini feeds the servos about 6.8-7.6 V. openxl330 trips at its upper limit (default 8.0 V) and
reports a non-latching 'supply_high' warning above 7.0 V, which the controller turns into a SupplyWarning when it opens.
voltage_protection=False turns both off (then nothing in the firmware trips on a high supply; the 3.5 V lower limit
stays). Pollen's 6 V average-voltage cap (Goal_PWM ceiling) has no openxl330 equivalent. See docs/compat.md.
"""
from __future__ import annotations

import math
import struct
import threading
import warnings
import time
from dataclasses import dataclass, field

from ..errors import NotSupported
from .dynamixel_table import MODE_PWM, StockTable
from .model import XL330_M288


class SupplyWarning(UserWarning):
    """An XL330 reports its supply above 7.0 V (status warning 'supply_high'); nothing is refused."""

BODY_ROTATION_ID = 10
STEWART_PLATFORM_IDS = [11, 12, 13, 14, 15, 16]
ANTENNAS_IDS = [17, 18]
ALL_IDS = [BODY_ROTATION_ID, *STEWART_PLATFORM_IDS, *ANTENNAS_IDS]
NAMES = {'body_rotation': 10, 'stewart_1': 11, 'stewart_2': 12, 'stewart_3': 13, 'stewart_4': 14, 'stewart_5': 15,
         'stewart_6': 16, 'right_antenna': 17, 'left_antenna': 18}
_PID_GAIN_ADDR = 80                         # stock D, I, P gains, 3 x u16


def _to_counts(rad, center=XL330_M288.center, cpr=XL330_M288.counts_per_rad):
    return round(center + rad * cpr)


def _to_rad(counts, center=XL330_M288.center, cpr=XL330_M288.counts_per_rad):
    return (counts - center) / cpr


@dataclass
class FullBodyPosition:
    body_yaw: float
    stewart: list
    antennas: list
    timestamp: float = field(default_factory=time.time)


@dataclass
class ControlLoopStats:
    period: list
    read_dt: list
    write_dt: list


class _StockRaw:
    """The XC330 body-rotation servo on stock firmware, through raw Protocol 2.0 reads/writes (stock addresses)."""

    def __init__(self, bus, servo_id):
        self.bus, self.id = bus, servo_id

    def position(self):
        return _to_rad(struct.unpack('<i', self.bus.read(self.id, 132, 4))[0])

    def goal(self, rad):
        self.bus.write(self.id, 116, struct.pack('<i', _to_counts(rad)))

    def torque(self, on):
        self.bus.write(self.id, 64, bytes([int(bool(on))]))

    def torque_enabled(self):
        return bool(self.bus.read(self.id, 64, 1)[0])

    def operating_mode(self, mode):
        if mode == MODE_PWM:
            raise NotSupported('Operating mode 16 (PWM control) is not supported (as in the Pollen controller)')
        self.bus.write(self.id, 11, bytes([mode]))


class ReachyMiniMotorController:
    """Same API as reachy_mini_motor_controller.ReachyMiniMotorController, on openxl330.

    serialport: a port name, or an open openxl330.Bus (tests: a simulated bus). body_rotation: 'stock' or None.
    check_supply: refuse to start when an XL330's supply is above openxl330's voltage limit (default True).
    Other keyword arguments go to StockTable (gains=, supply_v=, current_position=)."""

    def __init__(self, serialport, *, body_rotation='stock', voltage_protection=True, check_supply=True,
                 **table_options):
        from ..bus import Bus
        self._owns = not hasattr(serialport, 'group')
        self.bus = Bus(serialport) if self._owns else serialport
        try:
            self._xl_ids = [*STEWART_PLATFORM_IDS, *ANTENNAS_IDS]
            self.table = StockTable(self.bus, self._xl_ids, **table_options)
            self._body = _StockRaw(self.bus, BODY_ROTATION_ID) if body_rotation == 'stock' else None
            if body_rotation not in ('stock', None):
                raise NotSupported(f"body_rotation={body_rotation!r}: use 'stock' or None")
            if self._body is not None:
                self.bus.set_baud = self._refuse_set_baud    # this bus instance only
            if type(voltage_protection) is not bool:
                raise NotSupported(f'voltage_protection must be True or False, not {voltage_protection!r}')
            try:
                self.bus.group(self._xl_ids).set_voltage_protection(voltage_protection)
            except NotSupported:
                if not voltage_protection:
                    raise                     # cannot turn off what an older image always has on
                # no register 1152 (before fast17 rev b): the upper-voltage trip is always on there
            if check_supply:
                self.check_supply()
        except Exception:
            if self._owns:
                self.bus.close()
            raise

    # ------------------------------------------------------------ openxl330 additions
    def check_supply(self):
        """Check the XL330 supplies: NotSupported if a voltage fault is latched (supply outside 3.5 V..the upper limit,
        8.0 V by default); a SupplyWarning (Python warning, nothing refused) for each XL330 reporting 'supply_high'
        (above 7.0 V, the XL330's 6 V rating plus margin). With voltage_protection=False the firmware reports neither
        and only the 3.5 V lower limit trips. Returns {id: Status}."""
        status = self.bus.group(self._xl_ids).status()
        faulted = {i: s.voltage for i, s in status.items() if 'voltage' in s.faults}
        if faulted:
            raise NotSupported(
                f'voltage fault latched on IDs {sorted(faulted)} (supply {max(faulted.values()):.1f} V): the supply was '
                f'below 3.5 V or above the upper limit. Fix the supply, then clear the faults (Servo.clear_faults). '
                f'See docs/compat.md#supply-voltage')
        high = {i: s.voltage for i, s in status.items() if 'supply_high' in s.warnings}
        if high:
            warnings.warn(SupplyWarning(
                f'supply {max(high.values()):.1f} V on IDs {sorted(high)} is above 7.0 V (XL330 rated 6.0 V): the '
                f'servos run, and trip at the upper limit (default 8.0 V). See docs/compat.md#supply-voltage'),
                stacklevel=3)
        return status

    @staticmethod
    def _refuse_set_baud(*args, **kwargs):
        raise NotSupported('Bus.set_baud with the stock XC330 (ID 10) on the bus: openxl330 baud changes do not reach '
                           'stock firmware, which would be left at 1 Mbaud. Open with body_rotation=None to change baud')

    def close(self):
        if self._owns:
            self.bus.close()

    # ------------------------------------------------------------ helpers
    def _xl_positions(self, ids):
        counts = self.table.sync_read('Present_Position', ids)
        return [_to_rad(counts[i]) for i in ids]

    def _xl_goals(self, ids, positions):
        positions = list(positions)
        if len(positions) != len(ids):
            raise ValueError(f'expected {len(ids)} positions, got {len(positions)}')
        self.table.sync_write('Goal_Position', {i: _to_counts(p) for i, p in zip(ids, positions)})

    def _body_required(self):
        if self._body is None:
            raise NotSupported('body rotation (ID 10, XC330) is not driven: open with body_rotation="stock"')
        return self._body

    def _set_torque_ids(self, ids, enable):
        xl = [i for i in ids if i in self._xl_ids]
        if xl:
            self.table.sync_write('Torque_Enable', {i: int(enable) for i in xl})
        if BODY_ROTATION_ID in ids:
            self._body_required().torque(enable)
        unknown = [i for i in ids if i not in ALL_IDS]
        if unknown:
            raise ValueError(f'unknown Reachy Mini motor IDs {unknown}')

    # ------------------------------------------------------------ API
    def get_motor_name_id(self):
        return dict(NAMES)

    def check_missing_ids(self):
        missing = []
        for i in ([] if self._body is None else [BODY_ROTATION_ID]) + self._xl_ids:
            try:
                self.bus.ping(i)
            except Exception:
                missing.append(i)
        return missing

    def read_all_voltages(self):
        xl = self.table.sync_read('Present_Input_Voltage', self._xl_ids)
        body = struct.unpack('<H', self.bus.read(BODY_ROTATION_ID, 144, 2))[0] if self._body else 0
        return [body] + [xl[i] for i in self._xl_ids]

    def read_all_positions(self):
        body = self._body.position() if self._body else math.nan
        return [body] + self._xl_positions(self._xl_ids)

    def set_all_goal_positions(self, positions):
        positions = list(positions)
        if len(positions) != 9:
            raise ValueError(f'expected 9 positions, got {len(positions)}')
        if self._body is not None:
            self._body.goal(positions[0])
        elif positions[0] is not None and not math.isnan(positions[0]):
            self._body_required()
        self._xl_goals(self._xl_ids, positions[1:])

    def set_antennas_positions(self, positions):
        self._xl_goals(ANTENNAS_IDS, positions)

    def set_stewart_platform_position(self, position):
        self._xl_goals(STEWART_PLATFORM_IDS, position)

    def set_body_rotation(self, position):
        self._body_required().goal(position)

    def is_torque_enabled(self):
        xl = self.table.sync_read('Torque_Enable', self._xl_ids)
        body = self._body.torque_enabled() if self._body else True
        return body and all(xl.values())

    def enable_torque(self):
        self._set_torque_ids(([] if self._body is None else [BODY_ROTATION_ID]) + self._xl_ids, True)

    def enable_torque_on_ids(self, ids):
        self._set_torque_ids(list(ids), True)

    def disable_torque(self):
        self._set_torque_ids(([] if self._body is None else [BODY_ROTATION_ID]) + self._xl_ids, False)

    def disable_torque_on_ids(self, ids):
        self._set_torque_ids(list(ids), False)

    def set_stewart_platform_goal_current(self, current):
        current = list(current)
        if len(current) != 6:
            raise ValueError(f'expected 6 currents, got {len(current)}')
        self.table.sync_write('Goal_Current', dict(zip(STEWART_PLATFORM_IDS, current)))

    def read_stewart_platform_current(self):
        c = self.table.sync_read('Present_Current', STEWART_PLATFORM_IDS)
        return [c[i] for i in STEWART_PLATFORM_IDS]

    def _mode(self, ids, mode):
        if mode == MODE_PWM:
            raise NotSupported('Operating mode 16 (PWM control) is not supported (as in the Pollen controller)')
        self.table.sync_write('Operating_Mode', {i: mode for i in ids})

    def set_stewart_platform_operating_mode(self, mode):
        self._mode(STEWART_PLATFORM_IDS, mode)

    def read_stewart_platform_operating_mode(self):
        m = self.table.sync_read('Operating_Mode', STEWART_PLATFORM_IDS)
        return [m[i] for i in STEWART_PLATFORM_IDS]

    def set_antennas_operating_mode(self, mode):
        self._mode(ANTENNAS_IDS, mode)

    def set_body_rotation_operating_mode(self, mode):
        self._body_required().operating_mode(mode)

    def enable_body_rotation(self, enable):
        self._set_torque_ids([BODY_ROTATION_ID], enable)

    def enable_antennas(self, enable):
        self._set_torque_ids(ANTENNAS_IDS, enable)

    def enable_stewart_platform(self, enable):
        self._set_torque_ids(STEWART_PLATFORM_IDS, enable)

    def read_raw_bytes(self, id, addr, length):
        if id in self._xl_ids:
            reg = self._stock_register(addr, length)
            return _pack(self.table.read(id, reg.name), reg)
        return list(self.bus.read(id, addr, length))

    def write_raw_bytes(self, id, addr, data):
        data = bytes(data)
        if id in self._xl_ids:
            reg = self._stock_register(addr, len(data))
            fmt = {1: 'b', 2: 'h', 4: 'i'}[reg.size] if reg.signed else {1: 'B', 2: 'H', 4: 'I'}[reg.size]
            self.table.write(id, reg.name, struct.unpack('<' + fmt, data)[0])
            return
        self.bus.write(id, addr, data)

    def _stock_register(self, addr, length):
        from .dynamixel_table import BY_ADDRESS
        reg = BY_ADDRESS.get(addr)
        if reg is None or reg.size != length:
            raise NotSupported(f'raw access to stock address {addr} ({length} B) on an openxl330 servo: only whole '
                               f'single stock registers are translated')
        return reg

    def async_read_pid_gains(self, id):
        return tuple(self.table.read(id, n) for n in ('Position_P_Gain', 'Position_I_Gain', 'Position_D_Gain'))

    def async_write_pid_gains(self, id, p, i, d):
        for name, v in (('Position_D_Gain', d), ('Position_I_Gain', i), ('Position_P_Gain', p)):
            self.table.write(id, name, v)

    def write_raw_packet(self, data):
        raise NotSupported('write_raw_packet: raw packets bypass the stock-table translation; not available on '
                           'openxl330 servos')


def _pack(value, reg):
    fmt = {1: 'b', 2: 'h', 4: 'i'}[reg.size] if reg.signed else {1: 'B', 2: 'H', 4: 'I'}[reg.size]
    return list(struct.pack('<' + fmt, value))


class ReachyMiniPyControlLoop:
    """Same API as reachy_mini_motor_controller.ReachyMiniPyControlLoop: a background thread reads positions every
    `read_position_loop_period` (seconds or a timedelta) and runs queued commands in between."""

    def __init__(self, serialport, read_position_loop_period, allowed_retries=5, stats_pub_period=None,
                 voltage_rampup_timeout=30.0, **controller_options):
        self._c = ReachyMiniMotorController(serialport, **controller_options)
        self._period = _seconds(read_position_loop_period)
        self._retries = int(allowed_retries)
        self._stats_period = None if stats_pub_period is None else _seconds(stats_pub_period)
        self._lock = threading.Lock()
        self._stats = ControlLoopStats([], [], [])
        self._published = None
        self._last = self._read()
        self._torque = self._c.is_torque_enabled()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='reachy-mini-openxl330', daemon=True)
        self._thread.start()

    def _read(self):
        p = self._c.read_all_positions()
        return FullBodyPosition(p[0], p[1:7], p[7:9], time.time())

    def _run(self):
        last = time.monotonic()
        t0 = last
        while not self._stop.wait(max(0.0, self._period - (time.monotonic() - last))):
            now = time.monotonic()
            for attempt in range(self._retries + 1):
                try:
                    with self._lock:
                        r0 = time.monotonic()
                        self._last = self._read()
                        self._stats.read_dt.append(time.monotonic() - r0)
                    break
                except Exception as e:                       # kept for get_last_position() to raise
                    if attempt == self._retries:
                        self._last = e
            self._stats.period.append(now - last)
            last = now
            if self._stats_period and now - t0 >= self._stats_period:
                self._published, self._stats, t0 = self._stats, ControlLoopStats([], [], []), now

    def _call(self, fn, *args):
        with self._lock:
            w0 = time.monotonic()
            out = fn(*args)
            self._stats.write_dt.append(time.monotonic() - w0)
            return out

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
        self._c.close()

    def get_motor_name_id(self):
        return self._c.get_motor_name_id()

    def get_last_position(self):
        if isinstance(self._last, Exception):
            raise RuntimeError(f'last position read failed: {self._last}')
        return self._last

    def set_all_goal_positions(self, positions):
        p = positions
        values = [p.body_yaw, *p.stewart, *p.antennas] if isinstance(p, FullBodyPosition) else list(p)
        self._call(self._c.set_all_goal_positions, values)

    def set_stewart_platform_position(self, position):
        self._call(self._c.set_stewart_platform_position, position)

    def set_body_rotation(self, position):
        self._call(self._c.set_body_rotation, position)

    def set_antennas_positions(self, positions):
        self._call(self._c.set_antennas_positions, positions)

    def is_torque_enabled(self):
        return self._call(self._c.is_torque_enabled)

    def enable_torque(self):
        self._call(self._c.enable_torque)

    def enable_torque_on_ids(self, ids):
        self._call(self._c.enable_torque_on_ids, ids)

    def disable_torque(self):
        self._call(self._c.disable_torque)

    def disable_torque_on_ids(self, ids):
        self._call(self._c.disable_torque_on_ids, ids)

    def set_stewart_platform_goal_current(self, current):
        self._call(self._c.set_stewart_platform_goal_current, current)

    def get_stewart_platform_operating_mode(self):
        return self._call(self._c.read_stewart_platform_operating_mode)[0]

    def set_stewart_platform_operating_mode(self, mode):
        self._call(self._c.set_stewart_platform_operating_mode, mode)

    def set_antennas_operating_mode(self, mode):
        self._call(self._c.set_antennas_operating_mode, mode)

    def set_body_rotation_operating_mode(self, mode):
        self._call(self._c.set_body_rotation_operating_mode, mode)

    def enable_stewart_platform(self, enable):
        self._call(self._c.enable_stewart_platform, enable)

    def enable_body_rotation(self, enable):
        self._call(self._c.enable_body_rotation, enable)

    def enable_antennas(self, enable):
        self._call(self._c.enable_antennas, enable)

    def get_stats(self):
        return self._published

    def async_read_raw_bytes(self, id, addr, length):
        return self._call(self._c.read_raw_bytes, id, addr, length)

    def async_write_raw_bytes(self, id, addr, data):
        self._call(self._c.write_raw_bytes, id, addr, data)

    def async_read_pid_gains(self, id):
        return self._call(self._c.async_read_pid_gains, id)

    def async_write_pid_gains(self, id, p, i, d):
        self._call(self._c.async_write_pid_gains, id, p, i, d)

    def write_raw_packet(self, data):
        return self._c.write_raw_packet(data)


def _seconds(value):
    return value.total_seconds() if hasattr(value, 'total_seconds') else float(value)
