"""A stock XL330 control-table view of servos running openxl330 firmware, translated on the host.

    from openxl330 import Bus
    from openxl330.compat import StockTable

    table = StockTable(Bus("/dev/ttyUSB0"), [1, 2, 3])
    table.write(1, "Torque_Enable", 1)
    table.sync_write("Goal_Position", {1: 2048, 2: 2100})
    print(table.sync_read("Present_Position", [1, 2, 3]))

Values are stock raw units (counts, 0.229 rpm, mA, 0.1 V, PWM 885 = 100 %). Register names match without case or
underscores ("Goal_Position", "goal_position", "goalposition"). Only openxl330's public API is used; nothing is written
to the servo's NVM: the stock EEPROM settings the host may change (Operating_Mode, Drive_Mode, Homing_Offset, the
position limits, gains) live in this object and are lost when it is dropped, as the firmware forgets its PD
configuration at power-up. Software that rewrites its calibration on connect (LeRobot) needs nothing else.

Unsupported registers and values raise openxl330.NotSupported (never a silent no-op), and the EEPROM-area settings
are refused with openxl330.Refused while torque is on, as on a stock servo.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from ..errors import InvalidArgument, NotSupported, Refused, Rejected
from .. import images
from ..registers import PROTECT
from .model import model_for


def key(name: str) -> str:
    """The lookup key of a register name: lower case without underscores."""
    return name.replace('_', '').lower()


@dataclass(frozen=True)
class Register:
    name: str
    address: int
    size: int
    signed: bool = False
    writable: bool = True
    eeprom: bool = False          # refused while torque is on, as on a stock servo


# The stock XL330 control table (public ROBOTIS e-manual layout), every register the translator knows by name.
REGISTERS = [Register(*r) for r in [
    ('Model_Number', 0, 2, False, False), ('Model_Information', 2, 4, False, False),
    ('Firmware_Version', 6, 1, False, False), ('ID', 7, 1, False, True, True), ('Baud_Rate', 8, 1, False, True, True),
    ('Return_Delay_Time', 9, 1, False, True, True), ('Drive_Mode', 10, 1, False, True, True),
    ('Operating_Mode', 11, 1, False, True, True), ('Secondary_ID', 12, 1, False, True, True),
    ('Protocol_Type', 13, 1, False, True, True), ('Homing_Offset', 20, 4, True, True, True),
    ('Moving_Threshold', 24, 4, False, True, True), ('Temperature_Limit', 31, 1, False, True, True),
    ('Max_Voltage_Limit', 32, 2, False, True, True), ('Min_Voltage_Limit', 34, 2, False, True, True),
    ('PWM_Limit', 36, 2, False, True, True), ('Current_Limit', 38, 2, False, True, True),
    ('Acceleration_Limit', 40, 4, False, True, True), ('Velocity_Limit', 44, 4, False, True, True), ('Max_Position_Limit', 48, 4, True, True, True),
    ('Min_Position_Limit', 52, 4, True, True, True), ('Startup_Configuration', 60, 1, False, True, True),
    ('PWM_Slope', 62, 1, False, True, True), ('Shutdown', 63, 1, False, True, True),
    ('Torque_Enable', 64, 1), ('LED', 65, 1), ('Status_Return_Level', 68, 1),
    ('Registered_Instruction', 69, 1, False, False), ('Hardware_Error_Status', 70, 1, False, False),
    ('Velocity_I_Gain', 76, 2), ('Velocity_P_Gain', 78, 2), ('Position_D_Gain', 80, 2), ('Position_I_Gain', 82, 2),
    ('Position_P_Gain', 84, 2), ('Feedforward_2nd_Gain', 88, 2), ('Feedforward_1st_Gain', 90, 2),
    ('Bus_Watchdog', 98, 1, True), ('Goal_PWM', 100, 2, True), ('Goal_Current', 102, 2, True),
    ('Goal_Velocity', 104, 4, True), ('Profile_Acceleration', 108, 4), ('Profile_Velocity', 112, 4),
    ('Goal_Position', 116, 4, True), ('Realtime_Tick', 120, 2, False, False), ('Moving', 122, 1, False, False),
    ('Moving_Status', 123, 1, False, False), ('Present_PWM', 124, 2, True, False),
    ('Present_Current', 126, 2, True, False), ('Present_Velocity', 128, 4, True, False),
    ('Present_Position', 132, 4, True, False), ('Velocity_Trajectory', 136, 4, True, False),
    ('Position_Trajectory', 140, 4, True, False), ('Present_Input_Voltage', 144, 2, False, False),
    ('Present_Temperature', 146, 1, False, False), ('Backup_Ready', 147, 1, False, False),
]]
BY_KEY = {key(r.name): r for r in REGISTERS}
BY_ADDRESS = {r.address: r for r in REGISTERS}
BY_KEY['torquelimit'] = BY_KEY['currentlimit']       # rustypot alias names
BY_KEY['movingspeed'] = BY_KEY['velocitylimit']

# Stock operating modes, and what openxl330 does for each.
MODE_CURRENT, MODE_VELOCITY, MODE_POSITION, MODE_EXTENDED, MODE_CURRENT_POSITION, MODE_PWM = 0, 1, 3, 4, 5, 16
SUPPORTED_MODES = {MODE_CURRENT: 'torque command (Goal_Current -> N*m through the model)',
                   MODE_POSITION: 'PD position (goals clamped to the position limits)',
                   MODE_EXTENDED: 'PD position, multi-turn',
                   MODE_PWM: 'open-loop duty (Goal_PWM)'}
# Stock Hardware_Error_Status bits for openxl330 faults: 0 input voltage, 2 overheating, 3 motor encoder,
# 4 electrical shock (used for the firmware's integrity faults), 5 overload.
FAULT_TO_HW_ERROR = {'voltage': 0x01, 'temperature': 0x04, 'sensor': 0x08, 'current': 0x20, 'timing': 0x10}
BAUD_CODES = {9600: 0, 57600: 1, 115200: 2, 1_000_000: 3, 2_000_000: 4, 3_000_000: 5, 4_000_000: 6}
ALERT = 0x80                       # status-packet alert bit: a hardware error is latched


def bundled_releases():
    """The releases compat supports: exactly the images this openxl330 bundles."""
    return images.RELEASES


def check_release(bus, servo_id, info):
    """NotSupported unless the servo runs one of the bundled releases (exact pin match, Bus identity)."""
    if info.release not in bundled_releases():
        from ..install import update_command
        raise NotSupported(f'ID {servo_id} runs {info.release or info.firmware}; openxl330.compat supports only the '
                           f'releases this openxl330 bundles ({", ".join(bundled_releases())}): update it with '
                           f'`{update_command(bus.wire, servo_id)}`')


def register(name) -> Register:
    """The Register of a stock name or address; NotSupported if the stock XL330 has none there."""
    reg = BY_ADDRESS.get(name) if isinstance(name, int) else BY_KEY.get(key(str(name)))
    if reg is None:
        raise NotSupported(f'{name!r} is not a stock XL330 control-table register known to openxl330.compat')
    return reg


@dataclass
class _Joint:
    """Host-side copy of the stock settings openxl330 firmware does not keep."""
    operating_mode: int = MODE_POSITION
    drive_mode: int = 0
    homing_offset: int = 0
    min_limit: int = 0
    max_limit: int = 4095
    p_gain: int = 400
    i_gain: int = 0
    d_gain: int = 0
    profile_velocity: int = 0
    profile_acceleration: int = 0
    goal_position: int | None = None
    goal_current: int = 0
    goal_pwm: int = 885
    goal_velocity: int = 0
    led: int = 0
    bus_watchdog: int = 0
    enabled: bool = False          # last commanded / observed Torque_Enable
    extra: dict = field(default_factory=dict)


class StockTable:
    """Stock XL330 registers for openxl330 servos `ids` on `bus` (an openxl330.Bus).

    gains: 'mapped' (default) turns stock Position_P/D_Gain into N*m/rad, N*m*s/rad with model.StockModel's
    approximation at `supply_v` volts (None: the model's nominal 5 V); 'native' keeps openxl330's own tuned defaults
    (configure() defaults) and only stores the stock values for read-back.
    current_position: stock mode 5 (current-based position) caps current by Goal_Current, which openxl330 cannot do;
    it is refused (NotSupported) unless current_position='uncapped', which runs it as plain position control.
    """

    def __init__(self, bus, ids, *, gains='mapped', supply_v=None, current_position='refuse'):
        if gains not in ('mapped', 'native'):
            raise InvalidArgument(f"gains={gains!r}: use 'mapped' or 'native'")
        if current_position not in ('refuse', 'uncapped'):
            raise InvalidArgument(f"current_position={current_position!r}: use 'refuse' or 'uncapped'")
        self.bus = bus
        self.ids = list(ids)
        self.gains = gains
        self.supply_v = supply_v
        self.current_position = current_position
        self._groups = {}
        self._servos = {i: bus.servo(i) for i in self.ids}
        for i, s in self._servos.items():
            check_release(bus, i, s.info)
        self._group = self._g(self.ids)
        self._models = {i: model_for(s.info) for i, s in self._servos.items()}
        self._joints = {}
        for i, m in self._models.items():
            self._joints[i] = self._seeded_joint(i, m)
        for i, st in self._group.status().items():
            self._joints[i].enabled = st.enabled

    # ------------------------------------------------------------------ helpers
    def _seeded_joint(self, servo_id, m) -> _Joint:
        """A _Joint seeded from the servo's own stock EEPROM settings: openxl330 serves its NV journal as a stock EEPROM
        view at 0..63 (Drive_Mode 10, Operating_Mode 11, Homing_Offset 20, Max/Min_Position_Limit 48/52). A position
        limit pair that reads 0/0 is an absent key: the stock defaults apply.

        The firmware already applies the EEPROM Homing_Offset and Drive_Mode reverse to every position, velocity, current
        and duty, as stock firmware does: with no host write the table reports firmware values unchanged. A host write
        replaces the EEPROM value (stock semantics), and only the difference is applied on the host."""
        import struct
        ee = self.bus.read(servo_id, 0, 64)
        drive_mode, mode = ee[10], ee[11]
        homing, = struct.unpack_from('<i', ee, 20)
        max_limit, min_limit = struct.unpack_from('<ii', ee, 48)
        if max_limit == 0 and min_limit == 0:
            max_limit, min_limit = m.resolution - 1, 0
        return _Joint(operating_mode=mode, drive_mode=drive_mode, homing_offset=homing, min_limit=min_limit,
                      max_limit=max_limit, p_gain=m.position_p_gain, d_gain=m.position_d_gain,
                      extra={'ee_reverse': drive_mode & 1, 'ee_homing': homing})

    def _g(self, ids):
        ids = tuple(ids)
        if ids not in self._groups:
            self._groups[ids] = self.bus.group(list(ids))
        return self._groups[ids]

    def _check(self, servo_id):
        if servo_id not in self._joints:
            raise InvalidArgument(f'ID {servo_id} is not one of this table\'s servos {self.ids}')
        return self._joints[servo_id]

    @property
    def _cal(self):
        return self.bus.calibration

    def _dm(self, servo_id) -> int:
        """-1 when the host Drive_Mode reverse differs from the EEPROM one the firmware already applies, else 1."""
        j = self._joints[servo_id]
        return -1 if (j.drive_mode & 1) ^ j.extra.get('ee_reverse', 0) else 1

    def _stock_from_counts(self, servo_id, counts: int) -> int:
        """Stock Present_Position from firmware counts (which carry the EEPROM offset and reverse)."""
        j, m = self._joints[servo_id], self._models[servo_id]
        base = counts - j.extra.get('ee_homing', 0)
        if self._dm(servo_id) < 0:
            base = m.resolution - base                # mirror as FW231 does (4096 - x), keeps a turn in range
        return base + j.homing_offset

    def _counts_from_stock(self, servo_id, stock: int) -> int:
        j, m = self._joints[servo_id], self._models[servo_id]
        base = stock - j.homing_offset
        if self._dm(servo_id) < 0:
            base = m.resolution - base
        return base + j.extra.get('ee_homing', 0)

    def _rad_of_stock(self, servo_id, stock: int) -> float:
        """openxl330 calibrated radians of a stock position value (exact: counts round-trip through calibration)."""
        return self._cal.to_rad(servo_id, self._counts_from_stock(servo_id, stock))

    def _nm(self, servo_id, raw_direction_nm: float) -> float:
        """openxl330 calibrated-direction torque from a torque in firmware (encoder) direction under the drive mode."""
        return self._cal.sign(servo_id) * self._dm(servo_id) * raw_direction_nm

    def _kp_kd(self, servo_id):
        """(kp, kd) for configure()/set_gains(). Mapped Kd is floored at openxl330's default KD_COMP while the image's
        gear-train cancellation is loaded: the cancellation removes the back-EMF, viscous and most friction damping a
        stock P-only gain relies on, so Kd below the default rings or oscillates. A default, not a limit: write
        gains='native' or a larger D gain to change it."""
        if self.gains == 'native':
            return None, None
        j, m = self._joints[servo_id], self._models[servo_id]
        kd = m.kd_from_stock(j.d_gain, self.supply_v)
        if j.extra.get('comp', True):
            kd = max(kd, m.kd_default)
        return m.kp_from_stock(j.p_gain, self.supply_v), kd

    def _duration(self, servo_id, goal: int, present: int) -> float:
        """Stock profile -> quintic duration: peak quintic speed 1.875 d/T and acceleration 5.774 d/T^2 matched to
        Profile_Velocity / Profile_Acceleration (approximation of the stock trapezoid). 0 velocity = a step."""
        j, m = self._joints[servo_id], self._models[servo_id]
        if j.profile_velocity == 0:
            return 0.0
        d = abs(goal - present) / m.counts_per_rad
        t = 1.875 * d / m.velocity_from_stock(j.profile_velocity)
        if j.profile_acceleration:
            t = max(t, math.sqrt(5.7735 * d / m.accel_from_stock(j.profile_acceleration)))
        return t

    def _protections(self, servo_id):
        import struct
        return struct.unpack('<4I', self.bus.read(servo_id, PROTECT, 16))

    def _check_range(self, servo_id, reg, k, v):
        """Rejected with error 4 (data range), as a stock servo answers, for a value outside the register or its stock
        limit."""
        m = self._models[servo_id]
        bits = 8 * reg.size
        lo, hi = (-(1 << bits - 1), (1 << bits - 1) - 1) if reg.signed else (0, (1 << bits) - 1)
        if k in ('torqueenable', 'led'):
            lo, hi = 0, 1
        elif k == 'goalcurrent':
            lo, hi = -m.current_limit_ma, m.current_limit_ma
        elif k == 'goalpwm':
            lo, hi = -m.pwm_full, m.pwm_full
        elif k == 'buswatchdog':
            lo, hi = 0, 127
        if not lo <= v <= hi:
            raise Rejected(servo_id, 4)

    def _refuse_if_enabled(self, servo_id, reg):
        if self._joints[servo_id].enabled:
            raise Refused(f'{reg.name} of ID {servo_id} not written: it is an EEPROM-area setting, writable only with '
                          f'Torque_Enable 0 (as on a stock servo)')

    def _fixed(self, servo_id, reg, value, current):
        if value != current:
            raise NotSupported(f'{reg.name} = {value} on ID {servo_id}: openxl330 firmware has no such setting; '
                               f'only the value it reports ({current}) can be written')

    # ------------------------------------------------------------------ reads
    def read(self, servo_id, name) -> int:
        """One stock register of one servo, in stock raw units."""
        return self.sync_read(name, [servo_id])[servo_id]

    def sync_read(self, name, ids=None) -> dict:
        """{id: value} of one stock register; motion and status registers come from one group read."""
        reg = register(name)
        ids = self.ids if ids is None else list(ids)
        for i in ids:
            self._check(i)
        k = key(reg.name)
        if k in ('presentposition', 'presentvelocity', 'presentcurrent', 'presentpwm', 'moving'):
            states = self._g(ids).read()
            return {i: self._motion_value(i, k, states[i]) for i in ids}
        if k in ('hardwareerrorstatus', 'torqueenable', 'presentinputvoltage', 'presenttemperature'):
            status = self._g(ids).status()
            out = {}
            for i in ids:
                st = status[i]
                self._joints[i].enabled = st.enabled
                if k == 'hardwareerrorstatus':
                    out[i] = self._hw_error(st)
                elif k == 'torqueenable':
                    out[i] = int(st.enabled)
                elif k == 'presentinputvoltage':
                    out[i] = round(st.voltage * 10)
                else:
                    out[i] = round(st.temperature)
            return out
        return {i: self._static_value(i, reg) for i in ids}

    def _hw_error(self, st) -> int:
        bits = 0
        for name in st.faults:
            bits |= FAULT_TO_HW_ERROR.get(name, 0x10)
        if 'supply_high' in getattr(st, 'warnings', ()):
            bits |= FAULT_TO_HW_ERROR['voltage']          # stock input-voltage bit; clears with the warning
        return bits

    def alert(self, servo_id) -> int:
        """The status-packet error byte a stock servo would send: ALERT when a hardware error is latched."""
        return ALERT if self.sync_read('Hardware_Error_Status', [servo_id])[servo_id] else 0

    def _motion_value(self, servo_id, k, state) -> int:
        m, raw, dm = self._models[servo_id], state.raw, self._dm(servo_id)
        if k == 'presentposition':
            return self._stock_from_counts(servo_id, raw['position_counts'])
        if k == 'presentvelocity':
            return dm * m.velocity_to_stock(raw['velocity_rad_s'])
        if k == 'presentcurrent':
            return dm * round(raw['current_ma'])
        if k == 'presentpwm':
            return dm * round(raw['duty'] * m.pwm_full)
        threshold = self._joints[servo_id].extra.get('moving_threshold', 10)
        return int(abs(m.velocity_to_stock(raw['velocity_rad_s'])) > threshold)

    def _static_value(self, servo_id, reg) -> int:
        j, m, s = self._joints[servo_id], self._models[servo_id], self._servos[servo_id]
        k = key(reg.name)
        simple = {
            'modelnumber': m.model_number, 'modelinformation': 0, 'id': servo_id, 'returndelaytime': 0,
            'drivemode': j.drive_mode, 'operatingmode': j.operating_mode, 'secondaryid': 255, 'protocoltype': 2,
            'homingoffset': j.homing_offset, 'movingthreshold': j.extra.get('moving_threshold', 10),
            'minvoltagelimit': 35, 'pwmlimit': m.pwm_full, 'currentlimit': m.current_limit_ma,
            'velocitylimit': m.velocity_limit, 'maxpositionlimit': j.max_limit, 'minpositionlimit': j.min_limit,
            'startupconfiguration': 0, 'pwmslope': 140, 'shutdown': 0x34, 'led': j.led, 'statusreturnlevel': 2,
            'registeredinstruction': 0, 'velocityigain': m.velocity_i_gain, 'velocitypgain': m.velocity_p_gain,
            'positiondgain': j.d_gain, 'positionigain': j.i_gain, 'positionpgain': j.p_gain,
            'feedforward2ndgain': 0, 'feedforward1stgain': 0, 'buswatchdog': j.bus_watchdog, 'goalpwm': j.goal_pwm,
            'goalcurrent': j.goal_current, 'goalvelocity': j.goal_velocity,
            'profileacceleration': j.profile_acceleration, 'profilevelocity': j.profile_velocity,
            'realtimetick': int(time.monotonic() * 1000) % 32768, 'backupready': 0,
        }
        if k in simple:
            return simple[k]
        if k == 'firmwareversion':
            return 231
        if k == 'baudrate':
            return BAUD_CODES.get(self.bus.port.baudrate, 3)
        if k == 'goalposition':
            if j.goal_position is None:
                return self.read(servo_id, 'Present_Position')
            return j.goal_position
        if k in ('temperaturelimit', 'maxvoltagelimit'):
            _, temp_c, vmax_dv, _ = self._protections(servo_id)
            return temp_c if k == 'temperaturelimit' else vmax_dv
        raise NotSupported(f'{reg.name} of ID {servo_id}: openxl330 firmware does not provide it (no stock '
                           f'equivalent is computed by openxl330.compat)')

    # ------------------------------------------------------------------ writes
    def write(self, servo_id, name, value, *, verify=True):
        """One stock register of one servo (acknowledged: verify=True checks the result where openxl330 can)."""
        self.sync_write(name, {servo_id: value}, verify=verify)

    def sync_write(self, name, values: dict, *, verify=False):
        """Write one stock register of several servos. Goal_Position goes out as one openxl330 group move
        (verify=False: one Sync Write, no replies, like a stock Sync Write)."""
        reg = register(name)
        if not reg.writable:
            raise NotSupported(f'{reg.name} is read-only on a stock XL330')
        values = {i: _int(v, reg.name) for i, v in values.items()}
        for i in values:
            self._check(i)
        k = key(reg.name)
        for i, v in values.items():
            self._check_range(i, reg, k, v)
        if k == 'torqueenable':
            return self._torque_enable(values)
        if k == 'goalposition':
            return self._goal_position(values, verify)
        if k == 'goalcurrent':
            return self._goal_current(values, verify)
        if k == 'goalpwm':
            return self._goal_pwm(values, verify)
        for i, v in values.items():
            self._write_one(i, reg, k, v)

    def _write_one(self, i, reg, k, v):
        j, m = self._joints[i], self._models[i]
        if reg.eeprom:
            self._refuse_if_enabled(i, reg)
        if k == 'operatingmode':
            if v == MODE_CURRENT_POSITION and self.current_position == 'uncapped':
                j.operating_mode = v
                return
            if v not in SUPPORTED_MODES:
                hint = (" (pass current_position='uncapped' to run it as plain position control, Goal_Current is then "
                        "not a cap)" if v == MODE_CURRENT_POSITION else '')
                raise NotSupported(f'Operating_Mode {v} on ID {i}: openxl330 supports {sorted(SUPPORTED_MODES)} '
                                   f'(current, position, extended position, PWM){hint}')
            j.operating_mode = v
        elif k == 'drivemode':
            if v & ~0x01:
                raise NotSupported(f'Drive_Mode {v} on ID {i}: only bit 0 (reverse) is supported; the time-based '
                                   f'profile (bit 2) and torque-on-by-goal (bit 3) are not')
            j.drive_mode = v
        elif k == 'homingoffset':
            j.homing_offset = v
        elif k == 'minpositionlimit':
            j.min_limit = v
        elif k == 'maxpositionlimit':
            j.max_limit = v
        elif k == 'movingthreshold':
            j.extra['moving_threshold'] = v
        elif k == 'positionpgain' or k == 'positiondgain':
            setattr(j, 'p_gain' if k == 'positionpgain' else 'd_gain', v)
            if j.enabled and self.gains == 'mapped':
                self._servos[i].set_gains(*self._kp_kd(i))
        elif k == 'positionigain':
            self._fixed(i, reg, v, 0)
        elif k == 'profilevelocity':
            j.profile_velocity = v
        elif k == 'profileacceleration':
            j.profile_acceleration = v
        elif k == 'led':
            self._servos[i].led = bool(v)
            j.led = int(bool(v))
        elif k == 'buswatchdog':
            if v > 0:
                self._servos[i].lease(v * 0.02, 'hold')
            elif v == 0:
                self._servos[i].lease(0)
            j.bus_watchdog = max(v, 0)
        elif k == 'goalvelocity':
            self._fixed(i, reg, v, 0)
        elif k == 'returndelaytime':
            self._fixed(i, reg, v, 0)
        elif k in ('id', 'baudrate'):
            raise NotSupported(f'{reg.name} on ID {i}: set IDs and baud with openxl330 (Bus.set_baud; IDs in '
                               f'Dynamixel Wizard before installing, or the openxl330 ID tools), not through the '
                               f'stock table')
        elif k in ('temperaturelimit', 'maxvoltagelimit'):
            raise NotSupported(f'{reg.name} on ID {i}: change openxl330 protections through register 848 (see '
                               f'docs/api.md#protections); the stock table does not write them')
        else:
            self._fixed(i, reg, v, self._static_value(i, reg))

    def _torque_enable(self, values):
        on = [i for i, v in values.items() if v]
        off = [i for i, v in values.items() if not v]
        if off:
            self._g(off).disable()
            for i in off:
                self._joints[i].enabled = False
        for i in on:
            j = self._joints[i]
            if j.operating_mode == MODE_CURRENT_POSITION and self.current_position != 'uncapped':
                raise NotSupported(f'ID {i} is in Operating_Mode 5, which openxl330 does not support')
        if not on:
            return
        status = self._g(on).status()
        newly = [i for i in on if not status[i].enabled]
        for i in newly:                                  # configure() needs OFF; once per enable is cheap and safe
            j = self._joints[i]
            if j.drive_mode & ~0x01:
                raise NotSupported(f'ID {i} has Drive_Mode {j.drive_mode} (from its stock EEPROM): only bit 0 (reverse) '
                                   f'is supported; write Drive_Mode 0 or 1 first')
            if j.operating_mode not in SUPPORTED_MODES and j.operating_mode != MODE_CURRENT_POSITION:
                raise NotSupported(f'ID {i} is in Operating_Mode {j.operating_mode} (from its stock EEPROM), which '
                                   f'openxl330 does not support: write Operating_Mode 3, 4, 0 or 16 first')
            done = self._servos[i].configure(*self._kp_kd(i))
            if j.extra.get('comp', True) != bool(done.get('comp')):
                j.extra['comp'] = bool(done.get('comp'))
                self._servos[i].configure(*self._kp_kd(i))
        if newly:
            self._g(newly).enable()
        for i in on:
            j = self._joints[i]
            j.enabled = True
            j.goal_position = None
            if j.operating_mode == MODE_CURRENT:
                self._send_current({i: j.goal_current}, verify=True)
            elif j.operating_mode == MODE_PWM:
                self._send_pwm({i: j.goal_pwm})

    def _goal_position(self, values, verify):
        plan = {}
        for i, v in values.items():
            j = self._joints[i]
            if j.operating_mode not in (MODE_POSITION, MODE_EXTENDED, MODE_CURRENT_POSITION):
                raise NotSupported(f'Goal_Position on ID {i} in Operating_Mode {j.operating_mode}: switch to position '
                                   f'mode (3/4) first')
            if j.operating_mode != MODE_EXTENDED:
                v = min(max(v, j.min_limit), j.max_limit)          # stock clamps goals to the limits in mode 3
            j.goal_position = v
            if j.enabled:
                plan[i] = v
        if not plan:
            return                                               # stock keeps a goal written with torque off
        profiled = [i for i in plan if self._joints[i].profile_velocity]
        present = self.sync_read('Present_Position', profiled) if profiled else {}
        by_duration = {}
        for i, v in plan.items():
            t = self._duration(i, v, present[i]) if i in present else 0.0
            by_duration.setdefault(round(t, 4), {})[i] = self._rad_of_stock(i, v)
        for t, targets in by_duration.items():
            self._g(list(targets)).move(targets, duration=t, verify=verify)

    def _goal_current(self, values, verify):
        live = {}
        for i, v in values.items():
            j = self._joints[i]
            j.goal_current = v
            if j.operating_mode == MODE_CURRENT and j.enabled:
                live[i] = v
        if live:
            self._send_current(live, verify)

    def _send_current(self, values, verify):
        torques = {i: self._nm(i, v / 1000 * self._models[i].torque_per_amp_nm) for i, v in values.items()}
        self._g(list(torques)).set_torque(torques, verify=verify)

    def _goal_pwm(self, values, verify):
        live = {}
        for i, v in values.items():
            j, m = self._joints[i], self._models[i]
            if j.operating_mode != MODE_PWM and abs(v) != m.pwm_full:
                raise NotSupported(f'Goal_PWM {v} on ID {i} in Operating_Mode {j.operating_mode}: outside PWM mode a '
                                   f'stock Goal_PWM is a duty ceiling, which openxl330 firmware does not have; only '
                                   f'{m.pwm_full} (no ceiling) is accepted')
            j.goal_pwm = v
            if j.operating_mode == MODE_PWM and j.enabled:
                live[i] = v
        if live:
            self._send_pwm(live)

    def _send_pwm(self, values):
        for i, v in values.items():
            m = self._models[i]
            duty = max(-1.0, min(1.0, v / m.pwm_full))
            self._servos[i].set_duty(self._cal.sign(i) * self._dm(i) * duty)


def _int(value, what) -> int:
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise InvalidArgument(f'{what} = {value!r}: not a finite number')
        return int(value)
    if not isinstance(value, int):
        raise InvalidArgument(f'{what} = {value!r}: stock registers take integers')
    return value
