"""LeRobot on openxl330: route a LeRobot `SerialMotorsBus` (or the deprecated `DynamixelMotorsBus`) to openxl330.

    from lerobot.robots.koch_follower import KochFollower, KochFollowerConfig
    from openxl330.compat import lerobot as oxl

    robot = KochFollower(KochFollowerConfig(port="/dev/ttyUSB0"))
    oxl.patch_bus(robot.bus)          # before connect(); everything else is unchanged
    robot.connect()

LeRobot 0.7 does all servo I/O through one rustypot `Bus` it opens in `SerialMotorsBus._open_bus()`. patch_bus()
replaces that method on one bus instance with a `RustypotBackend`, which answers the same calls from a StockTable:
LeRobot's names, normalisation, calibration and recommended settings run unchanged. LeRobot still needs rustypot
installed for the motor definitions (`pip install "openxl330[lerobot]"`). Unsupported registers and modes raise
openxl330.NotSupported, which LeRobot does not catch, so they surface as-is.
"""
from __future__ import annotations

from ..errors import BusUncertain, InvalidArgument, NoReply, NotSupported
from .dynamixel_table import StockTable
from .model import model_for

LEROBOT_MODELS = ('xl330-m288', 'xl330-m077')
SUPPORTED_LEROBOT = ((0, 7), (0, 8))          # [min, max): patch_bus replaces LeRobot's private _open_bus()


def _check_lerobot_version():
    try:
        from importlib.metadata import PackageNotFoundError, version
        text = version('lerobot')
    except PackageNotFoundError:
        return                                  # not installed as a distribution (tests with stand-ins)
    parts = tuple(int(p) for p in text.split('.')[:2] if p.isdigit())
    lo, hi = SUPPORTED_LEROBOT
    if not lo <= parts < hi:
        raise NotSupported(f'LeRobot {text}: openxl330.compat.lerobot is tested with LeRobot 0.7.x only (it replaces '
                           f'the private SerialMotorsBus._open_bus)')


class RustypotBackend:
    """The subset of `rustypot.Bus` LeRobot's SerialMotorsBus calls, answered by openxl330.

    bus: an openxl330.Bus (it is closed by close() only if `owns_bus`). ids: the servo IDs to serve. Extra keyword
    arguments go to StockTable (gains=, supply_v=, current_position=)."""

    def __init__(self, bus, ids, *, owns_bus=False, **table_options):
        self._bus = bus
        self._owns = owns_bus
        self.table = StockTable(bus, ids, **table_options)

    # ---------------------------------------------------------------- port
    def is_open(self) -> bool:
        return self._bus is not None

    def close(self):
        if self._bus is not None and self._owns:
            self._bus.close()
        self._bus = None

    def set_timeout(self, timeout):
        """No-op: openxl330 waits per transaction with its own timeouts."""

    def set_baudrate(self, baudrate):
        if baudrate != self._bus.port.baudrate:
            raise NotSupported(f'set_baudrate({baudrate}): the openxl330 bus runs at {self._bus.port.baudrate}; change '
                               f'the servos and the port together with openxl330.Bus.set_baud()')

    def change_baudrate(self, *args, **kwargs):
        raise NotSupported('change_baudrate: use openxl330.Bus.set_baud()')

    def change_id(self, *args, **kwargs):
        raise NotSupported('change_id: IDs are set on the stock servo (Dynamixel Wizard) before installing openxl330')

    def reset_turn_count(self, *args, **kwargs):
        raise NotSupported('reset_turn_count: openxl330 positions are continuous multi-turn counts; use a Homing_Offset')

    # ---------------------------------------------------------------- scan
    def scan(self, definition=None) -> dict:
        """{id: stock model number} of every openxl330 servo answering at the current baud."""
        out = {}
        for info in self._bus.scan():
            if info.supported:
                try:
                    out[info.id] = model_for(info).model_number
                except NotSupported:
                    continue
        return out

    broadcast_scan = scan

    # ---------------------------------------------------------------- registers
    def read_register_with_error(self, motor_id, name, retries=0):
        value = self.table.read(motor_id, name)
        return value, self._status(motor_id, name)

    def read_register(self, motor_id, name, retries=0):
        return self.table.read(motor_id, name)

    def write_register_with_error(self, motor_id, name, value, retries=0):
        self.table.write(motor_id, name, value)
        return self._status(motor_id, name)

    def write_register(self, motor_id, name, value, retries=0):
        self.table.write(motor_id, name, value)

    def sync_read_register_with_error(self, ids, name, retries=0):
        values = self.table.sync_read(name, ids)
        return [(values[i], 0) for i in ids]

    def sync_read_register(self, ids, name, retries=0):
        values = self.table.sync_read(name, ids)
        return [values[i] for i in ids]

    def sync_read_raw_with_error(self, *args, **kwargs):
        raise NotSupported('sync_read_raw_with_error: raw stock addresses are not translated; read registers by name')

    def sync_write_register(self, ids, name, values, retries=0):
        ids, values = list(ids), list(values)
        if len(ids) != len(values):
            raise InvalidArgument(f'sync_write_register: {len(ids)} ids but {len(values)} values')
        self.table.sync_write(name, dict(zip(ids, values)))

    def set_torque(self, ids, enabled, retries=0) -> dict:
        """{id: error} of the servos that failed (empty when all went through), every servo tried, as rustypot."""
        failed = {}
        for i in ids:
            try:
                self.table.write(i, 'Torque_Enable', int(bool(enabled)))
            except (NoReply, BusUncertain) as e:         # per-motor transport failures are reported, as rustypot does;
                failed[i] = f'{type(e).__name__}: {e}'   # NotSupported / Faulted / Refused raise at once
        return failed

    def _status(self, motor_id, name):
        """The stock status-packet error byte. Always 0: an openxl330 refusal raises instead, and faults are read from
        Hardware_Error_Status (StockTable.alert() gives the stock alert bit on demand)."""
        return 0


def patch_bus(motors_bus, bus=None, **table_options):
    """Make one LeRobot SerialMotorsBus / DynamixelMotorsBus talk to openxl330 firmware. Call before connect().

    bus: an open openxl330.Bus to use (e.g. Bus.simulated(n) in tests); None opens openxl330.Bus(motors_bus.port) at
    connect() and closes it at disconnect(). table_options go to StockTable (gains='mapped'|'native', supply_v=,
    current_position='refuse'|'uncapped'). Returns motors_bus."""
    if not hasattr(type(motors_bus), '_open_bus'):
        raise NotSupported(f'{type(motors_bus).__name__} has no _open_bus(): openxl330.compat.lerobot needs LeRobot 0.7 '
                           f'or later (earlier releases call dynamixel_sdk directly)')
    _check_lerobot_version()
    models = {m.model for m in motors_bus.motors.values()}
    if not models <= set(LEROBOT_MODELS):
        raise NotSupported(f'openxl330 drives XL330 servos only; this bus has {sorted(models)}')
    ids = [m.id for m in motors_bus.motors.values()]

    def _open_bus():
        if bus is not None:
            return RustypotBackend(bus, ids, **table_options)
        from ..bus import Bus
        own = Bus(motors_bus.port)
        try:
            return RustypotBackend(own, ids, owns_bus=True, **table_options)
        except Exception:
            own.close()
            raise

    motors_bus._open_bus = _open_bus
    motors_bus._openxl330 = True
    return motors_bus


def make_bus(port, motors, calibration=None, *, bus=None, **table_options):
    """A LeRobot SerialMotorsBus already patched for openxl330 (lerobot must be installed)."""
    try:
        from lerobot.motors import SerialMotorsBus
    except ImportError as e:                                # pragma: no cover - depends on the environment
        raise ImportError('openxl330.compat.lerobot.make_bus needs LeRobot: pip install "openxl330[lerobot]"') from e
    return patch_bus(SerialMotorsBus(port, motors, calibration), bus, **table_options)
