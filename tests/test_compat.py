"""openxl330.compat: the stock-table translator, the LeRobot backend and the Reachy Mini API, on simulated buses."""
from __future__ import annotations

import math
import struct
import time

import pytest

from openxl330 import Bus, NotSupported, Refused, images, sim
from openxl330.calibration import Calibration
from openxl330.compat import XL330_M077, XL330_M288, StockTable, model_for, register
from openxl330.compat import dynamixel_table as dt
from openxl330.compat.lerobot import RustypotBackend, patch_bus
from openxl330.compat.reachy_mini import (FullBodyPosition, ReachyMiniMotorController, ReachyMiniPyControlLoop,
                                          _StockRaw, _to_counts, _to_rad)


def _settle(bus, seconds=0.4):
    time.sleep(seconds)                  # SimClock follows real time


FAST17 = dict(number=17, variant='comp', abi=5, source_id=0x80d4d494)   # fast17 rev b stand-in (as test_voltage)


def sim_bus(ids, release_id=None):
    program = images.bundled(images.DEFAULT_RELEASE).packaged().raw
    extra = {} if release_id is None else {'release_id': release_id}
    servos = [sim.SimServo(i, mode='app', program=program, seed=i, **extra) for i in ids]
    return Bus(sim.SimPort(servos, sim.SimClock()), calibration=Calibration())


@pytest.fixture
def bus():
    with Bus.simulated(3) as b:
        yield b


@pytest.fixture
def table(bus):
    return StockTable(bus, [1, 2, 3])


# ---------------------------------------------------------------------------------------------- register table
def test_register_lookup_ignores_case_and_underscores():
    assert register('Goal_Position') is register('goal_position') is register('GOALPOSITION') is register(116)
    assert register('torque_limit') is register('Current_Limit')
    with pytest.raises(NotSupported):
        register('Nonexistent_Register')


def test_register_table_matches_rustypot_xl330_definition():
    rustypot = pytest.importorskip('rustypot')
    definition = rustypot.find_model('xl330-m288')[1]
    for r in definition.registers():
        if r.name.startswith('indirect'):
            continue
        ours = register(r.name)
        assert (ours.address, ours.size) == (r.addr, r.size), r.name


def test_model_constants_and_gain_mapping_round_trip():
    assert model_for('XL330-M288') is XL330_M288 and model_for('XL330-M077') is XL330_M077
    with pytest.raises(NotSupported):
        model_for('XC330-T288')
    kp = XL330_M288.kp_from_stock(400)
    assert 0.4 < kp < 0.8                                   # bench: stock P400 ~ 0.5-0.6 N*m/rad stiffness
    assert XL330_M288.p_to_stock(kp) == 400
    assert XL330_M288.d_to_stock(XL330_M288.kd_from_stock(600)) == 600
    assert XL330_M288.kp_from_stock(400, supply_v=7.4) > kp  # stock gains act on duty: stiffer at higher supply
    assert math.isclose(XL330_M288.velocity_from_stock(XL330_M288.velocity_to_stock(3.0)), 3.0, rel_tol=1e-2)


# ---------------------------------------------------------------------------------------------- reads
def test_identity_and_static_registers(table):
    assert table.read(1, 'Model_Number') == 1200
    assert table.read(2, 'ID') == 2
    assert table.read(1, 'Baud_Rate') == 3
    assert table.read(1, 'Operating_Mode') == 3
    assert table.read(1, 'Max_Voltage_Limit') == 80                    # fast17: 8.0 V default
    assert table.read(1, 'Temperature_Limit') == 65
    assert table.read(1, 'Torque_Enable') == 0
    assert table.read(1, 'Hardware_Error_Status') == 0
    assert 40 <= table.read(1, 'Present_Input_Voltage') <= 60


def test_present_position_is_raw_counts_plus_homing_offset(table, bus):
    raw = bus.group([1]).read()[1].raw['position_counts']
    assert table.read(1, 'Present_Position') == raw
    table.write(1, 'Homing_Offset', 100)
    assert table.read(1, 'Present_Position') == raw + 100
    table.write(1, 'Drive_Mode', 1)
    assert table.read(1, 'Present_Position') == 4096 - raw + 100


def test_unsupported_reads_raise(table):
    for name in ('Moving_Status', 'Velocity_Trajectory', 'Position_Trajectory'):
        with pytest.raises(NotSupported):
            table.read(1, name)
    with pytest.raises(NotSupported):
        table.write(1, 'Present_Position', 5)


# ---------------------------------------------------------------------------------------------- position control
def test_enable_and_goal_position_moves_in_counts(table):
    table.sync_write('Torque_Enable', {1: 1, 2: 1, 3: 1})
    assert table.sync_read('Torque_Enable') == {1: 1, 2: 1, 3: 1}
    start = table.sync_read('Present_Position')
    table.sync_write('Goal_Position', {i: p + 300 for i, p in start.items()})
    _settle(table.bus)
    end = table.sync_read('Present_Position')
    for i in start:
        assert abs(end[i] - (start[i] + 300)) <= 3
    assert table.read(1, 'Goal_Position') == start[1] + 300


def test_drive_mode_reverses_motion(table, bus):
    table.write(1, 'Drive_Mode', 1)
    table.write(1, 'Torque_Enable', 1)
    raw0 = bus.group([1]).read()[1].raw['position_counts']
    table.write(1, 'Goal_Position', table.read(1, 'Present_Position') + 200)
    _settle(bus)
    raw1 = bus.group([1]).read()[1].raw['position_counts']
    assert abs((raw0 - raw1) - 200) <= 3                     # stock reverse: +goal turns the encoder backwards


def test_mode3_clamps_to_limits_mode4_does_not(table):
    table.write(1, 'Max_Position_Limit', 2500)
    table.write(1, 'Torque_Enable', 1)
    table.write(1, 'Goal_Position', 3500)
    assert table.read(1, 'Goal_Position') == 2500
    table.write(1, 'Torque_Enable', 0)
    table.write(1, 'Operating_Mode', 4)
    table.write(1, 'Torque_Enable', 1)
    table.write(1, 'Goal_Position', 4300)
    assert table.read(1, 'Goal_Position') == 4300
    _settle(table.bus, 0.8)
    assert abs(table.read(1, 'Present_Position') - 4300) <= 5


def test_goal_written_with_torque_off_is_stored_not_sent(table, bus):
    p0 = table.read(1, 'Present_Position')
    table.write(1, 'Goal_Position', p0 + 500)
    assert table.read(1, 'Goal_Position') == p0 + 500
    _settle(bus, 0.2)
    assert abs(table.read(1, 'Present_Position') - p0) <= 2
    table.write(1, 'Torque_Enable', 1)                     # enabling holds where the servo is, as openxl330 does
    _settle(bus, 0.2)
    assert abs(table.read(1, 'Present_Position') - p0) <= 2


def test_profile_velocity_turns_into_a_timed_move(table, bus):
    table.write(1, 'Profile_Velocity', 50)                  # 11.45 rpm
    table.write(1, 'Torque_Enable', 1)
    p0 = table.read(1, 'Present_Position')
    expected = table._duration(1, p0 + 1024, p0)
    assert 2.4 < expected < 2.5                             # 1.875 * (pi/2 rad) / 1.199 rad/s
    table.write(1, 'Goal_Position', p0 + 1024)
    back = struct.unpack('<iIII', bus.read(1, 256, 16))
    assert back[1] == round(expected * 2000)


def test_eeprom_settings_refused_while_enabled(table):
    table.write(1, 'Torque_Enable', 1)
    for name, value in (('Operating_Mode', 4), ('Homing_Offset', 5), ('Drive_Mode', 1), ('Min_Position_Limit', 0)):
        with pytest.raises(Refused):
            table.write(1, name, value)


# ---------------------------------------------------------------------------------------------- gains
def test_mapped_gains_configure_and_live_update(table, bus):
    table.write(1, 'Position_P_Gain', 800)
    table.write(1, 'Position_D_Gain', 400)
    table.write(1, 'Torque_Enable', 1)
    calls = []
    servo = table._servos[1]
    orig = servo.set_gains
    servo.set_gains = lambda kp, kd: (calls.append((kp, kd)), orig(kp, kd))
    table.write(1, 'Position_P_Gain', 1600)
    assert calls and math.isclose(calls[0][0], XL330_M288.kp_from_stock(1600))
    assert math.isclose(calls[0][1], max(XL330_M288.kd_from_stock(400), 0.10))
    assert table.read(1, 'Position_P_Gain') == 1600


def test_native_gains_keep_openxl330_defaults(bus):
    t = StockTable(bus, [1], gains='native')
    configured = []
    orig = t._servos[1].configure
    t._servos[1].configure = lambda kp=None, kd=None: (configured.append((kp, kd)), orig(kp, kd))[1]
    t.write(1, 'Position_P_Gain', 2000)
    t.write(1, 'Torque_Enable', 1)
    assert configured == [(None, None)]
    assert t.read(1, 'Position_P_Gain') == 2000


def test_integral_and_feedforward_gains_fail_loudly(table):
    table.write(1, 'Position_I_Gain', 0)
    with pytest.raises(NotSupported):
        table.write(1, 'Position_I_Gain', 10)
    table.write(1, 'Feedforward_1st_Gain', 0)
    with pytest.raises(NotSupported):
        table.write(1, 'Feedforward_1st_Gain', 100)
    with pytest.raises(NotSupported):
        table.write(1, 'Velocity_P_Gain', 100)


# ---------------------------------------------------------------------------------------------- modes
def test_current_mode_commands_torque_through_the_model(table, bus):
    table.write(1, 'Operating_Mode', 0)
    table.write(1, 'Goal_Current', 200)
    table.write(1, 'Torque_Enable', 1)
    mode = bus.servo(1).control_mode()
    assert mode['mode'] == 'torque'
    assert math.isclose(mode['torque'], 0.2 * XL330_M288.torque_per_amp_nm, rel_tol=1e-3)
    table.write(1, 'Goal_Current', -100)
    assert math.isclose(bus.servo(1).control_mode()['torque'], -0.1 * XL330_M288.torque_per_amp_nm, rel_tol=1e-3)
    with pytest.raises(NotSupported):
        table.write(1, 'Goal_Position', 2048)


def test_current_mode_respects_drive_mode(table, bus):
    table.write(1, 'Operating_Mode', 0)
    table.write(1, 'Drive_Mode', 1)
    table.write(1, 'Goal_Current', 200)
    table.write(1, 'Torque_Enable', 1)
    assert bus.servo(1).control_mode()['torque'] < 0


def test_pwm_mode_drives_duty(table, bus):
    table.write(1, 'Operating_Mode', 16)
    table.write(1, 'Torque_Enable', 1)
    table.write(1, 'Goal_PWM', 442)
    mode = bus.servo(1).control_mode()
    assert mode['mode'] == 'duty' and abs(mode['duty'] - 0.5) < 0.01


def test_goal_pwm_ceiling_outside_pwm_mode_is_unsupported(table):
    table.write(1, 'Goal_PWM', 885)
    with pytest.raises(NotSupported, match='ceiling'):
        table.write(1, 'Goal_PWM', 700)


def test_unsupported_modes_fail_loudly(table, bus):
    with pytest.raises(NotSupported):
        table.write(1, 'Operating_Mode', 1)
    with pytest.raises(NotSupported, match="current_position='uncapped'"):
        table.write(1, 'Operating_Mode', 5)
    t = StockTable(bus, [2], current_position='uncapped')
    t.write(2, 'Operating_Mode', 5)
    t.write(2, 'Torque_Enable', 1)
    t.write(2, 'Goal_Position', t.read(2, 'Present_Position') + 10)


def test_ids_baud_and_protections_are_not_written_through_the_table(table):
    for name, value in (('ID', 5), ('Baud_Rate', 1), ('Max_Voltage_Limit', 80), ('Temperature_Limit', 70),
                        ('Return_Delay_Time', 250), ('Current_Limit', 500), ('PWM_Limit', 600)):
        with pytest.raises(NotSupported):
            table.write(1, name, value)
    table.write(1, 'Return_Delay_Time', 0)                  # what LeRobot's configure_motors writes
    table.write(1, 'Current_Limit', XL330_M288.current_limit_ma)


def test_bus_watchdog_maps_to_hold_lease(table, bus):
    table.write(1, 'Torque_Enable', 1)
    table.write(1, 'Bus_Watchdog', 50)                      # 50 x 20 ms
    lease = bus.servo(1).lease_status()
    assert lease['ms'] == 1000 and lease['action'] == 'hold'
    table.write(1, 'Bus_Watchdog', 0)
    assert bus.servo(1).lease_status()['state'] == 'off'


def test_led_write(table):
    table.write(1, 'LED', 1)
    assert table.read(1, 'LED') == 1


# ---------------------------------------------------------------------------------------------- LeRobot backend
def test_rustypot_backend_surface(bus):
    be = RustypotBackend(bus, [1, 2, 3])
    assert be.scan() == {1: 1200, 2: 1200, 3: 1200}
    assert be.read_register_with_error(1, 'model_number') == (1200, 0)
    assert be.set_torque([1, 2], True) == {}
    rows = be.sync_read_register_with_error([1, 2], 'present_position')
    be.sync_write_register([1, 2], 'goal_position', [v + 100 for v, _ in rows])
    assert be.sync_read_register([1, 2], 'goal_position') == [v + 100 for v, _ in rows]
    with pytest.raises(NotSupported):
        be.set_baudrate(57600)
    with pytest.raises(NotSupported):
        be.change_id(1, 4)
    be.close()
    assert not be.is_open()
    assert bus.read(1, 0, 2) == struct.pack('<H', 1200)      # a bus the caller passed in stays open


def test_lerobot_serial_motors_bus_end_to_end(bus):
    lerobot = pytest.importorskip('lerobot.motors')
    pytest.importorskip('rustypot')
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode, SerialMotorsBus
    motors = {'shoulder': Motor(1, 'xl330-m288', MotorNormMode.RANGE_M100_100),
              'elbow': Motor(2, 'xl330-m288', MotorNormMode.DEGREES),
              'gripper': Motor(3, 'xl330-m288', MotorNormMode.RANGE_0_100)}
    mb = patch_bus(SerialMotorsBus('/dev/null-openxl330', motors), bus)
    mb.connect()                                             # handshake: Model_Number 1200 on every ID
    mb.configure_motors()
    with mb.torque_disabled():
        mb.set_operating_mode('extended_position', ['shoulder', 'elbow'])
        mb.set_operating_mode('position', ['gripper'])
    mb.disable_torque()
    mb.reset_calibration()
    homings = mb.set_half_turn_homings()
    assert set(homings) == set(motors)
    assert all(abs(v - 2047) <= 1 for v in mb.sync_read('Present_Position', normalize=False).values())
    cal = {name: MotorCalibration(id=m.id, drive_mode=0, homing_offset=homings[name], range_min=1000, range_max=3000)
           for name, m in motors.items()}
    mb.write_calibration(cal)
    assert mb.is_calibrated
    mb.enable_torque()
    mb.sync_write('Goal_Position', {'shoulder': 10.0, 'elbow': 20.0, 'gripper': 50.0})
    _settle(bus, 0.5)
    pos = mb.sync_read('Present_Position')
    assert abs(pos['shoulder'] - 10.0) < 1.0
    assert abs(pos['elbow'] - 20.0) < 1.0
    assert abs(pos['gripper'] - 50.0) < 1.0
    with pytest.raises(Refused):
        mb.set_operating_mode('position', ['shoulder'])      # EEPROM-area setting with torque on, as on stock
    mb.disable_torque(['shoulder'])
    with pytest.raises(NotSupported):
        mb.set_operating_mode('velocity', ['shoulder'])      # rustypot has the mode, openxl330 does not
    mb.disconnect()                                          # disables torque first
    assert all(not s.enabled for s in bus.group([1, 2, 3]).status().values())


def test_patch_bus_refuses_non_xl330_models():
    class FakeMotor:
        def __init__(self, id_, model):
            self.id, self.model = id_, model

    class FakeBus:
        motors = {'a': FakeMotor(1, 'xl430-w250')}
        port = 'x'

        def _open_bus(self):
            pass

    class OldLeRobotBus:
        motors = {'a': FakeMotor(1, 'xl330-m288')}

    with pytest.raises(NotSupported, match='XL330'):
        patch_bus(FakeBus())
    with pytest.raises(NotSupported, match='0.7'):
        patch_bus(OldLeRobotBus())


# ---------------------------------------------------------------------------------------------- Reachy Mini
@pytest.fixture
def reachy_bus(monkeypatch):
    monkeypatch.setattr(dt, 'bundled_releases', lambda: (images.DEFAULT_RELEASE, 'FW231 release 17'))
    b = sim_bus(range(11, 19), release_id=FAST17)
    yield b
    b.close()


def test_reachy_conversions_match_rustypot():
    assert _to_counts(0.0) == 2048
    assert _to_counts(math.pi / 2) == 3072
    assert math.isclose(_to_rad(1024), -math.pi / 2)


def test_reachy_controller_positions_torque_and_current(reachy_bus):
    c = ReachyMiniMotorController(reachy_bus, body_rotation=None)
    assert c.get_motor_name_id()['stewart_1'] == 11
    pos = c.read_all_positions()
    assert len(pos) == 9 and math.isnan(pos[0])
    assert not c.is_torque_enabled()
    c.enable_torque()
    c.set_stewart_platform_position([0.2] * 6)
    c.set_antennas_positions([-0.3, 0.3])
    _settle(reachy_bus, 0.5)
    pos = c.read_all_positions()
    assert all(abs(p - 0.2) < 0.01 for p in pos[1:7])
    assert abs(pos[7] + 0.3) < 0.01 and abs(pos[8] - 0.3) < 0.01
    assert c.async_read_pid_gains(11) == (400, 0, 0)
    c.disable_torque()
    c.set_stewart_platform_operating_mode(0)
    assert c.read_stewart_platform_operating_mode() == [0] * 6
    c.set_stewart_platform_goal_current([50] * 6)
    c.enable_stewart_platform(True)
    assert reachy_bus.servo(11).control_mode()['mode'] == 'torque'
    with pytest.raises(NotSupported):
        c.set_body_rotation(0.0)
    with pytest.raises(NotSupported):
        c.set_antennas_operating_mode(16)
    with pytest.raises(NotSupported):
        c.write_raw_packet(b'\xff')


def test_reachy_raw_bytes_are_translated_for_xl330(reachy_bus):
    c = ReachyMiniMotorController(reachy_bus, body_rotation=None)
    assert c.read_raw_bytes(11, 0, 2) == list(struct.pack('<H', 1200))
    c.write_raw_bytes(11, 84, struct.pack('<H', 900))
    assert c.async_read_pid_gains(11) == (900, 0, 0)
    with pytest.raises(NotSupported):
        c.read_raw_bytes(11, 80, 6)                          # multi-register spans are not translated


def test_reachy_supply_above_7_v_warns_and_runs(reachy_bus):
    from openxl330.compat.reachy_mini import SupplyWarning
    for s in reachy_bus.port.servos:
        s.supply_mv = 7600
    with pytest.warns(SupplyWarning, match='above 7.0 V'):
        c = ReachyMiniMotorController(reachy_bus, body_rotation=None)
    assert c.table.read(11, 'Hardware_Error_Status') == 0x01      # stock input-voltage bit, not latched
    c.enable_torque()
    assert c.is_torque_enabled()
    for s in reachy_bus.port.servos:
        s.supply_mv = 6700
    assert c.table.read(11, 'Hardware_Error_Status') == 0


def test_reachy_voltage_fault_refuses_and_protection_off_removes_the_trip(reachy_bus):
    for s in reachy_bus.port.servos:
        s.supply_mv = 8300
    with pytest.raises(NotSupported, match='voltage fault'):
        ReachyMiniMotorController(reachy_bus, body_rotation=None)
    reachy_bus.group(list(range(11, 19))).set_voltage_protection(False)   # off first, then clear the latch
    for i in range(11, 19):
        reachy_bus.servo(i).clear_faults()
    c = ReachyMiniMotorController(reachy_bus, body_rotation=None, voltage_protection=False)
    assert all(not s.vprot for s in reachy_bus.port.servos)
    st = c.check_supply()
    assert all(x.faults == [] and x.warnings == ['voltage_protection_off'] for x in st.values())
    for s in reachy_bus.port.servos:
        s.supply_mv = 7200
    from openxl330.compat.reachy_mini import SupplyWarning
    with pytest.warns(SupplyWarning):
        ReachyMiniMotorController(reachy_bus, body_rotation=None)    # default turns it back on
    assert all(s.vprot for s in reachy_bus.port.servos)


def test_only_bundled_releases_are_served(reachy_bus, monkeypatch):
    monkeypatch.setattr(dt, 'bundled_releases', lambda: (images.DEFAULT_RELEASE,))
    with pytest.raises(NotSupported, match='openxl330 update'):
        StockTable(reachy_bus, [11])


def test_reachy_body_rotation_uses_raw_stock_addresses():
    calls = []

    class FakeBus:
        def read(self, servo_id, address, size):
            calls.append(('r', servo_id, address, size))
            return struct.pack('<i', 3072) if size == 4 else b'\x01'

        def write(self, servo_id, address, data):
            calls.append(('w', servo_id, address, bytes(data)))

    body = _StockRaw(FakeBus(), 10)
    assert math.isclose(body.position(), math.pi / 2)
    body.goal(0.0)
    body.torque(True)
    assert body.torque_enabled()
    assert ('w', 10, 116, struct.pack('<i', 2048)) in calls and ('w', 10, 64, b'\x01') in calls
    with pytest.raises(NotSupported):
        body.operating_mode(16)


def test_reachy_control_loop(reachy_bus):
    loop = ReachyMiniPyControlLoop(reachy_bus, 0.01, body_rotation=None, stats_pub_period=0.05)
    try:
        loop.enable_torque()
        loop.set_all_goal_positions(FullBodyPosition(math.nan, [0.1] * 6, [0.0, 0.0]))
        _settle(reachy_bus, 0.5)
        last = loop.get_last_position()
        assert all(abs(p - 0.1) < 0.01 for p in last.stewart)
        assert loop.get_stewart_platform_operating_mode() == 3
        assert loop.get_stats() is not None
    finally:
        loop.close()


# ---------------------------------------------------------------------------------------------- review fixes
def test_mapped_kd_is_floored_on_a_compensated_image(bus):
    from openxl330.registers import KD_COMP
    t = StockTable(bus, [1])
    seen = []
    orig = t._servos[1].configure
    t._servos[1].configure = lambda kp=None, kd=None: (seen.append((kp, kd)), orig(kp, kd))[1]
    t.write(1, 'Torque_Enable', 1)                          # stock P400 / D0
    assert seen[-1][1] == KD_COMP and math.isclose(seen[-1][0], XL330_M288.kp_from_stock(400))
    t.write(1, 'Torque_Enable', 0)
    big = XL330_M288.d_to_stock(0.2)
    t.write(1, 'Position_D_Gain', big)
    t.write(1, 'Torque_Enable', 1)
    assert math.isclose(seen[-1][1], XL330_M288.kd_from_stock(big))   # above the floor: as mapped


def test_stock_eeprom_settings_seed_the_table_without_double_applying():
    # FW231 applies the EEPROM Homing_Offset and reverse itself (app.c tracker begin), so firmware counts already carry
    # them: with no host write the table reports the firmware counts unchanged, and the EEPROM values as registers.
    b = sim_bus([1])
    servo = b.port.servos[0]
    servo.eeprom[10] = 1
    servo.eeprom[20:24] = struct.pack('<i', -350)
    servo.eeprom[48:56] = struct.pack('<ii', 3000, 1000)
    t = StockTable(b, [1])
    assert t.read(1, 'Drive_Mode') == 1 and t.read(1, 'Homing_Offset') == -350
    assert (t.read(1, 'Min_Position_Limit'), t.read(1, 'Max_Position_Limit')) == (1000, 3000)
    fw = b.group([1]).read()[1].raw['position_counts']
    assert t.read(1, 'Present_Position') == fw
    t.write(1, 'Homing_Offset', -300)                        # replaces the EEPROM offset: +50 on the host
    assert t.read(1, 'Present_Position') == fw + 50
    t.write(1, 'Drive_Mode', 0)                              # replaces the EEPROM reverse: mirrored on the host
    assert t.read(1, 'Present_Position') == 4096 - (fw + 350) - 300
    t.write(1, 'Drive_Mode', 1)
    t.write(1, 'Homing_Offset', -350)
    assert t.read(1, 'Present_Position') == fw              # back to the EEPROM settings: exact again
    t.write(1, 'Torque_Enable', 1)                           # no host change: commands go out unchanged
    t.write(1, 'Goal_Position', fw + 100)
    assert struct.unpack('<i', b.read(1, 256, 16)[:4])[0] == fw + 100
    servo.eeprom[48:56] = bytes(8)                           # absent limit keys read 0/0: stock defaults
    servo.eeprom[11] = 1                                     # velocity mode in EEPROM: refused at enable
    t.write(1, 'Torque_Enable', 0)
    t2 = StockTable(b, [1])
    assert (t2.read(1, 'Min_Position_Limit'), t2.read(1, 'Max_Position_Limit')) == (0, 4095)
    with pytest.raises(NotSupported, match='EEPROM'):
        t2.write(1, 'Torque_Enable', 1)
    servo.eeprom[11] = 3
    servo.eeprom[10] = 1 | 4                                 # time-based profile bit: refused, not dropped
    t3 = StockTable(b, [1])
    with pytest.raises(NotSupported, match='Drive_Mode 5'):
        t3.write(1, 'Torque_Enable', 1)
    b.close()


@pytest.mark.parametrize('name, value', [('Goal_Current', 30000), ('Goal_Current', -40000), ('Goal_Current', 1751),
                                         ('Torque_Enable', 2), ('LED', 5), ('Goal_PWM', 886), ('Homing_Offset', 2 ** 31),
                                         ('Bus_Watchdog', 128)])
def test_out_of_range_values_are_a_stock_range_error(table, name, value):
    from openxl330 import Rejected
    with pytest.raises(Rejected) as e:
        table.write(1, name, value)
    assert e.value.error == 4


def test_reachy_refuses_baud_change_with_the_stock_xc330(reachy_bus):
    c = ReachyMiniMotorController(reachy_bus, body_rotation='stock')
    with pytest.raises(NotSupported, match='XC330'):
        c.bus.set_baud(3_000_000)


def test_patch_bus_checks_the_lerobot_version(monkeypatch):
    import importlib.metadata as md
    from openxl330.compat import lerobot as oxl
    monkeypatch.setattr(md, 'version', lambda name: '0.8.1')
    with pytest.raises(NotSupported, match='0.7'):
        oxl._check_lerobot_version()
    monkeypatch.setattr(md, 'version', lambda name: '0.7.3')
    oxl._check_lerobot_version()


@pytest.mark.parametrize('reverse, home', [(0, 0), (0, -350), (1, 0), (1, -350), (1, 600)])
def test_present_position_applies_the_eeprom_offset_and_reverse_exactly_once(reverse, home):
    b = sim_bus([1])
    s = b.port.servos[0]
    s.eeprom[10] = reverse
    s.eeprom[20:24] = struct.pack('<i', home)
    s.shaft = 1500
    stock = 4096 - 1500 - home if reverse else 1500 + home     # what a stock servo reports for this shaft
    t = StockTable(b, [1])
    assert t.read(1, 'Present_Position') == stock
    t.write(1, 'Torque_Enable', 1)
    t.write(1, 'Goal_Position', stock + 200)                 # +200 in the stock frame
    _settle(b)
    assert abs(t.read(1, 'Present_Position') - (stock + 200)) <= 3
    assert abs(s.shaft - (1500 - 200 if reverse else 1500 + 200)) <= 3
    b.close()
