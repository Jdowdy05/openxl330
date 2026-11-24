# Compatibility with stock Dynamixel software

openxl330 uses Dynamixel Protocol 2.0 framing, but its register map is its own, so software written for stock XL330 servos cannot drive it unchanged. `openxl330.compat` is an opt-in, host-side translation layer: it presents the stock XL330 control table (names, addresses and units) and answers it with openxl330's public API. Nothing in it changes the firmware, and nothing is written to a servo's nonvolatile memory.

It has three parts:

| Part | For | Install |
| --- | --- | --- |
| `openxl330.compat.StockTable` | any code that reads and writes stock register names | nothing extra |
| `openxl330.compat.lerobot` | Hugging Face LeRobot 0.7+ (`SerialMotorsBus`, the deprecated `DynamixelMotorsBus`) | `pip install "openxl330[lerobot]"` |
| `openxl330.compat.reachy_mini` | the Pollen Robotics Reachy Mini motor-controller API | nothing extra |

It serves only servos running a release this openxl330 bundles (exact image match); any other servo is refused with the `openxl330 update` command to run. This layer is beta. It has been tested against simulated servos and LeRobot's real bus class, not yet on hardware: see [needs hardware validation](#needs-hardware-validation).

## The stock table

```python
from openxl330 import Bus
from openxl330.compat import StockTable

with Bus.simulated(3) as bus:                  # Bus("/dev/ttyUSB0") on hardware
    table = StockTable(bus, [1, 2, 3])
    table.sync_write("Torque_Enable", {1: 1, 2: 1, 3: 1})
    start = table.sync_read("Present_Position")
    table.sync_write("Goal_Position", {i: p + 200 for i, p in start.items()})
    print(table.read(1, "Model_Number"), table.sync_read("Present_Current"))
```

Values out of a register's range, or beyond the stock limits for `Goal_Current` (±1750 mA), `Goal_PWM` (±885), `Torque_Enable` and `LED` (0 or 1), raise `openxl330.Rejected` with error 4, the stock data range error.

Values are stock raw units: position counts (4096 per turn), velocity 0.229 rpm, current mA, voltage 0.1 V, PWM 885 = 100 %. Names match without case or underscores (`Goal_Position`, `goal_position`). `read()`/`write()` address one servo; `sync_read()`/`sync_write()` turn into one openxl330 group read or group move.

The stock EEPROM settings that the host may change (`Operating_Mode`, `Drive_Mode`, `Homing_Offset`, the position limits, the PID gains, profile velocity and acceleration) are kept in the `StockTable` object, not in the servo. They start from the servo's own stock settings, which openxl330 keeps from the stock firmware and serves at addresses 0 to 63: a servo set to reverse mode or given a homing offset under stock firmware reports the same positions here. Changes made through the table are not written back. They are lost when the object is dropped, as openxl330 forgets its PD configuration at power-up. LeRobot writes its calibration on every connect, so it needs nothing else; other software must set them again after reconnecting. As on a stock servo, EEPROM-area settings are refused (`openxl330.Refused`) while torque is on.

Options:

- `gains="mapped"` (default) converts stock `Position_P_Gain`/`Position_D_Gain` into openxl330 N·m/rad and N·m·s/rad gains (see [approximations](#approximated)). On an image with the gear-train cancellation loaded (the bundled default), the mapped Kd is raised to openxl330's default 0.10 N·m·s/rad when it would be lower: the cancellation removes the back-EMF, viscous and most friction damping a stock P-only gain relies on, and a joint with too little Kd rings or oscillates. A larger stock D gain is used as mapped; `gains="native"` keeps openxl330's own tuned defaults (`configure()` defaults: Kp 3.5, Kd 0.10 with the gear-train cancellation) and only stores the stock values for read-back.
- `supply_v=` is the supply voltage the gain conversion assumes (default 5.0 V). Stock gains act on PWM duty, so the same stock gain is stiffer at a higher supply.
- `current_position="refuse"` (default) or `"uncapped"`: see [operating mode 5](#unsupported).

## LeRobot

LeRobot 0.7 does all servo I/O through one rustypot bus, opened in `SerialMotorsBus._open_bus()`. `patch_bus()` replaces that method on one bus instance, so LeRobot's names, normalisation, calibration and recommended settings run unchanged:

```python
from lerobot.robots.koch_follower import KochFollower, KochFollowerConfig
from openxl330.compat import lerobot as oxl

robot = KochFollower(KochFollowerConfig(port="/dev/ttyUSB0"))
oxl.patch_bus(robot.bus)        # before connect(); nothing else changes
robot.connect()
```

`patch_bus(motors_bus, bus=None, **options)` opens `openxl330.Bus(motors_bus.port)` at `connect()` and closes it at `disconnect()`; pass `bus=` to use an open openxl330 bus instead (tests: `Bus.simulated(n)`). The options are the `StockTable` options above. `make_bus(port, motors, calibration)` builds an already-patched `SerialMotorsBus`.

Every motor of the bus must be an XL330 model (`xl330-m288`; `xl330-m077` once openxl330 supports it). `patch_bus()` replaces a private LeRobot method, so it checks the LeRobot version and supports 0.7.x only (the `lerobot` extra pins `<0.8`); releases before 0.7 call `dynamixel_sdk` directly. The status byte the backend returns is always 0: latched openxl330 faults do not show as stock alert bits in every reply, so read `Hardware_Error_Status` to see them. LeRobot still needs rustypot installed for its motor definitions (the `lerobot` extra installs it).

What LeRobot does, and what happens:

| LeRobot call | openxl330 |
| --- | --- |
| `connect()` handshake (reads `Model_Number`) | 1200 for an XL330-M288 |
| `configure_motors()` (`Return_Delay_Time` 0) | accepted (openxl330 answers at once) |
| `set_operating_mode("position" / "extended_position" / "current" / "pwm")` | PD position, multi-turn PD position, torque command, open-loop duty |
| `set_operating_mode("current_based_position")` | `NotSupported` unless `current_position="uncapped"` |
| `set_operating_mode("velocity")` | `NotSupported` |
| `Drive_Mode`, `Homing_Offset`, position limits, `set_half_turn_homings()`, `write_calibration()`, `is_calibrated` | kept on the host, applied to every position read and goal |
| `enable_torque()` / `disable_torque()` | `configure()` then `enable()` / `disable()` |
| `sync_read("Present_Position")` | one Fast Sync Read of the motion block |
| `sync_write("Goal_Position", …)` | one group move (a step, or a timed quintic with a profile velocity) |
| `Position_P_Gain`, `Position_D_Gain` | mapped gains (live while enabled) |
| `Position_I_Gain` other than 0 | `NotSupported` (no integrator) |
| `ID`, `Baud_Rate` writes (`setup_motor`) | `NotSupported`: set IDs in Dynamixel Wizard before installing |

## Reachy Mini

`openxl330.compat.reachy_mini` provides `ReachyMiniMotorController` and `ReachyMiniPyControlLoop` with the method names and arguments of `reachy_mini_motor_controller`. Positions are radians around the stock centre count 2048, as rustypot converts them.

```python
from openxl330.compat.reachy_mini import ReachyMiniMotorController

c = ReachyMiniMotorController("/dev/ttyACM0")
c.enable_torque()
c.set_stewart_platform_position([0.0] * 6)
print(c.read_all_positions())
```

Reachy Mini has nine servos: the body rotation (ID 10) is an XC330, which openxl330 does not run; the Stewart platform (IDs 11 to 16) and the antennas (17, 18) are XL330-M288. The XC330 stays on stock firmware and is driven with raw stock reads and writes on the same bus (`body_rotation="stock"`, the default), or left out (`body_rotation=None`: `body_yaw` reads NaN and its commands raise `NotSupported`). This is an exception to openxl330's rule that every servo on a bus runs openxl330: both firmwares use Protocol 2.0 at 1 Mbaud with unique IDs, and the shim only sends unicast reads and writes to ID 10, which openxl330 servos ignore. Group commands go to the XL330 IDs only. An openxl330 baud change would not reach the XC330, so `Bus.set_baud()` on this bus raises `NotSupported` while the controller drives it (open with `body_rotation=None` to change baud).

`ReachyMiniPyControlLoop` runs a background thread that reads positions every period; commands run between reads.

### Supply voltage

Reachy Mini supplies its servos with about 6.8 to 7.6 V; the XL330 is rated for 6.0 V. openxl330 (release 17 and later) handles this with two protections, both on after every boot:

- an upper-voltage trip at the limit in register 848, 8.0 V by default (settable 5.0 to 8.0 V): above it the servo turns torque off and latches a `voltage` fault;
- a warning, `supply_high` in `Status.warnings`, while the supply is above 7.0 V (it clears at or below 6.8 V). It never latches and never turns torque off.

When the controller opens it reads every XL330's status. A latched `voltage` fault raises `NotSupported`: fix the supply, then clear the faults. A `supply_high` warning becomes a Python `SupplyWarning` and nothing is refused; through the stock table it also shows as `Hardware_Error_Status` bit 0 (input voltage), which clears with the warning. Regenerative braking briefly raises the supply above its idle value, so a 7.6 V supply leaves little margin under the 8.0 V trip.

`voltage_protection=False` turns off **both** the 8.0 V trip and the 7.0 V warning (register 1152, `Group.set_voltage_protection`). With it off, nothing in the firmware trips on a high supply voltage; only the 3.5 V lower limit stays. The controller writes the setting every time it opens, so the default (`True`) turns protection back on even if an earlier session turned it off; the firmware also turns it back on at every boot. On an image without the switch (before release 17) the trip is always on: `True` opens normally and `False` raises `NotSupported`. A fault latched before protection was turned off stays latched until cleared.

Pollen's controller also caps the average motor voltage at 6 V through `Goal_PWM`. openxl330 has no duty ceiling, so above 6 V the motors see the full supply at full duty. The shim adds no limit of its own.

## What maps cleanly

- Identity: `Model_Number` (1200), `ID`, `Baud_Rate` (as the bus runs), `Protocol_Type` 2.
- `Torque_Enable` (read from the servo's status), `LED` (write), `Hardware_Error_Status` (openxl330 faults as stock bits: voltage → bit 0, temperature → bit 2, sensor → bit 3, timing → bit 4, current → bit 5).
- `Present_Position` (raw counts with `Homing_Offset` and `Drive_Mode` applied), `Present_Velocity`, `Present_Current`, `Present_PWM`, `Present_Input_Voltage`, `Present_Temperature`, `Moving`.
- `Goal_Position` in modes 3 and 4 (mode 3 clamps to the position limits, like stock), `Goal_PWM` in PWM mode.
- `Max_Voltage_Limit` and `Temperature_Limit` read the live openxl330 protection limits; the `supply_high` warning sets `Hardware_Error_Status` bit 0 while it lasts.

## Approximated

These conversions use the constants in `openxl330/compat/model.py`, built from openxl330's model table (`openxl330.models`: K, default gains, model numbers):

- **Position gains.** Stock XL330 position control outputs PWM: duty = (P/128) × error in counts / 885, D likewise with P/16 per count/ms. openxl330 gains are torque per angle. The mapping is Kp = (P/128) / 885 × supply × K × 4096/(2π), with K = 0.0504 N·m per volt (openxl330 bench fit) and supply = `supply_v` (5 V by default); Kd uses the same scale with D/16 and a 1 ms rate unit. Stock P400 gives about 0.58 N·m/rad. The D-term rate unit is an assumption.
- **Goal_Current.** In current mode, mA become an openxl330 torque command through 0.25 N·m/A (openxl330 bench fit; the datasheet stall ratio 0.354 overstates what the motor delivers), with the gear-train compensation. openxl330 commands torque through its actuator model, not by closing a current loop.
- **Profile velocity and acceleration.** A nonzero `Profile_Velocity` turns a goal into an openxl330 quintic move whose peak speed matches it (T = 1.875 d / v), lengthened if `Profile_Acceleration` is set (T ≥ √(5.774 d / a)). Stock uses a trapezoid.
- **Drive_Mode and Homing_Offset.** openxl330 firmware applies the servo's stock EEPROM Homing_Offset and Drive_Mode reverse itself, as stock firmware does, so with no host write the positions are the firmware's own. A host write replaces the EEPROM value, as on stock, and only the difference is applied on the host: a different reverse mirrors positions as 4096 − counts (as the firmware does) and changes the sign of velocity, current, PWM and torque; a different offset shifts them. Drive_Mode bits other than reverse (from a stock EEPROM or written) raise `NotSupported` at `Torque_Enable`.
- **Bus_Watchdog** becomes openxl330's opt-in command lease with the `hold` action (value × 20 ms).
- **Current-mode torque** is set, not measured: see [torque estimates](torque.md).

## Unsupported

Each of these raises `openxl330.NotSupported` (never a silent no-op):

- Operating modes 1 (velocity) and 5 (current-based position). Mode 5 caps current by `Goal_Current`, which openxl330 cannot do; `current_position="uncapped"` runs it as plain position control, and `Goal_Current` is then not a cap. Use this only where an uncapped grip is safe.
- `Goal_PWM` other than 885 outside PWM mode (a stock duty ceiling), `Goal_Velocity` other than 0.
- `Position_I_Gain`, the velocity gains and the feedforward gains other than their stock defaults.
- Writing `ID`, `Baud_Rate`, `Return_Delay_Time` (other than 0), `Current_Limit`, `PWM_Limit`, `Velocity_Limit`, `Temperature_Limit`, `Max_Voltage_Limit` and the other fixed EEPROM settings. Change baud with `Bus.set_baud()` and protections through [register 848](api.md#protections).
- `Drive_Mode` bits other than reverse; `Moving_Status`, the trajectory registers, indirect addressing and raw multi-register reads; Reachy Mini `write_raw_packet`.
- rustypot `set_baudrate()` to another rate, `change_id()`, `change_baudrate()`, `reset_turn_count()`.

## Needs hardware validation

- The gain mapping (stiffness and damping of mapped stock gains against a stock servo at the same gains).
- The Goal_Current torque constant, and current-mode behaviour on a loaded joint.
- LeRobot calibration and teleoperation on a real arm (Koch or SO-ARM style), including the gripper in uncapped mode 5.
- Reachy Mini on its 6.8-7.6 V supply (warning, no trip, regenerative peaks under 8.0 V), the XC330 raw path on a shared bus, and that the stock `Homing_Offset` and `Drive_Mode` read from each servo's EEPROM view give the positions the robot was calibrated with.
- The Kd floor on compensated images (stiffness and settling with mapped stock P gains).
