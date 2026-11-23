# Python API

## Bus and discovery

| Call | Result |
| --- | --- |
| `Bus(port, baud=1000000, calibration=None, log=False)` | Opens a named half-duplex adapter or uses a serial-like object. |
| `Bus.simulated(n=3, calibration=None)` | In-memory bus with IDs 1 through `n`. |
| `Bus.ports()` | Available serial device names. |
| `bus.scan(all_bauds=False)` | A list of `ServoInfo` records. `all_bauds=True` also probes the other common rates and restores the host rate. |
| `bus.servo_info(id)` | Identity, baud and supported features, including identity for unsupported firmware without decoding its telemetry. |
| `bus.servo(id)` / `bus.group(ids, off_on_exit=False)` | A supported servo or an ordered group of supported servos. |
| `bus.set_baud(bps, ids=None)` | A fleet rate change report with `baud`, `previous`, `confirmed` and `missing`. All supported source servos hear the broadcast. `ids` selects the confirmations to report. |

A `Bus` context manager closes the connection on exit. Torque state remains as it was. Use `Group` with `off_on_exit=True` when you want it to attempt torque OFF. If the block also raised, any failure to turn torque off is available as `exc.off_on_exit_error`. Support the load and verify the result after a communication failure.

If closing a bus also fails while handling an error, the original error is raised with the close failure in `exc.close_error`.

Serialize requests on each `Bus`. Independent adapters can use separate `Bus` instances.

Existing `Servo` and `Group` handles recheck advertised device identity after a reset, rescan, or baud change. Obtain fresh handles if an identity has changed. Probes cannot distinguish a replacement that reports the same ID and identical metadata.

A clean missing reply raises `NoReply`. For group reads, `.partial` and `.missing` describe what came back. A Fast Sync Read with a missing segment also drops later segments. Unexpected or corrupt traffic raises `BusUncertain` and blocks further transmissions. Inspect the wiring and IDs before reopening or calling `bus.reset()` on the quiet line. When an installer may have left a servo in the loader, use the printed recovery instructions. See [troubleshooting](troubleshooting.md).

## Servo control and telemetry

| Call | Use |
| --- | --- |
| `servo.configure(kp=None, kd=None)` | Load PD gains while torque is OFF, once after each boot. Omitted gains are chosen according to whether compensation is loaded. Returns `kp`, `kd` and `comp`. |
| `servo.enable()` | Arm the configured servo with a hold target sampled during the operation. Keep the shaft still until arming finishes. |
| `servo.move(position, duration=0, profile="quintic", units="rad", verify=True)` | Position target in radians by default. Use `units="deg"` for degrees. Time is in seconds. A zero duration is a step. Profiles are `"quintic"` and `"linear"`. |
| `servo.set_gains(kp, kd)` | Update live proportional and derivative gains on a configured servo. |
| `servo.set_duty(duty)` / `servo.release_duty()` | Set open-loop duty from −1 to 1 while enabled. Release duty before requesting position tracking. |
| `servo.set_torque(nm, compensate=True, verify=True)` / `servo.release_torque()` | Command output torque in N·m directly, instead of a position, while enabled (release 16+). `release_torque()` returns to position control, holding where the joint is. See [torque command](#torque-command). |
| `servo.control_mode()` | What drives the motor now (`"position"`, `"duty"`, `"torque"` or `"raw_torque"`), the requested torque in N·m after the configured limit, the applied duty, and the controller's `saturated` (more than full duty asked for) and `limited` (the configured limit clipped the request) flags. |
| `servo.disable()` | Turn torque OFF. A loaded joint may fall. |
| `servo.state()` / `servo.position` | Motion state or position alone. |
| `servo.status()` | Enabled and ready flags, named and raw fault bits, voltage, temperature, position and worst tick time. |
| `servo.diagnostics()` | Bus counters, release, reply timing and last reset when available. Use `reply_timing()` or `last_reset()` for one item. |
| `servo.led = True` / `servo.clear_faults()` / `servo.reboot()` | Set the LED, clear faults while torque is OFF, or restart the application with torque OFF. |

`servo.reboot()` verifies that the application returns. On a faster bus it checks that the servo’s ID is free at 1 Mbaud before rebooting, since that is where the servo initially comes up.

Proportional gain is in N·m/rad and derivative gain in N·m·s/rad. Supported gain inputs are `0 <= kp <= 50` and `0 <= kd <= 2`. Supplying a value in range does not establish stability on a particular load.

The bundled image loads gear-train compensation at every boot, and `configure()` then reports `comp` as true. On top of the PD torque, it adds:

- **gear friction:** a constant term plus one proportional to the transmitted torque, `c0 + c1·|τ|` (about 4 mN·m and 0.31), faded in below 0.05 rad/s;
- **viscous loss and back-EMF:** about 0.021 N·m·s/rad, reduced at high duty;
- **rotor inertia:** feedforward from the planned acceleration, plus feedback that switches off below 0.3 rad/s.

Registers 324, 336 and 340 hold these settings ([register map](registers.md)).

`State` reports calibrated position (rad), velocity (rad/s), estimated `torque_motor` and `torque_out` (N·m), current (A) and duty (fraction). `State.raw` retains fields in firmware units and uses the hardware coordinate direction. [Torque estimation guidance](torque.md)

## Protections

The firmware turns torque OFF and latches a fault for three protections. The only other faults are integrity checks: `sensor` when more than 5 % of encoder or ADC samples in 1 s are bad, and `timing` when more than 5 % of control ticks in 1 s miss their deadline ([fault names](troubleshooting.md#fault-names)). The limits below are the defaults after every boot; you can change the upper voltage, temperature and current limits until the next boot (see below), and a current limit of 0 turns the overload protection off. There are no travel or speed trips, and no communication timeout unless you enable the opt-in [command lease](#command-lease).

| Condition | Default | Fault name |
| --- | --- | --- |
| Supply voltage below 3.5 V or above the upper limit | upper limit 7.0 V | `voltage` |
| Temperature at or above the limit | 65 °C | `temperature` |
| Current overload: RMS current above the limit (I²t, first-order filter with a time constant of about 2 s, so short bursts do not trip) | 1.0 A | `current` |

The defaults apply after every boot. Register 848 reads back the current limit (mA, 0 = off), the temperature limit (°C), the upper voltage limit (0.1 V units) and the live filtered RMS current (mA) as four little-endian 32-bit words.

```python
import struct
from openxl330 import Bus

with Bus.simulated(1) as bus:
    current_ma, temperature_c, vmax_dv, rms_ma = struct.unpack("<4I", bus.read(1, 848, 16))
    print(f"{current_ma / 1000} A RMS, {temperature_c} °C, {vmax_dv / 10} V, now {rms_ma / 1000} A RMS")
```

A 12-byte write to 848 (the first three words) changes the limits until the next boot. It is accepted only while torque is OFF. The lower voltage limit of 3.5 V is fixed. A tripped protection shows in `status()` faults; see [troubleshooting](troubleshooting.md#fault-names).

After a current trip, wait about 2 s, or reduce the load, before arming again. Clearing the fault does not reset the I²t filter, which decays with its own time constant of about 2 s. A servo armed straight away under the same load trips again almost at once.

## Groups

- `group.read(fast=True)` returns a dictionary keyed by ID using one Fast Sync Read. Use `fast=False` for standard Sync Read. `group.status()` is one Sync Read of the status blocks. `group.diagnostics()` gathers the diagnostic records by ID.
- `group.configure(kp=None, kd=None)` requires every member to be OFF. `group.enable()` checks readiness and configuration before arming members in order. A runtime error can leave only part of the group enabled. Inspect `enabled_ids`, `already_enabled_ids` and `unconfirmed_ids` on errors from the arming loop.
- `group.move({id: position, ...}, duration=..., profile=..., units=...)` sends all named targets in one Sync Write. By default it checks enabled state and reads back stored position and duration. Position readback alone does not confirm tracking. The readback has no profile identifier or sequence for distinguishing identical writes.
- `group.set_torque({id: nm, ...} or nm, compensate=True, verify=True)` commands torque on the named members, or on every member for a single number, in one Sync Write. With `verify=True`, every named member must be enabled, and each member's mode is read back. `verify=False` sends the Sync Write alone, for torque loops. `group.release_torque()` returns them all to position control, and `group.control_modes()` reads every member's mode in one Sync Read.
- `group.set_gains({id: (kp, kd), ...}, verify=True)` writes gains together and checks readback. For high-rate applications, explicit `verify=False` skips readback for moves and gain commands. Unverified moves can also set `settle_s` to adjust the default 2 ms wait in addition to the estimated transmission time. Qualify shorter waits on the adapter and fleet before using them for control.
- `group.disable()` attempts every reachable member after clean missing replies and refusals. If the wire state becomes uncertain, every member not yet confirmed still gets one OFF write (its reply is drained, not checked) and `BusUncertain` is raised with `emergency_off_ids`; that OFF is unconfirmed, so support the joints and check them on a new connection. Failures carry `confirmed_off_ids`, `unconfirmed_ids` and `off_failures`. `group.identify(id)` lights only the selected member.

## Torque command

From release 16 (register map ABI 4), `set_torque(nm)` commands torque directly instead of a position. `nm` is output torque at the horn in N·m, signed in the joint's calibrated direction.

- **`compensate=True` (default):** the torque goes through the [gear-train compensation](#servo-control-and-telemetry) (friction, back-EMF, rotor inertia), like the PD's own output.
- **`compensate=False`:** commands motor torque referred to the output, with no compensation: duty = torque / (K × supply voltage).

It takes effect at the next 2 kHz control tick, so within 0.5 ms.

- **What leaves torque mode:** `release_torque()`, `set_duty()`, `disable()` and any fault. After `release_torque()`, the PD holds where the joint is.
- **No added limits:** only the configured torque limit (`configure()`, at most 2 N·m) and full duty bound the command. The voltage, temperature and current protections and an opt-in [command lease](#command-lease) apply as always.
- **While torque mode is active:**
  - `move()` is refused with error 7;
  - an accepted torque command renews a running lease;
  - an expired `"hold"` lease leaves torque mode and refuses further torque commands until `enable()`.
- **Only while enabled:** by default (`verify=True`) this library refuses `set_torque()` while the servo is OFF, as it does for `set_duty()`. The firmware keeps a torque written to its register while OFF (by `verify=False`, raw register access or another program) and would apply it at the next ARM. `enable()` clears any such command, and any open-loop duty, before arming.
- **Torque loops:** `verify=False` skips the enabled check and sends the write alone, one bus transaction per update, like `move(verify=False)`. On a group it also skips the mode readback. Use `control_mode()` or `control_modes()` now and then to confirm.

The commanded torque is a request passed through the actuator model, not a measurement at the horn. The model's friction, back-EMF and inertia constants were fitted on a bench servo, so expect differences on yours (see [torque estimates](torque.md)).

The default limit of 2 N·m is far above what the motor can deliver at full duty: a few tenths of a N·m at 5 V. In practice, duty saturation bounds the torque, so a request beyond the stall torque delivers about the stall torque. `control_mode()` reports `saturated` when a request needs more than full duty, and `limited` when the configured limit clipped it. No other cut-off is added; set a lower limit with `configure()` if you want one.

```python
servo.set_torque(0.02)      # 20 mN·m at the horn, in the joint's + direction
servo.set_torque(-0.02)     # reverse
servo.release_torque()      # back to position control, holding here
```

## Command lease

The command lease is off after boot. Enable it with `servo.lease(timeout, action="off")` or `group.lease(timeout, action="off")`. Timeout is in seconds, up to 60. Send `heartbeat()` well within the timeout. Reads and plain broadcast writes do not renew the lease. Use `lease_status()` to inspect state and expirations.

On expiry, `action="off"` releases torque. With `action="hold"` the servo holds and refuses new motion commands until `enable()` re-arms it. Set `timeout=0` to disable the lease.

## Joint calibration

With default calibration, zero radians is encoder count 2048 and one turn is 4096 counts. Positions are continuous across turns within the servo’s target range. Calibrate with torque OFF and the mechanism supported.

```python
from openxl330 import Bus

with Bus.simulated(3) as bus:
    servo = bus.servo(1)
    joint = servo.calibrate(zero="here", direction=-1)
    print(joint.zero_counts, joint.direction)
```

`servo.calibrate` accepts `zero="here"` or a count. A direction of `-1` reverses the sign of position changes and the motion quantities reported by the host. Omit fields you want to keep. For a group, use `group.calibrate(zeros="here", directions={1: -1})` or pass a dictionary of individual zero values.

A physical `Bus` reads `~/.openxl330/calibration.json` by default, or uses the directory from `OPENXL330_HOME`. Pass a path or a `Calibration` instance explicitly for separate host profiles. The simulated bus uses in-memory calibration by default. Joint calibration never writes the servo’s own calibration region.

## Installer, raw protocol and diagnostics

Use `install.from_stock(bus.wire, stock_id=..., stock_baud=...)`, `install.update(bus.wire, servo_id=...)` and `install.recover(bus.wire, servo_id=...)` as the Python counterparts to the CLI. A Python caller opening a connection to a retained loader should use 57,600 baud and `check_adapter=False` to avoid the application probe. While a servo may be in the loader, the `Wire` refuses every request other than the loader session's, and `Wire.flash_read` never reads the loader region 0x08000000..0x08003FFF. Follow the instructions in the install error or receipt for choosing the recovery options. [Firmware installation](install.md)

`Bus` also exposes `ping`, `read`, `write`, `sync_read`, `fast_sync_read`, `sync_write`, `bulk_read` and `reboot`. Use the openxl330 [register map](registers.md) for raw access. A Protocol 2.0 client such as the optional DYNAMIXEL SDK can use the same packet framing with the openxl330 register map. For that client, run `python -m pip install dynamixel-sdk` separately. Use one client on a port at a time. [DYNAMIXEL SDK Python source](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/master/python)

`openxl330.experimental` holds the experimental `flash_all`, `uninstall_prepare` and `uninstall_restore`. See [experimental commands](experimental.md) for what they do and the evidence behind them.

The maintainer self-test exercises identity, torque-OFF soak, motion, group telemetry and the optional same-release installer round trip. Choose a fresh output directory for each run so earlier JSON reports are preserved. Try it with in-memory servos first.

```sh
python -m openxl330.selftest --sim 3 --ids 1,2,3 --roundtrip-id 2 --out selftest-report
```

With a real port, the self-test is an active bench procedure and moves each selected joint. Run it only with clear travel, stable power and the ability to stop the bench. Supplying `--roundtrip-id` also requests an installation.
