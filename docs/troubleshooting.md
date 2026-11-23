# Troubleshooting

Start by securing the load and keeping the joint's path clear. Use `openxl330 status --port PORT --ids ID` for torque state and fault names and `openxl330 diag --port PORT --ids ID` for bus counters, reply timing and reset information. Replace `PORT` and `ID` with the values for your setup.

## Bus and connection errors

| Error | What happened and what to do |
| --- | --- |
| `Error` | Base class for openxl330 errors. Read the message before deciding whether to retry. The CLI prints these errors to stderr and exits with status 1. |
| `AdapterError` | The serial adapter could not be opened. Check the connection, use `openxl330 ports` to find its current name and check your OS permissions. See [adapters](adapters.md). |
| `PortBusy` | The port may be held by Dynamixel Wizard, a monitor or another script. Close the owner of the port, then reconnect. On operating systems that use the same error for denied access, also check permissions. |
| `AdapterEchoes` | The adapter returned its own transmit bytes. Use a half-duplex TTL Dynamixel adapter with turn-around control, such as the U2D2 TTL interface or a compatible BusLinker-type adapter. |
| `BusUncertain` | An unexpected, partial, duplicate, out-of-order or corrupt reply, an interrupted exchange or a failed adapter operation left the wire state unknown. The wire stops transmitting. Check power, wiring, adapter, baud and unique IDs. On a quiet application bus, call `bus.reset()` or reconnect after addressing the cause; with the CLI, run the command again. While a servo may be in the retained loader, every request other than the loader session's is refused (`Refused`). During installation, follow the printed recovery guidance. |
| `NoReply` | One or more expected replies were missing. Check power, wiring, ID and baud. Python exposes `.missing` and `.partial` on group errors. A missing Fast Sync Read row also prevents the following rows from arriving, so check the first missing ID first. A complete absence of expected data can leave the wire usable, but a missing acknowledgement leaves the result of a write unconfirmed. |
| `NoServosFound` | A discovery scan received no replies. Check servo power, common ground, the selected adapter and baud. If every connected servo is running application firmware, try `openxl330 scan --port PORT --all-bauds` to check the common rates. |

### FTDI latency warning

`AdapterWarning` reports a configuration that can slow exchanges. On Linux, when the FTDI driver exposes `latency_timer`, the warning includes the exact command for your adapter to select 1 ms. It uses `sudo tee` to write the timer value and can need reapplying after unplugging the adapter. On macOS the FTDI driver's latency can dominate exchanges at around 15 ms, and this package cannot adjust it. On Windows the FTDI latency timer defaults to 16 ms: set it to 1 ms in Device Manager → Port Settings → Advanced (no warning is shown on Windows). The CLI prints this warning as one `warning: …` line. A compatible USB CDC adapter is another option when round-trip latency matters. See [adapters](adapters.md).

## Command and state errors

| Error | What happened and what to do |
| --- | --- |
| `InvalidArgument` | The library rejected an argument before sending the requested operation. Check types, units, ranges, register sizes and servo IDs. |
| `Refused` | The operation's prerequisites are unmet. Typical examples include attempting to configure an armed servo, enabling before `configure()` after a power-up, or attempting a new install while a loader session is pending. Follow the prerequisite in the message. A group ARM failure can carry `.enabled_ids`, `.already_enabled_ids` and `.unconfirmed_ids` so you can check which joints may be holding. |
| `Rejected` | A servo sent a complete protocol error reply. Inspect `.servo_id`, `.error` and `.meaning`. On a verified group move, error 7 can also mean the stored target failed verification. |
| `NotEnabled` | An expected joint is OFF. Check the fault list and whether the optional lease expired. For a healthy joint, configure the controller if needed and enable it before requesting movement. |
| `Faulted` | A latched fault prevents enabling. Check the fault bits and address the cause using the table below. With torque OFF, use `servo.clear_faults()` or `openxl330 clear --port PORT --ids ID`. Verify that the fault stays clear before resuming motion. |
| `NotSupported` | Recognized firmware lacks the requested feature. For example, last-reset data requires fast11 or later. Check `servo.info.features` and consider updating if the feature is needed. |
| `UnsupportedServo` | The model or firmware is incompatible, or the reported ABI is outside the supported range. Verify the model is XL330-M288 and check the firmware identity with discovery. `openxl330 status` shows an unknown ABI's identity, marks its telemetry unverified in the table and gives it as `null` in JSON. `openxl330 off` needs no identity check, so it still reaches such a servo. |
| `CalibrationError` | The host calibration file is unreadable or invalid, or the CLI could not save a change. Check the path and permissions. If the file is invalid, move it aside to keep a copy and regenerate the offsets. `scan`, `off`, `install`, `update` and `recover` work without a valid file; a per-ID reset needs one. With a valid file, `openxl330 calibrate --port PORT --id ID --reset` resets the selected joint. The host file is separate from the servo's factory calibration. |

### Rejected protocol codes

| Code | Meaning | What to check |
| ---: | --- | --- |
| 1 | Result fail | Check servo status and faults before retrying. |
| 2 | Instruction error | Check that the instruction is available in the installed firmware. |
| 3 | CRC error | The servo detected corruption in the request. Check the wiring, adapter and baud. |
| 4 | Data range error | Check the parameter range and the address for this firmware release. An unavailable register can also return this code. |
| 5 | Data length error | Match the payload or requested read length to the register size. |
| 6 | Data limit error | Check which configured or representable limit the request exceeds. |
| 7 | Access error | Some writes require torque OFF or a completed operation. An expired hold-action lease can reject moves and torque commands until an explicit re-arm. A position target is refused while the servo is in torque mode: call `release_torque()` first. Check torque, faults, lease state and `control_mode()` before retrying. |

### Fault names

The Python fault list names known bits and reports any other bit as `bit N`. The table includes legacy and diagnostic fault names the library can decode.

| Bit mask | Fault | What to check |
| ---: | --- | --- |
| 1 | `sensor` | Check the encoder, sensing, bridge and hardware fault readings. Review the first-trip record and sample budget if the fault repeats. |
| 2 | `timeout` | Check last-reset information for a controller or main-loop stall. Check lease status separately if a requested command lease has expired. |
| 4 | `current` | Check for a stall, obstruction or sustained load that exceeds the configured RMS current limit (1.0 A by default; see [protections](api.md#protections)). Clearing keeps the I²t state, so wait about 2 s or reduce the load before arming again. |
| 8 | `voltage` | The supply left 3.5–7.0 V (default upper limit). Check the supply under load, connectors, ground and regenerative rise during braking. |
| 16 | `temperature` | The servo reached 65 °C (default limit). Let it cool and reduce sustained mechanical load. |
| 32 | `timing` | Check the deadline counters and worst tick time. Record whether the fault coincided with bus traffic. |
| 64 | `config` | Check controller configuration and target values against the release and register map. |
| 128 | `travel` | Travel fault on firmware that implements this check. Check mechanical travel and joint coordinates. |
| 256 | `speed` | Speed fault on firmware that implements this check. Check motion rate, load and joint coordinates. |
| 512 | `daq` | Data-acquisition fault on firmware that reports it. Check the sample queue, readout rate and drops. |
| 1024 | `energy` | The controller reached a terminal condition. Check firmware release, controller status and the first-trip record before re-arming. |

## Installation and recovery

**`InstallError`** identifies the failing phase in `.step`. An attempted run can provide a receipt path in `.receipt` and a next command in `.recovery`. Use the printed command and any accompanying note. The run's receipt is usually under `~/.openxl330/records/` or `$OPENXL330_HOME/records/` when set.

**`UntestedServo`** is the installer refusal for stock firmware outside the proven versions. First install has been proven on XL330-M288 with stock V50, V52 and V53. The `--allow-untested` option applies to the stock version and still requires a supported model. An unknown or changed loader CRC across runs is reported as an install warning. A loader CRC that changes within an update fails verification.

| Phase | What to check |
| --- | --- |
| `precheck` | Check the host baud, package/release and whether an earlier attempt may have left a servo in the retained loader. Resolve that session before attempting a new install. |
| `census` | Verify power, wiring, the intended servo ID and baud, and isolate duplicate or unsupported devices. |
| `identify` | Verify the model, stock version and NV ID. An already-installed servo uses `update`. An older ID override may require explicit `--allow-id-change` to return to its NV ID. |
| `census_other_bauds` | Check for an unexpected responder at another baud. Devices at 57600 can interfere with the loader and a duplicate ID can collide after launch. Set unique stock IDs in Dynamixel Wizard before installation. |
| `backup` | Ensure local records can be written and that the servo answers calibration and loader-digest reads before an update proceeds. |
| `enter_loader` | Follow the recovery advice. The installer distinguishes an unacknowledged loader entry, a fresh prompt and an interrupted prompt. |
| `attach` | Nothing answered the retained loader. If the servo was power-cycled since the failed install, use `--loader-fresh`; otherwise run `openxl330 scan --port PORT --all-bauds` to find it (a servo that answers runs the firmware: use `update`). After a failed `--loader-fresh` attempt, run `recover` without it if the servo was not power-cycled, or isolate and power-cycle the servo before running the `--loader-fresh` command again. The message names the exact commands. Keep only one servo in the loader. |
| `upload` | Keep the original receipt, check power and communication and follow the recovery command if the servo remains in the loader. |
| `launch` | Inspect the receipt and printout for whether launch was acknowledged. Follow their recovery command rather than assuming the servo's state. |
| `verify` | Compare the identity and program verification result in the receipt. If the device is missing at the expected ID, use the suggested scan command. If the release identifier or preservation check differs unexpectedly, stop using the servo and share the receipt with the maintainer. |
| `census_after` | Check the wiring and IDs of bystanders after launch. A changed bus census can fail the overall run even though the target has left the loader. |
| `nv_journal` | A comparison of the stock NV-journal view differed unexpectedly after first install. Preserve the receipt and consult the maintainer before relying on the servo. |
| `records` | Check the local records directory and permissions. If the error follows launch, check the receipt and actual servo state before attempting another install. |
| `receipt` | Check that the path points to the matching failed run's receipt or directory and that the run left this ID in the loader. |
| `loader_digest` | Check that the bus is clean and that the servo can supply valid loader-digest evidence. Follow the phase-specific instructions if the error occurs during install or update. |
| `plan` | Experimental flash-all: the scan found something it will not handle (an ID at several bauds, an error reply, another model, an unproven stock version). Every problem is listed; fix them and run it again. Nothing was sent beyond pings. |
| `bystanders` | Experimental flash-all: a stock servo that stayed on the bus answered differently after another servo's install. The installed servo passed its checks; check the named one in Dynamixel Wizard and share the receipt. |
| `handoff` | Experimental uninstall: the servo may be in the retained loader. Run Dynamixel Wizard's Firmware Recovery, or put openxl330 back with the printed `recover --loader-fresh` command. |
| `restore` | Experimental uninstall-restore: the recovered servo reported torque on, or read back settings that differ from the record. Check it in Dynamixel Wizard. |

If the prompt was interrupted, isolate the target and follow the message about checking its boot state before using `--loader-fresh`. After an acknowledged launch, the usual next action is `update`, with exceptions for missing IDs and unexpected verification failures described above. Keep the raw wire log and calibration backups private. See [installation and recovery](install.md).

## My joint fell when I disabled it

`disable()` removes motor torque. Gravity or an external load can move the joint as soon as it goes OFF. Support the mechanism before disabling it, before opting into `off_on_exit`, and before testing an OFF-action lease. Closing the bus leaves the torque state as it was. If communication fails during OFF, treat the joint's torque state as unconfirmed until you can establish it safely.
