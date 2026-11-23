# Quickstart

## Try the simulator

Once the package is installed, these commands discover and inspect six simulated servos. Each command starts a fresh bus.

```sh
openxl330 scan --sim 6
openxl330 status --sim 6 --ids 1-6
openxl330 diag --sim 6 --ids 1
```

From a checkout or extracted source distribution, run the [examples](../examples/) with `--sim 6`.

```sh
python examples/scan.py --sim 6
python examples/arm_six.py --sim 6
python examples/read_torque.py --sim 6
python examples/calibrate.py --sim 6
python examples/install_one.py --sim 6 --stock-id 4
python examples/first_run.py --sim 6
python examples/gripper.py --sim 6
```

`Bus.simulated(n)` is available for Python programs. It starts with IDs 1 through `n` running the bundled release. By default it keeps joint calibration in memory. The install example supplies its own factory-state simulator to exercise the installer.

## Connect installed servos

Follow [wiring](wiring.md) and [adapter setup](adapters.md). Give each servo its own ID using Dynamixel Wizard while it still runs stock firmware, then follow the [installation guide](install.md) for each device. All installed servos start at 1 Mbaud.

```sh
openxl330 ports
openxl330 scan --port /dev/ttyUSB0
openxl330 status --port /dev/ttyUSB0 --ids 1-6
```

Substitute your adapter port and the IDs you actually see. The CLI accepts an inclusive range such as `1-6` or a comma-separated list such as `1,3,5`. Append `--json` to get structured output. If a search at the current rate finds nothing, try `scan --all-bauds`.

`status` prints the servos that answered and a `missing: [...]` line on stderr for the IDs that did not, then exits with status 1.

Use `openxl330 identify --port /dev/ttyUSB0 --ids 1-6 --id 3` to light the selected servo’s LED and switch off the others.

To turn torque off, use `openxl330 off --port /dev/ttyUSB0 --ids 1-6`, or `--all` to broadcast OFF to every ID at the current baud. OFF sends no identity check first, so it works whatever firmware a servo runs. Each servo is reported as `OFF` or `not answering`. Run `openxl330 diag --port /dev/ttyUSB0 --ids 1-6` for bus counters and reset and timing information where available.

## Make a first move

`examples/first_run.py` is a guided first move on one joint: it asks you to support the joint, clear its travel and keep a power cut within reach, checks status and faults, configures gains below the defaults, moves about 5° and back while printing telemetry, then turns torque OFF and confirms it.

```sh
python examples/first_run.py --port /dev/ttyUSB0 --id 1
```

For a gripper on one servo, see `examples/gripper.py` and the [gripper notes](gripper.md). Or use the [README example](../README.md#quickstart) with a supported load and a clear work area. Configure the PD gains with torque OFF after each boot. The enable call aims to hold the position sampled during arming. Keep the shaft still through this step. Send a small timed move and confirm the returned position before increasing travel or gains.

The example requests OFF on exit. Torque OFF releases the joint, so support it first. If the bus is uncertain, the library cannot confirm OFF over that connection. Keep a way to cut power within reach.

## Change baud or zero

The current bus rate is a session setting. For six installed servos at the default rate, a switch to 2 Mbaud looks like this.

```sh
openxl330 set-baud --port /dev/ttyUSB0 --to 2000000
openxl330 status --port /dev/ttyUSB0 --baud 2000000 --ids 1-6
```

The rate change checks for overlapping IDs at the destination. After a successful change it reminds you to pass `--baud` with the new rate on later commands. Review any missing confirmations before issuing more commands. To go back to 1 Mbaud, pass `--baud 2000000 --to 1000000` to `set-baud`, or power-cycle the servos. Reconfirm the live baud and torque state after a power cycle.

With the mechanism supported and torque OFF, bring a joint to its reference angle and use `openxl330 calibrate --port /dev/ttyUSB0 --id 1 --zero-here`. Add `--direction -1` if positive motion should be reversed. To restore the default center count and direction in the host calibration file, use `openxl330 calibrate --id 1 --reset`. Reset works without connecting the servo. Calibration changes how the host interprets that ID. The servo’s nonvolatile data stays as it was. See [calibration and control](api.md).
