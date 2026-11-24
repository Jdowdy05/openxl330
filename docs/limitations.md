# Known limitations

These are known gaps in openxl330 0.2. **None of the fast17 firmware has run on hardware yet**: its new features are verified in simulation and native tests only. Most have a planned fix. Reports and pull requests are welcome; see [CONTRIBUTING](../CONTRIBUTING.md).

## IDs change over the bus only on release 17 and later

Installing keeps the ID the servo had on its stock firmware, so set a unique ID in ROBOTIS Dynamixel Wizard *before* installing, while the servo still runs stock firmware. Dynamixel Wizard cannot change the ID of a servo running openxl330.

- **Release 17 and later:** `openxl330 set-id` changes the ID over the bus; see [installing](install.md#change-an-id-after-installing).
- **Releases before 17, and the logging images** (no room for the ID log), cannot. `openxl330 update --allow-id-change` only moves a servo back to its stock ID after an earlier tool gave it another one.

## Sensor reacquire is not yet checked on hardware

The firmware rejects implausible encoder samples and coasts on its estimator while samples are bad. It latches the `sensor` fault (torque OFF) when more than 5 % of samples in 1 s are bad.

Earlier releases also faulted after a long dropout at high speed, because the position had moved more than the tracker accepted in one update. fast17 reacquires instead: the jump gate and the tracker share a limit, a candidate position needs two agreeing samples, and the estimators restart at the confirmed position. This is verified in the firmware's native tests only, not yet on a servo. If a `sensor` fault still latches, clear it with `clear` once the joint is safe, and please report it.

## A stuck encoder bus needs a power cycle

The magnetic encoder is read over I2C. If a transfer is aborted while the sensor is driving the bus, the sensor can hold the bus low. Every later read then fails and the `sensor` fault latches. Clearing it does not help, because the reads keep failing: only a power cycle recovers.

A bus-clear sequence (clocking SCL until the sensor releases the bus) is designed, but it did not fit in the fast17 flash budget, and it changes pin handling on the failure path that has not been verified on this board.

A related deliberate rule: a sensor transfer that completes after its time budget counts as a bad sample. That happens when the control tick itself was delayed. It counts toward the 5 % budget; one such sample on its own never faults.

## No graphical tool yet

openxl330 is a Python library and a command-line tool. A graphical setup tool is planned for v1.1. Until then, use the CLI (`openxl330 --help`) and the [examples](../examples/).

## Hardware coverage

- **Supply voltage:** ROBOTIS rates the XL330 to about 6 V. Release 17 lets the upper limit go to 8.0 V and has no duty cap, so above 6 V the motor sees the full supply at full duty.
- **Model:** the XL330-M288 is supported; the XL330-M077 is a beta with assumed constants. See [servo models](models.md).
- **Qualified so far:** macOS with BusLinker and U2D2 adapters. Linux and Windows are expected to work but are not yet hardware-tested.
- **Not yet done:** a physical check of the 65 °C shutoff. See the README for the current status.
- **Experimental:** `flash-all` and the route back to stock firmware. See [experimental commands](experimental.md) and [returning to stock firmware](returning-to-stock.md).
- **Torque estimates:** these are model-based estimates, not measurements. See the [torque guide](torque.md) for their accuracy.
