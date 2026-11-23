# Known limitations

These are known gaps in openxl330 0.1.0. Most have a planned fix. Reports and pull requests are welcome; see [CONTRIBUTING](../CONTRIBUTING.md).

## IDs cannot be changed over the bus

The servo keeps the ID it had on its stock firmware. Set a unique ID in ROBOTIS Dynamixel Wizard *before* installing, while the servo still runs stock firmware. Dynamixel Wizard cannot change the ID of a servo running openxl330. `openxl330 update --allow-id-change` only moves a servo back to its stock ID after an earlier tool gave it another one. Setting IDs over the bus is planned for v1.1.

## A long encoder dropout at high speed latches a sensor fault

The firmware rejects implausible encoder samples and coasts on its estimator while samples are bad. It latches the `sensor` fault (torque OFF) when more than 5 % of samples in 1 s are bad. One case still faults earlier:

- A dropout lasting about 20 ms at about 9 rad/s, near the motor's no-load speed (about 11 rad/s), ends in a fault. At higher speeds, a shorter dropout is enough.
- When the samples come back, the position has moved more than the tracker accepts in one update, so it latches `sensor`.
- Slower motion and short dropouts recover normally.

A fix (a shared limit for the jump gate and the tracker, with two-sample confirmation) is designed, but not yet in the firmware. Clear the fault with `clear` once the joint is safe.

## A stuck encoder bus needs a power cycle

The magnetic encoder is read over I2C. If a transfer is aborted while the sensor is driving the bus, the sensor can hold the bus low. Every later read then fails and the `sensor` fault latches. Clearing it does not help, because the reads keep failing: only a power cycle recovers.

A bus-clear sequence (clocking SCL until the sensor releases the bus) is documented, but not yet built, because it changes pin handling on the failure path and has not been verified on this board.

A related deliberate rule: a sensor transfer that completes after its time budget counts as a bad sample. That happens when the control tick itself was delayed. It counts toward the 5 % budget; one such sample on its own never faults.

## No graphical tool yet

openxl330 is a Python library and a command-line tool. A graphical setup tool is planned for v1.1. Until then, use the CLI (`openxl330 --help`) and the [examples](../examples/).

## Hardware coverage

- **Model:** only the XL330-M288 is supported.
- **Qualified so far:** macOS with BusLinker and U2D2 adapters. Linux and Windows are expected to work but are not yet hardware-tested.
- **Not yet done:** a physical check of the 65 °C shutoff. See the README for the current status.
- **Experimental:** `flash-all` and the route back to stock firmware. See [experimental commands](experimental.md) and [returning to stock firmware](returning-to-stock.md).
- **Torque estimates:** these are model-based estimates, not measurements. See the [torque guide](torque.md) for their accuracy.
