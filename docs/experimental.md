# Experimental: flash-all and uninstall

> **Experimental.** `openxl330 experimental flash-all` has installed a six-servo arm from stock: V52 at 2 Mbaud, on a U2D2, with every bystander check unchanged. The steps behind uninstall were tested on one bench with two XL330-M288 servos, using prototype scripts. The uninstall commands themselves have so far been tested only against the simulated bus (macOS, November 2026). Read the limits below before using them, and please share receipts and reports.

Two commands are covered here:

- `openxl330 experimental flash-all` installs the bundled firmware on every stock XL330-M288 on the bus in one run. Each servo keeps its ID.
- `openxl330 experimental uninstall` and `uninstall-restore` put a servo back on stock firmware through ROBOTIS Dynamixel Wizard's Firmware Recovery.

openxl330 never ships, downloads or reads stock firmware. Dynamixel Wizard installs it.

## Flash every stock servo at once

Give every servo its own ID in Dynamixel Wizard first, as for a [single install](install.md). Factory servos all start at ID 1, and two servos at one ID collide. Then connect them all and run:

```sh
openxl330 experimental flash-all --port /dev/ttyUSB0
```

1. **Scan.** The bus is scanned at 57600, 1M, 2M, 3M and 115200 baud. Only pings are sent.
2. **Refusals.** The run stops before anything is installed if:
   - two servos share an ID (the scan collides);
   - an ID answers at several bauds;
   - a device answers a ping with an error;
   - a servo is not an XL330-M288;
   - a stock version is not proven (V50, V52 and V53 are), unless you pass `--allow-untested`;
   - a stock servo at 57600 would stay on the bus through another servo's loader session, and its version is not V53, the only version tested that way. Update it to the latest stock firmware in Dynamixel Wizard (*Firmware Update*), or pass `--allow-untested`.

   Every problem is listed at once.
3. **Plan.** The plan is shown and you confirm once. `--yes` skips the question, and outside a terminal `--yes` is required. Servos already on openxl330 firmware are skipped.
4. **Installs.** The stock servos are installed one at a time in ID order. Each install runs the same steps and checks as `openxl330 install`, with one difference: other stock XL330-M288 servos may stay on the bus, also at 57600 baud. Each of them has its control table bytes 0..119 read before the install and again after it. Its stock settings (0..63) and torque enable (64) must be unchanged. Other RAM differences, such as a hardware error status bit, are reported as warnings.
5. **Stops.** The first failed install stops the run. Its message names the servo, the IDs installed before it, the IDs not started, and the install's own recovery command. A stock servo whose control table changed also stops the run.
6. **Final check.** A final scan checks that every installed servo answers at 1 Mbaud on openxl330 firmware.

A stock servo at 9600 or 4 Mbaud is not seen by the scan.

## Go back to stock firmware

`openxl330 experimental uninstall` and `uninstall-restore` hand a servo to Dynamixel Wizard's Firmware Recovery and then restore its recorded stock settings, without a power cycle. See [returning to stock firmware](returning-to-stock.md) for the steps.

## Evidence and limits

What the bench showed:

- **A stock servo at 57600 baud ignores another servo's loader session.** 57600 is the loader's own baud. During two full sessions, one of them an interrupted upload and its recovery, a stock V53 servo transmitted nothing. Its control table bytes 0..119 were unchanged afterwards. This is why flash-all lets stock servos stay on the bus.
- **The loader entry only reaches its target.** It is addressed to one ID, so the other servos stay in their applications.
- **Firmware Recovery works without a power cycle.** Wizard picked up a servo waiting in its loader three times, each time with the other servo on the bus. An openxl330 servo on the bus at 1 Mbaud only counted framing errors, and a stock servo at 57600 was unchanged.
- **Recovery leaves the calibration alone.** After it, the factory calibration page was byte-identical to the first-install backup. Stock position readings matched openxl330's.

Not covered yet:

- stock servos other than V53 at 57600 staying on the bus as bystanders;
- writing back stock settings a user had changed (on the bench they were factory defaults, so only the ID was written);
- Dynamixel Wizard on Windows or Linux.

## Python

```python
from openxl330 import Bus, experimental

with Bus("/dev/ttyUSB0") as bus:
    result = experimental.flash_all(bus.wire, confirm=lambda plan: True)
```

These calls are also available from Python:

- `experimental.flash_all(wire, confirm=..., release=..., allow_untested=..., records=..., on_progress=...)`;
- `experimental.uninstall_prepare(wire, servo_id=...)`;
- `experimental.uninstall_restore(wire, record=..., baud=...)`;
- `experimental.plan(censuses)`, which shows what flash-all would do with a census.

`confirm` gets the plan before anything is installed. Failures raise `InstallError` with `.step` and `.recovery`, as for the [installer](install.md). The phases are listed in [troubleshooting](troubleshooting.md).
