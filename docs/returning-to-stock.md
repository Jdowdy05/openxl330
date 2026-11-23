# Returning to stock firmware

> **Experimental.** This route was tested on one bench with two XL330-M288 servos (macOS, November 2026), using prototype scripts. The `openxl330 experimental uninstall` commands themselves have so far been tested only against the simulated bus. Writing back stock settings that a user had changed has not been tested on hardware. See [experimental commands](experimental.md) for the evidence and its limits.

openxl330 never ships, downloads or reads stock firmware. ROBOTIS Dynamixel Wizard 2.0 installs it, and openxl330 records the servo's settings before and restores them after.

Dynamixel Wizard's *Firmware Update* cannot remove openxl330 firmware: it reports version 231, above any stock release, so Wizard says the firmware is already up to date. Wizard's *Firmware Recovery* works instead, in three steps.

1. **Record the servo and hand it to its loader.** The servo must run openxl330 at 1 Mbaud.

   ```sh
   openxl330 experimental uninstall --port /dev/ttyUSB0 --id 2
   ```

   This records:
   - the servo's ID, label and stock settings (the stock settings journal as the firmware reports it);
   - its first stock baud, from its first-install records when they exist;
   - a copy of its calibration and settings memory.

   The record goes into a run folder under `~/.openxl330/records/_runs/`, as `uninstall-record.json`. The servo is then turned OFF and sent to its retained ROBOTIS loader, which waits there.

   The command asks for confirmation; `--yes` skips it, and outside a terminal `--yes` is required. This step is refused if another servo already answers as ID 1 at 57600 baud. It is also refused if a stock servo other than V53 sits at 57600: it would stay there through the loader's banner and Wizard's recovery traffic, and only V53 was tested that way. Update that servo in Wizard, unplug it, or pass `--allow-untested`. After the recovery this servo starts as ID 1 at 57600 too, and the two would collide, so change that servo's ID or unplug it first.
2. **Run Wizard's recovery.** Close openxl330 and anything else using the port. In Dynamixel Wizard 2.0, open Tools > Firmware Recovery, choose XL330-M288, the latest firmware and your adapter.

   No power cycle is needed: Wizard finds the waiting loader. If Wizard asks for a power cycle anyway, unplug the other servos first. At power-up every servo's loader listens, so the recovery could reach all of them. The recovery takes about 16 s.

   It resets the stock settings to factory defaults (ID 1, 57600 baud, return delay 250, position mode) and leaves the factory calibration untouched.
3. **Restore the recorded settings.**

   ```sh
   openxl330 experimental uninstall-restore --port /dev/ttyUSB0 --record PATH/uninstall-record.json
   ```

   This finds the recovered servo at ID 1 / 57600 and writes back the recorded settings that differ. Those are return delay, drive mode, operating mode, secondary ID, homing offset, moving threshold, the temperature, voltage, PWM, current, velocity and position limits, startup configuration, PWM slope and shutdown. The protocol type is left alone. Then it writes the ID and the baud: `--stock-baud` if given, else the servo's first stock baud from the record, else 57600. Finally it reads the settings back and compares them.

   It asks for confirmation (`--yes` skips it). It scans the bus first and refuses if no stock XL330-M288 answers at ID 1 / 57600, or if the recorded ID answers anywhere on the bus. When the recorded ID still runs openxl330, the recovery was not done on that servo, and writing the record into another servo would give two servos one ID. `--stock-baud` accepts 57600, 115200, 1M, 2M or 3M, the bauds openxl330 can reach. A first stock baud outside these falls back to 57600; change it in Dynamixel Wizard afterwards if you need it.

Changed your mind after step 1? The loader is still waiting at its first prompt, so `openxl330 recover --port PORT --id ID --loader-fresh` puts openxl330 back.

## What changes and what is kept

- **Firmware:** the stock ROBOTIS firmware that Wizard installs, at the version Wizard offers.
- **Factory calibration:** kept. Wizard's recovery does not touch the servo's factory calibration page. On the bench it was byte-identical to the first-install backup, and stock position readings matched openxl330's.
- **Stock settings:** reset to factory defaults by the recovery, then written back by `uninstall-restore` from the record. The ID and baud are written last.
- **Records:** the record, a receipt and the servo's calibration backup stay under `~/.openxl330/records/` (or `$OPENXL330_HOME/records/`).

To install openxl330 again later, use `openxl330 install` as for any stock servo (see [install](install.md)).
