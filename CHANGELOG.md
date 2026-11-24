# Changelog

## 0.2.0 (2026-11-24, beta)

**Not yet run on hardware.**
- The fast17 firmware this release bundles has not run on a servo yet. Its new features (the ID change over the bus, sensor reacquire, the voltage warning and switch) are verified in simulation and in the firmware's native tests only.
- The XL330-M077 is a beta, with constants estimated from its gear ratio, not measured.
- The hardware evidence listed for 0.1.0 applies to fast16.

- **Bundled firmware: fast17** (release 17, register map ABI 5), as four images:
  - `fw231-comp2-fast17` (XL330-M288, performance; the default), `fw231-comp2-fast17-log` (XL330-M288, logging);
  - `fw231-comp2-fast17-m077` and `fw231-comp2-fast17-m077-log` (XL330-M077, beta).
  - The installer picks by the servo's model and `--variant` (`performance` or `logging`; `images.release_for()`). `update` keeps the variant a servo runs, and `recover` reinstalls the failed run's image.
  - New in the firmware: the ID log (performance images), sensor reacquire after an encoder dropout, the 8.0 V upper limit with its warning and switch, and the XL330-M077 images. The logging images add the 2 kHz data-acquisition drain and extra counters, and have no ID log.
  - fast16 is no longer bundled. Servos on older releases show an update in `status`.

- **Change an ID over the bus** (release 17+, register 1144):
  - `Servo.set_id()` and `openxl330 set-id`. The servo must be OFF and the new ID free; nothing is sent until the checks pass.
  - The change is a recorded run with a receipt. The host calibration entry moves with the servo, and older handles for either ID refuse further use.
  - `update()` keeps an ID set this way when the new release can (re-applying it if the install erased the log). An older release needs `--allow-id-change`.
- **XL330-M077 beta:**
  - `openxl330.models` holds the per-model constants.
  - The XL330-M077 is accepted only with `--beta` / `allow_beta=True`, and only with an image built for it.
  - The bus refuses an image built for the other model. `ServoInfo.beta` marks a beta model.
  - The PD configuration uses each model's torque constant and default gains. XL330-M288 bytes are unchanged.
- **Upper-voltage protection (release 17):**
  - The upper limit can be set up to 8.0 V, and that is its default.
  - Above 7.0 V the servo raises a non-latching warning, `Status.warnings` `'supply_high'`; it clears at 6.8 V. The CLI `status` shows it.
  - `servo.voltage_protection = False` and `group.set_voltage_protection()` turn off both the warning and the upper trip; it is back on at every boot.
- **Compatibility with stock Dynamixel software (beta):** `openxl330.compat`, a host-side translation layer.
  - `StockTable` presents the stock XL330 control table. It respects each servo's EEPROM Homing_Offset and Drive_Mode, which the firmware already applies.
  - A stand-in for LeRobot 0.7's motor bus (`openxl330[lerobot]`).
  - The Reachy Mini motor-controller API, with the upper-voltage protection switch.
  - It serves only the bundled releases. Tested in simulation and against LeRobot's real bus class, not yet on hardware. See [compatibility](docs/compat.md).
- **Register map ABI 5** (release 17) is recognised. Registers that exist only in the logging images are marked in the [register reference](docs/registers.md). See [servo models](docs/models.md).

## 0.1.0 (2026-11-23, beta)

The first public release is a beta. Before release, a six-servo arm was installed from stock with `experimental flash-all`, the torque command was checked, and a 600 s loaded run passed. The README lists what has been qualified on hardware.

- Python API and CLI for XL330-M288 servos running FW231, with multi-servo motion, telemetry and diagnostics.
- First installation, update and recovery with one bundled firmware image, `fw231-comp2-fast16` (register map ABI 4). It is fast14 plus a direct torque command, and a diagnostic: `last_reset()` reports `iwdg_update_timeout` when the watchdog's settings fail to apply at start-up. As in fast14, the temperature lookup table is generated from a thermistor model and matches the values in fast13. First-time installations preserve the stock nonvolatile ID. Installed servos start at 1 Mbaud.
- A direct torque command in N·m: `set_torque()` and `release_torque()` on servos and groups, compensated or raw, with `control_mode()` to read it back.
- Grouped motion and gain writes with verification by default, opt-in command leases, torque estimates and host-side joint calibration.
- The CLI `install` needs `--stock-id`, asks for confirmation (`--yes` to skip; required without a terminal), and prints a notice and upload progress on stderr. `off` reaches servos on any firmware and has `--all`. `--sim` runs keep their records in a temporary folder.
- Simulator, examples and a maintainer self-test that can run entirely on an in-memory bus.
- Experimental: `openxl330 experimental flash-all` installs on every stock servo of a bus in one run, each keeping its ID; `openxl330 experimental uninstall` and `uninstall-restore` go back to stock firmware through ROBOTIS Dynamixel Wizard's Firmware Recovery. Tested on one two-servo bench only; see `docs/experimental.md`.
- Turning servos OFF goes on after a communication glitch: if the bus state becomes uncertain partway through `Group.disable`, `off_on_exit`, `Servo.disable` or `openxl330 off --ids`, every remaining servo still gets one OFF write (its reply drained without checking), and the error reports them as unconfirmed.
- Stock V53 joins the proven first-install versions. `recover` compares the factory calibration page with the servo's record, not the stock settings journal, which changes after a trip back to stock.
- User documentation, cross-platform CI configuration and local wheel and source distribution builds.
