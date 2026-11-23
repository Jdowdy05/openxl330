# Changelog

## 0.1.0 (unreleased, beta)

The first public release is a beta: see the README for what has been qualified on hardware so far.

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
