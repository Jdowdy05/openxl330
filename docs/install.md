# Install the firmware

Installing openxl330 replaces the stock firmware of an **XL330-M288**. This installer only installs bundled openxl330 images. Use the [wiring guide](wiring.md) to check the model, adapter and power supply before continuing. Disconnect the load or support it before torque is released.

## First installation from stock firmware

1. Connect one servo. Use Dynamixel Wizard to give it a unique ID while it still runs the factory firmware. Record its ID, firmware version and baud rate. Close Wizard to release the adapter. [Dynamixel Wizard documentation](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)

   The ID you set in Dynamixel Wizard before installing is permanent in v1; changing it after flashing is not supported (except `openxl330 update --allow-id-change`, which moves a servo back to its stock NV ID). Factory servos all start at ID 1, so give each one its own ID before you install it.
2. Keep the other servos disconnected. Use stable power and leave the adapter and servo connected during the install. The installer supports model 1200 and has been used with stock V50/V52/V53 and loader v6. Reserve `--allow-untested` for an intentional bench test of another stock firmware version on this model.
3. Discover the adapter with `openxl330 ports`, then install using the recorded stock ID and baud. This example uses ID 4 at 57,600 baud.

   ```sh
   openxl330 install --port /dev/ttyUSB0 --stock-id 4 --stock-baud 57600
   ```

   The installer asks `This replaces the stock firmware; there is no supported way back. Continue? [y/N]`. Answer `y` to go on. Pass `--yes` to skip the question; without a terminal (in a script) the install is refused unless `--yes` is given. Before it starts it prints `Installing firmware on ID 4: about 30 s; do not unplug or interrupt.`, then the upload progress. Progress and messages go to stderr, so `--json` output stays pure JSON.

4. After success, scan at 1 Mbaud and check the release and torque state.

   ```sh
   openxl330 scan --port /dev/ttyUSB0
   openxl330 status --port /dev/ttyUSB0 --ids 4
   ```

`--stock-id` is required: there is no default ID. With `--stock-id 1`, the factory ID, the installer warns that you should set a unique ID in Dynamixel Wizard first if the servo will share a bus (`--yes` also skips this warning). `--stock-baud` defaults to 57,600, the factory setting. The new application starts at 1 Mbaud, with torque OFF, and keeps the stock nonvolatile ID. Assign distinct IDs before installing the rest of the fleet. The installer refuses an ID already in use. To select the image, check `openxl330 version` and use `--release` with a listed name.

## Several servos at once, and going back to stock (experimental)

`openxl330 experimental flash-all` installs on every stock servo of a bus in one run, each keeping its ID. `openxl330 experimental uninstall` and `uninstall-restore` put a servo back on stock firmware through Dynamixel Wizard's Firmware Recovery. Both are experimental. flash-all has installed a six-servo arm from stock; the route back to stock has been tested on a two-servo bench only: see [experimental commands](experimental.md) and [returning to stock firmware](returning-to-stock.md).

## Update an installed servo

Connect at 1 Mbaud, check the ID and release with `status`, and ensure the mechanism is safe to release. The status command shows an update hint for supported servos running an older release, such as fast13. Update one servo at a time.

```sh
openxl330 update --port /dev/ttyUSB0 --id 4
```

The update uses the bundled image. If an older tool set a device-block ID override, updating would return the servo to its stock nonvolatile ID. The installer requires `--allow-id-change` for that case and checks for collisions. Any previous device-block label or start-up baud override is removed by the bundled image.

## Recover an interrupted installation

Follow the command printed by the installer and inspect its receipt. Close the original session and isolate the target servo before attaching to the retained loader. The loader responds at 57,600 baud and recovery expects a single loader on the bus.

- If the servo remains in the loader with its prompt already completed, use the printed `openxl330 recover --port PORT --id ID` command. Add `--receipt PATH` to point at the failed run if needed.
- Use `--loader-fresh` when the loader is waiting for its initial prompt. If the error says the prompt was interrupted, follow its guidance about isolating and restarting the target and confirm it entered the loader before using the fresh option.
- If `recover` itself gets no answer from the loader, its message says what to run instead. If the servo was power-cycled since the failed install, use `openxl330 recover --port PORT --id ID --loader-fresh`. After a failed `--loader-fresh` attempt: if the servo was not power-cycled since the failed install, run `recover` without `--loader-fresh`; if it was power-cycled again or its prompt failed, isolate it, power-cycle it, and run the `--loader-fresh` command again. Otherwise find the servo with `openxl330 scan --port PORT --all-bauds`: a servo that answers there runs the firmware, so use `update`.
- If the application launch was acknowledged, the installer may ask you to scan at 1 Mbaud or use `update`. An unexpected loader, calibration or firmware verification result can instead require review of the receipt.

A successful recovery can report calibration preservation as unverified if usable records are missing. Unreadable loader-history entries are reported and ignored when checking previous evidence. Keep the receipt.

If the bundled image is missing or damaged, reinstall the Python package before installing firmware. Discovery, status, and torque OFF are still available for connected servos.

## What is verified and kept locally

The installer checks that torque is OFF before entering the loader for a first install or update. It writes only the application region and reads the installed program back for a byte-for-byte check. It checks the firmware release ID and leaves the loader and calibration regions intact. A first install compares the factory settings visible before and after installation and records the saved calibration after launch. An update compares the loader CRC and calibration across the operation. The stock application does not expose a full calibration-region read for the first-install baseline.

Current releases return the loader’s CRC directly. A legacy release can require hashing a temporary read in memory. Loader bytes are discarded and excluded from logs and records. An unrecognized loader CRC is reported as a warning.

Receipts, wire logs and per-servo calibration backups are under `~/.openxl330/records/` by default. A `--sim` install, update or recovery writes its records to a temporary folder instead and prints where it is. Set `OPENXL330_HOME` to choose another location. Treat wire logs and calibration backups as device data and review a receipt before sharing it. Host joint calibration is a separate file used for coordinates. [Troubleshooting](troubleshooting.md)
