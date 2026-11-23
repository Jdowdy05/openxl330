# Adapters and serial settings

OpenXL330 needs a **half-duplex Dynamixel TTL adapter**. Use the U2D2’s three-pin TTL connection or a compatible BusLinker-type adapter. Check that the adapter and its driver support the rate you intend to use. Supply servo power separately as described in [wiring](wiring.md). [U2D2 supported baud rates](https://emanual.robotis.com/docs/en/parts/interface/u2d2/#baudrate)

| System | Example port name |
| --- | --- |
| Linux | `/dev/ttyUSB0` or `/dev/ttyACM0` |
| macOS | `/dev/cu.usbserial-*` or `/dev/cu.usbmodem*` |
| Windows | `COM3` |

Hardware status: Linux and Windows are expected to work, not yet hardware-tested; reports welcome.

Run `openxl330 ports` and use the actual name it prints. Close Dynamixel Wizard and any other controller using the adapter first. On Linux, check your account’s permission to open the device. On Windows, check the adapter driver and COM assignment if no port appears.

`Bus` drains stale input and checks whether the adapter echoes a request. A full-duplex serial adapter or an adapter that returns its own transmit bytes may raise `AdapterEchoes`. Swap to a compatible adapter and reopen. Unexpected input can also latch `BusUncertain`. Check wiring and ID uniqueness before resetting or reopening the connection. [Troubleshooting](troubleshooting.md)

## Baud

The installed firmware boots at 1 Mbaud. For motion and telemetry, 1, 2 and 3 Mbaud are available as runtime rates. Set the host to the bus’s current rate with `--baud` or `Bus(port, baud=...)`. To change the fleet’s rate, use `openxl330 set-baud --to` or `Bus.set_baud()`. The package checks the destination for duplicate IDs and reports which selected IDs confirmed the change. A unit without confirmed communication may still be at either rate. Scan before resuming control.

First installation uses the actual **stock** baud for discovery. The retained loader uses 57,600 baud. [Firmware installation](install.md)

## FTDI latency

The USB-to-serial driver can add more delay than the wire transmission itself. With the macOS FTDI driver, a single exchange can take about 15 ms. A CDC-style adapter can give lower latency. Measure end-to-end performance on your adapter and check it under the same bus load and host scheduling as your application.

On Linux, the library warns if it can read the driver’s FTDI latency timer and its value is above 1 ms. The warning prints the command for the exact device. For example, with `ttyUSB0` and the timer file present, an administrator can lower it until the device is unplugged.

```sh
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

On Windows, the FTDI driver's latency timer defaults to 16 ms. Set it to 1 ms in Device Manager: open the adapter's COM port under Ports (COM & LPT), then Properties → Port Settings → Advanced → Latency Timer (msec). The setting is kept for that adapter. The library does not read or warn about it on Windows.

The opening warning can detect certain FTDI devices on macOS. It does not change driver settings. When using a command lease, allow margin for the longest exchange and host scheduling delay you observe.
