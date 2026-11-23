"""Generate registers.md from the public register map."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from openxl330 import registers as r


def render() -> str:
    # Numeric literals cover sizes and addresses documented in registers.py without named constants.
    rows = (
        (0, 64, "R", "Device identity view. Model at 0–1, firmware version at 6, effective ID at 7, and live baud index at 8."),
        (r.TORQUE_ENABLE, 1, "W", "Torque control. 1 requests ARM and 0 turns torque OFF."),
        (*r.STATUS, "R", "Status block. Includes torque state, readiness, fault bits, position, supply voltage, temperature, and worst tick time."),
        (r.LED, 1, "W", "Status LED. 1 is on and 0 is off."),
        (r.CLEAR_FAULTS, 1, "W", "Write `0x5A` to clear faults while torque is OFF and the sensor is healthy."),
        (*r.MARKER, "R", f"Firmware signature, beginning with `{r.FW231_MARKER.decode('ascii')}`."),
        (224, 32, "R", "First-trip diagnostic record."),
        (*r.TARGET, "R/W", f"Signed encoder counts, duration in {r.TICK_HZ:,} Hz ticks, profile (0 for quintic or 1 for linear), then a reserved zero word. One tick requests a step. The profile word reads zero."),
        (r.DUTY, 4, "W", f"Signed open-loop duty from −{r.DUTY_FULL} to +{r.DUTY_FULL}. The signed 32-bit minimum returns to PD control."),
        (r.TORQUE, 4, "W", "Torque command (release 16, ABI 4): signed output torque at the horn in N·m as Q24, passed through the gear-train compensation. The signed 32-bit minimum returns to PD control, holding where the shaft is. Kept while OFF and applied at the next ARM."),
        (r.TORQUE_RAW, 4, "W", "Raw torque command (release 16, ABI 4): signed motor torque referred to the output in N·m as Q24, with no compensation. The signed 32-bit minimum returns to PD control."),
        (r.PD_CONFIG, 48, "W", "PD and estimator configuration. Write while torque is OFF and read back at the PD configuration status block."),
        (*r.COMP, "R/W", "Primary gear-train compensation settings. The final word returns the current compensation contribution on read and carries the settings signature on write."),
        (r.GAINS, 8, "W", "Live Kp and Kd as signed Q16 values in N·m/rad and N·m·s/rad."),
        (336, 16, "R/W", "Additional gear-train compensation. Includes duty-dependent back-EMF and the near-stop inertia gate."),
        (340, 12, "R/W", "High-pass damping settings. The final word returns the filter state on read and carries the settings signature on write."),
        (640, 64, "R", "Controller snapshot. Includes reference, motion, duty and observer state. Word 4 is the requested output torque after the limit (Q24); word 5's high half is the applied duty; word 9 holds the controller flags (2 saturated, 8 limited); word 13 bit 1 is duty mode and bits 2–3 the torque mode (0 PD, 1 compensated, 2 raw)."),
        (*r.PD_CONFIG_READ, "R", "PD configuration status and readback, including the configured flag, token and gains."),
        (r.BAUD, 4, "R/W", "Live baud in bit/s. Bit 0 is set while a switch is unconfirmed. Accepts 1000000, 2000000 or 3000000 on write."),
        (r.PROTECT, "12 W / 16 R", "R/W", "Current, temperature and voltage protection settings. Write while torque is OFF. The extra read word gives filtered RMS current."),
        (852, 16, "R", "Deadline misses. Total, current window, worst window, and packed miss limit and window size."),
        (856, 24, "R", "Sample-budget counters for sensing and encoder quality."),
        (*r.BUSDIAG, "R", "Bus identity and diagnostics, including the ABI, UART errors, reply counters, and baud switches and reversions."),
        (*r.DEVBLOCK, "R", "Program device block. Older installs may report an ID, label or boot baud override here."),
        (r.TIMING[0], f"4 W / 16, {r.REPLY_TIMING[1]}, {r.TIMING[1]}, 48 or 52 R", "R/W", "Reply timing maxima in microseconds. Reads of 24 bytes or more also include predecessor completion timing and late-chain count. Four zero bytes on write reset timing counters."),
        (r.LEASE, "12 W / 16 R", "R/W", f"Optional command lease, from 0 (disabled) to {r.LEASE_MAX_MS:,} ms. Read back the configured action, state, expiry count and time since renewal."),
        (r.HEARTBEAT, 1, "W", "Renew an active command lease by an addressed write or Sync Write row."),
        (r.FSR_COUNTERS[0], f"4 W / 16, 24, 32 or {r.FSR_COUNTERS[1]} R", "R/W", "Fast Sync Read counters. A write changes the reply gap in microseconds until reboot."),
        (1084, 1, "W", "Fast-start path switch in firmware builds that include it. 1 enables it and 0 disables it until reboot."),
        (*r.MOTION, "R", "Position, estimated velocity, estimated torques, approximate signed current, and applied duty. See [torque estimates](torque.md)."),
        (*r.LAST_RESET, "R", "Last-reset reason, reset flags and hang counts. Available from fast11."),
        (*r.RELEASE_ID, "R", "Release number, variant, ABI, and source identifier. Available from fast12."),
        (*r.LOADER_DIGEST, "R", "Retained loader CRC-32 and covered byte count. Available from fast12."),
        (1536, "32 × n", "R", "Drain n data-acquisition records from the queue."),
    )
    heading = """# Firmware registers

This reference follows the application register map in [`openxl330.registers`](../src/openxl330/registers.py).
Addresses and sizes are in bytes. Multi-byte values use little-endian order. R means read, W means write, and R/W allows both.
Firmware variants and older releases can expose fewer diagnostics. Where operations share an address, use the size for the operation you want.

Use the [Python API](api.md) for joint coordinates and normal torque control. An ARM write at address 64 uses the stored target. The API samples position and writes a hold target before arming.

<!-- Generated by docs/generate_registers.py. -->

| Address | Size (bytes) | Access | Meaning |
| ---: | :--- | :---: | --- |
"""
    return heading + "".join(f"| {address} | {size} | {access} | {meaning} |\n" for address, size, access, meaning in rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="check the existing reference without changing it")
    args = parser.parse_args()
    target = Path(__file__).with_name("registers.md")
    content = render()
    if args.check:
        if not target.is_file() or target.read_text(encoding="utf-8") != content:
            print("docs/registers.md is out of date. Run python docs/generate_registers.py.", file=sys.stderr)
            return 1
    else:
        target.write_text(content, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
