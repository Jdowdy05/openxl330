"""Grip an object with one servo: open, close until contact, squeeze with a bounded torque, report, release.

The position controller bounds the squeeze. After contact the target goes a small angle past the contact point, so
the holding torque is about kp x (target - position). See docs/gripper.md.
"""
from __future__ import annotations

import argparse
import math
import struct
import sys

from openxl330 import Bus, Error

PROTECTIONS = 848         # register: current limit mA (0 = off), temperature limit °C, upper voltage 0.1 V, RMS mA
RATED_STALL_NM = 0.52     # ROBOTIS's stall torque rating for the XL330-M288 at 5.0 V
POLL_S = 0.01             # contact check interval while closing, seconds
SAMPLE_S = 0.5            # time between rows while holding, seconds
SETTLE_S = 0.2            # extra wait after a move, seconds
SIM_OPEN_DEG, SIM_CLOSED_DEG = 0.0, 60.0   # jaw positions on the simulated bus; hardware needs --open and --closed

CHECKLIST = (
    "Keep fingers out of the jaws. The object drops when the gripper lets go or torque turns OFF.",
    "Clear the jaw travel between the open and closed positions.",
    "Keep a way to cut the servo power within reach, such as a switch or the supply plug.",
)


def deg(rad: float) -> float:
    return math.degrees(rad)


def protections(bus, servo_id):
    """Register 848 (see docs/api.md, Protections): current limit A RMS (0 = off), temperature limit °C, upper voltage
    limit V, and the filtered RMS current A that the overload protection compares with its limit."""
    limit_ma, temperature_c, vmax_dv, rms_ma = struct.unpack("<4I", bus.read(servo_id, PROTECTIONS, 16))
    return limit_ma / 1000, temperature_c, vmax_dv / 10, rms_ma / 1000


def check_status(servo) -> bool:
    """Read the status before sending any command. Stop on latched faults or if torque is already on."""
    status = servo.status()
    print(f"ID {servo.id} runs {servo.info.release}: supply {status.voltage:.2f} V, {status.temperature:.0f} °C, "
          f"position {deg(status.position):+.2f} deg, faults: {', '.join(status.faults) or 'none'}")
    if status.faults:
        print(f"error: latched faults ({', '.join(status.faults)}). Fix the cause (docs/troubleshooting.md), clear "
              f"them with `openxl330 clear` while torque is OFF, then run this again", file=sys.stderr)
        return False
    if status.enabled or not status.ready:
        print("error: the servo must be OFF and ready. Check `openxl330 status`, support the load and use "
              "`openxl330 off` if needed", file=sys.stderr)
        return False
    return True


def go(bus, servo, target, speed):
    """A quintic move to `target` (rad) at about `speed` (rad/s) on average. Waits until it has finished."""
    duration = max(0.5, abs(target - servo.position) / speed)
    servo.move(target, duration=duration)
    bus.wire.sleep(duration + SETTLE_S)


def close_until_contact(bus, servo, open_rad, closed_rad, speed, contact_lag):
    """Close at a steady `speed` (rad/s) with a linear profile. Contact is when the jaws trail the planned path by more
    than `contact_lag` (rad); the controller then pushes with about kp x contact_lag. Returns the contact position, or
    None when the jaws reach `closed_rad` without contact."""
    direction = 1.0 if closed_rad > open_rad else -1.0
    stroke = abs(closed_rad - open_rad)
    duration = stroke / speed
    servo.move(closed_rad, duration=duration, profile="linear")
    start = bus.wire.clock()
    while True:
        elapsed = bus.wire.clock() - start
        planned = open_rad + direction * min(stroke, speed * elapsed)
        position = servo.position
        if direction * (planned - position) > contact_lag:
            return position
        if elapsed > duration + SETTLE_S:
            return None
        bus.wire.sleep(POLL_S)


def hold(bus, servo, target, kp, direction, seconds):
    """Hold for `seconds` and print one row per sample, signed so that closing is positive. Returns the last sample, or
    None if torque turned OFF (a protection tripped)."""
    print("    time   error  kp x error  torque_motor  torque_out  current  RMS current  temperature")
    print("       s     deg         N·m           N·m         N·m        A            A           °C")
    start = bus.wire.clock()
    while True:
        elapsed = bus.wire.clock() - start
        state, status = servo.state(), servo.status()
        limit_a, _, _, rms_a = protections(bus, servo.id)
        if not status.enabled:
            print(f"Torque turned OFF during the hold (faults: {', '.join(status.faults) or 'none'}). "
                  f"The gripper let go.")
            return None
        error = direction * (target - state.position)
        sample = dict(error=error, squeeze=kp * error, torque_motor=direction * state.torque_motor,
                      torque_out=direction * state.torque_out, current=abs(state.current), rms=rms_a, limit=limit_a)
        print(f"{elapsed:8.2f}{deg(error):+8.2f}{sample['squeeze']:+12.4f}{sample['torque_motor']:+14.4f}"
              f"{sample['torque_out']:+12.4f}{sample['current']:9.3f}{rms_a:13.3f}{status.temperature:13.0f}")
        if elapsed >= seconds:
            return sample
        bus.wire.sleep(SAMPLE_S)


def grip(bus, servo, args, interactive) -> bool:
    open_rad, closed_rad = math.radians(args.open), math.radians(args.closed)
    direction = 1.0 if closed_rad > open_rad else -1.0
    speed = math.radians(args.speed)

    # The bound: a target `overshoot` past the contact point gives a holding torque of about kp x overshoot.
    overshoot = args.squeeze / args.kp
    # Report contact once the approach pushes with about half the squeeze. Friction and kd x speed also make the jaws
    # trail the path in free air, so a false contact calls for a slower close or a larger squeeze, not a larger kp.
    contact_lag = overshoot / 2
    print(f"Squeeze {args.squeeze:.3f} N·m = kp {args.kp} N·m/rad x {deg(overshoot):.2f} deg past contact. "
          f"Contact = the jaws trail the closing path by {deg(contact_lag):.2f} deg")
    if args.squeeze >= RATED_STALL_NM:
        print(f"note: ROBOTIS rates the stall torque at about {RATED_STALL_NM} N·m at 5.0 V. The motor saturates and "
              f"draws stall current; see docs/gripper.md")

    limit_a, temperature_c, vmax_v, _ = protections(bus, servo.id)
    overload = f"{limit_a:.1f} A RMS" if limit_a else "off"
    print(f"Protections: current overload {overload}, temperature {temperature_c} °C, supply 3.5–{vmax_v:.1f} V")

    gains = servo.configure(kp=args.kp, kd=args.kd)
    print(f"Configured kp {gains['kp']} N·m/rad, kd {gains['kd']} N·m·s/rad. Enabling: keep the jaws still.")
    servo.enable()

    print(f"Opening to {args.open:+.1f} deg")
    go(bus, servo, open_rad, speed)
    if interactive:
        input("Place the object between the jaws and keep your fingers clear. Press Enter to close. ")

    print(f"Closing toward {args.closed:+.1f} deg at {args.speed:.0f} deg/s")
    contact = close_until_contact(bus, servo, open_rad, closed_rad, speed, contact_lag)
    if contact is None:
        target = closed_rad
        print("No contact before the closed position, so there is nothing to squeeze. Holding at the closed position"
              + (" (the simulator has no object)." if not interactive else "."))
    else:
        target = contact + direction * overshoot
        print(f"Contact at {deg(contact):+.2f} deg. Hold target {deg(target):+.2f} deg. "
              f"If the object slips out, the jaws close to the hold target.")
    servo.move(target)   # a step: the reference jumps to the hold target

    sample = hold(bus, servo, target, args.kp, direction, args.hold)
    if sample is None:
        return False
    # At rest both torque estimates come from duty, supply voltage and fixed coefficients. Near zero speed, stiction
    # and backlash make them uncertain (docs/torque.md). kp x error is what the controller asks for.
    print(f"Holding torque: kp x error = {sample['squeeze']:.4f} N·m; estimates: torque_motor "
          f"{sample['torque_motor']:.4f} N·m, torque_out {sample['torque_out']:.4f} N·m")
    limit = f"the {sample['limit']:.1f} A overload limit" if sample["limit"] else "no overload limit (off)"
    print(f"Current {sample['current']:.3f} A; filtered RMS {sample['rms']:.3f} A against {limit}")
    if args.lever:
        force = sample["squeeze"] / (args.lever / 1000)
        print(f"Holding force about {sample['squeeze']:.4f} N·m / {args.lever / 1000:.3f} m = {force:.2f} N")

    print(f"Releasing to {args.open:+.1f} deg")
    go(bus, servo, open_rad, speed)
    return True


def turn_off(servo) -> bool:
    """Request torque OFF and read it back. True once OFF is confirmed."""
    try:
        servo.disable()
        if not servo.status().enabled:
            print("Torque OFF confirmed")
            return True
        print("error: the servo still reports torque on. Cut the servo power", file=sys.stderr)
    except Error as exc:
        print(f"error: torque OFF not confirmed ({exc}). Cut the servo power", file=sys.stderr)
    return False


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Find --open and --closed with torque OFF: move the jaws by hand and read the position with "
               "`openxl330 status` (radians; 1 rad = 57.3 deg).",
    )
    connection = parser.add_mutually_exclusive_group(required=True)
    connection.add_argument("--port", help="adapter port")
    connection.add_argument("--sim", type=int, metavar="N", help="use N simulated servos")
    parser.add_argument("--id", type=int, default=1, help="servo ID (default 1)")
    parser.add_argument("--open", type=float, metavar="DEG", help="open jaw position, deg (required with --port)")
    parser.add_argument("--closed", type=float, metavar="DEG", help="closed jaw position, deg (required with --port)")
    parser.add_argument("--squeeze", type=float, default=0.10, metavar="NM",
                        help="holding torque, N·m (default 0.10)")
    parser.add_argument("--kp", type=float, default=1.0, help="proportional gain, N·m/rad (default 1.0)")
    parser.add_argument("--kd", type=float, default=0.03, help="derivative gain, N·m·s/rad (default 0.03)")
    parser.add_argument("--speed", type=float, default=30.0, metavar="DEG_S",
                        help="closing speed, deg/s (default 30)")
    parser.add_argument("--hold", type=float, default=3.0, metavar="S", help="hold time, s (default 3)")
    parser.add_argument("--lever", type=float, metavar="MM",
                        help="finger lever arm, mm: distance from the horn axis to the contact point")
    args = parser.parse_args()
    if args.sim is None and (args.open is None or args.closed is None):
        parser.error("--open and --closed are required with --port")
    args.open = SIM_OPEN_DEG if args.open is None else args.open
    args.closed = SIM_CLOSED_DEG if args.closed is None else args.closed
    if args.open == args.closed:
        parser.error("--open and --closed must differ")
    if not (0 < args.kp <= 50 and args.squeeze > 0 and args.speed > 0 and args.hold >= 0):
        parser.error("use 0 < --kp <= 50, --squeeze > 0, --speed > 0 and --hold >= 0")
    if args.lever is not None and args.lever <= 0:
        parser.error("--lever must be positive")

    interactive = args.sim is None
    try:
        print("Before gripping:")
        for number, step in enumerate(CHECKLIST, 1):
            if interactive:
                input(f"  {number}. {step}\n     Press Enter when done, or Ctrl+C to stop. ")
            else:
                print(f"  {number}. {step}")
        bus = Bus.simulated(args.sim) if args.sim is not None else Bus(args.port)
        with bus:
            servo = bus.servo(args.id)
            if not check_status(servo):
                return 1
            try:
                gripped = grip(bus, servo, args, interactive)
            finally:
                off = turn_off(servo)   # after success, an error, a tripped protection or Ctrl+C
            if not (gripped and off):
                return 1
    except EOFError:
        print("error: input closed before a confirmation", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("error: interrupted", file=sys.stderr)
        return 130
    except Error as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
