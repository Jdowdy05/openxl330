"""A safe first move on one servo: low gains, about 5 degrees out and back, then torque OFF."""
from __future__ import annotations

import argparse
import math
import sys

from openxl330 import Bus, Error

# Low PD gains, well below the ones configure() chooses when you give it no gains. With kp = 1.0 N·m/rad the joint
# pushes back with about 0.017 N·m per degree of error, so it is soft. Raise the gains only after this run works.
KP = 1.0           # proportional gain, N·m/rad
KD = 0.03          # derivative gain, N·m·s/rad
MOVE_DEG = 5.0     # size of the move, degrees
MOVE_S = 1.5       # duration of each half of the move, seconds: a slow quintic profile
SETTLE_S = 0.3     # time to keep watching after each half, seconds
SAMPLE_S = 0.1     # time between telemetry rows, seconds

# Steps 1 to 3 are physical. The script cannot check them, so on hardware it waits for you to confirm each one.
CHECKLIST = (
    "Support the joint. It must not fall or swing when torque turns OFF.",
    f"Clear the travel. The horn turns about {MOVE_DEG:.0f} degrees and back. Keep hands and cables out of its path.",
    "Keep a way to cut the servo power within reach, such as a switch or the supply plug.",
)


def confirm_checklist(interactive: bool) -> None:
    print("Before the first move:")
    for number, step in enumerate(CHECKLIST, 1):
        if interactive:
            input(f"  {number}. {step}\n     Press Enter when done, or Ctrl+C to stop. ")
        else:
            print(f"  {number}. {step}")
    if not interactive:
        print("  (simulated bus: not waiting for confirmation)")


def check_status(servo) -> bool:
    """Step 4: read the status before sending any command. Stop on latched faults or if torque is already on."""
    status = servo.status()
    print(f"4. ID {servo.id} runs {servo.info.release}: supply {status.voltage:.2f} V, {status.temperature:.0f} °C, "
          f"position {math.degrees(status.position):+.2f} deg, faults: {', '.join(status.faults) or 'none'}")
    if status.faults:
        print(f"error: latched faults ({', '.join(status.faults)}). Fix the cause (docs/troubleshooting.md), clear "
              f"them with `openxl330 clear` while torque is OFF, then run this again", file=sys.stderr)
        return False
    if status.enabled:
        print("error: torque is already on. Support the joint, turn it OFF with `openxl330 off`, then run this again",
              file=sys.stderr)
        return False
    if not status.ready:
        print("error: the servo is not ready to enable. Check `openxl330 status` and try again", file=sys.stderr)
        return False
    return True


def move_and_watch(bus, servo, target, phase):
    """Send one slow move and print telemetry until it has settled. Returns the last State."""
    servo.move(target, duration=MOVE_S, profile="quintic")
    start = bus.wire.clock()
    while True:
        elapsed = bus.wire.clock() - start
        # One read of the motion block. torque_motor and torque_out are model estimates, not measurements.
        # Near zero speed, stiction and backlash make them uncertain (docs/torque.md).
        state = servo.state()
        print(f"   {phase:<5}{elapsed:6.2f}{math.degrees(state.position):+11.2f}{state.velocity:+10.3f}"
              f"{state.current:+10.3f}{state.torque_motor:+14.4f}{state.torque_out:+12.4f}")
        if elapsed >= MOVE_S + SETTLE_S:
            return state
        bus.wire.sleep(SAMPLE_S)


def first_move(bus, servo) -> None:
    # Step 5: load the PD configuration while torque is OFF. The firmware forgets it at every power-up.
    gains = servo.configure(kp=KP, kd=KD)
    print(f"5. Configured kp {gains['kp']} N·m/rad, kd {gains['kd']} N·m·s/rad")

    # Step 6: enable. The servo holds the position it samples while arming, so keep the shaft still.
    print("6. Enabling. Keep the shaft still while it arms.")
    servo.enable()

    # Step 7: about 5 degrees out with a slow profile, then back to the start, reading telemetry on the way.
    start = servo.position
    print(f"7. Moving {MOVE_DEG:+.1f} deg over {MOVE_S} s and back, from {math.degrees(start):+.2f} deg")
    print("   phase  time   position  velocity   current  torque_motor  torque_out")
    print("             s        deg     rad/s         A           N·m         N·m")
    move_and_watch(bus, servo, start + math.radians(MOVE_DEG), "out")
    final = move_and_watch(bus, servo, start, "back")
    print(f"   Back within {abs(math.degrees(final.position - start)):.2f} deg of the start")


def turn_off(servo) -> bool:
    """Step 8: request torque OFF and read it back. True once OFF is confirmed."""
    try:
        servo.disable()
        if not servo.status().enabled:
            print("8. Torque OFF confirmed")
            return True
        print("error: the servo still reports torque on. Cut the servo power", file=sys.stderr)
    except Error as exc:
        print(f"error: torque OFF not confirmed ({exc}). Cut the servo power", file=sys.stderr)
    return False


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog="Run it with --sim first. On hardware it asks you to confirm each physical safety step.",
    )
    connection = parser.add_mutually_exclusive_group(required=True)
    connection.add_argument("--port", help="adapter port")
    connection.add_argument("--sim", type=int, metavar="N", help="use N simulated servos")
    parser.add_argument("--id", type=int, default=1, help="servo ID (default 1)")
    args = parser.parse_args()

    try:
        confirm_checklist(interactive=args.sim is None)
        bus = Bus.simulated(args.sim) if args.sim is not None else Bus(args.port)
        with bus:
            servo = bus.servo(args.id)
            if not check_status(servo):
                return 1
            try:
                first_move(bus, servo)
            finally:
                # Runs after success, after an error and after Ctrl+C.
                off = turn_off(servo)
            if not off:
                return 1
    except EOFError:
        print("error: no confirmation (input closed). Nothing was sent", file=sys.stderr)
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
