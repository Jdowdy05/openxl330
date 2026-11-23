# Using one XL330-M288 as a gripper

Notes for the [gripper example](../examples/gripper.py). kp is in N·m/rad ([API guide](api.md#servo-control-and-telemetry)).

## Set the squeeze

Once the jaws stop on an object, the controller's torque is about kp × (target − position). Set the target a small angle θ past the contact point, and the holding torque is about kp × θ. Choose the torque and kp, then θ = torque / kp.

If the object yields or slips, the jaws move toward the target and the torque falls, so it stays below about kp × θ. A lower kp with a larger θ tolerates errors in the contact position. A higher kp resists a pull more stiffly.

The example reports contact when the closing jaws trail the planned path by θ/2. If it reports contact in free air, close more slowly or raise the squeeze.

## From torque to force

Force ≈ torque / lever arm, from the horn axis to the contact point. For example, kp 1.0 N·m/rad and θ 0.10 rad (5.7°) give 0.10 N·m. At a 40 mm lever arm, that is 0.10 / 0.040 = 2.5 N. Gear friction makes the real force differ.

## At stall

A holding gripper is a stalled motor: all its power becomes heat. ROBOTIS rates the XL330-M288 at about 0.52 N·m stall torque and 1.47 A stall current at 5.0 V. Ask for more torque and the jaws push at stall, drawing about the stall current.

## Watch current and torque

`servo.state()` gives `current` (A), `torque_motor` and `torque_out` (N·m). These are model estimates, uncertain near zero speed because of stiction and backlash ([torque estimates](torque.md)). Compare them with kp × error, and use a load cell for a measured force. Register 848 reports the filtered RMS current the overload protection uses ([protections](api.md#protections)).

## Current overload

By default, torque turns OFF when the RMS current exceeds 1.0 A (I²t filter, time constant about 2.05 s). A hold below 1.0 A never trips it. In firmware tests, a constant 1.1 A tripped it in about 3.6 s, and 100 ms bursts of 2.5 A never did. By the same formula, a hold at the rated stall current trips in roughly 1.3 s.

A trip turns torque OFF and latches the `current` fault. The gripper opens or goes limp, and the object can drop. Clear the fault while torque is OFF (`clear_faults()` or `openxl330 clear`), then `enable()`. Clearing does not reset the I²t filter, so wait about 2 s, or squeeze less, before arming again: under the same load it trips again almost at once.

To change the limit, write register 848 while torque is OFF. It lasts until the next boot; 0 turns the protection off. A higher limit, or none, lets a hard grip last longer but heats the motor faster. The 65 °C protection remains.

## Temperature

Heating grows with the square of the current. Watch `status().temperature` during long holds. At 65 °C, torque turns OFF and `temperature` latches. Squeeze only as hard and as long as needed.
