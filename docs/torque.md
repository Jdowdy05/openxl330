# Torque estimates

`Servo.state()` and `Group.read()` return two model estimates in N·m, signed in the calibrated joint direction. They also report velocity, duty and current.

- **`torque_motor`** is electromagnetic motor torque referred to the output, estimated from duty, supply voltage and velocity using fixed actuator coefficients.
- **`torque_out`** estimates the load torque at the horn. The model subtracts estimated gear losses and reflected rotor inertia times the planned acceleration. The acceleration term uses the quintic reference. It is zero for a linear profile, a step, hold, open-loop duty, torque OFF or a built-in trajectory.
- **`set_torque()`** (release 16+) commands torque through the same actuator model, so a commanded torque is set, not measured. `control_mode()` reports the commanded value after the configured limit. In torque mode, `torque_out` uses no planned acceleration, so during speed changes it includes the rotor's inertia torque.
- **`current`** is an approximate signed current. The firmware assigns duty’s sign to current magnitude. Actual current can reverse during regeneration. Both torque estimates read zero while torque is OFF.

## When to use `torque_out`

Use it to inspect loads during a hold, at slow or steady speed, or on a timed quintic move the servo can follow. Near zero speed, stiction and backlash make the sign and magnitude uncertain. Treat the result as an estimate alongside position and velocity.

In bench measurements, constant-speed torque was within a few mN·m at 35% duty. Error at 85% duty was between −24 and +14 mN·m in that test. Quintic moves lasting 450 ms showed 22–27 mN·m RMS error. The measured values depend on the setup and trajectory.

## When to avoid `torque_out`

Avoid using it as a load measurement during position steps, open-loop transients, saturated moves or fast external pushes. In those conditions the rotor’s actual acceleration can dominate the output while the model subtracts only the planned term. Tracking error means the two terms differ. Saturated 150 ms quintic moves showed 144–165 mN·m RMS error in the bench measurements. Fast transients included peaks of 0.4–0.8 N·m from rotor inertia. The load-dependent friction correction amplifies braking torque opposite to velocity by about 1.56 times.

Use a calibrated sensor when you need measured load torque. The estimates are intended for telemetry and tuning. See [motion state](api.md#servo-control-and-telemetry) and the [register map](registers.md).
