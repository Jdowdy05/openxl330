# XL330-M077 support (BETA)

The firmware was developed and measured on the XL330-M288-T. The XL330-M077-T builds (`GEAR=77`: release-ID variants 4 `comp-m077` and 6 `comp-m077-log`) assume the same hardware with a lower gear ratio. **Every M077 constant below is derived from the M288, not measured.** The constants will be refit on hardware, and the images stay BETA until then.

## Published differences (ROBOTIS e-manual pages for the XL330-M077 and XL330-M288)

| | XL330-M077 | XL330-M288 |
|---|---|---|
| Gear ratio | 77.5 : 1 | 288.4 : 1 |
| Stock model number | 1190 | 1200 |
| No-load speed, 5.0 V | 383 rpm | 103 rpm |
| Stall torque, 5.0 V | 0.215 N·m | 0.52 N·m |

**Identical on both:** the MCU, the magnetic encoder (4096 counts/rev), the stall currents, size and weight.

Write r = 77.5 / 288.4 = 0.268724.
- The no-load speed ratio equals 1/r, which points to the same motor.
- The stall-torque ratio (0.41) is larger than r. Read at face value, the M077 gear train passes more of the motor torque, so the absolute output torque assumed below may be low by up to ×1.54.

## Scaling rules

| Quantity | Rule | Why |
|---|---|---|
| Torque constant at the output K (host config 320) | × r | same motor, r times the gear |
| Back-EMF and viscous damping b, b_drop | × r² | motor-side speed × r, torque × r |
| Reflected rotor inertia, inertia feedback J_fb | × r² | |
| Coulomb friction c0 | × r (low bound) | if gearbox friction dominates, it would be unchanged; the low bound under-compensates, which is the safe side |
| Load-dependent friction c1 | unchanged | dimensionless |
| Speeds at the output: veps, inertia gate | × 1/r | |
| Reference-acceleration feed-forward ja | horn inertia + 40 % of the M077 rotor | the same split as the M288 |
| PD gains | × the inertia ratio (≈ r²) | same bandwidth and damping |

## Resulting values

**Power-up compensation** (`firmware/app.c`, `APP_GEAR == 77`; Q formats as in `joint_control.h`):

| Table | Values | Fields |
|---|---|---|
| `app_comp_default` | 18394, 20054, 101, 469, 12194, 0, 0, 653, 0x102 | c0, c1, b, ja, veps, boost, boost ticks, J_fb, flags |
| `app_comp2_default` | 29, 73164, 1 | b_drop, vgate, gate on |

High-pass damping stays off.

**Other constants:**
- **Jump gate** (`APP_JUMP_COUNTS`): 32 counts per tick, about 98 rad/s at the output (M288: 12). The M077 no-load speed is 40-48 rad/s.
- **Motion block** (read 1088, `firmware/motion.h`): the M288 fit scaled the same way. The speed clamp is 64 rad/s.
- **Host defaults** (config 320): K = 0.0504 × r = 0.013544 N·m/V (Q24 227225).
  - Gains with the compensation loaded: Kp 0.253 N·m/rad, Kd 0.0072 N·m·s/rad.
  - Gains with the compensation zeroed: Kp 0.123, Kd 0.0031.
  - Never use the M288 gains on an M077.

## Safety

**Each image arms only on its own stock model.** `app_init` refuses any other NV model number, so the output stays off:
- an M288 image on an M077 cannot over-compensate it, which would likely self-spin;
- an M077 image will not run an M288.

Installers accept an M077 image only with an explicit beta flag.

**First power-up of an M077:**
1. Turn the horn by hand and check that one revolution reads 4096 counts.
2. Run with the compensation zeroed (write 324 all zero) and the conservative gains, using small moves.
3. Enable the compensation last.
