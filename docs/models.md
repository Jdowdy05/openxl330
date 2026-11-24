# Servo models

openxl330 knows two XL330 models. The host picks its constants from the model the servo reports: the torque constant written in every PD configuration, and the default gains `configure()` uses. They are in [`openxl330.models`](../src/openxl330/models.py).

| | XL330-M288 | XL330-M077 |
| --- | --- | --- |
| Model number | 1200 | 1190 |
| Support | Supported | **Beta**: `--beta` / `allow_beta=True` |
| Gear ratio | 288.4 : 1 | 77.5 : 1 |
| No-load speed at 5.0 V (ROBOTIS) | 103 rev/min | 383 rev/min |
| Stall torque at 5.0 V (ROBOTIS) | 0.52 N·m (about 0.23 N·m measured on the bench, roughly 45 %) | 0.215 N·m |
| Torque constant K (N·m/V at full duty) | 0.0504, measured | 0.01354, assumed |
| Default gains with compensation (Kp, Kd) | 3.5, 0.10, tuned | 0.253, 0.0072, assumed |
| Default gains without compensation (Kp, Kd) | 1.71, 0.043 | 0.123, 0.0031, assumed |
| Firmware image | `comp` variant | `comp-m077` variant (release 17+) |

The gear ratios, model numbers, speeds and stall torques are from the ROBOTIS e-manual pages for the [XL330-M077](https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/) and the [XL330-M288](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/). Both use the same motor and electronics, and list the same stall current.

## Why the XL330-M077 is a beta

The firmware side of the derivation is in [firmware/docs/M077_BETA.md](https://github.com/Jdowdy05/openxl330/blob/main/firmware/docs/M077_BETA.md).

Every XL330-M077 constant is an assumption. They are scaled from the measured XL330-M288 values by the gear ratio:

- the torque constant by the ratio (77.5 / 288.4);
- the gains by the ratio squared, for the same closed-loop bandwidth (the rotor dominates the joint's inertia).

The published figures do not quite agree with that scaling. The no-load speeds differ by 3.72×, the gear ratio, but the stall torques by only 2.42×. So the true torque constant may sit anywhere between the ratio-scaled 0.0135 N·m/V and a stall-scaled 0.0208 N·m/V. Torque commands and estimates on an XL330-M077 can be off by up to that factor until the constants are measured on hardware. The XL330-M077 stays a beta until they are.

## Rules

- **Installing** on a stock XL330-M077 needs `--beta` (CLI) or `allow_beta=True` (`install.from_stock`, `install.update`). The installer then picks its image, `fw231-comp2-fast17-m077` (or `-m077-log` with `--variant logging`). An image built for the other model is refused before anything is written. **None of these images has run on an XL330-M077 yet.**
- **No stock version of the XL330-M077 is proven.** With `--beta`, its stock firmware counts as untested: the receipt records it, and `--allow-untested` is not needed as well.
- **Images and models must match.** The bus refuses an XL330-M077 running an XL330-M288 image, or the reverse, because their torque constants differ by about 3.7×. Releases before 17 report model 1200 on every servo. Their firmware refuses to arm on an XL330-M077.
- **`ServoInfo.beta`** is `True` for a servo of a beta model.
- **flash-all is XL330-M288 only.** `experimental flash-all` and the route back to stock take XL330-M288 servos only. Install XL330-M077 servos one at a time.
