#ifndef MOTION_H
#define MOTION_H
/* FW231 motion block (owner 2026-11-20, "Yes, add it"; model improved after hardware run fsr6, "Yes, improve it"):
 * then planned acceleration for the rotor's inertia, owner: "Use planned acceleration"):
 * position, velocity, an estimated torque, signed current and the applied duty in ONE read (address 1088, 20 B),
 * so one Sync / Fast Sync Read gets them for every servo.
 *
 *   0 position_counts  i32  same as address 96 (4096 counts per output turn)
 *   4 v_hat            i32  Q16 rad/s, the controller's velocity estimate (same as read 640 word 6)
 *   8 est_tau_motor    i32  Q24 Nm, electromagnetic torque referred to the output shaft:
 *                           K*V*d - b(d)*v_hat, d = duty -1..1 (pwm/480), b(d) = B0 - BDROP*|d| (> 0 for |d| <= 1)
 *  12 est_tau_out      i32  Q24 Nm, torque at the horn: tau_motor - J_ROTOR*a_ref - loss, where
 *                           loss = s*(C0 + KQ*v_hat^2 + C1*|tau_out|), s = sat(v_hat/VEPS); solved exactly:
 *                           tau_out = t/(1 + s*C1*sign(t)), t = tau_motor - J_ROTOR*a_ref - s*(C0 + KQ*v_hat^2).
 *                           a_ref = the PLANNED acceleration of the running profile (quintic; 0 for a linear one),
 *                           0 otherwise: hold, pure steps (duration 1), open-loop duty (260), OFF, BAM.
 *                           So during planned moves it approximates the LOAD torque at the horn; during steps,
 *                           open-loop or external pushes it INCLUDES the rotor's inertia torque J_ROTOR*alpha (no
 *                           planned acceleration to remove it), and any tracking error shows as J_ROTOR*(a_ref-a).
 *                           (fast7 used the measured fast-tracker acceleration: its noise dominated, hardware fsr7.)
 *                           Unreliable near zero speed (stiction, backlash: the loss is not a function of v there)
 *  16 current_ma       i16  |I| (address 124) with the sign of the applied duty: an APPROXIMATION, regeneration
 *                           can reverse the real current; +|I| when the duty is 0
 *  18 applied_pwm      i16  duty actually applied by the last tick, -480..480 = -100..100 %
 *
 * Both torques are 0 while OFF. Fixed constants, NOT the live gain / gear-train cancellation settings: retuning or
 * zeroing those does not change it. Constants: the fit to hardware run fsr6 (16 constant-speed points
 * + 24 impulse-momentum windows, horn only, J_total 0.92e-3 from the pendulum sys-ID; constant-speed residual
 * 0.7 mNm rms, impulse error 8 %; 2 servos, <= 50 % duty at constant speed). The supply is the per-tick measured V.
 * Computed by the reader from one tick's coherent snapshot: no control-tick time, PRIMASK only for the copy. */
#include <stdbool.h>
#include <stdint.h>
_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "motion_block copies little-endian words");

enum {
    MOTION_ADDRESS = 1088u,
    MOTION_BYTES = 20u,
#ifndef APP_GEAR
#define APP_GEAR 288
#endif
#if APP_GEAR == 77
    /* XL330-M077 BETA (Stage 3, DERIVED from the M288 fit by r = 77.5/288.4; docs/M077_BETA.md): K x r, back-EMF
     * x r^2, quadratic loss x r^3 and C0 x r (motor side, the low bounds), C1 unchanged, J x r^2, VEPS x 1/r. */
    MOTION_K_C = 596,
    MOTION_B0_Q16 = 132,
    MOTION_BDROP_Q16 = 120,
    MOTION_KQ_Q24 = 687,
    MOTION_C0_Q24 = 21641,
    MOTION_C1_Q16 = 23593,
    MOTION_J_Q24 = 1090,
    MOTION_VEPS_Q16 = 12194,        /* 0.05/r exact, the same as the M077 comp table veps */
    MOTION_V_CLAMP_Q16 = 1 << 22   /* 64 rad/s (M077 no-load 40-48 rad/s) */
};
#else
    MOTION_K_C = 2219,            /* K 0.0620 Nm/V as mV*pwm/4 -> Q24 * 2^8: 0.0620 * 2^24 * 1024 / 480000 = 2219.07 */
    MOTION_B0_Q16 = 1822,         /* 0.0278 Nm s/rad back-EMF slope at zero duty */
    MOTION_BDROP_Q16 = 1665,      /* 0.0254: the slope falls to 0.0024 at full duty */
    MOTION_KQ_Q24 = 35400,        /* 0.00211 Nm per (rad/s)^2: speed-dependent loss */
    MOTION_C0_Q24 = 80531,        /* 4.8 mNm load-independent Coulomb (plant.py C0) */
    MOTION_C1_Q16 = 23593,        /* 0.36 per Nm transmitted (plant.py C1) */
    MOTION_J_Q24 = 15099,         /* 0.90e-3 kg m^2 rotor + gears reflected to the output (plant.py J_ROTOR) */
    MOTION_VEPS_Q16 = 3277,       /* 0.05 rad/s smooth-sign knee (same as the cancellation) */
    MOTION_V_CLAMP_Q16 = 1 << 21  /* 32 rad/s (no-load ~15) */
};
#endif

typedef struct {
    int32_t position_counts, v_hat_q16, a_ref_q16;    /* a_ref: planned acceleration, Q16 rad/s^2 (0 = none) */
    int16_t pwm;                  /* applied, -480..480 */
    uint16_t mv, current_ma;
    bool enabled;
} motion_in;

void motion_torque(const motion_in *in, int32_t *tau_motor_q24, int32_t *tau_out_q24);
void motion_block(const motion_in *in, uint8_t out[MOTION_BYTES]);

#endif
