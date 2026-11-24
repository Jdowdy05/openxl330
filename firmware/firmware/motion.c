#include "motion.h"
#include <string.h>

/* Divisions on magnitudes only (the unsigned divide is already linked; a signed one would add libgcc's __divsi3),
 * 32-bit where the clamped ranges allow and 64-bit products (the linked __aeabi_lmul) for v^2 and J*a_ref:
 * flash on the M0+. Inputs clamped to physical ranges first. */
void motion_torque(const motion_in *in, int32_t *tau_motor_q24, int32_t *tau_out_q24)
{
    *tau_motor_q24 = *tau_out_q24 = 0;
    if (!in->enabled) return;
    const bool neg_d = in->pwm < 0, neg_v = in->v_hat_q16 < 0;
    const uint32_t ap = (uint32_t)(neg_d ? -(int32_t)in->pwm : in->pwm);
    const uint32_t mv = in->mv > 9000u ? 9000u : in->mv;                         /* vmax trips at 7.0 V */
    uint32_t av = neg_v ? 0u - (uint32_t)in->v_hat_q16 : (uint32_t)in->v_hat_q16;
    if (av > (uint32_t)MOTION_V_CLAMP_Q16) av = (uint32_t)MOTION_V_CLAMP_Q16;     /* M288 32 rad/s (no-load ~15); M077 64 */
    const int32_t kv = (int32_t)((mv * ap >> 2) * MOTION_K_C >> 8);              /* K*V*|d|, Q24 Nm (< 2^32) */
    const int32_t be = (int32_t)((MOTION_B0_Q16 * (av >> 1) >> 7)                /* b(d)*|v| = B0*|v| ... */
                     - (MOTION_BDROP_Q16 * (av >> 5) / 480u * ap >> 3));         /* ... - BDROP*|d|*|v|, Q24 */
    const int32_t tm = (neg_d ? -kv : kv) - (neg_v ? -be : be);
    const int32_t ja = (int32_t)((int64_t)MOTION_J_Q24 * in->a_ref_q16 / 65536);  /* J_rotor*a_ref, Q24 */
    const uint32_t kq = MOTION_KQ_Q24 * (av >> 6) >> 10;                        /* KQ*|v|, Q24 (< 2^31) */
    const uint32_t sm = av >= MOTION_VEPS_Q16 ? 65536u : av * 65536u / MOTION_VEPS_Q16;   /* |s|, Q16 */
    /* outside the knee |s| = 1: C0 + KQ*v^2; inside it (|v| < 0.05 rad/s) KQ*v^2 < 6 nNm: |s|*C0 in 32 bits */
    const int32_t sl = (int32_t)(sm == 65536u ? MOTION_C0_Q24 + (uint32_t)((uint64_t)kq * av >> 16)
                                              : (MOTION_C0_Q24 / 4u) * sm >> 14);
    const int32_t t = tm - ja - (neg_v ? -sl : sl);                              /* tau_motor - J*a - s*(C0+KQ*v^2) */
    const int32_t c1 = (int32_t)(sm * MOTION_C1_Q16 >> 16);
    const int32_t den = 65536 + ((t < 0) != neg_v ? -c1 : c1);                   /* 1 + s*C1*sign(t) > 0 */
    *tau_motor_q24 = tm;
    { const uint64_t m = (uint64_t)(t < 0 ? -(int64_t)t : t) * 65536u / (uint32_t)den; *tau_out_q24 = (int32_t)(t < 0 ? -(int64_t)m : (int64_t)m); }
}

void motion_block(const motion_in *in, uint8_t out[MOTION_BYTES])
{
    int32_t w[5];                                       /* little-endian words, as on the wire */
    motion_torque(in, &w[2], &w[3]);
    int32_t i = in->current_ma > 32767u ? 32767 : (int32_t)in->current_ma;
    w[0] = in->position_counts; w[1] = in->v_hat_q16;
    w[4] = (int32_t)((uint32_t)(uint16_t)(in->pwm < 0 ? -i : i) | (uint32_t)(uint16_t)in->pwm << 16);
    memcpy(out, w, MOTION_BYTES);
}
