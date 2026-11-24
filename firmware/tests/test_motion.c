/* FW231 motion block (firmware/motion.c): the fixed-point torque estimate against a double-precision
 * reference of the same model, the wire layout, and edge cases (OFF, clamps, knee, signs). */
#include "motion.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned checks, failures;
#define CHECK(x) do{checks++;if(!(x)){failures++;fprintf(stderr,"%s:%d: FAIL %s\n",__func__,__LINE__,#x);return;}}while(0)

/* The model with the firmware's own fixed-point constants as doubles: only the arithmetic is under test here
 * (the constants against the specified values are checked separately). */
static void reference(int16_t pwm, uint16_t mv_in, int32_t vq, int32_t aq, double *tm, double *to)
{
    double K = MOTION_K_C * 480000.0 / 1024.0 / 16777216.0;            /* Nm/V */
    double B0 = MOTION_B0_Q16 / 65536.0, BD = MOTION_BDROP_Q16 / 65536.0, KQ = MOTION_KQ_Q24 / 16777216.0;
    double C0 = MOTION_C0_Q24 / 16777216.0, C1 = MOTION_C1_Q16 / 65536.0, J = MOTION_J_Q24 / 16777216.0;
    double VE = MOTION_VEPS_Q16 / 65536.0;
    double V = (mv_in > 9000u ? 9000u : mv_in) / 1000.0, d = pwm / 480.0, a = aq / 65536.0;
    double v = vq / 65536.0;
    if (v > 32.0) v = 32.0; else if (v < -32.0) v = -32.0;
    *tm = K * V * d - (B0 - BD * fabs(d)) * v;
    double s = fabs(v) >= VE ? (v < 0 ? -1.0 : 1.0) : v / VE;
    double t = *tm - J * a - s * (C0 + KQ * v * v);
    *to = t / (1.0 + s * C1 * (t < 0 ? -1.0 : 1.0));
}
static uint32_t mix(uint32_t x) { x ^= x >> 16; x *= 0x7feb352du; x ^= x >> 15; x *= 0x846ca68bu; x ^= x >> 16; return x; }

#if APP_GEAR == 77
/* Stage 3 (XL330-M077 BETA): the M288 fit scaled by r = 77.5/288.4 (docs/M077_BETA.md): K x r, back-EMF x r^2,
 * quadratic loss x r^3, C0 x r, C1 unchanged, J x r^2, VEPS x 1/r, speed clamp 64 rad/s. The M288 edge values below
 * do not apply; matches_the_reference() checks this build against the double model with these constants. */
static void m077_constants_are_the_scaled_ones(void)
{
    const double r = 77.5 / 288.4;
    CHECK(fabs(MOTION_K_C / (2219.0 * r) - 1.0) < 2e-3 && fabs(MOTION_B0_Q16 / (1822.0 * r * r) - 1.0) < 5e-3);
    CHECK(fabs(MOTION_BDROP_Q16 / (1665.0 * r * r) - 1.0) < 5e-3 && fabs(MOTION_KQ_Q24 / (35400.0 * r * r * r) - 1.0) < 2e-3);
    CHECK(fabs(MOTION_C0_Q24 / (80531.0 * r) - 1.0) < 1e-4 && MOTION_C1_Q16 == 23593);
    CHECK(fabs(MOTION_J_Q24 / (15099.0 * r * r) - 1.0) < 1e-3 && fabs(MOTION_VEPS_Q16 / (3277.0 / r) - 1.0) < 1e-4);
    CHECK(MOTION_V_CLAMP_Q16 == (1 << 22) && MOTION_B0_Q16 > MOTION_BDROP_Q16);
}
#endif
static void constants_are_the_specified_ones(void)   /* the fsr6 fit, 2026-11-20 */
{
    CHECK(fabs(MOTION_K_C * 480000.0 / 1024.0 / 16777216.0 - 0.0620) < 2e-6);
    CHECK(fabs(MOTION_B0_Q16 / 65536.0 - 0.0278) < 2e-6 && fabs(MOTION_BDROP_Q16 / 65536.0 - 0.0254) < 2e-5);
    CHECK(MOTION_B0_Q16 > MOTION_BDROP_Q16 && fabs((MOTION_B0_Q16 - MOTION_BDROP_Q16) / 65536.0 - 0.0024) < 2e-5);   /* b > 0 at full duty */
    CHECK(fabs(MOTION_KQ_Q24 / 16777216.0 - 0.00211) < 1e-8 && fabs(MOTION_J_Q24 / 16777216.0 - 0.90e-3) < 1e-7);
    CHECK(fabs(MOTION_C0_Q24 / 16777216.0 - 0.0048) < 1e-7 && fabs(MOTION_C1_Q16 / 65536.0 - 0.36) < 1e-5);
    CHECK(fabs(MOTION_VEPS_Q16 / 65536.0 - 0.05) < 1e-5);
}

static void matches_the_reference(void)
{
    double worst_m = 0, worst_o = 0;
    for (uint32_t k = 0; k < 200000u; k++) {
        uint32_t x = mix(k * 2654435761u + 7u);
        int16_t pwm = (int16_t)((int32_t)(x % 961u) - 480);
        uint16_t mv = (uint16_t)(3500u + (x = mix(x)) % 4000u);
        int32_t vq = (int32_t)((x = mix(x)) % 1572864u) - 786432;       /* +-12 rad/s */
        if (k % 5u == 0u) vq /= 300;                                      /* inside the 0.05 rad/s knee */
        int32_t aq = (int32_t)((x = mix(x)) % 65536000u) - 32768000;      /* planned accel +-500 rad/s^2 */
        if (k % 7u == 0u) aq = 0;
        motion_in in = {0, vq, aq, pwm, mv, 0, true};
        int32_t tm, to; motion_torque(&in, &tm, &to);
        double rm, ro; reference(pwm, mv, vq, aq, &rm, &ro);
        double em = fabs(tm / 16777216.0 - rm), eo = fabs(to / 16777216.0 - ro);
        if (em > worst_m) worst_m = em;
        if (eo > worst_o) worst_o = eo;
    }
    CHECK(worst_m < 2e-5 && worst_o < 5e-5);   /* fixed-point rounding; the model itself fits to 0.7 mNm */
    printf("motion: 200000 random ticks vs the double model: tau_motor within %.1f uNm, tau_out within %.1f uNm\n", worst_m * 1e6, worst_o * 1e6);
}

static void edges(void)
{
    int32_t tm, to; double rm, ro;
    const double C0 = 0.0048, KQ = 0.00211, C1 = 0.36, J = 0.90e-3;
    motion_in off = {123, 655360, 6553600, 480, 7000, 900, false};       /* OFF: both zero, whatever the inputs */
    motion_torque(&off, &tm, &to); CHECK(tm == 0 && to == 0);
    motion_in still = {0, 0, 0, 240, 5000, 0, true};                      /* v = 0, a = 0: tau_out = tau_motor */
    motion_torque(&still, &tm, &to); CHECK(tm > 0 && to == tm);
    CHECK(fabs(tm / 16777216.0 - 0.0620 * 5.0 * 0.5) < 1e-4);            /* K*V*d = 0.155 Nm */
    motion_in accel = {0, 0, 100 * 65536, 240, 5000, 0, true};           /* planned +100 rad/s^2: J_rotor*a_ref goes to the rotor */
    int32_t tm2; motion_torque(&accel, &tm2, &to);
    CHECK(fabs((tm2 - to) / 16777216.0 - J * 100.0) < 1e-5);             /* tau_out = tau_motor - 0.09 Nm */
    motion_in coast = {0, 65536, 0, 0, 5000, 0, true};                    /* duty 0 at +1 rad/s: back-EMF braking */
    motion_torque(&coast, &tm, &to); CHECK(fabs(tm / 16777216.0 + 0.0278) < 1e-5 && to < tm);
    motion_in motoring = {0, 5 * 65536, 0, 300, 5000, 0, true};           /* driving forward: the gears eat some */
    motion_torque(&motoring, &tm, &to); CHECK(tm > 0 && to < tm);
    reference(300, 5000, 5 * 65536, 0, &rm, &ro);
    CHECK(fabs(to / 16777216.0 - (rm - C0 - KQ * 25.0) / (1.0 + C1)) < 3e-5);   /* (tau_m - C0 - KQ v^2)/(1 + C1) */
    motion_in braking = {0, 5 * 65536, 0, -300, 5000, 0, true};           /* load drives against the motor */
    motion_torque(&braking, &tm, &to); CHECK(tm < 0 && to < tm);
    reference(-300, 5000, 5 * 65536, 0, &rm, &ro);
    CHECK(fabs(to / 16777216.0 - (rm - C0 - KQ * 25.0) / (1.0 - C1)) < 3e-5);
    static const int32_t vs[4] = {INT32_MIN, INT32_MAX, -(1 << 21) - 5, (1 << 21) + 5};
    for (unsigned i = 0; i < 4u; i++) {                                   /* clamps: no overflow (UBSan fail-fast) */
        motion_in big = {0, vs[i], vs[3 - i], (int16_t)(i & 1u ? 480 : -480), 65535u, 0, true};
        motion_torque(&big, &tm, &to);
        reference(big.pwm, big.mv, vs[i], vs[3 - i], &rm, &ro);
        CHECK(fabs(tm / 16777216.0 - rm) < 2e-4 && fabs(to / 16777216.0 - ro) / (fabs(ro) + 1.0) < 1e-4);
    }
}

static void wire_layout(void)
{
    motion_in in = {-123456, -3 * 65536, 0, -200, 5000, 640, true};
    uint8_t b[MOTION_BYTES]; memset(b, 0xee, sizeof b);
    motion_block(&in, b);
    int32_t w[4]; memcpy(w, b, 16);
    int32_t tm, to; motion_torque(&in, &tm, &to);
    CHECK(w[0] == -123456 && w[1] == -3 * 65536 && w[2] == tm && w[3] == to);
    CHECK((int16_t)(b[16] | b[17] << 8) == -640 && (int16_t)(b[18] | b[19] << 8) == -200);   /* sign of the duty */
    in.pwm = 0; in.current_ma = 40000u; motion_block(&in, b);
    CHECK((int16_t)(b[16] | b[17] << 8) == 32767 && b[18] == 0 && b[19] == 0);               /* +|I| at duty 0, clamped */
    in.enabled = false; motion_block(&in, b); memcpy(w, b, 16);
    CHECK(w[2] == 0 && w[3] == 0 && w[0] == -123456);                     /* OFF: position still reported */
}

int main(void)
{
#if APP_GEAR == 77
    m077_constants_are_the_scaled_ones();
    matches_the_reference();
    wire_layout();
#else
    constants_are_the_specified_ones();
    matches_the_reference();
    edges();
    wire_layout();
#endif
    printf("FW231 motion block: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
