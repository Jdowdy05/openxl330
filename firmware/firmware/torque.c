#include "torque.h"
#include <limits.h>
#include <stddef.h>

/* 65 samples of sin(pi*j/128), rounded to Q15, linearly interpolated.
 * The maximum interpolation error is <3 Q15 counts (<0.000092). */
static const uint16_t quarter_sine[65] = {
    0,804,1608,2410,3212,4011,4808,5602,
    6393,7179,7962,8739,9512,10278,11039,11793,
    12539,13279,14010,14732,15446,16151,16846,17530,
    18204,18868,19519,20159,20787,21403,22005,22594,
    23170,23731,24279,24811,25329,25832,26319,26790,
    27245,27683,28105,28510,28898,29268,29621,29956,
    30273,30571,30852,31113,31356,31580,31785,31971,
    32137,32285,32412,32521,32609,32678,32728,32757,
    32767
};

/* High half of an unsigned32x32 product using native32-bit multiplies.
 * Each partial sum fits uint32 (maximum middle/carry is4294901759).
 * Cortex-M0+ has no long multiply; this avoids a generic64-bit helper. */
static uint32_t multiply_high_u32(uint32_t a, uint32_t b) {
    uint32_t a0=a & 65535u, a1=a >> 16;
    uint32_t b0=b & 65535u, b1=b >> 16;
    uint32_t low=a0*b0;
    uint32_t carry=a1*b0+(low >> 16);
    uint32_t middle=(carry & 65535u)+a0*b1;
    return a1*b1+(carry >> 16)+(middle >> 16);
}

/* m=ceil(2^32/d). For n<2^32, floor(n*m/2^32) overestimates n/d
 * by at most1. The correction is exact; guard q*d against uint32 wrap. */
static uint32_t divide_u32(uint32_t n, uint32_t d) {
    uint32_t magic, maximum_quotient;
    if (d == 1000u) { magic=4294968u; maximum_quotient=4294967u; }
    else if (d == 1000000u) { magic=4295u; maximum_quotient=4294u; }
    else if (d == 32767u) { magic=131077u; maximum_quotient=131076u; }
    else return n/d;
    uint32_t q=multiply_high_u32(n,magic);
    if (q > maximum_quotient || q*d > n) --q;
    return q;
}

/* All callers bound numerator far from INT64_MIN and denominator >0.
 * Rounding remains nearest, with ties away from zero. Common physical values
 * use native32-bit operations; the full-domain64-bit fallback remains exact. */
static int64_t round_div(int64_t numerator, int64_t denominator) {
    if (numerator == 0) return 0;
    bool negative=numerator < 0;
    uint64_t magnitude=negative ? (uint64_t)(-numerator) : (uint64_t)numerator;
    uint64_t rounded=magnitude+(uint64_t)denominator/2u;
    uint64_t quotient;
    if (denominator == 4096) quotient=rounded >> 12;
    else if (denominator == 65536) quotient=rounded >> 16;
    else if ((uint64_t)denominator <= UINT32_MAX && rounded <= UINT32_MAX) {
        quotient=divide_u32((uint32_t)rounded,(uint32_t)denominator);
    } else if (denominator == 1000000 && rounded < (1ULL << 41)) {
        /* ceil(2^41/1000000)=2199024. The product is below2^63;
         * approximation error <1, so one downward correction suffices.
         * This covers large ordinary Kp products without a long divide. */
        quotient=(rounded*2199024u) >> 41;
        if (quotient*1000000u > rounded) --quotient;
    } else quotient=rounded/(uint64_t)denominator;
    return negative ? -(int64_t)quotient : (int64_t)quotient;
}

/* Exact round(value*factor/denominator), with factor<=denominator.
 * Split into integer quotient/remainder before scaling: the hot velocity
 * difference fits32 even though difference*dt does not. Both divisions then
 * stay32-bit for ordinary499/500/501us samples, including IRQ jitter.
 * Remainder and integer part have the same sign, preserving ties-away. */
static int64_t round_fraction(int64_t value, uint32_t factor, uint32_t denominator) {
    bool negative=value < 0;
    uint64_t magnitude=negative ? (uint64_t)(-value) : (uint64_t)value;
    if (magnitude <= UINT32_MAX) {
        uint32_t n=(uint32_t)magnitude;
        uint32_t whole=n/denominator;
        uint32_t remainder=n%denominator;
        if (denominator <= 65536u) {
            /* factor<=denominator, so both rounded remainder product and
             * final result fit uint32. Avoid even long multiplies here. */
            uint32_t tail_n=remainder*factor+denominator/2u;
            uint32_t result=whole*factor+divide_u32(tail_n,denominator);
            return negative ? -(int64_t)result : (int64_t)result;
        }
        uint64_t tail_n=(uint64_t)remainder*factor+denominator/2u;
        if (tail_n <= UINT32_MAX) {
            uint64_t result=(uint64_t)whole*factor
                + divide_u32((uint32_t)tail_n,denominator);
            return negative ? -(int64_t)result : (int64_t)result;
        }
    }
    return round_div(value*factor,denominator);
}

static bool within(int32_t value, int32_t limit) {
    return value >= -limit && value <= limit;
}

static int32_t clamp_i32(int64_t value, uint8_t *saturated) {
    if (value > INT32_MAX) { *saturated = 1; return INT32_MAX; }
    if (value < INT32_MIN) { *saturated = 1; return INT32_MIN; }
    return (int32_t)value;
}

int32_t torque_sin_q15(int32_t phase_counts) {
    uint32_t phase = (uint32_t)phase_counts & 4095u;
    uint32_t quadrant = phase >> 10;
    uint32_t offset = phase & 1023u;
    if ((quadrant & 1u) != 0u) offset = 1024u - offset;
    uint32_t index = offset >> 4;
    int32_t value;
    if (index == 64u) value = 32767;
    else {
        uint32_t fraction = offset & 15u;
        uint32_t low = quarter_sine[index];
        uint32_t high = quarter_sine[index+1u];
        value = (int32_t)(low + ((high-low)*fraction + 8u)/16u);
    }
    return quadrant >= 2u ? -value : value;
}

torque_config_t torque_config_default(void) {
    torque_config_t c = {
        .kp_mNm_rad=100, .kd_mNm_rad_s=20, .a_mNm_per_V=100,
        .b_mNm_per_rad_s=0, .c_positive_mNm=0, .c_negative_mNm=0,
        .gravity_mNm=0, .gravity_zero_counts=0,
        .velocity_tau_us=10000, .pwm_limit_ticks=480
    };
    return c;
}

bool torque_config_valid(const torque_config_t *c) {
    return c != NULL
        && c->kp_mNm_rad >= 0 && c->kp_mNm_rad <= TORQUE_MAX_GAIN
        && c->kd_mNm_rad_s >= 0 && c->kd_mNm_rad_s <= TORQUE_MAX_GAIN
        && c->a_mNm_per_V >= 1 && c->a_mNm_per_V <= TORQUE_MAX_MODEL
        && c->b_mNm_per_rad_s >= 0 && c->b_mNm_per_rad_s <= TORQUE_MAX_MODEL
        && c->c_positive_mNm >= 0 && c->c_positive_mNm <= TORQUE_MAX_MODEL
        && c->c_negative_mNm >= 0 && c->c_negative_mNm <= TORQUE_MAX_MODEL
        && within(c->gravity_mNm, TORQUE_MAX_MODEL)
        && c->velocity_tau_us <= TORQUE_MAX_TAU_US
        && c->pwm_limit_ticks <= TORQUE_PWM_FULL_TICKS;
}

bool torque_step_with_loss(const torque_config_t *c, const torque_command_t *g,
                 const torque_input_t *i, int32_t loss_mNm, torque_output_t *o) {
    if (o == NULL) return false;
    o->high_ticks = 0;
    o->requested_mNm = 0;
    o->saturated = 0;
    if (!torque_config_valid(c) || g == NULL || i == NULL
        || !within(g->target_velocity_mrad_s, TORQUE_MAX_VELOCITY_MRAD_S)
        || !within(g->feedforward_mNm, TORQUE_MAX_MODEL)
        || !within(i->velocity_mrad_s, TORQUE_MAX_VELOCITY_MRAD_S)
        || i->voltage_mV < 1000u || i->voltage_mV > 20000u
        || i->dt_us == 0u || i->dt_us > TORQUE_MAX_DT_US) return false;

    /* 2*pi =6283185 microradians, error <=0.31ppm. Converting before
     * multiplying the gain bounds the largest int64 product below6.6e17,
     * even for INT32_MIN -> INT32_MAX unwrapped position differences. */
    int64_t count_error = (int64_t)g->target_counts - i->position_counts;
    int64_t error_urad = round_div(count_error * 6283185, 4096);
    int64_t p_mNm = round_div(error_urad * c->kp_mNm_rad, 1000000);
    int64_t velocity_error = (int64_t)g->target_velocity_mrad_s - i->velocity_mrad_s;
    int64_t d_mNm = round_div(velocity_error * c->kd_mNm_rad_s, 1000);
    uint32_t phase = ((uint32_t)i->position_counts
                      - (uint32_t)c->gravity_zero_counts) & 4095u;
    int64_t gravity = round_div((int64_t)c->gravity_mNm
                               * torque_sin_q15((int32_t)phase), 32767);
    int64_t requested = p_mNm + d_mNm + g->feedforward_mNm + gravity;
    o->requested_mNm = clamp_i32(requested, &o->saturated);

    int64_t inverse_mNm = (int64_t)o->requested_mNm + loss_mNm
        + round_div((int64_t)c->b_mNm_per_rad_s * i->velocity_mrad_s, 1000);
    if (i->velocity_mrad_s > 0) inverse_mNm += c->c_positive_mNm;
    if (i->velocity_mrad_s < 0) inverse_mNm -= c->c_negative_mNm;
    /* tau=A*V*d -B*v -C_direction. Full carrier remains480 ticks even
     * when a smaller user cap is selected. Largest numerator <1.6e15. */
    int64_t numerator = inverse_mNm * TORQUE_PWM_FULL_TICKS * 1000;
    int64_t denominator = (int64_t)c->a_mNm_per_V * i->voltage_mV;
    /* Exact cap test before division: round(|n|/d)>cap iff
     * |n|+floor(d/2)>=(cap+1)*d. Requested torque is still reported in full.
     * Common saturated outputs (including cap0 qualification) need no divide. */
    uint64_t magnitude=numerator < 0 ? (uint64_t)(-numerator) : (uint64_t)numerator;
    uint64_t rounded=magnitude+(uint64_t)denominator/2u;
    uint64_t cap_boundary=((uint64_t)c->pwm_limit_ticks+1u)*(uint64_t)denominator;
    if (rounded >= cap_boundary) {
        o->high_ticks=numerator < 0 ? -(int16_t)c->pwm_limit_ticks : (int16_t)c->pwm_limit_ticks;
        o->saturated=1;
        return true;
    }
    int64_t ticks = round_div(numerator, denominator);
    if (ticks > c->pwm_limit_ticks) {
        ticks = c->pwm_limit_ticks;
        o->saturated = 1;
    } else if (ticks < -(int64_t)c->pwm_limit_ticks) {
        ticks = -(int64_t)c->pwm_limit_ticks;
        o->saturated = 1;
    }
    o->high_ticks = (int16_t)ticks;
    return true;
}

bool torque_step(const torque_config_t *c, const torque_command_t *g,
                 const torque_input_t *i, torque_output_t *o) {
    return torque_step_with_loss(c,g,i,0,o);
}

void torque_velocity_reset(torque_velocity_t *s) {
    if (s == NULL) return;
    s->last_us = 0;
    s->last_position_counts = 0;
    s->velocity_mrad_s = 0;
    s->velocity_q16 = 0;
    s->last_fraction_q16 = 0;
    s->initialized = 0;
}

static void velocity_anchor(torque_velocity_t *s, int32_t p, uint32_t t) {
    s->last_us = t;
    s->last_position_counts = p;
    s->velocity_mrad_s = 0;
    s->velocity_q16 = 0;
    s->last_fraction_q16 = 0;
    s->initialized = 1;
}

bool torque_velocity_update(torque_velocity_t *s, int32_t p, uint32_t t,
                            uint32_t tau) {
    if (s == NULL) return false;
    if (tau > TORQUE_MAX_TAU_US) {
        velocity_anchor(s, p, t);
        return false;
    }
    if (!s->initialized) {
        velocity_anchor(s, p, t);
        return true;
    }
    uint32_t dt = t - s->last_us;
    if (dt == 0u || dt > TORQUE_MAX_DT_US) {
        velocity_anchor(s, p, t);
        return false;
    }
    int64_t delta = (int64_t)p - s->last_position_counts;
    int64_t delta_urad = round_div(delta * 6283185, 4096);
    int64_t raw_mrad_s = round_div(delta_urad * 1000, dt);
    if (raw_mrad_s < -TORQUE_MAX_VELOCITY_MRAD_S
        || raw_mrad_s > TORQUE_MAX_VELOCITY_MRAD_S) {
        velocity_anchor(s, p, t);
        return false;
    }
    int64_t raw_q16 = raw_mrad_s * 65536;
    if (tau == 0u) s->velocity_q16 = raw_q16;
    else {
        int64_t difference = raw_q16 - s->velocity_q16;
        s->velocity_q16 += round_fraction(difference,dt,tau+dt);
    }
    s->velocity_mrad_s = (int32_t)round_div(s->velocity_q16, 65536);
    s->last_position_counts = p;
    s->last_fraction_q16 = 0;
    s->last_us = t;
    return true;
}

static void fractional_anchor(torque_velocity_t *s,int32_t p,int32_t f,uint32_t t){velocity_anchor(s,p,t);s->last_fraction_q16=f;}
/* Exact symmetric nearest rounding of bounded signed product, without division. */
static int64_t shift28(int64_t n){bool neg=n<0;uint64_t m=neg?(uint64_t)-n:(uint64_t)n;int64_t q=(int64_t)((m+UINT64_C(134217728))>>28);return neg?-q:q;}
bool torque_fractional_speed(int32_t p0,int32_t f0,int32_t p,int32_t f,uint32_t dt,int32_t *out){
 if(!out || f0 < -65535 || f0 > 65535 || f < -65535 || f > 65535 || !dt || dt>TORQUE_MAX_DT_US)return false;
 int64_t dq=((int64_t)p-p0)*65536+((int64_t)f-f0);
 if(dq<=-INT64_C(268435456) || dq>=INT64_C(268435456))return false;
 int64_t urad=shift28(dq*6283185),raw=round_div(urad*1000,dt);
 if(raw < -TORQUE_MAX_VELOCITY_MRAD_S || raw > TORQUE_MAX_VELOCITY_MRAD_S)return false;
 *out=(int32_t)raw;return true;
}
bool torque_velocity_update_fractional(torque_velocity_t *s,int32_t p,int32_t f,uint32_t t){
 if(!s)return false;
 if(f < -65535 || f > 65535){velocity_anchor(s,p,t);return false;}
 if(!s->initialized){fractional_anchor(s,p,f,t);return true;}
 int32_t raw;
 if(!torque_fractional_speed(s->last_position_counts,s->last_fraction_q16,p,f,t-s->last_us,&raw)){fractional_anchor(s,p,f,t);return false;}
 s->velocity_mrad_s=raw;s->velocity_q16=(int64_t)raw*65536;s->last_position_counts=p;s->last_fraction_q16=f;s->last_us=t;return true;
}
