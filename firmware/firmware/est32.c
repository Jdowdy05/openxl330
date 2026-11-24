/* 32-bit count-domain constant-acceleration tracker (research prototype).
 * No heap, no float, no 64-bit multiply. Fixed storage: 4 words.
 *
 * Input: calibrated position in Q16 counts (int64 so multi-turn is allowed);
 * internally only the per-tick delta (int32) is used.
 * State (Q20 counts): e = q_hat - y_last, dq per tick, ddq per tick^2.
 * Gains: dimensionless Q32 (k = K*2^32 for q, K_v*H, K_a*H^2), all < 1.
 * Outputs (one-tick lead optional):
 *   q_off_q32 : q_hat(+lead) - y_last, in Q32 rad (add to the firmware angle)
 *   v_q16     : rad/s Q16 using the firmware rational 411775/2^28 rad/count
 *   a_q16     : rad/s^2 Q16
 * Raw measurements are unaffected; callers log raw v/a separately.
 * Any rejection reports false and un-primes the state (the only mutation),
 * so the next sample re-initializes instead of locking out. A gross
 * innovation resyncs to the measurement with velocity seeded from the raw delta.
 */
#include "est32.h"

#define EST32_LIMIT (INT32_C(1) << 27)   /* all state magnitudes stay below this */

/* Exact round(x*k / 2^32), ties away from zero; |x| <= 2^31, k < 2^32. */
static int32_t mul_q32(int32_t x, uint32_t k)
{
    uint32_t m = x < 0 ? UINT32_C(0) - (uint32_t)x : (uint32_t)x;
    uint32_t mh = m >> 16, ml = m & 0xffffu, kh = k >> 16, kl = k & 0xffffu;
    uint32_t lo = ml * kl, mid1 = mh * kl, mid2 = ml * kh, hi = mh * kh;
    uint32_t t = (lo >> 16) + (mid1 & 0xffffu) + (mid2 & 0xffffu);
    uint32_t q = hi + (mid1 >> 16) + (mid2 >> 16) + (t >> 16);
    uint32_t frac = (t << 16) | (lo & 0xffffu);
    q += frac >= UINT32_C(0x80000000);
    return x < 0 ? -(int32_t)q : (int32_t)q;
}

static int32_t rshift_round(int32_t x, unsigned n)
{
    uint32_t m = x < 0 ? UINT32_C(0) - (uint32_t)x : (uint32_t)x;
    uint32_t q = (m + (UINT32_C(1) << (n - 1u))) >> n;
    return x < 0 ? -(int32_t)q : (int32_t)q;
}

static bool small(int32_t v) { return v > -EST32_LIMIT && v < EST32_LIMIT; }

bool est32_config_valid(const est32_config *c)
{
    return c && c->deadzone_q20 >= 0 && c->deadzone_q20 < (INT32_C(1) << 24)
        && c->resync_q20 > 0 && c->resync_q20 < EST32_LIMIT && c->lead_ticks <= 1u;
}

void est32_init(est32_state *s) { *s = (est32_state){0}; }

bool est32_update(est32_state *s, const est32_config *c, int64_t y_q16, est32_output *out)
{
    int32_t e, dq, ddq, r = 0;
    uint8_t resynced = 0;
    if (!s->primed) {
        e = 0; dq = 0; ddq = 0;
    } else {
        int64_t d64 = y_q16 - s->y_last_q16;
        if (d64 <= -(INT64_C(1) << 22) || d64 >= (INT64_C(1) << 22)) { s->primed = 0; return false; } /* >64 counts/tick */
        int32_t delta = (int32_t)d64 * 16;           /* Q20 counts */
        ddq = s->ddq;
        dq = s->dq + ddq;
        int32_t ep = s->e + s->dq + rshift_round(ddq, 1); /* predicted q_hat - y_last */
        r = delta - ep;                               /* innovation, |r| < 2^29 */
        if (r > c->resync_q20 || r < -c->resync_q20) {
            e = 0; dq = delta; ddq = 0; resynced = 1;   /* re-seed velocity from the raw delta */
        } else {
            int32_t rr = r;
            if (c->deadzone_q20) {
                if (rr > c->deadzone_q20) rr -= c->deadzone_q20;
                else if (rr < -c->deadzone_q20) rr += c->deadzone_q20;
                else rr = 0;
            }
            e = mul_q32(rr, c->k0) - r;               /* q_new - y_new */
            dq += mul_q32(rr, c->k1);
            ddq += mul_q32(rr, c->k2);
        }
    }
    if (!small(e) || !small(dq) || !small(ddq)) { s->primed = 0; return false; }
    int32_t eo = e, dqo = dq;
    if (c->lead_ticks) { eo = e + dq + rshift_round(ddq, 1); dqo = dq + ddq; }
    if (!small(eo) || !small(dqo)) { s->primed = 0; return false; }
    /* SI conversions with the firmware rational 411775/2^28 rad per count:
     * v_q16 = dq_q20 * 823550000/2^32 ; a_q16 = ddq_q20 * 383.495353...;
     * q_q32 = e_q20 * 411775/2^16 (= 6 + 1216282624/2^32), all exact before rounding */
    int32_t v = mul_q32(dqo, UINT32_C(823550000));
    if (ddq > 5599000 || ddq < -5599000) { s->primed = 0; return false; } /* a beyond +-32767 rad/s^2 Q16 */
    int32_t a = ddq * 383 + mul_q32(ddq, UINT32_C(2127525632));
    int32_t qo = eo * 6 + mul_q32(eo, UINT32_C(1216282624));
    s->e = e; s->dq = dq; s->ddq = ddq; s->y_last_q16 = y_q16; s->primed = 1;
    if (resynced && s->resyncs < UINT16_MAX) s->resyncs++;
    out->q_off_q32 = qo; out->v_q16 = v; out->a_q16 = a; out->innovation_q20 = r; out->resynced = resynced;
    return true;
}

bool est32_predict(const est32_state *s, int64_t *y_q16)
{
    if (!s->primed) return false;
    int32_t ep = s->e + s->dq + rshift_round(s->ddq, 1);     /* Q20 counts, |ep| < 2^29 */
    *y_q16 = s->y_last_q16 + rshift_round(ep, 4);
    return true;
}
