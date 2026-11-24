#ifndef EST32_H
#define EST32_H
/* Copied verbatim (types and functions) from
 * the offline estimator study (2026-11-15), its 32-bit count-domain
 * constant-acceleration tracker with bit-exact parity vs exact_ref32.py.
 * Research prototype; here used for D-term velocity and logged acceleration. */
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint32_t k0, k1, k2;   /* Q32 gains, < 2^32 */
    int32_t deadzone_q20;  /* 0 = linear */
    int32_t resync_q20;    /* |innovation| above this resyncs; must be < 2^27 */
    uint8_t lead_ticks;    /* 0 or 1 */
} est32_config;

typedef struct {
    int64_t y_last_q16;
    int32_t e, dq, ddq;    /* Q20 counts */
    uint8_t primed;
    uint16_t resyncs;
} est32_state;

typedef struct {
    int32_t q_off_q32, v_q16, a_q16, innovation_q20;
    uint8_t resynced;
} est32_output;

bool est32_config_valid(const est32_config *c);
void est32_init(est32_state *s);
bool est32_update(est32_state *s, const est32_config *c, int64_t y_q16, est32_output *out);
/* PD baseline: predicted measurement (Q16 counts) for a tick whose sample was bad.
 * Feeding it back through est32_update gives zero innovation (pure prediction). */
bool est32_predict(const est32_state *s, int64_t *y_q16);
#endif
