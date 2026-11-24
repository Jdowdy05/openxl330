#include "xl330_calibration.h"
#include <limits.h>

static uint16_t read16(const uint8_t *p, size_t offset) {
    return (uint16_t)((uint16_t)p[offset] | (uint16_t)((uint16_t)p[offset+1] << 8));
}
static int16_t signed16(uint16_t x) {
    return (int16_t)(x < 32768u ? (int32_t)x : (int32_t)x - 65536);
}
static uint16_t phase_of(const xl330_calibration *c, uint16_t raw) {
    return (uint16_t)(((uint32_t)raw + 4096u - c->origin) & 4095u);
}
static bool add_checked(int64_t a, int64_t b, int64_t *result) {
    if ((b > 0 && a > INT64_MAX-b) || (b < 0 && a < INT64_MIN-b)) return false;
    *result=a+b;
    return true;
}

bool xl330_calibration_load(xl330_calibration *c, const uint8_t *p, size_t size) {
    if (!c) return false;
    *c=(xl330_calibration){0};
    if (!p || size < 4096) return false;
    const uint16_t alternative_origin=read16(p,0x008), dense_origin=read16(p,0x810);
    size_t table;
    if (alternative_origin < 4095u) {
        c->segments=4;
        c->origin=alternative_origin;
        if (dense_origin < 4095u) c->origin=(uint16_t)((c->origin+dense_origin)&4095u);
        table=0x008;
    } else {
        if (dense_origin > 4095u || (read16(p,0x800)&255u) != 2u) return false;
        c->segments=64;
        c->origin=dense_origin;
        table=0x810;
    }
    c->knots[c->segments]=4096;
    for (unsigned i=1;i<c->segments;++i) {
        c->knots[i]=read16(p,table+8u*(c->segments-i));
        if (c->knots[i] <= c->knots[i-1] || c->knots[i] >= 4096u) return false;
    }
    c->auxiliary_e808=signed16(read16(p,0x808));
    const int16_t trim=signed16(read16(p,0xA10));
    if (read16(p,0xA18)==0 && trim>=-600 && trim<=600) c->timing_trim=trim;
    c->ready=true;
    return true;
}

bool xl330_calibration_map(const xl330_calibration *c, int64_t n,
                           bool reverse, int64_t *position) {
    if (!c || !c->ready || !position) return false;
    int64_t turns=n/4096, phase=n%4096;
    if (phase<0) { --turns; phase+=4096; }
    const uint32_t x=(uint32_t)(reverse ? phase : 4095-phase);
    unsigned lower=0, upper=c->segments;
    while (upper-lower>1) {
        const unsigned middle=(upper+lower)/2;
        if (x>c->knots[middle]) lower=middle; else upper=middle;
    }
    const uint32_t step=4096u/c->segments;
    const uint32_t width=(uint32_t)c->knots[lower+1]-c->knots[lower];
    if (!width) return false;
    uint32_t corrected=lower*step+(x-c->knots[lower])*step/width;
    if (corrected>4095u) corrected=4095;
    *position=turns*4096+(reverse ? corrected : 4095u-corrected);
    return true;
}

bool xl330_calibration_raw(const xl330_calibration *c, uint16_t raw,
                           bool reverse, uint16_t *position) {
    if (!c || !c->ready || !position || raw>4095u) return false;
    const uint16_t phase=phase_of(c,raw);
    const int64_t n=reverse ? phase : (4096u-phase)&4095u;
    int64_t y;
    if (!xl330_calibration_map(c,n,reverse,&y)) return false;
    *position=(uint16_t)y;
    return true;
}

bool xl330_tracker_begin(xl330_position_tracker *t, const xl330_calibration *c,
                         uint16_t raw, bool reverse, int32_t home,
                         uint16_t maximum_step, int64_t *position) {
    if (!t) return false;
    *t=(xl330_position_tracker){0};
    if (!c || !c->ready || !position || raw>4095u || maximum_step==0 || maximum_step>=2048u) return false;
    const uint16_t phase=phase_of(c,raw);
    const int64_t n=reverse ? phase : (4096u-phase)&4095u;
    int64_t y;
    if (!xl330_calibration_map(c,n,reverse,&y) || !add_checked(y,home,&y)) return false;
    t->uncorrected=n;
    t->homing_offset=home;
    t->previous_phase=phase;
    t->maximum_step=maximum_step;
    t->reverse=reverse;
    t->ready=true;
    *position=y;
    return true;
}

bool xl330_tracker_update(xl330_position_tracker *t, const xl330_calibration *c,
                          uint16_t raw, int64_t *position) {
    if (!t) return false;
    if (!t->ready || !c || !c->ready || !position || raw>4095u) { t->ready=false; return false; }
    const uint16_t phase=phase_of(c,raw);
    int32_t delta=t->reverse ? (int32_t)phase-t->previous_phase : (int32_t)t->previous_phase-phase;
    if (delta>2048) delta-=4096;
    if (delta< -2048) delta+=4096;
    int64_t n,y;
    if (delta>(int32_t)t->maximum_step || delta< -(int32_t)t->maximum_step ||
        !add_checked(t->uncorrected,delta,&n) ||
        !xl330_calibration_map(c,n,t->reverse,&y) ||
        !add_checked(y,t->homing_offset,&y)) { t->ready=false; return false; }
    t->uncorrected=n;
    t->previous_phase=phase;
    *position=y;
    return true;
}
