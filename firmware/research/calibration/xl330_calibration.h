#ifndef XL330_CALIBRATION_H
#define XL330_CALIBRATION_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Read-only, allocation-free C11. NVM buffer maps 0800E000..0800F000.
 * A false return means the caller must inhibit torque. No flash-write APIs.
 * "reverse" is exactly stock Drive Mode bit 0: false is stock normal mode.
 */
typedef struct {
    uint16_t knots[65];
    uint16_t origin;
    int16_t timing_trim;
    int16_t auxiliary_e808; /* Preserved metadata; not a position correction. */
    uint8_t segments;
    bool ready;
} xl330_calibration;

typedef struct {
    int64_t uncorrected;
    int32_t homing_offset;
    uint16_t previous_phase;
    uint16_t maximum_step; /* Must be 1..2047 counts/sample. */
    bool reverse;
    bool ready;
} xl330_position_tracker;

bool xl330_calibration_load(xl330_calibration *cal, const uint8_t *nvm, size_t size);
/* Piecewise-linear integer mapping over the calibration knots; reproduces the positions the stock firmware reports,
 * extended to non-overflowing int64_t. */
bool xl330_calibration_map(const xl330_calibration *cal, int64_t uncorrected,
                           bool reverse, int64_t *position);
/* Stock first-sample convention. Output is one turn in [0,4095]. */
bool xl330_calibration_raw(const xl330_calibration *cal, uint16_t raw12,
                           bool reverse, uint16_t *position);
/* Software turn tracking, zero turns known at startup. Reinitialize on config
 * changes. Any invalid sample/overflow latches ready=false until begin().
 * Bound maximum_step from actual polling rate and possible shaft velocity.
 * An outage or a full missed turn is unobservable from this sensor alone.
 */
bool xl330_tracker_begin(xl330_position_tracker *tracker,
                         const xl330_calibration *cal, uint16_t raw12,
                         bool reverse, int32_t homing_offset,
                         uint16_t maximum_step, int64_t *position);
bool xl330_tracker_update(xl330_position_tracker *tracker,
                          const xl330_calibration *cal, uint16_t raw12,
                          int64_t *position);
/* Optional signed Q16 residue preserves every stock integer result. The precise
 * coordinate is position*65536+residue (normal residue<=0, reverse>=0).
 * Failures preserve both outputs; tracker failure still latches ready=false. */
bool xl330_calibration_map_fraction(const xl330_calibration *,int64_t,bool,int64_t *,int32_t *);
bool xl330_tracker_begin_fraction(xl330_position_tracker *,const xl330_calibration *,uint16_t,bool,int32_t,uint16_t,int64_t *,int32_t *);
bool xl330_tracker_update_fraction(xl330_position_tracker *,const xl330_calibration *,uint16_t,int64_t *,int32_t *);
#endif
