#ifndef DEADLINE_H
#define DEADLINE_H
/* Exact sliding-window deadline-miss budget: window ticks, fault when misses in the
 * last `window` ticks exceed `limit`. Fixed RAM (one bit per tick). */
#include <stdbool.h>
#include <stdint.h>
#define DEADLINE_WINDOW 2000u
#define DEADLINE_MISS_LIMIT 100u
typedef struct {uint8_t bits[DEADLINE_WINDOW/8u];uint16_t index,window,window_max;uint32_t total;} deadline_budget;
/* Account one tick; returns true when the budget is exceeded (caller faults). */
bool deadline_tick(deadline_budget *,bool missed);
/* Compact variant: 20 buckets x 100 ticks (1 s, 50 ms granularity), same 5% limit. */
#define BUCKET_COUNT 20u
#define BUCKET_TICKS 100u
typedef struct {uint8_t bucket[BUCKET_COUNT];uint8_t current;uint8_t ticks;uint16_t window,window_max;uint32_t total;} bucket_budget;
bool bucket_tick(bucket_budget *,bool bad);
#endif
