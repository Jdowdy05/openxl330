#ifndef GENERAL_REFERENCE_H
#define GENERAL_REFERENCE_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {int32_t q,v,a,start,delta,vscale,ascale;uint32_t elapsed,duration;bool active,linear;} general_reference;
void general_reference_init(general_reference *,int32_t q_q16);
bool general_reference_start(general_reference *,int32_t target_q16,uint32_t duration_ticks);
bool general_reference_tick(general_reference *);
/* Constant velocity from current q to target over ticks (v = delta/T, a = 0). */
bool general_reference_start_linear(general_reference *,int32_t target_q16,uint32_t duration_ticks);
#endif
