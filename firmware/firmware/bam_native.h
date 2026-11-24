#ifndef BAM_NATIVE_H
#define BAM_NATIVE_H
#include <stdbool.h>
#include <stdint.h>
/* Offline reference mathematics; no hardware authority. Signed Q16 radians,
 * rad/s, rad/s²; center uses the same unmodified encoder coordinate.
 * enable: 0 terminal/no command, 1 enabled reference, 2 explicit OFF request.
 * junction: acceleration is undefined at t=3 up_down; reports right limit.
 * tick=12000 returns the analytic endpoint and terminal/no command metadata.
 * Distinct aligned output; rejection leaves it unchanged. */
typedef struct {int32_t q,v,a;uint8_t active,enable,terminal,junction;} bam_output;
bool bam_eval(uint8_t kind,uint32_t tick,int32_t center_q16,bam_output *out);
#endif
