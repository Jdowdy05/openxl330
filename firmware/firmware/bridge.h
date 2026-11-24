#ifndef XL330_BRIDGE_H
#define XL330_BRIDGE_H
#include <stdbool.h>
#include <stdint.h>

#define BRIDGE_MAX_HIGH_TICKS 480u
#define BRIDGE_FAULT_BAD_REQUEST (1u << 0)
#define BRIDGE_FAULT_NOT_INITIALIZED (1u << 1)
#define BRIDGE_FAULT_BREAK (1u << 2)
#define BRIDGE_FAULT_BREAK2 (1u << 3)
#define BRIDGE_FAULT_READBACK (1u << 4)

/* No initialized RAM dependency; valid before .bss is cleared. */
void bridge_early_off(void);
/* Requires nominal PLL64/AHB1/APB1. Leaves torque disabled. Clears SW faults. */
bool bridge_init(void);
/* Gate low first; zero compare and disable both timer output enables. */
void bridge_disable(void);
/* Raw high ticks, no stock +15 compensation; cap 0..480, |ticks|<=cap.
 * Invalid input/hardware faults latch until bridge_init(). A live sign change
 * returns false, blanks output, and sets no fault; call again on the next
 * control tick. Caller provides that tick interval and all motion permission.
 * Zero is always electrically disabled, including when this returns false.
 * Single caller / serialized control context is required. */
void bridge_set_fast_reversal(bool on);
bool bridge_set(int16_t signed_high_ticks, uint16_t absolute_cap);
int16_t bridge_applied(void);
/* Also polls break flags and fails closed if a break has appeared. */
uint32_t bridge_faults(void);
#endif
