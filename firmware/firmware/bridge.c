/* Own bridge driver for the board's H-bridge wiring:
 * PA11 enable; PB2 direction; TIM1_CCR1 PWM on PA8 (AF2); TIM1 mode/break setup.
 * This code has not been electrically validated. No brake/coast assertion.
 * Own raw-tick interface intentionally omits the stock firmware's +15 duty offset.
 */
#include "bridge.h"
#include "hw_io.h"

#define GPIOA ((uintptr_t)0x50000000u)
#define GPIOB ((uintptr_t)0x50000400u)
#define RCC ((uintptr_t)0x40021000u)
#define TIM1 ((uintptr_t)0x40012c00u)
#define MODER 0x00u
#define OTYPER 0x04u
#define OSPEEDR 0x08u
#define PUPDR 0x0cu
#define ODR 0x14u
#define BSRR 0x18u
#define AFRH 0x24u
#define BRR 0x28u
#define CR1 0x00u
#define CR2 0x04u
#define SMCR 0x08u
#define DIER 0x0cu
#define SR 0x10u
#define EGR 0x14u
#define CCMR1 0x18u
#define CCER 0x20u
#define PSC 0x28u
#define ARR 0x2cu
#define RCR 0x30u
#define CCR1 0x34u
#define BDTR 0x44u
#define AF1 0x60u
#define AF2 0x64u
#define GATE_PIN 0x0800u
#define PWM_PIN 0x0100u
#define DIRECTION_PIN 0x0004u
#define MOE 0x8000u
#define AOE 0x4000u
#define CEN 1u
#define TIM1_CLOCK 0x0800u
#define STOCK_BREAK_MANUAL 0x03003000u

static bool initialized;
static bool fast_reversal = true;
void bridge_set_fast_reversal(bool on) { fast_reversal = on; }
static int16_t applied;
static uint32_t faults;

static void modify(uintptr_t address, uint32_t clear_mask, uint32_t set_mask)
{
    hw_write32(address, (hw_read32(address) & ~clear_mask) | set_mask);
}

static void gpio_mode(uintptr_t port, unsigned pin, uint32_t mode)
{
    unsigned shift = pin * 2u;
    uint32_t mask = (uint32_t)1u << pin;
    modify(port + OTYPER, mask, 0u);              /* push-pull */
    modify(port + OSPEEDR, (uint32_t)3u << shift, (uint32_t)2u << shift);
    modify(port + PUPDR, (uint32_t)3u << shift, 0u);
    modify(port + MODER, (uint32_t)3u << shift, mode << shift);
}

/* No initialized RAM reads or writes in this helper or its callees. */
static void outputs_off(void)
{
    hw_write32(GPIOA + BRR, GATE_PIN);             /* first motor action */
    modify(TIM1 + BDTR, MOE | AOE, 0u);
    hw_write32(TIM1 + CCR1, 0u);
    hw_write32(TIM1 + CCER, 0u);
    modify(TIM1 + CR1, CEN, 0u);
    hw_write32(TIM1 + EGR, 1u);                   /* latch zero; AOE is off */
}

void bridge_early_off(void)
{
    modify(RCC + 0x34u, 0u, 3u);
    (void)hw_read32(RCC + 0x34u);                 /* clock-enable readback */
    hw_write32(GPIOA + BRR, GATE_PIN);             /* latch before GPIO mode */
    gpio_mode(GPIOA, 11u, 1u);
    modify(RCC + 0x40u, 0u, TIM1_CLOCK);
    (void)hw_read32(RCC + 0x40u);
    outputs_off();
    hw_write32(GPIOA + BRR, PWM_PIN);
    gpio_mode(GPIOA, 8u, 1u);                    /* driven low pre-timer setup */
}

void bridge_disable(void)
{
    outputs_off();
    applied = 0;
}

static uint32_t poll_breaks(void)
{
    uint32_t status = hw_read32(TIM1 + SR);
    if ((status & 0x80u) != 0u) faults |= BRIDGE_FAULT_BREAK;
    if ((status & 0x100u) != 0u) faults |= BRIDGE_FAULT_BREAK2;
    return faults;
}

static bool fail(uint32_t reason)
{
    faults |= reason;
    bridge_disable();
    return false;
}

static bool timer_configuration_matches(void)
{
    return hw_read32(TIM1 + PSC) == 3u &&
           hw_read32(TIM1 + ARR) == 479u &&
           hw_read32(TIM1 + CCMR1) == 0x68u &&
           (hw_read32(TIM1 + BDTR) & ~MOE) == STOCK_BREAK_MANUAL &&
           hw_read32(TIM1 + AF1) == 1u &&
           hw_read32(TIM1 + AF2) == 1u &&
           ((hw_read32(GPIOA + MODER) >> 16u) & 3u) == 2u &&
           (hw_read32(GPIOA + AFRH) & 15u) == 2u;
}

bool bridge_init(void)
{
    bridge_early_off();
    initialized = false;
    applied = 0;
    faults = 0u;
    /* Reset only TIM1, preserving other RCC reset/enable bits. */
    modify(RCC + 0x30u, 0u, TIM1_CLOCK);
    modify(RCC + 0x30u, TIM1_CLOCK, 0u);
    hw_write32(GPIOB + BRR, DIRECTION_PIN);
    gpio_mode(GPIOB, 2u, 1u);
    hw_write32(TIM1 + CR1, 0u);
    hw_write32(TIM1 + DIER, 0u);
    hw_write32(TIM1 + CCER, 0u);
    hw_write32(TIM1 + BDTR, 0u);
    hw_write32(TIM1 + PSC, 3u);
    hw_write32(TIM1 + ARR, 479u);                  /* 64MHz / (3+1) / 480 = 33.333kHz */
    hw_write32(TIM1 + RCR, 0u);
    hw_write32(TIM1 + CCR1, 0u);
    hw_write32(TIM1 + CCMR1, 0x68u);               /* PWM1, preload, active high */
    hw_write32(TIM1 + CR2, 0u);
    hw_write32(TIM1 + SMCR, 0u);
    /* RM0454 AF1/AF2 reset routing: external BKIN/BKIN2 enabled, no inversion.
     * Physical break wiring is unverified; these are stock/reset semantics. */
    hw_write32(TIM1 + AF1, 1u);
    hw_write32(TIM1 + AF2, 1u);
    hw_write32(TIM1 + BDTR, STOCK_BREAK_MANUAL);    /* AOE=0, MOE=0, DTG=0 */
    hw_write32(TIM1 + EGR, 1u);
    hw_write32(TIM1 + SR, 0u);
    modify(GPIOA + AFRH, 15u, 2u);
    gpio_mode(GPIOA, 8u, 2u);
    hw_write32(TIM1 + CR1, 0x80u);                 /* preload, stopped */
    if (!timer_configuration_matches() ||
        hw_read32(TIM1 + CCR1) != 0u || hw_read32(TIM1 + CCER) != 0u ||
        (hw_read32(TIM1 + BDTR) & (MOE | AOE)) != 0u ||
        (hw_read32(GPIOA + ODR) & GATE_PIN) != 0u) {
        return fail(BRIDGE_FAULT_READBACK);
    }
    if (poll_breaks() != 0u) return fail(0u);
    initialized = true;
    return true;
}

bool bridge_set(int16_t signed_high_ticks, uint16_t absolute_cap)
{
    /* Promotion before negation makes INT16_MIN well-defined. */
    int32_t wide = signed_high_ticks;
    uint32_t magnitude = (uint32_t)(wide < 0 ? -wide : wide);
    if (absolute_cap > BRIDGE_MAX_HIGH_TICKS ||
        magnitude > BRIDGE_MAX_HIGH_TICKS || magnitude > absolute_cap) {
        return fail(BRIDGE_FAULT_BAD_REQUEST);
    }
    if (signed_high_ticks == 0) bridge_disable();
    if (!initialized) return fail(BRIDGE_FAULT_NOT_INITIALIZED);
    if (poll_breaks() != 0u) return fail(0u);
    if (signed_high_ticks == 0) return true;
    bool reverse = applied != 0 && ((applied < 0) != (signed_high_ticks < 0));
    if (reverse && !fast_reversal) {
        /* Legacy blank: caller invokes again on a later control tick; not a fault. */
        bridge_disable();
        return false;
    }
    if (!timer_configuration_matches()) return fail(BRIDGE_FAULT_READBACK);
    if (reverse) {
        /* PD baseline fast reversal (2026-11-16): drive the output low NOW
         * (CCR 0 + UG transfers the preload and restarts the period), flip the
         * PHASE/direction pin with the gate held, then load the new duty, which
         * starts at the next PWM period (<=30 us). No 500 us dead tick.
         * Electrically unvalidated: relies on the driver handling PHASE changes. */
        hw_write32(TIM1 + CCR1, 0u);
        hw_write32(TIM1 + EGR, 1u);
        if (signed_high_ticks < 0) hw_write32(GPIOB + BSRR, DIRECTION_PIN);
        else hw_write32(GPIOB + BRR, DIRECTION_PIN);
        hw_write32(TIM1 + CCR1, magnitude);
    } else if (applied != 0) {
        /* Same-sign duty updates use timer preload, never change direction. */
        hw_write32(TIM1 + CCR1, magnitude);
    } else {
        hw_write32(GPIOA + BRR, GATE_PIN);
        modify(TIM1 + BDTR, MOE | AOE, 0u);
        if (signed_high_ticks < 0) hw_write32(GPIOB + BSRR, DIRECTION_PIN);
        else hw_write32(GPIOB + BRR, DIRECTION_PIN);
        hw_write32(TIM1 + CCR1, magnitude);
        hw_write32(TIM1 + CCER, 1u);
        modify(TIM1 + CR1, 0u, CEN);
        hw_write32(TIM1 + EGR, 1u);
        if (poll_breaks() != 0u) return fail(0u);
        modify(TIM1 + BDTR, AOE, MOE);             /* manual only */
    }
    if (hw_read32(TIM1 + CCR1) != magnitude ||
        (hw_read32(TIM1 + BDTR) & (MOE | AOE)) != MOE ||
        hw_read32(TIM1 + CCER) != 1u || (hw_read32(TIM1 + CR1) & CEN) == 0u ||
        (((hw_read32(GPIOB + ODR) & DIRECTION_PIN) != 0u) != (signed_high_ticks < 0))) {
        return fail(BRIDGE_FAULT_READBACK);
    }
    if (poll_breaks() != 0u) return fail(0u);
    if (applied == 0) hw_write32(GPIOA + BSRR, GATE_PIN); /* final enable action */
    if ((hw_read32(GPIOA + ODR) & GATE_PIN) == 0u) return fail(BRIDGE_FAULT_READBACK);
    /* Observe faults that arrived during the final gate write, too. Hardware
     * break remains asynchronous; this cannot promise a future fault-free gap. */
    if (poll_breaks() != 0u) return fail(0u);
    applied = signed_high_ticks;
    return true;
}

int16_t bridge_applied(void)
{
    return applied;
}

uint32_t bridge_faults(void)
{
    if (initialized && poll_breaks() != 0u) bridge_disable();
    return faults;
}
