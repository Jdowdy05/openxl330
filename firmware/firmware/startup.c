#include <stdint.h>
#include "bridge.h"
#include "platform.h"
#include "hw_io.h"
#ifndef APP_LAB
#define APP_LAB 0
#endif
/* fast10 (review S-3): the boot stack paint and its watermark feed only the lab read 896: LAB builds only (flash). */
#if APP_LAB
#include "observability.h"
/* Leaf: paint only below its live MSP and leave 32 bytes untouched. */
__attribute__((naked,noinline)) static uint32_t stack_paint_boot(void)
{
    __asm__ volatile(
      ".syntax unified\n" "mrs r1, msp\n" "mov r2, sp\n" "cmp r1, r2\n" "bne 2f\n"
      "ldr r0, =0x20001820\n" "cmp r1, r0\n" "bls 2f\n"
      "ldr r2, =0x20002000\n" "cmp r1, r2\n" "bhi 2f\n"
      "subs r1, #32\n" "movs r2, #3\n" "bics r1, r2\n"
      "ldr r0, =0x20001800\n" "cmp r1, r0\n" "bls 2f\n"
      "mov r2, r0\n" "ldr r3, =0xa5a5a5a5\n"
      "1: str r3, [r2]\n" "adds r2, #4\n" "cmp r2, r1\n" "blo 1b\n"
      "subs r1, r1, r0\n" "lsrs r0, r1, #2\n" "bx lr\n"
      "2: movs r0, #0\n" "bx lr\n");
}
#endif
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
void Reset_Handler(void) __attribute__((noreturn));
void Default_Handler(void) __attribute__((naked,noreturn));
void NMI_Handler(void);   /* platform.c (fast17) */
void PendSV_Handler(void);
void DMA1_Channel1_IRQHandler(void);

/* Cortex16 core entries plus32 external vectors. USART1 IRQ27 -> vector43;
 * TIM16 IRQ21 -> vector37. No SysTick interrupt: its counter is polled. */
__attribute__((used,section(".vectors"),aligned(256)))
const uintptr_t vectors[48] = {
    (uintptr_t)&_estack, (uintptr_t)Reset_Handler,
    (uintptr_t)NMI_Handler, (uintptr_t)Default_Handler,   /* fast17: NMI -> ECC check of the ID log page, else default */
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)PendSV_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)DMA1_Channel1_IRQHandler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)TIM16_IRQHandler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)USART1_IRQHandler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler, (uintptr_t)Default_Handler
};

void Reset_Handler(void)
{
    (void)platform_irq_save();
    hw_write32((uintptr_t)0xe000ed08u, 0x08004000u);
    __asm__ volatile("dsb\n\tisb" ::: "memory");
    bridge_early_off(); /* Before data/BSS; hw_read/write are stateless MMIO. */
    uint32_t *source = &_sidata;
    for (uint32_t *destination = &_sdata; destination < &_edata;) *destination++ = *source++;
    for (uint32_t *destination = &_sbss; destination < &_ebss;) *destination++ = 0u;
#if APP_LAB
    uint32_t words=stack_paint_boot();
    obs_watermark_init(&observation_stack,(uint32_t *)(uintptr_t)0x20001800u,(uint16_t)words);
#endif
    firmware_main();
}
void firmware_default_fault(uint32_t *stack, uint32_t exception) __attribute__((noreturn));
void firmware_default_fault(uint32_t *stack, uint32_t exception)
{
    bridge_disable();
    uintptr_t at=(uintptr_t)stack;
    uint32_t pc=at>=0x20000000u && at<=0x20001fe0u?stack[6]:0u;
    platform_debug_mark(0xd000u | exception, pc);
    /* fast11 (owner: "a hung servo should reboot back into its firmware"): once healthy, reboot this application, the
     * exception number recorded (read 1112); before that, or in a quick crash loop, the loader as before. */
    platform_hang_reset(PLATFORM_RESET_FAULT | ((exception & 0xffu) << 8));
}
void Default_Handler(void)
{
    __asm__ volatile("mrs r0, msp\n\tmrs r1, ipsr\n\tb firmware_default_fault");
}
