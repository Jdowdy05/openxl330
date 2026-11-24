/* fast17 (v1.1 Stage 2): the NMI vector. An ECC double error in the ID log page (a record torn by a power cut) is cleared
 * and latched by platform_nmi_idlog_ecc() (platform.c), and the NMI returns; any other NMI goes on to Default_Handler
 * with the stack and LR (EXC_RETURN) exactly as on entry, the fault path of fast16. Its own file so that
 * tests/golden_qemu/nmi_test.c runs this exact object on qemu with a stub check. */
#include <stdbool.h>
#if defined(__arm__)
void NMI_Handler(void) __attribute__((naked));
void NMI_Handler(void)
{
    __asm__ volatile(
        "push {r4, lr}\n\t"
        "bl platform_nmi_idlog_ecc\n\t"
        "pop {r4}\n\t"                 /* Thumb-1 POP lists load in register order: one register each */
        "pop {r1}\n\t"                 /* r1 = EXC_RETURN */
        "cmp r0, #0\n\t"
        "beq 1f\n\t"
        "bx r1\n"                       /* handled: exception return */
        "1:\n\t"
        "mov lr, r1\n\t"
        "ldr r0, =Default_Handler\n\t"
        "bx r0\n\t"                    /* not ours: the default fault path */
        ".ltorg");
}
#endif
