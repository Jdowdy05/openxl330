#ifndef XL330_HW_IO_H
#define XL330_HW_IO_H
#include <stdint.h>
#include <stddef.h>
/* Platform boundary: production performs volatile MMIO; tests provide a map. */
#if defined(__arm__) && !defined(PLATFORM_UART_TEST)
static inline uint32_t hw_read32(uintptr_t address) { return *(volatile uint32_t *)address; }
static inline void hw_write32(uintptr_t address, uint32_t value) { *(volatile uint32_t *)address = value; }
#define HW_IO_INLINE 1
#else
uint32_t hw_read32(uintptr_t address);
void hw_write32(uintptr_t address, uint32_t value);
#endif
/* Monotonic microseconds modulo2^32, also valid before interrupts are enabled. */
uint32_t hw_micros(void);
void hw_relax(void);
#endif
