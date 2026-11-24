#ifndef XL330_PLATFORM_H
#define XL330_PLATFORM_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t previous_cycles, microseconds;
    uint8_t remainder_cycles;
} platform_time_state;
/* Pure serialized accumulator for the 24-bit downcounter at nominal64MHz.
 * Caller must sample at least once per complete 24-bit wrap (~262.144ms).
 * hw_micros serializes this operation with saved/restored PRIMASK. */
void platform_time_reset(platform_time_state *, uint32_t counter);
uint32_t platform_time_advance(platform_time_state *, uint32_t counter);
uint32_t platform_irq_save(void);
void platform_irq_restore(uint32_t saved);
void platform_irq_enable(void);
/* Maximum outer PRIMASK interval, rounded up at nominal64MHz. Nested saved1
 * sections keep the outer start. Sampled after CPSID and before MSR restore:
 * pre-disable IRQs are excluded; a few entry/restore instructions are unmeasured.
 * Intervals must be shorter than the262.144ms SysTick wrap. Zero before clock. */
uint32_t platform_irq_max_us(void);
void platform_dma_read(uint8_t out[32]);
bool platform_arm_loader(void);
bool platform_disarm_loader(void); /* FW231: clear the boot-time loader request (BDRST pulse) */
void platform_enter_loader(void) __attribute__((noreturn));
enum {
    PLATFORM_DEBUG_BYTES = 56,
    PLATFORM_STAGE_INIT_ENTRY = 0x100,
    PLATFORM_STAGE_HSI_READY = 0x101,
    PLATFORM_STAGE_HSI_SWITCH = 0x102,
    PLATFORM_STAGE_PLL_STOP = 0x103,
    PLATFORM_STAGE_VOLTAGE_READY = 0x104,
    PLATFORM_STAGE_FLASH_LATENCY = 0x105,
    PLATFORM_STAGE_PLL_CONFIG = 0x106,
    PLATFORM_STAGE_PLL_READY = 0x107,
    PLATFORM_STAGE_PLL_SWITCH = 0x108,
    PLATFORM_STAGE_SYSTICK_READY = 0x109,
    PLATFORM_STAGE_UART_DISABLE = 0x10a,
    PLATFORM_STAGE_UART_ENABLE = 0x10b,
    PLATFORM_STAGE_INIT_READY = 0x10c,
    PLATFORM_STAGE_UART_FIFO = 0x10d,
    PLATFORM_STAGE_FLASH_PREFETCH = 0x10e
};
/* Marks update live RAM and immediately commit the retained SRAM record.
 * Failed init checks replace detail with the actual register.
 * debug_read emits14 little-endian u32 words: stage/detail/entryRCC_CSR,
 * RCC_CR/CFGR/PLLCFGR,PWR_CR1/SR2,FLASH_ACR,UART_ISR,UARTfaults,
 * I2C_ISR,ADC_ISR,maxPRIMASK_us(0 before clock). Caller serializes capture.
 * Every mark and orderly platform_enter_loader write best-effort reset evidence
 * at200017C0: magicD1A6CAFE,stage,detail,entryRCC_CSR (magic committed last).
 * BSS must exclude200017C0..17FF; reporter copies before its initialization.
 * Bootloader SRAM clobber/power loss can invalidate this diagnostic record.
 * BKP0/1 keep loader magic; no other backup registers are written. */
/* FW231: the boot/fault stage marks (retained trace at 0x200017C0) have no reader in FW231 (no read exposes
 * platform_debug_read), so they are LAB-only like the other lab reads (build.sh: -DPLATFORM_DEBUG_MARKS=$LAB).
 * Native tests keep them (default 1). Disabled, a mark still evaluates its arguments and writes nothing. */
#ifndef PLATFORM_DEBUG_MARKS
#define PLATFORM_DEBUG_MARKS 1
#endif
#if PLATFORM_DEBUG_MARKS
void platform_debug_mark(uint32_t stage, uint32_t detail);
#else
#define platform_debug_mark(stage, detail) ((void)(stage), (void)(detail))
#endif
void platform_debug_read(uint8_t out[PLATFORM_DEBUG_BYTES]);
bool platform_init(void);
#define PLATFORM_LED_PIN (1u << 4) /* PA4, active-low */
void platform_led(bool on);
/* fast17: the ID log page only (0x0800D800..0x0800DFFF); see platform.c. OFF only (the CPU stalls meanwhile). */
bool platform_nmi_idlog_ecc(void);   /* NMI: true when it was an ECC double error in the log page (cleared, latched) */
/* platform_flash_idlog, platform_idlog_ecc_take, platform_idlog_page: declared in idlog.h */
void platform_watchdog_reload(void); /* refresh the IWDG (no window); main loop and thread-mode waits only, never an ISR */
/* fast12 (owner: "turn it on for watchdog"): start the IWDG at boot. LSI 32 kHz, /64, 1000 counts = 2.0 s (LSI tolerance:
 * about 1.9-2.2 s). Only a reset stops it. */
enum { PLATFORM_IWDG_PR = 4u, PLATFORM_IWDG_RLR = 999u, PLATFORM_IWDG_MS = 2000u };
void platform_watchdog_start(void);
bool platform_uart_pop(uint8_t *byte);
uint32_t platform_uart_read(uint8_t *out, uint32_t max);   /* FW231: batch of up to max (<= 64) bytes */
/* USART1 TX uses the8-entry FIFO with half-empty threshold IRQ batches;
 * physical TC owns DE release and RX handoff. Accepts1..640 bytes. */
bool platform_uart_transmit(const uint8_t *data, uint16_t size);
/* FW231: same, but only if no RX byte is unread and no reception is in progress when the line is claimed;
 * otherwise returns false with *deferred = true and touches nothing. */
/* FW231 fast start (Fast Sync Read + chained replies): receiver timeout -> platform_uart_rto() in the
 * priority-0 USART IRQ (weak no-op unless main.c defines it); ring peek for its watcher; IRQ-context TX start. */
void platform_uart_rto(void);
uint32_t platform_uart_rx_written(void);
const volatile uint8_t *platform_uart_rx_ring(void);   /* PLATFORM_RX_RING (1024) bytes; index & 1023 */
enum { PLATFORM_RX_RING = 1024 };
uint32_t platform_uart_rx_next(void);
void platform_uart_set_rto(uint32_t bits);
bool platform_uart_tx_irq(const uint8_t *data, uint16_t size);
bool platform_uart_transmit_if_quiet(const uint8_t *data, uint16_t size, bool *deferred);
bool platform_uart_tx_idle(void);
bool platform_uart_set_baud(uint32_t bps); /* FW231: 1000000, 2000000 or 3000000 only */
uint32_t platform_uart_pending_faults(void); /* raw PE/FE/NE/ORE */
uint32_t platform_uart_faults(void); /* sticky DIAGNOSTIC: overflow1, hardware2, TX4, DMA TE 0x10; raw PE/FE/NE/ORE in bits8..11 */
uint32_t platform_uart_fatal(void);  /* FW231: only the DMA transfer error (0x10) is fatal */
uint32_t platform_uart_rx_loss(void);   /* FW231: changes whenever received bytes were lost or discarded */
void platform_uart_rx_discard(void);    /* FW231: drop every unread RX byte (after a baud switch) */
void platform_irq_max_reset(void);      /* FW231: restart the longest-PRIMASK and RX-backlog measurements */
uint32_t platform_uart_rx_backlog_max(void);
uint32_t platform_irq_max_site(void);   /* FW231: caller (return address) of the longest masked section */ /* FW231 diag: deepest unread RX backlog (bytes) seen by pop */
enum { PLATFORM_UART_COUNTS = 6 };   /* PE, FE, NE, ORE, RX overflow, TX abort */
void platform_uart_counts(uint32_t out[PLATFORM_UART_COUNTS]);
void platform_reset_app(void) __attribute__((noreturn)); /* FW231: 0x08 Reboot into this application */
/* FW231 fast11 (owner 2026-11-20: "a hung servo should reboot back into its firmware"). Read 1112 (12 B):
 *   word 0: cause of the last reset (bits 0-7) | detail (8-15: the exception number of a fault) | RCC_CSR reset
 *           flags 24..31 (16-23: OBL<<1, PIN<<2, PWR<<3, SFT<<4, IWDG<<5, WWDG<<6, LPWR<<7) | record found (24) |
 *           loader request cleared, i.e. hangs now reboot this application (25) | fast15: the IWDG did not latch its
 *           2 s setting at boot, so it runs at its reset default, about 0.5 s (26, PLATFORM_RESET_IWDG_UPDATE_TIMEOUT)
 *   word 1: hang reboots since power-on (stall, fault, DMA, watchdog); word 2: quick hangs in a row (< 60 s up) */
enum {
    PLATFORM_RESET_NONE = 0, PLATFORM_RESET_STALL = 1, PLATFORM_RESET_FAULT = 2, PLATFORM_RESET_DMA = 3,
    PLATFORM_RESET_WATCHDOG = 4, PLATFORM_RESET_HOST = 5, PLATFORM_RESET_SOFTWARE = 6, PLATFORM_RESET_PIN = 7,
    PLATFORM_RESET_POWER = 8,
    PLATFORM_FAST_HANG_LIMIT = 5   /* this many hangs in a row, each within 60 s of boot: the loader (boot-loop guard) */
};
#define PLATFORM_RESET_IWDG_UPDATE_TIMEOUT (1u << 26)   /* read 1112 word 0 (fast15) */
void platform_boot_capture(void);            /* first thing at boot (may enter the loader: boot-loop guard) */
void platform_app_healthy(void);             /* ran healthy: clear the boot-time loader request */
void platform_app_long_run(void);            /* up 60 s: a later hang starts no quick-hang streak */
void platform_reset_info(uint32_t out[3]);
void platform_hang_reset(uint32_t reason) __attribute__((noreturn));   /* reason | detail << 8 */
/* Starts TIM16 at APP_PERIOD_US (initial2kHz); returns time immediately before CEN is set. */
uint32_t platform_control_timer_start(void);
void platform_control_timer_ack(void);
bool platform_control_timer_pending(void);
void USART1_IRQHandler(void);
void TIM16_IRQHandler(void); /* main.c control/sensing integration */
void firmware_main(void) __attribute__((noreturn));
#endif
