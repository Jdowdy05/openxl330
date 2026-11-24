#include "observability.h"
#include "platform.h"
#include "idlog.h"

void platform_time_reset(platform_time_state *state, uint32_t counter)
{
    state->previous_cycles = counter & 0x00ffffffu;
    state->microseconds = 0u;
    state->remainder_cycles = 0u;
}

static inline __attribute__((always_inline)) uint32_t platform_time_advance_core(platform_time_state *state, uint32_t counter)
{
    counter &= 0x00ffffffu;
    uint32_t cycles = ((state->previous_cycles - counter) & 0x00ffffffu) + state->remainder_cycles;
    state->previous_cycles = counter;
    state->microseconds += cycles >> 6u;
    state->remainder_cycles = (uint8_t)(cycles & 63u);
    return state->microseconds;
}

uint32_t platform_time_advance(platform_time_state *state, uint32_t counter)
{
    return platform_time_advance_core(state, counter);
}

#ifndef PLATFORM_TIME_TEST
#include "bridge.h"
#include "hw_io.h"
#include "app.h"

#define RCC ((uintptr_t)0x40021000u)
#define GPIOA ((uintptr_t)0x50000000u)
#define GPIOC ((uintptr_t)0x50000800u)
#define UART ((uintptr_t)0x40013800u)
/* PD baseline: application link baud. 3 Mbaud = SYSCLK 64 MHz, OVER8, USARTDIV 43
 * (2.977 Mbaud, -0.78%). 1 Mbaud keeps the qualified HSI16 / BRR 16 setting.
 * The loader is entered through a full system reset, so this never affects it. */
/* Boot is always 1 Mbaud (HSI16, BRR 16: the qualified setting). 3 Mbaud is a
 * runtime switch (platform_uart_set_baud) that main confirms or reverts. */
#define UART_CLKSEL 2u                       /* HSI16 */
#define UART_OVER8 0u
#define UART_BRR_VALUE 16u
#define UART_FAST_CLKSEL 1u                  /* SYSCLK 64 MHz */
#define UART_FAST_OVER8 (1u << 15)
#define UART_FAST_BRR ((43u & 0xfff0u) | ((43u & 0xfu) >> 1))  /* USARTDIV 43 -> 2.977 Mbaud */
#define TIM16 ((uintptr_t)0x40014400u)
#define DMA1 ((uintptr_t)0x40020000u)
#define DMAMUX1 ((uintptr_t)0x40020800u)
#define DMA_EVENT_FLAGS 14u
#define UART_DMAT (1u<<7)
#define UART_DMAR (1u<<6)
#define UART_DDRE (1u<<13)
/* RX DMA: DMA1 channel2 (DMAMUX C1CR request50 USART1_RX), circular into rx_ring. */
#define RXDMA_CCR (DMA1+0x1cu)
#define RXDMA_CNDTR (DMA1+0x20u)
#define RXDMA_CPAR (DMA1+0x24u)
#define RXDMA_CMAR (DMA1+0x28u)
#define RXDMA_TC (1u<<5)
#define RXDMA_HT (1u<<6)
#define RXDMA_TE (1u<<7)
#define RXDMA_FLAGS (RXDMA_TC|RXDMA_HT|RXDMA_TE)
#define RXDMA_CONFIG ((1u<<7)|(1u<<5)) /* MINC CIRC, P->M 8-bit, PL0; no interrupt: HT/TC/TE flags are polled */
#define UART_EIE 1u
#define SYST ((uintptr_t)0xe000e010u)
#define FLASH_REG ((uintptr_t)0x40022000u)
#define PWR ((uintptr_t)0x40007000u)
#define RX_RING_SIZE 1024u
#define RX_HALF (RX_RING_SIZE/2u)
#define UART_RXNE (1u << 5)
#define UART_RE (1u << 2)
#define UART_IDLEIE (1u << 4)
#define UART_RXFNEIE (1u << 5)
#define UART_IDLE (1u << 4)
#define UART_RXFTIE (1u << 28)
#define UART_RXFTCFG_MASK (7u << 25)
#define UART_RXFT_HALF (2u << 25)
#define UART_REACK (1u << 22)
#define UART_RTOF (1u << 11)   /* FW231 fast start: receiver timeout (ISR/ICR bit 11, CR1.RTOIE 26, CR2.RTOEN 23, RTOR) */
#define UART_RTOIE (1u << 26)
#define UART_RTOEN (1u << 23)
#define UART_TXEIE (1u << 7)
#define UART_TCIE (1u << 6)
#define UART_FIFOEN (1u << 29)
#define UART_TXFT (1u << 27)
#define UART_TXFTIE (1u << 23)
#define UART_TXFTCFG_MASK (7u << 29)
#define UART_TXFT_HALF (2u << 29)
#define UART_TX_TIMEOUT_US 200000u
#define WAIT_POLLS 1000000u

static platform_time_state clock_state;
static volatile uint32_t debug_words[14];
static bool clock_initialized;
static uint32_t irq_outer_counter, irq_max_cycles;
/* FW231: where the longest outer masked section was taken (return address into the caller of
 * platform_irq_save; 0 = an inline section such as hw_micros). Read 1064 (52 B). */
static uint32_t irq_outer_site, irq_max_site;
static bool irq_outer_active;
/* rx_boundary: bytes written up to the last accounted HT/TC event (a multiple of
 * RX_HALF; its RX_HALF bit is the DMA's expected half); rx_consumed: bytes popped.
 * Both change only under PRIMASK. The DMA is the sole ring writer. */
static uint32_t rx_boundary, rx_consumed;
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint32_t uart_fault_bits;
static const uint8_t *tx_data;
static uint16_t tx_size;
static volatile uint16_t tx_at;
static volatile bool tx_busy, tx_complete, tx_dma_active, tx_dma_wait_tc;
#if PLATFORM_DEBUG_MARKS
static volatile uint32_t dma_counts[8]={0x414d4447u,1u};
#define DMA_COUNT(k) (dma_counts[k]++)
#else
#define DMA_COUNT(k) ((void)0)
#endif
/* Per-transfer failure latch is reset only when a new TX is claimed. */
static volatile uint8_t transfer_failed;
static void uart_error_set(uint32_t bits) { uart_fault_bits|=bits;transfer_failed=1u; }
#define UART_ERROR(bits) uart_error_set(bits)
/* FW231 (six-servo bus): uart_fault_bits stays a sticky DIAGNOSTIC record. Only a DMA
 * transfer error (0x10: RX DMA disabled by hardware = deaf servo, or a TX DMA bus error)
 * is fatal. Line errors (PE/FE/NE/ORE), RX ring overflow and aborted replies are counted
 * here and never cut the bridge: bus garbage (another servo's 57600 loader traffic, a
 * hot-plug, a glitch) must not stop a motor. Counted where the hardware flag is cleared. */
static volatile uint32_t uart_counts[PLATFORM_UART_COUNTS];
static volatile uint32_t rx_loss_epoch;   /* bumps whenever received bytes were lost (line error flush, overflow) */
static uint32_t rx_backlog_max;           /* deepest unread RX backlog seen by pop (bytes): foreground lag */
static void uart_count_raw(uint32_t raw){for(unsigned bit=0;bit<4u;bit++)if(raw&(1u<<bit))uart_counts[bit]++;rx_loss_epoch++;}

#ifndef PLATFORM_UART_TEST
#ifndef HW_IO_INLINE
uint32_t hw_read32(uintptr_t address) { return *(volatile uint32_t *)address; }
void hw_write32(uintptr_t address, uint32_t value) { *(volatile uint32_t *)address = value; }
#endif
void hw_relax(void) { __asm__ volatile("nop"); }
#endif

#ifdef PLATFORM_UART_TEST
extern uint32_t platform_test_irq_save(void);
extern void platform_test_irq_restore(uint32_t);
extern void platform_test_irq_enable(void);
extern void platform_test_loader(void) __attribute__((noreturn));
extern void platform_test_rx_dma_target(volatile uint8_t *,uint16_t);
#endif

static inline __attribute__((always_inline)) uint32_t platform_irq_save_core(void)
{
    uint32_t saved;
#ifdef PLATFORM_UART_TEST
    saved = platform_test_irq_save();
#else
    __asm__ volatile("mrs %0, primask\n\tcpsid i" : "=r"(saved) :: "memory");
#endif
    if (saved == 0u && clock_initialized) {
        /* Sample AFTER CPSID: pre-disable IRQ time cannot inflate the result.
         * A few instructions before this read and after the end read are
         * outside the measurement, so this remains a slight undercount. */
        irq_outer_counter = hw_read32(SYST + 8u);
        irq_outer_active = true;
        irq_outer_site = 0u;
    }
    return saved;
}

uint32_t platform_irq_save(void)
{
    uint32_t saved = platform_irq_save_core();
    if (saved == 0u) irq_outer_site = (uint32_t)(uintptr_t)__builtin_return_address(0);
    return saved;
}

static inline __attribute__((always_inline)) void platform_irq_restore_core(uint32_t saved)
{
    if (saved == 0u && clock_initialized && irq_outer_active) {
        uint32_t counter = hw_read32(SYST + 8u);
        uint32_t cycles = (irq_outer_counter - counter) & 0x00ffffffu;
        irq_outer_active = false;
        if (cycles > irq_max_cycles) { irq_max_cycles = cycles; irq_max_site = irq_outer_site; }
    }
#ifdef PLATFORM_UART_TEST
    platform_test_irq_restore(saved);
#else
    __asm__ volatile("msr primask, %0" :: "r"(saved) : "memory");
#endif
}

void platform_irq_restore(uint32_t saved)
{
    platform_irq_restore_core(saved);
}

void platform_irq_enable(void)
{
#ifdef PLATFORM_UART_TEST
    platform_test_irq_enable();
#else
    __asm__ volatile("cpsie i" ::: "memory");
#endif
}

uint32_t platform_irq_max_us(void)
{
    return clock_initialized ? (irq_max_cycles + 63u) >> 6u : 0u;
}

uint32_t hw_micros(void)
{
    uint32_t saved = platform_irq_save_core();
    uint32_t now = platform_time_advance_core(&clock_state, hw_read32(SYST + 8u));
    platform_irq_restore_core(saved);
    return now;
}

static void modify(uintptr_t address, uint32_t clear, uint32_t set)
{
    hw_write32(address, (hw_read32(address) & ~clear) | set);
}

/* Caller holds PRIMASK. Individual IFCR event clears avoid ES0486 GIF race.
 * If disable fails, hold DMA in peripheral reset and gate its clock; caller
 * must not release the source buffer and must reset into the loader. */
static bool dma_stop_locked(uint32_t *events)
{
    modify(UART+8u,UART_DMAT,0u);
    if(!(hw_read32(RCC+0x38u)&1u)){if(events)*events=0;return true;}
    uint32_t seen=hw_read32(DMA1)&DMA_EVENT_FLAGS;
    uint32_t ccr=hw_read32(DMA1+8u);hw_write32(DMA1+8u,ccr&~1u);
    for(unsigned i=0;i<32u;i++)if(!(hw_read32(DMA1+8u)&1u)){
        seen|=hw_read32(DMA1)&DMA_EVENT_FLAGS;hw_write32(DMA1+4u,DMA_EVENT_FLAGS);if(events)*events=seen;return true;
    }
    modify(RCC+0x28u,0u,1u);modify(RCC+0x38u,1u,0u);if(events)*events=seen;return false;
}
static bool dma_loader_quiesce(void)
{
    tx_dma_active=false;tx_dma_wait_tc=false;tx_busy=false;
    modify(UART+8u,UART_DMAR,0u); /* RX channel2/DMAMUX C1 are stopped by the DMA1 reset below */
    bool stopped=dma_stop_locked(NULL);
    hw_write32((uintptr_t)0xe000e180u,1u<<9);hw_write32((uintptr_t)0xe000e280u,1u<<9);
    if(hw_read32(RCC+0x38u)&1u)hw_write32(DMAMUX1,0u);
    modify(RCC+0x28u,0u,1u);modify(RCC+0x38u,1u,0u);return stopped;
}
static void debug_commit_retained(uint32_t stage, uint32_t detail, uint32_t entry_csr)
{
    hw_write32((uintptr_t)0x200017c0u, 0u); /* invalidate any old record */
    hw_write32((uintptr_t)0x200017c4u, stage);
    hw_write32((uintptr_t)0x200017c8u, detail);
    hw_write32((uintptr_t)0x200017ccu, entry_csr);
#ifndef PLATFORM_UART_TEST
    __asm__ volatile("dsb" ::: "memory");
#endif
    hw_write32((uintptr_t)0x200017c0u, 0xd1a6cafeu); /* commit magic LAST */
#ifndef PLATFORM_UART_TEST
    __asm__ volatile("dsb" ::: "memory");
#endif
}

#if PLATFORM_DEBUG_MARKS
void platform_debug_mark(uint32_t stage, uint32_t detail)
{
    debug_words[0] = stage;
    debug_words[1] = detail;
    /* Startup/fault marks only; also survives a reset that bypasses our
     * orderly loader path, subject to actual loader SRAM preservation. */
    debug_commit_retained(stage, detail, debug_words[2]);
}
#endif

void platform_debug_read(uint8_t out[PLATFORM_DEBUG_BYTES])
{
    if (!out) return;
    debug_words[3] = hw_read32(RCC);
    debug_words[4] = hw_read32(RCC + 8u);
    debug_words[5] = hw_read32(RCC + 0x0cu);
    debug_words[6] = hw_read32(PWR);
    debug_words[7] = hw_read32(PWR + 0x14u);
    debug_words[8] = hw_read32(FLASH_REG);
    debug_words[9] = hw_read32(UART + 0x1cu);
    debug_words[10] = uart_fault_bits;
    debug_words[11] = hw_read32((uintptr_t)0x40005418u);
    debug_words[12] = hw_read32((uintptr_t)0x40012400u);
    debug_words[13] = platform_irq_max_us();
    for (unsigned i = 0u; i < 14u; ++i) {
        uint32_t value = debug_words[i];
        for (unsigned j = 0u; j < 4u; ++j) out[i * 4u + j] = (uint8_t)(value >> (j * 8u));
    }
}

void platform_watchdog_reload(void)
{
    /* Refresh only while the window register is at its reset value (no window): true for the IWDG this firmware starts
     * (platform_watchdog_start) and for an inherited one without a window. Called from the main loop and its bounded
     * thread-mode waits only, never from an interrupt (fast12): a hung main loop must not be kept alive by the tick. */
    if ((hw_read32((uintptr_t)0x40003010u) & 0xfffu) == 0xfffu)
        hw_write32((uintptr_t)0x40003000u, 0xaaaau);
}

/* fast17 (v1.1 Stage 2): flash erase/program for the ID log page ONLY (firmware/idlog.h, 0x0800D800..0x0800DFFF).
 * Every address is checked against that page here, so no caller can reach the loader, the program pages or the servo's
 * own NVM. RM0454 section 3.3.6/3.3.7: unlock with the two keys, BSY1/CFGBSY idle and the error flags cleared first, PG +
 * two 32-bit writes (one double word) or PER + PNB + STRT, wait until BSY1 and CFGBSY clear, lock again. The CPU stalls
 * on flash fetches meanwhile (single bank): callers run only while OFF. Bounded waits (hw_micros). */
#define FLASH_KEYR (FLASH_REG + 0x08u)
#define FLASH_SR (FLASH_REG + 0x10u)
#define FLASH_CR (FLASH_REG + 0x14u)
#define FLASH_ECCR (FLASH_REG + 0x18u)
#define FLASH_SR_BUSY ((1u << 16) | (1u << 18))                 /* BSY1, CFGBSY */
#define FLASH_SR_ERRORS 0x43fau                                  /* OPERR PROGERR WRPERR PGAERR SIZERR PGSERR MISSERR FASTERR RDERR (not OPTVERR: review F1) */
#define FLASH_CR_LOCK (1u << 31)
#define IDLOG_PAGE_NUMBER ((IDLOG_ADDRESS - 0x08000000u) / IDLOG_BYTES)   /* 27 */
_Static_assert(IDLOG_ADDRESS % IDLOG_BYTES == 0u && IDLOG_ADDRESS + IDLOG_BYTES == 0x0800E000u, "the ID log is the last program page");
static bool flash_wait(uint32_t budget_us, bool errors)   /* errors: also fail on the error flags (after an operation) */
{
    uint32_t start = hw_micros();
    for (;;) {
        uint32_t sr = hw_read32(FLASH_SR);
        if (!(sr & FLASH_SR_BUSY)) return !errors || !(sr & FLASH_SR_ERRORS);
        if ((uint32_t)(hw_micros() - start) > budget_us) return false;
    }
}
/* record NULL: erase the page (22..40 ms); else program the double word at `offset` (~85 us). */
bool platform_flash_idlog(uint32_t offset, const uint8_t record[8])
{
    if (offset >= IDLOG_BYTES || (offset & 7u) || !flash_wait(1000u, false)) return false;   /* review F1: idle only, then clear */
    hw_write32(FLASH_SR, FLASH_SR_ERRORS | 1u);                  /* clear EOP and stale errors (rc_w1) */
    if (hw_read32(FLASH_CR) & FLASH_CR_LOCK) { hw_write32(FLASH_KEYR, 0x45670123u); hw_write32(FLASH_KEYR, 0xcdef89abu); }
    bool ok = !(hw_read32(FLASH_CR) & FLASH_CR_LOCK);
    if (ok && !record) {
        platform_watchdog_reload();
        hw_write32(FLASH_CR, (1u << 1) | (IDLOG_PAGE_NUMBER << 3));
        hw_write32(FLASH_CR, (1u << 1) | (IDLOG_PAGE_NUMBER << 3) | (1u << 16));
        ok = flash_wait(100000u, true);
    } else if (ok) {
        uint32_t w[2];
        for (unsigned i = 0; i < 8u; i++) ((uint8_t *)w)[i] = record[i];
        hw_write32(FLASH_CR, 1u);
        uint32_t irq = platform_irq_save();                     /* the two words back to back */
        hw_write32((uintptr_t)IDLOG_ADDRESS + offset, w[0]); hw_write32((uintptr_t)IDLOG_ADDRESS + offset + 4u, w[1]);
        platform_irq_restore(irq);
        ok = flash_wait(1000u, true);
    }
    modify(FLASH_CR, 3u, FLASH_CR_LOCK);                         /* PG and PER off, locked */
    return ok;
}
/* NMI: an ECC double error in the ID log page (a record torn by a power cut) is cleared and latched for idlog_scan; any
 * other NMI takes the default fault path unchanged. FLASH_ECCR: ECCD bit 31 (rc_w1), SYSF_ECC bit 20, ADDR_ECC bits
 * 13:0 = the double-word offset in main flash. */
static volatile bool idlog_ecc_latched;
bool platform_nmi_idlog_ecc(void)
{
    uint32_t e = hw_read32(FLASH_ECCR);
    uint32_t dw = e & 0x3fffu;
    if (!(e & (1u << 31)) || (e & (1u << 20)) || dw < (IDLOG_ADDRESS - 0x08000000u) / 8u || dw >= (IDLOG_ADDRESS - 0x08000000u + IDLOG_BYTES) / 8u)
        return false;
    hw_write32(FLASH_ECCR, (e & (1u << 24)) | (1u << 31));       /* clear ECCD only; keep ECCCIE */
    idlog_ecc_latched = true;
    return true;
}
bool platform_idlog_ecc_take(void) { bool e = idlog_ecc_latched; idlog_ecc_latched = false; return e; }
const volatile uint8_t *platform_idlog_page(void) { return (const volatile uint8_t *)(uintptr_t)IDLOG_ADDRESS; }

static uint32_t reset_info[3];   /* read 1112 (fast11: platform_boot_capture; fast15: bit 26 from platform_watchdog_start) */

/* fast12 (owner, 2026-11-20: "turn it on for watchdog"). The retained loader never started the IWDG (hardware fsr11-04),
 * so this firmware starts it once at boot. RM0444 order: start (forces the LSI on), unlock, prescaler + reload, wait for
 * the update, refresh. Timeout PLATFORM_IWDG_MS: well above the main loop's longest legitimate stall (UART TX wait 200 ms,
 * a v1.1 flash erase 25-40 ms) and above the 250 ms main-loop stall check, so a hung main loop is still caught there
 * first (recorded as a stall, read 1112); the IWDG is the backstop when interrupts stop too. Not stoppable by software;
 * any reset (also into the loader) stops it, so the loader and flashing run without it. */
void platform_watchdog_start(void)
{
    hw_write32((uintptr_t)0x40003000u, 0xccccu);
    hw_write32((uintptr_t)0x40003000u, 0x5555u);
    hw_write32((uintptr_t)0x40003004u, PLATFORM_IWDG_PR);
    hw_write32((uintptr_t)0x40003008u, PLATFORM_IWDG_RLR);
    uint32_t polls = 0u;
    while (polls < WAIT_POLLS && (hw_read32((uintptr_t)0x4000300cu) & 7u) != 0u) { hw_relax(); ++polls; }
    hw_write32((uintptr_t)0x40003000u, 0xaaaau);
    /* fast15 (release review 2026-11-23 P2): the update never latched, so the IWDG runs at its reset default (/4 x 4096,
     * about 0.5 s, still above the 250 ms stall check). Behaviour unchanged; read 1112 word 0 bit 26 reports it. Called
     * once per boot, after platform_boot_capture has rewritten word 0. */
    if (polls >= WAIT_POLLS) reset_info[0] |= PLATFORM_RESET_IWDG_UPDATE_TIMEOUT;
}

/* Backup-domain write access (RTCAPB+PWR clocks, PWR_CR1.DBP) then a BDRST pulse, which clears
 * TAMP_BKP0..4. Shared by arm (then writes BOOT/ENTR) and disarm. Order unchanged from the proven
 * arm sequence (own marker recovery.c). */
static bool backup_reset(void)
{
    modify(RCC + 0x3cu, 0u, (1u << 10) | (1u << 28));
    (void)hw_read32(RCC + 0x3cu);
    modify((uintptr_t)0x40007000u, 0u, 1u << 8);
    bool ready = false;
    for (uint32_t polls = 0u; polls < WAIT_POLLS; ++polls) {
        platform_watchdog_reload();
        if ((hw_read32((uintptr_t)0x40007000u) & (1u << 8)) != 0u) { ready = true; break; }
    }
    modify(RCC + 0x5cu, 0u, 1u << 16);
    modify(RCC + 0x5cu, 1u << 16, 0u);
    return ready;
}

bool platform_arm_loader(void)
{
    bool ready = backup_reset();
    hw_write32((uintptr_t)0x4000b100u, 0x544f4f42u);
    hw_write32((uintptr_t)0x4000b104u, 0x52544e45u);
    return ready && hw_read32((uintptr_t)0x4000b100u) == 0x544f4f42u &&
           hw_read32((uintptr_t)0x4000b104u) == 0x52544e45u;
}

/* FW231: firmware_main arms the retained loader at EVERY boot (so a crash/watchdog reset lands in
 * the loader). A software reset keeps the backup domain, so 0x08 Reboot must first clear that
 * request with a backup-domain reset (BDRST clears TAMP_BKP0..4). Hardware 2026-11-20 (ID3):
 * without this, 0x08 stayed in the loader until a power cycle. */
bool platform_disarm_loader(void)
{
    (void)backup_reset();
    return hw_read32((uintptr_t)0x4000b100u) != 0x544f4f42u && hw_read32((uintptr_t)0x4000b104u) != 0x52544e45u;
}

void platform_led(bool on)
{
    hw_write32(GPIOA + (on ? 0x28u : 0x18u), PLATFORM_LED_PIN); /* BRR = low = ON, BSRR = high = OFF */
}

static void dma_loader_quiesce_and_release(void)
{
    (void)dma_loader_quiesce();
    hw_write32(GPIOC + 0x28u, 1u << 6);
}

void platform_enter_loader(void)
{
    (void)platform_irq_save();
    bridge_early_off();
    bridge_disable();
    (void)dma_loader_quiesce();
    modify(RCC + 0x34u, 0u, 1u << 2);
    hw_write32(GPIOC + 0x28u, 1u << 6);
    /* Both arm_loader and the retained loader clear the backup domain.
     * Best-effort evidence instead uses16 bytes of reserved soft-reset SRAM;
     * linker excludes200017C0..17FF from BSS. Reporter must copy it before
     * its own initialization; loader SRAM clobber remains a measured limit. */
    uint32_t retained[3] = {debug_words[0], debug_words[1], debug_words[2]};
    (void)platform_arm_loader(); /* request reset even if backup acknowledgement failed */
    debug_commit_retained(retained[0], retained[1], retained[2]);
    #ifdef PLATFORM_UART_TEST
    platform_test_loader();
    #else
    __asm__ volatile("dsb" ::: "memory");
    hw_write32((uintptr_t)0xe000ed0cu, 0x05fa0004u);
    __asm__ volatile("dsb" ::: "memory");
    for (;;) hw_relax();
    #endif
}

/* FW231 fast11 (owner 2026-11-20: "a hung servo should reboot back into its firmware"). Reset record: 16 B of the
 * retained SRAM block (17D0..17DF, after the 16 B trace at 17C0; outside BSS, so startup never zeroes it; it survives a
 * system reset and is random after power-on, hence magic + check word). Written just before every reset this firmware
 * starts, read once at the next boot (platform_boot_capture) and rewritten with reason NONE, which carries the counts
 * over a reset nobody announced (the IWDG). Whether the retained loader keeps it across its own run is measured on
 * hardware; without it the RCC_CSR flags still tell watchdog / software / power-on apart. */
#define RESET_RECORD ((uintptr_t)0x200017d0u)
enum { RESET_MAGIC = 0x31545352u /* "RST1" */ };
static uint32_t hang_count, fast_hangs;
static bool app_healthy, long_run;
static void reset_record(uint32_t reason, uint32_t hangs, uint32_t fast, bool run)
{
    const uint32_t counts = (hangs & 0xffffu) | ((fast & 0xffu) << 16) | ((uint32_t)run << 24);
    hw_write32(RESET_RECORD, 0u);
    hw_write32(RESET_RECORD + 4u, reason);
    hw_write32(RESET_RECORD + 8u, counts);
    hw_write32(RESET_RECORD + 12u, reason ^ counts ^ 0xa5a5a5a5u);
#ifndef PLATFORM_UART_TEST
    __asm__ volatile("dsb" ::: "memory");
#endif
    hw_write32(RESET_RECORD, RESET_MAGIC);   /* commit last */
#ifndef PLATFORM_UART_TEST
    __asm__ volatile("dsb" ::: "memory");
#endif
}
/* Output off, DMA quiet, DE released, the boot-time loader request cleared: the loader boots this application again. */
static void reset_into_app(void) __attribute__((noreturn));
static void reset_into_app(void)
{
    dma_loader_quiesce_and_release();
    (void)platform_disarm_loader();   /* if it failed the reset still happens and lands in the loader (recoverable) */
    #ifdef PLATFORM_UART_TEST
    platform_test_loader();
    #else
    __asm__ volatile("dsb" ::: "memory");
    hw_write32((uintptr_t)0xe000ed0cu, 0x05fa0004u);
    __asm__ volatile("dsb" ::: "memory");
    for (;;) hw_relax();
    #endif
}

/* FW231: 0x08 Reboot (host). Same orderly output/DMA shutdown as the loader path, then CLEAR the loader request that
 * every boot arms, so the loader boots this application again. fast11: recorded as a host reboot, counts kept. */
void platform_reset_app(void)
{
    (void)platform_irq_save();
    bridge_early_off();
    bridge_disable();
    reset_record(PLATFORM_RESET_HOST, hang_count, fast_hangs, long_run);
    reset_into_app();
}

/* fast11: first thing at boot, before the loader request is armed. RCC_CSR reset flags (then cleared, RMVF) and the
 * record give the cause; hang reboots (stall, fault, DMA, watchdog) are counted since power-on. */
void platform_boot_capture(void)
{
    app_healthy = long_run = false;   /* zero at a real boot anyway (BSS) */
    const uint32_t csr = hw_read32(RCC + 0x60u);
    modify(RCC + 0x60u, 0u, 1u << 23);
    const uint32_t m = hw_read32(RESET_RECORD), r = hw_read32(RESET_RECORD + 4u), c = hw_read32(RESET_RECORD + 8u);
    const bool power = (csr & (1u << 27)) != 0u;      /* PWRRSTF (POR/BOR): SRAM is not ours */
    const bool valid = !power && m == RESET_MAGIC && hw_read32(RESET_RECORD + 12u) == (r ^ c ^ 0xa5a5a5a5u);
    uint32_t reason = valid ? r & 0xffffu : 0u;
    hang_count = valid ? c & 0xffffu : 0u;
    fast_hangs = valid ? (c >> 16) & 0xffu : 0u;
    const bool ran_long = valid && ((c >> 24) & 1u);
    if ((reason & 0xffu) == PLATFORM_RESET_NONE) {
        if (csr & (1u << 29)) { reason = PLATFORM_RESET_WATCHDOG; hang_count++; fast_hangs = ran_long ? 0u : fast_hangs + 1u; }
        else if (power) reason = PLATFORM_RESET_POWER;
        else if (csr & (1u << 28)) reason = PLATFORM_RESET_SOFTWARE;
        else if (csr & (1u << 26)) reason = PLATFORM_RESET_PIN;
    }
    reset_info[0] = reason | ((csr >> 24) << 16) | ((uint32_t)valid << 24);
    reset_info[1] = hang_count;
    reset_info[2] = fast_hangs;
    if (fast_hangs >= PLATFORM_FAST_HANG_LIMIT) {     /* boot-loop guard (watchdog streak): the loader, streak ended */
        reset_record(reason, hang_count, 0u, false);
        platform_enter_loader();
    }
    reset_record(PLATFORM_RESET_NONE, hang_count, fast_hangs, false);
}

/* fast11: the application ran healthy (main loop and ticks, called from the main loop): clear the loader request armed at
 * boot, so from now on a hang, a fault or a watchdog reset boots this application again. Boot-time failures still land
 * in the loader (a bad image or an early crash loop stays recoverable with the host tool's recover). */
void platform_app_healthy(void)
{
    if (app_healthy) return;
    (void)platform_disarm_loader();
    app_healthy = true;
    reset_info[0] |= 1u << 25;
}
/* fast11: up for 60 s: the next hang does not continue a quick-hang streak (also over an unannounced watchdog reset) */
void platform_app_long_run(void)
{
    uint32_t irq = platform_irq_save();
    long_run = true; fast_hangs = 0u;
    reset_record(PLATFORM_RESET_NONE, hang_count, 0u, true);
    platform_irq_restore(irq);
}
void platform_reset_info(uint32_t out[3]) { if (out) for (unsigned i = 0u; i < 3u; ++i) out[i] = reset_info[i]; }

/* fast11: a hang the firmware caught (main-loop stall, fault/unexpected vector, DMA stop failure). Healthy: reboot this
 * application, recorded. Before healthy, or the PLATFORM_FAST_HANG_LIMIT-th quick hang in a row: the loader as before. */
void platform_hang_reset(uint32_t reason)
{
    (void)platform_irq_save();
    bridge_early_off();
    bridge_disable();
    const uint32_t hangs = hang_count + 1u, fast = long_run ? 0u : fast_hangs + 1u;
    if (!app_healthy || fast >= PLATFORM_FAST_HANG_LIMIT) {
        reset_record(reason, hangs, 0u, false);       /* streak ended: a reflash or recover must boot the application */
        platform_enter_loader();
    }
    reset_record(reason, hangs, fast, false);
    reset_into_app();
}

static bool wait_early(uintptr_t address, uint32_t mask, uint32_t expected)
{
    for (uint32_t polls = 0u; polls < WAIT_POLLS; ++polls) {
        if ((hw_read32(address) & mask) == expected) return true;
        platform_watchdog_reload();
    }
    return false;
}

static bool wait_stage(uint32_t stage, uintptr_t address, uint32_t mask, uint32_t expected)
{
    platform_debug_mark(stage, expected);
    if (wait_early(address, mask, expected)) return true;
    platform_debug_mark(stage, hw_read32(address));
    return false;
}

static bool uart_wait(uint32_t mask, uint32_t expected, uint32_t start)
{
    for (uint32_t polls = 0u; polls < WAIT_POLLS; ++polls) {
        if ((uint32_t)(hw_micros() - start) >= UART_TX_TIMEOUT_US) return false;
        if ((hw_read32(UART + 0x1cu) & mask) == expected) return true;
        platform_watchdog_reload();
        hw_relax();
    }
    return false;
}

bool platform_init(void)
{
    clock_initialized = false;
    irq_outer_active = false;
    irq_outer_counter = irq_max_cycles = 0u;
    debug_words[2] = hw_read32(RCC + 0x60u); /* capture reset cause before changes */
    platform_debug_mark(PLATFORM_STAGE_INIT_ENTRY, 0u);
    /* This application owns all external vectors. Discard inherited enabled
     * and pending sources before clock handoff; this fixes the IRQ contract,
     * without asserting it caused any observed startup reset. */
    hw_write32(SYST, 0u);
    hw_write32((uintptr_t)0xe000e180u, 0xffffffffu); /* NVIC ICER, all32 IRQs */
    hw_write32((uintptr_t)0xe000e280u, 0xffffffffu); /* NVIC ICPR */
    hw_write32((uintptr_t)0xe000ed04u, (1u << 25) | (1u << 27)); /* SysTick/PendSV pending clear */
    if(!dma_loader_quiesce())return false; /* inherited DMA must not remain a bus master */
    /* Startup-only clock transition: RM0454 sections 3.3.4/4.1.4/5.2.3.
     * Every readiness/readback wait is bounded; caller retains output-off
     * and loader recovery on failure. HSI16 stays on for PLL and USART1. */
    modify(RCC, 0u, 1u << 8);
    if (!wait_stage(PLATFORM_STAGE_HSI_READY, RCC, 1u << 10, 1u << 10)) return false;
    modify(RCC + 8u, 7u, 0u);
    if (!wait_stage(PLATFORM_STAGE_HSI_SWITCH, RCC + 8u, 7u << 3, 0u)) return false;
    modify(RCC, 7u << 11, 0u);
    modify(RCC + 8u, (15u << 8) | (7u << 12), 0u);
    modify(RCC, 1u << 24, 0u);
    if (!wait_stage(PLATFORM_STAGE_PLL_STOP, RCC, 1u << 25, 0u)) return false;
    modify(RCC + 0x3cu, 0u, 1u << 28); /* PWR clock; preserve backup access. */
    (void)hw_read32(RCC + 0x3cu);
    modify(PWR, (3u << 9) | (1u << 14), 1u << 9); /* range1, normal run */
    if (!wait_stage(PLATFORM_STAGE_VOLTAGE_READY, PWR + 0x14u, (1u << 10) | (1u << 9), 0u)) return false; /* VOSF/REGLPF */
    modify(FLASH_REG, 7u, 2u); /* 2 wait states BEFORE increasing HCLK. */
    if (!wait_stage(PLATFORM_STAGE_FLASH_LATENCY, FLASH_REG, 7u, 2u)) return false;
    /* Unselected offline probe: preserve every bit except startup PRFTEN. */
    uint32_t prefetch_acr = hw_read32(FLASH_REG);
    if ((prefetch_acr & 7u) != 2u) {
        platform_debug_mark(PLATFORM_STAGE_FLASH_PREFETCH, prefetch_acr);
        return false;
    }
    prefetch_acr |= 1u << 8;
    hw_write32(FLASH_REG, prefetch_acr);
    if (!wait_stage(PLATFORM_STAGE_FLASH_PREFETCH, FLASH_REG, UINT32_MAX, prefetch_acr)) return false;
    /* HSI16 / M1 * N8 / R2 = 64MHz; only PLLR output enabled.
     * STM32G030 PLLR encodes divisor-1, unlike several other STM32 lines. */
    platform_debug_mark(PLATFORM_STAGE_PLL_CONFIG, 0x30000802u);
    hw_write32(RCC + 0x0cu, 0x30000802u);
    if ((hw_read32(RCC + 0x0cu) & 0xf0007f73u) != 0x30000802u) {
        platform_debug_mark(PLATFORM_STAGE_PLL_CONFIG, hw_read32(RCC + 0x0cu));
        return false;
    }
    modify(RCC, 0u, 1u << 24);
    if (!wait_stage(PLATFORM_STAGE_PLL_READY, RCC, 1u << 25, 1u << 25)) return false;
    modify(RCC + 8u, 7u, 2u);
    if (!wait_stage(PLATFORM_STAGE_PLL_SWITCH, RCC + 8u, 7u << 3, 2u << 3)) return false;
    hw_write32(SYST + 4u, 0x00ffffffu);
    hw_write32(SYST + 8u, 0u);
    hw_write32(SYST, 5u); /* CPU clock, enabled, NO SysTick IRQ */
    platform_time_reset(&clock_state, hw_read32(SYST + 8u));
    clock_initialized = true;
    platform_debug_mark(PLATFORM_STAGE_SYSTICK_READY, 0u);
    modify(RCC + 0x34u, 0u, (1u << 0) | (1u << 2));
    (void)hw_read32(RCC + 0x34u);
    hw_write32(GPIOC + 0x28u, 1u << 6);
    modify(GPIOC + 4u, 1u << 6, 0u);
    modify(GPIOC + 8u, 3u << 12, 2u << 12);
    modify(GPIOC + 12u, 3u << 12, 0u);
    modify(GPIOC, 3u << 12, 1u << 12);
    /* Status LED (owner, 2026-11-20): PA4 push-pull output, active-LOW (driving PA4 low lights it: LED on ->
     * GPIOA_BRR, off -> GPIOA_BSRR). Latched ON before output mode; main.c blinks it
     * for 3 s after every reset and then leaves it OFF (no idle LED current). */
    hw_write32(GPIOA + 0x28u, PLATFORM_LED_PIN);
    modify(GPIOA + 4u, PLATFORM_LED_PIN, 0u);
    modify(GPIOA + 8u, 3u << 8, 0u);
    modify(GPIOA + 12u, 3u << 8, 0u);
    modify(GPIOA, 3u << 8, 1u << 8);
    modify(GPIOA + 4u, (1u << 9) | (1u << 10), 0u);
    modify(GPIOA + 8u, (3u << 18) | (3u << 20), (2u << 18) | (2u << 20));
    modify(GPIOA + 12u, (3u << 18) | (3u << 20), (1u << 18) | (1u << 20));
    modify(GPIOA + 0x24u, (15u << 4) | (15u << 8), (1u << 4) | (1u << 8));
    modify(GPIOA, (3u << 18) | (3u << 20), (2u << 18) | (2u << 20));
    modify(RCC + 0x40u, 0u, 1u << 14);
    (void)hw_read32(RCC + 0x40u);
    hw_write32(UART, 0u);
    if (!wait_stage(PLATFORM_STAGE_UART_DISABLE, UART + 0x1cu, (1u << 21) | UART_REACK, 0u)) return false;
    /* G030 USART1 is the FULL instance with8-byte FIFOs (RM0454 section26.4).
     * RX is serviced by DMA (CR3.DMAR6) with DDRE13: a reception error stops
     * the request at the erroneous byte until the IRQ flushes and clears it.
     * No RX data interrupt (RXFTIE/RXFNEIE/IDLEIE all off); EIE/PEIE remain.
     * TXFTIE23 uses four empty slots (CR3.TXFTCFG[31:29]=010).
     * TC still means the FIFO AND physical shift register have finished. */
    platform_debug_mark(PLATFORM_STAGE_UART_FIFO, 0u);
    hw_write32(UART, UART_FIFOEN | UART_OVER8);  /* OVER8 only writable while UE=0 */
    if ((hw_read32(UART) & UART_FIFOEN) == 0u) {
        platform_debug_mark(PLATFORM_STAGE_UART_FIFO, hw_read32(UART));
        return false;
    }
    modify(RCC + 0x54u, 3u, UART_CLKSEL);
    hw_write32(UART + 4u, UART_RTOEN);           /* FW231: receiver timeout on (fast start); RTOR set per reply */
    hw_write32(UART + 0x14u, 150u);
    hw_write32(UART + 8u, UART_TXFT_HALF | UART_EIE | UART_DMAR | UART_DDRE);
    if ((hw_read32(UART + 8u) & (UART_TXFTCFG_MASK | UART_RXFTCFG_MASK | UART_RXFTIE | UART_EIE | UART_DMAR | UART_DDRE)) != (UART_TXFT_HALF | UART_EIE | UART_DMAR | UART_DDRE)) {
        platform_debug_mark(PLATFORM_STAGE_UART_FIFO, hw_read32(UART + 8u));
        return false;
    }
    hw_write32(UART + 0x2cu, 0u);
    hw_write32(UART + 12u, UART_BRR_VALUE);
    hw_write32(UART + 0x20u, 0xffffffffu);
    rx_boundary = rx_consumed = 0u; uart_fault_bits = 0u;
    for (unsigned i = 0u; i < PLATFORM_UART_COUNTS; ++i) uart_counts[i] = 0u;
    tx_data = NULL; tx_size = tx_at = 0u; tx_busy = tx_complete = tx_dma_active = tx_dma_wait_tc = false;transfer_failed=0;
    hw_write32(UART, UART_FIFOEN | UART_OVER8 | 1u | UART_RE | (1u << 3) | (1u<<8) | UART_RTOIE); /* FIFO,OVER8?,UE,RE,TE,PEIE,RTOIE */
    if ((hw_read32(UART) & (UART_FIFOEN | UART_IDLEIE | UART_RXFNEIE | (1u<<8))) != (UART_FIFOEN | (1u<<8))) {
        platform_debug_mark(PLATFORM_STAGE_UART_FIFO, hw_read32(UART));
        return false;
    }
    if (!wait_stage(PLATFORM_STAGE_UART_ENABLE, UART + 0x1cu, (1u << 21) | UART_REACK, (1u << 21) | UART_REACK)) return false;
    /* DMA1/DMAMUX: channel1 IRQ9 priority1; never clear global GIF. */
    modify(RCC+0x28u,1u,0u);modify(RCC+0x38u,0u,1u);(void)hw_read32(RCC+0x38u);
    hw_write32(DMA1+8u,0u);
    if(hw_read32(DMA1+8u)&1u)return false;
    hw_write32(DMA1+4u,DMA_EVENT_FLAGS);hw_write32(DMAMUX1,51u);
    /* RX channel2 (reset state after the quiesce reset released above): arm
     * circular; bytes already in the FIFO move as soon as it is enabled. */
    hw_write32(DMAMUX1+4u,50u);
    hw_write32(RXDMA_CNDTR,RX_RING_SIZE);hw_write32(RXDMA_CPAR,(uint32_t)(UART+0x24u));hw_write32(RXDMA_CMAR,(uint32_t)(uintptr_t)rx_ring);
#ifdef PLATFORM_UART_TEST
    platform_test_rx_dma_target(rx_ring,RX_RING_SIZE);
#endif
    hw_write32(RXDMA_CCR,RXDMA_CONFIG);hw_write32(RXDMA_CCR,RXDMA_CONFIG|1u);
    if(hw_read32(RXDMA_CCR)!=(RXDMA_CONFIG|1u))return false;
    modify((uintptr_t)0xe000e408u,0xffu<<8,0x40u<<8);
    hw_write32((uintptr_t)0xe000e280u,1u<<9);hw_write32((uintptr_t)0xe000e100u,1u<<9);
    /* UART IRQ27 is priority0, above TIM16's priority2. */
    modify((uintptr_t)0xe000e418u, 0xffu << 24, 0u);
    hw_write32((uintptr_t)0xe000e280u, 1u << 27);
    hw_write32((uintptr_t)0xe000e100u, 1u << 27);
    platform_debug_mark(PLATFORM_STAGE_INIT_READY, 0u);
    return true;
}

/* IRQ0 owns the final stop-bit -> receive transition. Foreground must never
 * flush RX or release DE again after this function has enabled reception. */
static void uart_finish_tx(bool complete)
{
    tx_dma_active=false;tx_dma_wait_tc=false;uint32_t events=0;
    if(!dma_stop_locked(&events)){UART_ERROR(0x10u);bridge_disable();platform_hang_reset(PLATFORM_RESET_DMA);}
    uint32_t raw=hw_read32(UART+0x1cu)&15u;
    if(raw){UART_ERROR(2u|(raw<<8));uart_count_raw(raw);complete=false;}
    if(events&8u){UART_ERROR(0x10u);bridge_disable();complete=false;}
    modify(UART, UART_TXEIE | UART_TCIE, 0u);
    modify(UART + 8u, UART_TXFTIE | UART_DMAT, 0u);
    hw_write32(GPIOC + 0x28u, 1u << 6);
    if (!complete) {DMA_COUNT(6);hw_write32(UART + 0x18u, 1u << 4);}
    hw_write32(UART + 0x18u, 1u << 3);
    /* Do not erase a different error asserted after the sampled status. */
    hw_write32(UART + 0x20u, (1u<<6)|UART_IDLE|raw);
    modify(UART, UART_RXFNEIE | UART_IDLEIE, UART_RE);
    tx_complete = complete;tx_busy = false;
}
void DMA1_Channel1_IRQHandler(void)
{
    uint32_t irq=platform_irq_save();
#if PLATFORM_DEBUG_MARKS
    uint32_t body=hw_read32(SYST+8u);observation_activity++;
#endif

    uint32_t flags=hw_read32(DMA1)&DMA_EVENT_FLAGS,all=flags;
    if((flags&8u) && tx_busy){DMA_COUNT(4);UART_ERROR(0x10u);bridge_disable();uart_finish_tx(false);}
    else if((flags&2u) && tx_busy && tx_dma_active){
        tx_dma_active=false;
        if(!dma_stop_locked(&all)){UART_ERROR(0x10u);bridge_disable();platform_hang_reset(PLATFORM_RESET_DMA);}
        uint32_t raw=hw_read32(UART+0x1cu)&15u;
        if(all&8u){if(raw)UART_ERROR(2u|(raw<<8));DMA_COUNT(4);UART_ERROR(0x10u);bridge_disable();uart_finish_tx(false);}
        else if(raw || transfer_failed){if(raw)UART_ERROR(2u|(raw<<8));DMA_COUNT(4);uart_finish_tx(false);} /* FW231: reply lost, counted, bridge untouched */
        else {DMA_COUNT(3);tx_dma_wait_tc=true;modify(UART,0u,UART_TCIE);}
    } else if(flags)hw_write32(DMA1+4u,flags);
#if PLATFORM_DEBUG_MARKS
    uint32_t cycles=(body-hw_read32(SYST+8u))&0x00ffffffu;if(cycles>dma_counts[7])dma_counts[7]=cycles;
#endif
    platform_irq_restore(irq);
}
#if PLATFORM_DEBUG_MARKS
void platform_dma_read(uint8_t out[32]){if(!out)return;for(unsigned i=0;i<8;i++){uint32_t v=dma_counts[i];for(unsigned j=0;j<4;j++)out[4*i+j]=(uint8_t)(v>>(8*j));}}
#endif

/* Account RX DMA half-boundary events (caller holds PRIMASK; sole caller pop).
 * No RX interrupt exists: the HT/TC/TE flags are set by hardware regardless of
 * interrupt enables and are polled here. Events alternate HT,TC,HT,...; both
 * pending or the wrong one means a whole half was not accounted (lost lap). At
 * boundary k the DMA starts overwriting the half that held bytes
 * [W_k-1024, W_k-512); all must be consumed by then. Pop accounts before it
 * consumes, so no byte is consumed after an unaccounted event. */
static uint32_t rx_dma_position(void){return (RX_RING_SIZE-hw_read32(RXDMA_CNDTR))&(RX_RING_SIZE-1u);}
static void rx_dma_account_locked(void)
{
    uint32_t flags=hw_read32(DMA1)&RXDMA_FLAGS;
    if(!flags)return;
    hw_write32(DMA1+4u,flags);
    if(flags==((rx_boundary&RX_HALF)?RXDMA_TC:RXDMA_HT)){
        rx_boundary+=RX_HALF;
        if(rx_boundary-rx_consumed<=RX_HALF)return;
    }
    /* Transfer error (hardware disabled the channel: 0x10), or overflow/lost lap
     * (1): latch the fault and discard the whole backlog, re-basing the boundary
     * on the DMA's current half so later accounting stays exact. */
    if(flags&RXDMA_TE){UART_ERROR(0x10u);bridge_disable();}
    else {UART_ERROR(1u);uart_counts[4]++;rx_loss_epoch++;} /* FW231: overflow/lost lap is counted; backlog discarded; parser resyncs */
    uint32_t position=rx_dma_position();
    if((position^rx_boundary)&RX_HALF)rx_boundary+=RX_HALF;
    rx_consumed=rx_boundary+(position&(RX_HALF-1u));
}
/* Errors and TC only; RX data never interrupts. With DDRE the erroneous byte
 * stays in the FIFO: flush before clearing so DMA cannot deliver it. */
void USART1_IRQHandler(void)
{
#if PLATFORM_DEBUG_MARKS
    uint32_t obs_start=hw_read32(SYST+8u);observation_activity++;observation_uart_entries++;
#endif
    uint32_t status = hw_read32(UART + 0x1cu);
    uint32_t control = hw_read32(UART);
    if ((status & 15u) != 0u) {
        UART_ERROR(2u | ((status & 15u) << 8));
        if (tx_busy) uart_finish_tx(false);
        else {
            uart_count_raw(status & 15u);
            hw_write32(UART + 0x18u, 1u << 3);
            hw_write32(UART + 0x20u, status & 15u);
        }
    } else if (tx_busy && tx_dma_wait_tc && (control & UART_TCIE) != 0u && (status & (1u << 6)) != 0u) {
        DMA_COUNT(5);uart_finish_tx(true);
    }
    /* FW231 fast start: the line has been idle for RTOR bit times since the last stop bit. */
    if ((status & UART_RTOF) != 0u) { hw_write32(UART + 0x20u, UART_RTOF); if (!tx_busy) platform_uart_rto(); }
#if PLATFORM_DEBUG_MARKS
    observation_uart_cycles+=(obs_start-hw_read32(SYST+8u))&0x00ffffffu;
#endif
}

/* FW231 fast start (bus.h bus_watch). Default: no watcher. main.c defines the real one. */
__attribute__((weak)) void platform_uart_rto(void) {}
/* USART IRQ or PRIMASK: bytes the DMA has written so far (same accounting as pop), one ring byte by index,
 * and the foreground's next unread index. A watcher cursor more than half a ring behind is stale. */
uint32_t platform_uart_rx_written(void) { rx_dma_account_locked(); return rx_boundary + (rx_dma_position() & (RX_HALF - 1u)); }
const volatile uint8_t *platform_uart_rx_ring(void) { return rx_ring; }   /* PLATFORM_RX_RING bytes, index & (size-1) */
uint32_t platform_uart_rx_next(void) { return rx_consumed; }
void platform_uart_set_rto(uint32_t bits) { hw_write32(UART + 0x14u, bits & 0x00ffffffu); }

/* Bytes written >= accounted boundary + offset within the current half. If the
 * DMA crossed into the next half after the poll (event still pending), the true
 * count is boundary+512+offset, so this is an underestimate, never an overestimate;
 * the next pop accounts the event. (One masked pop cannot span a whole half.) */
/* FW231: up to max bytes in ONE masked section (the main loop takes the backlog in batches instead of
 * paying a whole loop iteration per byte: hardware 2026-11-20 fsr-02, 62..73 bytes behind the wire). */
uint32_t platform_uart_read(uint8_t *out, uint32_t max) /* out non-NULL, max <= 64 (bounds the masked copy) */
{
    uint32_t saved = platform_irq_save();
    rx_dma_account_locked();
    uint32_t position = rx_dma_position();
    uint32_t written = rx_boundary + (position & (RX_HALF - 1u));
    uint32_t n = (int32_t)(written - rx_consumed) > 0 ? written - rx_consumed : 0u;
    if (n > rx_backlog_max) rx_backlog_max = n;                    /* FW231 diag */
    if (n > max) n = max;
    for (uint32_t k = 0u; k < n; ++k) out[k] = rx_ring[(rx_consumed + k) & (RX_RING_SIZE - 1u)];
    rx_consumed += n;
    platform_irq_restore(saved);
    return n;
}
bool platform_uart_pop(uint8_t *byte) { return platform_uart_read(byte, 1u) != 0u; }

uint32_t platform_uart_pending_faults(void) { return hw_read32(UART+0x1cu)&15u; }
uint32_t platform_uart_faults(void) { return uart_fault_bits; }
uint32_t platform_uart_fatal(void) { return uart_fault_bits&0x10u; }
uint32_t platform_uart_rx_loss(void) { return rx_loss_epoch; }
/* FW231 (review): after a baud switch, bytes received at the old rate must not be parsed (a CRC-valid
 * pre-switch frame could otherwise "confirm" the new rate). */
void platform_uart_rx_discard(void)
{
    uint32_t saved = platform_irq_save();
    rx_dma_account_locked();
    rx_consumed = rx_boundary + (rx_dma_position() & (RX_HALF - 1u));
    rx_loss_epoch++;
    platform_irq_restore(saved);
}
/* Write 1064 <- 0. The reset's own masked section is not measured (it would be the new maximum). */
void platform_irq_max_reset(void) { uint32_t saved = platform_irq_save(); irq_max_cycles = 0u; irq_max_site = 0u; rx_backlog_max = 0u; if (!saved) irq_outer_active = false; platform_irq_restore(saved); }
uint32_t platform_irq_max_site(void) { return irq_max_site; }
uint32_t platform_uart_rx_backlog_max(void) { return rx_backlog_max; }
void platform_uart_counts(uint32_t out[PLATFORM_UART_COUNTS]) { if(!out)return;uint32_t irq=platform_irq_save();for(unsigned i=0;i<PLATFORM_UART_COUNTS;i++)out[i]=uart_counts[i];platform_irq_restore(irq); }

bool platform_uart_tx_idle(void) { return !tx_busy; }
/* Caller: TX idle, foreground. Reprograms clock/BRR/OVER8 with UE=0; RX DMA (CR3)
 * stays armed. Error flags raised by the switch itself are flushed and cleared.
 * FW231: 1 Mbaud (HSI16, BRR16, qualified), 2 Mbaud (SYSCLK 64 MHz, OVER16, BRR 32, exact),
 * 3 Mbaud (SYSCLK, OVER8, USARTDIV 43 = 2.977 Mbaud). Anything else is refused. */
bool platform_uart_set_baud(uint32_t bps)
{
    uint32_t clksel, brr, over8;
    if (bps == 1000000u) { clksel = UART_CLKSEL; brr = UART_BRR_VALUE; over8 = 0u; }
    else if (bps == 2000000u) { clksel = UART_FAST_CLKSEL; brr = 32u; over8 = 0u; }
    else if (bps == 3000000u) { clksel = UART_FAST_CLKSEL; brr = UART_FAST_BRR; over8 = UART_FAST_OVER8; }
    else return false;
    uint32_t irq = platform_irq_save();
    if (tx_busy) { platform_irq_restore(irq); return false; }
    uint32_t cr1 = hw_read32(UART);
    hw_write32(UART, cr1 & ~1u);
    modify(RCC + 0x54u, 3u, clksel);
    hw_write32(UART + 12u, brr);
    cr1 = (cr1 & ~(1u << 15)) | over8;
    hw_write32(UART, cr1 & ~1u);
    hw_write32(UART, cr1 | 1u);
    bool ok = wait_early(UART + 0x1cu, (1u << 21) | UART_REACK, (1u << 21) | UART_REACK);
    hw_write32(UART + 0x18u, 1u << 3);          /* RXFRQ: drop any half-received byte */
    hw_write32(UART + 0x20u, 15u);              /* clear PE/FE/NE/ORE from the switch */
    platform_irq_restore(irq);
    return ok;
}

/* FW231 (review 2026-11-20): a scheduled group reply claims the line only if it is quiet at the
 * moment RE goes off: no unread RX bytes and no reception in progress (ISR.BUSY). Checked under the same
 * PRIMASK as the claim, so a tick ISR between the reply decision and this call cannot hide a predecessor
 * that started meanwhile. Deferred = nothing touched; the caller parses the bytes and decides again. */
static bool tx_quiet_required, tx_quiet_deferred;
bool platform_uart_transmit_if_quiet(const uint8_t *data, uint16_t size, bool *deferred)
{
    tx_quiet_required = true; tx_quiet_deferred = false;
    bool ok = platform_uart_transmit(data, size);
    tx_quiet_required = false;
    if (deferred) *deferred = tx_quiet_deferred;
    return ok;
}

/* PRIMASK held, or the priority-0 USART IRQ: this TX owns the UART from here; receiver off (REACK follows). */
static void tx_claim_locked(const uint8_t *data, uint16_t size)
{
    tx_data = data; tx_size = size; tx_at = 0u;
    tx_busy = true; tx_complete = false; tx_dma_active = false; tx_dma_wait_tc = false; transfer_failed = 0;
    modify(UART, UART_RE | UART_RXFNEIE | UART_IDLEIE | UART_TXEIE | UART_TCIE, 0u);
    modify(UART + 8u, UART_TXFTIE | UART_RXFTIE, 0u);
}
/* PRIMASK held, or the priority-0 USART IRQ; RE was turned off by the caller. Drive DE and start the TX DMA.
 * RX/error IRQ may have aborted after REACK changed but before this lock: never reassert DE/TXFTIE after
 * that interrupt's cleanup. False = not started (cleanup done here where the DMA was touched). */
static bool tx_arm_locked(const uint8_t *data, uint16_t size)
{
    uint32_t prearm_status=hw_read32(UART+0x1cu),prearm_raw=prearm_status&15u;
    if(prearm_raw)UART_ERROR(2u|(prearm_raw<<8)); /* counted when the IRQ clears it */
    if (!tx_busy || transfer_failed || (prearm_status&UART_REACK)) return false;
    hw_write32(GPIOC + 0x18u, 1u << 6);
    for (volatile unsigned lead = 0u; lead < 32u; ++lead) hw_relax();
    hw_write32(UART + 0x20u, 1u << 6);
    uint32_t events=0;
    if(!dma_stop_locked(&events)){UART_ERROR(0x10u);bridge_disable();platform_hang_reset(PLATFORM_RESET_DMA);}
    uint32_t raw=hw_read32(UART+0x1cu)&15u;
    if(events&8u){
        if(raw)UART_ERROR(2u|(raw<<8));DMA_COUNT(4);UART_ERROR(0x10u);bridge_disable();uart_finish_tx(false);return false;
    }
    if(transfer_failed || raw){
        if(raw)UART_ERROR(2u|(raw<<8));DMA_COUNT(4);uart_finish_tx(false);return false; /* FW231: reply lost, counted */
    }
    hw_write32(DMA1+12u,size);hw_write32(DMA1+16u,(uint32_t)(UART+0x28u));hw_write32(DMA1+20u,(uint32_t)(uintptr_t)data);
#ifdef PLATFORM_UART_TEST
    extern void platform_test_dma_source(const uint8_t *,uint16_t);platform_test_dma_source(data,size);
#endif
    hw_write32(DMA1+8u,(1u<<7)|(1u<<4)|(1u<<3)|(1u<<1));
    tx_dma_active=true;DMA_COUNT(2);hw_write32(DMA1+8u,(1u<<7)|(1u<<4)|(1u<<3)|(1u<<1)|1u);
    modify(UART+8u,0u,UART_DMAT);
    return true;
}

/* FW231 fast start: called from the priority-0 USART IRQ (nothing preempts it). Same start as
 * platform_uart_transmit but it never waits for completion: RE off, REACK low within a few kernel clocks
 * (bounded spin), then DE + DMA; completion is the existing TC interrupt (uart_finish_tx re-enables RX).
 * Refused while a reception is in progress (BUSY) or an error is pending. False = not started. */
bool platform_uart_tx_irq(const uint8_t *data, uint16_t size)
{
    if (!data || !size || size > 640u || tx_busy || (hw_read32(UART + 0x1cu) & ((1u << 16) | 15u))) return false;
    tx_claim_locked(data, size);
    for (unsigned i = 0u; i < 64u && (hw_read32(UART + 0x1cu) & UART_REACK); ++i) hw_relax();
    if (tx_arm_locked(data, size)) return true;
    if (tx_busy) uart_finish_tx(false);
    uart_counts[5]++;
    return false;
}

bool platform_uart_transmit(const uint8_t *data, uint16_t size)
{
    if (data == NULL || size == 0u || size > 640u) return false;
    uint32_t start = hw_micros();
    uint32_t saved = platform_irq_save();
    if (tx_busy) {
        uart_finish_tx(false);
        UART_ERROR(4u);uart_counts[5]++;
        platform_irq_restore(saved);
        return false;
    }
    if (tx_quiet_required) {
        rx_dma_account_locked();
        uint32_t written = rx_boundary + (rx_dma_position() & (RX_HALF - 1u));
        if ((int32_t)(written - rx_consumed) > 0 || (hw_read32(UART + 0x1cu) & (1u << 16)) != 0u) {   /* unread / BUSY */
            tx_quiet_deferred = true;
            platform_irq_restore(saved);
            return false;
        }
    }
    tx_claim_locked(data, size);
    platform_irq_restore(saved);
    bool ok = uart_wait(UART_REACK, 0u, start);
    if (ok) {
        saved = platform_irq_save();
        ok = tx_arm_locked(data, size);
        platform_irq_restore(saved);
        /* No byte feeding or RX cleanup in foreground. Even if sensor IRQ2
         * occupies foreground for almost1ms, UART IRQ0 runs at wire deadlines. */
        for (uint32_t polls = 0u; ok && tx_busy; ++polls) {
            if ((uint32_t)(hw_micros() - start) >= UART_TX_TIMEOUT_US || polls >= WAIT_POLLS) {
                ok = false; break;
            }
            platform_watchdog_reload();
            hw_relax();
        }
        ok = ok && tx_complete && !transfer_failed;
    }
    if (ok) ok = uart_wait(UART_REACK, UART_REACK, start);
    if(ok){
        saved=platform_irq_save();uint32_t status=hw_read32(UART+0x1cu);
        if((status&15u)!=0u)UART_ERROR(2u|((status&15u)<<8)); /* counted when the IRQ clears it */
        ok=!transfer_failed && tx_complete && !tx_busy && (status&UART_REACK)!=0u;
        platform_irq_restore(saved);
    }
    if (!ok) {
        saved = platform_irq_save();
        /* A fault may have completed cleanup in IRQ0 already; abort any
         * remaining transfer under the same lock, and leave receive enabled. */
        if (tx_busy) uart_finish_tx(false);
        UART_ERROR(4u);uart_counts[5]++;
        platform_irq_restore(saved);
    }
    return ok;
}


uint32_t platform_control_timer_start(void)
{
    modify(RCC + 0x40u, 0u, 1u << 17);
    (void)hw_read32(RCC + 0x40u);
    modify(RCC + 0x30u, 0u, 1u << 17);
    modify(RCC + 0x30u, 1u << 17, 0u);
    hw_write32(TIM16, 0u);
    hw_write32(TIM16 + 0x0cu, 0u);
    hw_write32(TIM16 + 0x28u, 63u);
    hw_write32(TIM16 + 0x2cu, APP_PERIOD_US - 1u);
    hw_write32(TIM16 + 0x14u, 1u);
    hw_write32(TIM16 + 0x10u, 0u);
    hw_write32(TIM16 + 0x0cu, 1u);
    modify((uintptr_t)0xe000e414u, 0xffu << 8, 0x80u << 8);
    hw_write32((uintptr_t)0xe000e280u, 1u << 21);
    hw_write32((uintptr_t)0xe000e100u, 1u << 21);
    uint32_t now = hw_micros();
    hw_write32(TIM16, 1u);
    return now;
}
void platform_control_timer_ack(void) { hw_write32(TIM16 + 0x10u, ~1u); }
bool platform_control_timer_pending(void) { return (hw_read32(TIM16 + 0x10u) & 1u) != 0u; }

/* Freestanding compiler support. These own implementations do not allocate. */
#ifndef PLATFORM_UART_TEST
void *memcpy(void *restrict destination, const void *restrict source, size_t count)
{
    uint8_t *d = destination; const uint8_t *s = source;
    for (size_t at = 0u; at < count; ++at) d[at] = s[at];
    return destination;
}
void *memset(void *destination, int value, size_t count)
{
    uint8_t *d = destination;
    for (size_t at = 0u; at < count; ++at) d[at] = (uint8_t)value;
    return destination;
}
void *memmove(void *destination, const void *source, size_t count)
{
    uint8_t *d = destination; const uint8_t *s = source;
    if ((uintptr_t)d <= (uintptr_t)s) {
        for (size_t at = 0u; at < count; ++at) d[at] = s[at];
    } else while (count != 0u) { --count; d[count] = s[count]; }
    return destination;
}
#endif
#endif
