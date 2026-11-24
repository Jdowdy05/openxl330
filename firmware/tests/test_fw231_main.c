/* Native integration tests for FW231 main.c with synthetic boot data,
 * simulated peripheral I/O, and an interrupt interleave after goal capture. */
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app.h"
#include "idlog.h"
#include "bus.h"

static uint8_t test_calibration[4096], test_nv[4096];
static const uint8_t test_devcfg[BUS_DEVCFG_BYTES] = {0};
static void capture_with_schedule(const app_state *, app_write_plan *);
#define BUS_DEVCFG_ADDRESS ((uintptr_t)test_devcfg)
#define app_init(state, cal, nv, allow) app_init(state, test_calibration, test_nv, allow)
#define app_write_capture capture_with_schedule
/* fast16 review P1-T1 (2026-11-23): main.c must call app_write for the duty/torque commands with interrupts masked, so the
 * control tick never sees tau_mode and tau_q24 half updated. The wrapper records the mask depth of every such call. */
static unsigned irq_depth, motion_writes, unmasked_motion_writes;
static uint8_t app_write_masked_check(void *, uint16_t, uint16_t, const uint8_t *);
#define app_write app_write_masked_check
#include "../firmware/main.c"
#undef app_write
#undef app_write_capture
#undef app_init
#undef BUS_DEVCFG_ADDRESS

#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

enum { DEVICE_ID = 3, GOAL = 256, DIAG_DEADLINE = 852, MAX_BATCH = 32 };

typedef struct {
    uint8_t bytes[MAX_BATCH];
    uint32_t count;
    bool rx_lost;
} rx_batch;

typedef struct {
    uint8_t bytes[DXL_REPLY_CAP];
    uint16_t count;
} tx_frame;

static rx_batch batches[8];
static tx_frame replies[4];
static size_t batch_count, batch_index, reply_count;
static unsigned idle_reads, pending_checks, pending_checks_left, captures;
static uint32_t mock_us, timer_release, sensor_work_us, rx_epoch, rx_cursor;
static int16_t applied_pwm;
static bool running_tick, advance_after_capture, advance_on_unlock;
static jmp_buf finished;

static void put16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

static void put32(uint8_t *out, uint32_t value)
{
    put16(out, (uint16_t)value);
    put16(out + 2, (uint16_t)(value >> 16));
}

static void init_boot_data(void)
{
    /* Uniform synthetic four-segment calibration and a minimal RAM fixture
     * for NV parsing. No data was read from a servo. */
    memset(test_calibration, 0xff, sizeof test_calibration);
    put16(test_calibration + 8, 0);
    for (unsigned i = 1; i < 4; ++i)
        put16(test_calibration + 8u + 8u * (4u - i), (uint16_t)(i * 1024u));

    memset(test_nv, 0xff, sizeof test_nv);
    memset(test_nv, 0, 16);
    put16(test_nv, 0xabab);
    const uint8_t keys[] = {0, 1, 7, 8, 10, 11, 20, 21, 22, 23, 36, 37, 38, 39};
    const uint8_t vals[] = {176, 4, DEVICE_ID, 3, 0, 3, 0, 0, 0, 0, 224, 1, 0, 0};
    for (unsigned i = 0; i < sizeof keys; ++i) {
        put32(test_nv + 16u + 8u * i, keys[i] | ((uint32_t)vals[i] << 16));
        put32(test_nv + 20u + 8u * i, 0);
    }
}

static void enqueue(const uint8_t *bytes, size_t size, bool loss_before)
{
    CHECK(batch_count < sizeof batches / sizeof batches[0]);
    CHECK(size > 0 && size <= MAX_BATCH);
    rx_batch *b = &batches[batch_count++];
    memcpy(b->bytes, bytes, size);
    b->count = (uint32_t)size;
    b->rx_lost = loss_before;
}

/* All fixture payloads are chosen to require no byte stuffing. */
static size_t instruction(uint8_t *packet, uint8_t id, uint8_t inst,
                          const uint8_t *params, size_t n)
{
    CHECK(n + 10 <= MAX_BATCH);
    memcpy(packet, "\xff\xff\xfd\x00", 4);
    packet[4] = id;
    put16(packet + 5, (uint16_t)(n + 3));
    packet[7] = inst;
    if (n)
        memcpy(packet + 8, params, n);
    for (size_t i = 9; i < n + 8; ++i)
        CHECK(packet[i - 2] != 0xffu || packet[i - 1] != 0xffu || packet[i] != 0xfdu);
    put16(packet + 8 + n, dxl_crc(packet, (uint16_t)(n + 8)));
    return n + 10;
}

static void check_status(size_t row, uint8_t error, size_t payload_size)
{
    CHECK(row < reply_count);
    const tx_frame *r = &replies[row];
    CHECK(r->count == payload_size + 11);
    CHECK(memcmp(r->bytes, "\xff\xff\xfd\x00", 4) == 0);
    CHECK(r->bytes[4] == DEVICE_ID);
    CHECK((size_t)(r->bytes[5] | (r->bytes[6] << 8)) + 7 == r->count);
    CHECK(r->bytes[7] == 0x55 && r->bytes[8] == error);
    CHECK(frame_crc_ok(r->bytes, r->count));
}

static void run_foreground(void)
{
    /* A fresh process hosts each case, just like reset of main.c statics. */
    if (setjmp(finished) == 0)
        firmware_main();
    CHECK(batch_index == batch_count);
}

uint32_t hw_micros(void) { return mock_us; }
static unsigned loader_reads;
uint32_t hw_read32(uintptr_t addr)
{
    if (addr >= APP_LOADER_BASE && addr < APP_PROGRAM_BASE) {   /* fast13: count reads of the loader region */
        ++loader_reads;
        return (uint32_t)addr * 2654435761u;
    }
    return 0;
}
void hw_write32(uintptr_t addr, uint32_t value) { (void)addr; (void)value; }
uint32_t platform_irq_save(void) { ++irq_depth; return 0; }
void platform_irq_restore(uint32_t saved)
{
    (void)saved;
    CHECK(irq_depth > 0u);
    --irq_depth;
    if (advance_on_unlock) {
        advance_on_unlock = false;
        /* The control tick advances this reference after the capture mask. */
        CHECK(general_reference_tick(&app.reference));
    }
}
void platform_irq_enable(void) {}
uint32_t platform_irq_max_us(void) { return 0; }
uint32_t platform_irq_max_site(void) { return 0; }
void platform_irq_max_reset(void) {}
static unsigned watchdog_reloads, watchdog_starts;
void platform_watchdog_reload(void) { ++watchdog_reloads; }
void platform_watchdog_start(void) { ++watchdog_starts; }
bool platform_arm_loader(void) { return true; }
bool platform_init(void) { return true; }
void platform_boot_capture(void) {}
void platform_app_healthy(void) {}
void platform_app_long_run(void) {}
void platform_reset_info(uint32_t out[3]) { memset(out, 0, 3 * sizeof *out); }
void platform_led(bool on) { (void)on; }
/* fast17: the ID log page (firmware/idlog.h), simulated: program only on erased double words, whole-page erase */
static uint8_t idlog_page[IDLOG_BYTES];
static unsigned idlog_programs, idlog_erases;
static void idlog_blank(void) { memset(idlog_page, 0xff, sizeof idlog_page); }
bool platform_flash_idlog(uint32_t offset, const uint8_t record[8])
{
    CHECK(offset < IDLOG_BYTES && !(offset & 7u));
    if (!record) { ++idlog_erases; idlog_blank(); return true; }
    for (unsigned i = 0; i < 8u; ++i) if (idlog_page[offset + i] != 0xffu) return false;
    memcpy(idlog_page + offset, record, 8); ++idlog_programs; return true;
}
bool platform_idlog_ecc_take(void) { return false; }
const volatile uint8_t *platform_idlog_page(void) { return idlog_page; }
void platform_enter_loader(void) { abort(); }
void platform_reset_app(void) { abort(); }
void platform_hang_reset(uint32_t reason) { (void)reason; abort(); }
void platform_dma_read(uint8_t out[32]) { memset(out, 0, 32); }
uint32_t platform_uart_rx_loss(void) { return rx_epoch; }
uint32_t platform_uart_rx_next(void) { return rx_cursor; }
uint32_t platform_uart_rx_written(void) { return rx_cursor; }
uint32_t platform_uart_rx_backlog_max(void) { return 0; }
void platform_uart_rx_discard(void) { abort(); }
void platform_uart_counts(uint32_t out[PLATFORM_UART_COUNTS])
{
    memset(out, 0, PLATFORM_UART_COUNTS * sizeof *out);
}
uint32_t platform_uart_fatal(void) { return 0; }
uint32_t platform_uart_pending_faults(void) { return 0; }
bool platform_uart_tx_idle(void) { return true; }
bool platform_uart_set_baud(uint32_t bps) { (void)bps; abort(); }
void platform_uart_set_rto(uint32_t bits) { (void)bits; }
const volatile uint8_t *platform_uart_rx_ring(void) { abort(); }
bool platform_uart_tx_irq(const uint8_t *data, uint16_t size)
{
    (void)data; (void)size;
    abort();
}
uint32_t platform_uart_read(uint8_t *out, uint32_t limit)
{
    if (batch_index == 0 && advance_after_capture) {
        CHECK(general_reference_start_linear(&app.reference, 65536, 10));
        app.enabled = true;
    }
    if (batch_index == batch_count) {
        /* A direct reply may still be due after the final read. */
        if (++idle_reads > 2)
            longjmp(finished, 1);
        return 0;
    }
    const rx_batch *b = &batches[batch_index++];
    CHECK(b->count <= limit);
    if (b->rx_lost)
        ++rx_epoch;
    mock_us += 50;
    rx_cursor += b->count;
    memcpy(out, b->bytes, b->count);
    return b->count;
}

bool platform_uart_transmit(const uint8_t *data, uint16_t n)
{
    CHECK(reply_count < sizeof replies / sizeof replies[0]);
    CHECK(n <= sizeof replies[0].bytes);
    tx_frame *r = &replies[reply_count++];
    r->count = n;
    memcpy(r->bytes, data, n);
    return true;
}

bool platform_uart_transmit_if_quiet(const uint8_t *data, uint16_t n, bool *deferred)
{
    *deferred = false;
    return platform_uart_transmit(data, n);
}

uint32_t platform_control_timer_start(void) { return mock_us; }
void platform_control_timer_ack(void) {}
bool platform_control_timer_pending(void)
{
    if (running_tick)
        return mock_us - timer_release >= APP_PERIOD_US;
    ++pending_checks;
    if (pending_checks_left == 0)
        return false;
    --pending_checks_left;
    return true;
}

bool bridge_init(void) { return true; }
void bridge_disable(void) { applied_pwm = 0; }
uint32_t bridge_faults(void) { return 0; }
int16_t bridge_applied(void) { return applied_pwm; }
bool bridge_set(int16_t pwm, uint16_t cap)
{
    CHECK(pwm >= -(int)cap && pwm <= (int)cap);
    applied_pwm = pwm;
    return true;
}

bool sensing_init(void) { return true; }
bool sensing_encoder_restore(void) { return true; }
bool sensing_encoder_config(uint16_t *before, uint16_t *after)
{
    *before = *after = 0;
    return true;
}
uint32_t sensing_last_faults(void) { return 0; }
uint16_t sensing_last_encoder_word(void) { return 512; }
bool sensing_read_encoder(uint16_t *raw)
{
    mock_us += sensor_work_us;
    *raw = sensing_last_encoder_word();
    return true;
}
bool sensing_read_encoder_status_byte(uint8_t *status)
{
    *status = 0x20u;
    return true;
}
bool sensing_read_adc(sensing_adc *adc)
{
    *adc = (sensing_adc){.voltage_decivolts = 50, .temperature_c = 25};
    return true;
}
bool sensing_read_adc_tick(sensing_adc *adc) { return sensing_read_adc(adc); }

static void capture_with_schedule(const app_state *state, app_write_plan *plan)
{
    app_write_capture(state, plan);
    ++captures;
    if (advance_after_capture) {
        advance_after_capture = false;
        advance_on_unlock = true;
    }
}

static void run_tick(uint32_t release, uint32_t latency, uint32_t work)
{
    timer_release = release;
    mock_us = release + latency;
    main_progress_us = mock_us;
    sensor_work_us = work;
    running_tick = true;
    TIM16_IRQHandler();
    running_tick = false;
}

static void check_deadline(uint32_t total, uint32_t window, uint32_t maximum, bool tripped)
{
    uint8_t payload[16];
    CHECK(read_atomic(&app, DIAG_DEADLINE, sizeof payload, payload) == 0);
    fprintf(stdout, "deadline total=%u window=%u maximum=%u faults=%#x\n",
            get32(payload), get32(payload + 4), get32(payload + 8), app.faults);
    CHECK(get32(payload) == total);
    CHECK(get32(payload + 4) == window);
    CHECK(get32(payload + 8) == maximum);
    CHECK(app.faults == (tripped ? APP_FAULT_TIMING : 0u));
}

static void deadline_regression(void)
{
    CHECK(app_init(&app, test_calibration, test_nv, true));
    next_tick_us = 1000;
    unsigned skipped = 0;
    for (unsigned i = 0; i < 2000; ++i) {
        if (i % 30 == 1 && skipped < 51) {
            ++skipped;
            continue;
        }
        run_tick(1000u + i * APP_PERIOD_US, 0, 8);
    }
    CHECK(skipped == 51);
    check_deadline(51, 51, 51, false);
    for (unsigned i = 2000; i < 4000; ++i)
        run_tick(1000u + i * APP_PERIOD_US, 0, 8);
    check_deadline(51, 0, 51, false);
}

static void deadline_threshold(void)
{
    CHECK(app_init(&app, test_calibration, test_nv, true));
    next_tick_us = 1000;
    run_tick(1000, 0, 8);
    for (unsigned i = 1; i <= 100; ++i)
        run_tick(1000u + 2u * i * APP_PERIOD_US, 0, 8);
    check_deadline(100, 100, 100, false);
    run_tick(1000u + 202u * APP_PERIOD_US, 0, 8);
    check_deadline(101, 101, 101, true);
}

static void deadline_overrun(void)
{
    CHECK(app_init(&app, test_calibration, test_nv, true));
    next_tick_us = 1000;
    run_tick(1000, 0, 8);
    /* The next release is dropped and the service after it also crosses its
     * own deadline, even though its useful work takes only 8 us. */
    run_tick(1000u + 2u * APP_PERIOD_US, APP_PERIOD_US - 1u, 8);
    check_deadline(2, 2, 2, false);
}

/* fast12 (owner: "turn it on for watchdog"): the tick never refreshes the IWDG (it keeps running when the main loop hangs);
 * the main loop does, and boot starts it. */
static void watchdog_only_from_main_loop(void)
{
    CHECK(app_init(&app, test_calibration, test_nv, true));
    next_tick_us = 1000;
    for (unsigned i = 0; i < 2000; ++i)
        run_tick(1000u + i * APP_PERIOD_US, 0, 8);
    CHECK(watchdog_reloads == 0);
    uint8_t ping[MAX_BATCH];
    enqueue(ping, instruction(ping, DEVICE_ID, 1, NULL, 0), false);
    run_foreground();
    CHECK(watchdog_starts == 1 && watchdog_reloads > 0);
}

/* fast13 (owner: "we want that"): the loader CRC is computed once at boot; read 1136 answers it without reading the loader
 * again (fast12 recomputed it per read, about 13 ms in the main loop). */
static void loader_crc_cached(void)
{
    uint8_t packet[MAX_BATCH], params[4];
    put16(params, 1136);
    put16(params + 2, 8);
    enqueue(packet, instruction(packet, DEVICE_ID, 2, params, sizeof params), false);
    loader_reads = 0;
    run_foreground();
    CHECK(loader_reads == APP_LOADER_BYTES / 4u);                /* the boot computation, once */
    CHECK(reply_count == 1);
    check_status(0, 0, 8);
    uint32_t want = 0xffffffffu;                                  /* zlib CRC-32, bitwise, over the same words */
    for (uintptr_t a = APP_LOADER_BASE; a < APP_PROGRAM_BASE; ++a) {
        want ^= (uint8_t)(((uint32_t)(a & ~(uintptr_t)3u) * 2654435761u) >> (8u * (a & 3u)));
        for (int k = 0; k < 8; ++k) want = (want >> 1) ^ (0xedb88320u & -(want & 1u));
    }
    want = ~want;
    CHECK(get32(replies[0].bytes + 9) == want && get32(replies[0].bytes + 13) == APP_LOADER_BYTES);
}

/* fast16: an accepted torque write (264/268) renews a running opt-in lease (main.c write_atomic -> bus_lease_motion); a
 * write that is not a motion command does not. INT32_MIN is accepted without a configuration, so no config 320 here. */
static void lease_renewal(bool torque)
{
    uint8_t packet[MAX_BATCH], params[6];
    lease = (bus_lease){.ms = 100u, .action = BUS_LEASE_ACT_HOLD, .state = BUS_LEASE_RUNNING, .last_us = 0u};
    mock_us = 50000u;
    put16(params, torque ? 268 : 1152); put32(params + 2, torque ? 0x80000000u : 1u);   /* fast17: 1152 = 1 (1064 is logging-only) */
    enqueue(packet, instruction(packet, DEVICE_ID, 3, params, torque ? sizeof params : 3u), false);
    run_foreground();
    check_status(0, 0, 0);
    CHECK(lease.state == BUS_LEASE_RUNNING && (torque ? lease.last_us >= 50000u : lease.last_us == 0u));
}

static uint8_t app_write_masked_check(void *ctx, uint16_t a, uint16_t n, const uint8_t *in)
{
    if (a == 260u || a == 264u || a == 268u) {
        ++motion_writes;
        if (irq_depth == 0u) ++unmasked_motion_writes;
    }
    return app_write(ctx, a, n, in);
}

/* Unicast 264/268/260 and a Sync Write 264 row all reach app_write under the mask (INT32_MIN: accepted unconfigured). */
static void torque_writes_masked(void)
{
    uint8_t packet[MAX_BATCH], params[6], sync[9];
    const uint16_t addresses[3] = {264, 268, 260};
    for (unsigned k = 0; k < 3u; ++k) {
        put16(params, addresses[k]); put32(params + 2, 0x80000000u);
        enqueue(packet, instruction(packet, DEVICE_ID, 3, params, sizeof params), false);
    }
    put16(sync, 264); put16(sync + 2, 4); sync[4] = DEVICE_ID; put32(sync + 5, 0x80000000u);
    enqueue(packet, instruction(packet, 0xfe, 0x83, sync, sizeof sync), false);
    run_foreground();
    for (unsigned k = 0; k < 3u; ++k) check_status(k, 0, 0);
    CHECK(motion_writes == 4u && unmasked_motion_writes == 0u && irq_depth == 0u);
}

/* fast17-size: the FW231 images leave out reads 860, 1536, 840, 600, 336, 340 and writes 840, 336, 340 (build.sh DAQ=0
 * TUNE2=0 DIAG860=0, as this harness is built): reads answer 4 and writes 7, like any address the firmware does not have;
 * the kept neighbours (852, 324, 1088) still answer; fast17 moved 856 (sample budget) to the logging builds too. One boot
 * per case holds at most 4 replies: part 0..3. */
static void trimmed_registers(unsigned part)
{
    static const struct { uint16_t address, size; bool write; uint8_t error; } rq[4][4] = {
        {{860, 20, false, 4}, {860, 12, false, 4}, {1536, 32, false, 4}, {840, 4, false, 4}},
        {{600, 32, false, 4}, {336, 16, false, 4}, {340, 12, false, 4}, {852, 16, false, 0}},
        {{840, 4, true, 7}, {336, 16, true, 7}, {340, 12, true, 7}, {324, 40, false, 0}},
        {{856, 24, false, 4}, {1088, 20, false, 0}, {64, 4, false, 0}, {108, 8, false, 0}}};   /* fast17: 856 is a logging-build read */
    uint8_t packet[MAX_BATCH];
    for (unsigned k = 0; k < 4u; ++k) {
        uint8_t p[2 + 16] = {0};
        put16(p, rq[part][k].address);
        if (rq[part][k].write) {
            if (rq[part][k].address == 840u) p[2] = 1u;
            enqueue(packet, instruction(packet, DEVICE_ID, 3, p, 2u + rq[part][k].size), false);
        } else {
            put16(p + 2, rq[part][k].size);
            enqueue(packet, instruction(packet, DEVICE_ID, 2, p, 4), false);
        }
    }
    run_foreground();
    CHECK(reply_count == 4u);
    for (unsigned k = 0; k < 4u; ++k)
        check_status(k, rq[part][k].error, rq[part][k].error || rq[part][k].write ? 0u : rq[part][k].size);
    if (part == 3u)   /* status words 108 (DAQ drops) and 112 (records queued | period << 16): no ring in the images */
        CHECK(get32(replies[3].bytes + 9) == 0u && get32(replies[3].bytes + 13) == (APP_PERIOD_US << 16));
}

/* fast17: write 1144 (set ID): unicast, OFF; the record is in flash before the reply, which leaves from the OLD ID; the
 * servo answers at the new ID right after (a ping to the old ID gets nothing); read 1144 = {9, source 2 (log), 1 used, 254
 * free, flags formatted}. */
static void set_id(void)
{
    uint8_t packet[MAX_BATCH], w[10] = {0}, r[4];
    put16(w, 1144); memcpy(w + 2, "SID1", 4); w[6] = 9;
    enqueue(packet, instruction(packet, DEVICE_ID, 3, w, sizeof w), false);
    enqueue(packet, instruction(packet, DEVICE_ID, 1, NULL, 0), false);          /* old ID: silent now */
    enqueue(packet, instruction(packet, 9, 1, NULL, 0), false);
    put16(r, 1144); put16(r + 2, 8);
    enqueue(packet, instruction(packet, 9, 2, r, sizeof r), false);
    run_foreground();
    CHECK(reply_count == 3u && idlog_erases == 1u && idlog_programs == 2u);
    check_status(0, 0, 0);                                                         /* from ID 3 */
    CHECK(replies[1].bytes[4] == 9u && replies[1].bytes[7] == 0x55 && replies[1].bytes[8] == 0);
    CHECK(replies[2].bytes[4] == 9u && replies[2].bytes[8] == 0 && replies[2].bytes[9] == 9u && replies[2].bytes[10] == 2u);
    CHECK(get32(replies[2].bytes + 11) == (1u | (254u << 16)) && replies[2].bytes[15] == 1u);
    CHECK(idlog_page[8 + 1] == 9u && idlog_page[0] == 'X');
}

/* fast17: 1144 errors and guards: a broadcast write and a Sync Write row write nothing (no flash program); bad magic 4,
 * size 5; read 1144 on a blank page = {0xFF none, source 0 NV, 0 used, 255 free, not formatted}. */
static void set_id_guards(void)
{
    uint8_t packet[MAX_BATCH], w[10] = {0}, bad[10] = {0}, shortw[6] = {0}, r[4];
    put16(w, 1144); memcpy(w + 2, "SID1", 4); w[6] = 9;
    memcpy(bad, w, sizeof bad); bad[2] = 'X';
    put16(shortw, 1144); memcpy(shortw + 2, "SID1", 4);
    enqueue(packet, instruction(packet, 0xfe, 3, w, sizeof w), false);            /* broadcast: no reply, nothing written */
    enqueue(packet, instruction(packet, DEVICE_ID, 3, bad, sizeof bad), false);
    enqueue(packet, instruction(packet, DEVICE_ID, 3, shortw, sizeof shortw), false);
    put16(r, 1144); put16(r + 2, 8);
    enqueue(packet, instruction(packet, DEVICE_ID, 2, r, sizeof r), false);
    run_foreground();
    CHECK(reply_count == 3u && idlog_programs == 0u && idlog_erases == 0u);
    check_status(0, 4, 0); check_status(1, 5, 0); check_status(2, 0, 8);
    CHECK(replies[2].bytes[9] == 0xffu && replies[2].bytes[10] == 0u && get32(replies[2].bytes + 11) == (255u << 16) && replies[2].bytes[15] == 0u);
}

/* Stage 3: PING reports the servo's stock model from its NV journal (1200 in this fixture), firmware 231. */
static void ping_model(void)
{
    uint8_t packet[MAX_BATCH];
    enqueue(packet, instruction(packet, DEVICE_ID, 1, NULL, 0), false);
    run_foreground();
    check_status(0, 0, 3);
    CHECK(replies[0].bytes[9] == 0xb0 && replies[0].bytes[10] == 0x04 && replies[0].bytes[11] == 231);
}

static void parser_recovery(bool loss, bool skip)
{
    uint8_t ping[MAX_BATCH];
    size_t ping_size = instruction(ping, DEVICE_ID, 1, NULL, 0);
    if (!loss) {
        enqueue(ping, 7, false);
        enqueue(ping + 7, ping_size - 7, false);
    } else {
        if (skip) {
            /* Beginning of a longer broadcast status, whose remaining bytes
             * were lost. The deployed parser skips its unstuffed body. */
            const uint8_t prefix[] = {0xff, 0xff, 0xfd, 0, 0xfe, 0x10, 0x01, 0x55};
            enqueue(prefix, sizeof prefix, false);
        } else {
            uint8_t packet[MAX_BATCH], params[12] = {0};
            put16(params, 65);
            instruction(packet, DEVICE_ID, 3, params, sizeof params);
            enqueue(packet, 9, false);
        }
        enqueue(ping, ping_size, true);
    }
    run_foreground();
    CHECK(reply_count == 1);
    check_status(0, 0, 3);
    CHECK(replies[0].bytes[9] == 176 && replies[0].bytes[10] == 4);
    CHECK(replies[0].bytes[11] == APP_FIRMWARE_VERSION);
    CHECK(parse_gap_resets == 0);
    CHECK(parse_loss_resets == (loss && !skip ? 1u : 0u));
    CHECK(app.faults == 0);
}

static void goal_retry(unsigned pending, bool group, bool invalid, bool reference_race)
{
    const uint32_t target = 137;
    uint8_t packet[MAX_BATCH], params[21] = {0};
    size_t size;
    advance_after_capture = reference_race;
    pending_checks_left = pending;
    put16(params, GOAL);
    if (group) {
        put16(params + 2, 16);
        params[4] = DEVICE_ID;
        put32(params + 5, target);
        put32(params + 9, 1);
        size = instruction(packet, 0xfe, 0x83, params, sizeof params);
    } else {
        put32(params + 2, target);
        put32(params + 6, invalid ? 0u : 1u);
        size = instruction(packet, DEVICE_ID, 3, params, 18);
    }
    enqueue(packet, size, false);
    put16(params + 2, 16);
    size = instruction(packet, DEVICE_ID, 2, params, 4);
    enqueue(packet, size, false);
    run_foreground();

    const bool accepted = pending < 3 && !invalid;
    const unsigned attempts = invalid ? 1u : accepted ? pending + 1u + reference_race : 3u;
    const size_t read_row = group ? 0 : 1;
    CHECK(reply_count == read_row + 1);
    if (!group)
        check_status(0, accepted ? 0 : invalid ? 4 : 7, 0);
    check_status(read_row, 0, 16);
    CHECK(get32(replies[read_row].bytes + 9) == (accepted ? target : 0));
    CHECK(app.target_configured == accepted);
    CHECK(app.epoch == (accepted ? 1u : 0u));
    CHECK(captures == attempts);
    CHECK(pending_checks == (invalid ? 0u : attempts));
    if (reference_race)
        CHECK(!app.reference.active);
    CHECK(app.faults == 0);
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    init_boot_data();
    idlog_blank();
    const char *test = argv[1];
    if (!strcmp(test, "deadline-51")) deadline_regression();
    else if (!strcmp(test, "deadline-threshold")) deadline_threshold();
    else if (!strcmp(test, "deadline-overrun")) deadline_overrun();
    else if (!strcmp(test, "parser-fragmented")) parser_recovery(false, false);
    else if (!strcmp(test, "parser-loss")) parser_recovery(true, false);
    else if (!strcmp(test, "parser-skip-loss")) parser_recovery(true, true);
    else if (!strcmp(test, "goal-unicast-pending")) goal_retry(1, false, false, false);
    else if (!strcmp(test, "goal-sync-pending")) goal_retry(2, true, false, false);
    else if (!strcmp(test, "goal-retry-ceiling")) goal_retry(3, false, false, false);
    else if (!strcmp(test, "goal-invalid")) goal_retry(0, false, true, false);
    else if (!strcmp(test, "goal-reference-race")) goal_retry(0, false, false, true);
    else if (!strcmp(test, "watchdog-main-loop-only")) watchdog_only_from_main_loop();
    else if (!strcmp(test, "loader-crc-cached")) loader_crc_cached();
    else if (!strcmp(test, "lease-renew-torque")) lease_renewal(true);
    else if (!strcmp(test, "lease-renew-other")) lease_renewal(false);
    else if (!strcmp(test, "torque-writes-masked")) torque_writes_masked();
    else if (!strcmp(test, "ping-model")) ping_model();
    else if (!strcmp(test, "set-id")) set_id();
    else if (!strcmp(test, "set-id-guards")) set_id_guards();
    else if (!strncmp(test, "trimmed-registers-", 18) && test[18] >= '0' && test[18] <= '3' && !test[19])
        trimmed_registers((unsigned)(test[18] - '0'));
    else CHECK(false);
    printf("PASS %s\n", test);
    return 0;
}
