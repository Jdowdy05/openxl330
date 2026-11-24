/* FW231 Fast Sync Read at byte level: main.c's foreground loop (32-byte receive batches, the header parser,
 * skip_left, the fast-start accounting, the reply abandon/due checks) with the PendSV watcher, the TIM16 control
 * tick and the USART receiver timeout interleaved at sub-microsecond resolution, running the REAL protocol.c
 * (dxl_process) and bus.c (bus_reply, bus_fast, bus_watch) per servo. A host sends back-to-back Fast Sync Reads
 * (~1.5 ms cycle, like the hardware stress) and checks every row. One servo may run on a clock 1-2 % off (HSI):
 * its ticks, loop costs, receiver timeout and its own bit rate scale with it.
 *
 * Hardware 2026-11-20 (fw231-comp2-fast8 on IDs 1/2/3, hardware run bus3-05, 4-byte FSR stress): 4-byte rows lost in
 * 5/5000 [1,2,3] and 2/5000 [3,2,1] reads at 1M, always the MIDDLE servo never arming (then its follower abandons);
 * 68-byte rows never. The old loop below reproduces that; the fixed loop (main.c now) must lose nothing.
 *
 * Mechanism (old loop): the watcher (PendSV) sends the middle row just after the loop-top watch_sent check; a control
 * tick delays the batch read until the next servo's row has begun; the inner do/while parses that row's first byte
 * BEFORE it checks watch_sent, so the top-of-loop accounting finds rem = -1, clamps it to 0, and skip_left is one
 * byte too long: it eats the 0xFF that starts the host's next request (within the 3 ms frame gap), which is lost. */
#include "bus.h"
#include "protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks, failures;
#define CHECK(x) do{checks++;if(!(x)){failures++;fprintf(stderr,"%s:%d: FAIL %s\n",__func__,__LINE__,#x);return;}}while(0)

enum { RING = 1024u, BATCH = 32u, WATCH_MAX = 160u, FRAME_GAP_US = 3000u, PERIOD_US = 500u, NDEV = 6, WMAX = 8 };
/* main-loop costs (us, the servo's own clock), calibrated like test_bus.c (hardware fsr-02: ~16 us per idle iteration):
 * C_TOP before the watch_sent check (loop head, hw_micros, watchdog, LED), C_MID from that check to the DMA position
 * snapshot inside the masked receive read (fault/lease checks, the read's entry), C_CHK the rest of the read and the
 * reply/baud/idle checks after it. */
enum { C_TOP = 3, C_MID = 4, C_CHK = 9, ISR_MIN = 180, ISR_MAX = 330, ISR_EVERY8 = 60, HOST_TIMEOUT_US = 10000 };

static uint32_t mix(uint32_t x) { x ^= x >> 16; x *= 0x7feb352du; x ^= x >> 15; x *= 0x846ca68bu; x ^= x >> 16; return x; }
static uint32_t rnd(uint32_t *s) { *s = mix(*s + 0x9e3779b9u); return *s; }
static uint32_t urange(uint32_t *s, uint32_t lo, uint32_t hi) { return lo + rnd(s) % (hi - lo + 1u); }

/* ------------------------------------------------------------------ wire */
typedef struct { int dev; uint16_t n, done; uint8_t b[320]; uint64_t start, byte_ns, end, hold; } wtx;
static wtx wire[WMAX]; static unsigned nw; static bool collision; static uint32_t bps;

/* ------------------------------------------------------------------ servo */
enum { S_TOP, S_READ, S_CHK, S_PARSE, S_PROC };
typedef struct {
    uint8_t id; int idx; int32_t ppm; bool fix, f4; int fake_at; uint32_t seed;   /* fix: fast9 loop; f4: fast10 (review F-4) */
    /* RX DMA ring; RE is off from the TX claim to TC, so a servo never receives its own bytes */
    uint8_t ring[RING]; uint32_t written, rx_next; uint64_t last_rx_end, re_off_from, re_off_to;
    /* main.c foreground state */
    uint8_t rxb[BATCH]; uint32_t rxn, rxi;
    uint8_t frame[DXL_MAX_WIRE]; uint16_t have, total, skip_left, fast_after, pending_size; bool pending_fast;
    uint32_t now, last_byte_us, frame_done_us;
    bus_reply reply; bus_fast fastw; bus_watch watch;
    bool watch_on, watch_sent, rto_due; uint32_t watch_cursor, watch_tx_us; uint16_t watch_len;
    uint8_t work[DXL_REPLY_CAP];
    /* counters: main.c's, and the read-1080 words 8..11 of the fix (fsr_seen, fsr_rows, hdr_missed, parse_dropped =
     * rejected frames: invalid length or a 0x8A with a bad CRC) */
    uint32_t fast_bad, fast_armed, fast_superseded, status_crc_fails, gap_resets;
    uint32_t seen, rows, hdr_missed, dropped, hdr, irq_started;
    /* CPU: thread state + remaining cost (real ns); TIM16; PendSV; blocking foreground TX; receiver timeout */
    int st; uint64_t left_ns, next_tick, tick_end; unsigned tick_k;
    bool pendsv_pend, pendsv_run; uint64_t pendsv_left; uint8_t pendsv_st;
    uint64_t blocked_until;
    uint32_t rtor_bits; uint64_t rto_fired_for, rto_lat;
    unsigned reads;
} fdev;
static fdev dv[NDEV]; static unsigned ndv;

static uint32_t lus(const fdev *d, uint64_t t) { return (uint32_t)((t + (uint64_t)((int64_t)t * d->ppm / 1000000)) / 1000u); }
static uint64_t rns(const fdev *d, uint32_t tenth_us) { return (uint64_t)tenth_us * 100u * 1000000u / (uint64_t)(1000000 + d->ppm); }
static uint64_t byte_ns_of(int32_t ppm) { return 10000000000ull / ((uint64_t)bps * (uint64_t)(1000000 + ppm) / 1000000u); }

static bool line_busy(const fdev *d, uint64_t t)          /* ISR.BUSY: a foreign byte is being received */
{
    for (unsigned i = 0; i < nw; i++) if (wire[i].dev != d->idx && wire[i].start <= t && t < wire[i].end) return true;
    return false;
}
static void tx_start(int dev, uint64_t claim, uint64_t at, const uint8_t *b, uint16_t n, uint64_t de_ns)
{
    if (nw == WMAX) { collision = true; return; }
    wtx *w = &wire[nw++];
    w->dev = dev; w->n = n; w->done = 0; memcpy(w->b, b, n); w->start = at;
    w->byte_ns = dev >= 0 ? byte_ns_of(dv[dev].ppm) : byte_ns_of(0);
    w->end = at + (uint64_t)n * w->byte_ns; w->hold = w->end + de_ns;
    for (unsigned i = 0; i + 1u < nw; i++) if (wire[i].hold > at && wire[i].start < w->hold) collision = true;   /* two drivers */
    if (dev >= 0) { dv[dev].re_off_from = claim; dv[dev].re_off_to = w->end; }
}

static uint8_t sim_read(void *ctx, uint16_t a, uint16_t n, uint8_t *out)
{
    fdev *d = ctx; (void)a;
    for (uint16_t i = 0; i < n; i++) out[i] = (uint8_t)(d->id * 29u + d->reads * 7u + i * 13u);   /* varies per read */
    static const uint8_t fake[7] = {0xff, 0xff, 0xfd, 0x00, 0x01, 0x40, 0x00};       /* a header-like run in the data (F-4 test) */
    if (d->fake_at >= 0 && d->fake_at + 7 <= (int)n) memcpy(out + d->fake_at, fake, sizeof fake);
    d->reads++;
    return 0;
}
static uint8_t sim_write(void *ctx, uint16_t a, uint16_t n, const uint8_t *in) { (void)ctx; (void)a; (void)n; (void)in; return 0; }
static bool crc_ok(const uint8_t *f, uint16_t n) { return n >= 10u && dxl_crc(f, (uint16_t)(n - 2u)) == (uint16_t)(f[n - 2u] | (f[n - 1u] << 8)); }

static void go(fdev *d, int st, uint32_t tenth_us) { d->st = st; d->left_ns = rns(d, tenth_us * urange(&d->seed, 80u, 120u) / 100u); }
static uint32_t byte_cost(const fdev *d)   /* 1.5 us per byte, +3.3 us while the foreground verifier runs (fsr2-02) */
{
    return 15u + (d->pending_size && d->reply.kind == BUS_REPLY_FAST && !d->reply.triggered ? 33u : 0u);
}
static bool watch_stop(fdev *d) { d->watch_on = false; return d->watch_sent; }

/* PendSV (main.c watch_run): the run reads rx_written once, feeds the watcher up to it, then decides under PRIMASK */
static void watch_start(fdev *d)
{
    d->rto_due = false; d->pendsv_run = true;
    if (!d->watch_on) { d->pendsv_st = 0xffu; d->pendsv_left = rns(d, 10u); return; }
    uint32_t w = d->written, n = w - d->watch_cursor;
    d->pendsv_st = n > WATCH_MAX ? (uint8_t)BUS_WATCH_DEAD : bus_watch_run(&d->watch, d->ring, RING - 1u, &d->watch_cursor, w);
    d->pendsv_left = rns(d, 30u + 12u * (n > WATCH_MAX ? 0u : n));
}
static void watch_finish(fdev *d, uint64_t t)
{
    uint8_t st = d->pendsv_st;
    if (st == 0xffu || st == BUS_WATCH_CAPTURING) return;
    if (st == BUS_WATCH_READY && d->watch.kind == BUS_REPLY_FAST) bus_fast_seal(&d->watch.fast, d->work, d->watch_len);
    bool tx_busy = t < d->re_off_to;
    if (st == BUS_WATCH_READY && d->watch_on && d->written == d->watch_cursor && !line_busy(d, t) && !tx_busy) {
        tx_start(d->idx, t, t + rns(d, 20u), d->work, d->watch_len, rns(d, urange(&d->seed, 0u, 290u)));
        d->watch_tx_us = lus(d, t); d->watch_sent = true; d->irq_started++;
    }
    d->watch_on = false;
}

/* main.c 482-497: one received byte through the follower check, skip_left and the header parser.
 * Returns 1 when an instruction frame is complete (dxl_process follows, S_PROC). */
static int parse_byte(fdev *d)
{
    uint8_t byte = d->rxb[d->rxi++];
    if (d->fix) { d->hdr = d->hdr << 8 | byte; if (d->hdr == 0xfffffd00u && !d->skip_left && d->have != 3u) d->hdr_missed++; }   /* a packet start the parser was not ready for */
    if (d->pending_size && d->reply.kind == BUS_REPLY_FAST && !d->reply.triggered) {
        uint8_t s = bus_fast_feed(&d->fastw, byte);
        if (s == BUS_FAST_READY) { bus_fast_seal(&d->fastw, d->work, d->pending_size); bus_reply_trigger(&d->reply, d->last_byte_us); }
        else if (s == BUS_FAST_FAILED && !watch_stop(d)) { d->pending_size = 0; d->pending_fast = false; bus_reply_done(&d->reply); d->fast_bad++; }
    }
    if (d->skip_left) { d->skip_left--; return 0; }
    if (d->have == 0) { if (byte == 0xff) d->frame[d->have++] = byte; return 0; }
    if (d->have == 1) { if (byte == 0xff) d->frame[d->have++] = byte; else d->have = 0; return 0; }
    if (d->have == 2) { if (byte == 0xfd) d->frame[d->have++] = byte; else d->have = byte == 0xff ? 2 : 0; return 0; }
    if (d->have == 3 && byte != 0) { d->have = byte == 0xff ? 1 : 0; return 0; }
    d->frame[d->have++] = byte;
    if (d->have == 7) {
        uint16_t len = (uint16_t)(d->frame[5] | (d->frame[6] << 8));
        if (len < 3 || len > DXL_MAX_WIRE - 7) {
            if (len > DXL_MAX_WIRE - 7 && d->frame[4] == 0xfeu) d->skip_left = len;
            else if (d->fix) d->dropped++;
            d->have = d->total = 0; return 0;
        }
        d->total = (uint16_t)(len + 7u);
    }
    if (!d->total || d->have != d->total) return 0;
    d->frame_done_us = d->last_byte_us;
    uint8_t ins = d->frame[7];
    if (ins == 0x55u) {
        if (crc_ok(d->frame, d->have)) bus_observe_status(&d->reply, d->frame[4], d->last_byte_us); else d->status_crc_fails++;
        d->have = d->total = 0; return 0;
    }
    bool ok = crc_ok(d->frame, d->have);
    if (d->fix && ins == 0x8au) { if (ok) d->seen++; else d->dropped++; }
    if (d->pending_size && ok) {
        if (watch_stop(d)) d->watch_sent = false;
        if (d->pending_fast) d->fast_superseded++;
        if (bus_observe_instruction(&d->reply)) { d->pending_size = 0; d->pending_fast = false; }
    }
    return 1;
}
/* main.c 502-511 at the end of dxl_process's time. True: a request was processed (break to the loop top). */
static bool process_frame(fdev *d)
{
    uint8_t ins = d->frame[7];
    dxl_io io = {.id = d->id, .firmware = 231, .model = 1200, .context = d, .read = sim_read, .write = sim_write};
    dxl_result r = dxl_process(&io, d->frame, d->have, d->work, sizeof d->work);
    d->have = d->total = 0;
    if (!r.valid_for_us) return false;
    d->pending_size = r.reply_bytes; d->pending_fast = d->pending_size && r.fast;
    if (d->pending_fast) {
        if (d->fix) d->rows++;
        d->fast_armed += r.response_slot != 0u;
        d->fast_after = (uint16_t)(r.fast_len + 7u - r.prior_bytes - r.fast_seg);
        if (r.response_slot) bus_fast_begin(&d->fastw, r.prior_bytes, r.fast_seg, r.fast_len, r.prev_id);
    }
    if (d->pending_size) bus_reply_arm(&d->reply, &r, ins, d->frame_done_us, bps); else bus_reply_done(&d->reply);
    if (d->reply.kind == BUS_REPLY_CHAIN || d->reply.kind == BUS_REPLY_FAST) {
        bus_watch_begin(&d->watch, d->reply.kind, d->reply.prev_id, &d->fastw);
        d->watch_cursor = d->rx_next - (d->rxn - d->rxi); d->watch_len = d->pending_size; d->watch_sent = false;
        d->rtor_bits = d->reply.gap_us * (bps / 1000000u); d->rto_fired_for = 0; d->watch_on = true;   /* RTOR write: fires at once if already idle */
    }
    return true;
}
static void loop_next(fdev *d) { if (d->rxi < d->rxn && !d->watch_sent) go(d, S_PARSE, byte_cost(d)); else go(d, S_TOP, C_TOP * 10u); }
static void gap_reset(fdev *d)
{
    if ((d->have || d->skip_left) && d->now - d->last_byte_us >= FRAME_GAP_US) {
        if (d->have) d->gap_resets++;
        d->have = d->total = 0; d->skip_left = 0;
    }
}

static void thread_action(fdev *d, uint64_t t)
{
    switch (d->st) {
    case S_TOP:                                           /* main.c 437-441 */
        d->now = lus(d, t);
        if (d->watch_sent) {
            d->watch_sent = false;
            if (d->pending_size) {
                if (!d->reply.triggered) bus_reply_trigger(&d->reply, d->watch_tx_us - d->reply.gap_us);
                bus_reply_sent(&d->reply, d->watch_tx_us);
                if (d->pending_fast) {
                    int32_t rem = (int32_t)(d->watch_cursor - d->rx_next) + (int32_t)(d->rxn - d->rxi);
                    d->have = d->total = 0; d->skip_left = (uint16_t)((rem > 0 ? rem : 0) + d->fast_after);
                }
                d->pending_size = 0; d->pending_fast = false;
            }
        }
        if (d->f4 && t >= d->re_off_from && t < d->re_off_to) d->last_byte_us = d->now;   /* fast10 F-4: our TX is line activity */
        go(d, S_READ, C_MID * 10u); return;
    case S_READ:                                          /* main.c 456-458: one masked batch read */
        if (d->rxi == d->rxn) {
            uint32_t avail = d->written - d->rx_next;
            d->rxi = 0; d->rxn = avail < BATCH ? avail : BATCH;
            for (uint32_t i = 0; i < d->rxn; i++) d->rxb[i] = d->ring[(d->rx_next + i) & (RING - 1u)];
            d->rx_next += d->rxn;
            if (d->rxn) { gap_reset(d); d->last_byte_us = d->now; }
        }
        go(d, S_CHK, C_CHK * 10u); return;
    case S_CHK: {                                         /* main.c 459-481 */
        bool held = d->rxi < d->rxn;
        if (d->pending_size && !d->watch_sent && bus_reply_abandon(&d->reply, d->now, d->last_byte_us, held)) { d->watch_on = false; d->pending_size = 0; d->pending_fast = false; }
        if (d->pending_size && bus_reply_due(&d->reply, d->now, d->last_byte_us, held, bps)) {
            if (d->reply.kind != BUS_REPLY_DIRECT && watch_stop(d)) { go(d, S_TOP, C_TOP * 10u); return; }
            uint32_t t0 = lus(d, t);
            if (d->reply.kind != BUS_REPLY_DIRECT && (d->written != d->rx_next || line_busy(d, t))) {   /* transmit_if_quiet: deferred */
                d->last_byte_us = lus(d, t); go(d, S_TOP, C_TOP * 10u); return;
            }
            tx_start(d->idx, t, t + rns(d, 30u), d->work, d->pending_size, rns(d, urange(&d->seed, 0u, 290u)));
            d->blocked_until = d->re_off_to + rns(d, 20u);  /* platform_uart_transmit waits for TC and REACK */
            if (d->f4) { d->have = d->total = 0; d->last_byte_us = lus(d, d->blocked_until); }   /* fast10 F-4: hw_micros() after TX */
            bus_reply_sent(&d->reply, t0); d->pending_size = 0;
            if (d->pending_fast) { d->have = d->total = 0; d->skip_left = d->fast_after; d->pending_fast = false; }
            go(d, S_TOP, C_TOP * 10u); return;
        }
        if (!held) { gap_reset(d); go(d, S_TOP, C_TOP * 10u); return; }
        if (d->fix && d->watch_sent) { go(d, S_TOP, C_TOP * 10u); return; }   /* THE FIX (main.c): account first */
        go(d, S_PARSE, byte_cost(d)); return;
    }
    case S_PARSE:
        if (parse_byte(d)) { go(d, S_PROC, urange(&d->seed, 400u, 1200u)); return; }   /* dxl_process: 40..120 us */
        loop_next(d); return;
    case S_PROC:
        if (process_frame(d)) { go(d, S_TOP, C_TOP * 10u); return; }
        loop_next(d); return;
    }
}

static void dev_step(fdev *d, uint64_t t, uint64_t dt)
{
    if (t >= d->next_tick) {                              /* TIM16, 500 us of the servo's clock */
        uint32_t len = urange(&d->seed, ISR_MIN, ISR_MAX) + ((d->tick_k & 7u) == 0u ? ISR_EVERY8 : 0u);
        d->tick_end = t + rns(d, len * 10u); d->next_tick += rns(d, PERIOD_US * 10u); d->tick_k++;
    }
    /* USART receiver timeout (priority 0, delayed by masked sections): once per idle period, RTOR bit times of the
     * servo's own clock after the last received stop bit; the RTOR write at arm re-checks at once */
    if (d->rtor_bits && d->last_rx_end && d->rto_fired_for != d->last_rx_end &&
        t >= d->last_rx_end + rns(d, d->rtor_bits * 10000000u / (bps / 1000u) / 1000u) + d->rto_lat) {
        d->rto_fired_for = d->last_rx_end; d->rto_lat = rns(d, urange(&d->seed, 0u, 200u));
        if (d->watch_on) { d->rto_due = true; d->pendsv_pend = true; }
    }
    if (t < d->tick_end) return;                          /* the tick runs: PendSV and the main loop wait */
    if (d->pendsv_run) { if (d->pendsv_left > dt) { d->pendsv_left -= dt; return; } d->pendsv_run = false; watch_finish(d, t); return; }
    if (d->pendsv_pend) { d->pendsv_pend = false; watch_start(d); return; }
    if (t < d->blocked_until) return;
    if (d->left_ns > dt) { d->left_ns -= dt; return; }
    d->left_ns = 0; thread_action(d, t);
}

/* ------------------------------------------------------------------ host */
typedef struct {
    uint8_t req[64], order[NDEV], buf[1400]; uint16_t req_n, x; unsigned n, got;
    uint64_t next_send, sent_at, last_rx; bool waiting; uint32_t seed;
    unsigned reads, lost, corrupt, missing[NDEV], bad_order;
} host_t;
static host_t host;

static uint16_t fast_request(uint8_t *w, const uint8_t *ids, unsigned n, uint16_t addr, uint16_t x)
{
    uint8_t body[16] = {0x8a, (uint8_t)addr, (uint8_t)(addr >> 8), (uint8_t)x, (uint8_t)(x >> 8)};
    for (unsigned i = 0; i < n; i++) body[5 + i] = ids[i];
    uint16_t at = 7, bn = (uint16_t)(5u + n);
    for (uint16_t i = 0; i < bn; i++) w[at++] = body[i];
    w[0] = 0xff; w[1] = 0xff; w[2] = 0xfd; w[3] = 0; w[4] = 0xfe;
    uint16_t len = (uint16_t)(at - 7 + 2); w[5] = (uint8_t)len; w[6] = (uint8_t)(len >> 8);
    uint16_t crc = dxl_crc(w, at); w[at++] = (uint8_t)crc; w[at++] = (uint8_t)(crc >> 8);
    return at;
}
/* complete valid rows at the start of what arrived (header + ERR 0, ID, data, CRC over every earlier byte) */
static unsigned rows_ok(void)
{
    const unsigned seg = host.x + 4u, len = 1u + host.n * seg;
    if (host.got < 8u || host.buf[0] != 0xff || host.buf[1] != 0xff || host.buf[2] != 0xfd || host.buf[3] || host.buf[4] != 0xfe ||
        host.buf[5] != (uint8_t)len || host.buf[6] != (uint8_t)(len >> 8) || host.buf[7] != 0x55) return 0;
    unsigned k = 0;
    for (; k < host.n && 8u + (k + 1u) * seg <= host.got; k++) {
        const uint8_t *g = host.buf + 8u + k * seg;
        uint16_t crc = dxl_crc(host.buf, (uint16_t)(8u + k * seg + seg - 2u));
        if (g[0] || g[1] != host.order[k] || g[seg - 2u] != (uint8_t)crc || g[seg - 1u] != (uint8_t)(crc >> 8)) break;
    }
    return k;
}
static void host_step(uint64_t t)
{
    if (!host.waiting) {
        if (t < host.next_send) return;
        tx_start(-1, t, t, host.req, host.req_n, 2000u);
        host.waiting = true; host.got = 0; host.sent_at = host.last_rx = t; return;
    }
    const unsigned total = 8u + host.n * (host.x + 4u);
    bool done = host.got >= total, timeout = t - host.last_rx > (uint64_t)HOST_TIMEOUT_US * 1000u;
    if (!done && !timeout) return;
    unsigned k = rows_ok();
    host.reads++;
    if (k < host.n) { host.lost++; host.missing[k]++; }
    else if (host.got != total) host.corrupt++;
    host.waiting = false;
    host.next_send = (done ? host.last_rx : t) + (uint64_t)urange(&host.seed, 500u, 1200u) * 1000u;   /* ~1.5 ms cycle */
}

/* ------------------------------------------------------------------ run */
typedef struct { unsigned reads, lost, corrupt, missing[NDEV]; bool collision; uint32_t seen[NDEV], rows[NDEV], hdr_missed[NDEV], dropped[NDEV], gap[NDEV], armed[NDEV], irq[NDEV], sent[NDEV]; } run_out;
static int g_fake_at = -1;   /* last servo: header-like bytes at this data offset (-1: none) */
static run_out run(const uint8_t *order, unsigned n, uint16_t x, uint32_t rate, int ver, int ppm_slot, int32_t ppm, unsigned reads, uint32_t seed)
{
    bps = rate; nw = 0; collision = false; ndv = n;
    for (unsigned i = 0; i < n; i++) {
        fdev *d = &dv[i]; memset(d, 0, sizeof *d);
        d->id = order[i]; d->idx = (int)i; d->fix = ver >= 9; d->f4 = ver >= 10; d->fake_at = i + 1u == n ? g_fake_at : -1;
        d->seed = mix(seed * 977u + i * 131u + 1u);
        d->ppm = (int)i == ppm_slot ? ppm : 0;
        bus_reply_init(&d->reply);
        d->next_tick = (uint64_t)urange(&d->seed, 0u, 499u) * 1000u; d->tick_end = 0;
        go(d, S_TOP, C_TOP * 10u);
    }
    memset(&host, 0, sizeof host);
    host.seed = mix(seed + 77u); host.n = n; host.x = x; memcpy(host.order, order, n);
    host.req_n = fast_request(host.req, order, n, 96u, x); host.next_send = 1000000u;
    const uint64_t dt = bps == 1000000u ? 500u : 250u;
    for (uint64_t t = 0; host.reads < reads && !collision; t += dt) {
        for (unsigned w = 0; w < nw; w++) {               /* deliver completed bytes */
            wtx *p = &wire[w];
            while (p->done < p->n && p->start + (uint64_t)(p->done + 1u) * p->byte_ns <= t) {
                uint8_t b = p->b[p->done]; uint64_t e = p->start + (uint64_t)(p->done + 1u) * p->byte_ns;
                for (unsigned i = 0; i < ndv; i++) {
                    fdev *d = &dv[i];
                    if ((int)i == p->dev || (e > d->re_off_from && e <= d->re_off_to)) continue;
                    d->ring[d->written++ & (RING - 1u)] = b; d->last_rx_end = e;
                }
                if (p->dev >= 0 && host.waiting && host.got < sizeof host.buf) { host.buf[host.got++] = b; host.last_rx = e; }
                p->done++;
            }
        }
        for (unsigned w = 0; w < nw;) { if (wire[w].done == wire[w].n && t >= wire[w].hold) wire[w] = wire[--nw]; else w++; }
        for (unsigned i = 0; i < ndv; i++) dev_step(&dv[i], t, dt);
        host_step(t);
    }
    run_out o = {.reads = host.reads, .lost = host.lost, .corrupt = host.corrupt, .collision = collision};
    for (unsigned i = 0; i < n; i++) {
        o.missing[i] = host.missing[i]; o.seen[i] = dv[i].seen; o.rows[i] = dv[i].rows; o.hdr_missed[i] = dv[i].hdr_missed;
        o.dropped[i] = dv[i].dropped; o.gap[i] = dv[i].gap_resets; o.armed[i] = dv[i].fast_armed; o.irq[i] = dv[i].irq_started; o.sent[i] = dv[i].reply.fast_sent;
    }
    return o;
}

static void print_run(const char *what, const uint8_t *order, unsigned n, const run_out *o, bool fix)
{
    printf("  %-34s order [", what);
    for (unsigned i = 0; i < n; i++) printf("%s%u", i ? "," : "", order[i]);
    printf("]: %u/%u reads lost rows (first missing row:", o->lost, o->reads);
    for (unsigned i = 0; i < n; i++) printf(" %u", o->missing[i]);
    printf(")");
    if (fix) {
        printf("; per servo seen/rows/hdr_missed/dropped:");
        for (unsigned i = 0; i < n; i++) printf(" %u/%u/%u/%u", o->seen[i], o->rows[i], o->hdr_missed[i], o->dropped[i]);
    }
    printf("; middle row by the watcher %u/%u%s\n", o->irq[1], o->sent[1], o->collision ? " COLLISION" : "");
}

/* The hardware case, old loop: rows lost, and only ever from the middle servo never arming. */
static void old_loop_reproduces_hardware(void)
{
    static const uint8_t orders[2][3] = {{1, 2, 3}, {3, 2, 1}};
    unsigned lost = 0, reads = 0;
    puts("Fast Sync Read 3 x 4 B @1M, back-to-back reads (~1.5 ms cycle), main.c loop BEFORE the fix:");
    for (unsigned o = 0; o < 2; o++) {
        run_out r = run(orders[o], 3, 4u, 1000000u, 8, -1, 0, 15000u, 11u + o);
        print_run("old loop", orders[o], 3, &r, false);
        CHECK(!r.collision && r.corrupt == 0u && r.missing[0] == 0u && r.missing[2] == 0u);   /* only ever the middle row */
        lost += r.lost; reads += r.reads;
    }
    CHECK(lost > 0u);                                     /* the model reproduces the hardware loss */
    printf("  -> %u/%u reads lost (%.3f %%), every one the middle servo not answering (hardware: 7/10000)\n", lost, reads, 100.0 * lost / reads);
}

/* The fix: nothing lost, in every order, with a +-1-2 % clock on any one servo, 4 B and 20 B rows, 1M and 3M. */
static void fixed_loop_loses_nothing(void)
{
    static const uint8_t orders[2][3] = {{1, 2, 3}, {3, 2, 1}};
    static const int32_t ppms[5] = {0, 20000, -20000, 10000, -10000};
    puts("Same, main.c loop WITH the fix (no byte parsed while the watcher's send is unaccounted):");
    for (unsigned o = 0; o < 2; o++) {
        run_out r = run(orders[o], 3, 4u, 1000000u, 10, -1, 0, 15000u, 11u + o);   /* the same seeds as the old loop */
        print_run("fixed loop", orders[o], 3, &r, true);
        CHECK(!r.collision && r.lost == 0u && r.corrupt == 0u);
        for (unsigned i = 0; i < 3; i++) CHECK(r.seen[i] == r.reads && r.rows[i] == r.reads && r.hdr_missed[i] == 0u && r.dropped[i] == 0u);
    }
    for (unsigned p = 1; p < 5; p++) for (unsigned slot = 0; slot < 3; slot++) {
        run_out r = run(orders[0], 3, 4u, 1000000u, 10, (int)slot, ppms[p], 3000u, 100u + p * 7u + slot);
        char what[64]; snprintf(what, sizeof what, "fixed, servo %u clock %+.0f %%", orders[0][slot], ppms[p] / 10000.0);
        print_run(what, orders[0], 3, &r, true);
        CHECK(!r.collision && r.lost == 0u && r.corrupt == 0u);
    }
    static const uint16_t xs[2] = {4u, 20u};
    static const uint32_t rates[2] = {1000000u, 3000000u};
    for (unsigned xi = 0; xi < 2; xi++) for (unsigned ri = 0; ri < 2; ri++) {
        if (xi == 0 && ri == 0) continue;                 /* done above */
        run_out r = run(orders[0], 3, xs[xi], rates[ri], 10, 1, 15000, 5000u, 300u + xi * 3u + ri);
        char what[64]; snprintf(what, sizeof what, "fixed, %u B @%uM, servo 2 +1.5 %%", xs[xi], rates[ri] / 1000000u);
        print_run(what, orders[0], 3, &r, true);
        CHECK(!r.collision && r.lost == 0u && r.corrupt == 0u);
    }
}

/* Where the old loop can and cannot lose a row (the hardware saw 68 B rows clean). */
static void old_loop_by_row_size_and_rate(void)
{
    static const uint8_t order[3] = {1, 2, 3};
    puts("Old loop, other row sizes and rates (the delayed read must land inside the next row's first byte window):");
    run_out r = run(order, 3, 68u, 1000000u, 8, -1, 0, 4000u, 500u);
    print_run("old loop, 68 B @1M", order, 3, &r, false);
    CHECK(!r.collision && r.lost == 0u);                  /* own row 72 B = 720 us: longer than any tick */
    r = run(order, 3, 4u, 3000000u, 8, -1, 0, 10000u, 501u);
    print_run("old loop, 4 B @3M", order, 3, &r, false);
    CHECK(!r.collision && r.missing[0] == 0u && r.missing[2] == 0u);
    r = run(order, 3, 20u, 1000000u, 8, -1, 0, 10000u, 502u);
    print_run("old loop, 20 B (motion block) @1M", order, 3, &r, false);
    CHECK(!r.collision && r.missing[0] == 0u && r.missing[2] == 0u);
}

/* fast10 (review F-4): a row whose TX plus the gap outlasts the 3 ms frame-gap timer (300 B at 1M: 3.04 ms) cleared skip_left in
 * fast9, because nothing is received while we transmit; the parser then hunted inside the next servo's unstuffed row, and
 * data that looks like a header (here FF FF FD 00 01 40 00 near the end of the last row) swallowed the next request. */
static void long_rows_keep_their_skip(void)
{
    static const uint8_t order[3] = {1, 2, 3};
    puts("Fast Sync Read 3 x 300 B @1M, the last row's data ends in a header-like FF FF FD 00 01 40 00 (review F-4):");
    g_fake_at = 290;
    run_out r9 = run(order, 3, 300u, 1000000u, 9, -1, 0, 400u, 700u);
    print_run("fast9 loop", order, 3, &r9, true);
    run_out r10 = run(order, 3, 300u, 1000000u, 10, -1, 0, 400u, 700u);
    print_run("fast10 loop (our TX = line activity)", order, 3, &r10, true);
    g_fake_at = -1;
    CHECK(!r9.collision && r9.lost > 0u);
    CHECK(!r10.collision && r10.lost == 0u && r10.corrupt == 0u);
    for (unsigned i = 0; i < 3; i++) CHECK(r10.seen[i] == r10.reads && r10.rows[i] == r10.reads);
}

int main(void)
{
    old_loop_reproduces_hardware();
    fixed_loop_loses_nothing();
    old_loop_by_row_size_and_rate();
    long_rows_keep_their_skip();
    printf("FW231 Fast Sync Read byte-level loop model: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
