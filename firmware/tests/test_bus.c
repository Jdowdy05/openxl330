/* FW231 six-servo bus: unit tests for bus.c plus a 6-servo wire simulation that
 * runs the REAL protocol.c + bus.c per simulated servo, with the main loop's
 * reply rules (status frames observed, CRC-valid instructions cancel, chained
 * replies), random 2 kHz tick-ISR preemption of every servo's foreground and the
 * ISR-preemptible window between the due decision and the first start bit.
 * Checks: no two packets ever overlap on the wire, replies arrive in row order
 * with the right data, missing servos fall back without collision, a new
 * instruction cancels pending replies. Prints the sync-read cycle times. */
#include "bus.h"
#include "protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks, failures;
#define CHECK(x) do{checks++;if(!(x)){failures++;fprintf(stderr,"%s:%d: FAIL %s\n",__func__,__LINE__,#x);return;}}while(0)

static uint16_t frame(uint8_t *w, uint8_t id, const uint8_t *body, uint16_t n)
{
    uint16_t at = 7;
    for (uint16_t i = 0; i < n; i++) {
        w[at++] = body[i];
        if (at >= 10 && w[at - 3] == 0xff && w[at - 2] == 0xff && w[at - 1] == 0xfd) w[at++] = 0xfd;
    }
    w[0] = 0xff; w[1] = 0xff; w[2] = 0xfd; w[3] = 0; w[4] = id;
    uint16_t len = (uint16_t)(at - 7 + 2); w[5] = (uint8_t)len; w[6] = (uint8_t)(len >> 8);
    uint16_t crc = dxl_crc(w, at); w[at++] = (uint8_t)crc; w[at++] = (uint8_t)(crc >> 8);
    return at;
}

/* ------------------------------------------------------------------ unit */
static void timing_helpers(void)
{
    CHECK(bus_baud_supported(1000000u) && bus_baud_supported(2000000u) && bus_baud_supported(3000000u));
    CHECK(!bus_baud_supported(57600u) && !bus_baud_supported(4000000u) && !bus_baud_supported(0u));
    CHECK(bus_byte_ns(1000000u) == 10000u && bus_byte_ns(2000000u) == 5000u && bus_byte_ns(3000000u) == 3360u);
    CHECK(bus_wire_us(27u, 1000000u) == 270u && bus_wire_us(27u, 3000000u) == 91u && bus_wire_us(1u, 3000000u) == 4u);
    CHECK(bus_idle_us(1000000u) == 30u && bus_idle_us(3000000u) == 20u && bus_idle_us(2000000u) == 20u);
    CHECK(bus_baud_index(1000000u) == 3u && bus_baud_index(2000000u) == 4u && bus_baud_index(3000000u) == 5u);
}

static dxl_result group(uint8_t slot, uint8_t prev, uint32_t prior)
{
    dxl_result r = {27u, true, false, true, slot, false, prev, prior, false, 0u, 0u};
    return r;
}

static void reply_scheduling(void)
{
    bus_reply b; bus_reply_init(&b);
    dxl_result unicast = {11u, true, false, false, 0u, false, 0xffu, 0u, false, 0u, 0u};
    bus_reply_arm(&b, &unicast, 3u, 1000u, 1000000u);
    CHECK(b.kind == BUS_REPLY_DIRECT && bus_reply_due(&b, 1000u, 1000u, false, 1000000u) && b.direct == 0u);
    bus_reply_sent(&b, 1000u);                        /* counted when it really goes out */
    CHECK(!bus_reply_pending(&b) && b.direct == 1u);
    dxl_result slot0 = group(0u, 0xffu, 0u);
    bus_reply_arm(&b, &slot0, 0x82u, 5u, 1000000u);
    CHECK(b.kind == BUS_REPLY_DIRECT && bus_reply_due(&b, 5u, 5u, false, 1000000u));
    /* chained: slot 2 after ID 7; two earlier 27-byte rows -> fallback 540 us + 2*600 us */
    dxl_result s2 = group(2u, 7u, 54u);
    bus_reply_arm(&b, &s2, 0x82u, 100u, 1000000u);
    CHECK(b.kind == BUS_REPLY_CHAIN && b.prev_id == 7u && b.fallback_us == 540u + 2u * BUS_SLOT_LATENCY_US && b.gap_us == BUS_CHAIN_GAP_US);
    CHECK(bus_reply_watching(&b) && !bus_reply_due(&b, 400u, 400u, false, 1000000u));
    bus_observe_status(&b, 5u, 500u);                        /* wrong ID: no trigger */
    CHECK(!b.triggered && b.statuses == 1u && !bus_reply_due(&b, 520u, 500u, false, 1000000u));
    bus_observe_status(&b, 7u, 600u);
    CHECK(b.triggered && !bus_reply_due(&b, 600u + BUS_CHAIN_GAP_US - 1u, 600u, false, 1000000u));   /* predecessor may still hold DE */
    /* Review: traffic after the predecessor's status (unread, or popped within the idle time) holds us */
    CHECK(!bus_reply_due(&b, 600u + BUS_CHAIN_GAP_US, 600u, true, 1000000u));
    CHECK(!bus_reply_due(&b, 600u + BUS_CHAIN_GAP_US, 600u + BUS_CHAIN_GAP_US - 10u, false, 1000000u));
    CHECK(bus_reply_due(&b, 600u + BUS_CHAIN_GAP_US, 600u, false, 1000000u) && b.chained == 0u);
    bus_reply_sent(&b, 600u + BUS_CHAIN_GAP_US + 7u);
    CHECK(!bus_reply_pending(&b) && b.lat_chain_max == BUS_CHAIN_GAP_US + 7u && b.chained == 1u && b.fallback == 0u);
    bus_reply_arm(&b, &unicast, 2u, 1000u, 1000000u);              /* latency maxima: direct, direct while armed */
    bus_reply_sent(&b, 1400u);
    bus_reply_arm(&b, &unicast, 2u, 2000u, 1000000u);
    bus_reply_sent(&b, 2250u);
    CHECK(b.lat_direct_max == 400u && b.lat_chain_max == BUS_CHAIN_GAP_US + 7u);
    /* predecessor finishes PAST our deadline (we deferred while it sent): still the full chain gap,
     * never the 30 us idle fallback (hardware 2026-11-20: 42/200 replies took that path). Gap 150 us here (knob
     * 1080) so the chain gap and the idle time differ; the 30 us default equals the 1M idle time. */
    b.gap_min_us = 150u;
    bus_reply_arm(&b, &s2, 0x82u, 100u, 1000000u);
    uint32_t dl = 100u + b.fallback_us;
    CHECK(!bus_reply_due(&b, dl + 10u, dl + 5u, false, 1000000u) && b.deferrals == 1u);   /* it is still sending */
    bus_observe_status(&b, 7u, dl + 200u);
    CHECK(b.late_chains == 1u && b.pred_done_max == b.fallback_us + 200u);
    CHECK(!bus_reply_due(&b, dl + 240u, dl + 200u, false, 1000000u));                       /* idle 40 us, gap not over */
    CHECK(!bus_reply_due(&b, dl + 200u + 150u - 1u, dl + 200u, false, 1000000u));
    CHECK(bus_reply_due(&b, dl + 200u + 150u, dl + 200u, false, 1000000u));
    bus_reply_sent(&b, dl + 200u + 150u);
    CHECK(b.chained == 2u && b.fallback == 0u);
    b.gap_min_us = 0u;
    b.deferrals = 0u;
    /* fallback: predecessor never answers; deadline deferred while the bus is busy */
    bus_reply_arm(&b, &s2, 0x82u, 100u, 1000000u);
    uint32_t deadline = 100u + b.fallback_us;
    CHECK(!bus_reply_due(&b, deadline - 1u, 0u, false, 1000000u));
    CHECK(!bus_reply_due(&b, deadline, deadline - 5u, false, 1000000u) && b.deferrals == 1u);
    CHECK(!bus_reply_due(&b, deadline + 10u, deadline + 2u, false, 1000000u) && b.deferrals == 1u); /* counted once */
    /* bytes still unread in the RX ring (foreground was held by the tick ISR): busy, even though the
     * last POPPED byte is old (hardware 2026-11-20, bus2-03: 81 replies released this way) */
    CHECK(!bus_reply_due(&b, deadline + 400u, deadline + 2u, true, 1000000u) && b.fallback == 0u);
    CHECK(bus_reply_due(&b, deadline + 40u, deadline + 2u, false, 1000000u) && b.fallback == 0u);
    bus_reply_done(&b);                                      /* e.g. transmit deferred by the line check: not counted */
    CHECK(b.fallback == 0u);
    bus_reply_arm(&b, &s2, 0x82u, 100u, 1000000u);
    CHECK(bus_reply_due(&b, deadline + 40u, deadline + 2u, false, 1000000u));
    bus_reply_sent(&b, deadline + 40u);
    CHECK(b.fallback == 1u && b.chained == 2u);
    /* a new instruction cancels a pending group reply, never a direct one */
    bus_reply_arm(&b, &s2, 0x92u, 0u, 3000000u);
    CHECK(b.gap_us == BUS_CHAIN_GAP_US && b.fallback_us == bus_wire_us(54u, 3000000u) + 2u * BUS_SLOT_LATENCY_US);
    CHECK(bus_observe_instruction(&b) && !bus_reply_pending(&b) && b.superseded == 1u);
    bus_reply_arm(&b, &unicast, 2u, 0u, 1000000u);
    CHECK(!bus_observe_instruction(&b) && bus_reply_pending(&b));
    bus_reply_done(&b);
    /* broadcast ping: stock ID * 3 ms, no chaining */
    dxl_result ping = {14u, true, false, true, 6u, false, 0xffu, 0u, false, 0u, 0u};
    bus_reply_arm(&b, &ping, 1u, 0u, 1000000u);
    CHECK(b.kind == BUS_REPLY_SLOT && b.fallback_us == 18000u && !bus_reply_watching(&b));
    bus_observe_status(&b, 5u, 15000u);
    CHECK(!bus_reply_due(&b, 17999u, 0u, false, 1000000u) && bus_reply_due(&b, 18000u, 0u, false, 1000000u));
    bus_reply_sent(&b, 18000u);
    CHECK(b.slotted == 1u);
    /* wrap-safe */
    bus_reply_arm(&b, &s2, 0x82u, 0xfffffff0u, 1000000u);
    CHECK(!bus_reply_due(&b, 0x10u, 0u, false, 1000000u));
    CHECK(bus_reply_due(&b, 0xfffffff0u + b.fallback_us, 0xfffffff0u, false, 1000000u));
}

static void baud_machine(void)
{
    bus_baud b; bus_baud_init(&b);
    CHECK(b.current == 1000000u && bus_baud_word(&b) == 1000000u && !bus_baud_switch_ready(&b));
    CHECK(bus_baud_request(&b, 57600u) == 4u && bus_baud_request(&b, 4000000u) == 4u && b.state == BUS_BAUD_STABLE);
    CHECK(bus_baud_request(&b, 1000000u) == 0u && b.state == BUS_BAUD_STABLE);          /* already there */
    CHECK(bus_baud_request(&b, 3000000u) == 0u && bus_baud_switch_ready(&b) && b.target == 3000000u);
    CHECK(bus_baud_request(&b, 2000000u) == 7u);                                         /* in progress */
    bus_baud_switched(&b, true, 100u);
    CHECK(b.current == 3000000u && b.previous == 1000000u && bus_baud_word(&b) == 3000001u);
    CHECK(!bus_baud_revert_due(&b, 100u + BUS_BAUD_CONFIRM_US - 1u) && bus_baud_request(&b, 1000000u) == 7u);
    bus_baud_frame(&b);
    CHECK(b.state == BUS_BAUD_STABLE && bus_baud_word(&b) == 3000000u && b.confirms == 1u);
    CHECK(!bus_baud_revert_due(&b, 100u + 3u * BUS_BAUD_CONFIRM_US));
    /* 3M -> 2M not confirmed: reverts to 3M (the previous working baud), not to 1M */
    CHECK(bus_baud_request(&b, 2000000u) == 0u);
    bus_baud_switched(&b, true, 5000u);
    CHECK(bus_baud_revert_due(&b, 5000u + BUS_BAUD_CONFIRM_US));
    bus_baud_reverted(&b, false, 5000u + BUS_BAUD_CONFIRM_US);                            /* UART busy: retry later */
    CHECK(b.state == BUS_BAUD_UNCONFIRMED && !bus_baud_revert_due(&b, 5000u + BUS_BAUD_CONFIRM_US + 1u));
    bus_baud_reverted(&b, true, 5000u + 2u * BUS_BAUD_CONFIRM_US);
    CHECK(b.state == BUS_BAUD_STABLE && b.current == 3000000u && b.reverts == 1u && b.switches == 2u);
    /* a failed hardware switch leaves the old baud */
    CHECK(bus_baud_request(&b, 1000000u) == 0u);
    bus_baud_switched(&b, false, 0u);
    CHECK(b.state == BUS_BAUD_STABLE && b.current == 3000000u);
}

static void put_block(uint8_t *b, uint8_t id, const char *label)
{
    static const uint8_t base[8] = {0x58, 0x4c, 0x44, 0x43, 1, 0xff, 0xff, 0xff};
    memcpy(b, base, 8); b[5] = id; memset(b + 8, 0, 20);
    if (label) memcpy(b + 8, label, strlen(label));
    uint16_t crc = dxl_crc(b, 28u); b[28] = (uint8_t)crc; b[29] = (uint8_t)(crc >> 8); b[30] = b[31] = 0xff;
}

static void device_block(void)
{
    /* The release default (also checked against the linked image by tests/check_image.py). */
    static const uint8_t release[32] = {0x58,0x4c,0x44,0x43,0x01,0xff,0xff,0xff, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0xb7,0x7e,0xff,0xff};
    uint8_t st = 9;
    CHECK(bus_devcfg_resolve(release, 1u, &st) == 1u && st == BUS_DEVCFG_DEFAULT);
    CHECK(bus_devcfg_boot_baud(release) == 1000000u);              /* v1 public image: NV-journal ID, always starts at 1 Mbaud */
    uint8_t b[32];
    put_block(b, 4u, "elbow-left");
    CHECK(bus_devcfg_resolve(b, 1u, &st) == 4u && st == BUS_DEVCFG_OVERRIDE);
    put_block(b, 0u, NULL);
    CHECK(bus_devcfg_resolve(b, 1u, &st) == 0u && st == BUS_DEVCFG_OVERRIDE);
    put_block(b, 252u, NULL);
    CHECK(bus_devcfg_resolve(b, 1u, &st) == 252u && st == BUS_DEVCFG_OVERRIDE);
    put_block(b, 253u, NULL);
    CHECK(bus_devcfg_resolve(b, 1u, &st) == 1u && st == BUS_DEVCFG_INVALID);
    CHECK(bus_devcfg_boot_baud(b) == 1000000u);                    /* an invalid block: NV-journal ID and 1 Mbaud too */
    put_block(b, 5u, "x"); b[12] ^= 1u;                      /* label bit flip: CRC */
    CHECK(bus_devcfg_resolve(b, 2u, &st) == 2u && st == BUS_DEVCFG_INVALID);
    put_block(b, 5u, NULL); b[30] = 0u;                      /* reserved byte */
    CHECK(bus_devcfg_resolve(b, 2u, &st) == 2u && st == BUS_DEVCFG_INVALID);
    put_block(b, 5u, NULL); b[4] = 2u; { uint16_t c = dxl_crc(b, 28u); b[28] = (uint8_t)c; b[29] = (uint8_t)(c >> 8); }
    CHECK(bus_devcfg_resolve(b, 2u, &st) == 2u && st == BUS_DEVCFG_INVALID);  /* unknown layout */
    memset(b, 0xff, 32);                                     /* erased flash */
    CHECK(bus_devcfg_resolve(b, 3u, &st) == 3u && st == BUS_DEVCFG_ABSENT);
    CHECK(bus_devcfg_resolve(NULL, 3u, &st) == 3u && st == BUS_DEVCFG_ABSENT);
    /* start-up baud (byte 6): 0xFF/3 = 1M, 4 = 2M, 5 = 3M; anything else invalidates the block */
    CHECK(bus_devcfg_boot_baud(release) == 1000000u);
    static const uint32_t expect[7] = {0, 0, 0, 1000000u, 2000000u, 3000000u, 0};
    for (uint8_t code = 2u; code <= 6u; code++) {
        put_block(b, 7u, "j"); b[6] = code; uint16_t c = dxl_crc(b, 28u); b[28] = (uint8_t)c; b[29] = (uint8_t)(c >> 8);
        uint8_t id = bus_devcfg_resolve(b, 1u, &st);
        if (expect[code]) CHECK(id == 7u && st == BUS_DEVCFG_OVERRIDE && bus_devcfg_boot_baud(b) == expect[code]);
        else CHECK(id == 1u && st == BUS_DEVCFG_INVALID && bus_devcfg_boot_baud(b) == 1000000u);
    }
    put_block(b, 7u, "j"); b[6] = 5u;                        /* 3M but stale CRC: ignored, 1M */
    CHECK(bus_devcfg_boot_baud(b) == 1000000u);
}

/* ---- Fast Sync Read 0x8A: the ROBOTIS e-manual example (Protocol 2.0, Fast Sync Read): Present Position
 * (132, 4 B) of ID3 = 166, ID7 = 2079, ID4 = 1023. Exact bytes for every servo's part. */
static uint32_t fake_pos;
static uint8_t fake_read(void *ctx, uint16_t a, uint16_t n, uint8_t *out)
{
    (void)ctx; if (a != 132u || n != 4u) return 4u;
    for (unsigned i = 0; i < 4u; i++) out[i] = (uint8_t)(fake_pos >> (8 * i));
    return 0u;
}
static void fast_sync_read_manual_example(void)
{
    static const uint8_t req[17] = {0xFF,0xFF,0xFD,0x00,0xFE,0x0A,0x00,0x8A,0x84,0x00,0x04,0x00,0x03,0x07,0x04,0x20,0xF2};
    static const uint8_t id3[16] = {0xFF,0xFF,0xFD,0x00,0xFE,0x19,0x00,0x55,0x00,0x03,0xA6,0x00,0x00,0x00,0x84,0x08};
    static const uint8_t id7[8] = {0x00,0x07,0x1F,0x08,0x00,0x00,0x16,0xCA};
    static const uint8_t id4[8] = {0x00,0x04,0xFF,0x03,0x00,0x00,0xD1,0x9E};
    CHECK(dxl_crc(req, 15u) == 0xF220u);
    uint8_t wire[32], out[64]; dxl_io io = {.firmware = 231, .model = 1200, .read = fake_read};
    /* ID 3, slot 0: header + segment + CRC, exactly as the manual */
    memcpy(wire, req, 17); io.id = 3; fake_pos = 166u;
    dxl_result r = dxl_process(&io, wire, 17, out, sizeof out);
    CHECK(r.valid_for_us && r.fast && r.group_reply && r.response_slot == 0u && r.reply_bytes == 16u && !memcmp(out, id3, 16));
    /* ID 7, slot 1: segment, CRC sealed from the verified 16-byte prefix */
    memcpy(wire, req, 17); io.id = 7; fake_pos = 2079u;
    r = dxl_process(&io, wire, 17, out, sizeof out);
    CHECK(r.fast && r.response_slot == 1u && r.prev_id == 3u && r.prior_bytes == 16u && r.fast_len == 25u && r.fast_seg == 8u && r.reply_bytes == 8u);
    bus_fast f; bus_fast_begin(&f, r.prior_bytes, r.fast_seg, r.fast_len, r.prev_id);
    for (unsigned i = 0; i < 15u; i++) CHECK(bus_fast_feed(&f, id3[i]) == BUS_FAST_CAPTURING);
    CHECK(bus_fast_feed(&f, id3[15]) == BUS_FAST_READY);
    bus_fast_seal(&f, out, 8u);
    CHECK(!memcmp(out, id7, 8));
    /* ID 4, slot 2: prefix = ID3 part + ID7 part */
    memcpy(wire, req, 17); io.id = 4; fake_pos = 1023u;
    r = dxl_process(&io, wire, 17, out, sizeof out);
    CHECK(r.response_slot == 2u && r.prev_id == 7u && r.prior_bytes == 24u);
    bus_fast_begin(&f, 24u, 8u, 25u, 7u);
    for (unsigned i = 0; i < 16u; i++) bus_fast_feed(&f, id3[i]);
    for (unsigned i = 0; i < 8u; i++) bus_fast_feed(&f, id7[i]);
    CHECK(f.state == BUS_FAST_READY);
    bus_fast_seal(&f, out, 8u);
    CHECK(!memcmp(out, id4, 8));
    uint8_t whole[32]; memcpy(whole, id3, 16); memcpy(whole + 16, id7, 8); memcpy(whole + 24, out, 8);
    CHECK(dxl_crc(whole, 30u) == (uint16_t)(whole[30] | (whole[31] << 8)));   /* last segment CRC = packet CRC */
    /* a follower stays silent on anything wrong before its segment */
    for (unsigned bad = 0; bad < 24u; bad++) {
        uint8_t pre[24]; memcpy(pre, id3, 16); memcpy(pre + 16, id7, 8); pre[bad] ^= 0x01u;
        bus_fast_begin(&f, 24u, 8u, 25u, 7u);
        for (unsigned i = 0; i < 24u; i++) bus_fast_feed(&f, pre[i]);
        CHECK(f.state == BUS_FAST_FAILED);
    }
    bus_fast_begin(&f, 20u, 8u, 25u, 3u);                     /* need not on a row boundary: refuse */
    CHECK(f.state == BUS_FAST_FAILED);
    bus_fast_begin(&f, 16u, 8u, 33u, 3u);                     /* our list had 4 rows (LEN 33); the packet says 3 */
    for (unsigned i = 0; i < 16u; i++) bus_fast_feed(&f, id3[i]);
    CHECK(f.state == BUS_FAST_FAILED);
    bus_fast_begin(&f, 24u, 8u, 25u, 3u);                     /* expected predecessor ID 3 at slot 1: ID 7 heard */
    for (unsigned i = 0; i < 16u; i++) bus_fast_feed(&f, id3[i]);
    for (unsigned i = 0; i < 8u; i++) bus_fast_feed(&f, id7[i]);
    CHECK(f.state == BUS_FAST_FAILED);
    /* duplicate ID in the list: silent */
    static const uint8_t dup_body[9] = {0x8A, 0x84, 0x00, 0x04, 0x00, 0x03, 0x07, 0x03};
    uint8_t dup[32]; uint16_t dn = frame(dup, 0xfe, dup_body, 8); io.id = 3;
    r = dxl_process(&io, dup, dn, out, sizeof out);
    CHECK(r.valid_for_us && r.reply_bytes == 0u);
    /* scheduler: a FAST follower waits for its verified prefix, then the chain gap; never falls back */
    bus_reply b; bus_reply_init(&b);
    dxl_result fr = {8u, true, false, true, 1u, false, 3u, 16u, true, 25u, 8u};
    bus_reply_arm(&b, &fr, 0x8Au, 1000u, 1000000u);
    CHECK(b.kind == BUS_REPLY_FAST && bus_reply_watching(&b) && b.fallback_us == 160u + BUS_SLOT_LATENCY_US);
    CHECK(!bus_reply_due(&b, 1000u + b.fallback_us + 100u, 0u, false, 1000000u));    /* no fallback */
    uint32_t dl = 1000u + b.fallback_us;
    CHECK(!bus_reply_abandon(&b, dl - 1u, 0u, false));
    CHECK(!bus_reply_abandon(&b, dl, 0u, true));                            /* unread bytes: check the prefix first */
    CHECK(!bus_reply_abandon(&b, dl, dl - BUS_CHAIN_GAP_US + 1u, false));    /* line active within the gap */
    CHECK(bus_reply_abandon(&b, dl, dl - BUS_CHAIN_GAP_US, false));
    CHECK(!bus_reply_pending(&b) && b.fast_dropped == 1u);
    bus_reply_arm(&b, &fr, 0x8Au, 1000u, 1000000u);
    bus_reply_trigger(&b, 1300u);
    CHECK(!bus_reply_due(&b, 1300u + BUS_CHAIN_GAP_US - 1u, 1300u, false, 1000000u) && bus_reply_due(&b, 1300u + BUS_CHAIN_GAP_US, 1300u, false, 1000000u));
    bus_reply_sent(&b, 1300u + BUS_CHAIN_GAP_US);
    CHECK(b.fast_sent == 1u && !bus_reply_abandon(&b, 0xffffffu, 0u, false));
    b.gap_min_us = 20u; bus_reply_arm(&b, &fr, 0x8Au, 0u, 1000000u);               /* RAM test gap (write 1080) */
    CHECK(b.gap_us == 20u);
    b.gap_min_us = 2u; bus_reply_arm(&b, &fr, 0x8Au, 0u, 1000000u);
    CHECK(b.gap_us == 10u);                                                         /* never below one byte time */
}

/* Fast start: the USART-IRQ watcher (bus_watch) over the bytes after the request. */
static uint16_t status_frame(uint8_t *w, uint8_t id, const uint8_t *data, uint16_t n)
{
    uint8_t body[300] = {0x55, 0x00}; memcpy(body + 2, data, n);
    return frame(w, id, body, (uint16_t)(n + 2u));
}
static uint8_t watch_run(bus_watch *w, const uint8_t *b, uint16_t n, uint16_t *ready_at)
{
    uint8_t st = BUS_WATCH_CAPTURING;
    for (uint16_t i = 0; i < n; i++) { st = bus_watch_feed(w, b[i]); if (st != BUS_WATCH_CAPTURING) { *ready_at = (uint16_t)(i + 1u); return st; } }
    *ready_at = n; return st;
}
static void fast_start_watcher(void)
{
    uint8_t wire[600], d1[16], d2[16]; uint16_t at = 0, end1, r;
    for (unsigned i = 0; i < 16u; i++) { d1[i] = (uint8_t)(i * 7u); d2[i] = (i % 3u) == 2u ? 0xfd : 0xff; }   /* d2: stuffed */
    end1 = at = status_frame(wire, 1, d1, 16);
    at = (uint16_t)(at + status_frame(wire + at, 2, d2, 16));
    CHECK(at > end1 + 27u);                                       /* ID 2's status really is stuffed */
    bus_watch w;
    bus_watch_begin(&w, BUS_REPLY_CHAIN, 2u, NULL);                /* we are slot 2, after ID 1 and ID 2 */
    CHECK(watch_run(&w, wire, at, &r) == BUS_WATCH_READY && r == at);
    bus_watch_begin(&w, BUS_REPLY_CHAIN, 1u, NULL);                /* slot 1: ready right after ID 1 */
    CHECK(watch_run(&w, wire, at, &r) == BUS_WATCH_READY && r == end1);
    bus_watch_begin(&w, BUS_REPLY_CHAIN, 3u, NULL);                /* predecessor not on the wire yet */
    CHECK(watch_run(&w, wire, at, &r) == BUS_WATCH_CAPTURING && r == at);
    uint8_t bad[600]; memcpy(bad, wire, at); bad[end1 + 12] ^= 0x10u;   /* ID 2's data hit: never ready */
    bus_watch_begin(&w, BUS_REPLY_CHAIN, 2u, NULL);
    CHECK(watch_run(&w, bad, at, &r) == BUS_WATCH_CAPTURING);
    memcpy(bad, wire, at); bad[end1] = 0x00u;                       /* not back to back: the foreground decides */
    bus_watch_begin(&w, BUS_REPLY_CHAIN, 2u, NULL);
    CHECK(watch_run(&w, bad, at, &r) == BUS_WATCH_DEAD && r == end1 + 1u);
    uint8_t ping[1] = {1}; uint16_t pn = frame(bad, 9, ping, 1);    /* a new instruction went by */
    bus_watch_begin(&w, BUS_REPLY_CHAIN, 2u, NULL);
    CHECK(watch_run(&w, bad, pn, &r) == BUS_WATCH_DEAD && r == pn);
    bad[5] = 0x01; bad[6] = 0x00;                                   /* LEN 1: impossible */
    bus_watch_begin(&w, BUS_REPLY_CHAIN, 2u, NULL);
    CHECK(watch_run(&w, bad, pn, &r) == BUS_WATCH_DEAD);
    /* FAST: a copy of the follower's verifier (e-manual bytes, ID 7 after ID 3) */
    static const uint8_t id3[16] = {0xFF,0xFF,0xFD,0x00,0xFE,0x19,0x00,0x55,0x00,0x03,0xA6,0x00,0x00,0x00,0x84,0x08};
    bus_fast f; bus_fast_begin(&f, 16u, 8u, 25u, 3u);
    bus_watch_begin(&w, BUS_REPLY_FAST, 3u, &f);
    CHECK(watch_run(&w, id3, 16, &r) == BUS_WATCH_READY && r == 16u && f.count == 0u);   /* the original is untouched */
    uint8_t seg[8] = {0x00, 0x07, 0x1F, 0x08, 0x00, 0x00, 0, 0};
    bus_fast_seal(&w.fast, seg, 8u);
    CHECK(seg[6] == 0x16 && seg[7] == 0xCA);
    uint8_t noisy[17] = {0x00}; memcpy(noisy + 1, id3, 16);        /* line noise before the header is skipped */
    bus_watch_begin(&w, BUS_REPLY_FAST, 3u, &f);
    CHECK(watch_run(&w, noisy, 17, &r) == BUS_WATCH_READY);
    memcpy(noisy, id3, 16); noisy[15] ^= 1u;
    bus_watch_begin(&w, BUS_REPLY_FAST, 3u, &f);
    CHECK(watch_run(&w, noisy, 16, &r) == BUS_WATCH_DEAD);
}

/* bus_watch_run (span runner used in PendSV) == bus_watch_feed byte by byte: same decision at the same byte,
 * for chained status streams (stuffing, wrong IDs, bad CRC, an instruction) and Fast Sync Read prefixes
 * (corrupted bytes), split at every kind of chunk boundary, across the ring wrap. */
static uint32_t mix(uint32_t x);
static void watch_run_matches_feed(void)
{
    static uint8_t ring[1024]; uint8_t stream[600];
    for (uint32_t trial = 0; trial < 3000u; trial++) {
        uint32_t x = mix(trial * 7919u + 1u); uint16_t n = 0; bool fast = trial & 1u;
        bus_fast f; uint8_t prev = 0;
        if (!fast) {                                           /* 1..4 status frames, random IDs/data */
            unsigned frames = 1u + (x = mix(x)) % 4u;
            for (unsigned k = 0; k < frames; k++) {
                uint8_t d[40]; unsigned dn = 1u + (x = mix(x)) % 30u;
                for (unsigned j = 0; j < dn; j++) d[j] = (x = mix(x)) % 3u == 0u ? 0xff : (uint8_t)x;
                if ((x = mix(x)) % 5u == 0u) { d[0] = 0xff; d[1] = 0xff; if (dn > 2u) d[2] = 0xfd; }
                uint8_t id = (uint8_t)(1u + (x = mix(x)) % 6u); prev = id;
                n = (uint16_t)(n + status_frame(stream + n, id, d, (uint16_t)dn));
            }
            if ((x = mix(x)) % 7u == 0u) { uint8_t p1[1] = {1}; n = (uint16_t)(n + frame(stream + n, 9, p1, 1)); }
            if ((x = mix(x)) % 6u == 0u) prev = (uint8_t)(1u + (x = mix(x)) % 6u);
        } else {                                               /* header + k rows of seg bytes, sealed */
            unsigned rows = 2u + (x = mix(x)) % 4u, xs = 1u + (x = mix(x)) % 20u, seg = xs + 4u, k = 1u + (x = mix(x)) % (rows - 1u);
            uint16_t len = (uint16_t)(1u + rows * seg);
            uint8_t head[8] = {0xff, 0xff, 0xfd, 0, 0xfe, (uint8_t)len, (uint8_t)(len >> 8), 0x55};
            memcpy(stream, head, 8); n = 8;
            for (unsigned r = 0; r < k; r++) {
                stream[n++] = 0; stream[n++] = (uint8_t)(10u + r);
                for (unsigned j = 0; j < xs; j++) stream[n++] = (x = mix(x)) % 4u == 0u ? 0xff : (uint8_t)x;
                uint16_t c = dxl_crc(stream, n); stream[n++] = (uint8_t)c; stream[n++] = (uint8_t)(c >> 8);
            }
            prev = (uint8_t)(10u + k - 1u);
            bus_fast_begin(&f, 8u + k * seg, (uint16_t)seg, len, (x = mix(x)) % 8u == 0u ? (uint8_t)(prev + 1u) : prev);
        }
        if ((x = mix(x)) % 4u == 0u) { uint32_t pos = (x = mix(x)) % n, bit = (x = mix(x)) % 8u; stream[pos] ^= (uint8_t)(1u << bit); }   /* a hit */
        bus_watch a, b; bus_watch_begin(&a, fast ? BUS_REPLY_FAST : BUS_REPLY_CHAIN, prev, fast ? &f : NULL); b = a;
        uint8_t sa = BUS_WATCH_CAPTURING; uint16_t at_a = n;
        for (uint16_t i = 0; i < n; i++) if ((sa = bus_watch_feed(&a, stream[i])) != BUS_WATCH_CAPTURING) { at_a = (uint16_t)(i + 1u); break; }
        uint32_t base = 1024u - (x = mix(x)) % 200u, at = base, end = base;  /* across the ring wrap */
        for (uint16_t i = 0; i < n; i++) ring[(base + i) & 1023u] = stream[i];
        uint8_t sb = BUS_WATCH_CAPTURING;
        while (end != base + n && sb == BUS_WATCH_CAPTURING) {
            uint32_t step = 1u + (x = mix(x)) % 40u; end = end + step > base + n ? base + n : end + step;
            sb = bus_watch_run(&b, ring, 1023u, &at, end);
        }
        CHECK(sa == sb && (sa == BUS_WATCH_CAPTURING ? at == base + n : at == base + at_a));
        if (fast && sa == BUS_WATCH_READY) CHECK(a.fast.crc == b.fast.crc && a.fast.count == b.fast.count);
    }
}
static void crc_table_matches_bit_loop(void)
{
    uint32_t x = 12345u;
    for (unsigned t = 0; t < 20000u; t++) {
        uint8_t d[48]; unsigned n = (x = mix(x + t)) % 48u;
        for (unsigned i = 0; i < n; i++) d[i] = (uint8_t)(x = mix(x));
        uint16_t ref = 0;
        for (unsigned i = 0; i < n; i++) { ref ^= (uint16_t)(d[i] << 8); for (unsigned k = 0; k < 8u; k++) ref = (uint16_t)((ref & 0x8000u) ? (ref << 1) ^ 0x8005u : ref << 1); }
        CHECK(dxl_crc(d, (uint16_t)n) == ref);
    }
}

static void put_lease(uint8_t *b, uint32_t ms, uint32_t act, uint32_t magic)
{
    for (unsigned i = 0; i < 4; i++) { b[i] = (uint8_t)(ms >> (8 * i)); b[4 + i] = (uint8_t)(act >> (8 * i)); b[8 + i] = (uint8_t)(magic >> (8 * i)); }
}

static void command_lease(void)
{
    bus_lease l = {0}; uint8_t b[12];
    CHECK(l.state == BUS_LEASE_OFF && !bus_lease_expire(&l, 0xffffffffu, true));      /* default: off, never expires */
    put_lease(b, 200u, BUS_LEASE_ACT_OFF, BUS_LEASE_MAGIC);
    CHECK(bus_lease_config(&l, b, 11u, 0u) == 5u && bus_lease_config(&l, b, 12u, 1000u) == 0u && l.state == BUS_LEASE_RUNNING);
    put_lease(b, 60001u, 0u, BUS_LEASE_MAGIC); CHECK(bus_lease_config(&l, b, 12u, 0u) == 4u);
    put_lease(b, 100u, 2u, BUS_LEASE_MAGIC); CHECK(bus_lease_config(&l, b, 12u, 0u) == 4u);
    put_lease(b, 100u, 0u, 0x12345678u); CHECK(bus_lease_config(&l, b, 12u, 0u) == 4u);
    CHECK(l.ms == 200u && l.last_us == 1000u);                                         /* rejected writes change nothing */
    CHECK(!bus_lease_expire(&l, 1000u + 199999u, true));
    bus_lease_renew(&l, 150000u);                                                     /* accepted command */
    CHECK(!bus_lease_expire(&l, 349999u, true) && !bus_lease_expire(&l, 400000u, false));   /* OFF: never expires */
    CHECK(bus_lease_expire(&l, 350000u, true) && l.state == BUS_LEASE_EXPIRED && l.expiries == 1u);
    CHECK(!bus_lease_expire(&l, 900000u, true) && l.expiries == 1u);                  /* fires once */
    bus_lease_renew(&l, 900000u);
    CHECK(l.state == BUS_LEASE_EXPIRED && !bus_lease_blocks_motion(&l));              /* only re-arm recovers; OFF response does not block */
    bus_lease_rearm(&l, 1000000u);
    CHECK(l.state == BUS_LEASE_RUNNING && !bus_lease_expire(&l, 1199999u, true) && bus_lease_expire(&l, 1200000u, true));
    put_lease(b, 50u, BUS_LEASE_ACT_HOLD, BUS_LEASE_MAGIC); CHECK(bus_lease_config(&l, b, 12u, 2000000u) == 0u);
    CHECK(bus_lease_expire(&l, 2050000u, true) && bus_lease_blocks_motion(&l));
    /* fast10 (review F-2): while holding, a new target is refused and ARM only re-arms; disarmed, nothing is blocked */
    CHECK(bus_lease_gate(&l, 256u, false, true) == 7u && bus_lease_gate(&l, 260u, false, true) == 7u && bus_lease_gate(&l, 64u, true, true) == 1u);
    CHECK(bus_lease_gate(&l, 328u, false, true) == 0u && bus_lease_gate(&l, 64u, false, true) == 0u);
    /* fast16: the torque commands are motion commands too (gate and renewal share bus_lease_motion); their neighbours are not */
    for (unsigned a = 0u; a < 2048u; ++a) CHECK(bus_lease_motion((uint16_t)a) == (a == 256u || a == 260u || a == 264u || a == 268u));
    CHECK(bus_lease_gate(&l, 264u, false, true) == 7u && bus_lease_gate(&l, 268u, false, true) == 7u);
    CHECK(bus_lease_gate(&l, 252u, false, true) == 0u && bus_lease_gate(&l, 272u, false, true) == 0u && bus_lease_gate(&l, 258u, false, true) == 0u);
    CHECK(bus_lease_gate(&l, 256u, false, false) == 0u && bus_lease_gate(&l, 260u, false, false) == 0u && bus_lease_gate(&l, 64u, true, false) == 0u);
    put_lease(b, 0u, 0u, BUS_LEASE_MAGIC); CHECK(bus_lease_config(&l, b, 12u, 0u) == 0u && l.state == BUS_LEASE_OFF && !bus_lease_blocks_motion(&l));
    bus_lease_rearm(&l, 5u); CHECK(l.state == BUS_LEASE_OFF);                         /* switched off stays off */
    put_lease(b, 10u, 0u, BUS_LEASE_MAGIC); CHECK(bus_lease_config(&l, b, 12u, 0xffffff00u) == 0u);
    CHECK(!bus_lease_expire(&l, 0x2000u, true) && bus_lease_expire(&l, 0xffffff00u + 10000u, true));   /* wrap-safe */
}

static void boot_window(void)
{
    bus_baud b; bus_baud_init(&b);
    bus_baud_boot(&b, 1000000u, 0u);                          /* default: nothing armed */
    CHECK(b.boot_state == BUS_BOOT_NONE && !bus_baud_wants_frames(&b) && !bus_baud_boot_due(&b, 10000000u));
    bus_baud_init(&b); bus_baud_boot(&b, 3000000u, 100u);    /* no host at 1M: switch after the window */
    CHECK(bus_baud_wants_frames(&b) && !bus_baud_boot_due(&b, 100u + BUS_RESCUE_US - 1u));
    CHECK(bus_baud_boot_due(&b, 100u + BUS_RESCUE_US));
    bus_baud_boot_done(&b, false);                            /* UART busy: still due */
    CHECK(b.boot_state == BUS_BOOT_WAITING && b.current == 1000000u && bus_baud_boot_due(&b, 100u + BUS_RESCUE_US + 5u));
    bus_baud_boot_done(&b, true);
    CHECK(b.boot_state == BUS_BOOT_SWITCHED && b.current == 3000000u && bus_baud_word(&b) == 3000000u && !bus_baud_wants_frames(&b));
    bus_baud_frame(&b);
    CHECK(b.boot_state == BUS_BOOT_SWITCHED && b.current == 3000000u);
    bus_baud_init(&b); bus_baud_boot(&b, 2000000u, 0u);      /* a 1M frame in the window: rescued, stays 1M */
    bus_baud_frame(&b);
    CHECK(b.boot_state == BUS_BOOT_RESCUED && !bus_baud_boot_due(&b, 50000000u) && b.current == 1000000u && !bus_baud_wants_frames(&b));
    CHECK(bus_baud_request(&b, 3000000u) == 0u && bus_baud_switch_ready(&b));   /* runtime 844 still works */
    bus_baud_init(&b); bus_baud_boot(&b, 4000000u, 0u);      /* unsupported: nothing armed */
    CHECK(b.boot_state == BUS_BOOT_NONE);
    bus_baud_init(&b); bus_baud_boot(&b, 3000000u, 0xfffff000u);   /* wrap-safe */
    CHECK(!bus_baud_boot_due(&b, 0x10u) && bus_baud_boot_due(&b, 0xfffff000u + BUS_RESCUE_US));
}

/* ------------------------------------------------------------ simulation */
enum { NDEV = 8, MAXPKT = 64, PERIOD_NS = 500000 };
typedef struct { uint64_t start, end, hold; int dev; uint16_t n; uint8_t bytes[700]; } packet;   /* hold: DE released */
typedef struct {
    uint8_t id; bool present;
    bus_reply r; uint8_t out[DXL_REPLY_CAP]; uint16_t out_n;
    uint64_t isr_phase, loop_ns, busy_until, last_run, last_rx; uint32_t seed; unsigned seen;
    unsigned isr_max_us, isr_min_us;
    uint8_t pattern;
    unsigned late_min_us, late_max_us, hold_max_us;   /* extra start latency of direct replies; DE release delay */
    unsigned stall_us;                                /* foreground blocked this long right after each request */
    unsigned quiet_deferrals;
    unsigned bidx, idle_policy;                       /* RX cursor byte; idle task policy (see dev_run) */
    uint64_t byte_cost_ns, idle_cost_ns, process_cost_ns, pred_end, parse_lag_max, succ_wait_max;
    uint64_t stall_until;
    unsigned replies;
    bus_fast fw; bool fast_reply; uint8_t corrupt; unsigned fast_bad, gap_us;   /* Fast Sync Read follower (main.c) */
    bus_watch w; bool fast_start, watch_on, watch_sent; unsigned wseen, wbidx, irq_started, irq_lat_max_us;   /* fast start */
    unsigned stall_after_us;                          /* foreground busy this long right AFTER processing a request */
    uint64_t rto_seen, watch_tx_ns, irq_wait_max;
    /* firmware cost model, calibrated on hardware fsr-02 (2026-11-20): one main-loop iteration costs iter_cost_ns
     * (hw_micros, lease/fault/watchdog/baud checks, the masked RX read) and takes up to `batch` bytes
     * (1 = FW231 up to 27ca17a7, 32 = now), each byte_cost_ns to parse. The USART-IRQ watcher costs irq_byte_ns
     * per byte (4.7 us with the per-byte divisions of 27ca17a7, ~0.8 us now) with irq_tick_bytes per control
     * period (0 = no budget, the 27ca17a7 behaviour with a 160-byte cap per IRQ). */
    uint64_t iter_cost_ns, irq_byte_ns, tick_max_ns, irq_max_ns, fast_byte_ns; unsigned batch, irq_tick_bytes, backlog_max;
    uint64_t cur_tick, cur_tick_irq_ns; unsigned cur_tick_bytes;
    bool old;                                         /* 27ca17a7: Fast Sync Read abandon on time alone */
    bool watch_low;                                   /* watcher in PendSV, below the tick (now); else priority 0 */
} sdev;
static packet pk[MAXPKT]; static unsigned npk;
static sdev dv[NDEV]; static unsigned ndv;
static uint32_t sim_bps;

static uint32_t mix(uint32_t x) { x ^= x >> 16; x *= 0x7feb352du; x ^= x >> 15; x *= 0x846ca68bu; x ^= x >> 16; return x; }
static uint64_t isr_len(const sdev *d, uint64_t k)
{
    uint32_t span = d->isr_max_us - d->isr_min_us + 1u;
    return (uint64_t)(d->isr_min_us + mix(d->seed ^ (uint32_t)k * 2654435761u) % span) * 1000u;
}
/* End of the tick ISR covering t, or 0 if the foreground can run at t. */
static uint64_t isr_end(const sdev *d, uint64_t t)
{
    if (t < d->isr_phase) return 0;
    uint64_t k = (t - d->isr_phase) / PERIOD_NS, s = d->isr_phase + k * PERIOD_NS, e = s + isr_len(d, k);
    return t < e ? e : 0;
}

static uint8_t sim_read(void *ctx, uint16_t a, uint16_t n, uint8_t *out)
{
    sdev *d = ctx;
    for (uint16_t i = 0; i < n; i++)
        out[i] = d->pattern ? ((i % 3u) == 2u ? 0xfd : 0xff) : (uint8_t)(d->id * 16u + a + i);
    return 0;
}
static uint8_t sim_write(void *ctx, uint16_t a, uint16_t n, const uint8_t *in) { (void)ctx; (void)a; (void)n; (void)in; return 0; }

static void wire_add(int dev, uint64_t start, const uint8_t *b, uint16_t n)
{
    packet *p = &pk[npk++];
    p->dev = dev; p->start = start; p->n = n; memcpy(p->bytes, b, n);
    p->end = start + (uint64_t)n * bus_byte_ns(sim_bps);
    p->hold = p->end;
}

static bool frame_crc_ok(const uint8_t *f, uint16_t n) { return n >= 10u && dxl_crc(f, (uint16_t)(n - 2u)) == (uint16_t)(f[n - 2u] | (f[n - 1u] << 8)); }

/* One foreground loop iteration of servo d at time t (mirrors main.c FW231). */
/* Next unread received byte available at time tm? (cursor = packet d->seen, byte d->bidx; own packets are
 * never received: RE is off during our TX) */
static bool byte_ready(sdev *d, uint64_t tm)
{
    while (d->seen < npk && pk[d->seen].dev == (int)(d - dv)) { d->seen++; d->bidx = 0; }
    return d->seen < npk && pk[d->seen].start + (uint64_t)(d->bidx + 1u) * bus_byte_ns(sim_bps) <= tm;
}
static bool line_busy(sdev *d, uint64_t tm)              /* ISR.BUSY: a foreign byte is on the wire right now */
{
    for (unsigned i = 0; i < npk; i++) if (pk[i].dev != (int)(d - dv) && pk[i].start <= tm && tm < pk[i].end) return true;
    return false;
}

static bool take_byte(sdev *d, uint64_t t, uint32_t now, uint64_t *cost);
static unsigned ready_from(sdev *d, unsigned seen, unsigned bidx, uint64_t t)   /* received after a cursor */
{
    unsigned n = 0; int me = (int)(d - dv);
    for (unsigned i = seen; i < npk && n < 4000u; i++) {
        if (pk[i].dev == me || pk[i].start >= t) continue;
        uint64_t got = (t - pk[i].start) / bus_byte_ns(sim_bps); if (got > pk[i].n) got = pk[i].n;
        unsigned skip = i == seen ? bidx : 0u;
        if (got > skip) n += (unsigned)got - skip;
    }
    return n;
}
static unsigned ready_bytes(sdev *d, uint64_t t) { return ready_from(d, d->seen, d->bidx, t); }   /* not yet parsed */
/* One iteration of the firmware main loop: the reply due-check runs BEFORE this iteration takes a byte;
 * then exactly one received byte is consumed (byte_cost_ns of foreground time), or, with nothing to read,
 * the idle task (stack watermark scan, idle_cost_ns) may run. idle_policy 0 = firmware before 2026-11-20
 * evening (scan whenever the ring is empty), 1 = now (only when the bus has been quiet for 2 ms, no
 * partial frame and no pending reply). */
static void dev_run(sdev *d, uint64_t t)
{
    uint32_t now = (uint32_t)(t / 1000u);
    if (d->watch_sent) {                                      /* main.c top of loop: the USART IRQ sent our reply */
        d->watch_sent = false;
        if (d->out_n) {
            uint32_t tx = (uint32_t)(d->watch_tx_ns / 1000u);
            if (!d->r.triggered) bus_reply_trigger(&d->r, tx - d->r.gap_us);
            bus_reply_sent(&d->r, tx); d->out_n = 0;
        }
    }
    bool pending = byte_ready(d, t);
    if (d->out_n && !d->watch_sent && bus_reply_abandon(&d->r, now, d->old ? now - 1000000u : (uint32_t)(d->last_rx / 1000u), d->old ? false : pending)) { d->watch_on = false; d->out_n = 0; }   /* Fast Sync Read: predecessor never completed */
    if (d->out_n && bus_reply_due(&d->r, now, (uint32_t)(d->last_rx / 1000u), pending, sim_bps)) {
        uint8_t kind = d->r.kind;
        if (kind != BUS_REPLY_DIRECT) { d->watch_on = false; if (d->watch_sent) return; }   /* watch_stop(): claim */
        /* platform_uart_transmit: RE off / REACK (~2 us), then DE + DMA under PRIMASK.
         * The tick ISR can still preempt before the masked section. */
        uint64_t go = t + 2000u, e = isr_end(d, go);
        if (!e) for (uint64_t s = t; s < go; s += 500u) if ((e = isr_end(d, s))) break;
        if (e) go = e + 2000u;
        if (kind != BUS_REPLY_DIRECT) {                       /* platform_uart_transmit_if_quiet at the claim */
            uint64_t claim = go - 1000u;
            if (byte_ready(d, claim) || line_busy(d, claim)) {
                d->last_rx = claim; d->busy_until = claim; d->quiet_deferrals++; return;   /* deferred: parse first */
            }
        }
        bus_reply_sent(&d->r, now);
        if (d->corrupt && d->fast_reply) d->out[d->out_n - 3u] ^= 0x40u;   /* test: a predecessor whose data got hit */
        go += 1000u;                                          /* DE lead + first start bit */
        if (kind == BUS_REPLY_DIRECT && d->late_max_us)       /* e.g. a busy slot-0 servo (FW230, armed) */
            go += (uint64_t)(d->late_min_us + mix(d->seed ^ d->replies * 7919u) % (d->late_max_us - d->late_min_us + 1u)) * 1000u;
        if ((kind == BUS_REPLY_CHAIN || kind == BUS_REPLY_FAST) && d->pred_end) {
            uint64_t wait = go > d->pred_end ? go - d->pred_end : 0u;  /* predecessor's last byte -> our start */
            if (wait > d->succ_wait_max) d->succ_wait_max = wait;
        }
        wire_add((int)(d - dv), go, d->out, d->out_n);
        if (d->hold_max_us)                                   /* TC IRQ held off by a masked tick section */
            pk[npk - 1].hold += (uint64_t)(mix(d->seed + d->replies * 104729u) % (d->hold_max_us + 1u)) * 1000u;
        d->replies++;
        d->busy_until = pk[npk - 1].hold + 1000u;             /* blocking TX until TC */
        d->out_n = 0;
        return;                                               /* firmware: continue; pop next iteration */
    }
    if (!pending) {                                           /* nothing to read: maybe the idle task */
        bool idle = d->idle_policy == 0 ? true : (d->bidx == 0u && !d->out_n && t - d->last_rx >= 2000000u);
        if (d->idle_cost_ns && idle) d->busy_until = t + d->idle_cost_ns;
        return;
    }
    packet *p = &pk[d->seen];
    if (d->stall_us && p->dev < 0 && p->bytes[7] != 0x55u && d->bidx == 0u && d->stall_until < p->end) {
        d->stall_until = p->end + (uint64_t)d->stall_us * 1000u;   /* busy elsewhere: request still unread */
        d->busy_until = d->stall_until; return;
    }
    unsigned backlog = ready_bytes(d, t);
    if (backlog > d->backlog_max) d->backlog_max = backlog;
    d->last_rx = t;
    uint64_t cost = d->iter_cost_ns;
    for (unsigned k = 0; k < d->batch && byte_ready(d, t); k++) {   /* main.c inner loop over one masked read */
        cost += d->byte_cost_ns;
        if (take_byte(d, t, now, &cost)) break;               /* a request was processed: back to the top */
    }
    if (d->stall_after_us && d->out_n && d->busy_until > t + cost) return;
    d->busy_until = t + cost;
}

/* One received byte through the firmware parser (main.c). True when it completed a request. */
static bool take_byte(sdev *d, uint64_t t, uint32_t now, uint64_t *cost)
{
    packet *p = &pk[d->seen];
    d->bidx++;
    uint8_t byte = p->bytes[d->bidx - 1u];                    /* main.c: Fast Sync Read follower check */
    if (d->out_n && d->r.kind == BUS_REPLY_FAST && !d->r.triggered) {
        *cost += d->fast_byte_ns;                             /* the foreground's verifier */
        uint8_t st = bus_fast_feed(&d->fw, byte);
        if (st == BUS_FAST_READY) {
            bus_fast_seal(&d->fw, d->out, d->out_n); bus_reply_trigger(&d->r, now);
            uint64_t end = p->start + (uint64_t)d->bidx * bus_byte_ns(sim_bps); d->pred_end = end;
            if (t - end > d->parse_lag_max) d->parse_lag_max = t - end;
        } else if (st == BUS_FAST_FAILED) { d->watch_on = false; if (!d->watch_sent) { d->out_n = 0; bus_reply_done(&d->r); d->fast_bad++; } }
    }
    if (d->bidx < p->n) return false;                         /* frame not complete yet */
    d->seen++; d->bidx = 0;
    uint8_t f[700]; memcpy(f, p->bytes, p->n);
    if (p->n < 10u) return false;                             /* e.g. a Fast Sync Read segment: never a frame */
    if (f[7] == 0x55u) {
        if (frame_crc_ok(f, p->n)) {
            bool was = d->r.triggered;
            bus_observe_status(&d->r, f[4], now);
            if (!was && d->r.triggered) {
                uint64_t lag = t - p->end; d->pred_end = p->end;   /* wire end -> status parsed */
                if (lag > d->parse_lag_max) d->parse_lag_max = lag;
            }
        }
        return false;
    }
    if (d->out_n && frame_crc_ok(f, p->n)) { d->watch_on = false; d->watch_sent = false; if (bus_observe_instruction(&d->r)) d->out_n = 0; }
    dxl_io io = {.id = d->id, .firmware = 231, .model = 1200, .context = d, .read = sim_read, .write = sim_write};
    dxl_result res = dxl_process(&io, f, p->n, d->out, sizeof d->out);
    if (!res.valid_for_us) return false;
    *cost += d->process_cost_ns;                              /* dxl_process + reply build */
    d->out_n = res.reply_bytes; d->pred_end = 0;
    if (d->out_n) bus_reply_arm(&d->r, &res, p->bytes[7], now, sim_bps); else bus_reply_done(&d->r);
    d->fast_reply = d->out_n && res.fast;
    if (d->fast_reply && res.response_slot) bus_fast_begin(&d->fw, res.prior_bytes, res.fast_seg, res.fast_len, res.prev_id);
    if (d->fast_start && d->out_n && (d->r.kind == BUS_REPLY_CHAIN || d->r.kind == BUS_REPLY_FAST)) {   /* arm the IRQ watcher */
        bus_watch_begin(&d->w, d->r.kind, d->r.prev_id, &d->fw);
        d->wseen = d->seen; d->wbidx = d->bidx; d->watch_on = true; d->watch_sent = false;
        d->rto_seen = 0;   /* main.c writes RTOR here: RM0444, a value <= the running counter sets RTOF at once (late arm) */
    }
    if (d->stall_after_us && d->out_n) d->busy_until = t + (uint64_t)d->stall_after_us * 1000u;
    return true;
}

/* Fast start (main.c platform_uart_rto): the USART receiver timeout fires once per idle period, gap_us after
 * the last foreign byte, whatever the foreground is doing (priority 0; up to irq_lat_max_us late from
 * PRIMASK sections). The watcher reads the RX ring from its own cursor; it may start the reply only when it
 * is READY on the last byte received. Bounded at WATCH_MAX bytes per run, like main.c. */
enum { SIM_WATCH_MAX = 160 };
/* DE released up to this late after a reply's last stop bit (TC IRQ held off by a masked section). Hardware:
 * masked sections 13..18 us during group reads (fsr3) and 28 us while armed (fsr4: the control tick's output
 * phase); a 10 us chain gap was clean in 2000 reads per step at 1M and 3M (servos OFF). Default gap 40 us.
 * (FW231 up to fsr3 modelled 120 us against a 150 us gap.) */
enum { SIM_DE_HOLD_US = 30 };
#ifndef SIM_OLD_ITER
#define SIM_OLD_ITER 16000u   /* ns per main-loop iteration (calibrated on fsr-02: backlog 70..73 B, predecessor 3.3..3.4 ms) */
#endif
static void irq_rto(sdev *d, uint64_t t)
{
    if (!d->watch_on) return;
    int me = (int)(d - dv); uint64_t last = 0;
    for (unsigned i = 0; i < npk; i++) {
        if (pk[i].dev == me || pk[i].start >= t) continue;
        if (pk[i].end > t) return;                            /* a frame is still arriving */
        if (pk[i].end > last) last = pk[i].end;
    }
    uint64_t lat = (uint64_t)(2u + mix(d->seed ^ (uint32_t)last) % (d->irq_lat_max_us + 1u)) * 1000u;
    if (!last || last <= d->rto_seen || t < last + (uint64_t)d->r.gap_us * 1000u + lat) return;
    if (d->watch_low && isr_end(d, t)) return;                /* PendSV: only after the control tick has finished */
    d->rto_seen = last;
    uint64_t k = t >= d->isr_phase ? (t - d->isr_phase) / PERIOD_NS : 0u, s0 = d->isr_phase + k * PERIOD_NS;
    if (k != d->cur_tick) { d->cur_tick = k; d->cur_tick_bytes = 0; d->cur_tick_irq_ns = 0; }
    unsigned avail = ready_from(d, d->wseen, d->wbidx, t);
    unsigned room = !d->watch_low && d->irq_tick_bytes ? d->irq_tick_bytes - d->cur_tick_bytes : SIM_WATCH_MAX;
    uint64_t dur = 3000u + (uint64_t)(avail > room ? 0u : avail) * d->irq_byte_ns;
    if (d->watch_low) {                                       /* the next tick preempts PendSV; the main loop waits */
        if (s0 + PERIOD_NS < t + dur) dur += isr_len(d, k + 1u);
        if (t + dur > d->busy_until) d->busy_until = t + dur;
    } else if (t >= s0 && t < s0 + isr_len(d, k) + d->cur_tick_irq_ns) {   /* priority 0: stretches the tick */
        d->cur_tick_irq_ns += dur;
        if (isr_len(d, k) + d->cur_tick_irq_ns > d->tick_max_ns) d->tick_max_ns = isr_len(d, k) + d->cur_tick_irq_ns;
    }
    if (dur > d->irq_max_ns) d->irq_max_ns = dur;
    if (avail > room) { d->watch_on = false; return; }        /* too much: the foreground decides */
    if (!d->watch_low) d->cur_tick_bytes += avail;
    uint8_t st = BUS_WATCH_CAPTURING;
    while (d->wseen < npk && st == BUS_WATCH_CAPTURING) {
        packet *p = &pk[d->wseen];
        if (p->dev == me) { d->wseen++; d->wbidx = 0; continue; }
        if (p->start >= t) break;
        st = bus_watch_feed(&d->w, p->bytes[d->wbidx++]);
        if (d->wbidx >= p->n) { d->wseen++; d->wbidx = 0; }
    }
    if (st == BUS_WATCH_CAPTURING) return;
    d->watch_on = false;
    uint64_t td = d->watch_low ? t + dur : t;                 /* PendSV decides at the end of its run, under PRIMASK */
    while (d->wseen < npk && pk[d->wseen].dev == me) d->wseen++;
    bool more = d->wbidx || (d->wseen < npk && pk[d->wseen].start < td);
    if (st != BUS_WATCH_READY || more || line_busy(d, td)) return;   /* the foreground decides */
    if (d->w.kind == BUS_REPLY_FAST) bus_fast_seal(&d->w.fast, d->out, d->out_n);
    if (d->corrupt && d->fast_reply) d->out[d->out_n - 3u] ^= 0x40u;
    uint64_t go = t + dur + 2000u;                            /* watcher run, then RE off, REACK, DE lead, start bit */
    if (go - last > d->irq_wait_max) d->irq_wait_max = go - last;
    wire_add(me, go, d->out, d->out_n);
    if (d->hold_max_us) pk[npk - 1].hold += (uint64_t)(mix(d->seed + d->replies * 104729u) % (d->hold_max_us + 1u)) * 1000u;
    d->replies++; d->irq_started++;
    d->watch_sent = true; d->watch_tx_ns = go;
}

typedef struct { uint64_t done_ns; bool collision, order_ok, data_ok; unsigned replies; } sim_result;

/* Host sends `req` at t=0, then waits up to `horizon`. Optional second request at t2. */
static sim_result simulate(const uint8_t *req, uint16_t req_n, uint64_t horizon, const uint8_t *req2, uint16_t req2_n, uint64_t t2)
{
    npk = 0;
    for (unsigned i = 0; i < ndv; i++) { bus_reply_init(&dv[i].r); dv[i].r.gap_min_us = dv[i].gap_us; dv[i].out_n = 0; dv[i].busy_until = 0; dv[i].last_run = 0; dv[i].last_rx = 0; dv[i].seen = 0; dv[i].bidx = 0; dv[i].stall_until = 0; dv[i].pred_end = 0; dv[i].watch_on = dv[i].watch_sent = false; dv[i].rto_seen = 0; }
    wire_add(-1, 0, req, req_n);
    bool sent2 = false;
    for (uint64_t t = 0; t < horizon; t += 250u) {
        if (req2 && !sent2 && t >= t2) { wire_add(-1, t, req2, req2_n); sent2 = true; }
        for (unsigned i = 0; i < ndv; i++) {
            sdev *d = &dv[i];
            if (d->present) irq_rto(d, t);
            if (!d->present || t < d->busy_until || isr_end(d, t) || (t % d->loop_ns) != 0u) continue;
            dev_run(d, t);
        }
        if ((t % 5000u) == 0u && (!req2 || sent2)) {        /* settled: nothing pending, everything seen */
            bool settled = true;
            for (unsigned i = 0; i < npk; i++) if (pk[i].end >= t) settled = false;
            for (unsigned i = 0; i < ndv && settled; i++) if (dv[i].present && (dv[i].out_n || dv[i].seen < npk)) settled = false;
            if (settled) break;
        }
    }
    sim_result r = {0, false, true, true, 0};
    for (unsigned i = 0; i < npk; i++) for (unsigned j = i + 1; j < npk; j++)
        if (pk[i].start < pk[j].hold && pk[j].start < pk[i].hold) r.collision = true;   /* two drivers on */
    for (unsigned i = 0; i < npk; i++) if (pk[i].dev >= 0) { r.replies++; if (pk[i].end > r.done_ns) r.done_ns = pk[i].end; }
    return r;
}

static void setup_devices(unsigned n, const uint8_t *ids, uint32_t seed, unsigned isr_min, unsigned isr_max)
{
    ndv = n;
    for (unsigned i = 0; i < n; i++) {
        sdev *d = &dv[i]; memset(d, 0, sizeof *d);
        d->id = ids[i]; d->present = true; d->seed = mix(seed * 31u + i);
        d->isr_phase = (uint64_t)(mix(d->seed + 7u) % 500u) * 1000u;
        d->loop_ns = (1u + mix(d->seed + 3u) % 8u) * 250u;     /* 0.25..2 us loop */
        d->isr_min_us = isr_min; d->isr_max_us = isr_max;
        /* foreground costs (firmware main loop, 64 MHz M0+): ~3 us per received byte, ~150 us to process
         * a request and build its reply, ~80 us stack-watermark scan as the idle task */
        d->byte_cost_ns = 3000u; d->process_cost_ns = 150000u; d->idle_cost_ns = 80000u; d->idle_policy = 1u;
        d->fast_start = true; d->irq_lat_max_us = 20u;                /* FW231 default: fast start on */
        d->batch = 32u; d->iter_cost_ns = SIM_OLD_ITER; d->byte_cost_ns = 1500u;   /* same loop overhead, once per batch */
        /* watcher in PendSV with the span runner (~1.2 us/byte estimated); the main loop's per-byte fast verifier
         * measured 3.3 us/byte on hardware (fsr2-02) */
        d->watch_low = true; d->irq_byte_ns = 1200u; d->irq_tick_bytes = 0u; d->fast_byte_ns = 3300u;
    }
}

/* Replies must be the present servos in row order, each a valid status of its ID. */
static bool replies_ok(const uint8_t *row_ids, unsigned rows, const uint16_t *sizes)
{
    unsigned k = 0;
    for (unsigned i = 0; i < npk; i++) {
        if (pk[i].dev < 0) continue;
        while (k < rows) { bool present = false; for (unsigned j = 0; j < ndv; j++) if (dv[j].id == row_ids[k] && dv[j].present) present = true; if (present) break; k++; }
        if (k >= rows) return false;
        const packet *p = &pk[i];
        if (!frame_crc_ok(p->bytes, p->n) || p->bytes[4] != row_ids[k] || p->bytes[7] != 0x55 || p->bytes[8] != 0) return false;
        uint8_t body[700]; uint16_t m = 0;   /* destuff data */
        for (uint16_t a = 9; a < p->n - 2; a++) { body[m++] = p->bytes[a]; if (m >= 3 && body[m-3] == 0xff && body[m-2] == 0xff && body[m-1] == 0xfd) a++; }
        if (m != sizes[k]) return false;
        k++;
    }
    for (; k < rows; k++) for (unsigned j = 0; j < ndv; j++) if (dv[j].id == row_ids[k] && dv[j].present) return false;
    return true;
}

static uint64_t pct(uint64_t *v, unsigned n, unsigned p)
{
    for (unsigned i = 1; i < n; i++) for (unsigned j = i; j && v[j - 1] > v[j]; j--) { uint64_t x = v[j]; v[j] = v[j - 1]; v[j - 1] = x; }
    return v[(n - 1) * p / 100];
}

static void sync_read_six(void)
{
    static const uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    static const uint32_t rates[3] = {1000000u, 2000000u, 3000000u};
    for (unsigned rate = 0; rate < 3; rate++) {
        sim_bps = rates[rate];
        uint8_t body[16] = {0x82, 64, 0, 16, 0, 1, 2, 3, 4, 5, 6}, req[64];
        uint16_t n = frame(req, 0xfe, body, 11), sizes[6]; for (unsigned i = 0; i < 6; i++) sizes[i] = 16;
        static uint64_t t[400]; unsigned runs = 0, chained = 0, fb = 0;
        for (uint32_t seed = 1; seed <= 400; seed++) {
            setup_devices(6, ids, seed + rate * 1000u, 120u, 350u);
            if (seed % 5u == 0u) dv[2].pattern = 1;          /* FF FF FD data: stuffed (longer) reply */
            sim_result r = simulate(req, n, 30000000u, NULL, 0, 0);
            CHECK(!r.collision && r.replies == 6u && replies_ok(ids, 6, sizes));
            t[runs++] = r.done_ns;
            for (unsigned i = 0; i < 6; i++) { chained += dv[i].r.chained; fb += dv[i].r.fallback; }
        }
        CHECK(fb == 0u && chained == 5u * runs);
        uint64_t p50 = pct(t, runs, 50), p99 = pct(t, runs, 99), mx = pct(t, runs, 100);
        printf("sync read 6 x 16 B @%u: all chained, no collision; cycle median %.2f ms, p99 %.2f ms, max %.2f ms (%u runs)\n",
               rates[rate], p50 / 1e6, p99 / 1e6, mx / 1e6, runs);
        CHECK(mx < (rate == 0 ? 5500000u : 4500000u));   /* chain gap per hop + realistic per-byte/processing costs */
    }
}

static void bulk_read_and_missing(void)
{
    static const uint8_t ids[6] = {11, 12, 13, 14, 15, 16};
    static const uint16_t sizes[6] = {4, 64, 16, 1, 32, 8};
    static const uint32_t rates[2] = {1000000u, 3000000u};
    /* masks of absent servos: none, middle, first two, last, alternate */
    static const uint8_t absent[5] = {0x00, 0x04, 0x03, 0x20, 0x15};
    for (unsigned rate = 0; rate < 2; rate++) for (unsigned a = 0; a < 5; a++) {
        sim_bps = rates[rate];
        uint8_t body[40] = {0x92}, req[80]; unsigned m = 1;
        for (unsigned i = 0; i < 6; i++) { body[m++] = ids[i]; body[m++] = 0; body[m++] = 1; body[m++] = (uint8_t)sizes[i]; body[m++] = 0; }
        uint16_t n = frame(req, 0xfe, body, (uint16_t)m);
        uint64_t worst = 0; unsigned fb = 0;
        for (uint32_t seed = 1; seed <= 150; seed++) {
            setup_devices(6, ids, seed * 7u + a + rate * 100u, 120u, 350u);
            for (unsigned i = 0; i < 6; i++) if (absent[a] & (1u << i)) dv[i].present = false;
            sim_result r = simulate(req, n, 40000000u, NULL, 0, 0);
            unsigned expect = 0; for (unsigned i = 0; i < 6; i++) if (!(absent[a] & (1u << i))) expect++;
            CHECK(!r.collision && r.replies == expect && replies_ok(ids, 6, sizes));
            if (r.done_ns > worst) worst = r.done_ns;
            for (unsigned i = 0; i < 6; i++) fb += dv[i].r.fallback;
        }
        if (absent[a] && absent[a] != 0x20) CHECK(fb > 0u);
        printf("bulk read 6 rows (4/64/16/1/32/8 B) @%u, absent mask 0x%02x: no collision, order/data ok, worst %.2f ms%s\n",
               rates[rate], absent[a], worst / 1e6, fb ? " (fallback used)" : "");
    }
}

static void stress_slow_isr(void)
{
    /* ISR up to 450 us (observed worst 347 us) with a missing servo in the middle. */
    static const uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    sim_bps = 1000000u;
    uint8_t body[16] = {0x82, 64, 0, 68, 0, 1, 2, 3, 4, 5, 6}, req[64];
    uint16_t n = frame(req, 0xfe, body, 11), sizes[6]; for (unsigned i = 0; i < 6; i++) sizes[i] = 68;
    unsigned runs = 0;
    for (uint32_t seed = 1; seed <= 300; seed++) {
        setup_devices(6, ids, seed + 77u, 300u, 450u);
        if (seed & 1u) dv[3].present = false;
        sim_result r = simulate(req, n, 40000000u, NULL, 0, 0);
        CHECK(!r.collision && replies_ok(ids, 6, sizes));
        runs++;
    }
    printf("stress: 68-byte status rows, tick ISR 300..450 us, servo 4 absent in half the runs: %u runs, no collision PASS\n", runs);
}

/* Hardware 2026-11-20: servo ID 1 (older firmware, armed) at slot 0 + ID3 (FW231) collided. Two hazards:
 * a slot-0 servo that starts late, and a predecessor whose driver enable outlasts its last byte. */
static void late_predecessor_and_slow_release(void)
{
    static const uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    static const uint32_t rates[2] = {1000000u, 3000000u};
    for (unsigned rate = 0; rate < 2; rate++) {
        sim_bps = rates[rate];
        uint8_t body[16] = {0x82, 64, 0, 68, 0, 1, 2, 3, 4, 5, 6}, req[64];
        uint16_t n = frame(req, 0xfe, body, 11), sizes[6]; for (unsigned i = 0; i < 6; i++) sizes[i] = 68;
        uint64_t worst = 0; unsigned runs = 0;
        for (uint32_t seed = 1; seed <= 300; seed++) {
            setup_devices(6, ids, seed * 13u + rate, 120u, 350u);
            dv[0].late_min_us = 500u; dv[0].late_max_us = 2500u;               /* slot 0 starts 0.5..2.5 ms late */
            for (unsigned i = 0; i < 6; i++) dv[i].hold_max_us = SIM_DE_HOLD_US;   /* DE released late */
            if (seed % 3u == 0u) dv[2].present = false;                       /* and a missing servo mid-chain */
            sim_result r = simulate(req, n, 60000000u, NULL, 0, 0);
            CHECK(!r.collision && replies_ok(ids, 6, sizes));
            if (r.done_ns > worst) worst = r.done_ns;
            runs++;
        }
        printf("late slot-0 servo (0.5..2.5 ms) + DE release up to %u us + missing servo @%u: %u runs, no collision, worst %.2f ms\n",
               SIM_DE_HOLD_US, rates[rate], runs, worst / 1e6);
    }
}

/* The two orders seen on hardware (servo ID 1 on older firmware at slot 0 + ID3, 2026-11-20, 42/200 fallbacks):
 * (a) the predecessor starts before our deadline but finishes after it: we must chain with the full
 *     gap (old code released ~30 us after its last byte; with a slow DE release that collides);
 * (b) we are stalled until after the predecessor already finished: in-order processing (request,
 *     then its status) must still chain, never wait for the fallback. */
static void predecessor_past_deadline_and_stalled_successor(void)
{
    static const uint8_t ids[2] = {1, 3};
    sim_bps = 1000000u;
    uint8_t body[16] = {0x82, 64, 0, 68, 0, 1, 3}, req[64];
    uint16_t n = frame(req, 0xfe, body, 7), sizes[2] = {68, 68};
    unsigned chained = 0, fallback = 0, late = 0, runs = 0;
    for (uint32_t seed = 1; seed <= 300; seed++) {           /* (a) */
        setup_devices(2, ids, seed * 17u + 3u, 120u, 350u);
        dv[1].fast_start = false;                                /* the foreground path (fast start covered below) */
        dv[0].late_min_us = 2600u; dv[0].late_max_us = 3000u;    /* starts before ID3's 4.01 ms deadline, ends after */
        dv[0].hold_max_us = SIM_DE_HOLD_US;
        sim_result r = simulate(req, n, 30000000u, NULL, 0, 0);
        CHECK(!r.collision && replies_ok(ids, 2, sizes));
        chained += dv[1].r.chained; fallback += dv[1].r.fallback; late += dv[1].r.late_chains; runs++;
    }
    CHECK(fallback == 0u && chained == runs && late > 0u);
    printf("predecessor finishing past our deadline: %u runs, %u chained (%u past the deadline), 0 fallback, no collision\n", runs, chained, late);
    chained = fallback = runs = 0;
    for (uint32_t seed = 1; seed <= 300; seed++) {           /* (b) */
        setup_devices(2, ids, seed * 29u + 5u, 120u, 350u);
        dv[1].fast_start = false;
        dv[1].stall_us = 2500u;                                  /* ID3 busy until servo ID 1 is long done */
        dv[0].hold_max_us = SIM_DE_HOLD_US;
        sim_result r = simulate(req, n, 30000000u, NULL, 0, 0);
        CHECK(!r.collision && replies_ok(ids, 2, sizes));
        chained += dv[1].r.chained; fallback += dv[1].r.fallback; runs++;
    }
    CHECK(fallback == 0u && chained == runs);
    printf("successor stalled until the predecessor finished: %u runs, all chained, 0 fallback\n", runs);
}

/* Hardware 2026-11-20 (bus2-06/07, both FW231): successors parsed the predecessor's status up to 4.5 ms
 * after the request although it ended on the wire by ~1.8 ms; chained_max stayed <= 0.51 ms, so the
 * delay was BEFORE the trigger. Cause: one byte per main-loop iteration, and the ~80 us stack-watermark
 * scan ran whenever the RX ring was momentarily empty, i.e. between bytes of the status being received.
 * Design target: no successor waits more than ~1 ms after the chain gap. */
static void idle_task_vs_status_parsing(void)
{
    static const uint8_t ids[2] = {1, 3};
    sim_bps = 1000000u;
    uint8_t body[16] = {0x82, 64, 0, 68, 0, 1, 3}, req[64];
    uint16_t n = frame(req, 0xfe, body, 7), sizes[2] = {68, 68};
    uint64_t lag[2] = {0, 0}, wait[2] = {0, 0};
    for (unsigned policy = 0; policy < 2; policy++) {
        for (uint32_t seed = 1; seed <= 300; seed++) {
            setup_devices(2, ids, seed * 37u + 11u, 230u, 260u);
            for (unsigned i = 0; i < 2; i++) { dv[i].idle_policy = policy; dv[i].fast_start = false; }
            sim_result r = simulate(req, n, 30000000u, NULL, 0, 0);
            CHECK(!r.collision && replies_ok(ids, 2, sizes));
            if (dv[1].parse_lag_max > lag[policy]) lag[policy] = dv[1].parse_lag_max;
            if (dv[1].succ_wait_max > wait[policy]) wait[policy] = dv[1].succ_wait_max;
        }
        printf("idle task %s: successor parsed the predecessor's status up to %.2f ms after it ended; reply started up to %.2f ms after it\n",
               policy ? "only on a quiet bus (now)" : "whenever the RX ring is empty (before)", lag[policy] / 1e6, wait[policy] / 1e6);
    }
    /* NOTE: with these cost estimates the simulation does NOT reproduce the 2.7 ms hardware lag (both
     * policies stay well under 1 ms), so the stack scan is not shown to be the cause; read 1064 (48 B)
     * now measures foreground stalls and RX backlog on hardware. The target is still enforced here. */
    CHECK(wait[1] <= (uint64_t)(BUS_CHAIN_GAP_US + 1000u) * 1000u);   /* target: <= gap + 1 ms */
}

/* Fast Sync Read 0x8A: everything the servos put on the wire, concatenated, must be ONE valid packet:
 * header, then `segments` rows (ERR 0, ID, data, CRC over every byte before it). */
static bool fast_packet_ok(const uint8_t *row_ids, unsigned rows, uint16_t x, unsigned segments, unsigned bad_from)
{
    static uint8_t all[4096]; unsigned m = 0;
    for (unsigned i = 0; i < npk; i++) if (pk[i].dev >= 0) { memcpy(all + m, pk[i].bytes, pk[i].n); m += pk[i].n; }
    const unsigned seg = x + 4u, len = 1u + rows * seg;
    if (m != (segments ? 8u + segments * seg : 0u)) return false;
    if (!segments) return true;
    if (all[0] != 0xff || all[1] != 0xff || all[2] != 0xfd || all[3] || all[4] != 0xfe || all[5] != (uint8_t)len || all[6] != (uint8_t)(len >> 8) || all[7] != 0x55) return false;
    for (unsigned k = 0; k < segments; k++) {
        const uint8_t *g = all + 8u + k * seg; uint8_t want[256];
        const sdev *d = NULL; for (unsigned j = 0; j < ndv; j++) if (dv[j].id == row_ids[k]) d = &dv[j];
        if (!d || g[0] != 0 || g[1] != row_ids[k]) return false;
        sim_read((void *)d, 64u, x, want);
        if (k < bad_from && memcmp(g + 2, want, x)) return false;
        uint16_t crc = dxl_crc(all, (uint16_t)(8u + k * seg + seg - 2u));
        if (k < bad_from && (g[seg - 2u] != (uint8_t)crc || g[seg - 1u] != (uint8_t)(crc >> 8))) return false;
    }
    return true;
}
static uint16_t fast_request(uint8_t *req, const uint8_t *ids, unsigned n, uint16_t x)
{
    uint8_t body[16] = {0x8a, 64, 0, (uint8_t)x, (uint8_t)(x >> 8)};
    for (unsigned i = 0; i < n; i++) body[5 + i] = ids[i];
    return frame(req, 0xfe, body, (uint16_t)(5u + n));
}

static void fast_sync_read_six(void)
{
    static const uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    static const uint32_t rates[3] = {1000000u, 2000000u, 3000000u};
    static const uint16_t widths[2] = {4u, 16u};
    for (unsigned rate = 0; rate < 3; rate++) for (unsigned w = 0; w < 2; w++) {
        sim_bps = rates[rate];
        uint8_t req[64]; uint16_t n = fast_request(req, ids, 6, widths[w]);
        static uint64_t t[300]; unsigned runs = 0, sent = 0;
        for (uint32_t seed = 1; seed <= 300; seed++) {
            setup_devices(6, ids, seed * 3u + rate * 1000u + w, 120u, 350u);
            if (seed % 4u == 0u) dv[2].pattern = 1;          /* FF FF FD in the data: unstuffed in a fast packet */
            sim_result r = simulate(req, n, 30000000u, NULL, 0, 0);
            CHECK(!r.collision && r.replies == 6u && fast_packet_ok(ids, 6, widths[w], 6, 6));
            for (unsigned i = 1; i < 6; i++) { sent += dv[i].r.fast_sent; CHECK(dv[i].fast_bad == 0u && dv[i].r.fast_dropped == 0u); }
            t[runs++] = r.done_ns;
        }
        CHECK(sent == 5u * runs);
        uint64_t p50 = pct(t, runs, 50), mx = pct(t, runs, 100);
        printf("fast sync read 6 x %u B @%u: one valid packet, no collision; cycle median %.2f ms, max %.2f ms (%u runs)\n",
               widths[w], rates[rate], p50 / 1e6, mx / 1e6, runs);
    }
}

static void fast_missing_corrupt_and_gap(void)
{
    static const uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    sim_bps = 1000000u;
    uint8_t req[64]; uint16_t n = fast_request(req, ids, 6, 4u);
    unsigned dropped = 0, bad = 0, runs = 0;
    for (uint32_t seed = 1; seed <= 200; seed++) {          /* servo 3 missing: 1, 2 answer; 4..6 stay silent */
        setup_devices(6, ids, seed * 11u + 1u, 120u, 350u);
        for (unsigned i = 0; i < 6; i++) dv[i].hold_max_us = SIM_DE_HOLD_US;
        dv[2].present = false;
        sim_result r = simulate(req, n, 40000000u, NULL, 0, 0);
        CHECK(!r.collision && r.replies == 2u && fast_packet_ok(ids, 6, 4u, 2, 2));
        for (unsigned i = 3; i < 6; i++) dropped += dv[i].r.fast_dropped;
        runs++;
    }
    CHECK(dropped == 3u * runs);
    printf("fast sync read, servo 3 missing: %u runs, rows 1-2 answer, rows 4-6 silent (abandoned), no collision\n", runs);
    runs = 0;
    for (uint32_t seed = 1; seed <= 200; seed++) {          /* servo 2's data hit on the wire: 3..6 must not append */
        setup_devices(6, ids, seed * 19u + 2u, 120u, 350u);
        dv[1].corrupt = 1;
        sim_result r = simulate(req, n, 40000000u, NULL, 0, 0);
        CHECK(!r.collision && r.replies == 2u && fast_packet_ok(ids, 6, 4u, 2, 1));
        for (unsigned i = 2; i < 6; i++) bad += dv[i].fast_bad;
        runs++;
    }
    CHECK(bad == 4u * runs);
    printf("fast sync read, servo 2's segment corrupted: %u runs, rows 3-6 silent (bad prefix), no collision\n", runs);
    /* RAM gap knob (write 1080): a short gap is safe only while every driver releases faster than it */
    static const uint32_t rates[2] = {1000000u, 3000000u};
    for (unsigned rate = 0; rate < 2; rate++) {
        sim_bps = rates[rate];
        n = fast_request(req, ids, 6, 4u);
        static uint64_t t[200]; unsigned collisions = 0; runs = 0;
        for (uint32_t seed = 1; seed <= 200; seed++) {
            setup_devices(6, ids, seed * 23u + rate, 120u, 350u);
            for (unsigned i = 0; i < 6; i++) dv[i].gap_us = 20u;
            sim_result r = simulate(req, n, 30000000u, NULL, 0, 0);
            CHECK(!r.collision && fast_packet_ok(ids, 6, 4u, 6, 6));
            t[runs++] = r.done_ns;
            for (unsigned i = 0; i < 6; i++) dv[i].hold_max_us = 120u;   /* same, with DE released up to 120 us late */
            r = simulate(req, n, 30000000u, NULL, 0, 0);
            collisions += r.collision;
        }
        CHECK(collisions > 0u);                                /* the model sees the hazard the hardware sweep measures */
        printf("fast sync read, 20 us gap @%u: clean with prompt DE release (median %.2f ms); %u/%u runs collide when DE is released up to 120 us late\n",
               rates[rate], pct(t, runs, 50) / 1e6, collisions, runs);
    }
}

/* Fast start vs a lagging foreground (hardware 2026-11-20: successors parsed the predecessor up to ~2.7 ms
 * late). Each successor's foreground is busy for 2.5 ms right after it processed the request (so it parses
 * the predecessor late); DE released up to SIM_DE_HOLD_US late. A successor that arms only after its predecessor
 * already finished is caught by the RTOR write at arm. */
static void fast_start_vs_foreground_lag(void)
{
    static const uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    static const uint32_t rates[2] = {1000000u, 3000000u};
    for (unsigned rate = 0; rate < 2; rate++) for (unsigned fast = 0; fast < 2; fast++) for (unsigned kind = 0; kind < 2; kind++) {
        sim_bps = rates[rate];
        uint8_t body[16] = {0x82, 64, 0, 16, 0, 1, 2, 3, 4, 5, 6}, req[64];
        uint16_t n = kind ? fast_request(req, ids, 6, 4u) : frame(req, 0xfe, body, 11), sizes[6];
        for (unsigned i = 0; i < 6; i++) sizes[i] = 16;
        static uint64_t t[200]; unsigned runs = 0, irq = 0; uint64_t wait = 0, run = 0;
        for (uint32_t seed = 1; seed <= 200; seed++) {
            setup_devices(6, ids, seed * 41u + rate * 7u + kind, 120u, 350u);
            for (unsigned i = 0; i < 6; i++) { dv[i].fast_start = fast; dv[i].hold_max_us = SIM_DE_HOLD_US; if (i) dv[i].stall_after_us = 2500u; }
            sim_result r = simulate(req, n, 40000000u, NULL, 0, 0);
            CHECK(!r.collision && r.replies == 6u && (kind ? fast_packet_ok(ids, 6, 4u, 6, 6) : replies_ok(ids, 6, sizes)));
            for (unsigned i = 1; i < 6; i++) {
                irq += dv[i].irq_started;
                if (dv[i].irq_max_ns > run) run = dv[i].irq_max_ns;
                if (dv[i].irq_wait_max > wait) wait = dv[i].irq_wait_max;
                CHECK(kind ? dv[i].r.fast_sent == 1u : dv[i].r.chained == 1u);
            }
            t[runs++] = r.done_ns;
        }
        /* PendSV waits for a running tick, so now and then the main loop (its stall just over) claims first */
        if (fast) { CHECK(irq * 100u >= 95u * 5u * runs && wait <= (uint64_t)(BUS_CHAIN_GAP_US + 22u + 2u + 1u + 350u) * 1000u + run); }   /* + a tick ISR (PendSV waits); +1 us sim step */
        else CHECK(irq == 0u);
        printf("%s 6 servos, successors' foreground 2.5 ms late @%u, fast start %s: median %.2f ms, max %.2f ms\n",
               kind ? "fast sync read 4 B" : "sync read 16 B", rates[rate], fast ? "ON " : "off", pct(t, runs, 50) / 1e6, pct(t, runs, 100) / 1e6);
        if (fast) printf("    %u/%u successors started by the watcher; predecessor's last byte -> our first start bit <= %.0f us (gap %u us + latency + a tick ISR at most + watcher run <= %.0f us)\n", irq, 5u * runs, wait / 1e3, BUS_CHAIN_GAP_US, run / 1e3);
    }
}

/* Replay of hardware run fsr-02 (2026-11-20, fw231-comp2-fast 27ca17a7, IDs 1+3 at 1 Mbaud, 68-byte rows, tick
 * 230..260 us): the old cost model must reproduce what the hardware showed (successor 62..73 bytes behind,
 * predecessor parsed 2.3..4.8 ms after the request, 71/200 fast rows lost with fast start off, watcher runs up to
 * 379 us); the current one must not. */
enum { M_NOW = 0, M_FAST2, M_FAST };                 /* now, fw231-comp2-fast2 519e2e58 (fsr2-02), -fast 27ca17a7 (fsr-02) */
static void set_model(sdev *d, unsigned m)
{
    if (m == M_FAST) { d->old = true; d->batch = 1u; d->iter_cost_ns = SIM_OLD_ITER; d->byte_cost_ns = 0u; d->fast_byte_ns = 4700u;
                       d->watch_low = false; d->irq_byte_ns = 4700u; d->irq_tick_bytes = 0u; }
    if (m == M_FAST2) { d->watch_low = false; d->irq_byte_ns = 3300u; d->irq_tick_bytes = 96u; }
}
typedef struct { unsigned backlog, lost, runs; uint64_t pred_done, irq_max, tick_max, p50; } replay_out;
static replay_out replay(unsigned old, bool fast_start, bool fast_read)
{
    static const uint8_t ids[2] = {1, 3};
    sim_bps = 1000000u;
    uint8_t body[16] = {0x82, 64, 0, 68, 0, 1, 3}, req[64];
    uint16_t n = fast_read ? fast_request(req, ids, 2, 68u) : frame(req, 0xfe, body, 7), sizes[2] = {68, 68};
    replay_out o = {0}; static uint64_t t[200];
    for (uint32_t seed = 1; seed <= 200; seed++) {
        setup_devices(2, ids, seed * 53u + old * 7u + fast_start * 3u + fast_read, 230u, 260u);
        for (unsigned i = 0; i < 2; i++) { set_model(&dv[i], old); dv[i].fast_start = fast_start; }
        sim_result r = simulate(req, n, 60000000u, NULL, 0, 0);
        if (r.collision || (fast_read ? !fast_packet_ok(ids, 2, 68u, r.replies, r.replies) : !replies_ok(ids, 2, sizes))) { o.runs = 0; return o; }
        if (r.replies < 2u) o.lost++;
        t[o.runs++] = r.done_ns;
        if (dv[1].backlog_max > o.backlog) o.backlog = dv[1].backlog_max;
        if ((uint64_t)dv[1].r.pred_done_max * 1000u > o.pred_done) o.pred_done = (uint64_t)dv[1].r.pred_done_max * 1000u;
        if (dv[1].irq_max_ns > o.irq_max) o.irq_max = dv[1].irq_max_ns;
        if (dv[1].tick_max_ns > o.tick_max) o.tick_max = dv[1].tick_max_ns;
    }
    o.p50 = pct(t, o.runs, 50);
    return o;
}
static void hardware_replay(void)
{
    static const char *name[3] = {"now                ", "519e2e58 (fsr2-02) ", "27ca17a7 (fsr-02)  "};
    for (unsigned m = M_FAST + 1u; m-- > 0u;) {
        for (unsigned fr = 0; fr < 2; fr++) for (unsigned fs = 0; fs < 2; fs++) {
            replay_out o = replay(m, fs, fr);
            CHECK(o.runs == 200u);
            printf("%s %s, fast start %s: <= %u B behind, predecessor done <= %.2f ms, %u/200 rows lost, watcher run <= %.0f us, %s %.0f us, median %.2f ms\n",
                   name[m], fr ? "fast read 2x68 B" : "sync read 2x68 B", fs ? "on " : "off", o.backlog, o.pred_done / 1e6, o.lost, o.irq_max / 1e3,
                   o.tick_max ? "ticks stretched to" : "ticks never stretched,", o.tick_max / 1e3, o.p50 / 1e6);
            if (m == M_NOW) CHECK(o.lost == 0u && o.backlog <= 40u && o.tick_max == 0u);   /* bus work never inside a tick */
            if (m == M_FAST2 && fs) CHECK(o.tick_max > 500000u);   /* the model reproduces fsr2-02's TIMING latch */
        }
    }
}

static void broadcast_ping(void)
{
    static const uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    sim_bps = 1000000u;
    uint8_t body[1] = {1}, req[16]; uint16_t n = frame(req, 0xfe, body, 1);
    for (uint32_t seed = 1; seed <= 50; seed++) {
        setup_devices(6, ids, seed, 120u, 350u);
        sim_result r = simulate(req, n, 30000000u, NULL, 0, 0);
        CHECK(!r.collision && r.replies == 6u);
        for (unsigned i = 0, k = 0; i < npk; i++) if (pk[i].dev >= 0) { CHECK(pk[i].bytes[4] == ids[k] && pk[i].n == 14u); k++; }
        CHECK(r.done_ns < 6u * 3000000u + 1000000u);
    }
    puts("broadcast ping: IDs 1..6 at stock ID*3 ms slots, no collision PASS");
}

static void new_instruction_cancels(void)
{
    /* Servo 1 is absent, so servo 2+ wait for fallback; the host gives up early and
     * pings ID 9 (absent). No stale group reply may start after that ping ends. */
    static const uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    sim_bps = 1000000u;
    uint8_t body[16] = {0x82, 64, 0, 16, 0, 1, 2, 3, 4, 5, 6}, req[64], b2[1] = {1}, req2[16];
    uint16_t n = frame(req, 0xfe, body, 11), n2 = frame(req2, 9, b2, 1);
    for (uint32_t seed = 1; seed <= 100; seed++) {
        setup_devices(6, ids, seed + 5u, 120u, 350u);
        dv[0].present = false;
        sim_result r = simulate(req, n, 20000000u, req2, n2, 300000u);
        unsigned superseded = 0;
        for (unsigned i = 1; i < 6; i++) superseded += dv[i].r.superseded;
        CHECK(r.replies == 0u && !r.collision && superseded == 5u);
    }
    puts("a new instruction cancels every pending group reply (no stale reply after it) PASS");
}

int main(void)
{
    timing_helpers();
    reply_scheduling();
    baud_machine();
    device_block();
    boot_window();
    command_lease();
    crc_table_matches_bit_loop();
    fast_sync_read_manual_example();
    fast_start_watcher();
    watch_run_matches_feed();
    puts("bus.c units: fast-start watcher + span runner == byte feed, Fast Sync Read (e-manual example bytes), nibble CRC, opt-in command lease, start-up baud + 1M rescue window, byte timing, chained/fallback/slot/direct scheduling, supersede, baud confirm-or-revert, device block PASS");
    sync_read_six();
    bulk_read_and_missing();
    stress_slow_isr();
    late_predecessor_and_slow_release();
    predecessor_past_deadline_and_stalled_successor();
    idle_task_vs_status_parsing();
    fast_sync_read_six();
    fast_missing_corrupt_and_gap();
    fast_start_vs_foreground_lag();
    hardware_replay();
    broadcast_ping();
    new_instruction_cancels();
    printf("FW231 bus: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
