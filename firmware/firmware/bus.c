#include "bus.h"

bool bus_baud_supported(uint32_t bps) { return bps == 1000000u || bps == 2000000u || bps == 3000000u; }

/* 10 bits per byte. 3 Mbaud is really 64 MHz * 2 / 43 = 2.977 Mbaud (3.36 us/byte). */
uint32_t bus_byte_ns(uint32_t bps)
{
    if (bps == 1000000u) return 10000u;
    if (bps == 2000000u) return 5000u;
    if (bps == 3000000u) return 3360u;
    return 0u;
}

uint32_t bus_wire_us(uint32_t bytes, uint32_t bps)
{
    /* FW231 fast9: 32-bit (flash; the 64-bit divide of this rounding cost ~30 B). A byte takes <= 10 us, so up to
     * 400000 bytes fit; more saturates (a prior_bytes that large never occurs: 252 rows x 640 B = 161280). */
    if (bytes > 400000u) return UINT32_MAX;
    return (bytes * bus_byte_ns(bps) + 999u) / 1000u;
}

uint32_t bus_idle_us(uint32_t bps)
{
    return bps == 1000000u ? 30u : 20u;   /* 3 byte times, at least 20 us (constant: no divide in the main loop) */
}

uint8_t bus_baud_index(uint32_t bps)
{
    return bps == 3000000u ? 5u : bps == 2000000u ? 4u : 3u;
}

void bus_reply_init(bus_reply *b) { if (b) *b = (bus_reply){.prev_id = 0xffu}; }

void bus_reply_arm(bus_reply *b, const dxl_result *r, uint8_t instruction, uint32_t now, uint32_t bps)
{
    if (!b || !r) return;
    b->triggered = false; b->deferred = false; b->armed_us = now; b->prev_id = 0xffu;
    b->fallback_us = 0u; b->gap_us = 0u; b->trigger_us = 0u;
    uint32_t floor_us = b->gap_min_us ? b->gap_min_us : BUS_CHAIN_GAP_US;
    if (!r->group_reply || r->response_slot == 0u) { b->kind = BUS_REPLY_DIRECT; return; }
    if (instruction == 1u) {                     /* broadcast ping: stock ID * 3 ms */
        b->kind = BUS_REPLY_SLOT;
        b->fallback_us = (uint32_t)r->response_slot * BUS_PING_SLOT_US;
        return;
    }
    b->kind = r->fast ? BUS_REPLY_FAST : BUS_REPLY_CHAIN;
    b->prev_id = r->prev_id;
    uint32_t gap = bus_wire_us(1u, bps);
    b->gap_us = gap < floor_us ? floor_us : gap;
    /* fast10 (review S-5): 32-bit, same values; slot latency <= 255 x 3000 us, the wire time saturates on its own */
    uint32_t wire_us = bus_wire_us(r->prior_bytes, bps), slot_us = (uint32_t)r->response_slot * BUS_SLOT_LATENCY_US;
    b->fallback_us = wire_us > 0x7fffffffu - slot_us ? 0x7fffffffu : wire_us + slot_us;
}

bool bus_reply_pending(const bus_reply *b) { return b && b->kind != BUS_REPLY_NONE; }
bool bus_reply_watching(const bus_reply *b) { return b && (b->kind == BUS_REPLY_CHAIN || b->kind == BUS_REPLY_FAST) && !b->triggered; }

bool bus_reply_due(bus_reply *b, uint32_t now, uint32_t last_rx_us, bool rx_pending, uint32_t bps)
{
    if (!b || b->kind == BUS_REPLY_NONE) return false;
    if (b->kind == BUS_REPLY_DIRECT) return true;
    if (b->kind == BUS_REPLY_FAST && !b->triggered) return false;   /* no fallback: abandon instead */
    if ((b->kind == BUS_REPLY_CHAIN || b->kind == BUS_REPLY_FAST) && b->triggered) {
        /* Predecessor seen: only the chain gap releases us, even past the fallback deadline (its
         * driver may still be on; hardware 2026-11-20 bus2-02). And never into newer activity:
         * unread bytes or traffic after its status (review 2026-11-20) are parsed first. */
        if ((uint32_t)(now - b->trigger_us) < b->gap_us) return false;
        if (rx_pending || (uint32_t)(now - last_rx_us) < bus_idle_us(bps)) return false;
        return true;
    }
    if ((uint32_t)(now - b->armed_us) < b->fallback_us) return false;
    /* Deadline reached: never start in the middle of somebody else's packet. Unread bytes in the
     * RX ring mean the line was active while we were held (tick ISR): parse them first. */
    if (rx_pending || (uint32_t)(now - last_rx_us) < bus_idle_us(bps)) {
        if (!b->deferred) { b->deferred = true; b->deferrals++; }
        return false;
    }
    return true;
}

void bus_reply_done(bus_reply *b) { if (b) b->kind = BUS_REPLY_NONE; }

void bus_reply_reset_maxima(bus_reply *b)
{
    if (!b) return;
    b->lat_direct_max = b->lat_chain_max = b->pred_done_max = b->late_chains = 0u;
}

void bus_reply_sent(bus_reply *b, uint32_t now)
{
    if (!b) return;
    /* the path is counted when the reply really goes out (a deferred attempt is not counted) */
    if (b->kind == BUS_REPLY_DIRECT) b->direct++;
    else if (b->kind == BUS_REPLY_CHAIN) { if (b->triggered) b->chained++; else b->fallback++; }
    else if (b->kind == BUS_REPLY_FAST) b->fast_sent++;
    else if (b->kind == BUS_REPLY_SLOT) b->slotted++;
    if (b->kind == BUS_REPLY_DIRECT) {
        uint32_t lat = now - b->armed_us;
        if (lat > b->lat_direct_max) b->lat_direct_max = lat;
    } else if ((b->kind == BUS_REPLY_CHAIN || b->kind == BUS_REPLY_FAST) && b->triggered) {
        uint32_t lat = now - b->trigger_us;
        if (lat > b->lat_chain_max) b->lat_chain_max = lat;
    }
    b->kind = BUS_REPLY_NONE;
}

void bus_observe_status(bus_reply *b, uint8_t id, uint32_t now)
{
    if (!b) return;
    b->statuses++;
    if (b->kind == BUS_REPLY_CHAIN && id == b->prev_id) bus_reply_trigger(b, now);
}

void bus_reply_trigger(bus_reply *b, uint32_t now)
{
    if (!b || b->triggered || (b->kind != BUS_REPLY_CHAIN && b->kind != BUS_REPLY_FAST)) return;
    b->triggered = true; b->trigger_us = now;
    uint32_t done = now - b->armed_us;                          /* how long the predecessor took */
    if (done > b->pred_done_max) b->pred_done_max = done;
    if (done >= b->fallback_us) b->late_chains++;               /* it finished after our deadline */
}

bool bus_reply_abandon(bus_reply *b, uint32_t now, uint32_t last_rx_us, bool rx_pending)
{
    if (!b || b->kind != BUS_REPLY_FAST || b->triggered || (uint32_t)(now - b->armed_us) < b->fallback_us) return false;
    if (rx_pending || (uint32_t)(now - last_rx_us) < b->gap_us) return false;   /* parse what arrived first */
    b->kind = BUS_REPLY_NONE; b->fast_dropped++;
    return true;
}

void bus_fast_begin(bus_fast *f, uint32_t need, uint16_t seg, uint16_t len, uint8_t prev_id)
{
    if (!f) return;
    *f = (bus_fast){.need = need, .seg = seg, .len = len, .prev_id = prev_id, .state = BUS_FAST_CAPTURING};
    if (seg < 5u || need < 8u + seg || (need - 8u) % seg) f->state = BUS_FAST_FAILED;   /* not a row boundary */
    else f->pred = (uint16_t)((need - 8u) / seg - 1u);                                /* once, at arm */
}

uint8_t bus_fast_feed(bus_fast *f, uint8_t byte)
{
    if (!f || f->state != BUS_FAST_CAPTURING) return f ? f->state : BUS_FAST_FAILED;
    if (!f->count && byte != 0xffu) return f->state;           /* line noise before the slot-0 header */
    uint32_t i = f->count++;
    bool ok;
    if (i < 8u) {                                              /* FF FF FD 00 FE LEN_L LEN_H 55 */
        static const uint8_t head[8] = {0xff, 0xff, 0xfd, 0x00, 0xfe, 0, 0, 0x55};
        ok = i == 5u ? byte == (uint8_t)f->len : i == 6u ? byte == (uint8_t)(f->len >> 8) : byte == head[i];
    } else {
        uint32_t off = f->off;
        if (off + 2u == f->seg) f->field = f->crc;              /* CRC of everything before this field */
        ok = off + 2u == f->seg ? byte == (uint8_t)f->field
           : off + 1u == f->seg ? byte == (uint8_t)(f->field >> 8)
           : off == 1u && f->segno == f->pred ? byte == f->prev_id : true;
        if (++off == f->seg) { off = 0u; f->segno++; }
        f->off = (uint16_t)off;
    }
    if (!ok) { f->state = BUS_FAST_FAILED; return f->state; }
    f->crc = dxl_crc_update(f->crc, byte);                     /* later CRCs include earlier CRC bytes */
    if (f->count == f->need) f->state = BUS_FAST_READY;
    return f->state;
}

void bus_fast_seal(const bus_fast *f, uint8_t *seg, uint16_t seg_len)
{
    if (!f || !seg || seg_len < 3u) return;
    uint16_t crc = f->crc;
    for (uint16_t i = 0; i + 2u < seg_len; i++) crc = dxl_crc_update(crc, seg[i]);
    seg[seg_len - 2u] = (uint8_t)crc; seg[seg_len - 1u] = (uint8_t)(crc >> 8);
}

void bus_watch_begin(bus_watch *w, uint8_t kind, uint8_t prev_id, const bus_fast *fast)
{
    if (!w) return;
    *w = (bus_watch){.kind = kind, .prev_id = prev_id};
    if (fast) w->fast = *fast;
}

uint8_t bus_watch_feed(bus_watch *w, uint8_t byte)
{
    if (!w) return BUS_WATCH_DEAD;
    if (w->kind == BUS_REPLY_FAST) {
        uint8_t st = bus_fast_feed(&w->fast, byte);
        return st == BUS_FAST_READY ? BUS_WATCH_READY : st == BUS_FAST_FAILED ? BUS_WATCH_DEAD : BUS_WATCH_CAPTURING;
    }
    uint32_t i = w->at;
    if (i < 4u) {                         /* FF FF FD 00: status frames follow each other directly after the */
        static const uint8_t head[4] = {0xffu, 0xffu, 0xfdu, 0x00u};   /* request; anything else -> foreground */
        if (byte != head[i]) return BUS_WATCH_DEAD;
        w->at = i + 1u;
        if (w->at == 4u) w->crc = 0x0e28u;                         /* dxl_crc(FF FF FD 00) */
        return BUS_WATCH_CAPTURING;
    }
    w->at = i + 1u;
    if (i == 4u) w->id = byte;
    else if (i == 5u) w->total = byte;
    else if (i == 6u) { w->total = (uint16_t)(w->total | (byte << 8)); if (w->total < 3u || w->total > 0xfff0u) return BUS_WATCH_DEAD; w->total = (uint16_t)(w->total + 7u); }
    else if (i == 7u) w->inst = byte;
    if (i + 2u < w->total || i < 7u) { w->crc = dxl_crc_update(w->crc, byte); return BUS_WATCH_CAPTURING; }
    if (i + 2u == w->total) { w->low = byte; return BUS_WATCH_CAPTURING; }
    w->at = 0;                                                 /* frame complete */
    if ((uint16_t)(w->low | (byte << 8)) != w->crc) return BUS_WATCH_CAPTURING;
    if (w->inst != 0x55u) return BUS_WATCH_DEAD;               /* a new instruction: the foreground supersedes */
    return w->id == w->prev_id ? BUS_WATCH_READY : BUS_WATCH_CAPTURING;
}

uint8_t bus_watch_run(bus_watch *w, const volatile uint8_t *ring, uint32_t mask, uint32_t *at, uint32_t end)
{
    uint32_t i = *at;
    uint8_t st = BUS_WATCH_CAPTURING;
    const bool fast = w->kind == BUS_REPLY_FAST;
    bus_fast *f = &w->fast;
    uint16_t *crc = fast ? &f->crc : &w->crc;
    while (i != end && st == BUS_WATCH_CAPTURING) {
        uint32_t n = 0;                    /* plain bytes from here (never up to a row end: CRC fields stop it) */
        if (fast) {
            if (f->count >= 8u) { uint32_t stop = f->segno == f->pred && f->off <= 1u ? 1u : f->seg - 2u; if (f->off < stop) n = stop - f->off; }
        } else if (w->at >= 8u && w->at + 2u < w->total) n = w->total - 2u - w->at;
        if (n > end - i) n = end - i;
        if (!n) { st = bus_watch_feed(w, ring[i++ & mask]); continue; }
        if (fast) { f->count += n; f->off = (uint16_t)(f->off + n); } else w->at += n;
        uint16_t c = *crc;
        while (n--) c = dxl_crc_update(c, ring[i++ & mask]);
        *crc = c;
    }
    *at = i;
    return st;
}

bool bus_observe_instruction(bus_reply *b)
{
    if (!b || (b->kind != BUS_REPLY_CHAIN && b->kind != BUS_REPLY_SLOT && b->kind != BUS_REPLY_FAST)) return false;
    b->kind = BUS_REPLY_NONE; b->superseded++;
    return true;
}

void bus_baud_init(bus_baud *b)
{
    if (b) *b = (bus_baud){.current = BUS_BOOT_BAUD, .previous = BUS_BOOT_BAUD, .target = BUS_BOOT_BAUD, .state = BUS_BAUD_STABLE};
}

uint8_t bus_baud_request(bus_baud *b, uint32_t bps)
{
    if (!b) return 5u;
    if (!bus_baud_supported(bps)) return 4u;
    if (b->state != BUS_BAUD_STABLE) return 7u;
    if (bps == b->current) return 0u;
    b->target = bps; b->state = BUS_BAUD_PENDING;
    return 0u;
}

bool bus_baud_switch_ready(const bus_baud *b) { return b && b->state == BUS_BAUD_PENDING; }

void bus_baud_switched(bus_baud *b, bool ok, uint32_t now)
{
    if (!b || b->state != BUS_BAUD_PENDING) return;
    if (!ok) { b->state = BUS_BAUD_STABLE; return; }
    b->previous = b->current; b->current = b->target; b->since_us = now;
    b->state = BUS_BAUD_UNCONFIRMED; b->switches++;
}

void bus_baud_frame(bus_baud *b)
{
    if (!b) return;
    if (b->state == BUS_BAUD_UNCONFIRMED) { b->state = BUS_BAUD_STABLE; b->confirms++; }
    if (b->boot_state == BUS_BOOT_WAITING) b->boot_state = BUS_BOOT_RESCUED;   /* a 1 Mbaud host is here */
}

void bus_baud_boot(bus_baud *b, uint32_t boot_bps, uint32_t boot_us)
{
    if (!b || boot_bps == BUS_BOOT_BAUD || !bus_baud_supported(boot_bps)) return;
    b->boot_target = boot_bps; b->boot_since_us = boot_us; b->boot_state = BUS_BOOT_WAITING;
}

bool bus_baud_boot_due(const bus_baud *b, uint32_t now)
{
    return b && b->boot_state == BUS_BOOT_WAITING && b->state == BUS_BAUD_STABLE &&
           (uint32_t)(now - b->boot_since_us) >= BUS_RESCUE_US;
}

void bus_baud_boot_done(bus_baud *b, bool ok)
{
    if (!b || b->boot_state != BUS_BOOT_WAITING || !ok) return;   /* UART busy: due again next loop */
    b->previous = b->current; b->current = b->boot_target; b->boot_state = BUS_BOOT_SWITCHED; b->switches++;
}

bool bus_baud_wants_frames(const bus_baud *b)
{
    return b && (b->state == BUS_BAUD_UNCONFIRMED || b->boot_state == BUS_BOOT_WAITING);
}

bool bus_baud_revert_due(const bus_baud *b, uint32_t now)
{
    return b && b->state == BUS_BAUD_UNCONFIRMED && (uint32_t)(now - b->since_us) >= BUS_BAUD_CONFIRM_US;
}

void bus_baud_reverted(bus_baud *b, bool ok, uint32_t now)
{
    if (!b || b->state != BUS_BAUD_UNCONFIRMED) return;
    if (!ok) { b->since_us = now; return; }      /* UART busy: try again after another window */
    b->current = b->previous; b->state = BUS_BAUD_STABLE; b->reverts++;
}

uint32_t bus_baud_word(const bus_baud *b)
{
    if (!b) return 0u;
    return b->current | (b->state == BUS_BAUD_UNCONFIRMED ? 1u : 0u);
}

uint8_t bus_devcfg_resolve(const uint8_t block[BUS_DEVCFG_BYTES], uint8_t nv_id, uint8_t *state)
{
    uint8_t st, id = nv_id;
    if (!block) st = BUS_DEVCFG_ABSENT;
    else {
        uint32_t magic = (uint32_t)block[0] | ((uint32_t)block[1] << 8) | ((uint32_t)block[2] << 16) | ((uint32_t)block[3] << 24);
        uint16_t crc = (uint16_t)(block[28] | ((uint16_t)block[29] << 8));
        if (magic != BUS_DEVCFG_MAGIC) st = BUS_DEVCFG_ABSENT;
        else if (block[4] != 1u || (block[6] != 0xffu && (block[6] < 3u || block[6] > 5u)) || block[7] != 0xffu || block[30] != 0xffu || block[31] != 0xffu ||
                 dxl_crc(block, 28u) != crc || (block[5] > 252u && block[5] != 0xffu)) st = BUS_DEVCFG_INVALID;
        else if (block[5] == 0xffu) st = BUS_DEVCFG_DEFAULT;
        else { st = BUS_DEVCFG_OVERRIDE; id = block[5]; }
    }
    if (state) *state = st;
    return id;
}

uint32_t bus_devcfg_boot_baud(const uint8_t block[BUS_DEVCFG_BYTES])
{
    uint8_t st;
    (void)bus_devcfg_resolve(block, 0u, &st);
    if (st != BUS_DEVCFG_DEFAULT && st != BUS_DEVCFG_OVERRIDE) return BUS_BOOT_BAUD;
    return block[6] == 5u ? 3000000u : block[6] == 4u ? 2000000u : BUS_BOOT_BAUD;
}

uint8_t bus_lease_config(bus_lease *b, const uint8_t *in, uint16_t n, uint32_t now)
{
    if (!b || !in || n != 12u) return 5u;
    uint32_t ms = (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
    uint32_t act = (uint32_t)in[4] | ((uint32_t)in[5] << 8) | ((uint32_t)in[6] << 16) | ((uint32_t)in[7] << 24);
    uint32_t magic = (uint32_t)in[8] | ((uint32_t)in[9] << 8) | ((uint32_t)in[10] << 16) | ((uint32_t)in[11] << 24);
    if (magic != BUS_LEASE_MAGIC || ms > BUS_LEASE_MAX_MS || act > BUS_LEASE_ACT_HOLD) return 4u;
    b->ms = ms; b->action = act; b->last_us = now;
    b->state = ms ? BUS_LEASE_RUNNING : BUS_LEASE_OFF;
    return 0u;
}

void bus_lease_renew(bus_lease *b, uint32_t now) { if (b && b->state == BUS_LEASE_RUNNING) b->last_us = now; }

void bus_lease_rearm(bus_lease *b, uint32_t now)
{
    if (b && b->ms) { b->state = BUS_LEASE_RUNNING; b->last_us = now; }
}

bool bus_lease_expire(bus_lease *b, uint32_t now, bool armed)
{
    if (!b || b->state != BUS_LEASE_RUNNING || !armed) return false;
    if ((uint32_t)(now - b->last_us) < b->ms * 1000u) return false;
    b->state = BUS_LEASE_EXPIRED; b->expiries++;
    return true;
}

bool bus_lease_blocks_motion(const bus_lease *b)
{
    return b && b->state == BUS_LEASE_EXPIRED && b->action == BUS_LEASE_ACT_HOLD;
}

uint8_t bus_lease_gate(const bus_lease *b, uint16_t address, bool arm, bool armed)
{
    if (!armed || !bus_lease_blocks_motion(b)) return 0u;
    if (bus_lease_motion(address)) return 7u;
    return arm ? 1u : 0u;
}
